module axi4_mwr_batch #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer AXIS_FIFO_WIDTH  = AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH + AXIS_DATA_WIDTH/32 + 2, // Data + TUSER + TKEEP + SOP/EOP
    parameter integer FIFO_DEPTH       = 128,
    parameter integer TIME_FEDILITY = 8,
    parameter integer DEPTH_FEDILITY = 8,
    // Assumed worst-case packet length, in BEATS.  Two roles:
    //   • initial (and floor) value of the per-FIFO runtime max-length trackers
    //   • sets the prog_full headroom, so a packet that has started can always
    //     stream to its EOP without the FIFO overflowing.
    // Set it to the true worst case for the traffic; the runtime tracker only
    // grows past it (see MAX_LEN / decay logic below) and growth beyond it
    // degrades to backpressure, never to data loss.
    parameter integer MAX_PKT_BEATS = 16,
    // Where batched MRd goes, when the batch_mrd input enables it:
    //   0 = shared   : MRd rides the channel's MWr FIFO (0/1).  The FIFO is
    //                  in-order, so MWr↔MRd order is preserved for free and no
    //                  interlock is needed.
    //   1 = dedicated: MRd gets its own per-channel FIFO (3 = ch0, 4 = ch1) with
    //                  independent batching triggers, so reads batch separately
    //                  from writes.  Costs 2 extra BRAM FIFOs and needs the
    //                  serve-side ordering interlock described at ch_mrd_ord_ok.
    // Ignored while batch_mrd is low (MRd then passes through FIFO 2).
    parameter integer MRD_DEDICATED_FIFO = 0,
    parameter         IF_TYPE = "CQ"  // "CQ" or "RQ" — selects tuser sideband bit layout (PG343)
)(
    input wire                          clk,
    input wire                          rst_n,

    input wire [TIME_FEDILITY-1:0]              time_threshold,
    // Depth trigger is PER-PACKET: the threshold counts whole packets resident
    // in a batching FIFO (EOP written), not beats.
    input wire [DEPTH_FEDILITY-1:0]             depth_threshold,
    // Runtime enable for MRd batching.  Sampled per beat at the AXI-S handshake,
    // so it is safe to change at any time: the routing decision for a packet is
    // taken atomically when its first beat is accepted (and MRd is always a
    // single-beat, descriptor-only request).  Requests already queued keep the
    // path they were admitted on, and the ordering barriers below account for
    // MRd sitting in either the batching FIFOs or FIFO 2 after a toggle.
    input wire                                  batch_mrd,

    // AXX4 Slave AXIS interface 0
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_0,
    input  wire [AXIS_DATA_WIDTH/32-1:0] s_axis_tkeep_0,   // PG343: tkeep is per-DWORD
    input  wire                          s_axis_tvalid_0,
    input  wire                          s_axis_tlast_0,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_0,
    output wire                          s_axis_tready_0,

    // AXX4 Slave AXIS interface 1
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_1,
    input  wire [AXIS_DATA_WIDTH/32-1:0] s_axis_tkeep_1,   // PG343: tkeep is per-DWORD
    input  wire                          s_axis_tvalid_1,
    input  wire                          s_axis_tlast_1,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_1,
    output wire                          s_axis_tready_1,

    // AXI4 Master Read Address Channel
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/32-1:0] m_axis_tkeep,    // PG343: tkeep is per-DWORD
    output wire                          m_axis_tvalid,
    output wire                          m_axis_tlast,
    output wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser,
    input  wire                          m_axis_tready
    //
    // NOTE: Per-packet telemetry (length, gap, type, address, tag, counts)
    // is collected externally by the axi4_telemetry passthrough module on the
    // master stream, so this module no longer exports telemetry ports.
    // time_threshold / depth_threshold are retained: they drive the internal
    // batching priority (timeout / packet-depth triggers) used by the arbiter,
    // alongside the parameter-seeded space trigger (see MAX_PKT_BEATS).
);

// =================================================================
// Internal FIFO Signals — instantiated as xpm_fifo_sync below
// =================================================================
// Count width: $clog2(FIFO_DEPTH)+1  e.g. 8 bits for FIFO_DEPTH=128
localparam integer FIFO_CNT_W = $clog2(FIFO_DEPTH) + 1;

// tkeep is per-DWORD (PG343): width = DATA_WIDTH/32 (16 for a 512-bit bus)
localparam integer TKEEP_WIDTH = AXIS_DATA_WIDTH/32;

// -----------------------------------------------------------------------------
// prog_full (almost_full) backpressure headroom.
//
// Writes in this module are *registered* (wr_en/din come from r_wr_en_x_r /
// r_fifo_din), so an accepted beat commits to the FIFO one cycle after the
// AXI-S handshake.  prog_full itself also has assertion latency.  If flow
// control gated on raw `full`, that pipeline delay would let extra beats be
// accepted after the FIFO had no room, and the trailing writes would overflow
// and be silently dropped.
//
// We instead deassert s_axis_tready on prog_full, reserving PROG_FULL_HEADROOM
// empty slots.  Minimum required for lossless operation is ~3 (registered
// write + prog_full latency).  We extend it to the longest in-flight packet so
// that a packet which has already started can always stream to completion
// without the FIFO ever overflowing — i.e. flow control at packet granularity.
//
// The "longest packet" term is the MAX_PKT_BEATS parameter, so the headroom
// tracks the declared worst case instead of a magic number.  PROG_FULL_THRESH
// is a static xpm generic and cannot follow the RUNTIME max tracker; if a
// packet longer than MAX_PKT_BEATS ever arrives, prog_full simply starts back-
// pressuring slightly before the space trigger flushes.  That is a stall, not
// an overflow.
// -----------------------------------------------------------------------------
// The floor keeps the xpm generic legal (fwft requires 5 .. DEPTH-5) if someone
// parameterises MAX_PKT_BEATS close to, or past, FIFO_DEPTH: that degrades to a
// tiny admit limit — effectively "never batch" — instead of an illegal generic.
localparam integer PROG_FULL_HEADROOM = MAX_PKT_BEATS + 3;
localparam integer PROG_FULL_THRESH_MIN = 5;
localparam integer PROG_FULL_THRESH_C = ((FIFO_DEPTH - PROG_FULL_HEADROOM) > PROG_FULL_THRESH_MIN)
                                      ?  (FIFO_DEPTH - PROG_FULL_HEADROOM) : PROG_FULL_THRESH_MIN;

// Packet-length bookkeeping (beats).  A packet can never usefully exceed the
// FIFO, so a FIFO count width also holds any tracked length.
localparam integer MAXLEN_W = FIFO_CNT_W;

// Runtime max-length tracker: seeded at (and floored at) the parameter, clamped
// at the admit limit.  Past that clamp the space trigger would be permanently
// asserted, so there is nothing to gain by tracking higher.
localparam [MAXLEN_W-1:0]   MAX_LEN_INIT  = MAX_PKT_BEATS;
localparam [MAXLEN_W-1:0]   MAX_LEN_CLAMP = PROG_FULL_THRESH_C;

// Admit limit: the fill level at which prog_full stops accepting new beats.
// The remaining-space calculation that drives the flush trigger measures against
// THIS, not against FIFO_DEPTH.  The slots above it are reserved headroom for a
// packet already in flight, so they are not space a new packet can be admitted
// into — and measuring against FIFO_DEPTH would put the flush threshold above
// the backpressure point, where it could never fire.
localparam [FIFO_CNT_W-1:0] ADMIT_LIMIT_C = PROG_FULL_THRESH_C;

// MRD_DEDICATED_FIFO as a 1-bit condition, and the master-side "are FIFOs 3/4
// present" flag.  Dedicated MRd FIFOs only exist when the parameter selects
// them; when it does not, the generate block below ties their signals off so the
// two xpm instances (and their BRAMs) are never built.
localparam MRD_DED = (MRD_DEDICATED_FIFO != 0);

// FIFO 0 (ch0 MWr)
wire                         almost_full_0,  almost_empty_0;
wire                         full_0,         empty_0;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_0; // driven by slave-facing process (assign)
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_0;// driven by xpm_fifo_sync dout
wire                         wr_en_0;        // driven by slave-facing process (assign)
wire                         rd_en_0;        // driven by master interface (assign)
wire [FIFO_CNT_W-1:0]        wr_data_count_0, rd_data_count_0;
wire                         data_valid_0;

// FIFO 1 (ch1 MWr)
wire                         almost_full_1,  almost_empty_1;
wire                         full_1,         empty_1;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_1;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_1;
wire                         wr_en_1;
wire                         rd_en_1;
wire [FIFO_CNT_W-1:0]        wr_data_count_1, rd_data_count_1;
wire                         data_valid_1;

// FIFO 2 (pass-through: request types that are not batched)
wire                         almost_full_2,  almost_empty_2;
wire                         full_2,         empty_2;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_2;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_2;
wire                         wr_en_2;
wire                         rd_en_2;
wire [FIFO_CNT_W-1:0]        wr_data_count_2, rd_data_count_2;
wire                         data_valid_2;

// FIFO 3 (ch0 MRd) / FIFO 4 (ch1 MRd) — only built when MRD_DEDICATED_FIFO
wire                         almost_full_3,  empty_3;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_3;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_3;
wire                         wr_en_3;
wire                         rd_en_3;
wire [FIFO_CNT_W-1:0]        wr_data_count_3;

wire                         almost_full_4,  empty_4;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_4;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_4;
wire                         wr_en_4;
wire                         rd_en_4;
wire [FIFO_CNT_W-1:0]        wr_data_count_4;

// =================================================================
// Xilinx xpm_fifo_sync Instantiations
//   READ_MODE       = "fwft"  : First-Word-Fall-Through
//   FIFO_MEMORY_TYPE= "bram"  : uses block-RAM resources
//   prog_full       → almost_full_x  (asserts at FIFO_DEPTH-PROG_FULL_HEADROOM
//                                      entries; drives s_axis_tready backpressure)
//   prog_empty      → almost_empty_x (asserts at 4 entries)
//   USE_ADV_FEATURES= "070F" enables:
//     [0] data_valid  [1] almost_empty  [2] rd_data_count  [3] prog_empty
//     [8] almost_full [9] wr_data_count [10] prog_full
// =================================================================

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE    ("bram"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
    .WRITE_DATA_WIDTH    (AXIS_FIFO_WIDTH),
    .WR_DATA_COUNT_WIDTH (FIFO_CNT_W),
    .PROG_FULL_THRESH    (PROG_FULL_THRESH_C),
    .FULL_RESET_VALUE    (0),
    .READ_MODE           ("fwft"),
    .FIFO_READ_LATENCY   (0),
    .READ_DATA_WIDTH     (AXIS_FIFO_WIDTH),
    .RD_DATA_COUNT_WIDTH (FIFO_CNT_W),
    .PROG_EMPTY_THRESH   (5),
    .DOUT_RESET_VALUE    ("0"),
    .WAKEUP_TIME         (0),
    .USE_ADV_FEATURES    ("070F")
) u_fifo_0 (
    .sleep         (1'b0),
    .rst           (~rst_n),
    .wr_clk        (clk),
    .wr_en         (wr_en_0),
    .din           (fifo_data_in_0),
    .full          (full_0),
    .prog_full     (almost_full_0),
    .wr_data_count (wr_data_count_0),
    .overflow      (),
    .wr_rst_busy   (),
    .rd_en         (rd_en_0),
    .dout          (fifo_data_out_0),
    .empty         (empty_0),
    .prog_empty    (almost_empty_0),
    .rd_data_count (rd_data_count_0),
    .underflow     (),
    .rd_rst_busy   (),
    .data_valid    (data_valid_0),
    .almost_empty  (),
    .almost_full   (),
    .dbiterr       (),
    .sbiterr       (),
    .injectdbiterr (1'b0),
    .injectsbiterr (1'b0)
);

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE    ("bram"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
    .WRITE_DATA_WIDTH    (AXIS_FIFO_WIDTH),
    .WR_DATA_COUNT_WIDTH (FIFO_CNT_W),
    .PROG_FULL_THRESH    (PROG_FULL_THRESH_C),
    .FULL_RESET_VALUE    (0),
    .READ_MODE           ("fwft"),
    .FIFO_READ_LATENCY   (0),
    .READ_DATA_WIDTH     (AXIS_FIFO_WIDTH),
    .RD_DATA_COUNT_WIDTH (FIFO_CNT_W),
    .PROG_EMPTY_THRESH   (5),
    .DOUT_RESET_VALUE    ("0"),
    .WAKEUP_TIME         (0),
    .USE_ADV_FEATURES    ("070F")
) u_fifo_1 (
    .sleep         (1'b0),
    .rst           (~rst_n),
    .wr_clk        (clk),
    .wr_en         (wr_en_1),
    .din           (fifo_data_in_1),
    .full          (full_1),
    .prog_full     (almost_full_1),
    .wr_data_count (wr_data_count_1),
    .overflow      (),
    .wr_rst_busy   (),
    .rd_en         (rd_en_1),
    .dout          (fifo_data_out_1),
    .empty         (empty_1),
    .prog_empty    (almost_empty_1),
    .rd_data_count (rd_data_count_1),
    .underflow     (),
    .rd_rst_busy   (),
    .data_valid    (data_valid_1),
    .almost_empty  (),
    .almost_full   (),
    .dbiterr       (),
    .sbiterr       (),
    .injectdbiterr (1'b0),
    .injectsbiterr (1'b0)
);

xpm_fifo_sync #(
    .FIFO_MEMORY_TYPE    ("bram"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
    .WRITE_DATA_WIDTH    (AXIS_FIFO_WIDTH),
    .WR_DATA_COUNT_WIDTH (FIFO_CNT_W),
    .PROG_FULL_THRESH    (PROG_FULL_THRESH_C),
    .FULL_RESET_VALUE    (0),
    .READ_MODE           ("fwft"),
    .FIFO_READ_LATENCY   (0),
    .READ_DATA_WIDTH     (AXIS_FIFO_WIDTH),
    .RD_DATA_COUNT_WIDTH (FIFO_CNT_W),
    .PROG_EMPTY_THRESH   (5),
    .DOUT_RESET_VALUE    ("0"),
    .WAKEUP_TIME         (0),
    .USE_ADV_FEATURES    ("070F")
) u_fifo_2 (
    .sleep         (1'b0),
    .rst           (~rst_n),
    .wr_clk        (clk),
    .wr_en         (wr_en_2),
    .din           (fifo_data_in_2),
    .full          (full_2),
    .prog_full     (almost_full_2),
    .wr_data_count (wr_data_count_2),
    .overflow      (),
    .wr_rst_busy   (),
    .rd_en         (rd_en_2),
    .dout          (fifo_data_out_2),
    .empty         (empty_2),
    .prog_empty    (almost_empty_2),
    .rd_data_count (rd_data_count_2),
    .underflow     (),
    .rd_rst_busy   (),
    .data_valid    (data_valid_2),
    .almost_empty  (),
    .almost_full   (),
    .dbiterr       (),
    .sbiterr       (),
    .injectdbiterr (1'b0),
    .injectsbiterr (1'b0)
);

// =================================================================
// Dedicated MRd FIFOs (3 = ch0, 4 = ch1) — MRD_DEDICATED_FIFO only.
//
// Same geometry as FIFOs 0/1.  When the parameter is 0 these are not built and
// the else-branch ties their signals to "permanently empty, never full", which
// constant-folds the MRd paths out of the routing, arbitration and FSM logic.
// =================================================================
generate
if (MRD_DED) begin : g_mrd_fifos

    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE    ("bram"),
        .ECC_MODE            ("no_ecc"),
        .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
        .WRITE_DATA_WIDTH    (AXIS_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH (FIFO_CNT_W),
        .PROG_FULL_THRESH    (PROG_FULL_THRESH_C),
        .FULL_RESET_VALUE    (0),
        .READ_MODE           ("fwft"),
        .FIFO_READ_LATENCY   (0),
        .READ_DATA_WIDTH     (AXIS_FIFO_WIDTH),
        .RD_DATA_COUNT_WIDTH (FIFO_CNT_W),
        .PROG_EMPTY_THRESH   (5),
        .DOUT_RESET_VALUE    ("0"),
        .WAKEUP_TIME         (0),
        .USE_ADV_FEATURES    ("070F")
    ) u_fifo_3 (
        .sleep         (1'b0),
        .rst           (~rst_n),
        .wr_clk        (clk),
        .wr_en         (wr_en_3),
        .din           (fifo_data_in_3),
        .full          (),
        .prog_full     (almost_full_3),
        .wr_data_count (wr_data_count_3),
        .overflow      (),
        .wr_rst_busy   (),
        .rd_en         (rd_en_3),
        .dout          (fifo_data_out_3),
        .empty         (empty_3),
        .prog_empty    (),
        .rd_data_count (),
        .underflow     (),
        .rd_rst_busy   (),
        .data_valid    (),
        .almost_empty  (),
        .almost_full   (),
        .dbiterr       (),
        .sbiterr       (),
        .injectdbiterr (1'b0),
        .injectsbiterr (1'b0)
    );

    xpm_fifo_sync #(
        .FIFO_MEMORY_TYPE    ("bram"),
        .ECC_MODE            ("no_ecc"),
        .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
        .WRITE_DATA_WIDTH    (AXIS_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH (FIFO_CNT_W),
        .PROG_FULL_THRESH    (PROG_FULL_THRESH_C),
        .FULL_RESET_VALUE    (0),
        .READ_MODE           ("fwft"),
        .FIFO_READ_LATENCY   (0),
        .READ_DATA_WIDTH     (AXIS_FIFO_WIDTH),
        .RD_DATA_COUNT_WIDTH (FIFO_CNT_W),
        .PROG_EMPTY_THRESH   (5),
        .DOUT_RESET_VALUE    ("0"),
        .WAKEUP_TIME         (0),
        .USE_ADV_FEATURES    ("070F")
    ) u_fifo_4 (
        .sleep         (1'b0),
        .rst           (~rst_n),
        .wr_clk        (clk),
        .wr_en         (wr_en_4),
        .din           (fifo_data_in_4),
        .full          (),
        .prog_full     (almost_full_4),
        .wr_data_count (wr_data_count_4),
        .overflow      (),
        .wr_rst_busy   (),
        .rd_en         (rd_en_4),
        .dout          (fifo_data_out_4),
        .empty         (empty_4),
        .prog_empty    (),
        .rd_data_count (),
        .underflow     (),
        .rd_rst_busy   (),
        .data_valid    (),
        .almost_empty  (),
        .almost_full   (),
        .dbiterr       (),
        .sbiterr       (),
        .injectdbiterr (1'b0),
        .injectsbiterr (1'b0)
    );

end else begin : g_no_mrd_fifos

    assign almost_full_3   = 1'b0;
    assign empty_3         = 1'b1;
    assign fifo_data_out_3 = {AXIS_FIFO_WIDTH{1'b0}};
    assign wr_data_count_3 = {FIFO_CNT_W{1'b0}};
    assign almost_full_4   = 1'b0;
    assign empty_4         = 1'b1;
    assign fifo_data_out_4 = {AXIS_FIFO_WIDTH{1'b0}};
    assign wr_data_count_4 = {FIFO_CNT_W{1'b0}};

end
endgenerate

// -----------------------------------------------------------------------------
// tuser sideband field offsets (LSB of each field).
//
// The descriptor (tdata) fields — request_type[78:75], address_type[1:0],
// address[63:2], tag[103:96] — share identical positions in both the CQ and RQ
// descriptor formats, so only the tuser SOP/EOP/EOP_PTR offsets move between
// interfaces (PG343 §1.2 vs §3.2):
//   CQ : is_sop[81:80] is_eop[87:86] is_eop0_ptr[91:88]
//   RQ : is_sop[21:20] is_eop[27:26] is_eop0_ptr[31:28]
// -----------------------------------------------------------------------------
localparam integer SOP_LO    = (IF_TYPE == "RQ") ? 20 : 80;
localparam integer EOP_LO    = (IF_TYPE == "RQ") ? 26 : 86;
localparam integer EOPPTR_LO = (IF_TYPE == "RQ") ? 28 : 88;

//Decoding Key Values from slave interfaces
wire [3:0]    request_type_0;
wire [1:0]    address_type_0;
wire [61:0]   address_0;
wire [7:0]    tag_0;
wire [1:0]    sop_0;
wire [1:0]    eop_0;
wire [3:0]    eop_ptr_0; 

wire [3:0]    request_type_1;
wire [1:0]    address_type_1;
wire [61:0]   address_1;
wire [7:0]    tag_1;
wire [1:0]    sop_1;
wire [1:0]    eop_1;
wire [3:0]    eop_ptr_1;

assign request_type_0 = s_axis_tdata_0[78:75];
assign address_type_0 = s_axis_tdata_0[1:0];
assign address_0      = s_axis_tdata_0[63:2];
assign tag_0          = s_axis_tdata_0[103:96];
assign sop_0          = s_axis_tuser_0[SOP_LO    + 1 : SOP_LO];
assign eop_0          = s_axis_tuser_0[EOP_LO    + 1 : EOP_LO];
assign eop_ptr_0      = s_axis_tuser_0[EOPPTR_LO + 3 : EOPPTR_LO];

assign request_type_1 = s_axis_tdata_1[78:75];
assign address_type_1 = s_axis_tdata_1[1:0];
assign address_1      = s_axis_tdata_1[63:2];
assign tag_1          = s_axis_tdata_1[103:96];
assign sop_1          = s_axis_tuser_1[SOP_LO    + 1 : SOP_LO];
assign eop_1          = s_axis_tuser_1[EOP_LO    + 1 : EOP_LO];
assign eop_ptr_1      = s_axis_tuser_1[EOPPTR_LO + 3 : EOPPTR_LO];

// =================================================================
// Local Constants
// =================================================================
// PG343 request_type encodings (tdata[78:75], identical in CQ and RQ):
localparam MWR_TYPE = 4'b0001;   // Memory Write  (posted)
localparam MRD_TYPE = 4'b0000;   // Memory Read   (non-posted, descriptor-only)

// =================================================================
// FIFO Data Packing
// Format: { tkeep[TKEEP_WIDTH], tlast[1], sop[1], tuser[AXIS_TUSER_WIDTH], tdata[AXIS_DATA_WIDTH] }
//
// tkeep is carried verbatim through the FIFO.  Straddle is OFF, so per PG343
// s_axis_tkeep is the authoritative per-DWORD valid indicator for the EOP beat;
// it must be preserved rather than reconstructed from the straddle-only
// is_eop0_ptr sideband (which is undriven/0 when straddle is disabled).
// =================================================================
wire [AXIS_FIFO_WIDTH-1:0] pack_ch0;
wire [AXIS_FIFO_WIDTH-1:0] pack_ch1;

assign pack_ch0 = {s_axis_tkeep_0, s_axis_tlast_0, sop_0[0], s_axis_tuser_0, s_axis_tdata_0};
assign pack_ch1 = {s_axis_tkeep_1, s_axis_tlast_1, sop_1[0], s_axis_tuser_1, s_axis_tdata_1};

// =================================================================
// Internal Registers
// =================================================================

// Registered FIFO write outputs (write enable is a single-cycle pulse)
reg  [AXIS_FIFO_WIDTH-1:0] r_fifo0_din;
reg                         r_wr_en_0_r;
reg  [AXIS_FIFO_WIDTH-1:0] r_fifo1_din;
reg                         r_wr_en_1_r;
reg  [AXIS_FIFO_WIDTH-1:0] r_fifo2_din;
reg                         r_wr_en_2_r;
reg  [AXIS_FIFO_WIDTH-1:0] r_fifo3_din;
reg                         r_wr_en_3_r;
reg  [AXIS_FIFO_WIDTH-1:0] r_fifo4_din;
reg                         r_wr_en_4_r;

// FIFO 2 packet-ownership lock.
// Once a channel begins writing a (multi-beat) pass-through packet into the shared
// FIFO 2, it owns FIFO 2 until that packet's EOP.  This keeps every FIFO-2
// packet contiguous — without it, beats from two concurrent pass-through packets
// (one per channel) would interleave into a single corrupted output packet.
reg                         r_f2_lock;     // 1 = a channel owns FIFO 2
reg                         r_f2_lock_ch;  // owning channel (0 or 1)

// Per-channel packet-class latch.
// request_type (tdata[78:75]) lives in the SOP/descriptor beat only; on the
// payload beats of a multi-beat packet those bits are arbitrary.  Latch the class
// decided at SOP and hold it to EOP so every beat of a packet routes to the same
// FIFO and the ready/ordering logic sees one stable class.  Mirrors r_f2_lock.
// It also pins the class against a mid-packet change of the batch_mrd input.
reg                         r_ch0_inpkt;   // 1 = ch0 mid-packet (SOP seen, EOP not yet)
reg  [1:0]                  r_ch0_class;   // latched class for the current ch0 packet
reg                         r_ch1_inpkt;
reg  [1:0]                  r_ch1_class;

// =================================================================
// Type Detection / Routing Class
//
// is_mwr_x / is_mrd_x are the LIVE descriptor decode — valid only in the SOP
// beat.  From them each beat gets one of three ROUTING CLASSES:
//
//   CLS_MWR  → the channel's MWr batching FIFO (0 = ch0, 1 = ch1).
//              Always MWr; also MRd when batch_mrd is on and the dedicated-FIFO
//              parameter is off.  Sharing one in-order FIFO is what makes that
//              case safe with no interlock: an MRd can never be emitted ahead of
//              an earlier MWr on the same channel.
//   CLS_MRD  → the channel's dedicated MRd FIFO (3 = ch0, 4 = ch1).
//              Only when batch_mrd is on AND MRD_DEDICATED_FIFO.  Because this
//              splits posted and non-posted traffic into parallel queues, MWr↔MRd
//              order is NOT preserved by the FIFOs and is instead enforced on the
//              serve side — see ch_mrd_ord_ok in the arbitration section.
//   CLS_PASS → shared pass-through FIFO 2 (every other request type, and MRd
//              while batch_mrd is low).
//
// ch_x_cls is the per-beat EFFECTIVE class used by all routing/ready logic: the
// live decode at SOP (r_chx_inpkt==0), or the latch mid-packet.
// =================================================================
localparam [1:0] CLS_PASS = 2'd0;
localparam [1:0] CLS_MWR  = 2'd1;
localparam [1:0] CLS_MRD  = 2'd2;

wire is_mwr_0 = (request_type_0 == MWR_TYPE);
wire is_mwr_1 = (request_type_1 == MWR_TYPE);
wire is_mrd_0 = (request_type_0 == MRD_TYPE);
wire is_mrd_1 = (request_type_1 == MRD_TYPE);

// MRd destination while batching is enabled: dedicated FIFO, or the MWr FIFO.
wire mrd_to_ded = batch_mrd &&  MRD_DED;
wire mrd_to_mwr = batch_mrd && !MRD_DED;

wire [1:0] cls_live_0 = is_mwr_0                  ? CLS_MWR  :
                        (is_mrd_0 && mrd_to_ded)  ? CLS_MRD  :
                        (is_mrd_0 && mrd_to_mwr)  ? CLS_MWR  : CLS_PASS;
wire [1:0] cls_live_1 = is_mwr_1                  ? CLS_MWR  :
                        (is_mrd_1 && mrd_to_ded)  ? CLS_MRD  :
                        (is_mrd_1 && mrd_to_mwr)  ? CLS_MWR  : CLS_PASS;

wire [1:0] ch0_cls = r_ch0_inpkt ? r_ch0_class : cls_live_0;
wire [1:0] ch1_cls = r_ch1_inpkt ? r_ch1_class : cls_live_1;

wire ch0_cls_mwr  = (ch0_cls == CLS_MWR);
wire ch1_cls_mwr  = (ch1_cls == CLS_MWR);
wire ch0_cls_mrd  = (ch0_cls == CLS_MRD);
wire ch1_cls_mrd  = (ch1_cls == CLS_MRD);
wire ch0_cls_pass = (ch0_cls == CLS_PASS);
wire ch1_cls_pass = (ch1_cls == CLS_PASS);

// =================================================================
// Ready Signals — Combinatorial, Type-Aware
//
//  Backpressure gates on almost_full (prog_full), NOT raw full: the write path
//  is registered (an accepted beat commits one cycle later) and prog_full has
//  its own assertion latency.  prog_full reserves PROG_FULL_HEADROOM slots so
//  every accepted beat has a landing spot and a started packet streams to EOP.
//
//  CLS_MWR path: ready when the channel's MWr FIFO (0/1) is not almost-full.
//
//  CLS_MRD path: ready when the channel's dedicated MRd FIFO (3/4) is not
//  almost-full.  No admission barrier — MWr↔MRd order is enforced on the serve
//  side instead (ch_mrd_ord_ok), so a read never has to stall the input and
//  head-of-line block the MWr stream behind it.
//
//  CLS_PASS path: ready only when ALL hold —
//    • FIFO 2 not almost-full, AND
//    • ORDERING BARRIER clear (ch_bar_ok): that channel's batching FIFOs are
//      empty with no registered write in flight.  This stops a pass-through
//      request being admitted — and thus emitted — ahead of an earlier MWr on
//      the same channel, which PCIe ordering forbids (a non-posted request may
//      not pass an earlier posted one).  The barrier covers the dedicated MRd
//      FIFO too, so that a batch_mrd 1→0 toggle cannot let a newly pass-through
//      MRd overtake reads still queued in FIFO 3/4.
//    • PACKET LOCK allows it: FIFO 2 free, or already owned by this channel.
//      Channel 0 wins a free lock; channel 1 defers to a same-cycle ch0
//      acquire.  This keeps each FIFO-2 packet contiguous (no cross-channel
//      beat interleave).
//
//  valid=0: assert ready optimistically; the real gate fires when valid is.
// =================================================================
// FIFO 2 packet-ownership lock state
wire f2_free = !r_f2_lock;
wire f2_own0 =  r_f2_lock && (r_f2_lock_ch == 1'b0);
wire f2_own1 =  r_f2_lock && (r_f2_lock_ch == 1'b1);

// A channel has batched traffic still pending if any beat it has accepted into
// its batching FIFO has not yet been read back out.  This is tracked with an
// explicit in-flight counter rather than `!empty || r_wr_en_r`.
//
// Why not `empty`: the xpm FWFT `empty` flag can lag the write by more than one
// cycle.  During that lag `empty` still reads high while `r_wr_en_x_r` has
// already cleared, so `!empty || r_wr_en_x_r` momentarily reports "drained"
// while a batched beat is actually in flight.  The ordering barrier below would
// then admit a pass-through beat into FIFO 2 ahead of it; the FSM commits to that
// FIFO-2 packet (MST_SERVE_2, r_in_pkt=1) but can never finish it, because the
// rest of the packet now sits behind a (real) non-empty batching FIFO that only
// the blocked FSM could drain — a cyclic, permanent deadlock.
//
// The counter is exact and latency-independent: +1 when a beat's registered
// write commits (r_wr_en_x_r), -1 when a beat is read out (rd_en_x).  ORing
// r_wr_en_x_r into `pending` covers the commit cycle itself, before the
// registered counter has incremented.  All terms are registered, so this
// introduces no combinational path through the tready logic.
// One counter per batching FIFO: 0/1 (MWr) and 3/4 (dedicated MRd).
reg [FIFO_CNT_W-1:0] r_batch_inflight_0;
reg [FIFO_CNT_W-1:0] r_batch_inflight_1;
reg [FIFO_CNT_W-1:0] r_batch_inflight_3;
reg [FIFO_CNT_W-1:0] r_batch_inflight_4;

wire f0_pending = (r_batch_inflight_0 != 0) || r_wr_en_0_r;
wire f1_pending = (r_batch_inflight_1 != 0) || r_wr_en_1_r;
wire f3_pending = (r_batch_inflight_3 != 0) || r_wr_en_3_r;
wire f4_pending = (r_batch_inflight_4 != 0) || r_wr_en_4_r;

wire ch0_batch_pending = f0_pending || f3_pending;
wire ch1_batch_pending = f1_pending || f4_pending;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_batch_inflight_0 <= {FIFO_CNT_W{1'b0}};
        r_batch_inflight_1 <= {FIFO_CNT_W{1'b0}};
        r_batch_inflight_3 <= {FIFO_CNT_W{1'b0}};
        r_batch_inflight_4 <= {FIFO_CNT_W{1'b0}};
    end else begin
        r_batch_inflight_0 <= r_batch_inflight_0 + (r_wr_en_0_r ? 1'b1 : 1'b0)
                                                 - (rd_en_0     ? 1'b1 : 1'b0);
        r_batch_inflight_1 <= r_batch_inflight_1 + (r_wr_en_1_r ? 1'b1 : 1'b0)
                                                 - (rd_en_1     ? 1'b1 : 1'b0);
        r_batch_inflight_3 <= r_batch_inflight_3 + (r_wr_en_3_r ? 1'b1 : 1'b0)
                                                 - (rd_en_3     ? 1'b1 : 1'b0);
        r_batch_inflight_4 <= r_batch_inflight_4 + (r_wr_en_4_r ? 1'b1 : 1'b0)
                                                 - (rd_en_4     ? 1'b1 : 1'b0);
    end
end

// Ordering barrier: a channel's pass-through beat may enter FIFO 2 only once
// that channel's batching FIFOs have fully drained.
wire ch0_bar_ok = !ch0_batch_pending;
wire ch1_bar_ok = !ch1_batch_pending;

// A channel currently presenting a pass-through beat.
wire ch0_pres_pass = s_axis_tvalid_0 && ch0_cls_pass;
wire ch1_pres_pass = s_axis_tvalid_1 && ch1_cls_pass;

// Channel 0 is about to take a free lock this cycle (channel 1 must defer).
wire ch0_wants_lock = ch0_pres_pass && ch0_bar_ok && !almost_full_2;

// Pass-through ready, per channel: FIFO 2 room + barrier clear + lock available.
wire ch0_pass_ok = !almost_full_2 && ch0_bar_ok && (f2_own0 || f2_free);
wire ch1_pass_ok = !almost_full_2 && ch1_bar_ok && (f2_own1 || (f2_free && !ch0_wants_lock));

wire s_tready_0_w = s_axis_tvalid_0 ? (ch0_cls_mwr ? !almost_full_0 :
                                       ch0_cls_mrd ? !almost_full_3 : ch0_pass_ok) : 1'b1;
wire s_tready_1_w = s_axis_tvalid_1 ? (ch1_cls_mwr ? !almost_full_1 :
                                       ch1_cls_mrd ? !almost_full_4 : ch1_pass_ok) : 1'b1;

assign s_axis_tready_0 = s_tready_0_w;
assign s_axis_tready_1 = s_tready_1_w;

// =================================================================
// AXI-S Handshake Fire Signals
// =================================================================
wire ch0_fire      = s_axis_tvalid_0 && s_tready_0_w;
wire ch1_fire      = s_axis_tvalid_1 && s_tready_1_w;
wire ch0_fire_mwr  = ch0_fire && ch0_cls_mwr;    // → FIFO 0
wire ch1_fire_mwr  = ch1_fire && ch1_cls_mwr;    // → FIFO 1
wire ch0_fire_mrd  = ch0_fire && ch0_cls_mrd;    // → FIFO 3
wire ch1_fire_mrd  = ch1_fire && ch1_cls_mrd;    // → FIFO 4
wire ch0_fire_pass = ch0_fire && ch0_cls_pass;   // → FIFO 2
wire ch1_fire_pass = ch1_fire && ch1_cls_pass;   // → FIFO 2

// =================================================================
// FIFO Output Assignments (driven from registers)
// =================================================================
assign fifo_data_in_0 = r_fifo0_din;
assign wr_en_0        = r_wr_en_0_r;
assign fifo_data_in_1 = r_fifo1_din;
assign wr_en_1        = r_wr_en_1_r;
assign fifo_data_in_2 = r_fifo2_din;
assign wr_en_2        = r_wr_en_2_r;
assign fifo_data_in_3 = r_fifo3_din;
assign wr_en_3        = r_wr_en_3_r;
assign fifo_data_in_4 = r_fifo4_din;
assign wr_en_4        = r_wr_en_4_r;

// =================================================================
// Master Interface Process — Declarations
// =================================================================

// --- State encoding ---
localparam [2:0] MST_IDLE    = 3'd0;
localparam [2:0] MST_SERVE_0 = 3'd1;   // transmitting a packet from FIFO 0 (ch0 MWr batch)
localparam [2:0] MST_SERVE_1 = 3'd2;   // transmitting a packet from FIFO 1 (ch1 MWr batch)
localparam [2:0] MST_SERVE_2 = 3'd3;   // transmitting a packet from FIFO 2 (pass-through, interleaved)
localparam [2:0] MST_SERVE_3 = 3'd4;   // transmitting a packet from FIFO 3 (ch0 MRd batch)
localparam [2:0] MST_SERVE_4 = 3'd5;   // transmitting a packet from FIFO 4 (ch1 MRd batch)

// --- State register and per-channel aging counters ---
reg [2:0] r_mst_state;
reg [7:0] r_wait_0;    // cycles ch0 has been requesting-but-not-served (saturates at 8'hFF)
reg [7:0] r_wait_1;    // same for ch1

// --- Packet-in-flight flag (master side) ---
// 1 = a packet has begun transmission (a non-EOP beat was accepted) and its
//     EOP has not yet been accepted.  Used so the FSM can safely re-arbitrate
//     out of a SERVE state when its FIFO drains *between* packets, while still
//     holding the state across a mid-packet underrun bubble.
reg       r_in_pkt;

// --- Mux current FIFO data/empty based on active state ---
// FIFO packing: { tkeep[TKEEP_WIDTH], tlast[1], sop[1], tuser[AXIS_TUSER_WIDTH], tdata[AXIS_DATA_WIDTH] }
wire [AXIS_FIFO_WIDTH-1:0] cur_data;
wire                        cur_empty;

assign cur_data  = (r_mst_state == MST_SERVE_0) ? fifo_data_out_0 :
                   (r_mst_state == MST_SERVE_1) ? fifo_data_out_1 :
                   (r_mst_state == MST_SERVE_2) ? fifo_data_out_2 :
                   (r_mst_state == MST_SERVE_3) ? fifo_data_out_3 :
                   (r_mst_state == MST_SERVE_4) ? fifo_data_out_4 :
                   {AXIS_FIFO_WIDTH{1'b0}};

assign cur_empty = (r_mst_state == MST_SERVE_0) ? empty_0 :
                   (r_mst_state == MST_SERVE_1) ? empty_1 :
                   (r_mst_state == MST_SERVE_2) ? empty_2 :
                   (r_mst_state == MST_SERVE_3) ? empty_3 :
                   (r_mst_state == MST_SERVE_4) ? empty_4 :
                   1'b1;

// --- Unpack FIFO fields from cur_data ---
// tkeep is carried verbatim from the slave side (straddle off → s_axis_tkeep is
// authoritative per PG343), not reconstructed from the straddle-only eop_ptr.
wire [TKEEP_WIDTH-1:0]      cur_tkeep  = cur_data[AXIS_FIFO_WIDTH-1 : AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH + 2];
wire                        cur_tlast  = cur_data[AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH + 1];
wire [AXIS_TUSER_WIDTH-1:0] cur_tuser  = cur_data[AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH - 1 : AXIS_DATA_WIDTH];
wire [AXIS_DATA_WIDTH-1:0]  cur_tdata  = cur_data[AXIS_DATA_WIDTH-1:0];

// --- AXI-S master handshake ---
wire cur_valid = (r_mst_state != MST_IDLE) && !cur_empty;
wire cur_fire  = cur_valid && m_axis_tready;    // accepted beat this cycle

// --- Master output assignments ---
assign m_axis_tvalid = cur_valid;
assign m_axis_tdata  = cur_tdata;
assign m_axis_tkeep  = cur_tkeep;
assign m_axis_tlast  = cur_tlast;
assign m_axis_tuser  = cur_tuser;

// --- FIFO read-enable assignments ---
// Assert rd_en on the current FIFO whenever a beat is accepted downstream.
// FWFT FIFOs: rd_en pops the current word; next word appears immediately.
assign rd_en_0 = (r_mst_state == MST_SERVE_0) && cur_fire;
assign rd_en_1 = (r_mst_state == MST_SERVE_1) && cur_fire;
assign rd_en_2 = (r_mst_state == MST_SERVE_2) && cur_fire;
assign rd_en_3 = (r_mst_state == MST_SERVE_3) && cur_fire;
assign rd_en_4 = (r_mst_state == MST_SERVE_4) && cur_fire;


// =================================================================
// AXI Slave-Facing Process
//
// On every AXI-S valid+ready handshake, by routing class (see CLS_* above):
//   • CLS_MWR  → FIFO 0 (ch0) / FIFO 1 (ch1)
//   • CLS_MRD  → FIFO 3 (ch0) / FIFO 4 (ch1), dedicated-MRd build only
//   • CLS_PASS → FIFO 2
//       FIFO 2 is shared, but a packet-ownership lock (r_f2_lock) grants it to
//       one channel for the duration of a pass-through packet, so beats from
//       the two channels never interleave within a FIFO-2 packet.
//
// Ready de-assertion (pass-through):
//   • FIFO 2 almost-full        → back-pressures pass-through on both channels.
//   • Lock held by other channel→ back-pressures the non-owning channel.
//   • Ordering barrier not clear→ back-pressures a channel's pass-through until
//                                 that channel's batching FIFOs have drained.
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_fifo0_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_0_r           <= 1'b0;
        r_fifo1_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_1_r           <= 1'b0;
        r_fifo2_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_2_r           <= 1'b0;
        r_fifo3_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_3_r           <= 1'b0;
        r_fifo4_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_4_r           <= 1'b0;
        r_f2_lock             <= 1'b0;
        r_f2_lock_ch          <= 1'b0;
        r_ch0_inpkt           <= 1'b0;
        r_ch0_class           <= CLS_PASS;
        r_ch1_inpkt           <= 1'b0;
        r_ch1_class           <= CLS_PASS;
    end else begin

        // ----------------------------------------------------------
        // Default: clear write-enable strobes every cycle
        // ----------------------------------------------------------
        r_wr_en_0_r           <= 1'b0;
        r_wr_en_1_r           <= 1'b0;
        r_wr_en_2_r           <= 1'b0;
        r_wr_en_3_r           <= 1'b0;
        r_wr_en_4_r           <= 1'b0;

        // ----------------------------------------------------------
        // FIFO 0 / FIFO 1 — MWr-class beats (plus MRd in shared mode)
        // ----------------------------------------------------------
        if (ch0_fire_mwr) begin
            r_fifo0_din <= pack_ch0;
            r_wr_en_0_r <= 1'b1;
        end

        if (ch1_fire_mwr) begin
            r_fifo1_din <= pack_ch1;
            r_wr_en_1_r <= 1'b1;
        end

        // ----------------------------------------------------------
        // FIFO 3 / FIFO 4 — dedicated MRd beats.  MRd is descriptor-only
        // (single-beat), so these need no ownership lock: each write is a
        // complete packet and the FIFOs are per-channel.
        // ----------------------------------------------------------
        if (ch0_fire_mrd) begin
            r_fifo3_din <= pack_ch0;
            r_wr_en_3_r <= 1'b1;
        end

        if (ch1_fire_mrd) begin
            r_fifo4_din <= pack_ch1;
            r_wr_en_4_r <= 1'b1;
        end

        // ----------------------------------------------------------
        // FIFO 2 — pass-through beats.
        //
        // The packet-ownership lock plus channel-0 priority (enforced by the
        // ready logic) guarantee at most one channel fires a pass-through beat
        // in any cycle, and that all beats of a packet arrive from the same
        // channel back-to-back.  So a plain two-way write — no cache — keeps
        // every FIFO-2 packet contiguous.
        // ----------------------------------------------------------
        if (ch0_fire_pass) begin
            r_fifo2_din <= pack_ch0;
            r_wr_en_2_r <= 1'b1;
        end else if (ch1_fire_pass) begin
            r_fifo2_din <= pack_ch1;
            r_wr_en_2_r <= 1'b1;
        end

        // ----------------------------------------------------------
        // FIFO 2 ownership lock: taken on a multi-beat pass-through SOP,
        // released on its EOP.  A single-beat packet never locks (the acquire
        // and release collapse into the EOP-clear, which wins).
        // ----------------------------------------------------------
        if (ch0_fire_pass) begin
            if (s_axis_tlast_0)        r_f2_lock <= 1'b0;
            else if (!r_f2_lock) begin r_f2_lock <= 1'b1; r_f2_lock_ch <= 1'b0; end
        end else if (ch1_fire_pass) begin
            if (s_axis_tlast_1)        r_f2_lock <= 1'b0;
            else if (!r_f2_lock) begin r_f2_lock <= 1'b1; r_f2_lock_ch <= 1'b1; end
        end

        // ----------------------------------------------------------
        // Per-channel packet-class latch.
        // Capture the class from the SOP/descriptor beat and hold it across the
        // packet's payload beats; clear at EOP.  Keyed on ch_x_fire so it
        // tracks accepted beats only.  A single-beat packet (SOP==EOP) clears
        // without ever setting r_chx_inpkt, so its class comes from the live
        // decode — exactly as the master/ready logic uses it.  (MRd is always
        // single-beat, so a batched MRd never engages this latch; the latch
        // still pins MWr and pass-through packets against a mid-packet
        // batch_mrd toggle.)
        // ----------------------------------------------------------
        if (ch0_fire) begin
            if (s_axis_tlast_0)         r_ch0_inpkt <= 1'b0;
            else if (!r_ch0_inpkt) begin
                r_ch0_inpkt <= 1'b1;
                r_ch0_class <= cls_live_0;
            end
        end
        if (ch1_fire) begin
            if (s_axis_tlast_1)         r_ch1_inpkt <= 1'b0;
            else if (!r_ch1_inpkt) begin
                r_ch1_inpkt <= 1'b1;
                r_ch1_class <= cls_live_1;
            end
        end

    end
end

// =================================================================
// Batching-Priority Process — Internal Declarations
// =================================================================
//
// This block computes ONLY the internal batching priority used by the
// master-side arbiter.  All packet-level telemetry (length, gap, type,
// address, tag, counts) is gathered externally by the axi4_telemetry
// passthrough module, so the idle-gap / max-burst-depth / conflict logic
// that used to live here has been removed.
//
// FIFO 2 (pass-through) is served by the arbiter whenever it is non-empty, so it
// needs no batching triggers.  The four BATCHING FIFOs each get an identical set:
//   FIFO 0 = ch0 MWr    FIFO 1 = ch1 MWr
//   FIFO 3 = ch0 MRd    FIFO 4 = ch1 MRd   (dedicated-MRd builds only)
// Everything below is replicated per batching FIFO with the FIFO number as the
// signal suffix, so each one is individually traceable in a waveform / ILA.

// -----------------------------------------------------------------------------
// Per-packet depth accounting.
//
// depth_threshold counts PACKETS, not beats.  r_pkt_cnt_x is the number of
// COMPLETE packets resident in FIFO x: incremented when a packet's EOP beat
// commits to the FIFO, decremented when that EOP beat is popped on the master
// side.  Counting only complete packets means a depth-triggered batch can
// always stream out without the FSM stalling mid-packet waiting for beats that
// have not been written yet.
//
// Same exact, latency-independent construction as r_batch_inflight_x above:
// both edges are derived from the registered write commit and the read strobe,
// never from the FWFT empty/count flags, which lag.
// -----------------------------------------------------------------------------
localparam integer FIFO_TLAST_BIT = AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH + 1;

wire wr_eop_0 = r_wr_en_0_r && r_fifo0_din[FIFO_TLAST_BIT];
wire wr_eop_1 = r_wr_en_1_r && r_fifo1_din[FIFO_TLAST_BIT];
wire wr_eop_3 = r_wr_en_3_r && r_fifo3_din[FIFO_TLAST_BIT];
wire wr_eop_4 = r_wr_en_4_r && r_fifo4_din[FIFO_TLAST_BIT];
wire rd_eop_0 = rd_en_0 && cur_tlast;
wire rd_eop_1 = rd_en_1 && cur_tlast;
wire rd_eop_3 = rd_en_3 && cur_tlast;
wire rd_eop_4 = rd_en_4 && cur_tlast;

reg [FIFO_CNT_W-1:0] r_pkt_cnt_0, r_pkt_cnt_1, r_pkt_cnt_3, r_pkt_cnt_4;

// Depth comparison width: r_pkt_cnt is FIFO_CNT_W wide, depth_threshold is
// DEPTH_FEDILITY wide.  Widen both to the larger of the two (the continuous
// assignments zero-extend) so the compare is unsigned and width-safe whatever
// the parameters are.
localparam integer DEPTH_CMP_W = (FIFO_CNT_W > DEPTH_FEDILITY) ? FIFO_CNT_W : DEPTH_FEDILITY;

wire [DEPTH_CMP_W-1:0] pkt_cnt_0_ext = r_pkt_cnt_0;
wire [DEPTH_CMP_W-1:0] pkt_cnt_1_ext = r_pkt_cnt_1;
wire [DEPTH_CMP_W-1:0] pkt_cnt_3_ext = r_pkt_cnt_3;
wire [DEPTH_CMP_W-1:0] pkt_cnt_4_ext = r_pkt_cnt_4;
wire [DEPTH_CMP_W-1:0] depth_thr_ext = depth_threshold;

// Depth trigger: at least one whole packet present AND the packet count has
// reached the threshold.  The non-zero term keeps depth_threshold==0 from
// selecting a FIFO that holds nothing but a partial packet.
wire depth_active_0 = (r_pkt_cnt_0 != 0) && (pkt_cnt_0_ext >= depth_thr_ext);
wire depth_active_1 = (r_pkt_cnt_1 != 0) && (pkt_cnt_1_ext >= depth_thr_ext);
wire depth_active_3 = (r_pkt_cnt_3 != 0) && (pkt_cnt_3_ext >= depth_thr_ext);
wire depth_active_4 = (r_pkt_cnt_4 != 0) && (pkt_cnt_4_ext >= depth_thr_ext);

// -----------------------------------------------------------------------------
// Runtime max-packet-length tracking (beats), per batching FIFO.
//
// Tracked separately from the packet-depth accounting above, because the two
// answer different questions: depth = "how many packets are batched up", max
// length = "how much room must I keep free to admit one more whole packet".
//
// r_beat_cnt_x counts the beats of the packet currently being written into
// FIFO x.  At that packet's EOP its total length is compared against
// r_max_len_x, which is a high-water mark:
//   • seeded at MAX_PKT_BEATS (the declared worst case),
//   • grows immediately when a longer packet is seen,
//   • decays by one beat each time the FIFO fully drains, so a single outlier
//     packet does not inflate the full margin for the rest of the run,
//   • never falls below MAX_PKT_BEATS, and never exceeds MAX_LEN_CLAMP.
// Growth wins over decay when both land in the same cycle.
//
// FIFOs 3/4 carry MRd only, which is always a single-beat descriptor, so their
// trackers just sit at the MAX_PKT_BEATS floor.  The logic is replicated anyway
// rather than special-cased: uniform, and correct if anything longer is ever
// routed there.
// -----------------------------------------------------------------------------
reg [MAXLEN_W-1:0] r_beat_cnt_0, r_beat_cnt_1, r_beat_cnt_3, r_beat_cnt_4;
reg [MAXLEN_W-1:0] r_max_len_0,  r_max_len_1,  r_max_len_3,  r_max_len_4;

// Length of the packet completing this cycle = beats already counted + this one.
wire [MAXLEN_W-1:0] pkt_len_0 = r_beat_cnt_0 + 1'b1;
wire [MAXLEN_W-1:0] pkt_len_1 = r_beat_cnt_1 + 1'b1;
wire [MAXLEN_W-1:0] pkt_len_3 = r_beat_cnt_3 + 1'b1;
wire [MAXLEN_W-1:0] pkt_len_4 = r_beat_cnt_4 + 1'b1;

wire pkt_end_0 = ch0_fire_mwr && s_axis_tlast_0;
wire pkt_end_1 = ch1_fire_mwr && s_axis_tlast_1;
wire pkt_end_3 = ch0_fire_mrd && s_axis_tlast_0;
wire pkt_end_4 = ch1_fire_mrd && s_axis_tlast_1;

// Decay tick: the cycle the FIFO transitions to empty (one burst fully drained).
reg  r_empty_0_d, r_empty_1_d, r_empty_3_d, r_empty_4_d;
wire decay_0 = empty_0 && !r_empty_0_d;
wire decay_1 = empty_1 && !r_empty_1_d;
wire decay_3 = empty_3 && !r_empty_3_d;
wire decay_4 = empty_4 && !r_empty_4_d;

// -----------------------------------------------------------------------------
// FULL trigger (cannot hold another max-length packet).
//
// space_x = slots still admittable before prog_full back-pressures the slave
// interface (ADMIT_LIMIT_C - fill, floored at 0).  Once that drops below one
// full max-length packet the FIFO is "full" in the only sense that matters for
// batching — it can no longer absorb another whole packet — so it must be
// drained, and drained BEFORE prog_full stalls the input.
//
// With the defaults (FIFO_DEPTH=128, MAX_PKT_BEATS=16 → admit limit 109) this
// arms at fill 94, one max-length packet short of the stall point.
// -----------------------------------------------------------------------------
wire [FIFO_CNT_W-1:0] space_0 = (wr_data_count_0 < ADMIT_LIMIT_C)
                              ? (ADMIT_LIMIT_C - wr_data_count_0) : {FIFO_CNT_W{1'b0}};
wire [FIFO_CNT_W-1:0] space_1 = (wr_data_count_1 < ADMIT_LIMIT_C)
                              ? (ADMIT_LIMIT_C - wr_data_count_1) : {FIFO_CNT_W{1'b0}};
wire [FIFO_CNT_W-1:0] space_3 = (wr_data_count_3 < ADMIT_LIMIT_C)
                              ? (ADMIT_LIMIT_C - wr_data_count_3) : {FIFO_CNT_W{1'b0}};
wire [FIFO_CNT_W-1:0] space_4 = (wr_data_count_4 < ADMIT_LIMIT_C)
                              ? (ADMIT_LIMIT_C - wr_data_count_4) : {FIFO_CNT_W{1'b0}};

wire full_active_0 = !empty_0 && (space_0 < r_max_len_0);
wire full_active_1 = !empty_1 && (space_1 < r_max_len_1);
wire full_active_3 = !empty_3 && (space_3 < r_max_len_3);
wire full_active_4 = !empty_4 && (space_4 < r_max_len_4);

// Timeout counters — increment while FIFO is non-empty, saturate at
// time_threshold, reset to 0 when the FIFO empties (burst ended)
reg [TIME_FEDILITY-1:0]   r_to_cnt_0, r_to_cnt_1, r_to_cnt_3, r_to_cnt_4;

// 3-bit batching priority per batching FIFO (internal, consumed by the arbiter):
//   [0] = timeout reached, [1] = packet depth reached, [2] = full
reg [2:0]                 priority_0, priority_1, priority_3, priority_4;

// Combinatorial timeout condition per batching FIFO
wire to_active_0 = !empty_0 && (r_to_cnt_0 >= time_threshold);
wire to_active_1 = !empty_1 && (r_to_cnt_1 >= time_threshold);
wire to_active_3 = !empty_3 && (r_to_cnt_3 >= time_threshold);
wire to_active_4 = !empty_4 && (r_to_cnt_4 >= time_threshold);

// -----------------------------------------------------------------------------
// DRAIN state — one latch per batching FIFO.
//
// Every trigger means the same thing: "this FIFO should now be drained".  Any of
// them — timeout, packet depth, or full — SETS the drain latch, and the latch
// holds until the FIFO is EMPTY.  So a FIFO that has been declared ready is
// emptied completely, rather than served only until the momentary condition
// stops holding (which for the depth trigger would emit a single packet and
// re-batch, and for the full trigger would hover at the threshold).
//
// Fairness: the drain latch only makes a FIFO ELIGIBLE for the arbiter.  The FSM
// still re-arbitrates at every packet boundary, FIFO 2 keeps its interleave
// precedence, and the aging counters pick between two eligible channels — so a
// draining FIFO cannot monopolise the master interface.
// -----------------------------------------------------------------------------
reg r_drain_0, r_drain_1, r_drain_3, r_drain_4;

// =================================================================
// Batching-Priority Process
//
//  Timeout counter (per batching FIFO):
//    • Increments every cycle while the FIFO is non-empty.
//    • Saturates at time_threshold (no wrap-around).
//    • Resets when the FIFO empties (burst ended).
//    • priority[0] is asserted while counter >= time_threshold.
//
//  priority[1] (packet depth) and priority[2] (full) are registered replicas of
//  the combinatorial conditions above, updated every cycle.  Any of the three
//  sets that FIFO's drain latch, which holds until the FIFO is empty.
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_to_cnt_0   <= {TIME_FEDILITY{1'b0}};
        r_to_cnt_1   <= {TIME_FEDILITY{1'b0}};
        r_to_cnt_3   <= {TIME_FEDILITY{1'b0}};
        r_to_cnt_4   <= {TIME_FEDILITY{1'b0}};
        priority_0   <= 3'b000;
        priority_1   <= 3'b000;
        priority_3   <= 3'b000;
        priority_4   <= 3'b000;
        r_pkt_cnt_0  <= {FIFO_CNT_W{1'b0}};
        r_pkt_cnt_1  <= {FIFO_CNT_W{1'b0}};
        r_pkt_cnt_3  <= {FIFO_CNT_W{1'b0}};
        r_pkt_cnt_4  <= {FIFO_CNT_W{1'b0}};
        r_beat_cnt_0 <= {MAXLEN_W{1'b0}};
        r_beat_cnt_1 <= {MAXLEN_W{1'b0}};
        r_beat_cnt_3 <= {MAXLEN_W{1'b0}};
        r_beat_cnt_4 <= {MAXLEN_W{1'b0}};
        r_max_len_0  <= MAX_LEN_INIT;
        r_max_len_1  <= MAX_LEN_INIT;
        r_max_len_3  <= MAX_LEN_INIT;
        r_max_len_4  <= MAX_LEN_INIT;
        r_empty_0_d  <= 1'b1;
        r_empty_1_d  <= 1'b1;
        r_empty_3_d  <= 1'b1;
        r_empty_4_d  <= 1'b1;
        r_drain_0    <= 1'b0;
        r_drain_1    <= 1'b0;
        r_drain_3    <= 1'b0;
        r_drain_4    <= 1'b0;
    end else begin

        // ----------------------------------------------------------
        // Timeout counters
        // Reset on FIFO empty; count up to (and hold at) time_threshold
        // ----------------------------------------------------------
        if      (empty_0)                     r_to_cnt_0 <= {TIME_FEDILITY{1'b0}};
        else if (r_to_cnt_0 < time_threshold) r_to_cnt_0 <= r_to_cnt_0 + 1'b1;

        if      (empty_1)                     r_to_cnt_1 <= {TIME_FEDILITY{1'b0}};
        else if (r_to_cnt_1 < time_threshold) r_to_cnt_1 <= r_to_cnt_1 + 1'b1;

        if      (empty_3)                     r_to_cnt_3 <= {TIME_FEDILITY{1'b0}};
        else if (r_to_cnt_3 < time_threshold) r_to_cnt_3 <= r_to_cnt_3 + 1'b1;

        if      (empty_4)                     r_to_cnt_4 <= {TIME_FEDILITY{1'b0}};
        else if (r_to_cnt_4 < time_threshold) r_to_cnt_4 <= r_to_cnt_4 + 1'b1;

        // ----------------------------------------------------------
        // Resident complete-packet counters (per-packet depth)
        // ----------------------------------------------------------
        r_pkt_cnt_0 <= r_pkt_cnt_0 + (wr_eop_0 ? 1'b1 : 1'b0)
                                   - (rd_eop_0 ? 1'b1 : 1'b0);
        r_pkt_cnt_1 <= r_pkt_cnt_1 + (wr_eop_1 ? 1'b1 : 1'b0)
                                   - (rd_eop_1 ? 1'b1 : 1'b0);
        r_pkt_cnt_3 <= r_pkt_cnt_3 + (wr_eop_3 ? 1'b1 : 1'b0)
                                   - (rd_eop_3 ? 1'b1 : 1'b0);
        r_pkt_cnt_4 <= r_pkt_cnt_4 + (wr_eop_4 ? 1'b1 : 1'b0)
                                   - (rd_eop_4 ? 1'b1 : 1'b0);

        // ----------------------------------------------------------
        // In-progress packet beat counters (saturating)
        // ----------------------------------------------------------
        if (ch0_fire_mwr) begin
            if      (s_axis_tlast_0)                r_beat_cnt_0 <= {MAXLEN_W{1'b0}};
            else if (r_beat_cnt_0 < MAX_LEN_CLAMP)  r_beat_cnt_0 <= r_beat_cnt_0 + 1'b1;
        end
        if (ch1_fire_mwr) begin
            if      (s_axis_tlast_1)                r_beat_cnt_1 <= {MAXLEN_W{1'b0}};
            else if (r_beat_cnt_1 < MAX_LEN_CLAMP)  r_beat_cnt_1 <= r_beat_cnt_1 + 1'b1;
        end
        if (ch0_fire_mrd) begin
            if      (s_axis_tlast_0)                r_beat_cnt_3 <= {MAXLEN_W{1'b0}};
            else if (r_beat_cnt_3 < MAX_LEN_CLAMP)  r_beat_cnt_3 <= r_beat_cnt_3 + 1'b1;
        end
        if (ch1_fire_mrd) begin
            if      (s_axis_tlast_1)                r_beat_cnt_4 <= {MAXLEN_W{1'b0}};
            else if (r_beat_cnt_4 < MAX_LEN_CLAMP)  r_beat_cnt_4 <= r_beat_cnt_4 + 1'b1;
        end

        // ----------------------------------------------------------
        // Max-length high-water marks: grow on a longer packet (clamped),
        // else decay one beat per full drain, never below MAX_LEN_INIT.
        // ----------------------------------------------------------
        if (pkt_end_0 && (pkt_len_0 > r_max_len_0))
            r_max_len_0 <= (pkt_len_0 > MAX_LEN_CLAMP) ? MAX_LEN_CLAMP : pkt_len_0;
        else if (decay_0 && (r_max_len_0 > MAX_LEN_INIT))
            r_max_len_0 <= r_max_len_0 - 1'b1;

        if (pkt_end_1 && (pkt_len_1 > r_max_len_1))
            r_max_len_1 <= (pkt_len_1 > MAX_LEN_CLAMP) ? MAX_LEN_CLAMP : pkt_len_1;
        else if (decay_1 && (r_max_len_1 > MAX_LEN_INIT))
            r_max_len_1 <= r_max_len_1 - 1'b1;

        if (pkt_end_3 && (pkt_len_3 > r_max_len_3))
            r_max_len_3 <= (pkt_len_3 > MAX_LEN_CLAMP) ? MAX_LEN_CLAMP : pkt_len_3;
        else if (decay_3 && (r_max_len_3 > MAX_LEN_INIT))
            r_max_len_3 <= r_max_len_3 - 1'b1;

        if (pkt_end_4 && (pkt_len_4 > r_max_len_4))
            r_max_len_4 <= (pkt_len_4 > MAX_LEN_CLAMP) ? MAX_LEN_CLAMP : pkt_len_4;
        else if (decay_4 && (r_max_len_4 > MAX_LEN_INIT))
            r_max_len_4 <= r_max_len_4 - 1'b1;

        // Empty-edge detect for the decay tick
        r_empty_0_d <= empty_0;
        r_empty_1_d <= empty_1;
        r_empty_3_d <= empty_3;
        r_empty_4_d <= empty_4;

        // ----------------------------------------------------------
        // Priority signals (registered, updated every cycle)
        //   [0] = timeout : FIFO non-empty, counter has hit threshold
        //   [1] = depth   : resident packet count has reached depth_threshold
        //   [2] = full    : free space below one full max-length packet
        // ----------------------------------------------------------
        priority_0 <= {full_active_0, depth_active_0, to_active_0};
        priority_1 <= {full_active_1, depth_active_1, to_active_1};
        priority_3 <= {full_active_3, depth_active_3, to_active_3};
        priority_4 <= {full_active_4, depth_active_4, to_active_4};

        // ----------------------------------------------------------
        // Drain latches: set by any trigger, cleared once the FIFO is truly
        // drained.  The clear uses the exact fx_pending in-flight counter, NOT
        // empty_x: the FWFT empty flag lags a write, so it can read high while
        // beats are still resident, and clearing on it ends a drain episode
        // early and leaves a partial batch behind (observed in simulation).
        // fx_pending is write-commit/read-exact, so it falls only when the last
        // beat has actually been read out.  The clear takes precedence, so the
        // latch always releases at the end of a drain even if a trigger is
        // still (stale) asserted.
        // ----------------------------------------------------------
        if      (!f0_pending)   r_drain_0 <= 1'b0;
        else if (|priority_0)   r_drain_0 <= 1'b1;

        if      (!f1_pending)   r_drain_1 <= 1'b0;
        else if (|priority_1)   r_drain_1 <= 1'b1;

        if      (!f3_pending)   r_drain_3 <= 1'b0;
        else if (|priority_3)   r_drain_3 <= 1'b1;

        if      (!f4_pending)   r_drain_4 <= 1'b0;
        else if (|priority_4)   r_drain_4 <= 1'b1;

    end
end


// --- Arbitration: next source after EOP (or when IDLE) ---
//
// Priority order:
//   1. FIFO 2 (pass-through): always interleaves when it has data — even
//      mid-burst.
//   2. The requesting channel's batching FIFO: MWr FIFO (0/1) first, else the
//      dedicated MRd FIFO (3/4).  A FIFO requests when it is draining, when a
//      trigger fires, or when a pass-through beat is stalled behind it on the
//      same channel and it must be emptied first to honour PCIe ordering (the
//      flush request releases the ch_bar_ok barrier in the ready logic).
//      Tie-breaking between the two channels:
//        • Any timeout flag asserted  → channel 1 always wins
//        • Otherwise                  → higher aging count (older) wins
//
// All three trigger kinds are equal reasons to drain, so none of them gets
// precedence over the others here; the drain latch is what makes a triggered
// FIFO keep winning until it is empty.
//
// Combinatorial; uses r_wait_0/r_wait_1 and the registered priority/drain state.
// -----------------------------------------------------------------------------
// Flush request, per batching FIFO: a pass-through beat on that channel is
// blocked by the ordering barrier until THIS FIFO drains.  It is deliberately
// keyed on the individual FIFO's pending flag, not the channel's: keying it on
// the channel would make FIFO 0 request service while empty (because FIFO 3 is
// the one holding data), and the arbiter would re-select the empty FIFO 0
// forever while FIFO 3 never drained.
wire f0_flush_req = ch0_pres_pass && f0_pending;
wire f1_flush_req = ch1_pres_pass && f1_pending;
wire f3_flush_req = ch0_pres_pass && f3_pending;
wire f4_flush_req = ch1_pres_pass && f4_pending;

// Serve-side MWr→MRd ordering interlock (dedicated-MRd builds).
//
// Splitting posted (MWr) and non-posted (MRd) traffic into parallel per-channel
// FIFOs means the FIFOs themselves no longer order them.  PCIe forbids a
// non-posted request passing an earlier posted one, so a channel's MRd FIFO is
// only served once that channel's MWr FIFO has fully drained: every MWr admitted
// before a queued MRd necessarily still sits in the MWr FIFO until then.
//
// The interlock is on the SERVE side, not admission, so MRd is never
// back-pressured at the input and can never head-of-line block the MWr stream
// behind it.  It costs some MRd latency (a queued read also waits for MWr that
// arrived after it), and it cannot deadlock: the MWr FIFO's own timeout trigger
// always eventually drains it, releasing the MRd FIFO.
//
// If this design's ordering model permits a read to pass a write, delete the
// two ch_mrd_ord_ok terms from trig_3/trig_4 and reads will batch fully
// independently of writes.
wire ch0_mrd_ord_ok = !f0_pending;
wire ch1_mrd_ord_ok = !f1_pending;

wire trig_0 = r_drain_0 || (|priority_0) || f0_flush_req;
wire trig_1 = r_drain_1 || (|priority_1) || f1_flush_req;
wire trig_3 = (r_drain_3 || (|priority_3) || f3_flush_req) && ch0_mrd_ord_ok;
wire trig_4 = (r_drain_4 || (|priority_4) || f4_flush_req) && ch1_mrd_ord_ok;

// Per-channel candidate: the MWr FIFO outranks the MRd FIFO (and by the
// interlock above the two can never both be requesting anyway).
wire       ch0_req = trig_0 || trig_3;
wire       ch1_req = trig_1 || trig_4;
wire [2:0] ch0_src = trig_0 ? MST_SERVE_0 : MST_SERVE_3;
wire [2:0] ch1_src = trig_1 ? MST_SERVE_1 : MST_SERVE_4;

// Timeout flag of whichever FIFO that channel is presenting.
wire ch0_sel_to = trig_0 ? priority_0[0] : priority_3[0];
wire ch1_sel_to = trig_1 ? priority_1[0] : priority_4[0];

reg [2:0] arb_src;
always @(*) begin
    if (!empty_2) begin
        arb_src = MST_SERVE_2;              // FIFO 2 interleave takes precedence
    end else if (ch0_req && ch1_req) begin  // both channels requesting — tie-break
        if (ch0_sel_to || ch1_sel_to)
            arb_src = ch1_src;              // any timeout event → channel 1 wins
        else
            // Serve the channel that has been waiting longer
            arb_src = (r_wait_0 >= r_wait_1) ? ch0_src : ch1_src;
    end else if (ch1_req) begin
        arb_src = ch1_src;
    end else if (ch0_req) begin
        arb_src = ch0_src;
    end else begin
        arb_src = MST_IDLE;                 // nothing ready
    end
end

// =================================================================
// Master Interface State Machine
//
//  • Transmits packets in packet-granularity: once a batching FIFO
//    starts a packet, it holds the master interface until that
//    packet's EOP is accepted by the downstream (m_axis_tready).
//  • All source FIFOs are mutually exclusive: none can start while
//    another is mid-packet.
//  • FIFO 2 is re-checked at every packet boundary; a FIFO 2 packet
//    can be injected between any two consecutive batched packets.
//  • IDLE: re-arbitrates every cycle until a source becomes available.
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_mst_state <= MST_IDLE;
        r_wait_0    <= 8'd0;
        r_wait_1    <= 8'd0;
        r_in_pkt    <= 1'b0;
    end else begin

        // ----------------------------------------------------------
        // Track packet-in-flight on the master side.
        //   Any accepted non-last beat opens a packet; the accepted
        //   EOP beat closes it.  Held across underrun bubbles (no fire).
        // ----------------------------------------------------------
        if (cur_fire)
            r_in_pkt <= ~cur_tlast;

        // ----------------------------------------------------------
        // Aging counters — per channel, covering both of that
        // channel's batching FIFOs.
        //   Clear while actively serving either of them.
        //   Increment (up to 8'hFF) whenever the channel is
        //   requesting but is not currently being served.
        // ----------------------------------------------------------
        if ((r_mst_state == MST_SERVE_0) || (r_mst_state == MST_SERVE_3))
            r_wait_0 <= 8'd0;
        else if (ch0_req && r_wait_0 < 8'hFF)
            r_wait_0 <= r_wait_0 + 1'b1;

        if ((r_mst_state == MST_SERVE_1) || (r_mst_state == MST_SERVE_4))
            r_wait_1 <= 8'd0;
        else if (ch1_req && r_wait_1 < 8'hFF)
            r_wait_1 <= r_wait_1 + 1'b1;

        // ----------------------------------------------------------
        // State transitions — occur only at packet boundaries
        // ----------------------------------------------------------
        case (r_mst_state)

            MST_IDLE: begin
                // No packet in flight; move to first available source
                if (arb_src != MST_IDLE)
                    r_mst_state <= arb_src;
            end

            MST_SERVE_0,
            MST_SERVE_1,
            MST_SERVE_2,
            MST_SERVE_3,
            MST_SERVE_4: begin
                // Re-arbitrate for the next packet when either:
                //   (a) downstream accepts this packet's EOP beat, or
                //   (b) the served FIFO has drained with no packet in
                //       flight (r_in_pkt == 0).
                //
                // Case (b) is essential: the arbiter selects a FIFO on
                // !empty, but on the cycle a FIFO's final (EOP) beat is
                // popped it still reads non-empty (FWFT), so arb_src can
                // re-select the same now-draining FIFO.  Without this
                // escape the FSM would wedge in a SERVE state forever
                // waiting for an EOP beat that can never arrive, dead-
                // locking the whole switch.  r_in_pkt==0 guarantees we
                // only leave at a true packet boundary, never mid-packet.
                if ((cur_fire && cur_tlast) || (cur_empty && !r_in_pkt))
                    r_mst_state <= arb_src;
            end

            default: r_mst_state <= MST_IDLE;

        endcase
    end
end




endmodule