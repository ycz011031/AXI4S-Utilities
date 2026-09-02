module axi4_mwr_batch #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer AXIS_FIFO_WIDTH  = AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH + AXIS_DATA_WIDTH/32 + 2, // Data + TUSER + TKEEP + SOP/EOP
    parameter integer FIFO_DEPTH       = 128,
    parameter integer TIME_FEDILITY = 8,
    parameter integer DEPTH_FEDILITY = 8,
    parameter         IF_TYPE = "CQ"  // "CQ" or "RQ" — selects tuser sideband bit layout (PG343)
)(
    input wire                          clk,
    input wire                          rst_n,

    input wire [TIME_FEDILITY-1:0]              time_threshold,
    input wire [DEPTH_FEDILITY-1:0]             depth_threshold,

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
    // batching priority (timeout / depth triggers) used by the arbiter.
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
// -----------------------------------------------------------------------------
localparam integer PROG_FULL_HEADROOM = 16;
localparam integer PROG_FULL_THRESH_C = FIFO_DEPTH - PROG_FULL_HEADROOM;

// FIFO 0 (ch0 MWr)
wire                         almost_full_0,  almost_empty_0;
wire                         full_0,         empty_0;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_0; // driven by slave-facing process (assign)
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_0;// driven by xpm_fifo_sync dout
wire                         wr_en_0;        // driven by slave-facing process (assign)
wire                         rd_en_0;        // driven by master interface (assign)
wire [FIFO_CNT_W-1:0]        wr_data_count_0, rd_data_count_0;
wire [5:0]                   data_count_0 = wr_data_count_0[5:0]; // 6-bit alias
wire                         data_valid_0;

// FIFO 1 (ch1 MWr)
wire                         almost_full_1,  almost_empty_1;
wire                         full_1,         empty_1;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_1;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_1;
wire                         wr_en_1;
wire                         rd_en_1;
wire [FIFO_CNT_W-1:0]        wr_data_count_1, rd_data_count_1;
wire [5:0]                   data_count_1 = wr_data_count_1[5:0];
wire                         data_valid_1;

// FIFO 2 (non-MWr / pass-through)
wire                         almost_full_2,  almost_empty_2;
wire                         full_2,         empty_2;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_in_2;
wire [AXIS_FIFO_WIDTH-1:0]   fifo_data_out_2;
wire                         wr_en_2;
wire                         rd_en_2;
wire [FIFO_CNT_W-1:0]        wr_data_count_2, rd_data_count_2;
wire [5:0]                   data_count_2 = wr_data_count_2[5:0];
wire                         data_valid_2;

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
localparam MWR_TYPE = 4'b0001;

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

// FIFO 2 packet-ownership lock.
// Once a channel begins writing a (multi-beat) non-MWr packet into the shared
// FIFO 2, it owns FIFO 2 until that packet's EOP.  This keeps every FIFO-2
// packet contiguous — without it, beats from two concurrent non-MWr packets
// (one per channel) would interleave into a single corrupted output packet.
reg                         r_f2_lock;     // 1 = a channel owns FIFO 2
reg                         r_f2_lock_ch;  // owning channel (0 or 1)

// Per-channel MWr/non-MWr packet-type latch.
// request_type (tdata[78:75]) lives in the SOP/descriptor beat only; on the
// payload beats of a multi-beat packet those bits are arbitrary.  Latch the type
// decided at SOP and hold it to EOP so every beat of a packet routes to the same
// FIFO and the ready/ordering logic sees one stable type.  Mirrors r_f2_lock.
reg                         r_ch0_inpkt;    // 1 = ch0 mid-packet (SOP seen, EOP not yet)
reg                         r_ch0_type_mwr; // latched is_mwr for the current ch0 packet
reg                         r_ch1_inpkt;
reg                         r_ch1_type_mwr;

// =================================================================
// Type Detection Wires
//
// is_mwr_x is the LIVE descriptor decode — valid only in the SOP beat.
// ch_x_eff_mwr is the per-beat EFFECTIVE type used by all routing/ready logic:
// the live decode at SOP (r_chx_inpkt==0), or the latched type mid-packet.
// =================================================================
wire is_mwr_0 = (request_type_0 == MWR_TYPE);
wire is_mwr_1 = (request_type_1 == MWR_TYPE);

wire ch0_eff_mwr = r_ch0_inpkt ? r_ch0_type_mwr : is_mwr_0;
wire ch1_eff_mwr = r_ch1_inpkt ? r_ch1_type_mwr : is_mwr_1;

// =================================================================
// Ready Signals — Combinatorial, Type-Aware
//
//  Backpressure gates on almost_full (prog_full), NOT raw full: the write path
//  is registered (an accepted beat commits one cycle later) and prog_full has
//  its own assertion latency.  prog_full reserves PROG_FULL_HEADROOM slots so
//  every accepted beat has a landing spot and a started packet streams to EOP.
//
//  MWr path (type 0001): ready when the dedicated FIFO (0/1) is not almost-full.
//
//  Non-MWr path: ready only when ALL hold —
//    • FIFO 2 not almost-full, AND
//    • ORDERING BARRIER clear (ch_bar_ok): that channel's MWr FIFO is empty
//      with no registered write in flight.  This stops a non-MWr request
//      (e.g. a read) being admitted — and thus emitted — ahead of an earlier
//      MWr on the same channel, which PCIe ordering forbids (a non-posted
//      request may not pass an earlier posted one).
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

// A channel has MWr still pending if any MWr beat it has accepted has not yet
// been read back out of its FIFO.  This is tracked with an explicit in-flight
// counter rather than `!empty || r_wr_en_r`.
//
// Why not `empty`: the xpm FWFT `empty` flag can lag the write by more than one
// cycle.  During that lag `empty` still reads high while `r_wr_en_x_r` has
// already cleared, so `!empty || r_wr_en_x_r` momentarily reports "drained"
// while an MWr beat is actually in flight.  The ordering barrier below would
// then admit a non-MWr into FIFO 2 ahead of that MWr; the FSM commits to that
// FIFO-2 packet (MST_SERVE_2, r_in_pkt=1) but can never finish it, because the
// rest of the packet now sits behind a (real) non-empty MWr FIFO that only the
// blocked FSM could drain — a cyclic, permanent deadlock.
//
// The counter is exact and latency-independent: +1 when a beat's registered
// write commits (r_wr_en_x_r), -1 when a beat is read out (rd_en_x).  ORing
// r_wr_en_x_r into `pending` covers the commit cycle itself, before the
// registered counter has incremented.  All terms are registered, so this
// introduces no combinational path through the tready logic.
reg [FIFO_CNT_W-1:0] r_mwr_inflight_0;
reg [FIFO_CNT_W-1:0] r_mwr_inflight_1;

wire ch0_mwr_pending = (r_mwr_inflight_0 != 0) || r_wr_en_0_r;
wire ch1_mwr_pending = (r_mwr_inflight_1 != 0) || r_wr_en_1_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_mwr_inflight_0 <= {FIFO_CNT_W{1'b0}};
        r_mwr_inflight_1 <= {FIFO_CNT_W{1'b0}};
    end else begin
        r_mwr_inflight_0 <= r_mwr_inflight_0 + (r_wr_en_0_r ? 1'b1 : 1'b0)
                                             - (rd_en_0     ? 1'b1 : 1'b0);
        r_mwr_inflight_1 <= r_mwr_inflight_1 + (r_wr_en_1_r ? 1'b1 : 1'b0)
                                             - (rd_en_1     ? 1'b1 : 1'b0);
    end
end

// Ordering barrier: a channel's non-MWr may enter FIFO 2 only once that
// channel's MWr FIFO has fully drained.
wire ch0_bar_ok = !ch0_mwr_pending;
wire ch1_bar_ok = !ch1_mwr_pending;

// A channel currently presenting a non-MWr beat.
wire ch0_pres_nonmwr = s_axis_tvalid_0 && !ch0_eff_mwr;
wire ch1_pres_nonmwr = s_axis_tvalid_1 && !ch1_eff_mwr;

// Channel 0 is about to take a free lock this cycle (channel 1 must defer).
wire ch0_wants_lock = ch0_pres_nonmwr && ch0_bar_ok && !almost_full_2;

// Non-MWr ready, per channel: FIFO 2 room + barrier clear + lock available.
wire ch0_nonmwr_ok = !almost_full_2 && ch0_bar_ok && (f2_own0 || f2_free);
wire ch1_nonmwr_ok = !almost_full_2 && ch1_bar_ok && (f2_own1 || (f2_free && !ch0_wants_lock));

wire s_tready_0_w = s_axis_tvalid_0 ? (ch0_eff_mwr ? !almost_full_0 : ch0_nonmwr_ok) : 1'b1;
wire s_tready_1_w = s_axis_tvalid_1 ? (ch1_eff_mwr ? !almost_full_1 : ch1_nonmwr_ok) : 1'b1;

assign s_axis_tready_0 = s_tready_0_w;
assign s_axis_tready_1 = s_tready_1_w;

// =================================================================
// AXI-S Handshake Fire Signals
// =================================================================
wire ch0_fire        = s_axis_tvalid_0 && s_tready_0_w;
wire ch1_fire        = s_axis_tvalid_1 && s_tready_1_w;
wire ch0_fire_mwr    = ch0_fire &&  ch0_eff_mwr;
wire ch1_fire_mwr    = ch1_fire &&  ch1_eff_mwr;
wire ch0_fire_nonmwr = ch0_fire && !ch0_eff_mwr;
wire ch1_fire_nonmwr = ch1_fire && !ch1_eff_mwr;

// =================================================================
// FIFO Output Assignments (driven from registers)
// =================================================================
assign fifo_data_in_0 = r_fifo0_din;
assign wr_en_0        = r_wr_en_0_r;
assign fifo_data_in_1 = r_fifo1_din;
assign wr_en_1        = r_wr_en_1_r;
assign fifo_data_in_2 = r_fifo2_din;
assign wr_en_2        = r_wr_en_2_r;

// =================================================================
// Master Interface Process — Declarations
// =================================================================

// --- State encoding ---
localparam [1:0] MST_IDLE    = 2'd0;
localparam [1:0] MST_SERVE_0 = 2'd1;   // transmitting a packet from FIFO 0 (ch0 MWr)
localparam [1:0] MST_SERVE_1 = 2'd2;   // transmitting a packet from FIFO 1 (ch1 MWr)
localparam [1:0] MST_SERVE_2 = 2'd3;   // transmitting a packet from FIFO 2 (non-MWr, interleaved)

// --- State register and FIFO 0/1 aging counters ---
reg [1:0] r_mst_state;
reg [7:0] r_wait_0;    // cycles FIFO 0 has been triggered-but-not-served (saturates at 8'hFF)
reg [7:0] r_wait_1;    // same for FIFO 1

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
                   {AXIS_FIFO_WIDTH{1'b0}};

assign cur_empty = (r_mst_state == MST_SERVE_0) ? empty_0 :
                   (r_mst_state == MST_SERVE_1) ? empty_1 :
                   (r_mst_state == MST_SERVE_2) ? empty_2 :
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


// =================================================================
// AXI Slave-Facing Process
//
// On every AXI-S valid+ready handshake:
//   • request_type == MWR_TYPE (4'b0001)
//       ch0 beat → FIFO 0
//       ch1 beat → FIFO 1
//   • All other request types → FIFO 2
//       FIFO 2 is shared, but a packet-ownership lock (r_f2_lock) grants it to
//       one channel for the duration of a non-MWr packet, so beats from the
//       two channels never interleave within a FIFO-2 packet.
//
// Ready de-assertion (non-MWr):
//   • FIFO 2 almost-full        → back-pressures non-MWr on both channels.
//   • Lock held by other channel→ back-pressures the non-owning channel.
//   • Ordering barrier not clear→ back-pressures a channel's non-MWr until
//                                 that channel's MWr FIFO has drained.
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_fifo0_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_0_r           <= 1'b0;
        r_fifo1_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_1_r           <= 1'b0;
        r_fifo2_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_2_r           <= 1'b0;
        r_f2_lock             <= 1'b0;
        r_f2_lock_ch          <= 1'b0;
        r_ch0_inpkt           <= 1'b0;
        r_ch0_type_mwr        <= 1'b0;
        r_ch1_inpkt           <= 1'b0;
        r_ch1_type_mwr        <= 1'b0;
    end else begin

        // ----------------------------------------------------------
        // Default: clear write-enable strobes every cycle
        // ----------------------------------------------------------
        r_wr_en_0_r           <= 1'b0;
        r_wr_en_1_r           <= 1'b0;
        r_wr_en_2_r           <= 1'b0;

        // ----------------------------------------------------------
        // FIFO 0 — MWr beats from channel 0
        // ----------------------------------------------------------
        if (ch0_fire_mwr) begin
            r_fifo0_din <= pack_ch0;
            r_wr_en_0_r <= 1'b1;
        end

        // ----------------------------------------------------------
        // FIFO 1 — MWr beats from channel 1
        // ----------------------------------------------------------
        if (ch1_fire_mwr) begin
            r_fifo1_din <= pack_ch1;
            r_wr_en_1_r <= 1'b1;
        end

        // ----------------------------------------------------------
        // FIFO 2 — Non-MWr beats.
        //
        // The packet-ownership lock plus channel-0 priority (enforced by the
        // ready logic) guarantee at most one channel fires a non-MWr beat in
        // any cycle, and that all beats of a packet arrive from the same
        // channel back-to-back.  So a plain two-way write — no cache — keeps
        // every FIFO-2 packet contiguous.
        // ----------------------------------------------------------
        if (ch0_fire_nonmwr) begin
            r_fifo2_din <= pack_ch0;
            r_wr_en_2_r <= 1'b1;
        end else if (ch1_fire_nonmwr) begin
            r_fifo2_din <= pack_ch1;
            r_wr_en_2_r <= 1'b1;
        end

        // ----------------------------------------------------------
        // FIFO 2 ownership lock: taken on a multi-beat non-MWr SOP, released
        // on its EOP.  A single-beat packet never locks (the acquire and
        // release collapse into the EOP-clear, which wins).
        // ----------------------------------------------------------
        if (ch0_fire_nonmwr) begin
            if (s_axis_tlast_0)        r_f2_lock <= 1'b0;
            else if (!r_f2_lock) begin r_f2_lock <= 1'b1; r_f2_lock_ch <= 1'b0; end
        end else if (ch1_fire_nonmwr) begin
            if (s_axis_tlast_1)        r_f2_lock <= 1'b0;
            else if (!r_f2_lock) begin r_f2_lock <= 1'b1; r_f2_lock_ch <= 1'b1; end
        end

        // ----------------------------------------------------------
        // Per-channel packet-type latch.
        // Capture is_mwr from the SOP/descriptor beat and hold it across the
        // packet's payload beats; clear at EOP.  Keyed on ch_x_fire so it
        // tracks accepted beats only.  A single-beat packet (SOP==EOP) clears
        // without ever setting r_chx_inpkt, so its type comes from the live
        // decode — exactly as the master/ready logic uses it.
        // ----------------------------------------------------------
        if (ch0_fire) begin
            if (s_axis_tlast_0)         r_ch0_inpkt <= 1'b0;
            else if (!r_ch0_inpkt) begin
                r_ch0_inpkt    <= 1'b1;
                r_ch0_type_mwr <= is_mwr_0;
            end
        end
        if (ch1_fire) begin
            if (s_axis_tlast_1)         r_ch1_inpkt <= 1'b0;
            else if (!r_ch1_inpkt) begin
                r_ch1_inpkt    <= 1'b1;
                r_ch1_type_mwr <= is_mwr_1;
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
// FIFO 2 (non-MWr) is served by the arbiter whenever it is non-empty, so it
// needs no timeout/depth priority — only FIFO 0 and FIFO 1 do.

// Zero-extended data_count for DEPTH_FEDILITY-wide comparisons
wire [DEPTH_FEDILITY-1:0] data_count_0_ext = {{(DEPTH_FEDILITY-6){1'b0}}, data_count_0};
wire [DEPTH_FEDILITY-1:0] data_count_1_ext = {{(DEPTH_FEDILITY-6){1'b0}}, data_count_1};

// Timeout counters — increment while FIFO is non-empty, saturate at
// time_threshold, reset to 0 when the FIFO empties (burst ended)
reg [TIME_FEDILITY-1:0]   r_to_cnt_0;
reg [TIME_FEDILITY-1:0]   r_to_cnt_1;

// 2-bit batching priority per MWr FIFO (internal, consumed by the arbiter):
//   [0] = timeout reached, [1] = depth reached
reg [1:0]                 priority_0;
reg [1:0]                 priority_1;

// Combinatorial timeout condition per MWr FIFO
wire to_active_0 = !empty_0 && (r_to_cnt_0 >= time_threshold);
wire to_active_1 = !empty_1 && (r_to_cnt_1 >= time_threshold);

// =================================================================
// Batching-Priority Process
//
//  Timeout counter (per MWr FIFO):
//    • Increments every cycle while the FIFO is non-empty.
//    • Saturates at time_threshold (no wrap-around).
//    • Resets when the FIFO empties (burst ended).
//    • priority[0] is asserted while counter >= time_threshold.
//
//  priority[1] (depth reached): registered replica of the
//  combinatorial depth comparison, updated every cycle.
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_to_cnt_0 <= {TIME_FEDILITY{1'b0}};
        r_to_cnt_1 <= {TIME_FEDILITY{1'b0}};
        priority_0 <= 2'b00;
        priority_1 <= 2'b00;
    end else begin

        // ----------------------------------------------------------
        // Timeout counters
        // Reset on FIFO empty; count up to (and hold at) time_threshold
        // ----------------------------------------------------------
        if      (empty_0)                     r_to_cnt_0 <= {TIME_FEDILITY{1'b0}};
        else if (r_to_cnt_0 < time_threshold) r_to_cnt_0 <= r_to_cnt_0 + 1'b1;

        if      (empty_1)                     r_to_cnt_1 <= {TIME_FEDILITY{1'b0}};
        else if (r_to_cnt_1 < time_threshold) r_to_cnt_1 <= r_to_cnt_1 + 1'b1;

        // ----------------------------------------------------------
        // Priority signals (registered, updated every cycle)
        //   [0] = timeout  : FIFO non-empty, counter has hit threshold
        //   [1] = depth    : data_count has reached depth_threshold
        // ----------------------------------------------------------
        priority_0 <= {(data_count_0_ext >= depth_threshold), to_active_0};
        priority_1 <= {(data_count_1_ext >= depth_threshold), to_active_1};

    end
end


// --- Arbitration: next source after EOP (or when IDLE) ---
//
// Priority order:
//   1. FIFO 2 (non-MWr): always interleaves when it has data — even mid-burst.
//   2. FIFO 0 / FIFO 1 (MWr batched): served when a batching trigger is active
//      (depth/timeout) OR when a non-MWr is stalled behind it on the same
//      channel and must be flushed first to honour PCIe ordering.  The flush
//      request drains the channel's MWr FIFO so the ordering barrier in the
//      ready logic (ch_bar_ok) can release the waiting non-MWr.
//      Tie-breaking when both triggered simultaneously:
//        • Any timeout flag asserted  → FIFO 1 always wins
//        • Otherwise                  → higher aging count (older) wins
//
// Combinatorial; uses r_wait_0/r_wait_1 and priority_x registered outputs.
wire ch0_flush_req = ch0_pres_nonmwr && ch0_mwr_pending; // non-MWr behind ch0 MWr
wire ch1_flush_req = ch1_pres_nonmwr && ch1_mwr_pending; // non-MWr behind ch1 MWr

wire trig_0 = (|priority_0) || ch0_flush_req;
wire trig_1 = (|priority_1) || ch1_flush_req;

reg [1:0] arb_src;
always @(*) begin
    if (!empty_2) begin
        arb_src = MST_SERVE_2;              // FIFO 2 interleave takes precedence
    end else begin
        case ({trig_1, trig_0})
            2'b11: begin                    // Both FIFO 0 and 1 triggered — tie-break
                if (priority_0[0] || priority_1[0])
                    arb_src = MST_SERVE_1;  // any timeout event → FIFO 1 wins
                else
                    // Serve the one that has been waiting longer
                    arb_src = (r_wait_0 >= r_wait_1) ? MST_SERVE_0 : MST_SERVE_1;
            end
            2'b10:   arb_src = MST_SERVE_1; // only FIFO 1 triggered
            2'b01:   arb_src = MST_SERVE_0; // only FIFO 0 triggered
            default: arb_src = MST_IDLE;    // nothing ready
        endcase
    end
end

// =================================================================
// Master Interface State Machine
//
//  • Transmits packets in packet-granularity: once FIFO 0 or 1 starts
//    a packet, it holds the master interface until that packet's EOP
//    is accepted by the downstream (m_axis_tready).
//  • FIFO 0 and FIFO 1 are mutually exclusive: neither can start while
//    the other is mid-packet.
//  • FIFO 2 is re-checked at every packet boundary; a FIFO 2 packet
//    can be injected between any two consecutive FIFO 0/1 packets.
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
        // Aging counters
        //   Clear while actively serving the FIFO.
        //   Increment (up to 8'hFF) whenever the FIFO has a trigger
        //   but is not currently being served.
        // ----------------------------------------------------------
        if (r_mst_state == MST_SERVE_0)
            r_wait_0 <= 8'd0;
        else if (|priority_0 && r_wait_0 < 8'hFF)
            r_wait_0 <= r_wait_0 + 1'b1;

        if (r_mst_state == MST_SERVE_1)
            r_wait_1 <= 8'd0;
        else if (|priority_1 && r_wait_1 < 8'hFF)
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
            MST_SERVE_2: begin
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