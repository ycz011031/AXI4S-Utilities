module axi4_mwr_batch #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer AXIS_FIFO_WIDTH  = AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH + 2, // Data + TUSER + SOP/EOP
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
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep_0,
    input  wire                          s_axis_tvalid_0,
    input  wire                          s_axis_tlast_0,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_0,
    output wire                          s_axis_tready_0,

    // AXX4 Slave AXIS interface 1
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_1,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep_1,
    input  wire                          s_axis_tvalid_1,
    input  wire                          s_axis_tlast_1,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_1,
    output wire                          s_axis_tready_1,

    // AXI4 Master Read Address Channel
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_tkeep,
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
// Format: { tlast[1], sop[1], tuser[AXIS_TUSER_WIDTH], tdata[AXIS_DATA_WIDTH] }
// Total  = 1 + 1 + 183 + 512 = 697 = AXIS_FIFO_WIDTH
// =================================================================
wire [AXIS_FIFO_WIDTH-1:0] pack_ch0;
wire [AXIS_FIFO_WIDTH-1:0] pack_ch1;

assign pack_ch0 = {s_axis_tlast_0, sop_0[0], s_axis_tuser_0, s_axis_tdata_0};
assign pack_ch1 = {s_axis_tlast_1, sop_1[0], s_axis_tuser_1, s_axis_tdata_1};

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

// One-beat cache for FIFO 2 arbitration.
// Needed when both channels present a non-MWr beat in the same cycle;
// channel 0 wins and channel 1's beat is held here until FIFO 2 is free.
reg  [AXIS_FIFO_WIDTH-1:0] r_cache2_data;
reg                         r_cache2_valid; // 1 = cache holds a pending beat
reg                         r_cache2_ch;    // 0 = buffered from ch0, 1 = from ch1

// =================================================================
// Type Detection Wires
// =================================================================
wire is_mwr_0 = (request_type_0 == MWR_TYPE);
wire is_mwr_1 = (request_type_1 == MWR_TYPE);

// =================================================================
// Ready Signals — Combinatorial, Type-Aware
//
//  Backpressure gates on almost_full (prog_full), NOT raw full.  Because the
//  write path is registered (accepted beat commits one cycle later) and
//  prog_full has its own assertion latency, gating on full would let beats be
//  accepted into a FIFO that has no room by the time the registered write
//  commits — those writes overflow and are dropped.  prog_full reserves
//  PROG_FULL_HEADROOM slots so every accepted beat is guaranteed a landing
//  spot, and a started packet can stream to EOP without overflow.
//
//  MWr path  (type 0001): ready when the dedicated FIFO (0 or 1) is not
//                          almost-full.
//  Non-MWr path          : ready when FIFO 2 is not almost-full AND the
//                          one-beat cache is empty (cache full ⇒ FIFO 2 still
//                          draining).
//  valid=0               : assert ready optimistically; the correct gate will
//                          fire on the cycle valid is presented.
// =================================================================
wire s_tready_0_w = s_axis_tvalid_0
    ? (is_mwr_0 ? !almost_full_0 : (!almost_full_2 && !r_cache2_valid))
    : 1'b1;

wire s_tready_1_w = s_axis_tvalid_1
    ? (is_mwr_1 ? !almost_full_1 : (!almost_full_2 && !r_cache2_valid))
    : 1'b1;

assign s_axis_tready_0 = s_tready_0_w;
assign s_axis_tready_1 = s_tready_1_w;

// =================================================================
// AXI-S Handshake Fire Signals
// =================================================================
wire ch0_fire        = s_axis_tvalid_0 && s_tready_0_w;
wire ch1_fire        = s_axis_tvalid_1 && s_tready_1_w;
wire ch0_fire_mwr    = ch0_fire &&  is_mwr_0;
wire ch1_fire_mwr    = ch1_fire &&  is_mwr_1;
wire ch0_fire_nonmwr = ch0_fire && !is_mwr_0;
wire ch1_fire_nonmwr = ch1_fire && !is_mwr_1;

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
// FIFO packing: { tlast[1], sop[1], tuser[AXIS_TUSER_WIDTH], tdata[AXIS_DATA_WIDTH] }
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
wire                        cur_tlast  = cur_data[AXIS_FIFO_WIDTH-1];
wire [AXIS_TUSER_WIDTH-1:0] cur_tuser  = cur_data[AXIS_DATA_WIDTH + AXIS_TUSER_WIDTH - 1 : AXIS_DATA_WIDTH];
wire [AXIS_DATA_WIDTH-1:0]  cur_tdata  = cur_data[AXIS_DATA_WIDTH-1:0];

// eop_ptr: byte index (0-based) of the last valid byte in this beat
// stored in tuser[EOPPTR_LO+3:EOPPTR_LO] per the slave-side decode (CQ/RQ)
wire [3:0] cur_eop_ptr = cur_tuser[EOPPTR_LO + 3 : EOPPTR_LO];

// --- TKEEP Recovery ---
// Non-EOP beat : every byte is valid → all 1s
// EOP beat     : bytes 0..eop_ptr are valid → (1 << (eop_ptr+1)) - 1
//   e.g. eop_ptr=11 → 64'h0000_0000_0000_0FFF
//   e.g. eop_ptr=63 → 64'hFFFF_FFFF_FFFF_FFFF  (overflow wraps: (1<<64)-1 = all-1s)
wire [AXIS_DATA_WIDTH/8-1:0] cur_tkeep =
    cur_tlast ? ((64'h1 << (cur_eop_ptr + 1)) - 64'h1)
              : {(AXIS_DATA_WIDTH/8){1'b1}};

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
//       FIFO 2 is shared; if both channels fire a non-MWr beat in the
//       same cycle, channel 0 is written directly while channel 1 is
//       stored in r_cache2.  The cache is drained on the next cycle
//       where no new non-MWr activity is present and FIFO 2 has space.
//
// Ready de-assertion:
//   • FIFO 0/1 full → back-pressures MWr on the respective channel.
//   • FIFO 2 full   → back-pressures non-MWr on both channels.
//   • Cache valid   → back-pressures non-MWr on both channels until
//                     the cached beat is drained into FIFO 2.
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_fifo0_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_0_r           <= 1'b0;
        r_fifo1_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_1_r           <= 1'b0;
        r_fifo2_din           <= {AXIS_FIFO_WIDTH{1'b0}};
        r_wr_en_2_r           <= 1'b0;
        r_cache2_data         <= {AXIS_FIFO_WIDTH{1'b0}};
        r_cache2_valid        <= 1'b0;
        r_cache2_ch           <= 1'b0;
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
        // FIFO 2 — Non-MWr beats, arbitrated between ch0 and ch1
        //
        // Backpressure invariant (enforced by ready logic above):
        //   When r_cache2_valid == 1, both channel readys for non-MWr
        //   are deasserted, so ch0_fire_nonmwr and ch1_fire_nonmwr
        //   are both 0.  The cache can therefore never be overwritten.
        //
        // Arbitration on conflict (both fire in the same cycle):
        //   Channel 0 has priority → written directly to FIFO 2.
        //   Channel 1 is buffered  → stored in r_cache2.
        //   Next idle cycle the cache is drained first.
        // ----------------------------------------------------------
        if (!ch0_fire_nonmwr && !ch1_fire_nonmwr) begin
            // No new non-MWr this cycle — drain pending cache if space available
            if (r_cache2_valid && !full_2) begin
                r_fifo2_din    <= r_cache2_data;
                r_wr_en_2_r    <= 1'b1;
                r_cache2_valid <= 1'b0;
            end

        end else if (ch0_fire_nonmwr && !ch1_fire_nonmwr) begin
            // Channel 0 non-MWr only — direct write; cache guaranteed empty
            r_fifo2_din <= pack_ch0;
            r_wr_en_2_r <= 1'b1;

        end else if (!ch0_fire_nonmwr && ch1_fire_nonmwr) begin
            // Channel 1 non-MWr only — direct write; cache guaranteed empty
            r_fifo2_din <= pack_ch1;
            r_wr_en_2_r <= 1'b1;

        end else begin
            // Conflict: both channels fire non-MWr in the same cycle.
            // Channel 0 wins → FIFO 2.  Channel 1 → one-beat cache.
            r_fifo2_din           <= pack_ch0;
            r_wr_en_2_r           <= 1'b1;
            r_cache2_data         <= pack_ch1;
            r_cache2_valid        <= 1'b1;
            r_cache2_ch           <= 1'b1;   // cache holds ch1 data
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
//   2. FIFO 0 / FIFO 1 (MWr batched): served only when a trigger is active.
//      Tie-breaking when both triggered simultaneously:
//        • Any timeout flag asserted  → FIFO 1 always wins
//        • Both depth-only (no TO)    → higher aging count (older) wins
//
// Combinatorial; uses r_wait_0/r_wait_1 and priority_x registered outputs.
reg [1:0] arb_src;
always @(*) begin
    if (!empty_2) begin
        arb_src = MST_SERVE_2;              // FIFO 2 interleave takes precedence
    end else begin
        case ({|priority_1, |priority_0})
            2'b11: begin                    // Both FIFO 0 and 1 triggered — tie-break
                if (priority_0[0] || priority_1[0])
                    arb_src = MST_SERVE_1;  // any timeout event → FIFO 1 wins
                else
                    // Pure depth tie → serve the one that has been waiting longer
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