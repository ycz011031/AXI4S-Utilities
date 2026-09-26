// =================================================================
// axi4_mwr_batch_top
//
// Top-level wrapper that instantiates the MWr batching engine
// (axi4_mwr_batch) preceded by a per-port telemetry passthrough
// (axi4_telemetry) on each of its two slave inputs.
//
// Dataflow:
//   in0 -> u_telemetry_0 (RQ) -> axi4_mwr_batch.s_axis_0 -.
//                                                          +-> m_axis (out)
//   in1 -> u_telemetry_1 (RQ) -> axi4_mwr_batch.s_axis_1 -'
//
// Configured for the Requester Request (RQ) interface:
//   - IF_TYPE = "RQ" on all sub-modules (selects tuser sop/eop offsets)
//   - AXIS_TUSER_WIDTH = 137 (RQ non-PASID tuser width, PG343 §3.1/§3.2)
//
// axi4_telemetry is a transparent passthrough that snoops each input
// stream and records per-packet telemetry (length, gap, type, address,
// tag); it does not alter the data path feeding the batcher.  With
// DMA_LOG = 0 the records are played back into an internal ILA; with
// DMA_LOG = 1 each port streams its records on m_axis_log_<n>_* to an
// external AXI DMA (S2MM), both ports armed by the shared log_enable.
// Record format: telemetry_dma_format.md.
// =================================================================
module axi4_mwr_batch_top #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 137,           // RQ tuser width (non-PASID)
    // --- Batcher (axi4_mwr_batch) ---
    parameter integer FIFO_DEPTH       = 128,
    parameter integer TIME_FEDILITY    = 8,
    parameter integer DEPTH_FEDILITY   = 8,
    parameter integer MAX_PKT_BEATS    = 16,            // assumed worst-case packet length, in beats
    parameter integer MRD_DEDICATED_FIFO = 0,           // 1 = batched MRd gets its own FIFO 3/4
    // --- Telemetry (axi4_telemetry) ---
    parameter integer TELEMETRY_DEPTH  = 512,
    parameter integer DATA_FIDELITY    = 8,
    parameter integer ILA_DEPTH        = 0,
    parameter         ENABLE_ILA       = 1,             // per-port ILA in each telemetry block (DMA_LOG = 0)
    parameter         DMA_LOG          = 0,             // 1 = log records to DDR via external AXI DMA
    parameter integer DMA_REGION_BYTES = 1048576,       // per-port DDR region size, bytes
    parameter integer LOG_TDATA_WIDTH  = 128            // m_axis_log_<n> tdata width
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Batching-priority thresholds (forwarded to axi4_mwr_batch).
    // depth_threshold counts whole PACKETS resident in a batching FIFO, not beats.
    input  wire [TIME_FEDILITY-1:0]      time_threshold,
    input  wire [DEPTH_FEDILITY-1:0]     depth_threshold,
    // Runtime enable for MRd batching (forwarded to axi4_mwr_batch).
    input  wire                          batch_mrd,

    // -------- Slave input port 0 (RQ) --------
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_0,
    input  wire [AXIS_DATA_WIDTH/32-1:0] s_axis_tkeep_0,   // PG343: tkeep is per-DWORD
    input  wire                          s_axis_tvalid_0,
    input  wire                          s_axis_tlast_0,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_0,
    output wire                          s_axis_tready_0,

    // -------- Slave input port 1 (RQ) --------
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_1,
    input  wire [AXIS_DATA_WIDTH/32-1:0] s_axis_tkeep_1,   // PG343: tkeep is per-DWORD
    input  wire                          s_axis_tvalid_1,
    input  wire                          s_axis_tlast_1,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_1,
    output wire                          s_axis_tready_1,

    // -------- Master output (RQ, batched) --------
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/32-1:0] m_axis_tkeep,    // PG343: tkeep is per-DWORD
    output wire                          m_axis_tvalid,
    output wire                          m_axis_tlast,
    output wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser,
    input  wire                          m_axis_tready,

    // -------- Telemetry DMA logging (DMA_LOG = 1) --------
    input  wire                          log_enable,       // rising edge arms both ports

    output wire                          log_busy_0,
    output wire                          log_done_0,
    output wire                          log_overflow_0,
    output wire [31:0]                   log_drop_count_0,
    output wire [LOG_TDATA_WIDTH-1:0]    m_axis_log_0_tdata,
    output wire [LOG_TDATA_WIDTH/8-1:0]  m_axis_log_0_tkeep,
    output wire                          m_axis_log_0_tvalid,
    output wire                          m_axis_log_0_tlast,
    input  wire                          m_axis_log_0_tready,

    output wire                          log_busy_1,
    output wire                          log_done_1,
    output wire                          log_overflow_1,
    output wire [31:0]                   log_drop_count_1,
    output wire [LOG_TDATA_WIDTH-1:0]    m_axis_log_1_tdata,
    output wire [LOG_TDATA_WIDTH/8-1:0]  m_axis_log_1_tkeep,
    output wire                          m_axis_log_1_tvalid,
    output wire                          m_axis_log_1_tlast,
    input  wire                          m_axis_log_1_tready
);

    // =============================================================
    // Telemetry -> Batcher interconnect (one set per input port)
    // =============================================================
    wire [AXIS_DATA_WIDTH-1:0]    tel0_tdata,  tel1_tdata;
    wire [AXIS_DATA_WIDTH/32-1:0] tel0_tkeep,  tel1_tkeep;   // PG343: tkeep is per-DWORD
    wire                          tel0_tvalid, tel1_tvalid;
    wire                          tel0_tlast,  tel1_tlast;
    wire [AXIS_TUSER_WIDTH-1:0]   tel0_tuser,  tel1_tuser;
    wire                          tel0_tready, tel1_tready;

    // =============================================================
    // Port 0 telemetry passthrough (RQ)
    // =============================================================
    axi4_telemetry #(
        .AXIS_DATA_WIDTH  (AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH (AXIS_TUSER_WIDTH),
        .TELEMETRY_DEPTH  (TELEMETRY_DEPTH),
        .DATA_FIDELITY    (DATA_FIDELITY),
        .ILA_DEPTH        (ILA_DEPTH),
        .ENABLE_ILA       (ENABLE_ILA),
        .IF_TYPE          ("RQ"),
        .DMA_LOG          (DMA_LOG),
        .DMA_REGION_BYTES (DMA_REGION_BYTES),
        .LOG_TDATA_WIDTH  (LOG_TDATA_WIDTH)
    ) u_telemetry_0 (
        .clk           (clk),
        .rst_n         (rst_n),
        // slave <- top input port 0
        .s_axis_tdata  (s_axis_tdata_0),
        .s_axis_tkeep  (s_axis_tkeep_0),
        .s_axis_tvalid (s_axis_tvalid_0),
        .s_axis_tlast  (s_axis_tlast_0),
        .s_axis_tuser  (s_axis_tuser_0),
        .s_axis_tready (s_axis_tready_0),
        // master -> batcher slave 0
        .m_axis_tdata  (tel0_tdata),
        .m_axis_tkeep  (tel0_tkeep),
        .m_axis_tvalid (tel0_tvalid),
        .m_axis_tlast  (tel0_tlast),
        .m_axis_tuser  (tel0_tuser),
        .m_axis_tready (tel0_tready),
        // DMA log
        .log_enable        (log_enable),
        .log_busy          (log_busy_0),
        .log_done          (log_done_0),
        .log_overflow      (log_overflow_0),
        .log_drop_count    (log_drop_count_0),
        .m_axis_log_tdata  (m_axis_log_0_tdata),
        .m_axis_log_tkeep  (m_axis_log_0_tkeep),
        .m_axis_log_tvalid (m_axis_log_0_tvalid),
        .m_axis_log_tlast  (m_axis_log_0_tlast),
        .m_axis_log_tready (m_axis_log_0_tready)
    );

    // =============================================================
    // Port 1 telemetry passthrough (RQ)
    // =============================================================
    axi4_telemetry #(
        .AXIS_DATA_WIDTH  (AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH (AXIS_TUSER_WIDTH),
        .TELEMETRY_DEPTH  (TELEMETRY_DEPTH),
        .DATA_FIDELITY    (DATA_FIDELITY),
        .ILA_DEPTH        (ILA_DEPTH),
        .ENABLE_ILA       (ENABLE_ILA),
        .IF_TYPE          ("RQ"),
        .DMA_LOG          (DMA_LOG),
        .DMA_REGION_BYTES (DMA_REGION_BYTES),
        .LOG_TDATA_WIDTH  (LOG_TDATA_WIDTH)
    ) u_telemetry_1 (
        .clk           (clk),
        .rst_n         (rst_n),
        // slave <- top input port 1
        .s_axis_tdata  (s_axis_tdata_1),
        .s_axis_tkeep  (s_axis_tkeep_1),
        .s_axis_tvalid (s_axis_tvalid_1),
        .s_axis_tlast  (s_axis_tlast_1),
        .s_axis_tuser  (s_axis_tuser_1),
        .s_axis_tready (s_axis_tready_1),
        // master -> batcher slave 1
        .m_axis_tdata  (tel1_tdata),
        .m_axis_tkeep  (tel1_tkeep),
        .m_axis_tvalid (tel1_tvalid),
        .m_axis_tlast  (tel1_tlast),
        .m_axis_tuser  (tel1_tuser),
        .m_axis_tready (tel1_tready),
        // DMA log
        .log_enable        (log_enable),
        .log_busy          (log_busy_1),
        .log_done          (log_done_1),
        .log_overflow      (log_overflow_1),
        .log_drop_count    (log_drop_count_1),
        .m_axis_log_tdata  (m_axis_log_1_tdata),
        .m_axis_log_tkeep  (m_axis_log_1_tkeep),
        .m_axis_log_tvalid (m_axis_log_1_tvalid),
        .m_axis_log_tlast  (m_axis_log_1_tlast),
        .m_axis_log_tready (m_axis_log_1_tready)
    );

    // =============================================================
    // MWr batching engine (RQ) — fed by both telemetry passthroughs
    // =============================================================
    axi4_mwr_batch #(
        .AXIS_DATA_WIDTH  (AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH (AXIS_TUSER_WIDTH),
        .FIFO_DEPTH       (FIFO_DEPTH),
        .TIME_FEDILITY    (TIME_FEDILITY),
        .DEPTH_FEDILITY   (DEPTH_FEDILITY),
        .MAX_PKT_BEATS    (MAX_PKT_BEATS),
        .MRD_DEDICATED_FIFO (MRD_DEDICATED_FIFO),
        .IF_TYPE          ("RQ")
    ) u_mwr_batch (
        .clk             (clk),
        .rst_n           (rst_n),
        .time_threshold  (time_threshold),
        .depth_threshold (depth_threshold),
        .batch_mrd       (batch_mrd),
        // slave 0 <- telemetry 0
        .s_axis_tdata_0  (tel0_tdata),
        .s_axis_tkeep_0  (tel0_tkeep),
        .s_axis_tvalid_0 (tel0_tvalid),
        .s_axis_tlast_0  (tel0_tlast),
        .s_axis_tuser_0  (tel0_tuser),
        .s_axis_tready_0 (tel0_tready),
        // slave 1 <- telemetry 1
        .s_axis_tdata_1  (tel1_tdata),
        .s_axis_tkeep_1  (tel1_tkeep),
        .s_axis_tvalid_1 (tel1_tvalid),
        .s_axis_tlast_1  (tel1_tlast),
        .s_axis_tuser_1  (tel1_tuser),
        .s_axis_tready_1 (tel1_tready),
        // master -> top output
        .m_axis_tdata    (m_axis_tdata),
        .m_axis_tkeep    (m_axis_tkeep),
        .m_axis_tvalid   (m_axis_tvalid),
        .m_axis_tlast    (m_axis_tlast),
        .m_axis_tuser    (m_axis_tuser),
        .m_axis_tready   (m_axis_tready)
    );

endmodule
