// =================================================================
// axi4_mwr_batch_top
//
// Top-level wrapper that instantiates the MWr batching engine
// (axi4_mwr_batch) preceded by a per-port telemetry passthrough
// (axi4_telemetry) on each of its two slave inputs.
//
// Dataflow:
//   in0 -> u_telemetry_0 (RQ) -> axi4_mwr_batch.s_axis_0 -.
//                                                          +-> [u_telemetry_out] -> m_axis (out)
//   in1 -> u_telemetry_1 (RQ) -> axi4_mwr_batch.s_axis_1 -'
//
// Configured for the Requester Request (RQ) interface:
//   - IF_TYPE = "RQ" on all sub-modules (selects tuser sop/eop offsets)
//   - AXIS_TUSER_WIDTH = 137 (RQ non-PASID tuser width, PG343 §3.1/§3.2)
//
// axi4_telemetry is a transparent passthrough that snoops a stream and
// records per-packet telemetry (length, gap, type, address, tag); it does
// not alter the data path.
//
//   DMA_LOG = 0 : the two input taps play their records back into their own
//                 ILA (ENABLE_ILA).  The log_* / m_axis_log_* ports are inert.
//   DMA_LOG = 1 : only the batched output is logged, once per TLP.  A tap
//                 (u_telemetry_out) snoops the output, and u_logger streams its
//                 records, m_axis_log_*, to an external AXI DMA (S2MM).  The
//                 input taps are plain passthroughs.  The batcher stamps every
//                 packet on entry (INGRESS_TS), so each record also carries:
//                   pkt_wait : cycles from the packet's input SOP to its output
//                              SOP (time spent in the batcher), saturating
//                   port     : the input it came from, 0 = in0, 1 = in1
//                 Input SOP = output SOP - pkt_wait, so the input timeline is
//                 recovered from the output records without logging twice.
//                 Record format: telemetry_dma_format.md.
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
    // --- DMA logging (axi4_telemetry_logger, DMA_LOG = 1) ---
    parameter         DMA_LOG          = 0,             // 1 = log the output port to DDR via external AXI DMA
    parameter integer DMA_REGION_BYTES = 1048576,       // DDR region size, bytes
    parameter integer LOG_TDATA_WIDTH  = 128,           // m_axis_log tdata width
    parameter integer LOG_FIFO_DEPTH   = 512            // logger FIFO depth, entries (cycles with a TLP end)
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Batching-priority thresholds (forwarded to axi4_mwr_batch).
    // depth_threshold counts whole PACKETS resident in a batching FIFO, not beats.
    input  wire [TIME_FEDILITY-1:0]      time_threshold,
    input  wire [DEPTH_FEDILITY-1:0]     depth_threshold,
    // Runtime enable for MRd batching (forwarded to axi4_mwr_batch).
    input  wire                          batch_mrd,
    // Runtime drain size (forwarded to axi4_mwr_batch):
    // 0 = whole snapshot, 1 = at most depth_threshold packets per grant.
    input  wire                          drain_depth_cap,

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
    input  wire                          log_enable,       // rising edge arms; low (or region full) stops
    output wire                          log_busy,
    output wire                          log_done,
    output wire                          log_overflow,
    output wire [1:0]                    log_stop_reason,  // 1 = log_enable low, 2 = region full
    output wire [31:0]                   log_drop_count,
    output wire [LOG_TDATA_WIDTH-1:0]    m_axis_log_tdata,
    output wire [LOG_TDATA_WIDTH/8-1:0]  m_axis_log_tkeep,
    output wire                          m_axis_log_tvalid,
    output wire                          m_axis_log_tlast,
    input  wire                          m_axis_log_tready
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

    // Batcher -> output (through u_telemetry_out when DMA_LOG = 1)
    wire [AXIS_DATA_WIDTH-1:0]    bat_tdata;
    wire [AXIS_DATA_WIDTH/32-1:0] bat_tkeep;
    wire                          bat_tvalid;
    wire                          bat_tlast;
    wire [AXIS_TUSER_WIDTH-1:0]   bat_tuser;
    wire                          bat_tready;
    wire                          bat_src;     // input the presented beat came from
    wire [15:0]                   bat_wait;    // cycles since it was accepted there

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
        .DMA_LOG          (DMA_LOG)
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
        // no sideband; record tap unused (only the output is logged)
        .s_axis_wait   (16'd0),
        .s_axis_src    (2'd0),
        .log_rec_valid (),
        .log_rec_data  (),
        .log_rec_src   ()
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
        .DMA_LOG          (DMA_LOG)
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
        // no sideband; record tap unused (only the output is logged)
        .s_axis_wait   (16'd0),
        .s_axis_src    (2'd0),
        .log_rec_valid (),
        .log_rec_data  (),
        .log_rec_src   ()
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
        .INGRESS_TS       (DMA_LOG),                    // stamp packets only when logging
        .IF_TYPE          ("RQ")
    ) u_mwr_batch (
        .clk             (clk),
        .rst_n           (rst_n),
        .time_threshold  (time_threshold),
        .depth_threshold (depth_threshold),
        .batch_mrd       (batch_mrd),
        .drain_depth_cap (drain_depth_cap),
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
        // master -> output
        .m_axis_tdata    (bat_tdata),
        .m_axis_tkeep    (bat_tkeep),
        .m_axis_tvalid   (bat_tvalid),
        .m_axis_tlast    (bat_tlast),
        .m_axis_tuser    (bat_tuser),
        .m_axis_tready   (bat_tready),
        .m_axis_src      (bat_src),
        .m_axis_wait     (bat_wait)
    );

    generate
    if (DMA_LOG == 1) begin : gen_dma_log

        // Output record tap
        wire         out_rec_valid;
        wire [119:0] out_rec_data;
        wire [1:0]   out_rec_src;

        // =========================================================
        // Output-port telemetry tap (RQ) — passthrough on the batched stream.
        // pkt_wait and the source input come from the batcher's stamp.
        // =========================================================
        axi4_telemetry #(
            .AXIS_DATA_WIDTH  (AXIS_DATA_WIDTH),
            .AXIS_TUSER_WIDTH (AXIS_TUSER_WIDTH),
            .TELEMETRY_DEPTH  (TELEMETRY_DEPTH),
            .DATA_FIDELITY    (DATA_FIDELITY),
            .ILA_DEPTH        (ILA_DEPTH),
            .ENABLE_ILA       (0),
            .IF_TYPE          ("RQ"),
            .DMA_LOG          (1)
        ) u_telemetry_out (
            .clk           (clk),
            .rst_n         (rst_n),
            // slave <- batcher master
            .s_axis_tdata  (bat_tdata),
            .s_axis_tkeep  (bat_tkeep),
            .s_axis_tvalid (bat_tvalid),
            .s_axis_tlast  (bat_tlast),
            .s_axis_tuser  (bat_tuser),
            .s_axis_tready (bat_tready),
            // master -> top output
            .m_axis_tdata  (m_axis_tdata),
            .m_axis_tkeep  (m_axis_tkeep),
            .m_axis_tvalid (m_axis_tvalid),
            .m_axis_tlast  (m_axis_tlast),
            .m_axis_tuser  (m_axis_tuser),
            .m_axis_tready (m_axis_tready),
            // batcher sideband
            .s_axis_wait   (bat_wait),
            .s_axis_src    ({1'b0, bat_src}),
            // record tap
            .log_rec_valid (out_rec_valid),
            .log_rec_data  (out_rec_data),
            .log_rec_src   (out_rec_src)
        );

        // =========================================================
        // Output records -> one DMA stream.  Each record goes to the logger
        // port of the input its packet came from, so the record's port field
        // (and a dropped record's) names that input: 0 = in0, 1 = in1.
        // =========================================================
        axi4_telemetry_logger #(
            .NUM_PORTS        (2),
            .FIFO_DEPTH       (LOG_FIFO_DEPTH),
            .DMA_REGION_BYTES (DMA_REGION_BYTES),
            .LOG_TDATA_WIDTH  (LOG_TDATA_WIDTH)
        ) u_logger (
            .clk               (clk),
            .rst_n             (rst_n),
            .rec_valid         ({out_rec_valid && (out_rec_src == 2'd1),
                                 out_rec_valid && (out_rec_src == 2'd0)}),
            .rec_data          ({2{out_rec_data}}),
            .log_enable        (log_enable),
            .log_busy          (log_busy),
            .log_done          (log_done),
            .log_overflow      (log_overflow),
            .log_stop_reason   (log_stop_reason),
            .log_drop_count    (log_drop_count),
            .m_axis_log_tdata  (m_axis_log_tdata),
            .m_axis_log_tkeep  (m_axis_log_tkeep),
            .m_axis_log_tvalid (m_axis_log_tvalid),
            .m_axis_log_tlast  (m_axis_log_tlast),
            .m_axis_log_tready (m_axis_log_tready)
        );

    end else begin : gen_no_dma_log

        // Batcher drives the output directly
        assign m_axis_tdata      = bat_tdata;
        assign m_axis_tkeep      = bat_tkeep;
        assign m_axis_tvalid     = bat_tvalid;
        assign m_axis_tlast      = bat_tlast;
        assign m_axis_tuser      = bat_tuser;
        assign bat_tready        = m_axis_tready;

        assign log_busy          = 1'b0;
        assign log_done          = 1'b0;
        assign log_overflow      = 1'b0;
        assign log_stop_reason   = 2'd0;
        assign log_drop_count    = 32'd0;
        assign m_axis_log_tdata  = {LOG_TDATA_WIDTH{1'b0}};
        assign m_axis_log_tkeep  = {(LOG_TDATA_WIDTH/8){1'b0}};
        assign m_axis_log_tvalid = 1'b0;
        assign m_axis_log_tlast  = 1'b0;

    end
    endgenerate

endmodule
