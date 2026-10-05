// =================================================================
// axi4_telemetry_logger
//
// Merges the record taps of up to four axi4_telemetry instances (DMA_LOG = 1)
// into ONE AXI4-Stream that an external AXI DMA (S2MM, simple mode) writes
// into a single DDR region.  Record format: telemetry_dma_format.md.
//
// Every 16-byte record carries an info byte (byte 14) naming its source port
// and kind (TLP / dropped TLP / trailer / pad).  Records appear in DDR in EOP
// time order across all ports; TLPs that end in the same cycle are ordered by
// port index.
//
//   taps -> claim -> xpm_fifo_sync (one entry per cycle) -> expander -> beat packer -> m_axis_log
//
// Recording control (one-shot, edge-armed):
//   IDLE  : wait for a rising edge of log_enable (edges seen in any other state
//           are ignored, so re-arming needs log_enable to go low and high again).
//   REC   : every TLP end claims one region slot.  If the FIFO is full the
//           records are dropped, but their slots are still claimed and each is
//           later emitted as a dropped-TLP record (zero data, tagged with its
//           port), so every port's n-th record is still its n-th TLP.
//   FINAL : write the closing FIFO entry (dropped records still owed).
//   DRAIN : the expander emits the owed records, pad records up to the last
//           slot of a beat, then the TRAILER record (stop reason, counts,
//           duration) with tlast.  Back to IDLE once the DMA accepts it.
//
// One slot is always reserved for the trailer, so a recording never exceeds
// DMA_REGION_BYTES and always ends with tlast: the DMA closes the transfer and
// nothing beyond the region, or already written, is overwritten.
//
// FIFO entry = {final, vmask[P], zcnt[P][CW], rec[P][112]}.  For each port p
// in order: emit zcnt[p] dropped-TLP records, then rec[p] if vmask[p].
// Carrying the owed-drop counts in the next entry that fits keeps drops in
// their original per-port position with one FIFO write per cycle at most.
// =================================================================
module axi4_telemetry_logger #(
    parameter integer NUM_PORTS        = 3,        // record sources, 1..4 (port id = index)
    parameter integer FIFO_DEPTH       = 512,      // entries; power of two, >= 16
    parameter integer DMA_REGION_BYTES = 1048576,  // DDR region, bytes; multiple of LOG_TDATA_WIDTH/8, >= 2 beats
    parameter integer LOG_TDATA_WIDTH  = 128       // 128, 256, 512 or 1024
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Record taps from axi4_telemetry (port p = bit/slice p)
    input  wire [NUM_PORTS-1:0]          rec_valid,
    input  wire [NUM_PORTS*112-1:0]      rec_data,

    // Control / status.  log_enable is synchronised internally.
    input  wire                          log_enable,
    output wire                          log_busy,        // recording or draining to the DMA
    output wire                          log_done,        // last recording closed (tlast sent); cleared on re-arm
    output wire                          log_overflow,    // >=1 record dropped in the last/current recording
    output wire [1:0]                    log_stop_reason, // 0 = none yet, 1 = log_enable low, 2 = region full
    output wire [31:0]                   log_drop_count,  // TLP records dropped (all ports)

    // AXI4-Stream master to the AXI DMA S2MM slave
    output wire [LOG_TDATA_WIDTH-1:0]    m_axis_log_tdata,
    output wire [LOG_TDATA_WIDTH/8-1:0]  m_axis_log_tkeep,
    output wire                          m_axis_log_tvalid,
    output wire                          m_axis_log_tlast,
    input  wire                          m_axis_log_tready
);

// =================================================================
// Constants
// =================================================================
localparam integer P          = NUM_PORTS;
localparam integer REC_BITS   = 128;
localparam integer DATA_BITS  = 112;                              // record bytes 0..13
localparam integer K          = LOG_TDATA_WIDTH / REC_BITS;       // records per beat
localparam integer N          = DMA_REGION_BYTES / (REC_BITS/8);  // record slots in region
localparam integer CAP        = N - 1;                            // slots for TLP records (1 kept for trailer)
localparam integer CW         = $clog2(N + 1);
localparam integer KW         = (K > 1) ? $clog2(K) : 1;
localparam integer PW         = (P > 1) ? $clog2(P) : 1;
localparam integer FIFO_WIDTH = 1 + P + P*CW + P*DATA_BITS;

// Record kinds (info byte [3:2])
localparam [1:0] KIND_TLP     = 2'd0;
localparam [1:0] KIND_DROP    = 2'd1;
localparam [1:0] KIND_TRAILER = 2'd2;
localparam [1:0] KIND_PAD     = 2'd3;

// Stop reasons
localparam [1:0] STOP_NONE    = 2'd0;
localparam [1:0] STOP_ENABLE  = 2'd1;
localparam [1:0] STOP_FULL    = 2'd2;

// Info byte: [7] = 1 (record written by the logger), [3:2] kind, [1:0] port
function [7:0] info_byte;
    input [1:0] kind;
    input [1:0] port;
    info_byte = {1'b1, 3'b000, kind, port};
endfunction

// =================================================================
// log_enable synchroniser + edge detect
// =================================================================
(* ASYNC_REG = "TRUE" *) reg [1:0] r_en_sync;
reg                                r_en_prev;
wire en_s    = r_en_sync[1];
wire en_rise = en_s && !r_en_prev;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_en_sync <= 2'b00;
        r_en_prev <= 1'b0;
    end else begin
        r_en_sync <= {r_en_sync[0], log_enable};
        r_en_prev <= en_s;
    end
end

// =================================================================
// Record FIFO
// =================================================================
reg                   fifo_wr_en;
reg  [FIFO_WIDTH-1:0] fifo_din;
wire                  fifo_full;
wire                  fifo_wr_rst_busy;
wire                  fifo_rd_en;
wire [FIFO_WIDTH-1:0] fifo_dout;
wire                  fifo_empty;

wire fifo_no_room = fifo_full || fifo_wr_rst_busy;

xpm_fifo_sync #(
    .DOUT_RESET_VALUE    ("0"),
    .ECC_MODE            ("no_ecc"),
    .FIFO_MEMORY_TYPE    ("auto"),
    .FIFO_READ_LATENCY   (0),
    .FIFO_WRITE_DEPTH    (FIFO_DEPTH),
    .FULL_RESET_VALUE    (0),
    .PROG_EMPTY_THRESH   (10),
    .PROG_FULL_THRESH    (10),
    .RD_DATA_COUNT_WIDTH (1),
    .READ_DATA_WIDTH     (FIFO_WIDTH),
    .READ_MODE           ("fwft"),
    .SIM_ASSERT_CHK      (0),
    .USE_ADV_FEATURES    ("0000"),
    .WAKEUP_TIME         (0),
    .WRITE_DATA_WIDTH    (FIFO_WIDTH),
    .WR_DATA_COUNT_WIDTH (1)
) u_log_fifo (
    .sleep         (1'b0),
    .rst           (~rst_n),
    .wr_clk        (clk),
    .wr_en         (fifo_wr_en),
    .din           (fifo_din),
    .full          (fifo_full),
    .prog_full     (),
    .wr_data_count (),
    .overflow      (),
    .wr_rst_busy   (fifo_wr_rst_busy),
    .almost_full   (),
    .wr_ack        (),
    .rd_en         (fifo_rd_en),
    .dout          (fifo_dout),
    .empty         (fifo_empty),
    .prog_empty    (),
    .rd_data_count (),
    .underflow     (),
    .rd_rst_busy   (),
    .almost_empty  (),
    .data_valid    (),
    .injectsbiterr (1'b0),
    .injectdbiterr (1'b0),
    .sbiterr       (),
    .dbiterr       ()
);

// =================================================================
// Recording control FSM + slot claiming (FIFO write side)
// =================================================================
localparam [1:0] LG_IDLE  = 2'd0;
localparam [1:0] LG_REC   = 2'd1;
localparam [1:0] LG_FINAL = 2'd2;
localparam [1:0] LG_DRAIN = 2'd3;

reg [1:0]    r_lg_state;
reg [CW-1:0] r_slots;               // TLP slots claimed (incl. dropped)
reg [CW-1:0] r_pending [0:P-1];     // dropped records not yet represented in the FIFO
reg [CW-1:0] r_drop_cnt;
reg [31:0]   r_duration;            // cycles from arm to stop, saturating
reg [1:0]    r_stop_reason;
reg          r_done;
reg          r_overflow;

wire         log_tlast_fire;

// Claim TLP ends in port order until the region (minus the trailer) is full.
wire [CW-1:0] avail = CAP - r_slots;
reg  [P-1:0]  claim;
reg  [2:0]    n_claim;
reg           any_pending;
reg  [P*CW-1:0] pending_flat;
integer       ci;

always @(*) begin
    claim       = {P{1'b0}};
    n_claim     = 3'd0;
    any_pending = 1'b0;
    for (ci = 0; ci < P; ci = ci + 1) begin
        if (r_lg_state == LG_REC && rec_valid[ci] && (n_claim < avail)) begin
            claim[ci] = 1'b1;
            n_claim   = n_claim + 1'b1;
        end
        if (r_pending[ci] != {CW{1'b0}})
            any_pending = 1'b1;
        pending_flat[ci*CW +: CW] = r_pending[ci];
    end
end

wire any_claim   = |claim;
wire region_full = any_claim && (n_claim == avail);

always @(*) begin
    fifo_wr_en = 1'b0;
    fifo_din   = {FIFO_WIDTH{1'b0}};
    if (r_lg_state == LG_REC && (any_claim || any_pending) && !fifo_no_room) begin
        // TLPs of this cycle (if any) plus drops still owed
        fifo_wr_en = 1'b1;
        fifo_din   = {1'b0, claim, pending_flat, rec_data};
    end else if (r_lg_state == LG_FINAL && !fifo_no_room) begin
        fifo_wr_en = 1'b1;
        fifo_din   = {1'b1, {P{1'b0}}, pending_flat, {(P*DATA_BITS){1'b0}}};
    end
end

integer si;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_lg_state    <= LG_IDLE;
        r_slots       <= {CW{1'b0}};
        r_drop_cnt    <= {CW{1'b0}};
        r_duration    <= 32'd0;
        r_stop_reason <= STOP_NONE;
        r_done        <= 1'b0;
        r_overflow    <= 1'b0;
        for (si = 0; si < P; si = si + 1)
            r_pending[si] <= {CW{1'b0}};
    end else begin
        case (r_lg_state)
            LG_IDLE: begin
                if (en_rise) begin
                    r_slots       <= {CW{1'b0}};
                    r_drop_cnt    <= {CW{1'b0}};
                    r_duration    <= 32'd0;
                    r_stop_reason <= STOP_NONE;
                    r_done        <= 1'b0;
                    r_overflow    <= 1'b0;
                    for (si = 0; si < P; si = si + 1)
                        r_pending[si] <= {CW{1'b0}};
                    r_lg_state    <= LG_REC;
                end
            end

            LG_REC: begin
                if (r_duration != 32'hFFFF_FFFF)
                    r_duration <= r_duration + 1'b1;

                r_slots <= r_slots + n_claim;
                if (fifo_no_room) begin
                    for (si = 0; si < P; si = si + 1)
                        if (claim[si])
                            r_pending[si] <= r_pending[si] + 1'b1;
                    r_drop_cnt <= r_drop_cnt + n_claim;
                    if (any_claim)
                        r_overflow <= 1'b1;
                end else begin
                    for (si = 0; si < P; si = si + 1)
                        r_pending[si] <= {CW{1'b0}};   // carried by this cycle's entry
                end

                if (region_full) begin
                    r_stop_reason <= STOP_FULL;
                    r_lg_state    <= LG_FINAL;
                end else if (!en_s) begin
                    r_stop_reason <= STOP_ENABLE;
                    r_lg_state    <= LG_FINAL;
                end
            end

            LG_FINAL: begin
                if (!fifo_no_room) begin
                    for (si = 0; si < P; si = si + 1)
                        r_pending[si] <= {CW{1'b0}};
                    r_lg_state <= LG_DRAIN;
                end
            end

            LG_DRAIN: begin
                if (log_tlast_fire) begin
                    r_done     <= 1'b1;
                    r_lg_state <= LG_IDLE;
                end
            end

            default: r_lg_state <= LG_IDLE;
        endcase
    end
end

// =================================================================
// Expander (FIFO read side): one record per cycle
// =================================================================
wire                     h_valid = !fifo_empty;
wire                     h_final = fifo_dout[FIFO_WIDTH-1];
wire [P-1:0]             h_vmask = fifo_dout[FIFO_WIDTH-2 -: P];

reg  [CW-1:0]            r_zd [0:P-1];    // dropped records already emitted, per port
reg  [P-1:0]             r_vd;            // TLP record already emitted, per port
reg                      r_tail;          // emitting pad + trailer
reg  [KW-1:0]            r_tail_pad;      // pad records still to emit

wire                     pk_accept;

// Work left in the head entry, per port
reg  [P-1:0]             work;
reg  [P-1:0]             zwork;           // dropped records left
reg  [PW-1:0]            sel;
reg                      sel_found;
reg                      more_after;      // work remains in the entry after this emission
integer                  wi;

always @(*) begin
    for (wi = 0; wi < P; wi = wi + 1) begin
        zwork[wi] = (r_zd[wi] != fifo_dout[P*DATA_BITS + wi*CW +: CW]);
        work[wi]  = zwork[wi] || (h_vmask[wi] && !r_vd[wi]);
    end
    sel       = {PW{1'b0}};
    sel_found = 1'b0;
    for (wi = P-1; wi >= 0; wi = wi - 1)
        if (work[wi]) begin
            sel       = wi;
            sel_found = 1'b1;
        end
    more_after = 1'b0;
    for (wi = 0; wi < P; wi = wi + 1)
        if (wi > sel && work[wi])
            more_after = 1'b1;
    // the selected port itself: another drop, or its TLP after the drops
    if (zwork[sel]) begin
        if ((r_zd[sel] + 1'b1) != fifo_dout[P*DATA_BITS + sel*CW +: CW])
            more_after = 1'b1;
        else if (h_vmask[sel] && !r_vd[sel])
            more_after = 1'b1;
    end
end

wire head_emit  = !r_tail && h_valid && sel_found && pk_accept;
wire head_pop   = !r_tail && h_valid && (!sel_found || (head_emit && !more_after));
wire tail_emit  = r_tail && pk_accept;
wire emit       = head_emit || tail_emit;
wire emit_last  = tail_emit && (r_tail_pad == {KW{1'b0}});

// Pad so the trailer is the last record of a beat
wire [KW-1:0] pad_cnt = (K - 1) - (r_slots % K);

assign fifo_rd_en = head_pop;

wire [DATA_BITS-1:0] trailer_data = {
    8'd0,                               // byte  13      reserved
    {6'd0, r_stop_reason},              // byte  12      stop_reason
    r_duration,                         // bytes 8..11   duration_cycles
    {{(32-CW){1'b0}}, r_drop_cnt},      // bytes 4..7    n_dropped
    {{(32-CW){1'b0}}, r_slots}          // bytes 0..3    n_records (TLP + dropped)
};

reg [REC_BITS-1:0] emit_rec;
always @(*) begin
    if (r_tail) begin
        if (r_tail_pad != {KW{1'b0}})
            emit_rec = {8'd0, info_byte(KIND_PAD, 2'd0), {DATA_BITS{1'b0}}};
        else
            emit_rec = {8'd0, info_byte(KIND_TRAILER, 2'd0), trailer_data};
    end else if (zwork[sel]) begin
        emit_rec = {8'd0, info_byte(KIND_DROP, sel), {DATA_BITS{1'b0}}};
    end else begin
        emit_rec = {8'd0, info_byte(KIND_TLP, sel), fifo_dout[sel*DATA_BITS +: DATA_BITS]};
    end
end

integer ei;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (ei = 0; ei < P; ei = ei + 1)
            r_zd[ei] <= {CW{1'b0}};
        r_vd       <= {P{1'b0}};
        r_tail     <= 1'b0;
        r_tail_pad <= {KW{1'b0}};
    end else begin
        if (head_pop) begin
            for (ei = 0; ei < P; ei = ei + 1)
                r_zd[ei] <= {CW{1'b0}};
            r_vd <= {P{1'b0}};
            if (h_final) begin
                r_tail     <= 1'b1;
                r_tail_pad <= pad_cnt;
            end
        end else if (head_emit) begin
            if (zwork[sel])
                r_zd[sel] <= r_zd[sel] + 1'b1;
            else
                r_vd[sel] <= 1'b1;
        end

        if (tail_emit) begin
            if (emit_last)
                r_tail <= 1'b0;
            else
                r_tail_pad <= r_tail_pad - 1'b1;
        end
    end
end

// =================================================================
// Beat packer: K records per beat, record i at tdata[128*i +: 128] so
// records land in DDR in order.  The pad count guarantees the trailer
// (tlast) closes a beat.
// =================================================================
reg [LOG_TDATA_WIDTH-1:0] r_asm;
reg [KW-1:0]              r_asm_idx;
reg [LOG_TDATA_WIDTH-1:0] r_out_data;
reg                       r_out_valid;
reg                       r_out_last;
reg [LOG_TDATA_WIDTH-1:0] beat_next;

wire out_free     = !r_out_valid || m_axis_log_tready;
wire asm_complete = (r_asm_idx == K - 1);

assign pk_accept = !asm_complete || out_free;

always @(*) begin
    beat_next = r_asm;
    beat_next[r_asm_idx*REC_BITS +: REC_BITS] = emit_rec;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_asm       <= {LOG_TDATA_WIDTH{1'b0}};
        r_asm_idx   <= {KW{1'b0}};
        r_out_data  <= {LOG_TDATA_WIDTH{1'b0}};
        r_out_valid <= 1'b0;
        r_out_last  <= 1'b0;
    end else begin
        if (r_out_valid && m_axis_log_tready)
            r_out_valid <= 1'b0;

        if (emit) begin
            if (asm_complete) begin
                r_out_data  <= beat_next;
                r_out_valid <= 1'b1;
                r_out_last  <= emit_last;
                r_asm       <= {LOG_TDATA_WIDTH{1'b0}};
                r_asm_idx   <= {KW{1'b0}};
            end else begin
                r_asm       <= beat_next;
                r_asm_idx   <= r_asm_idx + 1'b1;
            end
        end
    end
end

// synthesis translate_off
always @(posedge clk) begin
    if (rst_n && emit && emit_last && !asm_complete)
        $display("ERROR: %m trailer does not close a beat (t=%0t)", $time);
end
// synthesis translate_on

assign log_tlast_fire    = r_out_valid && r_out_last && m_axis_log_tready;

assign m_axis_log_tdata  = r_out_data;
assign m_axis_log_tkeep  = {(LOG_TDATA_WIDTH/8){1'b1}};
assign m_axis_log_tvalid = r_out_valid;
assign m_axis_log_tlast  = r_out_last;

assign log_busy          = (r_lg_state != LG_IDLE);
assign log_done          = r_done;
assign log_overflow      = r_overflow;
assign log_stop_reason   = r_stop_reason;
assign log_drop_count    = r_drop_cnt;   // zero-extended to 32 bits

endmodule
