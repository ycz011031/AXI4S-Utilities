`timescale 1ns/1ps
// =================================================================
// telemetry_dma_tb
//
// Self-checking testbench for axi4_telemetry DMA logging mode (DMA_LOG = 1).
//
// Four DMA-mode instances with different stream widths / region sizes /
// FIFO depths / sink behaviour snoop the same RQ stimulus.  Each harness
// models the AXI DMA S2MM sink, collects the logged records per recording
// (delimited by tlast) and checks them positionally against the TLPs the
// testbench sent while log_enable was high:
//   - slot j holds either the exact expected record j or all zeros (a drop)
//   - number of zero slots == log_drop_count
//   - region full   -> exactly REGION records, tlast on the last one
//   - enable-low    -> sent records + 1..K zero terminator/padding records
//   - no beats outside a recording, tkeep all ones, AXIS data stable under stall
//   - re-arm only on a rising edge of log_enable seen while idle
//
// Also elaborates an ILA-mode instance (DMA_LOG = 0) and the top wrapper
// with DMA_LOG = 1 to check the tie-offs and port plumbing.
//
// Run (Vivado xsim):
//   xvlog -sv SRC/sources_1/axi4_telemetry.v SRC/sources_1/axi4_mwr_batch.v \
//         SRC/sources_1/axi4_mwr_batch_top.v SRC/sim_11/telemetry_dma_tb.sv
//   xelab work.telemetry_dma_tb work.glbl -L xpm -L unisims_ver --snapshot tel_dma_sim
//   xsim tel_dma_sim -runall
// =================================================================

typedef logic [127:0] rec_t;

module tel_dma_harness #(
    parameter integer DW           = 512,
    parameter integer TUW          = 137,
    parameter integer FIFO_DEPTH   = 16,
    parameter integer REGION_BYTES = 512,
    parameter integer LOG_W        = 128,
    parameter integer READY_PCT    = 100,
    parameter         NAME         = "h"
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DW-1:0]     tdata,
    input  wire [DW/32-1:0]  tkeep,
    input  wire              tvalid,
    input  wire              tlast,
    input  wire [TUW-1:0]    tuser,
    input  wire              log_enable,
    input  wire              stall
);
    localparam integer K = LOG_W / 128;
    localparam integer N = REGION_BYTES / 16;

    wire [LOG_W-1:0]   log_tdata;
    wire [LOG_W/8-1:0] log_tkeep;
    wire               log_tvalid, log_tlast;
    reg                log_tready;
    wire               log_busy, log_done, log_overflow;
    wire [31:0]        log_drop_count;

    axi4_telemetry #(
        .AXIS_DATA_WIDTH  (DW),
        .AXIS_TUSER_WIDTH (TUW),
        .TELEMETRY_DEPTH  (FIFO_DEPTH),
        .ENABLE_ILA       (1),          // must be ignored in DMA mode (no ila_0 in sim)
        .IF_TYPE          ("RQ"),
        .DMA_LOG          (1),
        .DMA_REGION_BYTES (REGION_BYTES),
        .LOG_TDATA_WIDTH  (LOG_W)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .s_axis_tdata (tdata), .s_axis_tkeep (tkeep), .s_axis_tvalid (tvalid),
        .s_axis_tlast (tlast), .s_axis_tuser (tuser), .s_axis_tready (),
        .m_axis_tdata (), .m_axis_tkeep (), .m_axis_tvalid (), .m_axis_tlast (),
        .m_axis_tuser (), .m_axis_tready (1'b1),
        .log_enable (log_enable), .log_busy (log_busy), .log_done (log_done),
        .log_overflow (log_overflow), .log_drop_count (log_drop_count),
        .m_axis_log_tdata (log_tdata), .m_axis_log_tkeep (log_tkeep),
        .m_axis_log_tvalid (log_tvalid), .m_axis_log_tlast (log_tlast),
        .m_axis_log_tready (log_tready)
    );

    // ---------------- DMA S2MM sink model ----------------
    always @(posedge clk or negedge rst_n)
        if (!rst_n) log_tready <= 1'b0;
        else        log_tready <= !stall && ($urandom_range(99) < READY_PCT);

    rec_t recs[$];          // all records received, in order
    int   sess_end[$];      // recs index one past each recording's tlast record
    int   sess_drops[$];    // log_drop_count at each tlast
    int   errors = 0;

    reg             prev_v, prev_r, prev_l;
    reg [LOG_W-1:0] prev_d;

    always @(posedge clk) begin
        if (rst_n) begin
            if (log_tvalid && log_tready) begin
                if (!log_busy) begin
                    $display("ERROR [%s] beat accepted while not busy (t=%0t)", NAME, $time);
                    errors++;
                end
                if (log_tkeep !== {(LOG_W/8){1'b1}}) begin
                    $display("ERROR [%s] tkeep not all ones (t=%0t)", NAME, $time);
                    errors++;
                end
                for (int i = 0; i < K; i++)
                    recs.push_back(log_tdata[128*i +: 128]);
                if (log_tlast) begin
                    sess_end.push_back(recs.size());
                    sess_drops.push_back(log_drop_count);
                end
            end
            if (prev_v && !prev_r &&
                (!log_tvalid || log_tdata !== prev_d || log_tlast !== prev_l)) begin
                $display("ERROR [%s] AXIS beat changed while stalled (t=%0t)", NAME, $time);
                errors++;
            end
        end
        prev_v <= log_tvalid;
        prev_r <= log_tready;
        prev_d <= log_tdata;
        prev_l <= log_tlast;
    end

    // ---------------- Checker ----------------
    function automatic void check_all(ref rec_t sent[$], ref int s_start[$], ref int s_end[$]);
        int base = 0;
        if (sess_end.size() != s_start.size()) begin
            $display("ERROR [%s] %0d recordings logged, expected %0d",
                     NAME, sess_end.size(), s_start.size());
            errors++;
            return;
        end
        for (int s = 0; s < s_start.size(); s++) begin
            int n_sent = s_end[s] - s_start[s];
            int exp_total = (n_sent >= N) ? N : n_sent + (K - (n_sent % K));
            int n_real    = (n_sent >= N) ? N : n_sent;
            int got       = sess_end[s] - base;
            int zeros     = 0;
            if (got != exp_total) begin
                $display("ERROR [%s] rec %0d: %0d records logged, expected %0d",
                         NAME, s, got, exp_total);
                errors++;
            end
            for (int j = 0; j < got; j++) begin
                rec_t r = recs[base + j];
                if (j < n_real) begin
                    if (r == '0)
                        zeros++;
                    else if (r !== sent[s_start[s] + j]) begin
                        if (errors < 20)
                            $display("ERROR [%s] rec %0d slot %0d: got %h exp %h",
                                     NAME, s, j, r, sent[s_start[s] + j]);
                        errors++;
                    end
                end else if (r !== '0) begin
                    $display("ERROR [%s] rec %0d slot %0d: terminator/pad not zero: %h",
                             NAME, s, j, r);
                    errors++;
                end
            end
            if (zeros != sess_drops[s]) begin
                $display("ERROR [%s] rec %0d: %0d zero slots but log_drop_count=%0d",
                         NAME, s, zeros, sess_drops[s]);
                errors++;
            end
            $display("  [%s] recording %0d: sent %0d, logged %0d records (%0d dropped->zero, %0d term/pad)",
                     NAME, s, n_sent, got, zeros, got - n_real);
            base = sess_end[s];
        end
    endfunction
endmodule


module telemetry_dma_tb;
    localparam integer DW  = 512;
    localparam integer TUW = 137;

    localparam [3:0] T_MRD = 4'b0000;
    localparam [3:0] T_MWR = 4'b0001;
    localparam [3:0] T_CAS = 4'b0110;

    reg              clk = 0;
    reg              rst_n = 0;
    reg [DW-1:0]     tdata = '0;
    reg [DW/32-1:0]  tkeep = '1;
    reg              tvalid = 0;
    reg              tlast = 0;
    reg [TUW-1:0]    tuser = '0;
    reg              log_enable = 0;
    reg              stall = 0;

    always #2 clk = ~clk;   // posedges at t = 2, 6, 10, ...

    function automatic longint eidx();   // index of the posedge at $time
        return ($time - 2) / 4;
    endfunction

    // ---------------- DUT harnesses ----------------
    tel_dma_harness #(.FIFO_DEPTH(16),  .REGION_BYTES(32*16),   .LOG_W(128), .READY_PCT(70),  .NAME("h0 w128 N32 F16"))
        h0 (.clk(clk), .rst_n(rst_n), .tdata(tdata), .tkeep(tkeep), .tvalid(tvalid), .tlast(tlast),
            .tuser(tuser), .log_enable(log_enable), .stall(stall));
    tel_dma_harness #(.FIFO_DEPTH(16),  .REGION_BYTES(64*16),   .LOG_W(512), .READY_PCT(60),  .NAME("h1 w512 N64 F16"))
        h1 (.clk(clk), .rst_n(rst_n), .tdata(tdata), .tkeep(tkeep), .tvalid(tvalid), .tlast(tlast),
            .tuser(tuser), .log_enable(log_enable), .stall(stall));
    tel_dma_harness #(.FIFO_DEPTH(512), .REGION_BYTES(4096*16), .LOG_W(128), .READY_PCT(100), .NAME("h2 w128 N4096 F512"))
        h2 (.clk(clk), .rst_n(rst_n), .tdata(tdata), .tkeep(tkeep), .tvalid(tvalid), .tlast(tlast),
            .tuser(tuser), .log_enable(log_enable), .stall(stall));
    tel_dma_harness #(.FIFO_DEPTH(32),  .REGION_BYTES(48*16),   .LOG_W(256), .READY_PCT(80),  .NAME("h3 w256 N48 F32"))
        h3 (.clk(clk), .rst_n(rst_n), .tdata(tdata), .tkeep(tkeep), .tvalid(tvalid), .tlast(tlast),
            .tuser(tuser), .log_enable(log_enable), .stall(stall));

    wire all_idle = !h0.log_busy && !h1.log_busy && !h2.log_busy && !h3.log_busy;
    wire any_busy =  h0.log_busy ||  h1.log_busy ||  h2.log_busy ||  h3.log_busy;

    // ILA-mode instance: DMA outputs must stay tied off
    wire       ila_log_tvalid, ila_log_busy;
    axi4_telemetry #(.AXIS_DATA_WIDTH(DW), .AXIS_TUSER_WIDTH(TUW), .TELEMETRY_DEPTH(64),
                     .ENABLE_ILA(0), .IF_TYPE("RQ"), .DMA_LOG(0)) u_ila_mode (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(tdata), .s_axis_tkeep(tkeep), .s_axis_tvalid(tvalid), .s_axis_tlast(tlast),
        .s_axis_tuser(tuser), .s_axis_tready(),
        .m_axis_tdata(), .m_axis_tkeep(), .m_axis_tvalid(), .m_axis_tlast(), .m_axis_tuser(),
        .m_axis_tready(1'b1),
        .log_enable(log_enable), .log_busy(ila_log_busy), .log_done(), .log_overflow(),
        .log_drop_count(), .m_axis_log_tdata(), .m_axis_log_tkeep(),
        .m_axis_log_tvalid(ila_log_tvalid), .m_axis_log_tlast(), .m_axis_log_tready(1'b1));

    // Top wrapper elaboration check (DMA mode, inputs idle)
    axi4_mwr_batch_top #(.DMA_LOG(1), .ENABLE_ILA(1), .DMA_REGION_BYTES(1024), .LOG_TDATA_WIDTH(256)) u_top (
        .clk(clk), .rst_n(rst_n), .time_threshold(8'd10), .depth_threshold(8'd4), .batch_mrd(1'b0),
        .s_axis_tdata_0('0), .s_axis_tkeep_0('0), .s_axis_tvalid_0(1'b0), .s_axis_tlast_0(1'b0),
        .s_axis_tuser_0('0), .s_axis_tready_0(),
        .s_axis_tdata_1('0), .s_axis_tkeep_1('0), .s_axis_tvalid_1(1'b0), .s_axis_tlast_1(1'b0),
        .s_axis_tuser_1('0), .s_axis_tready_1(),
        .m_axis_tdata(), .m_axis_tkeep(), .m_axis_tvalid(), .m_axis_tlast(), .m_axis_tuser(),
        .m_axis_tready(1'b1),
        .log_enable(1'b0),
        .log_busy_0(), .log_done_0(), .log_overflow_0(), .log_drop_count_0(),
        .m_axis_log_0_tdata(), .m_axis_log_0_tkeep(), .m_axis_log_0_tvalid(), .m_axis_log_0_tlast(),
        .m_axis_log_0_tready(1'b1),
        .log_busy_1(), .log_done_1(), .log_overflow_1(), .log_drop_count_1(),
        .m_axis_log_1_tdata(), .m_axis_log_1_tkeep(), .m_axis_log_1_tvalid(), .m_axis_log_1_tlast(),
        .m_axis_log_1_tready(1'b1));

    int tb_errors = 0;
    always @(posedge clk)
        if (rst_n && (ila_log_tvalid || ila_log_busy)) begin
            $display("ERROR ILA-mode instance drove DMA log outputs (t=%0t)", $time);
            tb_errors++;
        end

    // ---------------- Stimulus + reference model ----------------
    rec_t    sent[$];          // expected record for every TLP sent
    int      s_start[$];       // sent[] index range of each expected recording
    int      s_end[$];
    longint  last_eop_e = -1;
    int      pkt_id = 0;

    function automatic rec_t make_rec(bit [63:0] byte_addr, int len, int gap, bit [3:0] typ,
                                      int dwc, bit [7:0] tag, bit [1:0] at);
        rec_t r = '0;
        int   pdw = (typ == T_MWR) ? ((dwc > 255) ? 255 : dwc) : 0;
        r[63:0]    = {byte_addr[63:2], 2'b00};
        r[71:64]   = (len > 255) ? 255 : len;
        r[79:72]   = (gap > 255) ? 255 : gap;
        r[87:80]   = {4'd0, typ};
        r[95:88]   = pdw;
        r[103:96]  = tag;
        r[111:104] = {6'd0, at};
        return r;
    endfunction

    task automatic idle(int n);
        repeat (n) begin
            @(posedge clk);
            tvalid <= 1'b0;
            tlast  <= 1'b0;
        end
    endtask

    // Send one TLP of 'beats' beats after 'gap' idle cycles.
    task automatic send_pkt(int beats, int gap);
        bit [3:0]  typ;
        bit [63:0] addr;
        bit [7:0]  tag;
        bit [1:0]  at;
        int        dwc;
        longint    sop_e;
        int        exp_gap;
        case (pkt_id % 4)
            0, 3: typ = T_MWR;
            1:    typ = T_MRD;
            default: typ = T_CAS;
        endcase
        addr = {16'hA5A5, 16'(pkt_id), 32'h0} | (64'(pkt_id) << 6);
        tag  = pkt_id[7:0];
        at   = pkt_id[1:0];
        dwc  = $urandom_range(1, 300);

        idle(gap);
        for (int b = 0; b < beats; b++) begin
            @(posedge clk);
            if (b == 0) begin
                reg [DW-1:0] d = {$urandom, $urandom, $urandom, $urandom};
                sop_e = eidx() + 1;              // beat is sampled on the next posedge
                d[1:0]   = at;
                d[63:2]  = addr[63:2];
                d[74:64] = dwc[10:0];
                d[78:75] = typ;
                d[103:96]= tag;
                tdata <= d;
            end else begin
                tdata <= {16{$urandom}};
            end
            tvalid <= 1'b1;
            tlast  <= (b == beats - 1);
        end
        exp_gap    = (last_eop_e < 0) ? 0 : int'(sop_e - last_eop_e - 1);
        last_eop_e = sop_e + beats - 1;
        sent.push_back(make_rec(addr, beats, exp_gap, typ, dwc, tag, at));
        pkt_id++;
    endtask

    task automatic send_many(int n, int max_beats, int max_gap);
        for (int i = 0; i < n; i++)
            send_pkt($urandom_range(1, max_beats), $urandom_range(0, max_gap));
    endtask

    task automatic arm();                  // start an expected recording
        @(posedge clk); log_enable <= 1'b1;
        idle(10);
        s_start.push_back(sent.size());
    endtask

    task automatic disarm();               // end it (bus must be idle)
        idle(10);
        s_end.push_back(sent.size());
        @(posedge clk); log_enable <= 1'b0;
    endtask

    task automatic wait_idle();
        fork
            begin
                idle(20);
                wait (all_idle);
                idle(5);
            end
            begin
                repeat (200000) @(posedge clk);
                $display("ERROR timeout waiting for recordings to drain");
                tb_errors++;
            end
        join_any
        disable fork;
    endtask

    int d_sess;   // index of the overflow recording

    initial begin
        idle(10);
        rst_n <= 1'b1;
        idle(40);   // let the XPM FIFOs leave reset

        // Traffic before arming must not be logged
        send_many(20, 3, 3);
        idle(10);
        sent.delete();               // not part of any recording

        // A: short recording, stop by enable-low
        $display("Scenario A: 10 packets, stop by enable low");
        arm();
        send_many(10, 4, 5);
        disarm();
        wait_idle();

        // B: 100 packets - fills small regions, fits the large one
        $display("Scenario B: 100 packets, mixed lengths/gaps");
        arm();
        send_many(100, 4, 3);
        disarm();
        wait_idle();

        // C: enable held high after region full: no second recording
        $display("Scenario C: 250 packets with enable held high");
        arm();
        send_many(200, 2, 2);
        idle(50);
        send_many(50, 2, 2);
        disarm();
        wait_idle();

        // D: DMA stalled, back-to-back single-beat packets -> overflow
        $display("Scenario D: DMA stalled, 80 back-to-back single-beat packets");
        @(posedge clk); stall <= 1'b1;
        arm();
        d_sess = s_start.size() - 1;
        send_many(80, 1, 0);
        disarm();
        idle(30);
        @(posedge clk); stall <= 1'b0;
        wait_idle();

        // E: enable pulse with no traffic -> terminator only
        $display("Scenario E: enable pulse, no traffic");
        arm();
        disarm();
        wait_idle();

        // F: rising edge while draining is ignored; re-arm needs a new edge
        $display("Scenario F: edge during drain ignored, then re-arm");
        @(posedge clk); stall <= 1'b1;
        arm();
        send_many(5, 3, 2);
        disarm();
        idle(20);                        // DUTs now in DRAIN (DMA stalled)
        @(posedge clk); log_enable <= 1'b1;
        idle(20);
        @(posedge clk); stall <= 1'b0;
        wait_idle();
        send_many(10, 3, 2);             // enable high but not armed: not logged
        idle(20);
        if (any_busy) begin
            $display("ERROR re-armed without a rising edge");
            tb_errors++;
        end
        @(posedge clk); log_enable <= 1'b0;
        idle(10);
        arm();
        send_many(7, 3, 2);
        disarm();
        wait_idle();

        // ---------------- Checks ----------------
        $display("---------------- results ----------------");
        h0.check_all(sent, s_start, s_end);
        h1.check_all(sent, s_start, s_end);
        h2.check_all(sent, s_start, s_end);
        h3.check_all(sent, s_start, s_end);

        if (h2.recs.size() != 0 && h2.log_overflow) begin
            $display("ERROR h2 (deep FIFO, always-ready DMA) reported overflow");
            tb_errors++;
        end
        foreach (h2.sess_drops[i])
            if (h2.sess_drops[i] != 0) begin
                $display("ERROR h2 dropped records in recording %0d", i);
                tb_errors++;
            end
        if (h0.sess_drops.size() > d_sess && h0.sess_drops[d_sess] == 0) begin
            $display("ERROR h0 did not overflow in scenario D (coverage)");
            tb_errors++;
        end

        tb_errors += h0.errors + h1.errors + h2.errors + h3.errors;
        if (tb_errors == 0) $display("*** TEST PASSED ***");
        else                $display("*** TEST FAILED: %0d errors ***", tb_errors);
        $finish;
    end
endmodule
