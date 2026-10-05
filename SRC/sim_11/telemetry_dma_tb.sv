`timescale 1ns/1ps
// =================================================================
// telemetry_dma_tb
//
// Self-checking testbench for the merged DMA telemetry log of
// axi4_mwr_batch_top (DMA_LOG = 1): three taps (in0, in1, batched out)
// merged by axi4_telemetry_logger into one AXI-S stream for the AXI DMA.
//
// Several top instances with different log stream widths / region sizes /
// FIFO depths / DMA sink behaviour receive identical stimulus.  Each harness
//   - monitors in0, in1 and out on the wire and rebuilds the expected record
//     of every TLP independently of the RTL (length, gap, type, addr, ...),
//   - models the AXI DMA S2MM sink and splits the log into recordings at tlast,
//   - checks every recording:
//       * per port, the n-th logged record is that port's n-th TLP, either
//         exact (kind TLP) or as a dropped record (kind DROP, zero data)
//       * TLP records of all ports appear in EOP time order (port order
//         within a cycle)
//       * region full -> exactly N-1 TLP/drop records; otherwise all TLPs
//       * pad records up to the last slot of a beat, then the trailer
//       * trailer: n_records, n_dropped, duration, stop reason
//       * log_stop_reason / log_drop_count pins agree with the trailer
//       * no beats outside a recording, tkeep all ones, AXIS hold rules
//       * re-arm only on a rising edge of log_enable seen while idle
//
// Also elaborates the top with DMA_LOG = 0 and checks the log tie-offs.
//
// Run (Vivado xsim):
//   xvlog -sv SRC/sources_1/axi4_telemetry.v SRC/sources_1/axi4_telemetry_logger.v \
//         SRC/sources_1/axi4_mwr_batch.v SRC/sources_1/axi4_mwr_batch_top.v \
//         SRC/sim_11/telemetry_dma_tb.sv <vivado>/data/verilog/src/glbl.v
//   xelab work.telemetry_dma_tb work.glbl -L xpm -L unisims_ver --snapshot tel_dma_sim
//   xsim tel_dma_sim -runall
// =================================================================

typedef logic [127:0] rec_t;

module tel_top_harness #(
    parameter integer DW           = 512,
    parameter integer TUW          = 137,
    parameter integer LOG_FIFO     = 16,
    parameter integer REGION_BYTES = 1024,
    parameter integer LOG_W        = 128,
    parameter integer READY_PCT    = 100,
    parameter         NAME         = "h"
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [DW-1:0]     tdata_0,
    input  wire              tvalid_0,
    input  wire              tlast_0,
    input  wire [TUW-1:0]    tuser_0,
    output wire              tready_0,
    input  wire [DW-1:0]     tdata_1,
    input  wire              tvalid_1,
    input  wire              tlast_1,
    input  wire [TUW-1:0]    tuser_1,
    output wire              tready_1,
    input  wire              out_tready,
    input  wire              log_enable,
    input  wire              stall
);
    localparam integer K   = LOG_W / 128;
    localparam integer N   = REGION_BYTES / 16;
    localparam integer CAP = N - 1;

    wire [DW-1:0]      out_tdata;
    wire               out_tvalid, out_tlast;

    wire [LOG_W-1:0]   log_tdata;
    wire [LOG_W/8-1:0] log_tkeep;
    wire               log_tvalid, log_tlast;
    reg                log_tready;
    wire               log_busy, log_done, log_overflow;
    wire [1:0]         log_stop_reason;
    wire [31:0]        log_drop_count;

    axi4_mwr_batch_top #(
        .AXIS_DATA_WIDTH  (DW),
        .AXIS_TUSER_WIDTH (TUW),
        .ENABLE_ILA       (1),          // ignored in DMA mode (no ila_0 in sim)
        .DMA_LOG          (1),
        .DMA_REGION_BYTES (REGION_BYTES),
        .LOG_TDATA_WIDTH  (LOG_W),
        .LOG_FIFO_DEPTH   (LOG_FIFO)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .time_threshold (8'd6), .depth_threshold (8'd4), .batch_mrd (1'b0),
        .s_axis_tdata_0 (tdata_0), .s_axis_tkeep_0 ({(DW/32){1'b1}}), .s_axis_tvalid_0 (tvalid_0),
        .s_axis_tlast_0 (tlast_0), .s_axis_tuser_0 (tuser_0), .s_axis_tready_0 (tready_0),
        .s_axis_tdata_1 (tdata_1), .s_axis_tkeep_1 ({(DW/32){1'b1}}), .s_axis_tvalid_1 (tvalid_1),
        .s_axis_tlast_1 (tlast_1), .s_axis_tuser_1 (tuser_1), .s_axis_tready_1 (tready_1),
        .m_axis_tdata (out_tdata), .m_axis_tkeep (), .m_axis_tvalid (out_tvalid),
        .m_axis_tlast (out_tlast), .m_axis_tuser (), .m_axis_tready (out_tready),
        .log_enable (log_enable), .log_busy (log_busy), .log_done (log_done),
        .log_overflow (log_overflow), .log_stop_reason (log_stop_reason),
        .log_drop_count (log_drop_count),
        .m_axis_log_tdata (log_tdata), .m_axis_log_tkeep (log_tkeep),
        .m_axis_log_tvalid (log_tvalid), .m_axis_log_tlast (log_tlast),
        .m_axis_log_tready (log_tready)
    );

    int errors = 0;

    // ---------------- Bus monitors: expected records ----------------
    longint cyc = 0;
    longint       exp_cyc[$];   // per expected TLP: EOP cycle
    int           exp_port[$];  // 0 = in0, 1 = in1, 2 = out
    logic [111:0] exp_data[$];  // expected record bytes 0..13
    // (parallel queues: xsim 2021.2 crashes reading fields of a struct queue)

    bit           in_pkt   [3];
    bit           have_eop [3];
    longint       last_eop [3];
    int           beats    [3];
    int           gap      [3];
    logic [DW-1:0] sop_d   [3];

    function automatic logic [111:0] make_rec(logic [DW-1:0] d, int len, int g);
        logic [111:0] r = '0;
        int dwc = d[74:64];
        r[63:0]    = {d[63:2], 2'b00};
        r[71:64]   = (len > 255) ? 255 : len;
        r[79:72]   = (g > 255) ? 255 : g;
        r[87:80]   = {4'd0, d[78:75]};
        r[95:88]   = (d[78:75] == 4'b0001) ? ((dwc > 255) ? 255 : dwc) : 0;
        r[103:96]  = d[103:96];
        r[111:104] = {6'd0, d[1:0]};
        return r;
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            cyc = 0;
            for (int p = 0; p < 3; p++) begin
                in_pkt[p] = 0; have_eop[p] = 0;
            end
        end else begin
            logic          v [3], l [3];
            logic [DW-1:0] d [3];
            cyc = cyc + 1;
            v[0] = tvalid_0 && tready_0;     l[0] = tlast_0;   d[0] = tdata_0;
            v[1] = tvalid_1 && tready_1;     l[1] = tlast_1;   d[1] = tdata_1;
            v[2] = out_tvalid && out_tready; l[2] = out_tlast; d[2] = out_tdata;
            for (int p = 0; p < 3; p++) begin
                if (v[p]) begin
                    if (!in_pkt[p]) begin
                        in_pkt[p] = 1;
                        beats[p]  = 0;
                        sop_d[p]  = d[p];
                        gap[p]    = have_eop[p] ? int'(cyc - last_eop[p] - 1) : 0;
                    end
                    beats[p]++;
                    if (l[p]) begin
                        exp_cyc.push_back(cyc);
                        exp_port.push_back(p);
                        exp_data.push_back(make_rec(sop_d[p], beats[p], gap[p]));
                        in_pkt[p]   = 0;
                        have_eop[p] = 1;
                        last_eop[p] = cyc;
                    end
                end
            end
        end
    end

    // ---------------- DMA S2MM sink model ----------------
    always @(posedge clk or negedge rst_n)
        if (!rst_n) log_tready <= 1'b0;
        else        log_tready <= !stall && ($urandom_range(99) < READY_PCT);

    rec_t recs[$];          // all records received, in order
    int   sess_end[$];      // recs index one past each recording's trailer
    int   sess_drops[$];    // log_drop_count at each tlast
    int   sess_reason[$];   // log_stop_reason at each tlast

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
                    sess_reason.push_back(log_stop_reason);
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
    function automatic void err(string msg);
        if (errors < 25) $display("ERROR [%s] %s", NAME, msg);
        errors++;
    endfunction

    // ws/we: EOP-cycle window of each expected recording; dur: enable high cycles
    function automatic void check_all(ref longint ws[$], ref longint we[$], ref longint dur[$]);
        int base = 0;
        if (sess_end.size() != ws.size()) begin
            err($sformatf("%0d recordings logged, expected %0d", sess_end.size(), ws.size()));
            return;
        end
        for (int s = 0; s < ws.size(); s++) begin
            int   E[$];                       // indices into exp_* of this recording
            int   n_e, n_rec, pad, exp_total, got, exp_reason;
            int   pidx[3], pcnt[3], pmap[];   // pmap[p*n_rec + k] = E index of port p's k-th TLP
            int   last_tlp, drops, per_port_tlp[3], per_port_drop[3];
            rec_t tr;

            // xsim does not re-run initialisers of loop-local variables: reset explicitly
            E.delete();
            last_tlp = -1;
            drops    = 0;
            foreach (exp_cyc[i])
                if (exp_cyc[i] >= ws[s] && exp_cyc[i] <= we[s])
                    E.push_back(i);
            n_e        = E.size();
            n_rec      = (n_e >= CAP) ? CAP : n_e;
            exp_reason = (n_e >= CAP) ? 2 : 1;
            pad        = (K - 1) - (n_rec % K);
            exp_total  = n_rec + pad + 1;
            got        = sess_end[s] - base;
            pmap       = new[3*n_rec + 1];
            for (int p = 0; p < 3; p++) begin
                pidx[p] = 0; pcnt[p] = 0; per_port_tlp[p] = 0; per_port_drop[p] = 0;
            end
            for (int i = 0; i < n_rec; i++) begin
                pmap[exp_port[E[i]]*n_rec + pcnt[exp_port[E[i]]]] = i;
                pcnt[exp_port[E[i]]]++;
            end

            if (got != exp_total) begin
                err($sformatf("rec %0d: %0d records logged, expected %0d", s, got, exp_total));
                base = sess_end[s];
                continue;
            end

            // TLP / dropped records
            for (int j = 0; j < n_rec; j++) begin
                rec_t r;
                int   info, kind, port, ei;
                logic [111:0] ed;
                r    = recs[base + j];
                info = r[119:112];
                kind = r[115:114];
                port = r[113:112];
                if (r[119] !== 1'b1 || r[118:116] !== 3'd0 || r[127:120] !== 8'd0)
                    err($sformatf("rec %0d slot %0d: bad info/reserved bytes %h", s, j, r));
                if (port > 2 || kind > 1) begin
                    err($sformatf("rec %0d slot %0d: kind %0d port %0d not a TLP/drop", s, j, kind, port));
                end else if (pidx[port] >= pcnt[port]) begin
                    err($sformatf("rec %0d slot %0d: extra record for port %0d", s, j, port));
                end else begin
                    ei = pmap[port*n_rec + pidx[port]];
                    pidx[port]++;
                    if (kind == 0) begin
                        per_port_tlp[port]++;
                        ed = exp_data[E[ei]];
                        if (r[111:0] !== ed)
                            err($sformatf("rec %0d slot %0d port %0d: got %h exp %h",
                                          s, j, port, r[111:0], ed));
                        if (ei < last_tlp)
                            err($sformatf("rec %0d slot %0d: TLP out of EOP order (%0d after %0d)",
                                          s, j, ei, last_tlp));
                        last_tlp = ei;
                    end else begin
                        per_port_drop[port]++;
                        drops++;
                        if (r[111:0] !== 112'd0)
                            err($sformatf("rec %0d slot %0d: dropped record data not zero", s, j));
                    end
                end
            end
            for (int p = 0; p < 3; p++)
                if (pidx[p] != pcnt[p])
                    err($sformatf("rec %0d: port %0d logged %0d records, expected %0d",
                                  s, p, pidx[p], pcnt[p]));

            // pad records
            for (int j = n_rec; j < n_rec + pad; j++)
                if (recs[base + j] !== {8'h00, 8'h8C, 112'd0})
                    err($sformatf("rec %0d slot %0d: bad pad record %h", s, j, recs[base + j]));

            // trailer
            tr = recs[base + got - 1];
            if (tr[127:112] !== 16'h0088)
                err($sformatf("rec %0d: last record is not a trailer: %h", s, tr));
            if (tr[31:0] != n_rec)
                err($sformatf("rec %0d: trailer n_records %0d, expected %0d", s, tr[31:0], n_rec));
            if (tr[63:32] != drops)
                err($sformatf("rec %0d: trailer n_dropped %0d, counted %0d", s, tr[63:32], drops));
            if (tr[103:96] != exp_reason)
                err($sformatf("rec %0d: trailer stop_reason %0d, expected %0d", s, tr[103:96], exp_reason));
            if (tr[111:104] !== 0)
                err($sformatf("rec %0d: trailer reserved byte not zero", s));
            if (exp_reason == 1 ? (tr[95:64] < dur[s] - 2 || tr[95:64] > dur[s] + 2)
                                : (tr[95:64] == 0 || tr[95:64] >= dur[s]))
                err($sformatf("rec %0d: trailer duration %0d, enable high %0d cycles", s, tr[95:64], dur[s]));
            if (sess_drops[s] != drops)
                err($sformatf("rec %0d: log_drop_count %0d, counted %0d", s, sess_drops[s], drops));
            if (sess_reason[s] != exp_reason)
                err($sformatf("rec %0d: log_stop_reason %0d, expected %0d", s, sess_reason[s], exp_reason));

            $display("  [%s] rec %0d: %0d TLPs (in0/in1/out %0d/%0d/%0d), logged %0d, dropped %0d (%0d/%0d/%0d), pad %0d, reason %s, dur %0d",
                     NAME, s, n_e, pcnt[0], pcnt[1], pcnt[2], got,
                     drops, per_port_drop[0], per_port_drop[1], per_port_drop[2], pad,
                     (exp_reason == 1) ? "enable" : "full", tr[95:64]);
            base = sess_end[s];
        end
    endfunction
endmodule


module telemetry_dma_tb;
    localparam integer DW  = 512;
    localparam integer TUW = 137;

    localparam [3:0] T_MRD = 4'b0000;
    localparam [3:0] T_MWR = 4'b0001;

    reg              clk = 0;
    reg              rst_n = 0;
    reg [DW-1:0]     tdata_0 = '0, tdata_1 = '0;
    reg              tvalid_0 = 0, tvalid_1 = 0;
    reg              tlast_0 = 0,  tlast_1 = 0;
    reg [TUW-1:0]    tuser_0 = '0, tuser_1 = '0;
    reg              out_tready = 0;
    reg              log_enable = 0;
    reg              stall = 0;

    always #2 clk = ~clk;

    always @(posedge clk) out_tready <= ($urandom_range(99) < 85);

    // ---------------- DUT harnesses ----------------
    `define HARNESS(inst, FIFO, REGION, W, RDY, NM) \
        tel_top_harness #(.LOG_FIFO(FIFO), .REGION_BYTES(REGION), .LOG_W(W), .READY_PCT(RDY), .NAME(NM)) inst ( \
            .clk(clk), .rst_n(rst_n), \
            .tdata_0(tdata_0), .tvalid_0(tvalid_0), .tlast_0(tlast_0), .tuser_0(tuser_0), .tready_0(), \
            .tdata_1(tdata_1), .tvalid_1(tvalid_1), .tlast_1(tlast_1), .tuser_1(tuser_1), .tready_1(), \
            .out_tready(out_tready), .log_enable(log_enable), .stall(stall));

    `HARNESS(h0, 16,  64*16,   128, 70,  "h0 w128 N64 F16")
    `HARNESS(h1, 16,  128*16,  512, 60,  "h1 w512 N128 F16")
    `HARNESS(h2, 512, 8192*16, 128, 100, "h2 w128 N8192 F512")
    `HARNESS(h3, 32,  96*16,   256, 80,  "h3 w256 N96 F32")

    wire tready_0 = h0.tready_0;
    wire tready_1 = h0.tready_1;
    wire all_idle = !h0.log_busy && !h1.log_busy && !h2.log_busy && !h3.log_busy;
    wire any_busy =  h0.log_busy ||  h1.log_busy ||  h2.log_busy ||  h3.log_busy;
    wire bus_busy = tvalid_0 || tvalid_1 || h0.out_tvalid;

    int tb_errors = 0;

    // All tops see identical stimulus, so their input backpressure must match
    always @(posedge clk)
        if (rst_n && ({h1.tready_0, h2.tready_0, h3.tready_0} !== {3{h0.tready_0}} ||
                      {h1.tready_1, h2.tready_1, h3.tready_1} !== {3{h0.tready_1}})) begin
            $display("ERROR harness input tready diverged (t=%0t)", $time);
            tb_errors++;
        end

    // DMA_LOG = 0 build: log ports must be tied off
    wire        ila_log_tvalid, ila_log_busy;
    wire [1:0]  ila_log_reason;
    axi4_mwr_batch_top #(.ENABLE_ILA(0), .DMA_LOG(0)) u_top_ila (
        .clk(clk), .rst_n(rst_n), .time_threshold(8'd6), .depth_threshold(8'd4), .batch_mrd(1'b0),
        .s_axis_tdata_0(tdata_0), .s_axis_tkeep_0('1), .s_axis_tvalid_0(tvalid_0), .s_axis_tlast_0(tlast_0),
        .s_axis_tuser_0(tuser_0), .s_axis_tready_0(),
        .s_axis_tdata_1(tdata_1), .s_axis_tkeep_1('1), .s_axis_tvalid_1(tvalid_1), .s_axis_tlast_1(tlast_1),
        .s_axis_tuser_1(tuser_1), .s_axis_tready_1(),
        .m_axis_tdata(), .m_axis_tkeep(), .m_axis_tvalid(), .m_axis_tlast(), .m_axis_tuser(),
        .m_axis_tready(out_tready),
        .log_enable(log_enable), .log_busy(ila_log_busy), .log_done(), .log_overflow(),
        .log_stop_reason(ila_log_reason), .log_drop_count(),
        .m_axis_log_tdata(), .m_axis_log_tkeep(), .m_axis_log_tvalid(ila_log_tvalid),
        .m_axis_log_tlast(), .m_axis_log_tready(1'b1));

    always @(posedge clk)
        if (rst_n && (ila_log_tvalid || ila_log_busy || ila_log_reason != 0)) begin
            $display("ERROR DMA_LOG=0 top drove log outputs (t=%0t)", $time);
            tb_errors++;
        end

    // ---------------- Stimulus ----------------
    int pkt_id = 0;

    // One TLP on input port 'port'.  MRd is always single-beat.
    task automatic send_pkt(int port, int beats, bit force_mrd);
        logic [DW-1:0]  d;
        logic [TUW-1:0] u;
        bit [3:0]       typ;
        int             r;
        int             id;
        r   = $urandom_range(99);
        id  = pkt_id;
        pkt_id++;
        typ = force_mrd ? T_MRD : (r < 60) ? T_MWR : (r < 85) ? T_MRD : 4'($urandom_range(2, 7));
        if (typ == T_MRD) beats = 1;
        for (int b = 0; b < beats; b++) begin
            d = {16{$urandom}};
            u = '0;
            if (b == 0) begin
                d[1:0]    = $urandom_range(0, 3);
                d[63:2]   = {port[3:0], 26'h0, id[31:0]} << 4;
                d[74:64]  = $urandom_range(1, 300);
                d[78:75]  = typ;
                d[103:96] = id[7:0];
                u[21:20]  = 2'b01;                 // RQ is_sop
            end
            if (b == beats - 1) u[27:26] = 2'b01;  // RQ is_eop
            if (port == 0) begin
                tdata_0 <= d; tuser_0 <= u; tvalid_0 <= 1'b1; tlast_0 <= (b == beats - 1);
                do @(posedge clk); while (!tready_0);
            end else begin
                tdata_1 <= d; tuser_1 <= u; tvalid_1 <= 1'b1; tlast_1 <= (b == beats - 1);
                do @(posedge clk); while (!tready_1);
            end
        end
        if (port == 0) begin tvalid_0 <= 1'b0; tlast_0 <= 1'b0; end
        else           begin tvalid_1 <= 1'b0; tlast_1 <= 1'b0; end
    endtask

    task automatic drive_port(int port, int n, int max_beats, int max_gap, bit mrd_only);
        for (int i = 0; i < n; i++) begin
            int g = $urandom_range(0, max_gap);
            repeat (g) @(posedge clk);
            send_pkt(port, $urandom_range(1, max_beats), mrd_only);
        end
    endtask

    task automatic traffic(int n, int max_beats, int max_gap, bit mrd_only = 0);
        fork
            drive_port(0, n, max_beats, max_gap, mrd_only);
            drive_port(1, n, max_beats, max_gap, mrd_only);
        join
    endtask

    // Wait until no beat has been offered on in0/in1/out for n cycles
    task automatic wait_quiet(int n = 300);
        int q = 0;
        while (q < n) begin
            @(posedge clk);
            q = bus_busy ? 0 : q + 1;
        end
    endtask

    longint ws[$], we[$], dur[$];   // expected recordings: EOP window, enable-high cycles
    longint t_arm;

    task automatic arm();
        wait_quiet();
        @(posedge clk); log_enable <= 1'b1;
        t_arm = h0.cyc;
        repeat (10) @(posedge clk);
        ws.push_back(h0.cyc);
    endtask

    task automatic disarm();
        wait_quiet();
        we.push_back(h0.cyc);
        @(posedge clk); log_enable <= 1'b0;
        dur.push_back(h0.cyc - t_arm);
    endtask

    task automatic wait_idle();
        fork
            begin
                repeat (20) @(posedge clk);
                wait (all_idle);
                repeat (5) @(posedge clk);
            end
            begin
                repeat (500000) @(posedge clk);
                $display("ERROR timeout waiting for recordings to drain");
                tb_errors++;
            end
        join_any
        disable fork;
    endtask

    int d_sess;

    initial begin
        repeat (10) @(posedge clk);
        rst_n <= 1'b1;
        repeat (40) @(posedge clk);

        // Traffic before arming must not be logged
        traffic(15, 4, 3);

        $display("Scenario A: 20 TLPs per input, stop by enable low");
        arm();
        traffic(20, 4, 4);
        disarm();
        wait_idle();

        $display("Scenario B: 150 TLPs per input - fills small regions");
        arm();
        traffic(150, 4, 2);
        disarm();
        wait_idle();

        $display("Scenario C: enable held high after region full");
        arm();
        traffic(100, 2, 1);
        wait_quiet();
        traffic(40, 2, 1);
        disarm();
        wait_idle();

        $display("Scenario D: DMA stalled, back-to-back single-beat MRd on both inputs");
        @(posedge clk); stall <= 1'b1;
        arm();
        d_sess = ws.size() - 1;
        traffic(100, 1, 0, 1);
        disarm();
        repeat (30) @(posedge clk);
        @(posedge clk); stall <= 1'b0;
        wait_idle();

        $display("Scenario E: enable pulse, no traffic");
        arm();
        disarm();
        wait_idle();

        $display("Scenario F: edge during drain ignored, then re-arm");
        @(posedge clk); stall <= 1'b1;
        arm();
        traffic(5, 3, 2);
        disarm();
        repeat (20) @(posedge clk);          // loggers now in DRAIN (DMA stalled)
        @(posedge clk); log_enable <= 1'b1;
        repeat (20) @(posedge clk);
        @(posedge clk); stall <= 1'b0;
        wait_idle();
        traffic(10, 3, 2);                   // enable high but not armed: not logged
        wait_quiet();
        if (any_busy) begin
            $display("ERROR re-armed without a rising edge");
            tb_errors++;
        end
        @(posedge clk); log_enable <= 1'b0;
        repeat (10) @(posedge clk);
        arm();
        traffic(7, 3, 2);
        disarm();
        wait_idle();

        // ---------------- Checks ----------------
        $display("---------------- results ----------------");
        h0.check_all(ws, we, dur);
        h1.check_all(ws, we, dur);
        h2.check_all(ws, we, dur);
        h3.check_all(ws, we, dur);

        foreach (h2.sess_drops[i])
            if (h2.sess_drops[i] != 0) begin
                $display("ERROR h2 (deep FIFO, always-ready DMA) dropped records in recording %0d", i);
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
