`timescale 1ns / 1ps

module axi4_switch_wrr_asym_tb();

  // Parameters
  localparam TDATA_L = 512;
  localparam TUSER_L = 81;
  localparam TKEEP_L = 16;
  localparam MAX_BEATS = 256;

  // Clock and reset
  reg clk   = 0;
  reg rst_n = 0;
  always #5 clk = ~clk; // 100 MHz clock

  // Adjustable weight registers (can be changed anytime during simulation)
  reg [3:0] s0_weight = 4'd4;
  reg [3:0] s1_weight = 4'd2;

  // AXI4-Stream signals
  reg  [TDATA_L-1:0] axi_s0_tdata_i  = 'x;
  reg  [TUSER_L-1:0] axi_s0_tuser_i  = 'x;
  reg                axi_s0_tlast_i   = 'x;
  reg  [TKEEP_L-1:0] axi_s0_tkeep_i  = 'x;
  reg                axi_s0_tvalid_i  = 0;
  wire               axi_s0_tready_o;

  reg  [TDATA_L-1:0] axi_s1_tdata_i  = 'x;
  reg  [TUSER_L-1:0] axi_s1_tuser_i  = 'x;
  reg                axi_s1_tlast_i   = 'x;
  reg  [TKEEP_L-1:0] axi_s1_tkeep_i  = 'x;
  reg                axi_s1_tvalid_i  = 0;
  wire               axi_s1_tready_o;

  wire [TDATA_L-1:0] axi_m0_tdata_o;
  wire [TUSER_L-1:0] axi_m0_tuser_o;
  wire               axi_m0_tlast_o;
  wire [TKEEP_L-1:0] axi_m0_tkeep_o;
  wire               axi_m0_tvalid_o;
  reg                axi_m0_tready_i = 1;

  reg  [1:0] s_req_supress = 2'b00;

  // DUT instance
  axi4_switch_wrr #(
    .TDATA_L (TDATA_L),
    .TUSER_L (TUSER_L),
    .TKEEP_L (TKEEP_L),
    .WEIGHT_L(4)
  ) dut (
    .clk            (clk),
    .rst_n          (rst_n),
    .s0_weight      (s0_weight),
    .s1_weight      (s1_weight),
    .s_req_supress  (s_req_supress),
    .axi_s0_tdata_i (axi_s0_tdata_i),
    .axi_s0_tuser_i (axi_s0_tuser_i),
    .axi_s0_tlast_i (axi_s0_tlast_i),
    .axi_s0_tkeep_i (axi_s0_tkeep_i),
    .axi_s0_tvalid_i(axi_s0_tvalid_i),
    .axi_s0_tready_o(axi_s0_tready_o),
    .axi_s1_tdata_i (axi_s1_tdata_i),
    .axi_s1_tuser_i (axi_s1_tuser_i),
    .axi_s1_tlast_i (axi_s1_tlast_i),
    .axi_s1_tkeep_i (axi_s1_tkeep_i),
    .axi_s1_tvalid_i(axi_s1_tvalid_i),
    .axi_s1_tready_o(axi_s1_tready_o),
    .axi_m0_tdata_o (axi_m0_tdata_o),
    .axi_m0_tuser_o (axi_m0_tuser_o),
    .axi_m0_tlast_o (axi_m0_tlast_o),
    .axi_m0_tkeep_o (axi_m0_tkeep_o),
    .axi_m0_tvalid_o(axi_m0_tvalid_o),
    .axi_m0_tready_i(axi_m0_tready_i)
  );

  // -----------------------------------------------------------------------
  // Expected beats per port (recorded at send time, in-order within port)
  // -----------------------------------------------------------------------
  reg [TDATA_L-1:0] exp_data_0 [0:MAX_BEATS-1];
  reg [TUSER_L-1:0] exp_user_0 [0:MAX_BEATS-1];
  reg               exp_last_0 [0:MAX_BEATS-1];
  integer           exp_cnt_0 = 0;

  reg [TDATA_L-1:0] exp_data_1 [0:MAX_BEATS-1];
  reg [TUSER_L-1:0] exp_user_1 [0:MAX_BEATS-1];
  reg               exp_last_1 [0:MAX_BEATS-1];
  integer           exp_cnt_1 = 0;

  // -----------------------------------------------------------------------
  // Received beats per port (captured at output, in-order within port)
  // -----------------------------------------------------------------------
  reg [TDATA_L-1:0] rcv_data_0 [0:MAX_BEATS-1];
  reg [TUSER_L-1:0] rcv_user_0 [0:MAX_BEATS-1];
  reg               rcv_last_0 [0:MAX_BEATS-1];
  integer           rcv_cnt_0 = 0;

  reg [TDATA_L-1:0] rcv_data_1 [0:MAX_BEATS-1];
  reg [TUSER_L-1:0] rcv_user_1 [0:MAX_BEATS-1];
  reg               rcv_last_1 [0:MAX_BEATS-1];
  integer           rcv_cnt_1 = 0;

  integer port0_counter = 0;
  integer port1_counter = 0;

  // Capture output beats without inline ordering checks.
  // Port is identified by bit 28 of tdata:
  //   port 0 uses 0xA0A0XXXX prefix -> bit 28 = 0
  //   port 1 uses 0xB0B0XXXX prefix -> bit 28 = 1
  always @(posedge clk) begin
    if (axi_m0_tvalid_o && axi_m0_tready_i) begin
      if (axi_m0_tdata_o[28] == 1'b0) begin
        rcv_data_0[rcv_cnt_0] = axi_m0_tdata_o;
        rcv_user_0[rcv_cnt_0] = axi_m0_tuser_o;
        rcv_last_0[rcv_cnt_0] = axi_m0_tlast_o;
        rcv_cnt_0 = rcv_cnt_0 + 1;
      end else begin
        rcv_data_1[rcv_cnt_1] = axi_m0_tdata_o;
        rcv_user_1[rcv_cnt_1] = axi_m0_tuser_o;
        rcv_last_1[rcv_cnt_1] = axi_m0_tlast_o;
        rcv_cnt_1 = rcv_cnt_1 + 1;
      end
    end
  end

  // send_beat: drives one AXI beat on the chosen port, respects tready.
  // Records the beat into the expected array before driving.
  task automatic send_beat(input integer port, input bit last);
    reg [TDATA_L-1:0] data;
    reg [TUSER_L-1:0] user;
    begin
      if (port == 0) begin
        data = {480'h0, 32'hA0A00000 + port0_counter};
        user = {480'h0, 32'hA0A00000 + port0_counter};
        exp_data_0[exp_cnt_0] = data;
        exp_user_0[exp_cnt_0] = user;
        exp_last_0[exp_cnt_0] = last;
        exp_cnt_0++;
        port0_counter++;
        axi_s0_tdata_i  = data;
        axi_s0_tuser_i  = user;
        axi_s0_tkeep_i  = {TKEEP_L{1'b1}};
        axi_s0_tlast_i  = last;
        axi_s0_tvalid_i = 1;
        @(posedge clk);
        if (!axi_s0_tready_o) begin
          wait (axi_s0_tready_o);
          @(posedge clk);
        end
        axi_s0_tvalid_i = 0;
        axi_s0_tlast_i  = 'x;
        axi_s0_tdata_i  = 'x;
        axi_s0_tuser_i  = 'x;
      end else begin
        data = {480'h0, 32'hB0B00000 + port1_counter};
        user = {480'h0, 32'hB0B00000 + port1_counter};
        exp_data_1[exp_cnt_1] = data;
        exp_user_1[exp_cnt_1] = user;
        exp_last_1[exp_cnt_1] = last;
        exp_cnt_1++;
        port1_counter++;
        axi_s1_tdata_i  = data;
        axi_s1_tuser_i  = user;
        axi_s1_tkeep_i  = {TKEEP_L{1'b1}};
        axi_s1_tlast_i  = last;
        axi_s1_tvalid_i = 1;
        @(posedge clk);
        if (!axi_s1_tready_o) begin
          wait (axi_s1_tready_o);
          @(posedge clk);
        end
        axi_s1_tvalid_i = 0;
        axi_s1_tlast_i  = 'x;
        axi_s1_tdata_i  = 'x;
        axi_s1_tuser_i  = 'x;
      end
    end
  endtask

  // -----------------------------------------------------------------------
  // Main test sequence
  // -----------------------------------------------------------------------
  integer i;
  integer errors;

  initial begin
    rst_n = 0;
    repeat(20) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);

    // Both ports concurrently send 5 packets of 3 beats each
    fork
      begin : port0_driver
        repeat(5) begin
          send_beat(0, 0);
          send_beat(0, 0);
          send_beat(0, 1);
        end
      end
      begin : port1_driver
        repeat(5) begin
          send_beat(1, 0);
          send_beat(1, 0);
          send_beat(1, 1);
        end
      end
    join

    // Allow any in-flight beats to drain
    repeat(20) @(posedge clk);

    // -----------------------------------------------------------------------
    // End-of-simulation checks: count and per-port in-order content
    // -----------------------------------------------------------------------
    errors = 0;

    if (rcv_cnt_0 !== exp_cnt_0) begin
      $display("FAIL: Port 0 beat count mismatch — sent %0d, received %0d.", exp_cnt_0, rcv_cnt_0);
      errors++;
    end
    if (rcv_cnt_1 !== exp_cnt_1) begin
      $display("FAIL: Port 1 beat count mismatch — sent %0d, received %0d.", exp_cnt_1, rcv_cnt_1);
      errors++;
    end

    for (i = 0; i < exp_cnt_0 && i < rcv_cnt_0; i++) begin
      if (rcv_data_0[i] !== exp_data_0[i] ||
          rcv_user_0[i] !== exp_user_0[i] ||
          rcv_last_0[i] !== exp_last_0[i]) begin
        $display("FAIL: Port 0 beat[%0d] — got data=%h last=%b, exp data=%h last=%b",
                 i, rcv_data_0[i], rcv_last_0[i], exp_data_0[i], exp_last_0[i]);
        errors++;
      end
    end

    for (i = 0; i < exp_cnt_1 && i < rcv_cnt_1; i++) begin
      if (rcv_data_1[i] !== exp_data_1[i] ||
          rcv_user_1[i] !== exp_user_1[i] ||
          rcv_last_1[i] !== exp_last_1[i]) begin
        $display("FAIL: Port 1 beat[%0d] — got data=%h last=%b, exp data=%h last=%b",
                 i, rcv_data_1[i], rcv_last_1[i], exp_data_1[i], exp_last_1[i]);
        errors++;
      end
    end

    if (errors == 0)
      $display("PASS: All %0d port-0 beats and %0d port-1 beats received correctly.",
               rcv_cnt_0, rcv_cnt_1);
    else
      $display("FAIL: %0d error(s) detected.", errors);
    $stop;
  end

endmodule
