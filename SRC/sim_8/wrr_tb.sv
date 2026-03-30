`timescale 1ns / 1ps

module axi4_switch_wrr_tb();

  // Parameters
  localparam TDATA_L = 512;
  localparam TUSER_L = 81;
  localparam TKEEP_L = 16;
  localparam MAX_EXPECTED = 128;

  // Clock and reset
  reg clk = 0;
  reg rst_n = 0;
  always #5 clk = ~clk; // 100 MHz clock

  // Global cycle counter
  integer cycle = 0;

  // AXI4-Stream signals
  reg  [TDATA_L-1:0] axi_s0_tdata_i = 'x;
  reg  [TUSER_L-1:0] axi_s0_tuser_i = 'x;
  reg                axi_s0_tlast_i = 'x;
  reg  [TKEEP_L-1:0] axi_s0_tkeep_i = 'x;
  reg                axi_s0_tvalid_i = 0;
  wire               axi_s0_tready_o;

  reg  [TDATA_L-1:0] axi_s1_tdata_i = 'x;
  reg  [TUSER_L-1:0] axi_s1_tuser_i = 'x;
  reg                axi_s1_tlast_i = 'x;
  reg  [TKEEP_L-1:0] axi_s1_tkeep_i = 'x;
  reg                axi_s1_tvalid_i = 0;
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
    .TDATA_L(TDATA_L),
    .TUSER_L(TUSER_L),
    .TKEEP_L(TKEEP_L),
    .WEIGHT_L(4)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),
    .s0_weight(4'b1111),
    .s1_weight(4'b1111),
    .s_req_supress(s_req_supress),
    .axi_s0_tdata_i(axi_s0_tdata_i),
    .axi_s0_tuser_i(axi_s0_tuser_i),
    .axi_s0_tlast_i(axi_s0_tlast_i),
    .axi_s0_tkeep_i(axi_s0_tkeep_i),
    .axi_s0_tvalid_i(axi_s0_tvalid_i),
    .axi_s0_tready_o(axi_s0_tready_o),
    .axi_s1_tdata_i(axi_s1_tdata_i),
    .axi_s1_tuser_i(axi_s1_tuser_i),
    .axi_s1_tlast_i(axi_s1_tlast_i),
    .axi_s1_tkeep_i(axi_s1_tkeep_i),
    .axi_s1_tvalid_i(axi_s1_tvalid_i),
    .axi_s1_tready_o(axi_s1_tready_o),
    .axi_m0_tdata_o(axi_m0_tdata_o),
    .axi_m0_tuser_o(axi_m0_tuser_o),
    .axi_m0_tlast_o(axi_m0_tlast_o),
    .axi_m0_tkeep_o(axi_m0_tkeep_o),
    .axi_m0_tvalid_o(axi_m0_tvalid_o),
    .axi_m0_tready_i(axi_m0_tready_i)
  );

  reg [TDATA_L-1:0] expected_data_0 [0:MAX_EXPECTED-1];
  reg [TUSER_L-1:0] expected_user_0 [0:MAX_EXPECTED-1];
  reg               expected_last_0 [0:MAX_EXPECTED-1];
  reg [TDATA_L-1:0] expected_data_1 [0:MAX_EXPECTED-1];
  reg [TUSER_L-1:0] expected_user_1 [0:MAX_EXPECTED-1];
  reg               expected_last_1 [0:MAX_EXPECTED-1];
  integer expected_head_0 = 0, expected_tail_0 = 0;
  integer expected_head_1 = 0, expected_tail_1 = 0;

  integer port0_counter = 0;
  integer port1_counter = 0;

  task push_expected(input integer port, input [TDATA_L-1:0] data, input [TUSER_L-1:0] user, input bit last);
    begin
      if (port == 0) begin
        expected_data_0[expected_tail_0] = data;
        expected_user_0[expected_tail_0] = user;
        expected_last_0[expected_tail_0] = last;
        expected_tail_0++;
      end else begin
        expected_data_1[expected_tail_1] = data;
        expected_user_1[expected_tail_1] = user;
        expected_last_1[expected_tail_1] = last;
        expected_tail_1++;
      end
    end
  endtask

  task pop_and_check_output;
    reg [TDATA_L-1:0] exp_data;
    reg [TUSER_L-1:0] exp_user;
    reg               exp_last;
    begin
      if (axi_m0_tdata_o[28] == 1'b0) begin // Port 0: MSB = 0xA = 1010
        exp_data = expected_data_0[expected_head_0];
        exp_user = expected_user_0[expected_head_0];
        exp_last = expected_last_0[expected_head_0];
        expected_head_0++;
      end else begin
        exp_data = expected_data_1[expected_head_1];
        exp_user = expected_user_1[expected_head_1];
        exp_last = expected_last_1[expected_head_1];
        expected_head_1++;
      end

      if (axi_m0_tdata_o !== exp_data || axi_m0_tuser_o !== exp_user || axi_m0_tlast_o !== exp_last) begin
        $fatal("Mismatch: Got {data=%h, user=%h, last=%b}, Expected {data=%h, user=%h, last=%b}",
          axi_m0_tdata_o, axi_m0_tuser_o, axi_m0_tlast_o, exp_data, exp_user, exp_last);
      end
    end
  endtask

  task send_beat(input integer port, input bit last);
    reg [TDATA_L-1:0] data;
    reg [TUSER_L-1:0] user;
    begin
      if (port == 0) begin
        data = {480'h0, 32'hA0A00000 + port0_counter};
        user = {480'h0, 32'hA0A00000 + port0_counter};
        port0_counter++;
        axi_s0_tdata_i  = data;
        axi_s0_tuser_i  = user;
        axi_s0_tkeep_i  = {TKEEP_L{1'b1}};
        axi_s0_tlast_i  = last;
        axi_s0_tvalid_i = 1;
        push_expected(0, data, user, last);
        @(posedge clk);
        if (axi_s0_tready_o) begin
          axi_s0_tvalid_i = 0;
          axi_s0_tlast_i  = 'x;
        end else begin
          wait (axi_s0_tready_o);
          @(posedge clk);
          axi_s0_tvalid_i = 0;
          axi_s0_tlast_i  = 'x;
        end
      end else begin
        data = {480'h0, 32'hB0B00000 + port1_counter};
        user = {480'h0, 32'hB0B00000 + port1_counter};
        port1_counter++;
        axi_s1_tdata_i  = data;
        axi_s1_tuser_i  = user;
        axi_s1_tkeep_i  = {TKEEP_L{1'b1}};
        axi_s1_tlast_i  = last;
        axi_s1_tvalid_i = 1;
        push_expected(1, data, user, last);
        @(posedge clk);
        if (axi_s1_tready_o) begin
          axi_s1_tvalid_i = 0;
          axi_s1_tlast_i  = 'x;
        end else begin
          wait (axi_s1_tready_o);
          @(posedge clk);
          axi_s1_tvalid_i = 0;
          axi_s1_tlast_i  = 'x;
        end
      end
    end
  endtask

  always @(posedge clk) begin
    cycle <= cycle + 1;
    if (axi_m0_tvalid_o && axi_m0_tready_i) begin
      pop_and_check_output();
    end
  end

  initial begin
    axi_s0_tvalid_i = 0;
    axi_s1_tvalid_i = 0;
    repeat (20) @(posedge clk);
    rst_n = 1;
  end

  // Port 0 Process
  always @(posedge clk) begin
    if (!rst_n) disable port0_driver;
    else begin : port0_driver
      case (cycle)
        30: send_beat(0, 1);
        40: send_beat(0, 1);
        50: send_beat(0, 1);
        80: send_beat(0, 1);
        100: begin send_beat(0, 0); send_beat(0, 1); end
        120: begin send_beat(0, 0); send_beat(0, 1); end
        140: begin send_beat(0, 0); send_beat(0, 1); end
        150: begin send_beat(0, 0); send_beat(0, 1); send_beat(0, 0); send_beat(0, 1); end
        160: send_beat(0, 0);
        170: send_beat(0, 1);
      endcase
    end
  end

  // Port 1 Process
  always @(posedge clk) begin
    if (!rst_n) disable port1_driver;
    else begin : port1_driver
      case (cycle)
        35: send_beat(1, 1);
        70: begin send_beat(1, 0); send_beat(1, 0); send_beat(1, 1); end
        90: begin send_beat(1, 0); send_beat(1, 1); end
        110: send_beat(1, 1);
        130: begin send_beat(1, 0); send_beat(1, 1); end
        140: begin send_beat(1, 0); send_beat(1, 1); end
        150: begin send_beat(1, 0); send_beat(1, 1); send_beat(1, 0); send_beat(1, 1); end
        165: send_beat(1, 1);
      endcase
    end
  end

  initial begin
    repeat (180) @(posedge clk);
    if ((expected_head_0 != expected_tail_0) || (expected_head_1 != expected_tail_1)) begin
        $display("ERROR: Expected queue not empty. Port0: %0d remaining, Port1: %0d remaining",
                 expected_tail_0 - expected_head_0, expected_tail_1 - expected_head_1);
    end else $display("Test passed. All transactions matched expected output.");
    $stop;
  end

endmodule
