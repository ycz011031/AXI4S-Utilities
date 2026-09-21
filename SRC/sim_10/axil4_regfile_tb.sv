`timescale 1ns/1ps
//
// Self-checking testbench for axil4_regfile.
//
// Map under test (NUM_REGS = 6, 32-bit, non-power-of-two on purpose):
//   0x00 reg0  RW          init 0x0000_0001
//   0x04 reg1  RW          init 0x0000_0080
//   0x08 reg2  RO          <- hw_status[2]
//   0x0C reg3  W1C         <- hw_set[3]
//   0x10 reg4  self-clear  init 0x0000_0000
//   0x14 reg5  RW          init 0xDEAD_BEEF
//   0x18+      unmapped    -> DECERR
//
module axil4_regfile_tb;

localparam integer DW  = 32;
localparam integer AW  = 6;
localparam integer NR  = 6;

reg clk = 1'b0;
reg rstn = 1'b0;
always #5 clk = ~clk;

// AXI4-Lite master side
reg  [AW-1:0]   awaddr  = 0;
reg             awvalid = 0;
wire            awready;
reg  [DW-1:0]   wdata   = 0;
reg  [DW/8-1:0] wstrb   = 0;
reg             wvalid  = 0;
wire            wready;
wire [1:0]      bresp;
wire            bvalid;
reg             bready  = 0;
reg  [AW-1:0]   araddr  = 0;
reg             arvalid = 0;
wire            arready;
wire [DW-1:0]   rdata;
wire [1:0]      rresp;
wire            rvalid;
reg             rready  = 0;

reg  [NR*DW-1:0] hw_status = 0;
reg  [NR*DW-1:0] hw_set    = 0;
wire [NR*DW-1:0] reg_value;
wire [NR-1:0]    reg_wr_stb;
wire [NR-1:0]    reg_rd_stb;

axil4_regfile #(
    .C_S_AXI_DATA_WIDTH  (DW),
    .C_S_AXI_ADDR_WIDTH  (AW),
    .NUM_REGS            (NR),
    .REG_RW_MASK         (6'b100011),
    .REG_W1C_MASK        (6'b001000),
    .REG_SELF_CLEAR_MASK (6'b010000),
    .REG_INIT            ({32'hDEAD_BEEF, 32'h0000_0000, 32'h0000_0000,
                           32'h0000_0000, 32'h0000_0080, 32'h0000_0001}),
    .DECODE_ERR_EN       (1),
    .RO_WR_ERR_EN        (1)
) dut (
    .s_axi_aclk    (clk),
    .s_axi_aresetn (rstn),
    .s_axi_awaddr  (awaddr),
    .s_axi_awprot  (3'b000),
    .s_axi_awvalid (awvalid),
    .s_axi_awready (awready),
    .s_axi_wdata   (wdata),
    .s_axi_wstrb   (wstrb),
    .s_axi_wvalid  (wvalid),
    .s_axi_wready  (wready),
    .s_axi_bresp   (bresp),
    .s_axi_bvalid  (bvalid),
    .s_axi_bready  (bready),
    .s_axi_araddr  (araddr),
    .s_axi_arprot  (3'b000),
    .s_axi_arvalid (arvalid),
    .s_axi_arready (arready),
    .s_axi_rdata   (rdata),
    .s_axi_rresp   (rresp),
    .s_axi_rvalid  (rvalid),
    .s_axi_rready  (rready),
    .hw_status     (hw_status),
    .hw_set        (hw_set),
    .reg_value     (reg_value),
    .reg_wr_stb    (reg_wr_stb),
    .reg_rd_stb    (reg_rd_stb)
);

integer errors = 0;
integer checks = 0;

task check32(input [255:0] name, input [DW-1:0] got, input [DW-1:0] exp);
begin
    checks = checks + 1;
    if (got !== exp) begin
        errors = errors + 1;
        $display("  FAIL %0s: got 0x%08x expected 0x%08x", name, got, exp);
    end else
        $display("  pass %0s = 0x%08x", name, got);
end
endtask

task check2(input [255:0] name, input [1:0] got, input [1:0] exp);
begin
    checks = checks + 1;
    if (got !== exp) begin
        errors = errors + 1;
        $display("  FAIL %0s: resp %b expected %b", name, got, exp);
    end else
        $display("  pass %0s resp = %b", name, got);
end
endtask

// ---------------------------------------------------------------------------
// AXI4-Lite BFM.  All stimulus is driven on the falling edge and all DUT
// outputs are sampled there too, so nothing races the DUT clock edge: a
// (valid && ready) pair observed at a negedge transfers on the next posedge.
// ---------------------------------------------------------------------------
task axi_write(input [AW-1:0] addr, input [DW-1:0] data,
               input [DW/8-1:0] strb, output [1:0] resp);
    reg aw_done, w_done;
begin
    @(negedge clk);
    awaddr = addr; awvalid = 1'b1;
    wdata  = data; wstrb = strb; wvalid = 1'b1;
    bready = 1'b1;
    aw_done = 1'b0; w_done = 1'b0;
    while (!aw_done || !w_done) begin
        if (awvalid && awready) aw_done = 1'b1;
        if (wvalid  && wready)  w_done  = 1'b1;
        @(negedge clk);
        if (aw_done) awvalid = 1'b0;
        if (w_done)  wvalid  = 1'b0;
    end
    while (!bvalid) @(negedge clk);
    resp = bresp;
    @(negedge clk);
    bready = 1'b0;
end
endtask

// Write with W presented before AW (legal, and the usual case behind a
// register slice / crossbar).
task axi_write_wfirst(input [AW-1:0] addr, input [DW-1:0] data,
                      input [DW/8-1:0] strb, output [1:0] resp);
begin
    @(negedge clk);
    wdata = data; wstrb = strb; wvalid = 1'b1; bready = 1'b1;
    while (!wready) @(negedge clk);
    @(negedge clk);
    wvalid = 1'b0;
    repeat (3) @(negedge clk);          // leave W parked in the holding reg
    awaddr = addr; awvalid = 1'b1;
    while (!awready) @(negedge clk);
    @(negedge clk);
    awvalid = 1'b0;
    while (!bvalid) @(negedge clk);
    resp = bresp;
    @(negedge clk);
    bready = 1'b0;
end
endtask

task axi_read(input [AW-1:0] addr, output [DW-1:0] data, output [1:0] resp);
begin
    @(negedge clk);
    araddr = addr; arvalid = 1'b1; rready = 1'b1;
    while (!arready) @(negedge clk);
    @(negedge clk);
    arvalid = 1'b0;
    while (!rvalid) @(negedge clk);
    data = rdata;
    resp = rresp;
    @(negedge clk);
    rready = 1'b0;
end
endtask

// Capture the self-clearing register in the cycle its write strobe is high.
reg [DW-1:0] cap_r4 = 0;
always @(posedge clk)
    if (reg_wr_stb[4]) cap_r4 <= reg_value[4*DW +: DW];

// Count write strobes per register to prove they pulse exactly once.
reg [7:0] wr_stb_cnt [0:NR-1];
reg [7:0] rd_stb_cnt [0:NR-1];
integer k;
initial for (k = 0; k < NR; k = k + 1) begin
    wr_stb_cnt[k] = 0; rd_stb_cnt[k] = 0;
end
always @(posedge clk) if (rstn) begin
    for (k = 0; k < NR; k = k + 1) begin
        if (reg_wr_stb[k]) wr_stb_cnt[k] = wr_stb_cnt[k] + 1;
        if (reg_rd_stb[k]) rd_stb_cnt[k] = rd_stb_cnt[k] + 1;
    end
end

reg [DW-1:0] d;
reg [1:0]    r;

initial begin
    hw_status = 0;
    hw_set    = 0;
    repeat (4) @(negedge clk);
    rstn = 1'b1;
    repeat (2) @(negedge clk);

    $display("\n--- 1. reset values read back ---");
    axi_read(6'h00, d, r); check32("reg0 init", d, 32'h0000_0001); check2("reg0", r, 2'b00);
    axi_read(6'h04, d, r); check32("reg1 init", d, 32'h0000_0080);
    axi_read(6'h14, d, r); check32("reg5 init", d, 32'hDEAD_BEEF);
    check32("reg_value[5] init", reg_value[5*DW +: DW], 32'hDEAD_BEEF);

    $display("\n--- 2. read-only register tracks hw_status ---");
    hw_status[2*DW +: DW] = 32'hCAFE_BABE;
    axi_read(6'h08, d, r); check32("reg2 RO", d, 32'hCAFE_BABE); check2("reg2", r, 2'b00);
    hw_status[2*DW +: DW] = 32'h1234_5678;
    axi_read(6'h08, d, r); check32("reg2 RO updated", d, 32'h1234_5678);

    $display("\n--- 3. RW write / readback / reg_value fanout ---");
    axi_write(6'h00, 32'h1234_5678, 4'hF, r); check2("wr reg0", r, 2'b00);
    axi_read(6'h00, d, r);  check32("reg0 readback", d, 32'h1234_5678);
    check32("reg_value[0]", reg_value[0*DW +: DW], 32'h1234_5678);

    $display("\n--- 4. byte strobes ---");
    axi_write(6'h04, 32'hFFFF_FFFF, 4'b0010, r);
    check32("reg1 byte1 only", reg_value[1*DW +: DW], 32'h0000_FF80);
    axi_write(6'h04, 32'hAABB_CCDD, 4'b1001, r);
    check32("reg1 bytes 3,0", reg_value[1*DW +: DW], 32'hAA00_FFDD);
    axi_write(6'h04, 32'h0000_0000, 4'b0000, r);
    check32("reg1 no strobes", reg_value[1*DW +: DW], 32'hAA00_FFDD);

    $display("\n--- 5. W1C register ---");
    @(negedge clk); hw_set[3*DW +: DW] = 32'h0000_0005;
    @(negedge clk); hw_set[3*DW +: DW] = 32'h0000_0000;
    axi_read(6'h0C, d, r); check32("reg3 set by hw", d, 32'h0000_0005);
    axi_write(6'h0C, 32'h0000_0001, 4'hF, r);           // clear bit 0
    axi_read(6'h0C, d, r); check32("reg3 after W1C", d, 32'h0000_0004);
    axi_write(6'h0C, 32'h0000_0004, 4'b0000, r);        // no strobes -> no clear
    axi_read(6'h0C, d, r); check32("reg3 W1C strobe-gated", d, 32'h0000_0004);
    // hardware set must win over a simultaneous software clear: drive hw_set
    // during the very cycle the write commits, so both land on the same edge
    fork
        axi_write(6'h0C, 32'hFFFF_FFFF, 4'hF, r);
        begin
            wait (dut.wr_sel[3] === 1'b1);              // commit cycle, pre-edge
            hw_set[3*DW +: DW] = 32'h0000_0040;
            @(posedge clk);                             // set and clear collide here
            @(negedge clk);
            hw_set[3*DW +: DW] = 32'h0000_0000;
        end
    join
    axi_read(6'h0C, d, r); check32("reg3 set beats clear", d, 32'h0000_0040);
    axi_write(6'h0C, 32'hFFFF_FFFF, 4'hF, r);
    axi_read(6'h0C, d, r); check32("reg3 fully cleared", d, 32'h0000_0000);

    $display("\n--- 6. self-clearing command register ---");
    axi_write(6'h10, 32'h0000_ABCD, 4'hF, r);
    check32("reg4 held one cycle", cap_r4, 32'h0000_ABCD);
    check32("reg4 self-cleared", reg_value[4*DW +: DW], 32'h0000_0000);
    axi_read(6'h10, d, r); check32("reg4 reads back init", d, 32'h0000_0000);

    $display("\n--- 7. write to read-only register ---");
    axi_write(6'h08, 32'hFFFF_FFFF, 4'hF, r);
    check2("wr reg2 (RO)", r, 2'b10);                   // SLVERR, RO_WR_ERR_EN=1
    axi_read(6'h08, d, r); check32("reg2 unchanged", d, 32'h1234_5678);

    $display("\n--- 8. decode errors past the end of the map ---");
    axi_read(6'h18, d, r);  check2("rd 0x18", r, 2'b11); check32("rd 0x18 data", d, 32'h0);
    axi_read(6'h3C, d, r);  check2("rd 0x3C", r, 2'b11);
    axi_write(6'h18, 32'hFFFF_FFFF, 4'hF, r); check2("wr 0x18", r, 2'b11);
    axi_read(6'h14, d, r);  check32("reg5 untouched", d, 32'hDEAD_BEEF);

    $display("\n--- 9. W before AW ---");
    axi_write_wfirst(6'h14, 32'h0BAD_F00D, 4'hF, r);
    check2("wfirst resp", r, 2'b00);
    axi_read(6'h14, d, r);  check32("reg5 wfirst", d, 32'h0BAD_F00D);

    $display("\n--- 10. back-to-back traffic ---");
    axi_write(6'h00, 32'h1111_1111, 4'hF, r);
    axi_write(6'h04, 32'h2222_2222, 4'hF, r);
    axi_write(6'h14, 32'h3333_3333, 4'hF, r);
    axi_read(6'h00, d, r); check32("b2b reg0", d, 32'h1111_1111);
    axi_read(6'h04, d, r); check32("b2b reg1", d, 32'h2222_2222);
    axi_read(6'h14, d, r); check32("b2b reg5", d, 32'h3333_3333);

    $display("\n--- 11. strobe outputs ---");
    // reg0 written in tests 3 and 10 -> 2 pulses; reg2 is RO -> never strobed.
    check32("reg0 wr_stb count", {24'h0, wr_stb_cnt[0]}, 32'd2);
    check32("reg2 wr_stb count", {24'h0, wr_stb_cnt[2]}, 32'd0);
    check32("reg4 wr_stb count", {24'h0, wr_stb_cnt[4]}, 32'd1);
    if (rd_stb_cnt[2] == 0) begin
        errors = errors + 1;
        $display("  FAIL reg2 rd_stb never pulsed");
    end else
        $display("  pass reg2 rd_stb count = %0d", rd_stb_cnt[2]);
    checks = checks + 1;

    $display("\n=====================================================");
    if (errors == 0) $display(" PASS - %0d checks, 0 errors", checks);
    else             $display(" FAIL - %0d checks, %0d errors", checks, errors);
    $display("=====================================================\n");
    $finish;
end

initial begin
    #200000;
    $display("TIMEOUT");
    $finish;
end

endmodule
