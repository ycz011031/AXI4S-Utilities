`timescale 1ns / 1ps

module pcie_ats_tb();

    // Parameters matching the DUT
    localparam integer AXIS_DATA_WIDTH  = 512;
    localparam integer AXIS_TUSER_WIDTH = 229;
    localparam integer RQ_AXIS_TUSER_W  = 183;
    
    // Clock and Reset
    reg clk;
    reg rst;
    
    // AXI-Stream CQ Input signals
    reg [AXIS_DATA_WIDTH-1:0]    s_axis_tdata;
    reg [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep;
    reg                          s_axis_tvalid;
    reg                          s_axis_tlast;
    reg [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser;
    wire                         s_axis_tready;
    
    // AXI-Stream CQ Output signals (transparent)
    wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata;
    wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_tkeep;
    wire                          m_axis_tvalid;
    wire                          m_axis_tlast;
    wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser;
    reg                           m_axis_tready;
    
    // RQ AXI-Stream Output signals
    wire [AXIS_DATA_WIDTH-1:0]    rq_axis_tdata;
    wire [AXIS_DATA_WIDTH/8-1:0]  rq_axis_tkeep;
    wire                          rq_axis_tvalid;
    wire [RQ_AXIS_TUSER_W-1:0]    rq_axis_tuser;
    reg                           rq_axis_tready;
    wire                          rq_axis_tlast;
    
    // Debug outputs
    wire                           ats_hit;
    wire [7:0]                     ats_tag;
    wire [7:0]                     ats_msg_code;
    wire [2:0]                     ats_msg_routing;
    wire [AXIS_DATA_WIDTH-1:0]     ats_tdata;
    wire [AXIS_DATA_WIDTH/8-1:0]   ats_tkeep;
    wire [AXIS_TUSER_WIDTH-1:0]    ats_tuser;
    
    // Clock generation - 250 MHz (4ns period)
    initial begin
        clk = 0;
        forever #2 clk = ~clk;
    end
    
    // DUT instantiation
    pcie_cq_ats_snoop #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(AXIS_TUSER_WIDTH),
        .RQ_AXIS_TUSER_W(RQ_AXIS_TUSER_W)
    ) dut (
        .clk(clk),
        .rst(rst),
        // CQ Input
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tkeep(s_axis_tkeep),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tlast(s_axis_tlast),
        .s_axis_tuser(s_axis_tuser),
        .s_axis_tready(s_axis_tready),
        // CQ Output (transparent)
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tkeep(m_axis_tkeep),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tuser(m_axis_tuser),
        .m_axis_tready(m_axis_tready),
        // RQ Output
        .rq_axis_tdata(rq_axis_tdata),
        .rq_axis_tkeep(rq_axis_tkeep),
        .rq_axis_tvalid(rq_axis_tvalid),
        .rq_axis_tuser(rq_axis_tuser),
        .rq_axis_tready(rq_axis_tready),
        .rq_axis_tlast(rq_axis_tlast),
        // Debug outputs
        .ats_hit(ats_hit),
        .ats_tag(ats_tag),
        .ats_msg_code(ats_msg_code),
        .ats_msg_routing(ats_msg_routing),
        .ats_tdata(ats_tdata),
        .ats_tkeep(ats_tkeep),
        .ats_tuser(ats_tuser)
    );
    
    // Main test sequence
    initial begin
        // Initialize signals
        rst <= 0;
        s_axis_tdata <= 512'd0;
        s_axis_tkeep <= 64'd0;
        s_axis_tvalid <= 1'b0;
        s_axis_tlast <= 1'b0;
        s_axis_tuser <= 229'd0;
        m_axis_tready <= 1'b1;  // Always ready to accept passthrough data
        rq_axis_tready <= 1'b1; // Always ready to accept RQ completions
        
        // Apply reset
        #10;
        rst <= 1;
        #20;
        
        $display("========================================");
        $display("Starting ATS Invalidation Request Test");
        $display("========================================");
        
        // Wait a few cycles
        repeat(5) @(posedge clk);
        
        // Send captured ATS data from host
        @(posedge clk);
        s_axis_tdata  <= 512'h0000000000000000000000000000000000000000000000000000000000000000000000000000000000f0ffff0000000000020100960070020000000000000098;
        s_axis_tkeep  <= 64'hffffffffffff003f;
        s_axis_tuser  <= 229'h000000000000000000007fffce7f00000005410000000000ff00000f0f;
        s_axis_tvalid <= 1'b1;
        s_axis_tlast  <= 1'b1;
        
        $display("Time=%0t: Sending ATS Invalidation Request", $time);
        $display("  tdata = %h", s_axis_tdata);
        $display("  tkeep = %h", s_axis_tkeep);
        $display("  tuser = %h", s_axis_tuser);
        
        // Wait for handshake
        @(posedge clk);
        while (!s_axis_tready) @(posedge clk);
        
        // Deassert valid after one clock cycle
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;
        s_axis_tdata  <= 512'd0;
        s_axis_tkeep  <= 64'd0;
        s_axis_tuser  <= 229'd0;
        
        // Wait for ATS hit detection and RQ response
        repeat(10) @(posedge clk);
        
        $display("\n========================================");
        $display("Test Complete");
        $display("========================================");
        $finish;
    end
    
    // Monitor ATS hit
    always @(posedge clk) begin
        if (ats_hit) begin
            $display("\nTime=%0t: *** ATS HIT DETECTED ***", $time);
            $display("  ats_tag         = 0x%h", ats_tag);
            $display("  ats_msg_code    = 0x%h", ats_msg_code);
            $display("  ats_msg_routing = 0x%h", ats_msg_routing);
            $display("  ats_tdata       = 0x%h", ats_tdata);
            $display("  ats_tkeep       = 0x%h", ats_tkeep);
            $display("  ats_tuser       = 0x%h", ats_tuser);
        end
    end
    
    // Monitor RQ output (Invalidation Completion)
    always @(posedge clk) begin
        if (rq_axis_tvalid && rq_axis_tready) begin
            $display("\nTime=%0t: RQ Invalidation Completion Sent", $time);
            $display("  rq_tdata  = 0x%h", rq_axis_tdata);
            $display("  rq_tkeep  = 0x%h", rq_axis_tkeep);
            $display("  rq_tuser  = 0x%h", rq_axis_tuser);
            $display("  rq_tlast  = %b", rq_axis_tlast);
        end
    end
    
    // Monitor CQ passthrough
    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            $display("\nTime=%0t: CQ Passthrough Transaction", $time);
            $display("  m_tdata  = 0x%h", m_axis_tdata);
            $display("  m_tkeep  = 0x%h", m_axis_tkeep);
            $display("  m_tlast  = %b", m_axis_tlast);
        end
    end
    
    // Dump waveforms
    initial begin
        $dumpfile("pcie_ats_tb.vcd");
        $dumpvars(0, pcie_ats_tb);
    end

endmodule
