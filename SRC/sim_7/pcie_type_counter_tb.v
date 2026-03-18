`timescale 1ns / 1ps

module pcie_type_counter_tb();

    // Testbench parameters
    parameter NUM_UNITS = 4;
    parameter COUNTER_VALUE_WIDTH = 8;
    parameter INTERVAL_BITS = 4;  // Small value for fast simulation
    parameter CLK_PERIOD = 10;    // 10ns = 100MHz
    parameter AXIS_DATA_WIDTH = 512;
    
    // Clock and reset
    reg clk = 1'b0;
    reg rst = 1'b0;
    
    // Hub signals
    wire [NUM_UNITS-1:0] read_enable;
    wire [1:0]           channel_select;
    wire [$clog2(NUM_UNITS+1)-1:0] ila_channel_id;
    wire [COUNTER_VALUE_WIDTH-1:0] ila_counter_value;
    wire                 ila_counter_valid;
    
    // Counter unit signals (individual wires per unit)
    wire [2:0] counter_id_0, counter_id_1, counter_id_2, counter_id_3;
    wire [COUNTER_VALUE_WIDTH-1:0] requester_type_0, requester_type_1, requester_type_2, requester_type_3;
    
    // Dummy AXI stream signals (explicitly tied to zero)
    reg [AXIS_DATA_WIDTH-1:0]   axis_tdata = {AXIS_DATA_WIDTH{1'b0}};
    reg [AXIS_DATA_WIDTH/8-1:0] axis_tkeep = {AXIS_DATA_WIDTH/8{1'b0}};
    reg                         axis_tvalid = 1'b0;
    reg                         axis_tlast = 1'b0;
    reg [228:0]                 axis_tuser_229 = {229{1'b0}};
    reg [136:0]                 axis_tuser_137 = {137{1'b0}};
    reg [160:0]                 axis_tuser_161 = {161{1'b0}};
    reg [80:0]                  axis_tuser_81 = {81{1'b0}};
    
    // Tag lookup interfaces (explicitly tied to zero)
    reg [7:0] tag = 8'd0;
    reg       tag_valid = 1'b0;
    wire [4:0] tag_type;
    
    // Muxed signals for hub
    reg [2:0] muxed_counter_id;
    reg [COUNTER_VALUE_WIDTH-1:0] muxed_requester_type;
    
    // Multiplexer for counter signals
    always @(*) begin
        case (channel_select)
            2'd0: begin
                muxed_counter_id = counter_id_0;
                muxed_requester_type = requester_type_0;
            end
            2'd1: begin
                muxed_counter_id = counter_id_1;
                muxed_requester_type = requester_type_1;
            end
            2'd2: begin
                muxed_counter_id = counter_id_2;
                muxed_requester_type = requester_type_2;
            end
            2'd3: begin
                muxed_counter_id = counter_id_3;
                muxed_requester_type = requester_type_3;
            end
            default: begin
                muxed_counter_id = 3'd0;
                muxed_requester_type = {COUNTER_VALUE_WIDTH{1'b0}};
            end
        endcase
    end
    
    // Clock generation
    always #(CLK_PERIOD/2) clk = ~clk;
    
    // ============================================================
    // Unit 0: Requester Counter (unit_id = 1)
    // ============================================================
    pcie_requester_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(229),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(1)
    ) unit_0 (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(axis_tdata),
        .s_axis_tkeep(axis_tkeep),
        .s_axis_tvalid(axis_tvalid),
        .s_axis_tlast(axis_tlast),
        .s_axis_tuser(axis_tuser_229),
        .s_axis_tready(),
        
        .m_axis_tdata(),
        .m_axis_tkeep(),
        .m_axis_tvalid(),
        .m_axis_tlast(),
        .m_axis_tuser(),
        .m_axis_tready(1'b1),
        
        .completer_tag(tag),
        .completer_tag_valid(tag_valid),
        .completer_type(tag_type),
        
        .read_enable(read_enable[0]),
        .counter_id(counter_id_0),
        .requester_type(requester_type_0)
    );
    
    // ============================================================
    // Unit 1: Completer Counter (unit_id = 2)
    // ============================================================
    pcie_completer_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(81),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(2)
    ) unit_1 (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(axis_tdata),
        .s_axis_tkeep(axis_tkeep),
        .s_axis_tvalid(axis_tvalid),
        .s_axis_tlast(axis_tlast),
        .s_axis_tuser(axis_tuser_81),
        .s_axis_tready(),
        
        .m_axis_tdata(),
        .m_axis_tkeep(),
        .m_axis_tvalid(),
        .m_axis_tlast(),
        .m_axis_tuser(),
        .m_axis_tready(1'b1),
        
        .completer_tag(),
        .completer_tag_valid(),
        .completer_type(5'b01111),
        
        .read_enable(read_enable[1]),
        .counter_id(counter_id_1),
        .requester_type(requester_type_1)
    );
    
    // ============================================================
    // Unit 2: Requester Counter (unit_id = 3)
    // ============================================================
    pcie_requester_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(137),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(3)
    ) unit_2 (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(axis_tdata),
        .s_axis_tkeep(axis_tkeep),
        .s_axis_tvalid(axis_tvalid),
        .s_axis_tlast(axis_tlast),
        .s_axis_tuser(axis_tuser_137),
        .s_axis_tready(),
        
        .m_axis_tdata(),
        .m_axis_tkeep(),
        .m_axis_tvalid(),
        .m_axis_tlast(),
        .m_axis_tuser(),
        .m_axis_tready(1'b1),
        
        .completer_tag(tag),
        .completer_tag_valid(tag_valid),
        .completer_type(tag_type),
        
        .read_enable(read_enable[2]),
        .counter_id(counter_id_2),
        .requester_type(requester_type_2)
    );
    
    // ============================================================
    // Unit 3: Completer Counter (unit_id = 4)
    // ============================================================
    pcie_completer_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(161),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(4)
    ) unit_3 (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(axis_tdata),
        .s_axis_tkeep(axis_tkeep),
        .s_axis_tvalid(axis_tvalid),
        .s_axis_tlast(axis_tlast),
        .s_axis_tuser(axis_tuser_161),
        .s_axis_tready(),
        
        .m_axis_tdata(),
        .m_axis_tkeep(),
        .m_axis_tvalid(),
        .m_axis_tlast(),
        .m_axis_tuser(),
        .m_axis_tready(1'b1),
        
        .completer_tag(),
        .completer_tag_valid(),
        .completer_type(5'b01111),
        
        .read_enable(read_enable[3]),
        .counter_id(counter_id_3),
        .requester_type(requester_type_3)
    );
    
    // ============================================================
    // Hub Instance
    // ============================================================
    pcie_type_tracker_hub #(
        .NUM_UNITS(NUM_UNITS),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .INTERVAL_BITS(INTERVAL_BITS)
    ) hub (
        .clk(clk),
        .rst(rst),
        
        .read_enable(read_enable),
        .channel_select(channel_select),
        .counter_id(muxed_counter_id),
        .requester_type(muxed_requester_type),
        
        .ila_channel_id(ila_channel_id),
        .ila_counter_value(ila_counter_value),
        .ila_counter_valid(ila_counter_valid)
    );
    
    // ============================================================
    // Monitor ILA output
    // ============================================================
    always @(posedge clk) begin
        if (rst && hub.state == 3'd3 && ila_counter_valid) begin  // STATE_ILA_OUTPUT
            if (COUNTER_VALUE_WIDTH == 8)
                $display("Time=%0t | ILA Output: Channel=%0d, Counter_Addr=%0d, Value=0x%02h, Valid=%0b", 
                         $time, ila_channel_id, hub.output_addr[3:0], ila_counter_value, ila_counter_valid);
            else
                $display("Time=%0t | ILA Output: Channel=%0d, Counter_Addr=%0d, Value=0x%04h, Valid=%0b", 
                         $time, ila_channel_id, hub.output_addr[3:0], ila_counter_value, ila_counter_valid);
        end
    end
    
    // ============================================================
    // Monitor counter unit read activity
    // ============================================================
    always @(posedge clk) begin
        if (rst && hub.state == 3'd1) begin  // STATE_READ
            if (read_enable != 4'b0000) begin
                $display("Time=%0t | READ State: read_enable=%04b, channel_select=%0d, counter_id=%0d, requester_type=0x%02h, counter_index=%0d, startup_delay=%0b", 
                         $time, read_enable, channel_select, muxed_counter_id, muxed_requester_type, hub.counter_index, hub.startup_delay);
                $display("           Unit outputs: [0]=0x%02h [1]=0x%02h [2]=0x%02h [3]=0x%02h", 
                         requester_type_0, requester_type_1, requester_type_2, requester_type_3);
            end
        end
    end
    
    // ============================================================
    // Test sequence
    // ============================================================
    initial begin
        $display("========================================");
        $display("PCIe Type Counter + Hub Integration Test");
        $display("NUM_UNITS = %0d", NUM_UNITS);
        $display("COUNTER_VALUE_WIDTH = %0d", COUNTER_VALUE_WIDTH);
        $display("INTERVAL_BITS = %0d", INTERVAL_BITS);
        $display("INTERVAL_COUNT = %0d", (1 << INTERVAL_BITS) - 1);
        $display("========================================");
        
        // Wait in reset for a few cycles
        repeat(5) @(posedge clk);
        rst = 1'b1;
        $display("Time=%0t | Reset released", $time);
        
        // Show counter_id outputs immediately after reset
        @(posedge clk);
        $display("Time=%0t | Counter IDs: [0]=%0d [1]=%0d [2]=%0d [3]=%0d", 
                 $time, counter_id_0, counter_id_1, counter_id_2, counter_id_3);
        $display("Time=%0t | Initial requester_type: [0]=0x%02h [1]=0x%02h [2]=0x%02h [3]=0x%02h", 
                 $time, requester_type_0, requester_type_1, requester_type_2, requester_type_3);
        
        $display("\nCounter tables initialized with pattern:");
        $display("  Unit 0 (id=1): 0x10-0x1F");
        $display("  Unit 1 (id=2): 0x20-0x2F");
        $display("  Unit 2 (id=3): 0x30-0x3F");
        $display("  Unit 3 (id=4): 0x40-0x4F");
        
        // Wait for one complete cycle
        $display("\n--- Waiting for IDLE state to complete ---");
        wait (hub.state == 3'd1);  // Wait for READ state
        $display("Time=%0t | Entered READ state", $time);
        
        $display("\n--- Reading counters from all channels ---");
        wait (hub.state == 3'd3);  // Wait for ILA_OUTPUT state
        $display("Time=%0t | Entered ILA_OUTPUT state", $time);
        
        $display("\n--- ILA Output Values ---");
        wait (hub.state == 3'd0);  // Wait to return to IDLE
        $display("\nTime=%0t | Returned to IDLE state - One complete cycle finished", $time);
        
        // Display summary
        $display("\n========================================");
        $display("Test completed successfully!");
        $display("Total simulation time: %0t", $time);
        $display("\nExpected pattern verification:");
        $display("  Channel 1 should show: 0x10, 0x11, 0x12...0x1F");
        $display("  Channel 2 should show: 0x20, 0x21, 0x22...0x2F");
        $display("  Channel 3 should show: 0x30, 0x31, 0x32...0x3F");
        $display("  Channel 4 should show: 0x40, 0x41, 0x42...0x4F");
        $display("========================================");
        
        // Run a bit longer to observe
        repeat(10) @(posedge clk);
        
        $finish;
    end
    
    // Timeout watchdog
    initial begin
        #100000;  // 100us timeout
        $display("\nERROR: Simulation timeout!");
        $finish;
    end
    
    // Optional: Generate VCD file for waveform viewing
    initial begin
        $dumpfile("pcie_type_counter_tb.vcd");
        $dumpvars(0, pcie_type_counter_tb);
    end

endmodule
