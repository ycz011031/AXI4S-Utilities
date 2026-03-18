`timescale 1ns / 1ps

module pcie_monitor_hub_tb();

    // Testbench parameters
    parameter NUM_UNITS = 4;
    parameter COUNTER_VALUE_WIDTH = 8;  // 8 for uint8, 16 for uint16
    parameter INTERVAL_BITS = 4;  // Small value for fast simulation
    parameter CLK_PERIOD = 10;    // 10ns = 100MHz
    
    // DUT signals
    reg                          clk;
    reg                          rst;
    wire [NUM_UNITS-1:0]         read_enable;
    wire [1:0]                   channel_select;  // $clog2(4) = 2
    reg  [2:0]                   counter_id;
    reg  [COUNTER_VALUE_WIDTH-1:0] requester_type;
    wire [$clog2(NUM_UNITS+1)-1:0] ila_channel_id;
    wire [COUNTER_VALUE_WIDTH-1:0] ila_counter_value;
    wire                         ila_counter_valid;
    
    // Testbench variables
    integer cycle_count;
    integer i;
    
    // Instantiate DUT
    pcie_type_tracker_hub #(
        .NUM_UNITS(NUM_UNITS),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .INTERVAL_BITS(INTERVAL_BITS)
    ) dut (
        .clk(clk),
        .rst(rst),
        .read_enable(read_enable),
        .channel_select(channel_select),
        .counter_id(counter_id),
        .requester_type(requester_type),
        .ila_channel_id(ila_channel_id),
        .ila_counter_value(ila_counter_value),
        .ila_counter_valid(ila_counter_valid)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    // Counter simulation logic - mimics edge-triggered atomic read sequence
    // Encoding: 8'hXY where X = channel (1-based), Y = counter_id
    
    // Simulate counter unit behavior with edge detection and atomic sequence
    reg [3:0] sim_counter_index [3:0];  // Counter index for each unit
    reg       sim_cycle_toggle [3:0];   // Cycle toggle for each unit
    reg       sim_read_enable_prev [3:0]; // Previous read_enable for edge detection
    reg       sim_read_active [3:0];    // Active read sequence flag
    integer ch;
    
    always @(posedge clk) begin
        if (!rst) begin
            counter_id <= 3'd0;
            requester_type <= {COUNTER_VALUE_WIDTH{1'b0}};
            for (ch = 0; ch < 4; ch = ch + 1) begin
                sim_counter_index[ch] <= 4'd0;
                sim_cycle_toggle[ch] <= 1'b0;
                sim_read_enable_prev[ch] <= 1'b0;
                sim_read_active[ch] <= 1'b0;
            end
        end else begin
            // Simulate each channel's counter unit
            for (ch = 0; ch < 4; ch = ch + 1) begin
                // Store previous read_enable
                sim_read_enable_prev[ch] <= read_enable[ch];
                
                // Detect rising edge
                if (read_enable[ch] && !sim_read_enable_prev[ch]) begin
                    sim_read_active[ch] <= 1'b1;
                    sim_counter_index[ch] <= 4'd0;
                    sim_cycle_toggle[ch] <= 1'b0;
                end
                
                // Active read sequence
                if (sim_read_active[ch]) begin
                    sim_cycle_toggle[ch] <= ~sim_cycle_toggle[ch];
                    
                    if (sim_cycle_toggle[ch]) begin
                        if (sim_counter_index[ch] == 4'd15) begin
                            sim_read_active[ch] <= 1'b0;
                            sim_counter_index[ch] <= 4'd0;
                        end else begin
                            sim_counter_index[ch] <= sim_counter_index[ch] + 1'b1;
                        end
                    end
                end
            end
            
            // Output based on which channel is currently reading
            if (sim_read_active[0]) begin
                counter_id <= 3'd1;
                requester_type <= {{COUNTER_VALUE_WIDTH-8{1'b0}}, 4'h1, sim_counter_index[0]};
            end else if (sim_read_active[1]) begin
                counter_id <= 3'd2;
                requester_type <= {{COUNTER_VALUE_WIDTH-8{1'b0}}, 4'h2, sim_counter_index[1]};
            end else if (sim_read_active[2]) begin
                counter_id <= 3'd3;
                requester_type <= {{COUNTER_VALUE_WIDTH-8{1'b0}}, 4'h3, sim_counter_index[2]};
            end else if (sim_read_active[3]) begin
                counter_id <= 3'd4;
                requester_type <= {{COUNTER_VALUE_WIDTH-8{1'b0}}, 4'h4, sim_counter_index[3]};
            end else begin
                counter_id <= 3'd0;
                requester_type <= {COUNTER_VALUE_WIDTH{1'b0}};
            end
        end
    end
    
    // Monitor ILA output
    always @(posedge clk) begin
        if (rst && dut.state == 3'd3 && ila_counter_valid) begin  // STATE_ILA_OUTPUT = 3'd3
            if (COUNTER_VALUE_WIDTH == 8)
                $display("Time=%0t | ILA Output: Channel=%0d, Counter_Addr=%0d, Value=0x%02h, Valid=%0b", 
                         $time, ila_channel_id, dut.output_addr[3:0], ila_counter_value, ila_counter_valid);
            else
                $display("Time=%0t | ILA Output: Channel=%0d, Counter_Addr=%0d, Value=0x%04h, Valid=%0b", 
                         $time, ila_channel_id, dut.output_addr[3:0], ila_counter_value, ila_counter_valid);
        end
    end
    
    // Test sequence
    initial begin
        $display("========================================");
        $display("PCIe Type Tracker Hub Testbench");
        $display("NUM_UNITS = %0d", NUM_UNITS);
        $display("COUNTER_VALUE_WIDTH = %0d", COUNTER_VALUE_WIDTH);
        $display("INTERVAL_BITS = %0d", INTERVAL_BITS);
        $display("INTERVAL_COUNT = %0d", (1 << INTERVAL_BITS) - 1);
        $display("========================================");
        
        // Initialize
        rst = 0;
        cycle_count = 0;
        
        // Apply reset
        repeat(5) @(posedge clk);
        rst = 1;
        $display("Time=%0t | Reset released", $time);
        
        // Wait for one complete cycle (IDLE -> READ -> DELAY -> ... -> ILA_OUTPUT -> back to IDLE)
        $display("\n--- Waiting for IDLE state to complete ---");
        wait (dut.state == 3'd1);  // Wait for READ state
        $display("Time=%0t | Entered READ state", $time);
        
        $display("\n--- Reading counters from all channels ---");
        wait (dut.state == 3'd3);  // Wait for ILA_OUTPUT state (now 3'd3)
        $display("Time=%0t | Entered ILA_OUTPUT state", $time);
        
        $display("\n--- ILA Output Values ---");
        wait (dut.state == 3'd0);  // Wait to return to IDLE
        $display("\nTime=%0t | Returned to IDLE state - One complete cycle finished", $time);
        
        // Display summary
        $display("\n========================================");
        $display("Test completed successfully!");
        $display("Total simulation time: %0t", $time);
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
        $dumpfile("pcie_monitor_hub_tb.vcd");
        $dumpvars(0, pcie_monitor_hub_tb);
    end

endmodule
