module pcie_type_tracker_hub #(
    parameter integer NUM_UNITS = 4
)
(
    input  wire                          clk,
    input  wire                          rst,

    // Interface to counter units
    output reg  [NUM_UNITS-1:0]          read_enable,
    output wire [$clog2(NUM_UNITS)-1:0]  channel_select,
    input  wire [2:0]                    counter_id,
    input  wire [7:0]                    requester_type,

    // ILA interface
    output reg  [$clog2(NUM_UNITS)-1:0]  ila_channel_id,
    output reg  [7:0]                    ila_counter_value
);

    localparam CHANNEL_WIDTH = $clog2(NUM_UNITS);
    
    // Timing: 2^31 cycles @ 250MHz = ~8.59 seconds (rounded down from 10s for power-of-2)
    localparam INTERVAL_BITS = 31;
    localparam [INTERVAL_BITS-1:0] INTERVAL_COUNT = {INTERVAL_BITS{1'b1}}; // 2^31 - 1

    // State machine
    localparam STATE_IDLE       = 3'd0;
    localparam STATE_READ       = 3'd1;
    localparam STATE_ILA_OUTPUT = 3'd2;

    reg [2:0] state;
    reg [INTERVAL_BITS-1:0] interval_counter;
    reg [CHANNEL_WIDTH-1:0] current_channel;
    reg [4:0] counter_index;      // 0-15 for 16 counters per unit
    reg       cycle_count;         // Toggle for 2-cycle persistence
    reg [5:0] storage_addr;        // Address for storage (4 units * 16 counters = 64)
    reg [5:0] output_addr;         // Address for ILA output

    // Storage for collected counter values
    (* ram_style = "distributed" *) reg [7:0] counter_storage [63:0];

    assign channel_select = current_channel;

    // Main state machine
    always @(posedge clk) begin
        if (!rst) begin
            state <= STATE_IDLE;
            interval_counter <= 0;
            current_channel <= 0;
            counter_index <= 0;
            cycle_count <= 0;
            storage_addr <= 0;
            output_addr <= 0;
            read_enable <= 0;
            ila_channel_id <= 0;
            ila_counter_value <= 8'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    ila_channel_id <= 0; // Keep channel_id at 0 during idle
                    read_enable <= 0;
                    
                    // Count to interval
                    if (interval_counter == INTERVAL_COUNT) begin
                        interval_counter <= 0;
                        state <= STATE_READ;
                        current_channel <= 0;
                        counter_index <= 0;
                        cycle_count <= 0;
                        storage_addr <= 0;
                        // Enable read for first channel
                        read_enable[0] <= 1'b1;
                    end else begin
                        interval_counter <= interval_counter + 1'b1;
                    end
                end

                STATE_READ: begin
                    // Hold read_enable for current channel
                    read_enable <= ({{NUM_UNITS-1{1'b0}}, 1'b1} << current_channel);
                    
                    // Toggle cycle count every cycle
                    cycle_count <= ~cycle_count;
                    
                    // Every 2 cycles, move to next counter or channel
                    if (cycle_count) begin
                        // Store the counter value
                        counter_storage[storage_addr] <= requester_type;
                        storage_addr <= storage_addr + 1'b1;
                        
                        if (counter_index == 5'd15) begin
                            // Finished reading all 16 counters for this channel
                            counter_index <= 0;
                            
                            if (current_channel == NUM_UNITS - 1) begin
                                // Finished all channels, move to ILA output
                                state <= STATE_ILA_OUTPUT;
                                current_channel <= 0;
                                output_addr <= 0;
                                read_enable <= 0;
                            end else begin
                                // Move to next channel
                                current_channel <= current_channel + 1'b1;
                            end
                        end else begin
                            // Move to next counter in same channel
                            counter_index <= counter_index + 1'b1;
                        end
                    end
                end

                STATE_ILA_OUTPUT: begin
                    read_enable <= 0;
                    
                    // Toggle cycle count every cycle
                    cycle_count <= ~cycle_count;
                    
                    // Output channel_id (1-based)
                    ila_channel_id <= output_addr[5:4] + 1'b1; // Upper bits = channel (1-based)
                    
                    // Output counter value
                    ila_counter_value <= counter_storage[output_addr];
                    
                    // Every 2 cycles, move to next value
                    if (cycle_count) begin
                        if (output_addr == (NUM_UNITS * 16 - 1)) begin
                            // Finished outputting all values
                            state <= STATE_IDLE;
                            interval_counter <= 0;
                            ila_channel_id <= 0; // Return to idle (0)
                        end else begin
                            output_addr <= output_addr + 1'b1;
                        end
                    end
                end

                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
