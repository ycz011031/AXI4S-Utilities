module pcie_type_tracker_hub #(
    parameter integer NUM_UNITS = 4,
    parameter integer INTERVAL_BITS = 31,
    parameter integer COUNTER_VALUE_WIDTH = 8  // 8 for uint8, 16 for uint16
)
(
    input  wire                          clk,
    input  wire                          rst,

    // Interface to counter units
    output reg  [NUM_UNITS-1:0]          read_enable,
    output wire [$clog2(NUM_UNITS)-1:0]  channel_select,
    input  wire [2:0]                    counter_id,
    input  wire [COUNTER_VALUE_WIDTH-1:0] requester_type,

    // ILA interface
    output reg  [$clog2(NUM_UNITS+1)-1:0]  ila_channel_id,
    output reg  [COUNTER_VALUE_WIDTH-1:0] ila_counter_value,
    output reg                           ila_counter_valid
);

    localparam CHANNEL_WIDTH = $clog2(NUM_UNITS);
    localparam NUM_COUNTERS_PER_UNIT = 16;  // Fixed: 16 counters per unit (req_type is 4 bits)
    localparam COUNTER_WIDTH = 4;  // $clog2(16) = 4
    localparam STORAGE_DEPTH = NUM_UNITS * NUM_COUNTERS_PER_UNIT;
    localparam STORAGE_ADDR_WIDTH = $clog2(STORAGE_DEPTH);
    
    // Timing: 2^31 cycles @ 250MHz = ~8.59 seconds (rounded down from 10s for power-of-2)
    localparam [INTERVAL_BITS-1:0] INTERVAL_COUNT = {INTERVAL_BITS{1'b1}}; // 2^31 - 1

    // State machine
    localparam STATE_IDLE       = 3'd0;
    localparam STATE_READ       = 3'd1;
    localparam STATE_DELAY      = 3'd2;
    localparam STATE_ILA_OUTPUT = 3'd3;

    reg [2:0] state;
    reg [INTERVAL_BITS-1:0] interval_counter;
    reg [CHANNEL_WIDTH-1:0] current_channel;
    reg [COUNTER_WIDTH-1:0] counter_index;
    reg       cycle_count;         // Toggle for 2-cycle persistence
    reg [1:0] delay_counter;       // 2-cycle delay counter
    reg [1:0] pulse_counter;       // 2-cycle read_enable pulse counter
    reg       startup_delay;       // Wait 2 cycles for counter unit to start outputting data
    reg [1:0] valid_counter;       // Counter for valid pulse (2 cycles valid, 2 cycles idle)
    reg [STORAGE_ADDR_WIDTH-1:0] storage_addr;
    reg [STORAGE_ADDR_WIDTH-1:0] output_addr;

    // Storage for collected counter values
    (* ram_style = "distributed" *) reg [COUNTER_VALUE_WIDTH-1:0] counter_storage [STORAGE_DEPTH-1:0];

    assign channel_select = current_channel;

    // Main state machine
    always @(posedge clk) begin
        if (!rst) begin
            state <= STATE_IDLE;
            interval_counter <= 0;
            current_channel <= 0;
            counter_index <= 0;
            cycle_count <= 0;
            delay_counter <= 0;
            pulse_counter <= 0;
            valid_counter <= 0;
            startup_delay <= 0;
            storage_addr <= 0;
            output_addr <= 0;
            read_enable <= 0;
            ila_channel_id <= 0;
            ila_counter_value <= {COUNTER_VALUE_WIDTH{1'b0}};
            ila_counter_valid <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    ila_channel_id <= 0; // Keep channel_id at 0 during idle
                    ila_counter_valid <= 1'b0;
                    read_enable <= 0;
                    
                    // Count to interval
                    if (interval_counter == INTERVAL_COUNT) begin
                        interval_counter <= 0;
                        state <= STATE_READ;
                        current_channel <= 0;
                        counter_index <= 0;
                        cycle_count <= 0;
                        storage_addr <= 0;
                        pulse_counter <= 0;
                        startup_delay <= 1;  // Wait for counter unit to start
                        // Enable read for first channel (2-cycle pulse)
                        read_enable[0] <= 1'b1;
                    end else begin
                        interval_counter <= interval_counter + 1'b1;
                    end
                end

                STATE_READ: begin
                    // Pulse read_enable for 2 cycles at start of unit read
                    if (pulse_counter < 2'd2) begin
                        pulse_counter <= pulse_counter + 1'b1;
                    end else begin
                        read_enable <= 0;
                    end
                    
                    // Toggle cycle count every cycle
                    cycle_count <= ~cycle_count;
                    
                    // Clear startup delay after first toggle
                    if (cycle_count && startup_delay) begin
                        startup_delay <= 0;
                    end
                    
                    // Every 2 cycles, move to next counter or channel (skip first sample during startup)
                    if (cycle_count && !startup_delay) begin
                        // Store the counter value
                        counter_storage[storage_addr] <= requester_type;
                        storage_addr <= storage_addr + 1'b1;
                        
                        if (counter_index == NUM_COUNTERS_PER_UNIT - 1) begin
                            // Finished reading all counters for this channel
                            counter_index <= 0;
                            
                            if (current_channel == NUM_UNITS - 1) begin
                                // Finished all channels, move to ILA output
                                state <= STATE_ILA_OUTPUT;
                                current_channel <= 0;
                                output_addr <= 0;
                                read_enable <= 0;
                                valid_counter <= 0;
                                // Pre-load first output values
                                ila_channel_id <= 1;  // First channel
                                ila_counter_value <= counter_storage[0];
                                ila_counter_valid <= 1'b1;  // Start with valid high
                            end else begin
                                // Move to delay state before next channel
                                state <= STATE_DELAY;
                                read_enable <= 0;
                                delay_counter <= 0;
                            end
                        end else begin
                            // Move to next counter in same channel
                            counter_index <= counter_index + 1'b1;
                        end
                    end
                end

                STATE_DELAY: begin
                    read_enable <= 0;
                    delay_counter <= delay_counter + 1'b1;
                    
                    // After 2 cycles, move to next channel
                    if (delay_counter == 2'd1) begin
                        state <= STATE_READ;
                        current_channel <= current_channel + 1'b1;
                        pulse_counter <= 0;
                        startup_delay <= 1;  // Wait for next counter unit to start
                        // Enable read for next channel (2-cycle pulse)
                        read_enable <= ({{NUM_UNITS-1{1'b0}}, 1'b1} << (current_channel + 1'b1));
                    end
                end

                STATE_ILA_OUTPUT: begin
                    read_enable <= 0;
                    
                    // Valid signal pattern: 2 cycles high, 2 cycles low
                    if (valid_counter < 2'd2) begin
                        ila_counter_valid <= 1'b1;
                    end else begin
                        ila_counter_valid <= 1'b0;
                    end
                    
                    // Set outputs based on current address (stable during valid cycles)
                    ila_channel_id <= (output_addr >> COUNTER_WIDTH) + 1'b1;
                    ila_counter_value <= counter_storage[output_addr];
                    
                    // Increment valid counter
                    valid_counter <= valid_counter + 1'b1;
                    
                    // Every 4 cycles (2 valid + 2 idle), move to next value
                    if (valid_counter == 2'd3) begin
                        valid_counter <= 0;
                        
                        if (output_addr == (STORAGE_DEPTH - 1)) begin
                            // Finished outputting all values
                            state <= STATE_IDLE;
                            interval_counter <= 0;
                            ila_channel_id <= 0; // Return to idle (0)
                            ila_counter_valid <= 1'b0;
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
