// Hardware test log 1.0: successfully able to compile (at buffer depth = 4) with out major timing violations, successfully run on hardware and was able to detect downstream pcie devices
// Future work: add xilinx fifo generator integration or BRAM inference for buffer storage to reduce timing violations at higher buffer depths
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

module axi4_straddle_convertor #
(
    parameter integer AXI_TUSER_L        = 161, //161 for RC, other defaults to CQ interface
    parameter integer BUFFER_SIZE        = 4
)
(
    // Global Signals
    input wire                                  ACLK,
    input wire                                  ARESETN,

    // Slave AXI Interface
    input wire [AXI_TUSER_L-1:0]               S_AXIS_TUSER,
    input wire [511:0]                         S_AXIS_TDATA,
    input wire [15:0]                          S_AXIS_TKEEP,
    input wire                                 S_AXIS_TLAST,
    input wire                                 S_AXIS_TVALID,
    output wire                                S_AXIS_TREADY,

    // Master AXI Interface
    output wire [AXI_TUSER_L-1:0]              M_AXIS_TUSER,
    output wire [511:0]                        M_AXIS_TDATA,
    output wire [15:0]                         M_AXIS_TKEEP,
    output wire                                M_AXIS_TLAST,
    output wire                                M_AXIS_TVALID,
    input wire                                 M_AXIS_TREADY,
    
    // Error output
    output reg [1:0]                                error_invalid_state
);




reg [511:0] data0_reg [0:BUFFER_SIZE-1];
reg [15:0]  keep0_reg [0:BUFFER_SIZE-1];

reg [511:0] data1_reg [0:BUFFER_SIZE-1];
reg [15:0]  keep1_reg [0:BUFFER_SIZE-1];

// First_BE and Last_BE registers (only used when AXI_TUSER_L != 161)
generate
    if (AXI_TUSER_L != 161) begin : gen_be_regs
        reg [3:0]  first_be0_reg [0:BUFFER_SIZE-1];
        reg [3:0]  last_be0_reg [0:BUFFER_SIZE-1];
        reg [3:0]  first_be1_reg [0:BUFFER_SIZE-1];
        reg [3:0]  last_be1_reg [0:BUFFER_SIZE-1];
    end
endgenerate

// Write and read pointers for each data_reg
reg [$clog2(BUFFER_SIZE)-1:0] write_ptr0, read_ptr0;
reg [$clog2(BUFFER_SIZE)-1:0] write_ptr1, read_ptr1;

// Track which byte lanes are valid at current write position
// [0] = lane 00 valid, [1] = lane 10 valid
reg [1:0] buffer0_lanes_valid;
reg [1:0] buffer1_lanes_valid;

reg illegal_sop_encountered;
reg illegal_eop_encountered;

reg buffer1_full, buffer1_empty;
reg buffer0_full, buffer0_empty;
reg [1:0] tlp_active;
reg [1:0] tlp_active_next;
reg [1:0] tlp_active_current;

reg old_tlp;
reg old_tlp_current;

// EOP bit vectors to track TLP boundaries in each buffer
reg [BUFFER_SIZE-1:0] buffer0_eop;
reg [BUFFER_SIZE-1:0] buffer1_eop;
reg reading_from_buffer0;  // Tracks which buffer we're currently reading from

// Track start of packet for output
reg output_is_sop;  // Next output beat is SOP

// Byte lane tracker: tracks which byte lane each TLP occupies
// [1:0] - byte lanes for TLP0 (bit 0 = lane 00, bit 1 = lane 10)
// [3:2] - byte lanes for TLP1 (bit 2 = lane 00, bit 3 = lane 10)
reg [3:0] byte_lane_tracker;
reg [3:0] byte_lane_tracker_next;
reg [3:0] byte_lane_tracker_current;

// Delayed EOP signals for one-cycle delay in processing
reg [3:0] is_eop_delayed;

// Masked TKEEP signals for EOP handling
reg [15:0] keep0_masked;
reg [15:0] keep1_masked;
reg [15:0] mask0, mask1;

wire [3:0]  is_sop;
wire [3:0]  is_eop;
wire [1:0]  is_sop0_ptr, is_sop1_ptr;
wire [3:0]  is_eop0_ptr, is_eop1_ptr;
wire discontinue;

wire [73:0] first_be0, first_be1, last_be0, last_be1;

// Decoding TUSER to find SOP and EOP
// TODO support cq AXI_STREAM_TUSER format
assign first_be0 = S_AXIS_TUSER[3:0];
assign first_be1 = S_AXIS_TUSER[7:4];
assign last_be0  = S_AXIS_TUSER[11:8];
assign last_be1  = S_AXIS_TUSER[15:12];
assign is_sop = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[67:64] : S_AXIS_TUSER[81:80];
assign is_eop = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[79:76] : S_AXIS_TUSER[87:86];
assign is_sop0_ptr = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[69:68] : S_AXIS_TUSER[83:82];
assign is_sop1_ptr = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[71:70] : S_AXIS_TUSER[85:84];
assign is_eop0_ptr = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[83:80] : S_AXIS_TUSER[91:88];
assign is_eop1_ptr = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[87:84] : S_AXIS_TUSER[95:92];
assign discontinue = (AXI_TUSER_L == 161) ? S_AXIS_TUSER[96]    : S_AXIS_TUSER[96];

assign S_AXIS_TREADY = !buffer0_full && !buffer1_full;

// Calculate next write pointer if current position gets filled
wire [$clog2(BUFFER_SIZE)-1:0] next_write_ptr0 = (write_ptr0 == BUFFER_SIZE - 1) ? 0 : write_ptr0 + 1;
wire [$clog2(BUFFER_SIZE)-1:0] next_write_ptr1 = (write_ptr1 == BUFFER_SIZE - 1) ? 0 : write_ptr1 + 1;
wire [$clog2(BUFFER_SIZE)-1:0] next_next_write_ptr0 = (next_write_ptr0 == BUFFER_SIZE - 1) ? 0 : next_write_ptr0 + 1;
wire [$clog2(BUFFER_SIZE)-1:0] next_next_write_ptr1 = (next_write_ptr1 == BUFFER_SIZE - 1) ? 0 : next_write_ptr1 + 1;

// Check if buffers would be full after incrementing (almost full detection)
wire buffer0_would_be_full = (next_write_ptr0 == read_ptr0);
wire buffer1_would_be_full = (next_write_ptr1 == read_ptr1);
// Check if we have room for write_ptr+1 (needed when buffer0_lane0_filled and writing to next position)
wire buffer0_next_would_be_full = (next_next_write_ptr0 == read_ptr0);
wire buffer1_next_would_be_full = (next_next_write_ptr1 == read_ptr1);

reg buffer0_lane0_filled;
reg buffer1_lane0_filled;

// Output from buffers - alternate based on EOP markers
wire buffer0_has_data = (write_ptr0 != read_ptr0);
wire buffer1_has_data = (write_ptr1 != read_ptr1);

// Use reading_from_buffer0 as tiebreaker when both buffers have data
wire actual_read_from_buffer0 = buffer0_has_data && (!buffer1_has_data || reading_from_buffer0);

assign M_AXIS_TVALID = buffer0_has_data || buffer1_has_data;
assign M_AXIS_TDATA = actual_read_from_buffer0 ? data0_reg[read_ptr0] : data1_reg[read_ptr1];
assign M_AXIS_TKEEP = actual_read_from_buffer0 ? keep0_reg[read_ptr0] : keep1_reg[read_ptr1];
assign M_AXIS_TLAST = actual_read_from_buffer0 ? buffer0_eop[read_ptr0] : buffer1_eop[read_ptr1];

// Generate output TUSER fields
reg [3:0] output_eop_ptr;
reg [AXI_TUSER_L-1:0] output_tuser;
reg [3:0] output_first_be, output_last_be;

always @(*) begin
    // Calculate EOP pointer from TKEEP (find last valid byte)
    output_eop_ptr = 4'b0000;
    if (M_AXIS_TKEEP[15]) output_eop_ptr = 4'd15;
    else if (M_AXIS_TKEEP[14]) output_eop_ptr = 4'd14;
    else if (M_AXIS_TKEEP[13]) output_eop_ptr = 4'd13;
    else if (M_AXIS_TKEEP[12]) output_eop_ptr = 4'd12;
    else if (M_AXIS_TKEEP[11]) output_eop_ptr = 4'd11;
    else if (M_AXIS_TKEEP[10]) output_eop_ptr = 4'd10;
    else if (M_AXIS_TKEEP[9]) output_eop_ptr = 4'd9;
    else if (M_AXIS_TKEEP[8]) output_eop_ptr = 4'd8;
    else if (M_AXIS_TKEEP[7]) output_eop_ptr = 4'd7;
    else if (M_AXIS_TKEEP[6]) output_eop_ptr = 4'd6;
    else if (M_AXIS_TKEEP[5]) output_eop_ptr = 4'd5;
    else if (M_AXIS_TKEEP[4]) output_eop_ptr = 4'd4;
    else if (M_AXIS_TKEEP[3]) output_eop_ptr = 4'd3;
    else if (M_AXIS_TKEEP[2]) output_eop_ptr = 4'd2;
    else if (M_AXIS_TKEEP[1]) output_eop_ptr = 4'd1;
    else output_eop_ptr = 4'd0;
    
    // Extract first_be and last_be from appropriate buffer
    if (AXI_TUSER_L != 161) begin
        if (actual_read_from_buffer0) begin
            output_first_be = gen_be_regs.first_be0_reg[read_ptr0];
            output_last_be = gen_be_regs.last_be0_reg[read_ptr0];
        end else begin
            output_first_be = gen_be_regs.first_be1_reg[read_ptr1];
            output_last_be = gen_be_regs.last_be1_reg[read_ptr1];
        end
    end else begin
        output_first_be = 4'h0;
        output_last_be = 4'h0;
    end
    
    // Build output TUSER
    if (AXI_TUSER_L == 161) begin
        output_tuser = {161{1'b0}};
        output_tuser[64] = output_is_sop;  // is_sop[0]
        output_tuser[76] = M_AXIS_TLAST;   // is_eop[0]
        output_tuser[83:80] = output_eop_ptr;  // eop0_ptr
    end else begin
        output_tuser = {AXI_TUSER_L{1'b0}};
        // Only populate first_be and last_be at SOP
        if (output_is_sop) begin
            output_tuser[3:0] = output_first_be;   // first_be0
            output_tuser[11:8] = output_last_be;   // last_be0
        end
        output_tuser[80] = output_is_sop;      // is_sop[0]
        output_tuser[86] = M_AXIS_TLAST;       // is_eop[0]
        output_tuser[91:88] = output_eop_ptr;  // eop0_ptr
    end
end

assign M_AXIS_TUSER = output_tuser&{AXI_TUSER_L{M_AXIS_TVALID}};


// Combinational logic for TKEEP masking based on EOP
always @(*) begin
    // Default: pass through TKEEP unchanged
    keep0_masked = S_AXIS_TKEEP;
    keep1_masked = S_AXIS_TKEEP;
    mask0 = S_AXIS_TKEEP & ((16'h1 << (is_eop0_ptr + 1)) - 1);
    mask1 = S_AXIS_TKEEP & ((16'h1 << (is_eop1_ptr + 1)) - 1);
    
    
    if (S_AXIS_TVALID && S_AXIS_TREADY) begin
        // Apply EOP mask for TLP0 if it's ending
        if (tlp_active_current[0] && !tlp_active_next[0]) begin
            // when both TLP is ending, apply mask0 when TLP0 is old TLP
            keep0_masked = (is_eop[0]&&(!old_tlp_current)) ? mask0 : mask1;
        end
        
        // Apply EOP mask for TLP1 if it's ending
        if (tlp_active_current[1] && !tlp_active_next[1]) begin
            // when both TLP is ending, apply mask1 when TLP1 is old TLP 
            keep1_masked = (is_eop[1]&&old_tlp_current) ? mask0 : mask1;
        end
    end
end

// Combinational logic for next state of tlp_active and byte_lane_tracker
always @(*) begin
    tlp_active_next            = tlp_active;
    tlp_active_current         = tlp_active;
    byte_lane_tracker_current  = byte_lane_tracker;
    byte_lane_tracker_next     = byte_lane_tracker;
    old_tlp_current            = old_tlp;
    illegal_sop_encountered    = 1'b0;
    illegal_eop_encountered    = 1'b0;
    
    if (S_AXIS_TVALID && S_AXIS_TREADY) begin
        // Handle SOP (start of packet) - left shift and fill with 1
        casez ({is_sop[1:0],tlp_active})
            4'b11??:begin
                tlp_active_current        = 2'b11;
                byte_lane_tracker_current = 4'b1001;
                old_tlp_current           = 1'b0;
            end
            4'b0100:begin
                tlp_active_current        = 2'b01; // TLP0 starting
                byte_lane_tracker_current = (is_sop0_ptr == 2'b00) ? 4'b0011 : 4'b0010;
                old_tlp_current           = 1'b0;
            end
            4'b0101:begin
                tlp_active_current        = 2'b11; // TLP1 starting
                byte_lane_tracker_current = (is_sop0_ptr == 2'b00) ? 4'b0110 : 4'b1001;
                old_tlp_current           = 1'b0;
            end
            4'b0110:begin
                tlp_active_current        = 2'b11; // TLP0 starting
                byte_lane_tracker_current = (is_sop0_ptr == 2'b00) ? 4'b1001 : 4'b0110;
                old_tlp_current           = 1'b1;
            end
            4'b00??:begin
                tlp_active_current        = tlp_active;
                byte_lane_tracker_current = byte_lane_tracker;
                old_tlp_current           = old_tlp;
            end
            default:begin
                tlp_active_current        = tlp_active;
                byte_lane_tracker_current = byte_lane_tracker;
                old_tlp_current           = old_tlp;
                illegal_sop_encountered   = 1'b1;
            end
        endcase
        casez ({is_eop[1:0],tlp_active_current})
            4'b1111:begin
                tlp_active_next           = 2'b00;
                byte_lane_tracker_next    = 4'b0000;
            end
            4'b0111:begin
                tlp_active_next           = (!old_tlp_current)? 2'b10 : 2'b01; // TLP0 ending
                byte_lane_tracker_next    = (!old_tlp_current)? 4'b1100 : 4'b0011;
            end
            4'b0101:begin
                tlp_active_next           = 2'b00; // TLP0 ending
                byte_lane_tracker_next    = 4'b0000; //all bytelane released
            end
            4'b0110:begin
                tlp_active_next           = 2'b00; // TLP1 ending
                byte_lane_tracker_next    = 4'b0000;
            end
            4'b00??:begin
                tlp_active_next           = tlp_active_current;
                byte_lane_tracker_next    = byte_lane_tracker_current;
            end
            default:begin
                tlp_active_next           = tlp_active_current;
                byte_lane_tracker_next    = byte_lane_tracker_current;
                illegal_eop_encountered   = 1'b1;
            end
        endcase 
    end
end

// Sequential logic
always @(posedge ACLK) begin
    if (!ARESETN) begin
        write_ptr0 <= 0;
        read_ptr0  <= 0;
        write_ptr1 <= 0;
        read_ptr1  <= 0;
        
        buffer0_lanes_valid <= 2'b00;
        buffer1_lanes_valid <= 2'b00;
        
        tlp_active        <= 2'b00;
        byte_lane_tracker <= 4'b0000;
        is_eop_delayed    <= 4'b0000;
        buffer0_full      <= 0;
        buffer0_empty     <= 1;
        buffer1_full      <= 0;
        buffer1_empty     <= 1;
        old_tlp           <= 0;
        
        buffer0_eop <= {BUFFER_SIZE{1'b0}};
        buffer1_eop <= {BUFFER_SIZE{1'b0}};
       
        buffer0_lane0_filled <= 0;
        buffer1_lane0_filled <= 0;

        reading_from_buffer0 <= 1;  // Start with buffer0
        output_is_sop <= 1;  // First output is SOP
        error_invalid_state  <= 2'b00;

    end else begin
        is_eop_delayed <= (S_AXIS_TVALID)? is_eop : 4'b1111;
        old_tlp        <= old_tlp_current;
        
        // Update TLP active state and byte lane tracker
        tlp_active        <= S_AXIS_TREADY? tlp_active_next : tlp_active_current;
        byte_lane_tracker <= S_AXIS_TREADY? byte_lane_tracker_next : byte_lane_tracker_current;
        

        if (S_AXIS_TVALID && S_AXIS_TREADY) begin
            // Error: is_sop[1] should not be high when any TLP is already active
            error_invalid_state <= error_invalid_state |{illegal_sop_encountered, illegal_eop_encountered};
        end
        
        // Handle S_AXIS transaction - buffer writes
        if (S_AXIS_TVALID && S_AXIS_TREADY) begin
            // Sample first_be and last_be at SOP for non-161 TUSER
            // first_be0/last_be0 correspond to is_sop[0] with is_sop0_ptr
            // first_be1/last_be1 correspond to is_sop[1] with is_sop1_ptr
            if (AXI_TUSER_L != 161) begin
                if (is_sop[0]) begin
                    // TLP0 starting - sample BE values based on is_sop0_ptr
                    if (!buffer0_lane0_filled) begin
                        gen_be_regs.first_be0_reg[write_ptr0] <= (is_sop0_ptr == 2'b00) ? first_be0 : first_be1;
                        gen_be_regs.last_be0_reg[write_ptr0] <= (is_sop0_ptr == 2'b00) ? last_be0 : last_be1;
                    end else begin
                        gen_be_regs.first_be0_reg[write_ptr0+1] <= (is_sop0_ptr == 2'b00) ? first_be0 : first_be1;
                        gen_be_regs.last_be0_reg[write_ptr0+1] <= (is_sop0_ptr == 2'b00) ? last_be0 : last_be1;
                    end
                end
                if (is_sop[1]) begin
                    // TLP1 starting - sample BE values based on is_sop1_ptr
                    if (!buffer1_lane0_filled) begin
                        gen_be_regs.first_be1_reg[write_ptr1] <= (is_sop1_ptr == 2'b00) ? first_be0 : first_be1;
                        gen_be_regs.last_be1_reg[write_ptr1] <= (is_sop1_ptr == 2'b00) ? last_be0 : last_be1;
                    end else begin
                        gen_be_regs.first_be1_reg[write_ptr1+1] <= (is_sop1_ptr == 2'b00) ? first_be0 : first_be1;
                        gen_be_regs.last_be1_reg[write_ptr1+1] <= (is_sop1_ptr == 2'b00) ? last_be0 : last_be1;
                    end
                end
            end
            
            // Write to buffer0 if TLP0 is active
            if (tlp_active_current[0]) begin
                // Extract correct byte lanes based on byte_lane_tracker[1:0]
                case (byte_lane_tracker_current[1:0])
                    2'b01: begin // Only lane 00 active
                        if (!buffer0_lane0_filled) begin
                            buffer0_lane0_filled <= 1;
                            data0_reg[write_ptr0][255:0] <= S_AXIS_TDATA[255:0];
                            keep0_reg[write_ptr0][7:0] <= keep0_masked[7:0];
                        end else begin
                            buffer0_lane0_filled <= 0;
                            data0_reg[write_ptr0][511:256] <= S_AXIS_TDATA[255:0];
                            keep0_reg[write_ptr0][15:8] <= keep0_masked[7:0];
                        end
                    end
                    2'b10: begin // Only lane 10 active
                        if (!buffer0_lane0_filled) begin
                            buffer0_lane0_filled <= 1;
                            data0_reg[write_ptr0][255:0] <= S_AXIS_TDATA[511:256];
                            keep0_reg[write_ptr0][7:0] <= keep0_masked[15:8];
                        end else begin
                            buffer0_lane0_filled <= 0;
                            data0_reg[write_ptr0][511:256] <= S_AXIS_TDATA[511:256];
                            keep0_reg[write_ptr0][15:8] <= keep0_masked[15:8];
                        end
                    end
                    2'b11: begin // Both lanes active
                        if (!buffer0_lane0_filled) begin
                            // Write lane 00 to current position, lane 10 to upper half
                            buffer0_lane0_filled <= 0;
                            data0_reg[write_ptr0][255:0] <= S_AXIS_TDATA[255:0];
                            data0_reg[write_ptr0][511:256] <= S_AXIS_TDATA[511:256];
                            keep0_reg[write_ptr0][7:0] <= keep0_masked[7:0];
                            keep0_reg[write_ptr0][15:8] <= keep0_masked[15:8];
                        end else begin
                            // Lane 00 already filled, write lane 00 to upper half, lane 10 to next position
                            buffer0_lane0_filled <= 1;
                            data0_reg[write_ptr0][511:256] <= S_AXIS_TDATA[255:0];
                            data0_reg[write_ptr0+1][255:0] <= S_AXIS_TDATA[511:256];
                            keep0_reg[write_ptr0][15:8] <= keep0_masked[7:0];
                            keep0_reg[write_ptr0+1][7:0] <= keep0_masked[15:8];
                        end
                    end
                    default: begin // 2'b00 - no lanes active, shouldn't happen
                        buffer0_lane0_filled <= buffer0_lane0_filled;
                    end
                endcase
                
                // Mark EOP bit if TLP0 is ending - check if writing to write_ptr0 or write_ptr0+1
                if (tlp_active_current[0] && !tlp_active_next[0]) begin
                    // Determine where EOP should be marked based on where data was written
                    if (byte_lane_tracker_current[1] && buffer0_lane0_filled && byte_lane_tracker_current[0]) begin
                        // Writing to write_ptr0+1 (lane 10 data goes to next position's lane 00)
                        buffer0_eop[write_ptr0+1] <= 1'b1;
                    end else begin
                        // Writing to write_ptr0
                        buffer0_eop[write_ptr0] <= 1'b1;
                    end
                end else begin
                    buffer0_eop[write_ptr0] <= 1'b0;
                end
                
                // Increment write pointer when both lanes filled or EOP detected
                if (tlp_active_current[0] && !tlp_active_next[0]) begin
                    // EOP detected - always increment and clear lane0_filled
                    write_ptr0 <= next_write_ptr0;
                    buffer0_lane0_filled <= 0;
                end else begin
                    casez({buffer0_lane0_filled, byte_lane_tracker_current[1:0]})
                        3'b11?: write_ptr0 <= next_write_ptr0;
                        3'b1?1: write_ptr0 <= next_write_ptr0;
                        3'b?11: write_ptr0 <= next_write_ptr0;
                        default: write_ptr0 <= write_ptr0;
                    endcase
                end
            end
            // Write to buffer1 if TLP1 is active
            if (tlp_active_current[1]) begin
                // Extract correct byte lanes based on byte_lane_tracker[3:2]
                case (byte_lane_tracker_current[3:2])
                    2'b01: begin // Only lane 00 active
                        if (!buffer1_lane0_filled) begin
                            buffer1_lane0_filled <= 1;
                            data1_reg[write_ptr1][255:0] <= S_AXIS_TDATA[255:0];
                            keep1_reg[write_ptr1][7:0] <= keep1_masked[7:0];
                        end else begin
                            buffer1_lane0_filled <= 0;
                            data1_reg[write_ptr1][511:256] <= S_AXIS_TDATA[255:0];
                            keep1_reg[write_ptr1][15:8] <= keep1_masked[7:0];
                        end
                    end
                    2'b10: begin // Only lane 10 active
                        if (!buffer1_lane0_filled) begin
                            buffer1_lane0_filled <= 1;
                            data1_reg[write_ptr1][255:0] <= S_AXIS_TDATA[511:256];
                            keep1_reg[write_ptr1][7:0] <= keep1_masked[15:8];
                        end else begin
                            buffer1_lane0_filled <= 0;
                            data1_reg[write_ptr1][511:256] <= S_AXIS_TDATA[511:256];
                            keep1_reg[write_ptr1][15:8] <= keep1_masked[15:8];
                        end
                    end
                    2'b11: begin // Both lanes active
                        if (!buffer1_lane0_filled) begin
                            // Write lane 00 to current position, lane 10 to upper half
                            buffer1_lane0_filled <= 0;
                            data1_reg[write_ptr1][255:0] <= S_AXIS_TDATA[255:0];
                            data1_reg[write_ptr1][511:256] <= S_AXIS_TDATA[511:256];
                            keep1_reg[write_ptr1][7:0] <= keep1_masked[7:0];
                            keep1_reg[write_ptr1][15:8] <= keep1_masked[15:8];
                        end else begin
                            // Lane 00 already filled, write lane 00 to upper half, lane 10 to next position
                            buffer1_lane0_filled <= 1;
                            data1_reg[write_ptr1][511:256] <= S_AXIS_TDATA[255:0];
                            data1_reg[write_ptr1+1][255:0] <= S_AXIS_TDATA[511:256];
                            keep1_reg[write_ptr1][15:8] <= keep1_masked[7:0];
                            keep1_reg[write_ptr1+1][7:0] <= keep1_masked[15:8];
                        end
                    end
                    default: begin // 2'b00 - no lanes active, shouldn't happen
                        buffer1_lane0_filled <= buffer1_lane0_filled;
                    end
                endcase
                
                // Mark EOP bit if TLP1 is ending - check if writing to write_ptr1 or write_ptr1+1
                if (tlp_active_current[1] && !tlp_active_next[1]) begin
                    // Determine where EOP should be marked based on where data was written
                    if (byte_lane_tracker_current[3] && buffer1_lane0_filled && byte_lane_tracker_current[2]) begin
                        // Writing to write_ptr1+1 (lane 10 data goes to next position's lane 00)
                        buffer1_eop[write_ptr1+1] <= 1'b1;
                    end else begin
                        // Writing to write_ptr1
                        buffer1_eop[write_ptr1] <= 1'b1;
                    end
                end else begin
                    buffer1_eop[write_ptr1] <= 1'b0;
                end
                
                // Increment write pointer when both lanes filled or EOP detected
                if (tlp_active_current[1] && !tlp_active_next[1]) begin
                    // EOP detected - always increment and clear lane0_filled
                    write_ptr1 <= next_write_ptr1;
                    buffer1_lane0_filled <= 0;
                end else begin
                    casez({buffer1_lane0_filled, byte_lane_tracker_current[3:2]})
                        3'b11?: write_ptr1 <= next_write_ptr1;
                        3'b1?1: write_ptr1 <= next_write_ptr1;
                        3'b?11: write_ptr1 <= next_write_ptr1;
                        default: write_ptr1 <= write_ptr1;
                    endcase
                end
            end
        end

        // Handle M_AXIS transaction (reading from buffer) - switch on EOP
        if (M_AXIS_TVALID && M_AXIS_TREADY) begin
            if (actual_read_from_buffer0) begin
                read_ptr0 <= read_ptr0 + 1;
                // Update reading_from_buffer0 only at EOP and when both buffers have data
                if (buffer0_eop[read_ptr0] && buffer1_has_data) begin
                    reading_from_buffer0 <= 0;
                end
                // Set SOP for next beat if current beat is EOP
                if (buffer0_eop[read_ptr0]) begin
                    output_is_sop <= 1;
                end else begin
                    output_is_sop <= 0;
                end
            end else begin
                read_ptr1 <= read_ptr1 + 1;
                // Update reading_from_buffer0 only at EOP and when both buffers have data
                if (buffer1_eop[read_ptr1] && buffer0_has_data) begin
                    reading_from_buffer0 <= 1;
                end
                // Set SOP for next beat if current beat is EOP
                if (buffer1_eop[read_ptr1]) begin
                    output_is_sop <= 1;
                end else begin
                    output_is_sop <= 0;
                end
            end
        end

        // Update buffer status
        buffer0_empty <= (write_ptr0 == read_ptr0) && (buffer0_lanes_valid == 2'b00);
        // Buffer is full if next position would equal read pointer, OR if lane0_filled and next+1 would be full
        buffer0_full  <= buffer0_would_be_full || (buffer0_lane0_filled && buffer0_next_would_be_full);
        
        buffer1_empty <= (write_ptr1 == read_ptr1) && (buffer1_lanes_valid == 2'b00);
        buffer1_full  <= buffer1_would_be_full || (buffer1_lane0_filled && buffer1_next_would_be_full);
    end
end
endmodule