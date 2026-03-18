module pcie_requester_type_counter_unit #
(
    parameter integer AXIS_DATA_WIDTH = 512,
    parameter integer AXIS_TUSER_WIDTH = 229,
    parameter integer COUNTER_VALUE_WIDTH = 8,  // 8 for uint8, 16 for uint16
    parameter integer unit_id = 1// unit_id starts from 1, zero in output indicates idle state
)
(
    input  wire                          clk,
    input  wire                          rst,

    // AXI-stream input (from PCIe CQ)
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep,
    input  wire                          s_axis_tvalid,
    input  wire                          s_axis_tlast,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser,
    output wire                          s_axis_tready,

    // AXI-stream output (transparent to user logic)
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_tkeep,
    output wire                          m_axis_tvalid,
    output wire                          m_axis_tlast,
    output wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser,
    input  wire                          m_axis_tready,

    // Tag to requeste type lookup table (to completer counter)
    input wire [7:0]                     completer_tag,
    input wire                           completer_tag_valid,
    output reg [4:0]                     completer_type,

    // Transaction type counters (to Hub)
    input  wire                          read_enable,
    output wire [2:0]                    counter_id,
    output reg  [COUNTER_VALUE_WIDTH-1:0] requester_type

);

    // ============================================================
    // Pass-through (transparent) path
    // ============================================================
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tvalid = s_axis_tvalid;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tuser  = s_axis_tuser;
    assign s_axis_tready = m_axis_tready;

    // ============================================================
    // PCIe TLP Field Extraction (requester descriptor formatting)
    // ============================================================
    wire [3:0] req_type = s_axis_tdata[78:75];
    wire [7:0] tag      = s_axis_tdata[103:96];
    wire [1:0] sop      = (AXIS_TUSER_WIDTH == 229) ? s_axis_tuser[81:80] : s_axis_tuser[21:20]; // Start of Packet indicator
    wire       is_sop   = (sop != 2'b00);      // SOP when != 0

    // ============================================================
    // Utility register arrays
    // ============================================================
    (* ram_style = "distributed" *) reg  [4:0] tag_type_table [255:0];
    (* ram_style = "distributed" *) reg  [COUNTER_VALUE_WIDTH-1:0] type_count_table [15:0];

    // Loop variables for initialization
    integer i;

    // Posted transaction detection
    wire is_posted = (req_type == 4'b0001) ||  // Memory Write
                     (req_type == 4'b1100) ||  // General Messages
                     (req_type == 4'b1101) ||  // Vendor Defined Messages
                     (req_type == 4'b1110);    // ATS Messages

    // Counter read state
    reg [3:0] counter_index;
    reg       cycle_toggle;
    reg       read_enable_prev;  // For edge detection
    reg       read_active;       // Indicates active read sequence

    assign counter_id = unit_id[2:0]; // Output the unit ID as counter ID (3 bits)

    // ============================================================
    // Transaction Type Counters
    // ============================================================
    always @(posedge clk) begin
        if (!rst) begin
            // Initialize all tag_type_table entries to 5'b01111
            for (i = 0; i < 256; i = i + 1) begin
                tag_type_table[i] <= 5'b01111;
            end
            
            // Initialize all type_count_table entries
            `ifdef SYNTHESIS
                // Synthesis: Initialize to zeros
                for (i = 0; i < 16; i = i + 1) begin
                    type_count_table[i] <= {COUNTER_VALUE_WIDTH{1'b0}};
                end
            `else
                // Simulation: Initialize with test pattern (channel in upper nibble, counter index in lower)
                for (i = 0; i < 16; i = i + 1) begin
                    type_count_table[i] <= {{COUNTER_VALUE_WIDTH-8{1'b0}}, unit_id[3:0], i[3:0]};
                end
            `endif
        end else begin
            // Count transaction types on valid transfers at SOP only
            if (s_axis_tvalid && s_axis_tready && is_sop) begin

                // Maintain tag to requester type mapping
                // Only write to table for non-posted transactions
                if (!is_posted) begin
                    if (tag_type_table[tag] == 5'b01111) begin
                        tag_type_table[tag] <= {1'b0, req_type}; // Store type with MSB=0 for requester
                    end else if (tag_type_table[tag] != {1'b0, req_type}) begin
                        tag_type_table[tag][4] <= 1'b1; // Set conflict bit to 1 if type mismatch
                    end
                end
                
                // Update counters based on request type
                type_count_table[req_type] <= type_count_table[req_type] + 1'b1; // Increment count if type matches
                
            end
            // Ideally we would put this as a dedicated state machine, however this would create multi driver registers in synthesis
            // In reality this wouldn't be an issue since request and completion would not arrive in the same cycle for the same tag.
            if (completer_tag_valid) begin
                // Output the table value and reset the entry
                completer_type <= tag_type_table[completer_tag];
                tag_type_table[completer_tag] <= 5'b01111;
            end
        end
    end



    // ============================================================
    // Counter Read Logic - Edge-triggered atomic read sequence
    // ============================================================
    always @(posedge clk) begin
        if (!rst) begin
            counter_index <= 4'd0;
            cycle_toggle <= 1'b0;
            read_enable_prev <= 1'b0;
            read_active <= 1'b0;
            requester_type <= {COUNTER_VALUE_WIDTH{1'b0}};
        end else begin
            // Store previous read_enable for edge detection
            read_enable_prev <= read_enable;
            
            // Detect rising edge of read_enable
            if (read_enable && !read_enable_prev) begin
                // Start new read sequence
                read_active <= 1'b1;
                counter_index <= 4'd0;
                cycle_toggle <= 1'b0;
            end
            
            // Active read sequence
            if (read_active) begin
                // Toggle every cycle
                cycle_toggle <= ~cycle_toggle;
                
                // Switch index every 2 cycles (when toggle goes from 1 to 0)
                if (cycle_toggle) begin
                    if (counter_index == 4'd15) begin
                        // Finished all 16 counters
                        read_active <= 1'b0;
                        counter_index <= 4'd0;
                    end else begin
                        counter_index <= counter_index + 1'b1;
                    end
                end
                
                // Output current counter value
                requester_type <= type_count_table[counter_index];
            end else begin
                requester_type <= {COUNTER_VALUE_WIDTH{1'b0}};
            end
        end
    end

endmodule
