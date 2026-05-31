module axi4_telemetry #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer TELEMETRY_DEPTH  = 512,  // Depth: 512 packets (increased from 256 for better characterization)
    parameter integer DATA_FIDELITY    = 8     // Max value bits for length/gap counters (default 8 = max 255)
)(
    input wire                          clk,
    input wire                          rst_n,

    // AXI4-Stream Monitor Interface (passive tap)
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep,
    input  wire                          s_axis_tvalid,
    input  wire                          s_axis_tlast,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser,
    input  wire                          s_axis_tready,

    // Telemetry Data Output (for ILA capture)
    output reg                           tel_enable,      // High during streaming window
    output reg                           tel_valid,       // High when data is valid (2-cycle pattern)
    output reg [DATA_FIDELITY-1:0]       tel_pkt_length,  // Packet length in beats
    output reg [DATA_FIDELITY-1:0]       tel_pkt_gap,     // Gap between packets in cycles
    output reg [3:0]                     tel_pkt_type,    // Request type field [78:75]
    output reg [31:0]                    tel_pkt_addr,    // MSB 32 bits of address (MWr/MRd only)
    output reg [DATA_FIDELITY-1:0]       tel_payload_dw,  // Payload size in DWORDs (MWr only)
    output reg                           tel_is_sop,      // SOP marker for current packet
    output reg                           tel_is_eop,      // EOP marker for current packet
    
    // Additional telemetry for PCIe characterization
    output reg [7:0]                     tel_pkt_tag,     // PCIe tag [103:96]
    output reg [1:0]                     tel_addr_type,   // Address type [1:0]
    output reg [DATA_FIDELITY-1:0]       tel_total_pkts,  // Total packets seen (saturating)
    output reg [DATA_FIDELITY-1:0]       tel_mwr_count,   // MWr packets in this buffer
    output reg [DATA_FIDELITY-1:0]       tel_mrd_count,   // MRd packets in this buffer
    output reg [DATA_FIDELITY-1:0]       tel_other_count  // Other packet types in this buffer
);

// =================================================================
// Local Parameters
// =================================================================
localparam MWR_TYPE = 4'b0001;
localparam MRD_TYPE = 4'b0000;

// Address width for telemetry buffer
localparam ADDR_WIDTH = $clog2(TELEMETRY_DEPTH);

// Streaming state machine
localparam [2:0] ST_COLLECT     = 3'd0;  // Collecting telemetry data
localparam [2:0] ST_WAIT_STREAM = 3'd1;  // Waiting for stream interval
localparam [2:0] ST_STREAM_V1   = 3'd2;  // Streaming cycle 1 (valid high)
localparam [2:0] ST_STREAM_V2   = 3'd3;  // Streaming cycle 2 (valid high)
localparam [2:0] ST_STREAM_I1   = 3'd4;  // Streaming cycle 3 (valid low)
localparam [2:0] ST_STREAM_I2   = 3'd5;  // Streaming cycle 4 (valid low)

// Timing constants
localparam STREAM_DURATION = 1024;  // Total streaming window (256 pkts × 4 cycles/pkt)
localparam STREAM_GAP      = 8192;  // Gap between streaming windows

// =================================================================
// Packet Field Decoding (from tdata/tuser)
// =================================================================
wire [3:0]    request_type;
wire [1:0]    address_type;
wire [61:0]   address_full;
wire [31:0]   address_msb;
wire [7:0]    tag;
wire [1:0]    sop;
wire [1:0]    eop;
wire [3:0]    eop_ptr;
wire [9:0]    dword_count;

assign request_type = s_axis_tdata[78:75];
assign address_type = s_axis_tdata[1:0];
assign address_full = s_axis_tdata[63:2];
assign address_msb  = address_full[61:30];  // Top 32 bits of the 62-bit address field
assign tag          = s_axis_tdata[103:96];
assign sop          = s_axis_tuser[81:80];
assign eop          = s_axis_tuser[87:86];
assign eop_ptr      = s_axis_tuser[91:88];
assign dword_count  = s_axis_tdata[73:64];  // DW count field for MWr/MRd

// Transaction handshake
wire beat_fire = s_axis_tvalid && s_axis_tready;
wire is_sop    = beat_fire && sop[0];
wire is_eop    = beat_fire && s_axis_tlast;

// Type checks
wire is_mwr = (request_type == MWR_TYPE);
wire is_mrd = (request_type == MRD_TYPE);

// =================================================================
// Telemetry Collection Registers
// =================================================================

// Packet tracking
reg [DATA_FIDELITY-1:0]  r_beat_count;       // Current packet beat counter
reg [DATA_FIDELITY-1:0]  r_gap_count;        // Gap counter between packets
reg                      r_in_packet;        // 1 = currently processing a packet
reg                      r_gap_active;       // 1 = counting gap after EOP

// Current packet info (captured on SOP)
reg [3:0]                r_cur_pkt_type;
reg [31:0]               r_cur_pkt_addr;
reg [7:0]                r_cur_pkt_tag;
reg [1:0]                r_cur_addr_type;
reg [9:0]                r_cur_dword_count;

// Telemetry buffer storage (ring buffer)
localparam TEL_ENTRY_WIDTH = DATA_FIDELITY * 4 + 4 + 32 + 8 + 2;  // length + gap + payload_dw + type_count + type + addr + tag + addr_type

reg [DATA_FIDELITY-1:0]  mem_pkt_length   [0:TELEMETRY_DEPTH-1];
reg [DATA_FIDELITY-1:0]  mem_pkt_gap      [0:TELEMETRY_DEPTH-1];
reg [3:0]                mem_pkt_type     [0:TELEMETRY_DEPTH-1];
reg [31:0]               mem_pkt_addr     [0:TELEMETRY_DEPTH-1];
reg [DATA_FIDELITY-1:0]  mem_payload_dw   [0:TELEMETRY_DEPTH-1];
reg [7:0]                mem_pkt_tag      [0:TELEMETRY_DEPTH-1];
reg [1:0]                mem_addr_type    [0:TELEMETRY_DEPTH-1];

// Buffer management
reg [ADDR_WIDTH-1:0]     r_write_ptr;        // Next write location
reg [ADDR_WIDTH-1:0]     r_read_ptr;         // Current read location for streaming
reg [ADDR_WIDTH-1:0]     r_valid_count;      // Number of valid entries in buffer

// Statistics
reg [DATA_FIDELITY-1:0]  r_total_pkts;       // Total packets seen (saturating)
reg [DATA_FIDELITY-1:0]  r_buf_mwr_count;    // MWr count in current buffer
reg [DATA_FIDELITY-1:0]  r_buf_mrd_count;    // MRd count in current buffer
reg [DATA_FIDELITY-1:0]  r_buf_other_count;  // Other types in current buffer

// Streaming control
reg [2:0]                r_stream_state;
reg [12:0]               r_stream_counter;   // Counts cycles within streaming/gap phases (13 bits for 8192)
reg [ADDR_WIDTH-1:0]     r_stream_pkt_cnt;   // Packet counter during streaming

// =================================================================
// Packet Collection Process
//
// Monitors AXI-Stream transactions and records:
//   - Packet length (number of beats from SOP to EOP)
//   - Gap between packets (cycles from EOP to next SOP)
//   - Packet type, address, tag, and other metadata
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_beat_count      <= {DATA_FIDELITY{1'b0}};
        r_gap_count       <= {DATA_FIDELITY{1'b0}};
        r_in_packet       <= 1'b0;
        r_gap_active      <= 1'b0;
        r_cur_pkt_type    <= 4'd0;
        r_cur_pkt_addr    <= 32'd0;
        r_cur_pkt_tag     <= 8'd0;
        r_cur_addr_type   <= 2'd0;
        r_cur_dword_count <= 10'd0;
        r_write_ptr       <= {ADDR_WIDTH{1'b0}};
        r_valid_count     <= {ADDR_WIDTH{1'b0}};
        r_total_pkts      <= {DATA_FIDELITY{1'b0}};
        r_buf_mwr_count   <= {DATA_FIDELITY{1'b0}};
        r_buf_mrd_count   <= {DATA_FIDELITY{1'b0}};
        r_buf_other_count <= {DATA_FIDELITY{1'b0}};
    end else begin

        // ----------------------------------------------------------
        // SOP: Start of Packet
        // ----------------------------------------------------------
        if (is_sop) begin
            r_in_packet    <= 1'b1;
            r_beat_count   <= 1;  // First beat
            r_gap_active   <= 1'b0;
            
            // Capture packet metadata
            r_cur_pkt_type    <= request_type;
            r_cur_pkt_tag     <= tag;
            r_cur_addr_type   <= address_type;
            r_cur_dword_count <= dword_count;
            
            // Capture address MSB for MWr/MRd only
            if (is_mwr || is_mrd)
                r_cur_pkt_addr <= address_msb;
            else
                r_cur_pkt_addr <= 32'd0;
                
        // ----------------------------------------------------------
        // Mid-packet beat
        // ----------------------------------------------------------
        end else if (r_in_packet && beat_fire && !s_axis_tlast) begin
            // Increment beat counter (saturate at max)
            if (r_beat_count < {DATA_FIDELITY{1'b1}})
                r_beat_count <= r_beat_count + 1'b1;
        end

        // ----------------------------------------------------------
        // EOP: End of Packet (can coincide with SOP for single-beat pkts)
        // ----------------------------------------------------------
        if (is_eop) begin
            // Store packet telemetry in buffer
            mem_pkt_length[r_write_ptr] <= r_beat_count;
            mem_pkt_gap[r_write_ptr]    <= r_gap_count;
            mem_pkt_type[r_write_ptr]   <= r_cur_pkt_type;
            mem_pkt_addr[r_write_ptr]   <= r_cur_pkt_addr;
            mem_pkt_tag[r_write_ptr]    <= r_cur_pkt_tag;
            mem_addr_type[r_write_ptr]  <= r_cur_addr_type;
            
            // Calculate payload in DWORDs (for MWr, use dword_count; others = 0)
            if (r_cur_pkt_type == MWR_TYPE) begin
                if (r_cur_dword_count[9:0] > {{(10-DATA_FIDELITY){1'b0}}, {DATA_FIDELITY{1'b1}}})
                    mem_payload_dw[r_write_ptr] <= {DATA_FIDELITY{1'b1}};  // Saturate
                else
                    mem_payload_dw[r_write_ptr] <= r_cur_dword_count[DATA_FIDELITY-1:0];
            end else begin
                mem_payload_dw[r_write_ptr] <= {DATA_FIDELITY{1'b0}};
            end
            
            // Advance write pointer (ring buffer)
            r_write_ptr <= r_write_ptr + 1'b1;
            
            // Update valid count (saturate at TELEMETRY_DEPTH)
            if (r_valid_count < TELEMETRY_DEPTH[ADDR_WIDTH-1:0])
                r_valid_count <= r_valid_count + 1'b1;
            
            // Update statistics
            if (r_total_pkts < {DATA_FIDELITY{1'b1}})
                r_total_pkts <= r_total_pkts + 1'b1;
                
            // Update type counters
            if (r_cur_pkt_type == MWR_TYPE) begin
                if (r_buf_mwr_count < {DATA_FIDELITY{1'b1}})
                    r_buf_mwr_count <= r_buf_mwr_count + 1'b1;
            end else if (r_cur_pkt_type == MRD_TYPE) begin
                if (r_buf_mrd_count < {DATA_FIDELITY{1'b1}})
                    r_buf_mrd_count <= r_buf_mrd_count + 1'b1;
            end else begin
                if (r_buf_other_count < {DATA_FIDELITY{1'b1}})
                    r_buf_other_count <= r_buf_other_count + 1'b1;
            end
            
            // Start gap counting
            r_in_packet  <= 1'b0;
            r_gap_active <= 1'b1;
            r_gap_count  <= {DATA_FIDELITY{1'b0}};
        end

        // ----------------------------------------------------------
        // Gap counting (between packets)
        // ----------------------------------------------------------
        if (r_gap_active && !is_sop) begin
            if (r_gap_count < {DATA_FIDELITY{1'b1}})
                r_gap_count <= r_gap_count + 1'b1;
        end

        // ----------------------------------------------------------
        // Buffer reset on stream start (clear stats, keep data)
        // ----------------------------------------------------------
        if (r_stream_state == ST_COLLECT && r_stream_counter == (STREAM_GAP - 1)) begin
            r_buf_mwr_count   <= {DATA_FIDELITY{1'b0}};
            r_buf_mrd_count   <= {DATA_FIDELITY{1'b0}};
            r_buf_other_count <= {DATA_FIDELITY{1'b0}};
        end

    end
end

// =================================================================
// Streaming State Machine
//
// Streams out telemetry data in a cyclic pattern:
//   - 1024 cycles streaming (512 packets × 2 cycles valid + 2 cycles invalid)
//   - 8192 cycles gap
//   - Repeat
//
// Data output pattern per packet:
//   Cycle 0-1: valid high (data stable)
//   Cycle 2-3: valid low  (data stable but marked invalid)
// =================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_stream_state   <= ST_WAIT_STREAM;
        r_stream_counter <= 13'd0;
        r_stream_pkt_cnt <= {ADDR_WIDTH{1'b0}};
        r_read_ptr       <= {ADDR_WIDTH{1'b0}};
        tel_enable       <= 1'b0;
        tel_valid        <= 1'b0;
        tel_pkt_length   <= {DATA_FIDELITY{1'b0}};
        tel_pkt_gap      <= {DATA_FIDELITY{1'b0}};
        tel_pkt_type     <= 4'd0;
        tel_pkt_addr     <= 32'd0;
        tel_payload_dw   <= {DATA_FIDELITY{1'b0}};
        tel_is_sop       <= 1'b0;
        tel_is_eop       <= 1'b0;
        tel_pkt_tag      <= 8'd0;
        tel_addr_type    <= 2'd0;
        tel_total_pkts   <= {DATA_FIDELITY{1'b0}};
        tel_mwr_count    <= {DATA_FIDELITY{1'b0}};
        tel_mrd_count    <= {DATA_FIDELITY{1'b0}};
        tel_other_count  <= {DATA_FIDELITY{1'b0}};
    end else begin

        case (r_stream_state)

            // --------------------------------------------------
            // COLLECT: Waiting for next streaming interval
            // --------------------------------------------------
            ST_COLLECT: begin
                tel_enable <= 1'b0;
                tel_valid  <= 1'b0;
                
                if (r_stream_counter < (STREAM_GAP - 1)) begin
                    r_stream_counter <= r_stream_counter + 1'b1;
                end else begin
                    // Start streaming
                    r_stream_state   <= ST_STREAM_V1;
                    r_stream_counter <= 13'd0;
                    r_stream_pkt_cnt <= {ADDR_WIDTH{1'b0}};
                    r_read_ptr       <= r_write_ptr - r_valid_count;  // Start from oldest entry
                end
            end

            // --------------------------------------------------
            // STREAM_V1: First valid cycle (load data)
            // --------------------------------------------------
            ST_STREAM_V1: begin
                tel_enable <= 1'b1;
                tel_valid  <= 1'b1;
                
                // Load telemetry data from memory
                if (r_stream_pkt_cnt < r_valid_count) begin
                    tel_pkt_length  <= mem_pkt_length[r_read_ptr];
                    tel_pkt_gap     <= mem_pkt_gap[r_read_ptr];
                    tel_pkt_type    <= mem_pkt_type[r_read_ptr];
                    tel_pkt_addr    <= mem_pkt_addr[r_read_ptr];
                    tel_payload_dw  <= mem_payload_dw[r_read_ptr];
                    tel_pkt_tag     <= mem_pkt_tag[r_read_ptr];
                    tel_addr_type   <= mem_addr_type[r_read_ptr];
                    tel_is_sop      <= 1'b1;
                    tel_is_eop      <= 1'b0;
                end else begin
                    // No more valid data, output zeros
                    tel_pkt_length  <= {DATA_FIDELITY{1'b0}};
                    tel_pkt_gap     <= {DATA_FIDELITY{1'b0}};
                    tel_pkt_type    <= 4'd0;
                    tel_pkt_addr    <= 32'd0;
                    tel_payload_dw  <= {DATA_FIDELITY{1'b0}};
                    tel_pkt_tag     <= 8'd0;
                    tel_addr_type   <= 2'd0;
                    tel_is_sop      <= 1'b0;
                    tel_is_eop      <= 1'b0;
                end
                
                // Statistics (constant during streaming)
                tel_total_pkts  <= r_total_pkts;
                tel_mwr_count   <= r_buf_mwr_count;
                tel_mrd_count   <= r_buf_mrd_count;
                tel_other_count <= r_buf_other_count;
                
                r_stream_state   <= ST_STREAM_V2;
                r_stream_counter <= r_stream_counter + 1'b1;
            end

            // --------------------------------------------------
            // STREAM_V2: Second valid cycle (data held)
            // --------------------------------------------------
            ST_STREAM_V2: begin
                tel_enable    <= 1'b1;
                tel_valid     <= 1'b1;
                tel_is_sop    <= 1'b0;
                tel_is_eop    <= 1'b1;
                
                r_stream_state   <= ST_STREAM_I1;
                r_stream_counter <= r_stream_counter + 1'b1;
            end

            // --------------------------------------------------
            // STREAM_I1: First invalid cycle (data held, valid low)
            // --------------------------------------------------
            ST_STREAM_I1: begin
                tel_enable <= 1'b1;
                tel_valid  <= 1'b0;
                
                r_stream_state   <= ST_STREAM_I2;
                r_stream_counter <= r_stream_counter + 1'b1;
            end

            // --------------------------------------------------
            // STREAM_I2: Second invalid cycle, advance to next packet
            // --------------------------------------------------
            ST_STREAM_I2: begin
                tel_enable <= 1'b1;
                tel_valid  <= 1'b0;
                
                r_read_ptr       <= r_read_ptr + 1'b1;
                r_stream_pkt_cnt <= r_stream_pkt_cnt + 1'b1;
                r_stream_counter <= r_stream_counter + 1'b1;
                
                // Check if streaming window complete (1024 cycles)
                if (r_stream_counter >= (STREAM_DURATION - 1)) begin
                    r_stream_state   <= ST_COLLECT;
                    r_stream_counter <= 13'd0;
                end else begin
                    r_stream_state <= ST_STREAM_V1;
                end
            end

            default: begin
                r_stream_state <= ST_COLLECT;
            end

        endcase
    end
end

endmodule
