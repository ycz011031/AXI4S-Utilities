module axi4_telemetry #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer TELEMETRY_DEPTH  = 512,  // Depth: number of packets to buffer
    parameter integer DATA_FIDELITY    = 8,    // Max value bits for length/gap counters (default 8 = max 255)
    parameter integer ILA_DEPTH        = 0     // ILA sample depth (0 = auto-set to STREAM_DURATION)
)(
    input wire                          clk,
    input wire                          rst_n,

    // AXI4-Stream Slave Interface (input)
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep,
    input  wire                          s_axis_tvalid,
    input  wire                          s_axis_tlast,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser,
    output wire                          s_axis_tready,

    // AXI4-Stream Master Interface (transparent passthrough)
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_tkeep,
    output wire                          m_axis_tvalid,
    output wire                          m_axis_tlast,
    output wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser,
    input  wire                          m_axis_tready
);

// =================================================================
// Transparent AXI4-Stream Passthrough
// =================================================================
assign m_axis_tdata  = s_axis_tdata;
assign m_axis_tkeep  = s_axis_tkeep;
assign m_axis_tvalid = s_axis_tvalid;
assign m_axis_tlast  = s_axis_tlast;
assign m_axis_tuser  = s_axis_tuser;
assign s_axis_tready = m_axis_tready;

// =================================================================
// Local Parameters
// =================================================================
localparam MWR_TYPE = 4'b0001;
localparam MRD_TYPE = 4'b0000;

// Address width for telemetry buffer
localparam ADDR_WIDTH = $clog2(TELEMETRY_DEPTH);

// Streaming state machine
localparam [2:0] ST_COLLECT   = 3'd0;  // Collecting telemetry data (and gap before streaming)
localparam [2:0] ST_STREAM_V1 = 3'd1;  // Streaming cycle 1 (valid high)
localparam [2:0] ST_STREAM_V2 = 3'd2;  // Streaming cycle 2 (valid high)
localparam [2:0] ST_STREAM_I1 = 3'd3;  // Streaming cycle 3 (valid low)
localparam [2:0] ST_STREAM_I2 = 3'd4;  // Streaming cycle 4 (valid low)

// Timing constants (auto-calculated from TELEMETRY_DEPTH)
localparam STREAM_DURATION = TELEMETRY_DEPTH * 4;  // 4 cycles per packet (2 valid + 2 invalid)
localparam STREAM_GAP      = STREAM_DURATION * 8;  // Gap is 8x the streaming duration

// ILA sample depth (auto-set to stream duration if not specified)
localparam ACTUAL_ILA_DEPTH = (ILA_DEPTH == 0) ? STREAM_DURATION : ILA_DEPTH;

// Note: With default TELEMETRY_DEPTH=512:
//   STREAM_DURATION  = 2048 cycles
//   STREAM_GAP       = 16384 cycles
//   ACTUAL_ILA_DEPTH = 2048 samples (captures one complete streaming window)

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

// Transaction handshake (monitoring the passthrough)
wire beat_fire = s_axis_tvalid && m_axis_tready;
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
reg [15:0]               r_stream_counter;   // Counts cycles within streaming/gap phases (16 bits to handle larger depths)
reg [ADDR_WIDTH-1:0]     r_stream_pkt_cnt;   // Packet counter during streaming

// Telemetry output registers (internal, captured by ILA)
reg                      tel_enable;         // High during streaming window
reg                      tel_valid;          // High when data is valid (2-cycle pattern)
reg [DATA_FIDELITY-1:0]  tel_pkt_length;     // Packet length in beats
reg [DATA_FIDELITY-1:0]  tel_pkt_gap;        // Gap between packets in cycles
reg [3:0]                tel_pkt_type;       // Request type field [78:75]
reg [31:0]               tel_pkt_addr;       // MSB 32 bits of address (MWr/MRd only)
reg [DATA_FIDELITY-1:0]  tel_payload_dw;     // Payload size in DWORDs (MWr only)
reg [7:0]                tel_pkt_tag;        // PCIe tag [103:96]
reg [1:0]                tel_addr_type;      // Address type [1:0]
reg [DATA_FIDELITY-1:0]  tel_total_pkts;     // Total packets seen (saturating)
reg [DATA_FIDELITY-1:0]  tel_mwr_count;      // MWr packets in this buffer
reg [DATA_FIDELITY-1:0]  tel_mrd_count;      // MRd packets in this buffer
reg [DATA_FIDELITY-1:0]  tel_other_count;    // Other packet types in this buffer

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
        r_stream_state   <= ST_COLLECT;
        r_stream_counter <= 16'd0;
        r_stream_pkt_cnt <= {ADDR_WIDTH{1'b0}};
        r_read_ptr       <= {ADDR_WIDTH{1'b0}};
        tel_enable       <= 1'b0;
        tel_valid        <= 1'b0;
        tel_pkt_length   <= {DATA_FIDELITY{1'b0}};
        tel_pkt_gap      <= {DATA_FIDELITY{1'b0}};
        tel_pkt_type     <= 4'd0;
        tel_pkt_addr     <= 32'd0;
        tel_payload_dw   <= {DATA_FIDELITY{1'b0}};
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
                    r_stream_counter <= 16'd0;
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
                end else begin
                    // No more valid data, output zeros
                    tel_pkt_length  <= {DATA_FIDELITY{1'b0}};
                    tel_pkt_gap     <= {DATA_FIDELITY{1'b0}};
                    tel_pkt_type    <= 4'd0;
                    tel_pkt_addr    <= 32'd0;
                    tel_payload_dw  <= {DATA_FIDELITY{1'b0}};
                    tel_pkt_tag     <= 8'd0;
                    tel_addr_type   <= 2'd0;
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
                
                // Check if streaming window complete
                if (r_stream_counter >= (STREAM_DURATION - 1)) begin
                    r_stream_state   <= ST_COLLECT;
                    r_stream_counter <= 16'd0;
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

// =================================================================
// Integrated Logic Analyzer (ILA) for Debug
// Each signal has a dedicated probe for easy viewing and independent triggering
// =================================================================

// ILA IP Core Instantiation
// Configuration:
//   - 20 individual probes (one per signal)
//   - Sample depth: ACTUAL_ILA_DEPTH (defaults to STREAM_DURATION)
//   - Each probe has its own comparator for flexible triggering
//
// Create with Vivado TCL:
//   See comments below for automated IP generation script

ila_0 u_ila_telemetry (
    .clk     (clk),
    
    // Control & Status Signals
    .probe0  (rst_n),              // [0:0]   System reset
    .probe1  (tel_enable),         // [0:0]   Streaming window active
    .probe2  (tel_valid),          // [0:0]   Data valid flag
    .probe3  (r_gap_active),       // [0:0]   Counting gap between packets
    .probe4  (r_in_packet),        // [0:0]   Currently in packet
    
    // State Machine & Counters
    .probe5  (r_stream_state),     // [2:0]   Stream state machine
    .probe6  (r_stream_counter),   // [15:0]  Stream/gap cycle counter
    
    // Packet Telemetry Data
    .probe7  (tel_pkt_length),     // [7:0]   Packet length in beats
    .probe8  (tel_pkt_gap),        // [7:0]   Gap between packets (cycles)
    .probe9  (tel_pkt_type),       // [3:0]   PCIe request type
    .probe10 (tel_pkt_addr),       // [31:0]  Packet address (MSB 32 bits)
    .probe11 (tel_payload_dw),     // [7:0]   Payload size in DWORDs
    .probe12 (tel_pkt_tag),        // [7:0]   PCIe transaction tag
    .probe13 (tel_addr_type),      // [1:0]   Address type
    
    // Statistics
    .probe14 (tel_total_pkts),     // [7:0]   Total packets seen (saturating)
    .probe15 (tel_mwr_count),      // [7:0]   MWr packet count in buffer
    .probe16 (tel_mrd_count),      // [7:0]   MRd packet count in buffer
    .probe17 (tel_other_count),    // [7:0]   Other packet types in buffer
    
    // Internal Buffer State
    .probe18 (r_valid_count),      // [8:0]   Valid entries in buffer (max 512)
    .probe19 (r_write_ptr)         // [8:0]   Current write pointer
);


endmodule
