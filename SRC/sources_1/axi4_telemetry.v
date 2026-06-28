module axi4_telemetry #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer TELEMETRY_DEPTH  = 512,  // Depth: number of packets to buffer
    parameter integer DATA_FIDELITY    = 8,    // Max value bits for length/gap counters (default 8 = max 255)
    parameter integer ILA_DEPTH        = 0,    // ILA sample depth (0 = auto-set to STREAM_DURATION)
    parameter         ENABLE_ILA       = 1,    // 1 = Enable ILA instantiation (requires ila_0 IP), 0 = Disable
    parameter         IF_TYPE          = "CQ"  // "CQ" or "RQ" — selects tuser sideband bit layout (PG343)
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

// -----------------------------------------------------------------------------
// tuser sideband field offsets (LSB of each field).
//
// The descriptor (tdata) fields — request_type[78:75], address_type[1:0],
// address[63:2], dword_count[74:64], tag[103:96] — share identical positions in
// both the CQ and RQ descriptor formats, so only the tuser SOP/EOP/EOP_PTR
// offsets move between interfaces (PG343 §1.2 vs §3.2):
//   CQ : is_sop[81:80] is_eop[87:86] is_eop0_ptr[91:88]
//   RQ : is_sop[21:20] is_eop[27:26] is_eop0_ptr[31:28]
// -----------------------------------------------------------------------------
localparam integer SOP_LO    = (IF_TYPE == "RQ") ? 20 : 80;
localparam integer EOP_LO    = (IF_TYPE == "RQ") ? 26 : 86;
localparam integer EOPPTR_LO = (IF_TYPE == "RQ") ? 28 : 88;

// Streaming state machine
localparam [2:0] ST_COLLECT   = 3'd0;  // Collecting telemetry data (and gap before streaming)
localparam [2:0] ST_READ_REQ  = 3'd1;  // Issue BRAM read request (1-cycle latency)
localparam [2:0] ST_STREAM_V1 = 3'd2;  // Streaming cycle 1 (valid high, data arrives)
localparam [2:0] ST_STREAM_V2 = 3'd3;  // Streaming cycle 2 (valid high)
localparam [2:0] ST_STREAM_I1 = 3'd4;  // Streaming cycle 3 (valid low)
localparam [2:0] ST_STREAM_I2 = 3'd5;  // Streaming cycle 4 (valid low, next read issued)

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
wire [10:0]   dword_count;

assign request_type = s_axis_tdata[78:75];
assign address_type = s_axis_tdata[1:0];
assign address_full = s_axis_tdata[63:2];
assign address_msb  = address_full[61:30];  // Top 32 bits of the 62-bit address field
assign tag          = s_axis_tdata[103:96];
assign sop          = s_axis_tuser[SOP_LO    + 1 : SOP_LO];
assign eop          = s_axis_tuser[EOP_LO    + 1 : EOP_LO];
assign eop_ptr      = s_axis_tuser[EOPPTR_LO + 3 : EOPPTR_LO];
assign dword_count  = s_axis_tdata[74:64];  // DW count field for MWr/MRd (11 bits, 0-1024 per PG343)

// Transaction handshake (monitoring the passthrough)
wire beat_fire = s_axis_tvalid && m_axis_tready;
wire is_sop    = beat_fire && (|sop);   // catch SOP on either segment (tuser is_sop field)
wire is_eop    = beat_fire && ((|eop) | (s_axis_tlast));   // use tuser is_eop field, not tlast

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
reg [10:0]               r_cur_dword_count;

// Telemetry buffer storage - SINGLE PACKED BRAM
// Packing format (LSB to MSB):
//   [7:0]     pkt_length
//   [15:8]    pkt_gap
//   [19:16]   pkt_type
//   [51:20]   pkt_addr
//   [59:52]   payload_dw
//   [67:60]   pkt_tag
//   [69:68]   addr_type
localparam TEL_ENTRY_WIDTH = 70;  // Total packed width

// Use XPM Block RAM for better synthesis and routing
wire                        mem_ena;
wire                        mem_wea;
wire [ADDR_WIDTH-1:0]       mem_addra;
wire [TEL_ENTRY_WIDTH-1:0]  mem_dina;
wire [TEL_ENTRY_WIDTH-1:0]  mem_douta;

wire                        mem_enb;
wire [ADDR_WIDTH-1:0]       mem_addrb;
wire [TEL_ENTRY_WIDTH-1:0]  mem_doutb;

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
// Block RAM Instantiation (XPM)
// Single dual-port BRAM for better routing and timing
// =================================================================

xpm_memory_sdpram #(
    .ADDR_WIDTH_A(ADDR_WIDTH),
    .ADDR_WIDTH_B(ADDR_WIDTH),
    .AUTO_SLEEP_TIME(0),
    .BYTE_WRITE_WIDTH_A(TEL_ENTRY_WIDTH),
    .CASCADE_HEIGHT(0),
    .CLOCKING_MODE("common_clock"),
    .ECC_MODE("no_ecc"),
    .MEMORY_INIT_FILE("none"),
    .MEMORY_INIT_PARAM("0"),
    .MEMORY_OPTIMIZATION("true"),
    .MEMORY_PRIMITIVE("block"),          // Force Block RAM (not distributed)
    .MEMORY_SIZE(TELEMETRY_DEPTH * TEL_ENTRY_WIDTH),
    .MESSAGE_CONTROL(0),
    .READ_DATA_WIDTH_B(TEL_ENTRY_WIDTH),
    .READ_LATENCY_B(1),                  // 1-cycle read latency
    .READ_RESET_VALUE_B("0"),
    .RST_MODE_A("SYNC"),
    .RST_MODE_B("SYNC"),
    .SIM_ASSERT_CHK(0),
    .USE_EMBEDDED_CONSTRAINT(0),
    .USE_MEM_INIT(0),
    .USE_MEM_INIT_MMI(0),
    .WAKEUP_TIME("disable_sleep"),
    .WRITE_DATA_WIDTH_A(TEL_ENTRY_WIDTH),
    .WRITE_MODE_B("read_first"),
    .WRITE_PROTECT(1)
) u_telemetry_bram (
    .clka(clk),
    .clkb(clk),
    .ena(mem_ena),
    .enb(mem_enb),
    .wea(mem_wea),
    .addra(mem_addra),
    .dina(mem_dina),
    .addrb(mem_addrb),
    .doutb(mem_doutb),
    .regceb(1'b1),
    .rstb(~rst_n),
    .sleep(1'b0),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .dbiterrb(),
    .sbiterrb()
);

// BRAM write port control
reg                     r_mem_wea;
reg [ADDR_WIDTH-1:0]    r_mem_addra;
reg [TEL_ENTRY_WIDTH-1:0] r_mem_dina;

assign mem_ena   = 1'b1;  // Always enabled
assign mem_wea   = r_mem_wea;
assign mem_addra = r_mem_addra;
assign mem_dina  = r_mem_dina;

// BRAM read port control
reg                     r_mem_enb;
reg [ADDR_WIDTH-1:0]    r_mem_addrb;

assign mem_enb   = r_mem_enb;
assign mem_addrb = r_mem_addrb;

// Unpack read data (registered output from BRAM, already has 1 cycle latency)
wire [DATA_FIDELITY-1:0] mem_rd_pkt_length  = mem_doutb[7:0];
wire [DATA_FIDELITY-1:0] mem_rd_pkt_gap     = mem_doutb[15:8];
wire [3:0]               mem_rd_pkt_type    = mem_doutb[19:16];
wire [31:0]              mem_rd_pkt_addr    = mem_doutb[51:20];
wire [DATA_FIDELITY-1:0] mem_rd_payload_dw  = mem_doutb[59:52];
wire [7:0]               mem_rd_pkt_tag     = mem_doutb[67:60];
wire [1:0]               mem_rd_addr_type   = mem_doutb[69:68];

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
        r_cur_dword_count <= 11'd0;
        r_write_ptr       <= {ADDR_WIDTH{1'b0}};
        r_valid_count     <= {ADDR_WIDTH{1'b0}};
        r_total_pkts      <= {DATA_FIDELITY{1'b0}};
        r_buf_mwr_count   <= {DATA_FIDELITY{1'b0}};
        r_buf_mrd_count   <= {DATA_FIDELITY{1'b0}};
        r_buf_other_count <= {DATA_FIDELITY{1'b0}};
        r_mem_wea         <= 1'b0;
        r_mem_addra       <= {ADDR_WIDTH{1'b0}};
        r_mem_dina        <= {TEL_ENTRY_WIDTH{1'b0}};
    end else begin

        // Default: no write
        r_mem_wea <= 1'b0;

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
            
            // Capture address MSB for all packet types
            r_cur_pkt_addr <= address_msb;
                
        // ----------------------------------------------------------
        // Mid-packet beat (not EOP; EOP beat counted at write time)
        // ----------------------------------------------------------
        end else if (r_in_packet && beat_fire && !(|eop)) begin
            // Increment beat counter (saturate at max)
            if (r_beat_count < {DATA_FIDELITY{1'b1}})
                r_beat_count <= r_beat_count + 1'b1;
        end

        // ----------------------------------------------------------
        // EOP: End of Packet (can coincide with SOP for single-beat pkts)
        // ----------------------------------------------------------
        if (is_eop) begin
            // Pack all fields into single BRAM write
            // Calculate payload in DWORDs inline (for MWr, use dword_count; others = 0)
            // Beat count: include the EOP beat itself (+1), except for single-beat
            // packets where is_sop fires on the same cycle (store 1 directly).
            r_mem_dina <= {
                r_cur_addr_type,      // [69:68]
                r_cur_pkt_tag,        // [67:60]
                (r_cur_pkt_type == MWR_TYPE) ?  // [59:52] payload_dw
                    ((r_cur_dword_count[10:0] > {{(11-DATA_FIDELITY){1'b0}}, {DATA_FIDELITY{1'b1}}}) ?
                        {DATA_FIDELITY{1'b1}} : r_cur_dword_count[DATA_FIDELITY-1:0]) :
                    {DATA_FIDELITY{1'b0}},
                r_cur_pkt_addr,       // [51:20]
                r_cur_pkt_type,       // [19:16]
                r_gap_count,          // [15:8]
                is_sop ? {{(DATA_FIDELITY-1){1'b0}}, 1'b1}  // single-beat packet
                       : (r_beat_count < {DATA_FIDELITY{1'b1}} ? r_beat_count + 1'b1
                                                                 : {DATA_FIDELITY{1'b1}})  // [7:0]
            };
            r_mem_addra <= r_write_ptr;
            r_mem_wea   <= 1'b1;  // Write enable
            
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
// Streams out telemetry data in a cyclic pattern with BRAM read pipelining:
//   - COLLECT: gap between streaming windows
//   - READ_REQ: issue BRAM read (1-cycle latency)
//   - STREAM_V1/V2: 2 cycles valid (data from BRAM)
//   - STREAM_I1/I2: 2 cycles invalid (next read issued in I2)
//
// Timing: 512 packets × (1 read + 4 output) = 2560 cycles per stream
// =================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_stream_state   <= ST_COLLECT;
        r_stream_counter <= 16'd0;
        r_stream_pkt_cnt <= {ADDR_WIDTH{1'b0}};
        r_read_ptr       <= {ADDR_WIDTH{1'b0}};
        r_mem_enb        <= 1'b0;
        r_mem_addrb      <= {ADDR_WIDTH{1'b0}};
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
                r_mem_enb  <= 1'b0;
                
                if (r_stream_counter < (STREAM_GAP - 1)) begin
                    r_stream_counter <= r_stream_counter + 1'b1;
                end else begin
                    // Start streaming - issue first read
                    r_stream_state   <= ST_READ_REQ;
                    r_stream_counter <= 16'd0;
                    r_stream_pkt_cnt <= {ADDR_WIDTH{1'b0}};
                    r_read_ptr       <= r_write_ptr - r_valid_count;  // Start from oldest entry
                end
            end

            // --------------------------------------------------
            // READ_REQ: Issue BRAM read request
            // Data will be available next cycle
            // --------------------------------------------------
            ST_READ_REQ: begin
                tel_enable <= 1'b1;
                
                if (r_stream_pkt_cnt < r_valid_count) begin
                    // Issue read request
                    r_mem_enb   <= 1'b1;
                    r_mem_addrb <= r_read_ptr;
                end else begin
                    r_mem_enb <= 1'b0;
                end
                
                r_stream_state   <= ST_STREAM_V1;
                r_stream_counter <= r_stream_counter + 1'b1;
            end

            // --------------------------------------------------
            // STREAM_V1: First valid cycle (data from BRAM available)
            // --------------------------------------------------
            ST_STREAM_V1: begin
                tel_enable <= 1'b1;
                tel_valid  <= 1'b1;
                
                // Load telemetry data from BRAM output
                if (r_stream_pkt_cnt < r_valid_count) begin
                    tel_pkt_length  <= mem_rd_pkt_length;
                    tel_pkt_gap     <= mem_rd_pkt_gap;
                    tel_pkt_type    <= mem_rd_pkt_type;
                    tel_pkt_addr    <= mem_rd_pkt_addr;
                    tel_payload_dw  <= mem_rd_payload_dw;
                    tel_pkt_tag     <= mem_rd_pkt_tag;
                    tel_addr_type   <= mem_rd_addr_type;
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
            // Issue next BRAM read request here
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
                    r_mem_enb        <= 1'b0;
                end else begin
                    // Continue streaming - go to READ_REQ for next packet
                    r_stream_state <= ST_READ_REQ;
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

// ILA IP Core Instantiation (conditional)
// Configuration:
//   - 20 individual probes (one per signal)
//   - Sample depth: ACTUAL_ILA_DEPTH (defaults to STREAM_DURATION)
//   - Each probe has its own comparator for flexible triggering
//
// To enable ILA:
//   1. Generate the ila_0 IP using the TCL script below
//   2. Set ENABLE_ILA parameter to 1 when instantiating this module
//
// Create with Vivado TCL:
//   See comments below for automated IP generation script

generate
    if (ENABLE_ILA == 1) begin : gen_ila
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
    end
endgenerate

// =================================================================
// ILA IP Generation Script (Vivado TCL)
// =================================================================
// Step 1: Copy and run in Vivado TCL console to generate the ILA IP:
//
// create_ip -name ila -vendor xilinx.com -library ip -module_name ila_0
// set_property -dict [list \
//   CONFIG.C_NUM_OF_PROBES {20} \
//   CONFIG.C_PROBE0_WIDTH {1} \
//   CONFIG.C_PROBE1_WIDTH {1} \
//   CONFIG.C_PROBE2_WIDTH {1} \
//   CONFIG.C_PROBE3_WIDTH {1} \
//   CONFIG.C_PROBE4_WIDTH {1} \
//   CONFIG.C_PROBE5_WIDTH {3} \
//   CONFIG.C_PROBE6_WIDTH {16} \
//   CONFIG.C_PROBE7_WIDTH {8} \
//   CONFIG.C_PROBE8_WIDTH {8} \
//   CONFIG.C_PROBE9_WIDTH {4} \
//   CONFIG.C_PROBE10_WIDTH {32} \
//   CONFIG.C_PROBE11_WIDTH {8} \
//   CONFIG.C_PROBE12_WIDTH {8} \
//   CONFIG.C_PROBE13_WIDTH {2} \
//   CONFIG.C_PROBE14_WIDTH {8} \
//   CONFIG.C_PROBE15_WIDTH {8} \
//   CONFIG.C_PROBE16_WIDTH {8} \
//   CONFIG.C_PROBE17_WIDTH {8} \
//   CONFIG.C_PROBE18_WIDTH {9} \
//   CONFIG.C_PROBE19_WIDTH {9} \
//   CONFIG.C_DATA_DEPTH {2048} \
//   CONFIG.C_TRIGIN_EN {false} \
//   CONFIG.C_TRIGOUT_EN {false} \
//   CONFIG.C_ADV_TRIGGER {true} \
//   CONFIG.C_EN_STRG_QUAL {1} \
//   CONFIG.ALL_PROBE_SAME_MU {true} \
// ] [get_ips ila_0]
//
// Step 2: Set ENABLE_ILA=1 when instantiating this module:
//   axi4_telemetry #(.ENABLE_ILA(1)) u_telemetry (...);
//
// Note: C_DATA_DEPTH should match STREAM_DURATION:
//   256 packets  -> 1024 depth
//   512 packets  -> 2048 depth (default)
//   1024 packets -> 4096 depth
// =================================================================

endmodule
