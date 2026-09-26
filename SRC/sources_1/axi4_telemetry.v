module axi4_telemetry #(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 183,
    parameter integer TELEMETRY_DEPTH  = 512,  // Depth: number of packets to buffer
    parameter integer DATA_FIDELITY    = 8,    // Max value bits for length/gap counters (default 8 = max 255)
    parameter integer ILA_DEPTH        = 0,    // ILA sample depth (0 = auto-set to STREAM_DURATION)
    parameter         ENABLE_ILA       = 1,    // 1 = Enable ILA instantiation (requires ila_0 IP), 0 = Disable
    parameter         IF_TYPE          = "CQ", // "CQ" or "RQ" — selects tuser sideband bit layout (PG343)

    // --- DMA logging mode (see telemetry_dma_format.md) ---
    // 0 = legacy ILA mode: BRAM ring + cyclic playback into ila_0.
    // 1 = DMA mode: one 16-byte record per TLP is streamed on m_axis_log_* into
    //     an external AXI DMA (S2MM, simple mode).  The ILA and the playback ring
    //     are not built; TELEMETRY_DEPTH becomes the depth of the record FIFO that
    //     absorbs DMA backpressure (power of two, >= 16).
    parameter         DMA_LOG          = 0,
    parameter integer DMA_REGION_BYTES = 1048576, // DDR region size, bytes; multiple of LOG_TDATA_WIDTH/8
    parameter integer LOG_TDATA_WIDTH  = 128      // m_axis_log tdata width: 128, 256, 512 or 1024
)(
    input wire                          clk,
    input wire                          rst_n,

    // AXI4-Stream Slave Interface (input)
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_tdata,
    input  wire [AXIS_DATA_WIDTH/32-1:0] s_axis_tkeep,   // PG343: tkeep is per-DWORD
    input  wire                          s_axis_tvalid,
    input  wire                          s_axis_tlast,
    input  wire [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser,
    output wire                          s_axis_tready,

    // AXI4-Stream Master Interface (transparent passthrough)
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata,
    output wire [AXIS_DATA_WIDTH/32-1:0] m_axis_tkeep,   // PG343: tkeep is per-DWORD
    output wire                          m_axis_tvalid,
    output wire                          m_axis_tlast,
    output wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser,
    input  wire                          m_axis_tready,

    // DMA logging control / status (DMA_LOG = 1; inert when DMA_LOG = 0).
    // A recording starts on a RISING EDGE of log_enable while idle and ends when
    // the region is full or log_enable goes low.  log_enable is synchronised
    // internally, so it may come from another clock domain.
    input  wire                          log_enable,
    output wire                          log_busy,       // recording or draining to the DMA
    output wire                          log_done,       // last recording closed (tlast sent); cleared on re-arm
    output wire                          log_overflow,   // >=1 record dropped in the last/current recording
    output wire [31:0]                   log_drop_count, // records dropped (written as zero records)

    // AXI4-Stream master to the AXI DMA S2MM slave (DMA_LOG = 1)
    output wire [LOG_TDATA_WIDTH-1:0]    m_axis_log_tdata,
    output wire [LOG_TDATA_WIDTH/8-1:0]  m_axis_log_tkeep,
    output wire                          m_axis_log_tvalid,
    output wire                          m_axis_log_tlast,
    input  wire                          m_axis_log_tready
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

// Max value the ADDR_WIDTH-wide valid-entry counter may hold.  A ring that
// tracks occupancy with (write_ptr - read_ptr) and no separate "full" bit
// tops out at DEPTH-1.  Computed as DEPTH[ADDR_WIDTH-1:0]-1 this is all-ones
// for power-of-two depths and DEPTH-1 otherwise.
//
// (The old saturation test compared `r_valid_count < TELEMETRY_DEPTH[ADDR_WIDTH-1:0]`.
//  For a power-of-two DEPTH — e.g. the default 512 — TELEMETRY_DEPTH[ADDR_WIDTH-1:0]
//  is 0, so the test was `< 0`, the count never advanced past 0, and the
//  streamer replayed nothing but zeros on every window.)
localparam [ADDR_WIDTH-1:0] VALID_COUNT_MAX = TELEMETRY_DEPTH[ADDR_WIDTH-1:0] - 1'b1;

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
wire [7:0]    tag;
wire [1:0]    sop;
wire [1:0]    eop;
wire [3:0]    eop_ptr;
wire [10:0]   dword_count;

assign request_type = s_axis_tdata[78:75];
assign address_type = s_axis_tdata[1:0];
// Full descriptor address field: Address[61:0] is a DWORD address, i.e. byte
// address bits [63:2].  Byte address [1:0] is not carried here — it is implied
// by first_be in tuser — so the recorded address resolves to a DWORD.
assign address_full = s_axis_tdata[63:2];
assign tag          = s_axis_tdata[103:96];
assign sop          = s_axis_tuser[SOP_LO    + 1 : SOP_LO];
assign eop          = s_axis_tuser[EOP_LO    + 1 : EOP_LO];
assign eop_ptr      = s_axis_tuser[EOPPTR_LO + 3 : EOPPTR_LO];
assign dword_count  = s_axis_tdata[74:64];  // DW count field for MWr/MRd (11 bits, 0-1024 per PG343)

// Transaction handshake (monitoring the passthrough)
//
// Straddle is DISABLED on this interface.  Per PG343 the tuser is_sop/is_eop
// sideband fields are *optional* when straddle is off (RQ §3.2: "new TLP always
// starts after tlast") and are typically tied low by the upstream, so they
// cannot be used to frame packets.  Derive framing from the AXI-S handshake:
//   EOP = tlast — the authoritative end-of-TLP marker when straddle is off.
//         (|eop is OR'd in only so a future straddle-enabled build still works.)
//   SOP = the first accepted beat of a new packet, i.e. a beat that fires while
//         we are not already inside a packet (r_in_packet == 0).
reg  r_in_packet;   // 1 = currently processing a packet (declared here so the
                    // is_sop expression below may reference it — Verilog forbids
                    // use-before-declaration)
wire beat_fire = s_axis_tvalid && m_axis_tready;
wire is_eop    = beat_fire && (s_axis_tlast || (|eop));
wire is_sop    = beat_fire && !r_in_packet;

// Type checks
wire is_mwr = (request_type == MWR_TYPE);
wire is_mrd = (request_type == MRD_TYPE);

// =================================================================
// Telemetry Collection Registers
// =================================================================

// Packet tracking (r_in_packet is declared up with the handshake wires above)
reg [DATA_FIDELITY-1:0]  r_beat_count;       // Current packet beat counter
reg [DATA_FIDELITY-1:0]  r_gap_count;        // Gap counter between packets
reg                      r_gap_active;       // 1 = counting gap after EOP

// Current packet info (captured on SOP)
reg [3:0]                r_cur_pkt_type;
reg [61:0]               r_cur_pkt_addr;     // Full Address[61:0] (DWORD address)
reg [7:0]                r_cur_pkt_tag;
reg [1:0]                r_cur_addr_type;
reg [10:0]               r_cur_dword_count;

// Telemetry buffer storage - SINGLE PACKED BRAM
// Packing format (LSB to MSB):
//   [7:0]     pkt_length
//   [15:8]    pkt_gap
//   [19:16]   pkt_type
//   [81:20]   pkt_addr     (full Address[61:0], DWORD address)
//   [89:82]   payload_dw
//   [97:90]   pkt_tag
//   [99:98]   addr_type
//
// NOTE: these slices are literal constants and do NOT track DATA_FIDELITY.
// The layout is only valid for DATA_FIDELITY == 8; changing that parameter
// desynchronises the pack expression from the unpack wires below.
localparam TEL_ENTRY_WIDTH = 100;  // Total packed width

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
reg [61:0]               tel_pkt_addr;       // Full Address[61:0] (DWORD address)
reg [DATA_FIDELITY-1:0]  tel_payload_dw;     // Payload size in DWORDs (MWr only)
reg [7:0]                tel_pkt_tag;        // PCIe tag [103:96]
reg [1:0]                tel_addr_type;      // Address type [1:0]
reg [DATA_FIDELITY-1:0]  tel_total_pkts;     // Total packets seen (saturating)
reg [DATA_FIDELITY-1:0]  tel_mwr_count;      // MWr packets in this buffer
reg [DATA_FIDELITY-1:0]  tel_mrd_count;      // MRd packets in this buffer
reg [DATA_FIDELITY-1:0]  tel_other_count;    // Other packet types in this buffer

// =================================================================
// Block RAM Instantiation (XPM)
// Single dual-port BRAM for better routing and timing.
// Built in ILA mode only; in DMA mode the playback FSM below is left
// dangling (reads zeros, drives nothing) and is trimmed by synthesis.
// =================================================================

generate
if (DMA_LOG == 0) begin : gen_ring_bram
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
end else begin : gen_no_ring_bram
    assign mem_doutb = {TEL_ENTRY_WIDTH{1'b0}};
end
endgenerate

// BRAM write port control
reg                     r_mem_wea;
reg [ADDR_WIDTH-1:0]    r_mem_addra;
reg [TEL_ENTRY_WIDTH-1:0] r_mem_dina;

assign mem_ena   = 1'b1;  // Always enabled
assign mem_wea   = r_mem_wea;
assign mem_addra = r_mem_addra;
assign mem_dina  = r_mem_dina;

// BRAM read port control.
//
// Present the read address COMBINATIONALLY during ST_READ_REQ so that the
// 1-cycle BRAM read latency lands the data in ST_STREAM_V1, where it is
// latched.  (Previously addrb/enb were registered inside ST_READ_REQ, so they
// only took effect in V1 and doutb was not valid until V2 — the V1 latch then
// captured the *previous* entry.  That shifted the whole window by one slot:
// the first slot showed stale data and the last buffered packet was dropped
// and emitted as zero.)
assign mem_enb   = (r_stream_state == ST_READ_REQ) &&
                   (r_stream_pkt_cnt < r_valid_count);
assign mem_addrb = r_read_ptr;

// Unpack read data (registered output from BRAM, already has 1 cycle latency)
wire [DATA_FIDELITY-1:0] mem_rd_pkt_length  = mem_doutb[7:0];
wire [DATA_FIDELITY-1:0] mem_rd_pkt_gap     = mem_doutb[15:8];
wire [3:0]               mem_rd_pkt_type    = mem_doutb[19:16];
wire [61:0]              mem_rd_pkt_addr    = mem_doutb[81:20];
wire [DATA_FIDELITY-1:0] mem_rd_payload_dw  = mem_doutb[89:82];
wire [7:0]               mem_rd_pkt_tag     = mem_doutb[97:90];
wire [1:0]               mem_rd_addr_type   = mem_doutb[99:98];

// Effective per-packet metadata used by the EOP write.  For a single-beat
// packet is_sop and is_eop fire on the same cycle, so the r_cur_* capture
// registers (written in the SOP branch below) have NOT yet updated and still
// hold the *previous* packet's values.  Select the live decode in that case
// so a 1-beat packet stores its own type/addr/tag/dword_count.
wire [3:0]  eff_pkt_type = is_sop ? request_type : r_cur_pkt_type;
wire [61:0] eff_pkt_addr = is_sop ? address_full : r_cur_pkt_addr;
wire [7:0]  eff_pkt_tag  = is_sop ? tag          : r_cur_pkt_tag;
wire [1:0]  eff_addr_typ = is_sop ? address_type : r_cur_addr_type;
wire [10:0] eff_dw_count = is_sop ? dword_count  : r_cur_dword_count;

// Per-packet values recorded at EOP (shared by the ILA ring and the DMA log).
// Beat count: include the EOP beat itself (+1), except for single-beat
// packets where is_sop fires on the same cycle (store 1 directly).
// Payload in DWORDs: dword_count for MWr (clamped), 0 for all other types.
wire [DATA_FIDELITY-1:0] eop_pkt_length =
    is_sop ? {{(DATA_FIDELITY-1){1'b0}}, 1'b1}
           : (r_beat_count < {DATA_FIDELITY{1'b1}} ? r_beat_count + 1'b1
                                                    : {DATA_FIDELITY{1'b1}});
wire [DATA_FIDELITY-1:0] eop_payload_dw =
    (eff_pkt_type == MWR_TYPE) ?
        ((eff_dw_count[10:0] > {{(11-DATA_FIDELITY){1'b0}}, {DATA_FIDELITY{1'b1}}}) ?
            {DATA_FIDELITY{1'b1}} : eff_dw_count[DATA_FIDELITY-1:0]) :
        {DATA_FIDELITY{1'b0}};

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
        r_cur_pkt_addr    <= 62'd0;
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
            
            // Capture the full address for all packet types
            r_cur_pkt_addr <= address_full;
                
        // ----------------------------------------------------------
        // Mid-packet beat (not EOP; EOP beat counted at write time)
        // ----------------------------------------------------------
        end else if (r_in_packet && beat_fire && !is_eop) begin
            // Increment beat counter (saturate at max)
            if (r_beat_count < {DATA_FIDELITY{1'b1}})
                r_beat_count <= r_beat_count + 1'b1;
        end

        // ----------------------------------------------------------
        // EOP: End of Packet (can coincide with SOP for single-beat pkts)
        // ----------------------------------------------------------
        if (is_eop) begin
            // Pack all fields into single BRAM write
            r_mem_dina <= {
                eff_addr_typ,         // [99:98]
                eff_pkt_tag,          // [97:90]
                eop_payload_dw,       // [89:82]
                eff_pkt_addr,         // [81:20]
                eff_pkt_type,         // [19:16]
                r_gap_count,          // [15:8]
                eop_pkt_length        // [7:0]
            };
            r_mem_addra <= r_write_ptr;
            r_mem_wea   <= 1'b1;  // Write enable
            
            // Advance write pointer (ring buffer)
            r_write_ptr <= r_write_ptr + 1'b1;
            
            // Update valid count (saturate at DEPTH-1; see VALID_COUNT_MAX)
            if (r_valid_count < VALID_COUNT_MAX)
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
        tel_enable       <= 1'b0;
        tel_valid        <= 1'b0;
        tel_pkt_length   <= {DATA_FIDELITY{1'b0}};
        tel_pkt_gap      <= {DATA_FIDELITY{1'b0}};
        tel_pkt_type     <= 4'd0;
        tel_pkt_addr     <= 62'd0;
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
                    // Start streaming - issue first read
                    r_stream_state   <= ST_READ_REQ;
                    r_stream_counter <= 16'd0;
                    r_stream_pkt_cnt <= {ADDR_WIDTH{1'b0}};
                    r_read_ptr       <= r_write_ptr - r_valid_count;  // Start from oldest entry
                end
            end

            // --------------------------------------------------
            // READ_REQ: Issue BRAM read request
            // The address/enable are driven combinationally (see the mem_enb /
            // mem_addrb assigns) from this state, so mem[r_read_ptr] is being
            // fetched this cycle and lands on doutb next cycle (ST_STREAM_V1).
            // --------------------------------------------------
            ST_READ_REQ: begin
                tel_enable <= 1'b1;

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
                    tel_pkt_addr    <= 62'd0;
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
            // STREAM_I2: Second invalid cycle, advance to next packet.
            // The next read is issued combinationally when the FSM re-enters
            // ST_READ_REQ with the just-incremented r_read_ptr.
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
// DMA Logging Path (DMA_LOG = 1)
//
// Writes one 16-byte record per TLP into a single DDR region through an
// external AXI DMA (S2MM).  Record layout: telemetry_dma_format.md.
//
//   EOP -> event reg -> xpm_fifo_sync -> zero expander -> beat packer -> m_axis_log
//
// Recording control (one-shot, edge-armed):
//   IDLE  : wait for a rising edge of log_enable (edges seen in any other state
//           are ignored, so re-arming needs log_enable to go low and high again).
//   REC   : every EOP claims one region slot.  If the FIFO is full the record
//           is dropped, but its slot is still claimed and it is later emitted
//           as an all-zero record, so every TLP keeps its position in DDR.
//   FINAL : write the closing FIFO entry: zeros still owed for drops, plus -
//           on an enable-low stop - one zero terminator record and zero padding
//           up to a whole beat.
//   DRAIN : wait for the tlast beat to be accepted by the DMA, then back to IDLE.
//
// The stream never exceeds DMA_REGION_BYTES, and tlast is on its last beat, so
// the DMA closes the transfer and nothing already in DDR is overwritten.
//
// FIFO entry = {last, zcnt, rec_valid, rec[111:0]}: emit zcnt zero records,
// then rec (if rec_valid).  'last' marks the entry whose final record carries
// tlast.  Carrying the owed-zero count in the entry keeps dropped records in
// their original position without needing a FIFO write per dropped record.
// =================================================================
localparam integer LOG_REC_BITS      = 128;
localparam integer LOG_REC_BYTES     = LOG_REC_BITS / 8;
localparam integer LOG_DATA_BITS     = 112;                                 // 14 data bytes + 2 pad
localparam integer LOG_RECS_PER_BEAT = LOG_TDATA_WIDTH / LOG_REC_BITS;      // records per AXIS beat
localparam integer LOG_REGION_RECS   = DMA_REGION_BYTES / LOG_REC_BYTES;    // record slots in region
localparam integer LOG_CNT_WIDTH     = $clog2(LOG_REGION_RECS + 1);
localparam integer LOG_BEAT_IDX_W    = (LOG_RECS_PER_BEAT > 1) ? $clog2(LOG_RECS_PER_BEAT) : 1;
localparam integer LOG_FIFO_WIDTH    = 1 + LOG_CNT_WIDTH + 1 + LOG_DATA_BITS;

// Record data bytes 0..13, little-endian (byte 0 = tdata[7:0]).
// Byte-sized fields assume DATA_FIDELITY == 8 (as does the ILA ring layout).
wire [LOG_DATA_BITS-1:0] log_rec_data = {
    {6'd0, eff_addr_typ},     // byte  13      addr_type
    eff_pkt_tag,              // byte  12      pkt_tag
    eop_payload_dw,           // byte  11      payload_dw
    {4'd0, eff_pkt_type},     // byte  10      pkt_type
    r_gap_count,              // byte   9      pkt_gap
    eop_pkt_length,           // byte   8      pkt_length
    {eff_pkt_addr, 2'b00}     // bytes  7..0   pkt_addr (byte address)
};

generate
if (DMA_LOG == 1) begin : gen_dma_log

    // ------------------------------------------------------------
    // log_enable synchroniser + edge detect
    // ------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] r_en_sync;
    reg                                r_en_prev;
    wire en_s    = r_en_sync[1];
    wire en_rise = en_s && !r_en_prev;

    // ------------------------------------------------------------
    // Record event register (breaks the tdata -> FIFO din path)
    // ------------------------------------------------------------
    reg                     r_ev_valid;
    reg [LOG_DATA_BITS-1:0] r_ev_rec;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_en_sync  <= 2'b00;
            r_en_prev  <= 1'b0;
            r_ev_valid <= 1'b0;
            r_ev_rec   <= {LOG_DATA_BITS{1'b0}};
        end else begin
            r_en_sync  <= {r_en_sync[0], log_enable};
            r_en_prev  <= en_s;
            r_ev_valid <= is_eop;
            if (is_eop)
                r_ev_rec <= log_rec_data;
        end
    end

    // ------------------------------------------------------------
    // Record FIFO
    // ------------------------------------------------------------
    reg                       fifo_wr_en;
    reg  [LOG_FIFO_WIDTH-1:0] fifo_din;
    wire                      fifo_full;
    wire                      fifo_wr_rst_busy;
    wire                      fifo_rd_en;
    wire [LOG_FIFO_WIDTH-1:0] fifo_dout;
    wire                      fifo_empty;

    wire fifo_no_room = fifo_full || fifo_wr_rst_busy;

    xpm_fifo_sync #(
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .FIFO_MEMORY_TYPE    ("auto"),
        .FIFO_READ_LATENCY   (0),
        .FIFO_WRITE_DEPTH    (TELEMETRY_DEPTH),
        .FULL_RESET_VALUE    (0),
        .PROG_EMPTY_THRESH   (10),
        .PROG_FULL_THRESH    (10),
        .RD_DATA_COUNT_WIDTH (1),
        .READ_DATA_WIDTH     (LOG_FIFO_WIDTH),
        .READ_MODE           ("fwft"),
        .SIM_ASSERT_CHK      (0),
        .USE_ADV_FEATURES    ("0000"),
        .WAKEUP_TIME         (0),
        .WRITE_DATA_WIDTH    (LOG_FIFO_WIDTH),
        .WR_DATA_COUNT_WIDTH (1)
    ) u_log_fifo (
        .sleep         (1'b0),
        .rst           (~rst_n),
        .wr_clk        (clk),
        .wr_en         (fifo_wr_en),
        .din           (fifo_din),
        .full          (fifo_full),
        .prog_full     (),
        .wr_data_count (),
        .overflow      (),
        .wr_rst_busy   (fifo_wr_rst_busy),
        .almost_full   (),
        .wr_ack        (),
        .rd_en         (fifo_rd_en),
        .dout          (fifo_dout),
        .empty         (fifo_empty),
        .prog_empty    (),
        .rd_data_count (),
        .underflow     (),
        .rd_rst_busy   (),
        .almost_empty  (),
        .data_valid    (),
        .injectsbiterr (1'b0),
        .injectdbiterr (1'b0),
        .sbiterr       (),
        .dbiterr       ()
    );

    // ------------------------------------------------------------
    // Recording control FSM (FIFO write side)
    // ------------------------------------------------------------
    localparam [1:0] LG_IDLE  = 2'd0;
    localparam [1:0] LG_REC   = 2'd1;
    localparam [1:0] LG_FINAL = 2'd2;
    localparam [1:0] LG_DRAIN = 2'd3;

    reg [1:0]               r_lg_state;
    reg [LOG_CNT_WIDTH-1:0] r_slots;       // region slots claimed so far
    reg [LOG_CNT_WIDTH-1:0] r_pending;     // dropped records not yet represented in the FIFO
    reg [LOG_CNT_WIDTH-1:0] r_drop_cnt;    // records dropped in this recording
    reg                     r_stop_full;   // recording ended because the region filled
    reg                     r_done;
    reg                     r_overflow;

    wire rec_claim     = (r_lg_state == LG_REC) && r_ev_valid;
    wire rec_last_slot = rec_claim && (r_slots == LOG_REGION_RECS - 1);
    wire rec_fits      = rec_claim && !fifo_no_room;

    // Zero records appended on an enable-low stop: one terminator plus padding
    // to a whole beat (1..LOG_RECS_PER_BEAT).  None when the region filled.
    wire [LOG_CNT_WIDTH-1:0] final_pad =
        r_stop_full ? {LOG_CNT_WIDTH{1'b0}}
                    : LOG_RECS_PER_BEAT - (r_slots % LOG_RECS_PER_BEAT);

    wire log_tlast_fire;

    always @(*) begin
        fifo_wr_en = 1'b0;
        fifo_din   = {LOG_FIFO_WIDTH{1'b0}};
        if (rec_fits) begin
            fifo_wr_en = 1'b1;
            fifo_din   = {rec_last_slot, r_pending, 1'b1, r_ev_rec};
        end else if (r_lg_state == LG_FINAL && !fifo_no_room) begin
            fifo_wr_en = 1'b1;
            fifo_din   = {1'b1, r_pending + final_pad, 1'b0, {LOG_DATA_BITS{1'b0}}};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_lg_state  <= LG_IDLE;
            r_slots     <= {LOG_CNT_WIDTH{1'b0}};
            r_pending   <= {LOG_CNT_WIDTH{1'b0}};
            r_drop_cnt  <= {LOG_CNT_WIDTH{1'b0}};
            r_stop_full <= 1'b0;
            r_done      <= 1'b0;
            r_overflow  <= 1'b0;
        end else begin
            case (r_lg_state)
                LG_IDLE: begin
                    if (en_rise) begin
                        r_slots     <= {LOG_CNT_WIDTH{1'b0}};
                        r_pending   <= {LOG_CNT_WIDTH{1'b0}};
                        r_drop_cnt  <= {LOG_CNT_WIDTH{1'b0}};
                        r_stop_full <= 1'b0;
                        r_done      <= 1'b0;
                        r_overflow  <= 1'b0;
                        r_lg_state  <= LG_REC;
                    end
                end

                LG_REC: begin
                    if (rec_claim) begin
                        r_slots <= r_slots + 1'b1;
                        if (rec_fits) begin
                            r_pending <= {LOG_CNT_WIDTH{1'b0}};
                        end else begin
                            r_pending  <= r_pending + 1'b1;
                            r_drop_cnt <= r_drop_cnt + 1'b1;
                            r_overflow <= 1'b1;
                        end
                    end

                    if (rec_last_slot) begin
                        // Region full.  If this record was dropped, its zero
                        // (and any earlier owed zeros) still need a FIFO entry.
                        r_stop_full <= 1'b1;
                        r_lg_state  <= rec_fits ? LG_DRAIN : LG_FINAL;
                    end else if (!en_s) begin
                        r_lg_state  <= LG_FINAL;
                    end
                end

                LG_FINAL: begin
                    if (!fifo_no_room) begin
                        r_slots    <= r_slots + final_pad;
                        r_pending  <= {LOG_CNT_WIDTH{1'b0}};
                        r_lg_state <= LG_DRAIN;
                    end
                end

                LG_DRAIN: begin
                    if (log_tlast_fire) begin
                        r_done     <= 1'b1;
                        r_lg_state <= LG_IDLE;
                    end
                end

                default: r_lg_state <= LG_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------
    // Zero expander (FIFO read side): one record per cycle
    // ------------------------------------------------------------
    wire                     h_valid  = !fifo_empty;
    wire                     h_last   = fifo_dout[LOG_FIFO_WIDTH-1];
    wire [LOG_CNT_WIDTH-1:0] h_zcnt   = fifo_dout[LOG_FIFO_WIDTH-2 -: LOG_CNT_WIDTH];
    wire                     h_rvalid = fifo_dout[LOG_DATA_BITS];
    wire [LOG_DATA_BITS-1:0] h_rec    = fifo_dout[LOG_DATA_BITS-1:0];

    reg  [LOG_CNT_WIDTH-1:0] r_zdone;      // zeros already emitted for the head entry
    wire                     pk_accept;    // beat packer can take a record

    wire                     z_phase   = (r_zdone != h_zcnt);
    wire                     z_lastone = ((r_zdone + 1'b1) == h_zcnt);
    wire                     emit      = h_valid && pk_accept;
    wire                     emit_pop  = emit && (!z_phase || (z_lastone && !h_rvalid));
    wire                     emit_last = emit_pop && h_last;
    wire [LOG_REC_BITS-1:0]  emit_rec  = z_phase ? {LOG_REC_BITS{1'b0}}
                                                 : {{(LOG_REC_BITS-LOG_DATA_BITS){1'b0}}, h_rec};

    assign fifo_rd_en = emit_pop;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            r_zdone <= {LOG_CNT_WIDTH{1'b0}};
        else if (emit)
            r_zdone <= emit_pop ? {LOG_CNT_WIDTH{1'b0}} : r_zdone + 1'b1;
    end

    // ------------------------------------------------------------
    // Beat packer: LOG_RECS_PER_BEAT records per beat, record i at
    // tdata[128*i +: 128] so records land in DDR in order.  Region size
    // and enable-stop padding guarantee the tlast record closes a beat.
    // ------------------------------------------------------------
    reg [LOG_TDATA_WIDTH-1:0] r_asm;
    reg [LOG_BEAT_IDX_W-1:0]  r_asm_idx;
    reg [LOG_TDATA_WIDTH-1:0] r_out_data;
    reg                       r_out_valid;
    reg                       r_out_last;
    reg [LOG_TDATA_WIDTH-1:0] beat_next;

    wire out_free     = !r_out_valid || m_axis_log_tready;
    wire asm_complete = (r_asm_idx == LOG_RECS_PER_BEAT - 1);

    assign pk_accept = !asm_complete || out_free;

    always @(*) begin
        beat_next = r_asm;
        beat_next[r_asm_idx*LOG_REC_BITS +: LOG_REC_BITS] = emit_rec;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_asm       <= {LOG_TDATA_WIDTH{1'b0}};
            r_asm_idx   <= {LOG_BEAT_IDX_W{1'b0}};
            r_out_data  <= {LOG_TDATA_WIDTH{1'b0}};
            r_out_valid <= 1'b0;
            r_out_last  <= 1'b0;
        end else begin
            if (r_out_valid && m_axis_log_tready)
                r_out_valid <= 1'b0;

            if (emit) begin
                if (asm_complete) begin
                    r_out_data  <= beat_next;
                    r_out_valid <= 1'b1;
                    r_out_last  <= emit_last;
                    r_asm       <= {LOG_TDATA_WIDTH{1'b0}};
                    r_asm_idx   <= {LOG_BEAT_IDX_W{1'b0}};
                end else begin
                    r_asm       <= beat_next;
                    r_asm_idx   <= r_asm_idx + 1'b1;
                end
            end
        end
    end

    // synthesis translate_off
    always @(posedge clk) begin
        if (rst_n && emit && emit_last && !asm_complete)
            $display("ERROR: %m tlast record does not close a beat (t=%0t)", $time);
    end
    // synthesis translate_on

    assign log_tlast_fire    = r_out_valid && r_out_last && m_axis_log_tready;

    assign m_axis_log_tdata  = r_out_data;
    assign m_axis_log_tkeep  = {(LOG_TDATA_WIDTH/8){1'b1}};
    assign m_axis_log_tvalid = r_out_valid;
    assign m_axis_log_tlast  = r_out_last;

    assign log_busy          = (r_lg_state != LG_IDLE);
    assign log_done          = r_done;
    assign log_overflow      = r_overflow;
    assign log_drop_count    = r_drop_cnt;   // zero-extended to 32 bits

end else begin : gen_no_dma_log

    assign m_axis_log_tdata  = {LOG_TDATA_WIDTH{1'b0}};
    assign m_axis_log_tkeep  = {(LOG_TDATA_WIDTH/8){1'b0}};
    assign m_axis_log_tvalid = 1'b0;
    assign m_axis_log_tlast  = 1'b0;
    assign log_busy          = 1'b0;
    assign log_done          = 1'b0;
    assign log_overflow      = 1'b0;
    assign log_drop_count    = 32'd0;

end
endgenerate

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

// Probe10 carries the address as a full 64-bit BYTE address so it can be read
// directly off the waveform without a mental shift.  tel_pkt_addr holds
// Address[61:0] (a DWORD address = byte address [63:2]), so the low two bits
// are appended as zero; the true byte offset within the DWORD is implied by
// first_be and is not recorded.
wire [63:0] tel_pkt_addr_byte = {tel_pkt_addr, 2'b00};

generate
    if (ENABLE_ILA == 1 && DMA_LOG == 0) begin : gen_ila
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
            .probe10 (tel_pkt_addr_byte), // [63:0]  Packet byte address (full)
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
//   CONFIG.C_PROBE10_WIDTH {64} \
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
//
// IMPORTANT: probe10 widened from 32 to 64 bits when the telemetry moved to
// full-address capture.  An ila_0 core generated before that change has a
// 32-bit probe10 and will fail elaboration with a port width mismatch.
// Re-run the script above (or reset_target/generate_target on the existing IP)
// so the core is rebuilt at the new width.
// =================================================================

endmodule
