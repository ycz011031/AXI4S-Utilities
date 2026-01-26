module pcie_cq_type_counter #
(
    parameter integer AXIS_DATA_WIDTH = 512,
    parameter integer AXIS_TUSER_WIDTH = 229
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

    // Transaction type counters (to ILA)
    output reg  [7:0]                    cnt_mem_read,           // 0000
    output reg  [7:0]                    cnt_mem_write,          // 0001
    output reg  [7:0]                    cnt_io_read,            // 0010
    output reg  [7:0]                    cnt_io_write,           // 0011
    output reg  [7:0]                    cnt_mem_fetch_add,      // 0100
    output reg  [7:0]                    cnt_mem_swap,           // 0101
    output reg  [7:0]                    cnt_mem_cas,            // 0110
    output reg  [7:0]                    cnt_locked_read,        // 0111
    output reg  [7:0]                    cnt_cfg0_read,          // 1000
    output reg  [7:0]                    cnt_cfg1_read,          // 1001
    output reg  [7:0]                    cnt_cfg0_write,         // 1010
    output reg  [7:0]                    cnt_cfg1_write,         // 1011
    output reg  [7:0]                    cnt_message,            // 1100
    output reg  [7:0]                    cnt_vendor_msg,         // 1101
    output reg  [7:0]                    cnt_ats_msg,            // 1110
    output reg  [7:0]                    cnt_reserved            // 1111
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
    // PCIe TLP Field Extraction (CQ descriptor formatting)
    // ============================================================
    wire [3:0] req_type = s_axis_tdata[78:75];
    wire [1:0] sop      = s_axis_tuser[81:80]; // Start of Packet indicator
    wire       is_sop   = (sop != 2'b00);      // SOP when != 0

    // ============================================================
    // Transaction Type Counters
    // ============================================================
    always @(posedge clk) begin
        if (!rst) begin
            cnt_mem_read      <= 8'd0;
            cnt_mem_write     <= 8'd0;
            cnt_io_read       <= 8'd0;
            cnt_io_write      <= 8'd0;
            cnt_mem_fetch_add <= 8'd0;
            cnt_mem_swap      <= 8'd0;
            cnt_mem_cas       <= 8'd0;
            cnt_locked_read   <= 8'd0;
            cnt_cfg0_read     <= 8'd0;
            cnt_cfg1_read     <= 8'd0;
            cnt_cfg0_write    <= 8'd0;
            cnt_cfg1_write    <= 8'd0;
            cnt_message       <= 8'd0;
            cnt_vendor_msg    <= 8'd0;
            cnt_ats_msg       <= 8'd0;
            cnt_reserved      <= 8'd0;
        end else begin
            // Count transaction types on valid transfers at SOP only
            if (s_axis_tvalid && s_axis_tready && is_sop) begin
                case (req_type)
                    4'b0000: if (cnt_mem_read      != 8'hFF) cnt_mem_read      <= cnt_mem_read      + 1'b1; // Memory Read
                    4'b0001: if (cnt_mem_write     != 8'hFF) cnt_mem_write     <= cnt_mem_write     + 1'b1; // Memory Write
                    4'b0010: if (cnt_io_read       != 8'hFF) cnt_io_read       <= cnt_io_read       + 1'b1; // I/O Read
                    4'b0011: if (cnt_io_write      != 8'hFF) cnt_io_write      <= cnt_io_write      + 1'b1; // I/O Write
                    4'b0100: if (cnt_mem_fetch_add != 8'hFF) cnt_mem_fetch_add <= cnt_mem_fetch_add + 1'b1; // Memory Fetch and Add
                    4'b0101: if (cnt_mem_swap      != 8'hFF) cnt_mem_swap      <= cnt_mem_swap      + 1'b1; // Memory Unconditional Swap
                    4'b0110: if (cnt_mem_cas       != 8'hFF) cnt_mem_cas       <= cnt_mem_cas       + 1'b1; // Memory Compare and Swap
                    4'b0111: if (cnt_locked_read   != 8'hFF) cnt_locked_read   <= cnt_locked_read   + 1'b1; // Locked Read (Legacy)
                    4'b1000: if (cnt_cfg0_read     != 8'hFF) cnt_cfg0_read     <= cnt_cfg0_read     + 1'b1; // Type 0 Config Read
                    4'b1001: if (cnt_cfg1_read     != 8'hFF) cnt_cfg1_read     <= cnt_cfg1_read     + 1'b1; // Type 1 Config Read
                    4'b1010: if (cnt_cfg0_write    != 8'hFF) cnt_cfg0_write    <= cnt_cfg0_write    + 1'b1; // Type 0 Config Write
                    4'b1011: if (cnt_cfg1_write    != 8'hFF) cnt_cfg1_write    <= cnt_cfg1_write    + 1'b1; // Type 1 Config Write
                    4'b1100: if (cnt_message       != 8'hFF) cnt_message       <= cnt_message       + 1'b1; // Message (non-ATS, non-Vendor)
                    4'b1101: if (cnt_vendor_msg    != 8'hFF) cnt_vendor_msg    <= cnt_vendor_msg    + 1'b1; // Vendor-Defined Message
                    4'b1110: if (cnt_ats_msg       != 8'hFF) cnt_ats_msg       <= cnt_ats_msg       + 1'b1; // ATS Message
                    4'b1111: if (cnt_reserved      != 8'hFF) cnt_reserved      <= cnt_reserved      + 1'b1; // Reserved
                endcase
            end
        end
    end

endmodule
