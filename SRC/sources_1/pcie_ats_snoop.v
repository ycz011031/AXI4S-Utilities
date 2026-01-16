module pcie_cq_ats_snoop #
(
    parameter integer AXIS_DATA_WIDTH = 512,
    parameter integer AXIS_TUSER_WIDTH = 228
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

    // RQ AXI-stream output (Invalidation Completion)
    output reg  [AXIS_DATA_WIDTH-1:0]    rq_axis_tdata,
    output reg  [AXIS_DATA_WIDTH/8-1:0]  rq_axis_tkeep,
    output reg                           rq_axis_tvalid,
    input  wire                          rq_axis_tready,
    output reg                           rq_axis_tlast,

    // Debug outputs (to ILA)
    output reg                           ats_hit,       // pulse
    output reg [7:0]                     ats_tag,
    output reg [7:0]                     ats_msg_code,
    output reg [2:0]                     ats_msg_routing
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
    wire [7:0] msg_code  = s_axis_tdata[111:104];
    wire [2:0] routing   = s_axis_tdata[114:112];
    wire [7:0] tag       = s_axis_tdata[103:96];
    wire [3:0] req_type  = s_axis_tdata[78:75];

    wire is_message_tlp  = (req_type[3:2] == 2'b10); // Type = 10xx
    wire is_ats_msg      = (req_type == 4'b1110);    // Per PCIe table

    // identify invalidation requests (Message Code 0x14 or 0x15)
    wire is_inv_req = (msg_code == 8'h14) || (msg_code == 8'h15);

    // invalidation completion code (adjust if needed)
    localparam [7:0] INV_COMPLETE_CODE = 8'h30;

    // ============================================================
    // ATS Snooper
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            ats_hit        <= 1'b0;
            ats_tag        <= 8'd0;
            ats_msg_code   <= 8'd0;
            ats_msg_routing<= 3'd0;
        end else begin
            //ats_hit <= 1'b0;

            if (s_axis_tvalid && s_axis_tready) begin
                if (is_ats_msg) begin
                    ats_hit         <= 1'b1;
                    ats_tag         <= tag;
                    ats_msg_code    <= msg_code;
                    ats_msg_routing <= routing;
                end
            end
        end
    end

    // ============================================================
    // Invalidation Completion Generator (RQ AXIS)
    // ============================================================
    always @(posedge clk) begin
        if (rst) begin
            rq_axis_tvalid <= 1'b0;
            rq_axis_tdata  <= {AXIS_DATA_WIDTH{1'b0}};
            rq_axis_tkeep  <= {AXIS_DATA_WIDTH/8{1'b0}};
            rq_axis_tlast  <= 1'b0;
        end else begin
            rq_axis_tvalid <= 1'b0;
            rq_axis_tlast  <= 1'b0;

            if (s_axis_tvalid && s_axis_tready && is_inv_req) begin
                if (rq_axis_tready) begin
                    rq_axis_tvalid <= 1'b1;
                    rq_axis_tlast  <= 1'b1;

                    // only first beat used, single beat TLP
                    rq_axis_tkeep  <= {AXIS_DATA_WIDTH/8{1'b1}};
                    rq_axis_tdata  <= {AXIS_DATA_WIDTH{1'b0}};

                    // === Build Completion Descriptor ===
                    rq_axis_tdata[78:75]   <= 4'b1000;              // Message type
                    rq_axis_tdata[103:96]  <= tag;                  // Copy tag
                    rq_axis_tdata[111:104] <= INV_COMPLETE_CODE;    // Completion message code
                    rq_axis_tdata[114:112] <= 3'b000;               // Routing = 0 (adjust if needed)

                    rq_axis_tdata[74:64]   <= 11'd1;                // DW count = 1

                    rq_axis_tdata[79]      <= 1'b0;                 // Poison = 0
                    rq_axis_tdata[127]     <= 1'b0;                 // T9 = 0
                end
            end
        end
    end

endmodule
