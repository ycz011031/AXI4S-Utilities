module pcie_cq_ats_snoop #
(
    parameter integer AXIS_DATA_WIDTH  = 512,
    parameter integer AXIS_TUSER_WIDTH = 229,
    parameter integer RQ_AXIS_TUSER_W  = 183
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
    output reg  [RQ_AXIS_TUSER_W-1:0]    rq_axis_tuser,
    input  wire                          rq_axis_tready,
    output reg                           rq_axis_tlast,

    // Debug outputs (to ILA)
    output reg                           ats_hit,       // pulse
    output reg [7:0]                     ats_tag,
    output reg [7:0]                     ats_msg_code,
    output reg [2:0]                     ats_msg_routing,
    output reg [AXIS_DATA_WIDTH-1:0]     ats_tdata,
    output reg [AXIS_DATA_WIDTH/8-1:0]   ats_tkeep,
    output reg [AXIS_TUSER_WIDTH-1:0]    ats_tuser
);



    // ============================================================
    // PCIe TLP Field Extraction (CQ descriptor formatting)
    // ============================================================
    wire [7:0] msg_code  = s_axis_tdata[111:104];
    wire [2:0] routing   = s_axis_tdata[114:112];
    wire [7:0] tag       = s_axis_tdata[103:96];
    wire [3:0] req_type  = s_axis_tdata[78:75];
    wire [1:0] sop       = s_axis_tuser[81:80]; // Start of Packet indicator
    wire       is_sop    = (sop != 2'b00);      // SOP when != 0

    wire is_message_tlp  = (req_type[3:2] == 2'b10); // Type = 10xx
    wire is_ats_msg      = (req_type == 4'b1110);    // Per PCIe table

    // identify invalidation requests (Message Code 0x14 or 0x15)
    wire is_inv_req = (msg_code == 8'h14) || (msg_code == 8'h15);

    // invalidation completion code (adjust if needed)
    localparam [7:0] INV_COMPLETE_CODE = 8'h02;
    
    // ============================================================
    // Pass-through (transparent) path
    // ============================================================
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tvalid = is_ats_msg ? 1'b0 : s_axis_tvalid;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tuser  = s_axis_tuser;
    assign s_axis_tready = m_axis_tready;

    // ============================================================
    // ATS Snooper
    // ============================================================
    always @(posedge clk) begin
        if (!rst) begin
            ats_hit        <= 1'b0;
            ats_tag        <= 8'd0;
            ats_msg_code   <= 8'd0;
            ats_msg_routing<= 3'd0;
            ats_tdata      <= {AXIS_DATA_WIDTH{1'b0}};
            ats_tkeep      <= {AXIS_DATA_WIDTH/8{1'b0}};
            ats_tuser      <= {AXIS_TUSER_WIDTH{1'b0}};
        end else begin
            //ats_hit <= 1'b0;           
            if (s_axis_tvalid & s_axis_tready & is_sop & is_ats_msg) begin
                ats_hit         <= 1'b1;
                ats_tag         <= tag;
                ats_msg_code    <= msg_code;
                ats_msg_routing <= routing;
                ats_tdata       <= s_axis_tdata;
                ats_tkeep       <= s_axis_tkeep;
                ats_tuser       <= s_axis_tuser;               
            end
            else ats_hit <= 1'b0;
        end
    end
    // ============================================================
    // Invalidation Completion Generator (RQ AXIS)
    // ============================================================
    always @(posedge clk) begin
        if (!rst) begin
            rq_axis_tvalid <= 1'b0;
            rq_axis_tdata  <= {AXIS_DATA_WIDTH{1'b0}};
            rq_axis_tkeep  <= {AXIS_DATA_WIDTH/8{1'b0}};
            rq_axis_tlast  <= 1'b0;
            rq_axis_tuser  <= {RQ_AXIS_TUSER_W{1'b0}};
        end else begin
            if (rq_axis_tvalid && rq_axis_tready) begin
                rq_axis_tvalid <= 1'b0; 
                rq_axis_tlast <= 1'b0; 
                rq_axis_tdata <= {AXIS_DATA_WIDTH{1'b0}}; 
                rq_axis_tkeep <= {AXIS_DATA_WIDTH/8{1'b0}}; 
                rq_axis_tuser <= {RQ_AXIS_TUSER_W{1'b0}};
            end else if (ats_hit) begin
                rq_axis_tvalid <= 1'b1; 
                rq_axis_tlast <= 1'b1;
                rq_axis_tkeep <= 64'h0000_0000_0000_FFFF; // Only first 16 bytes (descriptor) are valid - no payload for messages
                
                // RQ TLP TDATA Assignments
                rq_axis_tdata[63:0]    <= {{32{1'b0}},32'h01000096}; // TODO: DW2 and DW3 content (Hard coded now, Destination ID and iTag Vector needs to be dynamically assigned based on received invalidation request)
                rq_axis_tdata[74:64]   <= 11'd0; // Dword Count = 0 (Completion No Payload)
                rq_axis_tdata[78:75]   <= 4'b1110; //Request Type = Message (ATS Invalidation Completion)
                rq_axis_tdata[79]      <= 1'b0; // Poisoned Request = 0
                rq_axis_tdata[87:80]   <= 8'h00; // Requester Function/Device Number = 0 (TODO: Hardcoded, needs to be dynamically assigned based on device ID (destination ID) of invalidation request)
                rq_axis_tdata[95:88]   <= 8'h98; // Requester Bus Number = 0 (TODO: Verify against IP)
                rq_axis_tdata[103:96]  <= 8'd0; // Tag - copy from received invalidation request
                rq_axis_tdata[111:104] <= INV_COMPLETE_CODE; // Message Code - Invalidation Completion code TODO: Adjust if needed
                rq_axis_tdata[114:112] <= 3'b010; // Message Routing - Route to Root Complex (0) TODO: Adjust if needed
                rq_axis_tdata[119:115] <= 5'd0; // Reserved = 0
                rq_axis_tdata[120]     <= 1'b1; // Requester ID Enable/T8 = 0 (TODO: Currently Hardcoded, needs to be dynamical assigned based on device ID (destination ID) of invalidation request)
                rq_axis_tdata[123:121] <= 3'd0; // Transaction Class = 0 
                rq_axis_tdata[126:124] <= 3'd0; // Attributes = 0 (No Snoop=0, Relaxed Ordering=0, ID-Based Ordering=0)
                rq_axis_tdata[127]     <= 1'b0; // T9 = 0

                //RQ TLP TUSER Assignments
                rq_axis_tuser[7:0]     <= 8'h00; // first_be[7:0] = 0 (not applicable for messages) (TODO: Verify)
                rq_axis_tuser[15:8]    <= 8'h00; // last_be[15:8] = 0 (not applicable for messages)
                rq_axis_tuser[21:20]   <= 2'b01; // is_sop[21:20] = 01 (single TLP starting at byte lane 0)
                rq_axis_tuser[23:22]   <= 2'b00; // is_sop0_ptr[23:22] = 00 (starts at byte lane 0)
                rq_axis_tuser[27:26]   <= 2'b01; // is_eop[27:26] = 01 (single TLP ending)
                rq_axis_tuser[31:28]   <= 4'd0; // is_eop0_ptr[31:28] = 0 (last Dword offset = 0, descriptor only)
                rq_axis_tuser[36]      <= 1'b0; // discontinue[36] = 0
            end
        end
    end

endmodule
