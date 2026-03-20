module pcie_type_tracker_top #(
    parameter integer AXIS_DATA_WIDTH = 512,
    parameter integer COUNTER_VALUE_WIDTH = 8  // 8 for uint8, 16 for uint16
)
(
    input  wire                          clk,
    input  wire                          rst,

    // CQ Interface (Completer Request) - TUSER=229
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_cq_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_cq_tkeep,
    input  wire                          s_axis_cq_tvalid,
    input  wire                          s_axis_cq_tlast,
    input  wire [228:0]                  s_axis_cq_tuser,
    output wire                          s_axis_cq_tready,
    
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_cq_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_cq_tkeep,
    output wire                          m_axis_cq_tvalid,
    output wire                          m_axis_cq_tlast,
    output wire [228:0]                  m_axis_cq_tuser,
    input  wire                          m_axis_cq_tready,

    // CC Interface (Completer Completion) - TUSER=81
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_cc_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_cc_tkeep,
    input  wire                          s_axis_cc_tvalid,
    input  wire                          s_axis_cc_tlast,
    input  wire [80:0]                   s_axis_cc_tuser,
    output wire                          s_axis_cc_tready,
    
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_cc_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_cc_tkeep,
    output wire                          m_axis_cc_tvalid,
    output wire                          m_axis_cc_tlast,
    output wire [80:0]                   m_axis_cc_tuser,
    input  wire                          m_axis_cc_tready,

    // RQ Interface (Requester Request) - TUSER=137
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_rq_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_rq_tkeep,
    input  wire                          s_axis_rq_tvalid,
    input  wire                          s_axis_rq_tlast,
    input  wire [136:0]                  s_axis_rq_tuser,
    output wire                          s_axis_rq_tready,
    
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_rq_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_rq_tkeep,
    output wire                          m_axis_rq_tvalid,
    output wire                          m_axis_rq_tlast,
    output wire [136:0]                  m_axis_rq_tuser,
    input  wire                          m_axis_rq_tready,

    // RC Interface (Requester Completion) - TUSER=161
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_rc_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_rc_tkeep,
    input  wire                          s_axis_rc_tvalid,
    input  wire                          s_axis_rc_tlast,
    input  wire [160:0]                  s_axis_rc_tuser,
    output wire                          s_axis_rc_tready,
    
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_rc_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_rc_tkeep,
    output wire                          m_axis_rc_tvalid,
    output wire                          m_axis_rc_tlast,
    output wire [160:0]                  m_axis_rc_tuser,
    input  wire                          m_axis_rc_tready,

    // ILA interface
    output wire [$clog2(5)-1:0] ila_channel_id,
    output wire [COUNTER_VALUE_WIDTH-1:0] ila_counter_value,
    output wire                          ila_counter_valid
);

    localparam NUM_UNITS = 4;

    // Hub to unit interfaces
    wire [NUM_UNITS-1:0] read_enable;
    wire [1:0]           channel_select;
    wire [2:0]           counter_id [NUM_UNITS-1:0];
    wire [COUNTER_VALUE_WIDTH-1:0] requester_type [NUM_UNITS-1:0];

    // Tag lookup interface between CQ and CC units
    wire [7:0]           cq_cc_completer_tag;
    wire                 cq_cc_completer_tag_valid;
    wire [4:0]           cq_cc_completer_type;

    // Tag lookup interface between RQ and RC units
    wire [7:0]           rq_rc_completer_tag;
    wire                 rq_rc_completer_tag_valid;
    wire [4:0]           rq_rc_completer_type;

    // Mux for counter_id and requester_type based on channel_select
    wire [2:0] muxed_counter_id = counter_id[channel_select];
    wire [COUNTER_VALUE_WIDTH-1:0] muxed_requester_type = requester_type[channel_select];

    // ============================================================
    // Unit 0: CQ Requester Counter (TUSER=229)
    // ============================================================
    pcie_requester_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(229),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(1)
    ) cq_requester_unit (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(s_axis_cq_tdata),
        .s_axis_tkeep(s_axis_cq_tkeep),
        .s_axis_tvalid(s_axis_cq_tvalid),
        .s_axis_tlast(s_axis_cq_tlast),
        .s_axis_tuser(s_axis_cq_tuser),
        .s_axis_tready(s_axis_cq_tready),
        
        .m_axis_tdata(m_axis_cq_tdata),
        .m_axis_tkeep(m_axis_cq_tkeep),
        .m_axis_tvalid(m_axis_cq_tvalid),
        .m_axis_tlast(m_axis_cq_tlast),
        .m_axis_tuser(m_axis_cq_tuser),
        .m_axis_tready(m_axis_cq_tready),
        
        .completer_tag(cq_cc_completer_tag),
        .completer_tag_valid(cq_cc_completer_tag_valid),
        .completer_type(cq_cc_completer_type),
        
        .read_enable(read_enable[0]),
        .counter_id(counter_id[0]),
        .requester_type(requester_type[0])
    );

    // ============================================================
    // Unit 1: CC Completer Counter (TUSER=81)
    // ============================================================
    pcie_completer_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(81),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(2)
    ) cc_completer_unit (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(s_axis_cc_tdata),
        .s_axis_tkeep(s_axis_cc_tkeep),
        .s_axis_tvalid(s_axis_cc_tvalid),
        .s_axis_tlast(s_axis_cc_tlast),
        .s_axis_tuser(s_axis_cc_tuser),
        .s_axis_tready(s_axis_cc_tready),
        
        .m_axis_tdata(m_axis_cc_tdata),
        .m_axis_tkeep(m_axis_cc_tkeep),
        .m_axis_tvalid(m_axis_cc_tvalid),
        .m_axis_tlast(m_axis_cc_tlast),
        .m_axis_tuser(m_axis_cc_tuser),
        .m_axis_tready(m_axis_cc_tready),
        
        .completer_tag(cq_cc_completer_tag),
        .completer_tag_valid(cq_cc_completer_tag_valid),
        .completer_type(cq_cc_completer_type),
        
        .read_enable(read_enable[1]),
        .counter_id(counter_id[1]),
        .requester_type(requester_type[1])
    );

    // ============================================================
    // Unit 2: RQ Requester Counter (TUSER=137)
    // ============================================================
    pcie_requester_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(137),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(3)
    ) rq_requester_unit (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(s_axis_rq_tdata),
        .s_axis_tkeep(s_axis_rq_tkeep),
        .s_axis_tvalid(s_axis_rq_tvalid),
        .s_axis_tlast(s_axis_rq_tlast),
        .s_axis_tuser(s_axis_rq_tuser),
        .s_axis_tready(s_axis_rq_tready),
        
        .m_axis_tdata(m_axis_rq_tdata),
        .m_axis_tkeep(m_axis_rq_tkeep),
        .m_axis_tvalid(m_axis_rq_tvalid),
        .m_axis_tlast(m_axis_rq_tlast),
        .m_axis_tuser(m_axis_rq_tuser),
        .m_axis_tready(m_axis_rq_tready),
        
        .completer_tag(rq_rc_completer_tag),
        .completer_tag_valid(rq_rc_completer_tag_valid),
        .completer_type(rq_rc_completer_type),
        
        .read_enable(read_enable[2]),
        .counter_id(counter_id[2]),
        .requester_type(requester_type[2])
    );

    // ============================================================
    // Unit 3: RC Completer Counter (TUSER=161)
    // ============================================================
    pcie_completer_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(161),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(4)
    ) rc_completer_unit (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(s_axis_rc_tdata),
        .s_axis_tkeep(s_axis_rc_tkeep),
        .s_axis_tvalid(s_axis_rc_tvalid),
        .s_axis_tlast(s_axis_rc_tlast),
        .s_axis_tuser(s_axis_rc_tuser),
        .s_axis_tready(s_axis_rc_tready),
        
        .m_axis_tdata(m_axis_rc_tdata),
        .m_axis_tkeep(m_axis_rc_tkeep),
        .m_axis_tvalid(m_axis_rc_tvalid),
        .m_axis_tlast(m_axis_rc_tlast),
        .m_axis_tuser(m_axis_rc_tuser),
        .m_axis_tready(m_axis_rc_tready),
        
        .completer_tag(rq_rc_completer_tag),
        .completer_tag_valid(rq_rc_completer_tag_valid),
        .completer_type(rq_rc_completer_type),
        
        .read_enable(read_enable[3]),
        .counter_id(counter_id[3]),
        .requester_type(requester_type[3])
    );

    // ============================================================
    // Hub Instance
    // ============================================================
    pcie_type_tracker_hub #(
        .NUM_UNITS(NUM_UNITS),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH)
    ) hub (
        .clk(clk),
        .rst(rst),
        
        .read_enable(read_enable),
        .channel_select(channel_select),
        .counter_id(muxed_counter_id),
        .requester_type(muxed_requester_type),
        
        .ila_channel_id(ila_channel_id),
        .ila_counter_value(ila_counter_value),
        .ila_counter_valid(ila_counter_valid)
    );

endmodule

// ============================================================
// 2-Port PCIe Type Tracker Top Module
// ============================================================
module pcie_type_tracker_2port #(
    parameter integer AXIS_DATA_WIDTH = 512,
    parameter integer COUNTER_VALUE_WIDTH = 8,  // 8 for uint8, 16 for uint16
    parameter INTERFACE_TYPE = "CQ_CC"  // "CQ_CC" or "RQ_RC"
)
(
    input  wire                          clk,
    input  wire                          rst,

    // Requester Interface (CQ or RQ depending on INTERFACE_TYPE)
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_req_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_req_tkeep,
    input  wire                          s_axis_req_tvalid,
    input  wire                          s_axis_req_tlast,
    input  wire [228:0]                  s_axis_req_tuser,  // Max width (CQ=229, RQ=137)
    output wire                          s_axis_req_tready,
    
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_req_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_req_tkeep,
    output wire                          m_axis_req_tvalid,
    output wire                          m_axis_req_tlast,
    output wire [228:0]                  m_axis_req_tuser,
    input  wire                          m_axis_req_tready,

    // Completer Interface (CC or RC depending on INTERFACE_TYPE)
    input  wire [AXIS_DATA_WIDTH-1:0]    s_axis_cpl_tdata,
    input  wire [AXIS_DATA_WIDTH/8-1:0]  s_axis_cpl_tkeep,
    input  wire                          s_axis_cpl_tvalid,
    input  wire                          s_axis_cpl_tlast,
    input  wire [160:0]                  s_axis_cpl_tuser,  // Max width (CC=81, RC=161)
    output wire                          s_axis_cpl_tready,
    
    output wire [AXIS_DATA_WIDTH-1:0]    m_axis_cpl_tdata,
    output wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_cpl_tkeep,
    output wire                          m_axis_cpl_tvalid,
    output wire                          m_axis_cpl_tlast,
    output wire [160:0]                  m_axis_cpl_tuser,
    input  wire                          m_axis_cpl_tready,

    // ILA interface
    output wire [$clog2(3)-1:0]          ila_channel_id,  // 0=idle, 1=req, 2=cpl
    output wire [COUNTER_VALUE_WIDTH-1:0] ila_counter_value,
    output wire                          ila_counter_valid
);

    localparam NUM_UNITS = 2;
    
    // TUSER widths based on interface type
    localparam REQ_TUSER_WIDTH = (INTERFACE_TYPE == "CQ_CC") ? 229 : 137;
    localparam CPL_TUSER_WIDTH = (INTERFACE_TYPE == "CQ_CC") ? 81  : 161;
    
    // Unit IDs
    localparam REQ_UNIT_ID = 1;
    localparam CPL_UNIT_ID = 2;

    // Hub to unit interfaces
    wire [NUM_UNITS-1:0] read_enable;
    wire                 channel_select;  // Only 1 bit for 2 units
    wire [2:0]           counter_id_req, counter_id_cpl;
    wire [COUNTER_VALUE_WIDTH-1:0] requester_type_req, requester_type_cpl;

    // Tag lookup interface between requester and completer
    wire [7:0]           completer_tag;
    wire                 completer_tag_valid;
    wire [4:0]           completer_type;

    // Mux for counter signals
    wire [2:0] muxed_counter_id = channel_select ? counter_id_cpl : counter_id_req;
    wire [COUNTER_VALUE_WIDTH-1:0] muxed_requester_type = channel_select ? requester_type_cpl : requester_type_req;

    // ============================================================
    // Unit 0: Requester Counter (CQ or RQ)
    // ============================================================
    pcie_requester_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(REQ_TUSER_WIDTH),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(REQ_UNIT_ID)
    ) requester_unit (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(s_axis_req_tdata),
        .s_axis_tkeep(s_axis_req_tkeep),
        .s_axis_tvalid(s_axis_req_tvalid),
        .s_axis_tlast(s_axis_req_tlast),
        .s_axis_tuser(s_axis_req_tuser[REQ_TUSER_WIDTH-1:0]),
        .s_axis_tready(s_axis_req_tready),
        
        .m_axis_tdata(m_axis_req_tdata),
        .m_axis_tkeep(m_axis_req_tkeep),
        .m_axis_tvalid(m_axis_req_tvalid),
        .m_axis_tlast(m_axis_req_tlast),
        .m_axis_tuser(m_axis_req_tuser[REQ_TUSER_WIDTH-1:0]),
        .m_axis_tready(m_axis_req_tready),
        
        .completer_tag(completer_tag),
        .completer_tag_valid(completer_tag_valid),
        .completer_type(completer_type),
        
        .read_enable(read_enable[0]),
        .counter_id(counter_id_req),
        .requester_type(requester_type_req)
    );
    
    // Tie off unused upper bits of m_axis_req_tuser
    assign m_axis_req_tuser[228:REQ_TUSER_WIDTH] = {(229-REQ_TUSER_WIDTH){1'b0}};

    // ============================================================
    // Unit 1: Completer Counter (CC or RC)
    // ============================================================
    pcie_completer_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(CPL_TUSER_WIDTH),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH),
        .unit_id(CPL_UNIT_ID)
    ) completer_unit (
        .clk(clk),
        .rst(rst),
        
        .s_axis_tdata(s_axis_cpl_tdata),
        .s_axis_tkeep(s_axis_cpl_tkeep),
        .s_axis_tvalid(s_axis_cpl_tvalid),
        .s_axis_tlast(s_axis_cpl_tlast),
        .s_axis_tuser(s_axis_cpl_tuser[CPL_TUSER_WIDTH-1:0]),
        .s_axis_tready(s_axis_cpl_tready),
        
        .m_axis_tdata(m_axis_cpl_tdata),
        .m_axis_tkeep(m_axis_cpl_tkeep),
        .m_axis_tvalid(m_axis_cpl_tvalid),
        .m_axis_tlast(m_axis_cpl_tlast),
        .m_axis_tuser(m_axis_cpl_tuser[CPL_TUSER_WIDTH-1:0]),
        .m_axis_tready(m_axis_cpl_tready),
        
        .completer_tag(completer_tag),
        .completer_tag_valid(completer_tag_valid),
        .completer_type(completer_type),
        
        .read_enable(read_enable[1]),
        .counter_id(counter_id_cpl),
        .requester_type(requester_type_cpl)
    );
    
    // Tie off unused upper bits of m_axis_cpl_tuser
    assign m_axis_cpl_tuser[160:CPL_TUSER_WIDTH] = {(161-CPL_TUSER_WIDTH){1'b0}};

    // ============================================================
    // Hub Instance
    // ============================================================
    pcie_type_tracker_hub #(
        .NUM_UNITS(NUM_UNITS),
        .COUNTER_VALUE_WIDTH(COUNTER_VALUE_WIDTH)
    ) hub (
        .clk(clk),
        .rst(rst),
        
        .read_enable(read_enable),
        .channel_select(channel_select),
        .counter_id(muxed_counter_id),
        .requester_type(muxed_requester_type),
        
        .ila_channel_id(ila_channel_id),
        .ila_counter_value(ila_counter_value),
        .ila_counter_valid(ila_counter_valid)
    );

endmodule
