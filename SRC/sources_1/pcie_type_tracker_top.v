module pcie_type_tracker_top #(
    parameter integer AXIS_DATA_WIDTH = 512
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
    output wire [1:0]                    ila_channel_id,
    output wire [7:0]                    ila_counter_value
);

    localparam NUM_UNITS = 4;

    // Hub to unit interfaces
    wire [NUM_UNITS-1:0] read_enable;
    wire [1:0]           channel_select;
    wire [2:0]           counter_id [NUM_UNITS-1:0];
    wire [7:0]           requester_type [NUM_UNITS-1:0];

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
    wire [7:0] muxed_requester_type = requester_type[channel_select];

    // ============================================================
    // Unit 0: CQ Requester Counter (TUSER=229)
    // ============================================================
    pcie_requester_type_counter_unit #(
        .AXIS_DATA_WIDTH(AXIS_DATA_WIDTH),
        .AXIS_TUSER_WIDTH(229),
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
        .NUM_UNITS(NUM_UNITS)
    ) hub (
        .clk(clk),
        .rst(rst),
        
        .read_enable(read_enable),
        .channel_select(channel_select),
        .counter_id(muxed_counter_id),
        .requester_type(muxed_requester_type),
        
        .ila_channel_id(ila_channel_id),
        .ila_counter_value(ila_counter_value)
    );

endmodule
