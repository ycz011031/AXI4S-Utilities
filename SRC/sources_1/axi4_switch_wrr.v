`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/30/2025 06:46:43 AM
// Design Name: 
// Module Name: axi4_switch_custom
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: dual input single output axi4 switch 
// 
// Revision 0.01 - First draft
// Additional Comments:
//            1. current implementation assumes symetrical input and output AXI port
//            2. module assumes all ports operate on same clock
//            3. module does not support straddle mode 
//////////////////////////////////////////////////////////////////////////////////


module axi4_switch_wrr
#(  
    parameter  TDATA_L = 512, 
    parameter  TUSER_L = 81,
    parameter  TKEEP_L = 16,
    parameter  WEIGHT_L = 4
)(
    input wire       clk,
    input wire       rst_n,
    input wire [1:0] s_req_supress,

    input wire [WEIGHT_L-1 : 0] s0_weight,
    input wire [WEIGHT_L-1 : 0] s1_weight,
       
    // AXI4-Stream slave interface 0 (connects to master)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s0_i TDATA"  *)
    input  wire [TDATA_L-1 : 0] axi_s0_tdata_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s0_i TUSER"  *)
    input  wire [TUSER_L-1 : 0] axi_s0_tuser_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s0_i TLAST"  *)
    input  wire                 axi_s0_tlast_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s0_i TKEEP"  *)
    input  wire [TKEEP_L-1 : 0] axi_s0_tkeep_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s0_i TVALID" *)
    input  wire                 axi_s0_tvalid_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s0_i TREADY" *)
    output reg                  axi_s0_tready_o,
    
    // AXI4-Stream slave interface 1
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s1_i TDATA"  *)
    input  wire [TDATA_L-1 : 0] axi_s1_tdata_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s1_i TUSER"  *)
    input  wire [TUSER_L-1 : 0] axi_s1_tuser_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s1_i TLAST"  *)
    input  wire                 axi_s1_tlast_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s1_i TKEEP"  *)
    input  wire [TKEEP_L-1 : 0] axi_s1_tkeep_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s1_i TVALID" *)
    input  wire                 axi_s1_tvalid_i,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_s1_i TREADY" *)
    output reg                  axi_s1_tready_o,
    
    // AXI4-Stream master interface 0 (output)
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_m0_o TDATA"  *)
    output reg  [TDATA_L-1 : 0] axi_m0_tdata_o,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_m0_o TUSER"  *)
    output reg  [TUSER_L-1 : 0] axi_m0_tuser_o,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_m0_o TLAST"  *)
    output reg                  axi_m0_tlast_o,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_m0_o TKEEP"  *)
    output reg  [TKEEP_L-1 : 0] axi_m0_tkeep_o,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_m0_o TVALID" *)
    output reg                  axi_m0_tvalid_o,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 axi_m0_o TREADY" *)
    input  wire                 axi_m0_tready_i
);
    
    


localparam NUM_IN = 2;


reg [NUM_IN-1 : 0] port_busy; //set to high when tvalid is recieved while tready is high, set to low when tlast is recieved
reg [NUM_IN-1 : 0] port_order; // tracks the last transmission


wire [NUM_IN-1 :0] port_valid;
wire [NUM_IN-1 :0] port_last;
wire [NUM_IN-1 : 0] port_ready;

reg [NUM_IN-1 : 0] cur_state;

reg [WEIGHT_L : 0] s0_counter;
reg [WEIGHT_L : 0] s1_counter;

assign port_valid = {axi_s1_tvalid_i, axi_s0_tvalid_i};
assign port_last  = {axi_s1_tlast_i , axi_s0_tlast_i};
assign port_ready = {axi_s1_tready_o, axi_s0_tready_o};

// function arbitrate [1:0] results(input [WEIGHT_L : 0] counter0, counter1, input [WEIGHT_L-1 : 0] weight0, weight1);
//     wire [WEIGHT_L : 0] counter0_next, counter1_next;
//     wire [WEIGHT_L : 0] counter0_sel, counter1_sel;
//     assign counter0_next = (counter0 + weight0 > {1'b0,(WEIGHT_L-1){1'b1}})? {1'b0,(WEIGHT_L-1){1'b1}} : counter0 + weight0;
//     assign counter1_next = (counter1 + weight1 > {1'b0,(WEIGHT_L-1){1'b1}})? {1'b0,(WEIGHT_L-1){1'b1}} : counter1 + weight1;
//     assign counter0_sel = counter0_next - (counter0_next + counter1_next);
//     assign counter1_sel = counter1_next - (counter0_next + counter1_next);
//     begin
//         if ((signed)counter0_next >= (signed)counter1_next)begin
            
//         end
        
        


//     end 
// endfunction



/////////////////////////////////////////////////////////////////////////////////////WRR Aribitration logic///////////////////////////////////////////////////////////////////////////////////////////
reg  [WEIGHT_L : 0] counter0_next, counter1_next;
wire [WEIGHT_L : 0] counter0_add_, counter1_add_;
wire [WEIGHT_L : 0] counter0_add, counter1_add;
wire [WEIGHT_L : 0] counter0_sel_, counter1_sel_;
wire [WEIGHT_L : 0] counter0_sel, counter1_sel;
reg  [1:0] arbit_result;

assign counter0_add_ = s0_counter + s0_weight;
assign counter1_add_ = s1_counter + s1_weight;
assign counter0_add = ((s0_counter[WEIGHT_L] == 1'b0) &&  (counter0_add_[WEIGHT_L] == 1'b1)) ? {1'b0, {WEIGHT_L{1'b1}}} : counter0_add_;
assign counter1_add = ((s1_counter[WEIGHT_L] == 1'b0) &&  (counter1_add_[WEIGHT_L] == 1'b1)) ? {1'b0, {WEIGHT_L{1'b1}}} : counter1_add_;
assign counter0_sel_ = counter0_add - (counter0_add + counter1_add);
assign counter1_sel_ = counter1_add - (counter0_add + counter1_add);
assign counter0_sel = ((counter0_add[WEIGHT_L] == 1'b1) && (counter0_sel_[WEIGHT_L] == 1'b0)) ? {1'b1, {WEIGHT_L{1'b0}}} : counter0_sel_;
assign counter1_sel = ((counter1_add[WEIGHT_L] == 1'b1) && (counter1_sel_[WEIGHT_L] == 1'b0)) ? {1'b1, {WEIGHT_L{1'b0}}} : counter1_sel_;

always @(*)begin
    if ($signed(counter0_add) >= $signed(counter1_add))begin
        counter0_next <= counter0_sel;
        counter1_next <= counter1_add;
        arbit_result  <= 2'b01;
    end else begin
        counter0_next <= counter0_add;
        counter1_next <= counter1_sel;
        arbit_result  <= 2'b10;
    end
end
/////////////////////////////////////////////////////////////////////////////////////End of WRR Aribitration logic///////////////////////////////////////////////////////////////////////////////////////////




always @(*) begin
    axi_s0_tready_o = 1'b0;
    axi_s1_tready_o = 1'b0;
    axi_m0_tlast_o  = 1'b0;
    axi_m0_tvalid_o = 1'b0;
    axi_m0_tdata_o  = {TDATA_L{1'bx}};
    axi_m0_tuser_o  = {TUSER_L{1'b0}};
    axi_m0_tkeep_o  = {TKEEP_L{1'bx}};
    case (cur_state)
        2'b00: begin
            case (port_valid)
                2'b01 : begin
                    axi_s0_tready_o = axi_m0_tready_i;
                    axi_m0_tdata_o  = axi_s0_tdata_i;
                    axi_m0_tuser_o  = axi_s0_tuser_i;
                    axi_m0_tlast_o  = axi_s0_tlast_i;
                    axi_m0_tkeep_o  = axi_s0_tkeep_i;
                    axi_m0_tvalid_o = axi_s0_tvalid_i;
                end
                2'b10 : begin
                    axi_s1_tready_o = axi_m0_tready_i;
                    axi_m0_tdata_o  = axi_s1_tdata_i;
                    axi_m0_tuser_o  = axi_s1_tuser_i;
                    axi_m0_tlast_o  = axi_s1_tlast_i;
                    axi_m0_tkeep_o  = axi_s1_tkeep_i;
                    axi_m0_tvalid_o = axi_s1_tvalid_i;
                end
                2'b11 : begin
                    axi_s0_tready_o = axi_m0_tready_i & port_order[0];
                    axi_s1_tready_o = axi_m0_tready_i & port_order[1];
                    axi_m0_tdata_o  = (port_order[0]) ?  axi_s0_tdata_i : axi_s1_tdata_i;
                    axi_m0_tuser_o  = (port_order[0]) ?  axi_s0_tuser_i : axi_s1_tuser_i;
                    axi_m0_tlast_o  = (port_order[0]) ?  axi_s0_tlast_i : axi_s1_tlast_i;
                    axi_m0_tkeep_o  = (port_order[0]) ?  axi_s0_tkeep_i : axi_s1_tkeep_i;
                    axi_m0_tvalid_o = (port_order[0]) ?  axi_s0_tvalid_i: axi_s1_tvalid_i;
                end
             endcase
        end
        2'b01: begin
            axi_s0_tready_o = axi_m0_tready_i;
            axi_m0_tdata_o  = axi_s0_tdata_i;
            axi_m0_tuser_o  = axi_s0_tuser_i;
            axi_m0_tlast_o  = axi_s0_tlast_i;
            axi_m0_tkeep_o  = axi_s0_tkeep_i;
            axi_m0_tvalid_o = axi_s0_tvalid_i;
         end
         2'b10 : begin
            axi_s1_tready_o = axi_m0_tready_i;
            axi_m0_tdata_o  = axi_s1_tdata_i;
            axi_m0_tuser_o  = axi_s1_tuser_i;
            axi_m0_tlast_o  = axi_s1_tlast_i;
            axi_m0_tkeep_o  = axi_s1_tkeep_i;
            axi_m0_tvalid_o = axi_s1_tvalid_i;
         end
         default : begin
            axi_s0_tready_o = 1'b0;
            axi_s1_tready_o = 1'b0;
         end            
      endcase                            
end

always @(posedge clk) begin
    if (!rst_n) begin
        port_busy  <= 2'b00;
        port_order <= 2'b01;
        cur_state  <= 2'b00;
        s0_counter <= {WEIGHT_L{1'b0}};
        s1_counter <= {WEIGHT_L{1'b0}};
        
    end else begin
        port_order <= arbit_result;
        case(cur_state)
            2'b00 : begin
                if      (port_valid == 2'b01) begin
                    if (port_last[0] != 1'b1 && port_ready[0]) cur_state <= 2'b01;
                end
                else if (port_valid == 2'b10 ) begin
                    if (port_last[1] != 1'b1 && port_ready[1]) cur_state <= 2'b10;
                end
                else if (port_valid == 2'b11) begin // Might need to add logic to check ready signal
                    if (!(port_order & port_last & port_ready))begin
                        cur_state <= port_order;
                        s0_counter <= counter0_next;
                        s1_counter <= counter1_next;
                    end 
                end
            end
            2'b01 : begin
                if (port_last[0] && port_ready[0]) cur_state  <= 2'b00;
            end
            2'b10 : begin
                if (port_last[1] && port_ready[1]) cur_state  <= 2'b00;
            end
            default : cur_state <= 2'b11;
        endcase
      end
   end        
                       
endmodule
