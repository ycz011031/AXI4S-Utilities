// =============================================================================
// axil4_regfile - Configurable AXI4-Lite register file
// -----------------------------------------------------------------------------
// A generic, parameterised control/status register block intended to sit on an
// AXI4-Lite control port (e.g. from the PCIe BAR / MicroBlaze / Zynq GP port)
// and fan out to the datapath modules in this project (axi4_mwr_batch
// thresholds, axi4_telemetry configuration, switch arbitration weights, ...).
//
// The register map is described entirely by parameters - no RTL edits are
// needed to add, remove or re-type a register:
//
//   NUM_REGS             how many DATA_WIDTH-wide registers exist
//   REG_RW_MASK[i]       1 = register i is software read/write (has storage)
//                        0 = register i is read-only, its read data comes
//                            straight from hw_status[i]
//   REG_W1C_MASK[i]      1 = register i is write-1-to-clear (sticky status /
//                            interrupt flags).  Hardware sets bits through
//                            hw_set[i]; software clears them by writing 1s.
//                            Implies storage (RW_MASK need not also be set).
//   REG_SELF_CLEAR_MASK  1 = register i reverts to its REG_INIT value one
//                            clock after a software write (command / "go"
//                            strobe registers).  Implies storage.
//   REG_INIT             flattened {NUM_REGS x DATA_WIDTH} reset values
//
// All four are written as hex literals.  The masks are one bit per register,
// widened to a whole number of bytes (MASK_W = ceil(NUM_REGS/8)*8) so the hex
// digits always line up - 2 digits per byte - whatever NUM_REGS happens to be;
// mask bits above NUM_REGS-1 are padding and are ignored.  REG_INIT is one
// DATA_WIDTH field per register with register 0 in the least significant
// position, so the literal reads {regN-1, ..., reg1, reg0} left to right.
//
// Registers are mapped densely starting at offset 0, one register per
// DATA_WIDTH/8 bytes: register i lives at byte offset i*(DATA_WIDTH/8).
// Accesses past the last register return DECERR (see DECODE_ERR_EN).
//
// Hardware-side ports are flattened vectors (Verilog-2001 has no 2-D ports);
// slice them as  hw_status[i*DATA_WIDTH +: DATA_WIDTH].
//
// Example - 4 registers: 0 = RW control (reset 0x0000_0001), 1 = RW threshold
// (reset 0x80), 2 = RO status, 3 = W1C error flags.  MASK_W = 8, INIT_W = 128:
//
//   axil4_regfile #(
//       .NUM_REGS            (4),
//       .C_S_AXI_ADDR_WIDTH  (4),
//       .REG_RW_MASK         (8'h03),   // regs 1, 0
//       .REG_W1C_MASK        (8'h08),   // reg 3
//       .REG_SELF_CLEAR_MASK (8'h00),
//       .REG_INIT            (128'h00000000_00000000_00000080_00000001)
//   ) u_regs ( ... );      //   reg3     reg2     reg1     reg0
//
// Protocol notes:
//   * AW and W are accepted independently and may arrive in either order; the
//     write commits (and B is issued) once both are present.  One outstanding
//     write and one outstanding read are supported - sufficient for AXI4-Lite,
//     which has no bursts and no transaction IDs.
//   * awprot/arprot are ignored (no privilege/security filtering).
//
// Interface identity:
//   Every bus port carries an X_INTERFACE_INFO attribute binding it to the
//   "S_AXI" bus interface (xilinx.com:interface:aximm:1.0), so Vivado does not
//   have to guess from port names - the module drops straight into a block
//   design as an RTL module reference, and packages as IP with the interface
//   already formed.  The X_INTERFACE_PARAMETER block on s_axi_awaddr declares
//   PROTOCOL AXI4LITE explicitly and states which optional AXI signals exist
//   (HAS_*), so no AXI4-Full features are ever inferred.  Slave direction is
//   taken from the port directions; DATA_WIDTH / ADDR_WIDTH are deliberately
//   left out of the parameter string so Vivado derives them from the actual
//   (parameterised) port widths instead of a stale hard-coded value.
//   s_axi_aclk carries ASSOCIATED_BUSIF / ASSOCIATED_RESET so the clock and
//   the active-low reset are tied to the interface.
// =============================================================================

module axil4_regfile #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,   // 32 or 64
    parameter integer C_S_AXI_ADDR_WIDTH = 6,    // must cover NUM_REGS*(DATA_WIDTH/8) bytes
    parameter integer NUM_REGS           = 16,

    // ---- Derived widths - DO NOT OVERRIDE ---------------------------------
    // MASK_W rounds NUM_REGS up to a whole byte so every mask is a tidy hex
    // literal (2 hex digits per byte).  Bits above NUM_REGS-1 are padding.
    parameter integer MASK_W = ((NUM_REGS + 7) / 8) * 8,
    parameter integer INIT_W = NUM_REGS * C_S_AXI_DATA_WIDTH,

    // ---- Per-register type masks, hex: bit i describes register i ---------
    parameter [MASK_W-1:0] REG_RW_MASK         = {MASK_W{1'b1}},
    parameter [MASK_W-1:0] REG_W1C_MASK        = {MASK_W{1'b0}},
    parameter [MASK_W-1:0] REG_SELF_CLEAR_MASK = {MASK_W{1'b0}},

    // ---- Reset values, hex: one DATA_WIDTH field per register -------------
    // Register i occupies REG_INIT[i*DATA_WIDTH +: DATA_WIDTH], so register 0
    // is the least significant field: {regN-1, ..., reg1, reg0}.
    parameter [INIT_W-1:0] REG_INIT = {INIT_W{1'b0}},

    // 1 = unmapped address returns DECERR, 0 = returns OKAY (reads return 0)
    parameter integer DECODE_ERR_EN = 1,
    // 1 = write to a read-only register returns SLVERR, 0 = silently ignored
    parameter integer RO_WR_ERR_EN  = 0
)(
    // ---- Clock and reset (bound to the S_AXI interface) -------------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *)
    // FREQ_HZ is a placeholder: Vivado overwrites it during clock propagation
    // once the module is connected in a block design.
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI_ACLK, ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 100000000, PHASE 0.0, INSERT_VIP 0" *)
    input  wire                              s_axi_aclk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI_ARESETN, POLARITY ACTIVE_LOW, INSERT_VIP 0" *)
    input  wire                              s_axi_aresetn,

    // ---- AXI4-Lite write address channel ----------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, READ_WRITE_MODE READ_WRITE, ID_WIDTH 0, AWUSER_WIDTH 0, WUSER_WIDTH 0, BUSER_WIDTH 0, ARUSER_WIDTH 0, RUSER_WIDTH 0, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 1, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 0, MAX_BURST_LENGTH 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, INSERT_VIP 0" *)
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
    input  wire [2:0]                        s_axi_awprot,   // ignored
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire                              s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire                              s_axi_awready,

    // ---- AXI4-Lite write data channel -------------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0]   s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire                              s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire                              s_axi_wready,

    // ---- AXI4-Lite write response channel ---------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output reg  [1:0]                        s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output reg                               s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire                              s_axi_bready,

    // ---- AXI4-Lite read address channel -----------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
    input  wire [2:0]                        s_axi_arprot,   // ignored
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire                              s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire                              s_axi_arready,

    // ---- AXI4-Lite read data channel --------------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output reg  [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output reg  [1:0]                        s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output reg                               s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire                              s_axi_rready,

    // ---- Hardware side ----------------------------------------------------
    // Read data for read-only registers (ignored for registers with storage)
    input  wire [NUM_REGS*C_S_AXI_DATA_WIDTH-1:0] hw_status,
    // Per-bit "set" input for W1C registers (tie 0 when unused)
    input  wire [NUM_REGS*C_S_AXI_DATA_WIDTH-1:0] hw_set,
    // Current value of every register (storage value, or hw_status when RO)
    output wire [NUM_REGS*C_S_AXI_DATA_WIDTH-1:0] reg_value,
    // One-cycle strobe, aligned with the new value appearing on reg_value
    output reg  [NUM_REGS-1:0]                    reg_wr_stb,
    // One-cycle strobe when software reads register i (aligned with rvalid)
    output reg  [NUM_REGS-1:0]                    reg_rd_stb
);

// =================================================================
// Local Parameters
// =================================================================
localparam integer DW        = C_S_AXI_DATA_WIDTH;
localparam integer SW        = C_S_AXI_DATA_WIDTH/8;      // wstrb width
localparam integer AW        = C_S_AXI_ADDR_WIDTH;
localparam integer ADDR_LSB  = (DW == 64) ? 3 : 2;        // byte-offset bits
localparam integer IDX_W     = (NUM_REGS > 1) ? $clog2(NUM_REGS) : 1;
localparam integer MAP_BYTES = NUM_REGS * SW;             // size of the aperture

// Internal address width: at least 32 bits so the single range comparison
// below never truncates, and never narrower than the port itself.
localparam integer AW_X = (AW > 32) ? AW : 32;

// A register has flip-flops behind it if it is software-writable, W1C, or
// self-clearing.  Everything else is a pure read-only window onto hw_status.
// Declaring this NUM_REGS wide (not MASK_W) drops the byte-alignment padding
// from the mask parameters, so it stays width-matched with the decodes below.
localparam [NUM_REGS-1:0] STORAGE_MASK =
           REG_RW_MASK | REG_W1C_MASK | REG_SELF_CLEAR_MASK;

localparam [1:0] RESP_OKAY   = 2'b00;
localparam [1:0] RESP_SLVERR = 2'b10;
localparam [1:0] RESP_DECERR = 2'b11;

// =================================================================
// Write Channel - AW and W are latched independently
// =================================================================
reg           aw_held;
reg           w_held;
reg [AW-1:0]  awaddr_q;
reg [DW-1:0]  wdata_q;
reg [SW-1:0]  wstrb_q;

// Stall the write channels only while a response is actually waiting to be
// taken.  When bvalid && bready the slot frees this cycle, so a new address /
// data pair may be accepted and even committed back to back.
wire b_stall = s_axi_bvalid && !s_axi_bready;

assign s_axi_awready = !aw_held && !b_stall;
assign s_axi_wready  = !w_held  && !b_stall;

wire aw_take = s_axi_awvalid && s_axi_awready;
wire w_take  = s_axi_wvalid  && s_axi_wready;

// "ok" = this half of the write is available now, either from the holding
// register or straight off the bus in the same cycle (write-through, so an
// aligned AW+W pair costs a single cycle).
wire aw_ok = aw_held || aw_take;
wire w_ok  = w_held  || w_take;

wire          wr_commit = aw_ok && w_ok && !b_stall;
wire [AW-1:0] wr_addr   = aw_held ? awaddr_q : s_axi_awaddr;
wire [DW-1:0] wr_data   = w_held  ? wdata_q  : s_axi_wdata;
wire [SW-1:0] wr_strb   = w_held  ? wstrb_q  : s_axi_wstrb;

always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        aw_held  <= 1'b0;
        w_held   <= 1'b0;
        awaddr_q <= {AW{1'b0}};
        wdata_q  <= {DW{1'b0}};
        wstrb_q  <= {SW{1'b0}};
    end else begin
        if (aw_take) awaddr_q <= s_axi_awaddr;
        if (w_take) begin
            wdata_q <= s_axi_wdata;
            wstrb_q <= s_axi_wstrb;
        end

        if (wr_commit) begin
            // Both halves consumed this cycle - drop the holds.
            aw_held <= 1'b0;
            w_held  <= 1'b0;
        end else begin
            if (aw_take) aw_held <= 1'b1;
            if (w_take)  w_held  <= 1'b1;
        end
    end
end

// =================================================================
// Address Decode
// =================================================================
// Implicit zero-extension of the (unsigned) address onto a wider wire keeps
// the range check from wrapping when the aperture is larger than the register
// map.  A single unsigned compare then covers both "upper bits clear" and
// "index < NUM_REGS", including the non-power-of-two NUM_REGS case.
wire [AW_X-1:0] wr_addr_x = wr_addr;
wire [AW_X-1:0] rd_addr_x = s_axi_araddr;

wire [IDX_W-1:0] wr_index = wr_addr_x[ADDR_LSB +: IDX_W];
wire [IDX_W-1:0] rd_index = rd_addr_x[ADDR_LSB +: IDX_W];

wire wr_in_range = (wr_addr_x < MAP_BYTES);
wire rd_in_range = (rd_addr_x < MAP_BYTES);

wire rd_accept = s_axi_arvalid && s_axi_arready;

// One-hot decodes.  *_dec is the pure address match; *_sel adds the qualifier
// that an access is actually happening this cycle.
wire [NUM_REGS-1:0] wr_dec;
wire [NUM_REGS-1:0] rd_dec;
wire [NUM_REGS-1:0] wr_sel;
wire [NUM_REGS-1:0] rd_sel;

// Writes only land on registers that have storage; a write aimed at a
// read-only register decodes but is dropped (optionally with SLVERR).
wire wr_hits_storage = wr_in_range && (|(wr_dec & STORAGE_MASK));

// =================================================================
// Register Array
// =================================================================
wire [DW-1:0] reg_val [0:NUM_REGS-1];

// Expand wstrb into a per-bit write mask once, shared by every register.
reg [DW-1:0] wr_bitmask;
integer bi;
always @(*) begin
    for (bi = 0; bi < SW; bi = bi + 1)
        wr_bitmask[bi*8 +: 8] = {8{wr_strb[bi]}};
end

genvar gi;
generate
for (gi = 0; gi < NUM_REGS; gi = gi + 1) begin : g_reg

    localparam [DW-1:0] INIT_I = REG_INIT[gi*DW +: DW];

    assign wr_dec[gi] = (wr_index == gi[IDX_W-1:0]);
    assign rd_dec[gi] = (rd_index == gi[IDX_W-1:0]);
    assign wr_sel[gi] = wr_commit && wr_in_range && wr_dec[gi] && STORAGE_MASK[gi];
    assign rd_sel[gi] = rd_accept && rd_in_range && rd_dec[gi];

    if (STORAGE_MASK[gi]) begin : g_storage
        reg  [DW-1:0] r;
        wire [DW-1:0] set_i = hw_set[gi*DW +: DW];
        // Bits software is clearing this cycle (W1C only, byte-strobe aware)
        wire [DW-1:0] w1c_clr = wr_sel[gi] ? (wr_data & wr_bitmask) : {DW{1'b0}};

        always @(posedge s_axi_aclk) begin
            if (!s_axi_aresetn) begin
                r <= INIT_I;
            end else if (REG_W1C_MASK[gi]) begin
                // Sticky status: a hardware set wins over a simultaneous
                // software clear, so an event arriving during the clearing
                // write is never lost.
                r <= (r & ~w1c_clr) | set_i;
            end else if (wr_sel[gi]) begin
                r <= (r & ~wr_bitmask) | (wr_data & wr_bitmask);
            end else if (REG_SELF_CLEAR_MASK[gi]) begin
                // Command register: the written value is visible for exactly
                // one clock, then snaps back to its reset value.
                r <= INIT_I;
            end
        end

        assign reg_val[gi] = r;
    end else begin : g_ro
        // Read-only window - no flip-flops, reads see hardware directly.
        assign reg_val[gi] = hw_status[gi*DW +: DW];
    end

    assign reg_value[gi*DW +: DW] = reg_val[gi];
end
endgenerate

// Strobes are registered so they line up with the cycle in which reg_value
// shows the freshly written data / s_axi_rdata presents the read data.
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        reg_wr_stb <= {NUM_REGS{1'b0}};
        reg_rd_stb <= {NUM_REGS{1'b0}};
    end else begin
        reg_wr_stb <= wr_sel;
        reg_rd_stb <= rd_sel;
    end
end

// =================================================================
// Read Data Mux
// =================================================================
reg [DW-1:0] rd_mux;
integer ri;
always @(*) begin
    rd_mux = {DW{1'b0}};
    for (ri = 0; ri < NUM_REGS; ri = ri + 1)
        if (rd_dec[ri]) rd_mux = reg_val[ri];
end

// =================================================================
// Write Response Channel
// =================================================================
wire [1:0] wr_resp = (!wr_in_range && (DECODE_ERR_EN != 0)) ? RESP_DECERR :
                     (wr_in_range && !wr_hits_storage &&
                      (RO_WR_ERR_EN != 0))                  ? RESP_SLVERR :
                                                              RESP_OKAY;

always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        s_axi_bvalid <= 1'b0;
        s_axi_bresp  <= RESP_OKAY;
    end else if (wr_commit) begin
        s_axi_bvalid <= 1'b1;
        s_axi_bresp  <= wr_resp;
    end else if (s_axi_bvalid && s_axi_bready) begin
        s_axi_bvalid <= 1'b0;
    end
end

// =================================================================
// Read Address / Read Data Channels
// =================================================================
// Accept a new address whenever the read data register is free, or is being
// drained this cycle - one read in flight, full back-to-back throughput.
assign s_axi_arready = !s_axi_rvalid || s_axi_rready;

always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        s_axi_rvalid <= 1'b0;
        s_axi_rdata  <= {DW{1'b0}};
        s_axi_rresp  <= RESP_OKAY;
    end else if (rd_accept) begin
        s_axi_rvalid <= 1'b1;
        s_axi_rdata  <= rd_in_range ? rd_mux : {DW{1'b0}};
        s_axi_rresp  <= (!rd_in_range && (DECODE_ERR_EN != 0)) ? RESP_DECERR
                                                               : RESP_OKAY;
    end else if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0;
    end
end

endmodule
