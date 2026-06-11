`timescale 1ns/1ps

module mwr_batch_tb;

localparam int AXIS_DATA_WIDTH  = 512;
localparam int AXIS_TUSER_WIDTH = 183;
localparam int TIME_FEDILITY    = 8;
localparam int DEPTH_FEDILITY   = 8;

localparam bit [3:0] MWR_TYPE = 4'b0001;

// -----------------------------------------------------------------------------
// DUT IO
// -----------------------------------------------------------------------------
reg                           clk;
reg                           rst_n;

reg  [TIME_FEDILITY-1:0]      time_threshold;
reg  [DEPTH_FEDILITY-1:0]     depth_threshold;

reg  [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_0;
reg  [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep_0;
reg                           s_axis_tvalid_0;
reg                           s_axis_tlast_0;
reg  [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_0;
wire                          s_axis_tready_0;

reg  [AXIS_DATA_WIDTH-1:0]    s_axis_tdata_1;
reg  [AXIS_DATA_WIDTH/8-1:0]  s_axis_tkeep_1;
reg                           s_axis_tvalid_1;
reg                           s_axis_tlast_1;
reg  [AXIS_TUSER_WIDTH-1:0]   s_axis_tuser_1;
wire                          s_axis_tready_1;

wire [AXIS_DATA_WIDTH-1:0]    m_axis_tdata;
wire [AXIS_DATA_WIDTH/8-1:0]  m_axis_tkeep;
wire                          m_axis_tvalid;
wire                          m_axis_tlast;
wire [AXIS_TUSER_WIDTH-1:0]   m_axis_tuser;
reg                           m_axis_tready;

// Telemetry ports were removed from axi4_mwr_batch; packet-level telemetry is
// now collected by the axi4_telemetry passthrough module on the master stream.

axi4_mwr_batch dut (
	.clk                   (clk),
	.rst_n                 (rst_n),
	.time_threshold        (time_threshold),
	.depth_threshold       (depth_threshold),

	.s_axis_tdata_0        (s_axis_tdata_0),
	.s_axis_tkeep_0        (s_axis_tkeep_0),
	.s_axis_tvalid_0       (s_axis_tvalid_0),
	.s_axis_tlast_0        (s_axis_tlast_0),
	.s_axis_tuser_0        (s_axis_tuser_0),
	.s_axis_tready_0       (s_axis_tready_0),

	.s_axis_tdata_1        (s_axis_tdata_1),
	.s_axis_tkeep_1        (s_axis_tkeep_1),
	.s_axis_tvalid_1       (s_axis_tvalid_1),
	.s_axis_tlast_1        (s_axis_tlast_1),
	.s_axis_tuser_1        (s_axis_tuser_1),
	.s_axis_tready_1       (s_axis_tready_1),

	.m_axis_tdata          (m_axis_tdata),
	.m_axis_tkeep          (m_axis_tkeep),
	.m_axis_tvalid         (m_axis_tvalid),
	.m_axis_tlast          (m_axis_tlast),
	.m_axis_tuser          (m_axis_tuser),
	.m_axis_tready         (m_axis_tready)
);

// -----------------------------------------------------------------------------
// Clock/Reset
// -----------------------------------------------------------------------------
initial clk = 1'b0;
always #5 clk = ~clk;

task automatic reset_dut;
begin
	rst_n          = 1'b0;
	time_threshold = 8'd6;
	depth_threshold= 8'd10;

	s_axis_tdata_0 = '0;
	s_axis_tkeep_0 = '0;
	s_axis_tvalid_0= 1'b0;
	s_axis_tlast_0 = 1'b0;
	s_axis_tuser_0 = '0;

	s_axis_tdata_1 = '0;
	s_axis_tkeep_1 = '0;
	s_axis_tvalid_1= 1'b0;
	s_axis_tlast_1 = 1'b0;
	s_axis_tuser_1 = '0;

	m_axis_tready  = 1'b0;

	repeat (10) @(posedge clk);
	rst_n = 1'b1;
	repeat (5) @(posedge clk);
end
endtask

// -----------------------------------------------------------------------------
// Scoreboard database (out-of-order capable)
// -----------------------------------------------------------------------------
typedef struct {
	bit [3:0] exp_type;
	int       exp_port;
	int       exp_beats;

	int       seen_beats;
	bit       saw_eop;
	bit       bad;
} pkt_info_t;

pkt_info_t pkt_db [longint unsigned];

longint unsigned next_pkt_id;
semaphore id_lock;

int total_expected;
int total_observed_beats;
int total_completed;
int total_unknown_packets;

function automatic bit [3:0] gen_nonmwr_type;
	int r;
begin
	r = $urandom_range(0, 5);
	case (r)
		0: gen_nonmwr_type = 4'h2;
		1: gen_nonmwr_type = 4'h3;
		2: gen_nonmwr_type = 4'h4;
		3: gen_nonmwr_type = 4'h5;
		4: gen_nonmwr_type = 4'h6;
		default: gen_nonmwr_type = 4'h7;
	endcase
end
endfunction

task automatic register_expected(
	input longint unsigned pkt_id,
	input int              port,
	input bit [3:0]        req_type,
	input int              beats
);
begin
	pkt_db[pkt_id].exp_type   = req_type;
	pkt_db[pkt_id].exp_port   = port;
	pkt_db[pkt_id].exp_beats  = beats;
	pkt_db[pkt_id].seen_beats = 0;
	pkt_db[pkt_id].saw_eop    = 1'b0;
	pkt_db[pkt_id].bad        = 1'b0;
	total_expected            = total_expected + 1;
end
endtask

task automatic collect_output_beat;
	longint unsigned pkt_id;
	bit [3:0]        req_type;
	pkt_info_t       info;
begin
	pkt_id   = longint'(m_axis_tdata[63:2]);
	req_type = m_axis_tdata[78:75];

	total_observed_beats = total_observed_beats + 1;

	if (!pkt_db.exists(pkt_id)) begin
		pkt_db[pkt_id].exp_type   = 4'h0;
		pkt_db[pkt_id].exp_port   = -1;
		pkt_db[pkt_id].exp_beats  = -1;
		pkt_db[pkt_id].seen_beats = 0;
		pkt_db[pkt_id].saw_eop    = 1'b0;
		pkt_db[pkt_id].bad        = 1'b1;
		total_unknown_packets     = total_unknown_packets + 1;
	end

	info = pkt_db[pkt_id];

	// First beat must carry SOP; later beats must not.
	if (info.seen_beats == 0 && m_axis_tuser[80] != 1'b1)
		info.bad = 1'b1;
	if (info.seen_beats > 0 && m_axis_tuser[80] == 1'b1)
		info.bad = 1'b1;

	// Type must remain constant and match expected when known.
	if (info.exp_beats > 0 && req_type != info.exp_type)
		info.bad = 1'b1;

	info.seen_beats = info.seen_beats + 1;

	if (m_axis_tlast) begin
		if (info.saw_eop)
			info.bad = 1'b1;
		info.saw_eop = 1'b1;
		total_completed = total_completed + 1;
	end

	pkt_db[pkt_id] = info;
end
endtask

// -----------------------------------------------------------------------------
// AXIS packet driver task (either input port)
// ----------------------------------------------------------------------------
task automatic drive_packet(
	input int              port,
	input bit [3:0]        req_type,
	input longint unsigned pkt_id,
	input int              beats
);
	int b;
	reg [AXIS_DATA_WIDTH-1:0] d;
	reg [AXIS_TUSER_WIDTH-1:0] u;
	reg [AXIS_DATA_WIDTH/8-1:0] k;
	bit last;
	bit [3:0] eop_ptr;
begin
	for (b = 0; b < beats; b = b + 1) begin
		d = '0;
		u = '0;
		k = {AXIS_DATA_WIDTH/8{1'b1}};
		last = (b == beats - 1);

		// Required protocol fields.
		d[78:75]  = req_type;
		d[63:2]   = pkt_id[61:0];
		d[1:0]    = 2'b01;
		d[103:96] = pkt_id[7:0];

		// Extra identity markers for debug/packet trace.
		d[191:160] = pkt_id[31:0];
		d[223:192] = b[31:0];
		d[255:224] = port[31:0];

		u[81:80] = (b == 0) ? 2'b01 : 2'b00;         // SOP field
		u[87:86] = last ? 2'b01 : 2'b00;             // EOP field
		eop_ptr  = last ? $urandom_range(0, 15) : 4'd0;
		u[91:88] = eop_ptr;                          // EOP byte pointer

		if (port == 0) begin
			s_axis_tdata_0  <= d;
			s_axis_tkeep_0  <= k;
			s_axis_tvalid_0 <= 1'b1;
			s_axis_tlast_0  <= last;
			s_axis_tuser_0  <= u;

			do begin
				@(posedge clk);
			end while (!s_axis_tready_0);

			s_axis_tvalid_0 <= 1'b0;
			s_axis_tlast_0  <= 1'b0;
			s_axis_tdata_0  <= '0;
			s_axis_tuser_0  <= '0;
			s_axis_tkeep_0  <= '0;
		end else begin
			s_axis_tdata_1  <= d;
			s_axis_tkeep_1  <= k;
			s_axis_tvalid_1 <= 1'b1;
			s_axis_tlast_1  <= last;
			s_axis_tuser_1  <= u;

			do begin
				@(posedge clk);
			end while (!s_axis_tready_1);

			s_axis_tvalid_1 <= 1'b0;
			s_axis_tlast_1  <= 1'b0;
			s_axis_tdata_1  <= '0;
			s_axis_tuser_1  <= '0;
			s_axis_tkeep_1  <= '0;
		end
	end
end
endtask

task automatic drive_port(
	input int port,
	input int num_packets
);
	int i;
	int beats;
	bit [3:0] req_type;
	longint unsigned pkt_id;
begin
	for (i = 0; i < num_packets; i = i + 1) begin
		beats = $urandom_range(1, 12);

		if ($urandom_range(0, 99) < 55)
			req_type = MWR_TYPE;
		else
			req_type = gen_nonmwr_type();

		id_lock.get(1);
		pkt_id = next_pkt_id;
		next_pkt_id = next_pkt_id + 1;
		id_lock.put(1);

		register_expected(pkt_id, port, req_type, beats);
		drive_packet(port, req_type, pkt_id, beats);

		repeat ($urandom_range(0, 4)) @(posedge clk);
	end
end
endtask

// -----------------------------------------------------------------------------
// Output collector and sink backpressure
// -----------------------------------------------------------------------------
bit traffic_done;

always @(posedge clk) begin
	if (!rst_n)
		m_axis_tready <= 1'b0;
	else if (traffic_done)
		m_axis_tready <= 1'b1;
	else
		m_axis_tready <= ($urandom_range(0, 99) < 80);
end

always @(posedge clk) begin
	if (rst_n && m_axis_tvalid && m_axis_tready)
		collect_output_beat();
end

task automatic report_results;
	longint unsigned id;
	int missing;
	int incomplete;
	int good;
begin
	missing    = 0;
	incomplete = 0;
	good       = 0;

	foreach (pkt_db[id]) begin
		if (pkt_db[id].exp_beats > 0) begin
			if (pkt_db[id].seen_beats == 0) begin
				missing = missing + 1;
			end else if (!pkt_db[id].saw_eop || pkt_db[id].bad ||
						 (pkt_db[id].seen_beats != pkt_db[id].exp_beats)) begin
				incomplete = incomplete + 1;
			end else begin
				good = good + 1;
			end
		end
	end

	$display("\n================ mwr_batch_tb summary ================");
	$display("expected packets      : %0d", total_expected);
	$display("good packets          : %0d", good);
	$display("incomplete packets    : %0d", incomplete);
	$display("missing packets       : %0d", missing);
	$display("unknown output packet : %0d", total_unknown_packets);
	$display("observed output beats : %0d", total_observed_beats);
	$display("completed packets     : %0d", total_completed);
	$display("======================================================\n");

	if ((missing == 0) && (incomplete == 0) && (total_unknown_packets == 0))
		$display("TEST PASS");
	else
		$error("TEST FAIL: missing=%0d incomplete=%0d unknown=%0d",
			   missing, incomplete, total_unknown_packets);
end
endtask

// -----------------------------------------------------------------------------
// Main test
// -----------------------------------------------------------------------------
initial begin
	next_pkt_id          = 1;
	id_lock              = new(1);
	total_expected       = 0;
	total_observed_beats = 0;
	total_completed      = 0;
	total_unknown_packets= 0;
	traffic_done         = 1'b0;

	reset_dut();

	fork
		drive_port(0, 1000);
		drive_port(1, 1000);
	join

	traffic_done = 1'b1;

	// Drain any buffered packets.
	repeat (4000) @(posedge clk);

	report_results();
	#50;
	$finish;
end

// Watchdog
initial begin
	repeat (500000) @(posedge clk);
	$fatal(1, "Timeout in mwr_batch_tb");
end

endmodule

