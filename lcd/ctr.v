module ctr(
	input wire CLK,
	input wire [7:0] SW,
	output reg V0,
	output reg RS,
	output wire RW,
	output reg E,
	output reg [7:0] D,
	output wire K,
	output wire [7:0] LED
);

localparam STATE_START = 2'b00;
localparam STATE_HIGH = 2'b01;
localparam STATE_LOW = 2'b10;
localparam STATE_END = 2'b11;

assign RW = 0;
assign LED = 0;
assign K = 1;

reg [8:0] data [0:15];

initial begin
	data[4'h0] <= 9'h038;
	data[4'h1] <= 9'h00f;
	data[4'h2] <= 9'h080;
	data[4'h3] <= 9'h148;
	data[4'h4] <= 9'h165;
	data[4'h5] <= 9'h16c;
	data[4'h6] <= 9'h16c;
	data[4'h7] <= 9'h16f;
	data[4'h8] <= 9'h12c;
	data[4'h9] <= 9'h120;
	data[4'ha] <= 9'h177;
	data[4'hb] <= 9'h16f;
	data[4'hc] <= 9'h172;
	data[4'hd] <= 9'h16c;
	data[4'he] <= 9'h164;
	data[4'hf] <= 9'h121;
end

reg [1:0] state;
reg [11:0] wctr;
reg [3:0] addr;

initial state = STATE_START;
initial wctr = 12'hfff;
initial addr = 0;
initial RS = 0;
initial D = 0;
initial E = 0;

always @(posedge CLK) begin
	if (wctr > 0) begin
		wctr <= wctr - 1;
	end else begin
		case (state)
		STATE_START: begin
			state <= STATE_HIGH;
			wctr <= 13;
		end
		STATE_HIGH: begin
			state <= STATE_LOW;
			wctr <= 50 * 40;
		end
		STATE_LOW: begin
			if (addr == 4'hf) begin
				state <= STATE_END;
			end else begin
				addr <= addr + 1;
				state <= STATE_HIGH;
				wctr <= 13;
			end
		end
		STATE_END: begin
		end
		endcase
	end
end

always @(posedge CLK) begin
	{RS, D} <= data[addr];
	case (state)
	STATE_START: begin
		E <= 0;
	end
	STATE_HIGH: begin
		E <= 1;
	end
	STATE_LOW: begin
		E <= 0;
	end
	STATE_END: begin
		E <= 0;
	end
	endcase
end

reg [7:0] ctr;


always @(posedge CLK) begin
	ctr <= ctr + 1;
	V0 <= ctr[7:0] <= SW;
end

endmodule
