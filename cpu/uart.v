`default_nettype none

module uart_tx(
	input wire CLK,
	output reg TX,
	output wire RDY,
	input wire [7:0] D,
	input wire EN
);

reg [11:0] counter;
reg [3:0] bitidx;
reg active;

assign RDY = !active;

localparam MAX_CNT = 625;

initial counter = 0;
initial active = 0;
initial TX = 1;

reg [8:0] data;

always @(posedge CLK) begin
	if (!active) begin
		if (EN) begin
			active <= 1;
			counter <= 0;
			TX <= 0;
			bitidx <= 0;
			data <= {1'b1, D};
		end
	end else begin
		if (counter == MAX_CNT - 1) begin
			counter <= 0;
			bitidx <= bitidx + 1;
			if (bitidx < 9) begin
				TX <= data[bitidx];
			end else begin
				active <= 0;
			end
		end else
			counter <= counter + 1;
	end
end

endmodule

module uart_rx(
	input wire CLK,
	input wire RX,
	output reg [7:0] D,
	output reg EN,
	output reg ERR
);

reg [11:0] counter;
reg [3:0] bitidx;
reg [3:0] state;
reg rx_tmp, rx_s;

localparam STATE_IDLE = 0;
localparam STATE_START = 1;
localparam STATE_DATA = 2;
localparam STATE_STOP = 3;

localparam MAX_CNT = 625 - 1;

initial counter = 0;
initial EN = 0;
initial D = 0;
initial ERR = 0;
initial state = STATE_IDLE;
initial rx_tmp <= 1;
initial rx_s <= 1;

always @(posedge CLK) begin
	rx_tmp <= RX;
	rx_s <= rx_tmp;
	EN <= 0;
	ERR <= 1'bx;
	if (counter)
		counter <= counter - 1;
	else case (state)
		STATE_IDLE: begin
			if (!rx_s) begin
				counter <= MAX_CNT / 2;
				state <= STATE_START;
				bitidx <= 0;
			end
		end
		STATE_START:
		begin
			counter <= MAX_CNT;
			if (rx_s) begin
				EN <= 1;
				ERR <= 1;
				state <= STATE_IDLE;
			end else begin
				state <= STATE_DATA;
			end
		end
		STATE_DATA:
		begin
			counter <= MAX_CNT;
			D <= {rx_s, D[7:1]};
			bitidx <= bitidx + 1;
			if (bitidx == 7)
				state <= STATE_STOP;
		end
		STATE_STOP:
		begin
			if (rx_s) begin
				EN <= 1;
				ERR <= 0;
				state <= STATE_IDLE;
			end else begin
				EN <= 1;
				ERR <= 1;
				state <= STATE_IDLE;
			end
		end
	endcase
end

endmodule
