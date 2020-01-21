`default_nettype none

module top(
	input wire CLK,
	input wire [5:0] BTN,
	input wire RX,
	output wire TX,
	output reg [7:0] LED
);

reg tx_EN;
reg [7:0] tx_D;
wire tx_RDY;

reg [5:0] btn_tmp;
reg [5:0] btn_s;

always @(posedge CLK) begin
	btn_tmp <= ~BTN;
	btn_s <= btn_tmp;
end

uart_tx mah_tx(
	.CLK(CLK),
	.TX(TX),
	.D(tx_D),
	.EN(tx_EN),
	.RDY(tx_RDY)
);

reg [8:0] uart_rdata;
reg uart_rx_full;
initial uart_rx_full = 0;

wire [7:0] rx_D;
wire rx_EN;
wire rx_ERR;

uart_rx mah_rx(
	.CLK(CLK),
	.RX(RX),
	.D(rx_D),
	.EN(rx_EN),
	.ERR(rx_ERR)
);

wire mem_en;
wire [3:0] mem_we;
wire [31:2] mem_addr;
reg [31:2] prev_mem_addr;
wire [31:0] mem_wdata;
reg [31:0] mem_rdata;
reg [15:0] rst;
initial rst = 16'hffff;

always @(posedge CLK) begin
	if (btn_s[0])
		rst <= 16'hffff;
	else
		rst <= rst >> 1;
end

cpu mah_cpu (
	.mem_en(mem_en),
	.mem_rdy(1),
	.mem_we(mem_we),
	.mem_addr(mem_addr),
	.mem_wdata(mem_wdata),
	.mem_rdata(mem_rdata),
	.rst(rst[0]),
	.clk(CLK)
);

reg [31:0] ram [0:2047];

initial $readmemh("firmware.hex", ram);

reg [31:0] ram_rdata;

always @(posedge CLK) begin
	if (mem_en && !mem_addr[31]) begin
		ram_rdata <= ram[mem_addr];
		if (mem_we[0])
			ram[mem_addr][7:0] <= mem_wdata[7:0];
		if (mem_we[1])
			ram[mem_addr][15:8] <= mem_wdata[15:8];
		if (mem_we[2])
			ram[mem_addr][23:16] <= mem_wdata[23:16];
		if (mem_we[3])
			ram[mem_addr][31:24] <= mem_wdata[31:24];
	end
	prev_mem_addr <= mem_addr;
end

always @* begin
	mem_rdata = 32'hxxxxxxxx;
	if (!prev_mem_addr[31])
		mem_rdata = ram_rdata;
	else if ({prev_mem_addr, 2'b00} == 32'h80000000)
		mem_rdata = uart_rdata;
	else if ({prev_mem_addr, 2'b00} == 32'h80000004)
		mem_rdata = {tx_RDY, uart_rx_full};
	else if ({prev_mem_addr, 2'b00} == 32'h8000000c)
		mem_rdata = btn_s;
end


always @(posedge CLK) begin
	if (rx_EN) begin
		uart_rdata <= {rx_ERR, rx_D};
		uart_rx_full <= 1;
	end
	tx_EN <= 0;
	if (mem_en && {mem_addr, 2'b00} == 32'h80000000) begin
		if (mem_we)
			tx_EN <= 1;
		else
			uart_rx_full <= 0;
	end
	tx_D <= mem_wdata;
	if (mem_en && {mem_addr, 2'b00} == 32'h80000008 && mem_we)
		LED <= mem_wdata;
end


endmodule


