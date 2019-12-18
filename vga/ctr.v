module ctr(
	input wire CLK,
	input wire uclk,
	input wire BTN0,
	output reg HSYNC,
	output reg VSYNC,
	output reg [2:0] VGAR,
	output reg [2:0] VGAG,
	output reg [2:1] VGAB,
	input wire EPP_ASTB,
	input wire EPP_DSTB,
	input wire EPP_WR,
	output reg EPP_WAIT,
	inout wire [7:0] EPP_D,
	output wire [7:0] LED
);

localparam STATE_INIT = 8'h00;
localparam STATE_WAIT = 8'h01;

localparam ADDR_X = 8'h00;
localparam ADDR_Y = 8'h01;
localparam ADDR_D = 8'h02;

wire clk;

DCM_SP #(
	.CLKFX_DIVIDE(7),
	.CLKFX_MULTIPLY(11)
) moj_dcm (
	.CLKIN(uclk),
	.CLKFX(clk),
	.RST(0)
);

assign LED = BTN0;

reg [9:0] hc, hc1, hc2;
reg [9:0] vc, vc1, vc2;
reg xc;

reg [7:0] font [0:2047];
reg [8:0] fb [0:2047];

initial begin
	$readmemh("font.hex", font);
	$readmemh("fb.hex", fb);
end

localparam MAX_HC = 800;
localparam MAX_VC = 449;
localparam HS_START = 640 + 16;
localparam HS_END = HS_START + 96;
localparam VS_START = 400 + 12;
localparam VS_END = VS_START + 2;

always @(posedge clk) begin
	xc <= !xc;
	if (xc) begin
		if (hc == MAX_HC - 1) begin
			hc <= 0;
			if (vc == MAX_VC - 1)
				vc <= 0;
			else
				vc <= vc + 1;
		end else
			hc <= hc + 1;
	end
end

wire DE2;

assign DE2 = (hc2 < 640 && vc2 < 400);

wire [10:0] fbaddr0;
reg [8:0] chr1;

wire [2:0] ix0;
wire [3:0] iy0;
reg [2:0] ix1, ix2;
reg [3:0] iy1;
wire [6:0] ox0;
wire [5:0] oy0;
reg [7:0] fontbit2;

assign {ox0, ix0} = hc;
assign {oy0, iy0} = vc;
assign fbaddr0 = ox0 + oy0 * 80;

always @(posedge clk) begin
	hc1 <= hc;
	vc1 <= vc;
	hc2 <= hc1;
	vc2 <= vc1;
	chr1 <= fb[fbaddr0];
	iy1 <= iy0;
	ix1 <= ix0;
	ix2 <= ix1;
	fontbit2 <= font[{chr1[6:0], iy1}];
	//fontbit2 <= ^{chr1[6:0], iy1, ix1};
	VGAR <= DE2 ? fontbit2[ix2] ? 7 : 0 : 0;
	VGAG <= DE2 ? fontbit2[ix2] ? 7 : 0 : 0;
	VGAB <= DE2 ? fontbit2[ix2] ? 3 : 0 : 0;
	HSYNC <= (hc2 >= HS_START && hc2 < HS_END);
	VSYNC <= (vc2 >= VS_START && vc2 < VS_END);
end

reg astb0, astb;
reg dstb0, dstb;
reg wr0, wr;
reg [7:0] wdata0, wdata;

reg [7:0] fbrd;
reg [7:0] rdata;
reg rdata_oe;

reg [7:0] state;

reg [7:0] reg_a;
reg [7:0] reg_x;
reg [7:0] reg_y;

assign EPP_D = rdata_oe ? rdata : 8'hzz;

initial begin
	EPP_WAIT <= 0;
	rdata_oe <= 0;
	rdata <= 0;
	state <= STATE_INIT;
	astb0 <= 1;
	astb <= 1;
	dstb0 <= 1;
	dstb <= 1;
end

wire [13:0] fba;
assign fba = reg_x + reg_y * 80;

always @(posedge clk) begin
	astb0 <= EPP_ASTB;
	dstb0 <= EPP_DSTB;
	wr0 <= EPP_WR;
	wdata0 <= EPP_D;
	astb <= astb0;
	dstb <= dstb0;
	wr <= wr0;
	wdata <= wdata0;
	fbrd <= fb[fba];

	case (state)
	STATE_INIT: begin
		EPP_WAIT <= 0;
		if (!astb && !wr) begin
			reg_a <= wdata;
			state <= STATE_WAIT;
		end
		if (!dstb && !wr) begin
			case (reg_a)
			ADDR_X: begin
				reg_x <= wdata;
			end
			ADDR_Y: begin
				reg_y <= wdata;
			end
			ADDR_D: begin
				fb[fba] <= wdata;
				if (reg_x == 79) begin
					reg_x <= 0;
					if (reg_y == 24)
						reg_y <= 0;
					else
						reg_y <= reg_y + 1;
				end else
					reg_x <= reg_x + 1;
			end
			endcase
			state <= STATE_WAIT;
		end
		if (!dstb && wr) begin
			rdata_oe <= 1;
			state <= STATE_WAIT;
			case (reg_a)
			ADDR_X: begin
				rdata <= reg_x;
			end
			ADDR_Y: begin
				rdata <= reg_y;
			end
			ADDR_D: begin
				rdata <= fbrd;
				if (reg_x == 79) begin
					reg_x <= 0;
					if (reg_y == 24)
						reg_y <= 0;
					else
						reg_y <= reg_y + 1;
				end else
					reg_x <= reg_x + 1;
			end
			endcase
		end
	end
	STATE_WAIT: begin
		EPP_WAIT <= 1;
		if (astb && dstb) begin
			rdata_oe <= 0;
			state <= STATE_INIT;
		end
	end
	endcase
end

endmodule
