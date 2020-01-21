`default_nettype none

module cpu(
	output reg mem_en,
	input wire mem_rdy,
	output reg [3:0] mem_we,
	output reg [31:2] mem_addr,
	output reg [31:0] mem_wdata,
	input wire [31:0] mem_rdata,
	input wire rst,
	input wire clk
);


// Main state machine

localparam STATE_RESET = 8'h00;
localparam STATE_FETCH = 8'h01;
localparam STATE_DECODE = 8'h02;
localparam STATE_EXEC_WRITE = 8'h03;
localparam STATE_BC = 8'h04;
localparam STATE_JAL = 8'h05;
localparam STATE_JALR = 8'h06;
localparam STATE_LOAD_A = 8'h07;
localparam STATE_LOAD_D = 8'h08;
localparam STATE_STORE = 8'h09;

reg [7:0] state;
reg [7:0] next_state;

initial state = STATE_RESET;


// Registers.

reg [31:0] pc;
reg [31:0] regs[0:31];

integer i;
initial begin
	regs[0] = 0;
	for (i = 1; i < 32; i = i + 1)
		regs[i] = 32'haaaaaaaa;
end
initial pc = -4;

// Opcode decoding.

wire [6:0] op_opcode = mem_rdata[6:0];
wire [4:0] op_rd = mem_rdata[11:7];
wire [2:0] op_funct3 = mem_rdata[14:12];
wire [6:0] op_funct7 = mem_rdata[31:25];
wire [4:0] op_rs1 = mem_rdata[19:15];
wire [4:0] op_rs2 = mem_rdata[24:20];
wire [31:0] op_i_imm = {{20{mem_rdata[31]}}, mem_rdata[31:20]};
wire [31:0] op_s_imm = {{20{mem_rdata[31]}}, mem_rdata[31:25], mem_rdata[11:7]};
wire [31:0] op_b_imm = {{20{mem_rdata[31]}}, mem_rdata[7], mem_rdata[30:25], mem_rdata[11:8], 1'b0};
wire [31:0] op_j_imm = {{12{mem_rdata[31]}}, mem_rdata[19:12], mem_rdata[20], mem_rdata[30:21], 1'b0};
wire [31:0] op_u_imm = {mem_rdata[31:12], 12'h000};

reg [31:0] next_imm;
reg next_add_sub;

localparam OP_LOAD   = 7'b0000011;
localparam OP_IMM    = 7'b0010011;
localparam OP_AUIPC  = 7'b0010111;
localparam OP_STORE  = 7'b0100011;
localparam OP_MAIN   = 7'b0110011;
localparam OP_LUI    = 7'b0110111;
localparam OP_BRANCH = 7'b1100011;
localparam OP_JALR   = 7'b1100111;
localparam OP_JAL    = 7'b1101111;

localparam OP_BRANCH_BEQ  = 3'b000;
localparam OP_BRANCH_BNE  = 3'b001;
localparam OP_BRANCH_BLT  = 3'b100;
localparam OP_BRANCH_BGE  = 3'b101;
localparam OP_BRANCH_BLTU = 3'b110;
localparam OP_BRANCH_BGEU = 3'b111;

localparam OP_LOAD_LB  = 3'b000;
localparam OP_LOAD_LH  = 3'b001;
localparam OP_LOAD_LW  = 3'b010;
localparam OP_LOAD_LBU = 3'b100;
localparam OP_LOAD_LHU = 3'b101;

localparam OP_STORE_SB = 3'b000;
localparam OP_STORE_SH = 3'b001;
localparam OP_STORE_SW = 3'b010;

localparam OP_MAIN_ADD  = 3'b000; // And SUB
localparam OP_MAIN_SLL  = 3'b001;
localparam OP_MAIN_SLT  = 3'b010;
localparam OP_MAIN_SLTU = 3'b011;
localparam OP_MAIN_XOR  = 3'b100;
localparam OP_MAIN_SRL  = 3'b101; // And SRAI
localparam OP_MAIN_OR   = 3'b110;
localparam OP_MAIN_AND  = 3'b111;

// Decoder -> execution.

reg [4:0] dec_rd;
reg [31:0] dec_imm;
reg [31:0] reg_rs1;
reg [31:0] reg_rs2;
reg jump;

// Adder muxes.

localparam ADD_MUX1_ZERO = 2'b00;
localparam ADD_MUX1_RS1 = 2'b01;
localparam ADD_MUX1_PC = 2'b10;

localparam ADD_MUX2_FOUR = 2'b00;
localparam ADD_MUX2_IMM = 2'b01;
localparam ADD_MUX2_RS2 = 2'b10;

reg [1:0] next_add_mux1;
reg [1:0] add_mux1;
reg [1:0] next_add_mux2;
reg [1:0] add_mux2;

// Conditions.

localparam COND_LT = 2'b00;
localparam COND_LTU = 2'b01;
localparam COND_EQ = 2'b10;

localparam BIT_OP_AND = 2'b00;
localparam BIT_OP_OR = 2'b01;
localparam BIT_OP_XOR = 2'b10;

reg [1:0] cond_sel;
reg [1:0] next_cond_sel;
reg cond_neg;
reg next_cond_neg;
reg cond;
reg [1:0] next_bit_op;
reg [1:0] bit_op;
reg next_shift_right;
reg next_shift_arith;
reg shift_right;
reg shift_arith;
reg [2:0] next_mem_width;
reg [2:0] mem_width;
reg [1:0] next_mem_loaddr;
reg [1:0] mem_loaddr;

// Write select.

localparam WSEL_ADD = 3'b000;
localparam WSEL_COND = 3'b001;
localparam WSEL_BIT = 3'b010;
localparam WSEL_SHIFT = 3'b011;

reg [2:0] write_sel;
reg [2:0] next_write_sel;

// Adder interface.

reg add_sub;
reg [31:0] add_in1;
reg [31:0] add_in2;

reg [31:0] ld_out;
wire [31:0] add_out;
reg [31:0] bit_out;
reg [31:0] shift_out;
wire add_lt;
wire add_ltu;

always @* begin
	case (add_mux1)
	ADD_MUX1_ZERO: add_in1 = 0;
	ADD_MUX1_RS1: add_in1 = reg_rs1;
	ADD_MUX1_PC: add_in1 = pc;
	default: add_in1 = 32'hxxxxxxxx;
	endcase
end

always @* begin
	case (add_mux2)
	ADD_MUX2_FOUR: add_in2 = 4;
	ADD_MUX2_RS2: add_in2 = reg_rs2;
	ADD_MUX2_IMM: add_in2 = dec_imm;
	default: add_in2 = 32'hxxxxxxxx;
	endcase
end

// Register write control.

reg reg_we;
reg [31:0] reg_wdata;

// Decoder.

always @* begin
	next_state = 8'hxx;
	mem_en = 0;
	mem_we = 0;
	mem_addr = 30'hxxxxxxxx;
	mem_wdata = 32'hxxxxxxxx;
	next_imm = 32'hxxxxxxxx;
	next_add_sub = 1'bx;
	next_add_mux1 = 2'bxx;
	next_add_mux2 = 2'bxx;
	reg_we = 0;
	reg_wdata = 32'hxxxxxxxx;
	next_write_sel = 3'bxxx;
	next_cond_sel = 2'bxx;
	next_cond_neg = 1'bx;
	next_shift_right = 1'bx;
	next_shift_arith = 1'bx;
	next_bit_op = 2'bxx;
	next_mem_width = 3'bxxx;
	next_mem_loaddr = 2'bxx;
	jump = 0;
	case (state)
	STATE_RESET: begin
		next_state = STATE_FETCH;
	end
	STATE_FETCH: begin
		mem_en = 1;
		mem_addr = add_out[31:2];
		next_state = STATE_DECODE;
	end
	STATE_DECODE: begin
		case (op_opcode)
		OP_LOAD: begin
			next_imm = op_i_imm;
			next_add_mux1 = ADD_MUX1_RS1;
			next_add_mux2 = ADD_MUX2_IMM;
			next_add_sub = 0;
			next_state = STATE_LOAD_A;
			next_mem_width = op_funct3;
		end
		OP_STORE: begin
			next_imm = op_s_imm;
			next_add_mux1 = ADD_MUX1_RS1;
			next_add_mux2 = ADD_MUX2_IMM;
			next_add_sub = 0;
			next_state = STATE_STORE;
			next_mem_width = op_funct3;
		end
		OP_LUI: begin
			next_imm = op_u_imm;
			next_add_mux1 = ADD_MUX1_ZERO;
			next_add_mux2 = ADD_MUX2_IMM;
			next_add_sub = 0;
			next_write_sel = WSEL_ADD;
			next_state = STATE_EXEC_WRITE;
		end
		OP_AUIPC: begin
			next_imm = op_u_imm;
			next_add_mux1 = ADD_MUX1_PC;
			next_add_mux2 = ADD_MUX2_IMM;
			next_add_sub = 0;
			next_write_sel = WSEL_ADD;
			next_state = STATE_EXEC_WRITE;
		end
		OP_MAIN: begin
			next_add_mux1 = ADD_MUX1_RS1;
			next_add_mux2 = ADD_MUX2_RS2;
			case (op_funct3)
			OP_MAIN_ADD: begin
				next_add_sub = op_funct7[5];
				next_write_sel = WSEL_ADD;
			end
			OP_MAIN_SLT: begin
				next_add_sub = 1;
				next_write_sel = WSEL_COND;
				next_cond_sel = COND_LT;
				next_cond_neg = 0;
			end
			OP_MAIN_SLTU: begin
				next_add_sub = 1;
				next_write_sel = WSEL_COND;
				next_cond_sel = COND_LTU;
				next_cond_neg = 0;
			end
			OP_MAIN_SLL: begin
				next_write_sel = WSEL_SHIFT;
				next_shift_right = 0;
			end
			OP_MAIN_SRL: begin
				next_write_sel = WSEL_SHIFT;
				next_shift_right = 1;
				next_shift_arith = op_funct7[5];
			end
			OP_MAIN_XOR: begin
				next_write_sel = WSEL_BIT;
				next_bit_op = BIT_OP_XOR;
			end
			OP_MAIN_OR: begin
				next_write_sel = WSEL_BIT;
				next_bit_op = BIT_OP_OR;
			end
			OP_MAIN_AND: begin
				next_write_sel = WSEL_BIT;
				next_bit_op = BIT_OP_AND;
			end
			default: next_state = STATE_FETCH;
			endcase
			next_state = STATE_EXEC_WRITE;
		end
		OP_IMM: begin
			next_imm = op_i_imm;
			next_add_mux1 = ADD_MUX1_RS1;
			next_add_mux2 = ADD_MUX2_IMM;
			case (op_funct3)
			OP_MAIN_ADD: begin
				next_add_sub = 0;
				next_write_sel = WSEL_ADD;
			end
			OP_MAIN_SLT: begin
				next_add_sub = 1;
				next_write_sel = WSEL_COND;
				next_cond_sel = COND_LT;
				next_cond_neg = 0;
			end
			OP_MAIN_SLTU: begin
				next_add_sub = 1;
				next_write_sel = WSEL_COND;
				next_cond_sel = COND_LTU;
				next_cond_neg = 0;
			end
			OP_MAIN_SLL: begin
				next_write_sel = WSEL_SHIFT;
				next_shift_right = 0;
			end
			OP_MAIN_SRL: begin
				next_write_sel = WSEL_SHIFT;
				next_shift_right = 1;
				next_shift_arith = op_funct7[5];
			end
			OP_MAIN_XOR: begin
				next_write_sel = WSEL_BIT;
				next_bit_op = BIT_OP_XOR;
			end
			OP_MAIN_OR: begin
				next_write_sel = WSEL_BIT;
				next_bit_op = BIT_OP_OR;
			end
			OP_MAIN_AND: begin
				next_write_sel = WSEL_BIT;
				next_bit_op = BIT_OP_AND;
			end
			default: next_state = STATE_FETCH;
			endcase
			next_state = STATE_EXEC_WRITE;
		end
		OP_BRANCH: begin
			next_imm = op_b_imm;
			next_add_mux1 = ADD_MUX1_RS1;
			next_add_mux2 = ADD_MUX2_RS2;
			next_add_sub = 1;
			next_state = STATE_BC;
			next_bit_op = BIT_OP_XOR;
			case (op_funct3)
			OP_BRANCH_BLT: begin
				next_cond_sel = COND_LT;
				next_cond_neg = 0;
			end
			OP_BRANCH_BLTU: begin
				next_cond_sel = COND_LTU;
				next_cond_neg = 0;
			end
			OP_BRANCH_BGE: begin
				next_cond_sel = COND_LT;
				next_cond_neg = 1;
			end
			OP_BRANCH_BGEU: begin
				next_cond_sel = COND_LTU;
				next_cond_neg = 1;
			end
			OP_BRANCH_BEQ: begin
				next_cond_sel = COND_EQ;
				next_cond_neg = 0;
			end
			OP_BRANCH_BNE: begin
				next_cond_sel = COND_EQ;
				next_cond_neg = 1;
			end
			endcase
		end
		OP_JAL: begin
			next_imm = op_j_imm;
			next_add_mux1 = ADD_MUX1_PC;
			next_add_mux2 = ADD_MUX2_FOUR;
			next_add_sub = 0;
			next_state = STATE_JAL;
		end
		OP_JALR: begin
			next_imm = op_i_imm;
			next_add_mux1 = ADD_MUX1_PC;
			next_add_mux2 = ADD_MUX2_FOUR;
			next_add_sub = 0;
			next_state = STATE_JALR;
		end
		default: next_state = STATE_FETCH;
		endcase
	end
	STATE_EXEC_WRITE: begin
		case (write_sel)
			WSEL_ADD: reg_wdata = add_out;
			WSEL_COND: reg_wdata = cond;
			WSEL_BIT: reg_wdata = bit_out;
			WSEL_SHIFT: reg_wdata = shift_out;
		endcase
		reg_we = 1;
		next_state = STATE_FETCH;
	end
	STATE_BC: begin
		jump = cond;
		next_add_mux1 = ADD_MUX1_PC;
		next_add_mux2 = ADD_MUX2_IMM;
		next_state = STATE_FETCH;
	end
	STATE_JAL: begin
		reg_wdata = add_out;
		reg_we = 1;
		next_add_mux1 = ADD_MUX1_PC;
		next_add_mux2 = ADD_MUX2_IMM;
		jump = 1;
		next_state = STATE_FETCH;
	end
	STATE_JALR: begin
		reg_wdata = add_out;
		reg_we = 1;
		next_add_mux1 = ADD_MUX1_RS1;
		next_add_mux2 = ADD_MUX2_IMM;
		jump = 1;
		next_state = STATE_FETCH;
	end
	STATE_LOAD_A: begin
		mem_addr = add_out[31:2];
		next_mem_loaddr = add_out[1:0];
		mem_en = 1;
		next_state = STATE_LOAD_D;
	end
	STATE_LOAD_D: begin
		reg_wdata = ld_out;
		reg_we = 1;
		next_state = STATE_FETCH;
	end
	STATE_STORE: begin
		mem_addr = add_out[31:2];
		mem_en = 1;
		case (mem_width)
		3'b000: begin
			mem_we = 4'h1 << add_out[1:0];
			mem_wdata = {4{reg_rs2[7:0]}};
		end
		3'b001: begin
			mem_we = 4'h3 << (add_out[1] * 2);
			mem_wdata = {2{reg_rs2[15:0]}};
		end
		3'b010: begin
			mem_we = 4'hf;
			mem_wdata = reg_rs2;
		end
		default: begin
			mem_we = 4'hx;
			mem_wdata = 32'hxxxxxxxx;
		end
		endcase
		next_state = STATE_FETCH;
	end
	endcase
end

always @* begin
	case (cond_sel)
	COND_LT: cond = add_lt ^ cond_neg;
	COND_LTU: cond = add_ltu ^ cond_neg;
	COND_EQ: cond = !bit_out ^ cond_neg;
	default: cond = 1'bx;
	endcase
end

always @(posedge clk) begin
	if (state == STATE_DECODE) begin
		dec_rd <= op_rd;
		dec_imm <= next_imm;
		add_mux1 <= next_add_mux1;
		add_mux2 <= next_add_mux2;
		add_sub <= next_add_sub;
		reg_rs1 <= regs[op_rs1];
		reg_rs2 <= regs[op_rs2];
		cond_sel <= next_cond_sel;
		cond_neg <= next_cond_neg;
		write_sel <= next_write_sel;
		bit_op <= next_bit_op;
		shift_arith <= next_shift_arith;
		shift_right <= next_shift_right;
		mem_width <= next_mem_width;
	end
	if (state == STATE_LOAD_A) begin
		mem_loaddr <= next_mem_loaddr;
	end
	if (next_state == STATE_FETCH) begin
		if (jump) begin
			add_mux1 <= next_add_mux1;
			add_mux2 <= next_add_mux2;
		end else begin
			add_mux1 <= ADD_MUX1_PC;
			add_mux2 <= ADD_MUX2_FOUR;
		end
		add_sub <= 0;
	end
	if (state == STATE_RESET)
		pc <= -4;
	else if (state == STATE_FETCH)
		pc <= add_out;
	if (reg_we && dec_rd) begin
		regs[dec_rd] <= reg_wdata;
	end
	if (rst)
		state <= STATE_RESET;
	else
		state <= next_state;
end


// Add

wire [31:0] add_nin2 = add_sub ? ~add_in2 : add_in2;
wire [32:0] add_tmp = {1'b0, add_in1} + {1'b0, add_nin2} + add_sub;
assign add_out = add_tmp[31:0];
assign add_ltu = ~add_tmp[32];
assign add_lt = (add_in1[31] == add_in2[31]) ? add_ltu : add_in1[31];

// Logic

always @* begin
	case (bit_op)
		BIT_OP_AND: bit_out = reg_rs1 & add_in2;
		BIT_OP_OR: bit_out = reg_rs1 | add_in2;
		BIT_OP_XOR: bit_out = reg_rs1 ^ add_in2;
		default: bit_out = 32'hxxxxxxxx;
	endcase
end

// Barrel shifter

reg shift_sign;
always @* begin
	shift_sign = reg_rs1[31] & shift_arith;
	if (shift_right)
		shift_out = {{32{shift_sign}}, reg_rs1} >> add_in2[4:0];
	else
		shift_out = reg_rs1 << add_in2[4:0];
end

// Load data

reg [31:0] ld_tmp;
always @* begin
	ld_tmp[31:16] = mem_rdata[31:16];
	ld_tmp[15:8] = mem_loaddr[1] ? mem_rdata[31:24] : mem_rdata[15:8];
	ld_tmp[7:0] = mem_rdata[mem_loaddr * 8 +: 8];
	case (mem_width)
	3'b000: ld_out = {{24{ld_tmp[7]}}, ld_tmp[7:0]};
	3'b001: ld_out = {{16{ld_tmp[15]}}, ld_tmp[15:0]};
	3'b010: ld_out = ld_tmp;
	3'b100: ld_out = {24'd0, ld_tmp[7:0]};
	3'b101: ld_out = {16'd0, ld_tmp[15:0]};
	default: ld_out = 32'hxxxxxxxx;
	endcase
end

endmodule
