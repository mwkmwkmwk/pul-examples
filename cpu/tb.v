
module tb;

reg clk;

top moj_top(
	.CLK(clk),
	.BTN(6'b111111),
	.RX(1'b1)
);

initial clk = 0;

always begin
	#5 clk <= 1;
	#5 clk <= 0;
end

initial begin
	$dumpfile("cpu.vcd");
	$dumpvars(0, moj_top);
	#3000000
	$finish;
end

endmodule
