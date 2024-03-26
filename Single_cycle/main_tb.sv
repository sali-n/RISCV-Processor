module main_tb();
	logic clk_i,reset_i,intr;
	
	main UUT(clk_i,reset_i,intr);
	
	initial begin
	 clk_i = 0;
	 forever #5 clk_i = ~clk_i;
	end
	
	initial begin
		reset_i <= 1; intr <= 0;
		@(posedge clk_i);
		reset_i <= 0;
		repeat (200) @(posedge clk_i);
		intr <= 1;
		@(posedge clk_i);
		intr <= 0;
		repeat (50) @(posedge clk_i);
		$stop;
	end
endmodule