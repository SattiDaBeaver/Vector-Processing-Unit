`timescale 1ns / 1ps

module testbench ( );

	parameter CLOCK_PERIOD = 10;

	reg CLOCK_50;

	// Logic
	logic [7:0]		dataIn;
	logic			rst;
	logic			TXen;

	logic			TXout;
	logic 			busy;

	//

	initial begin
        CLOCK_50 <= 1'b0;
	end // initial
	always @ (*)
	begin : Clock_Generator
		#((CLOCK_PERIOD) / 2) CLOCK_50 <= ~CLOCK_50;
	end
	
	initial begin
        rst <= 1'b1;
		TXen <= 0;
        #20 
		rst <= 1'b0;

		#20 
		dataIn <= 8'h70;
		#10
		TXen <= 1;
		#20
		TXen <= 0;
	end // initial
	
	UART_TX #(
		.dataWidth(8),
		.stopBits(2)
	)
	U1 (
		.clk(CLOCK_50),
		.rst(rst),
		.dataIn(dataIn),
		.TXen(TXen),

		.TXout(TXout),
		.busy(busy)
	);

endmodule
