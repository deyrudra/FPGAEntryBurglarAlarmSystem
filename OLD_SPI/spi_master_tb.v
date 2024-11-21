`timescale 1ms/1ms

module spi_master_tb();

	parameter bits = 32;

	reg clk_system;
	reg start_transfer;
	reg [bits-1:0] data_inR;
	wire [bits-1:0] data_outR;
	reg [$clog2(bits):0] size_transfer;
	wire cs;
	reg reset_system;
	wire clk_spi;
	wire miso;
	wire mosi;

	spi
	#(
		.reg_width(bits)
	) spi
	(
		.clk_system(clk_system),
		.start_transfer(start_transfer),
		.data_inR(data_inR),
		.data_outR(data_outR),
		.size_transfer(size_transfer),
		.cs(cs),
		.reset_system(reset_system),
		.clk_spi(clk_spi),
		.miso(miso),
		.mosi(mosi)
	);

	assign miso = mosi;
	always
		#2 clk_system = !clk_system;

	initial
	begin
		clk_system = 0;
		start_transfer = 0;
		data_inR = 0;
		reset_system = 0;
		size_transfer = bits;
		#4;
		reset_system = 1;
	end

	initial
	begin
		$dumpfile("simple_spi.lxt");
			$dumpvars(0,spi);
	end

	integer i;
	task transact_test;
		input [bits-1:0] data;
		begin
			data_inR = data[bits-1:0];
			#3 start_transfer = 1;
			#4 start_transfer = 0;
			for( i=0; i < bits; i = i + 1)
			begin
				#4;
			end
			#16;
		end
	endtask	

	initial
	begin
		#10;
		transact_test( {1'b0, 64'hDEADBEEF} );
		$finish;
	end

endmodule