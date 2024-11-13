module rfid// this uses the RC522 module to read an rfid tag's UID and returns it. 
# (parameter bits = 32, parameter counter_width = $clog2(bits)) //32 bit UID
(
	// Pass Through System
	input reset_system,
	input clk_system,
	input start_transfer,
	input get_uid,

	// SPI Side
	input miso, // Connect to GPIO1[0]
	output wire mosi, // Connect to GPIO0[3]
	output wire clk_spi, // Connect to GPIO0[2]
	output reg cs // Connect to GPIO0[1]

	
	// I set up GPIO0 as output and GPIO1 as input, such that there is no confusion.

);

	// Register + Addresses (hex)
	parameter CommandReq = 8'h01;
	parameter BitFramingReg = 8'h0D;
	parameter FIFODataReg = 8'h09;
	
	// Command + Codes (hex)
	parameter reqaCmd = 8'h26;
	parameter anticollisionCmd1 = 8'h93;
	parameter anticollisionCmd2 = 8'h20;
	parameter transceiveCmd = 8'h0C;
	parameter idleCmd = 8'h00;
	
	// internal registers
	reg [bits-1:0] data_inR;
	wire [bits-1:0] data_outR;
	reg [counter_width:0] size_transfer;



endmodule


module 