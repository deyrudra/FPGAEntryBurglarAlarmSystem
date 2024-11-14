module rfid // this uses the RC522 module to read an rfid tag's UID and returns it, it will then compare it to a registered (stored) UID.
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
	reg [7:0] currentReg;
	reg [7:0] currentCmd;
	
	
endmodule


module rfid_formatRegAddr (regAddr, formattedRegAddr, WR); // ONLY for register Addressses
	input wire [7:0] regAddr;
	output reg [7:0] formattedRegAddr;
	input WR;
	
	always @(regAddr)
	begin
		formattedRegAddr = regAddr << 1;
		formattedRegAddr[7] = ~WR;
	end
	
	

endmodule



module rfid_writeReg (regAddr, cmd, clk_system);
	// Parameters
	parameter reg_width = 8;
	parameter counter_width = $clog2(reg_width);

	// External Signals
	input wire [7:0] regAddr;
	input wire [7:0] cmd;
	input clk_system;
	
	// Internal Signals
	wire [7:0] formattedRegAddr;
	reg [reg_width-1:0] data_inR;
	reg [counter_width:0] size_transfer;
	reg start_transfer;
	wire [7:0] spi_data_out;
	wire spi_cs;
	wire spi_clk;
	wire spi_mosi;

	
	rfid_formatRegAddr U1 (.regAddr(regAddr), .formattedRegAddr(formattedRegAddr), .WR(1'b1));
	
	// Write code for SPI now.
	
	// Basic Idea is to set data_inR to regAddr followed by formattedRegAddr.
		
	
	// SPI Module Instance
	spi #(.reg_width(reg_width), .counter_width(counter_width)) spi_inst 
	(
		.reset_system(1'b1),           // Assume always enabled for now
		.clk_system(clk_system),
		.start_transfer(start_transfer),
		.data_inR(data_inR),
		.size_transfer(size_transfer),
		.data_outR(spi_data_out),
		.miso(1'b0),                   // Placeholder for MISO input
		.mosi(spi_mosi),
		.clk_spi(spi_clk),
		.cs(spi_cs)
	);
	
	
	initial begin
		start_transfer = 0;
		size_transfer = 8;
		
		// Step 1: First register selection
		@(posedge clk_system);
		data_inR = formattedRegAddr;
		start_transfer = 1;
		@(posedge clk_system);
		start_transfer = 0;
		
		// Wait until transfer completes
		wait (spi_cs == 1);
		
		// Step 2: Second send command
		@(posedge clk_system);
		data_inR = cmd;
		start_transfer = 1;
		@(posedge clk_system);
		start_transfer = 0;
	end
	
	
	

endmodule



module rfid_obtainUID (UID);
	output reg [31:0] UID;

endmodule

