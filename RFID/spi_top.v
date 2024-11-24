module spi_top (input CLOCK_50,      // 50 MHz system clock
    input [3:0] KEY,            // Active low reset  
    input [0:0] GPIO_0,         // MISO (Not used in sending)
	 input [9:0] SW,
    output [2:0] GPIO_1,         // MOSI
    output [9:0] LEDR              // LEDR[0] signal
);
	// spi_master common inputs
	// parameter bits = 8; // Defines the size of the dataIn Register
	parameter spi_clk_div = 28'd6250000; // How much to divide the 50MHz transfer to. 6250000 will give us 8Hz.
	reg [15:0] data_in;
	wire [15:0] data_out;
	wire reset;
	wire busy_transaction;
	wire start;
	wire spi_clk;
	wire ss;
	wire miso;
	wire mosi;
	wire bits_8;
	
	// spi_master 8 bit outputs
	wire [7:0] data_out_8;
	wire busy_transaction_8;
	wire spi_clk_8;
	wire ss_8;
	wire mosi_8;

	// spi_master 16 bit outputs
	wire [15:0] data_out_16;
	wire busy_transaction_16;
	wire spi_clk_16;
	wire ss_16;
	wire mosi_16;
	
	// Using a Multiplexer for Shared Outputs // bits_8 = 1 is 8 bits, bits_8 = 0 is 16 bits (active low)
	assign data_out = bits_8 ? data_out_8 : data_out_16;
	assign busy_transaction = bits_8 ? busy_transaction_8 : busy_transaction_16;
	assign spi_clk = bits_8 ? spi_clk_8 : spi_clk_16;
	assign ss = bits_8 ? ss_8 : ss_16;
	assign mosi = bits_8 ? mosi_8 : mosi_16;
	
	// Using a switch setting bits to 8.
	assign bits_8 = SW[0];
	
	// Using FPGA to transfer
	assign reset = KEY[0];
   assign start = KEY[1];
	assign busy_transaction = LEDR[0];
	assign miso = GPIO_0[0];
	assign GPIO_1[0] = mosi;
	assign GPIO_1[1] = ss;
	assign GPIO_1[2] = spi_clk;
	assign LEDR[0] = busy_transaction;
	assign LEDR[8:1] = data_out;

	
	// 8 bit SPI Bus Instantiation
	spi_master 

	#(
		.bits_transfer(8), .spi_clk_div(spi_clk_div) // Defines the size of the dataIn Register
	) 
	
	U1 (
		 CLOCK_50,          // 50 MHz system clock
		 reset,             // Active low reset
		 start,             // Active low start
		 miso,              // MISO
		 mosi_8,              // MOSI
		 ss_8,                // SS (Slave Select)
		 spi_clk_8,           // SPI Clock
		 busy_transaction_8,  // busy signal for during transfer
		 data_in,           // Data to be transferred
		 data_out_8           // Data to be received
	);
	
	// 16 bit SPI Bus Instantiation
	spi_master 

	#(
		.bits_transfer(16), .spi_clk_div(spi_clk_div) // Defines the size of the dataIn Register
	) 
	
	U2 (
		 CLOCK_50,          // 50 MHz system clock
		 reset,             // Active low reset
		 start,             // Active low start
		 miso,              // MISO
		 mosi_16,              // MOSI
		 ss_16,                // SS (Slave Select)
		 spi_clk_16,           // SPI Clock
		 busy_transaction_16,  // busy signal for during transfer
		 data_in,           // Data to be transferred
		 data_out_16           // Data to be received
	);
	

	task transfer_8(input [7:0] dataIN);
        begin
            // Blocking assignment to assign value to the internal register
            data_in = dataIN;  // Assign constant value to data_in
        end
    endtask
	 
	 task transfer_16(input [15:0] dataIN);
        begin
            // Blocking assignment to assign value to the internal register
            data_in = dataIN;  // Assign constant value to data_in
        end
    endtask
	 
	 
	task address_formatter (
		input reg [7:0] hex_address,   // 8-bit hexadecimal address input
		output reg [7:0] formatted_address); // 9-bit output with 1 in front
	
		formatted_address = hex_address << 1; // left-bit shift on hex_address 
		
	endtask

	/*
	task writeRegister (
		input reg [7:0] register_hex_address, // unformatted register addresss
		input reg [7:0] command_hex_value // command hex value
	);
		reg [7:0] formatted_register;
		
		address_formatter(register_hex_address, formatted_register);
		
		transfer(formatted_register);
		transfer(command_hex_value);
		
	
	endtask
	 */


	
	// Transfer Selector (multiplexer)
	wire [1:0] transfer_select;
	
	//temp assignments
	assign transfer_select[1] = ~KEY[3]; //temp
	assign transfer_select[0] = ~KEY[2]; //temp
	 
	always @(posedge CLOCK_50) 
	begin
		case (transfer_select) // ADD MORE just adjust multiplexer size...
			2'b01: transfer_8(8'hCD); 
			2'b10: transfer_8(8'hEF); 
			2'b11: transfer_8(8'hFA);  
			default: transfer_8(8'h00); // Default Case
	  endcase
	end
	
	// End of Transfer Selector
	

	// -----------------------------------------------------------
	
	
	/*
	// Test for Address Formatter | PASSED
	wire [7:0] unformatted = 8'b00001111;
	wire [7:0] formatted;
	
	always @(posedge CLOCK_50)
	begin
		address_formatter(unformatted, formatted);
	end

	assign LEDR[bits:1] = formatted;
	
	// End of Test for Address Formatter
	*/
	
	
	// -----------------------------------------------------------
	
	
	/*
	// Test for writeRegister | FAILED
	// Currently sends only the command. This seems to be an issue with the number of bits being transferred.
	
	wire [7:0] register = 8'h12;
	wire [7:0] command = 8'h26;
	
	always @(posedge CLOCK_50)
	begin
		writeRegister(register, command);
	end
	
	// End of Test for writeRegister 
	*/
	
endmodule

