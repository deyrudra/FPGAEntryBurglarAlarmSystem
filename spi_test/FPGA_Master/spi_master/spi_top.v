module spi_top (input CLOCK_50,      // 50 MHz system clock
    input [1:0] KEY,            // Active low reset  
    input [0:0] GPIO_0,         // MISO (Not used in sending)
	 input [9:0] SW,
    output [2:0] GPIO_1,         // MOSI
    output [0:0] LEDR              // LEDR[0] signal
);

	parameter bits = 8; // Defines the size of the dataIn Register

	wire [bits-1:0] data;

	assign data = SW[0] ? 8'hAA :
				  SW[1] ? 8'hBB :
				  SW[2] ? 8'hCC :
				  SW[3] ? 8'hDD :
				  SW[4] ? 8'hEE :
				  SW[5] ? 8'hFF :
				  SW[6] ? 8'h11 : 
				  SW[7] ? 8'h22 : 
				  SW[8] ? 8'h33 : 
				  SW[9] ? 8'h44 : 8'h55;
					   
	
	spi_master 

	#(
		.bits_transfer(bits) // Defines the size of the dataIn Register
	) 
	
	U1 (
		 CLOCK_50,         // 50 MHz system clock
		 KEY[0],            // Active low reset
		 KEY[1],            // KEY[1] signal for sending data Start Button (Active Low)
		 GPIO_0[0],         // MISO (Not used in sending)
		 GPIO_1[0],         // MOSI
		 GPIO_1[1],         // SS (Slave Select)
		 GPIO_1[2],         // SPI Clock
		 LEDR[0],            // LEDR[0] signal for during transfer
		 data           // Data to be transferred
	); 


endmodule

