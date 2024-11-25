module RFID (input CLOCK_50,      // 50 MHz system clock
    input [3:0] KEY,            // Active low reset  
    input [0:0] GPIO_0,         // MISO (Not used in sending)
	 input [9:0] SW,
    output [2:0] GPIO_1,         // MOSI
    output [9:0] LEDR,              // LEDR[0] signal
	 output [7:0] LEDG
);
	// spi_master common inputs
	// parameter bits = 8; // Defines the size of the dataIn Register
	parameter spi_clk_div = 28'd1562500; // How much to divide the 50MHz transfer to. 6250000 will give us 8Hz.
	reg [7:0] data_in;
	wire [7:0] data_out;
	wire reset;
	wire busy_transaction;
	reg start = 1;
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
   // assign start = KEY[1];
	assign miso = GPIO_0[0];
	assign GPIO_1[0] = mosi;
	assign GPIO_1[1] = ss;
	assign GPIO_1[2] = spi_clk;
	assign LEDR[0] = busy_transaction;
	assign LEDR[8:1] = data_in[7:0];
	assign LEDR[9] = test;
	
	// test
	assign LEDG[7:0] = data_out[7:0];
	reg test = 0;
	
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
	
	
	reg [27:0] clk_div;           // Clock divider register
    wire sclk_enable = (clk_div == 28'd0); // Enable signal for SPI clock toggle
	reg clock2;
	
    always @(posedge CLOCK_50) begin
		clk_div <= (clk_div == 28'd0) ? 28'd5000000 : clk_div - 1; // Generate 8 Hz SPI clock
    end
	
	reg [31:0] step = 32'd0;
	reg [31:0] process = 32'd0;
	wire control;
	
	// RC522 booleans
	reg isAntennaOn = 0;
	reg [7:0] antennaValue;
	
	// temp
	assign control = ~KEY[2]; // manual control now
	
   always @(posedge CLOCK_50) begin
        if (~reset) begin
            step <= 0;        // Reset to step 0
            data_in = 0;      // Reset the result
        end else begin
				// if (sclk_enable)
				// begin
				
				case (process) // Process = which process you want to run, there is multiple
					32'd0: // Process #1 RC_522 Initialization
					begin
						if (start == 0)
					   begin
							start = 1;
					   end
						if (control && sclk_enable)
						begin
							clock2 <= ~clock2;
							if (clock2)
							begin
								case (step) // Steps = Sequence in Operations
								
								 32'd0: begin // TxModeReg Reset
									loadReg(8'h12); 
								 end
								 32'd1: begin
									send();
								 end
								 32'd2: begin 
									 loadCom(8'h00); 
								 end 
								 32'd3: begin
									 send();
								 end
								 
								 32'd4: begin // RxModeReg Reset
									loadReg(8'h13); 
								 end
								 32'd5: begin
									send();
								 end
								 32'd6: begin
									 loadCom(8'h00); 
								 end 
								 32'd7: begin 
									 send();
								 end
								 
								 32'd8: begin // modWidthReg Reset
									loadReg(8'h24); 
								 end
								 32'd9: begin 
									 send();
								 end
								 32'd10: begin
									 loadCom(8'h26); 
								 end 
								 32'd11: begin 
									 send();
								 end
								 
								 32'd12: begin // Other Registers Reset 0
									 loadReg(8'h2A); 
								 end 
								 32'd13: begin 
									 send();
								 end
								 32'd14: begin
									 loadCom(8'h80); 
								 end 
								 32'd15: begin 
									 send();
								 end
								 
								 32'd16: begin // Other Registers Reset 1
									 loadReg(8'h2B); 
								 end 
								 32'd17: begin 
									 send();
								 end
								 32'd18: begin
									 loadCom(8'hA9); 
								 end 
								 32'd19: begin 
									 send();
								 end
								 
								 32'd20: begin // Other Registers Reset 2
									 loadReg(8'h2C); 
								 end 
								 32'd21: begin 
									 send();
								 end
								 32'd22: begin
									 loadCom(8'h03); 
								 end 
								 32'd23: begin 
									 send();
								 end
								 
								 32'd24: begin // Other Registers Reset 3
									 loadReg(8'h2D); 
								 end 
								 32'd25: begin 
									 send();
								 end
								 32'd26: begin
									 loadCom(8'hE8); 
								 end 
								 32'd27: begin 
									 send();
								 end
								 
								 32'd28: begin // TxASKReg Reset
									 loadReg(8'h15); 
								 end
								 
								 32'd29: begin 
									 send();
								 end
								 32'd30: begin
									 loadCom(8'h40); 
								 end 
								 32'd31: begin 
									 send();
								 end
								 
								 32'd32: begin // TxSelReg Reset
									 loadReg(8'h11); 
								 end 
								 32'd33: begin 
									 send();
								 end
								 32'd34: begin
									 loadCom(8'h3D); 
								 end 
								 32'd35: begin 
									 send();
								 end
								 
								 
								 // ---------- Resetting Antenna -------- //
								 
								 // Reading Register Given Register Address.
								 32'd36: begin 
									 loadRegRead(8'h14); // 8'h80 to turn MSB == 1 (0x80) for reading. The first value is just the register.
								 end 
								 32'd37: begin 
									 send();
								 end
								 
								 32'd38: begin
									loadCom(8'hAA);
								 /*
									if ((data_out & 8'h03) != 8'h03)
									begin
										// Do stuff here
									end
									else
									begin
										test <= 1;
									end
									step <= step + 1;
								*/
								 end 
								 32'd39: begin 
									 send();
									 
								 end
								 
								 32'd40: begin
									if (data_out[7:0] == 8'b10000000) // i.e. bin is NOT 10000011 or anything of the form xxxxxx11
									 begin
										// Do stuff here
										isAntennaOn = 1;
										antennaValue <= data_out[7:0];
										
									 end
									 step <= step + 1;
								 end
								 
								 32'd41: begin
									  if (isAntennaOn == 1)
									  begin
											loadReg(8'h14); 
									  end
									  else
									  begin
											step <= step + 1 + 3; // skip next 3 steps
									  end
									  
									  
								 end
								 
								 32'd42: begin
										send(); 
								 end
								 
								 32'd43: begin
										loadCom(antennaValue | 8'h03); 
								 end
								 
								 
								 32'd44: begin
									 send(); 
								 end

								 
								 // ---------- Resetting Antenna End -------- //
								 
								 32'd45: begin
									  // Sequence complete; Hold result

									  step <= step;
								 end
							endcase
							end
							
						end
					end
					32'd1: // Process #2
					begin
	
					end
					
					32'd2: // Process #3 
					begin
					end
					
					
				endcase
				// end
				
        end
   end // End of always
	
	// tasks
	task loadReg;
		input [7:0] data;
		data_in <= (data << 1) & 8'h7F;
		step <= step + 1;
	endtask
	
	task loadRegRead;
		input [7:0] data;
		data_in <= data << 1 | 8'h80;
		step <= step + 1;
	endtask
	
	task loadCom;
		input [7:0] data;
		data_in <= data;
		step <= step + 1;
	endtask
	
	task send;
		start = 0;
		step <= step + 1;
	endtask
	
	
	
	/*
	task rc_writeRegister;
			input [7:0] regAddress; // 8-bit register address
			input [7:0] command; // 8-bit data to write
			
			
			begin
            // Blocking assignment to assign value to the internal register
				start = 0;
            data_in = regAddress;  // Assign constant value to data_in
				// data_in = command;  // Assign constant value to data_in
				start = 1;
			end
			
			
	endtask
	*/
	/*
	
	reg [7:0] regAddress_reg;
	reg [7:0] command_reg;
	wire [7:0] regAddress;
 	wire [7:0] command;
	assign regAddress = regAddress_reg;
	assign command = command_reg;
	wire [7:0] regAddress = 8'h2A;
 	wire [7:0] command = 8'h80; 
	
	wire done;
	register_write w1 (CLOCK_50, 8'h2A, 8'h80, KEY[1], done, start, data_in);

	reg [1:0] register_write_state = 2'b00;
	
	always @ (posedge CLOCK_50)
	begin
		regAddress_reg = 8'h2A;
		command_reg = 8'h80;
		case (register_write_state)
		2'b00 :
		begin
			
		end
		endcase
	end
	*/
	
	/*
	wire [7:0] regAddress1 = 8'h2A;
 	wire [7:0] command1 = 8'h80;
	register_write w2 (CLOCK_50, regAddress1, command1, KEY[2], done, start, data_in);
	*/
	
	
	
	
	
	
	
	
	
	
	// THESE ARE OLD TESTS, THE ABOVE IS THE NEW TASKS NEEDED TO IMPLEMENT RFID
	// -----------------------------------------------------------------------
	
	/*
	task transfer_8(input [7:0] dataIN);
        begin
            // Blocking assignment to assign value to the internal register
            data_in = dataIN;  // Assign constant value to data_in
        end
    endtask
	 */
	 
	 // -----------------------------------------------------------------------
	 
	 /*
	 task transfer_16(input [15:0] dataIN);
        begin
            // Blocking assignment to assign value to the internal register
            data_in = dataIN;  // Assign constant value to data_in
        end
    endtask
	 */
	 
	 // -----------------------------------------------------------------------
	 
	 /*
	task address_formatter (
		input reg [7:0] hex_address,   // 8-bit hexadecimal address input
		output reg [7:0] formatted_address); // 9-bit output with 1 in front
	
		formatted_address = hex_address << 1; // left-bit shift on hex_address 
		
	endtask
	*/
	
	// -----------------------------------------------------------------------

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

	// -----------------------------------------------------------------------
	
	/*
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
	*/

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

