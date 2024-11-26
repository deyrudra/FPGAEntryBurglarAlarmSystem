module security_system (input CLOCK_50,
    input [3:0] KEY,
    input [1:0] GPIO_0,
	 input [9:0] SW,
    output [5:0] GPIO_1,
    output [9:0] LEDR,
	 output [7:0] LEDG,
	 output reg [6:0] HEX0, 
    output reg [6:0] HEX1,  
    output reg [6:0] HEX2,   
    output reg [6:0] HEX3,    
    output reg [6:0] HEX4,    
    output reg [6:0] HEX5     
	 
	 );


	// SPI Wires and Parameters
	// parameter spi_clk_div = 28'd390625; 
	parameter spi_clk_div = 28'd48828; 
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
	
	// FPGA SPI MASTER -> ARDUINO (soft) SPI SLAVE I/O
	assign reset = KEY[0];
   // assign start = KEY[1];
	assign miso = GPIO_0[0];
	assign GPIO_1[0] = mosi;
	assign GPIO_1[1] = ss;
	assign GPIO_1[2] = spi_clk;
	assign LEDR[0] = busy_transaction;
	assign LEDR[8:1] = data_in[7:0];
	// assign LEDR[9] = test;
	
		
	// External LEDs I/O
	reg authenticated = 0;
	reg engaged = 0;
	assign GPIO_1[3] = authenticated;
	assign GPIO_1[4] = engaged;
	
	
	// HC-SR501 I/O
	wire motionDetected;
	assign motionDetected = GPIO_0[1]; // HC-SR501 pin
	assign LEDR[9] = motionDetected;
	
	// Voltage Buzzer I/O
	reg buzzer = 0;
	assign GPIO_1[5] = buzzer;
	
	// Countdown
	reg start_timer = 0;
	
	
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
		 mosi,              // MOSI
		 ss,                // SS (Slave Select)
		 spi_clk,           // SPI Clock
		 busy_transaction,  // busy signal for during transfer
		 data_in,           // Data to be transferred
		 data_out           // Data to be received
	);
	
	

	// DATA TRANSFER CLOCK
	reg [27:0] clk_div; // Clock divider register
    wire sclk_enable = (clk_div == 28'd0); // Enable signal for SPI clock toggle
	
	
    always @(posedge CLOCK_50) begin
		// clk_div <= (clk_div == 28'd0) ? 28'd5000000 : clk_div - 1; 
		clk_div <= (clk_div == 28'd0) ? 28'd500000 : clk_div - 1; 
    end
	
	
	// Registers and Wires ------------------------------------------------------

	// General
	reg [2:0] process = 2'd0;
	
	// Process #1 Registers and Wires
	reg [31:0] step = 32'd0;
	wire control;
	reg [31:0] currentUID = 32'h00000000;
	reg clock2;
	
	// System State Machine
	reg [2:0] system_state = 2'd0; // 1: System Disengaged Mode, 2: System Engaged Mode, 3: Countdown Mode, 4: Alert Mode
	reg clock3;
	wire control2;
	wire engageSystem;
	assign engageSystem = ~KEY[1];
	reg [7:0] systemStateCommand = 8'hBA;
	 
	// temp
	// assign control = ~KEY[2]; // manual control for now 
	// assign control2 = ~KEY[2]; // manual control for now 
	assign control = 1; // manual control for now 
	assign control2 = 1; // manual control for now 
	 
	 //---------------------------------------------------
	 
    // Registers for timekeeping
    reg [25:0] sec_count;       // Counter for one-second interval
    reg [5:0] seconds = 6'd50; // Seconds (0 to 59)
    reg [5:0] minutes = 6'd0;  // Minutes (0 to 59)
	 reg counter_is_zero = 0;
    // 7-segment decoder function
    function [6:0] seg7;
        input [3:0] digit;
        case (digit)
            4'd0: seg7 = 7'b1000000;
            4'd1: seg7 = 7'b1111001;
            4'd2: seg7 = 7'b0100100;
            4'd3: seg7 = 7'b0110000;
            4'd4: seg7 = 7'b0011001;
            4'd5: seg7 = 7'b0010010;
            4'd6: seg7 = 7'b0000010;
            4'd7: seg7 = 7'b1111000;
            4'd8: seg7 = 7'b0000000;
            4'd9: seg7 = 7'b0010000;
            default: seg7 = 7'b1111111; // Blank
        endcase
    endfunction

    // Clock divider for 1-second pulses
    always @(posedge CLOCK_50) begin
		  if (~start_timer) begin
            sec_count <= 0;
        end else if (sec_count == 50000000 - 1) begin
            sec_count <= 0;
        end else begin
            sec_count <= sec_count + 1;
        end
    end

    // Countdown logic
    always @(posedge CLOCK_50) begin
		  if (~start_timer) begin
            seconds <= 6'd60;
            minutes <= 6'd0;
				counter_is_zero <= 0;
        end else if (sec_count == 50000000 - 1) begin
            if (seconds == 0) begin
                if (minutes > 0) begin
                    minutes <= minutes - 1;
                    seconds <= 6'd59;
                end else begin
						counter_is_zero <= 1;
					 end
            end else begin
                seconds <= seconds - 1;
            end
        end
    end
	 	
	always @(*) begin
        HEX0 = seg7(seconds % 10);       // Least significant digit of seconds
        HEX1 = seg7(seconds / 10);       // Most significant digit of seconds
        HEX2 = seg7(minutes % 10);             // Optional colon (blank in this case)
        HEX3 = seg7(minutes / 10);          // Least significant digit of minutes
        HEX4 = 7'b1111111;      // Most significant digit of minutes
        HEX5 = 7'b1111111;               // Blank
    end
	 
 //---------------------------------------------------
	
   always @(posedge CLOCK_50) begin
        if (~reset) begin
            step <= 0;        // Reset to step 0
            data_in = 0;      // Reset the result
				authenticated <= 0;
				start_timer <= 0;
				system_state <= 2'd0;
				buzzer <= 0;
				
				
        end else begin
		  
				// System State Machine --------------------------------------------------------
				if (sclk_enable)
				begin
					clock3 <= ~clock3;
					if (clock3)
					begin
						case (system_state) // Steps = Sequence in Operations
							2'd0: begin // 1: System Disengaged Mode
								engaged <= 0; // Engage Status
								start_timer <= 0;
								buzzer <= 0;
								if (engageSystem)
								begin
									system_state <= 2'd1;
									authenticated <= 0; // Set authenticated to 0, system is no longer authenticated.
								end
								else
								begin
									system_state <= system_state;
								end
							end
							2'd1: begin // 2: System Engaged Mode
								engaged <= 1; // Engage Status
								if (authenticated)
								begin
									system_state <= 2'd0;
								end
								else if (motionDetected) // burglar detected
								begin
									system_state <= 2'd2;
								end
								else
								begin
									system_state <= system_state;
								end
							end
							2'd2: begin // 3: Countdown Mode
								start_timer <= 1;
								if (authenticated)
								begin
									system_state <= 2'd0;
								end
								else if (counter_is_zero == 1)
								begin
									system_state <= 2'd3;
								end
							end
							2'd3: begin // 4: Alert Mode
								if (authenticated)
								begin
									system_state <= 2'd0;
								end
								else
								begin
									// Alarms, buzzers, vga output. 
									buzzer <= 1;
									system_state <= system_state;
								end
							end
						endcase
					end
					
				end
				// End of System State Machine --------------------------------------------------
				
				
				case (process) // Process = which process you want to run, there is multiple
				// Process #1 Obtaining the UID --------------------------------------------------
					2'd0:
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
								32'd0: begin
									loadData(8'hAA); // 0xAA is command for give me UID if key present
								end
								
								32'd1: begin
									sendData();
								end
								
								32'd2: begin
									loadData(8'h00);
								end
								
								32'd3: begin
									sendData();
								end
								
								32'd4: begin
									currentUID[31:24] = data_out[7:0];
									step <= step + 1;
								end
								
								32'd5: begin
									loadData(8'h00);
								end
								
								32'd6: begin
									sendData();
								end
								
								32'd7: begin
									currentUID[23:16] = data_out[7:0];
									step <= step + 1;
								end
								
								32'd8: begin
									loadData(8'h00);
								end
								
								32'd9: begin
									sendData();
								end
								
								32'd10: begin
									currentUID[15:8] = data_out[7:0];
									step <= step + 1;
								end
								
								32'd11: begin
									loadData(8'h00);
								end
								
								32'd12: begin
									sendData();
								end
								
								32'd13: begin
									currentUID[7:0] = data_out[7:0];
									step <= step + 1;
								end
								 
							   32'd14: begin
									if (currentUID[31:0] == 32'h332C1EB7 || currentUID[31:0] == 32'h336BF410)
									begin
										authenticated <= 1;
									end
									step <= step + 1;
								end
								
								32'd15: begin
									if (system_state == 2'd0)
									begin
										systemStateCommand = 8'hBA;
									end
									else if (system_state == 2'd1)
									begin
										systemStateCommand = 8'hBB;
									end
									else if (system_state == 2'd2)
									begin
										systemStateCommand = 8'hBC;
									end
									else if (system_state == 2'd3)
									begin
										systemStateCommand = 8'hBD;
									end
									step <= step + 1;
								end
								
								32'd16: begin
									loadData(systemStateCommand);
								end
								
								32'd17: begin
									sendData();
								end
								
								32'd18: begin
									  step <= 32'd0;
								end
								endcase
							end
							
						end
					end
					// End Of Process #1 Obtaining the UID --------------------------------------------------

				endcase
				// end
        end
		  
		  
   end // End of always
	
	// tasks
	task loadData;
		input [7:0] data;
		data_in <= data;
		step <= step + 1;
	endtask
	
	task sendData;
		start = 0;
		step <= step + 1;
	endtask
	
	
endmodule
