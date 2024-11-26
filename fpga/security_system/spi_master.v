module spi_master 
#(
	parameter bits_transfer = 8, // size of the dataIn Register
	parameter counter_width = $clog2(bits_transfer), // size of the register for the counter.
	parameter spi_clk_div = 28'd6250000 // How much to divide by for the speed of the slave clock.
)

(
	 // System Wires
    input wire CLOCK_50,
    input wire reset, // Active low reset
    input wire start, // Start signal for sending data
	 
	 // Slave Wires
    input miso, // MISO GPIO_0_0
    output reg mosi, // MOSI GPIO_1_0
    output reg ss, // SS GPIO_1_1
    output reg spi_clk, // SPI Clock GPIO_1_2
	 
	 // Transfer Wires
    output reg busy, // signal turned on when transferring data
	input wire [bits_transfer-1:0] data_in, // Data in  Ex. 16'hAA
	output reg [bits_transfer-1:0] data_out // Data out
);
    reg [counter_width:0] bit_count;         // Bit counter
    reg [bits_transfer-1:0] mosiR;         // 8-bit shift register for number
	reg [bits_transfer-1:0] misoR;
    reg [1:0] state;             // FSM state

    localparam IDLE     = 2'b00; // Idle state
    localparam LOAD     = 2'b01; // Load data state
    localparam TRANSFER = 2'b10; // Transfer data state
	 // Create read data state

    // SPI clock divider
    reg [27:0] clk_div;           // Clock divider register
    wire sclk_enable = (clk_div == 28'd0); // Enable signal for SPI clock toggle

    always @(posedge CLOCK_50 or negedge reset) begin
        if (!reset) begin
            clk_div <= 28'd0;
        end else begin
            clk_div <= (clk_div == 28'd0) ? spi_clk_div : clk_div - 1; // Generate 8 Hz SPI clock
				// clk_div <= (clk_div == 28'd0) ? 28'd400 : clk_div - 1; // Generate 125 kHz SPI clock
        end
    end

    // SPI state machine
    always @(posedge CLOCK_50 or negedge reset) begin
        if (!reset) begin
            state <= IDLE;
            spi_clk <= 0; // SPI Clock
				ss <= 1; // SS
            busy <= 0;
            bit_count <= 0;
            mosiR <= 0;
				misoR <= 0;
				data_out <= 0;
        end else begin
            case (state)
                IDLE: begin
                    spi_clk <= 0; // SCLK low
						  mosi <= data_in[bits_transfer-1]; // MSB
						  bit_count <= 1; // temp
                    ss <= 1; // SS high (inactive)
                    busy <= 0;
						  bit_count <= 0;
						  mosiR <= 0;
                    if (!start) begin
                        state <= LOAD;
                    end
                end

                LOAD: begin
                    ss <= 0;        // SS low (select slave)
                    mosiR <= data_in[bits_transfer-2:0]; // Load the number 1010 into the shift register
                    bit_count <= bits_transfer-1;      // Set bit counter for 16 bits
                    busy <= 1;            // Indicate busy
                    state <= TRANSFER;
						  
                end

                TRANSFER: begin
                    if (sclk_enable) begin
                            spi_clk <= ~spi_clk; // Toggle SPI clock
                        if (spi_clk) begin
                            // Shift data out on rising edge of SCLK
                            mosi <= mosiR[bits_transfer-2]; // Send MSB first
                            mosiR <= {mosiR[bits_transfer-3:0], 1'b0}; // Shift left
									 misoR <= {misoR[bits_transfer-2:0], miso};
                            if (bit_count == 0) begin
                                state <= IDLE; // End transmission
                                ss <= 1; // SS high (deselect slave)
                                busy <= 0;     // Not busy anymore
										  data_out <= {misoR[bits_transfer-2:0], miso}; // test
                            end else begin
                                bit_count <= bit_count - 1;
                            end
								 end
								 
								 if (~spi_clk) begin
									
								 end

                    end
                end

                default: state <= IDLE;
            endcase
        end
		  
		
    end
endmodule
