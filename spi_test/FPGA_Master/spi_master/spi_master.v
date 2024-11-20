module spi_master 
#(
	parameter bits_transfer = 16, // Defines the size of the dataIn Register
	parameter counter_width = $clog2(bits_transfer) // Defines the size of the register for the counter.
)

(
	 // System Wires
    input wire CLOCK_50,         // 50 MHz system clock
    input wire rst_n,            // Active low reset
    input wire start,            // Start signal for sending data
	 
	 // Slave Wires
    input GPIO_0_0,         // MISO (Not used in sending)
    output reg GPIO_1_0,         // MOSI
    output reg GPIO_1_1,         // SS (Slave Select)
    output reg GPIO_1_2,         // SPI Clock
	 
	 // Transfer Wires
    output reg busy,              // Busy signal while transferring
	 input wire [bits_transfer-1:0] data_in   // Data in  Ex. 16'hDEAD
);
	 // reg [15:0] data_in = 16'hDEAD;           // Data in 
    reg [counter_width:0] bit_count;         // Bit counter
    reg [bits_transfer-1:0] shift_reg;         // 8-bit shift register for number
    reg [1:0] state;             // FSM state

    localparam IDLE     = 2'b00; // Idle state
    localparam LOAD     = 2'b01; // Load data state
    localparam TRANSFER = 2'b10; // Transfer data state
	 // Create read data state

    // SPI clock divider
    reg [27:0] clk_div;           // Clock divider register
    wire sclk_enable = (clk_div == 28'd0); // Enable signal for SPI clock toggle

    always @(posedge CLOCK_50 or negedge rst_n) begin
        if (!rst_n) begin
            clk_div <= 28'd0;
        end else begin
            clk_div <= (clk_div == 28'd0) ? 28'd6250000 : clk_div - 1; // Generate 8 Hz SPI clock
				// clk_div <= (clk_div == 28'd0) ? 28'd400 : clk_div - 1; // Generate 125 kHz SPI clock
        end
    end

    // SPI state machine
    always @(posedge CLOCK_50 or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            GPIO_1_2 <= 0; // SPI Clock
				GPIO_1_1 <= 1; // SS
				// GPIO_1_0 <= 1; //temp
            // GPIO_1_0 <= 0; // MOSI
            busy <= 0;
            bit_count <= 0;
            shift_reg <= 0;
        end else begin
            case (state)
                IDLE: begin
                    GPIO_1_2 <= 0; // SCLK low
						  GPIO_1_0 <= data_in[bits_transfer-1]; //temp
						  bit_count <= 1; // temp
                    // GPIO_1_0 <= 0; // MOSI low
                    GPIO_1_1 <= 1; // SS high (inactive)
                    busy <= 0;
						  // bit_count <= 0;
						  shift_reg <= 0;
                    if (!start) begin
                        state <= LOAD;
                    end
                end

                LOAD: begin
                    GPIO_1_1 <= 0;        // SS low (select slave)
                    shift_reg <= data_in[bits_transfer-2:0]; // Load the number 1010 into the shift register
                    bit_count <= bits_transfer-1;      // Set bit counter for 16 bits
                    busy <= 1;            // Indicate busy
                    state <= TRANSFER;
						  
                end

                TRANSFER: begin
                    if (sclk_enable) begin
                        GPIO_1_2 <= ~GPIO_1_2; // Toggle SPI clock
                        if (GPIO_1_2) begin
                            // Shift data out on rising edge of SCLK
                            GPIO_1_0 <= shift_reg[bits_transfer-2]; // Send MSB first
                            shift_reg <= {shift_reg[bits_transfer-3:0], 1'b0}; // Shift left
                            if (bit_count == 0) begin
                                state <= IDLE; // End transmission
                                GPIO_1_1 <= 1; // SS high (deselect slave)
                                busy <= 0;     // Not busy anymore
                            end else begin
                                bit_count <= bit_count - 1;
                            end
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end


endmodule