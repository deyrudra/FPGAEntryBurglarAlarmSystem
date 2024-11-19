module spi_top (input CLOCK_50,      // 50 MHz system clock
    input [1:0] KEY,            // Active low reset  
    output [0:0] GPIO_0,         // MISO (Not used in sending)
    output [2:0] GPIO_1,         // MOSI
    output [0:0] LEDR              // LEDR[0] signal
);

spi_master U1 (
    CLOCK_50,         // 50 MHz system clock
    KEY[0],            // Active low reset
    KEY[1],            // KEY[1] signal for sending data
    GPIO_0[0],         // MISO (Not used in sending)
    GPIO_1[0],         // MOSI
    GPIO_1[1],         // SS (Slave Select)
    GPIO_1[2],         // SPI Clock
    LEDR[0]              // LEDR[0] signal
); 


endmodule