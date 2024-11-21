## README.md

`spi_master`

- Contains a simple SPI implementation in Verilog.
  - Can transmit and receive through the MOSI and MISO lines.
- Testing:
  - Can be done on the Arduino, look at GitHub `playground` branch underneath `spi_test/Arduino_Slave`
  - Optionally: You can run `spi_master` from `playground` underneath `spi_test/FPGA_Master`.

- **New Implementation Includes:**

  - Setting the `spi_clk` division as a parameter.
  - Creating the following tasks:
    - `transfer_8()` : Writes 8 bits of data to the SPI bus (MOSI pin)
    - `transfer_16()`: Writes 16 bits of data to the SPI bus (MOSI pin) 
    - `address_formatter()`: Reformats the register address, to the standard of the RC522.
  - Added a few tests for each task.

  