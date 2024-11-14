`timescale 1ms/1ms
module hexDisplay_tb();

    // Testbench variables
    reg [31:0] total_seconds_elapsed;
    wire [6:0] HEX0;
    wire [6:0] HEX1;
    wire [6:0] HEX2;
    wire [6:0] HEX3;
    wire [6:0] HEX4;
    wire [6:0] HEX5;

    // Instantiate the time_display module
    hexDisplay uut (
        .total_seconds_elapsed(total_seconds_elapsed),
        .HEX0(HEX0),
        .HEX1(HEX1),
        .HEX2(HEX2),
        .HEX3(HEX3),
        .HEX4(HEX4),
        .HEX5(HEX5)
    );

    // Test procedure
    initial begin
        // Initialize with the test value
        total_seconds_elapsed = 1509; // 1509 seconds = 25 minutes and 9 seconds
        
        // Wait for a short time to observe outputs
        #10;
        
        // Display output in the console
        $display("Total seconds: %d", total_seconds_elapsed);
        $display("HEX0: %b", HEX0);
        $display("HEX1: %b", HEX1);
        $display("HEX2: %b", HEX2);
        $display("HEX3: %b", HEX3);
        $display("HEX4: %b", HEX4);
        $display("HEX5: %b", HEX5);
        
        // Finish simulation
        #10 $finish;
    end

endmodule