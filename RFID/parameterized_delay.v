module parameterized_delay #(
    parameter DELAY_CYCLES = 20 // Number of clock cycles to delay
)(
    input wire clk,       // Clock signal
    input wire reset,     // Asynchronous reset
    output reg delayed_signal // Output signal after delay
);
    reg [$clog2(DELAY_CYCLES):0] counter; // Counter width based on DELAY_CYCLES

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            counter <= 0;
            delayed_signal <= 1'b0;
        end else if (counter < DELAY_CYCLES) begin
            counter <= counter + 1; // Increment the counter
        end else begin
            delayed_signal <= 1'b1; // Activate the delayed signal
        end
    end
endmodule