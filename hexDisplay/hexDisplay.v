module hexDisplay(
    input [31:0] total_seconds_elapsed,
    output reg [6:0] HEX0,
    output reg [6:0] HEX1,
    output reg [6:0] HEX2,
    output reg [6:0] HEX3,
    output reg [6:0] HEX4,
    output reg [6:0] HEX5
);

    // Internal variables 
    reg [5:0] seconds;
    reg [5:0] minutes;
    reg [4:0] hours;

    // Convert total_seconds_elapsed to hours, minutes, and seconds
    always @(*) begin
        seconds = total_seconds_elapsed % 60;
        minutes = (total_seconds_elapsed / 60) % 60;
        hours = (total_seconds_elapsed / 3600) % 24;
    end

	 
    function [6:0] seg_decoder;
        input [3:0] digit;
        case (digit)
            4'd0: seg_decoder = 7'b1000000;
            4'd1: seg_decoder = 7'b1111001;
            4'd2: seg_decoder = 7'b0100100;
            4'd3: seg_decoder = 7'b0110000;
            4'd4: seg_decoder = 7'b0011001;
            4'd5: seg_decoder = 7'b0010010;
            4'd6: seg_decoder = 7'b0000010;
            4'd7: seg_decoder = 7'b1111000;
            4'd8: seg_decoder = 7'b0000000;
            4'd9: seg_decoder = 7'b0010000;
            default: seg_decoder = 7'b1111111; // show blank
        endcase
    endfunction

    always @(*) begin
        HEX0 = seg_decoder(seconds % 10); // Lower digit of seconds
        HEX1 = seg_decoder(seconds / 10); // Upper digit of seconds
        HEX2 = seg_decoder(minutes % 10); // Lower digit of minutes
        HEX3 = seg_decoder(minutes / 10); // Upper digit of minutes
        HEX4 = seg_decoder(hours % 10);   // Lower digit of hours
        HEX5 = seg_decoder(hours / 10);   // Upper digit of hours
    end

endmodule