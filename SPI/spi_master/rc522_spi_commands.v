module address_formatter (
    input wire [7:0] hex_address,   // 8-bit hexadecimal address input
    output reg [7:0] formatted_address // 9-bit output with 1 in front
);
    always @* begin
        formatted_address = hex_address << 1; // left-bit shift on hex_address
		  
    end
endmodule
