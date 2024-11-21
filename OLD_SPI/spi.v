/// SPI
module spi
# (parameter reg_width = 8, parameter counter_width = $clog2(reg_width), parameter reset = 0, idle = 1, load = 2, transact = 3, unload = 4)
(
	// System Side
	input reset_system,
	input clk_system,
	input start_transfer, 
	input [reg_width-1:0] data_inR,
	input [counter_width:0] size_transfer,
	output reg [reg_width-1:0] data_outR,

	// SPI Side
	input miso, 
	output wire mosi,
	output wire clk_spi,
	output reg cs
);

	reg [reg_width-1:0] mosiR;
	reg [reg_width-1:0] misoR;
	reg [counter_width:0] count;
	reg [2:0] state;

	
	always @(state)
	begin
    case (state)
        reset:
        begin
            data_outR <= 0;
            cs <= 1;
        end
        idle:
        begin
            data_outR <= data_outR;
            cs <= 1;
        end
        load:
        begin
            data_outR <= data_outR;
            cs <= 0;
        end
        transact:
        begin
            cs <= 0;
        end
        unload:
        begin
            data_outR <= misoR;
            cs <= 0;
        end

        default:
            state = reset;
    endcase
	end
	
	always @(posedge clk_system)
	begin
		 if (!reset_system)
			  state = reset;
		 else
			  case (state)
					reset:
						 state = idle;
					idle:
						 if (start_transfer)
							  state = load;
					load:
						 if (count != 0)
							  state = transact;
						 else
							  state = reset;
					transact:
						 if (count != 0)
							  state = transact;
						 else
							  state = unload;
					unload:
						 if (start_transfer)
							  state = load;
						 else
							  state = idle;
			  endcase
	end
	
	assign mosi = ( ~cs ) ? mosiR[reg_width-1] : 1'bz;
	assign clk_spi = ( state == transact ) ? clk_system : 1'b0;

	always @(posedge clk_spi or posedge clk_system)
	begin
		if (clk_system)	
		begin
			case (state)
			  reset:
			  begin
					misoR <= 0;
					mosiR <= 0;
					count <= 0;
			  end
			  idle:
			  begin
					misoR <= 0;
					mosiR <= 0;
					count <= 0;
			  end
			  load:
			  begin
					misoR <= 0;
					mosiR <= data_inR;
					count <= size_transfer;

			  end
			  transact:
			  begin
					
			  end
			  unload:
			  begin
					misoR <= 0;
					mosiR <= 0;
					count <= count;
			  end

			  default:
					state = reset;
			endcase
		
		end

		if (clk_spi)
		begin
			if (state == transact)
			begin
				misoR <= {misoR[reg_width-2:0], miso};
				mosiR <= {mosiR[reg_width-2:0], 1'b0};
				count <= count-1;
			end
			
		
		end
	
	end

endmodule