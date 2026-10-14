module $__ICE40_RAM4K_ (...);

parameter INIT = 0;
parameter OPTION_HAS_BE = 0;
parameter WIDTH = 16;
parameter PORT_R_WIDTH = WIDTH;
parameter PORT_W_WIDTH = WIDTH;
parameter PORT_W_WR_BE_WIDTH = 16;
parameter PORT_R_CLK_POL = 1;
parameter PORT_W_CLK_POL = 1;

input PORT_R_CLK;
input PORT_R_RD_EN;
input [10:0] PORT_R_ADDR;
output [PORT_R_WIDTH-1:0] PORT_R_RD_DATA;

input PORT_W_CLK;
input PORT_W_WR_EN;
input [15:0] PORT_W_WR_BE;
input [10:0] PORT_W_ADDR;
input [PORT_W_WIDTH-1:0] PORT_W_WR_DATA;

wire [15:0] RDATA;
wire [15:0] WDATA;
wire [15:0] MASK;
// without byte enables every bit is written
wire [15:0] WR_BE = OPTION_HAS_BE ? PORT_W_WR_BE : 16'hffff;
// LSE wires a RAM whose ports have the same width straight through: the address
// goes to RADDR/WADDR as is and data bit b to the pin that mode uses for it
// (DPIN). The bit-reversed order of the generic mapping lets mixed widths share
// one INIT layout, which LSE does not do.
`ifdef LSE
localparam DIRECT = PORT_R_WIDTH == PORT_W_WIDTH;
`else
localparam DIRECT = 0;
`endif
localparam W = PORT_R_WIDTH;
localparam G = 16 / W;                  // bits of one 16-bit word that share a data pin
localparam SH = W == 16 ? 3 : W == 8 ? 2 : W == 4 ? 1 : 0;

// the pin carrying data bit b in read/write mode for width W (iCE40 SB_RAM40_4K)
function integer DPIN;
	input integer b;
	DPIN = W == 16 ? b : W == 8 ? 2*b : W == 4 ? 4*b + 1 : 8*b + 3;
endfunction

wire [10:0] RADDR = DIRECT ? PORT_R_ADDR >> SH : {PORT_R_ADDR[0], PORT_R_ADDR[1], PORT_R_ADDR[2], PORT_R_ADDR[10:3]};
wire [10:0] WADDR = DIRECT ? PORT_W_ADDR >> SH : {PORT_W_ADDR[0], PORT_W_ADDR[1], PORT_W_ADDR[2], PORT_W_ADDR[10:3]};

function [1:0] mode;
	input integer width;
	case (width)
	16: mode = 0;
	8: mode = 1;
	4: mode = 2;
	2: mode = 3;
	endcase
endfunction

function [255:0] slice_init;
	input [3:0] idx;
	integer i;
	reg [7:0] ri;
	reg [11:0] a;
	for (i = 0; i < 256; i = i + 1) begin
		ri = i;
		if (DIRECT) begin
			// physical bit: word {idx, ri[7:4]}, pin ri[3:0]; pin group g holds data
			// bit ri[3:0]/G of logical word (ri[3:0]%G)*256 + {idx, ri[7:4]}
			a = (((ri[3:0] % G) * 256 + {idx, ri[7:4]}) * W) + ri[3:0] / G;
		end else
			a = {idx, ri[7:4], ri[0], ri[1], ri[2], ri[3]};
		slice_init[i] = INIT[a];
	end
endfunction

`define INSTANCE(type, rclk, wclk) \
	type #( \
		.INIT_0(slice_init(0)), \
		.INIT_1(slice_init(1)), \
		.INIT_2(slice_init(2)), \
		.INIT_3(slice_init(3)), \
		.INIT_4(slice_init(4)), \
		.INIT_5(slice_init(5)), \
		.INIT_6(slice_init(6)), \
		.INIT_7(slice_init(7)), \
		.INIT_8(slice_init(8)), \
		.INIT_9(slice_init(9)), \
		.INIT_A(slice_init(10)), \
		.INIT_B(slice_init(11)), \
		.INIT_C(slice_init(12)), \
		.INIT_D(slice_init(13)), \
		.INIT_E(slice_init(14)), \
		.INIT_F(slice_init(15)), \
		.READ_MODE(mode(PORT_R_WIDTH)), \
		.WRITE_MODE(mode(PORT_W_WIDTH)) \
	) _TECHMAP_REPLACE_ ( \
		.RDATA(RDATA), \
		.rclk(PORT_R_CLK), \
		.RCLKE(PORT_R_RD_EN), \
		.RE(1'b1), \
		.RADDR(RADDR), \
		.WDATA(WDATA), \
		.wclk(PORT_W_CLK), \
		.WCLKE(PORT_W_WR_EN), \
		.WE(1'b1), \
		.WADDR(WADDR), \
		.MASK(MASK), \
	);

generate

if (DIRECT) begin
	genvar b;
	for (b = 0; b < W; b = b + 1) begin:dpin
		assign PORT_R_RD_DATA[b] = RDATA[DPIN(b)];
		assign WDATA[DPIN(b)] = PORT_W_WR_DATA[b];
		if (W == 16)
			assign MASK[b] = !WR_BE[b];
	end
end else begin
case(PORT_R_WIDTH)
	2: begin
		assign PORT_R_RD_DATA = {
			RDATA[11],
			RDATA[3]
		};
	end
	4: begin
		assign PORT_R_RD_DATA = {
			RDATA[13],
			RDATA[5],
			RDATA[9],
			RDATA[1]
		};
	end
	8: begin
		assign PORT_R_RD_DATA = {
			RDATA[14],
			RDATA[6],
			RDATA[10],
			RDATA[2],
			RDATA[12],
			RDATA[4],
			RDATA[8],
			RDATA[0]
		};
	end
	16: begin
			assign PORT_R_RD_DATA = {
				RDATA[15],
				RDATA[7],
				RDATA[11],
				RDATA[3],
				RDATA[13],
				RDATA[5],
				RDATA[9],
				RDATA[1],
				RDATA[14],
				RDATA[6],
				RDATA[10],
				RDATA[2],
				RDATA[12],
				RDATA[4],
				RDATA[8],
				RDATA[0]
			};
	end
endcase

case(PORT_W_WIDTH)
	2: begin
		assign {
			WDATA[11],
			WDATA[3]
		} = PORT_W_WR_DATA;
	end
	4: begin
		assign {
			WDATA[13],
			WDATA[5],
			WDATA[9],
			WDATA[1]
		} = PORT_W_WR_DATA;
	end
	8: begin
		assign {
			WDATA[14],
			WDATA[6],
			WDATA[10],
			WDATA[2],
			WDATA[12],
			WDATA[4],
			WDATA[8],
			WDATA[0]
		} = PORT_W_WR_DATA;
	end
	16: begin
		begin
			assign WDATA = {
				PORT_W_WR_DATA[15],
				PORT_W_WR_DATA[7],
				PORT_W_WR_DATA[11],
				PORT_W_WR_DATA[3],
				PORT_W_WR_DATA[13],
				PORT_W_WR_DATA[5],
				PORT_W_WR_DATA[9],
				PORT_W_WR_DATA[1],
				PORT_W_WR_DATA[14],
				PORT_W_WR_DATA[6],
				PORT_W_WR_DATA[10],
				PORT_W_WR_DATA[2],
				PORT_W_WR_DATA[12],
				PORT_W_WR_DATA[4],
				PORT_W_WR_DATA[8],
				PORT_W_WR_DATA[0]
			};
			assign MASK = ~{
				WR_BE[15],
				WR_BE[7],
				WR_BE[11],
				WR_BE[3],
				WR_BE[13],
				WR_BE[5],
				WR_BE[9],
				WR_BE[1],
				WR_BE[14],
				WR_BE[6],
				WR_BE[10],
				WR_BE[2],
				WR_BE[12],
				WR_BE[4],
				WR_BE[8],
				WR_BE[0]
			};
		end
	end
endcase

end

if (PORT_R_CLK_POL) begin
	if (PORT_W_CLK_POL) begin
		`INSTANCE(SB_RAM40_4K, RCLK, WCLK)
	end else begin
		`INSTANCE(SB_RAM40_4KNW, RCLK, WCLKN)
	end
end else begin
	if (PORT_W_CLK_POL) begin
		`INSTANCE(SB_RAM40_4KNR, RCLKN, WCLK)
	end else begin
		`INSTANCE(SB_RAM40_4KNRNW, RCLKN, WCLKN)
	end
end

endgenerate

endmodule
