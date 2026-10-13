// Explicit SB_IO pads for ice40_iopack -pads, with LSE's pin types: a plain output
// pad sets its unused input half to 2'b00 (a registered output pad, made by
// ice40_iopack itself, uses 2'b10). IOPACK_PULLUP enables the 100k pull-up on pads
// that are read; LSE leaves it off on output-only pads.
`ifndef IOPACK_PULLUP
`define IOPACK_PULLUP 0
`endif

module \$__ICE40_IOPACK_IN (output Y, inout P);
	SB_IO #(.PIN_TYPE(6'b000001), .PULLUP(`IOPACK_PULLUP)) _TECHMAP_REPLACE_ (
		.PACKAGE_PIN(P), .D_IN_0(Y));
endmodule

module \$__ICE40_IOPACK_OUT (input A, inout P);
	SB_IO #(.PIN_TYPE(6'b011000), .PULLUP(1'b0)) _TECHMAP_REPLACE_ (
		.PACKAGE_PIN(P), .D_OUT_0(A));
endmodule

module \$__ICE40_IOPACK_TOUT (input E, input A, inout P);
	SB_IO #(.PIN_TYPE(6'b101000), .PULLUP(1'b0)) _TECHMAP_REPLACE_ (
		.PACKAGE_PIN(P), .OUTPUT_ENABLE(E), .D_OUT_0(A));
endmodule

module \$__ICE40_IOPACK_TINOUT (input E, output Y, input A, inout P);
	SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(`IOPACK_PULLUP)) _TECHMAP_REPLACE_ (
		.PACKAGE_PIN(P), .OUTPUT_ENABLE(E), .D_OUT_0(A), .D_IN_0(Y));
endmodule
