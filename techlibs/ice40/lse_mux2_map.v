// a 2:1 multiplexer as its own LUT, as LSE leaves them
module \$_MUX_ (input A, B, S, output Y);
	SB_LUT4 #(.LUT_INIT(16'h00CA)) _TECHMAP_REPLACE_ (.I0(A), .I1(B), .I2(S), .I3(1'b0), .O(Y));
endmodule
