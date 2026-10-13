// LSE's 4:1 multiplexer in two LUT4s (synth_ice40 -lse, after muxcover -mux4):
//   L1 = T ? (S ? D : C) : S        L2 = T ? L1 : (L1 ? B : A)
// With T low, L1 passes S through and L2 picks A or B.
module \$_MUX4_ (input A, B, C, D, S, T, output Y);
	wire l1;
	(* lse_fixed *) SB_LUT4 #(.LUT_INIT(16'hE4AA)) lut1 (.I0(S), .I1(C), .I2(D), .I3(T), .O(l1));
	(* lse_fixed *) SB_LUT4 #(.LUT_INIT(16'hAAD8)) lut2 (.I0(l1), .I1(B), .I2(A), .I3(T), .O(Y));
endmodule
