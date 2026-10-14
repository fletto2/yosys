// Port declarations of the Radiant iCE40UP primitives that ice40_radiant turns into
// SB_* cells (read as black boxes by synth_ice40).

(* blackbox *)
module HSOSC(CLKHFPU, CLKHFEN, CLKHF);
	parameter CLKHF_DIV = "0b00";
	input CLKHFPU;
	input CLKHFEN;
	output CLKHF;
endmodule

(* blackbox *)
module HSOSC1P8V(CLKHFPU, CLKHFEN, CLKHF);
	parameter CLKHF_DIV = "0b00";
	input CLKHFPU;
	input CLKHFEN;
	output CLKHF;
endmodule

(* blackbox *)
module LSOSC(CLKLFPU, CLKLFEN, CLKLF);
	input CLKLFPU;
	input CLKLFEN;
	output CLKLF;
endmodule

(* blackbox *)
module LSOSC1P8V(CLKLFPU, CLKLFEN, CLKLF);
	input CLKLFPU;
	input CLKLFEN;
	output CLKLF;
endmodule

(* blackbox *)
module RGB(CURREN, RGBLEDEN, RGB0PWM, RGB1PWM, RGB2PWM, RGB2, RGB1, RGB0);
	parameter CURRENT_MODE = "0";
	parameter RGB0_CURRENT = "0b000000";
	parameter RGB1_CURRENT = "0b000000";
	parameter RGB2_CURRENT = "0b000000";
	input CURREN;
	input RGBLEDEN;
	input RGB0PWM;
	input RGB1PWM;
	input RGB2PWM;
	output RGB2;
	output RGB1;
	output RGB0;
endmodule

(* blackbox *)
module WARMBOOT(S1, S0, BOOT);
	input S1;
	input S0;
	input BOOT;
endmodule

(* blackbox *)
module FILTER(FILTERIN, FILTEROUT);
	input FILTERIN;
	output FILTEROUT;
endmodule

(* blackbox *)
module SP256K(AD, DI, MASKWE, WE, CS, CK, STDBY, SLEEP, PWROFF_N, DO);
	input [13:0] AD;
	input [15:0] DI;
	input [3:0] MASKWE;
	input WE;
	input CS;
	input CK;
	input STDBY;
	input SLEEP;
	input PWROFF_N;
	output [15:0] DO;
endmodule

(* blackbox *)
module PLL_B(REFERENCECLK, FEEDBACK, DYNAMICDELAY7, DYNAMICDELAY6, DYNAMICDELAY5, DYNAMICDELAY4, DYNAMICDELAY3, DYNAMICDELAY2, DYNAMICDELAY1, DYNAMICDELAY0, BYPASS, RESET_N, SCLK, SDI, LATCH, INTFBOUT, OUTCORE, OUTGLOBAL, OUTCOREB, OUTGLOBALB, SDO, LOCK);
	parameter FEEDBACK_PATH = "SIMPLE";
	parameter DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED";
	parameter FDA_FEEDBACK = "0";
	parameter DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED";
	parameter FDA_RELATIVE = "0";
	parameter SHIFTREG_DIV_MODE = "0";
	parameter PLLOUT_SELECT_PORTA = "SHIFTREG_0deg";
	parameter PLLOUT_SELECT_PORTB = "SHIFTREG_0deg";
	parameter DIVR = "0";
	parameter DIVF = "0";
	parameter DIVQ = "1";
	parameter FILTER_RANGE = "0";
	parameter EXTERNAL_DIVIDE_FACTOR = "NONE";
	parameter ENABLE_ICEGATE_PORTA = "0";
	parameter ENABLE_ICEGATE_PORTB = "0";
	parameter TEST_MODE = "0";
	parameter FREQUENCY_PIN_REFERENCECLK = "NONE";
	input REFERENCECLK;
	input FEEDBACK;
	input DYNAMICDELAY7;
	input DYNAMICDELAY6;
	input DYNAMICDELAY5;
	input DYNAMICDELAY4;
	input DYNAMICDELAY3;
	input DYNAMICDELAY2;
	input DYNAMICDELAY1;
	input DYNAMICDELAY0;
	input BYPASS;
	input RESET_N;
	input SCLK;
	input SDI;
	input LATCH;
	output INTFBOUT;
	output OUTCORE;
	output OUTGLOBAL;
	output OUTCOREB;
	output OUTGLOBALB;
	output SDO;
	output LOCK;
endmodule

(* blackbox *)
module MAC16(CLK, CE, C15, C14, C13, C12, C11, C10, C9, C8, C7, C6, C5, C4, C3, C2, C1, C0, A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0, B15, B14, B13, B12, B11, B10, B9, B8, B7, B6, B5, B4, B3, B2, B1, B0, D15, D14, D13, D12, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, AHOLD, BHOLD, CHOLD, DHOLD, IRSTTOP, IRSTBOT, ORSTTOP, ORSTBOT, OLOADTOP, OLOADBOT, ADDSUBTOP, ADDSUBBOT, OHOLDTOP, OHOLDBOT, CI, ACCUMCI, SIGNEXTIN, O31, O30, O29, O28, O27, O26, O25, O24, O23, O22, O21, O20, O19, O18, O17, O16, O15, O14, O13, O12, O11, O10, O9, O8, O7, O6, O5, O4, O3, O2, O1, O0, CO, ACCUMCO, SIGNEXTOUT);
	parameter NEG_TRIGGER = "0b0";
	parameter A_REG = "0b0";
	parameter B_REG = "0b0";
	parameter C_REG = "0b0";
	parameter D_REG = "0b0";
	parameter TOP_8x8_MULT_REG = "0b0";
	parameter BOT_8x8_MULT_REG = "0b0";
	parameter PIPELINE_16x16_MULT_REG1 = "0b0";
	parameter PIPELINE_16x16_MULT_REG2 = "0b0";
	parameter TOPADDSUB_UPPERINPUT = "0b0";
	parameter BOTADDSUB_UPPERINPUT = "0b0";
	parameter MODE_8x8 = "0b0";
	parameter A_SIGNED = "0b0";
	parameter B_SIGNED = "0b0";
	parameter TOPOUTPUT_SELECT = "0b00";
	parameter TOPADDSUB_LOWERINPUT = "0b00";
	parameter TOPADDSUB_CARRYSELECT = "0b00";
	parameter BOTOUTPUT_SELECT = "0b00";
	parameter BOTADDSUB_LOWERINPUT = "0b00";
	parameter BOTADDSUB_CARRYSELECT = "0b00";
	input CLK;
	input CE;
	input C15;
	input C14;
	input C13;
	input C12;
	input C11;
	input C10;
	input C9;
	input C8;
	input C7;
	input C6;
	input C5;
	input C4;
	input C3;
	input C2;
	input C1;
	input C0;
	input A15;
	input A14;
	input A13;
	input A12;
	input A11;
	input A10;
	input A9;
	input A8;
	input A7;
	input A6;
	input A5;
	input A4;
	input A3;
	input A2;
	input A1;
	input A0;
	input B15;
	input B14;
	input B13;
	input B12;
	input B11;
	input B10;
	input B9;
	input B8;
	input B7;
	input B6;
	input B5;
	input B4;
	input B3;
	input B2;
	input B1;
	input B0;
	input D15;
	input D14;
	input D13;
	input D12;
	input D11;
	input D10;
	input D9;
	input D8;
	input D7;
	input D6;
	input D5;
	input D4;
	input D3;
	input D2;
	input D1;
	input D0;
	input AHOLD;
	input BHOLD;
	input CHOLD;
	input DHOLD;
	input IRSTTOP;
	input IRSTBOT;
	input ORSTTOP;
	input ORSTBOT;
	input OLOADTOP;
	input OLOADBOT;
	input ADDSUBTOP;
	input ADDSUBBOT;
	input OHOLDTOP;
	input OHOLDBOT;
	input CI;
	input ACCUMCI;
	input SIGNEXTIN;
	output O31;
	output O30;
	output O29;
	output O28;
	output O27;
	output O26;
	output O25;
	output O24;
	output O23;
	output O22;
	output O21;
	output O20;
	output O19;
	output O18;
	output O17;
	output O16;
	output O15;
	output O14;
	output O13;
	output O12;
	output O11;
	output O10;
	output O9;
	output O8;
	output O7;
	output O6;
	output O5;
	output O4;
	output O3;
	output O2;
	output O1;
	output O0;
	output CO;
	output ACCUMCO;
	output SIGNEXTOUT;
endmodule

(* blackbox *)
module I2C_B(SBCLKI, SBRWI, SBSTBI, SBADRI7, SBADRI6, SBADRI5, SBADRI4, SBADRI3, SBADRI2, SBADRI1, SBADRI0, SBDATI7, SBDATI6, SBDATI5, SBDATI4, SBDATI3, SBDATI2, SBDATI1, SBDATI0, SCLI, SDAI, SBDATO7, SBDATO6, SBDATO5, SBDATO4, SBDATO3, SBDATO2, SBDATO1, SBDATO0, SBACKO, I2CIRQ, I2CWKUP, SCLO, SCLOE, SDAO, SDAOE);
	parameter I2C_SLAVE_INIT_ADDR = "0b1111100001";
	parameter BUS_ADDR74 = "0b0001";
	parameter I2C_CLK_DIVIDER = "0";
	parameter SDA_INPUT_DELAYED = "0";
	parameter SDA_OUTPUT_DELAYED = "0";
	parameter FREQUENCY_PIN_SBCLKI = "NONE";
	input SBCLKI;
	input SBRWI;
	input SBSTBI;
	input SBADRI7;
	input SBADRI6;
	input SBADRI5;
	input SBADRI4;
	input SBADRI3;
	input SBADRI2;
	input SBADRI1;
	input SBADRI0;
	input SBDATI7;
	input SBDATI6;
	input SBDATI5;
	input SBDATI4;
	input SBDATI3;
	input SBDATI2;
	input SBDATI1;
	input SBDATI0;
	input SCLI;
	input SDAI;
	output SBDATO7;
	output SBDATO6;
	output SBDATO5;
	output SBDATO4;
	output SBDATO3;
	output SBDATO2;
	output SBDATO1;
	output SBDATO0;
	output SBACKO;
	output I2CIRQ;
	output I2CWKUP;
	output SCLO;
	output SCLOE;
	output SDAO;
	output SDAOE;
endmodule

(* blackbox *)
module SPI_B(SBCLKI, SBRWI, SBSTBI, SBADRI7, SBADRI6, SBADRI5, SBADRI4, SBADRI3, SBADRI2, SBADRI1, SBADRI0, SBDATI7, SBDATI6, SBDATI5, SBDATI4, SBDATI3, SBDATI2, SBDATI1, SBDATI0, MI, SI, SCKI, SCSNI, SBDATO7, SBDATO6, SBDATO5, SBDATO4, SBDATO3, SBDATO2, SBDATO1, SBDATO0, SBACKO, SPIIRQ, SPIWKUP, SO, SOE, MO, MOE, SCKO, SCKOE, MCSNO3, MCSNO2, MCSNO1, MCSNO0, MCSNOE3, MCSNOE2, MCSNOE1, MCSNOE0);
	parameter BUS_ADDR74 = "0b0000";
	parameter SPI_CLK_DIVIDER = "0";
	parameter FREQUENCY_PIN_SBCLKI = "NONE";
	input SBCLKI;
	input SBRWI;
	input SBSTBI;
	input SBADRI7;
	input SBADRI6;
	input SBADRI5;
	input SBADRI4;
	input SBADRI3;
	input SBADRI2;
	input SBADRI1;
	input SBADRI0;
	input SBDATI7;
	input SBDATI6;
	input SBDATI5;
	input SBDATI4;
	input SBDATI3;
	input SBDATI2;
	input SBDATI1;
	input SBDATI0;
	input MI;
	input SI;
	input SCKI;
	input SCSNI;
	output SBDATO7;
	output SBDATO6;
	output SBDATO5;
	output SBDATO4;
	output SBDATO3;
	output SBDATO2;
	output SBDATO1;
	output SBDATO0;
	output SBACKO;
	output SPIIRQ;
	output SPIWKUP;
	output SO;
	output SOE;
	output MO;
	output MOE;
	output SCKO;
	output SCKOE;
	output MCSNO3;
	output MCSNO2;
	output MCSNO1;
	output MCSNO0;
	output MCSNOE3;
	output MCSNOE2;
	output MCSNOE1;
	output MCSNOE0;
endmodule

(* blackbox *)
module IB(I, O);
	input I;
	output O;
endmodule

(* blackbox *)
module OB(I, O);
	input I;
	output O;
endmodule

(* blackbox *)
module OBZ_B(I, T_N, O);
	input I;
	input T_N;
	output O;
endmodule

(* blackbox *)
module BB_B(T_N, I, O, B);
	input T_N;
	input I;
	output O;
	inout B;
endmodule
(* blackbox *)
module HSOSC_CORE(CLKHFPU, CLKHFEN, TRIM9, TRIM8, TRIM7, TRIM6, TRIM5, TRIM4, TRIM3, TRIM2, TRIM1, TRIM0, CLKHF);
	parameter CLKHF_DIV = "0b00";
	parameter FABRIC_TRIME = "DISABLE";
	input CLKHFPU;
	input CLKHFEN;
	input TRIM9;
	input TRIM8;
	input TRIM7;
	input TRIM6;
	input TRIM5;
	input TRIM4;
	input TRIM3;
	input TRIM2;
	input TRIM1;
	input TRIM0;
	output CLKHF;
endmodule

(* blackbox *)
module LSOSC_CORE(CLKLFPU, CLKLFEN, TRIM9, TRIM8, TRIM7, TRIM6, TRIM5, TRIM4, TRIM3, TRIM2, TRIM1, TRIM0, CLKLF);
	parameter FABRIC_TRIME = "DISABLE";
	input CLKLFPU;
	input CLKLFEN;
	input TRIM9;
	input TRIM8;
	input TRIM7;
	input TRIM6;
	input TRIM5;
	input TRIM4;
	input TRIM3;
	input TRIM2;
	input TRIM1;
	input TRIM0;
	output CLKLF;
endmodule

(* blackbox *)
module RGB_CORE(CURREN, RGBLEDEN, RGB0PWM, RGB1PWM, RGB2PWM, TRIM9, TRIM8, TRIM7, TRIM6, TRIM5, TRIM4, TRIM3, TRIM2, TRIM1, TRIM0, RGB2, RGB1, RGB0);
	parameter CURRENT_MODE = "0";
	parameter RGB0_CURRENT = "0b000000";
	parameter RGB1_CURRENT = "0b000000";
	parameter RGB2_CURRENT = "0b000000";
	parameter FABRIC_TRIME = "DISABLE";
	input CURREN;
	input RGBLEDEN;
	input RGB0PWM;
	input RGB1PWM;
	input RGB2PWM;
	input TRIM9;
	input TRIM8;
	input TRIM7;
	input TRIM6;
	input TRIM5;
	input TRIM4;
	input TRIM3;
	input TRIM2;
	input TRIM1;
	input TRIM0;
	output RGB2;
	output RGB1;
	output RGB0;
endmodule

(* blackbox *)
module RGBPWM(LEDDCS, LEDDCLK, LEDDDAT7, LEDDDAT6, LEDDDAT5, LEDDDAT4, LEDDDAT3, LEDDDAT2, LEDDDAT1, LEDDDAT0, LEDDADDR3, LEDDADDR2, LEDDADDR1, LEDDADDR0, LEDDDEN, LEDDEXE, PWMOUT2, PWMOUT1, PWMOUT0, LEDDON);
	input LEDDCS;
	input LEDDCLK;
	input LEDDDAT7;
	input LEDDDAT6;
	input LEDDDAT5;
	input LEDDDAT4;
	input LEDDDAT3;
	input LEDDDAT2;
	input LEDDDAT1;
	input LEDDDAT0;
	input LEDDADDR3;
	input LEDDADDR2;
	input LEDDADDR1;
	input LEDDADDR0;
	input LEDDDEN;
	input LEDDEXE;
	output PWMOUT2;
	output PWMOUT1;
	output PWMOUT0;
	output LEDDON;
endmodule

(* blackbox *)
module OB_RGB(T_N, I, O, B);
	input T_N;
	input I;
	output O;
	inout B;
endmodule

(* blackbox *)
module BB_OD(T_N, I, O, B);
	input T_N;
	input I;
	output O;
	inout B;
endmodule

(* blackbox *)
module BB_I3C(T_N, I, O, B, PU_ENB, WEAK_PU_ENB);
	input T_N;
	input I;
	output O;
	inout B;
	input PU_ENB;
	input WEAK_PU_ENB;
endmodule

(* blackbox *)
module IOL_B(PADDI, DO1, DO0, CE, IOLTO, HOLD, INCLK, OUTCLK, PADDO, PADDT, DI1, DI0);
	parameter LATCHIN = "NONE_REG";
	parameter DDROUT = "NO";
	input PADDI;
	input DO1;
	input DO0;
	input CE;
	input IOLTO;
	input HOLD;
	input INCLK;
	input OUTCLK;
	output PADDO;
	output PADDT;
	output DI1;
	output DI0;
endmodule

(* blackbox *)
module PUR(PUR_N);
	parameter RST_PULSE = "1";
	input PUR_N;
endmodule

(* blackbox *)
module FD1P3XZ(D, SP, CK, SR, Q);
	parameter REGSET = "RESET";
	parameter SRMODE = "CE_OVER_LSR";
	input D;
	input SP;
	input CK;
	input SR;
	output Q;
endmodule

(* blackbox *)
module FD1P3BZ(D, CK, SP, PD, Q);
	input D;
	input CK;
	input SP;
	input PD;
	output Q;
endmodule

(* blackbox *)
module FD1P3DZ(D, CK, SP, CD, Q);
	input D;
	input CK;
	input SP;
	input CD;
	output Q;
endmodule

(* blackbox *)
module FD1P3IZ(D, CK, SP, CD, Q);
	input D;
	input CK;
	input SP;
	input CD;
	output Q;
endmodule

(* blackbox *)
module FD1P3JZ(D, CK, SP, PD, Q);
	input D;
	input CK;
	input SP;
	input PD;
	output Q;
endmodule

(* blackbox *)
module IFD1P3AZ(D, SP, CK, Q);
	input D;
	input SP;
	input CK;
	output Q;
endmodule

(* blackbox *)
module OFD1P3AZ(D, SP, CK, Q);
	input D;
	input SP;
	input CK;
	output Q;
endmodule

(* blackbox *)
module LUT4(A, B, C, D, Z);
	parameter INIT = "0x0000";
	input A;
	input B;
	input C;
	input D;
	output Z;
endmodule

(* blackbox *)
module FA2(A0, B0, C0, D0, CI0, A1, B1, C1, D1, CI1, CO0, CO1, S0, S1);
	parameter INIT0 = "0xc33c";
	parameter INIT1 = "0xc33c";
	input A0;
	input B0;
	input C0;
	input D0;
	input CI0;
	input A1;
	input B1;
	input C1;
	input D1;
	input CI1;
	output CO0;
	output CO1;
	output S0;
	output S1;
endmodule

(* blackbox *)
module CCU2_B(A0, B0, C0, CIN, A1, B1, C1, S0, S1, COUT);
	parameter INIT0 = "0xc33c";
	parameter INIT1 = "0xc33c";
	input A0;
	input B0;
	input C0;
	input CIN;
	input A1;
	input B1;
	input C1;
	output S0;
	output S1;
	output COUT;
endmodule

(* blackbox *)
module VHI(Z);
	output Z;
endmodule

(* blackbox *)
module VLO(Z);
	output Z;
endmodule

(* blackbox *)
module INV(A, Z);
	input A;
	output Z;
endmodule

(* blackbox *)
module BUF(A, Z);
	parameter BUF_DELAY = "0";
	input A;
	output Z;
endmodule

(* blackbox *)
module EBR_B(RADDR10, RADDR9, RADDR8, RADDR7, RADDR6, RADDR5, RADDR4, RADDR3, RADDR2, RADDR1, RADDR0, WADDR10, WADDR9, WADDR8, WADDR7, WADDR6, WADDR5, WADDR4, WADDR3, WADDR2, WADDR1, WADDR0, MASK_N15, MASK_N14, MASK_N13, MASK_N12, MASK_N11, MASK_N10, MASK_N9, MASK_N8, MASK_N7, MASK_N6, MASK_N5, MASK_N4, MASK_N3, MASK_N2, MASK_N1, MASK_N0, WDATA15, WDATA14, WDATA13, WDATA12, WDATA11, WDATA10, WDATA9, WDATA8, WDATA7, WDATA6, WDATA5, WDATA4, WDATA3, WDATA2, WDATA1, WDATA0, RCLKE, RCLK, RE, WCLKE, WCLK, WE, RDATA15, RDATA14, RDATA13, RDATA12, RDATA11, RDATA10, RDATA9, RDATA8, RDATA7, RDATA6, RDATA5, RDATA4, RDATA3, RDATA2, RDATA1, RDATA0);
	parameter INIT_0 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_1 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_2 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_3 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_4 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_5 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_6 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_7 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_8 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_9 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_A = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_B = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_C = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_D = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_E = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INIT_F = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter DATA_WIDTH_W = "2";
	parameter DATA_WIDTH_R = "2";
	input RADDR10;
	input RADDR9;
	input RADDR8;
	input RADDR7;
	input RADDR6;
	input RADDR5;
	input RADDR4;
	input RADDR3;
	input RADDR2;
	input RADDR1;
	input RADDR0;
	input WADDR10;
	input WADDR9;
	input WADDR8;
	input WADDR7;
	input WADDR6;
	input WADDR5;
	input WADDR4;
	input WADDR3;
	input WADDR2;
	input WADDR1;
	input WADDR0;
	input MASK_N15;
	input MASK_N14;
	input MASK_N13;
	input MASK_N12;
	input MASK_N11;
	input MASK_N10;
	input MASK_N9;
	input MASK_N8;
	input MASK_N7;
	input MASK_N6;
	input MASK_N5;
	input MASK_N4;
	input MASK_N3;
	input MASK_N2;
	input MASK_N1;
	input MASK_N0;
	input WDATA15;
	input WDATA14;
	input WDATA13;
	input WDATA12;
	input WDATA11;
	input WDATA10;
	input WDATA9;
	input WDATA8;
	input WDATA7;
	input WDATA6;
	input WDATA5;
	input WDATA4;
	input WDATA3;
	input WDATA2;
	input WDATA1;
	input WDATA0;
	input RCLKE;
	input RCLK;
	input RE;
	input WCLKE;
	input WCLK;
	input WE;
	output RDATA15;
	output RDATA14;
	output RDATA13;
	output RDATA12;
	output RDATA11;
	output RDATA10;
	output RDATA9;
	output RDATA8;
	output RDATA7;
	output RDATA6;
	output RDATA5;
	output RDATA4;
	output RDATA3;
	output RDATA2;
	output RDATA1;
	output RDATA0;
endmodule

(* blackbox *)
module PDP4K(DI, ADW, ADR, CKW, CKR, CEW, CER, RE, WE, MASK_N, DO);
	parameter INITVAL_0 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_1 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_2 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_3 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_4 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_5 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_6 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_7 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_8 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_9 = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_A = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_B = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_C = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_D = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_E = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter INITVAL_F = "0x0000000000000000000000000000000000000000000000000000000000000000";
	parameter DATA_WIDTH_W = "2";
	parameter DATA_WIDTH_R = "2";
	input [15:0] DI;
	input [10:0] ADW;
	input [10:0] ADR;
	input CKW;
	input CKR;
	input CEW;
	input CER;
	input RE;
	input WE;
	input [15:0] MASK_N;
	output [15:0] DO;
endmodule

(* blackbox *)
module VFB_B(ADDRESS13, ADDRESS12, ADDRESS11, ADDRESS10, ADDRESS9, ADDRESS8, ADDRESS7, ADDRESS6, ADDRESS5, ADDRESS4, ADDRESS3, ADDRESS2, ADDRESS1, ADDRESS0, DATAIN15, DATAIN14, DATAIN13, DATAIN12, DATAIN11, DATAIN10, DATAIN9, DATAIN8, DATAIN7, DATAIN6, DATAIN5, DATAIN4, DATAIN3, DATAIN2, DATAIN1, DATAIN0, MASKWREN3, MASKWREN2, MASKWREN1, MASKWREN0, WREN, CHIPSELECT, CLOCK, RDMARGINEN, RDMARGIN3, RDMARGIN2, RDMARGIN1, RDMARGIN0, STANDBY, SLEEP, POWEROFF_N, TEST, DATAOUT15, DATAOUT14, DATAOUT13, DATAOUT12, DATAOUT11, DATAOUT10, DATAOUT9, DATAOUT8, DATAOUT7, DATAOUT6, DATAOUT5, DATAOUT4, DATAOUT3, DATAOUT2, DATAOUT1, DATAOUT0);
	input ADDRESS13;
	input ADDRESS12;
	input ADDRESS11;
	input ADDRESS10;
	input ADDRESS9;
	input ADDRESS8;
	input ADDRESS7;
	input ADDRESS6;
	input ADDRESS5;
	input ADDRESS4;
	input ADDRESS3;
	input ADDRESS2;
	input ADDRESS1;
	input ADDRESS0;
	input DATAIN15;
	input DATAIN14;
	input DATAIN13;
	input DATAIN12;
	input DATAIN11;
	input DATAIN10;
	input DATAIN9;
	input DATAIN8;
	input DATAIN7;
	input DATAIN6;
	input DATAIN5;
	input DATAIN4;
	input DATAIN3;
	input DATAIN2;
	input DATAIN1;
	input DATAIN0;
	input MASKWREN3;
	input MASKWREN2;
	input MASKWREN1;
	input MASKWREN0;
	input WREN;
	input CHIPSELECT;
	input CLOCK;
	input RDMARGINEN;
	input RDMARGIN3;
	input RDMARGIN2;
	input RDMARGIN1;
	input RDMARGIN0;
	input STANDBY;
	input SLEEP;
	input POWEROFF_N;
	input TEST;
	output DATAOUT15;
	output DATAOUT14;
	output DATAOUT13;
	output DATAOUT12;
	output DATAOUT11;
	output DATAOUT10;
	output DATAOUT9;
	output DATAOUT8;
	output DATAOUT7;
	output DATAOUT6;
	output DATAOUT5;
	output DATAOUT4;
	output DATAOUT3;
	output DATAOUT2;
	output DATAOUT1;
	output DATAOUT0;
endmodule
