/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "kernel/yosys.h"
#include "kernel/sigtools.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct Ice40RadiantPass : public Pass {
	Ice40RadiantPass() : Pass("ice40_radiant", "turn Radiant iCE40UP primitives into SB_* cells") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_radiant [selection]\n");
		log("\n");
		log("Designs written for Lattice Radiant instantiate its iCE40UP primitives. This pass\n");
		log("replaces them by the matching SB_* cells, so such designs (and Radiant's own\n");
		log("netlists) go through this flow:\n");
		log("  HSOSC, LSOSC, HSOSC_CORE, LSOSC_CORE (and 1P8V), RGB, RGB_CORE, RGBPWM,\n");
		log("  WARMBOOT, FILTER, PLL_B, I2C_B, SPI_B, MAC16, SP256K, VFB_B, EBR_B, PDP4K,\n");
		log("  LUT4, FA2, CCU2_B, FD1P3XZ/BZ/DZ/IZ/JZ, IFD1P3AZ, OFD1P3AZ, INV, BUF,\n");
		log("  VHI, VLO, BB_I3C\n");
		log("The pad buffers IB, OB, OBZ_B, BB_B, BB_OD and OB_RGB become plain or tristate\n");
		log("connections for iopadmap, IOL_B becomes flip-flops that ice40_iopack packs\n");
		log("into the SB_IO (DDR output is not supported), PUR is dropped. The port\n");
		log("declarations are in +/ice40/radiant_cells.v.\n");
		log("\n");
	}

	// Radiant writes most parameters as strings: "0b0101", "0x1f", "12"
	static Const value(const Const &p, int width)
	{
		if (!(p.flags & RTLIL::CONST_FLAG_STRING))
			return Const(p.as_int(), width);
		std::string s = p.decode_string();
		long v = 0;
		if (s.size() > 2 && s[0] == '0' && (s[1] == 'b' || s[1] == 'B'))
			v = strtol(s.c_str() + 2, nullptr, 2);
		else if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
			v = strtol(s.c_str() + 2, nullptr, 16);
		else
			v = strtol(s.c_str(), nullptr, 10);
		return Const(v, width);
	}
	static std::string bin_string(const Const &p, int width)
	{
		Const v = value(p, width);
		std::string s = "0b";
		for (int i = width - 1; i >= 0; i--)
			s += v[i] == State::S1 ? '1' : '0';
		return s;
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_RADIANT pass.\n");
		for (auto module : design->selected_modules()) {
			int count = 0;
			std::vector<Cell*> cells;
			for (auto cell : module->selected_cells())
				cells.push_back(cell);
			for (auto cell : cells) {
				IdString t = cell->type;
				auto port = [&](const char *name) {
					IdString id = RTLIL::escape_id(name);
					return cell->hasPort(id) ? cell->getPort(id) : SigSpec();
				};
				// a port that Radiant splits into scalar bits (A15 .. A0)
				auto bus = [&](const char *name, int width) {
					SigSpec s;
					for (int i = 0; i < width; i++) {
						SigSpec b = port(stringf("%s%d", name, i).c_str());
						s.append(b.empty() ? SigSpec(State::S0) : b);
					}
					return s;
				};
				auto param = [&](const char *name, const char *dflt) {
					IdString id = RTLIL::escape_id(name);
					return cell->hasParam(id) ? cell->getParam(id) : Const(std::string(dflt));
				};
				Cell *n = nullptr;
				auto make = [&](const char *type) {
					n = module->addCell(NEW_ID, RTLIL::escape_id(type));
					n->attributes = cell->attributes;
				};
				auto conn = [&](const char *to, SigSpec sig) {
					if (!sig.empty())
						n->setPort(RTLIL::escape_id(to), sig);
				};
				if (t.in(ID(HSOSC), ID(HSOSC1P8V))) {
					make("SB_HFOSC");
					conn("CLKHFPU", port("CLKHFPU")); conn("CLKHFEN", port("CLKHFEN")); conn("CLKHF", port("CLKHF"));
					n->setParam(ID(CLKHF_DIV), Const(bin_string(param("CLKHF_DIV", "0b00"), 2)));
				} else if (t.in(ID(LSOSC), ID(LSOSC1P8V))) {
					make("SB_LFOSC");
					conn("CLKLFPU", port("CLKLFPU")); conn("CLKLFEN", port("CLKLFEN")); conn("CLKLF", port("CLKLF"));
				} else if (t.in(ID(RGB), ID(RGB1P8V))) {
					make("SB_RGBA_DRV");
					for (auto p : {"CURREN", "RGBLEDEN", "RGB0PWM", "RGB1PWM", "RGB2PWM", "RGB0", "RGB1", "RGB2"})
						conn(p, port(p));
					n->setParam(ID(CURRENT_MODE), Const(bin_string(param("CURRENT_MODE", "0"), 1)));
					for (auto p : {"RGB0_CURRENT", "RGB1_CURRENT", "RGB2_CURRENT"})
						n->setParam(RTLIL::escape_id(p), Const(bin_string(param(p, "0b000000"), 6)));
				} else if (t == ID(WARMBOOT)) {
					make("SB_WARMBOOT");
					for (auto p : {"S1", "S0", "BOOT"})
						conn(p, port(p));
				} else if (t == ID(FILTER)) {
					make("SB_FILTER_50NS");
					conn("FILTERIN", port("FILTERIN")); conn("FILTEROUT", port("FILTEROUT"));
				} else if (t == ID(SP256K)) {
					make("SB_SPRAM256KA");
					conn("ADDRESS", port("AD")); conn("DATAIN", port("DI")); conn("DATAOUT", port("DO"));
					conn("MASKWREN", port("MASKWE")); conn("WREN", port("WE")); conn("CHIPSELECT", port("CS"));
					conn("CLOCK", port("CK")); conn("STANDBY", port("STDBY")); conn("SLEEP", port("SLEEP"));
					conn("POWEROFF", port("PWROFF_N"));
				} else if (t == ID(PLL_B)) {
					bool two = !port("OUTCOREB").empty() || !port("OUTGLOBALB").empty();
					make(two ? "SB_PLL40_2F_CORE" : "SB_PLL40_CORE");
					conn("REFERENCECLK", port("REFERENCECLK")); conn("EXTFEEDBACK", port("FEEDBACK"));
					conn("DYNAMICDELAY", bus("DYNAMICDELAY", 8)); conn("BYPASS", port("BYPASS"));
					conn("RESETB", port("RESET_N")); conn("SCLK", port("SCLK")); conn("SDI", port("SDI"));
					conn("LATCHINPUTVALUE", port("LATCH")); conn("SDO", port("SDO")); conn("LOCK", port("LOCK"));
					if (two) {
						conn("PLLOUTCOREA", port("OUTCORE")); conn("PLLOUTGLOBALA", port("OUTGLOBAL"));
						conn("PLLOUTCOREB", port("OUTCOREB")); conn("PLLOUTGLOBALB", port("OUTGLOBALB"));
						n->setParam(ID(PLLOUT_SELECT_PORTA), param("PLLOUT_SELECT_PORTA", "SHIFTREG_0deg"));
						n->setParam(ID(PLLOUT_SELECT_PORTB), param("PLLOUT_SELECT_PORTB", "SHIFTREG_0deg"));
						n->setParam(ID(ENABLE_ICEGATE_PORTA), value(param("ENABLE_ICEGATE_PORTA", "0"), 1));
						n->setParam(ID(ENABLE_ICEGATE_PORTB), value(param("ENABLE_ICEGATE_PORTB", "0"), 1));
					} else {
						conn("PLLOUTCORE", port("OUTCORE")); conn("PLLOUTGLOBAL", port("OUTGLOBAL"));
						n->setParam(ID(PLLOUT_SELECT), param("PLLOUT_SELECT_PORTA", "SHIFTREG_0deg"));
						n->setParam(ID(ENABLE_ICEGATE), value(param("ENABLE_ICEGATE_PORTA", "0"), 1));
					}
					for (auto p : {"FEEDBACK_PATH", "DELAY_ADJUSTMENT_MODE_FEEDBACK", "DELAY_ADJUSTMENT_MODE_RELATIVE"})
						n->setParam(RTLIL::escape_id(p), param(p, p[0] == 'F' ? "SIMPLE" : "FIXED"));
					n->setParam(ID(FDA_FEEDBACK), value(param("FDA_FEEDBACK", "0"), 4));
					n->setParam(ID(FDA_RELATIVE), value(param("FDA_RELATIVE", "0"), 4));
					n->setParam(ID(SHIFTREG_DIV_MODE), value(param("SHIFTREG_DIV_MODE", "0"), 2));
					n->setParam(ID(DIVR), value(param("DIVR", "0"), 4));
					n->setParam(ID(DIVF), value(param("DIVF", "0"), 7));
					n->setParam(ID(DIVQ), value(param("DIVQ", "1"), 3));
					n->setParam(ID(FILTER_RANGE), value(param("FILTER_RANGE", "0"), 3));
					n->setParam(ID(TEST_MODE), value(param("TEST_MODE", "0"), 1));
					Const ext = param("EXTERNAL_DIVIDE_FACTOR", "NONE");
					n->setParam(ID(EXTERNAL_DIVIDE_FACTOR), ext.decode_string() == "NONE" ? Const(1, 32) : value(ext, 32));
				} else if (t == ID(MAC16)) {
					make("SB_MAC16");
					for (auto p : {"CLK", "CE", "AHOLD", "BHOLD", "CHOLD", "DHOLD", "IRSTTOP", "IRSTBOT", "ORSTTOP",
							"ORSTBOT", "OLOADTOP", "OLOADBOT", "ADDSUBTOP", "ADDSUBBOT", "OHOLDTOP", "OHOLDBOT",
							"CI", "ACCUMCI", "SIGNEXTIN", "CO", "ACCUMCO", "SIGNEXTOUT"})
						conn(p, port(p));
					for (auto p : {"A", "B", "C", "D"})
						conn(p, bus(p, 16));
					SigSpec o;
					for (int i = 0; i < 32; i++) {
						SigSpec b = port(stringf("O%d", i).c_str());
						o.append(b.empty() ? SigSpec(module->addWire(NEW_ID)) : b);
					}
					conn("O", o);
					for (auto p : {"NEG_TRIGGER", "A_REG", "B_REG", "C_REG", "D_REG", "TOP_8x8_MULT_REG", "BOT_8x8_MULT_REG",
							"PIPELINE_16x16_MULT_REG1", "PIPELINE_16x16_MULT_REG2", "TOPADDSUB_UPPERINPUT",
							"BOTADDSUB_UPPERINPUT", "MODE_8x8", "A_SIGNED", "B_SIGNED"})
						n->setParam(RTLIL::escape_id(p), value(param(p, "0b0"), 1));
					for (auto p : {"TOPOUTPUT_SELECT", "TOPADDSUB_LOWERINPUT", "TOPADDSUB_CARRYSELECT", "BOTOUTPUT_SELECT",
							"BOTADDSUB_LOWERINPUT", "BOTADDSUB_CARRYSELECT"})
						n->setParam(RTLIL::escape_id(p), value(param(p, "0b00"), 2));
				} else if (t.in(ID(I2C_B), ID(SPI_B))) {
					bool i2c = t == ID(I2C_B);
					make(i2c ? "SB_I2C" : "SB_SPI");
					for (auto p : {"SBCLKI", "SBRWI", "SBSTBI", "SBACKO"})
						conn(p, port(p));
					for (int i = 0; i < 8; i++) {
						conn(stringf("SBADRI%d", i).c_str(), port(stringf("SBADRI%d", i).c_str()));
						conn(stringf("SBDATI%d", i).c_str(), port(stringf("SBDATI%d", i).c_str()));
						conn(stringf("SBDATO%d", i).c_str(), port(stringf("SBDATO%d", i).c_str()));
					}
					if (i2c) {
						for (auto p : {"SCLI", "SDAI", "I2CIRQ", "I2CWKUP", "SCLO", "SCLOE", "SDAO", "SDAOE"})
							conn(p, port(p));
						n->setParam(ID(I2C_SLAVE_INIT_ADDR), Const(bin_string(param("I2C_SLAVE_INIT_ADDR", "0b1111100001"), 10)));
						n->setParam(ID(BUS_ADDR74), Const(bin_string(param("BUS_ADDR74", "0b0001"), 4)));
					} else {
						for (auto p : {"MI", "SI", "SCKI", "SCSNI", "SPIIRQ", "SPIWKUP", "SO", "SOE", "MO", "MOE", "SCKO", "SCKOE"})
							conn(p, port(p));
						for (int i = 0; i < 4; i++) {
							conn(stringf("MCSNO%d", i).c_str(), port(stringf("MCSNO%d", i).c_str()));
							conn(stringf("MCSNOE%d", i).c_str(), port(stringf("MCSNOE%d", i).c_str()));
						}
						n->setParam(ID(BUS_ADDR74), Const(bin_string(param("BUS_ADDR74", "0b0000"), 4)));
					}
				} else if (t.in(ID(IB), ID(OB))) {
					module->connect(port("O"), port("I"));
				} else if (t == ID(OBZ_B)) {
					// despite its name, T_N high drives the pad (Radiant's PIO model)
					module->addTribuf(NEW_ID, port("I"), port("T_N"), port("O"));
				} else if (t.in(ID(BB_B), ID(OB_RGB))) {
					module->addTribuf(NEW_ID, port("I"), port("T_N"), port("B"));
					if (!port("O").empty())
						module->connect(port("O"), port("B"));
				} else if (t == ID(BB_OD)) {
					// open drain: pulls low while T_N is high and I is low
					SigBit en = module->AndnotGate(NEW_ID, port("T_N").as_bit(), port("I").as_bit());
					module->addTribuf(NEW_ID, State::S0, en, port("B"));
					if (!port("O").empty())
						module->connect(port("O"), port("B"));
				} else if (t == ID(BB_I3C)) {
					make("SB_IO_I3C");
					conn("PACKAGE_PIN", port("B")); conn("OUTPUT_ENABLE", port("T_N")); conn("D_OUT_0", port("I"));
					conn("D_IN_0", port("O")); conn("PU_ENB", port("PU_ENB")); conn("WEAK_PU_ENB", port("WEAK_PU_ENB"));
					n->setParam(ID(PIN_TYPE), Const(0b101001, 6));
					n->setParam(ID(PULLUP), Const(0, 1));
					n->setParam(ID(WEAK_PULLUP), Const(0, 1));
					n->setParam(ID(NEG_TRIGGER), Const(0, 1));
				} else if (t == ID(HSOSC_CORE)) {
					make("SB_HFOSC");
					conn("CLKHFPU", port("CLKHFPU")); conn("CLKHFEN", port("CLKHFEN")); conn("CLKHF", port("CLKHF"));
					for (int i = 0; i < 10; i++)
						conn(stringf("TRIM%d", i).c_str(), port(stringf("TRIM%d", i).c_str()));
					n->setParam(ID(CLKHF_DIV), Const(bin_string(param("CLKHF_DIV", "0b00"), 2)));
					n->setParam(ID(TRIM_EN), Const(param("FABRIC_TRIME", "DISABLE").decode_string() == "ENABLE" ? "0b1" : "0b0"));
				} else if (t == ID(LSOSC_CORE)) {
					make("SB_LFOSC");
					conn("CLKLFPU", port("CLKLFPU")); conn("CLKLFEN", port("CLKLFEN")); conn("CLKLF", port("CLKLF"));
					if (param("FABRIC_TRIME", "DISABLE").decode_string() == "ENABLE")
						log_warning("%s: SB_LFOSC has no trim inputs, TRIM of %s ignored.\n", log_id(module), log_id(cell));
				} else if (t == ID(RGB_CORE)) {
					make("SB_RGBA_DRV");
					for (auto p : {"CURREN", "RGBLEDEN", "RGB0PWM", "RGB1PWM", "RGB2PWM", "RGB0", "RGB1", "RGB2"})
						conn(p, port(p));
					n->setParam(ID(CURRENT_MODE), Const(bin_string(param("CURRENT_MODE", "0"), 1)));
					for (auto p : {"RGB0_CURRENT", "RGB1_CURRENT", "RGB2_CURRENT"})
						n->setParam(RTLIL::escape_id(p), Const(bin_string(param(p, "0b000000"), 6)));
					if (param("FABRIC_TRIME", "DISABLE").decode_string() == "ENABLE")
						log_warning("%s: SB_RGBA_DRV has no trim inputs, TRIM of %s ignored.\n", log_id(module), log_id(cell));
				} else if (t == ID(RGBPWM)) {
					make("SB_LEDDA_IP");
					for (auto p : {"LEDDCS", "LEDDCLK", "LEDDDEN", "LEDDEXE", "PWMOUT0", "PWMOUT1", "PWMOUT2", "LEDDON"})
						conn(p, port(p));
					for (int i = 0; i < 8; i++)
						conn(stringf("LEDDDAT%d", i).c_str(), port(stringf("LEDDDAT%d", i).c_str()));
					for (int i = 0; i < 4; i++)
						conn(stringf("LEDDADDR%d", i).c_str(), port(stringf("LEDDADDR%d", i).c_str()));
					conn("LEDDRST", State::S0);
				} else if (t == ID(VFB_B)) {
					make("SB_SPRAM256KA");
					conn("ADDRESS", bus("ADDRESS", 14)); conn("DATAIN", bus("DATAIN", 16)); conn("MASKWREN", bus("MASKWREN", 4));
					SigSpec o;
					for (int i = 0; i < 16; i++) {
						SigSpec b = port(stringf("DATAOUT%d", i).c_str());
						o.append(b.empty() ? SigSpec(module->addWire(NEW_ID)) : b);
					}
					conn("DATAOUT", o);
					conn("WREN", port("WREN")); conn("CHIPSELECT", port("CHIPSELECT")); conn("CLOCK", port("CLOCK"));
					conn("STANDBY", port("STANDBY")); conn("SLEEP", port("SLEEP")); conn("POWEROFF", port("POWEROFF_N"));
				} else if (t.in(ID(EBR_B), ID(PDP4K))) {
					bool pdp = t == ID(PDP4K);
					make("SB_RAM40_4K");
					auto vec = [&](const char *ebr, const char *pdp_name, int width) {
						if (!pdp)
							return bus(ebr, width);
						SigSpec s = port(pdp_name);
						if (s.empty()) s = SigSpec(State::S0, width);
						s.extend_u0(width);
						return s;
					};
					conn("RADDR", vec("RADDR", "ADR", 11)); conn("WADDR", vec("WADDR", "ADW", 11));
					conn("MASK", vec("MASK_N", "MASK_N", 16)); conn("WDATA", vec("WDATA", "DI", 16));
					SigSpec rd = pdp ? port("DO") : SigSpec();
					if (!pdp)
						for (int i = 0; i < 16; i++) {
							SigSpec b = port(stringf("RDATA%d", i).c_str());
							rd.append(b.empty() ? SigSpec(module->addWire(NEW_ID)) : b);
						}
					if (rd.empty()) rd = module->addWire(NEW_ID, 16);
					conn("RDATA", rd);
					conn("RCLK", port(pdp ? "CKR" : "RCLK")); conn("WCLK", port(pdp ? "CKW" : "WCLK"));
					conn("RCLKE", port(pdp ? "CER" : "RCLKE")); conn("WCLKE", port(pdp ? "CEW" : "WCLKE"));
					conn("RE", port("RE")); conn("WE", port("WE"));
					auto mode = [&](const char *p) {
						int w = value(param(p, "2"), 32).as_int();
						return Const(w == 16 ? 0 : w == 8 ? 1 : w == 4 ? 2 : 3, 2);
					};
					n->setParam(ID(WRITE_MODE), mode("DATA_WIDTH_W"));
					n->setParam(ID(READ_MODE), mode("DATA_WIDTH_R"));
					for (int i = 0; i < 16; i++) {
						std::string pname = pdp ? stringf("INITVAL_%X", i) : stringf("INIT_%X", i);
						Const v = param(pname.c_str(), "0x0");
						std::string h = (v.flags & RTLIL::CONST_FLAG_STRING) ? v.decode_string() : "";
						Const init(State::S0, 256);
						if (h.size() > 2 && h[0] == '0' && (h[1] == 'x' || h[1] == 'X')) {
							int bit = 0;
							for (int k = GetSize(h) - 1; k >= 2 && bit < 256; k--, bit += 4) {
								int d = isdigit(h[k]) ? h[k] - '0' : (tolower(h[k]) - 'a' + 10);
								for (int j = 0; j < 4; j++)
									init.set(bit + j, (d >> j) & 1 ? State::S1 : State::S0);
							}
						} else if (!(v.flags & RTLIL::CONST_FLAG_STRING))
							init = Const(v.to_bits()), init.resize(256, State::S0);
						n->setParam(RTLIL::escape_id(stringf("INIT_%X", i)), init);
					}
				} else if (t.in(ID(FD1P3XZ), ID(FD1P3BZ), ID(FD1P3DZ), ID(FD1P3IZ), ID(FD1P3JZ))) {
					bool async, set;
					SigSpec sr;
					if (t == ID(FD1P3XZ)) {
						async = param("SRMODE", "CE_OVER_LSR").decode_string() == "ASYNC";
						set = param("REGSET", "RESET").decode_string() == "SET";
						sr = port("SR");
					} else {
						async = t.in(ID(FD1P3BZ), ID(FD1P3DZ));
						set = t.in(ID(FD1P3BZ), ID(FD1P3JZ));
						sr = port(set ? "PD" : "CD");
					}
					SigSpec sp = port("SP");
					if (sp.empty()) sp = State::S1;
					bool has_sr = !sr.empty() && sr != SigSpec(State::S0);
					std::string type = "SB_DFFE";
					if (has_sr)
						type += async ? (set ? "S" : "R") : (set ? "SS" : "SR");
					make(type.c_str());
					SigSpec dd = port("D");
					conn("C", port("CK")); conn("E", sp); conn("D", dd.empty() ? SigSpec(State::S0) : dd); conn("Q", port("Q"));
					if (has_sr)
						conn(set ? "S" : "R", sr);
				} else if (t.in(ID(IFD1P3AZ), ID(OFD1P3AZ))) {
					make("SB_DFFE");
					SigSpec sp = port("SP");
					conn("C", port("CK")); conn("E", sp.empty() ? SigSpec(State::S1) : sp); conn("D", port("D")); conn("Q", port("Q"));
				} else if (t == ID(IOL_B)) {
					// I/O registers as plain flip-flops; ice40_iopack packs them into the SB_IO
					SigSpec ce = port("CE");
					if (ce.empty()) ce = State::S1;
					if (param("DDROUT", "NO").decode_string() == "YES" && !port("PADDO").empty())
						log_error("%s: DDR output of %s is not supported.\n", log_id(module), log_id(cell));
					auto ff = [&](const char *type, SigSpec clk, SigSpec d, SigSpec q) {
						if (q.empty() || clk.empty() || clk == SigSpec(State::S0))
							return;
						Cell *f = module->addCell(NEW_ID, RTLIL::escape_id(type));
						f->setPort(ID::C, clk); f->setPort(ID::E, ce); f->setPort(ID::D, d); f->setPort(ID::Q, q);
					};
					ff("SB_DFFE", port("OUTCLK"), port("DO0"), port("PADDO"));
					ff("SB_DFFE", port("OUTCLK"), port("IOLTO"), port("PADDT"));
					std::string latchin = param("LATCHIN", "NONE_REG").decode_string();
					if (latchin == "NONE_REG" || latchin == "NONE_DDR" || latchin == "LATCH_REG")
						ff("SB_DFFE", port("INCLK"), port("PADDI"), port("DI0"));
					else if (!port("DI0").empty())
						module->connect(port("DI0"), port("PADDI"));
					ff("SB_DFFNE", port("INCLK"), port("PADDI"), port("DI1"));
				} else if (t == ID(LUT4)) {
					make("SB_LUT4");
					const char *in[4] = {"A", "B", "C", "D"};
					for (int i = 0; i < 4; i++) {
						SigSpec s = port(in[i]);
						conn(stringf("I%d", i).c_str(), s.empty() ? SigSpec(State::S0) : s);
					}
					conn("O", port("Z"));
					n->setParam(ID(LUT_INIT), value(param("INIT", "0x0000"), 16));
				} else if (t.in(ID(FA2), ID(CCU2_B))) {
					// each half is an iCE40 logic cell: a LUT for the sum, SB_CARRY on B, C
					bool ccu = t == ID(CCU2_B);
					SigSpec ci0 = port(ccu ? "CIN" : "CI0");
					SigSpec co0 = port("CO0");
					if (ccu || co0.empty())
						co0 = module->addWire(NEW_ID);
					for (int h = 0; h < 2; h++) {
						auto P = [&](const char *b) { return port(stringf("%s%d", b, h).c_str()); };
						SigSpec a = P("A"), b = P("B"), c = P("C"), d = ccu ? (h ? co0 : ci0) : P("D");
						SigSpec ci = h ? (ccu ? co0 : port("CI1")) : ci0;
						SigSpec co = h ? port(ccu ? "COUT" : "CO1") : co0;
						Cell *l = module->addCell(NEW_ID, ID(SB_LUT4));
						l->setPort(ID(I0), a.empty() ? SigSpec(State::S0) : a);
						l->setPort(ID(I1), b.empty() ? SigSpec(State::S0) : b);
						l->setPort(ID(I2), c.empty() ? SigSpec(State::S0) : c);
						l->setPort(ID(I3), d.empty() ? SigSpec(State::S0) : d);
						SigSpec sum = P("S");
						l->setPort(ID::O, sum.empty() ? SigSpec(module->addWire(NEW_ID)) : sum);
						l->setParam(ID(LUT_INIT), value(param(h ? "INIT1" : "INIT0", "0xc33c"), 16));
						if (!co.empty()) {
							Cell *k = module->addCell(NEW_ID, ID(SB_CARRY));
							k->setPort(ID(I0), b.empty() ? SigSpec(State::S0) : b);
							k->setPort(ID(I1), c.empty() ? SigSpec(State::S0) : c);
							// an unconnected carry-in is 1 in Radiant's model
							k->setPort(ID(CI), ci.empty() ? SigSpec(State::S1) : ci);
							k->setPort(ID(CO), co);
						}
					}
				} else if (t == ID(INV)) {
					module->addNotGate(NEW_ID, port("A").as_bit(), port("Z").as_bit());
				} else if (t == ID(BUF)) {
					module->connect(port("Z"), port("A"));
				} else if (t == ID(VHI)) {
					module->connect(port("Z"), State::S1);
				} else if (t == ID(VLO)) {
					module->connect(port("Z"), State::S0);
				} else if (t == ID(PUR)) {
					// power-up reset for simulation only; the device has none to configure
				} else
					continue;
				IdString name = cell->name;
				module->remove(cell);
				if (n)
					module->rename(n, name);
				count++;
			}
			if (count)
				log("%s: %d Radiant primitives replaced.\n", log_id(module), count);
		}
	}
} Ice40RadiantPass;

PRIVATE_NAMESPACE_END
