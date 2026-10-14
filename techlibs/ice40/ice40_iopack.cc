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

struct Ice40IopackPass : public Pass {
	Ice40IopackPass() : Pass("ice40_iopack", "pack I/O registers and pads into SB_IO cells") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_iopack\n");
		log("\n");
		log("Packs I/O registers and pads the way Lattice LSE does. Run it on the mapped top\n");
		log("module (synth_ice40 does):\n");
		log("\n");
		log("    - a flip-flop that is the only driver of an output port, feeding nothing\n");
		log("      else, becomes the pad's output register; a synchronous reset/set is\n");
		log("      folded into a LUT in front of it\n");
		log("    - a flip-flop that is the only reader of an input/inout pad becomes the\n");
		log("      pad's input register; a falling-edge clock is replaced by an inverted\n");
		log("      copy of the LUT driving it, so the tile's shared NegClk bit stays free\n");
		log("    - every other pad gets an explicit SB_IO, with the unused input half of\n");
		log("      an output-only pad set to 2'b00 (plain) or 2'b10 (registered output)\n");
		log("    - the 100k pull-up is enabled on every SB_IO\n");
		log("\n");
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);

		log_header(design, "Executing ICE40_IOPACK pass (pack I/O registers and pads into SB_IO).\n");
		Module *module = design->top_module();
		if (module == nullptr)
			log_cmd_error("No top module found.\n");

		SigMap sigmap(module);
		dict<SigBit, std::pair<Cell*, IdString>> driver;
		dict<SigBit, int> nusers;
		dict<SigBit, SigBit> port_bit;
		pool<SigBit> useioff_bits, useioff_attr;

		for (auto wire : module->wires()) {
			if (!wire->port_id)
				continue;
			for (auto bit : SigSpec(wire)) {
				port_bit[sigmap(bit)] = bit;
				if (wire->port_output)
					nusers[sigmap(bit)]++;
				if (wire->port_output)
					useioff_bits.insert(sigmap(bit));
				if (wire->port_output && wire->has_attribute(ID(syn_useioff)))
					useioff_attr.insert(sigmap(bit));
			}
		}
		for (auto cell : module->cells())
			for (auto &conn : cell->connections())
				for (auto bit : sigmap(conn.second)) {
					if (bit.wire == nullptr)
						continue;
					if (cell->output(conn.first))
						driver[bit] = {cell, conn.first};
					else
						nusers[bit]++;
				}

		const int in_unused = 0b10;
		std::vector<Cell*> remove;
		dict<SigBit, SigBit> inverted_clock;
		int n_out = 0, n_in = 0;

		auto lut = [&](SigSpec inputs, int init) {
			Wire *out = module->addWire(NEW_ID);
			Cell *c = module->addCell(NEW_ID, ID(SB_LUT4));
			inputs.append(SigSpec(State::S0, 4 - GetSize(inputs)));
			c->setPort(ID(I0), inputs[0]);
			c->setPort(ID(I1), inputs[1]);
			c->setPort(ID(I2), inputs[2]);
			c->setPort(ID(I3), inputs[3]);
			c->setPort(ID::O, out);
			c->setParam(ID(LUT_INIT), Const(init, 16));
			return SigBit(out);
		};

		// LSE puts the inversion into the logic that makes the clock rather than
		// using NegClk, which the two pads of an I/O tile share
		auto invert_clock = [&](SigBit clk) {
			clk = sigmap(clk);
			if (inverted_clock.count(clk))
				return inverted_clock.at(clk);
			SigBit inv;
			auto it = driver.find(clk);
			if (it != driver.end() && it->second.first->type == ID(SB_LUT4) && it->second.second == ID::O) {
				Cell *src = it->second.first;
				SigSpec in = {src->getPort(ID(I0)), src->getPort(ID(I1)), src->getPort(ID(I2)), src->getPort(ID(I3))};
				inv = lut(in, ~src->getParam(ID(LUT_INIT)).as_int() & 0xffff);
			} else
				inv = lut(clk, 0x0001);
			return inverted_clock[clk] = inv;
		};

		auto new_io = [&](SigBit pin, int out_type, int in_type) {
			Cell *io = module->addCell(NEW_ID, ID(SB_IO));
			io->setPort(ID(PACKAGE_PIN), pin);
			io->setParam(ID(PIN_TYPE), Const(out_type << 2 | in_type, 6));
			// LSE enables the pull-up only on pads that are read
			io->setParam(ID(PULLUP), Const(in_type == 0b10 && out_type != 0 ? 0 : 1, 1));
			io->setParam(ID(NEG_TRIGGER), Const(0, 1));
			return io;
		};

		for (auto bit : useioff_bits) {
			auto it = driver.find(bit);
			if (it == driver.end() || it->second.second != ID::Q || nusers[bit] != 1)
				continue;
			Cell *ff = it->second.first;
			IdString t = ff->type;
			if (!t.in(ID(SB_DFF), ID(SB_DFFE), ID(SB_DFFSR), ID(SB_DFFESR), ID(SB_DFFSS), ID(SB_DFFESS)))
				continue;
			// LSE keeps a flip-flop with a synchronous reset/set in the fabric unless the
			// port asks for the I/O register (syn_useioff)
			if (t.in(ID(SB_DFFSR), ID(SB_DFFESR), ID(SB_DFFSS), ID(SB_DFFESS)) && !useioff_attr.count(bit))
				continue;
			SigBit d = ff->getPort(ID::D);
			if (t.in(ID(SB_DFFSR), ID(SB_DFFESR)))
				d = lut({d, ff->getPort(ID::R)}, 0x0002);
			else if (t.in(ID(SB_DFFSS), ID(SB_DFFESS)))
				d = lut({d, ff->getPort(ID::S)}, 0x000e);
			Cell *io = new_io(port_bit.at(bit), 0b0101, in_unused);
			io->setPort(ID(OUTPUT_CLK), ff->getPort(ID::C));
			io->setPort(ID(D_OUT_0), d);
			if (ff->hasPort(ID::E))
				io->setPort(ID(CLOCK_ENABLE), ff->getPort(ID::E));
			remove.push_back(ff);
			n_out++;
		}

		// a flip-flop from an input pad straight to an output pad is already the
		// output register
		pool<Cell*> packed(remove.begin(), remove.end());
		for (auto ff : module->selected_cells()) {
			IdString t = ff->type;
			if (!t.in(ID(SB_DFF), ID(SB_DFFN), ID(SB_DFFE), ID(SB_DFFNE)) || packed.count(ff))
				continue;
			SigBit d = sigmap(ff->getPort(ID::D));
			if (!port_bit.count(d))
				continue;
			Wire *pw = port_bit.at(d).wire;
			if (!pw->port_input || nusers[d] != 1 + (pw->port_output ? 1 : 0))
				continue;
			Cell *tbuf = nullptr;
			auto it = driver.find(d);
			if (it != driver.end()) {
				if (it->second.first->type != ID($_TBUF_))
					continue;
				tbuf = it->second.first;
			}
			SigBit clk = ff->getPort(ID::C);
			if (t.in(ID(SB_DFFN), ID(SB_DFFNE)))
				clk = invert_clock(clk);
			Cell *io = new_io(port_bit.at(d), tbuf ? 0b1010 : 0b0000, 0b00);
			io->setPort(ID(INPUT_CLK), clk);
			io->setPort(ID(D_IN_0), ff->getPort(ID::Q));
			if (ff->hasPort(ID::E))
				io->setPort(ID(CLOCK_ENABLE), ff->getPort(ID::E));
			if (tbuf) {
				io->setPort(ID(D_OUT_0), tbuf->getPort(ID::A));
				io->setPort(ID(OUTPUT_ENABLE), tbuf->getPort(ID::E));
				remove.push_back(tbuf);
			}
			remove.push_back(ff);
			n_in++;
		}

		for (auto cell : remove)
			module->remove(cell);
		log("Packed %d output and %d input registers into SB_IO cells.\n", n_out, n_in);

		Pass::call(design, stringf("iopadmap -bits -inpad $__ICE40_IOPACK_IN Y:P -outpad $__ICE40_IOPACK_OUT A:P "
				"-toutpad $__ICE40_IOPACK_TOUT E:A:P -tinoutpad $__ICE40_IOPACK_TINOUT E:Y:A:P "
				"-ignore SB_IO PACKAGE_PIN -ignore SB_IO_I3C PACKAGE_PIN -ignore SB_IO_OD PACKAGEPIN "
				"-ignore SB_RGBA_DRV RGB0 -ignore SB_RGBA_DRV RGB1 -ignore SB_RGBA_DRV RGB2 "
				"-ignore SB_RGB_DRV RGB0 -ignore SB_RGB_DRV RGB1 -ignore SB_RGB_DRV RGB2 %s", log_id(module)));
		Pass::call(design, stringf("techmap -map +/ice40/iopack_map.v -D IOPACK_PULLUP=1 %s", log_id(module)));
	}
} Ice40IopackPass;

PRIVATE_NAMESPACE_END
