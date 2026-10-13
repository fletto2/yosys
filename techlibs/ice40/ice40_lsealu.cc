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
#include <functional>

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct Ice40LseAluPass : public Pass {
	Ice40LseAluPass() : Pass("ice40_lsealu", "iCE40: map $alu cells to LUT logic where Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsealu [options] [selection]\n");
		log("\n");
		log("Maps the $alu cells that Lattice LSE builds from LUTs rather than a carry chain\n");
		log("to plain gates (generic techmap), leaving the rest for arith_map.v:\n");
		log("\n");
		log("    - adders/subtractors narrower than -min_width bits (default 6)\n");
		log("    - comparisons (sum output unused), at any width\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		int min_width = 6;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-min_width" && argidx + 1 < args.size()) {
				min_width = atoi(args[++argidx].c_str());
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);
		log_header(design, "Executing ICE40_LSEALU pass.\n");

		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			pool<SigBit> used;
			for (auto cell : module->cells())
				for (auto &conn : cell->connections())
					if (!cell->output(conn.first))
						for (auto bit : sigmap(conn.second))
							used.insert(bit);
			for (auto wire : module->wires())
				if (wire->port_output)
					for (auto bit : sigmap(SigSpec(wire)))
						used.insert(bit);

			std::vector<Cell*> soft;
			for (auto cell : module->selected_cells()) {
				if (cell->type != ID($alu))
					continue;
				bool narrow = cell->getParam(ID::Y_WIDTH).as_int() < min_width;
				bool sum_used = false;
				for (auto bit : sigmap(cell->getPort(ID::Y)))
					if (used.count(bit))
						sum_used = true;
				bool const_operand = sigmap(cell->getPort(ID::A)).is_fully_const() ||
						sigmap(cell->getPort(ID::B)).is_fully_const();
				(void)const_operand;
				if (narrow || !sum_used)
					soft.push_back(cell);
			}
			for (auto cell : soft)
				cell->set_bool_attribute(ID(ice40_lsealu));
			log("%s: %d $alu cells mapped to LUT logic.\n", log_id(module), GetSize(soft));
		}
		Pass::call(design, "techmap a:ice40_lsealu");
	}
} Ice40LseAluPass;

struct Ice40LseAddEnablePass : public Pass {
	Ice40LseAddEnablePass() : Pass("ice40_lseaddenable", "fold an enable into the adder, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lseaddenable [selection]\n");
		log("\n");
		log("Rewrites  S ? Q + K : Q  as  Q + (S ? K : 0)  (likewise for -), so a counter like\n");
		log("`if (en) q <= q + 1' adds `en' instead of using a clock enable. This is what LSE\n");
		log("does; run it before opt_dff extracts the enable.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEADDENABLE pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, std::pair<Cell*, int>> add_out;      // bit -> (add/sub cell, index in Y)
			for (auto cell : module->cells())
				if (cell->type.in(ID($add), ID($sub))) {
					SigSpec y = sigmap(cell->getPort(ID::Y));
					for (int i = 0; i < GetSize(y); i++)
						add_out[y[i]] = {cell, i};
				}
			// LSE only folds when `if (en) q <= q + k' is the register's whole next-state
			// logic: the mux must feed a flip-flop's D directly, holding that flip-flop
			// (through muxes that only add a constant, i.e. a sync reset/set in front)
			dict<SigSpec, Cell*> mux_of_y, pmux_of_y;
			for (auto cell : module->cells())
				if (cell->type == ID($mux))
					mux_of_y[sigmap(cell->getPort(ID::Y))] = cell;
				else if (cell->type == ID($pmux))
					pmux_of_y[sigmap(cell->getPort(ID::Y))] = cell;
			dict<SigSpec, SigSpec> ff_q_of_d;
			pool<SigSpec> behind_const;

			for (auto cell : module->cells())
				if (cell->type.in(ID($dff), ID($adff), ID($dffe), ID($adffe))) {
					SigSpec d = sigmap(cell->getPort(ID::D)), q = sigmap(cell->getPort(ID::Q));
					ff_q_of_d[d] = q;
					for (int depth = 0; depth < 4 && mux_of_y.count(d); depth++) {
						Cell *m = mux_of_y.at(d);
						SigSpec ma = sigmap(m->getPort(ID::A)), mb = sigmap(m->getPort(ID::B));
						if (ma.is_fully_const()) d = mb;
						else if (mb.is_fully_const()) d = ma;
						else break;
						ff_q_of_d[d] = q;
						if (cell->type.in(ID($adff), ID($adffe)))
							behind_const.insert(d);
					}
					// with an async reset, LSE also keeps the enable when the add is one of
					// several next-state choices (a clear or a load in front of it)
					if (!cell->type.in(ID($adff), ID($adffe)))
						continue;
					std::vector<SigSpec> todo = {sigmap(cell->getPort(ID::D))};
					pool<SigSpec> seen;
					while (!todo.empty()) {
						SigSpec x = todo.back(); todo.pop_back();
						if (seen.count(x) || GetSize(seen) > 16)
							continue;
						seen.insert(x);
						Cell *m = mux_of_y.count(x) ? mux_of_y.at(x) : (pmux_of_y.count(x) ? pmux_of_y.at(x) : nullptr);
						if (m == nullptr)
							continue;
						SigSpec ma = sigmap(m->getPort(ID::A)), mb = sigmap(m->getPort(ID::B));
						std::vector<SigSpec> ins = {ma};
						for (int i = 0; i + GetSize(ma) <= GetSize(mb); i += GetSize(ma))
							ins.push_back(mb.extract(i, GetSize(ma)));
						bool load = false;
						for (auto &in : ins)
							if (!in.is_fully_const() && in != q && !mux_of_y.count(in) && !pmux_of_y.count(in))
								load = true;
						for (auto &in : ins)
							if (mux_of_y.count(in) || pmux_of_y.count(in)) {
								if (load || m->type == ID($pmux))
									behind_const.insert(in);
								todo.push_back(in);
							}
					}
				}
			int count = 0;
			for (auto mux : module->selected_cells()) {
				if (mux->type != ID($mux))
					continue;
				SigSpec my = sigmap(mux->getPort(ID::Y));
				// muxes not in front of a flip-flop are folded too
				bool direct = ff_q_of_d.count(my);
				if (behind_const.count(my))
					continue;

				SigSpec a = sigmap(mux->getPort(ID::A)), b = sigmap(mux->getPort(ID::B));
				for (int side = 0; side < 2; side++) {
					SigSpec q = side ? b : a, sum = side ? a : b;   // the hold input and the add input
					if (direct && q != ff_q_of_d.at(my))
						continue;
					if (!add_out.count(sum[0]))
						continue;
					Cell *add = add_out.at(sum[0]).first;
					SigSpec y = sigmap(add->getPort(ID::Y));
					if (GetSize(sum) > GetSize(y) || y.extract(0, GetSize(sum)) != sum)
						continue;
					SigSpec opA = sigmap(add->getPort(ID::A)), opB = sigmap(add->getPort(ID::B));
					// the add must take the hold value (zero-extended or truncated) as one operand
					auto same = [&](const SigSpec &op) {
						int n = std::min(GetSize(op), GetSize(q));
						if (op.extract(0, n) != q.extract(0, n))
							return false;
						for (int i = n; i < GetSize(op); i++)
							if (op[i] != State::S0) return false;
						return n == GetSize(q);
					};
					IdString kport;
					if (same(opA)) kport = ID::B;
					else if (same(opB) && add->type == ID($add)) kport = ID::A;
					else continue;
					SigSpec k = add->getPort(kport);
					// measured: LSE keeps the enable when a constant step has exactly one
					// trailing zero (2, 6, 10, 14, ...) and folds it for every other step
					if (k.is_fully_const() && GetSize(k) >= 2 && k[0] == State::S0 && k[1] == State::S1)
						continue;
					SigSpec gated = side ? module->Mux(NEW_ID, k, SigSpec(State::S0, GetSize(k)), mux->getPort(ID::S))
							: module->Mux(NEW_ID, SigSpec(State::S0, GetSize(k)), k, mux->getPort(ID::S));
					Cell *nadd = module->addCell(NEW_ID, add->type);
					nadd->parameters = add->parameters;
					nadd->setPort(ID::A, kport == ID::B ? add->getPort(ID::A) : gated);
					nadd->setPort(ID::B, kport == ID::B ? gated : add->getPort(ID::B));
					Wire *ny = module->addWire(NEW_ID, GetSize(y));
					nadd->setPort(ID::Y, ny);
					module->connect(mux->getPort(ID::Y), SigSpec(ny).extract(0, GetSize(sum)));
					mux->setPort(ID::Y, module->addWire(NEW_ID, GetSize(sum)));
					count++;
					break;
				}
			}
			log("%s: folded %d enables into adders.\n", log_id(module), count);
		}
	}
} Ice40LseAddEnablePass;

struct Ice40LseRamCePass : public Pass {
	Ice40LseRamCePass() : Pass("ice40_lseramce", "drop the clock enable of flip-flops loaded from RAM read data, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lseramce\n");
		log("\n");
		log("LSE (area goal) does not give a flip-flop a clock enable when its D comes straight\n");
		log("from an SPRAM or EBR read port and its output is read by other logic (one that\n");
		log("only drives a top-level output keeps it): the enable becomes a hold mux in front\n");
		log("of D. This pass does the same to SB_DFF*E* cells, following RAM read data through\n");
		log("muxes of RAM data and module ports.\n");
		log("Run after the flip-flops are mapped to SB_DFF* cells.\n");

		log("\n");
	}

	typedef std::pair<IdString, IdString> RamId;	// (module, cell)
	Design *design;
	dict<IdString, dict<SigBit, pool<RamId>>> ram_bits;	// per module: bit -> RAMs whose read data it is
	dict<IdString, dict<std::pair<IdString, int>, pool<RamId>>> in_ram, out_ram;	// port bit -> RAMs
	dict<IdString, std::vector<std::pair<Module*, Cell*>>> parents;

	static bool is_ram(Cell *cell) {
		return cell->type.in(ID(SB_RAM40_4K), ID(SB_RAM40_4KNR), ID(SB_RAM40_4KNW), ID(SB_RAM40_4KNRNW), ID(SB_SPRAM256KA));
	}

	void compute(Module *module)
	{
		SigMap sigmap(module);
		dict<SigBit, pool<RamId>> bits;
		for (auto &it : in_ram[module->name]) {
			Wire *w = module->wire(it.first.first);
			if (w && it.first.second < w->width)
				bits[sigmap(SigBit(w, it.first.second))] = it.second;
		}
		for (auto cell : module->cells()) {
			if (is_ram(cell))
				for (auto b : sigmap(cell->getPort(cell->type == ID(SB_SPRAM256KA) ? ID(DATAOUT) : ID(RDATA))))
					bits[b].insert(RamId(module->name, cell->name));
			if (out_ram.count(cell->type))
				for (auto &it : out_ram.at(cell->type))
					if (cell->hasPort(it.first.first) && it.first.second < GetSize(cell->getPort(it.first.first)))
						bits[sigmap(cell->getPort(it.first.first)[it.first.second])] = it.second;
		}
		// data selected by a mux from RAM read data is still RAM data (bank select,
		// byte select, a register holding the last read)
		for (bool changed = true; changed; ) {
			changed = false;
			for (auto cell : module->cells())
				if (cell->type.in(ID($_MUX_), ID($mux))) {
					SigSpec a = sigmap(cell->getPort(ID::A)), b = sigmap(cell->getPort(ID::B)), y = sigmap(cell->getPort(ID::Y));
					for (int i = 0; i < GetSize(y); i++)
						if (!bits.count(y[i]) && (bits.count(a[i]) || bits.count(b[i]))) {
							pool<RamId> r;
							if (bits.count(a[i])) r = bits.at(a[i]);
							if (bits.count(b[i])) for (auto &x : bits.at(b[i])) r.insert(x);
							bits[y[i]] = r;
							changed = true;
						}
				}
		}
		ram_bits[module->name] = bits;
	}

	// is the signal read by any logic (module output ports followed up the hierarchy;
	// a top-level output alone does not count)
	bool used_internally(Module *module, SigBit bit, int depth = 0)
	{
		SigMap sigmap(module);
		bit = sigmap(bit);
		for (auto cell : module->cells())
			for (auto &conn : cell->connections())
				if (!cell->output(conn.first))
					for (auto b : sigmap(conn.second))
						if (b == bit)
							return true;
		if (depth > 8 || !parents.count(module->name))
			return false;
		for (auto w : module->wires())
			if (w->port_output)
				for (int i = 0; i < w->width; i++)
					if (sigmap(SigBit(w, i)) == bit)
						for (auto &pc : parents.at(module->name))
							if (pc.second->hasPort(w->name) && i < GetSize(pc.second->getPort(w->name)) &&
									used_internally(pc.first, pc.second->getPort(w->name)[i], depth + 1))
								return true;
		return false;
	}

	void execute(std::vector<std::string> args, RTLIL::Design *d) override
	{
		design = d;
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSERAMCE pass.\n");
		ram_bits.clear(); in_ram.clear(); out_ram.clear(); parents.clear();

		for (auto module : design->modules())
			if (!module->get_blackbox_attribute())
				for (auto cell : module->cells())
					if (design->module(cell->type) != nullptr && !design->module(cell->type)->get_blackbox_attribute())
						parents[cell->type].push_back({module, cell});

		for (int iter = 0; iter < 10; iter++) {
			bool changed = false;
			for (auto module : design->modules()) {
				if (module->get_blackbox_attribute())
					continue;
				compute(module);
				SigMap sigmap(module);
				auto &bits = ram_bits.at(module->name);
				for (auto w : module->wires())
					if (w->port_output)
						for (int i = 0; i < w->width; i++) {
							SigBit b = sigmap(SigBit(w, i));
							auto key = std::make_pair(w->name, i);
							if (bits.count(b) && (!out_ram[module->name].count(key) || out_ram[module->name].at(key) != bits.at(b))) {
								out_ram[module->name][key] = bits.at(b);
								changed = true;
							}
						}
			}
			// an input bit is RAM data if every instance connects it to RAM data
			for (auto &pit : parents) {
				Module *sub = design->module(pit.first);
				for (auto w : sub->wires()) {
					if (!w->port_input)
						continue;
					for (int i = 0; i < w->width; i++) {
						bool all = true;
						pool<RamId> rams;
						for (auto &pc : pit.second) {
							if (!pc.second->hasPort(w->name) || i >= GetSize(pc.second->getPort(w->name))) { all = false; break; }
							SigMap sm(pc.first);
							SigBit b = sm(pc.second->getPort(w->name)[i]);
							auto &pb = ram_bits[pc.first->name];
							if (!pb.count(b)) { all = false; break; }
							for (auto &r : pb.at(b)) rams.insert(r);
						}
						auto key = std::make_pair(w->name, i);
						if (all && (!in_ram[sub->name].count(key) || in_ram[sub->name].at(key) != rams)) {
							in_ram[sub->name][key] = rams;
							changed = true;
						}
					}
				}
			}
			if (!changed)
				break;
		}

		static const dict<IdString, IdString> noen = {
			{ID(SB_DFFE), ID(SB_DFF)}, {ID(SB_DFFER), ID(SB_DFFR)}, {ID(SB_DFFES), ID(SB_DFFS)},
			{ID(SB_DFFNE), ID(SB_DFFN)}, {ID(SB_DFFNER), ID(SB_DFFNR)}, {ID(SB_DFFNES), ID(SB_DFFNS)},
		};
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			auto &bits = ram_bits[module->name];
			std::vector<Cell*> todo;
			for (auto cell : module->selected_cells()) {
				if (!noen.count(cell->type))
					continue;
				SigBit db = sigmap(cell->getPort(ID::D)).as_bit();
				if (bits.count(db) && used_internally(module, cell->getPort(ID::Q).as_bit()))
					todo.push_back(cell);
			}
			for (auto cell : todo) {
				SigBit q = cell->getPort(ID::Q).as_bit(), dd = cell->getPort(ID::D).as_bit(), e = cell->getPort(ID::E).as_bit();
				SigBit nd = module->addWire(NEW_ID);
				module->addMuxGate(NEW_ID, q, dd, e, nd);
				cell->unsetPort(ID::E);
				cell->setPort(ID::D, nd);
				cell->type = noen.at(cell->type);
			}
			log("%s: %d flip-flops lose their enable.\n", log_id(module), GetSize(todo));
		}
	}
} Ice40LseRamCePass;

struct Ice40LseConstPass : public Pass {
	Ice40LseConstPass() : Pass("ice40_lseconst", "propagate constants across module boundaries, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lseconst\n");
		log("\n");
		log("LSE flattens before optimizing, so constants cross module boundaries. Without\n");
		log("flattening this pass does the same: an input port bit that every instance ties\n");
		log("to the same constant is replaced by that constant inside the module, and an\n");
		log("output port bit driven by a constant is replaced by it in every parent.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSECONST pass.\n");
		auto user = [&](IdString type) {
			Module *m = design->module(type);
			return m != nullptr && !m->get_blackbox_attribute() ? m : nullptr;
		};
		int total = 0;
		for (int iter = 0; iter < 8; iter++) {
			int count = 0;
			dict<IdString, std::vector<std::pair<Module*, Cell*>>> parents;
			for (auto module : design->modules())
				if (!module->get_blackbox_attribute())
					for (auto cell : module->cells())
						if (user(cell->type))
							parents[cell->type].push_back({module, cell});
			// inputs tied to one constant by every instance
			for (auto &pit : parents) {
				Module *sub = design->module(pit.first);
				dict<SigBit, State> ties;
				for (auto w : sub->wires()) {
					if (!w->port_input || w->port_output)
						continue;
					for (int i = 0; i < w->width; i++) {
						bool ok = true;
						State v = State::Sx;
						for (auto &pc : pit.second) {
							if (!pc.second->hasPort(w->name) || i >= GetSize(pc.second->getPort(w->name))) { ok = false; break; }
							SigMap sm(pc.first);
							SigBit b = sm(pc.second->getPort(w->name)[i]);
							if (b.wire || (b.data != State::S0 && b.data != State::S1)) { ok = false; break; }
							if (v != State::Sx && v != b.data) { ok = false; break; }
							v = b.data;
						}
						if (ok && v != State::Sx)
							ties[SigBit(w, i)] = v;
					}
				}
				if (ties.empty())
					continue;
				SigMap sm(sub);
				dict<SigBit, State> mapped;
				for (auto &t : ties)
					mapped[sm(t.first)] = t.second;
				bool any = false;
				for (auto cell : sub->cells())
					for (auto &conn : cell->connections()) {
						if (cell->output(conn.first))
							continue;
						SigSpec sig = conn.second;
						bool changed = false;
						for (int i = 0; i < GetSize(sig); i++) {
							SigBit b = sm(sig[i]);
							if (mapped.count(b)) { sig[i] = mapped.at(b); changed = true; }
						}
						if (changed) { cell->setPort(conn.first, sig); any = true; count++; }
					}
				for (auto w : sub->wires())
					if (w->port_output)
						for (int i = 0; i < w->width; i++) {
							SigBit b = sm(SigBit(w, i));
							if (mapped.count(b) && SigBit(w, i) != b) {
								// an output fed straight from a tied input
								sub->connect(SigBit(w, i), mapped.at(b));
							}
						}
				(void)any;
			}
			// outputs driven by a constant
			for (auto &pit : parents) {
				Module *sub = design->module(pit.first);
				SigMap sm(sub);
				for (auto w : sub->wires()) {
					if (!w->port_output)
						continue;
					for (int i = 0; i < w->width; i++) {
						SigBit b = sm(SigBit(w, i));
						if (b.wire || (b.data != State::S0 && b.data != State::S1))
							continue;
						for (auto &pc : pit.second) {
							if (!pc.second->hasPort(w->name) || i >= GetSize(pc.second->getPort(w->name)))
								continue;
							SigSpec sig = pc.second->getPort(w->name);
							if (!sig[i].wire)
								continue;
							SigBit outer = sig[i];
							sig[i] = pc.first->addWire(NEW_ID);
							pc.second->setPort(w->name, sig);
							pc.first->connect(outer, b.data);
							count++;
						}
					}
				}
			}
			total += count;
			if (count == 0)
				break;
			Pass::call(design, "opt_expr");
			Pass::call(design, "opt_clean");
		}
		log("Propagated %d constant connections across module boundaries.\n", total);
	}
} Ice40LseConstPass;

struct Ice40LsePolarityPass : public Pass {
	Ice40LsePolarityPass() : Pass("ice40_lsepolarity", "choose the output polarity of internal LUTs as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsepolarity [selection]\n");
		log("\n");
		log("ABC maps AND nodes, so a LUT that only feeds other LUTs often comes out\n");
		log("complemented (a mux as NOT mux, re-inverted in the next LUT). LSE maps from\n");
		log("positive gates and keeps such LUTs uninverted. This pass complements SB_LUT4s\n");
		log("whose output only drives SB_LUT4 inputs, absorbing the inversion into the LUTs\n");
		log("they drive, when that makes more LUT inputs positive unate (greedy). Area is\n");
		log("unchanged.\n");

		log("\n");
	}
	// +1 per input the function is positive unate in, -1 per negative unate input
	static int unate_score(const Const &init)
	{
		int score = 0;
		for (int k = 0; k < 4; k++) {
			bool pos = true, neg = true, dep = false;
			for (int i = 0; i < 16; i++) {
				if (i & (1 << k)) continue;
				State lo = init[i], hi = init[i | (1 << k)];
				if (lo != hi) dep = true;
				if (lo == State::S1 && hi == State::S0) pos = false;
				if (lo == State::S0 && hi == State::S1) neg = false;
			}
			if (!dep) continue;
			if (pos) score++;
			else if (neg) score--;
		}
		return score;
	}
	static Const flip_input(const Const &g, int pin)
	{
		Const ng = g;
		for (int i = 0; i < 16; i++)
			ng.set(i, g[i ^ (1 << pin)]);
		return ng;
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEPOLARITY pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, std::vector<std::pair<Cell*, int>>> lut_users;
			pool<SigBit> other_users;
			for (auto w : module->wires())
				if (w->port_output)
					for (auto b : sigmap(w))
						other_users.insert(b);
			for (auto cell : module->cells())
				for (auto &conn : cell->connections()) {
					if (cell->output(conn.first))
						continue;
					int pin = -1;
					if (cell->type == ID(SB_LUT4)) {
						if (conn.first == ID(I0)) pin = 0;
						else if (conn.first == ID(I1)) pin = 1;
						else if (conn.first == ID(I2)) pin = 2;
						else if (conn.first == ID(I3)) pin = 3;
					}
					for (auto b : sigmap(conn.second)) {
						if (!b.wire) continue;
						if (pin >= 0) lut_users[b].push_back({cell, pin});
						else other_users.insert(b);
					}
				}
			for (auto conn : module->connections())
				for (auto b : sigmap(conn.second))
					other_users.insert(b);
			auto init_of = [](Cell *c) { Const v = c->getParam(ID(LUT_INIT)); v.extu(16); return v; };
			int count = 0;
			for (int pass = 0; pass < 4; pass++) {
				int flips = 0;
				for (auto cell : module->selected_cells()) {
					if (cell->type != ID(SB_LUT4))
						continue;
					SigBit o = sigmap(cell->getPort(ID::O).as_bit());
					if (other_users.count(o) || !lut_users.count(o))
						continue;
					Const init = init_of(cell), ninit = init;
					for (int i = 0; i < 16; i++)
						ninit.set(i, init[i] == State::S1 ? State::S0 : State::S1);
					int before = unate_score(init), after = unate_score(ninit);
					dict<Cell*, Const> fan;
					for (auto &u : lut_users.at(o))
						fan[u.first] = fan.count(u.first) ? fan.at(u.first) : init_of(u.first);
					for (auto &f : fan) before += unate_score(f.second);
					for (auto &u : lut_users.at(o))
						fan[u.first] = flip_input(fan.at(u.first), u.second);
					for (auto &f : fan) after += unate_score(f.second);
					bool flip = after > before;
					if (!flip)
						continue;
					cell->setParam(ID(LUT_INIT), ninit);
					for (auto &u : lut_users.at(o))
						u.first->setParam(ID(LUT_INIT), flip_input(init_of(u.first), u.second));
					flips++;
				}
				count += flips;
				if (flips == 0)
					break;
			}
			log("%s: complemented %d LUTs.\n", log_id(module), count);
		}
	}
} Ice40LsePolarityPass;

struct Ice40LseCmpPass : public Pass {
	Ice40LseCmpPass() : Pass("ice40_lsecmp", "build comparisons of two signals as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsecmp [selection]\n");
		log("\n");
		log("Replaces $eq/$ne and unsigned $lt/$le/$gt/$ge of two non-constant operands (at\n");
		log("least 3 bits) by the SB_LUT4 structures LSE builds:\n");
		log("\n");
		log("    - equality: one LUT per bit pair that is 1 when the pair differs, OR4\n");
		log("      trees over those, and a final NOR ($eq) or OR ($ne)\n");
		log("    - magnitude: a ripple chain from the low bits, the first LUT taking two\n");
		log("      bits and every other one bit and the chain\n");
		log("\n");
	}
	Module *module;
	int count_luts;

	// a LUT computing f over up to 4 inputs
	SigBit lut(std::vector<SigBit> in, std::function<bool(const std::vector<bool>&)> f)
	{
		int n = GetSize(in);
		Const init(State::S0, 16);
		for (int i = 0; i < 16; i++) {
			std::vector<bool> v;
			for (int k = 0; k < n; k++)
				v.push_back((i >> k) & 1);
			init.set(i, f(v) ? State::S1 : State::S0);
		}
		SigBit y = module->addWire(NEW_ID);
		Cell *c = module->addCell(NEW_ID, ID(SB_LUT4));
		c->setParam(ID(LUT_INIT), init);
		IdString pins[4] = {ID(I0), ID(I1), ID(I2), ID(I3)};
		for (int k = 0; k < 4; k++)
			c->setPort(pins[k], k < n ? SigSpec(in[k]) : SigSpec(State::S0));
		c->setPort(ID::O, y);
		c->set_bool_attribute(ID(lse_fixed));
		count_luts++;
		return y;
	}

	SigBit equality(SigSpec a, SigSpec b, bool ne)
	{
		std::vector<SigBit> diff;
		int n = GetSize(a), i = 0;
		for (; i + 1 < n; i += 2)
			diff.push_back(lut({a[i], a[i+1], b[i], b[i+1]}, [](const std::vector<bool> &v) {
				return (v[0] != v[2]) || (v[1] != v[3]);
			}));
		if (i < n)
			diff.push_back(lut({a[i], b[i]}, [](const std::vector<bool> &v) { return v[0] != v[1]; }));
		while (GetSize(diff) > 4) {
			std::vector<SigBit> next;
			for (int k = 0; k < GetSize(diff); k += 4) {
				std::vector<SigBit> grp(diff.begin() + k, diff.begin() + std::min(k + 4, GetSize(diff)));
				if (GetSize(grp) == 1)
					next.push_back(grp[0]);
				else
					next.push_back(lut(grp, [](const std::vector<bool> &v) {
						for (bool x : v) if (x) return true;
						return false;
					}));
			}
			diff = next;
		}
		// the last stage stays plain logic: LSE merges it into whatever reads it
		SigBit any = module->ReduceOr(NEW_ID, diff);
		return ne ? any : module->LogicNot(NEW_ID, any).as_bit();
	}

	// a < b (or a <= b with or_equal), unsigned, rippled from bit 0
	SigBit less(SigSpec a, SigSpec b, bool or_equal)
	{
		int n = GetSize(a);
		bool init = or_equal;
		SigBit c = lut({b[0], b[1], a[1], a[0]}, [init](const std::vector<bool> &v) {
			bool b0 = v[0], b1 = v[1], a1 = v[2], a0 = v[3];
			bool l0 = (!a0 && b0) || (a0 == b0 && init);
			return (!a1 && b1) || (a1 == b1 && l0);
		});
		for (int i = 2; i < n; i++)
			c = lut({c, b[i], a[i]}, [](const std::vector<bool> &v) {
				return (!v[2] && v[1]) || (v[2] == v[1] && v[0]);
			});
		return c;
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSECMP pass.\n");
		for (auto m : design->selected_modules()) {
			module = m;
			count_luts = 0;
			SigMap sigmap(module);
			std::vector<Cell*> todo;
			for (auto cell : module->selected_cells())
				if (cell->type.in(ID($eq), ID($ne), ID($lt), ID($le), ID($gt), ID($ge)))
					todo.push_back(cell);
			// an operand that is a constant plus a small signal is mostly constant: LSE
			// simplifies those, leave them to ABC
			pool<SigBit> const_offset;
			for (auto cell : module->cells())
				if (cell->type.in(ID($add), ID($sub)) &&
						(cell->getPort(ID::A).is_fully_const() || cell->getPort(ID::B).is_fully_const() ||
						 GetSize(sigmap(cell->getPort(ID::A)).to_sigbit_pool()) <= 2 || GetSize(sigmap(cell->getPort(ID::B)).to_sigbit_pool()) <= 2))
					for (auto bit : sigmap(cell->getPort(ID::Y)))
						const_offset.insert(bit);
			int count = 0;
			for (auto cell : todo) {
				bool is_signed = cell->getParam(ID::A_SIGNED).as_bool() && cell->getParam(ID::B_SIGNED).as_bool();
				bool eq = cell->type.in(ID($eq), ID($ne));
				if (is_signed && !eq)
					continue;
				int w = std::max(GetSize(cell->getPort(ID::A)), GetSize(cell->getPort(ID::B)));
				SigSpec a = sigmap(cell->getPort(ID::A)), b = sigmap(cell->getPort(ID::B));
				if (w < 3 || a.has_const() || b.has_const())
					continue;
				bool offset = false;
				for (auto bit : a) if (const_offset.count(bit)) offset = true;
				for (auto bit : b) if (const_offset.count(bit)) offset = true;
				if (offset)
					continue;
				a.extend_u0(w, is_signed);
				b.extend_u0(w, is_signed);
				SigBit r;
				if (eq)
					r = equality(a, b, cell->type == ID($ne));
				else if (cell->type == ID($lt)) r = less(a, b, false);
				else if (cell->type == ID($le)) r = less(a, b, true);
				else if (cell->type == ID($gt)) r = less(b, a, false);
				else r = less(b, a, true);
				SigSpec y = cell->getPort(ID::Y);
				SigSpec ry(r);
				ry.extend_u0(GetSize(y));
				module->connect(y, ry);
				module->remove(cell);
				count++;
			}
			log("%s: %d comparisons built as %d LUTs.\n", log_id(module), count, count_luts);
		}
	}
} Ice40LseCmpPass;

PRIVATE_NAMESPACE_END
