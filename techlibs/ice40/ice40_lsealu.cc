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
#include "kernel/ff.h"
#include "kernel/ffinit.h"
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
		for (auto module : design->selected_modules())
		for (int round = 0, count = 1; round < 4 && count > 0; round++) {
			// repeated: once `if (en) q <= q + k' is folded, an outer condition (a case
			// arm, another if) around it is folded too, as LSE does
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
			// registers that address a memory: LSE folds their enable (and reset) into logic
			pool<SigBit> mem_addr;
			for (auto cell : module->cells())
				if (cell->type.in(ID($memrd), ID($memrd_v2), ID($memwr), ID($memwr_v2)))
					for (auto bit : sigmap(cell->getPort(ID::ADDR)))
						mem_addr.insert(bit);

			for (auto cell : module->cells())
				if (cell->type.in(ID($dff), ID($adff), ID($dffe), ID($adffe))) {
					SigSpec d = sigmap(cell->getPort(ID::D)), q = sigmap(cell->getPort(ID::Q));
					ff_q_of_d[d] = q;
					bool addr = false;
					for (auto bit : q)
						if (mem_addr.count(bit))
							addr = true;
					if (addr)
						behind_const.insert(d);
					for (int depth = 0; depth < 4 && mux_of_y.count(d); depth++) {
						Cell *m = mux_of_y.at(d);
						SigSpec ma = sigmap(m->getPort(ID::A)), mb = sigmap(m->getPort(ID::B));
						if (ma.is_fully_const()) d = mb;
						else if (mb.is_fully_const()) d = ma;
						else break;
						ff_q_of_d[d] = q;
						// a counter with a sync or async clear keeps its enable
						behind_const.insert(d);
					}
					// with an async reset, LSE also keeps the enable when the add is one of
					// several next-state choices (a load in front of it)
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
								if (load)
									behind_const.insert(in);
								todo.push_back(in);
							}
					}
				}
			count = 0;
			std::vector<Cell*> pmuxes;
			for (auto mux : module->selected_cells()) {
				if (mux->type == ID($pmux))
					pmuxes.push_back(mux);
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
					// trailing zero (2, 6, 10, 14, ...)
					if (sigmap(k).is_fully_const() && GetSize(k) >= 2 && k[0] == State::S0 && k[1] == State::S1)
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
			// a case arm: default Q, one arm Q + K  ->  default Q + (arm ? K : 0)
			for (auto pm : pmuxes) {
				SigSpec py = sigmap(pm->getPort(ID::Y));
				if (behind_const.count(py))
					continue;
				SigSpec q = sigmap(pm->getPort(ID::A));
				if (ff_q_of_d.count(py) && q != ff_q_of_d.at(py))
					continue;
				SigSpec pb = pm->getPort(ID::B), ps = pm->getPort(ID::S);
				int w = GetSize(q), n = GetSize(ps);
				for (int i = 0; i < n; i++) {
					SigSpec sum = sigmap(pb.extract(i * w, w));
					if (!add_out.count(sum[0]))
						continue;
					Cell *add = add_out.at(sum[0]).first;
					SigSpec y = sigmap(add->getPort(ID::Y));
					if (GetSize(sum) > GetSize(y) || y.extract(0, GetSize(sum)) != sum)
						continue;
					SigSpec opA = sigmap(add->getPort(ID::A)), opB = sigmap(add->getPort(ID::B));
					auto same = [&](const SigSpec &op) {
						int m = std::min(GetSize(op), GetSize(q));
						if (op.extract(0, m) != q.extract(0, m))
							return false;
						for (int j = m; j < GetSize(op); j++)
							if (op[j] != State::S0) return false;
						return m == GetSize(q);
					};
					IdString kport;
					if (same(opA)) kport = ID::B;
					else if (same(opB) && add->type == ID($add)) kport = ID::A;
					else continue;
					SigSpec k = add->getPort(kport);
					if (sigmap(k).is_fully_const() && GetSize(k) >= 2 && k[0] == State::S0 && k[1] == State::S1)
						continue;
					SigSpec gated = module->Mux(NEW_ID, SigSpec(State::S0, GetSize(k)), k, ps[i]);
					Cell *nadd = module->addCell(NEW_ID, add->type);
					nadd->parameters = add->parameters;
					nadd->setPort(ID::A, kport == ID::B ? add->getPort(ID::A) : gated);
					nadd->setPort(ID::B, kport == ID::B ? gated : add->getPort(ID::B));
					Wire *ny = module->addWire(NEW_ID, GetSize(y));
					nadd->setPort(ID::Y, ny);
					SigSpec nb, ns;
					for (int j = 0; j < n; j++)
						if (j != i) {
							nb.append(pb.extract(j * w, w));
							ns.append(ps[j]);
						}
					pm->setPort(ID::A, SigSpec(ny).extract(0, w));
					if (GetSize(ns) == 0) {
						module->connect(pm->getPort(ID::Y), SigSpec(ny).extract(0, w));
						pm->setPort(ID::Y, module->addWire(NEW_ID, w));
						pm->setPort(ID::B, SigSpec(State::Sx, w));
						pm->setPort(ID::S, State::S0);
						pm->setParam(ID::S_WIDTH, 1);
					} else {
						pm->setPort(ID::B, nb);
						pm->setPort(ID::S, ns);
						pm->setParam(ID::S_WIDTH, GetSize(ns));
					}
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
			// in a syn_hier="hard" module, a flip-flop with one load value made from at most
			// two module inputs by plain logic (no flip-flop or RAM in between, so the hold
			// mux fits a LUT4 with Q and the enable) gets no enable either; a second load
			// value shows up as signals shared by D and the enable. Not at the top: synthesized
			// on its own, such a module keeps these enables
			bool hard = module->get_string_attribute(ID(syn_hier)) == "hard" && !module->get_bool_attribute(ID::top);
			dict<SigBit, Cell*> gate_of;
			pool<SigBit> inputs;
			if (hard) {
				for (auto c : module->cells())
					if (c->type.begins_with("$_") && c->hasPort(ID::Y) && GetSize(c->getPort(ID::Y)) == 1)
						gate_of[sigmap(c->getPort(ID::Y).as_bit())] = c;
				for (auto w : module->wires())
					if (w->port_input)
						for (auto b : sigmap(SigSpec(w)))
							inputs.insert(b);
			}
			// the leaves of a gate cone; false if it reaches anything but inputs and constants
			auto leaves = [&](SigBit start, pool<SigBit> &out) {
				std::vector<SigBit> stack = {sigmap(start)};
				pool<SigBit> seen;
				while (!stack.empty()) {
					SigBit b = stack.back();
					stack.pop_back();
					if (!b.wire || seen.count(b))
						continue;
					seen.insert(b);
					out.insert(b);
					if (gate_of.count(b)) {
						for (auto &conn : gate_of.at(b)->connections())
							if (conn.first != ID::Y)
								for (auto x : sigmap(conn.second))
									stack.push_back(x);
					} else if (!inputs.count(b))
						return false;
				}
				return true;
			};
			// decided per register (Q wire and enable net), as LSE sees it: one flip-flop
			// that needs the enable keeps it for all
			typedef std::pair<Wire*, SigBit> RegKey;
			dict<RegKey, std::vector<Cell*>> hard_group;
			pool<RegKey> hard_keep;
			for (auto cell : module->selected_cells()) {
				if (!noen.count(cell->type))
					continue;
				SigBit db = sigmap(cell->getPort(ID::D)).as_bit();
				if (bits.count(db) && used_internally(module, cell->getPort(ID::Q).as_bit())) {
					todo.push_back(cell);
					continue;
				}
				if (!hard)
					continue;
				SigBit en = sigmap(cell->getPort(ID::E).as_bit());
				RegKey key(cell->getPort(ID::Q).as_bit().wire, en);
				hard_group[key].push_back(cell);
				pool<SigBit> dl, el;
				bool ok = leaves(db, dl);
				// the hold mux must fit one LUT4 with Q and the enable
				int nin = 0;
				for (auto b : dl)
					if (!gate_of.count(b))
						nin++;
				if (ok && nin <= 2) {
					leaves(en, el);
					for (auto b : dl)
						if (el.count(b))
							ok = false;
				}
				if (!ok || nin > 2)
					hard_keep.insert(key);
			}
			for (auto &it : hard_group)
				if (!hard_keep.count(it.first))
					for (auto cell : it.second)
						todo.push_back(cell);
			for (auto cell : todo) {
				SigBit q = cell->getPort(ID::Q).as_bit(), dd = cell->getPort(ID::D).as_bit(), e = cell->getPort(ID::E).as_bit();
				SigBit nd = module->addWire(NEW_ID);
				module->addMuxGate(NEW_ID, q, dd, e, nd);
				cell->unsetPort(ID::E);
				cell->setPort(ID::D, nd);
				cell->type = noen.at(cell->type);
			}
			log("%s: %d flip-flops lose their enable.\n", log_id(module), GetSize(todo));

			// a register that addresses a RAM gets neither a clock enable nor a synchronous
			// reset/set pin: LSE builds both into the logic in front of it
			pool<SigBit> addr_bits;
			// (EBR only: LSE keeps the enable of an SPRAM address register)
			for (auto cell : module->cells())
				if (is_ram(cell) && cell->type != ID(SB_SPRAM256KA))
					for (auto port : {ID(RADDR), ID(WADDR)})
						if (cell->hasPort(port))
							for (auto b : sigmap(cell->getPort(port)))
								addr_bits.insert(b);
			// a register whose next value is the RAM address itself (a read address merged
			// into the RAM: addr = rst ? 0 : en ? next : q) too: LSE gives both one net
			{
				dict<SigBit, Cell*> gate_drv;
				for (auto c : module->cells())
					if (c->type.in(ID($_MUX_), ID($_AND_), ID($_ANDNOT_), ID($_OR_)))
						gate_drv[sigmap(c->getPort(ID::Y).as_bit())] = c;
				pool<SigBit> extra;
				for (auto b : addr_bits) {
					std::vector<std::pair<SigBit, int>> stack = {{b, 0}};
					while (!stack.empty()) {
						auto [x, depth] = stack.back();
						stack.pop_back();
						if (!gate_drv.count(x) || depth >= 2)
							continue;
						Cell *g = gate_drv.at(x);
						for (auto port : {ID::A, ID::B}) {
							SigBit y = sigmap(g->getPort(port).as_bit());
							extra.insert(y);
							stack.push_back({y, depth + 1});
						}
					}
				}
				for (auto y : extra)
					addr_bits.insert(y);
			}
			int nsr = 0;
			std::vector<Cell*> srs;
			dict<SigBit, Cell*> or_of;
			for (auto c : module->cells())
				if (c->type == ID($_OR_))
					or_of[sigmap(c->getPort(ID::Y).as_bit())] = c;
			for (auto cell : module->selected_cells())
				if (cell->type.in(ID(SB_DFFE), ID(SB_DFFSR), ID(SB_DFFSS), ID(SB_DFFESR), ID(SB_DFFESS)) &&
						addr_bits.count(sigmap(cell->getPort(ID::Q).as_bit())))
					srs.push_back(cell);
			for (auto cell : srs) {
				bool set = cell->type.in(ID(SB_DFFSS), ID(SB_DFFESS)), sync = cell->type != ID(SB_DFFE);
				SigBit q = cell->getPort(ID::Q).as_bit(), nd = cell->getPort(ID::D).as_bit();
				SigBit r = sync ? cell->getPort(set ? ID::S : ID::R).as_bit() : SigBit();
				if (cell->hasPort(ID::E)) {
					// a reset that wins over the enable shows up as E = en | R; build
					// R ? 0 : (en ? D : Q) as the RTL says, not with the merged enable
					SigBit e = cell->getPort(ID::E).as_bit(), en = e;
					if (sync && or_of.count(sigmap(e))) {
						Cell *g = or_of.at(sigmap(e));
						SigBit ga = sigmap(g->getPort(ID::A).as_bit()), gb = sigmap(g->getPort(ID::B).as_bit());
						if (ga == sigmap(r)) en = g->getPort(ID::B).as_bit();
						else if (gb == sigmap(r)) en = g->getPort(ID::A).as_bit();
					}
					nd = module->MuxGate(NEW_ID, q, nd, en);
				}
				if (sync) {
					nd = set ? module->OrGate(NEW_ID, nd, r) : module->AndnotGate(NEW_ID, nd, r);
					cell->unsetPort(set ? ID::S : ID::R);
				}
				if (cell->hasPort(ID::E))
					cell->unsetPort(ID::E);
				cell->setPort(ID::D, nd);
				cell->type = ID(SB_DFF);
				nsr++;
			}
			if (nsr)
				log("%s: %d RAM address flip-flops lose their enable/synchronous reset.\n", log_id(module), nsr);
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
			// a kept net (syn_keep) keeps its value, as do ports
			for (auto w : module->wires())
				if (w->port_output || w->get_bool_attribute(ID::keep))
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
			auto init_of = [](Cell *c) { Const v = c->getParam(ID(LUT_INIT)); v.extu(16); return v; };
			// the sum LUT of a carry chain keeps its polarity (LSE never inverts an adder's sum)
			pool<SigBit> carry_nets;
			for (auto cell : module->cells())
				if (cell->type == ID(SB_CARRY))
					for (auto port : {ID(CI), ID(CO)})
						for (auto bb : sigmap(cell->getPort(port)))
							if (bb.wire) carry_nets.insert(bb);
			int count = 0;
			for (int pass = 0; pass < 4; pass++) {
				int flips = 0;
				for (auto cell : module->selected_cells()) {
					if (cell->type != ID(SB_LUT4))
						continue;
					SigBit o = sigmap(cell->getPort(ID::O).as_bit());
					if (other_users.count(o) || !lut_users.count(o))
						continue;
					if (carry_nets.count(sigmap(cell->getPort(ID(I3)).as_bit())))
						continue;
					bool feeds_sum = false;
					for (auto &u : lut_users.at(o))
						if (carry_nets.count(sigmap(u.first->getPort(ID(I3)).as_bit())))
							feeds_sum = true;
					if (feeds_sum)
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
	SigBit less(SigSpec a, SigSpec b, bool or_equal, bool plain = false)
	{
		if (plain)
			return (or_equal ? module->Le(NEW_ID, a, b) : module->Lt(NEW_ID, a, b)).as_bit();
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
			// an operand that is a constant plus one or two signal bits is mostly
			// constant: LSE simplifies those, leave them to ABC
			// (followed through chains of adds, e.g. (524 + line_mode) - 3)
			// an operand that depends on only one or two signal bits (a constant plus a
			// bit or two, or a small case table) is mostly constant: LSE simplifies those,
			// leave them to ABC (followed through combinational logic, e.g. (524 +
			// line_mode) - 3)
			dict<SigBit, Cell*> driver;
			for (auto cell : module->cells())
				if (cell->type.begins_with("$") && !RTLIL::builtin_ff_cell_types().count(cell->type) &&
						!cell->type.begins_with("$mem") && !cell->type.in(ID($eq), ID($ne), ID($lt), ID($le), ID($gt), ID($ge)))
					for (auto &conn : cell->connections())
						if (cell->output(conn.first))
							for (auto bit : sigmap(conn.second))
								driver[bit] = cell;
			std::function<void(SigBit, pool<SigBit>&, pool<Cell*>&)> support = [&](SigBit bit, pool<SigBit> &vars, pool<Cell*> &seen) {
				if (!bit.wire || GetSize(vars) > 2)
					return;
				if (!driver.count(bit)) {
					vars.insert(bit);
					return;
				}
				Cell *c = driver.at(bit);
				if (seen.count(c))
					return;
				seen.insert(c);
				for (auto &conn : c->connections())
					if (c->input(conn.first))
						for (auto b : sigmap(conn.second))
							support(b, vars, seen);
			};
			auto few_vars = [&](const SigSpec &sig) {
				pool<SigBit> vars;
				pool<Cell*> seen;
				for (auto bit : sig)
					support(bit, vars, seen);
				return GetSize(vars) <= 2;
			};
			// any bit from an add/sub of one or two signal bits counts too
			auto offset_bit = [&](const SigSpec &sig) {
				for (auto bit : sig)
					if (driver.count(bit) && driver.at(bit)->type.in(ID($add), ID($sub))) {
						Cell *c = driver.at(bit);
						if (few_vars(SigSpec({sigmap(c->getPort(ID::A)), sigmap(c->getPort(ID::B))})))
							return true;
					}
				return false;
			};
			int count = 0;
			for (auto cell : todo) {
				bool is_signed = cell->getParam(ID::A_SIGNED).as_bool() && cell->getParam(ID::B_SIGNED).as_bool();
				bool eq = cell->type.in(ID($eq), ID($ne));
				if (is_signed && !eq)
					continue;
				int w = std::max(GetSize(cell->getPort(ID::A)), GetSize(cell->getPort(ID::B)));
				SigSpec a = sigmap(cell->getPort(ID::A)), b = sigmap(cell->getPort(ID::B));
				// zero-extension of one operand is fine (handled below); other constant bits
				// are left to cmp2lut/ABC
				SigSpec ac = a, bc = b;
				ac.extend_u0(w, is_signed); bc.extend_u0(w, is_signed);
				auto lead0 = [&](const SigSpec &sig) { int n = 0; while (n < GetSize(sig) && sig[GetSize(sig)-1-n] == State::S0) n++; return n; };
				auto other_const = [&](const SigSpec &sig) {
					int n = lead0(sig);
					for (int i = 0; i < GetSize(sig) - n; i++)
						if (!sig[i].wire) return true;
					return false;
				};
				if (w < 3 || other_const(ac) || other_const(bc) || lead0(ac) > w - 2 || lead0(bc) > w - 2)
					continue;
				if (offset_bit(a) || offset_bit(b) || (eq && (few_vars(a) || few_vars(b))))
					continue;
				// against a nearly constant operand only the high-bit override is built
				// here, the rest is left to ABC
				bool plain = !eq && (few_vars(a) || few_vars(b));
				a.extend_u0(w, is_signed);
				b.extend_u0(w, is_signed);
				SigBit r;
				if (eq)
					r = equality(a, b, cell->type == ID($ne));
				else {
					SigSpec x = a, y = b;      // x < y (or x <= y)
					bool or_equal = cell->type.in(ID($le), ID($ge));
					if (cell->type.in(ID($gt), ID($ge)))
						std::swap(x, y);
					// LSE ripples only over the bits both operands have; the other
					// operand's extra high bits are ORed and override the result (which
					// becomes a flip-flop's sync reset/set)
					int k = 0;
					while (k < w - 2 && y[w-1-k] == State::S0 && x[w-1-k].wire) k++;
					int kx = 0;
					while (kx < w - 2 && x[w-1-kx] == State::S0 && y[w-1-kx].wire) kx++;
					if (k > 0) {
						SigBit hi = module->ReduceOr(NEW_ID, x.extract(w - k, k));
						SigBit lo = less(x.extract(0, w - k), y.extract(0, w - k), or_equal, plain);
						r = module->Mux(NEW_ID, lo, State::S0, hi).as_bit();
					} else if (kx > 0) {
						SigBit hi = module->ReduceOr(NEW_ID, y.extract(w - kx, kx));
						SigBit lo = less(x.extract(0, w - kx), y.extract(0, w - kx), or_equal, plain);
						r = module->Mux(NEW_ID, lo, State::S1, hi).as_bit();
					} else if (plain)
						continue;
					else
						r = less(x, y, or_equal);
				}
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

struct Ice40LseMuxChainPass : public Pass {
	Ice40LseMuxChainPass() : Pass("ice40_lsemuxchain", "build priority mux chains as 4:1 muxes, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsemuxchain [selection]\n");
		log("\n");
		log("LSE builds a priority chain  s0 ? a0 : s1 ? a1 : s2 ? a2 : a3  that several bits\n");
		log("share (an if/else-if on a bus) as a 4:1 multiplexer with encoded selects\n");
		log("T = !(s0 | s1) and S = T ? !s2 : !s0. This pass rewrites such $_MUX_ chains\n");
		log("into $_MUX4_ cells, for the LSE 4:1 mapping in two LUT4s.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEMUXCHAIN pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, Cell*> mux_of;
			dict<SigBit, int> fanout;
			for (auto cell : module->cells()) {
				if (cell->type == ID($_MUX_))
					mux_of[sigmap(cell->getPort(ID::Y).as_bit())] = cell;
				for (auto &conn : cell->connections())
					if (!cell->output(conn.first))
						for (auto b : sigmap(conn.second))
							fanout[b]++;
			}
			for (auto w : module->wires())
				if (w->port_output)
					for (auto b : sigmap(w))
						fanout[b]++;
			// root mux -> (m1, m2) with each next link single-fanout on the A side
			struct Chain { Cell *m0, *m1, *m2; IdString n0, n1, n2; };
			dict<std::tuple<SigBit, SigBit, SigBit>, std::vector<Chain>> groups;
			pool<Cell*> inner;
			for (auto cell : module->selected_cells()) {
				if (cell->type != ID($_MUX_))
					continue;
				SigBit a = sigmap(cell->getPort(ID::A).as_bit());
				if (!mux_of.count(a) || fanout[a] != 1)
					continue;
				Cell *m1 = mux_of.at(a);
				SigBit a1 = sigmap(m1->getPort(ID::A).as_bit());
				if (!mux_of.count(a1) || fanout[a1] != 1)
					continue;
				Cell *m2 = mux_of.at(a1);
				inner.insert(m1);
				inner.insert(m2);
				// exactly three levels: a longer chain LSE builds differently
				if (mux_of.count(sigmap(m2->getPort(ID::A).as_bit())))
					continue;
				groups[std::make_tuple(sigmap(cell->getPort(ID::S).as_bit()), sigmap(m1->getPort(ID::S).as_bit()),
						sigmap(m2->getPort(ID::S).as_bit()))].push_back({cell, m1, m2, cell->name, m1->name, m2->name});
			}
			int count = 0;
			pool<Cell*> used;
			for (auto &g : groups) {
				std::vector<Chain> chains;
				for (auto &c : g.second)
					// only the outermost three of a longer chain, each mux in one chain
					if (!inner.count(c.m0) && !used.count(c.m0) && !used.count(c.m1) && !used.count(c.m2) &&
							c.m0 != c.m1 && c.m1 != c.m2 && c.m0 != c.m2) {
						chains.push_back(c);
						used.insert(c.m0);
						used.insert(c.m1);
						used.insert(c.m2);
					}
				if (GetSize(chains) < 2)
					continue;
				SigBit s0 = std::get<0>(g.first), s1 = std::get<1>(g.first), s2 = std::get<2>(g.first);
				// selects that are bits of one index (a decoded $bmux/$shiftx) make a
				// binary tree, not a priority chain
				if ((s0.wire && (s0.wire == s1.wire || s0.wire == s2.wire)) || (s1.wire && s1.wire == s2.wire))
					continue;
				log("  %d bits like %s, selects %s %s %s\n", GetSize(chains), log_signal(chains[0].m0->getPort(ID::Y)),
						log_signal(s0), log_signal(s1), log_signal(s2));
				SigBit t = module->NorGate(NEW_ID, s0, s1);
				SigBit sel = module->MuxGate(NEW_ID, module->NotGate(NEW_ID, s0), module->NotGate(NEW_ID, s2), t);
				for (auto &c : chains) {
					if (!module->cell(c.n0) || !module->cell(c.n1) || !module->cell(c.n2))
						continue;
					Cell *m4 = module->addCell(NEW_ID, ID($_MUX4_));
					m4->setPort(ID::A, c.m0->getPort(ID::B));
					m4->setPort(ID::B, c.m1->getPort(ID::B));
					m4->setPort(ID::C, c.m2->getPort(ID::B));
					m4->setPort(ID::D, c.m2->getPort(ID::A));
					m4->setPort(ID::S, sel);
					m4->setPort(ID::T, t);
					m4->setPort(ID::Y, c.m0->getPort(ID::Y));
					module->remove(c.m0);
					module->remove(c.m1);
					module->remove(c.m2);
					count++;
				}
			}
			log("%s: %d priority chains built as 4:1 muxes.\n", log_id(module), count);
		}
	}
} Ice40LseMuxChainPass;

struct Ice40LseMuxSelPass : public Pass {
	Ice40LseMuxSelPass() : Pass("ice40_lsemuxsel", "split a shared mux select into the muxes, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsemuxsel [selection]\n");
		log("\n");
		log("When an SB_LUT4 S with three or four inputs is only the select of several 2:1\n");
		log("multiplexer LUTs and S = x op R for one of its inputs x (op an AND or OR, with\n");
		log("either polarity), LSE builds R as the shared LUT and puts the gate with x into\n");
		log("every multiplexer LUT. This pass does the same; the LUT count is unchanged.\n");
		log("A select LUT of only two inputs goes whole into the multiplexer LUTs.\n");
		log("An x driven by a flip-flop is preferred.\n");
		log("\n");
	}
	static bool bit(int init, int idx) { return (init >> idx) & 1; }
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEMUXSEL pass.\n");
		IdString pins[4] = {ID(I0), ID(I1), ID(I2), ID(I3)};
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, std::vector<std::pair<Cell*, IdString>>> users;
			dict<SigBit, Cell*> driver;
			pool<SigBit> port_bits;
			for (auto w : module->wires())
				if (w->port_output)
					for (auto b : sigmap(w)) port_bits.insert(b);
			for (auto cell : module->cells())
				for (auto &conn : cell->connections()) {
					for (auto b : sigmap(conn.second)) {
						if (!b.wire) continue;
						if (cell->output(conn.first)) driver[b] = cell;
						else users[b].push_back({cell, conn.first});
					}
				}
			auto init_of = [](Cell *c) { return c->getParam(ID(LUT_INIT)).as_int() & 0xffff; };
			// pin index of the select if the LUT is a 2:1 mux on three live inputs
			auto mux_select = [&](Cell *c, int &sel, int &lo, int &hi) {
				int init = init_of(c);
				std::vector<int> live;
				int mask = 0;
				for (int i = 0; i < 4; i++) {
					SigBit pb = sigmap(c->getPort(pins[i])).as_bit();
					if (pb.wire) { live.push_back(i); mask |= 1 << i; }
					else if (pb.data != State::S0) return false;
				}
				if (GetSize(live) != 3) return false;
				for (int si = 0; si < 3; si++) {
					int s = live[si], a = live[(si+1)%3], b = live[(si+2)%3];
					for (int swap = 0; swap < 2; swap++) {
						int x0 = swap ? b : a, x1 = swap ? a : b;
						bool ok = true;
						for (int idx = 0; idx < 16 && ok; idx++) {
							if (idx & ~mask) continue;   // unused inputs are tied to 0
							bool want = ((idx >> s) & 1) ? ((idx >> x1) & 1) : ((idx >> x0) & 1);
							if (bit(init, idx) != want) ok = false;
						}
						if (ok) { sel = s; lo = x0; hi = x1; return true; }
					}
				}
				return false;
			};
			int count = 0;
			std::vector<Cell*> luts;
			for (auto cell : module->selected_cells())
				if (cell->type == ID(SB_LUT4) && !cell->get_bool_attribute(ID(lse_fixed)))
					luts.push_back(cell);
			// nets newly wired into a multiplexer: their users are not in the map
			pool<SigBit> rewired;
			for (auto S : luts) {
				SigBit so = sigmap(S->getPort(ID::O).as_bit());
				if (port_bits.count(so) || !users.count(so) || rewired.count(so))
					continue;
				std::vector<std::tuple<Cell*, int, int, int>> muxes;
				bool only_selects = true;
				for (auto &u : users.at(so)) {
					int sel, lo, hi;
					if (u.first->type != ID(SB_LUT4) || u.first == S || !mux_select(u.first, sel, lo, hi) ||
							u.second != pins[sel]) { only_selects = false; break; }
					muxes.push_back(std::make_tuple(u.first, sel, lo, hi));
				}
				if (!only_selects || muxes.empty())
					continue;
				std::vector<int> live;
				int smask = 0;
				bool tied1 = false;
				for (int i = 0; i < 4; i++) {
					SigBit pb = sigmap(S->getPort(pins[i])).as_bit();
					if (pb.wire) { live.push_back(i); smask |= 1 << i; }
					else if (pb.data != State::S0) tied1 = true;
				}
				if (tied1)
					continue;
				int sinit = init_of(S);
				// a select of two signals goes whole into every multiplexer LUT
				if (GetSize(live) == 2) {
					SigBit s0 = S->getPort(pins[live[0]]).as_bit(), s1 = S->getPort(pins[live[1]]).as_bit();
					rewired.insert(sigmap(s0));
					rewired.insert(sigmap(s1));
					for (auto &m : muxes) {
						Cell *M = std::get<0>(m);
						int sel = std::get<1>(m), lo = std::get<2>(m), hi = std::get<3>(m);
						int free = -1;
						for (int i = 0; i < 4; i++)
							if (i != sel && i != lo && i != hi) free = i;
						M->setPort(pins[sel], s0);
						M->setPort(pins[free], s1);
						Const ninit(State::S0, 16);
						for (int idx = 0; idx < 16; idx++) {
							int sidx = (((idx >> sel) & 1) << live[0]) | (((idx >> free) & 1) << live[1]);
							int v = bit(sinit, sidx) ? ((idx >> hi) & 1) : ((idx >> lo) & 1);
							ninit.set(idx, v ? State::S1 : State::S0);
						}
						M->setParam(ID(LUT_INIT), ninit);
					}
					module->remove(S);
					count++;
					continue;
				}
				if (GetSize(live) < 3 || GetSize(muxes) < 2)
					continue;
				// find x and op with S = op(x ^ px, R)
				int best_x = -1, best_op = -1, best_px = 0, best_score = -1;
				for (int x : live)
					for (int op = 0; op < 2; op++)        // 0: AND, 1: OR
						for (int px = 0; px < 2; px++) {
							// with x at the "dominating" value the output is constant
							int dom = op == 0 ? px : !px;  // x value that forces the result
							bool ok = true;
							for (int idx = 0; idx < 16 && ok; idx++)
								if (!(idx & ~smask) && ((idx >> x) & 1) == dom && bit(sinit, idx) != (op == 1))
									ok = false;
							if (!ok) continue;
							Cell *d = driver.count(sigmap(S->getPort(pins[x]).as_bit())) ? driver.at(sigmap(S->getPort(pins[x]).as_bit())) : nullptr;
							int score = (d && d->type.begins_with("\\SB_DFF")) ? 2 : 1;
							if (score > best_score) { best_score = score; best_x = x; best_op = op; best_px = px; }
						}
				if (best_x < 0)
					continue;
				// R: S with x at its non-dominating value
				int nd = best_op == 0 ? !best_px : best_px;
				SigBit xsig = S->getPort(pins[best_x]).as_bit();
				rewired.insert(sigmap(xsig));
				Const rinit(State::S0, 16);
				for (int idx = 0; idx < 16; idx++) {
					// R ignores the x pin (now tied to 0) and the unused pins
					int j = ((idx & smask) & ~(1 << best_x)) | (nd << best_x);
					rinit.set(idx, bit(sinit, j) ? State::S1 : State::S0);
				}
				S->setParam(ID(LUT_INIT), rinit);
				S->setPort(pins[best_x], State::S0);
				// each mux: inputs lo, hi, x, R -> g(x, R) ? hi : lo
				for (auto &m : muxes) {
					Cell *M = std::get<0>(m);
					int sel = std::get<1>(m), lo = std::get<2>(m), hi = std::get<3>(m);
					int free = -1;
					for (int i = 0; i < 4; i++)
						if (i != sel && i != lo && i != hi) free = i;
					M->setPort(pins[free], xsig);
					Const ninit(State::S0, 16);
					for (int idx = 0; idx < 16; idx++) {
						int xv = (idx >> free) & 1, rv = (idx >> sel) & 1;
						int xe = xv ^ best_px;
						int g = best_op == 0 ? (xe & rv) : (xe | rv);
						int v = g ? ((idx >> hi) & 1) : ((idx >> lo) & 1);
						ninit.set(idx, v ? State::S1 : State::S0);
					}
					M->setParam(ID(LUT_INIT), ninit);
				}
				count++;
			}
			log("%s: %d shared selects split into their multiplexers.\n", log_id(module), count);
		}
	}
} Ice40LseMuxSelPass;

struct Ice40LseReducePass : public Pass {
	Ice40LseReducePass() : Pass("ice40_lsereduce", "move OR reductions above wide multiplexers as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsereduce [selection]\n");
		log("\n");
		log("An OR tree over the outputs of 2:1 multiplexers that share a select becomes a\n");
		log("multiplexer of two OR trees, one per data side, repeated down through further\n");
		log("2:1 levels. Done only for chains of at least two 2:1 levels; 4:1 multiplexers\n");
		log("($_MUX4_, after muxcover) stop it. An AND with the select counts as a\n");
		log("multiplexer with a zero side.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEREDUCE pass.\n");
		for (auto module : design->selected_modules()) {
			int count = 0;
			for (bool changed = true; changed; ) {
				changed = false;
				SigMap sigmap(module);
				dict<SigBit, Cell*> driver;
				dict<SigBit, int> fanout;
				for (auto cell : module->cells()) {
					for (auto &conn : cell->connections()) {
						if (cell->output(conn.first))
							for (auto bit : sigmap(conn.second))
								driver[bit] = cell;
						else
							for (auto bit : sigmap(conn.second))
								fanout[bit]++;
					}
				}
				for (auto w : module->wires())
					if (w->port_output)
						for (auto bit : sigmap(SigSpec(w)))
							fanout[bit]++;
				pool<SigBit> or_input;
				for (auto cell : module->cells())
					if (cell->type == ID($_OR_)) {
						or_input.insert(sigmap(cell->getPort(ID::A).as_bit()));
						or_input.insert(sigmap(cell->getPort(ID::B).as_bit()));
					}
				auto drv = [&](SigBit b, IdString type) -> Cell* {
					b = sigmap(b);
					if (!driver.count(b) || driver.at(b)->type != type)
						return nullptr;
					return driver.at(b);
				};
				for (auto root : module->selected_cells()) {
					if (root->type != ID($_OR_))
						continue;
					SigBit out = sigmap(root->getPort(ID::Y).as_bit());
					// only the root of a tree: not an OR feeding a single OR
					if (fanout[out] == 1 && or_input.count(out))
						continue;
					std::vector<SigBit> leaves;
					std::vector<Cell*> tree;
					std::function<void(SigBit)> collect = [&](SigBit b) {
						b = sigmap(b);
						Cell *c = drv(b, ID($_OR_));
						if (c && (c == root || fanout[b] == 1)) {
							tree.push_back(c);
							collect(c->getPort(ID::A).as_bit());
							collect(c->getPort(ID::B).as_bit());
						} else
							leaves.push_back(b);
					};
					collect(out);
					if (GetSize(leaves) < 2)
						continue;
					auto or_tree = [&](std::vector<SigBit> bits) -> SigBit {
						std::vector<SigBit> v;
						for (auto b : bits) {
							if (b == State::S1)
								return State::S1;
							if (b != State::S0)
								v.push_back(b);
						}
						if (v.empty())
							return State::S0;
						SigBit r = v[0];
						for (int i = 1; i < GetSize(v); i++)
							r = module->OrGate(NEW_ID, r, v[i]);
						return r;
					};
					// split the leaves by a common 2:1 select, if they have one
					auto split = [&](const std::vector<SigBit> &bits, SigBit &sel, std::vector<SigBit> &side_a, std::vector<SigBit> &side_b) {
						sel = SigBit();
						for (auto b : bits)
							if (Cell *m = drv(b, ID($_MUX_))) {
								sel = sigmap(m->getPort(ID::S).as_bit());
								break;
							}
						if (!sel.wire)
							return false;
						int n = 0;
						for (auto b : bits) {
							b = sigmap(b);
							if (b == State::S0)
								continue;
							if (Cell *m = drv(b, ID($_MUX_)); m && sigmap(m->getPort(ID::S).as_bit()) == sel) {
								side_a.push_back(m->getPort(ID::A).as_bit());
								side_b.push_back(m->getPort(ID::B).as_bit());
								n++;
							} else if (Cell *a = drv(b, ID($_AND_)); a && (sigmap(a->getPort(ID::A).as_bit()) == sel || sigmap(a->getPort(ID::B).as_bit()) == sel)) {
								side_b.push_back(sigmap(a->getPort(ID::A).as_bit()) == sel ? a->getPort(ID::B).as_bit() : a->getPort(ID::A).as_bit());
							} else
								return false;
						}
						return n > 0;
					};
					std::function<int(const std::vector<SigBit>&, int)> levels = [&](const std::vector<SigBit> &bits, int depth) -> int {
						SigBit sel;
						std::vector<SigBit> sa, sb;
						if (depth > 8 || !split(bits, sel, sa, sb))
							return 0;
						return 1 + std::max(levels(sa, depth + 1), levels(sb, depth + 1));
					};
					std::function<SigBit(const std::vector<SigBit>&, int)> push = [&](const std::vector<SigBit> &bits, int depth) -> SigBit {
						SigBit sel;
						std::vector<SigBit> sa, sb;
						if (depth > 8 || !split(bits, sel, sa, sb))
							return or_tree(bits);
						return module->MuxGate(NEW_ID, push(sa, depth + 1), push(sb, depth + 1), sel);
					};
					if (levels(leaves, 0) < 2)
						continue;
					SigBit y = push(leaves, 0);
					SigSpec root_y = root->getPort(ID::Y);
					for (auto c : tree)
						module->remove(c);
					module->connect(root_y, y);
					count++;
					changed = true;
					break;
				}
			}
			log("%s: %d OR trees moved above a multiplexer.\n", log_id(module), count);
		}
	}
} Ice40LseReducePass;

struct Ice40LseMuxCmpPass : public Pass {
	Ice40LseMuxCmpPass() : Pass("ice40_lsemuxcmp", "compare both inputs of a multiplexer instead of its output, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsemuxcmp [selection]\n");
		log("\n");
		log("A $mux whose output is read only by comparisons against constants ($eq, $ne)\n");
		log("and reductions ($reduce_or, $reduce_and, $reduce_bool, $logic_not) is removed:\n");
		log("each of those cells is applied to both data inputs and the results are\n");
		log("multiplexed instead. Compares that decode a case statement (select of a\n");
		log("$pmux) do not count. Run right after proc.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEMUXCMP pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, std::vector<Cell*>> readers;
			pool<SigBit> outside;
			for (auto w : module->wires())
				if (w->port_output || w->get_bool_attribute(ID::keep))
					for (auto b : sigmap(SigSpec(w)))
						outside.insert(b);
			for (auto cell : module->cells())
				for (auto &conn : cell->connections())
					if (cell->input(conn.first))
						for (auto b : sigmap(conn.second))
							readers[b].push_back(cell);
			// case statement decoders (selects of a $pmux) stay with their case
			pool<SigBit> pmux_sel;
			for (auto cell : module->cells())
				if (cell->type == ID($pmux))
					for (auto b : sigmap(cell->getPort(ID::S)))
						pmux_sel.insert(b);
			auto is_cmp = [&](Cell *c) {
				bool cmp = c->type.in(ID($reduce_or), ID($reduce_and), ID($reduce_bool), ID($logic_not)) ||
						(c->type.in(ID($eq), ID($ne)) && (sigmap(c->getPort(ID::A)).is_fully_const() ||
						sigmap(c->getPort(ID::B)).is_fully_const()));
				if (!cmp)
					return false;
				for (auto b : sigmap(c->getPort(ID::Y)))
					if (pmux_sel.count(b))
						return false;
				return true;
			};
			int count = 0;
			std::vector<Cell*> muxes;
			for (auto cell : module->selected_cells())
				if (cell->type == ID($mux))
					muxes.push_back(cell);
			for (auto mux : muxes) {
				SigSpec y = sigmap(mux->getPort(ID::Y));
				pool<Cell*> users;
				bool ok = true;
				for (auto b : y) {
					if (outside.count(b)) {
						ok = false;
						break;
					}
					if (!readers.count(b))
						continue;
					for (auto c : readers.at(b)) {
						if (!is_cmp(c)) {
							ok = false;
							break;
						}
						users.insert(c);
					}
					if (!ok)
						break;
				}
				if (!ok || users.empty())
					continue;
				SigSpec a = sigmap(mux->getPort(ID::A)), bb = sigmap(mux->getPort(ID::B));
				SigBit sel = mux->getPort(ID::S).as_bit();
				// a reader's operand bits, taken from one side of the multiplexer
				auto side = [&](SigSpec sig, const SigSpec &from) {
					SigSpec r = sigmap(sig);
					for (int i = 0; i < GetSize(r); i++)
						for (int k = 0; k < GetSize(y); k++)
							if (r[i] == y[k]) {
								r[i] = from[k];
								break;
							}
					return r;
				};
				for (auto c : users) {
					SigSpec cy = c->getPort(ID::Y);
					Cell *ca = module->addCell(NEW_ID, c);
					Cell *cb = module->addCell(NEW_ID, c);
					for (auto port : {ID::A, ID::B})
						if (c->hasPort(port)) {
							ca->setPort(port, side(c->getPort(port), a));
							cb->setPort(port, side(c->getPort(port), bb));
						}
					SigSpec ya = module->addWire(NEW_ID, GetSize(cy)), yb = module->addWire(NEW_ID, GetSize(cy));
					ca->setPort(ID::Y, ya);
					cb->setPort(ID::Y, yb);
					module->remove(c);
					module->addMux(NEW_ID, ya, yb, sel, cy);
				}
				log("  %s (%s)\n", log_id(mux), log_signal(mux->getPort(ID::Y)));
				module->remove(mux);
				count++;
				// readers changed: rebuild before the next multiplexer
				readers.clear();
				for (auto cell : module->cells())
					for (auto &conn : cell->connections())
						if (cell->input(conn.first))
							for (auto b : sigmap(conn.second))
								readers[b].push_back(cell);
			}
			log("%s: %d multiplexers replaced by compares of both inputs.\n", log_id(module), count);
		}
	}
} Ice40LseMuxCmpPass;

struct Ice40LseCutPass : public Pass {
	Ice40LseCutPass() : Pass("ice40_lsecut", "keep nets as LUT boundaries through abc") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsecut -insert [selection]\n");
		log("    ice40_lsecut -remove [selection]\n");
		log("    ice40_lsecut -conds [selection]\n");
		log("\n");
		log("-insert puts a $__ICE40_LSECUT cell after every bit of the selected wires that is\n");
		log("driven by a gate, and makes all readers read its output, so abc sees the net as\n");
		log("a boundary (a LUT output that the logic after it reads as is). -remove takes\n");
		log("the cells out again. -conds marks (attribute lse_cut) the select of every $mux\n");
		log("that depends on more than two signals: the condition of an if, which LSE\n");
		log("builds as its own LUT.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		bool insert = false, remove = false, conds = false;
		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++) {
			if (args[argidx] == "-insert") { insert = true; continue; }
			if (args[argidx] == "-remove") { remove = true; continue; }
			if (args[argidx] == "-conds") { conds = true; continue; }
			break;
		}
		extra_args(args, argidx, design);
		if (int(insert) + int(remove) + int(conds) != 1)
			log_cmd_error("Give one of -insert, -remove and -conds.\n");
		log_header(design, "Executing ICE40_LSECUT pass.\n");
		for (auto module : design->selected_modules()) {
			int count = 0;
			if (conds) {
				SigMap sigmap(module);
				dict<SigBit, Cell*> driver;
				for (auto cell : module->cells())
					if (cell->type.begins_with("$") && !RTLIL::builtin_ff_cell_types().count(cell->type) && !cell->type.begins_with("$mem"))
						for (auto &conn : cell->connections())
							if (cell->output(conn.first))
								for (auto bit : sigmap(conn.second))
									driver[bit] = cell;
				// bit-level readers of each net: a condition read by one 2:1 mux bit
				// merges into it
				dict<SigBit, int> readers;
				for (auto cell : module->cells())
					for (auto &conn : cell->connections())
						if (cell->input(conn.first)) {
							int w = cell->type == ID($mux) && conn.first == ID::S ? GetSize(cell->getPort(ID::Y)) : 1;
							for (auto b : sigmap(conn.second))
								readers[b] += w;
						}
				for (auto cell : module->selected_cells()) {
					if (cell->type != ID($mux))
						continue;
					SigSpec sw = cell->getPort(ID::S);
					SigBit sb = sigmap(sw.as_bit());
					if (!driver.count(sb) || !sw[0].wire || readers[sb] < 2)
						continue;
					pool<SigBit> vars;
					pool<Cell*> seen;
					std::vector<SigBit> stack = {sb};
					while (!stack.empty() && GetSize(vars) <= 2) {
						SigBit x = stack.back();
						stack.pop_back();
						if (!x.wire)
							continue;
						if (!driver.count(x)) { vars.insert(x); continue; }
						Cell *c = driver.at(x);
						if (seen.count(c))
							continue;
						seen.insert(c);
						for (auto &conn : c->connections())
							if (c->input(conn.first))
								for (auto y : sigmap(conn.second))
									stack.push_back(y);
					}
					if (GetSize(vars) > 2) {
						sw[0].wire->set_bool_attribute(ID(lse_cut));
						count++;
					}
				}
				log("%s: %d conditions marked.\n", log_id(module), count);
				continue;
			}
			if (remove) {
				std::vector<Cell*> cuts;
				for (auto cell : module->cells())
					if (cell->type == ID($__ICE40_LSECUT))
						cuts.push_back(cell);
				for (auto cell : cuts) {
					module->connect(cell->getPort(ID::Y), cell->getPort(ID::A));
					module->remove(cell);
					count++;
				}
				log("%s: %d cuts removed.\n", log_id(module), count);
				continue;
			}
			SigMap sigmap(module);
			pool<SigBit> driven;
			for (auto cell : module->cells())
				if (cell->type.begins_with("$_"))
					for (auto &conn : cell->connections())
						if (cell->output(conn.first))
							for (auto b : sigmap(conn.second))
								driven.insert(b);
			pool<SigBit> bits;
			for (auto w : module->selected_wires())
				for (auto b : sigmap(SigSpec(w)))
					if (driven.count(b))
						bits.insert(b);
			if (bits.empty())
				continue;
			dict<SigBit, SigBit> repl;
			for (auto b : bits) {
				SigBit y = module->addWire(NEW_ID);
				Cell *c = module->addCell(NEW_ID, ID($__ICE40_LSECUT));
				c->setPort(ID::A, b);
				c->setPort(ID::Y, y);
				repl[b] = y;
				count++;
			}
			for (auto cell : module->cells()) {
				if (cell->type == ID($__ICE40_LSECUT))
					continue;
				for (auto &conn : cell->connections()) {
					if (!cell->input(conn.first))
						continue;
					SigSpec sig = sigmap(conn.second);
					bool changed = false;
					for (auto &b : sig)
						if (repl.count(b)) { b = repl.at(b); changed = true; }
					if (changed)
						cell->setPort(conn.first, sig);
				}
			}
			log("%s: %d nets cut.\n", log_id(module), count);
		}
	}
} Ice40LseCutPass;

struct Ice40LseTwoWayPass : public Pass {
	Ice40LseTwoWayPass() : Pass("ice40_lsetwoway", "reduce a multiplexer tree over two words to one multiplexer, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsetwoway [selection]\n");
		log("\n");
		log("A tree of $mux cells (inner ones read only by the tree) whose data inputs are\n");
		log("just two words W0 and W1 (and don't-cares) becomes one $mux between them. Its\n");
		log("select is the same tree built one bit wide with W0 = 0 and W1 = 1, shared by\n");
		log("all bits. Over three or four words it becomes a 4:1 multiplexer whose two\n");
		log("selects are the leaf index bits, built the same way.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSETWOWAY pass.\n");
		for (auto module : design->selected_modules()) {
			int count = 0;
			for (bool changed = true; changed; ) {
				changed = false;
				SigMap sigmap(module);
				dict<SigBit, int> nreaders;
				for (auto cell : module->cells())
					for (auto &conn : cell->connections())
						if (cell->input(conn.first))
							for (auto b : sigmap(conn.second))
								nreaders[b]++;
				for (auto w : module->wires())
					if (w->port_output || w->get_bool_attribute(ID::keep))
						for (auto b : sigmap(SigSpec(w)))
							nreaders[b] += 2;
				dict<SigSpec, Cell*> mux_of;
				pool<SigBit> mux_bits;
				for (auto cell : module->cells())
					if (cell->type == ID($mux)) {
						SigSpec y = sigmap(cell->getPort(ID::Y));
						mux_of[y] = cell;
						for (auto b : y)
							mux_bits.insert(b);
					}
				// a $mux whose output is read only by another $mux is an inner node
				auto inner = [&](const SigSpec &sig) {
					if (!mux_of.count(sig))
						return false;
					for (auto b : sig)
						if (nreaders[b] != 1)
							return false;
					return true;
				};
				for (auto root : module->selected_cells()) {
					if (root->type != ID($mux))
						continue;
					SigSpec ry = sigmap(root->getPort(ID::Y));
					if (inner(ry) && GetSize(ry) > 0) {
						// part of a bigger tree: start from its root
						bool parent_mux = false;
						for (auto cell : module->cells())
							if (cell->type == ID($mux) && cell != root)
								for (auto port : {ID::A, ID::B})
									if (sigmap(cell->getPort(port)) == ry)
										parent_mux = true;
						if (parent_mux)
							continue;
					}
					std::vector<SigSpec> leaves;
					int nodes = 0;
					std::function<void(SigSpec)> walk = [&](SigSpec sig) {
						if (inner(sig) && nodes < 64) {
							Cell *m = mux_of.at(sig);
							nodes++;
							walk(sigmap(m->getPort(ID::A)));
							walk(sigmap(m->getPort(ID::B)));
						} else {
							if (sig.is_fully_undef())
								return;
							for (auto &l : leaves)
								if (l == sig)
									return;
							leaves.push_back(sig);
						}
					};
					nodes++;
					walk(sigmap(root->getPort(ID::A)));
					walk(sigmap(root->getPort(ID::B)));
					bool has_const = false;
					for (auto &l : leaves)
						if (l.is_fully_const()) has_const = true;
					int k = GetSize(leaves);
					if (has_const || k < 2 || k > 4 || (k > 2 && nodes <= 3) || nodes < 2)
						continue;
					if (k > 2) {
						// encoded 4:1: leaf index bits as one-bit trees
						std::function<SigBit(SigSpec, int)> idx_tree = [&](SigSpec sig, int bitn) -> SigBit {
							for (int li = 0; li < k; li++)
								if (sig == leaves[li]) return (li >> bitn) & 1 ? State::S1 : State::S0;
							if (sig.is_fully_undef()) return State::Sx;
							Cell *m = mux_of.at(sig);
							SigBit a = idx_tree(sigmap(m->getPort(ID::A)), bitn), b = idx_tree(sigmap(m->getPort(ID::B)), bitn);
							return module->MuxGate(NEW_ID, a, b, m->getPort(ID::S).as_bit());
						};
						SigBit e0 = idx_tree(ry, 0), e1 = idx_tree(ry, 1);
						SigSpec l3 = k > 3 ? leaves[3] : SigSpec(State::Sx, GetSize(ry));
						SigSpec m0 = module->Mux(NEW_ID, leaves[0], leaves[1], e0);
						SigSpec m1 = module->Mux(NEW_ID, leaves[2], l3, e0);
						SigSpec y = root->getPort(ID::Y);
						module->remove(root);
						module->addMux(NEW_ID, m0, m1, e1, y);
						log("  %s: %d multiplexers over %d words\n", log_signal(y), nodes, k);
						count++;
						changed = true;
						break;
					}
					// the select: the tree one bit wide, W0 -> 0, W1 -> 1
					std::function<SigBit(SigSpec)> bit_tree = [&](SigSpec sig) -> SigBit {
						if (sig == leaves[0]) return State::S0;
						if (sig == leaves[1]) return State::S1;
						if (sig.is_fully_undef()) return State::Sx;
						Cell *m = mux_of.at(sig);
						SigBit a = bit_tree(sigmap(m->getPort(ID::A))), b = bit_tree(sigmap(m->getPort(ID::B)));
						return module->MuxGate(NEW_ID, a, b, m->getPort(ID::S).as_bit());
					};
					SigBit sel = bit_tree(ry);
					SigSpec y = root->getPort(ID::Y);
					module->remove(root);
					module->addMux(NEW_ID, leaves[0], leaves[1], sel, y);
					log("  %s: %d multiplexers over %s and %s\n", log_signal(y), nodes, log_signal(leaves[0]), log_signal(leaves[1]));
					count++;
					changed = true;
					break;
				}
			}
			log("%s: %d multiplexer trees over two words reduced.\n", log_id(module), count);
		}
	}
} Ice40LseTwoWayPass;

struct Ice40LseAndSrPass : public Pass {
	Ice40LseAndSrPass() : Pass("ice40_lseandsr", "take a factor of an AND in front of a flip-flop as its synchronous reset, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lseandsr [selection]\n");
		log("\n");
		log("A $dff whose D is 'L && R' (read only by it) becomes a $sdff that resets when\n");
		log("one factor is 0, loading the other: the factor that is a single signal (or\n");
		log("its inverse) when only one of them is, R when neither is; nothing when both\n");
		log("are single signals, or when the reset factor compares two words.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEANDSR pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, Cell*> driver;
			dict<SigBit, int> nreaders;
			for (auto cell : module->cells()) {
				for (auto &conn : cell->connections()) {
					if (cell->output(conn.first))
						for (auto b : sigmap(conn.second))
							driver[b] = cell;
					else
						for (auto b : sigmap(conn.second))
							nreaders[b]++;
				}
			}
			for (auto w : module->wires())
				if (w->port_output)
					for (auto b : sigmap(SigSpec(w)))
						nreaders[b]++;
			// a single signal: an input, a flip-flop output or the inverse of one
			std::function<bool(SigBit)> single = [&](SigBit b) {
				if (!b.wire || !driver.count(b))
					return true;
				Cell *c = driver.at(b);
				if (RTLIL::builtin_ff_cell_types().count(c->type))
					return true;
				if (c->type.in(ID($not), ID($logic_not), ID($_NOT_)) && GetSize(c->getPort(ID::A)) == 1)
					return single(sigmap(c->getPort(ID::A).as_bit()));
				return false;
			};
			// is there a comparison of two non-constant words in front of b
			auto compares_words = [&](SigBit b) {
				pool<Cell*> seen;
				std::vector<SigBit> stack = {b};
				while (!stack.empty()) {
					SigBit x = stack.back();
					stack.pop_back();
					if (!x.wire || !driver.count(x))
						continue;
					Cell *c = driver.at(x);
					if (seen.count(c) || RTLIL::builtin_ff_cell_types().count(c->type))
						continue;
					seen.insert(c);
					if (c->type.in(ID($lt), ID($le), ID($gt), ID($ge), ID($eq), ID($ne)) &&
							!sigmap(c->getPort(ID::A)).is_fully_const() && !sigmap(c->getPort(ID::B)).is_fully_const() &&
							GetSize(c->getPort(ID::A)) > 1)
						return true;
					for (auto &conn : c->connections())
						if (c->input(conn.first))
							for (auto y : sigmap(conn.second))
								stack.push_back(y);
				}
				return false;
			};
			int count = 0;
			FfInitVals initvals(&sigmap, module);
			std::vector<Cell*> ffs;
			for (auto cell : module->selected_cells())
				if (cell->type == ID($dff))
					ffs.push_back(cell);
			for (auto cell : ffs) {
				FfData ff(&initvals, cell);
				std::vector<int> plain_bits;
				std::vector<std::tuple<int, SigBit, SigBit>> reset_bits;	// bit, reset factor, data
				for (int i = 0; i < ff.width; i++) {
					SigBit d = sigmap(ff.sig_d[i]);
					Cell *g = driver.count(d) ? driver.at(d) : nullptr;
					if (!g || nreaders[d] != 1 || !g->type.in(ID($logic_and), ID($and)) ||
							GetSize(g->getPort(ID::A)) != 1 || GetSize(g->getPort(ID::B)) != 1) {
						plain_bits.push_back(i);
						continue;
					}
					SigBit l = sigmap(g->getPort(ID::A).as_bit()), r = sigmap(g->getPort(ID::B).as_bit());
					bool sl = single(l), sr = single(r);
					if (!l.wire || !r.wire || (sl && sr)) {
						plain_bits.push_back(i);
						continue;
					}
					// a factor that compares two words stays in D
					SigBit rf = sl ? l : r;
					if (compares_words(rf)) {
						plain_bits.push_back(i);
						continue;
					}
					reset_bits.push_back({i, rf, sl ? r : l});
				}
				if (reset_bits.empty())
					continue;
				if (!plain_bits.empty())
					ff.slice(plain_bits).emit();
				for (auto &rb : reset_bits) {
					FfData bit = ff.slice({std::get<0>(rb)});
					bit.has_srst = true;
					bit.sig_srst = std::get<1>(rb);
					bit.pol_srst = false;
					bit.val_srst = Const(State::S0, 1);
					bit.sig_d = std::get<2>(rb);
					bit.emit();
					count++;
				}
				module->remove(cell);
			}
			log("%s: %d flip-flops reset by a factor of their AND.\n", log_id(module), count);
		}
	}
} Ice40LseAndSrPass;

struct Ice40LseAddLoadPass : public Pass {
	Ice40LseAddLoadPass() : Pass("ice40_lseaddload", "fold a load next to an adder into the adder's operands, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lseaddload [selection]\n");
		log("\n");
		log("A counter q <= s ? X : (q + K), K a constant or one signal bit (an enable\n");
		log("folded in) and X not constant, becomes q <= (s ? 0 : q) + (s ? X : K): the\n");
		log("load goes through the adder, the operands gated in front of the carry chain.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEADDLOAD pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, int> nreaders;
			for (auto cell : module->cells())
				for (auto &conn : cell->connections())
					if (cell->input(conn.first))
						for (auto b : sigmap(conn.second))
							nreaders[b]++;
			for (auto w : module->wires())
				if (w->port_output)
					for (auto b : sigmap(SigSpec(w)))
						nreaders[b]++;
			dict<SigSpec, Cell*> add_of;
			for (auto cell : module->cells())
				if (cell->type == ID($add))
					add_of[sigmap(cell->getPort(ID::Y))] = cell;
			int count = 0;
			std::vector<Cell*> muxes;
			for (auto cell : module->selected_cells())
				if (cell->type == ID($mux))
					muxes.push_back(cell);
			for (auto mux : muxes) {
				int width = GetSize(mux->getPort(ID::Y));
				for (int side = 0; side < 2; side++) {
					SigSpec sum = sigmap(mux->getPort(side ? ID::B : ID::A));
					if (!add_of.count(sum))
						continue;
					Cell *add = add_of.at(sum);
					bool only = true;
					for (auto b : sum)
						if (nreaders[b] != 1) only = false;
					if (!only || GetSize(add->getPort(ID::Y)) != width)
						continue;
					bool sgn = add->getParam(ID::A_SIGNED).as_bool() && add->getParam(ID::B_SIGNED).as_bool();
					SigSpec p = add->getPort(ID::A), k = add->getPort(ID::B);
					p.extend_u0(width, sgn);
					k.extend_u0(width, sgn);
					SigSpec x = mux->getPort(side ? ID::A : ID::B);
					SigBit s = mux->getPort(ID::S).as_bit();
					// only a counter: the register plus a constant (or the enable folded into
					// it), loaded with data rather than cleared
					if (sigmap(x).is_fully_const() || sigmap(x) == sigmap(p))
						continue;
					SigSpec y0 = sigmap(mux->getPort(ID::Y));
					bool counter = false;
					for (auto ff : module->cells())
						if (RTLIL::builtin_ff_cell_types().count(ff->type) && ff->hasPort(ID::D) &&
								sigmap(ff->getPort(ID::D)) == y0 && sigmap(ff->getPort(ID::Q)) == sigmap(p))
							counter = true;
					if (!counter)
						continue;
					// K: a constant, or depending on one signal only (en ? c : 0)
					pool<SigBit> vars;
					pool<Cell*> seen;
					std::vector<SigBit> stack;
					for (auto b : sigmap(k)) stack.push_back(b);
					while (!stack.empty() && GetSize(vars) <= 1) {
						SigBit b = stack.back();
						stack.pop_back();
						if (!b.wire)
							continue;
						Cell *d = nullptr;
						for (auto c : module->cells())
							if (c->type.in(ID($mux), ID($not), ID($logic_not), ID($and), ID($or), ID($xor), ID($pos)))
								for (auto &conn : c->connections())
									if (c->output(conn.first))
										for (auto ob : sigmap(conn.second))
											if (ob == b) d = c;
						if (!d) { vars.insert(b); continue; }
						if (seen.count(d))
							continue;
						seen.insert(d);
						for (auto &conn : d->connections())
							if (d->input(conn.first))
								for (auto ib : sigmap(conn.second))
									stack.push_back(ib);
					}
					if (GetSize(vars) > 1)
						continue;
					// the sum is taken when s selects its side; the load otherwise
					SigSpec zero(State::S0, width);
					SigSpec np = side ? module->Mux(NEW_ID, zero, p, s) : module->Mux(NEW_ID, p, zero, s);
					SigSpec nk = side ? module->Mux(NEW_ID, x, k, s) : module->Mux(NEW_ID, k, x, s);
					add->setPort(ID::A, np);
					add->setPort(ID::B, nk);
					add->setParam(ID::A_WIDTH, width);
					add->setParam(ID::B_WIDTH, width);
					add->setParam(ID::A_SIGNED, false);
					add->setParam(ID::B_SIGNED, false);
					SigSpec y = mux->getPort(ID::Y);
					log("  %s: load %s\n", log_signal(y), log_signal(x));
					module->remove(mux);
					module->connect(y, add->getPort(ID::Y));
					count++;
					break;
				}
			}
			log("%s: %d loads folded into adders.\n", log_id(module), count);
		}
	}
} Ice40LseAddLoadPass;

struct Ice40LseHoldSelPass : public Pass {
	Ice40LseHoldSelPass() : Pass("ice40_lseholdsel", "pick the data select of a two-word register as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lseholdsel [selection]\n");
		log("\n");
		log("A register whose next state is a $mux tree over two words and its own output\n");
		log("(the hold), with a hold under both arms of the root, gets D = hold ? Q :\n");
		log("(path ? W1 : W0), where W1 is the word under the root's 1 arm and path is the\n");
		log("tree one bit wide with W1 = 1 and W0 = hold = 0. opt_dff then makes !hold the\n");
		log("enable. LSE selects with that full path; with a hold under one arm only it\n");
		log("drops the hold muxes, as opt_dff does.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSEHOLDSEL pass.\n");
		for (auto module : design->selected_modules()) {
			int count = 0;
			SigMap sigmap(module);
			dict<SigBit, int> nreaders;
			for (auto cell : module->cells())
				for (auto &conn : cell->connections())
					if (cell->input(conn.first))
						for (auto b : sigmap(conn.second))
							nreaders[b]++;
			for (auto w : module->wires())
				if (w->port_output || w->get_bool_attribute(ID::keep))
					for (auto b : sigmap(SigSpec(w)))
						nreaders[b] += 2;
			dict<SigSpec, Cell*> mux_of;
			for (auto cell : module->cells())
				if (cell->type == ID($mux))
					mux_of[sigmap(cell->getPort(ID::Y))] = cell;
			auto inner = [&](const SigSpec &sig) {
				if (!mux_of.count(sig))
					return false;
				for (auto b : sig)
					if (nreaders[b] != 1)
						return false;
				return true;
			};
			for (auto ff : module->selected_cells()) {
				if (!ff->type.in(ID($dff), ID($adff)))
					continue;
				SigSpec d = sigmap(ff->getPort(ID::D)), q = sigmap(ff->getPort(ID::Q));
				if (!mux_of.count(d))
					continue;
				Cell *root = mux_of.at(d);
				std::vector<SigSpec> words;
				bool ok = true;
				std::function<bool(SigSpec)> walk = [&](SigSpec sig) -> bool {
					if (sig == q)
						return true;
					if (inner(sig)) {
						Cell *m = mux_of.at(sig);
						bool ha = walk(sigmap(m->getPort(ID::A)));
						bool hb = walk(sigmap(m->getPort(ID::B)));
						return ha || hb;
					}
					if (sig.is_fully_undef() || sig.is_fully_const()) {
						ok = false;
						return false;
					}
					bool seen = false;
					for (auto &w : words)
						if (w == sig) seen = true;
					if (!seen)
						words.push_back(sig);
					return false;
				};
				SigSpec ra = sigmap(root->getPort(ID::A)), rb = sigmap(root->getPort(ID::B));
				bool hold_b = walk(rb);
				int nb = GetSize(words);
				bool hold_a = walk(ra);
				if (!ok || !hold_a || !hold_b || GetSize(words) != 2 || nb != 1 || rb == q || ra == q)
					continue;
				SigSpec w1 = words[0], w0 = words[1];
				// the tree one bit wide, with the given leaf values
				std::function<SigBit(SigSpec, State, State, State)> bit_tree = [&](SigSpec sig, State v1, State v0, State vq) -> SigBit {
					if (sig == q) return vq;
					if (sig == w1) return v1;
					if (sig == w0) return v0;
					Cell *m = mux_of.at(sig);
					SigBit a = bit_tree(sigmap(m->getPort(ID::A)), v1, v0, vq), b = bit_tree(sigmap(m->getPort(ID::B)), v1, v0, vq);
					return module->MuxGate(NEW_ID, a, b, m->getPort(ID::S).as_bit());
				};
				SigBit path = bit_tree(d, State::S1, State::S0, State::S0);
				SigBit hold = bit_tree(d, State::S0, State::S0, State::S1);
				SigSpec data = module->Mux(NEW_ID, w0, w1, path);
				SigSpec next = module->Mux(NEW_ID, data, ff->getPort(ID::Q), hold);
				ff->setPort(ID::D, next);
				log("  %s: select %s over %s and %s\n", log_id(ff), log_signal(path), log_signal(w1), log_signal(w0));
				count++;
			}
			log("%s: %d register selects rebuilt.\n", log_id(module), count);
		}
	}
} Ice40LseHoldSelPass;

struct Ice40LseTogglePass : public Pass {
	Ice40LseTogglePass() : Pass("ice40_lsetoggle", "turn a conditional register toggle into an XOR, as Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsetoggle [selection]\n");
		log("\n");
		log("s ? ~q : q on register outputs q becomes q ^ s before enables are extracted:\n");
		log("LSE makes a toggle an XOR and takes only the conditions around it as the\n");
		log("enable.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSETOGGLE pass.\n");
		for (auto module : design->selected_modules()) {
			int count = 0;
			SigMap sigmap(module);
			pool<SigBit> ff_q;
			for (auto cell : module->cells())
				if (cell->is_builtin_ff())
					for (auto b : sigmap(cell->getPort(ID::Q)))
						ff_q.insert(b);
			dict<SigBit, Cell*> driver;
			for (auto cell : module->cells())
				for (auto &conn : cell->connections())
					if (cell->output(conn.first))
						for (auto b : sigmap(conn.second))
							driver[b] = cell;
			dict<SigBit, std::vector<std::pair<Cell*, IdString>>> readers;
			for (auto cell : module->cells())
				for (auto &conn : cell->connections())
					if (cell->input(conn.first))
						for (auto b : sigmap(conn.second))
							readers[b].push_back({cell, conn.first});
			dict<SigSpec, SigSpec> inverted;
			for (auto cell : module->cells())
				if (cell->type == ID($not) || (cell->type == ID($logic_not) && GetSize(cell->getPort(ID::A)) == 1))
					inverted[sigmap(cell->getPort(ID::Y))] = sigmap(cell->getPort(ID::A));
			for (auto cell : module->selected_cells()) {
				if (cell->type != ID($mux))
					continue;
				SigSpec a = sigmap(cell->getPort(ID::A)), b = sigmap(cell->getPort(ID::B));
				bool flip = inverted.count(a) && inverted.at(a) == b;
				if (flip)
					std::swap(a, b);
				else if (!inverted.count(b) || inverted.at(b) != a)
					continue;
				bool all_q = true;
				for (auto bit : a)
					if (!ff_q.count(bit)) all_q = false;
				if (!all_q)
					continue;
				SigBit sel = sigmap(cell->getPort(ID::S).as_bit());
				// a case on the register itself gives the same shape; not a toggle
				bool reads_q = false;
				pool<SigBit> qbits(a.begin(), a.end()), seen;
				std::vector<SigBit> todo = {sel};
				while (!todo.empty() && !reads_q && GetSize(seen) < 256) {
					SigBit b = todo.back();
					todo.pop_back();
					if (!seen.insert(b).second)
						continue;
					if (qbits.count(b)) {
						reads_q = true;
						break;
					}
					if (!driver.count(b) || driver.at(b)->is_builtin_ff())
						continue;
					for (auto &conn : driver.at(b)->connections())
						if (driver.at(b)->input(conn.first))
							for (auto x : sigmap(conn.second))
								if (x.wire) todo.push_back(x);
				}
				if (reads_q)
					continue;
				// only in the register's own next state: through multiplexers into its D
				bool feeds_back = false;
				pool<SigBit> fseen;
				std::vector<SigBit> ftodo;
				for (auto b : sigmap(cell->getPort(ID::Y)))
					ftodo.push_back(b);
				while (!ftodo.empty() && !feeds_back && GetSize(fseen) < 256) {
					SigBit b = ftodo.back();
					ftodo.pop_back();
					if (!fseen.insert(b).second)
						continue;
					for (auto &r : readers[b]) {
						Cell *rc = r.first;
						if (rc->is_builtin_ff() && r.second == ID::D) {
							for (auto x : sigmap(rc->getPort(ID::Q)))
								if (qbits.count(x)) feeds_back = true;
						} else if (rc->type.in(ID($mux), ID($pmux)) && r.second != ID::S)
							for (auto x : sigmap(rc->getPort(ID::Y)))
								ftodo.push_back(x);
					}
				}
				if (!feeds_back)
					continue;
				if (flip)
					sel = module->NotGate(NEW_ID, sel);
				SigSpec y = cell->getPort(ID::Y);
				module->remove(cell);
				module->addXor(NEW_ID, a, SigSpec(sel, GetSize(a)), y);
				log_debug("  %s toggles on %s\n", log_signal(a), log_signal(sel));
				count++;
			}
			log("%s: %d toggles made XORs.\n", log_id(module), count);
		}
	}
} Ice40LseTogglePass;

struct Ice40LseCeCostPass : public Pass {
	Ice40LseCeCostPass() : Pass("ice40_lsececost", "keep a flip-flop enable only where Lattice LSE does") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    ice40_lsececost [selection]\n");
		log("\n");
		log("For each $_DFFE_* without a synchronous reset, compares the LUTs needed with\n");
		log("the enable (D, plus the enable logic if no other flip-flop shares it) and\n");
		log("without it (E ? D : Q), estimated from the support of the logic only that\n");
		log("flip-flop reads. LSE drops the enable when that is cheaper, or as cheap and D\n");
		log("is an inverter.\n");
		log("\n");
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		extra_args(args, 1, design);
		log_header(design, "Executing ICE40_LSECECOST pass.\n");
		for (auto module : design->selected_modules()) {
			SigMap sigmap(module);
			dict<SigBit, Cell*> driver;
			dict<SigBit, int> nreaders;
			for (auto cell : module->cells())
				for (auto &conn : cell->connections()) {
					if (cell->output(conn.first))
						for (auto b : sigmap(conn.second))
							driver[b] = cell;
					if (cell->input(conn.first))
						for (auto b : sigmap(conn.second))
							nreaders[b]++;
				}
			for (auto w : module->wires())
				if (w->port_output || w->get_bool_attribute(ID::keep))
					for (auto b : sigmap(SigSpec(w)))
						nreaders[b] += 2;
			auto gate = [&](Cell *c) {
				return c && c->type.in(ID($_NOT_), ID($_AND_), ID($_OR_), ID($_XOR_), ID($_XNOR_), ID($_NAND_), ID($_NOR_),
						ID($_ANDNOT_), ID($_ORNOT_), ID($_MUX_), ID($_NMUX_), ID($_AOI3_), ID($_OAI3_), ID($_AOI4_), ID($_OAI4_));
			};
			// the inputs of the logic behind sig that nothing else reads; empty: sig is a plain net
			auto cone = [&](SigBit sig, pool<SigBit> &leaves) -> bool {
				Cell *root = driver.count(sig) ? driver.at(sig) : nullptr;
				if (!gate(root)) {
					leaves.insert(sig);
					return false;
				}
				std::vector<Cell*> todo = {root};
				pool<Cell*> seen = {root};
				while (!todo.empty()) {
					Cell *c = todo.back();
					todo.pop_back();
					for (auto &conn : c->connections()) {
						if (!c->input(conn.first))
							continue;
						for (auto b : sigmap(conn.second)) {
							if (!b.wire)
								continue;
							Cell *d = driver.count(b) ? driver.at(b) : nullptr;
							if (gate(d) && nreaders[b] == 1 && GetSize(seen) < 16) {
								if (seen.insert(d).second)
									todo.push_back(d);
							} else
								leaves.insert(b);
						}
					}
				}
				return true;
			};
			auto luts = [](bool logic, int support) {
				if (!logic)
					return 0;
				return support <= 4 ? 1 : (support + 1) / 3;
			};
			dict<SigBit, std::vector<Cell*>> groups;
			for (auto cell : module->selected_cells())
				// with a synchronous reset LSE keeps the enable (CE over LSR)
				if (cell->type.begins_with("$_DFFE_")) {
					// dfflegalize gives each flip-flop of an active-low enable its own inverter
					SigBit e = sigmap(cell->getPort(ID::E));
					Cell *ec = driver.count(e) ? driver.at(e) : nullptr;
					if (ec && ec->type == ID($_NOT_))
						e = sigmap(ec->getPort(ID::A));
					groups[e].push_back(cell);
				}
			RTLIL::Selection sel(false, false, design);
			int dropped = 0;
			for (auto &it : groups) {
				pool<SigBit> e_leaves;
				bool e_logic = cone(it.first, e_leaves);
				// logic behind an enable several flip-flops share is there anyway
				int e_cost = GetSize(it.second) == 1 ? luts(e_logic, GetSize(e_leaves)) : 0;
				for (auto ff : it.second) {
					SigBit d = sigmap(ff->getPort(ID::D)), q = sigmap(ff->getPort(ID::Q));
					pool<SigBit> d_leaves;
					bool d_logic = cone(d, d_leaves);
					int with_ce = e_cost + luts(d_logic, GetSize(d_leaves));
					pool<SigBit> all = d_leaves;
					for (auto b : e_leaves)
						all.insert(b);
					all.insert(q);
					int without = luts(true, GetSize(all));
					Cell *dc = driver.count(d) ? driver.at(d) : nullptr;
					bool inverter = dc && dc->type == ID($_NOT_);
					if (without < with_ce || (without == with_ce && inverter)) {
						log_debug("  %s: %d LUTs with the enable, %d without\n", log_signal(q), with_ce, without);
						sel.select(module, ff);
						dropped++;
					}
				}
			}
			if (dropped)
				Pass::call_on_selection(design, sel, "dffunmap -ce-only");
			log("%s: %d enables dropped.\n", log_id(module), dropped);
		}
	}
} Ice40LseCeCostPass;

PRIVATE_NAMESPACE_END
