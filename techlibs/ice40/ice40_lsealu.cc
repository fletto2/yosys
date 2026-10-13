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

			// a register that addresses a RAM gets neither a clock enable nor a synchronous
			// reset/set pin: LSE builds both into the logic in front of it
			pool<SigBit> addr_bits;
			for (auto cell : module->cells())
				if (is_ram(cell))
					for (auto port : {ID(RADDR), ID(WADDR), ID(ADDRESS)})
						if (cell->hasPort(port))
							for (auto b : sigmap(cell->getPort(port)))
								addr_bits.insert(b);
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
			// an operand that is a constant plus one or two signal bits is mostly
			// constant: LSE simplifies those, leave them to ABC
			// (followed through chains of adds, e.g. (524 + line_mode) - 3)
			pool<SigBit> const_offset;
			dict<SigBit, Cell*> add_of;
			for (auto cell : module->cells())
				if (cell->type.in(ID($add), ID($sub)))
					for (auto bit : sigmap(cell->getPort(ID::Y)))
						add_of[bit] = cell;
			std::function<void(Cell*, pool<SigBit>&, pool<Cell*>&)> support = [&](Cell *c, pool<SigBit> &vars, pool<Cell*> &seen) {
				if (seen.count(c) || GetSize(vars) > 2)
					return;
				seen.insert(c);
				for (auto port : {ID::A, ID::B})
					for (auto bit : sigmap(c->getPort(port))) {
						if (!bit.wire)
							continue;
						if (add_of.count(bit))
							support(add_of.at(bit), vars, seen);
						else
							vars.insert(bit);
					}
			};
			for (auto cell : module->cells())
				if (cell->type.in(ID($add), ID($sub))) {
					pool<SigBit> vars;
					pool<Cell*> seen;
					support(cell, vars, seen);
					if (GetSize(vars) <= 2)
						for (auto bit : sigmap(cell->getPort(ID::Y)))
							const_offset.insert(bit);
				}
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
						SigBit lo = less(x.extract(0, w - k), y.extract(0, w - k), or_equal);
						r = module->Mux(NEW_ID, lo, State::S0, hi).as_bit();
					} else if (kx > 0) {
						SigBit hi = module->ReduceOr(NEW_ID, y.extract(w - kx, kx));
						SigBit lo = less(x.extract(0, w - kx), y.extract(0, w - kx), or_equal);
						r = module->Mux(NEW_ID, lo, State::S1, hi).as_bit();
					} else
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
			for (auto S : luts) {
				SigBit so = sigmap(S->getPort(ID::O).as_bit());
				if (port_bits.count(so) || !users.count(so))
					continue;
				std::vector<std::tuple<Cell*, int, int, int>> muxes;
				bool only_selects = true;
				for (auto &u : users.at(so)) {
					int sel, lo, hi;
					if (u.first->type != ID(SB_LUT4) || u.first == S || !mux_select(u.first, sel, lo, hi) ||
							u.second != pins[sel]) { only_selects = false; break; }
					muxes.push_back(std::make_tuple(u.first, sel, lo, hi));
				}
				if (!only_selects || GetSize(muxes) < 2)
					continue;
				std::vector<int> live;
				int smask = 0;
				bool tied1 = false;
				for (int i = 0; i < 4; i++) {
					SigBit pb = sigmap(S->getPort(pins[i])).as_bit();
					if (pb.wire) { live.push_back(i); smask |= 1 << i; }
					else if (pb.data != State::S0) tied1 = true;
				}
				if (GetSize(live) < 3 || tied1)
					continue;
				int sinit = init_of(S);
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

PRIVATE_NAMESPACE_END
