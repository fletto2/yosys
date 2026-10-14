/*
 *  yosys -- Yosys Open SYnthesis Suite
 *
 *  Copyright (C) 2012  Claire Xenia Wolf <claire@yosyshq.com>
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

#include "kernel/log.h"
#include "kernel/register.h"
#include "kernel/sigtools.h"
#include "kernel/consteval.h"
#include "kernel/celltypes.h"
#include "fsmdata.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

static RTLIL::Module *module;
static SigMap assign_map;
typedef std::pair<RTLIL::Cell*, RTLIL::IdString> sig2driver_entry_t;
static SigSet<sig2driver_entry_t> sig2driver, sig2user;
static std::set<RTLIL::Cell*> muxtree_cells;
static SigPool sig_at_port;

static bool check_state_mux_tree(RTLIL::SigSpec old_sig, RTLIL::SigSpec sig, pool<Cell*> &recursion_monitor, dict<RTLIL::SigSpec, bool> &mux_tree_cache)
{
	if (mux_tree_cache.find(sig) != mux_tree_cache.end())
		return mux_tree_cache.at(sig);

	if (sig.is_fully_const() || old_sig == sig) {
ret_true:
		mux_tree_cache[sig] = true;
		return true;
	}

	if (sig_at_port.check_any(assign_map(sig))) {
ret_false:
		mux_tree_cache[sig] = false;
		return false;
	}

	std::set<sig2driver_entry_t> cellport_list;
	sig2driver.find(sig, cellport_list);
	for (auto &cellport : cellport_list)
	{
		if ((cellport.first->type != ID($mux) && cellport.first->type != ID($pmux)) || cellport.second != ID::Y) {
			goto ret_false;
		}

		if (recursion_monitor.count(cellport.first)) {
			log_warning("logic loop in mux tree at signal %s in module %s.\n",
					log_signal(sig), module);
			goto ret_false;
		}

		recursion_monitor.insert(cellport.first);

		RTLIL::SigSpec sig_a = assign_map(cellport.first->getPort(ID::A));
		RTLIL::SigSpec sig_b = assign_map(cellport.first->getPort(ID::B));

		if (!check_state_mux_tree(old_sig, sig_a, recursion_monitor, mux_tree_cache)) {
			recursion_monitor.erase(cellport.first);
			goto ret_false;
		}

		for (int i = 0; i < sig_b.size(); i += sig_a.size())
			if (!check_state_mux_tree(old_sig, sig_b.extract(i, sig_a.size()), recursion_monitor, mux_tree_cache)) {
				recursion_monitor.erase(cellport.first);
				goto ret_false;
			}

		recursion_monitor.erase(cellport.first);
		muxtree_cells.insert(cellport.first);
	}

	goto ret_true;
}

static bool check_state_users(RTLIL::SigSpec sig)
{
	if (sig_at_port.check_any(assign_map(sig)))
		return false;

	std::set<sig2driver_entry_t> cellport_list;
	sig2user.find(sig, cellport_list);
	for (auto &cellport : cellport_list) {
		RTLIL::Cell *cell = cellport.first;
		if (muxtree_cells.count(cell) > 0)
			continue;
		if (cell->type == ID($logic_not) && assign_map(cell->getPort(ID::A)) == sig)
			continue;
		if (cellport.second != ID::A && cellport.second != ID::B)
			return false;
		if (!cell->hasPort(ID::A) || !cell->hasPort(ID::B) || !cell->hasPort(ID::Y))
			return false;
		for (auto &port_it : cell->connections())
			if (port_it.first != ID::A && port_it.first != ID::B && port_it.first != ID::Y)
				return false;
		if (assign_map(cell->getPort(ID::A)) == sig && cell->getPort(ID::B).is_fully_const())
			continue;
		if (assign_map(cell->getPort(ID::B)) == sig && cell->getPort(ID::A).is_fully_const())
			continue;
		return false;
	}

	return true;
}

static bool depends_on(RTLIL::SigSpec sig, const pool<RTLIL::SigBit> &target, pool<RTLIL::SigBit> &seen, int depth)
{
	for (auto bit : assign_map(sig)) {
		if (bit.wire == nullptr || seen.count(bit))
			continue;
		seen.insert(bit);
		if (target.count(bit))
			return true;
		if (depth == 0)
			continue;
		std::set<sig2driver_entry_t> drivers;
		sig2driver.find(bit, drivers);
		for (auto &d : drivers) {
			if (d.first->type.in(ID($dff), ID($adff), ID($dffe), ID($adffe), ID($sdff), ID($sdffe), ID($sdffce)))
				continue;
			for (auto &conn : d.first->connections())
				if (d.first->input(conn.first) && depends_on(conn.second, target, seen, depth - 1))
					return true;
		}
	}
	return false;
}

// LSE only extracts a state machine whose next state is chosen by a case on its own
// value (not a chain of ?: compares), with every choice made outside that case picking
// a constant or the state itself: walking back from D, each mux must have one such
// input, and the walk must end at a case whose select depends on the state. With a
// synchronous reset a choice between constants (mode ? B : A) is accepted too; with an
// asynchronous one it is not (lsecmp t/fs1..fs6, spimemio vs layer_renderer).
static bool lse_override_ok(RTLIL::SigSpec sig_q, RTLIL::SigSpec sig, bool const_trees)
{
	pool<RTLIL::SigBit> q_bits = sig_q.to_sigbit_pool();
	std::function<bool(RTLIL::SigSpec, int)> const_tree = [&](RTLIL::SigSpec x, int depth) -> bool {
		x = assign_map(x);
		if (x.is_fully_const())
			return true;
		std::set<sig2driver_entry_t> drivers;
		sig2driver.find(x, drivers);
		if (depth > 4 || drivers.size() != 1 || drivers.begin()->first->type != ID($mux))
			return false;
		RTLIL::Cell *m = drivers.begin()->first;
		return const_tree(m->getPort(ID::A), depth + 1) && const_tree(m->getPort(ID::B), depth + 1);
	};
	for (int guard = 0; guard < 64; guard++) {
		sig = assign_map(sig);
		if (sig.is_fully_const() || sig == sig_q)
			return false;
		std::set<sig2driver_entry_t> drivers;
		sig2driver.find(sig, drivers);
		if (drivers.size() != 1)
			return false;
		RTLIL::Cell *cell = drivers.begin()->first;
		if (!cell->type.in(ID($mux), ID($pmux)))
			return false;
		pool<RTLIL::SigBit> seen;
		bool on_state = depends_on(cell->getPort(ID::S), q_bits, seen, 8);
		if (cell->type == ID($pmux))
			return on_state;
		// a ?: on the state picking a constant is one more override (if (state != DONE &&
		// x) next = IDLE); a chain of them down to constants is not a case
		RTLIL::SigSpec a = assign_map(cell->getPort(ID::A)), b = assign_map(cell->getPort(ID::B));
		if (a == sig_q || a.is_fully_const() || (const_trees && const_tree(a, 0)))
			sig = b;
		else if (b == sig_q || b.is_fully_const() || (const_trees && const_tree(b, 0)))
			sig = a;
		else
			return false;
	}
	return false;
}

static void detect_fsm(RTLIL::Wire *wire, bool ignore_self_reset=false, bool lse=false)
{
	bool has_fsm_encoding_attr = wire->attributes.count(ID::fsm_encoding) > 0 && wire->attributes.at(ID::fsm_encoding).decode_string() != "none";
	bool has_fsm_encoding_none = wire->attributes.count(ID::fsm_encoding) > 0 && wire->attributes.at(ID::fsm_encoding).decode_string() == "none";
	bool has_init_attr = wire->attributes.count(ID::init) > 0;
	bool is_module_port = sig_at_port.check_any(assign_map(RTLIL::SigSpec(wire)));
	bool looks_like_state_reg = false, looks_like_good_state_reg = false;
	bool is_self_resetting = false;

	if (has_fsm_encoding_none)
		return;

	if (wire->width <= 1) {
		if (has_fsm_encoding_attr) {
			log_warning("Removing fsm_encoding attribute from 1-bit net: %s.%s\n", wire->module, wire);
			wire->attributes.erase(ID::fsm_encoding);
		}
		return;
	}

	std::set<sig2driver_entry_t> cellport_list;
	sig2driver.find(RTLIL::SigSpec(wire), cellport_list);

	for (auto &cellport : cellport_list)
	{
		if ((cellport.first->type != ID($dff) && cellport.first->type != ID($adff)) || cellport.second != ID::Q)
			continue;

		muxtree_cells.clear();
		pool<Cell*> recursion_monitor;
		RTLIL::SigSpec sig_q = assign_map(cellport.first->getPort(ID::Q));
		RTLIL::SigSpec sig_d = assign_map(cellport.first->getPort(ID::D));
		dict<RTLIL::SigSpec, bool> mux_tree_cache;

		if (sig_q != assign_map(wire))
			continue;

		looks_like_state_reg = check_state_mux_tree(sig_q, sig_d, recursion_monitor, mux_tree_cache);
		looks_like_good_state_reg = lse || check_state_users(sig_q);
		if (lse && looks_like_state_reg && !lse_override_ok(sig_q, sig_d, cellport.first->type == ID($dff))) {
			log("Not marking %s.%s as FSM state register: its next state is not a case on itself with constant or hold overrides (LSE rule).\n", wire->module, wire);
			return;
		}

		if (!looks_like_state_reg)
			break;

		ConstEval ce(wire->module);

		std::set<sig2driver_entry_t> cellport_list;
		sig2user.find(sig_q, cellport_list);

		auto sig_q_bits = sig_q.to_sigbit_pool();

		for (auto &cellport : cellport_list)
		{
			RTLIL::Cell *cell = cellport.first;
			bool set_output = false, clr_output = false;

			if (cell->type.in(ID($ne), ID($reduce_or), ID($reduce_bool)))
				set_output = true;

			if (cell->type.in(ID($eq), ID($logic_not), ID($reduce_and)))
				clr_output = true;

			if (set_output || clr_output) {
				for (auto &port_it : cell->connections())
					if (cell->input(port_it.first))
						for (auto bit : assign_map(port_it.second))
							if (bit.wire != nullptr && !sig_q_bits.count(bit))
								goto next_cellport;
			}

			if (set_output || clr_output) {
				for (auto &port_it : cell->connections())
					if (cell->output(port_it.first)) {
						SigSpec sig = assign_map(port_it.second);
						Const val(set_output ? State::S1 : State::S0, GetSize(sig));
						ce.set(sig, val);
					}
			}
		next_cellport:;
		}

		SigSpec sig_y = sig_d, sig_undef;
		if (!ignore_self_reset) {
			if (cellport.first->type == ID($adff)) {
				SigSpec sig_arst = assign_map(cellport.first->getPort(ID::ARST));
				if (ce.eval(sig_arst, sig_undef))
					is_self_resetting = true;
			}
			else if (ce.eval(sig_y, sig_undef))
				is_self_resetting = true;
		}
	}

	if (has_fsm_encoding_attr)
	{
		vector<string> warnings;

		if (is_module_port)
			warnings.push_back("Forcing FSM recoding on module port might result in larger circuit.\n");

		if (!looks_like_good_state_reg)
			warnings.push_back("Users of state reg look like FSM recoding might result in larger circuit.\n");

		if (has_init_attr)
			warnings.push_back("Initialization value on FSM state register is ignored. Possible simulation-synthesis mismatch!\n");

		if (!looks_like_state_reg)
			warnings.push_back("Doesn't look like a proper FSM. Possible simulation-synthesis mismatch!\n");

		if (is_self_resetting)
			warnings.push_back("FSM seems to be self-resetting. Possible simulation-synthesis mismatch!\n");

		if (!warnings.empty()) {
			string warnmsg = stringf("Regarding the user-specified fsm_encoding attribute on %s.%s:\n", wire->module, wire);
			for (auto w : warnings) warnmsg += "    " + w;
			log_warning("%s", warnmsg);
		} else {
			log("FSM state register %s.%s already has fsm_encoding attribute.\n", wire->module, wire);
		}
	}
	else
	if (looks_like_state_reg && looks_like_good_state_reg && !has_init_attr && !is_module_port && !is_self_resetting)
	{
		log("Found FSM state register %s.%s.\n", wire->module, wire);
		wire->attributes[ID::fsm_encoding] = RTLIL::Const("auto");
	}
	else
	if (looks_like_state_reg)
	{
		log("Not marking %s.%s as FSM state register:\n", wire->module, wire);

		if (is_module_port)
			log("    Register is connected to module port.\n");

		if (!looks_like_good_state_reg)
			log("    Users of register don't seem to benefit from recoding.\n");

		if (has_init_attr)
			log("    Register has an initialization value.\n");

		if (is_self_resetting)
			log("    Circuit seems to be self-resetting.\n");
	}
}

struct FsmDetectPass : public Pass {
	FsmDetectPass() : Pass("fsm_detect", "finding FSMs in design") { }
	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    fsm_detect [options] [selection]\n");
		log("\n");
		log("This pass detects finite state machines by identifying the state signal.\n");
		log("The state signal is then marked by setting the attribute 'fsm_encoding'\n");
		log("on the state signal to \"auto\".\n");
		log("\n");
		log("    -ignore-self-reset\n");
		log("        Mark FSMs even if they are self-resetting\n");
		log("\n");
		log("    -lse\n");
		log("        Follow Lattice LSE: skip the check that recoding benefits the state\n");
		log("        register's users, and reject registers whose next-state value can be\n");
		log("        chosen between two non-constant values outside the state decoding.\n");
		log("\n");
		log("Existing 'fsm_encoding' attributes are not changed by this pass.\n");
		log("\n");
		log("Signals can be protected from being detected by this pass by setting the\n");
		log("'fsm_encoding' attribute to \"none\".\n");
		log("\n");
		log("This pass uses a subset of FF types to detect FSMs. Run 'opt -nosdff -nodffe'\n");
		log("before this pass to prepare the design for fsm_detect.\n");
		log("\n");
#ifdef YOSYS_ENABLE_VERIFIC
		log("The Verific frontend may optimize the design in a way that interferes with FSM\n");
		log("detection. Run 'verific -cfg db_infer_wide_muxes_post_elaboration 0' before\n");
		log("reading the source, and 'bmuxmap -pmux' after 'proc' for best results.\n");
		log("\n");
#endif
	}
	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		log_header(design, "Executing FSM_DETECT pass (finding FSMs in design).\n");

		bool ignore_self_reset = false, lse = false;

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++)
		{
			if (args[argidx] == "-ignore-self-reset") {
				ignore_self_reset = true;
				continue;
			}
			if (args[argidx] == "-lse") {
				lse = true;
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		CellTypes ct;
		ct.setup_internals();
		ct.setup_internals_anyinit();
		ct.setup_internals_mem();
		ct.setup_stdcells();
		ct.setup_stdcells_mem();

		for (auto mod : design->selected_modules())
		{
			module = mod;
			assign_map.set(module);

			sig2driver.clear();
			sig2user.clear();
			sig_at_port.clear();
			for (auto cell : module->cells())
				for (auto &conn_it : cell->connections()) {
					if (ct.cell_output(cell->type, conn_it.first) || !ct.cell_known(cell->type)) {
						RTLIL::SigSpec sig = conn_it.second;
						assign_map.apply(sig);
						sig2driver.insert(sig, sig2driver_entry_t(cell, conn_it.first));
					}
					if (!ct.cell_known(cell->type) || ct.cell_input(cell->type, conn_it.first)) {
						RTLIL::SigSpec sig = conn_it.second;
						assign_map.apply(sig);
						sig2user.insert(sig, sig2driver_entry_t(cell, conn_it.first));
					}
				}

			for (auto wire : module->wires())
				if (wire->port_id != 0)
					sig_at_port.add(assign_map(wire));

			for (auto wire : module->selected_wires())
				detect_fsm(wire, ignore_self_reset, lse);
		}

		assign_map.clear();
		sig2driver.clear();
		sig2user.clear();
		muxtree_cells.clear();
	}
} FsmDetectPass;

PRIVATE_NAMESPACE_END
