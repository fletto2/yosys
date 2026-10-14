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

#include "kernel/register.h"
#include "kernel/celltypes.h"
#include "kernel/rtlil.h"
#include "kernel/log.h"

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct SynthIce40Pass : public ScriptPass
{
	SynthIce40Pass() : ScriptPass("synth_ice40", "synthesis for iCE40 FPGAs") { }

	void help() override
	{
		//   |---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|---v---|
		log("\n");
		log("    synth_ice40 [options]\n");
		log("\n");
		log("This command runs synthesis for iCE40 FPGAs, mapping the design the way Lattice\n");
		log("LSE (Radiant/iCEcube2) does so the netlist comes out close to the vendor\n");
		log("flow's. Read the sources with 'read_verilog', which honours the vendor\n");
		log("'/* synthesis ... */' directives.\n");
		log("\n");
		log("    -device < hx | lp | u >\n");
		log("        the device family, for the cell models. default: hx\n");
		log("\n");
		log("    -top <module>\n");
		log("        use the specified module as top module\n");
		log("\n");
		log("    -blif <file>\n");
		log("        write the design to the specified BLIF file. writing of an output file\n");
		log("        is omitted if this parameter is not specified.\n");
		log("\n");
		log("    -edif <file>\n");
		log("        write the design to the specified EDIF file. writing of an output file\n");
		log("        is omitted if this parameter is not specified.\n");
		log("\n");
		log("    -json <file>\n");
		log("        write the design to the specified JSON file. writing of an output file\n");
		log("        is omitted if this parameter is not specified.\n");
		log("\n");
		log("    -run <from_label>:<to_label>\n");
		log("        only run the commands between the labels (see below). an empty\n");
		log("        from label is synonymous to 'begin', and empty to label is\n");
		log("        synonymous to the end of the command list.\n");
		log("\n");
		log("    -flatten\n");
		log("        flatten the design before synthesis. LSE keeps the module boundaries,\n");
		log("        so by default the design is not flattened\n");
		log("\n");
		log("    -noflatten\n");
		log("        do not flatten design before synthesis (the default)\n");
		log("\n");

		log("    -nocarry\n");
		log("        do not use SB_CARRY cells in output netlist\n");
		log("\n");
		log("    -nodffe\n");
		log("        do not use SB_DFFE* cells in output netlist\n");
		log("\n");
		log("    -dffe_min_ce_use <min_ce_use>\n");
		log("        do not use SB_DFFE* cells if the resulting CE line would go to less\n");
		log("        than min_ce_use SB_DFFE* in output netlist\n");
		log("\n");
		log("    -nobram\n");
		log("        do not use SB_RAM40_4K* cells in output netlist\n");
		log("\n");
		log("    -spram\n");
		log("        enable automatic inference of SB_SPRAM256KA\n");
		log("\n");
		log("    -dsp\n");
		log("        use iCE40 UltraPlus DSP cells for large arithmetic (the default)\n");
		log("\n");
		log("    -nodsp\n");
		log("        do not use DSP cells\n");
		log("\n");
		log("    -noabc\n");
		log("        use built-in Yosys LUT techmapping instead of ABC\n");
		log("\n");
		log("    -vpr\n");
		log("        generate an output netlist (and BLIF file) suitable for VPR\n");
		log("        (this feature is experimental and incomplete)\n");
		log("\n");

		log("    -latches <info|warn|error>\n");
		log("        select the behaviour for latches that cannot be mapped to a\n");
		log("        dedicated hardware primitive and are implemented using LUTs\n");
		log("        instead. 'error' (the default) aborts synthesis, 'warn' only\n");
		log("        prints a warning, and 'info' permits them with an info-level message.\n");
		log("\n");
		log("\n");
		log("The following commands are executed by this synthesis command:\n");
		help_script();
		log("\n");
	}

	string top_opt, blif_file, edif_file, json_file, device_opt, latches;
	bool nocarry, nodffe, nobram, spram, dsp, flatten, noabc, vpr;
	int min_ce_use;

	void clear_flags() override
	{
		top_opt = "-auto-top";
		blif_file = "";
		edif_file = "";
		json_file = "";
		nocarry = false;
		nodffe = false;
		min_ce_use = 3;   // LSE leaves enables used by one or two flip-flops in the logic
		nobram = false;
		spram = false;
		dsp = true;
		flatten = false;
		noabc = false;
		vpr = false;
		device_opt = "hx";
		latches = "error";
	}

	void execute(std::vector<std::string> args, RTLIL::Design *design) override
	{
		string run_from, run_to;
		clear_flags();

		size_t argidx;
		for (argidx = 1; argidx < args.size(); argidx++)
		{
			if (args[argidx] == "-top" && argidx+1 < args.size()) {
				top_opt = "-top " + args[++argidx];
				continue;
			}
			if (args[argidx] == "-blif" && argidx+1 < args.size()) {
				blif_file = args[++argidx];
				continue;
			}
			if (args[argidx] == "-edif" && argidx+1 < args.size()) {
				edif_file = args[++argidx];
				continue;
			}
			if (args[argidx] == "-json" && argidx+1 < args.size()) {
				json_file = args[++argidx];
				continue;
			}
			if (args[argidx] == "-run" && argidx+1 < args.size()) {
				size_t pos = args[argidx+1].find(':');
				if (pos == std::string::npos)
					break;
				run_from = args[++argidx].substr(0, pos);
				run_to = args[argidx].substr(pos+1);
				continue;
			}
			if (args[argidx] == "-flatten") {
				flatten = true;
				continue;
			}
			if (args[argidx] == "-noflatten") {
				flatten = false;
				continue;
			}
			if (args[argidx] == "-retime") {
				// removed, retiming is not supported with ABC9
				continue;
			}
			if (args[argidx] == "-relut") {
				// removed, opt_lut is always run
				continue;
			}
			if (args[argidx] == "-nocarry") {
				nocarry = true;
				continue;
			}
			if (args[argidx] == "-nodffe") {
				nodffe = true;
				continue;
			}
			if (args[argidx] == "-dffe_min_ce_use" && argidx+1 < args.size()) {
				min_ce_use = atoi(args[++argidx].c_str());
				continue;
			}
			if (args[argidx] == "-nobram") {
				nobram = true;
				continue;
			}
			if (args[argidx] == "-spram") {
				spram = true;
				continue;
			}
			if (args[argidx] == "-dsp") {
				dsp = true;
				continue;
			}
			if (args[argidx] == "-nodsp") {
				dsp = false;
				continue;
			}
			if (args[argidx] == "-noabc") {
				noabc = true;
				continue;
			}
			if (args[argidx] == "-vpr") {
				vpr = true;
				continue;
			}
			if (args[argidx] == "-abc9") {
				// removed, ABC9 is used if not -noabc
				continue;
			}
			if (args[argidx] == "-noabc9") {
				// removed, ABC9 is used if not -noabc
				continue;
			}
			if (args[argidx] == "-device" && argidx+1 < args.size()) {
				device_opt = args[++argidx];
				continue;
			}
			if (args[argidx] == "-no-rw-check") {
				// always on: LSE builds no read-during-write bypass logic
				continue;
			}
			if (args[argidx] == "-latches" && argidx+1 < args.size()) {
				latches = args[++argidx];
				continue;
			}
			break;
		}
		extra_args(args, argidx, design);

		if (!design->full_selection())
			log_cmd_error("This command only operates on fully selected designs!\n");
		if (device_opt != "hx" && device_opt != "lp" && device_opt !="u")
			log_cmd_error("Invalid or no device specified: '%s'\n", device_opt);
		if (latches != "info" && latches != "warn" && latches != "error")
			log_cmd_error("Invalid value '%s' for -latches (expected info, warn or error)\n", latches.c_str());

		log_header(design, "Executing SYNTH_ICE40 pass.\n");
		log_push();

		run_script(design, run_from, run_to);

		log_pop();
	}

	void script() override
	{
		std::string define;
		if (device_opt == "lp")
			define = "-D ICE40_LP";
		else if (device_opt == "u")
			define = "-D ICE40_U";
		else
			define = "-D ICE40_HX";
		// LSE builds no read-during-write bypass logic for inferred RAMs
		std::string no_rw_check_opt = " -no-rw-check";

		if (check_label("begin"))
		{
			run("read_verilog " + define + " -lib -specify +/ice40/cells_sim.v");
			run(stringf("hierarchy -check %s", help_mode ? "-top <top>" : top_opt));
			run("proc -latches " + (latches == "info" ? std::string("info") : std::string("warn")));
			// before opt_merge shares a compared multiplexer with an identical one
			run("ice40_lsemuxcmp");
			// identical drivers of one net (e.g. a duplicated assign) are one driver, as for LSE
			run("opt_merge");
		}

		if (check_label("flatten"))
		{
			if (flatten || help_mode) {
				run("check", "                 (if -flatten)");
				run("flatten", "               (if -flatten)");
			}
			// with or without flattening: a `x ? y : 'bz' on a top-level inout must
			// stay a tristate buffer for the pad, not become plain logic
			run("tribuf -logic");
			run("deminout");
		}

		if (check_label("coarse"))
		{
			run("ice40_lseconst");
			run("ice40_lseaddenable");
			run("opt_expr");
			run("opt_clean");
			run("check");
			run("opt -nodffe -nosdff");
			run("fsm -lse");
			// LSE builds case statements as binary mux trees on the select bits
			// (after fsm, which needs the original $pmux structure)
			run("pmux2shiftx");
			run("pmuxtree");
			run("opt");
			run("wreduce");
			run("peepopt");
			run("opt_clean");
			run("share");
			run("ice40_lsecmp");
			run("techmap -map +/cmp2lut.v -D LUT_WIDTH=4");
			run("opt_expr");
			run("opt_clean");
			// LSE takes the read address register into the RAM rather than a data
			// register with a reset; also reserves memory port registers before ice40_dsp
			run("memory_dff -lse" + no_rw_check_opt);
			if (help_mode || dsp) {
				run("wreduce t:$mul");
				run("techmap -map +/mul2dsp.v -map +/ice40/dsp_map.v -D DSP_A_MAXWIDTH=16 -D DSP_B_MAXWIDTH=16 "
						"-D DSP_A_MINWIDTH=2 -D DSP_B_MINWIDTH=2 -D DSP_Y_MINWIDTH=11 "
						"-D DSP_NAME=$__MUL16X16", "(if -dsp)");
				run("select a:mul2dsp", "              (if -dsp)");
				run("setattr -unset mul2dsp", "        (if -dsp)");
				run("opt_expr -fine", "                (if -dsp)");
				run("wreduce", "                       (if -dsp)");
				run("select -clear", "                 (if -dsp)");
				run("ice40_dsp", "                     (if -dsp)");
				run("chtype -set $mul t:$__soft_mul", "(if -dsp)");
			}
			run("alumacc");
			// LSE adds several operands with a chain of two-input carry adders, not a
			// carry-save tree
			run("maccmap -unmap");
			run("techmap -max_iter 1 -map +/techmap.v t:$add t:$sub");
			run("opt_clean");
			run("opt");
			run("memory -nomap" + no_rw_check_opt);
			run("opt_clean");
		}

		if (check_label("map_ram"))
		{
			std::string args = "";
			if (help_mode)
				args += " [-no-auto-huge] [-no-auto-block]";
			else {
				if (!spram)
					args += " -no-auto-huge";
				if (nobram)
					args += " -no-auto-block";
			}
			run("memory_libmap -lib +/ice40/brams.txt -lib +/ice40/spram.txt" + args, "(-no-auto-huge unless -spram, -no-auto-block if -nobram)");
			run("techmap -D LSE -map +/ice40/brams_map.v -map +/ice40/spram_map.v");
			run("ice40_braminit");
		}

		if (check_label("map_ffram"))
		{
			run("opt -fast -mux_undef -undriven -fine");
			run("memory_map");
			run("opt -undriven -fine");
		}

		if (check_label("map_gates"))
		{
			run("ice40_lsealu");
			if (nocarry)
				run("techmap");
			else {
				run("ice40_wrapcarry");
				run("techmap -D LSE -map +/techmap.v -map +/ice40/arith_map.v");
			}
			run("opt -fast");
			run("ice40_opt");
		}

		if (check_label("map_ffs"))
		{
			if (nodffe)
				run(stringf("dfflegalize -cell $_DFF_?_ 0 -cell $_DFF_?P?_ 0 -cell $_SDFF_?P?_ 0 -cell $_DLATCH_?_ x"));
			else
				run(stringf("dfflegalize -cell $_DFF_?_ 0 -cell $_DFFE_?P_ 0 -cell $_DFF_?P?_ 0 -cell $_DFFE_?P?P_ 0 -cell $_SDFF_?P?_ 0 -cell $_SDFFCE_?P?P_ 0 -cell $_DLATCH_?_ x -mince %d", min_ce_use));
			run("techmap -map +/ice40/ff_map.v");
			run("opt_expr -mux_undef");
			run("simplemap");
			run("ice40_opt -full");
			run("ice40_lseramce");
		}

		if (check_label("map_luts"))
		{
			if (latches == "error" || help_mode)
				run("check -latchonly -assert", "(only if -latches error, the default)");
			run("techmap -map +/ice40/latches_map.v");
			if (noabc || help_mode) {
				run("simplemap", "                               (if -noabc)");
				if (noabc || help_mode)
					run("techmap -map +/gate2lut.v -D LUT_WIDTH=4", "(only if -noabc)");
			}
			if (!noabc || help_mode) {
				// LSE builds 4:1 multiplexers from two LUT4s
				run("ice40_lsemuxchain", "                (unless -noabc)");
				run("muxcover -mux4=300", "                (unless -noabc)");
				run("ice40_lsereduce", "                 (unless -noabc)");
				run("techmap -map +/ice40/lse_mux_map.v", "(unless -noabc)");
				// LSE duplicates small logic into its readers rather than sharing it, so
				// no area recovery. A kept net (syn_keep) is a LUT output for LSE: map
				// the logic in front of it first, so the rest reads the net itself
				run("select -set lse_keep w:* a:keep %i %cie* t:$_* %i", "(unless -noabc)");
				run("abc -lut 4 -script +strash;dch,-f;if,-K,4,-F,0,-A,0;mfs2 @lse_keep", "(unless -noabc)");
				run("abc -lut 4 -script +strash;dch,-f;if,-K,4,-F,0,-A,0;mfs2", "(unless -noabc)");
			}
			run("ice40_wrapcarry -unwrap");
			run("techmap -map +/ice40/ff_map.v");
			run("clean");
			// no opt_lut: LSE never merges other logic into a carry LC's LUT
		}

		if (check_label("map_cells"))
		{
			if (help_mode)
				run("techmap -map +/ice40/cells_map.v", "(skip if -vpr)");
			else if (!vpr)
				run("techmap -map +/ice40/cells_map.v");
			run("clean");
		}

		if (check_label("polarity")) {
			run("ice40_lsemuxsel");
			run("ice40_lsepolarity");
		}

		if (check_label("map_iopads"))
			run("ice40_iopack");

		if (check_label("check"))
		{
			run("autoname");
			run("hierarchy -check");
			run("stat");
			run("check -noinit");
			run("blackbox =A:whitebox");
		}

		if (check_label("blif"))
		{
			if (!blif_file.empty() || help_mode) {
				if (vpr || help_mode) {
					run(stringf("opt_clean -purge"),
							"                                 (vpr mode)");
					run(stringf("write_blif -attr -cname -conn -param %s",
							help_mode ? "<file-name>" : blif_file.c_str()),
							" (vpr mode)");
				}
				if (!vpr)
					run(stringf("write_blif -gates -attr -param %s",
							help_mode ? "<file-name>" : blif_file.c_str()),
							"       (non-vpr mode)");
			}
		}

		if (check_label("edif"))
		{
			if (!edif_file.empty() || help_mode)
				run(stringf("write_edif %s", help_mode ? "<file-name>" : edif_file));
		}

		if (check_label("json"))
		{
			if (!json_file.empty() || help_mode)
				run(stringf("write_json %s", help_mode ? "<file-name>" : json_file));
		}
	}
} SynthIce40Pass;

PRIVATE_NAMESPACE_END
