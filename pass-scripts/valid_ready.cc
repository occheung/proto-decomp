#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/ff.h"
#include "graph.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


bool PASS_DEBUG = 0;
#define LOG(...) {if (PASS_DEBUG) { log(__VA_ARGS__); } else {}}

struct ValidReadyPass : public Pass {
    ValidReadyPass() : Pass("valid_ready") { }

    void execute(vector<string> args, Design *design) override {
        bool experimental = false;
        if (args.size() > 1) {
            if (args[1] == "-debug") {
                PASS_DEBUG = 1;
            } else {
                PASS_DEBUG = 0;
                if (args[1] == "-exp") {
                    experimental = true;
                }
            }
        } else {
            PASS_DEBUG = 0;
        }

        RTLIL::Module* top = design->top_module();

        if (top == nullptr) {
            log_error("No top module found!\n");
            return;
        }

        const SigMap sigmap(top);

        CircuitGraph circuit_graph(sigmap, top);

        FfInitVals ff_init_vals(&sigmap, top);

        // Find all FFs that has enable pins
        std::set<RTLIL::Cell*> ff_with_enable;
        for (RTLIL::Cell* cell: top->cells()) {
            if (RTLIL::builtin_ff_cell_types().count(cell->type)) {
                FfData ff_data(&ff_init_vals, cell);
                if (ff_data.has_ce) {
                    ff_with_enable.insert(cell);
                }
            }
        }

        // Select FFs that has a feedback path
        // Some Q pins must eventually feed to a D pin of the same cell
        std::set<RTLIL::Cell*> self_loop_dffe;
        for (RTLIL::Cell* ff_cell: ff_with_enable) {
            bool has_path = false;
            int port_size = GetSize(ff_cell->getPort("\\D"));
            for (int src_pin_idx = 0; src_pin_idx < port_size; ++src_pin_idx) {
                for (int sink_pin_idx = 0; sink_pin_idx < port_size; ++sink_pin_idx) {
                    has_path = !(circuit_graph.get_intermediate_comb_cells(
                        {ff_cell, "\\Q", src_pin_idx}, {ff_cell, "\\D", sink_pin_idx}).empty());
                    if (has_path) {
                        self_loop_dffe.insert(ff_cell);
                        break;
                    }
                }

                if (has_path) {
                    break;
                }
            }
        }

        // Consider each self looping register:
        // Could bits in the same register influence other bits?
        std::set<RTLIL::Cell*> independent_dffes;
        for (RTLIL::Cell* dffe: self_loop_dffe) {
            int dffe_width = GetSize(dffe->getPort("\\Q"));

            bool cross_influence = false;
            for (int i = 0; i < dffe_width; ++i) {
                for (int j = 0; j < dffe_width; ++j) {
                    if (i == j) {
                        // We do not care about direct paths (Q[i] -> D[i])
                        continue;
                    }
                    // Find a path from Q[i] to D[j]
                    std::set<RTLIL::Cell*> connected_cells = circuit_graph.get_intermediate_comb_cells(
                        {dffe, "\\Q", i},
                        {dffe, "\\D", j}
                    );
                    if (!connected_cells.empty()) {
                        cross_influence = true;
                        break;
                    }
                }

                if (cross_influence) {
                    break;
                }
            }

            if (!cross_influence && dffe_width > 1) {
                LOG("DFFE %s is not a FSM\n", log_id(dffe));
                independent_dffes.insert(dffe);
            }
        }

        for (RTLIL::Cell* independent_dffe: independent_dffes) {
            self_loop_dffe.erase(independent_dffe);
        }

        // Find data registers
        // TODO: Define data registers as registers with CE but not classified as FSMs
        std::set<RTLIL::Cell*> data_regs;
        for (RTLIL::Cell* ff: ff_with_enable) {
            if (self_loop_dffe.count(ff) == 0) {
                data_regs.insert(ff);
                log("Data register: %s\n", log_id(ff));
            }
        }

        // Get MUXes. These are potentially routing data.
        std::set<RTLIL::Cell*> muxes;
        for (RTLIL::Cell* cell: top->cells()) {
            if (cell->type == "$_MUX_") {
                muxes.insert(cell);
            }
        }

        for (RTLIL::Cell* ff_cell: self_loop_dffe) {
            log("Filtered FF Cell: %s; FF type: %s\n", ff_cell->name.c_str(), ff_cell->type.c_str());
        }

        // Highlighted AND gate collections
        // Collect AND gate that fans into a FSM without SCC
        // The fan-ins of the AND gate are likely valid and ready bits
        std::set<std::pair<CellPin, CellPin>> and_gates_inputs;

        // Store potential valid/ready bits of the entire circuit
        std::set<CellPin> ctrl_candidates;

        // Properly group up the control bits by DFFE directional edges
        dict<std::pair<RTLIL::Cell*, RTLIL::Cell*>, std::set<CellPin>> dff_loop_to_ctrl_pin_map;

        // Find the strongly connected component that involves the D,E pins
        for (RTLIL::Cell* dffe: self_loop_dffe) {
            log("Suspected state register: %s\n", dffe->name.c_str());
            std::set<RTLIL::Cell*> scc_set;
            int port_size = GetSize(dffe->getPort("\\Q"));
            for (int bit_idx = 0; bit_idx < port_size; ++bit_idx) {
                std::set<RTLIL::Cell*> bit_scc_set = circuit_graph.get_intermediate_comb_cells(
                    {dffe, "\\Q", bit_idx}, {dffe, "\\EN", 0});
                scc_set.insert(bit_scc_set.begin(), bit_scc_set.end());
            }

            // Remove dffe from the collected set:
            // We are not interested at it
            scc_set.erase(dffe);

            // If the SCC is empty, and EN is connected to an AND gate, add the AND gate
            // However, should ce_over_srst be enabled, there should be an OR gate
            // that routes the SRST signal to CE through it, else SRST is disabled
            // unconditionally.
            if (scc_set.empty()) {
                // Find the FfData
                Source en_src;
                FfData dffe_data(&ff_init_vals, dffe);
                if (dffe_data.has_srst && dffe_data.ce_over_srst) {
                    const Source& ce_src = circuit_graph.source_map[dffe]["\\EN"][0];
                    if (std::holds_alternative<RTLIL::SigBit>(ce_src)) {
                        // FF cannot be a state register of interest if its CE is
                        // solely tied to const or external
                        continue;
                    }
                    RTLIL::Cell* ce_src_or_cell = std::get<0>(std::get<CellPin>(ce_src));
                    if (ce_src_or_cell->type != ID($_OR_)) {
                        // FIXME: De Morgan
                        continue;
                    }
                    const Source& srst_src = circuit_graph.source_map[dffe]["\\SRST"][0];
                    const Source& or_a = circuit_graph.source_map[ce_src_or_cell]["\\A"][0];
                    const Source& or_b = circuit_graph.source_map[ce_src_or_cell]["\\B"][0];

                    // One _OR_ source must match SRST source
                    if (srst_src == or_a) {
                        en_src = or_b;
                    } else if (srst_src == or_b) {
                        en_src = or_a;
                    } else {
                        // FIXME: Pin logic ordering...
                        continue;
                    }
                } else {
                    en_src = circuit_graph.source_map[dffe]["\\EN"][0];
                }
                // const Source& en_src = circuit_graph.source_map[dffe]["\\EN"][0];
                if (std::holds_alternative<CellPin>(en_src)) {
                    RTLIL::Cell* en_src_cell = std::get<0>(std::get<CellPin>(en_src));
                    if (en_src_cell->type == ID($_AND_)) {
                        LOG("DFFE does not have a SCC, but it has a direct AND gate\n");
                        scc_set.insert(en_src_cell);

                        // Store the AND gate's fan in logics, if both of them are connected pins
                        // We do not consider the case where valid/ready bits are undriven (external)
                        const Source& a_src = circuit_graph.source_map[en_src_cell]["\\A"][0];
                        const Source& b_src = circuit_graph.source_map[en_src_cell]["\\B"][0];

                        if (std::holds_alternative<CellPin>(a_src) && std::holds_alternative<CellPin>(b_src)) {
                            and_gates_inputs.insert(std::make_pair(
                                std::get<CellPin>(a_src),
                                std::get<CellPin>(b_src)));
                            LOG("Adding Cell %s Port %s, Cell %s Port %s into AND check list\n",
                                std::get<0>(std::get<CellPin>(a_src))->name.c_str(),
                                std::get<1>(std::get<CellPin>(a_src)).c_str(),
                                std::get<0>(std::get<CellPin>(b_src))->name.c_str(),
                                std::get<1>(std::get<CellPin>(b_src)).c_str()
                            );
                        }
                    }
                }
            }

            for (RTLIL::Cell* scc_cell: scc_set) {
                LOG("SCC Cell %s, type: %s\n", scc_cell->name.c_str(), scc_cell->type.c_str());
            }

            // Find the drivers of the SCC set
            // Start from the EN pin, trace the source until one of the source
            // does not belong to the SCC set
            // Record these sources
            dict<CellPin, std::set<CellPin>> non_scc_sources;
            for (RTLIL::Cell* scc_cell: scc_set) {
                for (const auto& [src_port, src_vec]: circuit_graph.source_map[scc_cell]) {
                    for (const Source& src: src_vec) {
                        // Reject constants
                        // FIXME: There should not be external pins
                        if (std::holds_alternative<RTLIL::SigBit>(src)) {
                            continue;
                        }

                        CellPin src_pin = std::get<CellPin>(src);
                        RTLIL::Cell* src_cell = std::get<0>(src_pin);
                        if (scc_set.count(src_cell) == 0) {
                            non_scc_sources[src_pin] = {};
                        }
                    }
                }
            }

            // Filter off the dffe Q pins
            {
                int q_port_size = GetSize(dffe->getPort("\\Q"));
                for (int pin_idx = 0; pin_idx < q_port_size; ++pin_idx) {
                    non_scc_sources.erase({dffe, "\\Q", pin_idx});
                }
            }

            // Compute the intermediate nodes for all paths from any sources
            // to port E
            for (auto src_pin_it = non_scc_sources.begin(); src_pin_it != non_scc_sources.end(); ++src_pin_it) {
                const auto& [src_cell, src_port, _pin_idx] = src_pin_it->first;
                LOG("Cell: %s; Port: %s\n", src_cell->name.c_str(), src_port.c_str());
                std::set<CellPin> dominated_pins = circuit_graph.get_dominated_pins(src_pin_it->first, {dffe, "\\EN", 0});

                // Remove the dffe pin. We know for sure it is in the domination set.
                dominated_pins.erase({dffe, "\\EN", 0});
                // Remove the source itself. It does not help solving the dominance problem.
                dominated_pins.erase(src_pin_it->first);

                for (const CellPin& pin: dominated_pins) {
                    const auto& [dom_cell, dom_port, _dom_pin_idx] = pin;
                    LOG("Dominated pin: Cell: %s; Port: %s\n", dom_cell->name.c_str(), dom_port.c_str());
                }

                src_pin_it->second = dominated_pins;
            }

            // Compute dominance for the SCC to aproximate the input of the FSM
            if (experimental) {
                std::set<CellPin> non_scc_drivers;
                for (const auto& [driver, _non_driver]: non_scc_sources) {
                    non_scc_drivers.insert(driver);
                }
                std::set<CellPin> fsm_inputs = circuit_graph.get_non_dominated_pins(non_scc_drivers);
                log("Inputs:\n");
                for (const auto& [in_cell, in_pin_name, in_pin_idx]: fsm_inputs) {
                    log("Cell %s Pin %s:%d\n", in_cell->name.c_str(), in_pin_name.c_str(), in_pin_idx);
                }

                std::set<CellPin> fsm_state_bits;
                for (int i = 0; i < GetSize(dffe->getPort("\\Q")); ++i) {
                    fsm_state_bits.insert({dffe, "\\Q", i});
                }

                std::set<CellPin> fsm_outputs = circuit_graph.get_dominated_frontier(
                    fsm_state_bits,
                    fsm_inputs
                );

                log("Outputs:\n");
                for (const auto& [out_cell, out_pin_name, out_pin_idx]: fsm_outputs) {
                    log("Cell %s Pin %s:%d\n", out_cell->name.c_str(), out_pin_name.c_str(), out_pin_idx);
                }
            }

            // For each potential source pin of the FSM, find if it is dominated
            std::set<CellPin> non_dominated_sources;
            for (const auto& [src_pin, _dom_set]: non_scc_sources) {
                std::map<IdString, bool> pin_dominance;
                for (const auto& [port, _sig_spec]: std::get<0>(src_pin)->connections()) {
                    if (std::get<0>(src_pin)->input(port)) {
                        pin_dominance[port] = false;
                    }
                }

                // Look up every other sources, see if this source is dominated
                for (const auto& [other_src_pin, other_dom_set]: non_scc_sources) {
                    if (src_pin == other_src_pin) {
                        continue;
                    }

                    // The source is dominated if every input of the cell is in
                    // some dominance cell
                    for (const auto& [dom_cell, dom_port, _dom_pin_idx]: other_dom_set) {
                        if (dom_cell == std::get<0>(src_pin)) {
                            pin_dominance[dom_port] = true;
                        }
                    }
                }

                LOG("Cell %s\n", std::get<0>(src_pin)->name.c_str());
                for (const auto& [port, dominated]: pin_dominance) {
                    LOG("Port %s dominance: %s\n", port.c_str(), dominated? "true": "false");
                    if (!dominated) {
                        non_dominated_sources.insert(src_pin);
                    }
                }
            }

            for (const auto& [cell, port, _idx]: non_dominated_sources) {
                LOG("Non dominated cell: %s; Port: %s\n", cell->name.c_str(), port.c_str());
            }

            // Attempt to find a path that goes through exactly 1 DFF
            // We can assume that valid does not form a combinatorial path with ready
            // Valid-gating of ready is the only sensible exception
            // (enough that I'm aware of)
            //
            // Ready bit should react 1 cycle later than valid bit
            std::set<RTLIL::Cell*> dffe_sources = circuit_graph.dff_source_graph[dffe];
            std::set<RTLIL::Cell*> dffe_sinks = circuit_graph.dff_sink_graph[dffe];

            std::set<RTLIL::Cell*> viable_intermediate_dffes;
            for (RTLIL::Cell* dffe_src: dffe_sources) {
                // Not interested in self-loop 1-hop
                if (dffe_src == dffe) {
                    continue;
                }

                // Not interested in intermediate DFF that lacks clock enable
                // We hypothesized the valid/ready bits shoudl drive the enable pins
                FfData ff_data(&ff_init_vals, dffe_src);
                if (!ff_data.has_ce) {
                    continue;
                }

                // The looped-in DFFE should also control a valid/ready bit
                // by symmetry
                if (self_loop_dffe.count(dffe_src) == 0) {
                    continue;
                }

                if (dffe_sinks.count(dffe_src)) {
                    viable_intermediate_dffes.insert(dffe_src);
                }
            }

            log("Viable 1-hop intermediate DFFE:\n");
            for (RTLIL::Cell* inter_dffe: viable_intermediate_dffes) {
                log("%s\n", inter_dffe->name.c_str());
            }

            if (experimental) {
                // Find the intersection of the loop path and the DFF inputs
                for (const CellPin& connected_input_bit: non_dominated_sources) {
                    for (RTLIL::Cell* inter_dffe: viable_intermediate_dffes) {
                        bool found_path = false;
                        for (int i = 0; i < GetSize(inter_dffe->getPort("\\Q")); ++i) {
                            if (!circuit_graph.get_intermediate_comb_cells(
                                {inter_dffe, "\\Q", i},
                                connected_input_bit
                            ).empty()) {
                                found_path = true;
                            } 
                        }
                        
                        if (found_path) {
                            // Mark the bit, exclude this input in output ctrl bit finding
                            std::set<CellPin> adjusted_fsm_inputs = non_dominated_sources;
                            adjusted_fsm_inputs.erase(connected_input_bit);

                            std::set<CellPin> fsm_state_bits;
                            for (int i = 0; i < GetSize(dffe->getPort("\\Q")); ++i) {
                                fsm_state_bits.insert({dffe, "\\Q", i});
                            }

                            std::set<CellPin> fsm_outputs = circuit_graph.get_dominated_frontier(
                                fsm_state_bits,
                                adjusted_fsm_inputs
                            );

                            log("Guess cell %s port %s index %d as input connection\n",
                                log_id(std::get<0>(connected_input_bit)),
                                log_id(std::get<1>(connected_input_bit)),
                                std::get<2>(connected_input_bit));
                            // for (const auto& [out_cell, out_pin_name, out_pin_idx]: fsm_outputs) {
                            //     log("Cell %s Pin %s:%d\n", out_cell->name.c_str(), out_pin_name.c_str(), out_pin_idx);
                            // }
                            // log("Guess ends\n\n");

                            // See if any eligible output pins complete the path
                            for (CellPin fsm_output: fsm_outputs) {
                                const auto& [out_cell, out_pin_name, out_pin_idx] = fsm_output;
                                log("Cell %s Pin %s:%d ", out_cell->name.c_str(), out_pin_name.c_str(), out_pin_idx);
                                std::set<RTLIL::Cell*> half_path_set = circuit_graph.get_intermediate_comb_cells(
                                    fsm_output, {inter_dffe, "\\EN", 0});
                                if (!half_path_set.empty()) {
                                    // Check if the output pin drives some other output pins
                                    // that also belongs to the path
                                    bool primary_output = true;
                                    for (Sink output_sink: circuit_graph.sink_map.at(out_cell).at(out_pin_name).at(out_pin_idx)) {
                                        if (std::holds_alternative<RTLIL::SigBit>(output_sink)) {
                                            continue;
                                        }

                                        RTLIL::Cell* succ_cell = std::get<0>(std::get<CellPin>(output_sink));
                                        bool cell_controls_output = false;
                                        for (CellPin other_out_pin: fsm_outputs) {
                                            if (std::get<0>(other_out_pin) == succ_cell) {
                                                cell_controls_output = true;
                                                break;
                                            }
                                        }
                                        if (cell_controls_output && half_path_set.count(succ_cell)) {
                                            primary_output = false;
                                            break;
                                        }
                                    }
                                    if (primary_output) {
                                        log("PRIMARILY ");
                                    }
                                    log("forms a complete path\n");
                                } else {
                                    log("does NOT form a complete path\n");
                                }
                            }
                        }
                    }
                }
            }
            
            // Determine which non-SCC source is related to ready bit
            dict<CellPin, std::set<RTLIL::Cell*>> reg_ctrl_candidates;
            for (const CellPin& potential_rdy_bit: non_dominated_sources) {
                std::set<RTLIL::Cell*> cell_intersections;
                dict<CellPin, std::set<RTLIL::Cell*>> rdy_srcs; 
                for (RTLIL::Cell* inter_dffe: viable_intermediate_dffes) {
                    // Try all Q pins
                    int q_port_size = GetSize(inter_dffe->getPort("\\Q"));
                    for (int q_idx = 0; q_idx < q_port_size; ++q_idx) {
                        std::set<RTLIL::Cell*> cells_in_path = circuit_graph.get_intermediate_comb_cells(
                            {inter_dffe, "\\Q", q_idx}, potential_rdy_bit);
                        cell_intersections.insert(cells_in_path.begin(), cells_in_path.end());
                        if (!cells_in_path.empty()) {
                            if (rdy_srcs.count(potential_rdy_bit) == 0) {
                                rdy_srcs[potential_rdy_bit] = {};
                            }
                            rdy_srcs[potential_rdy_bit].insert(inter_dffe);

                            std::pair<RTLIL::Cell*, RTLIL::Cell*> directed_edge = {inter_dffe, dffe};
                            if (dff_loop_to_ctrl_pin_map.count(directed_edge) == 0) {
                                dff_loop_to_ctrl_pin_map[directed_edge] = {};
                            }
                            dff_loop_to_ctrl_pin_map[directed_edge].insert(potential_rdy_bit);
                        }
                    }
                }
                if (!cell_intersections.empty()) {
                    // reg_ctrl_candidates.insert(potential_rdy_bit);
                    reg_ctrl_candidates[potential_rdy_bit] = rdy_srcs[potential_rdy_bit];
                }
            }

            // Save to global set
            for (const auto& [ctrl_bit, _inter_dff_set]: reg_ctrl_candidates) {
                ctrl_candidates.insert(ctrl_bit);
            }

            // Save the candidates to a yosys set
            log("Found %ld control bit candidate\n", reg_ctrl_candidates.size());
            for (const auto& [candidate_pin, inter_dff_set]: reg_ctrl_candidates) {
                log("Cell %s; Port %s from DFF ", std::get<0>(candidate_pin)->name.c_str(), std::get<1>(candidate_pin).c_str());
                for (const RTLIL::Cell* dff: inter_dff_set) {
                    log("%s, ", dff->name.c_str());
                }
                log("\n");
            }
            log("\n");
        }

        // Process AND gates that feed into registers EN port,
        // while the corresponding register does not have a SCC
        std::set<CellPin> additional_ctrls;
        for (const auto& [a_pin, b_pin]: and_gates_inputs) {
            // TODO: Do I worry about potential self-inference?
            if (ctrl_candidates.count(a_pin)) {
                additional_ctrls.insert(b_pin);
                LOG("Found Cell %s Port %s in candidates; inferring Cell %s Port %s\n", 
                    std::get<0>(a_pin)->name.c_str(), std::get<1>(a_pin).c_str(),
                    std::get<0>(b_pin)->name.c_str(), std::get<1>(b_pin).c_str());
            } else if (ctrl_candidates.count(b_pin)) {
                additional_ctrls.insert(a_pin);
            }
        }

        ctrl_candidates.insert(additional_ctrls.begin(), additional_ctrls.end());
        log("Found %ld control bits in total\n", ctrl_candidates.size());
        for (const CellPin& candidate_pin: ctrl_candidates) {
            log("Cell %s; Port %s\n", std::get<0>(candidate_pin)->name.c_str(), std::get<1>(candidate_pin).c_str());
        }

        // Eliminate directed edges that has multiple control bits
        std::set<std::pair<RTLIL::Cell*, RTLIL::Cell*>> directed_edges_to_erase;
        for (auto [directed_edge, ctrl_bits]: dff_loop_to_ctrl_pin_map) {
            if (ctrl_bits.size() > 1) {
                directed_edges_to_erase.insert(directed_edge);
            }
        }
        for (std::pair<RTLIL::Cell*, RTLIL::Cell*> directed_edge: directed_edges_to_erase) {
            auto [dffe_src, dffe_sink] = directed_edge;
            dff_loop_to_ctrl_pin_map.erase(directed_edge);
            LOG("Remove directed edge %s -> %s\n", log_id(dffe_src), log_id(dffe_sink));
        }

        // Try to promote inferred AND pins to replace control bits
        // This is to improve positional precision
        log("Provisional ctrl pin mappings:\n");
        for (auto& [directed_edge, ctrl_bits]: dff_loop_to_ctrl_pin_map) {
            auto [dffe_src, dffe_sink] = directed_edge;
            for (CellPin ctrl_bit: ctrl_bits) {
                log("%s -> %s:%s -> %s\n",
                    log_id(dffe_src), std::get<0>(ctrl_bit)->name.c_str(), std::get<1>(ctrl_bit).c_str(), log_id(dffe_sink));
            }
        }
        log("\n\n");

        {
            auto dff_loop_to_ctrl_pin_map_clone = dff_loop_to_ctrl_pin_map;
            for (auto [directed_edge, ctrl_bits]: dff_loop_to_ctrl_pin_map) {
                // If there is a combinatorial path from such bit to the original guess
                // Replace the original bit with the new
                //
                // Note that the new ctrl candidate should still be dependent on the
                // DFFE source
                auto [dffe_src, dffe_sink] = directed_edge;

                log("Examine edge: %s -> %s\n", log_id(dffe_src), log_id(dffe_sink));
                std::set<CellPin> new_ctrl_bits = ctrl_bits;
                for (CellPin old_ctrl_bit: ctrl_bits) {
                    for (const auto& new_ctrl_bit: additional_ctrls) {
                        if (new_ctrl_bit == old_ctrl_bit) {
                            continue;
                        }

                        if (!circuit_graph.get_intermediate_comb_cells(
                            new_ctrl_bit, old_ctrl_bit
                        ).empty()) {
                            bool src_dependent = false;
                            for (int i = 0; i < GetSize(dffe_src->getPort("\\Q")); ++i) {
                                if (!circuit_graph.get_intermediate_comb_cells(
                                    {dffe_src, "\\Q", i},
                                    new_ctrl_bit
                                ).empty()) {
                                    src_dependent = true;
                                    break;
                                }
                            }

                            if (src_dependent) {
                                log("Erase old ctrl: %s:%s\n", std::get<0>(old_ctrl_bit)->name.c_str(), std::get<1>(old_ctrl_bit).c_str());
                                log("Replace with %s:%s\n", std::get<0>(new_ctrl_bit)->name.c_str(), std::get<1>(new_ctrl_bit).c_str());
                                new_ctrl_bits.erase(old_ctrl_bit);
                                log("Old ctrl bit count: %ld\n", new_ctrl_bits.count(old_ctrl_bit));
                                new_ctrl_bits.insert(new_ctrl_bit);
                            }
                        }
                    }
                }

                dff_loop_to_ctrl_pin_map_clone[directed_edge] = new_ctrl_bits;
            }

            dff_loop_to_ctrl_pin_map = dff_loop_to_ctrl_pin_map_clone;

            log("Promoted control bits\n");
        }

        // Try to promote inferred AND pins to replace control bits
        // This is to improve positional precision
        log("ctrl pin mappings after promotion:\n");
        for (auto& [directed_edge, ctrl_bits]: dff_loop_to_ctrl_pin_map) {
            auto [dffe_src, dffe_sink] = directed_edge;
            int ctrl_bit_count = 0;
            for (CellPin ctrl_bit: ctrl_bits) {
                log("Ctrl bit %d\n", ctrl_bit_count);
                log("%s -> %s:%s -> %s\n",
                    log_id(dffe_src), std::get<0>(ctrl_bit)->name.c_str(), std::get<1>(ctrl_bit).c_str(), log_id(dffe_sink));
                ctrl_bit_count++;
            }
        }
        log("\n\n");

        std::set<RTLIL::Cell*> data_path_cell;
        data_path_cell.insert(data_regs.begin(), data_regs.end());

        // Include muxes that reaches data registers' data pins
        // TODO: Also include muxes that reaches output due to incomplete information
        std::set<RTLIL::Cell*> sinked_muxes;
        {
            for (RTLIL::Cell* mux_cell: muxes) {
                bool has_dependent_data_sink = false;
                for (RTLIL::Cell* data_reg: data_regs) {
                    if (has_dependent_data_sink) {
                        break;
                    }

                    for (int i = 0; i < GetSize(data_reg->getPort("\\D")); ++i) {
                        if (!circuit_graph.get_intermediate_comb_cells(
                            {mux_cell, "\\Y", 0},
                            {data_reg, "\\D", i}
                        ).empty()) {
                            has_dependent_data_sink = true;
                            break;
                        }
                    }
                }

                if (has_dependent_data_sink) {
                    log("Mux %s has dependent data sink\n", log_id(mux_cell));
                    sinked_muxes.insert(mux_cell);
                }
            }
        }
        data_path_cell.insert(sinked_muxes.begin(), sinked_muxes.end());

        // Record if DFFE loop has an associated data register
        std::set<std::set<std::pair<RTLIL::Cell*, CellPin>>> eligible_ctrl_bit_pairs;
        for (const auto& [dffe_edge, ctrl_bits]: dff_loop_to_ctrl_pin_map) {
            const auto& [dffe_src, dffe_sink] = dffe_edge;
            // Find the opposite edge
            std::set<CellPin> opposite_ctrl_bits;
            std::pair<RTLIL::Cell*, RTLIL::Cell*> opposite_edge = {dffe_sink, dffe_src};
            if (dff_loop_to_ctrl_pin_map.count(opposite_edge)) {
                opposite_ctrl_bits = dff_loop_to_ctrl_pin_map.at(opposite_edge);
            } else {
                // The current DFFE cannot complete a loop with eligible ctrl bits
                continue;
            }

            for (const CellPin& ctrl_bit: ctrl_bits) {
                LOG("Exanime %s <-> %s\n", log_id(dffe_src), log_id(dffe_sink));

                for (RTLIL::Cell* data_reg: data_path_cell) {
                    CellPin data_path_ctrl_pin;
                    if (RTLIL::builtin_ff_cell_types().count(data_reg->type)) {
                        data_path_ctrl_pin = {data_reg, "\\EN", 0};
                    } else {
                        data_path_ctrl_pin = {data_reg, "\\S", 0};
                    }

                    // Control bits may be sourced to a datapath control pin combinatorially
                    if (!circuit_graph.get_intermediate_comb_cells(
                        ctrl_bit, data_path_ctrl_pin
                    ).empty()) {
                        for (const CellPin& other_ctrl_bit: opposite_ctrl_bits) {
                            if (!circuit_graph.get_intermediate_comb_cells(
                                other_ctrl_bit, data_path_ctrl_pin
                            ).empty()) {
                                log("Cell %s Port %s reaches Cell %s CE\n",
                                    std::get<0>(ctrl_bit)->name.c_str(),
                                    std::get<1>(ctrl_bit).c_str(),
                                    log_id(data_reg));
                                log("Other ctrl pin: Cell %s; Port %s\n",
                                    std::get<0>(other_ctrl_bit)->name.c_str(),
                                    std::get<1>(other_ctrl_bit).c_str()
                                );
                                std::set<std::pair<RTLIL::Cell*, CellPin>> eligible_pair = {
                                    {dffe_src, ctrl_bit},
                                    {dffe_sink, other_ctrl_bit}};
                                eligible_ctrl_bit_pairs.insert(eligible_pair);
                            }
                        }
                    }
                }

                // Control bits may also be sourced to any FSM control pin,
                // then to datapath control pin
                //
                // This will introduce 1 clock cycle delay
                for (RTLIL::Cell* inter_dffe: self_loop_dffe) {
                    if (!circuit_graph.get_intermediate_comb_cells(
                        ctrl_bit, {inter_dffe, "\\EN", 0}
                    ).empty()) {
                        for (CellPin other_ctrl_bit: opposite_ctrl_bits) {
                            if (!circuit_graph.get_intermediate_comb_cells(
                                other_ctrl_bit, {inter_dffe, "\\EN", 0}
                            ).empty()) {
                                // Get all data output pins, try to reach some MUX selects
                                FfData ff_data(&ff_init_vals, inter_dffe);
                                for (int i = 0; i < GetSize(ff_data.sig_q); ++i) {
                                    CellPin src = {inter_dffe, "\\Q", i};

                                    for (RTLIL::Cell* mux: sinked_muxes) {
                                        if (!circuit_graph.get_intermediate_comb_cells(
                                            src, {mux, "\\S", 0}
                                        ).empty()) {
                                            log("Cell %s Port %s reaches intermediate DFFE %s CE\n",
                                                std::get<0>(ctrl_bit)->name.c_str(),
                                                std::get<1>(ctrl_bit).c_str(),
                                                log_id(inter_dffe));
                                            log("Cell %s Port %s also reaches intermediate DFFE %s CE\n",
                                                std::get<0>(other_ctrl_bit)->name.c_str(),
                                                std::get<1>(other_ctrl_bit).c_str(),
                                                log_id(inter_dffe));
                                            log("DFFE reaches MUX %s\n", log_id(mux));

                                            std::set<std::pair<RTLIL::Cell*, CellPin>> eligible_pair = {
                                                {dffe_src, ctrl_bit},
                                                {dffe_sink, other_ctrl_bit}};
                                            eligible_ctrl_bit_pairs.insert(eligible_pair);
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Check if the ready bits and valid bits are functionally separable
        for (const auto& eligible_pair: eligible_ctrl_bit_pairs) {
            auto eligible_it = eligible_pair.begin();
            auto [dffe_src, eligible_src] = *eligible_it;

            std::advance(eligible_it, 1);
            auto [dffe_sink, eligible_sink] = *eligible_it;

            std::set<RTLIL::Cell*> src_path;
            for (int i = 0; i < GetSize(dffe_src->getPort("\\Q")); ++i) {
                std::set<RTLIL::Cell*> src_i_path = circuit_graph.get_intermediate_comb_cells(
                    {dffe_src, "\\Q", i}, eligible_src);
                src_path.insert(src_i_path.begin(), src_i_path.end());
            }

            std::set<RTLIL::Cell*> sink_path;
            for (int i = 0; i < GetSize(dffe_sink->getPort("\\Q")); ++i) {
                std::set<RTLIL::Cell*> sink_i_path = circuit_graph.get_intermediate_comb_cells(
                    {dffe_sink, "\\Q", i}, eligible_sink);
                sink_path.insert(sink_i_path.begin(), sink_i_path.end());
            }

            log("Examine Cell %s Port %s <-> Cell %s Port %s\n\n",
                std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(),
                std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str());
            
            log("Src Path:\n");
            for (RTLIL::Cell* src_cell: src_path) {
                log("%s\n", log_id(src_cell));
            }

            log("Sink Path:\n");
            for (RTLIL::Cell* sink_cell: sink_path) {
                log("%s\n", log_id(sink_cell));
            }

            log("\n\n");

            bool has_common_gate = false;
            for (RTLIL::Cell* src_gate: src_path) {
                if (sink_path.count(src_gate)) {
                    has_common_gate = true;
                    break;
                }
            }

            if (has_common_gate) {
                log("Found intertwined path between Cell %s Port %s and Cell %s Port %s\n",
                    std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(),
                    std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str());
            }

            // Ensure there are not mutual interference
            std::set<RTLIL::Cell*> dffe_src_sink_path;
            for (int i = 0; i < GetSize(dffe_src->getPort("\\Q")); ++i) {
                std::set<RTLIL::Cell*> path_i = circuit_graph.get_intermediate_comb_cells(
                    {dffe_src, "\\Q", i}, eligible_sink);
                dffe_src_sink_path.insert(path_i.begin(), path_i.end());
            }

            std::set<RTLIL::Cell*> dffe_sink_src_path;
            for (int i = 0; i < GetSize(dffe_sink->getPort("\\Q")); ++i) {
                std::set<RTLIL::Cell*> path_i = circuit_graph.get_intermediate_comb_cells(
                    {dffe_sink, "\\Q", i}, eligible_src);
                dffe_sink_src_path.insert(path_i.begin(), path_i.end());
            }

            if (!dffe_src_sink_path.empty() || !dffe_sink_src_path.empty()) {
                log("Cell %s:%s and Cell %s:%s are combinatorially influenced by opposite DFFE\n",
                    std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(),
                    std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str());
            }

            // TODO: Actually remove infracting control bits
        }

        log("Eligible ctrl bit pairs: %ld\n", eligible_ctrl_bit_pairs.size());

        for (const auto& eligible_pair: eligible_ctrl_bit_pairs) {
            auto eligible_it = eligible_pair.begin();
            auto [dffe_src, eligible_src] = *eligible_it;

            std::advance(eligible_it, 1);
            auto [dffe_sink, eligible_sink] = *eligible_it;

            log("Ctrl src: Cell %s Port %s from %s; sink: Cell %s Port %s from %s\n",
                std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(), log_id(dffe_src),
                std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str(), log_id(dffe_sink));
        }

    }
} ValidReadyPass;

PRIVATE_NAMESPACE_END
