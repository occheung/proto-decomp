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
        if (args.size() > 1) {
            if (args[1] == "-debug") {
                PASS_DEBUG = 1;
            } else {
                PASS_DEBUG = 0;
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

        // Find registers without CE. Allow ctrl bits to sink D pin instead.
        // The register should have a self feedback loop. Polarity does not matter.
        std::set<RTLIL::Cell*> self_loop_dff;
        {
            std::set<RTLIL::Cell*> ff_no_ce;
            for (RTLIL::Cell* cell: top->cells()) {
                if (RTLIL::builtin_ff_cell_types().count(cell->type)) {
                    FfData ff_data(&ff_init_vals, cell);
                    if (!ff_data.has_ce) {
                        // Check for a path from Q pin to D pin
                        for (int i = 0; i < GetSize(ff_data.sig_q); ++i) {
                            for (int j = 0; i < GetSize(ff_data.sig_d); ++j) {
                                if (!circuit_graph.get_intermediate_comb_cells(
                                    {cell, "\\Q", i},
                                    {cell, "\\D", j}
                                ).empty()) {
                                    ff_no_ce.insert(cell);
                                    self_loop_dff.insert(cell);
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            log("Self-looping FF without CE:\n");
            for (RTLIL::Cell* cell: ff_no_ce) {
                log("%s\n", log_id(cell));
            }
        }

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
        // std::set<RTLIL::Cell*> self_loop_dffe;
        for (RTLIL::Cell* ff_cell: ff_with_enable) {
            bool has_path = false;
            int port_size = GetSize(ff_cell->getPort("\\D"));
            for (int src_pin_idx = 0; src_pin_idx < port_size; ++src_pin_idx) {
                for (int sink_pin_idx = 0; sink_pin_idx < port_size; ++sink_pin_idx) {
                    has_path = !(circuit_graph.get_intermediate_comb_cells(
                        {ff_cell, "\\Q", src_pin_idx}, {ff_cell, "\\D", sink_pin_idx}).empty());
                    if (has_path) {
                        self_loop_dff.insert(ff_cell);
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
        std::set<RTLIL::Cell*> independent_self_loop_dffs;
        for (RTLIL::Cell* dffe: self_loop_dff) {
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
                log("DFFE %s is not an FSM\n", log_id(dffe));
                independent_self_loop_dffs.insert(dffe);
            }
        }

        for (RTLIL::Cell* independent_dff: independent_self_loop_dffs) {
            self_loop_dff.erase(independent_dff);
        }

        // Find data registers
        // TODO: Define data registers as registers with CE but not classified as FSMs
        std::set<RTLIL::Cell*> data_regs;
        for (RTLIL::Cell* ff: ff_with_enable) {
            if (self_loop_dff.count(ff) == 0) {
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

        for (RTLIL::Cell* ff_cell: self_loop_dff) {
            log("Filtered FF Cell: %s; FF type: %s\n", ff_cell->name.c_str(), ff_cell->type.c_str());
        }

        {
            for (const auto& candidate_reg: self_loop_dff) {
                log("Candidate register: %s:%s\n", log_id(candidate_reg), candidate_reg->type.c_str());
            }
        }

        // Highlighted AND gate collections
        // Collect AND gate that fans into a FSM without SCC
        // The fan-ins of the AND gate are likely valid and ready bits
        std::set<std::pair<CellPin, CellPin>> and_gates_inputs;

        // Store potential valid/ready bits of the entire circuit
        std::set<CellPin> ctrl_candidates;

        // // Properly group up the control bits by DFFE directional edges
        // dict<std::pair<RTLIL::Cell*, RTLIL::Cell*>, std::set<CellPin>> dff_loop_to_ctrl_pin_map;

        // Record the most senior primary output, mapped by
        // directed edges from a DFF to another
        //
        // FIXME: The pimary output should converge? Given that there should be a 1-bit edge
        // connecting the DFFs on the path.
        dict<std::pair<RTLIL::Cell*, RTLIL::Cell*>, std::pair<std::set<RTLIL::Cell*>, std::set<CellPin>>> dff_edge_primary_outputs;

        // Find the strongly connected component.
        for (RTLIL::Cell* dff: self_loop_dff) {
            log("Suspected state register: %s\n", dff->name.c_str());
            std::set<RTLIL::Cell*> scc_set;

            FfData dff_data(&ff_init_vals, dff);
            IdString ctrl_sink_port = dff_data.has_ce? "\\EN": "\\D";
            for (int i = 0; i < GetSize(dff->getPort("\\Q")); ++i) {
                for (int j = 0; j < GetSize(dff->getPort(ctrl_sink_port)); ++j) {
                    std::set<RTLIL::Cell*> bit_scc_set = circuit_graph.get_intermediate_comb_cells(
                        {dff, "\\Q", i}, {dff, ctrl_sink_port, j});
                    scc_set.insert(bit_scc_set.begin(), bit_scc_set.end());
                }
            }

            // Remove dff from the collected set:
            // We are not interested at it
            scc_set.erase(dff);

            // If the SCC is empty, and EN is connected to an AND gate, add the AND gate
            // to a list. It could be a sink to a control bit.
            // However, should ce_over_srst be enabled, there should be an OR gate
            // that routes the SRST signal to CE through it, else SRST is disabled
            // unconditionally.
            //
            // Note that in the non-CE DFF case: SCC cannot be empty because a feedback 
            // loop from Q to D was verified, and the sink pin is D port instead of CE port.
            if (scc_set.empty()) {
                Source en_src;

                if (!dff_data.has_ce) {
                    log_error("Register %s should be a data register.\n", log_id(dff));
                }

                if (dff_data.has_srst && dff_data.ce_over_srst) {
                    const Source& ce_src = circuit_graph.source_map[dff]["\\EN"][0];
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
                    const Source& srst_src = circuit_graph.source_map[dff]["\\SRST"][0];
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
                    en_src = circuit_graph.source_map[dff]["\\EN"][0];
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
                int q_port_size = GetSize(dff->getPort("\\Q"));
                for (int pin_idx = 0; pin_idx < q_port_size; ++pin_idx) {
                    non_scc_sources.erase({dff, "\\Q", pin_idx});
                }
            }

            // Compute the intermediate nodes for all paths from any sources
            // to port E
            for (auto src_pin_it = non_scc_sources.begin(); src_pin_it != non_scc_sources.end(); ++src_pin_it) {
                const auto& [src_cell, src_port, _pin_idx] = src_pin_it->first;
                LOG("Cell: %s; Port: %s\n", src_cell->name.c_str(), src_port.c_str());
                std::set<CellPin> dominated_pins;
                if (dff_data.has_ce) {
                    dominated_pins = circuit_graph.get_dominated_pins(src_pin_it->first, {dff, "\\EN", 0});
                } else {
                    for (int i = 0; i < GetSize(dff_data.sig_d); ++i) {
                        std::set<CellPin> i_dom_pins = circuit_graph.get_dominated_pins(
                            src_pin_it->first, {dff, "\\D", i});
                        dominated_pins.insert(i_dom_pins.begin(), i_dom_pins.end());
                    }
                }

                // // Remove the dffe pin. We know for sure it is in the domination set.
                // dominated_pins.erase({dffe, "\\EN", 0});
                // Remove the source itself. It does not help solving the dominance problem.
                dominated_pins.erase(src_pin_it->first);
                // Remove the control sinks.
                // If it is a DFFE, remove CE pins; Otherwise, remove all D pins.
                if (dff_data.has_ce) {
                    dominated_pins.erase({dff, "\\EN", 0});
                } else {
                    for (int i = 0; i < GetSize(dff_data.sig_d); ++i) {
                        dominated_pins.erase({dff, "\\D", i});
                    }
                }

                for (const CellPin& pin: dominated_pins) {
                    const auto& [dom_cell, dom_port, _dom_pin_idx] = pin;
                    LOG("Dominated pin: Cell: %s; Port: %s\n", dom_cell->name.c_str(), dom_port.c_str());
                }

                src_pin_it->second = dominated_pins;
            }

            // Compute dominance for the SCC to approximate the interface of the FSM
            {
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
                for (int i = 0; i < GetSize(dff->getPort("\\Q")); ++i) {
                    fsm_state_bits.insert({dff, "\\Q", i});
                }

                // Give a More machine approximation of the output
                std::set<CellPin> fsm_outputs = circuit_graph.get_dominated_frontier(
                    fsm_state_bits,
                    {}
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
            std::set<RTLIL::Cell*> dff_sources = circuit_graph.dff_source_graph[dff];
            std::set<RTLIL::Cell*> dff_sinks = circuit_graph.dff_sink_graph[dff];

            std::set<RTLIL::Cell*> viable_intermediate_dffs;
            for (RTLIL::Cell* dff_src: dff_sources) {
                // Not interested in self-loop 1-hop
                if (dff_src == dff) {
                    continue;
                }

                // // Not interested in intermediate DFF that lacks clock enable
                // // We hypothesized the valid/ready bits shoudl drive the enable pins
                // FfData ff_data(&ff_init_vals, dffe_src);
                // if (!ff_data.has_ce) {
                //     continue;
                // }

                // The looped-in DFFE should also control a valid/ready bit
                // by symmetry
                if (self_loop_dff.count(dff_src) == 0) {
                    continue;
                }

                if (dff_sinks.count(dff_src)) {
                    viable_intermediate_dffs.insert(dff_src);
                }
            }

            log("Viable 1-hop intermediate DFF:\n");
            for (RTLIL::Cell* inter_dff: viable_intermediate_dffs) {
                log("%s\n", inter_dff->name.c_str());
            }

            {
                // Find the intersection of the loop path and the DFF inputs
                for (const CellPin& connected_input_bit: non_dominated_sources) {
                    for (RTLIL::Cell* inter_dff: viable_intermediate_dffs) {
                        bool found_path = false;
                        for (int i = 0; i < GetSize(inter_dff->getPort("\\Q")); ++i) {
                            if (!circuit_graph.get_intermediate_comb_cells(
                                {inter_dff, "\\Q", i},
                                connected_input_bit
                            ).empty()) {
                                found_path = true;
                            }
                        }

                        if (found_path) {
                            // // Mark the bit, exclude this input in output ctrl bit finding
                            // std::set<CellPin> adjusted_fsm_inputs = non_dominated_sources;
                            // adjusted_fsm_inputs.erase(connected_input_bit);

                            std::set<CellPin> fsm_state_bits;
                            for (int i = 0; i < GetSize(dff->getPort("\\Q")); ++i) {
                                fsm_state_bits.insert({dff, "\\Q", i});
                            }

                            // Find the output bits using Moore machine approximation
                            std::set<CellPin> fsm_outputs = circuit_graph.get_dominated_frontier(
                                fsm_state_bits,
                                // adjusted_fsm_inputs,
                                {}
                            );

                            // log("Guess cell %s port %s index %d as input connection\n",
                            //     log_id(std::get<0>(connected_input_bit)),
                            //     log_id(std::get<1>(connected_input_bit)),
                            //     std::get<2>(connected_input_bit));
                            // for (const auto& [out_cell, out_pin_name, out_pin_idx]: fsm_outputs) {
                            //     log("Cell %s Pin %s:%d\n", out_cell->name.c_str(), out_pin_name.c_str(), out_pin_idx);
                            // }
                            // log("Guess ends\n\n");

                            // See if any eligible output pins complete the path
                            std::set<CellPin> primary_outputs;
                            std::set<RTLIL::Cell*> all_outputs_half_path_set;
                            for (CellPin fsm_output: fsm_outputs) {
                                const auto& [out_cell, out_pin_name, out_pin_idx] = fsm_output;
                                log("Cell %s Pin %s:%d ", out_cell->name.c_str(), out_pin_name.c_str(), out_pin_idx);
                                
                                FfData inter_dff_data(&ff_init_vals, inter_dff);
                                IdString dff_ctrl_port = inter_dff_data.has_ce? "\\EN": "\\D";
                                std::set<RTLIL::Cell*> half_path_set;
                                for (int i = 0; i < GetSize(inter_dff->getPort(dff_ctrl_port)); ++i) {
                                    std::set<RTLIL::Cell*> i_half_path_set = circuit_graph.get_intermediate_comb_cells(
                                        fsm_output, {inter_dff, dff_ctrl_port, i});
                                    half_path_set.insert(i_half_path_set.begin(), i_half_path_set.end());
                                }

                                if (!half_path_set.empty()) {
                                    all_outputs_half_path_set.insert(half_path_set.begin(), half_path_set.end());
                                    // Check if the output pin drives some other output pins
                                    // that also belongs to the path
                                    //
                                    // TODO: May strengthen this check if we first rewrite the FSM s.t.
                                    // the outputs are a function of state variable then everything else
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
                                        primary_outputs.insert({out_cell, out_pin_name, out_pin_idx});
                                        log("PRIMARILY ");
                                    }
                                    log("forms a complete path\n");
                                } else {
                                    log("does NOT form a complete path\n");
                                }
                                auto [input_cell, input_port, input_idx] = connected_input_bit;
                                log("directed to %s:%s:%d\n", 
                                    log_id(input_cell), input_port.c_str(), input_idx);
                            }

                            dff_edge_primary_outputs[{dff, inter_dff}] = std::make_pair(all_outputs_half_path_set, primary_outputs);
                        }
                    }
                }
            }
        }

        for (const auto& [dff_edge, payload]: dff_edge_primary_outputs) {
            auto [src_dff, sink_dff] = dff_edge;
            auto [half_path, primary_outputs] = payload;

            log("\n");
            log("Edge: %s -> %s\n", log_id(src_dff), log_id(sink_dff));
            log("Primary outputs:\n");
            for (const CellPin& primary_output: primary_outputs) {
                auto [cell, port, idx] = primary_output;
                log("%s:%s:%d\n", log_id(cell), port.c_str(), idx);
            }
            log("\n");
        }

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
        std::map<std::set<std::pair<RTLIL::Cell*, CellPin>>, std::set<std::pair<RTLIL::Cell*, int>>> eligible_ctrl_bit_pairs;
        std::map<std::set<std::pair<RTLIL::Cell*, CellPin>>, std::map<CellPin, std::set<CellPin>>> cut_edges;
        // Record the corresponding associated data components
        for (const auto& [dff_edge, payload]: dff_edge_primary_outputs) {
            const auto& [dff_src, dff_sink] = dff_edge;
            const auto& [half_path_set, primary_output] = payload;

            // Find the opposite edge
            std::pair<RTLIL::Cell*, RTLIL::Cell*> opposite_edge = {dff_sink, dff_src};
            std::set<RTLIL::Cell*> opposite_half_path_set;
            std::set<CellPin> opposite_primary_output;
            if (dff_edge_primary_outputs.count(opposite_edge)) {
                opposite_half_path_set = dff_edge_primary_outputs.at(opposite_edge).first;
                opposite_primary_output = dff_edge_primary_outputs.at(opposite_edge).second;
            } else {
                // The current DFFE cannot complete a loop with eligible ctrl bits
                continue;
            }

            auto get_first_divergence = [&] (
                CellPin primary_out,
                std::set<RTLIL::Cell*> control_path_cells,
                std::set<RTLIL::Cell*> data_path_cells,
                std::set<RTLIL::Cell*> opposite_data_path_cells
            ) -> std::pair<CellPin, int> {
                std::set<RTLIL::Cell*> common_cells;
                std::set<RTLIL::Cell*> divergent_cells;
                for (RTLIL::Cell* data_path_cell: data_path_cells) {
                    if (control_path_cells.count(data_path_cell)) {
                        common_cells.insert(data_path_cell);
                    } else {
                        divergent_cells.insert(data_path_cell);
                    }
                }

                CellPin first_divergence;
                bool found_divergence = false;
                std::vector<CellPin> work_list;
                work_list.push_back(primary_out);

                for (RTLIL::Cell* common_cell: common_cells) {
                    LOG("Common cell: %s\n", log_id(common_cell));
                }

                // Keep track of the divergence depth
                // Eventually we need the depth to optimize triangulation guess
                int depth = -1;

                while (!work_list.empty() && !found_divergence) {
                    depth += 1;

                    auto [curr_cell, curr_port, curr_pin_idx] = work_list.back();
                    work_list.pop_back();

                    LOG("Trace from cell %s:%s:%d\n", log_id(curr_cell), curr_port.c_str(), curr_pin_idx);

                    // If the logic path starts to branch, stop trasversing.
                    bool found_suitable_successor = false;
                    vector<Sink> curr_sinks = circuit_graph.sink_map.at(curr_cell).at(curr_port)[curr_pin_idx];
                    for (Sink curr_sink: curr_sinks) {
                        if (std::holds_alternative<RTLIL::SigBit>(curr_sink)) {
                            continue;
                        }

                        auto [sink_cell, sink_port, sink_port_idx] = std::get<CellPin>(curr_sink);

                        // The first condition is the first divergence condition. Self explanatory.
                        // The second condition is to force selection before merging with opposite cells.
                        if (divergent_cells.count(sink_cell) || opposite_data_path_cells.count(sink_cell)) {
                            found_divergence = true;
                            first_divergence = {curr_cell, curr_port, curr_pin_idx};
                            break;
                        }

                        // If the sink is a sequential element WHILE not being the data component target, do not propagate there
                        if (RTLIL::builtin_ff_cell_types().count(sink_cell->type)) {
                            continue;
                        }

                        if (common_cells.count(sink_cell)) {
                            if (found_suitable_successor) {
                                // Stop searching divergence. The current pin is our best bet.
                                found_divergence = true;
                                first_divergence = {curr_cell, curr_port, curr_pin_idx};
                                break;
                            }
                            found_suitable_successor = true;
                            for (const auto& [port_name, port_sig]: sink_cell->connections()) {
                                if (sink_cell->output(port_name)) {
                                    for (int i = 0; i < GetSize(port_sig); ++i) {
                                        work_list.push_back({sink_cell, port_name, i});
                                    }
                                }
                            }
                        }
                    }
                }

                if (!found_divergence) {
                    log_error("Divergnece cannot be located despite detected\n");
                    return make_pair(primary_out, depth);
                }

                return make_pair(first_divergence, depth);
            };

            for (const CellPin& ctrl_bit: primary_output) {
                log("\n\n");
                log("Exanime %s <-> %s\n", log_id(dff_src), log_id(dff_sink));

                for (const CellPin& other_ctrl_bit: opposite_primary_output) {
                    // Keep track of the proposed control bits paring and their depth
                    // Only confirm the ones with least depth
                    //
                    // FIXME: Depth comarison is multidimensional since both directions have a depth
                    int joint_depth = std::numeric_limits<int>::max();
                    CellPin src_first_divergence_opt;
                    CellPin sink_first_divergence_opt;
                    std::map<CellPin, std::set<CellPin>> edges_to_be_cut_opt;

                    std::set<std::pair<RTLIL::Cell*, int>> dependent_data_components;

                    for (RTLIL::Cell* data_reg: data_path_cell) {
                        CellPin data_path_ctrl_pin;
                        int priority;
                        if (RTLIL::builtin_ff_cell_types().count(data_reg->type)) {
                            data_path_ctrl_pin = {data_reg, "\\EN", 0};
                            if (independent_self_loop_dffs.count(data_reg)) {
                                priority = 1;
                            } else {
                                priority = 3;
                            }
                        } else {
                            data_path_ctrl_pin = {data_reg, "\\S", 0};
                            priority = 2;
                        }

                        // Control bits may be sourced to a datapath control pin combinatorially
                        std::set<RTLIL::Cell*> ctrl_bit_to_data_path_set = circuit_graph.get_intermediate_comb_cells(
                            ctrl_bit, data_path_ctrl_pin);

                        std::set<RTLIL::Cell*> opposite_ctrl_bit_to_data_path = circuit_graph.get_intermediate_comb_cells(
                            other_ctrl_bit, data_path_ctrl_pin);
                        
                        if ((!ctrl_bit_to_data_path_set.empty()) && (!opposite_ctrl_bit_to_data_path.empty())) {
                            LOG("From direction\n");
                            auto [src_first_divergence, src_depth] = get_first_divergence(
                                ctrl_bit, half_path_set,
                                ctrl_bit_to_data_path_set, opposite_ctrl_bit_to_data_path);

                            LOG("To direction\n");
                            auto [sink_first_divergence, sink_depth] = get_first_divergence(
                                other_ctrl_bit, opposite_half_path_set,
                                opposite_ctrl_bit_to_data_path, ctrl_bit_to_data_path_set);

                            log("Cell %s Port %s reaches Cell %s CE\n",
                                std::get<0>(ctrl_bit)->name.c_str(),
                                std::get<1>(ctrl_bit).c_str(),
                                log_id(data_reg));
                            log("Other ctrl pin: Cell %s; Port %s\n",
                                std::get<0>(other_ctrl_bit)->name.c_str(),
                                std::get<1>(other_ctrl_bit).c_str()
                            );

                            // Record data element regardless of priority
                            // Give direct DFFE priority 3
                            // Direct MUX priority 2
                            // Self looping DFFE priority 1
                            dependent_data_components.insert({data_reg, priority});

                            // Insert the new eligible pair if the divergence depth is shallower
                            if (src_depth + sink_depth < joint_depth) {
                                joint_depth = src_depth + sink_depth;
                                src_first_divergence_opt = src_first_divergence;
                                sink_first_divergence_opt = sink_first_divergence;
                                edges_to_be_cut_opt.clear();
                                edges_to_be_cut_opt[src_first_divergence] = {};
                                edges_to_be_cut_opt[sink_first_divergence] = {};

                                {
                                    const auto& [first_div_cell, first_div_port, first_div_idx] = src_first_divergence;
                                    for (const Sink& first_div_sink: circuit_graph.sink_map
                                            .at(first_div_cell).at(first_div_port).at(first_div_idx)) {
                                        if (std::holds_alternative<RTLIL::SigBit>(first_div_sink)) {
                                            continue;
                                        }
                                        CellPin first_div_sink_pin = std::get<CellPin>(first_div_sink);
                                        // Add the sink to the edges to be cut set if
                                        // the sink is part of the half path set
                                        if (half_path_set.count(std::get<0>(first_div_sink_pin))) {
                                            edges_to_be_cut_opt[src_first_divergence].insert(first_div_sink_pin);
                                        }
                                    }
                                }

                                {
                                    const auto& [first_div_cell, first_div_port, first_div_idx] = sink_first_divergence;
                                    for (const Sink& first_div_sink: circuit_graph.sink_map
                                            .at(first_div_cell).at(first_div_port).at(first_div_idx)) {
                                        if (std::holds_alternative<RTLIL::SigBit>(first_div_sink)) {
                                            continue;
                                        }
                                        CellPin first_div_sink_pin = std::get<CellPin>(first_div_sink);
                                        // Add the sink to the edges to be cut set if
                                        // the sink is part of the opposite half path set
                                        if (opposite_half_path_set.count(std::get<0>(first_div_sink_pin))) {
                                            edges_to_be_cut_opt[src_first_divergence].insert(first_div_sink_pin);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (joint_depth != std::numeric_limits<int>::max()) {
                        std::set<std::pair<RTLIL::Cell*, CellPin>> eligible_pair = {
                            {dff_src, src_first_divergence_opt},
                            {dff_sink, sink_first_divergence_opt}};
                        // eligible_ctrl_bit_pairs.insert(eligible_pair);
                        eligible_ctrl_bit_pairs[eligible_pair] = dependent_data_components;
                        cut_edges[eligible_pair] = edges_to_be_cut_opt;

                        // FIXME: continue the loop
                        // Should we prioritize direct ctrl pin connections?
                    }

                    // Control bits may also be sourced to any FSM control pin,
                    // then to datapath control pin
                    //
                    // This will introduce 1 clock cycle delay
                    for (RTLIL::Cell* inter_dff: self_loop_dff) {
                        // The intermediate DFF should not be either the sink or the source DFFs
                        if (inter_dff == dff_src || inter_dff == dff_sink) {
                            continue;
                        }

                        std::set<RTLIL::Cell*> ctrl_data_path_set = circuit_graph.get_intermediate_comb_cells(
                            ctrl_bit, {inter_dff, "\\EN", 0});
                        std::set<RTLIL::Cell*> opposite_ctrl_data_path_set = circuit_graph.get_intermediate_comb_cells(
                            other_ctrl_bit, {inter_dff, "\\EN", 0});
                        
                        if ((!ctrl_data_path_set.empty()) && (!opposite_ctrl_data_path_set.empty())) {
                            log("Cell %s Port %s reaches intermediate DFFE %s CE\n",
                                std::get<0>(ctrl_bit)->name.c_str(),
                                std::get<1>(ctrl_bit).c_str(),
                                log_id(inter_dff));
                            log("Cell %s Port %s also reaches intermediate DFFE %s CE\n",
                                std::get<0>(other_ctrl_bit)->name.c_str(),
                                std::get<1>(other_ctrl_bit).c_str(),
                                log_id(inter_dff));

                            // Triangulate the exact location for both directions
                            LOG("From direction\n");
                            auto [src_first_divergence, src_depth] = get_first_divergence(
                                ctrl_bit, half_path_set,
                                ctrl_data_path_set, opposite_ctrl_data_path_set);

                            LOG("To direction\n");
                            auto [sink_first_divergence, sink_depth] = get_first_divergence(
                                other_ctrl_bit, opposite_half_path_set,
                                opposite_ctrl_data_path_set, ctrl_data_path_set);
                            
                            // FIXME: Uncomment to optimize computation
                            // We comment it out to acquire the complete set of data registers
                            // // If the shallowness will not improve, don't both finding a sinkable MUX
                            // if (src_depth + sink_depth >= joint_depth) {
                            //     continue;
                            // }

                            // Get all data output pins, try to reach some MUX selects
                            FfData ff_data(&ff_init_vals, inter_dff);
                            for (int i = 0; i < GetSize(ff_data.sig_q); ++i) {
                                CellPin src = {inter_dff, "\\Q", i};

                                for (RTLIL::Cell* mux: sinked_muxes) {
                                    if (!circuit_graph.get_intermediate_comb_cells(
                                        src, {mux, "\\S", 0}
                                    ).empty()) {
                                        // log("Cell %s Port %s reaches intermediate DFFE %s CE\n",
                                        //     std::get<0>(ctrl_bit)->name.c_str(),
                                        //     std::get<1>(ctrl_bit).c_str(),
                                        //     log_id(inter_dffe));
                                        // log("Cell %s Port %s also reaches intermediate DFFE %s CE\n",
                                        //     std::get<0>(other_ctrl_bit)->name.c_str(),
                                        //     std::get<1>(other_ctrl_bit).c_str(),
                                        //     log_id(inter_dff));
                                        log("DFFE reaches MUX %s\n", log_id(mux));
                                        dependent_data_components.insert({mux, 0});

                                        if (src_depth + sink_depth >= joint_depth) {
                                            continue;
                                        }
                                        joint_depth = src_depth + sink_depth;
                                        src_first_divergence_opt = src_first_divergence;
                                        sink_first_divergence_opt = sink_first_divergence;

                                        // FIXME: Omitting the break for logging
                                        // // The distance from ctrl register to data component doesn't impact the hueristic
                                        // break;

                                        edges_to_be_cut_opt.clear();
                                        edges_to_be_cut_opt[src_first_divergence] = {};
                                        edges_to_be_cut_opt[sink_first_divergence] = {};

                                        {
                                            const auto& [first_div_cell, first_div_port, first_div_idx] = src_first_divergence;
                                            for (const Sink& first_div_sink: circuit_graph.sink_map
                                                    .at(first_div_cell).at(first_div_port).at(first_div_idx)) {
                                                if (std::holds_alternative<RTLIL::SigBit>(first_div_sink)) {
                                                    continue;
                                                }
                                                CellPin first_div_sink_pin = std::get<CellPin>(first_div_sink);
                                                // Add the sink to the edges to be cut set if
                                                // the sink is part of the half path set
                                                if (half_path_set.count(std::get<0>(first_div_sink_pin))) {
                                                    edges_to_be_cut_opt[src_first_divergence].insert(first_div_sink_pin);
                                                }
                                            }
                                        }

                                        {
                                            const auto& [first_div_cell, first_div_port, first_div_idx] = sink_first_divergence;
                                            for (const Sink& first_div_sink: circuit_graph.sink_map
                                                    .at(first_div_cell).at(first_div_port).at(first_div_idx)) {
                                                if (std::holds_alternative<RTLIL::SigBit>(first_div_sink)) {
                                                    continue;
                                                }
                                                CellPin first_div_sink_pin = std::get<CellPin>(first_div_sink);
                                                // Add the sink to the edges to be cut set if
                                                // the sink is part of the half path set
                                                if (half_path_set.count(std::get<0>(first_div_sink_pin))) {
                                                    edges_to_be_cut_opt[src_first_divergence].insert(first_div_sink_pin);
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (joint_depth != std::numeric_limits<int>::max()) {
                        std::set<std::pair<RTLIL::Cell*, CellPin>> eligible_pair = {
                            {dff_src, src_first_divergence_opt},
                            {dff_sink, sink_first_divergence_opt}};
                        // eligible_ctrl_bit_pairs.insert(eligible_pair);
                        eligible_ctrl_bit_pairs[eligible_pair] = dependent_data_components;
                        cut_edges[eligible_pair] = edges_to_be_cut_opt;
                    }
                }
            }
        }

        // Check if the ready bits and valid bits are functionally separable
        for (const auto& [eligible_pair, _data_components]: eligible_ctrl_bit_pairs) {
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

            LOG("Examine Cell %s Port %s <-> Cell %s Port %s\n\n",
                std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(),
                std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str());
            
            LOG("Src Path:\n");
            for (RTLIL::Cell* src_cell: src_path) {
                LOG("%s\n", log_id(src_cell));
            }

            LOG("Sink Path:\n");
            for (RTLIL::Cell* sink_cell: sink_path) {
                LOG("%s\n", log_id(sink_cell));
            }

            LOG("\n\n");

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

        std::set<RTLIL::Cell*> global_data_interfaces;

        std::map<std::set<std::pair<RTLIL::Cell*, CellPin>>, std::set<CellPin>> module_interfaces;
        for (const auto& [eligible_pair, data_components]: eligible_ctrl_bit_pairs) {
            auto eligible_it = eligible_pair.begin();
            auto [dffe_src, eligible_src] = *eligible_it;

            std::advance(eligible_it, 1);
            auto [dffe_sink, eligible_sink] = *eligible_it;

            log("Ctrl src: Cell %s Port %s from %s; sink: Cell %s Port %s from %s\n",
                std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(), log_id(dffe_src),
                std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str(), log_id(dffe_sink));
            log("\n");
            
            // for (const auto& [data_component, depth]: data_components) {
            //     log("Data component: %s\n", log_id(data_component));
            //     log("Depth: %d\n", depth);
            // }
            // log("Recorded %ld cells in total\n", data_components.size());

            // Approximate the data interface by tracing the data components backward
            // Perferentially use data components with lower latency
            int highest_priority = 0;
            std::set<RTLIL::Cell*> data_component_highest_priority;
            for (const auto& [data_component, priority]: data_components) {
                if (priority > highest_priority) {
                    highest_priority = priority;
                    data_component_highest_priority = {};
                }
                if (priority == highest_priority) {
                    data_component_highest_priority.insert(data_component);
                }
            }
            // log("%ld cells has the highest priority\n", data_component_highest_priority.size());

            std::set<CellPin> data_sources;
            // Convert all RTLIL::Cell the data line representation
            for (RTLIL::Cell* ctrl_cell: data_component_highest_priority) {
                if (RTLIL::builtin_ff_cell_types().count(ctrl_cell->type)) {
                    FfData ctrl_data(&ff_init_vals, ctrl_cell);
                    if (ctrl_data.has_ce) {
                        for (int i = 0; i < GetSize(ctrl_data.sig_d); ++i) {
                            Source src = circuit_graph.source_map.at(ctrl_cell).at("\\D").at(i);
                            if (std::holds_alternative<CellPin>(src)) {
                                data_sources.insert(std::get<CellPin>(src));
                            }
                        }
                    } else {
                        log_error("Found a ctrl pin that isn't properly CE from cell %s\n", log_id(ctrl_cell));
                    }
                } else if (ctrl_cell->type.in(ID($_MUX_))) {
                    // Put the output of the MUX tree into the sources
                    // Therefore, ignore any MUXes that feeds into another MUX in the most prioritized sources
                    vector<Sink> ctrl_data_sinks = circuit_graph.sink_map.at(ctrl_cell).at("\\Y").at(0);
                    bool chained_mux = false;
                    for (const Sink& data_sink: ctrl_data_sinks) {
                        if (std::holds_alternative<RTLIL::SigBit>(data_sink)) {
                            continue;
                        }

                        auto [data_sink_cell, data_sink_port, data_sink_pin_idx] = std::get<CellPin>(data_sink);
                        if (data_component_highest_priority.count(data_sink_cell) && data_sink_cell->type.in(ID($_MUX_))) {
                            chained_mux = true;
                            break;
                        }
                    }

                    if (!chained_mux) {
                        data_sources.insert({ctrl_cell, "\\Y", 0});
                    }
                } else {
                    log_error("Found control cell that is neither a DFF nor a MUX\n");
                }
            }

            log("Found data interfaces at priority %d:\n", highest_priority);
            for (auto & [data_cell, data_port, data_pin_idx]: data_sources) {
                global_data_interfaces.insert(data_cell);
                log("%s:%s:%d\n", log_id(data_cell), data_port.c_str(), data_pin_idx);
            }
            log("\n\n");

            module_interfaces[eligible_pair] = data_sources;
        }

        // Record in Yosys set
        std::stringstream ss;
        ss << "select -set data_comp ";

        for (RTLIL::Cell* data_comp: global_data_interfaces) {
            ss << "c:" << data_comp->name.c_str() << " ";
        }

        Pass::call(design, ss.str().c_str());
        // Pass::call(design, "show -color red @data_comp");

        // Directed graph. Each edge states key can reach value eventually
        std::map<std::set<std::pair<RTLIL::Cell*, CellPin>>, std::set<std::set<std::pair<RTLIL::Cell*, CellPin>>>> reachability_map;

        for (const auto& [src_pair, data_interface]: module_interfaces) {
            for (const auto& [other_pair, other_data_if]: module_interfaces) {
                if (src_pair == other_pair) {
                    continue;
                }

                if (circuit_graph.reachable(data_interface, other_data_if)) {
                    if (reachability_map.count(src_pair)) {
                        reachability_map[src_pair] = {};
                    }

                    auto src_it = src_pair.begin();
                    auto [dff_src_src, eligible_src_src] = *src_it;

                    std::advance(src_it, 1);
                    auto [dff_sink_src, eligible_sink_src] = *src_it;

                    auto other_it = other_pair.begin();
                    auto [dff_src_sink, eligible_src_sink] = *other_it;

                    std::advance(other_it, 1);
                    auto [dff_sink_sink, eligible_sink_sink] = *other_it;

                    log("DFF %s (CTRL %s:%s:%d) reaches DFF %s (CTRL %s:%s:%d)\n",
                        log_id(dff_src_src), log_id(std::get<0>(eligible_src_src)), std::get<1>(eligible_src_src).c_str(), std::get<2>(eligible_src_src),
                        log_id(dff_src_sink), log_id(std::get<0>(eligible_src_sink)), std::get<1>(eligible_src_sink).c_str(), std::get<2>(eligible_src_sink)
                    );

                    reachability_map[src_pair].insert(other_pair);
                }
            }
        }

        // TODO: Remove transitive closure

        // For each pair, cut the DFF edge in the DFF graph
        // Then, check reachability of both DFF with other pairs respectively
        std::set<std::pair<CellPin, CellPin>> valid_ready_pair;
        for (auto &[if_src, if_sinks]: reachability_map) {
            auto src_it = if_src.begin();
            auto [if_src_dff, if_src_pin] = *src_it;

            std::advance(src_it, 1);
            auto [if_other_src_dff, if_other_src_pin] = *src_it;

            // Find the next combinatorial gates that goes through the interface control bits
            std::set<CellPin> if_src_next_pins = cut_edges.at(if_src).at(if_src_pin);
            std::set<CellPin> if_other_src_next_pins = cut_edges.at(if_src).at(if_other_src_pin);

            for (const auto& if_sink: if_sinks) {
                auto sink_it = if_sink.begin();
                auto [if_sink_dff, if_sink_pin] = *sink_it;

                std::advance(sink_it, 1);
                auto [if_other_sink_dff, if_other_sink_pin] = *sink_it;

                auto cut_graph_dff_reachable = [&](
                    RTLIL::Cell* src,
                    RTLIL::Cell* dest,
                    std::map<RTLIL::Cell*, std::set<RTLIL::Cell*>> removed_edges
                ) {
                    std::set<RTLIL::Cell*> reached_cells;
                    
                    std::vector<RTLIL::Cell*> work_list;
                    work_list.push_back(src);

                    while (!work_list.empty()) {
                        RTLIL::Cell* curr = work_list.back();
                        work_list.pop_back();

                        reached_cells.insert(curr);

                        if (curr == dest) {
                            return true;
                        }

                        // Propagate
                        if (circuit_graph.dff_sink_graph.count(curr)) {
                            for (RTLIL::Cell* dff_sink: circuit_graph.dff_sink_graph.at(curr)) {
                                // Ignore removed edges
                                if (removed_edges.count(curr)) {
                                    if (removed_edges.at(curr).count(dff_sink)) {
                                        continue;
                                    }
                                }

                                // Ignore data registers
                                if (data_regs.count(dff_sink)) {
                                    continue;
                                }

                                if (reached_cells.count(dff_sink) == 0) {
                                    work_list.push_back(dff_sink);
                                }
                            }
                        }
                    }

                    return false;
                };

                auto reachable_dff_sinks = [&](CellPin ctrl_bit) {
                    std::set<RTLIL::Cell*> ret;

                    std::vector<CellPin> work_list;
                    work_list.push_back(ctrl_bit);
                    while (!work_list.empty()) {
                        auto [curr_cell, curr_port, curr_idx] = work_list.back();
                        work_list.pop_back();

                        for (Sink& sink: circuit_graph.sink_map.at(curr_cell).at(curr_port).at(curr_idx)) {
                            if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                                continue;
                            }

                            auto [sink_cell, sink_port, _sink_idx] = std::get<CellPin>(sink);

                            // Push it back to the returned list if a DFF is found
                            // Do no propagate through temporal boundary
                            if (RTLIL::builtin_ff_cell_types().count(sink_cell->type)) {
                                ret.insert(sink_cell);
                                continue;
                            }

                            // Propagate through combinatorial cells
                            for (const auto& [sink_out_port, sink_out_sig]: sink_cell->connections()) {
                                if (sink_cell->output(sink_out_port)) {
                                    for (int i = 0; i < GetSize(sink_out_sig); ++i) {
                                        work_list.push_back({sink_cell, sink_out_port, i});
                                    }
                                }
                            }
                        }
                    }

                    return ret;
                };

                auto reachable_dff_sources = [&](CellPin ctrl_bit) {
                    std::set<RTLIL::Cell*> ret;

                    std::vector<CellPin> work_list;
                    work_list.push_back(ctrl_bit);
                    while (!work_list.empty()) {
                        // We assume the pins are always outputs of some cells
                        auto [curr_cell, curr_port, curr_idx] = work_list.back();
                        work_list.pop_back();

                        if (RTLIL::builtin_ff_cell_types().count(curr_cell->type)) {
                            ret.insert(curr_cell);
                            continue;
                        }

                        for (const auto& [curr_in_port, curr_in_sig]: curr_cell->connections()) {
                            if (curr_cell->input(curr_in_port)) {
                                for (int i = 0; i < GetSize(curr_in_sig); ++i) {
                                    const Source& source = circuit_graph.source_map.at(curr_cell).at(curr_in_port).at(i);
                                    if (std::holds_alternative<RTLIL::SigBit>(source)) {
                                        continue;
                                    }
                                    work_list.push_back(std::get<CellPin>(source));
                                }
                            }
                        }
                    }

                    return ret;
                };

                // Find potential DFF paths that pass through the chokepoints
                // Remove these paths
                std::set<CellPin> chokepoints = {if_src_pin, if_other_src_pin, if_sink_pin, if_other_sink_pin};
                std::map<RTLIL::Cell*, std::set<RTLIL::Cell*>> removals;
                for (CellPin chokepoint: chokepoints) {
                    std::set<RTLIL::Cell*> dff_srcs = reachable_dff_sources(chokepoint);
                    std::set<RTLIL::Cell*> dff_sinks = reachable_dff_sinks(chokepoint);

                    for (RTLIL::Cell* dff_src: dff_srcs) {
                        if (removals.count(dff_src) == 0) {
                            removals[dff_src] = {};
                        }
                        removals[dff_src].insert(dff_sinks.begin(), dff_sinks.end());
                    }
                }

                // Test which DFFs are reachable to which
                // For each source DFF, try to reach one of the sink DFFs
                auto cut_graph_dff_reachable_from_ctrl = [&](
                    const std::set<CellPin> ctrl_bit_critical_sinks,
                    RTLIL::Cell* ctrl_fsm_dff,
                    RTLIL::Cell* target_dff,
                    std::map<RTLIL::Cell*, std::set<RTLIL::Cell*>> removed_edges
                ) {
                    std::set<RTLIL::Cell*> dff_direct_sinks;
                    for (const CellPin& crit_sink_pin: ctrl_bit_critical_sinks) {
                        // Convert sink pins into outputs
                        auto [crit_cell, crit_port, crit_idx] = crit_sink_pin;
                        if (RTLIL::builtin_ff_cell_types().count(crit_cell->type)) {
                            dff_direct_sinks.insert(crit_cell);
                            continue;
                        }

                        for (const auto& [out_port, out_sig]: crit_cell->connections()) {
                            if (crit_cell->output(out_port)) {
                                for (int i = 0; i < GetSize(out_sig); ++i) {
                                    std::set<RTLIL::Cell*> reachables = reachable_dff_sinks({
                                        crit_cell, out_port, i
                                    });
                                    dff_direct_sinks.insert(reachables.begin(), reachables.end());
                                }
                            }
                        }
                    }

                    for (RTLIL::Cell* direct_sink: dff_direct_sinks) {
                        // First, the target DFFs could be reached directly
                        if (target_dff == direct_sink) {
                            return true;
                        }

                        // Ignore data registers
                        if (data_regs.count(direct_sink)) {
                            continue;
                        }

                        // It is also possible to reach the target indirectly
                        if (cut_graph_dff_reachable(
                            direct_sink,
                            target_dff,
                            removed_edges
                        )) {
                            return true;
                        }
                    }

                    return false;
                };

                bool this_src_this_sink_reachable = cut_graph_dff_reachable_from_ctrl(
                    if_src_next_pins, if_src_dff, if_sink_dff, removals
                );
                bool this_src_other_sink_reachable = cut_graph_dff_reachable_from_ctrl(
                    if_src_next_pins, if_src_dff, if_other_sink_dff, removals
                );
                bool other_src_this_sink_reachable = cut_graph_dff_reachable_from_ctrl(
                    if_other_src_next_pins, if_other_src_dff, if_sink_dff, removals
                );
                bool other_src_other_sink_reachable = cut_graph_dff_reachable_from_ctrl(
                    if_other_src_next_pins, if_other_src_dff, if_other_sink_dff, removals
                );

                if (this_src_this_sink_reachable) {
                    log("DFF %s and %s should be grouped up together\n", log_id(if_src_dff), log_id(if_sink_dff));
                    valid_ready_pair.insert({if_other_src_pin, if_src_pin});
                    valid_ready_pair.insert({if_sink_pin, if_other_sink_pin});
                }

                if (this_src_other_sink_reachable) {
                    log("DFF %s and %s should be grouped up together\n", log_id(if_src_dff), log_id(if_other_sink_dff));
                    valid_ready_pair.insert({if_other_src_pin, if_src_pin});
                    valid_ready_pair.insert({if_other_sink_pin, if_sink_pin});
                }

                if (other_src_this_sink_reachable) {
                    log("DFF %s and %s should be grouped up together\n", log_id(if_other_src_dff), log_id(if_sink_dff));
                    valid_ready_pair.insert({if_src_pin, if_other_src_pin});
                    valid_ready_pair.insert({if_other_sink_pin, if_sink_pin});
                }

                if (other_src_other_sink_reachable) {
                    log("DFF %s and %s should be grouped up together\n", log_id(if_other_src_dff), log_id(if_other_sink_dff));
                    valid_ready_pair.insert({if_other_src_pin, if_src_pin});
                    valid_ready_pair.insert({if_other_sink_pin, if_sink_pin});
                }

                if (
                    !this_src_this_sink_reachable &&
                    !this_src_other_sink_reachable &&
                    !other_src_this_sink_reachable &&
                    !other_src_other_sink_reachable
                ) {
                    log("Grouping unclear!\n");
                }
            }
        }

        log("\n");
        for (auto [valid, ready]: valid_ready_pair) {
            log("Valid: %s:%s\n", log_id(std::get<0>(valid)), std::get<1>(valid).c_str());
            log("Ready: %s:%s\n", log_id(std::get<0>(ready)), std::get<1>(ready).c_str());
            log("\n");
        }
    }
} ValidReadyPass;

PRIVATE_NAMESPACE_END
