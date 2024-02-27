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

        log("FF with clock enable:\n");
        for (RTLIL::Cell* ff_cell: ff_with_enable) {
            log("FF Cell: %s; FF type: %s\n", ff_cell->name.c_str(), ff_cell->type.c_str());
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

        for (RTLIL::Cell* ff_cell: self_loop_dffe) {
            LOG("Filtered FF Cell: %s; FF type: %s\n", ff_cell->name.c_str(), ff_cell->type.c_str());
        }

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

            // for (RTLIL::Cell* dffe_src: dffe_sources) {
            //     log("%s -> %s\n", dffe_src->name.c_str(), dffe->name.c_str());
            // }

            // for (RTLIL::Cell* dffe_sink: dffe_sinks) {
            //     log("%s <- %s\n", dffe_sink->name.c_str(), dffe->name.c_str());
            // }

            {
                std::vector<RTLIL::Cell*> work_list;
                work_list.push_back(dffe);

                while (!work_list.empty()) {
                    RTLIL::Cell* cell = work_list.back();
                    work_list.pop_back();

                    // The cell can reach all its output
                    for (const auto& [port, sink_lists]: circuit_graph.sink_map[cell]) {
                        // Reject inputs & clocks
                        if (!cell->output(port)) {
                            continue;
                        }

                        for (const std::vector<Sink>& sink_list: sink_lists) {
                            for (const Sink& sink: sink_list) {
                                // Reject non-cell connections
                                if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                                    continue;
                                }

                                const auto& [sink_cell, sink_port, _sink_pin_idx] = std::get<CellPin>(sink);

                                // Only propagate the sink if it is not a flip flop
                                if (RTLIL::builtin_ff_cell_types().count(sink_cell->type) == 0) {
                                    work_list.push_back(sink_cell);
                                }
                            }
                        }
                    }
                }
            }

            std::set<RTLIL::Cell*> viable_intermediate_dffs;
            for (RTLIL::Cell* dffe_src: dffe_sources) {
                // Not interested in self-loop 1-hop
                if (dffe_src == dffe) {
                    continue;
                }

                if (dffe_sinks.count(dffe_src)) {
                    viable_intermediate_dffs.insert(dffe_src);
                }
            }

            LOG("Viable 1-hop intermeediate DFF:\n");
            for (RTLIL::Cell* inter_dff: viable_intermediate_dffs) {
                LOG("%s\n", inter_dff->name.c_str());
            }
            
            // Determine which non-SCC source is related to ready bit
            std::set<CellPin> rdy_candidates;
            for (const CellPin& potential_rdy_bit: non_dominated_sources) {
                std::set<RTLIL::Cell*> cell_intersections;
                for (RTLIL::Cell* inter_dff: viable_intermediate_dffs) {
                    // Try all Q pins
                    int q_port_size = GetSize(inter_dff->getPort("\\Q"));
                    for (int q_idx = 0; q_idx < q_port_size; ++q_idx) {
                        std::set<RTLIL::Cell*> cells_in_path = circuit_graph.get_intermediate_comb_cells(
                            {inter_dff, "\\Q", q_idx}, potential_rdy_bit);
                        cell_intersections.insert(cells_in_path.begin(), cells_in_path.end());
                    }
                }
                if (!cell_intersections.empty()) {
                    rdy_candidates.insert(potential_rdy_bit);
                }
            }

            // Save the candidates to a yosys set
            log("Found %ld ready bit candidate\n", rdy_candidates.size());
            for (const CellPin& candidate_pin: rdy_candidates) {
                log("Cell %s; Port %s\n", std::get<0>(candidate_pin)->name.c_str(), std::get<1>(candidate_pin).c_str());
            }
        }
    }
} ValidReadyPass;

PRIVATE_NAMESPACE_END
