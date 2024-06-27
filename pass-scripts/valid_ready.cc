#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/ff.h"
#include "graph.h"
#include "data_path.h"
#include "partition.h"
#include "util.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


bool PASS_DEBUG = 0;
bool PASS_DECOMP = 0;
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
            if (args[1] == "-decomp") {
                PASS_DECOMP = 1;
            } else {
                PASS_DECOMP = 0;
            }
        } else {
            PASS_DEBUG = 0;
            PASS_DECOMP = 0;
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
                            for (int j = 0; j < GetSize(ff_data.sig_d); ++j) {
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
            log("Analyze DFFE %s\n", log_id(dffe));
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
            } else {
                log("DFFE %s is likely a FSM\n", log_id(dffe));
            }
        }

        for (RTLIL::Cell* independent_dff: independent_self_loop_dffs) {
            log("Remove independent DFF %s from set\n", log_id(independent_dff));
            self_loop_dff.erase(independent_dff);
        }

        log("Begin data path analysis\n");
        DataPath data_path(sigmap, top);
        log("End data path analysis\n");

        // Find data registers
        // TODO: Define data registers as registers with CE but not classified as FSMs
        log("\n");
        log("Formulate data registers\n");
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

        // Record the most senior primary output, mapped by
        // directed edges from a DFF to another
        //
        // FIXME: The pimary output should converge? Given that there should be a 1-bit edge
        // connecting the DFFs on the path.
        dict<std::pair<RTLIL::Cell*, RTLIL::Cell*>, std::pair<std::set<RTLIL::Cell*>, std::set<CellPin>>> dff_edge_primary_outputs;

        // Set of interconnected DFF groups
        std::set<ConnectedModules> handshake_compute_buffer;

        // Find the strongly connected component.
        for (RTLIL::Cell* dff: self_loop_dff) {
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

                // The looped-in DFFE should also control a valid/ready bit
                // by symmetry
                if (self_loop_dff.count(dff_src) == 0) {
                    continue;
                }

                if (dff_sinks.count(dff_src)) {
                    viable_intermediate_dffs.insert(dff_src);
                    handshake_compute_buffer.insert(ConnectedModules({dff}, {dff_src}));
                }
            }

            log("Viable 1-hop intermediate DFF:\n");
            for (RTLIL::Cell* inter_dff: viable_intermediate_dffs) {
                log("%s\n", inter_dff->name.c_str());
            }
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
        std::map<Handshake, std::set<std::pair<RTLIL::Cell*, int>>> eligible_ctrl_bit_pairs;
        std::map<Handshake, std::map<CellPin, std::set<CellPin>>> cut_edges;

        while (!handshake_compute_buffer.empty()) {
            for (const ConnectedModules& potential_handshake: handshake_compute_buffer) {
                auto [source_fsm_ffs, sink_fsm_ffs] = potential_handshake.decompose();
                auto [half_path_set, primary_outputs] = circuit_graph.get_primary_outputs(
                    source_fsm_ffs,
                    sink_fsm_ffs,
                    &ff_init_vals
                );

                auto [opposite_half_path_set, opposite_primary_outputs] = circuit_graph.get_primary_outputs(
                    sink_fsm_ffs,
                    source_fsm_ffs,
                    &ff_init_vals
                );

                // Discard this potential handshake if any set of primary outputs turn out to be empty
                if (primary_outputs.empty() || opposite_primary_outputs.empty()) {
                    continue;
                }

                log("\n\n");
                log("Examime from FFs:\n");
                for (RTLIL::Cell* src_ff: source_fsm_ffs) {
                    log("%s\n", log_id(src_ff));
                }
                log("To FFs:\n");
                for (RTLIL::Cell* sink_ff: sink_fsm_ffs) {
                    log("%s\n", log_id(sink_ff));
                }
                for (const CellPin& ctrl_bit: primary_outputs) {
                    log("From primary output: %s:%s\n", log_id(std::get<0>(ctrl_bit)), log_id(std::get<1>(ctrl_bit)));
                }
                for (const CellPin& ctrl_bit: opposite_primary_outputs) {
                    log("To primary output: %s:%s\n", log_id(std::get<0>(ctrl_bit)), log_id(std::get<1>(ctrl_bit)));
                }

                for (const CellPin& ctrl_bit: primary_outputs) {
                    for (const CellPin& other_ctrl_bit: opposite_primary_outputs) {
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
                                auto [src_first_divergence, src_depth] = circuit_graph.get_first_divergence(
                                    ctrl_bit, half_path_set,
                                    ctrl_bit_to_data_path_set, opposite_ctrl_bit_to_data_path);

                                auto [sink_first_divergence, sink_depth] = circuit_graph.get_first_divergence(
                                    other_ctrl_bit, opposite_half_path_set,
                                    opposite_ctrl_bit_to_data_path, ctrl_bit_to_data_path_set);

                                log("Cell %s Port %s reaches Cell %s CE\n",
                                    std::get<0>(ctrl_bit)->name.c_str(),
                                    std::get<1>(ctrl_bit).c_str(),
                                    log_id(data_reg));
                                log("Other ctrl pin: Cell %s; Port %s\n",
                                    std::get<0>(other_ctrl_bit)->name.c_str(),
                                    std::get<1>(other_ctrl_bit).c_str());

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
                                                edges_to_be_cut_opt[sink_first_divergence].insert(first_div_sink_pin);
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Control bits may also be sourced to any FSM control pin,
                        // then to datapath control pin
                        //
                        // This will introduce 1 clock cycle delay
                        for (RTLIL::Cell* inter_dff: self_loop_dff) {
                            // The intermediate DFF should not be either the sink or the source DFFs
                            if (source_fsm_ffs.count(inter_dff) || sink_fsm_ffs.count(inter_dff)) {
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
                                auto [src_first_divergence, src_depth] = circuit_graph.get_first_divergence(
                                    ctrl_bit, half_path_set,
                                    ctrl_data_path_set, opposite_ctrl_data_path_set);

                                auto [sink_first_divergence, sink_depth] = circuit_graph.get_first_divergence(
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
                                            LOG("DFFE reaches MUX %s\n", log_id(mux));
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
                                                    if (opposite_half_path_set.count(std::get<0>(first_div_sink_pin))) {
                                                        edges_to_be_cut_opt[sink_first_divergence].insert(first_div_sink_pin);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        if (joint_depth != std::numeric_limits<int>::max()) {
                            Handshake eligible_pair(
                                src_first_divergence_opt,
                                source_fsm_ffs,
                                sink_first_divergence_opt,
                                sink_fsm_ffs
                            );

                            eligible_ctrl_bit_pairs[eligible_pair] = dependent_data_components;
                            cut_edges[eligible_pair] = edges_to_be_cut_opt;

                            log("A handshake candidate is found, handshake buffer size: %ld\n", eligible_ctrl_bit_pairs.size());
                            for (RTLIL::Cell* source_ff: source_fsm_ffs) {
                                log("Source FF %s\n", log_id(source_ff));
                            }
                            for (RTLIL::Cell* sink_ff: sink_fsm_ffs) {
                                log("Sink FF %s\n", log_id(sink_ff));
                            }
                            log("Has %ld data components\n", dependent_data_components.size());
                            log("\n\n");
                        }
                    }
                }
            }
            // Clear compute buffer, as the computation is completed.
            handshake_compute_buffer.clear();

            log("Eligible ctrl bit pairs: %ld\n", eligible_ctrl_bit_pairs.size());
            for (const auto& [eligible_pair, data_components]: eligible_ctrl_bit_pairs) {
                auto [handshake_src, handshake_sink] = eligible_pair.decompose();
                auto [eligible_src, dffe_srcs] = handshake_src;
                auto [eligible_sink, dffe_sinks] = handshake_sink;

                log("Ctrl src: Cell %s Port %s\n",
                    std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str()
                );
                log("Src FFs:\n");
                for (RTLIL::Cell* dffe_src: dffe_srcs) {
                    log("%s\n", log_id(dffe_src));
                }
                log("Ctrl sink: Cell %s Port %s\n",
                    std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str()
                );
                log("Sink FFs:\n");
                for (RTLIL::Cell* dffe_sink: dffe_sinks) {
                    log("%s\n", log_id(dffe_sink));
                }
                log("\n");
            }

            /*
                Run passes to pruify the handshake results
                1. Check that the valid/ready bits are separable
                2. Find FFs that belong to the same group by:
                    - Check against FF groups that have multiple point of contacts
                3. Remove different valid/ready pairs that share the same backing flip flops using (2)
                4. Classify flip flops as the same group using the reuslt of (2)
                5. Finally, deduplicate handshakes with identical control bits
            */

            // Check if the ready bits and valid bits are functionally separable
            std::set<Handshake> cross_influenced_handshake;
            for (const auto& [eligible_pair, _data_components]: eligible_ctrl_bit_pairs) {
                auto [handshake_src, handshake_sink] = eligible_pair.decompose();
                auto [eligible_src, dffe_srcs] = handshake_src;
                auto [eligible_sink, dffe_sinks] = handshake_sink;

                std::set<RTLIL::Cell*> src_path;
                for (RTLIL::Cell* single_dffe_src: dffe_srcs) {
                    for (int i = 0; i < GetSize(single_dffe_src->getPort("\\Q")); ++i) {
                        std::set<RTLIL::Cell*> src_i_path = circuit_graph.get_intermediate_comb_cells(
                            {single_dffe_src, "\\Q", i}, eligible_src);
                        src_path.insert(src_i_path.begin(), src_i_path.end());
                    }
                }

                std::set<RTLIL::Cell*> sink_path;
                for (RTLIL::Cell* single_dffe_sink: dffe_sinks) {
                    for (int i = 0; i < GetSize(single_dffe_sink->getPort("\\Q")); ++i) {
                        std::set<RTLIL::Cell*> sink_i_path = circuit_graph.get_intermediate_comb_cells(
                            {single_dffe_sink, "\\Q", i}, eligible_sink);
                        sink_path.insert(sink_i_path.begin(), sink_i_path.end());
                    }
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
                    cross_influenced_handshake.insert(eligible_pair);
                    log("Found intertwined path between Cell %s Port %s and Cell %s Port %s\n",
                        std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(),
                        std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str());
                }

                // Ensure there are not mutual interference
                std::set<RTLIL::Cell*> dffe_src_sink_path;
                for (RTLIL::Cell* single_dffe_src: dffe_srcs) {
                    for (int i = 0; i < GetSize(single_dffe_src->getPort("\\Q")); ++i) {
                        std::set<RTLIL::Cell*> path_i = circuit_graph.get_intermediate_comb_cells(
                            {single_dffe_src, "\\Q", i}, eligible_sink);
                        dffe_src_sink_path.insert(path_i.begin(), path_i.end());
                    }
                }

                std::set<RTLIL::Cell*> dffe_sink_src_path;
                for (RTLIL::Cell* single_dffe_sink: dffe_sinks) {
                    for (int i = 0; i < GetSize(single_dffe_sink->getPort("\\Q")); ++i) {
                        std::set<RTLIL::Cell*> path_i = circuit_graph.get_intermediate_comb_cells(
                            {single_dffe_sink, "\\Q", i}, eligible_src);
                        dffe_sink_src_path.insert(path_i.begin(), path_i.end());
                    }
                }

                if (!dffe_src_sink_path.empty() || !dffe_sink_src_path.empty()) {
                    log("Cell %s:%s and Cell %s:%s are combinatorially influenced by opposite DFFE\n",
                        std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str(),
                        std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str());
                    cross_influenced_handshake.insert(eligible_pair);
                }
            }

            // Commit with removal
            for (const Handshake& removing_x_influence: cross_influenced_handshake) {
                eligible_ctrl_bit_pairs.erase(removing_x_influence);
            }

            // Re-classify potential handshake pairs
            // 1. No FSMs should have multiple valid/ready candidates
            // In this case, classify both infracting FFs as within the same module
            std::set<std::set<RTLIL::Cell*>> same_module_ffs_list;
            // Prune these from the previously found handshakes
            std::set<Handshake> intra_module_handshakes;
            {
                // A cache for handshakes that had been encountered, mapped by its driving DFFs
                // It is just to make handshake removal a bit simpler
                std::map<std::set<RTLIL::Cell*>, Handshake> observed_ff_linkages;
                for (const auto& [handshake, _data_components]: eligible_ctrl_bit_pairs) {
                    auto [handshake_src, handshake_sink] = handshake.decompose();
                    auto [eligible_src, dffe_src] = handshake_src;
                    auto [eligible_sink, dffe_sink] = handshake_sink;

                    std::set<RTLIL::Cell*> curr_ffs;
                    curr_ffs.insert(dffe_src.begin(), dffe_src.end());
                    curr_ffs.insert(dffe_sink.begin(), dffe_sink.end());

                    if (observed_ff_linkages.count(curr_ffs)) {
                        same_module_ffs_list.insert(curr_ffs);
                        intra_module_handshakes.insert(handshake);
                        // Insert the handshake that had been cached as well
                        intra_module_handshakes.insert(observed_ff_linkages.at(curr_ffs));
                    } else {
                        observed_ff_linkages.insert({curr_ffs, handshake});
                    }
                }
            }

            for (const Handshake& vr_handshake: intra_module_handshakes) {
                eligible_ctrl_bit_pairs.erase(vr_handshake);
            }
            log("Handshake counts after same module removal: %ld\n", eligible_ctrl_bit_pairs.size());

            // FIXME: Coalesce the FF sets, so any entries that share common cells would be joined

            // 2. Classify handshakes that has the same module source as the same handshake
            // We do so by joining the handshake ff sources of the handshake entries
            std::set<Handshake> original_handshake_removal;
            {
                for (std::set<RTLIL::Cell*> same_module_ffs: same_module_ffs_list) {
                    std::map<std::set<RTLIL::Cell*>, std::set<Handshake>> other_ctrl_map;
                    for (const auto& [handshake, _data_components]: eligible_ctrl_bit_pairs) {
                        // If the backing DFFs are a subset of the FFs in the same module,
                        // consider FF expansion
                        std::set<RTLIL::Cell*> opposite_if = handshake.get_opposite_backing_if_backed(same_module_ffs);
                        if (!opposite_if.empty()) {
                            if (other_ctrl_map.count(opposite_if) == 0) {
                                other_ctrl_map[opposite_if] = {};
                            }
                            other_ctrl_map[opposite_if].insert(handshake);
                        }
                    }

                    // Examine the map of opposite controls
                    // If any key has more than 1 handshakes,
                    // update the ff backings with all FFs that backed the handshakes
                    for (const auto& [opposite_if, handshake_copies]: other_ctrl_map) {
                        if (handshake_copies.size() > 1) {
                            // Compute union
                            std::set<RTLIL::Cell*> backing_union;
                            for (const Handshake& handshake: handshake_copies) {
                                // Opposite of opposite is unity
                                std::set<RTLIL::Cell*> backed_ffs = handshake.get_opposite_backing(opposite_if);
                                backing_union.insert(backed_ffs.begin(), backed_ffs.end());
                            }

                            // Update backing, and store it in the recompute buffer
                            for (Handshake original_handshake: handshake_copies) {
                                Handshake new_handshake = original_handshake;
                                new_handshake.insert_backing_ff(backing_union);
                                ConnectedModules uncomputed_handshake(new_handshake);

                                // Store it in the recompute buffer for recomputation
                                handshake_compute_buffer.insert(uncomputed_handshake);
                                // The old handshake should be removed
                                original_handshake_removal.insert(original_handshake);
                            }
                        }
                    }
                }
            }
            for (const Handshake& recomputing_handshake: original_handshake_removal) {
                eligible_ctrl_bit_pairs.erase(recomputing_handshake);
            }
            log("Recompute buffer size: %ld\n", handshake_compute_buffer.size());
        }

        log("Eligible ctrl bit pairs: %ld\n", eligible_ctrl_bit_pairs.size());

        // Repopulate eligible pairs by de-duplication of handshakes with the same control bits
        {
            std::map<HandshakeBits, std::set<std::pair<Handshake, std::set<std::pair<RTLIL::Cell*, int>>>>> handshake_bits_collection;
            for (const auto& [handshake, data_components]: eligible_ctrl_bit_pairs) {
                HandshakeBits handshake_bits(handshake);
                if (handshake_bits_collection.count(handshake_bits)) {
                    handshake_bits_collection[handshake_bits].insert(
                        {handshake, data_components}
                    );
                } else {
                    handshake_bits_collection.insert({
                        handshake_bits, {{handshake, data_components}}
                    });
                }
            }

            // If any handshake bits pair maps to multiple handshakes,
            // remove these handshakes from the set and re-insert the union
            // std::map<Handshake, std::set<std::pair<RTLIL::Cell*, int>>> handshake_buffer;
            for (const auto& [handshake_bits, handshake_info_set]: handshake_bits_collection) {
                auto [ctrl0, ctrl1] = handshake_bits.decompose();
                if (handshake_info_set.size() > 1) {
                    // Backing FF union
                    std::set<RTLIL::Cell*> backing_ffs0;
                    std::set<RTLIL::Cell*> backing_ffs1;
                    std::set<std::pair<RTLIL::Cell*, int>> data_union;
                    for (const auto& [handshake, data_components]: handshake_info_set) {
                        std::set<RTLIL::Cell*> ctrl0_backing = handshake.get_backing_ffs(ctrl0);
                        std::set<RTLIL::Cell*> ctrl1_backing = handshake.get_backing_ffs(ctrl1);

                        backing_ffs0.insert(ctrl0_backing.begin(), ctrl0_backing.end());
                        backing_ffs1.insert(ctrl1_backing.begin(), ctrl1_backing.end());
                        data_union.insert(data_components.begin(), data_components.end());
                    }

                    // Remove original handshake info, repopulate with the union
                    Handshake handshake_pair(
                        ctrl0, backing_ffs0, ctrl1, backing_ffs1
                    );

                    for (const auto& [handshake, _data_components]: handshake_info_set) {
                        eligible_ctrl_bit_pairs.erase(handshake);
                    }
                    eligible_ctrl_bit_pairs.insert({handshake_pair, data_union});
                }
            }
        }

        std::set<RTLIL::Cell*> global_data_interfaces;
        std::set<RTLIL::Cell*> experimental_data_ifs;
        std::set<RTLIL::Cell*> experimental_data_ends;

        std::map<Handshake, std::set<Wire>> module_data_interfaces;

        log("New handshake count: %ld\n", eligible_ctrl_bit_pairs.size());
        for (const auto& [eligible_pair, data_components]: eligible_ctrl_bit_pairs) {
            auto [handshake_src, handshake_sink] = eligible_pair.decompose();
            auto [eligible_src, dffe_srcs] = handshake_src;
            auto [eligible_sink, dffe_sinks] = handshake_sink;

            log("Ctrl src: Cell %s Port %s\n",
                std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str()
            );
            log("Src FFs:\n");
            for (RTLIL::Cell* dffe_src: dffe_srcs) {
                log("%s\n", log_id(dffe_src));
            }
            log("Ctrl sink: Cell %s Port %s\n",
                std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str()
            );
            log("Sink FFs:\n");
            for (RTLIL::Cell* dffe_sink: dffe_sinks) {
                log("%s\n", log_id(dffe_sink));
            }
            log("\n");
        }

        {
            std::stringstream ss;
            ss << "select -set handshake_bits ";

            for (const auto& [eligible_pair, _data_components]: eligible_ctrl_bit_pairs) {
                auto [handshake_src, handshake_sink] = eligible_pair.decompose();
                auto [eligible_src, dffe_srcs] = handshake_src;
                auto [eligible_sink, dffe_sinks] = handshake_sink;

                ss << "c:" << std::get<0>(eligible_src)->name.c_str() << " ";
                ss << "c:" << std::get<0>(eligible_sink)->name.c_str() << " ";
            }

            Pass::call(design, ss.str().c_str());
        }

        {
            int handshake_idx = 0;
            for (const auto& [_eligible_pair, data_components]: eligible_ctrl_bit_pairs) {
                std::stringstream ss;
                ss << "select -set handshake_data_comp" << handshake_idx << " ";
                for (auto [data_comp, _priority]: data_components) {
                    ss << "c:" << data_comp->name.c_str() << " ";
                }

                Pass::call(design, ss.str().c_str());
                handshake_idx++;
            }
        }

        if (!PASS_DECOMP) {
            return;
        }

        // Perform data components appearance counting
        std::map<RTLIL::Cell*, std::set<Handshake>> appearance_counter;
        for (const auto& [handshake, data_components]: eligible_ctrl_bit_pairs) {
            for (auto [data_cell, _priority]: data_components) {
                if (appearance_counter.count(data_cell) == 0) {
                    appearance_counter.insert({data_cell, {}});
                }
                appearance_counter[data_cell].insert(handshake);
            }
        }
        log("Unique data cell found: %d\n", GetSize(appearance_counter));

        // Create a set of initial subgraphs
        // This encapsulate the absolute minimum or maximum of the frontier
        // We really only need the maximum to stop propagation of frontier
        std::map<Handshake, std::set<std::shared_ptr<DataElement>>> initial_subgraphs;
        for (const auto& [handshake, data_components]: eligible_ctrl_bit_pairs) {
            // Acquire pure data components
            std::set<RTLIL::Cell*> data_cells;
            for (auto [cell, _priority]: data_components) {
                data_cells.insert(cell);
            }
            initial_subgraphs[handshake] = data_path.get_subgraph(data_cells);
        }

        {
            int hs_idx = 0;
            for (const auto& [_hs, data_elms]: initial_subgraphs) {
                std::stringstream ss;
                ss << "select -set initial_subgraph" << hs_idx << " ";
                for (std::shared_ptr<DataElement> data_elm: data_elms) {
                    for (RTLIL::Cell* subgraph_node: data_elm->subgraph_nodes) {
                        ss << "c:" << subgraph_node->name.c_str() << " ";
                    }
                }

                Pass::call(design, ss.str().c_str());
                ++hs_idx;
            }
        }

        // Create a provisional subgraph for every handshake entry
        // using the least appeared elements
        std::map<Handshake, std::set<std::shared_ptr<DataElement>>> provisional_subgraphs;
        std::map<Handshake, std::set<std::shared_ptr<DataElement>>> finalized_subgraphs;

        // The collection of frontiers associated to the corresponding handshakes
        std::map<Handshake, std::vector<std::set<std::shared_ptr<DataElement>>>> frontier_collections;

        auto get_provisional_subgraph = [&](
            const Handshake& curr_handshake
        ) {
            const std::set<std::pair<RTLIL::Cell*, int>>& data_components = eligible_ctrl_bit_pairs.at(curr_handshake);
            int least_frequency = std::numeric_limits<int>::max();
            std::set<RTLIL::Cell*> least_appeared_elements;
            for (auto [cell, _priority]: data_components) {
                int frequency = appearance_counter.at(cell).size();
                if (frequency < least_frequency) {
                    least_frequency = frequency;
                    least_appeared_elements.clear();
                }

                if (frequency == least_frequency) {
                    least_appeared_elements.insert(cell);
                }
            }
            return data_path.get_subgraph(least_appeared_elements);
        };

        for (const auto& [handshake, _data_components]: eligible_ctrl_bit_pairs) {
            provisional_subgraphs[handshake] = {};
        }

        // Resolve all subgraphs
        do {
            for (auto& [handshake, provisional_subgraph]: provisional_subgraphs) {
                provisional_subgraph = get_provisional_subgraph(handshake);
            }

            // Start from the smallest subgraph
            Handshake simplest_handshake = std::get<0>(*provisional_subgraphs.begin());
            std::set<std::shared_ptr<DataElement>> smallest_subgraph = std::get<1>(*provisional_subgraphs.begin());

            int least_subgraph_size = 0;
            for (const std::shared_ptr<DataElement>& data_elm: smallest_subgraph) {
                least_subgraph_size += data_elm->subgraph_nodes.size();
            }

            for (const auto& [handshake, subgraph]: provisional_subgraphs) {
                int subgraph_size = 0;
                for (const std::shared_ptr<DataElement>& data_elm: subgraph) {
                    subgraph_size += data_elm->subgraph_nodes.size();
                }

                if (subgraph_size < least_subgraph_size) {
                    least_subgraph_size = subgraph_size;
                    simplest_handshake = handshake;
                    smallest_subgraph = subgraph;
                }
            }

            // Handshake decided, save the subgraph
            finalized_subgraphs[simplest_handshake] = smallest_subgraph;

            // Guess the data interface
            log("Examine new handshake:\n");
            simplest_handshake.write_log();
            auto get_datapath_input = [&](
                const std::set<std::shared_ptr<DataElement>>& subgraph_nodes
            ){
                std::set<std::shared_ptr<DataElement>> inputs;
                // The data element is an input element if it has no predecessor
                // within the data component set
                for (std::shared_ptr<DataElement> node: subgraph_nodes) {
                    bool node_has_extern_input = false;
                    bool node_has_predecessor = !(node->source.empty());
                    for (std::shared_ptr<DataElement> prev: node->source) {
                        if (smallest_subgraph.count(prev) == 0) {
                            node_has_extern_input = true;
                        }
                    }

                    if (!node_has_predecessor || node_has_extern_input) {
                        inputs.insert(node);
                    }
                }

                return inputs;
            };

            // {
            //     std::stringstream ss;
            //     ss << "select -set data_subgraph_example ";

            //     for (std::shared_ptr<DataElement> data_comp: data_subgraph) {
            //         for (RTLIL::Cell* data_cell: data_comp->subgraph_nodes) {
            //             ss << "c:" << data_cell->name.c_str() << " ";
            //         }
            //     }

            //     Pass::call(design, ss.str().c_str());
            // }

            std::set<std::shared_ptr<DataElement>> data_subgraph_input_raw = get_datapath_input(smallest_subgraph);
            log("Got %ld datapath inputs\n", data_subgraph_input_raw.size());

            // // Replace inputs with successors, until all inputs are not successors or predecessors
            // auto promote_inputs = [&](
            //     const std::set<std::shared_ptr<DataElement>>& subgraph_nodes,
            //     const std::set<std::shared_ptr<DataElement>>& data_inputs
            // ) {
            //     // Find if some input is a successor of some other inputs
            //     std::set<std::shared_ptr<DataElement>> frontier = data_inputs;
            //     std::set<std::shared_ptr<DataElement>> new_frontier;
            //     std::set<std::shared_ptr<DataElement>> removed_frontier;

            //     do {
            //         // Update frontier
            //         for (std::shared_ptr<DataElement> cell: removed_frontier) {
            //             frontier.erase(cell);
            //         }
            //         frontier.insert(new_frontier.begin(), new_frontier.end());
            //         new_frontier.clear();

            //         for (std::shared_ptr<DataElement> frontier_src: frontier) {
            //             for (std::shared_ptr<DataElement> frontier_sink: frontier) {
            //                 if (frontier_src == frontier_sink) {
            //                     continue;
            //                 }
            //                 if (!data_path.get_subgraph_inter_nodes(frontier_src, frontier_sink, subgraph_nodes).empty()) {
            //                     // Move the frontier to its descendents
            //                     removed_frontier.insert(frontier_src);
            //                     for (std::shared_ptr<DataElement> next: frontier_src->sink) {
            //                         if (subgraph_nodes.count(next)) {
            //                             new_frontier.insert(next);
            //                         }
            //                     }
            //                 }
            //             }
            //         }

            //         // Do not propagate to removed frontier
            //         for (std::shared_ptr<DataElement> removed_node: removed_frontier) {
            //             new_frontier.erase(removed_node);
            //         }
            //     } while (!new_frontier.empty());

            //     // No new data elements to be added
            //     // Remove anything that is still in the removed frontier
            //     for (std::shared_ptr<DataElement> cell: removed_frontier) {
            //         frontier.erase(cell);
            //     }

            //     return frontier;
            // };

            std::set<std::shared_ptr<DataElement>> data_subgraph_input = data_path.promote_inputs(
                smallest_subgraph, data_subgraph_input_raw);
            log("Computed subgraph input of size %d\n", GetSize(data_subgraph_input));
            // for (std::shared_ptr<DataElement> single_subgraph_input: data_subgraph_input) {
            //     log("Data cells: \n");
            //     for (RTLIL::Cell* inner: single_subgraph_input->subgraph_nodes) {
            //         log("%s\n", log_id(inner));
            //     }
            //     log("\n");
            // }

            // log("\n\n");

            // Create a finite list of frontiers for each handshake interface, sorted
            std::vector<std::set<std::shared_ptr<DataElement>>> data_elm_iface_list = data_path.collect_frontiers(
                smallest_subgraph, data_subgraph_input
            );
            log("Reporting %d potential interfaces\n", GetSize(data_elm_iface_list));
            frontier_collections[simplest_handshake] = data_elm_iface_list;

            std::set<Wire> data_cut_if = data_path.get_intersection_wires(data_subgraph_input);
            log("Data interface: \n");
            for (auto [from_pin, to_pin]: data_cut_if) {
                auto [from_cell, from_port, from_idx] = from_pin;
                auto [to_cell, to_port, to_idx] = to_pin;
                experimental_data_ends.insert(from_cell);
                log("%s:%s:%d -> %s:%s:%d\n",
                    log_id(from_cell), from_port.c_str(), from_idx,
                    log_id(to_cell), to_port.c_str(), to_idx);
            }
            module_data_interfaces[simplest_handshake] = data_cut_if;

            // Remove the processed handshake
            provisional_subgraphs.erase(simplest_handshake);

            // Remove nodes that are successor to the guessed interface from the possible data components
            // This includes the interfacing nodes themselves
            for (auto& [handshake, data_components]: eligible_ctrl_bit_pairs) {
                std::set<RTLIL::Cell*> successor_graph = data_path.get_subgraph_successor_cells(
                    smallest_subgraph, data_cut_if
                );

                for (RTLIL::Cell* used_successor: successor_graph) {
                    // TODO: Get rid of the priority system
                    // We hack by erasing all possible priority for now.
                    for (int i = 0; i < 4; ++i) {
                        data_components.erase({used_successor, i});
                    }
                }
            }

            // Update element counter
            for (auto& [data_cell, collection]: appearance_counter) {
                collection.erase(simplest_handshake);
            }
        } while (!provisional_subgraphs.empty());

        // DEBUG
        {
            int hs_idx = 0;
            for (const auto& [_hs, data_if]: module_data_interfaces) {
                std::stringstream ss;
                ss << "select -set initial_data_if" << hs_idx << " ";
                for (auto [from_cell, to_cell]: data_if) {
                    ss << "c:" << std::get<0>(to_cell)->name.c_str() << " ";
                }

                Pass::call(design, ss.str().c_str());
                ++hs_idx;
            }

            hs_idx = 0;
            for (const auto& [hs, data_if_vec]: frontier_collections) {
                hs.write_ctrl_pins();
                for (int frontier_idx = 0; frontier_idx < GetSize(data_if_vec); ++frontier_idx) {
                    {
                        std::stringstream ss;
                        ss << "select -set frontier_data_if" << hs_idx << "_" << frontier_idx << " ";
                        log("Created set: %s\n", ss.str().c_str());
                        std::set<Wire> data_if = data_path.get_intersection_wires(data_if_vec.at(frontier_idx));
                        std::set<RTLIL::Cell*> data_if_sink;
                        for (auto [_from_cell, to_cell]: data_if) {
                            ss << "c:" << std::get<0>(to_cell)->name.c_str() << " ";
                            data_if_sink.insert(std::get<0>(to_cell));
                        }
                        log("Has data sink size: %d\n", GetSize(data_if_sink));

                        Pass::call(design, ss.str().c_str());
                    }

                    {
                        std::stringstream ss;
                        ss << "select -set frontier_data_if_src" << hs_idx << "_" << frontier_idx << " ";
                        log("Created set: %s\n", ss.str().c_str());
                        std::set<Wire> data_if = data_path.get_intersection_wires(data_if_vec.at(frontier_idx));
                        std::set<RTLIL::Cell*> data_if_sink;
                        for (auto [from_cell, _to_cell]: data_if) {
                            ss << "c:" << std::get<0>(from_cell)->name.c_str() << " ";
                            data_if_sink.insert(std::get<0>(from_cell));
                        }
                        log("Has data source size: %d\n", GetSize(data_if_sink));

                        Pass::call(design, ss.str().c_str());
                    }
                }
                ++hs_idx;
            }
        }
        // DEBUG END

        // Prune interfaces that are self-overlapping
        // Idea: If the FFs belong to the same module interface, it should conclude
        // the same valid/ready bit set using the first divergence

        // Record in Yosys set
        {
            std::stringstream ss;
            ss << "select -set data_comp ";

            for (RTLIL::Cell* data_comp: experimental_data_ifs) {
                ss << "c:" << data_comp->name.c_str() << " ";
            }

            Pass::call(design, ss.str().c_str());
        }

        {
            std::stringstream ss_end;
            ss_end << "select -set data_ends ";

            for (RTLIL::Cell* data_comp: experimental_data_ends) {
                ss_end << "c:" << data_comp->name.c_str() << " ";
            }

            Pass::call(design, ss_end.str().c_str());
        }


        // Pass::call(design, "show -color red @data_comp");

        // TODO: Establish connectivity with an undirected graph
        // We establish the directionality later
        std::map<std::pair<Handshake, CellPin>, std::set<std::pair<Handshake, CellPin>>> ctrl_pin_connectivity;

        {
            // 1. Collect all roadblocks.
            // Our search should not touch data components nor control bits
            // Note: We should replace data components with data interfaces
            // However, the data interface guessing has dubious accuracy

            // A map from control pins to its corresponding handshakes
            std::map<CellPin, std::set<Handshake>> ctrl_pins;
            for (const auto& [handshake, _data_if]: module_data_interfaces) {
                auto [handshake_if0, handshake_if1] = handshake.decompose();
                auto [ctrl0, _dffs0] = handshake_if0;
                auto [ctrl1, _dffs1] = handshake_if1;

                for (CellPin ctrl_bit: {ctrl0, ctrl1}) {
                    if (ctrl_pins.count(ctrl_bit) == 0) {
                        ctrl_pins.insert({ctrl_bit, {}});
                    }

                    ctrl_pins[ctrl_bit].insert(handshake);
                }
            }

            // A set of data components
            std::set<RTLIL::Cell*> blocking_data_components;
            // FIXME: Any better candidates than the data cells?
            blocking_data_components = data_path_cell;

            // 2. For each control bit, find its dependent control bit and its associated bit
            auto get_ctrl_connectivity = [&](
                // Convention: This pin is an output pin
                CellPin origin,
                Handshake origin_handshake
            ) -> std::set<std::pair<Handshake, CellPin>> {
                std::set<std::pair<RTLIL::Cell*, int>> reached_cell_chs;
                std::set<std::pair<Handshake, CellPin>> source_handshakes;

                // Convention: Work list should only hold output pins
                std::vector<CellPin> work_list = {origin};
                while (!work_list.empty()) {
                    CellPin curr = work_list.back();
                    work_list.pop_back();

                    // Update reached information to avoid loop
                    // Note: All RTLIL cells in default cell lib only has 1 output port
                    auto [sink_cell, _sink_port, sink_port_idx] = curr;
                    reached_cell_chs.insert({sink_cell, sink_port_idx});

                    // Stop propagation if:
                    // - the current pin is a control bit of some other handshake
                    // - It is a different pin from the origin and it is a control bit
                    if (ctrl_pins.count(curr)) {
                        std::set<Handshake> controlling_handshakes = ctrl_pins.at(curr);

                        // Never report dependency by the same handshake at this stage
                        controlling_handshakes.erase(origin_handshake);

                        if ((controlling_handshakes.size() > 1) || (curr != origin)) {
                            // Record adjacent handshakes
                            // We ignore all connections between the same handshake
                            for (const Handshake& src_handshake: controlling_handshakes) {
                                source_handshakes.insert({src_handshake, curr});
                            }
                        }

                        if (curr != origin) {
                            continue;
                        }
                    }

                    // Find dependent inputs.
                    std::set<Source> srcs;
                    for (auto [in_port, in_port_sig]: sink_cell->connections()) {
                        if (sink_cell->input(in_port)) {
                            // If the cell is a DFF, only propagate through the control signals and the corresponding data pin
                            if (RTLIL::builtin_ff_cell_types().count(sink_cell->type) && in_port == "\\D") {
                                Source sub_src = circuit_graph.source_map.at(sink_cell).at("\\D").at(sink_port_idx);
                                srcs.insert(sub_src);
                            } else {
                                for (int i = 0; i < GetSize(in_port_sig); ++i) {
                                    Source i_src = circuit_graph.source_map.at(sink_cell).at(in_port).at(i);
                                    srcs.insert(i_src);
                                }
                            }
                        }
                    }
                    for (Source src: srcs) {
                        // No need to propagate to external pins
                        if (std::holds_alternative<RTLIL::SigBit>(src)) {
                            continue;
                        }

                        auto [src_cell, src_port, src_port_idx] = std::get<CellPin>(src);
                        // Loop avoiding
                        if (reached_cell_chs.count({src_cell, src_port_idx})) {
                            continue;
                        }

                        // Do not propagate to data cells
                        if (blocking_data_components.count(src_cell)) {
                            continue;
                        }

                        // if (data_path.acyclic_node_map.count(src_cell)) {
                        //     continue;
                        // }

                        // Propagate
                        work_list.push_back({src_cell, src_port, src_port_idx});
                    }
                }

                // Any handshake bit is always dependent on the opposite handshake bit
                source_handshakes.insert({origin_handshake, origin_handshake.get_opposite_ctrl(origin)});

                // All block-less reachable are resolved
                return source_handshakes;
            };

            for (const auto& [ctrl_pin, handshake_set]: ctrl_pins) {
                for (const Handshake& handshake: handshake_set) {
                    ctrl_pin_connectivity[{handshake, ctrl_pin}] = get_ctrl_connectivity(ctrl_pin, handshake);
                }
            }

            // Identify intra-module loop
            // It is a 1-hop check
            std::map<std::pair<Handshake, CellPin>, std::set<std::pair<Handshake, CellPin>>> intra_ifaces;
            log("Control handshake connectivity:\n");
            for (auto [sink, srcs]: ctrl_pin_connectivity) {
                auto [sink_handshake, sink_ctrl_pin] = sink;
                auto [sink_cell, sink_port, sink_port_idx] = sink_ctrl_pin;
                log("%s:%s:%d depends on:\n", log_id(sink_cell), log_id(sink_port), sink_port_idx);

                std::set<Handshake> encountered_handshakes;
                std::set<Handshake> duplicated_handshakes;
                for (auto [src_handshake, src_ctrl_pin]: srcs) {
                    auto [src_cell, src_port, src_port_idx] = src_ctrl_pin;
                    log("%s:%s:%d\n", log_id(src_cell), log_id(src_port), src_port_idx);
                    log("\n");

                    if (encountered_handshakes.count(src_handshake)) {
                        duplicated_handshakes.insert(src_handshake);
                    } else {
                        encountered_handshakes.insert(src_handshake);
                    }
                }
                log("\n\n");

                for (const Handshake& dup_hs: duplicated_handshakes) {
                    auto [handshake_if0, handshake_if1] = dup_hs.decompose();
                    auto [ctrl0, _dffs0] = handshake_if0;
                    auto [ctrl1, _dffs1] = handshake_if1;
                    for (CellPin ctrl: {ctrl0, ctrl1}) {
                        if (ctrl_pin_connectivity.at({dup_hs, ctrl}).count(sink)) {
                            if (intra_ifaces.count(sink) == 0) {
                                intra_ifaces.insert({sink, {}});
                            }
                            intra_ifaces[sink].insert({dup_hs, ctrl});
                        }
                    }
                }
            }

            log("\n");
            log("Intra interfaces:\n");
            for (const auto& [sink, intra_srcs]: intra_ifaces) {
                auto [sink_handshake, sink_ctrl_pin] = sink;
                auto [sink_cell, sink_port, sink_port_idx] = sink_ctrl_pin;
                log("%s:%s:%d depends on:\n", log_id(sink_cell), log_id(sink_port), sink_port_idx);
                for (auto [src_handshake, src_ctrl_pin]: intra_srcs) {
                    auto [src_cell, src_port, src_port_idx] = src_ctrl_pin;
                    log("%s:%s:%d\n", log_id(src_cell), log_id(src_port), src_port_idx);

                    ctrl_pin_connectivity[sink].erase({src_handshake, src_ctrl_pin});
                }
            }
        }

        // Coalesce the interfaces to generate a module boundary
        PseudoModuleCollection pseudo_modules(ctrl_pin_connectivity);
        pseudo_modules.write_log();

        // // FIXME: Remove debug
        // // DEBUG
        // {
        //     // Replace the module data interfaces with hand selected interfaces
        //     int hs_idx = 0;
        //     for (auto& [hs, default_data_if]: module_data_interfaces) {
        //         std::vector<std::set<std::shared_ptr<DataElement>>> hs_frontiers = frontier_collections.at(hs);
        //         int frontier_idx;
        //         if (hs_idx == 1) {
        //             frontier_idx = 2;
        //         } else {
        //             frontier_idx = 5;
        //         }

        //         std::set<std::shared_ptr<DataElement>> frontier = hs_frontiers.at(frontier_idx);
        //         std::set<Wire> selected_data_if = data_path.get_intersection_wires(frontier);

        //         default_data_if.clear();
        //         default_data_if = selected_data_if;

        //         ++hs_idx;
        //     }
        // }
        // // DEBUG END

        // Guess Valid/ready direction
        VrModuleCollection vr_modules(
            pseudo_modules, module_data_interfaces, circuit_graph
        );

        // Collect possible common sinks
        std::set<RTLIL::Cell*> potential_common_sinks;
        for (const VrModule& vr_module: vr_modules.inner) {
            for (auto [hs, _in_pin]: vr_module.ctrl_inputs) {
                auto [ctrl0_info, ctrl1_info] = hs.decompose();
                auto [ctrl0_cell, ctrl0_port, ctrl0_idx] = std::get<0>(ctrl0_info);
                auto [ctrl1_cell, ctrl1_port, ctrl1_idx] = std::get<0>(ctrl1_info);

                std::vector<Sink> ctrl0_pin_sinks = circuit_graph.sink_map.at(ctrl0_cell).at(ctrl0_port).at(ctrl0_idx);
                std::vector<Sink> ctrl1_pin_sinks = circuit_graph.sink_map.at(ctrl1_cell).at(ctrl1_port).at(ctrl1_idx);

                for (Sink ctrl0_sink: ctrl0_pin_sinks) {
                    if (std::holds_alternative<RTLIL::SigBit>(ctrl0_sink)) {
                        continue;
                    }

                    for (Sink ctrl1_sink: ctrl1_pin_sinks) {
                        if (std::holds_alternative<RTLIL::SigBit>(ctrl1_sink)) {
                            continue;
                        }

                        RTLIL::Cell* ctrl0_sink_cell = std::get<0>(std::get<CellPin>(ctrl0_sink));
                        RTLIL::Cell* ctrl1_sink_cell = std::get<0>(std::get<CellPin>(ctrl1_sink));

                        if (ctrl0_sink_cell == ctrl1_sink_cell) {
                            potential_common_sinks.insert(ctrl0_sink_cell);
                        }
                    }
                }
            }
        }

        // Convert into partition after consolidating the handshake interfaces
        std::map<ValidReadyProto, Handshake> handshake_map;
        std::vector<Partition> partitioned_modules;

        auto vr_module_to_partition = [&](
            const VrModule& vr_module,
            const VrModuleCollection& vr_collection
        ) -> Partition {
            std::set<ValidReadyProto> ingress_ifaces;
            std::set<ValidReadyProto> egress_ifaces;
            for (const auto& [ingress_handshake, ingress_wires]: vr_module.data_inputs) {
                // Search for valid
                CellPin valid;
                bool found_valid = false;
                for (const auto& [ctrl_ingress_handshake, ctrl_in]: vr_module.ctrl_inputs) {
                    if (ctrl_ingress_handshake == ingress_handshake) {
                        found_valid = true;
                        valid = ctrl_in;
                    }
                }

                CellPin ready;
                bool found_ready = false;
                for (const auto& [ctrl_ingress_handshake, ctrl_out]: vr_module.ctrl_outputs) {
                    if (ctrl_ingress_handshake == ingress_handshake) {
                        found_ready = true;
                        ready = ctrl_out;
                    }
                }

                if (!(found_valid && found_ready)) {
                    log_error("Supplied invalid valid/ready module to partition\n");
                }

                handshake_map.insert({{valid, ready, ingress_wires}, ingress_handshake});

                ingress_ifaces.insert({
                    valid, ready, ingress_wires
                });
            }

            for (const auto& [egress_handshake, egress_wires]: vr_module.data_outputs) {
                // Search for valid
                CellPin valid;
                bool found_valid = false;
                for (const auto& [ctrl_egress_handshake, ctrl_out]: vr_module.ctrl_outputs) {
                    if (ctrl_egress_handshake == egress_handshake) {
                        found_valid = true;
                        valid = ctrl_out;
                    }
                }

                CellPin ready;
                bool found_ready = false;
                for (const auto& [ctrl_egress_handshake, ctrl_in]: vr_module.ctrl_inputs) {
                    if (ctrl_egress_handshake == egress_handshake) {
                        found_ready = true;
                        ready = ctrl_in;
                    }
                }

                if (!(found_valid && found_ready)) {
                    log_error("Supplied invalid valid/ready module to partition\n");
                }

                handshake_map.insert({{valid, ready, egress_wires}, egress_handshake});

                egress_ifaces.insert({
                    valid, ready, egress_wires
                });
            }

            // Find all other handshakes that are neither ingress nor egress of the module
            std::set<Handshake> module_handshakes;
            for (const auto& [ingress_handshake, _in_pin]: vr_module.ctrl_inputs) {
                module_handshakes.insert(ingress_handshake);
            }

            std::set<ValidReadyProto> other_module_ifaces;
            // Look up the rest of the modules, find other handshakes
            for (const VrModule& other_module: vr_collection.inner) {
                // No need to explicitly make other_module not loop into vr_module
                // Handhskae comparison would have rejected all handshakes

                for (const auto& [ingress_hs, ingress_wires]: other_module.data_inputs) {
                    // Make sure that the current handshake is not part of the vr_module
                    if (module_handshakes.count(ingress_hs)) {
                        continue;
                    }

                    // Search for valid
                    CellPin valid;
                    bool found_valid = false;
                    for (const auto& [ctrl_ingress_handshake, ctrl_in]: other_module.ctrl_inputs) {
                        if (ctrl_ingress_handshake == ingress_hs) {
                            found_valid = true;
                            valid = ctrl_in;
                        }
                    }

                    CellPin ready;
                    bool found_ready = false;
                    for (const auto& [ctrl_ingress_handshake, ctrl_out]: other_module.ctrl_outputs) {
                        if (ctrl_ingress_handshake == ingress_hs) {
                            found_ready = true;
                            ready = ctrl_out;
                        }
                    }

                    if (!(found_valid && found_ready)) {
                        log_error("Supplied invalid valid/ready module to partition\n");
                    }

                    other_module_ifaces.insert({
                        valid, ready, ingress_wires
                    });
                }

                for (const auto& [egress_hs, egress_wires]: other_module.data_outputs) {
                    // Make sure that the current handshake is not part of the vr_module
                    if (module_handshakes.count(egress_hs)) {
                        continue;
                    }

                    // Search for valid
                    CellPin valid;
                    bool found_valid = false;
                    for (const auto& [ctrl_egress_handshake, ctrl_out]: other_module.ctrl_outputs) {
                        if (ctrl_egress_handshake == egress_hs) {
                            found_valid = true;
                            valid = ctrl_out;
                        }
                    }

                    CellPin ready;
                    bool found_ready = false;
                    for (const auto& [ctrl_egress_handshake, ctrl_in]: other_module.ctrl_inputs) {
                        if (ctrl_egress_handshake == egress_hs) {
                            found_ready = true;
                            ready = ctrl_in;
                        }
                    }

                    if (!(found_valid && found_ready)) {
                        log_error("Supplied invalid valid/ready module to partition\n");
                    }

                    other_module_ifaces.insert({
                        valid, ready, egress_wires
                    });
                }
            }

            return Partition(ingress_ifaces, egress_ifaces, potential_common_sinks, other_module_ifaces, &circuit_graph, &ff_init_vals);
        };
        {
            for (int i = 0; i < GetSize(vr_modules.inner); ++i) {
                partitioned_modules.push_back(vr_module_to_partition(vr_modules.inner[i], vr_modules));
            }

            for (int i = 0; i < GetSize(partitioned_modules); ++i) {
                partitioned_modules.at(i).write_interface_log();
            }

            for (int i = 0; i < GetSize(partitioned_modules); ++i) {
                std::stringstream ss;
                ss << "init_synth_module" << i;
                partitioned_modules[i].to_shakeflow(ss.str());
                {
                    std::stringstream ss_end;
                    ss_end << "select -set " << ss.str() << " ";

                    for (RTLIL::Cell* data_comp: partitioned_modules[i].cells) {
                        ss_end << "c:" << data_comp->name.c_str() << " ";
                    }

                    Pass::call(design, ss_end.str().c_str());
                }
            }
        }

        log("Recorded proto to handshake mapping entries: %ld\n", handshake_map.size());

        log("Full module has %ld cells\n", top->cells().size());

        std::vector<std::vector<int>> interference;
        for (int i = 0; i < GetSize(partitioned_modules); ++i) {
            std::vector<int> row;
            for (int j = 0; j < GetSize(partitioned_modules); ++j) {
                row.push_back(std::numeric_limits<int>::max());
            }
            interference.push_back(row);
        }

        // Check module size
        auto compute_interference = [&] (
            int log_idx
        ) {
            for (int i = 0; i < GetSize(partitioned_modules); ++i) {
                if (i == log_idx) {
                    log("Module %d has %ld VR ingress and %ld VR egress\n", i, partitioned_modules[i].vr_srcs.size(), partitioned_modules[i].vr_sinks.size());
                    log("Module %d has %ld cells\n", i, partitioned_modules[i].cells.size());
                }
                for (int j = 0; j < GetSize(partitioned_modules); ++j) {
                    if (i == j) {
                        continue;
                    }
                    std::set<RTLIL::Cell*> union_cells;
                    union_cells.insert(partitioned_modules[i].cells.begin(), partitioned_modules[i].cells.end());
                    union_cells.insert(partitioned_modules[j].cells.begin(), partitioned_modules[j].cells.end());

                    // TODO: Reject acceptable overlaps
                    std::set<RTLIL::Cell*> common_acceptable_overlaps;
                    {
                        common_acceptable_overlaps.insert(
                            partitioned_modules[i].acceptable_overlaps.begin(),
                            partitioned_modules[i].acceptable_overlaps.end());
                        
                        common_acceptable_overlaps.insert(
                            partitioned_modules[j].acceptable_overlaps.begin(),
                            partitioned_modules[j].acceptable_overlaps.end());
                    }

                    std::set<RTLIL::Cell*> common_cells;
                    for (RTLIL::Cell* i_cell: partitioned_modules[i].cells) {
                        if (partitioned_modules[j].cells.count(i_cell) && (common_acceptable_overlaps.count(i_cell) == 0)) {
                            common_cells.insert(i_cell);
                        }
                    }

                    size_t intersect_size = common_cells.size();
                    interference[i][j] = intersect_size;
                    if (i == log_idx) {
                        log("Cross interference with Module %d: %ld\n", j, intersect_size);
                        // for (RTLIL::Cell* union_cell: union_cells) {
                        //     if (partitioned_modules[i].cells.count(union_cell) && partitioned_modules[j].cells.count(union_cell)) {
                        //         log("%s\n", log_id(union_cell));
                        //     }
                        // }
                        log('\n');
                    }
                }
            }
        };

        for (int i = 0; i < GetSize(partitioned_modules); ++i) {
            compute_interference(i);
        }

        // Iterative all modules, find the module with the lowest leaky degree
        std::map<int, std::set<int>> leaky_iface_list;
        // Populate leaky degree
        for (int i = 0; i < GetSize(partitioned_modules); ++i) {
            // Find all adjacent modules
            for (int j = 0; j < GetSize(partitioned_modules); ++j) {
                if (i == j) {
                    continue;
                }

                std::set<ValidReadyProto> adjacent_handshakes = partitioned_modules[i].adjacent_interfaces(partitioned_modules[j]);
                if (adjacent_handshakes.empty()) {
                    continue;
                }

                // Reject multi-connect modules
                if (adjacent_handshakes.size() > 1) {
                    log_error("Found multi-connect module\n");
                    continue;
                }

                if (interference[i][j] > 0) {
                    if (leaky_iface_list.count(i) == 0) {
                        leaky_iface_list[i] = {};
                    }
                    leaky_iface_list[i].insert(j);
                }
            }
        }

        // Find the module that has the minimum leaky degree, but non-zero
        while (!leaky_iface_list.empty()) {
            int simplest_module_idx = -1;
            int lowest_leaky_degree = std::numeric_limits<int>::max();
            std::set<int> adjacent_leaky_modules;

            for (auto [module_idx, adjacent_idxes]: leaky_iface_list) {
                int leaky_adjacencies = GetSize(adjacent_idxes);
                log("Module %d has %d leaky interfaces\n", module_idx, leaky_adjacencies);
                if (leaky_adjacencies < lowest_leaky_degree) {
                    simplest_module_idx = module_idx;
                    lowest_leaky_degree = leaky_adjacencies;
                    adjacent_leaky_modules = adjacent_idxes;
                }
            }

            log("Resolve module %d\n", simplest_module_idx);

            // Collect handshakes
            std::map<int, std::pair<ValidReadyProto, Handshake>> handshake_entries;
            for (int adjacent_module_idx: adjacent_leaky_modules) {
                ValidReadyProto handshake_proto = *(partitioned_modules[simplest_module_idx].adjacent_interfaces(
                    partitioned_modules[adjacent_module_idx]).begin());
                Handshake hs = handshake_map.at(handshake_proto);
                // // Retrieve handshake
                // Handshake hs = handshake_map.at(handshake_proto);

                // std::set<Wire> wire_interface = std::get<2>(handshake_proto);
                // // Convert junction back to data elms
                // std::set<std::shared_ptr<DataElement>> old_iface_elms;
                // for (auto [_from_cell, to_cell]: data_interface) {
                //     old_iface_elms.insert(
                //         data_path.acyclic_node_map.at(std::get<0>(to_cell)));
                // }

                handshake_entries.insert({adjacent_module_idx, {handshake_proto, hs}});
            }

            // Retrieve the handshakes protocol entry that represent the adjacent edges
            // Use it to populate the possible data interfaces location list
            std::vector<std::vector<std::tuple<int, ValidReadyProto, Handshake, std::set<std::shared_ptr<DataElement>>>>> adjust_vectors;
            for (auto [adj_module_idx, vr_info]: handshake_entries) {
                auto [adj_proto, adj_hs] = vr_info;
                adj_hs.write_log();
                if (adjust_vectors.empty()) {
                    // std::map<Handshake, std::set<std::shared_ptr<DataElement>>> new_entry;
                    // new_entry[adj_hs] = frontier_collections.at(adj_hs);
                    // adjust_vectors.push_back(new_entry);

                    for (std::set<std::shared_ptr<DataElement>> frontier: frontier_collections.at(adj_hs)) {
                        std::vector<std::tuple<int, ValidReadyProto, Handshake, std::set<std::shared_ptr<DataElement>>>> new_vector;
                        new_vector.push_back({
                            adj_module_idx, adj_proto, adj_hs, frontier
                        });
                        adjust_vectors.push_back(new_vector);
                    }
                } else {
                    // Generate every entry in the adjust vector by a different frontier of this new handshake
                    std::vector<std::vector<std::tuple<int, ValidReadyProto, Handshake, std::set<std::shared_ptr<DataElement>>>>> augmented_vectors;
                    for (std::set<std::shared_ptr<DataElement>> frontier: frontier_collections.at(adj_hs)) {
                        for (auto vector_entry: adjust_vectors) {
                            // std::map<Handshake, std::set<std::shared_ptr<DataElement>>> new_entry = vector_entry;
                            // new_entry.insert({adj_hs, frontier});
                            // augmented_vectors.push_back(new_entry);

                            vector_entry.push_back({
                                adj_module_idx, adj_proto, adj_hs, frontier
                            });
                            augmented_vectors.push_back(vector_entry);
                        }
                    }
                    // Replace the original vector
                    adjust_vectors = augmented_vectors;
                }
            }
            log("Adjust vector size: %d\n", GetSize(adjust_vectors));

            // Brute force apply all possible vectors
            bool found_suitable_vector = false;
            for (const auto& data_vector: adjust_vectors) {
                // Update all interface according to the data vector first
                for (auto [adj_module_idx, old_proto, hs, new_frontier]: data_vector) {
                    log("New frontier size to module %d: %d\n", adj_module_idx, GetSize(new_frontier));
                    std::set<Wire> new_data_cut_if = data_path.get_intersection_wires(new_frontier);

                    vr_modules.inner[simplest_module_idx].update_data_interface(hs, new_data_cut_if);
                    vr_modules.inner[adj_module_idx].update_data_interface(hs, new_data_cut_if);
                }
                
                // Then, regenerate the partition
                for (auto [adj_module_idx, old_proto, hs, new_frontier]: data_vector) {
                    partitioned_modules[simplest_module_idx] = vr_module_to_partition(vr_modules.inner[simplest_module_idx], vr_modules);
                    partitioned_modules[adj_module_idx] = vr_module_to_partition(vr_modules.inner[adj_module_idx], vr_modules);
                }

                // Recalculate interference after applying every vector
                compute_interference(simplest_module_idx);
                bool interfere_other_module = false;
                for (int adj_idx: adjacent_leaky_modules) {
                    // Defensive programming
                    // Technically it is not needed since module cannot self connect
                    if (simplest_module_idx == adj_idx) {
                        continue;
                    }
                    if (interference[simplest_module_idx][adj_idx] != 0) {
                        interfere_other_module = true;
                        break;
                    }
                }

                if (!interfere_other_module) {
                    // Update
                    for (auto [adj_module_idx, old_proto, hs, new_frontier]: data_vector) {
                        std::set<Wire> new_data_cut_if = data_path.get_intersection_wires(new_frontier);

                        ValidReadyProto new_proto = {
                            std::get<0>(old_proto),
                            std::get<1>(old_proto),
                            new_data_cut_if
                        };

                        handshake_map.erase(old_proto);
                        handshake_map.insert({new_proto, hs});
                    }

                    // Remove resolved modules
                    leaky_iface_list.erase(simplest_module_idx);

                    // The resolved module should not be leaky to any other modules as well
                    std::set<int> unleaked_module_sources;
                    for (auto& [leaky_src_mod, leaky_sink_mod]: leaky_iface_list) {
                        if (leaky_sink_mod.count(simplest_module_idx) != 0) {
                            leaky_sink_mod.erase(simplest_module_idx);
                        }
                        if (leaky_sink_mod.empty()) {
                            unleaked_module_sources.insert(leaky_src_mod);
                        }
                    }

                    // Cleanup
                    for (int unleaked: unleaked_module_sources) {
                        leaky_iface_list.erase(unleaked);
                    }

                    found_suitable_vector = true;
                    break;
                    log("\n\n\n");
                }
            }

            if (!found_suitable_vector) {
                log("No suitable datapath interface sampled\n");
                return;
            }
        }

        log("\n\n");
        log("Data interface fixed:\n");
        for (int i = 0; i < GetSize(partitioned_modules); ++i) {
            compute_interference(i);
        }

        // for (int i = 0; i < GetSize(partitioned_modules); ++i) {
        //     for (int j = 0; j < GetSize(partitioned_modules); ++j) {
        //         if (i == j) {
        //             continue;
        //         }

        //         std::set<ValidReadyProto> adjacent_handshakes = partitioned_modules[i].adjacent_interfaces(partitioned_modules[j]);
        //         if (adjacent_handshakes.empty()) {
        //             continue;
        //         }

        //         // Reject multi-connect modules
        //         if (adjacent_handshakes.size() > 1) {
        //             log("Found multi-connect module\n");
        //             continue;
        //         }
        //         ValidReadyProto handshake_proto = *adjacent_handshakes.begin();

        //         log("Adjust module %d and %d\n", i, j);

        //         while (interference[i][j] > 0) {
        //             // Retrieve existing data interface
        //             std::set<Wire> data_interface = std::get<2>(handshake_proto);

        //             // Retrieve subgraph
        //             log("Retrieve handshake\n");
        //             Handshake hs = handshake_map.at(handshake_proto);
        //             log("Retrieve successful\n");
        //             std::set<std::shared_ptr<DataElement>> subgraph = initial_subgraphs.at(hs);
        //             log("Retrieve subgraph successful\n");

        //             std::set<std::shared_ptr<DataElement>> old_iface_elms;
        //             for (auto [_from_cell, to_cell]: data_interface) {
        //                 old_iface_elms.insert(
        //                     data_path.acyclic_node_map.at(std::get<0>(to_cell)));
        //             }

        //             // Advance the old frontier, and promote its inputs
        //             std::set<std::shared_ptr<DataElement>> new_elms = data_path.advance_frontier(subgraph, old_iface_elms);
        //             log("New input elements: %d\n", GetSize(new_elms));
        //             std::set<std::shared_ptr<DataElement>> new_frontier = data_path.promote_inputs(subgraph, new_elms);
        //             log("New frontier width: %d\n", GetSize(new_frontier));

        //             std::set<Wire> new_data_cut_if = data_path.get_intersection_wires(new_frontier);
        //             ValidReadyProto new_proto = {
        //                 std::get<0>(handshake_proto),
        //                 std::get<1>(handshake_proto),
        //                 new_data_cut_if
        //             };

        //             log("New interface:\n");
        //             for (const auto& [from_cell, to_cell]: new_data_cut_if) {
        //                 log("%s:%s:%d\n", log_id(std::get<0>(from_cell)), log_id(std::get<1>(from_cell)), std::get<2>(from_cell));
        //             }

        //             if (data_interface == new_data_cut_if) {
        //                 log_error("Data interface failed to advance\n");
        //             }

        //             // Update info
        //             handshake_map.erase(handshake_proto);
        //             handshake_map.insert({new_proto, hs});
        //             handshake_proto = new_proto;

        //             vr_modules.inner[i].update_data_interface(hs, new_data_cut_if);
        //             vr_modules.inner[j].update_data_interface(hs, new_data_cut_if);

        //             partitioned_modules[i] = vr_module_to_partition(vr_modules.inner[i]);
        //             partitioned_modules[j] = vr_module_to_partition(vr_modules.inner[j]);

        //             compute_interference(-1);
        //         }
        //     }
        // }

        for (int i = 0; i < GetSize(partitioned_modules); ++i) {
            std::stringstream ss;
            ss << "synth_module" << i;
            partitioned_modules[i].to_shakeflow(ss.str());
            {
                std::stringstream ss_end;
                ss_end << "select -set " << ss.str() << " ";

                for (RTLIL::Cell* data_comp: partitioned_modules[i].cells) {
                    ss_end << "c:" << data_comp->name.c_str() << " ";
                }

                Pass::call(design, ss_end.str().c_str());
            }
        }
    }
} ValidReadyPass;

PRIVATE_NAMESPACE_END
