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

                                LOG("Cell %s Port %s reaches Cell %s CE\n",
                                    std::get<0>(ctrl_bit)->name.c_str(),
                                    std::get<1>(ctrl_bit).c_str(),
                                    log_id(data_reg));
                                LOG("Other ctrl pin: Cell %s; Port %s\n",
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
                                                edges_to_be_cut_opt[src_first_divergence].insert(first_div_sink_pin);
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
                                LOG("Cell %s Port %s reaches intermediate DFFE %s CE\n",
                                    std::get<0>(ctrl_bit)->name.c_str(),
                                    std::get<1>(ctrl_bit).c_str(),
                                    log_id(inter_dff));
                                LOG("Cell %s Port %s also reaches intermediate DFFE %s CE\n",
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

            for (const auto& [eligible_pair, data_components]: eligible_ctrl_bit_pairs) {
                auto [handshake_src, handshake_sink] = eligible_pair.decompose();
                auto [eligible_src, dffe_srcs] = handshake_src;
                auto [eligible_sink, dffe_sinks] = handshake_sink;

                ss << "c:" << std::get<0>(eligible_src)->name.c_str() << " ";
                ss << "c:" << std::get<0>(eligible_sink)->name.c_str() << " ";
            }

            Pass::call(design, ss.str().c_str());
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

        // Create a provisional subgraph for every handshake entry
        // using the least appeared elements
        std::map<Handshake, std::set<std::shared_ptr<DataElement>>> provisional_subgraphs;

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

            // Replace inputs with successors, until all inputs are not successors or predecessors
            auto promote_inputs = [&](
                const std::set<std::shared_ptr<DataElement>>& subgraph_nodes,
                const std::set<std::shared_ptr<DataElement>>& data_inputs
            ) {
                // Find if some input is a successor of some other inputs
                std::set<std::shared_ptr<DataElement>> frontier = data_inputs;
                std::set<std::shared_ptr<DataElement>> new_frontier;
                std::set<std::shared_ptr<DataElement>> removed_frontier;

                do {
                    // Update frontier
                    for (std::shared_ptr<DataElement> cell: removed_frontier) {
                        frontier.erase(cell);
                    }
                    frontier.insert(new_frontier.begin(), new_frontier.end());
                    new_frontier.clear();

                    for (std::shared_ptr<DataElement> frontier_src: frontier) {
                        for (std::shared_ptr<DataElement> frontier_sink: frontier) {
                            if (frontier_src == frontier_sink) {
                                continue;
                            }
                            if (!data_path.get_inter_nodes(frontier_src, frontier_sink).empty()) {
                                // Move the frontier to its descendents
                                removed_frontier.insert(frontier_src);
                                for (std::shared_ptr<DataElement> next: frontier_src->sink) {
                                    if (subgraph_nodes.count(next)) {
                                        new_frontier.insert(next);
                                    }
                                }
                            }
                        }
                    }

                    // Do not propagate to removed frontier
                    for (std::shared_ptr<DataElement> removed_node: removed_frontier) {
                        new_frontier.erase(removed_node);
                    }
                } while (!new_frontier.empty());

                // No new data elements to be added
                // Remove anything that is still in the removed frontier
                for (std::shared_ptr<DataElement> cell: removed_frontier) {
                    frontier.erase(cell);
                }

                return frontier;
            };

            std::set<std::shared_ptr<DataElement>> data_subgraph_input = promote_inputs(
                smallest_subgraph, data_subgraph_input_raw);

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
        std::map<Handshake, std::set<Handshake>> connectivity_graph;
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

            // // Check if any control pin considers itself connected to both sides of some other handshake
            // // If so resolve it by checking for path to the direct sink
            // for (auto& [sink, srcs]: ctrl_pin_connectivity) {
            //     std::set<Handshake> src_handshakes;
            //     std::set<Handshake> duplicating_handshakes;
            //     for (auto& [src_handshake, src_ctrl_pin]: srcs) {
            //         src_handshakes.insert()
            //     }
            // }

            log("Control handshake connectivity:\n");
            for (auto [sink, srcs]: ctrl_pin_connectivity) {
                auto [sink_handshake, sink_ctrl_pin] = sink;
                auto [sink_cell, sink_port, sink_port_idx] = sink_ctrl_pin;
                log("%s:%s:%d depends on:\n", log_id(sink_cell), log_id(sink_port), sink_port_idx);
                for (auto [src_handshake, src_ctrl_pin]: srcs) {
                    // src_handshake.write_ctrl_pins();
                    auto [src_cell, src_port, src_port_idx] = src_ctrl_pin;
                    log("%s:%s:%d\n", log_id(src_cell), log_id(src_port), src_port_idx);
                    log("\n");
                }
                log("\n\n");
            }
        }

        // Coalesce the interfaces to generate a module boundary
        PseudoModuleCollection pseudo_modules(ctrl_pin_connectivity);
        pseudo_modules.write_log();

        VrModuleCollection vr_modules(
            pseudo_modules, module_data_interfaces, circuit_graph
        );

        // Convert into partition after consolidating the handshake interfaces
        std::vector<Partition> partitioned_modules;
        {
            for (const VrModule& vr_module: vr_modules.inner) {
                // Act as an adapter to the following constructor
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

                    ingress_ifaces.insert({
                        valid, ready, ingress_wires
                    });
                }

                for (const auto& [egress_handshake, egress_wires]: vr_module.data_outputs) {
                    // Search for valid
                    CellPin valid;
                    bool found_valid = false;
                    for (const auto& [ctrl_egress_handshake, ctrl_out]: vr_module.ctrl_inputs) {
                        if (ctrl_egress_handshake == egress_handshake) {
                            found_valid = true;
                            valid = ctrl_out;
                        }
                    }

                    CellPin ready;
                    bool found_ready = false;
                    for (const auto& [ctrl_egress_handshake, ctrl_in]: vr_module.ctrl_outputs) {
                        if (ctrl_egress_handshake == egress_handshake) {
                            found_ready = true;
                            ready = ctrl_in;
                        }
                    }

                    if (!(found_valid && found_ready)) {
                        log_error("Supplied invalid valid/ready module to partition\n");
                    }

                    egress_ifaces.insert({
                        valid, ready, egress_wires
                    });
                }

                partitioned_modules.push_back(
                    Partition(ingress_ifaces, egress_ifaces, circuit_graph, &ff_init_vals));
            }
        }

        // Check module size
        for (const Partition& partition: partitioned_modules) {
            log("Module has %ld cells\n", partition.cells.size());
        }

        for (int i = 0; i < GetSize(partitioned_modules); ++i) {
            std::stringstream ss;
            ss << "synth_module" << i;
            partitioned_modules[i].to_shakeflow(ss.str());
        }
    }
} ValidReadyPass;

PRIVATE_NAMESPACE_END
