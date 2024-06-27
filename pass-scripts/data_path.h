#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/ff.h"
#include "graph.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


struct DataElement {
    // Descendents
    std::set<std::shared_ptr<DataElement>> sink;
    // Predecessors
    std::set<std::shared_ptr<DataElement>> source;

    // Encapsulation
    std::set<RTLIL::Cell*> subgraph_nodes;
};


struct DataPath {
    dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> data_sink_map;
    dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> data_source_map;

    // // Acyclic graph
    // std::set<std::shared_ptr<DataElement>> acyclic_data_graph;

    // RTLIL data cell to acyclic graph node mapping
    std::map<RTLIL::Cell*, std::shared_ptr<DataElement>> acyclic_node_map;

    // Standard circuit graph
    CircuitGraph circuit_graph;

    DataPath(const SigMap& sigmap, RTLIL::Module* module): circuit_graph{sigmap, module} {
        FfInitVals ff_init_vals(&sigmap, module);

        std::set<RTLIL::Cell*> buffers;

        std::set<RTLIL::Cell*> registers;
        for (RTLIL::Cell* ff: module->cells()) {
            if (RTLIL::builtin_ff_cell_types().count(ff->type)) {
                registers.insert(ff);
            }
        }

        // Find registers that does not have combinatorial path back to itself
        for (RTLIL::Cell* ff: registers) {
            if (RTLIL::builtin_ff_cell_types().count(ff->type)) {
                FfData ff_data(&ff_init_vals, ff);
                bool has_feedback = false;

                for (int i = 0; i < GetSize(ff_data.sig_q); ++i) {
                    for (int j = 0; j < GetSize(ff_data.sig_d); ++j) {
                        if (!this->circuit_graph.get_intermediate_comb_cells(
                            {ff, "\\Q", i}, {ff, "\\D", j}
                        ).empty()) {
                            has_feedback = true;
                        }
                    }
                }

                if (!has_feedback) {
                    buffers.insert(ff);
                }
            }
        }

        std::set<RTLIL::Cell*> loop_back_registers = registers;
        for (RTLIL::Cell* buf_reg: buffers) {
            loop_back_registers.erase(buf_reg);
        }

        std::set<RTLIL::Cell*> memories;

        // Find registers that are memories
        for (RTLIL::Cell* self_loop_reg: loop_back_registers) {
            FfData ff_data(&ff_init_vals, self_loop_reg);

            // Never classify self-feeding registers with bit width 1 as data registers
            if (ff_data.width <= 1) {
                continue;
            }

            bool has_cross_path = false;
            for (int i = 0; i < GetSize(ff_data.sig_q); ++i) {
                for (int j = 0; j < GetSize(ff_data.sig_d); ++j) {
                    if (i == j) {
                        continue;
                    }

                    if (!this->circuit_graph.get_intermediate_comb_cells(
                        {self_loop_reg, "\\Q", i}, {self_loop_reg, "\\D", j}
                    ).empty()) {
                        has_cross_path = true;
                    }
                }
            }

            if (!has_cross_path) {
                memories.insert(self_loop_reg);
            }
        }

        // Registers that are part of the data path
        std::set<RTLIL::Cell*> buf_or_mems;

        // Classify whatever between buffers or memories are data components
        // Starts with buffers or memories
        std::set<RTLIL::Cell*> data_components;
        {
            buf_or_mems.insert(buffers.begin(), buffers.end());
            buf_or_mems.insert(memories.begin(), memories.end());
            data_components = buf_or_mems;
        }
        for (RTLIL::Cell* elm_from: buf_or_mems) {
            FfData elm_from_data(&ff_init_vals, elm_from);
            int from_width = elm_from_data.width;
            for (RTLIL::Cell* elm_to: buf_or_mems) {
                FfData elm_to_data(&ff_init_vals, elm_to);
                int to_width = elm_to_data.width;

                std::set<RTLIL::Cell*> reachables;

                for (int i = 0; i < from_width; ++i) {
                    for (int j = 0; j < to_width; ++j) {
                        std::set<RTLIL::Cell*> reachables_inters = this->circuit_graph.get_intermediate_comb_cells(
                            {elm_from, "\\Q", i},
                            {elm_to, "\\D", j}
                        );
                        reachables.insert(reachables_inters.begin(), reachables_inters.end());
                    }
                }
                data_components.insert(reachables.begin(), reachables.end());
            }
        }

        // Generate an entry on the data sink/source map for all data compoenents
        // Regadless it has sinks/sources or not, the data component should still be represented by a node
        for (RTLIL::Cell* data_cell: data_components) {
            this->data_sink_map[data_cell] = {};
            this->data_source_map[data_cell] = {};
        }

        // std::stringstream ss;
        // ss << "select -set data_comp ";

        // log("Data Components:\n");
        // for (RTLIL::Cell* data_comp: data_components) {
        //     // ss << "c:" << data_comp->name.c_str() << " ";
        //     log("%s\n", log_id(data_comp));
        // }
        // log("\n");

        // Pass::call(design, ss.str().c_str());
        // Pass::call(design, "show -color red @data_comp");

        // Establish data path cells hierarchy
        // We do not care about the port connectivity in detail
        // Edges of different ports should be simplified into a common node
        // Use the already established circuit graph
        {
            for (const auto& [src_cell, port_map]: this->circuit_graph.sink_map) {
                if (data_components.count(src_cell) == 0) {
                    continue;
                }
                for (const auto& [src_port, port_idx_vec]: port_map) {
                    for (size_t src_port_idx = 0; src_port_idx < port_idx_vec.size(); ++src_port_idx) {
                        const std::vector<Sink>& port_idx_sinks = port_idx_vec[src_port_idx];
                        for (const Sink& sink: port_idx_sinks) {
                            if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                                continue;
                            }

                            auto [sink_cell, sink_port, sink_idx] = std::get<CellPin>(sink);
                            if (data_components.count(sink_cell)) {
                                // Copy to data path graph
                                // if (this->data_sink_map.count(src_cell) == 0) {
                                //     this->data_sink_map[src_cell] = {};
                                // }
                                this->data_sink_map[src_cell].insert(sink_cell);

                                // if (this->data_source_map.count(sink_cell) == 0) {
                                //     this->data_source_map[sink_cell] = {};
                                // }
                                this->data_source_map[sink_cell].insert(src_cell);
                            }
                        }
                    }
                }
            }
        }

        // // Return the DFFs that forms a loop that origins from the source
        // auto get_dff_loop = [&] (
        //     RTLIL::Cell* src,
        //     const dict<RTLIL::Cell*, std::set<RTLIL::Cell*>>& data_comp_sinks
        // ) {
        //     std::set<RTLIL::Cell*> ret;
        //     std::vector<std::vector<RTLIL::Cell*>> path_work_list;
        //     path_work_list.push_back({src});

        //     while (!path_work_list.empty()) {
        //         std::vector<RTLIL::Cell*> path = path_work_list.back();
        //         path_work_list.pop_back();

        //         // Attempt to continue the path
        //         if (data_comp_sinks.count(path.back()) == 0) {
        //             continue;
        //         }
        //         for (RTLIL::Cell* next_cell_sink: data_comp_sinks.at(path.back())) {
        //             // Report and store the path (without order)
        //             // if it is about to propagate back to the source
        //             if (next_cell_sink == src) {
        //                 ret.insert(path.begin(), path.end());
        //                 continue;
        //             }

        //             // Loop detection
        //             if (std::count(path.begin(), path.end(), next_cell_sink)) {
        //                 continue;
        //             }

        //             // Propagate and extend the path
        //             path.push_back(next_cell_sink);
        //             path_work_list.push_back(path);
        //         }
        //     }

        //     return ret;
        // };

        // // De-cyclicize the graph
        // //
        // // Cycles must be originated from DFFs
        // std::set<std::set<RTLIL::Cell*>> cyclic_components;
        // for (RTLIL::Cell* data_dff: buf_or_mems) {
        //     std::set<RTLIL::Cell*> self_loop_dffs = get_dff_loop(data_dff, this->data_sink_map);
        //     if (self_loop_dffs.empty()) {
        //         continue;
        //     }

        //     cyclic_components.insert(self_loop_dffs);

        //     log("\n");
        //     log("Source DFF: %s\n", log_id(data_dff));
        //     for (RTLIL::Cell* loop_elm: self_loop_dffs) {
        //         log("Loop inter: %s\n", log_id(loop_elm));
        //     }
        //     log("\n");
        // }

        // Acquire limited cycles (i.e. memories)
        // We assume only 1-DFF cycles need to be decyclized
        auto get_single_dff_loop = [&] (
            RTLIL::Cell* src,
            const dict<RTLIL::Cell*, std::set<RTLIL::Cell*>>& data_comp_sinks
        ) {
            std::set<RTLIL::Cell*> ret;
            std::vector<std::vector<RTLIL::Cell*>> path_work_list;
            path_work_list.push_back({src});

            while (!path_work_list.empty()) {
                std::vector<RTLIL::Cell*> path = path_work_list.back();
                path_work_list.pop_back();

                // Attempt to continue the path
                if (data_comp_sinks.count(path.back()) == 0) {
                    continue;
                }
                for (RTLIL::Cell* next_cell_sink: data_comp_sinks.at(path.back())) {
                    // Report and store the path (without order)
                    // if it is about to propagate back to the source
                    if (next_cell_sink == src) {
                        ret.insert(path.begin(), path.end());
                        continue;
                    }

                    // Loop detection
                    if (std::count(path.begin(), path.end(), next_cell_sink)) {
                        continue;
                    }

                    // If traversed to another DFF that is not the source, reject path
                    if (RTLIL::builtin_ff_cell_types().count(next_cell_sink->type)) {
                        continue;
                    }

                    // Propagate and extend the path
                    path.push_back(next_cell_sink);
                    path_work_list.push_back(path);
                }
            }

            return ret;
        };

        log("Begin finding single-cyclic DFF\n");
        std::set<std::set<RTLIL::Cell*>> cyclic_components;
        for (RTLIL::Cell* data_dff: memories) {
            std::set<RTLIL::Cell*> self_loop_dffs = get_single_dff_loop(data_dff, this->data_sink_map);
            if (self_loop_dffs.empty()) {
                continue;
            }

            cyclic_components.insert(self_loop_dffs);

            log("\n");
            log("Source DFF: %s\n", log_id(data_dff));
            for (RTLIL::Cell* loop_elm: self_loop_dffs) {
                log("Loop inter: %s\n", log_id(loop_elm));
            }
            log("\n");
        }

        std::set<RTLIL::Cell*> data_cells_in_cycles;
        for (const std::set<RTLIL::Cell*>& cyclic_comp: cyclic_components) {
            data_cells_in_cycles.insert(cyclic_comp.begin(), cyclic_comp.end());
        }

        // Replace cyclic components with data elements instances

        // Transfer every cells not in a cyclic path
        for (auto [src_cell, data_sinks]: this->data_sink_map) {
            if (data_cells_in_cycles.count(src_cell) == 0) {
                std::shared_ptr<DataElement> data_cell_ptr = std::make_shared<DataElement>();
                data_cell_ptr->subgraph_nodes = {src_cell};
                // this->acyclic_data_graph.insert(data_cell_ptr);

                // Record to map
                this->acyclic_node_map[src_cell] = data_cell_ptr;
            }
        }

        // Transfer cyclic components
        for (const std::set<RTLIL::Cell*>& cyclic_comp: cyclic_components) {
            std::shared_ptr<DataElement> cyclic_data_ptr = std::make_shared<DataElement>();
            cyclic_data_ptr->subgraph_nodes = cyclic_comp;
            // this->acyclic_data_graph.insert(cyclic_data_ptr);

            for (RTLIL::Cell* cyclic_cell: cyclic_comp) {
                this->acyclic_node_map[cyclic_cell] = cyclic_data_ptr;
            }
        }

        // Transfer edges
        for (const auto& [src_cell, data_sinks]: this->data_sink_map) {
            for (RTLIL::Cell* sink_cell: data_sinks) {
                const std::shared_ptr<DataElement>& src_elm = this->acyclic_node_map.at(src_cell);
                const std::shared_ptr<DataElement>& sink_elm = this->acyclic_node_map.at(sink_cell);

                if (src_elm != sink_elm) {
                    src_elm->sink.insert(sink_elm);
                    sink_elm->source.insert(src_elm);
                }
            }
        }

        // In addition, we can consider grouping up MUXes that have the same control source
        std::map<RTLIL::Cell*, std::set<RTLIL::Cell*>> mux_cell_groups;

        for (RTLIL::Cell* cell: module->cells()) {
            if (cell->type == "$_MUX_") {
                // Consult the custom made graph to find the sources of select signal
                Source s_src = this->circuit_graph.source_map.at(cell).at("\\S").at(0);
                if (std::holds_alternative<RTLIL::SigBit>(s_src)) {
                    continue;
                }
                RTLIL::Cell* s_src_cell = std::get<0>(std::get<CellPin>(s_src));
                
                if (mux_cell_groups.count(s_src_cell) == 0) {
                    mux_cell_groups[s_src_cell] = {};
                }
                mux_cell_groups[s_src_cell].insert(cell);
            }
        }

        std::map<RTLIL::Cell*, std::set<std::shared_ptr<DataElement>>> mux_elm_groups;
        std::map<RTLIL::Cell*, std::set<RTLIL::Cell*>> mux_subgraph_cell_groups;
        for (auto [src_cell, muxes]: mux_cell_groups) {
            std::set<std::shared_ptr<DataElement>> data_elm_set;
            std::set<RTLIL::Cell*> subgraph_cell_set;
            bool found_elm_in_subgraph = false;
            for (RTLIL::Cell* mux: muxes) {
                if (this->acyclic_node_map.count(mux) == 0) {
                    continue;
                }

                std::shared_ptr<DataElement> acyclic_node = this->acyclic_node_map.at(mux);
                // Check that the node solely contains the MUX and nothing else
                if (acyclic_node->subgraph_nodes.size() == 1 && *(acyclic_node->subgraph_nodes.begin()) == mux) {
                    data_elm_set.insert(acyclic_node);
                    subgraph_cell_set.insert(mux);
                    found_elm_in_subgraph = true;
                }
            }

            // Make an entry for the mux group
            if (found_elm_in_subgraph) {
                mux_elm_groups.insert({src_cell, data_elm_set});
                mux_subgraph_cell_groups.insert({src_cell, subgraph_cell_set});
            }
        }

        // log("Found %d mux element groups\n", GetSize(mux_elm_groups));

        // {
        //     for (auto [src_cell, muxes]: mux_elm_groups) {
        //         log("MUX group source: %s\n", log_id(src_cell));
        //         log("Inner muxes:\n");

        //         for (std::shared_ptr<DataElement> mux_elm: muxes) {
        //             RTLIL::Cell* inner_mux = *(mux_elm->subgraph_nodes.begin());
        //             log("%s\n", log_id(inner_mux));
        //         }
        //     }
        // }

        // Simplify the data graph by grouping up all these MUX groups into 1 data element
        for (auto [src_cell, mux_elms]: mux_elm_groups) {
            std::shared_ptr<DataElement> lumped_mux = std::make_shared<DataElement>();

            // Populate enclosed RTLIL cells
            lumped_mux->subgraph_nodes = mux_subgraph_cell_groups.at(src_cell);
            
            // Collect external I/O
            for (std::shared_ptr<DataElement> mux_elm: mux_elms) {
                for (std::shared_ptr<DataElement> sink_elm: mux_elm->sink) {
                    // Ignore all internal connections
                    if (mux_elms.count(sink_elm) == 0) {
                        lumped_mux->sink.insert(sink_elm);

                        // Update the sink node
                        // - its source should not contain the old cell
                        // - its source should contain the lumped cell
                        sink_elm->source.erase(mux_elm);
                        sink_elm->source.insert(lumped_mux);
                    }
                }
            }

            for (std::shared_ptr<DataElement> mux_elm: mux_elms) {
                for (std::shared_ptr<DataElement> source_elm: mux_elm->source) {
                    // Ignore all internal connections
                    if (mux_elms.count(source_elm) == 0) {
                        lumped_mux->source.insert(source_elm);

                        // Update the source node
                        // - its sink should not contain the old cell
                        // - its sink should contain the lumped cell
                        source_elm->sink.erase(mux_elm);
                        source_elm->sink.insert(lumped_mux);
                    }
                }
            }

            // Replace the original mux elements with the new lumped element
            for (std::shared_ptr<DataElement> mux_elm: mux_elms) {
                RTLIL::Cell* mux_cell = *(mux_elm->subgraph_nodes.begin());
                this->acyclic_node_map[mux_cell] = lumped_mux;
            }
        }

        // // DEBUG
        // {
        //     RTLIL::Cell* mux_in_interest = module->cell("$auto$simplemap.cc:267:simplemap_mux$3345");
        //     if (this->acyclic_node_map.count(mux_in_interest)) {
        //         log("MUX in question found\n");

        //         std::shared_ptr<DataElement> acyclic_node = this->acyclic_node_map.at(mux_in_interest);
        //         log("Acyclic node has %d sinks\n", GetSize(acyclic_node->sink));
        //     } else {
        //         log("MUX in question not found\n");
        //     }
        //     // exit(1);
        // }
    }
/*
    // Find the smallest subgraph that enclose all cells
    std::set<std::shared_ptr<DataElement>> get_subgraph(
        std::set<RTLIL::Cell*> data_cells
    ) const {
        std::set<std::shared_ptr<DataElement>> ret;

        for (RTLIL::Cell* from_cell: data_cells) {
            for (RTLIL::Cell* to_cell: data_cells) {
                // log("Propagating through %s and %s\n", log_id(from_cell), log_id(to_cell));

                // FIXME: Ideally the data cells should be within the pre-register map
                if (this->acyclic_node_map.count(from_cell) == 0 || this->acyclic_node_map.count(to_cell) == 0) {
                    continue;
                }

                std::set<std::shared_ptr<DataElement>> intermediate_elms = this->get_inter_nodes(
                    this->acyclic_node_map.at(from_cell),
                    this->acyclic_node_map.at(to_cell));
                ret.insert(intermediate_elms.begin(), intermediate_elms.end());
            }
        }

        return ret;
    }
*/
    std::set<std::shared_ptr<DataElement>> get_subgraph(
        std::set<RTLIL::Cell*> data_cells
    ) const {
        log("Finding a subgraph for %d data cells\n", GetSize(data_cells));
        // Convert the RTLIL cells into data elements
        // This is to check the final element counts
        std::set<std::shared_ptr<DataElement>> original_data_elms;
        for (RTLIL::Cell* data_cell: data_cells) {
            if (this->acyclic_node_map.count(data_cell)) {
                original_data_elms.insert(
                    this->acyclic_node_map.at(data_cell));
            }
        }
        log("Translated data cells into %d data elements\n", GetSize(original_data_elms));

        // Collect the nodes that have been reached
        std::set<std::shared_ptr<DataElement>> reached_elms;

        // Keep a collection of paths propagating from all nodes
        // Perform BFS to advance the depth as least as possible
        std::deque<std::deque<std::shared_ptr<DataElement>>> constructing_paths;

        // A graph only consist of the pre-determined data cells
        std::map<std::shared_ptr<DataElement>, std::set<std::shared_ptr<DataElement>>> directed_edges;

        // Populate edge map
        for (std::shared_ptr<DataElement> data_elm: original_data_elms) {
            directed_edges[data_elm] = {};
        }

        // Populate with 2-length paths initially to the constructing set
        for (std::shared_ptr<DataElement> data_elm: original_data_elms) {
            // Note: There is a possibility that the data elements simply had no sinks
            // In this case, the current elements are simply a frontier
            // if (data_elm->sink.empty()) {
            //     reached_elms.insert(data_elm);
            // }
            // else {
            //     for (std::shared_ptr<DataElement> sink: data_elm->sink) {
            //         if (data_elm.get() == sink.get()) {
            //             continue;
            //         }
            //         constructing_paths.push_back({data_elm, sink});
            //     }
            // }

            for (std::shared_ptr<DataElement> sink: data_elm->sink) {
                if (data_elm.get() == sink.get()) {
                    continue;
                }
                constructing_paths.push_back({data_elm, sink});
            }

            // The original data cells' element counterpart should always be part of the subgraph
            reached_elms.insert(data_elm);
        }

        auto edge_causes_cycles = [&](
            std::pair<std::shared_ptr<DataElement>, std::shared_ptr<DataElement>> new_edge,
            const std::map<std::shared_ptr<DataElement>, std::set<std::shared_ptr<DataElement>>>& edge_graph
        ) -> bool {
            auto [new_src, new_sink] = new_edge;

            // Check if new_sink can already reach new_src
            std::vector<std::shared_ptr<DataElement>> work_list = {new_sink};
            std::set<std::shared_ptr<DataElement>> reached;

            while (!work_list.empty()) {
                std::shared_ptr<DataElement> curr = work_list.back();
                work_list.pop_back();

                // If we have reached new_src, simply return true
                if (curr.get() == new_src.get()) {
                    return true;
                }

                if (reached.count(curr)) {
                    // No need to repeat procesing
                    continue;
                }
                reached.insert(curr);

                for (std::shared_ptr<DataElement> sink: edge_graph.at(curr)) {
                    work_list.push_back(sink);
                }
            }

            return false;
        };

        while (!constructing_paths.empty()) {
            std::deque<std::shared_ptr<DataElement>> curr_path = constructing_paths.front();
            constructing_paths.pop_front();

            // FIXME: Arbitrary limit
            if (GetSize(curr_path) > 4) {
                continue;
            }

            // First, check the completeness of the path
            std::shared_ptr<DataElement> start_node = curr_path.front();
            std::shared_ptr<DataElement> end_node = curr_path.back();

            // This path goes between known nodes
            if (original_data_elms.count(start_node) && original_data_elms.count(end_node)) {
                // Check that this new path will not cause a loop
                if (edge_causes_cycles({start_node, end_node}, directed_edges)) {
                    // Terminate propagation, why?
                    // We assume the subgraph edges should have a length less than
                    // minimum path that would cause a cycle
                    log("Cycle avoided in the subgraph\n");
                    break;
                }

                // Record the edge
                directed_edges[start_node].insert(end_node);

                // In addition, record the intermediate element pointers
                reached_elms.insert(curr_path.begin(), curr_path.end());

                continue;
            }

            // Otherwise, propagate the path towards the sink back to the constructing queue
            for (std::shared_ptr<DataElement> sink: curr_path.back()->sink) {
                // TODO: Do not propagate into a loop
                if (std::find(curr_path.begin(), curr_path.end(), sink) != curr_path.end()) {
                    continue;
                }

                std::deque<std::shared_ptr<DataElement>> new_path = curr_path;
                new_path.push_back(sink);

                if (std::find(constructing_paths.begin(), constructing_paths.end(), new_path) == constructing_paths.end()) {
                    constructing_paths.push_back(new_path);
                }
            }
        }

        log("%d enclosed elements found before cycle detected\n", GetSize(reached_elms));
        return reached_elms;
    }

    std::set<std::shared_ptr<DataElement>> get_inter_nodes(
        std::shared_ptr<DataElement> src,
        std::shared_ptr<DataElement> sink
    ) const {
        // Base case
        if (src == sink) {
            return {src};
        }

        std::set<std::shared_ptr<DataElement>> ret;

        // Attempt to propagate
        for (std::shared_ptr<DataElement> next: src->sink) {
            std::set<std::shared_ptr<DataElement>> found_sinks = this->get_inter_nodes(
                next, sink);
            ret.insert(found_sinks.begin(), found_sinks.end());
        }

        // If the search actually propagated, add the current node
        if (!ret.empty()) {
            ret.insert(src);
        }

        return ret;
    }

    std::set<std::shared_ptr<DataElement>> get_subgraph_inter_nodes(
        std::shared_ptr<DataElement> src,
        std::shared_ptr<DataElement> sink,
        const std::set<std::shared_ptr<DataElement>>& subgraph_nodes
    ) const {
        // Base case
        if (src == sink) {
            return {src};
        }

        std::set<std::shared_ptr<DataElement>> ret;

        // Attempt to propagate
        for (std::shared_ptr<DataElement> next: src->sink) {
            // Only propagate over the subgraph
            if (subgraph_nodes.count(next) == 0) {
                continue;
            }
            std::set<std::shared_ptr<DataElement>> found_sinks = this->get_subgraph_inter_nodes(
                next, sink, subgraph_nodes);
            ret.insert(found_sinks.begin(), found_sinks.end());
        }

        // If the search actually propagated, add the current node
        if (!ret.empty()) {
            ret.insert(src);
        }

        return ret;
    }

    std::set<Wire> get_intersection_wires(
        std::set<std::shared_ptr<DataElement>> data_if
    ) const {
        std::set<Wire> ret;

        for (std::shared_ptr<DataElement> curr: data_if) {
            bool found_iface = false;
            for (RTLIL::Cell* curr_node: curr->subgraph_nodes) {
                // Loop over all ports and pin numbers
                for (auto [port_name, curr_src_vec]: this->circuit_graph.source_map.at(curr_node)) {
                    for (size_t i = 0; i < curr_src_vec.size(); ++i) {
                        Source curr_src = curr_src_vec.at(i);
                        // Constants should not be an interface
                        // Assume data opreration is non-unity, so it should be a module input
                        if (std::holds_alternative<RTLIL::SigBit>(curr_src)) {
                            continue;
                        }

                        // The proposed sink should appear in the data path graph
                        auto [curr_src_cell, curr_src_port, curr_src_idx] = std::get<CellPin>(curr_src);
                        // Reject sources that are not part of the data path
                        if (this->acyclic_node_map.count(curr_src_cell) == 0) {
                            continue;
                        }

                        // The source could still be within the same DataElement
                        // This is due to acyclic-ization
                        if (curr->subgraph_nodes.count(curr_src_cell)) {
                            continue;
                        }

                        ret.insert(std::make_pair<CellPin, CellPin>(
                            {
                                curr_src_cell,
                                curr_src_port,
                                curr_src_idx
                            },
                            {
                                curr_node,
                                port_name,
                                i
                            }
                        ));
                        found_iface = true;
                    }
                }
            }

            // FIXME: Compromise for selecting the successor if no suitable predecessor
            if (!found_iface) {
                for (RTLIL::Cell* curr_node: curr->subgraph_nodes) {
                    for (auto [port_name, curr_sink_vec]: this->circuit_graph.sink_map.at(curr_node)) {
                        for (size_t i = 0; i < curr_sink_vec.size(); ++i) {
                            for (Sink curr_sink: curr_sink_vec.at(i)) {
                                // FIXME: Assume sinks are not external interfaces
                                if (std::holds_alternative<RTLIL::SigBit>(curr_sink)) {
                                    continue;
                                }

                                auto [curr_sink_cell, curr_sink_port, curr_sink_idx] = std::get<CellPin>(curr_sink);
                                // Reject sources that are not part of the data path
                                if (this->acyclic_node_map.count(curr_sink_cell) == 0) {
                                    continue;
                                }

                                // The sink could still be within the same DataElement
                                // This is due to acyclic-ization
                                if (curr->subgraph_nodes.count(curr_sink_cell)) {
                                    continue;
                                }

                                // Report the wire driver
                                ret.insert(std::make_pair<CellPin, CellPin>(
                                    {
                                        curr_node,
                                        port_name,
                                        i
                                    },
                                    {
                                        curr_sink_cell,
                                        curr_sink_port,
                                        curr_sink_idx
                                    }
                                ));
                                found_iface = true;
                            }
                        }
                    }
                }
            }

            // FIXME: Directly use output pins if no suitable intrerface is found
            if (!found_iface) {
                for (RTLIL::Cell* curr_node: curr->subgraph_nodes) {
                    for (auto [port_name, port_sig]: curr_node->connections()) {
                        for (int i = 0; i < GetSize(port_sig); ++i) {
                            for (Sink sink: this->circuit_graph.sink_map.at(curr_node).at(port_name).at(i)) {
                                if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                                    continue;
                                }

                                auto [sink_cell, sink_port, sink_idx] = std::get<CellPin>(sink);
                                ret.insert(std::make_pair<CellPin, CellPin>(
                                    {curr_node, port_name, i},
                                    {sink_cell, sink_port, sink_idx}
                                ));
                            }
                        }
                    }
                }
            }
        }

        return ret;
    }

    std::set<RTLIL::Cell*> get_subgraph_successor_cells(
        const std::set<std::shared_ptr<DataElement>>& subgraph,
        const std::set<Wire>& interface
    ) const {
        std::set<RTLIL::Cell*> ret;
        std::vector<std::shared_ptr<DataElement>> work_list;
        for (auto [from_cell, to_cell]: interface) {
            work_list.push_back(this->acyclic_node_map.at(std::get<0>(from_cell)));
            work_list.push_back(this->acyclic_node_map.at(std::get<0>(to_cell)));
        }

        while (!work_list.empty()) {
            std::shared_ptr<DataElement> curr_elm = work_list.back();
            work_list.pop_back();

            ret.insert(curr_elm->subgraph_nodes.begin(), curr_elm->subgraph_nodes.end());
            for (std::shared_ptr<DataElement> sink: curr_elm->sink) {
                if (subgraph.count(sink)) {
                    work_list.push_back(sink);
                }
            }
        }

        return ret;
    }

    std::set<std::shared_ptr<DataElement>> promote_inputs(
        const std::set<std::shared_ptr<DataElement>>& subgraph_nodes,
        const std::set<std::shared_ptr<DataElement>>& data_inputs
    ) const {
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
                    if (!this->get_subgraph_inter_nodes(frontier_src, frontier_sink, subgraph_nodes).empty()) {
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
    }

    std::set<std::shared_ptr<DataElement>> advance_frontier(
        const std::set<std::shared_ptr<DataElement>>& subgraph,
        const std::set<std::shared_ptr<DataElement>> frontier
    ) const {
        // Promote all cells to its successors
        std::set<std::shared_ptr<DataElement>> new_frontier_elms;
        std::set<std::shared_ptr<DataElement>> frontier_elms = frontier;

        for (std::shared_ptr<DataElement> frontier_elm: frontier_elms) {
            for (std::shared_ptr<DataElement> next_elm: frontier_elm->sink) {
                if (subgraph.count(next_elm)) {
                    new_frontier_elms.insert(next_elm);
                }
            }
        }

        if (!new_frontier_elms.empty()) {
            return new_frontier_elms;
        }

        // Otherwise, we allow the process to go beyond the subgraph.
        frontier_elms = frontier;

        for (std::shared_ptr<DataElement> frontier_elm: frontier_elms) {
            for (std::shared_ptr<DataElement> next_elm: frontier_elm->sink) {
                new_frontier_elms.insert(next_elm);
            }
        }

        return new_frontier_elms;
    }

    std::vector<std::set<std::shared_ptr<DataElement>>> collect_frontiers(
        const std::set<std::shared_ptr<DataElement>>& subgraph,
        const std::set<std::shared_ptr<DataElement>>& first_frontier
    ) const {
        std::vector<std::set<std::shared_ptr<DataElement>>> frontier_collection;

        frontier_collection.push_back(first_frontier);

        std::set<std::shared_ptr<DataElement>> curr = first_frontier;
        std::set<std::shared_ptr<DataElement>> next = {};
        do {
            std::set<std::shared_ptr<DataElement>> next_candidates = this->advance_frontier(subgraph, curr);
            if (next_candidates.empty()) {
                break;
            }
            next = this->promote_inputs(subgraph, next_candidates);

            if (std::find(frontier_collection.begin(), frontier_collection.end(), next) != frontier_collection.end()) {
                // We have reached a loop,
                // There are no more frontier to advance
                break;
            }

            frontier_collection.push_back(next);
            curr = next;
        } while (true);

        return frontier_collection;
    }
};


struct DataPathPass : public Pass {
    DataPathPass() : Pass("data_path") { }

    void execute(vector<string>, Design *design) override {
        RTLIL::Module* top = design->top_module();

        if (top == nullptr) {
            log_error("No top module found!\n");
            return;
        }

        const SigMap sigmap(top);

        DataPath dp(sigmap, top);
    }
} DataPathPass;

PRIVATE_NAMESPACE_END
