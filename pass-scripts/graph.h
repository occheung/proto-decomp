#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include <variant>


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


#ifndef CIRCUITGRAPH_H
#define CIRCUITGRAPH_H


typedef std::tuple<RTLIL::Cell*, IdString, int> CellPin;
typedef std::variant<CellPin, RTLIL::SigBit> Sink;
// Syntactic sugar
typedef Sink Source;
typedef std::pair<CellPin, CellPin> Wire;

typedef std::pair<RTLIL::Cell*, CellPin> HandshakeInfo;
typedef std::set<Wire> DataInterface;
typedef std::tuple<CellPin, CellPin, DataInterface> ValidReadyProto;


struct CircuitGraph {
    // Respecting the directions of the directed edges
    // Maps the output ports to all sinks
    dict<RTLIL::Cell*, dict<IdString, std::vector<std::vector<Sink>>>> sink_map;

    // Opposite the directions of the directed edges
    // Maps the input ports to its source.
    // There should only be 1 source per input bit.
    dict<RTLIL::Cell*, dict<IdString, std::vector<Source>>> source_map;

    // Abstracted time domain graph
    // Only shows abstract connections between FFs
    dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> dff_sink_graph;
    dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> dff_source_graph;

    CircuitGraph(const SigMap& sigmap, RTLIL::Module* module) {
        // Temporarily maintain a SigBit to cell pin mapping
        // A SigBit may only be driven by a cell output
        dict<RTLIL::SigBit, CellPin> bit_out_pin_map;
        // A SigBit may drive multiple cell inputs
        dict<RTLIL::SigBit, std::set<CellPin>> bit_in_pin_map;

        // Populate sink/source maps
        for (RTLIL::Cell* cell: module->cells()) {
            // Populate cell in both maps
            dict<IdString, std::vector<Source>> source_dict;
            this->source_map[cell] = source_dict;

            dict<IdString, std::vector<std::vector<Sink>>> sink_dict;
            this->sink_map[cell] = sink_dict;

            for (auto& [port_name, unmapped_port_sig_spec]: cell->connections()) {
                RTLIL::SigSpec port_sig_spec = sigmap(unmapped_port_sig_spec);

                if (cell->input(port_name)) {
                    // Populate input port in source map
                    std::vector<Source> sources;
                    for (int i = 0; i < GetSize(port_sig_spec); ++i) {
                        Source src;
                        sources.push_back(src);
                    }
                    this->source_map[cell][port_name] = sources;
                } else {
                    // Populate output port in sink map
                    std::vector<std::vector<Sink>> sink_lists;
                    for (int i = 0; i < GetSize(port_sig_spec); ++i) {
                        std::vector<Sink> sink_list;
                        sink_lists.push_back(sink_list);
                    }
                    this->sink_map[cell][port_name] = sink_lists;
                }
            }
        }

        // Collect a set of input wires and output wires
        std::set<RTLIL::Wire*> input_wires;
        std::set<RTLIL::Wire*> output_wires;
        for (RTLIL::Wire* wire: module->wires()) {
            if (wire->port_input) {
                input_wires.insert(wire);
            } else if (wire->port_output) {
                output_wires.insert(wire);
            }
        }

        // Check all inputs and outputs before concluding the wire is an IO
        // This is necessaery because IO may be connected indirectly to cells
        auto is_input_wire = [&](
            RTLIL::Wire* wire
        ) -> bool {
            for (RTLIL::Wire* in_wire: input_wires) {
                if (sigmap(in_wire) == sigmap(wire)) {
                    return true;
                }
            }

            return false;
        };
        auto is_output_wire = [&](
            RTLIL::Wire* wire
        ) -> bool {
            for (RTLIL::Wire* out_wire: output_wires) {
                if (sigmap(out_wire) == sigmap(wire)) {
                    return true;
                }
            }

            return false;
        };

        // Populate SigBit maps
        for (RTLIL::Cell* cell: module->cells()) {
            for (auto& [port_name, unmapped_port_sig_spec]: cell->connections()) {
                RTLIL::SigSpec port_sig_spec = sigmap(unmapped_port_sig_spec);

                for (int i = 0; i < GetSize(port_sig_spec); ++i) {
                    RTLIL::SigBit bit = port_sig_spec[i];
                    CellPin pin = {cell, port_name, i};
                    if (cell->input(port_name)) {
                        // If the bit is just a constant, skip the SigBit map process
                        // If the bit is module input, skip as well
                        // It does not contribute to the connectivity problem
                        if (!(bit.is_wire()) || is_input_wire(bit.wire)) {
                            this->source_map[cell][port_name][i] = bit;
                        } else {
                            // Fill in SigBit info
                            if (bit_in_pin_map.count(bit) == 0) {
                                std::set<CellPin> pin_list;
                                bit_in_pin_map[bit] = pin_list;
                            }
                            bit_in_pin_map[bit].insert(pin);
                        }
                    } else {
                        // Filter output pin connections towards module outputs
                        // Directly write the pin to the sink map
                        if (is_output_wire(bit.wire)) {
                            this->sink_map[cell][port_name][i].push_back(bit);
                        }

                        // Output wire needs to be propagated to the SigBit maps
                        // It can drive other internal cells
                        bit_out_pin_map[bit] = pin;
                    }
                }
            }
        }

        // Use SigBit equalities to construct graphs
        for (auto& [output_sig_bit, output_cell_pin]: bit_out_pin_map) {
            auto& [output_cell, output_port_name, output_pin_idx] = output_cell_pin;

            // If the SigBit is not present in the input map, it must be a module output
            // There are no info for Sink and Source maps
            if (bit_in_pin_map.count(output_sig_bit) == 0) {
                continue;
            }

            // Note: Module outputs were filtered
            const std::set<CellPin>& input_cell_pins = bit_in_pin_map.at(output_sig_bit);
            for (CellPin input_cell_pin: input_cell_pins) {
                auto& [input_cell, input_port_name, input_pin_idx] = input_cell_pin;
                this->source_map[input_cell][input_port_name][input_pin_idx] = output_cell_pin;
                this->sink_map[output_cell][output_port_name][output_pin_idx].push_back(input_cell_pin);
            }
        }

        // Populate the DFF graph
        // Note: Self loop is only possible for DFF cells, since combinatorial loops are illegal structure
        for (RTLIL::Cell* dff_src_cell: module->cells()) {
            // Only start from DFFs
            if (RTLIL::builtin_ff_cell_types().count(dff_src_cell->type) == 0) {
                continue;
            }

            if (this->dff_sink_graph.count(dff_src_cell) == 0) {
                this->dff_sink_graph[dff_src_cell] = {};
            }

            std::vector<RTLIL::Cell*> list;
            list.push_back(dff_src_cell);

            while (!list.empty()) {
                RTLIL::Cell* cell = list.back();
                list.pop_back();

                // The cell can reach all its output
                for (const auto& [port, sink_lists]: this->sink_map[cell]) {
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
                                list.push_back(sink_cell);
                            } else {
                                // Only record reachable DFFs
                                this->dff_sink_graph[dff_src_cell].insert(sink_cell);
                            }
                        }
                    }
                }
            }
        }

        // Invert the sink graph to find the source graph
        for (const auto& [dff_src, dff_sinks]: this->dff_sink_graph) {
            for (RTLIL::Cell* dff_sink: dff_sinks) {
                if (this->dff_source_graph.count(dff_sink) == 0) {
                    this->dff_source_graph[dff_sink] = {};
                }
                this->dff_source_graph[dff_sink].insert(dff_src);
            }
        }
    }

    void print_maps() const {
        for (const auto& [output_cell, out_port_dict]: this->sink_map) {
            log("From cell %s\n", output_cell->name.c_str());
            for (const auto& [output_port, sink_lists]: out_port_dict) {
                for (size_t port_idx = 0; port_idx < sink_lists.size(); ++port_idx) {
                    const auto& sink_list = sink_lists[port_idx];
                    for (const Sink& sink: sink_list) {
                        if (std::holds_alternative<CellPin>(sink)) {
                            const auto& [sink_cell, sink_port, sink_idx] = std::get<CellPin>(sink);
                            log("%s[%ld] -> %s.%s[%d]\n", output_port.c_str(), port_idx, sink_cell->name.c_str(), sink_port.c_str(), sink_idx);
                        } else {
                            const RTLIL::SigBit& sink_bit = std::get<RTLIL::SigBit>(sink);
                            log("%s[%ld] -> OUTPUT:%d[%d]\n", output_port.c_str(), port_idx, sink_bit.wire->port_id, sink_bit.offset);
                        }
                    }
                }
            }
        }

        for (const auto& [dff_src, dff_dests]: this->dff_sink_graph) {
            log("DFF source: %s -> {", dff_src->name.c_str());
            for (const RTLIL::Cell* dff_dest: dff_dests) {
                log("%s, ", dff_dest->name.c_str());
            }
            log("}\n");
        }
    }

    // Note: Combinatorial graph is acyclic
    std::set<RTLIL::Cell*> get_intermediate_comb_cells(
        CellPin src,
        CellPin dest
    ) const {
        // Defensive programming
        if (src == dest) {
            return { std::get<0>(dest) };
        }

        std::set<RTLIL::Cell*> ret;

        auto [src_cell, src_port, src_idx] = src;

        // If the destination is an output, check if dest is the output of src_cell
        const auto& [dest_cell, dest_port, dest_pin] = dest;
        if (dest_cell->output(dest_port)) {
            if (src_cell == dest_cell) {
                return { src_cell };
            }
        }

        const dict<IdString, std::vector<std::vector<Sink>>>& cell_sink_map = this->sink_map.at(src_cell);
        if (cell_sink_map.count(src_port) == 0) {
            return {};
        }

        const std::vector<Sink>& sink_list = cell_sink_map.at(src_port)[src_idx];
        for (const Sink& sink: sink_list) {
            // Reject any sink that are constants or external port
            if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                continue;
            }

            CellPin next_src = std::get<CellPin>(sink);
            // If propagating to the destination, then the destination is found
            if (next_src == dest) {
                ret.insert(std::get<0>(next_src));
                continue;
            }

            // If the propagating target is non-combinatorial AND not
            // the destination, do NOT propagate
            RTLIL::Cell* sink_cell = std::get<0>(next_src);
            if (RTLIL::builtin_ff_cell_types().count(sink_cell->type)) {
                continue;
            }

            // Propagate to all possible outputs of the sink cell
            for (const auto& [sink_port, sink_sig_spec]: sink_cell->connections()) {
                if (sink_cell->output(sink_port)) {
                    for (int i = 0; i < GetSize(sink_sig_spec); ++i) {
                        std::set<RTLIL::Cell*> reached_recur = get_intermediate_comb_cells(
                            {sink_cell, sink_port, i}, dest);
                        ret.insert(reached_recur.begin(), reached_recur.end());
                    }
                }
            }
        }

        // If the children reports destination cell was reached,
        // append the source cell to the return set
        //
        // Note that the returned set from children forms a one-to-one mapping
        // with the fact that the destination was reached
        if (!(ret.empty())) {
            ret.insert(std::get<0>(src));
        }

        return ret;
    }

    // Return true if src can reach dest, as long as it is the case for some pair
    // Only data dependence is considered. (i.e. DFF output does not depend on control inputs)
    //
    // Assume the CellPin are all outputs.
    bool reachable(std::set<CellPin> srcs, std::set<CellPin> dests) const {
        std::set<CellPin> visited;
        std::vector<CellPin> work_list;

        for (CellPin src: srcs) {
            work_list.push_back(src);
            visited.insert(src);
        }

        while (!work_list.empty()) {
            auto [src_cell, src_port, src_pin_idx] = work_list.back();
            work_list.pop_back();

            // Base case: We have reached one of the destination
            if (dests.count({src_cell, src_port, src_pin_idx})) {
                return true;
            }

            for (Sink sink: this->sink_map.at(src_cell).at(src_port).at(src_pin_idx)) {
                if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                    continue;
                }

                auto [sink_cell, sink_port, sink_pin_idx] = std::get<CellPin>(sink);

                // DFFs: Propagate to Q port if not already visited
                // MUXes: Propagate to Y port if not already visited
                // Otherwise: Propagate to every single outputs if not already visited
                if (RTLIL::builtin_ff_cell_types().count(sink_cell->type)) {
                    CellPin prop = {sink_cell, "\\Q", sink_pin_idx};
                    if (visited.count(prop) == 0) {
                        visited.insert(prop);
                        work_list.push_back(prop);
                    }
                } else if (sink_cell->type.in(ID($_MUX_))) {
                    CellPin prop = {sink_cell, "\\Y", 0};
                    if (visited.count(prop) == 0) {
                        visited.insert(prop);
                        work_list.push_back(prop);
                    }
                } else {
                    for (const auto& [output_port, output_sig]: sink_cell->connections()) {
                        if (sink_cell->output(output_port)) {
                            for (int i = 0; i < GetSize(output_sig); ++i) {
                                CellPin prop = {sink_cell, output_port, i};
                                if (visited.count(prop) == 0) {
                                    visited.insert(prop);
                                    work_list.push_back(prop);
                                }
                            }
                        }
                    }
                }
            }
        }

        return false;
    }

    // Return true if any sources manage to reach the sinks BEFORE any sinks
    // reach the sources
    bool cyclic_first_reachable(
        std::set<CellPin> srcs,
        std::set<CellPin> sinks
    ) const {
        enum PropSource {
            SOURCE, SINK
        };
        std::deque<std::pair<CellPin, PropSource>> work_list;
        std::set<CellPin> visited_from_src;
        std::set<CellPin> visited_from_sink;

        // Populate work list
        for (CellPin src: srcs) {
            work_list.push_back({src, SOURCE});
        }
        for (CellPin sink: sinks) {
            work_list.push_back({sink, SINK});
        }

        while (!work_list.empty()) {
            auto [curr_pin, prop_src] = work_list.front();
            work_list.pop_front();

            // Loop avoidance
            if (prop_src == PropSource::SOURCE) {
                if (visited_from_src.count(curr_pin)) {
                    continue;
                }
                visited_from_src.insert(curr_pin);
            } else {
                if (visited_from_sink.count(curr_pin)) {
                    continue;
                }
                visited_from_sink.insert(curr_pin);
            }

            // Check if we have propagated to the opposite side
            if (prop_src == PropSource::SOURCE) {
                if (sinks.count(curr_pin)) {
                    // A source have reached a sink before any sink reaches the sources
                    return true;
                }
            } else {
                if (srcs.count(curr_pin)) {
                    // A sink have reached a source before any source reaches the sink
                    return false;
                }
            }

            // Find the next cell
            auto [curr_cell, curr_port, curr_idx] = curr_pin;
            for (Sink next: this->sink_map.at(curr_cell).at(curr_port).at(curr_idx)) {
                if (std::holds_alternative<RTLIL::SigBit>(next)) {
                    continue;
                }

                auto [next_cell, next_src_port, next_src_idx] = std::get<CellPin>(next);
                for (auto [out_port, out_sig]: next_cell->connections()) {
                    if (next_cell->output(out_port)) {
                        // If we are propagating through DFF from port D,
                        // only propagate to the corresponding bit of port Q
                        if (RTLIL::builtin_ff_cell_types().count(next_cell->type) && next_src_port == "\\D") {
                            if (out_port != "\\Q") {
                                continue;
                            }
                            work_list.push_back(
                                {{next_cell, "\\Q", next_src_idx}, prop_src}
                            );
                        }

                        else {
                            for (int i = 0; i < GetSize(out_sig); ++i) {
                                work_list.push_back(
                                    {{next_cell, out_port, i}, prop_src}
                                );
                            }
                        }
                    }
                }
            }
        }

        // Source have simply failed to reach the sink
        return false;
    }

    std::set<CellPin> get_dominated_frontier(
        // The returned leaves must be at least partially dependent on the
        // mandatory sources
        const std::set<CellPin> mandatory_srcs,
        // The returned leaves may depend on the tolerated sources
        const std::set<CellPin> tolerated_srcs
        // The returned leaves must be totally dependent to input sources only
    ) const {
        // Directly controlled pins
        std::set<CellPin> weak_ctrl_srcs = tolerated_srcs;
        std::set<CellPin> mandated_ctrl_srcs = mandatory_srcs;
        std::set<CellPin> mandatorily_dominated_pins;
        std::set<RTLIL::Cell*> mandatorily_dominated_cells;
        // Initialize the work list with dependencies of both mandatory and tolerated
        // sources. It is possible that the intermediates in the SSC may drive some
        // of the output pins
        //
        // Keep the previous pin to compute mandatorily connected cell easier
        std::vector<std::pair<CellPin, RTLIL::Cell*>> work_list;

        for (const auto& [src_cell, src_pin, src_idx]: mandatory_srcs) {
            for (const Sink& sink: this->sink_map.at(src_cell).at(src_pin).at(src_idx)) {
                if (std::holds_alternative<CellPin>(sink)) {
                    work_list.push_back({
                        {src_cell, src_pin, src_idx},
                        std::get<0>(std::get<CellPin>(sink))
                    });
                }
            }
        }

        for (const auto& [src_cell, src_pin, src_idx]: tolerated_srcs) {
            for (const Sink& sink: this->sink_map.at(src_cell).at(src_pin).at(src_idx)) {
                if (std::holds_alternative<CellPin>(sink)) {
                    work_list.push_back({
                        {src_cell, src_pin, src_idx},
                        std::get<0>(std::get<CellPin>(sink))
                    });
                }
            }
        }

        while (!work_list.empty()) {
            auto [prev, curr] = work_list.back();
            work_list.pop_back();

            // Do not process non-combinatorial elements
            if (RTLIL::builtin_ff_cell_types().count(curr->type)) {
                continue;
            }

            // If the pin is totally directly controlled by controlled sources,
            // Add the pin to the appropriate ctrl and dominance lists.
            // Push all its dependencies to the work list
            bool dominated = true;
            bool mandatory_dominance = false;
            for (const auto& [_pin_id, pin_srcs]: this->source_map.at(curr)) {
                for (const Source& pin_src: pin_srcs) {
                    // We started with connected pins
                    if (!std::holds_alternative<CellPin>(pin_src)) {
                        // Sigbit can hold constants as well
                        // Constant should not be considered in dominance
                        if (std::get<RTLIL::SigBit>(pin_src).is_wire()) {
                            dominated = false;
                            break;
                        }
                    } else {
                        CellPin cell_pin_src = std::get<CellPin>(pin_src);
                        if (weak_ctrl_srcs.count(cell_pin_src) == 0
                                && mandated_ctrl_srcs.count(cell_pin_src) == 0) {
                            dominated = false;
                            break;
                        }

                        // If any input pin of the cell is solely driven by mandated
                        // pins, this cell qualifies for an output pin
                        //
                        // Note: It is not necessarily a leaf
                        if (mandated_ctrl_srcs.count(cell_pin_src)) {
                            mandatory_dominance = true;
                        }
                    }
                }
            }

            if (dominated) {
                for (const auto& [pin_id, output_pins]: this->sink_map.at(curr)) {
                    for (size_t i = 0; i < output_pins.size(); ++i) {
                        if (mandatory_dominance) {
                            mandated_ctrl_srcs.insert({curr, pin_id, i});
                            mandatorily_dominated_pins.insert({curr, pin_id, i});
                            mandatorily_dominated_cells.insert(curr);
                        } else {
                            weak_ctrl_srcs.insert({curr, pin_id, i});
                        }

                        for (const Sink& sink: output_pins[i]) {
                            if (std::holds_alternative<CellPin>(sink)) {
                                work_list.push_back({
                                    {curr, pin_id, i},
                                    std::get<0>(std::get<CellPin>(sink))
                                });
                            }
                        }
                    }
                }
            }

        }

        // log("Dominated pins:\n");
        // for (auto [curr_cell, curr_port_id, curr_port_idx]: mandatorily_dominated_pins) {
        //     log("Cell %s Port %s Index %d\n", log_id(curr_cell), log_id(curr_port_id), curr_port_idx);
        // }

        // Only include pins that drives a non-dominated cell
        std::set<CellPin> frontier;
        for (auto [curr_cell, curr_port_id, curr_port_idx]: mandatorily_dominated_pins) {
            for (const Sink& sink: this->sink_map.at(curr_cell).at(curr_port_id).at(curr_port_idx)) {
                if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                    continue;
                }

                RTLIL::Cell* sink_cell = std::get<0>(std::get<CellPin>(sink));
                if (mandatorily_dominated_cells.count(sink_cell) == 0) {
                    frontier.insert({
                        curr_cell, curr_port_id, curr_port_idx
                    });
                }
            }
        }

        // If somehow there isn't a frontier, it means that the mandatory sources are the frontier
        if (frontier.empty()) {
            return mandatory_srcs;
        }

        return frontier;
    }

    std::pair<std::set<RTLIL::Cell*> ,std::set<CellPin>> get_primary_outputs(
        std::set<RTLIL::Cell*> source_fsm_ffs,
        std::set<RTLIL::Cell*> sink_fsm_ffs,
        FfInitVals* ff_init_vals
    ) const {
        // If the source and sinks are not interconnected,
        // Immediately return an empty set
        bool has_path = false;
        for (RTLIL::Cell* src_ff: source_fsm_ffs) {
            for (int out_pin_idx = 0; out_pin_idx < GetSize(src_ff->getPort("\\Q")); ++out_pin_idx) {
                CellPin src_pin = {src_ff, "\\Q", out_pin_idx};
                for (RTLIL::Cell* sink_ff: sink_fsm_ffs) {
                    // Decide whether the sinking port is EN or D
                    FfData sink_ff_data(ff_init_vals, sink_ff);
                    IdString sink_port = sink_ff_data.has_ce? "\\EN": "\\D";

                    for (int in_pin_idx = 0; in_pin_idx < GetSize(src_ff->getPort(sink_port)); ++in_pin_idx) {
                        CellPin sink_pin = {sink_ff, sink_port, in_pin_idx};

                        // Check reachability
                        std::set<RTLIL::Cell*> inter_cells = this->get_intermediate_comb_cells(src_pin, sink_pin);
                        if (!inter_cells.empty()) {
                            has_path = true;
                        }
                    }
                }
            }
        }

        if (!has_path) {
            return {{}, {}};
        }

        // Find the output pins of the FF group
        // Output pins feed to cells that are not totally dependent on the FF group
        std::set<CellPin> ff_group_state_bits;
        for (RTLIL::Cell* src_ff: source_fsm_ffs) {
            for (int out_pin_idx = 0; out_pin_idx < GetSize(src_ff->getPort("\\Q")); ++out_pin_idx) {
                ff_group_state_bits.insert({src_ff, "\\Q", out_pin_idx});
            }
        }
        std::set<CellPin> ff_group_outputs = this->get_dominated_frontier(
            ff_group_state_bits,
            {}
        );

        // Find the primary output pins
        // Output pins are primary if it does not drive any outputs that
        // also feeds into the sink FF
        std::set<CellPin> primary_outputs;
        std::set<RTLIL::Cell*> all_outputs_half_path_set;
        for (CellPin output: ff_group_outputs) {
            const auto& [out_cell, out_pin_name, out_pin_idx] = output;

            // A set of cells that are intermediates between the output pins and the sink FFs
            std::set<RTLIL::Cell*> half_path_set;
            for (RTLIL::Cell* sink_ff: sink_fsm_ffs) {
                FfData sink_ff_data(ff_init_vals, sink_ff);
                IdString sink_port = sink_ff_data.has_ce? "\\EN": "\\D";

                for (int sink_port_idx = 0; sink_port_idx < GetSize(sink_ff->getPort(sink_port)); ++sink_port_idx) {
                    std::set<RTLIL::Cell*> i_half_path_set = this->get_intermediate_comb_cells(
                        output, {sink_ff, sink_port, sink_port_idx}
                    );

                    half_path_set.insert(i_half_path_set.begin(), i_half_path_set.end());
                } 
            }

            if (!half_path_set.empty()) {
                all_outputs_half_path_set.insert(half_path_set.begin(), half_path_set.end());

                bool primary_output = true;
                for (Sink output_sink: this->sink_map.at(out_cell).at(out_pin_name).at(out_pin_idx)) {
                    // Not interested in boundary cases
                    if (std::holds_alternative<RTLIL::SigBit>(output_sink)) {
                        continue;
                    }

                    // If there exist some successor of this output pin, where it is also an output pin,
                    // This output pin is not a primary output pin
                    RTLIL::Cell* succ_cell = std::get<0>(std::get<CellPin>(output_sink));
                    bool cell_controls_output = false;
                    for (CellPin other_out_pin: ff_group_outputs) {
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
                    primary_outputs.insert(output);
                }
            }
        }

        return {
            all_outputs_half_path_set, primary_outputs
        };
    }

    std::pair<CellPin, int> get_first_divergence(
        CellPin primary_out,
        std::set<RTLIL::Cell*> control_path_cells,
        std::set<RTLIL::Cell*> data_path_cells,
        std::set<RTLIL::Cell*> opposite_data_path_cells
    ) {
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

        // Keep track of the divergence depth
        // Eventually we need the depth to optimize triangulation guess
        int depth = -1;

        while (!work_list.empty() && !found_divergence) {
            depth += 1;

            auto [curr_cell, curr_port, curr_pin_idx] = work_list.back();
            work_list.pop_back();

            // If the logic path starts to branch, stop trasversing.
            bool found_suitable_successor = false;
            vector<Sink> curr_sinks = this->sink_map.at(curr_cell).at(curr_port)[curr_pin_idx];
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

    std::set<RTLIL::Cell*> get_first_convergence(
        const std::set<RTLIL::Cell*>& ctrl_to_data_path,
        const std::set<RTLIL::Cell*>& opposite_ctrl_to_data_path
    ) const {
        // Compute the intersection of 2 sets
        std::set<RTLIL::Cell*> set_intersect;
        for (RTLIL::Cell* data_path_cell: ctrl_to_data_path) {
            if (opposite_ctrl_to_data_path.count(data_path_cell)) {
                set_intersect.insert(data_path_cell);
            }
        }

        // Find the root(s) of the set, which are all the cells that have no
        // predecessors within the intersecting set
        std::set<RTLIL::Cell*> roots;
        for (RTLIL::Cell* common_cell: set_intersect) {
            bool found_predecessor = false;
            for (const auto& [in_port, in_port_sig_info]: this->source_map.at(common_cell)) {
                for (Source src: in_port_sig_info) {
                    if (std::holds_alternative<RTLIL::SigBit>(src)) {
                        continue;
                    }

                    RTLIL::Cell* src_cell = std::get<0>(std::get<CellPin>(src));
                    if (set_intersect.count(src_cell)) {
                        found_predecessor = true;
                    }
                }
            }

            if (!found_predecessor) {
                roots.insert(common_cell);
            }
        }

        return roots;
    }
};

#endif

PRIVATE_NAMESPACE_END
