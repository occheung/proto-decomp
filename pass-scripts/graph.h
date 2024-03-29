#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include <variant>


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


typedef std::tuple<RTLIL::Cell*, IdString, int> CellPin;
typedef std::variant<CellPin, RTLIL::SigBit> Sink;
// Syntactic sugar
typedef Sink Source;


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
                        if (!(bit.is_wire()) || bit.wire->port_input) {
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
                        //
                        // FIXME: The output wire can drive other cells in the same module
                        if (bit.wire->port_output) {
                            this->sink_map[cell][port_name][i].push_back(bit);
                        }
                        // else {
                        //     bit_out_pin_map[bit] = pin;
                        // }

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

    // Note: We only consider combinatorial elements
    std::set<CellPin> get_non_dominated_pins(const std::set<CellPin> srcs) const {
        // Directly controlled pins
        std::set<CellPin> controlled_srcs = srcs;
        std::set<CellPin> dominated_pins;
        // Initialize the work list with dependencies of srcs
        std::vector<RTLIL::Cell*> work_list;
        for (const auto& [src_cell, src_pin, src_idx]: srcs) {
            for (const Sink& sink: this->sink_map.at(src_cell).at(src_pin).at(src_idx)) {
                if (std::holds_alternative<CellPin>(sink)) {
                    work_list.push_back(std::get<0>(std::get<CellPin>(sink)));
                }
            }
        }


        while (!work_list.empty()) {
            RTLIL::Cell* curr = work_list.back();
            work_list.pop_back();

            // Do not process non-combinatorial elements
            if (RTLIL::builtin_ff_cell_types().count(curr->type)) {
                continue;
            }

            // If the pin is totally directly controlled by controlled sources,
            // Add the pin to the controlled pins and dominated pins.
            // Push all its dependencies to the work list
            bool dominated = true;
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
                        if (controlled_srcs.count(std::get<CellPin>(pin_src)) == 0) {
                            dominated = false;
                            break;
                        }
                    }
                }
            }

            if (dominated) {
                for (const auto& [pin_id, output_pins]: this->sink_map.at(curr)) {
                    for (size_t i = 0; i < output_pins.size(); ++i) {
                        controlled_srcs.insert({curr, pin_id, i});
                        dominated_pins.insert({curr, pin_id, i});

                        for (const Sink& sink: output_pins[i]) {
                            if (std::holds_alternative<CellPin>(sink)) {
                                work_list.push_back(std::get<0>(std::get<CellPin>(sink)));
                            }
                        }
                    }
                }
            }
        }

        std::set<CellPin> non_dominated_pins;
        for (const CellPin& src: srcs) {
            if (dominated_pins.count(src) == 0) {
                non_dominated_pins.insert(src);
            }
        }

        return non_dominated_pins;
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

                            // // Remove the previous pin from the list
                            // // It is no longer a leaf
                            // mandatorily_dominated_pins.erase(prev);
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

        return frontier;
    }

    // FIXME: This is not correct
    // We should use fixed point computation of dominance instead.
    std::set<CellPin> get_dominated_pins(
        CellPin src,
        CellPin dest
    ) const {
        // Defensive programming
        if (src == dest) {
            return { dest };
        }

        std::set<CellPin> ret;

        RTLIL::Cell* src_cell = std::get<0>(src);
        const dict<IdString, std::vector<std::vector<Sink>>>& cell_sink_map = this->sink_map.at(src_cell);

        for (const auto& [port_name, port_sink_lists]: cell_sink_map) {
            for (const std::vector<Sink>& sink_list: port_sink_lists) {
                for (const Sink& sink: sink_list) {
                    // Reject any sink that are constants or external port
                    if (std::holds_alternative<RTLIL::SigBit>(sink)) {
                        continue;
                    }

                    CellPin next_src = std::get<CellPin>(sink);
                    // If propagating to the destination, then the destination is found
                    if (next_src == dest) {
                        ret.insert(next_src);
                        continue;
                    }

                    // If the propagating target is non-combinatorial AND not
                    // the destination, do NOT propagate
                    RTLIL::Cell* sink_cell = std::get<0>(next_src);
                    if (RTLIL::builtin_ff_cell_types().count(sink_cell->type)) {
                        continue;
                    }

                    std::set<CellPin> reached_recur = get_dominated_pins(next_src, dest);
                    ret.insert(reached_recur.begin(), reached_recur.end());
                }
            }
        }

        // If the children reports destination cell was reached,
        // append the source pin to the return set
        //
        // Note that the returned set from children forms a one-to-one mapping
        // with the fact that the destination was reached
        if (!(ret.empty())) {
            ret.insert(src);
        }

        return ret;
    }
};

PRIVATE_NAMESPACE_END
