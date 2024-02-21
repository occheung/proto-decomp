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
        // ostringstream oss;
        for (const auto& [output_cell, out_port_dict]: this->sink_map) {
            log("From cell %s\n", output_cell->name.c_str());
            // oss << output_cell->name << "\n";
            for (const auto& [output_port, sink_lists]: out_port_dict) {
                for (int port_idx = 0; port_idx < sink_lists.size(); ++port_idx) {
                    const auto& sink_list = sink_lists[port_idx];
                    for (const Sink& sink: sink_list) {
                        if (std::holds_alternative<CellPin>(sink)) {
                            const auto& [sink_cell, sink_port, sink_idx] = std::get<CellPin>(sink);
                            log("%s[%d] -> %s.%s[%d]\n", output_port.c_str(), port_idx, sink_cell->name.c_str(), sink_port.c_str(), sink_idx);
                        } else {
                            const RTLIL::SigBit& sink_bit = std::get<RTLIL::SigBit>(sink);
                            log("%s[%d] -> OUTPUT:%d[%d]\n", output_port.c_str(), port_idx, sink_bit.wire->port_id, sink_bit.offset);
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

        RTLIL::Cell* src_cell = std::get<0>(src);

        // If the destination is an output, check if dest is the output of src_cell
        const auto& [dest_cell, dest_port, dest_pin] = dest;
        if (dest_cell->output(dest_port)) {
            if (src_cell == dest_cell) {
                return { src_cell };
            }
        }

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
                        ret.insert(std::get<0>(next_src));
                        continue;
                    }

                    // If the propagating target is non-combinatorial AND not
                    // the destination, do NOT propagate
                    RTLIL::Cell* sink_cell = std::get<0>(next_src);
                    if (RTLIL::builtin_ff_cell_types().count(sink_cell->type)) {
                        continue;
                    }

                    std::set<RTLIL::Cell*> reached_recur = get_intermediate_comb_cells(next_src, dest);
                    ret.insert(reached_recur.begin(), reached_recur.end());
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
