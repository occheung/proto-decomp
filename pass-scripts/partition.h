#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/ff.h"
#include "graph.h"
#include "util.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


struct Partition {
    std::set<RTLIL::Cell*> cells;

    std::set<RTLIL::SigBit> external_inputs;
    std::map<CellPin, RTLIL::SigBit> external_outputs;

    std::set<ValidReadyProto> vr_srcs;
    std::set<ValidReadyProto> vr_sinks;

    const CircuitGraph circuit_graph;
    FfInitVals* ff_init_vals_ptr;

    Partition(
        std::set<ValidReadyProto> iface_sources,
        std::set<ValidReadyProto> iface_sinks,
        const CircuitGraph& circuit_graph,
        FfInitVals* ff_init_vals_ptr
    ): 
        vr_srcs(iface_sources),
        vr_sinks(iface_sinks),
        circuit_graph(circuit_graph),
        ff_init_vals_ptr(ff_init_vals_ptr)
    {
        // Identify module I/O
        // The partition should exclude the input cells but include the output cells
        std::set<RTLIL::Cell*> inputs;
        std::set<RTLIL::Cell*> outputs;

        // Process input interfaces
        // Input:  valid, data
        // Output: ready
        for (auto [valid_pin, ready_pin, data_ifaces]: iface_sources) {
            inputs.insert(std::get<0>(valid_pin));
            outputs.insert(std::get<0>(ready_pin));

            for (auto [data_src, data_sink]: data_ifaces) {
                inputs.insert(std::get<0>(data_src));
                this->cells.insert(std::get<0>(data_sink));
            }
        }

        // Process output interfaces
        // Input:  ready
        // Output: valid, data
        for (auto [valid_pin, ready_pin, data_ifaces]: iface_sinks) {
            outputs.insert(std::get<0>(valid_pin));
            inputs.insert(std::get<0>(ready_pin));

            for (auto [data_src, _data_sink]: data_ifaces) {
                outputs.insert(std::get<0>(data_src));
            }
        }

        // Initialize included cells
        // See definition above
        this->cells = outputs;

        auto trace_inputs = [&](
            std::set<RTLIL::Cell*> frontier_set
        ) -> std::set<RTLIL::Cell*> {
            std::set<RTLIL::Cell*> newly_found_cells;

            std::vector<RTLIL::Cell*> work_list(frontier_set.begin(), frontier_set.end());
            while (!work_list.empty()) {
                RTLIL::Cell* curr = work_list.back();
                work_list.pop_back();

                // Propagate
                for (auto [in_port, in_idx_vec]: this->circuit_graph.source_map.at(curr)) {
                    // If the current cell is a DFF, ignore CLK
                    if (RTLIL::builtin_ff_cell_types().count(curr->type) && in_port == "\\CLK") {
                        continue;
                    }
                    for (Source curr_src: in_idx_vec) {
                        // If the current cell is a DFF, and the current port is SRST
                        // Report error if SRST is not tied to an external input
                        if (RTLIL::builtin_ff_cell_types().count(curr->type) && in_port == "\\SRST") {
                            if (std::holds_alternative<CellPin>(curr_src)) {
                                log_error("DFF %s has a non-external reset input\n", log_id(curr));
                            }
                            // Otherwise, do not consider the SRST input as an external input
                            continue;
                        }

                        if (std::holds_alternative<RTLIL::SigBit>(curr_src)) {
                            // Identify if the bit is a constant or a wire
                            // A wire implies an external input connection
                            RTLIL::SigBit bit = std::get<RTLIL::SigBit>(curr_src);
                            if (bit.is_wire()) {
                                this->external_inputs.insert(bit);
                            }
                            continue;
                        }

                        // Input source is from another cell
                        else {
                            RTLIL::Cell* curr_src_cell = std::get<0>(std::get<CellPin>(curr_src));
                            // Skip propagation if
                            // - It clearly does not belong to the partition
                            // - It had already been processed
                            if (inputs.count(curr_src_cell) || this->cells.count(curr_src_cell)) {
                                continue;
                            }

                            work_list.push_back(curr_src_cell);
                            this->cells.insert(curr_src_cell);
                            newly_found_cells.insert(curr_src_cell);
                        }
                    }
                }
            }

            return newly_found_cells;
        };

        // Find external outputs
        auto trace_outputs = [&](
            std::set<RTLIL::Cell*> frontier_set
        ) -> std::set<RTLIL::Cell*> {
            std::set<RTLIL::Cell*> newly_found_cells;

            std::vector<RTLIL::Cell*> frontier(frontier_set.begin(), frontier_set.end());
            while (!frontier.empty()) {
                RTLIL::Cell* curr = frontier.back();
                frontier.pop_back();
                for (const auto& [port_out, out_idx_vec]: this->circuit_graph.sink_map.at(curr)) {
                    if (curr->output(port_out)) {
                        for (size_t i = 0; i < out_idx_vec.size(); ++i) {
                            CellPin src_pin = {curr, port_out, i};

                            std::vector<Sink> sink_vec = out_idx_vec.at(i);
                            for (Sink sink: sink_vec) {
                                if (std::holds_alternative<CellPin>(sink)) {
                                    CellPin sink_pin = std::get<CellPin>(sink);
                                    RTLIL::Cell* sink_cell = std::get<0>(sink_pin);

                                    if (this->cells.count(sink_cell) == 0) {
                                        frontier.push_back(sink_cell);
                                        // Add to cells
                                        this->cells.insert(sink_cell);
                                        newly_found_cells.insert(sink_cell);
                                    }
                                }
                                else {
                                    RTLIL::SigBit sink_ext_bit = std::get<RTLIL::SigBit>(sink);
                                    // Sink should not be constants
                                    if (!sink_ext_bit.is_wire()) {
                                        log_error("Found a constant sink\n");
                                    }

                                    // Add to external outputs
                                    this->external_outputs[src_pin] = sink_ext_bit;
                                }
                            }
                        }
                    }
                }
            }

            return newly_found_cells;
        };

        std::set<RTLIL::Cell*> newly_traversed_cells = outputs;

        while (!newly_traversed_cells.empty()) {
            newly_traversed_cells = trace_inputs(newly_traversed_cells);
            newly_traversed_cells = trace_outputs(newly_traversed_cells);
        }
    }

    void to_shakeflow(std::string filename) const {
        // Assign a canonical name to each valid/ready interface
        std::map<ValidReadyProto, std::string> vr_name_map;
        {
            int i = 0;
            for (const ValidReadyProto& proto: this->vr_srcs) {
                std::stringstream ss;
                ss << "vr" << i;
                vr_name_map[proto] = ss.str();
                ++i;
            }
            for (const ValidReadyProto& proto: this->vr_sinks) {
                std::stringstream ss;
                ss << "vr" << i;
                vr_name_map[proto] = ss.str();
                ++i;
            }
        }

        // Partition the network of cells into DFF and combinatorial cells
        std::set<RTLIL::Cell*> dffs;
        std::set<RTLIL::Cell*> combs;
        for (RTLIL::Cell* curr: this->cells) {
            if (RTLIL::builtin_ff_cell_types().count(curr->type)) {
                dffs.insert(curr);
            } else {
                combs.insert(curr);
            }
        }
        // Find the size of state variable
        // Order all DFF data pins
        std::vector<std::pair<RTLIL::Cell*, int>> dff_pin_order;
        for (RTLIL::Cell* dff: dffs) {
            FfData ff_data(this->ff_init_vals_ptr, dff);
            for (int i = 0; i < ff_data.width; ++i) {
                dff_pin_order.push_back({dff, i});
            }
        }

        std::ofstream f;
        f.open(filename + ".rs");

        auto sanitize = [](std::string in) {
            auto find_and_replace = [](
                std::string& s,
                std::string const& toReplace,
                std::string const& replaceWith
            ) {
                std::string buf;
                std::size_t pos = 0;
                std::size_t prevPos;

                // Reserves rough estimate of final size of string.
                buf.reserve(s.size());

                while (true) {
                    prevPos = pos;
                    pos = s.find(toReplace, pos);
                    if (pos == std::string::npos)
                        break;
                    buf.append(s, prevPos, pos - prevPos);
                    buf += replaceWith;
                    pos += toReplace.size();
                }

                buf.append(s, prevPos, s.size() - prevPos);
                s.swap(buf);
            };

            find_and_replace(in, "\\", "_");
            find_and_replace(in, ".", "_");
            find_and_replace(in, ":", "_");
            find_and_replace(in, "$", "_");
            return in;
        };

        auto cell_pin_to_name = [&](const CellPin& pin) {
            const auto [cell, port, idx] = pin;
            std::stringstream s;
            s << sanitize(cell->name.str());
            s << "_";
            s << sanitize(port.str());
            s << "_";
            s << idx;

            return s.str();
        };

        auto sigbit_wire_to_name = [&](const SigBit& bit) {
            std::stringstream s;
            s << sanitize(bit.wire->name.str());
            s << "_" << bit.offset;

            return s.str();
        };

        auto insert_header = [&]() {
            f << "#![allow(non_snake_case)]";
            f << "use shakeflow::*;";
            f << "use shakeflow_std::*;";
        };

        auto generate_interfaces = [&]() {
            f << "#[derive(Debug, Interface)]";
            f << "pub struct IC {";

            for (const auto& proto: this->vr_srcs) {
                const auto& [_valid_info, _ready_info, data_iface] = proto;
                // Compute data width
                // A data pin may have multiple sinks
                int dw = 0;
                std::set<CellPin> visited_data_pins;
                for (const auto& [data_src, _data_sink]: data_iface) {
                    if (visited_data_pins.count(data_src)) {
                        continue;
                    }
                    visited_data_pins.insert(data_src);
                    ++dw;
                }

                f << vr_name_map.at(proto) << ": VrChannel<Bits<U<" << dw << ">>>,";
            }

            // If there are external inputs, append a ext_in field
            if (!this->external_inputs.empty()) {
                f << "ext_in: UniChannel<Bits<U<" << this->external_inputs.size() << ">>>,";
            }

            f << "}";

            f << "#[derive(Debug, Interface)]";
            f << "pub struct OC {";

            for (const auto& proto: this->vr_sinks) {
                const auto& [_valid_info, _ready_info, data_iface] = proto;
                // Compute data width
                // A data pin may have multiple sinks
                int dw = 0;
                std::set<CellPin> visited_data_pins;
                for (const auto& [data_src, _data_sink]: data_iface) {
                    if (visited_data_pins.count(data_src)) {
                        continue;
                    }
                    visited_data_pins.insert(data_src);
                    ++dw;
                }

                f << vr_name_map.at(proto) << ": VrChannel<Bits<U<" << dw << ">>>,";
            }

            // If there are external inputs, append a ext_out field
            if (!this->external_outputs.empty()) {
                f << "ext_out: UniChannel<Bits<U<" << this->external_outputs.size() << ">>>,";
            }

            f << "}";
        };

        auto begin_main_fsm = [&]() {
            f << "fn main_fsm<'a>";
            f << "(ic_fwd: Expr<'a, ICFwd>, oc_bwd: Expr<'a, OCBwd>, state: Expr<'a, Array<bool, U<";
            f << dff_pin_order.size();
            f << ">>>)";
            f << "-> (Expr<'a, OCFwd>, Expr<'a, ICBwd>, Expr<'a, Array<bool, U<";
            f << dff_pin_order.size();
            f << ">>>) {";
        };

        auto end_main_fsm = [&]() {
            // Recompose interfaces
            for (const ValidReadyProto& proto: this->vr_srcs) {
                const CellPin& ready_pin = std::get<1>(proto);
                f << "let " << vr_name_map.at(proto) << " = Expr::<Ready>::new(" << cell_pin_to_name(ready_pin) << ");";
            }
            for (const ValidReadyProto& proto: this->vr_sinks) {
                const auto& [valid_pin, _ready_pin, data_iface] = proto;
                f << "let " << vr_name_map.at(proto) << " = Expr::<Valid<_>>::new(" << cell_pin_to_name(valid_pin) << ", [";
                // Arbitrarily format data, compute data width
                // A data pin may have multiple sinks
                std::set<CellPin> visited_data_pins;
                for (const auto& [data_src, _data_sink]: data_iface) {
                    if (visited_data_pins.count(data_src)) {
                        continue;
                    }

                    f << cell_pin_to_name(data_src) << ", ";
                    visited_data_pins.insert(data_src);
                }
                f << "].into());";
            }

            f << "(";
            f << "OCFwdProj {";
            for (const ValidReadyProto& proto: this->vr_sinks) {
                f << vr_name_map.at(proto) << ", ";
            }
            // If there were external outputs, we need to consider the ext_out field
            // See link_external_outputs for ext_out generation
            if (!this->external_outputs.empty()) {
                f << "ext_out,";
            }
            f << "}.into(), ";

            f << "ICBwdProj {";
            for (const ValidReadyProto& proto: this->vr_srcs) {
                f << vr_name_map.at(proto) << ", ";
            }
            // If there were an external input, we need to consider the ext_in field
            // However, since ext_in is unidirectional, ext_in is simply an empty type
            if (!this->external_inputs.empty()) {
                f << "ext_in: ().into(),";
            }
            f << "}.into(), ";

            f << "[";
            for (size_t i = 0; i < dff_pin_order.size(); ++i) {
                f << "new_state" << i << ", ";
            }
            f << "].into()";
            f << ")";
            f << "}";
        };

        auto compose = [&]() {
            f << "pub fn m() -> Module<IC, OC> {";
            f << "composite::<IC, OC, _>(\"m\", None, None, |src, k| {";

            // Express initial state as a boolean array
            f << "let init_state = [";
            for (auto [dff, idx]: dff_pin_order) {
                FfData ff_data(this->ff_init_vals_ptr, dff);
                // Shakeflow automatically generate reset logic with the reset signal
                // There are no way to temper the reset
                
                // FIXME: Some FF might not have a reset pin
                // Automatically assign as false in these situations
                if (ff_data.has_srst) {
                    if (ff_data.val_srst[idx] == RTLIL::State::S1) {
                        f << "true, ";
                    } else {
                        f << "false, ";
                    }
                } else {
                    f << "false, ";
                }
            }
            f << "];";
            f << "src.fsm::<_, OC, _>(k, None, init_state.into(), main_fsm)";
            f << "}).build()";
            f << "}";
        };

        auto write_input = [&] {
            std::set<CellPin> processed_pins;

            // valid/ready sources contribute valid and data input
            for (const auto& proto: this->vr_srcs) {
                std::string vr_iface_name = vr_name_map.at(proto);
                const auto& [valid_pin, _ready_pin, data_iface] = proto;

                // Valid processing
                f << "let " << cell_pin_to_name(valid_pin) << " = ic_fwd." << vr_iface_name << ".valid;";
                processed_pins.insert(valid_pin);

                // Data processing
                std::set<CellPin> processed_data_pin;
                // Arbitrarily designate data pin order
                int pin_idx = 0;
                for (const auto& [data_src, _data_sink]: data_iface) {
                    if (processed_data_pin.count(data_src)) {
                        continue;
                    }
                    f << "let " << cell_pin_to_name(data_src) << " = ic_fwd." << vr_iface_name << ".inner[" << pin_idx << "];";
                    ++pin_idx;
                    
                    processed_data_pin.insert(data_src);
                    processed_pins.insert(data_src);
                }
            }

            // valid/ready sinks contribute ready
            for (const auto& proto: this->vr_sinks) {
                std::string vr_iface_name = vr_name_map.at(proto);
                const CellPin& ready_pin = std::get<1>(proto);
                f << "let " << cell_pin_to_name(ready_pin) << " = oc_bwd." << vr_iface_name << ".ready;";
                processed_pins.insert(ready_pin);
            }

            // External input. Self-explanatory
            // Arbitrarily designate pin order
            {
                int pin_idx = 0;
                for (RTLIL::SigBit ext_in: this->external_inputs) {
                    if (!ext_in.is_wire()) {
                        log_error("External input set collected a constant\n");
                    }

                    f << "let " << sigbit_wire_to_name(ext_in) << " = ic_fwd.ext_in[" << pin_idx << "];";
                    ++pin_idx;
                }
            }

            return processed_pins;
        };

        auto write_ff_output = [&]() {
            int i = 0;
            std::set<CellPin> processed_pins;
            for (const auto& [dff, idx]: dff_pin_order) {
                f << "let " << cell_pin_to_name({dff, "\\Q", idx}) << " = state[" << i << "];";
                processed_pins.insert({dff, "\\Q", idx});
                ++i;
            }

            return processed_pins;
        };

        std::map<RTLIL::Cell*, std::set<Source>> dependency_graph;
        // Populate
        for (RTLIL::Cell* cell: combs) {
            dependency_graph[cell] = {};
            for (const auto& [port, src_indices]: circuit_graph.source_map.at(cell)) {
                if (cell->input(port)) {
                    for (Source src: src_indices) {
                        // Ignore constants & external inputs
                        // These inputs has no dependencies
                        if (std::holds_alternative<RTLIL::SigBit>(src)) {
                            continue;
                        }
                        dependency_graph[cell].insert(src);
                    }
                }
            }
        }

        auto get_cells_with_resolved_input = [&](
            std::set<CellPin> processed_outputs
        ) -> std::set<RTLIL::Cell*> {
            std::set<RTLIL::Cell*> ret;
            for (auto& [cell, srcs]: dependency_graph) {
                for (CellPin proc_out: processed_outputs) {
                    // Wrap into a source
                    Source src = proc_out;
                    srcs.erase(src);
                }
                if (srcs.empty()) {
                    ret.insert(cell);
                }
            }

            // Remove returning cells from the graph
            for (RTLIL::Cell* ret_cell: ret) {
                dependency_graph.erase(ret_cell);
            }

            return ret;
        };

        auto src_to_str = [&](
            Source src
        ) -> std::string {
            if (std::holds_alternative<RTLIL::SigBit>(src)) {
                RTLIL::SigBit bit = std::get<RTLIL::SigBit>(src);
                if (bit.is_wire()) {
                    return sigbit_wire_to_name(bit);
                } else {
                    return bit.data == RTLIL::State::S1? "true.into()" : "false.into()";
                }
            } else {
                return cell_pin_to_name(std::get<CellPin>(src));
            }
        };

        auto write_cells = [&](
            std::set<RTLIL::Cell*> cells
        ) -> std::set<CellPin> {
            std::set<CellPin> processed_pins;
            for (RTLIL::Cell* curr: cells) {
                if (curr->type == "$_NOT_") {
                    Source src_a = circuit_graph.source_map.at(curr).at("\\A").at(0);
                    std::string arg_a = src_to_str(src_a);

                    f << "let " << cell_pin_to_name({curr, "\\Y", 0}) << " = !" << arg_a << ";";
                    processed_pins.insert({curr, "\\Y", 0});
                } else if (curr->type == "$_AND_") {
                    Source src_a = circuit_graph.source_map.at(curr).at("\\A").at(0);
                    Source src_b = circuit_graph.source_map.at(curr).at("\\B").at(0);

                    std::string arg_a = src_to_str(src_a);
                    std::string arg_b = src_to_str(src_b);

                    f << "let " << cell_pin_to_name({curr, "\\Y", 0}) << " = " << arg_a << " & " << arg_b << ";";
                    processed_pins.insert({curr, "\\Y", 0});
                } else if (curr->type == "$_OR_") {
                    Source src_a = circuit_graph.source_map.at(curr).at("\\A").at(0);
                    Source src_b = circuit_graph.source_map.at(curr).at("\\B").at(0);

                    std::string arg_a = src_to_str(src_a);
                    std::string arg_b = src_to_str(src_b);

                    f << "let " << cell_pin_to_name({curr, "\\Y", 0}) << " = " << arg_a << " | " << arg_b << ";";
                    processed_pins.insert({curr, "\\Y", 0});
                } else if (curr->type == "$_XOR_") {
                    Source src_a = circuit_graph.source_map.at(curr).at("\\A").at(0);
                    Source src_b = circuit_graph.source_map.at(curr).at("\\B").at(0);

                    std::string arg_a = src_to_str(src_a);
                    std::string arg_b = src_to_str(src_b);

                    f << "let " << cell_pin_to_name({curr, "\\Y", 0}) << " = " << arg_a << " ^ " << arg_b << ";";
                    processed_pins.insert({curr, "\\Y", 0});
                } else if (curr->type == "$_MUX_") {
                    Source src_a = circuit_graph.source_map.at(curr).at("\\A").at(0);
                    Source src_b = circuit_graph.source_map.at(curr).at("\\B").at(0);
                    Source src_s = circuit_graph.source_map.at(curr).at("\\S").at(0);

                    std::string arg_a = src_to_str(src_a);
                    std::string arg_b = src_to_str(src_b);
                    std::string arg_s = src_to_str(src_s);

                    f << "let " << cell_pin_to_name({curr, "\\Y", 0}) << " = select!( " << arg_s << " => " << arg_b << ", default => " << arg_a << ",);";
                    processed_pins.insert({curr, "\\Y", 0});
                } else {
                    log_error("Found unsupported logic type %s from cell %s\n", curr->type.c_str(), log_id(curr));
                }
            }

            return processed_pins;
        };

        // DFF processing: By linking up the state variables
        auto link_state_variable = [&]() {
            int state_idx = 0;
            for (auto [dff, idx]: dff_pin_order) {
                FfData ff_data(this->ff_init_vals_ptr, dff);

                f << "let new_state" << state_idx << " = ";
                if (ff_data.has_ce) {
                    f << "select! (";
                    if (ff_data.pol_ce) {
                        f << "!";
                    }
                    f << src_to_str(circuit_graph.source_map.at(dff).at("\\EN").at(0)) << " => state[" << state_idx << "], ";
                    f << "default => ";
                }
                f << src_to_str(circuit_graph.source_map.at(dff).at("\\D").at(0));

                if (ff_data.has_ce) {
                    f << ",)";
                }
                f << ";";

                ++state_idx;
            }
        };

        // Recompose the external outputs
        auto link_external_outputs = [&]() {
            if (!this->external_outputs.empty()) {
                std::stringstream s;
                s << "let ext_out = [ ";
                for (const auto& [src_pin, extern_out]: this->external_outputs) {
                    f << "let " << sigbit_wire_to_name(extern_out) << " = " << cell_pin_to_name(src_pin) << ";";
                    s << sigbit_wire_to_name(extern_out) << ", ";
                }
                s << "].into();";
                f << s.str();
            }
        };

        insert_header();
        generate_interfaces();
        begin_main_fsm();

        std::set<CellPin> processed_pins;
        {
            std::set<CellPin> processed_srcs = write_input();
            processed_pins.insert(processed_srcs.begin(), processed_srcs.end());
        }
        {
            std::set<CellPin> processed_srcs = write_ff_output();
            processed_pins.insert(processed_srcs.begin(), processed_srcs.end());
        }

        while (!processed_pins.empty()) {
            std::set<RTLIL::Cell*> available_cells = get_cells_with_resolved_input(processed_pins);
            processed_pins.clear();

            processed_pins = write_cells(available_cells);
        }
        link_state_variable();
        link_external_outputs();

        end_main_fsm();
        compose();
    }
};


PRIVATE_NAMESPACE_END
