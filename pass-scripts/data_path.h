#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/ff.h"
#include "graph.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct DataPath {
    dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> data_sink_map;
    dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> data_source_map;

    DataPath(const SigMap& sigmap, RTLIL::Module* module) {
        CircuitGraph circuit_graph(sigmap, module);

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
                        if (!circuit_graph.get_intermediate_comb_cells(
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

        // log("Buffer registers:\n");
        // for (RTLIL::Cell* buf_reg: buffers) {
        //     log("%s\n", log_id(buf_reg));
        // }
        // log("\n");

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

                    if (!circuit_graph.get_intermediate_comb_cells(
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

        // log("Memories:\n");
        // for (RTLIL::Cell* mem: memories) {
        //     log("%s\n", log_id(mem));
        // }
        // log("\n");

        // Classify whatever between buffers or memories are data components
        // Starts with buffers or memories
        std::set<RTLIL::Cell*> data_components;
        std::set<RTLIL::Cell*> buf_or_mems;
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

                for (int i = 0; i < from_width; ++i) {
                    for (int j = 0; j < to_width; ++j) {
                        std::set<RTLIL::Cell*> reachables_inters = circuit_graph.get_intermediate_comb_cells(
                            {elm_from, "\\Q", i},
                            {elm_to, "\\D", j}
                        );
                        data_components.insert(reachables_inters.begin(), reachables_inters.end());
                    }
                }
            }
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
        dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> data_sink_map;
        dict<RTLIL::Cell*, std::set<RTLIL::Cell*>> data_source_map;

        // Use the already established circuit graph
        {
            for (const auto& [src_cell, port_map]: circuit_graph.sink_map) {
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
                                if (data_sink_map.count(src_cell) == 0) {
                                    data_sink_map[src_cell] = {};
                                }
                                data_sink_map[src_cell].insert(sink_cell);

                                if (data_source_map.count(sink_cell) == 0) {
                                    data_source_map[sink_cell] = {};
                                }
                                data_source_map[sink_cell].insert(src_cell);
                            }
                        }
                    }
                }
            }
        }
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
