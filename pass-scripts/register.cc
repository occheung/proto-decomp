#include "kernel/yosys.h"
#include "kernel/rtlil.h"
#include "kernel/sigtools.h"
#include "kernel/ffinit.h"
#include "kernel/ff.h"
#include "graph.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct RegisterRecover : public Pass {
    RegisterRecover() : Pass("register") { }

    bool groupable(FfInitVals& ff_init_vals, const SigMap& sigmap, RTLIL::Cell* dff, RTLIL::Cell* dff_other) {
        // Only DFFs of identical types can be grouped up together
        if (dff->type != dff_other->type) {
            return false;
        }

        FfData dff_data(&ff_init_vals, dff);
        FfData dff_other_data(&ff_init_vals, dff_other);
        if (dff_data.has_clk) {
            if (sigmap(dff_data.sig_clk) != sigmap(dff_other_data.sig_clk)) {
                return false;
            }
        }

        if (dff_data.has_ce) {
            if (sigmap(dff_data.sig_ce) != sigmap(dff_other_data.sig_ce)) {
                return false;
            }
        }

        if (dff_data.has_arst) {
            if (sigmap(dff_data.sig_arst) != sigmap(dff_other_data.sig_arst)) {
                return false;
            }
        }

        if (dff_data.has_srst) {
            if (sigmap(dff_data.sig_srst) != sigmap(dff_other_data.sig_srst)) {
                return false;
            }
        }

        if (dff_data.has_aload) {
            if (sigmap(dff_data.sig_aload) != sigmap(dff_other_data.sig_aload)) {
                return false;
            }
        }

        if (dff_data.has_sr) {
            if (sigmap(dff_data.sig_clr) != sigmap(dff_other_data.sig_clr)) {
                return false;
            }
            if (sigmap(dff_data.sig_set) != sigmap(dff_other_data.sig_set)) {
                return false;
            }
        }

        // TODO: What if the FF is too primitive to have any of these features
        return true;
    }

    void replace_ff_groups(RTLIL::Module* module, const std::set<RTLIL::Cell*>& reg, FfInitVals& ff_init_vals) {
        // Figure out the type of high-level flip flop
        RTLIL::Cell* ff = *reg.begin();

        FfData ff_data(&ff_init_vals, ff);
        IdString dff_type;

        dict<IdString, RTLIL::SigSpec> dff_inputs;

        // if (ff_data.has_clk) {
        //     if (ff_data.has_ce) {
        //         dff_inputs["EN"] = 
        //         if (ff_data.has_arst) {
        //             dff_type = "$adffe";
        //         } else if (ff_data.has_aload) {
        //             dff_type = "$aldffe";
        //         } else if (ff_data.has_sr) {
        //             dff_type = "$dffsre";
        //         } else if (ff_data.has_srst) {
        //             // FIXME: Check polarity priority
        //             dff_type = "$sdffe";
        //         } else {
        //             dff_type = "$dffe";
        //         }
        //     } else {
        //         if (ff_data.has_arst) {
        //             dff_type = "$adff";
        //         } else if (ff_data.has_aload) {
        //             dff_type = "$aldff";
        //         } else if (ff_data.has_sr) {
        //             dff_type = "$dffsr";
        //         } else if (ff_data.has_srst) {
        //             dff_type = "$sdff";
        //         } else {
        //             dff_type = "$dff";
        //         }
        //     }
        // } else {
        //     // Ignore non flip flops types
        //     return;
        // }

        std::ostringstream id_name;
        id_name << ff_data.name.c_str() << "_duped";

        FfData fl_ff_data(ff_data);
        fl_ff_data.cell = nullptr;
        fl_ff_data.name = id_name.str();
        // Set width & unselect fine gate
        fl_ff_data.width = reg.size();
        fl_ff_data.is_fine = false;
        
        // Simply collect and reassign D pins and Q pins
        std::vector<RTLIL::SigBit> d_pins;
        std::vector<RTLIL::SigBit> q_pins;
        std::vector<RTLIL::State> val_arsts;
        std::vector<RTLIL::State> val_srsts;
        std::vector<RTLIL::State> val_inits;

        for (RTLIL::Cell* single_ff: reg) {
            FfData single_ff_data(&ff_init_vals, single_ff);
            const SigSpec& sig_d = single_ff_data.sig_d;
            const SigSpec& sig_q = single_ff_data.sig_q;
            d_pins.insert(d_pins.end(), sig_d.begin(), sig_d.end());
            q_pins.insert(q_pins.end(), sig_q.begin(), sig_q.end());

            // FIXME: async load & init values
            val_arsts.insert(val_arsts.end(), single_ff_data.val_arst.bits.begin(), single_ff_data.val_arst.bits.end());
            val_srsts.insert(val_srsts.end(), single_ff_data.val_srst.bits.begin(), single_ff_data.val_srst.bits.end());
            val_inits.insert(val_inits.end(), single_ff_data.val_init.bits.begin(), single_ff_data.val_init.bits.end());
        }
        fl_ff_data.sig_d = RTLIL::SigSpec(d_pins);
        fl_ff_data.sig_q = RTLIL::SigSpec(q_pins);
        fl_ff_data.val_arst = RTLIL::Const(val_arsts);
        fl_ff_data.val_srst = RTLIL::Const(val_srsts);
        fl_ff_data.val_init = RTLIL::Const(val_inits);

        fl_ff_data.emit();

        // Purge the fine-grained flip flops that made up the register
        for (RTLIL::Cell* single_ff: reg) {
            module->remove(single_ff);
        }
    }

    void execute(vector<string>, Design* design) override {
        RTLIL::Module* top = design->top_module();

        if (top == nullptr) {
            log_error("No top module found!\n");
            return;
        }

        const SigMap sigmap(top);

        CircuitGraph circuit_graph(sigmap, top);

        FfInitVals ff_init_vals(&sigmap, top);

        // Registers set
        // Similarity is transitive
        std::vector<std::set<RTLIL::Cell*>> registers;
        for (RTLIL::Cell* ff_cell: top->cells()) {
            if (RTLIL::builtin_ff_cell_types().count(ff_cell->type)) {
                // Either add this flip-flop to some existing register
                bool matched = false;
                for (int reg_idx = 0; reg_idx < registers.size(); ++reg_idx) {
                    if (this->groupable(ff_init_vals, sigmap, *registers[reg_idx].begin(), ff_cell)) {
                        matched = true;
                        registers[reg_idx].insert(ff_cell);
                        break;
                    }
                }
                // Or group it into a separate register on its own
                if (!matched) {
                    registers.push_back({ff_cell});
                }
            }
        }

        for (const std::set<RTLIL::Cell*>& reg: registers) {
            log("Register FFs:\n");
            for (RTLIL::Cell* ff: reg) {
                log("%s\n", ff->name.c_str());
            }
        }

        // Elevate DFF abstraction
        for (const std::set<RTLIL::Cell*>& reg: registers) {
            replace_ff_groups(top, reg, ff_init_vals);
        }
    }

} RegisterRecover;

PRIVATE_NAMESPACE_END
