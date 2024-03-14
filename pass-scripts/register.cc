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

    bool groupable(
        FfInitVals& ff_init_vals,
        const SigMap& sigmap,
        RTLIL::Cell* dff,
        RTLIL::Cell* dff_other
    ) {
        // Only perform trivial matching
        // So if the signal input polarity is different, do not group
        FfData dff_data(&ff_init_vals, dff);
        FfData dff_other_data(&ff_init_vals, dff_other);

        if (dff_data.has_clk ^ dff_other_data.has_clk) {
            return false;
        } else if (dff_data.has_clk) {
            if ((dff_data.pol_clk != dff_other_data.pol_clk) ||
                    (sigmap(dff_data.sig_clk) != sigmap(dff_other_data.sig_clk))) {
                return false;
            }
        }

        if (dff_data.has_ce ^ dff_other_data.has_ce) {
            return false;
        } else if (dff_data.has_ce) {
            if ((dff_data.pol_ce != dff_other_data.pol_ce) ||
                    (sigmap(dff_data.sig_ce) != sigmap(dff_other_data.sig_ce))) {
                return false;
            }
        }

        if (dff_data.has_srst ^ dff_other_data.has_srst) {
            return false;
        } else if (dff_data.has_srst) {
            if ((dff_data.pol_srst != dff_other_data.pol_srst) ||
                    (sigmap(dff_data.sig_srst) != sigmap(dff_other_data.sig_srst))) {
                return false;
            }
        }

        // Both FF has the same has_srst and has_ce
        // Check they respect the same CE/SRST precedence
        if (dff_data.has_ce && dff_data.has_srst) {
            if (dff_data.ce_over_srst != dff_other_data.ce_over_srst) {
                return false;
            }
        }

        if (dff_data.has_arst ^ dff_other_data.has_arst) {
            return false;
        } else if (dff_data.has_arst) {
            if ((dff_data.pol_arst != dff_other_data.pol_arst) ||
                    (sigmap(dff_data.sig_arst) != sigmap(dff_other_data.sig_arst))) {
                return false;
            }
        }

        if (dff_data.has_aload ^ dff_other_data.has_aload) {
            return false;
        } else if (dff_data.has_aload) {
            if ((dff_data.pol_aload != dff_other_data.pol_aload) ||
                    (sigmap(dff_data.sig_aload) != sigmap(dff_other_data.sig_aload))) {
                return false;
            }
        }

        if (dff_data.has_sr ^ dff_other_data.has_sr) {
            return false;
        } else if (dff_data.has_sr) {
            if ((dff_data.pol_clr != dff_other_data.pol_clr) ||
                    (dff_data.pol_set != dff_other_data.pol_set) ||
                    (sigmap(dff_data.sig_clr) != sigmap(dff_other_data.sig_clr)) ||
                    (sigmap(dff_data.sig_set) != sigmap(dff_other_data.sig_set))) {
                return false;
            }
        }

        return true;
    }

    void replace_ff_groups(RTLIL::Module* module, const std::set<RTLIL::Cell*>& reg, FfInitVals& ff_init_vals) {
        // Figure out the type of high-level flip flop
        RTLIL::Cell* ff = *reg.begin();

        FfData ff_data(&ff_init_vals, ff);

        // Build the higher-level flip flops using the configs of the fine grain ff
        FfData fl_ff_data(ff_data);
        fl_ff_data.cell = nullptr;
        fl_ff_data.name = NEW_ID;
        // Set width & unselect fine gate
        fl_ff_data.width = 0;
        fl_ff_data.is_fine = false;
        
        // Simply collect and reassign D pins and Q pins
        std::vector<RTLIL::SigBit> d_pins;
        std::vector<RTLIL::SigBit> q_pins;
        std::vector<RTLIL::SigBit> clr_pins;
        std::vector<RTLIL::SigBit> set_pins;
        std::vector<RTLIL::State> val_arsts;
        std::vector<RTLIL::State> val_srsts;
        std::vector<RTLIL::State> val_inits;

        for (RTLIL::Cell* single_ff: reg) {
            FfData single_ff_data(&ff_init_vals, single_ff);
            const SigSpec& sig_d = single_ff_data.sig_d;
            const SigSpec& sig_q = single_ff_data.sig_q;
            d_pins.insert(d_pins.end(), sig_d.begin(), sig_d.end());
            q_pins.insert(q_pins.end(), sig_q.begin(), sig_q.end());

            const SigSpec& sig_clr = single_ff_data.sig_clr;
            const SigSpec& sig_set = single_ff_data.sig_set;
            clr_pins.insert(clr_pins.end(), sig_clr.begin(), sig_clr.end());
            set_pins.insert(set_pins.end(), sig_set.begin(), sig_set.end());

            val_arsts.insert(val_arsts.end(), single_ff_data.val_arst.bits.begin(), single_ff_data.val_arst.bits.end());
            val_srsts.insert(val_srsts.end(), single_ff_data.val_srst.bits.begin(), single_ff_data.val_srst.bits.end());
            val_inits.insert(val_inits.end(), single_ff_data.val_init.bits.begin(), single_ff_data.val_init.bits.end());
            fl_ff_data.width += GetSize(sig_d);
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
                for (size_t reg_idx = 0; reg_idx < registers.size(); ++reg_idx) {
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

        // Elevate DFF abstraction
        for (const std::set<RTLIL::Cell*>& reg: registers) {
            replace_ff_groups(top, reg, ff_init_vals);
        }
    }

} RegisterRecover;

PRIVATE_NAMESPACE_END
