#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "graph.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN


#ifndef UTIL_H
#define UTIL_H


struct Handshake {
    // Record the handshake bits and the FFs that back the bits
    std::map<CellPin, std::set<RTLIL::Cell*>> info;

    Handshake(
        CellPin ctrl0,
        std::set<RTLIL::Cell*> ffs0,
        CellPin ctrl1,
        std::set<RTLIL::Cell*> ffs1
    ) {
        if (ctrl0 == ctrl1) {
            log_error("Handshake entry has the same handshake control bits on both directions\n");
        }
        // this->info = {
        //     {ctrl0, ffs0},
        //     {ctrl1, ffs1}
        // };
        this->info[ctrl0] = ffs0;
        this->info[ctrl1] = ffs1;
    }

    Handshake() = delete;
    Handshake(const Handshake& other) = default;

    bool has_same_handshake_bits(const Handshake& other) const {
        auto this_info_it = this->info.begin();
        const auto& [this_ctrl0, _this_ffs0] = *this_info_it;
        std::advance(this_info_it, 1);
        const auto& [this_ctrl1, _this_ffs1] = *this_info_it;

        auto other_info_it = other.info.begin();
        const auto& [other_ctrl0, _other_ffs0] = *other_info_it;
        std::advance(other_info_it, 1);
        const auto& [other_ctrl1, _other_ffs1] = *other_info_it;

        if ((this_ctrl0 == other_ctrl0) && (this_ctrl1 == other_ctrl1)) {
            return true;
        }

        // Handshake bits may appear in different order
        if ((this_ctrl0 == other_ctrl1) && (this_ctrl1 == other_ctrl0)) {
            return true;
        }

        return false;
    }

    // void insert_backing_ff(CellPin ctrl, RTLIL::Cell* new_backing_ff) {
    //     this->info.at(ctrl).insert(new_backing_ff);
    // }

    void insert_backing_ff(std::set<RTLIL::Cell*> new_backing_ffs) {
        // Check which interface has a backing of some subset of the new set
        bool modules_backs_ctrl[2] = {true, true};
        CellPin ctrl_pins[2];
        int i = 0;
        for (auto [ctrl_pin, backed_ffs_ctrl]: this->info) {
            ctrl_pins[i] = ctrl_pin;
            for (RTLIL::Cell* backed_ff_ctrl: backed_ffs_ctrl) {
                if (new_backing_ffs.count(backed_ff_ctrl) == 0) {
                    modules_backs_ctrl[i] = false;
                }
            }
            ++i;
        }

        if (modules_backs_ctrl[0] && modules_backs_ctrl[1]) {
            log_error("Attempted to update backing with a set that intersects both interfaces\n");
        }

        if (modules_backs_ctrl[0]) {
            this->info[ctrl_pins[0]] = new_backing_ffs;
            return;
        }

        if (modules_backs_ctrl[1]) {
            this->info[ctrl_pins[1]] = new_backing_ffs;
            return;
        }

        log_error("New FFs backing does not share subset relationship with the original backings\n");
    }

    std::pair<std::pair<CellPin, RTLIL::Cell*>, std::pair<CellPin, RTLIL::Cell*>> decompose_single() const {
        auto info_it = this->info.begin();
        auto [pin0, dffs0] = *info_it;
        std::advance(info_it, 1);
        auto [pin1, dffs1] = *info_it;

        if ((dffs0.size() != 1) || (dffs1.size() != 1)) {
            log_error("Attempted to decompose a handshake interface with multiple backing DFFs\n");
        }

        return {
            {pin0, *dffs0.begin()},
            {pin1, *dffs1.begin()}
        };
    }

    std::pair<std::pair<CellPin, std::set<RTLIL::Cell*>>, std::pair<CellPin, std::set<RTLIL::Cell*>>> decompose() const {
        auto info_it = this->info.begin();
        auto [pin0, dffs0] = *info_it;
        std::advance(info_it, 1);
        auto [pin1, dffs1] = *info_it;

        return {
            {pin0, dffs0},
            {pin1, dffs1}
        };
    }

    // It checks if the handshake interface is backed by a subset of the args
    // If so, return backing FFs of the side that is not backed
    // If not, return an empty set to signify the handhsake is not backed
    std::set<RTLIL::Cell*> get_opposite_backing_if_backed(
        std::set<RTLIL::Cell*> ffs_in_module
    ) const {
        bool modules_backs_ctrl[2] = {true, true};
        std::set<RTLIL::Cell*> backing_ffs[2];
        int i = 0;
        for (auto [ctrl_pin, backed_ffs_ctrl]: this->info) {
            backing_ffs[i] = backed_ffs_ctrl;

            for (RTLIL::Cell* backed_ff_ctrl: backed_ffs_ctrl) {
                if (ffs_in_module.count(backed_ff_ctrl) == 0) {
                    modules_backs_ctrl[i] = false;
                }
            }

            ++i;
        }

        // Sanity
        if (modules_backs_ctrl[0] && modules_backs_ctrl[1]) {
            log_error("A valid ready interface is doubly-backed by a module\n");
        }

        if (modules_backs_ctrl[0]) {
            return backing_ffs[1];
        }

        if (modules_backs_ctrl[1]) {
            return backing_ffs[0];
        }

        return {};
    }

    std::set<RTLIL::Cell*> get_opposite_backing(
        std::set<RTLIL::Cell*> this_ff_backing
    ) const {
        std::set<RTLIL::Cell*> ffs[2];
        int i = 0;
        for (const auto& [_ctrl_pin, ff_backing]: this->info) {
            ffs[i] = ff_backing;
            ++i;
        }

        if (ffs[0] == this_ff_backing) {
            return ffs[1];
        }

        if (ffs[1] == this_ff_backing) {
            return ffs[0];
        }

        log_error("Neither interface of the handshake were backed by this set of FFs.\n");
        return {};
    }

    std::set<RTLIL::Cell*> get_backing_ffs(
        CellPin ctrl_pin
    ) const {
        auto [if0, if1] = this->decompose();
        auto [ctrl0, ffs0] = if0;
        auto [ctrl1, ffs1] = if1;
        if (ctrl0 == ctrl_pin) {
            return ffs0;
        } else if (ctrl1 == ctrl_pin) {
            return ffs1;
        }

        log_error("Attempted to find the backing FFs of control pins not part the handshake\n");
        return {};
    }

    CellPin get_opposite_ctrl(
        CellPin ctrl_pin
    ) const {
        auto [if0, if1] = this->decompose();
        auto [ctrl0, _ffs0] = if0;
        auto [ctrl1, _ffs1] = if1;
        if (ctrl0 == ctrl_pin) {
            return ctrl1;
        } else if (ctrl1 == ctrl_pin) {
            return ctrl0;
        }

        log_error("Attempted to find the opposite control pins that is not part the handshake\n");
        return {};
    }

    bool operator ==(const Handshake& other) const {
        return this->info == other.info;
    }

    bool operator <(const Handshake& other) const {
        {
            auto this_it = this->info.begin();
            auto other_it = other.info.begin();
            for (; (this_it != this->info.end()) && (other_it != other.info.end()); ++this_it, ++other_it) {
                auto [this_pin, _this_dffs] = *this_it;
                auto [other_pin, _other_dffs] = *other_it;
                if (this_pin != other_pin) {
                    return this_pin < other_pin;
                }
            }
        }

        // The execution flow reaches here iff the pin set are identical
        // Compare the backing FF instead
        {
            auto this_it = this->info.begin();
            auto other_it = other.info.begin();
            for (; (this_it != this->info.end()) && (other_it != other.info.end()); ++this_it, ++other_it) {
                auto [_this_pin, this_dffs] = *this_it;
                auto [_other_pin, other_dffs] = *other_it;
                if (this_dffs != other_dffs) {
                    return this_dffs < other_dffs;
                }
            }
        }
        return false;
    }

    void write_log() const {
        auto [handshake_src, handshake_sink] = this->decompose();
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

    void write_ctrl_pins() const {
        auto [handshake_src, handshake_sink] = this->decompose();
        auto [eligible_src, _dffe_srcs] = handshake_src;
        auto [eligible_sink, _dffe_sinks] = handshake_sink;

        log("Ctrl src: Cell %s Port %s\n",
            std::get<0>(eligible_src)->name.c_str(), std::get<1>(eligible_src).c_str()
        );
        // log("Src FFs:\n");
        // for (RTLIL::Cell* dffe_src: dffe_srcs) {
        //     log("%s\n", log_id(dffe_src));
        // }
        log("Ctrl sink: Cell %s Port %s\n",
            std::get<0>(eligible_sink)->name.c_str(), std::get<1>(eligible_sink).c_str()
        );
        // log("Sink FFs:\n");
        // for (RTLIL::Cell* dffe_sink: dffe_sinks) {
        //     log("%s\n", log_id(dffe_sink));
        // }
        // log("\n");
    }
};


struct ConnectedModules {
    std::set<std::set<RTLIL::Cell*>> inner;

    ConnectedModules() = delete;

    ConnectedModules(
        std::set<RTLIL::Cell*> dff0,
        std::set<RTLIL::Cell*> dff1
    ) {
        this->inner.insert(dff0);
        this->inner.insert(dff1);
    }

    ConnectedModules(const Handshake& handshake) {
        auto [if0, if1] = handshake.decompose();
        std::set<RTLIL::Cell*> dff0 = std::get<1>(if0);
        std::set<RTLIL::Cell*> dff1 = std::get<1>(if1);
        this->inner = {dff0, dff1};
    }

    bool operator <(const ConnectedModules& other) const {
        return this->inner < other.inner;
    }

    std::pair<std::set<RTLIL::Cell*>, std::set<RTLIL::Cell*>> decompose() const {
        auto inner_it = this->inner.begin();
        std::set<RTLIL::Cell*> dff0 = *inner_it;
        std::advance(inner_it, 1);
        std::set<RTLIL::Cell*> dff1 = *inner_it;

        return {dff0, dff1};
    }
};


struct HandshakeBits {
    std::set<CellPin> inner;

    HandshakeBits() = delete;

    HandshakeBits(const Handshake& handshake) {
        auto [if0, if1] = handshake.decompose();
        CellPin ctrl0 = std::get<0>(if0);
        CellPin ctrl1 = std::get<0>(if1);
        this->inner = {ctrl0, ctrl1};
    }

    std::pair<CellPin, CellPin> decompose() const {
        auto inner_it = this->inner.begin();
        CellPin ctrl0 = *inner_it;
        std::advance(inner_it, 1);
        CellPin ctrl1 = *inner_it;

        return {ctrl0, ctrl1};
    }

    bool operator <(const HandshakeBits& other) const {
        return this->inner < other.inner;
    }
};


struct PseudoModule {
    std::set<std::pair<Handshake, CellPin>> inputs;
    std::set<std::pair<Handshake, CellPin>> outputs;

    PseudoModule() = default;

    bool operator <(const PseudoModule& other) const {
        if (this->inputs != other.inputs) {
            return this->inputs < other.inputs;
        } else if (this->outputs != other.outputs) {
            return this->outputs < other.outputs;
        } else {
            return false;
        }
    }

    bool operator ==(const PseudoModule& other) const {
        return (this->inputs == other.inputs) && (this->outputs == other.outputs);
    }

    std::shared_ptr<PseudoModule> coalesce(
        const PseudoModule& other
    ) const {
        // Modules can be coalesced together if they share some of the
        // same handshake pin at the same input interface
        //
        // Note: We can apply the same condition to the output theoretically.
        // However, this possibility hsould be ruled out by construction.
        bool has_common_ctrl = false;
        for (const auto& [in_handshake, in_pin]: this->inputs) {
            for (const auto& [other_in_handshake, other_in_pin]: other.inputs) {
                if ((in_handshake == other_in_handshake) && (in_pin == other_in_pin)) {
                    has_common_ctrl = true;
                }
            }
        }

        if (has_common_ctrl) {
            std::shared_ptr<PseudoModule> module_union = std::make_shared<PseudoModule>();
            module_union->inputs.insert(this->inputs.begin(), this->inputs.end());
            module_union->inputs.insert(other.inputs.begin(), other.inputs.end());
            module_union->outputs.insert(this->outputs.begin(), this->outputs.end());
            module_union->outputs.insert(other.outputs.begin(), other.outputs.end());

            return module_union;
        } else {
            return nullptr;
        }
    }

    void write_log() const {
        log("Inputs:\n");
        for (const auto& [_handshake, ctrl_pin]: this->inputs) {
            auto [cell, port, idx] = ctrl_pin;
            log("%s:%s:%d\n", log_id(cell), log_id(port), idx);
        }
        log("Outputs:\n");
        for (const auto& [_handshake, ctrl_pin]: this->outputs) {
            auto [cell, port, idx] = ctrl_pin;
            log("%s:%s:%d\n", log_id(cell), log_id(port), idx);
        }
    }
};


struct PseudoModuleCollection {
    std::set<std::shared_ptr<PseudoModule>> inner;

    PseudoModuleCollection() = delete;
    PseudoModuleCollection(
        const std::map<std::pair<Handshake, CellPin>, std::set<std::pair<Handshake, CellPin>>>& connectivity_graph
    ) {
        for (const auto& [sink, srcs]: connectivity_graph) {
            // Register modules first
            std::shared_ptr<PseudoModule> module_ptr = std::make_shared<PseudoModule>();
            module_ptr->inputs = srcs;
            module_ptr->outputs = {sink};

            this->inner.insert(module_ptr);
        }

        // Coalesce modules
        std::set<std::shared_ptr<PseudoModule>> work_list = this->inner;

        while (!work_list.empty()) {
            std::shared_ptr<PseudoModule> curr = *work_list.begin();
            work_list.erase(curr);

            bool can_coalesce = false;
            std::pair<
                std::shared_ptr<PseudoModule>,
                std::shared_ptr<PseudoModule>
            > replacement = {nullptr, nullptr};

            for (std::shared_ptr<PseudoModule> owned_module: this->inner) {
                if (*owned_module == *curr) {
                    continue;
                }

                std::shared_ptr<PseudoModule> module_union = curr->coalesce(*owned_module);
                if (module_union) {
                    // Pending for removal
                    can_coalesce = true;
                    replacement = {owned_module, module_union};
                    break;
                }
            }

            if (can_coalesce) {
                // Remove the 2 old modules, and insert the new
                // Remove/insert on both the owned set and the work list
                auto [old_module, new_module] = replacement;

                work_list.erase(old_module);
                this->inner.erase(old_module);
                this->inner.erase(curr);

                work_list.insert(new_module);
                this->inner.insert(new_module);
            }
        }
    }

    void write_log() const {
        log("Found %ld module\n\n", this->inner.size());
        for (std::shared_ptr<PseudoModule> module: this->inner) {
            log("Begin module\n");
            module->write_log();
            log("\n");
        }
    }
};


struct VrModule {
    std::set<std::pair<Handshake, CellPin>> ctrl_inputs;
    std::set<std::pair<Handshake, CellPin>> ctrl_outputs;

    std::set<Handshake> uncategorized_handshakes;

    std::set<std::pair<Handshake, std::set<Wire>>> data_inputs;
    std::set<std::pair<Handshake, std::set<Wire>>> data_outputs;

    VrModule() = delete;
    VrModule(
        std::shared_ptr<PseudoModule> pseudo_module,
        const std::map<Handshake, std::set<Wire>>& data_ifs,
        const CircuitGraph& circuit_graph
    ) {
        // Collect all the handshakes
        std::set<Handshake> handshakes;
        for (const auto& [handshake, _in_pin]: pseudo_module->inputs) {
            handshakes.insert(handshake);
        }

        // Verify module integrity:
        // Handshakes featured as inputs should also feature as outputs
        {
            std::set<Handshake> check_handshakes;
            for (const auto& [handshake, _out_pin]: pseudo_module->outputs) {
                check_handshakes.insert(handshake);
            }

            if (check_handshakes != handshakes) {
                log("Module with issue:\n");
                pseudo_module->write_log();
                log_error("Found handshake entries discrepencies in a pesudo module.\n");
            }
        }

        this->ctrl_inputs = pseudo_module->inputs;
        this->ctrl_outputs = pseudo_module->outputs;
        this->uncategorized_handshakes = handshakes;

        // Attempt to populate data direction
        std::set<Handshake> output_handshakes;
        std::set<Handshake> source_handshakes;
        for (const Handshake& handshake: this->uncategorized_handshakes) {
            for (const Handshake& other_handshake: this->uncategorized_handshakes) {
                if (handshake == other_handshake) {
                    continue;
                }

                std::set<CellPin> data_srcs;
                std::set<CellPin> other_data_srcs;
                for (auto [wire_src, _wire_sink]: data_ifs.at(handshake)) {
                    data_srcs.insert(wire_src);
                }
                for (auto [wire_src, _wire_sink]: data_ifs.at(other_handshake)) {
                    other_data_srcs.insert(wire_src);
                }

                if (circuit_graph.cyclic_first_reachable(
                    data_srcs, other_data_srcs
                )) {
                    // Other handshake data depends on this handshake data
                    // Hence, other handshake is an output handshake
                    output_handshakes.insert(other_handshake);
                    this->data_outputs.insert({other_handshake, data_ifs.at(other_handshake)});

                    // The source handshake is recorded
                    // Note that it does NOT imply the said handshake is an input
                    // Outputs depending on outputs is completely legal
                    source_handshakes.insert(handshake);
                }
            }
        }

        // If any handshake had been categorized, we assume the remaining source handshakes are inputs
        if (!output_handshakes.empty()) {
            for (const Handshake& src: source_handshakes) {
                if (output_handshakes.count(src) == 0) {
                    this->data_inputs.insert({src, data_ifs.at(src)});
                }
            }

            // Update uncategorized handshakes
            for (const Handshake& src: source_handshakes) {
                this->uncategorized_handshakes.erase(src);
            }

            for (const Handshake& output: output_handshakes) {
                this->uncategorized_handshakes.erase(output);
            }
        }
    }

    void update_data_interface(const Handshake& hs, std::set<Wire> data_iface) {
        std::set<std::pair<Handshake, std::set<Wire>>> removable_inputs;
        for (const auto& [handshake, hs_data_if]: this->data_inputs) {
            if (handshake == hs) {
                removable_inputs.insert({handshake, hs_data_if});
            }
        }

        for (const std::pair<Handshake, std::set<Wire>>& removing_pair: removable_inputs) {
            this->data_inputs.erase(removing_pair);
            this->data_inputs.insert({hs, data_iface});
        }

        std::set<std::pair<Handshake, std::set<Wire>>> removable_outputs;
        for (const auto& [handshake, hs_data_if]: this->data_outputs) {
            if (handshake == hs) {
                removable_outputs.insert({handshake, hs_data_if});
            }
        }

        for (const std::pair<Handshake, std::set<Wire>>& removing_pair: removable_outputs) {
            this->data_outputs.erase(removing_pair);
            this->data_outputs.insert({hs, data_iface});
        }
    }
};

struct VrModuleCollection {
    std::vector<VrModule> inner;

    VrModuleCollection(
        const PseudoModuleCollection& pseudo_modules,
        const std::map<Handshake, std::set<Wire>>& data_ifs,
        const CircuitGraph& circuit_graph
    ) {
        // Transfer modules
        std::set<Handshake> ingress_handshakes;
        std::set<Handshake> egress_handshakes;
        for (std::shared_ptr<PseudoModule> module: pseudo_modules.inner) {
            VrModule vr_module(module, data_ifs, circuit_graph);
            for (const auto& [handshake, _in_data_if]: vr_module.data_inputs) {
                ingress_handshakes.insert(handshake);
            }
            for (const auto& [handshake, _out_data_if]: vr_module.data_outputs) {
                egress_handshakes.insert(handshake);
            }
            this->inner.push_back(vr_module);
        }

        // Spread direction info
        for (VrModule& vr_module: this->inner) {
            for (Handshake handshake: vr_module.uncategorized_handshakes) {
                if (ingress_handshakes.count(handshake)) {
                    vr_module.data_outputs.insert({handshake, data_ifs.at(handshake)});
                } else if (egress_handshakes.count(handshake)) {
                    vr_module.data_inputs.insert({handshake, data_ifs.at(handshake)});
                } else {
                    log_error("Handshake direction completely unknown\n");
                }
            }

            vr_module.uncategorized_handshakes.clear();
        }
    }
};


template<class T>
constexpr auto pair_swap(T&& pair) {
    return std::make_pair(
        std::forward<T>(pair).second,
        std::forward<T>(pair).first
    );
}


#endif

PRIVATE_NAMESPACE_END
