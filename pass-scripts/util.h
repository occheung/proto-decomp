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

    bool operator ==(const Handshake& other) const {
        return this->info == other.info;
    }

    bool operator <(const Handshake& other) const {
        auto this_it = this->info.begin();
        auto other_it = other.info.begin();
        for (; (this_it != this->info.end()) && (other_it != other.info.end()); ++this_it, ++other_it) {
            auto [this_pin, _this_dffs] = *this_it;
            auto [other_pin, _other_dffs] = *other_it;
            if (this_pin != other_pin) {
                return this_pin < other_pin;
            }
        }

        // The execution flow reaches here iff the pin set are identical
        return false;
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


template<class T>
constexpr auto pair_swap(T&& pair) {
    return std::make_pair(
        std::forward<T>(pair).second,
        std::forward<T>(pair).first
    );
}


#endif

PRIVATE_NAMESPACE_END
