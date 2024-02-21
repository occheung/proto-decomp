#include "kernel/register.h"
#include "kernel/log.h"
#include <stdlib.h>
#include <stdio.h>

USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

struct SimpleOptPass : public Pass {
	SimpleOptPass() : Pass("simple_opt", "perform simple optimizations") { }

	void execute(std::vector<std::string>, RTLIL::Design *design) override
	{
        while (1) {
            design->scratchpad_unset("opt.did_something");

            Pass::call(design, "opt_dff");
            Pass::call(design, "opt_expr -mux_undef -mux_bool -undriven -noclkinv -fine");

            Pass::call(design, "opt_clean -purge");
            if (design->scratchpad_get_bool("opt.did_something") == false)
                break;
        }
	}
} SimpleOptPass;

PRIVATE_NAMESPACE_END
