from migen import *
from migen.build.generic_platform import GenericPlatform

import math
import os


class BsgSerializer(Module):
    def __init__(self,
        parallel_width=32, parallel_buffer_depth=8,
        serial_width=8, serial_buffer_depth=2):
        producer_if = {
            # Inputs
            "ready_o": Signal(),
            "data_i": Signal(parallel_width),
            "v_i": Signal(),

            # To piso
            "v_o": Signal(),
            "data_o": Signal(parallel_width),
            "yumi_i": Signal(),
        }

        consumer_if = {
            # To piso
            "ready_o": Signal(),
            "data_i": Signal(serial_width),
            "v_i": Signal(),

            # Outputs
            "v_o": Signal(),
            "data_o": Signal(serial_width),
            "yumi_i": Signal(),
        }

        self.io = set([
            producer_if["ready_o"], producer_if["data_i"], producer_if["v_i"],
            consumer_if["v_o"], consumer_if["data_o"], consumer_if["yumi_i"],
        ])

        self.specials += Instance("bsg_two_fifo",
            p_width_p=parallel_width,
            p_els=parallel_buffer_depth,

            i_clk_i=ClockSignal(),
            i_reset_i=ResetSignal(),

            o_ready_o=producer_if["ready_o"],
            i_data_i=producer_if["data_i"],
            i_v_i=producer_if["v_i"],

            o_v_o=producer_if["v_o"],
            o_data_o=producer_if["data_o"],
            i_yumi_i=producer_if["yumi_i"],
        )

        self.specials += Instance("bsg_parallel_in_serial_out",
            p_width_p=serial_width,
            p_els_p=parallel_width//serial_width,

            i_clk_i=ClockSignal(),
            i_reset_i=ResetSignal(),

            i_valid_i=producer_if["v_o"],
            i_data_i=producer_if["data_o"],
            o_ready_and_o=producer_if["yumi_i"],
            
            o_valid_o=consumer_if["v_i"],
            o_data_o=consumer_if["data_i"],
            i_yumi_i=consumer_if["ready_o"],
        )

        self.specials += Instance("bsg_two_fifo",
            p_width_p=serial_width,
            p_els=serial_buffer_depth,

            i_clk_i=ClockSignal(),
            i_reset_i=ResetSignal(),

            o_ready_o=consumer_if["ready_o"],
            i_data_i=consumer_if["data_i"],
            i_v_i=consumer_if["v_i"],
            
            o_v_o=consumer_if["v_o"],
            o_data_o=consumer_if["data_o"],
            i_yumi_i=consumer_if["yumi_i"],
        )


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    module = BsgSerializer()
    convert(module, module.io).write("bsg_serializer.v")

    from lib.script import generate_rtlil
    generate_rtlil("hdl/lib/bsg_serializer", "bsg_serializer.v")
