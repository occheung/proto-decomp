from migen import *
from pyrtl import *


class Producer(Module):
    def __init__(self):
        self.data = Signal(32, reset_less=True)
        self.stb = Signal(1, reset_less=True)
        self.rdy = Signal(1, reset_less=True)

        self.io = set([self.data, self.stb, self.rdy])

        # cycle accumulator, trigger trasmission when cycle == 0
        cycles = Signal(4, reset_less=True)
        self.sync += cycles.eq(cycles + 1)

        # self.submodules.fsm = FSM(reset_state="WAIT_CLK")

        # self.fsm.act("WAIT_CLK",
        #     If(cycles == 0,
        #         NextState("WAIT_RDY"),
        #         NextValue(self.stb, 1),
        #         NextValue(self.data, self.data + 0xdeadbeef)
        #     ),
        # )

        # self.fsm.act("WAIT_RDY",
        #     If(self.rdy,
        #         NextState("WAIT_CLK"),
        #         NextValue(self.stb, 0)
        #     )
        # )

        self.sync += [
            If(cycles == 0,
                self.stb.eq(1),
                If(~self.rdy,
                    self.data.eq(self.data + 0xdeadbeef))
            ).Else(
                self.stb.eq(self.stb & ~self.rdy)
            )
        ]


class ForwardingProducer(Module):
    def __init__(self):
        self.data = Signal(32, reset_less=True)
        self.stb = Signal(1, reset_less=True)
        self.rdy = Signal(1, reset_less=True)

        self.io = set([self.data, self.stb, self.rdy])

        cycles = Signal(4, reset_less=True)
        curr_data = Signal(32, reset_less=True)
        pending_data = Signal(32, reset_less=True)

        self.submodules.fsm = FSM(reset_state="WAIT_TRIG")

        self.fsm.act("WAIT_TRIG",
            If(cycles == 0,
                self.stb.eq(1),
                self.data.eq(curr_data),
                NextValue(pending_data, curr_data),

                If(~self.rdy,
                    NextState("WAIT_RDY")
                )
                # Otherwise, prepare for the next transmission
                # Note that this state exactly does the preparation
            ).Else(
                self.stb.eq(0)
            )
        )

        self.fsm.act("WAIT_RDY",
            self.stb.eq(1),
            self.data.eq(pending_data),

            If(self.rdy,
                NextState("WAIT_TRIG")
            )
        )

        self.sync += [
            cycles.eq(cycles + 1),
            curr_data.eq(curr_data + 0xdeadbeef),
        ]


class ValidFSMProducer(Module):
    def __init__(self):
        self.data = Signal(32, reset_less=True)
        self.stb = Signal(1, reset_less=True)
        self.rdy = Signal(1, reset_less=True)

        self.io = set([self.data, self.stb, self.rdy])

        cycles = Signal(4, reset_less=True)
        pending_data = Signal(32, reset_less=True)

        self.submodules.fsm = FSM(reset_state="VALID_LOW")

        self.fsm.act("VALID_LOW",
            self.stb.eq(0),
            If(cycles == 0,
                NextValue(self.data, pending_data),
                NextState("VALID_HIGH"),
            )
        )

        self.fsm.act("VALID_HIGH",
            self.stb.eq(1),
            If(self.rdy,
                If(cycles != 0,
                    NextState("VALID_LOW"),
                ).Else(
                    NextValue(self.data, pending_data),
                )
            )
        )

        self.sync += [
            cycles.eq(cycles + 1),
            pending_data.eq(pending_data + 0xdeadbeef)
        ]


if __name__ == "__main__":
    # from migen.fhdl.verilog import convert
    # module = ValidFSMProducer()
    # convert(module, module.io, name="Producer").write("simple_producer.v")

    import subprocess
    subprocess.run([
        "yosys", "-m", "sim_opt.so", "-p",
        'read_verilog -nolatches simple_producer.v; \
        hierarchy -check -top Producer; \
        proc; \
        simple_opt; \
        write_blif producer.blif; show; \
        write_verilog producer_synth.v; \
        write_rtlil simple_producer.rtlil'
    ])
