from migen import *


class Consumer(Module):
    def __init__(self):
        self.rdy = Signal(1, reset_less=True)
        self.stb = Signal(1, reset_less=True)
        self.dat_i = Signal(32, reset_less=True)
        self.dat_o = Signal(8, reset_less=True)

        self.io = set([ self.rdy, self.stb, self.dat_i, self.dat_o ])

        dat_buf = Array([ Signal(8) for _ in range(4) ])
        counter = Signal(2, reset_less=True)

        # Pretend the circuit always takes 4 cycles to process
        self.sync += [
            If(self.stb,
                counter.eq(counter + 1)
            ).Else(
                counter.eq(0)
            )
        ]

        self.comb += [
            self.rdy.eq(counter == 3),
            self.dat_o.eq(dat_buf[counter])
        ]

        self.comb += [ dat_buf[i].eq(self.dat_i[i*8:(i+1)*8]) for i in range(4) ]


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    module = Consumer()
    convert(module, module.io, name="Consumer").write("consumer.v")

    import subprocess
    subprocess.run([
        "yosys", "-p",
        'read_verilog -nolatches consumer.v; \
        hierarchy -check -top Consumer; \
        proc; \
        opt_dff -nosdff; \
        opt_clean ; \
        write_blif consumer.blif; \
        show'
    ])
