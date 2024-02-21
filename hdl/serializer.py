from migen import *
from pyrtl import *
from argparse import ArgumentParser
from producer import *
from consumer import Consumer
import os


class Serializer(Module):
    def __init__(self, producer_cls):
        # self.submodules.producer = ForwardingProducer()
        self.submodules.producer = producer_cls()
        # self.submodules.producer = Producer()
        self.submodules.consumer = Consumer()

        self.comb += [
            self.producer.rdy.eq(self.consumer.rdy),

            self.consumer.stb.eq(self.producer.stb),
            self.consumer.dat_i.eq(self.producer.data)
        ]

        self.io = set([ self.consumer.dat_o ])

        # self.dat_i = Signal(32, reset_less=True)
        # self.dat_o = Signal(32, reset_less=True)
        # self.stb = Signal(1, reset_less=True)
        # self.rdy = Signal(1, reset_less=True)

        # self.specials.producer = Instance("Producer",
        #     o_data=self.dat_i,
        #     o_stb=self.stb,
        #     i_rdy=self.rdy,
        # )
        
        # self.specials.consumer = Instance("Consumer",
        #     i_dat_i=self.dat_i,
        #     i_stb=self.stb,
        #     o_dat_o=self.dat_o,
        # )

        # self.io = set([ self.dat_o ])

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("producer")
    parser.add_argument("--verilog", action="store_true", default=False)
    parser.add_argument("--run_yosys", action="store_true", default=False)
    parser.add_argument("--analyze", action="store_true", default=False)

    args = parser.parse_args()
    get_class = lambda x: globals()[x]

    import re
    producer_snake = re.sub(r'(?<!^)(?=[A-Z])', '_', args.producer).lower()
    verilog_filename = producer_snake + "_serializer.v"
    rtlil_filename = producer_snake + "_serializer.rtlil"

    if args.verilog:
        producer_cls = get_class(args.producer)
        module = Serializer(producer_cls)

        from migen.fhdl.verilog import convert
        convert(module, module.io).write(verilog_filename)

    if args.run_yosys:
        import subprocess
        subprocess.run([
            "yosys", "-m", "sim_opt.so", "-p",
            'read_verilog -nolatches ' + verilog_filename + '; \
            hierarchy -check -top top; \
            proc; \
            simple_opt; \
            synth; \
            write_blif serializer.blif; show; \
            write_verilog serializer_synth.v; \
            write_rtlil ' + rtlil_filename
        ])
    
    if args.analyze:
        import subprocess
        subprocess.run([
            "yosys", "-m", "extract.so", "-p",
            'read_rtlil ' + rtlil_filename + '; \
            extract_subgraph; \
            show'
        ])
