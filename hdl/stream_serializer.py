from migen import *
from misoc.interconnect import stream

import itertools


class Serializer(Module):
    def __init__(self):
        SRC_LAYOUT = [("data", 4)]
        SINK_LAYOUT = [("data", 1)]

        self.submodules.source_fifo = stream.SyncFIFO(
            SRC_LAYOUT,     # layout
            8,              # depth
            # Keep buffer=False. It allows async reading
        )

        self.submodules.sink_converter = stream.StrideConverter(
            SRC_LAYOUT,
            SINK_LAYOUT,
        )

        self.comb += self.source_fifo.source.connect(self.sink_converter.sink)

        self.io = set([ f[0] for f in itertools.chain(
            self.source_fifo.sink.iter_flat(), self.sink_converter.source.iter_flat()) ])


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    module = Serializer()
    convert(module, module.io).write("stream_producer.v")
