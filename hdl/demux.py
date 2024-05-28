from migen import *
from misoc.interconnect import stream

import itertools


class StreamDemux(Module):
    def __init__(self):
        DATA_WIDTH = 8
        LAYOUT = [("data", DATA_WIDTH)]
        NUM_OF_SINKS = 4

        self.submodules.source_fifo = stream.SyncFIFO(
            LAYOUT,     # layout
            DATA_WIDTH,              # depth
            # Keep buffer=False. It allows async reading
        )

        self.submodules.demux = stream.Demultiplexer(
            LAYOUT, NUM_OF_SINKS
        )
        self.comb += self.source_fifo.source.connect(self.demux.sink)

        for i in range(NUM_OF_SINKS):
            sink_fifo = stream.SyncFIFO(
                LAYOUT,
                DATA_WIDTH,
            )
            self.comb += getattr(self.demux, "source"+str(i)).connect(sink_fifo.sink)
            setattr(self.submodules, "sink_fifo"+str(i), sink_fifo)
        
        # Use round-robin allocation
        self.alloc_update = Signal()
        self.comb += Case(self.demux.sel, {
            i: self.alloc_update.eq(self.demux.sink.stb & self.demux.sink.ack)
        })

        self.sync += If(self.alloc_update, self.demux.sel.eq(self.demux.sel + 1))
        # cycles = Signal(log2_int(NUM_OF_SINKS))
        # self.sync += cycles.eq(cycles + 1)
        # self.comb += self.demux.sel.eq(cycles)
        
        io_list = []
        io_list += [ f[0] for f in self.source_fifo.sink.iter_flat() ]
        for i in range(NUM_OF_SINKS):
            io_list += [ f[0] for f in getattr(self, "sink_fifo"+str(i)).source.iter_flat() ]
        self.io = set(io_list)

        # self.io = set([ f[0] for f in itertools.chain(
        #     self.source_fifo.sink.iter_flat(), self.sink_fifo.source.iter_flat()) ])


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    module = StreamDemux()
    convert(module, module.io).write("stream_demux.v")
