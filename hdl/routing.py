from migen import *
from migen.genlib.coding import PriorityEncoder
from misoc.interconnect import stream

import itertools


class StreamRouter(Module):
    def __init__(self):
        DATA_WIDTH = 8
        LAYOUT = [("data", DATA_WIDTH)]
        DATA_DEPTH = 8
        NUM_OF_BRANCHES = 4

        # All stream sources
        for i in range(NUM_OF_BRANCHES):
            src_fifo = stream.SyncFIFO(LAYOUT, DATA_DEPTH)
            setattr(self.submodules, "source_fifo"+str(i), src_fifo)
        
        # MUX that selects 1 source
        self.submodules.mux = stream.Multiplexer(
            LAYOUT, NUM_OF_BRANCHES
        )
        for i in range(NUM_OF_BRANCHES):
            self.comb += getattr(self, "source_fifo"+str(i)).source.connect(
                getattr(self.mux, "sink"+str(i)))

        # DEMUX that selects 1 sink
        self.submodules.demux = stream.Demultiplexer(LAYOUT, NUM_OF_BRANCHES)
        self.comb += self.mux.source.connect(self.demux.sink)

        # All stream sinks
        for i in range(NUM_OF_BRANCHES):
            sink_fifo = stream.SyncFIFO(LAYOUT, DATA_DEPTH)
            self.comb += getattr(self.demux, "source"+str(i)).connect(sink_fifo.sink)
            setattr(self.submodules, "sink_fifo"+str(i), sink_fifo)
        
        # MUX sel:
        # Use priority encoder to generate the next possible allocation
        # Only update allocation when a finish signal (stb & ack) is detected
        self.submodules.encoder = PriorityEncoder(NUM_OF_BRANCHES)
        for i in range(NUM_OF_BRANCHES):
            self.comb += self.encoder.i[i].eq(getattr(self.mux, "sink"+str(i)).stb)
        
        self.sync += If(self.mux.source.ack & self.mux.source.stb,
            self.mux.sel.eq(self.encoder.o)
        )

        # DEMUX sel: Use round-robin allocation
        self.sync += If(self.demux.sink.stb & self.demux.sink.ack,
            self.demux.sel.eq(self.demux.sel + 1)
        )
        
        io_list = []
        for i in range(NUM_OF_BRANCHES):
            io_list += [ f[0] for f in getattr(self, "source_fifo"+str(i)).sink.iter_flat() ]

        for i in range(NUM_OF_BRANCHES):
            io_list += [ f[0] for f in getattr(self, "sink_fifo"+str(i)).source.iter_flat() ]
        self.io = set(io_list)


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    module = StreamRouter()
    convert(module, module.io).write("stream_router.v")
