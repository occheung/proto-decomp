from migen import *
from migen.genlib.coding import PriorityEncoder
from misoc.interconnect import stream

import itertools


DATA_WIDTH = 8
X_WIDTH = 2
NODE_LAYOUT = [
    ("data", DATA_WIDTH),
]
ROUTE_LAYOUT = NODE_LAYOUT + [
    ("x", X_WIDTH),
]
DATA_DEPTH = 8


class RingNode(Module):
    def __init__(self, x):
        self.proc_src = stream.Endpoint(NODE_LAYOUT)
        self.proc_sink = stream.Endpoint(NODE_LAYOUT)
        self.route_src = stream.Endpoint(ROUTE_LAYOUT)
        self.route_sink = stream.Endpoint(ROUTE_LAYOUT)

        self.submodules.own_fifo = stream.SyncFIFO(NODE_LAYOUT, DATA_DEPTH)
        self.comb += self.proc_src.connect(self.own_fifo.sink)

        self.submodules.route_fifo = stream.SyncFIFO(ROUTE_LAYOUT, DATA_DEPTH)
        self.comb += self.route_src.connect(self.route_fifo.sink)

        # DEMUX the route source to either an intermediate endpoint or the sink
        self.submodules.demux = stream.Demultiplexer(ROUTE_LAYOUT, 2)
        self.route_inter = stream.Endpoint(ROUTE_LAYOUT)

        self.comb += [
            self.route_fifo.source.connect(self.demux.sink),
            self.demux.source0.connect(self.route_inter),
            self.demux.source1.connect(self.proc_sink, omit=["x"]),

            self.demux.sel.eq(self.demux.sink.x == x),
        ]

        # Convert the node owned fifo to the route layout
        self.own_inter = stream.Endpoint(ROUTE_LAYOUT)
        self.comb += [
            # Connect everything except the cooridnate field
            self.own_fifo.source.connect(self.own_inter, omit=["x"]),

            # Statically tie the coordinate
            self.own_inter.x.eq(x),
        ]

        # MUX
        # 
        # Multiplex the node owned fifo and the route fifo
        self.submodules.mux = stream.Multiplexer(ROUTE_LAYOUT, 2)
        self.comb += [
            self.route_inter.connect(self.mux.sink0),
            self.own_inter.connect(self.mux.sink1),
            self.mux.source.connect(self.route_sink),
        ]

        # MUX sel:
        # Use priority encoder to generate the next possible allocation
        # Only update allocation when a finish signal (stb & ack) is detected
        self.submodules.encoder = PriorityEncoder(2)
        self.comb += [
            self.encoder.i[0].eq(self.mux.sink0.stb),
            self.encoder.i[1].eq(self.mux.sink1.stb),
        ]
        
        self.sync += If(self.mux.source.ack & self.mux.source.stb,
            self.mux.sel.eq(self.encoder.o)
        )

        proc_io_list = [ f[0] for f in self.proc_src.iter_flat() ] \
            + [ f[0] for f in self.proc_sink.iter_flat() ]
        route_io_list = [ f[0] for f in self.route_src.iter_flat() ] \
            + [ f[0] for f in self.route_sink.iter_flat() ]
        io_list = proc_io_list + route_io_list

        self.io = set(io_list)
        self.proc_io = set(proc_io_list)
        self.route_io = set(route_io_list)


class RingRouter(Module):
    def __init__(self, length):
        nodes = []
        self.io = set()
        for i in range(length):
            node = RingNode(i)
            nodes.append(node)

            self.io |= node.proc_io
        
        # Connect nodes
        for i in range(length):
            from_idx = i
            to_idx = (i + 1) % length

            self.comb += [
                nodes[from_idx].route_sink.connect(nodes[to_idx].route_src),
            ]

        # Add to submodules
        for i in range(length):
            setattr(self.submodules, "node"+str(i), nodes[i])


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    # module = RingNode(0)
    # convert(module, module.io).write("ring_node.v")

    module = RingRouter(4)
    convert(module, module.io).write("ring_router.v")
