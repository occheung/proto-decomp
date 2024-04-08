from migen import *
from migen.build.generic_platform import GenericPlatform

import math
import os


def safe_log(x):
    return 1 if x == 1 else math.ceil(math.log(x, 2))


class BsgCacheToAxiRx(Module):
    # Initialize IOs
    def __init__(self, num_cache=1, addr_width=28, data_width=32,
            block_size_in_words=4, axi_id_width=6, axi_data_width=32,
            axi_burst_len=1, axi_burst_type=0):
        # self.clock_domains.cd_sys = ClockDomain()

        params = dict(
            num_cache_p=num_cache,
            addr_width_p=addr_width,
            data_width_p=data_width,
            block_size_in_words_p=block_size_in_words,
            tag_fifo_els_p=num_cache,

            axi_id_width_p=axi_id_width,
            axi_data_width_p=axi_data_width,
            axi_burst_len_p=axi_burst_len,
            axi_burst_type_p=axi_burst_type,

            # lg_num_cache_lp=safe_log(num_cache),
            # data_width_ratio_lp=axi_data_width//data_width
        )

        lg_num_cache_lp = safe_log(num_cache)

        parameters = {"p_{}".format(k): v for k, v in params.items()}

        # # params
        # self.num_cache_p = num_cache
        # self.addr_width_p = addr_width
        # self.data_width_p = data_width
        # self.block_size_in_words_p = block_size_in_words
        # self.tag_fifo_els_p = self.num_cache_p

        # self.axi_id_width_p = axi_id_width
        # self.axi_data_width_p = axi_data_width
        # self.axi_burst_len_p = axi_burst_len
        # self.axi_burst_type_p = axi_burst_type

        # self.lg_num_cache_lp = safe_log(self.num_cache_p)
        # self.data_width_ratio_lp = self.axi_data_width_p // self.data_width_p

        self.v_i = Signal()
        self.yumi_o = Signal()
        self.cache_id_i = Signal(lg_num_cache_lp)
        self.addr_i = Signal(params["addr_width_p"])

        # cache dma read channel
        self.dma_data_o = Signal(params["num_cache_p"] * params["data_width_p"])
        self.dma_data_v_o = Signal(params["num_cache_p"])
        self.dma_data_ready_i = Signal(params["num_cache_p"])

        # axi read address channel
        self.axi_arid_o = Signal(params["axi_id_width_p"])
        self.axi_araddr_addr_o = Signal(params["addr_width_p"])
        self.axi_araddr_cache_id_o = Signal(lg_num_cache_lp)
        self.axi_arlen_o = Signal(8)
        self.axi_arsize_o = Signal(3)
        self.axi_arburst_o = Signal(2)
        self.axi_arcache_o = Signal(4)
        self.axi_arprot_o = Signal(3)
        self.axi_arlock_o = Signal()
        self.axi_arvalid_o = Signal()
        self.axi_arready_i = Signal()

        # axi read data channel
        self.axi_rid_i = Signal(params["axi_id_width_p"])
        self.axi_rdata_i = Signal(params["axi_data_width_p"])
        self.axi_rresp_i = Signal(2)
        self.axi_rlast_i = Signal()
        self.axi_rvalid_i = Signal()
        self.axi_rready_o = Signal()

        self.io = set([
            self.v_i, self.yumi_o, self.cache_id_i, self.addr_i, self.dma_data_o, self.dma_data_v_o, self.dma_data_ready_i,
            self.axi_arid_o, self.axi_araddr_addr_o, self.axi_araddr_cache_id_o, self.axi_arlen_o, self.axi_arsize_o, self.axi_arburst_o,
            self.axi_arcache_o, self.axi_arprot_o, self.axi_arlock_o, self.axi_arvalid_o, self.axi_arready_i,

            self.axi_rid_i, self.axi_rdata_i, self.axi_rresp_i, self.axi_rlast_i, self.axi_rvalid_i, self.axi_rready_o, 
        ])

        self.specials += Instance(
            "bsg_cache_to_axi_rx",
            i_clk_i=ClockSignal(),
            i_reset_i=ResetSignal(),

            i_v_i=self.v_i,
            o_yumi_o=self.yumi_o,
            i_cache_id_i=self.cache_id_i,
            i_addr_i=self.addr_i,

            o_dma_data_o=self.dma_data_o,
            o_dma_data_v_o=self.dma_data_v_o,
            i_dma_data_ready_i=self.dma_data_ready_i,

            o_axi_arid_o=self.axi_arid_o,
            o_axi_araddr_addr_o=self.axi_araddr_addr_o,
            o_axi_araddr_cache_id_o=self.axi_araddr_cache_id_o,
            o_axi_arlen_o=self.axi_arlen_o,
            o_axi_arsize_o=self.axi_arsize_o,
            o_axi_arburst_o=self.axi_arburst_o,
            o_axi_arcache_o=self.axi_arcache_o,
            o_axi_arprot_o=self.axi_arprot_o,
            o_axi_arlock_o=self.axi_arlock_o,
            o_axi_arvalid_o=self.axi_arvalid_o,
            i_axi_arready_i=self.axi_arready_i,

            i_axi_rid_i=self.axi_rid_i,
            i_axi_rdata_i=self.axi_rdata_i,
            i_axi_rresp_i=self.axi_rresp_i,
            i_axi_rlast_i=self.axi_rlast_i,
            i_axi_rvalid_i=self.axi_rvalid_i,
            o_axi_rready_o=self.axi_rready_o,

            **parameters
        )

        # vdir = os.path.join(os.path.abspath(
        #     os.path.dirname(__file__)), "lib/bsg_cache_to_axi_rx")
        # platform.add_source_dir(vdir)


if __name__ == "__main__":
    from migen.fhdl.verilog import convert
    module = BsgCacheToAxiRx(
        num_cache=4,
        addr_width=30,
        data_width=4,
        block_size_in_words=4,
        axi_id_width=6,
        axi_data_width=32,
        axi_burst_len=1,
        axi_burst_type=0
    )
    convert(module, module.io).write("bsg_cache_to_axi_rx.v")
    # cache_to_axi_rx = BsgCacheToAxiRx(platform)
    # platform.finalize(cache_to_axi_rx)
    # from migen.fhdl.verilog import convert

    # platform.get_verilog(cache_to_axi_rx, create_clock_domains=True)
    # convert(cache_to_axi_rx).write("cache_to_axi_rx.v")

    # vdir = os.path.join(os.path.abspath(os.path.dirname(__file__)), "verilog")
    # platform.add_source(os.path.join(vdir, variant+".v"))


    from lib.script import generate_rtlil
    generate_rtlil("hdl/lib/bsg_cache_to_axi_rx", "bsg_cache_to_axi_rx.v")