from migen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone

from litex.build.io import SDRTristate

# USB Host -----------------------------------------------------------------------------------------

class USBHost(Module, AutoCSR):
    def __init__(self, platform, pads):
        self.wb_ctrl = wb_ctrl = wishbone.Interface(data_width=32)
        self.wb_dma  = wb_dma  = wishbone.Interface(data_width=32)

        self.interrupt = Signal()

        usb_ios = Record([
            ("dp_i",  1), ("dp_o",  1), ("dp_oe", 1),
            ("dm_i",  1), ("dm_o",  1), ("dm_oe", 1),
        ])

        self.specials += Instance("UsbOhciWishbone",
            # Clk / Rst.
            i_clk   = ClockSignal("sys"),
            i_reset = ResetSignal("sys"),

            # Wishbone Control.
            i_io_ctrl_CYC      = wb_ctrl.cyc,
            i_io_ctrl_STB      = wb_ctrl.stb,
            o_io_ctrl_ACK      = wb_ctrl.ack,
            i_io_ctrl_WE       = wb_ctrl.we,
            i_io_ctrl_ADR      = wb_ctrl.adr,
            o_io_ctrl_DAT_MISO = wb_ctrl.dat_r,
            i_io_ctrl_DAT_MOSI = wb_ctrl.dat_w,
            i_io_ctrl_SEL      = wb_ctrl.sel,

            # Wishbone DMA.
            o_io_dma_CYC      = wb_dma.cyc,
            o_io_dma_STB      = wb_dma.stb,
            i_io_dma_ACK      = wb_dma.ack,
            o_io_dma_WE       = wb_dma.we,
            o_io_dma_ADR      = wb_dma.adr,
            i_io_dma_DAT_MISO = wb_dma.dat_r,
            o_io_dma_DAT_MOSI = wb_dma.dat_w,
            o_io_dma_SEL      = wb_dma.sel,
            i_io_dma_ERR      = wb_dma.err,
            o_io_dma_CTI      = wb_dma.cti,
            o_io_dma_BTE      = wb_dma.bte,

            # Interrupt.
            o_io_interrupt = self.interrupt,

            # USB
            i_io_usb_0_dp_read        = usb_ios.dp_i,
            o_io_usb_0_dp_write       = usb_ios.dp_o,
            o_io_usb_0_dp_writeEnable = usb_ios.dp_oe,
            i_io_usb_0_dm_read        = usb_ios.dm_i,
            o_io_usb_0_dm_write       = usb_ios.dm_o,
            o_io_usb_0_dm_writeEnable = usb_ios.dm_oe,
        )
        self.specials += SDRTristate(
            io = pads.dp,
            o  = usb_ios.dp_o,
            oe = usb_ios.dp_oe,
            i  = usb_ios.dp_i,
        )
        self.specials += SDRTristate(
            io = pads.dm,
            o  = usb_ios.dm_o,
            oe = usb_ios.dm_oe,
            i  = usb_ios.dm_i,
        )
        platform.add_source("UsbOhciWishbone.v")
