from migen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone

# USB Host -----------------------------------------------------------------------------------------

class USBHost(Module, AutoCSR):
    def __init__(self, platform, usb_pads):
        self.wb_ctrl = wb_ctrl = wishbone.Interface(data_width=32)
        self.wb_dma  = wb_dma  = wishbone.Interface(data_width=32) # CHECKME.

        self.specials += Instance("UsbOhciWishbone",
            # Clk / Rst.
            i_clk   = ClockSignal("sys"),
            i_reset = ResetSignal("sys"),

            # Wishbone Control. CHECKME: SEL missing? DAT_MISO-> dat_r? DAT_MOSI > dat_w?
            i_io_ctrl_CYC      = wb_ctrl.cyc,
            i_io_ctrl_STB      = wb_ctrl.stb,
            o_io_ctrl_ACK      = wb_ctrl.ack,
            i_io_ctrl_WE       = wb_ctrl.we,
            i_io_ctrl_ADR      = wb_ctrl.adr,
            o_io_ctrl_DAT_MISO = wb_ctrl.dat_r,
            i_io_ctrl_DAT_MOSI = wb_ctrl.dat_w,

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
            o_io_interrupt = Signal(), # FIXME: Connect.

            # USB
            i_io_usb_0_dp_read        = usb_pads.dp_read,
            o_io_usb_0_dp_write       = usb_pads.dp_write,
            o_io_usb_0_dp_writeEnable = usb_pads.dp_write_enable,
            i_io_usb_0_dm_read        = usb_pads.dm_read,
            o_io_usb_0_dm_write       = usb_pads.dm_write,
            o_io_usb_0_dm_writeEnable = usb_pads.dm_write_enable,
        )
        platform.add_source("UsbOhciWishbone.v") # FIXME: Add generated netlist.
