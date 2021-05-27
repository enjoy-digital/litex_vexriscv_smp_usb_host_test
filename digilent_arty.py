#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse
import json

from migen import *

from litex_boards.platforms import arty
from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict

from litex.soc.cores.clock import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

from litedram.modules import MT41K128M16
from litedram.phy import s7ddrphy

from liteeth.phy.mii import LiteEthPHYMII

from litespi.modules import S25FL128L
from litespi.opcodes import SpiNorFlashOpCodes as Codes
from litespi.phy.generic import LiteSPIPHY
from litespi import LiteSPI

from vexriscv_smp import VexRiscvSMP

from litex_json2dts import generate_dts

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq, with_mapped_flash):
        self.rst = Signal()
        self.clock_domains.cd_usb       = ClockDomain()
        self.clock_domains.cd_sys       = ClockDomain()
        self.clock_domains.cd_sys4x     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_idelay    = ClockDomain()
        self.clock_domains.cd_eth       = ClockDomain()

        # # #

        self.submodules.pll = pll = S7MMCM(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset") | self.rst)
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_usb,       48e6, margin=0)
        pll.create_clkout(self.cd_sys,       sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_idelay,    200e6)
        pll.create_clkout(self.cd_eth,       25e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        self.comb += platform.request("eth_ref_clk").eq(self.cd_eth.clk)
# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    csr_map = {**SoCCore.csr_map, **{
        "ctrl":       0,
        "uart":       2,
        "timer0":     3,
    }}
    interrupt_map = {**SoCCore.interrupt_map, **{
        "uart":       0,
        "timer0":     1,
    }}
    mem_map = {**SoCCore.mem_map, **{
        "ethmac":       0xb0000000, # len: 0x2000
        "usb_host":     0xc0000000,
        "spiflash":     0xd0000000,
        "csr":          0xf0000000,
    }}
    def __init__(self, variant="a7-35", toolchain="vivado", sys_clk_freq=int(100e6), with_ethernet=False, with_etherbone=False, eth_ip="192.168.1.50", eth_dynamic_ip=False, ident_version=True, with_jtagbone=True, with_mapped_flash=False, **kwargs):
        platform = arty.Platform(variant=variant, toolchain=toolchain)

        # SoCCore ----------------------------------------------------------------------------------
        kwargs["cpu_type"]    = "vexriscv_smp"
        kwargs["cpu_variant"] = "linux"
        kwargs["cpu_cls"]  = VexRiscvSMP
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on Arty A7",
            ident_version  = ident_version,
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq, with_mapped_flash)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype        = "DDR3",
                nphases        = 4,
                sys_clk_freq   = sys_clk_freq)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41K128M16(sys_clk_freq, "1:4"),
                l2_cache_size = kwargs.get("l2_size", 8192)
            )

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.submodules.ethphy = LiteEthPHYMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip)

        # Jtagbone ---------------------------------------------------------------------------------
        if with_jtagbone:
            self.add_jtagbone()

        # Flash (through LiteSPI, experimental).
        if with_mapped_flash:
            self.submodules.spiflash_phy  = LiteSPIPHY(platform.request("spiflash4x"), S25FL128L(Codes.READ_1_1_4))
            self.submodules.spiflash_mmap = LiteSPI(self.spiflash_phy, clk_freq=sys_clk_freq, mmap_endianness=self.cpu.endianness)
            spiflash_region = SoCRegion(origin=self.mem_map.get("spiflash", None), size=S25FL128L.total_size, cached=False)
            self.bus.add_slave(name="spiflash", slave=self.spiflash_mmap.bus, region=spiflash_region)

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)

        # USB Host ---------------------------------------------------------------------------------
        from usb_host import USBHost
        from litex.build.generic_platform import Subsignal, Pins, IOStandard
        _usb_pmod_ios = [
            ("usb_pmoda", 0,
                Subsignal("dp", Pins("pmoda:0")),
                Subsignal("dm", Pins("pmoda:4")),
                IOStandard("LVCMOS33"),
            )
        ]
        platform.add_extension(_usb_pmod_ios)
        self.submodules.usb_host = USBHost(platform, platform.request("usb_pmoda"), ports_count=1, phy_frequency=48000000)
        self.bus.add_slave("usb_host_ctrl", self.usb_host.wb_ctrl, region=SoCRegion(origin=self.mem_map["usb_host"], size=0x100000, cached=False)) # FIXME: Mapping.
        self.dma_bus.add_master("usb_host_dma", master=self.usb_host.wb_dma)

        self.comb += self.cpu.interrupt[16].eq(self.usb_host.interrupt)

    # DTS generation ---------------------------------------------------------------------------
    def generate_dts(self, board_name):
        json_src = os.path.join("build", board_name, "csr.json")
        dts = os.path.join("build", board_name, "{}.dts".format(board_name))

        with open(json_src) as json_file, open(dts, "w") as dts_file:
            dts_content = generate_dts(json.load(json_file), polling=False)
            dts_file.write(dts_content)

    # DTS compilation --------------------------------------------------------------------------
    def compile_dts(self, board_name, symbols=False):
        dts = os.path.join("build", board_name, "{}.dts".format(board_name))
        dtb = os.path.join("build", board_name, "{}.dtb".format(board_name))
        subprocess.check_call(
            "dtc {} -O dtb -o {} {}".format("-@" if symbols else "", dtb, dts), shell=True)

    # DTB combination --------------------------------------------------------------------------
    def combine_dtb(self, board_name, overlays=""):
        dtb_in = os.path.join("build", board_name, "{}.dtb".format(board_name))
        dtb_out = os.path.join("rv32.dtb")
        if overlays == "":
            shutil.copyfile(dtb_in, dtb_out)
        else:
            subprocess.check_call(
                "fdtoverlay -i {} -o {} {}".format(dtb_in, dtb_out, overlays), shell=True)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Arty A7")
    parser.add_argument("--toolchain",           default="vivado",                 help="Toolchain use to build (default: vivado)")
    parser.add_argument("--build",               action="store_true",              help="Build bitstream")
    parser.add_argument("--load",                action="store_true",              help="Load bitstream")
    parser.add_argument("--variant",             default="a7-35",                  help="Board variant: a7-35 (default) or a7-100")
    parser.add_argument("--sys-clk-freq",        default=100e6,                     help="System clock frequency (default: 96MHz)")
    ethopts = parser.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",      action="store_true",              help="Enable Ethernet support")
    ethopts.add_argument("--with-etherbone",     action="store_true",              help="Enable Etherbone support")
    parser.add_argument("--eth-ip",              default="192.168.1.50", type=str, help="Ethernet/Etherbone IP address")
    parser.add_argument("--eth-dynamic-ip",      action="store_true",              help="Enable dynamic Ethernet IP addresses setting")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",     action="store_true",              help="Enable SPI-mode SDCard support")
    sdopts.add_argument("--with-sdcard",         action="store_true",              help="Enable SDCard support")
    parser.add_argument("--sdcard-adapter",      type=str,                         help="SDCard PMOD adapter: digilent (default) or numato")
    parser.add_argument("--no-ident-version",    action="store_false",             help="Disable build time output")
    parser.add_argument("--with-jtagbone",       action="store_true",              help="Enable Jtagbone support")
    parser.add_argument("--with-mapped-flash",   action="store_true",              help="Enable Memory Mapped Flash")
    builder_args(parser)
    soc_core_args(parser)
    vivado_build_args(parser)
    args = parser.parse_args()

    assert not (args.with_etherbone and args.eth_dynamic_ip)

    soc = BaseSoC(
        variant           = args.variant,
        toolchain         = args.toolchain,
        sys_clk_freq      = int(float(args.sys_clk_freq)),
        with_ethernet     = args.with_ethernet,
        with_etherbone    = args.with_etherbone,
        eth_ip            = args.eth_ip,
        eth_dynamic_ip    = args.eth_dynamic_ip,
        ident_version     = args.no_ident_version,
        with_jtagbone     = args.with_jtagbone,
        with_mapped_flash = args.with_mapped_flash,
        **soc_core_argdict(args)
    )
    if args.sdcard_adapter == "numato":
        soc.platform.add_extension(arty._numato_sdcard_pmod_io)
    else:
        soc.platform.add_extension(arty._sdcard_pmod_io)
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()
    builder = Builder(soc, csr_json=f"build/digilent_arty/csr.json")
    builder_kwargs = vivado_build_argdict(args) if args.toolchain == "vivado" else {}
    builder.build(**builder_kwargs, run=args.build)

    soc.generate_dts("digilent_arty")
    soc.compile_dts("digilent_arty")
    soc.combine_dtb("digilent_arty")

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
