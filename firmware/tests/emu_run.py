#!/usr/bin/env python3
# Runs a micro-controller image (Kalico's C klipper.elf or kaliru-mcu) in the
# Unicorn emulator with a small, deterministic model of the chip and drives it
# with a mcu_run.py command script. Prints the answers (clocks unmasked: the
# emulated time is the same for both images when they run the same code) and
# writes every peripheral register access to a log. Run it on both images and
# diff both outputs.
#
# usage: emu_run.py <elf> <klipper.dict> <config dir> <script> <access log> [--kalico <dir>]
#
# The model (register addresses, bits and irq numbers come from the generated
# chip header src/<board>/chip/<mcu>.rs):
# - time is the board's clock (the DWT cycle counter, or the timer counter of
#   the boards that use one); it advances TICK cycles on each read of it (so
#   it depends only on what the code does), and jumps to the next timer
#   expiry when the cpu waits for an interrupt;
# - interrupts are taken only at that wait (wfi), so the order of events does
#   not depend on instruction counts;
# - the serial port has a transmitter that is always ready and a receiver fed
#   from the script;
# - the other peripheral registers are memory, with the status bits the clock
#   setup waits for set; a register read POLL times in a row (a wait for
#   status bits the model does not set) then reads inverted, then each single
#   bit, then 0, so the wait ends the same way for any image.
import json
import os
import re
import struct
import subprocess
import sys

import unicorn
import unicorn.arm_const as A

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcu_run  # noqa: E402

TICK = 16  # cycles per clock read
POLL = 1000  # reads of a register in a row that end a wait for it
EXC_RETURN = 0xFFFFFFF9
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def autoconf(path):
    cfg = {}
    for line in open(os.path.join(path, "autoconf.h")):
        m = re.match(r"#define (CONFIG_\w+) (.*)$", line)
        if m:
            v = m.group(2).strip()
            cfg[m.group(1)] = v.strip('"') if v.startswith('"') else int(v, 0) if re.fullmatch(r"-?(0x[0-9a-fA-F]+|\d+)", v) else v
    return cfg


class Chip:
    """the values of a generated chip header: constants, peripheral
    addresses, struct field offsets"""

    def __init__(self, path):
        text = open(path).read()
        self.const = {}
        for m in re.finditer(r"^pub const (\w+): (?:u32|i32|IRQn_Type) = (-?(?:0x[0-9a-f]+|\d+));", text, re.M):
            self.const[m.group(1)] = int(m.group(2), 0)
        self.ptr, self.ptype = {}, {}
        for m in re.finditer(r"^pub const (\w+): \*mut (\w+) = (0x[0-9a-f]+) as", text, re.M):
            self.ptr[m.group(1)] = int(m.group(3), 16)
            self.ptype[m.group(1)] = m.group(2)
        self.off = {}
        for m in re.finditer(r"offset_of!\((\w+), (\w+)\) == (\d+)\)", text):
            self.off[(m.group(1), m.group(2))] = int(m.group(3))
        # (hc32: the bit fields of the `REG_f` views)
        self.field = {}
        for mod in re.finditer(r"^pub mod (\w+) \{\n(.*?)^\}", text, re.M | re.S):
            for m in re.finditer(r"pub const (\w+): Field = Field \{ pos: (\d+), width: (\d+) \}", mod.group(2)):
                self.field[(mod.group(1), m.group(1))] = (int(m.group(2)), int(m.group(3)))
        # (the other members of anonymous unions: accessor methods)
        for impl in re.finditer(r"^impl (\w+) \{\n(.*?)^\}", text, re.M | re.S):
            for m in re.finditer(r"pub fn (\w+)\(&self\)[^{]*\{[^}]*?\.add\((\d+)\)", impl.group(2)):
                self.off[(impl.group(1), m.group(1))] = int(m.group(2))

    def reg(self, periph, struct_name, field):
        return self.ptr[periph] + self.off[(struct_name, field)]

    def at(self, periph, field):
        """the address of a register of a peripheral"""
        return self.ptr[periph] + self.off[(self.ptype[periph], field)]


def load_segments(path):
    """the (address, data) of the ELF's loaded segments, at their load
    addresses"""
    data = open(path, "rb").read()
    phoff, = struct.unpack_from("<I", data, 28)
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    out = []
    for i in range(phnum):
        p_type, p_offset, p_vaddr, p_paddr, p_filesz = struct.unpack_from("<IIIII", data, phoff + i * phentsize)
        if p_type == 1 and p_filesz:
            out.append((p_paddr, data[p_offset : p_offset + p_filesz]))
    return out


def wait_points(path):
    """the addresses of the wfi instructions (where the cpu waits for an
    interrupt: irq_wait), or on the Cortex-M7 the nop of irq_wait's
    `cpsie i; nop; cpsid i`"""
    dis = subprocess.run(["arm-none-eabi-objdump", "-d", path], capture_output=True, text=True).stdout
    wfi = [int(m.group(1), 16) for m in re.finditer(r"^\s*([0-9a-f]+):\s+bf30\s+wfi", dis, re.M)]
    nop = [int(m.group(1), 16) for m in re.finditer(
        r"^\s*[0-9a-f]+:\s+b662\s+cpsie\s+i\s*\n\s*([0-9a-f]+):\s+bf00\s+nop\s*\n\s*[0-9a-f]+:\s+b672\s+cpsid\s+i", dis, re.M)]
    return wfi + nop


def wrapped_before(a, b):
    """a before b, as 32 bit times"""
    return ((a - b) & 0xFFFFFFFF) >= 0x80000000


# ---- time sources


class DwtTime:
    """generic/armcm_timer.c: the DWT cycle counter, SysTick one-shots
    (LOAD = ticks; VAL = 0; LOAD = 0)"""

    def __init__(self, chip):
        self.cyccnt = chip.reg("DWT", "DWT_Type", "CYCCNT")
        self.ctrl = chip.reg("SysTick", "SysTick_Type", "CTRL")
        self.load = chip.reg("SysTick", "SysTick_Type", "LOAD")
        self.val = chip.reg("SysTick", "SysTick_Type", "VAL")
        self.due = None

    def read(self, emu, addr):
        if addr == self.cyccnt:
            return emu.tick()
        return None

    def write(self, emu, addr, value):
        if addr == self.cyccnt:
            emu.time = value
            return True
        if addr == self.val:
            load = emu.get(self.load, 4) & 0xFFFFFF
            self.due = (emu.time + load) & 0xFFFFFFFF if load else None
        return False

    def expiry(self, emu):
        """(the time and exception number of the next timer interrupt)"""
        if self.due is not None and emu.get(self.ctrl, 4) & 3 == 3:
            return self.due, 15
        return None

    def taken(self, emu):
        self.due = None


class Stm32TimTime:
    """stm32/stm32f0_timer.c: TIM2 (32 bit) or TIM3 (16 bit) counts the
    clock, its CCR1 match is the timer interrupt"""

    def __init__(self, chip):
        c = chip.const
        tim = "TIM2" if "TIM2" in chip.ptr else "TIM3"
        self.mask = 0xFFFFFFFF if tim == "TIM2" else 0xFFFF
        self.cnt = chip.reg(tim, "TIM_TypeDef", "CNT")
        self.ccr1 = chip.reg(tim, "TIM_TypeDef", "CCR1")
        self.dier = chip.reg(tim, "TIM_TypeDef", "DIER")
        self.irq = c[tim + "_IRQn"] if tim + "_IRQn" in c else c["TIM3_TIM4_IRQn"]
        self.cc1ie = c["TIM_DIER_CC1IE"]
        self.due = None

    def read(self, emu, addr):
        if addr == self.cnt:
            return emu.tick() & self.mask
        return None

    def write(self, emu, addr, value):
        if addr == self.cnt:
            emu.time = value
            return True
        if addr == self.ccr1:
            self.due = (emu.time + ((value - emu.time) & self.mask)) & 0xFFFFFFFF
        return False

    def expiry(self, emu):
        if self.due is not None and emu.get(self.dier, 4) & self.cc1ie and self.irq in emu.nvic_enabled:
            return self.due, 16 + self.irq
        return None

    def taken(self, emu):
        self.due = None


# ---- boards


class Stm32:
    """the stm32 parts of the model: clock setup status bits, the usart"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "stm32", "chip", cfg["CONFIG_MCU"] + ".rs"))
        c = chip.const
        cpu = {
            "CONFIG_MACH_STM32F0": A.UC_CPU_ARM_CORTEX_M0, "CONFIG_MACH_STM32G0": A.UC_CPU_ARM_CORTEX_M0,
            "CONFIG_MACH_STM32F1": A.UC_CPU_ARM_CORTEX_M3, "CONFIG_MACH_STM32F2": A.UC_CPU_ARM_CORTEX_M3,
            "CONFIG_MACH_STM32F7": A.UC_CPU_ARM_CORTEX_M7, "CONFIG_MACH_STM32H7": A.UC_CPU_ARM_CORTEX_M7,
        }
        self.cpu = next((v for k, v in cpu.items() if cfg.get(k)), A.UC_CPU_ARM_CORTEX_M4)
        self.time = Stm32TimTime(chip) if cfg.get("CONFIG_MACH_STM32F0") or cfg.get("CONFIG_MACH_STM32G0") else DwtTime(chip)
        rcc_cr = chip.reg("RCC", "RCC_TypeDef", "CR")
        rcc_cfgr = chip.reg("RCC", "RCC_TypeDef", "CFGR")
        ready = 0
        for n in ("RCC_CR_HSIRDY", "RCC_CR_HSERDY", "RCC_CR_PLLRDY"):
            ready |= c.get(n, 0)
        # (register address: function of the stored value giving the value read)
        sws = c["RCC_CFGR_SWS_Pos"] - c["RCC_CFGR_SW_Pos"]
        self.read_rules = {
            rcc_cr: lambda v: v | ready,
            rcc_cfgr: lambda v: (v & ~c["RCC_CFGR_SWS_Msk"]) | ((v & c["RCC_CFGR_SW_Msk"]) << sws),
        }
        n = [k for k in ("USART1", "USART2", "USART3") if cfg.get("CONFIG_STM32_SERIAL_" + k)]
        if not n:
            sys.exit("only the stm32 usart serial port is modelled")
        usart = n[0]
        # (the irq a usart shares on some chips, as stm32f0_serial.c names it)
        shared = {"USART2": ["USART2_LPUART2_IRQn"], "USART3": ["USART3_4_5_6_LPUART1_IRQn", "USART3_4_5_6_IRQn"]}
        self.irq = next(c[x] for x in [usart + "_IRQn"] + shared.get(usart, []) if x in c)
        u = chip.ptr[usart]
        off = lambda f: u + chip.off[("USART_TypeDef", f)]  # noqa: E731
        bit = lambda *names: next(c[x] for x in names if x in c)  # noqa: E731
        self.cr1 = off("CR1")
        if ("USART_TypeDef", "SR") in chip.off:
            # (stm32f1/f2/f4: SR, DR)
            self.sr = off("SR")
            self.dr = self.tdr = off("DR")
            self.RXNE, self.TXE, self.TC = c["USART_SR_RXNE"], c["USART_SR_TXE"], c["USART_SR_TC"]
        else:
            # (the others: ISR, RDR, TDR; the fifo names on g0/h7, see stm32f0_serial.c)
            self.sr = off("ISR")
            self.dr, self.tdr = off("RDR"), off("TDR")
            self.RXNE = bit("USART_ISR_RXNE", "USART_ISR_RXNE_RXFNE")
            self.TXE = bit("USART_ISR_TXE", "USART_ISR_TXE_TXFNF")
            self.TC = c["USART_ISR_TC"]
        self.RXNEIE = bit("USART_CR1_RXNEIE", "USART_CR1_RXNEIE_RXFNEIE")
        self.TXEIE = bit("USART_CR1_TXEIE", "USART_CR1_TXEIE_TXFNFIE")

    def uart_read(self, emu, addr):
        if addr == self.sr:
            return self.TXE | self.TC | (self.RXNE if emu.rx else 0)
        if addr == self.dr:
            return emu.rx_byte()
        return None

    def uart_write(self, emu, addr, value):
        if addr == self.tdr:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def uart_irq(self, emu):
        cr1 = emu.get(self.cr1, 4)
        if (emu.rx and cr1 & self.RXNEIE) or cr1 & self.TXEIE:
            return self.irq
        return None


class Lpc176x:
    """lpc176x: the uart (serial.c: LSR THRE, IIR, IER), DWT time; its gpio
    block sits at 0x2009c000"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "lpc176x", "chip", "lpc17xx.rs"))
        self.cpu = A.UC_CPU_ARM_CORTEX_M3
        self.time = DwtTime(chip)
        self.mmio = [(chip.ptr["LPC_GPIO0"] & ~0xFFFF, 0x10000)]
        self.read_rules = {}
        uart = "UART0" if cfg.get("CONFIG_LPC_SERIAL_UART0_P03_P02") else "UART3"
        self.irq = chip.const[uart + "_IRQn"]
        p = "LPC_" + uart
        self.rbr = chip.at(p, "RBR")
        self.ier, self.iir = chip.at(p, "IER"), chip.at(p, "IIR")
        self.lcr, self.lsr = chip.at(p, "LCR"), chip.at(p, "LSR")

    def uart_read(self, emu, addr):
        if addr == self.lsr:
            # (THRE, and RDR when a byte is there)
            return (1 << 5) | (1 if emu.rx else 0)
        if addr == self.iir:
            ier = emu.get(self.ier, 4)
            return 0x04 if emu.rx and ier & 1 else 0x02 if ier & 2 else 0x01
        if addr == self.rbr and not emu.get(self.lcr, 1) & 0x80:
            return emu.rx_byte()
        return None

    def uart_write(self, emu, addr, value):
        # (offset 0 is THR, or DLL with the DLAB bit set)
        if addr == self.rbr and not emu.get(self.lcr, 1) & 0x80:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def uart_irq(self, emu):
        ier = emu.get(self.ier, 4)
        if (emu.rx and ier & 1) or ier & 2:
            return self.irq
        return None


class Atsam:
    """atsam: the uart (serial.c: SR RXRDY / TXRDY, IER / IDR mask), DWT
    time"""

    def __init__(self, cfg):
        mcu = cfg["CONFIG_MCU"]
        chip = self.chip = Chip(os.path.join(ROOT, "src", "atsam", "chip", mcu + ".rs"))
        c = chip.const
        self.cpu = A.UC_CPU_ARM_CORTEX_M3 if cfg.get("CONFIG_MACH_SAM3X") else (
            A.UC_CPU_ARM_CORTEX_M7 if cfg.get("CONFIG_MACH_SAME70") else A.UC_CPU_ARM_CORTEX_M4)
        self.time = DwtTime(chip)
        self.read_rules = {}
        # (serial.c: sam3x UART, sam4s UART1, sam4e UART0)
        port, irq = ("UART", "UART_IRQn") if cfg.get("CONFIG_MACH_SAM3X") else (
            ("UART1", "UART1_IRQn") if cfg.get("CONFIG_MACH_SAM4S") else ("UART0", "UART0_IRQn"))
        self.irq = c[irq]
        self.sr, self.rhr, self.thr = chip.at(port, "UART_SR"), chip.at(port, "UART_RHR"), chip.at(port, "UART_THR")
        self.ier, self.idr, self.imr = chip.at(port, "UART_IER"), chip.at(port, "UART_IDR"), chip.at(port, "UART_IMR")
        self.RXRDY, self.TXRDY, self.TXEMPTY = c["UART_SR_RXRDY"], c["UART_SR_TXRDY"], c["UART_SR_TXEMPTY"]
        self.mask = 0

    def uart_read(self, emu, addr):
        if addr == self.sr:
            return self.TXRDY | self.TXEMPTY | (self.RXRDY if emu.rx else 0)
        if addr == self.rhr:
            return emu.rx_byte()
        if addr == self.imr:
            return self.mask
        return None

    def uart_write(self, emu, addr, value):
        if addr == self.thr:
            emu.tx += bytes([value & 0xFF])
            return True
        if addr == self.ier:
            self.mask |= value
            return True
        if addr == self.idr:
            self.mask &= ~value
            return True
        return False

    def uart_irq(self, emu):
        if (emu.rx and self.mask & self.RXRDY) or self.mask & self.TXRDY:
            return self.irq
        return None


class TcTime:
    """atsamd/timer.c (samd21, samc21): a 32 bit TC counts the clock, its
    CC[0] match is the timer interrupt"""

    def __init__(self, chip, tc):
        self.count = chip.ptr[tc] + chip.off[("TcCount32", "COUNT")]
        self.cc0 = chip.ptr[tc] + chip.off[("TcCount32", "CC")]
        self.intenset = chip.ptr[tc] + chip.off[("TcCount32", "INTENSET")]
        self.irq = chip.const[tc + "_IRQn"]
        self.mc0 = chip.const["TC_INTENSET_MC0"]
        self.due = None

    def read(self, emu, addr):
        if addr == self.count:
            return emu.tick()
        return None

    def write(self, emu, addr, value):
        if addr == self.count:
            emu.time = value
            return True
        if addr == self.cc0:
            self.due = value
        return False

    def expiry(self, emu):
        if self.due is not None and emu.get(self.intenset, 1) & self.mc0 and self.irq in emu.nvic_enabled:
            return self.due, 16 + self.irq
        return None

    def taken(self, emu):
        self.due = None


class Atsamd:
    """atsamd: the SERCOM0 usart (serial.c: INTFLAG DRE / RXC, INTENSET /
    INTENCLR mask), the TC (samd21, samc21) or DWT (samd51, same5x) time"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "atsamd", "chip", cfg["CONFIG_MCU"] + ".rs"))
        c = chip.const
        samx5 = cfg.get("CONFIG_MACH_SAMX5")
        self.cpu = A.UC_CPU_ARM_CORTEX_M4 if samx5 else A.UC_CPU_ARM_CORTEX_M0
        self.time = DwtTime(chip) if samx5 else TcTime(chip, "TC0" if cfg.get("CONFIG_MACH_SAMC21") else "TC4")
        self.read_rules = {}
        u = lambda f: chip.ptr["SERCOM0"] + chip.off[("SercomUsart", f)]  # noqa: E731
        self.intflag, self.data = u("INTFLAG"), u("DATA")
        self.intenset, self.intenclr = u("INTENSET"), u("INTENCLR")
        self.DRE, self.TXC, self.RXC = c["SERCOM_USART_INTFLAG_DRE"], c["SERCOM_USART_INTFLAG_TXC"], c["SERCOM_USART_INTFLAG_RXC"]
        # (samd51: one irq per flag, serial.c enables them all)
        self.dre_irq = c["SERCOM0_0_IRQn"] if samx5 else c["SERCOM0_IRQn"]
        self.rxc_irq = c["SERCOM0_2_IRQn"] if samx5 else c["SERCOM0_IRQn"]
        self.mask = 0

    def uart_read(self, emu, addr):
        if addr == self.intflag:
            return self.DRE | self.TXC | (self.RXC if emu.rx else 0)
        if addr == self.data:
            return emu.rx_byte()
        if addr in (self.intenset, self.intenclr):
            return self.mask
        return None

    def uart_write(self, emu, addr, value):
        if addr == self.data:
            emu.tx += bytes([value & 0xFF])
            return True
        if addr == self.intenset:
            self.mask |= value
            return True
        if addr == self.intenclr:
            self.mask &= ~value
            return True
        return False

    def uart_irq(self, emu):
        if emu.rx and self.mask & self.RXC:
            return self.rxc_irq
        if self.mask & self.DRE:
            return self.dre_irq
        return None


class Hc32f460:
    """hc32f460: the usart (serial.c: SR RXNE / TXE / TC, DR RDR / TDR, CR1
    RIE / TXEIE / TCIE; its irqs are the ones IrqRegistration routed the
    usart sources to), DWT time"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "hc32f460", "chip", "hc32f460.rs"))
        c = chip.const
        self.cpu = A.UC_CPU_ARM_CORTEX_M4
        self.time = DwtTime(chip)
        self.read_rules = {}
        n = 3 if cfg.get("CONFIG_HC32F460_SERIAL_PH2_PB10") else 1
        p = "M4_USART%d" % n
        self.sr, self.dr, self.cr1 = chip.at(p, "SR"), chip.at(p, "DR"), chip.at(p, "CR1")
        f = lambda r, x: chip.field[("stc_usart_%s_field_t" % r, x)][0]  # noqa: E731
        self.RXNE, self.TC, self.TXE = 1 << f("sr", "RXNE"), 1 << f("sr", "TC"), 1 << f("sr", "TXE")
        self.RDR = f("dr", "RDR")
        self.RIE, self.TXEIE, self.TCIE = 1 << f("cr1", "RIE"), 1 << f("cr1", "TXEIE"), 1 << f("cr1", "TCIE")
        self.src = {k: c["INT_USART%d_%s" % (n, k)] for k in ("RI", "TI", "TCI")}
        self.sel0 = chip.at("M4_INTC", "SEL0")

    def routed(self, emu, source):
        """the irq IrqRegistration selected for an interrupt source"""
        for i in range(32):
            if emu.get(self.sel0 + 4 * i, 4) & 0x1FF == source:
                return i
        return None

    def uart_read(self, emu, addr):
        if addr == self.sr:
            return self.TXE | self.TC | (self.RXNE if emu.rx else 0)
        if addr == self.dr:
            return emu.rx_byte() << self.RDR
        return None

    def uart_write(self, emu, addr, value):
        if addr == self.dr:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def uart_irq(self, emu):
        cr1 = emu.get(self.cr1, 4)
        for cond, src in ((emu.rx and cr1 & self.RIE, "RI"), (cr1 & self.TXEIE, "TI"), (cr1 & self.TCIE, "TCI")):
            if cond:
                return self.routed(emu, self.src[src])
        return None


class RpTime:
    """rp2040/timer.c: the TIMER counts the clock (timerawl), alarm[0] is the
    timer interrupt"""

    def __init__(self, chip):
        self.timerawl = chip.at("timer_hw", "timerawl")
        self.alarm0 = chip.at("timer_hw", "alarm")
        self.inte = chip.at("timer_hw", "inte")
        self.irq = chip.const["TIMER_IRQ_0_IRQn"]
        self.due = None

    def read(self, emu, addr):
        if addr == self.timerawl:
            return emu.tick()
        return None

    def write(self, emu, addr, value):
        if addr == self.alarm0:
            self.due = value
        return False

    def expiry(self, emu):
        if self.due is not None and emu.get(self.inte, 4) & 1 and self.irq in emu.nvic_enabled:
            return self.due, 16 + self.irq
        return None

    def taken(self, emu):
        self.due = None


class Rp2040:
    """rp2040: the PL011 uart (serial.c: fr RXFE, imsc, mis), the TIMER;
    its SIO block, and the xor / set / clear aliases of the peripheral
    registers (hw_set_alias)"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "rp2040", "chip", "rp2040.rs"))
        c = chip.const
        self.cpu = A.UC_CPU_ARM_CORTEX_M0
        self.time = RpTime(chip)
        self.mmio = [(chip.ptr["sio_hw"], 0x1000)]
        self.read_rules = {}
        n = 1 if any(k.startswith("CONFIG_RPXXXX_SERIAL_UART1") and v == 1 for k, v in cfg.items()) else 0
        p = "uart%d_hw" % n
        self.dr, self.fr = chip.at(p, "dr"), chip.at(p, "fr")
        self.imsc, self.mis = chip.at(p, "imsc"), chip.at(p, "mis")
        self.irq = c["UART%d_IRQ_IRQn" % n]
        self.RXFE = c["UART_UARTFR_RXFE_BITS"]
        self.RX = c["UART_UARTIMSC_RXIM_BITS"] | c["UART_UARTIMSC_RTIM_BITS"]

    def alias(self, addr):
        """(the register and operation of an alias address)"""
        if 0x40000000 <= addr < 0x60000000 and addr & 0x3000:
            return addr & ~0x3000, (addr >> 12) & 3
        return None

    def uart_read(self, emu, addr):
        if addr == self.fr:
            return 0 if emu.rx else self.RXFE
        if addr == self.dr:
            return emu.rx_byte()
        if addr == self.mis:
            return emu.get(self.imsc, 4) & (self.RX if emu.rx else 0)
        return None

    def uart_write(self, emu, addr, value):
        if addr == self.dr:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def uart_irq(self, emu):
        if emu.rx and emu.get(self.imsc, 4) & self.RX:
            return self.irq
        return None


BOARDS = {"CONFIG_MACH_STM32": Stm32, "CONFIG_MACH_LPC176X": Lpc176x, "CONFIG_MACH_ATSAM": Atsam,
          "CONFIG_MACH_ATSAMD": Atsamd, "CONFIG_MACH_HC32F460": Hc32f460, "CONFIG_MACH_RP2040": Rp2040}


class Emu(mcu_run.Mcu):
    mask_clocks = False

    def __init__(self, elf, dictionary, config, log):
        self.dictionary = dictionary
        cfg = autoconf(config)
        board = next((b for k, b in BOARDS.items() if cfg.get(k)), None)
        if board is None:
            sys.exit("no model of this board")
        b = self.board = board(cfg)
        uc = self.uc = unicorn.Uc(unicorn.UC_ARCH_ARM, unicorn.UC_MODE_THUMB | unicorn.UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(b.cpu)
        # (memory: the image's flash, the ram, the peripherals and the system
        # control space)
        ram = (cfg["CONFIG_RAM_START"], cfg["CONFIG_RAM_SIZE"])
        segs = [(a, d) for a, d in load_segments(elf) if not ram[0] <= a < ram[0] + ram[1]]
        lo = min(a for a, d in segs) & ~0xFFFF
        hi = (max(a + len(d) for a, d in segs) + 0xFFFF) & ~0xFFFF
        uc.mem_map(lo, hi - lo)
        uc.mem_map(ram[0] & ~0xFFF, (ram[1] + (ram[0] & 0xFFF) + 0xFFF) & ~0xFFF)
        for a, d in segs:
            uc.mem_write(a, d)
        for a, n in [(0x40000000, 0x20000000), (0xE0000000, 0x00100000)] + getattr(b, "mmio", []):
            uc.mmio_map(a, n, self.mmio_read, a, self.mmio_write, a)
        vt = cfg["CONFIG_FLASH_APPLICATION_ADDRESS"]
        sp, reset = struct.unpack("<II", uc.mem_read(vt, 8))
        self.vectors = vt
        uc.reg_write(A.UC_ARM_REG_SP, sp)
        self.pc = reset
        for a in wait_points(elf):
            uc.hook_add(unicorn.UC_HOOK_CODE, self.on_wait, begin=a, end=a)
        # (a branch to EXC_RETURN raises the cpu's exception exit)
        uc.hook_add(unicorn.UC_HOOK_INTR, self.on_intr)
        # (an access outside the model's memory: the crash's address)
        self.bad_access = None
        uc.hook_add(unicorn.UC_HOOK_MEM_INVALID, self.on_invalid)
        self.mem = {}
        self.log = open(log, "w")
        self.time = 0
        self.nvic_enabled = set()
        self.active = 0
        self.rx = b""
        self.tx = b""
        self.poll_addr, self.poll_count = None, 0
        self.stopped = None
        self.crashed = None
        c = b.chip
        self.icsr = c.reg("SCB", "SCB_Type", "ICSR")
        self.iser = c.reg("NVIC", "NVIC_Type", "ISER")
        self.icer = c.reg("NVIC", "NVIC_Type", "ICER")
        self.clock_freq = cfg["CONFIG_CLOCK_FREQ"]
        # (mcu_run.Mcu state)
        self.last_clock = 0
        self.parser = mcu_run.msgproto.MessageParser()
        self.seq = 0
        self.buf = b""
        self.hwlog = None
        self.run()

    # ---- memory mapped registers

    def get(self, addr, size):
        return sum(self.mem.get(addr + i, 0) << (8 * i) for i in range(size))

    def put(self, addr, size, value):
        for i in range(size):
            self.mem[addr + i] = (value >> (8 * i)) & 0xFF

    def tick(self):
        """a read of the clock"""
        self.time = (self.time + TICK) & 0xFFFFFFFF
        return self.time

    def rx_byte(self):
        v = self.rx[0] if self.rx else 0
        self.rx = self.rx[1:]
        return v

    def mmio_read(self, uc, offset, size, base):
        addr = base + offset
        b = self.board
        v = b.time.read(self, addr)
        if v is None:
            v = b.uart_read(self, addr)
        if v is not None:
            pass
        elif addr == self.icsr:
            v = (self.get(addr, 4) & ~0x1FF) | self.active
        elif self.iser <= addr < self.iser + 32:
            n = (addr - self.iser) * 8
            v = sum(1 << (i - n) for i in self.nvic_enabled if n <= i < n + 32)
        else:
            v = self.get(addr, size)
            rule = b.read_rules.get(addr)
            if rule:
                v = rule(v)
            if addr == self.poll_addr:
                self.poll_count += 1
                n = self.poll_count - POLL
                if n >= 0:
                    n %= 8 * size + 2
                    v = ~v if n == 0 else 0 if n == 8 * size + 1 else 1 << (n - 1)
            else:
                self.poll_addr, self.poll_count = addr, 1
        v &= (1 << (8 * size)) - 1
        self.log.write("R %08x %d %x\n" % (addr, size, v))
        return v

    def mmio_write(self, uc, offset, size, value, base):
        addr = base + offset
        b = self.board
        self.log.write("W %08x %d %x\n" % (addr, size, value))
        self.poll_addr = None
        if b.uart_write(self, addr, value) or b.time.write(self, addr, value):
            return
        if self.iser <= addr < self.iser + 32:
            n = (addr - self.iser) * 8
            self.nvic_enabled |= {n + i for i in range(32) if value >> i & 1}
            return
        if self.icer <= addr < self.icer + 32:
            n = (addr - self.icer) * 8
            self.nvic_enabled -= {n + i for i in range(32) if value >> i & 1}
            return
        alias = getattr(b, "alias", lambda a: None)(addr)
        if alias:
            # (rp2040: an xor / set / clear alias of a register)
            reg, op = alias
            old = self.get(reg, 4)
            self.put(reg, 4, old ^ value if op == 1 else old | value if op == 2 else old & ~value)
            return
        self.put(addr, size, value)

    # ---- exceptions

    def enter(self, num, ret):
        uc = self.uc
        sp = uc.reg_read(A.UC_ARM_REG_SP)
        xpsr = uc.reg_read(A.UC_ARM_REG_XPSR)
        if sp & 4:
            sp -= 4
            xpsr |= 1 << 9
        regs = [uc.reg_read(r) for r in (A.UC_ARM_REG_R0, A.UC_ARM_REG_R1, A.UC_ARM_REG_R2, A.UC_ARM_REG_R3,
                                          A.UC_ARM_REG_R12, A.UC_ARM_REG_LR)]
        sp -= 32
        uc.mem_write(sp, struct.pack("<8I", *regs, ret, xpsr | (1 << 24)))
        uc.reg_write(A.UC_ARM_REG_SP, sp)
        uc.reg_write(A.UC_ARM_REG_LR, EXC_RETURN)
        self.active = num
        handler, = struct.unpack("<I", uc.mem_read(self.vectors + 4 * num, 4))
        return handler

    def on_intr(self, uc, intno, user):
        pc = uc.reg_read(A.UC_ARM_REG_PC)
        if pc & ~1 != EXC_RETURN & ~1:
            self.log.flush()
            sys.exit("cpu exception %d at pc %x" % (intno, pc))
        sp = uc.reg_read(A.UC_ARM_REG_SP)
        r0, r1, r2, r3, r12, lr, pc, xpsr = struct.unpack("<8I", uc.mem_read(sp, 32))
        for r, v in zip((A.UC_ARM_REG_R0, A.UC_ARM_REG_R1, A.UC_ARM_REG_R2, A.UC_ARM_REG_R3, A.UC_ARM_REG_R12,
                         A.UC_ARM_REG_LR), (r0, r1, r2, r3, r12, lr)):
            uc.reg_write(r, v)
        sp += 32 + (4 if xpsr & (1 << 9) else 0)
        uc.reg_write(A.UC_ARM_REG_SP, sp)
        self.active = 0
        uc.reg_write(A.UC_ARM_REG_PC, pc | 1)

    def on_invalid(self, uc, access, address, size, value, user):
        self.bad_access = address
        return False

    def on_wait(self, uc, address, size, user):
        self.stopped = address
        uc.emu_stop()

    def pending(self, advance_until):
        """the exception to take now, if any (advancing the time to the next
        timer expiry if it is not after `advance_until`)"""
        b = self.board
        irq = b.uart_irq(self)
        if irq is not None and irq in self.nvic_enabled:
            return 16 + irq
        e = b.time.expiry(self)
        if e is not None:
            due, num = e
            if not wrapped_before(self.time, due):
                b.time.taken(self)
                return num
            if advance_until is not None and not wrapped_before(advance_until, due):
                self.time = due
                b.time.taken(self)
                return num
        return None

    def run(self, advance_until=None):
        """run until the cpu waits with nothing to do (advancing the time
        through the timer expiries up to `advance_until`)"""
        uc = self.uc
        if self.crashed:
            return
        for _ in range(1000000):
            self.stopped = None
            try:
                uc.emu_start(self.pc | 1, 0xFFFFFFFE, count=50000000)
            except unicorn.UcError as e:
                # (the image crashed: an outcome to compare, the run ends)
                # (the access, not the pc: the images' code addresses differ)
                self.crashed = "%s%s" % (e, " at %08x" % self.bad_access if self.bad_access is not None else "")
                self.log.write("# crash %s\n" % self.crashed)
                return
            if self.stopped is None:
                sys.exit("no wait for interrupts in 50M instructions (pc %x)" % uc.reg_read(A.UC_ARM_REG_PC))
            self.pc = self.stopped
            num = self.pending(advance_until)
            if num is None:
                # (the time the quiet period ends at)
                if advance_until is not None and wrapped_before(self.time, advance_until):
                    self.time = advance_until
                return
            self.pc = self.enter(num, (self.pc + 2) | 1) & ~1

    # ---- mcu_run.Mcu interface

    def close(self):
        self.log.close()
        return ""

    def send_raw(self, data):
        self.rx += bytes(data)
        self.run()

    def read_blocks(self, until_ack, quiet=0.01, timeout=2.0):
        # (the answers so far, then those of a quiet period)
        self.run((self.time + int(quiet * self.clock_freq)) & 0xFFFFFFFF)
        self.buf += self.tx
        self.tx = b""
        out = []
        if self.crashed:
            out.append(("crash", "(see the log)"))
        while True:
            n = self.parser.check_packet(self.buf)
            if n == 0:
                break
            if n < 0:
                out.append(("garbage", self.buf[:1].hex()))
                self.buf = self.buf[1:]
                continue
            out.append(("block", self.buf[:n]))
            self.buf = self.buf[n:]
        return out

    def wait(self, seconds):
        return self.read_blocks(False, quiet=seconds)

    def identify(self):
        # (the build's data dictionary, not fetched with identify commands:
        # the C and Rust dictionaries differ in their version fields, so the
        # transfer would differ in length, and so the sequence numbers and
        # times after it)
        raw = open(self.dictionary, "rb").read()
        self.parser.process_identify(raw, decompress=False)
        d = json.loads(raw)
        for k in ("version", "build_versions"):
            d.pop(k, None)
        return json.dumps(d, sort_keys=True), len(raw)

    def prepare(self, line):
        # (the linux scripts' names: gpioN / analogN the board's pin number N,
        # spidevN.M / i2c.N its spi / i2c bus number N)
        enums = self.parser.get_enumerations()
        # (a board without hardware spi / i2c buses, hc32f460: software buses
        # on fixed pins instead)
        if not {"spi_bus", "i2c_bus", "bus"} & set(enums):
            line = re.sub(r"^spi_set_bus oid=(\d+) spi_bus=\S+ mode=(\d+) rate=\d+$",
                          r"spi_set_sw_bus oid=\1 miso_pin=gpio62 mosi_pin=gpio63 sclk_pin=gpio64 mode=\2 pulse_ticks=10", line)
            line = re.sub(r"^i2c_set_bus oid=(\d+) i2c_bus=\S+ rate=\d+ address=(\d+)$",
                          r"i2c_set_sw_bus oid=\1 scl_pin=gpio60 sda_pin=gpio61 pulse_ticks=10 address=\2", line)

        def name(enum, value, default):
            return {v: k for k, v in enums.get(enum, {}).items()}.get(value, default)

        # (a number the board has none for: the one at that index)
        def value(enum, n, default):
            names = sorted(enums.get(enum, {}).items(), key=lambda kv: kv[1])
            return name(enum, n, names[n % len(names)][0] if names else default)

        line = re.sub(r"\b(?:gpio|analog)(\d+)\b", lambda m: value("pin", int(m.group(1)), m.group(0)), line)
        # (atsamd: one "bus" enumeration of its sercoms)
        spi, i2c = [e if e in enums else "bus" for e in ("spi_bus", "i2c_bus")]
        line = re.sub(r"\bspidev(\d+)\.\d+\b", lambda m: value(spi, int(m.group(1)), m.group(0)), line)
        line = re.sub(r"\bi2c\.(\d+)\b", lambda m: value(i2c, int(m.group(1)), m.group(0)), line)
        self.log.write("# %s\n" % line)
        return super().prepare(line)


def main():
    elf, dictionary, config, script, log = sys.argv[1:6]
    emu = Emu(elf, dictionary, config, log)
    out, err = mcu_run.run_script(emu, script)
    print("\n".join(out))


if __name__ == "__main__":
    main()
