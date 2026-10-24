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
    addresses, and at their run addresses too where those differ (same70:
    the flash is also seen at 0, where the code is linked)"""
    data = open(path, "rb").read()
    phoff, = struct.unpack_from("<I", data, 28)
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    out = []
    for i in range(phnum):
        p_type, p_offset, p_vaddr, p_paddr, p_filesz = struct.unpack_from("<IIIII", data, phoff + i * phentsize)
        if p_type == 1 and p_filesz:
            out.append((p_paddr, data[p_offset : p_offset + p_filesz]))
            if p_vaddr != p_paddr:
                out.append((p_vaddr, data[p_offset : p_offset + p_filesz]))
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


class Ported:
    """a board whose serial port is a `port` object (a uart or a usb device
    with its host)"""

    def uart_read(self, emu, addr):
        return self.port.read(emu, addr)

    def uart_write(self, emu, addr, value):
        return self.port.write(emu, addr, value)

    def uart_irq(self, emu):
        return self.port.irq(emu)

    def host(self, emu):
        return self.port.host(emu)


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
        self.mmio = []
        self.h7 = cfg.get("CONFIG_MACH_STM32H7")
        if cfg.get("CONFIG_SERIAL"):
            self.port = Stm32Usart(cfg, chip)
        elif cfg.get("CONFIG_USBSERIAL") and cfg.get("CONFIG_HAVE_STM32_USBFS"):
            self.port = UsbFs(cfg, chip)
        elif cfg.get("CONFIG_USBSERIAL") and cfg.get("CONFIG_HAVE_STM32_USBOTG"):
            self.port = UsbOtg(cfg, chip)
        elif cfg.get("CONFIG_CANSERIAL") and cfg.get("CONFIG_HAVE_STM32_CANBUS"):
            self.port = BxCan(cfg, chip)
        elif cfg.get("CONFIG_CANSERIAL") and cfg.get("CONFIG_HAVE_STM32_FDCANBUS"):
            self.port = FdCan(cfg, chip)
        else:
            sys.exit("only the stm32 usart, usbfs, usbotg, bxcan and fdcan ports are modelled")

    def setup(self, uc):
        # (the system memory with the unique device id, chipid.c: 0 here; a
        # rom, not registers: C reads it through plain pointers, in the
        # order and sizes gcc picks)
        uc.mem_map(0x1FF00000, 0x100000, unicorn.UC_PROT_READ | unicorn.UC_PROT_EXEC)
        # (stm32h7: the AXI SRAM, where dfu_reboot.c keeps its flag)
        if self.h7:
            uc.mem_map(0x24000000, 0x20000)

    def uart_read(self, emu, addr):
        return self.port.read(emu, addr)

    def uart_write(self, emu, addr, value):
        return self.port.write(emu, addr, value)

    def uart_irq(self, emu):
        return self.port.irq(emu)

    def host(self, emu):
        return self.port.host(emu)


class Stm32Usart:
    """stm32/serial.c, stm32f0_serial.c: the usart"""

    def __init__(self, cfg, chip):
        c = chip.const
        usart = next(k for k in ("USART1", "USART2", "USART3") if cfg.get("CONFIG_STM32_SERIAL_" + k))
        # (the irq a usart shares on some chips, as stm32f0_serial.c names it)
        shared = {"USART2": ["USART2_LPUART2_IRQn"], "USART3": ["USART3_4_5_6_LPUART1_IRQn", "USART3_4_5_6_IRQn"]}
        self.irq_num = next(c[x] for x in [usart + "_IRQn"] + shared.get(usart, []) if x in c)
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

    def read(self, emu, addr):
        if addr == self.sr:
            return self.TXE | self.TC | (self.RXNE if emu.rx else 0)
        if addr == self.dr:
            return emu.rx_byte()
        return None

    def write(self, emu, addr, value):
        if addr == self.tdr:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def irq(self, emu):
        cr1 = emu.get(self.cr1, 4)
        if (emu.rx and cr1 & self.RXNEIE) or cr1 & self.TXEIE:
            return self.irq_num
        return None

    def host(self, emu):
        return False


class UsbHost:
    """the host side of a usb cdc acm device: a reset, the requests of an
    enumeration (and one the device does not support), then the serial data
    on the bulk endpoints. The device controller model gives it `ready()`,
    `reset()`, `setup(data)`, `out(ep, data)` and `in_(ep)` ("ack", "nak" or
    "stall"). What the device answers is written to the access log."""

    LINE_CODING = struct.pack("<IBBB", 250000, 0, 0, 8)
    REQUESTS = [
        # (bmRequestType, bRequest, wValue, wIndex, wLength, out data)
        (0x80, 6, 0x0100, 0, 64, b""),       # GET_DESCRIPTOR device
        (0x00, 5, 0x0001, 0, 0, b""),        # SET_ADDRESS 1
        (0x80, 6, 0x0100, 0, 18, b""),       # GET_DESCRIPTOR device
        (0x80, 6, 0x0200, 0, 9, b""),        # GET_DESCRIPTOR configuration
        (0x80, 6, 0x0200, 0, 255, b""),
        (0x80, 6, 0x0300, 0, 255, b""),      # GET_DESCRIPTOR string 0, 1, 2, 3
        (0x80, 6, 0x0301, 0x0409, 255, b""),
        (0x80, 6, 0x0302, 0x0409, 255, b""),
        (0x80, 6, 0x0303, 0x0409, 255, b""),
        (0x80, 0, 0, 0, 2, b""),             # GET_STATUS (not supported: stall)
        (0x00, 9, 0x0001, 0, 0, b""),        # SET_CONFIGURATION 1
        (0x21, 0x20, 0, 0, 7, LINE_CODING),  # SET_LINE_CODING
        (0xA1, 0x21, 0, 0, 7, b""),          # GET_LINE_CODING
        (0x21, 0x22, 0x0003, 0, 0, b""),     # SET_CONTROL_LINE_STATE dtr rts
    ]

    def __init__(self, dev, board="generic"):
        self.dev = dev
        self.steps = self.enumerate()
        self.enumerated = False
        # (the endpoints: Kalico's src/<board>/usb_cdc_ep.h, or the generic one)
        path = os.path.join(mcu_run.KALICO, "src", board, "usb_cdc_ep.h")
        if not os.path.exists(path):
            path = os.path.join(mcu_run.KALICO, "src", "generic", "usb_cdc_ep.h")
        ep = {m.group(1): int(m.group(2)) for m in re.finditer(r"(USB_CDC_\w+) = (\d+)", open(path).read())}
        self.EP0_SIZE, self.BULK_IN = ep["USB_CDC_EP0_SIZE"], ep["USB_CDC_EP_BULK_IN"]
        self.BULK_OUT, self.BULK_OUT_SIZE = ep["USB_CDC_EP_BULK_OUT"], ep["USB_CDC_EP_BULK_OUT_SIZE"]

    def step(self, emu):
        """what the host can do now; True if it did something"""
        did = False
        while not self.enumerated:
            r = next(self.steps, None)
            if r is None:
                self.enumerated = True
            elif not r:
                return did
            else:
                did = True
        dev = self.dev
        while emu.rx:
            if dev.out(emu, self.BULK_OUT, emu.rx[: self.BULK_OUT_SIZE]) != "ack":
                break
            emu.rx = emu.rx[self.BULK_OUT_SIZE :]
            did = True
        while True:
            r, data = dev.in_(emu, self.BULK_IN)
            if r != "ack":
                break
            emu.tx += data
            did = True
        return did

    def enumerate(self):
        """a generator: True for a step done, False for a wait on the device"""
        while not self.dev.ready(self.emu):
            yield False
        self.dev.reset(self.emu)
        yield True
        for req in self.REQUESTS:
            yield from self.control(*req)

    def control(self, rtype, request, value, index, length, data):
        emu, dev = self.emu, self.dev
        log = emu.log
        while not dev.setup(emu, struct.pack("<BBHHH", rtype, request, value, index, length)):
            yield False
        yield True
        if length and rtype & 0x80:
            # (data in, then a zero length packet out)
            got = b""
            while True:
                r, d = dev.in_(emu, 0)
                if r == "nak":
                    yield False
                    continue
                if r == "stall":
                    log.write("# usb control %02x %02x: stall\n" % (rtype, request))
                    return
                got += d
                yield True
                if len(d) < self.EP0_SIZE or len(got) >= length:
                    break
            log.write("# usb control %02x %02x in %s\n" % (rtype, request, got.hex()))
            while dev.out(emu, 0, b"") == "nak":
                yield False
            yield True
            return
        # (data out, then a zero length packet in)
        for i in range(0, len(data), self.EP0_SIZE):
            while True:
                r = dev.out(emu, 0, data[i : i + self.EP0_SIZE])
                if r == "nak":
                    yield False
                    continue
                if r == "stall":
                    log.write("# usb control %02x %02x: stall\n" % (rtype, request))
                    return
                yield True
                break
        while True:
            r, d = dev.in_(emu, 0)
            if r == "nak":
                yield False
                continue
            log.write("# usb control %02x %02x: %s\n" % (rtype, request, "stall" if r == "stall" else "ack"))
            yield True
            return


class CanHost:
    """the host side of a canbus node (generic/canserial.c): the admin query
    for unassigned nodes, the node id assignment to the uuid the node
    answered, then the serial data in frames to the node's id and from the
    next id. The controller model gives it `ready()`, `send(id, data)`
    (False while it has no room) and `recv()` (the frames the node sent, in
    order). What the node answers is written to the access log."""

    ADMIN, ADMIN_RESP = 0x3F0, 0x3F1
    NODEID = 4

    def __init__(self, dev):
        self.dev = dev
        self.steps = self.assign()
        self.assigned = False
        self.id = (self.NODEID << 1) + 0x100
        self.admin = []

    def step(self, emu):
        """what the host can do now; True if it did something"""
        self.emu = emu
        did = False
        for fid, data in self.dev.recv(emu):
            did = True
            if fid == self.ADMIN_RESP:
                emu.log.write("# can admin response %s\n" % data.hex())
                self.admin.append(data)
            elif fid == self.id + 1:
                emu.tx += data
            else:
                emu.log.write("# can frame %x %s\n" % (fid, data.hex()))
        while not self.assigned:
            r = next(self.steps, None)
            if r is None:
                self.assigned = True
            elif not r:
                return did
            else:
                did = True
        while emu.rx and self.dev.send(emu, self.id, emu.rx[:8]):
            emu.rx = emu.rx[8:]
            did = True
        return did

    def assign(self):
        """a generator: True for a step done, False for a wait on the node"""
        dev = self.dev
        while not dev.ready(self.emu):
            yield False
        # (CANBUS_CMD_QUERY_UNASSIGNED; its answer: CANBUS_RESP_NEED_NODEID,
        # the uuid)
        while not dev.send(self.emu, self.ADMIN, b"\x00"):
            yield False
        yield True
        while not self.admin:
            yield False
        uuid = self.admin.pop(0)[1:7]
        # (CANBUS_CMD_SET_KLIPPER_NODEID)
        while not dev.send(self.emu, self.ADMIN, b"\x01" + uuid + bytes([self.NODEID])):
            yield False
        yield True
        # (Kalico's extended query: CANBUS_RESP_HAVE_NODEID once the node
        # took its id)
        while not dev.send(self.emu, self.ADMIN, b"\x00\x01"):
            yield False
        yield True
        while not self.admin:
            yield False
        self.admin.pop(0)


class BxCan:
    """stm32/can.c: the bxcan controller (MCR INRQ / MSR INAK, the three
    transmit mailboxes: sent at once, TSR TME always set and RQCP once one
    was sent; receive fifo 0 of three frames: RF0R FMP0 / RFOM0, IER
    FMPIE0 / TMEIE) for the CanHost"""

    def __init__(self, cfg, chip):
        c = chip.const
        f4_can2 = cfg.get("CONFIG_MACH_STM32F4") and (
            cfg.get("CONFIG_STM32_CANBUS_PB5_PB6") or cfg.get("CONFIG_STM32_CANBUS_PB12_PB13"))
        n = "CAN" if cfg.get("CONFIG_MACH_STM32F0") else ("CAN2" if f4_can2 else "CAN1")
        r = lambda f: chip.at(n, f)  # noqa: E731
        self.mcr, self.msr, self.tsr, self.rf0r, self.ier = r("MCR"), r("MSR"), r("TSR"), r("RF0R"), r("IER")
        self.tx_mb, self.rx_mb = r("sTxMailBox"), r("sFIFOMailBox")
        irq = (lambda k: c["CEC_CAN_IRQn"]) if n == "CAN" else (lambda k: c["%s_%s_IRQn" % (n, k)])  # noqa: E731
        self.rx_irq, self.tx_irq = irq("RX0"), irq("TX")
        self.INRQ, self.INAK = c["CAN_MCR_INRQ"], c["CAN_MSR_INAK"]
        self.TME = c["CAN_TSR_TME0"] | c["CAN_TSR_TME1"] | c["CAN_TSR_TME2"]
        self.RQCP = c["CAN_TSR_RQCP0"]
        self.FMP0, self.RFOM0 = c["CAN_RF0R_FMP0"], c["CAN_RF0R_RFOM0"]
        self.FMPIE0, self.TMEIE = c["CAN_IER_FMPIE0"], c["CAN_IER_TMEIE"]
        self.TXRQ, self.STID_Pos = c["CAN_TI0R_TXRQ"], c["CAN_TI0R_STID_Pos"]
        self.sent = False
        self.rx = []
        self.tx = []
        self.host_side = CanHost(self)

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.msr:
            return self.INAK if emu.get(self.mcr, 4) & self.INRQ else 0
        if addr == self.tsr:
            return self.TME | (self.RQCP if self.sent else 0)
        if addr == self.rf0r:
            return len(self.rx)
        if self.rx_mb <= addr < self.rx_mb + 16:
            if not self.rx:
                return 0
            fid, data = self.rx[0]
            words = (fid << self.STID_Pos, len(data)) + struct.unpack("<II", data.ljust(8, b"\0"))
            return words[(addr - self.rx_mb) // 4]
        return None

    def write(self, emu, addr, value):
        if addr == self.rf0r:
            if value & self.RFOM0 and self.rx:
                self.rx.pop(0)
            return True
        if addr in (self.tx_mb, self.tx_mb + 16, self.tx_mb + 32) and value & self.TXRQ:
            # (the mailbox's frame goes out)
            dlc = emu.get(addr + 4, 4) & 0xF
            data = struct.pack("<II", emu.get(addr + 8, 4), emu.get(addr + 12, 4))[:dlc]
            self.tx.append(((value >> self.STID_Pos) & 0x7FF, data))
            self.sent = True
            emu.put(addr, 4, value & ~self.TXRQ)
            return True
        return False

    def used_bits(self, emu, addr):
        # (a mailbox's TDLR / TDHR bytes past its TDTR DLC are not sent;
        # can.c copies the whole message data words)
        for mb in (self.tx_mb, self.tx_mb + 16, self.tx_mb + 32):
            if addr in (mb + 8, mb + 12):
                left = min(emu.get(mb + 4, 4) & 0xF, 8) - (addr - mb - 8)
                return (1 << (8 * max(0, min(left, 4)))) - 1
        return 0xFFFFFFFF

    def irq(self, emu):
        ier = emu.get(self.ier, 4)
        if self.rx and ier & self.FMPIE0:
            return self.rx_irq
        if self.sent and ier & self.TMEIE:
            return self.tx_irq
        return None

    def host(self, emu):
        return self.host_side.step(emu)

    # ---- the host's frames

    def ready(self, emu):
        return not emu.get(self.mcr, 4) & self.INRQ and emu.get(self.ier, 4) & self.FMPIE0

    def send(self, emu, fid, data):
        if len(self.rx) >= 3:
            return False
        self.rx.append((fid, bytes(data)))
        return True

    def recv(self, emu):
        frames, self.tx = self.tx, []
        return frames


class FdCan:
    """stm32/fdcan.c: the fdcan controller (its message ram at SRAMCAN_BASE
    in fdcan.c's `struct fdcan_msg_ram` layout; the transmit fifo: a TXBAR
    request goes out at once, TXFQS TFQPI the next put index; receive fifo
    0: RXF0S fill level / get index, RXF0A acknowledge; IR flags cleared by
    writing 1, IE, ILE EINT0) for the CanHost. The node handles one frame
    per RF0N interrupt (fdcan.c clears RF0N first), as frames on a bus come
    one at a time: the host sends the next when the fifo is empty."""

    # (struct fdcan_msg_ram: FLS[28], FLE[16], RXF0[3], RXF1[3], TEF[6],
    # TXFIFO[3] of 2 + 16 words)
    RXF0, TXFIFO, FIFO_SIZE, RAM_SIZE = 176, 632, 72, 848

    def __init__(self, cfg, chip):
        c = chip.const
        can2 = any(cfg.get("CONFIG_STM32_CANBUS_" + p) for p in ("PB0_PB1", "PC2_PC3", "PB5_PB6", "PB12_PB13"))
        n = "FDCAN2" if can2 else "FDCAN1"
        r = lambda f: chip.at(n, f)  # noqa: E731
        self.txfqs, self.txbar, self.ir, self.ie = r("TXFQS"), r("TXBAR"), r("IR"), r("IE")
        self.ile, self.rxf0s, self.rxf0a, self.cccr = r("ILE"), r("RXF0S"), r("RXF0A"), r("CCCR")
        self.ram = c["SRAMCAN_BASE"] + (self.RAM_SIZE if can2 else 0)
        self.memory = [(self.ram, self.RAM_SIZE)]
        self.irq_num = c["TIM16_FDCAN_IT0_IRQn"] if cfg.get("CONFIG_MACH_STM32G0") else c[n + "_IT0_IRQn"]
        self.RF0N, self.TC, self.TFE = c["FDCAN_IR_RF0N"], c["FDCAN_IE_TCE"], c["FDCAN_IE_TFEE"]
        self.INIT, self.EINT0 = c["FDCAN_CCCR_INIT"], c["FDCAN_ILE_EINT0"]
        self.TFQPI_Pos, self.F0GI_Pos = c["FDCAN_TXFQS_TFQPI_Pos"], c["FDCAN_RXF0S_F0GI_Pos"]
        self.flags = 0
        # (the transmit put index; receive fifo 0: its frames' indexes, put index)
        self.tx_put = 0
        self.rx_idx = []
        self.rx_put = 0
        self.tx = []
        self.host_side = CanHost(self)

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.ir:
            return self.flags
        if addr == self.txfqs:
            return self.tx_put << self.TFQPI_Pos
        if addr == self.rxf0s:
            return len(self.rx_idx) | ((self.rx_idx[0] if self.rx_idx else 0) << self.F0GI_Pos)
        return None

    def write(self, emu, addr, value):
        if addr == self.ir:
            self.flags &= ~value
            return True
        if addr == self.rxf0a:
            if self.rx_idx and self.rx_idx[0] == value:
                self.rx_idx.pop(0)
            return True
        if addr == self.txbar:
            # (the fifo element's frame goes out)
            for i in range(3):
                if value >> i & 1:
                    e = self.ram + self.TXFIFO + i * self.FIFO_SIZE
                    dlc = (emu.get(e + 4, 4) >> 16) & 0xF
                    data = struct.pack("<II", emu.get(e + 8, 4), emu.get(e + 12, 4))[:dlc]
                    fid = (emu.get(e, 4) >> 18) & 0x7FF
                    emu.log.write("# can frame out %x %s\n" % (fid, data.hex()))
                    self.tx.append((fid, data))
                    self.tx_put = (i + 1) % 3
            self.flags |= self.TC | self.TFE
            return True
        return False

    def irq(self, emu):
        if self.flags & emu.get(self.ie, 4) and emu.get(self.ile, 4) & self.EINT0:
            return self.irq_num
        return None

    def host(self, emu):
        return self.host_side.step(emu)

    # ---- the host's frames

    def ready(self, emu):
        return not emu.get(self.cccr, 4) & self.INIT and emu.get(self.ie, 4) & self.RF0N

    def send(self, emu, fid, data):
        if self.rx_idx:
            return False
        emu.log.write("# can frame in %x %s\n" % (fid, bytes(data).hex()))
        e = self.ram + self.RXF0 + self.rx_put * self.FIFO_SIZE
        words = (fid << 18, len(data) << 16) + struct.unpack("<II", bytes(data).ljust(8, b"\0"))
        for i, w in enumerate(words):
            emu.put(e + 4 * i, 4, w)
        self.rx_idx.append(self.rx_put)
        self.rx_put = (self.rx_put + 1) % 3
        self.flags |= self.RF0N
        return True

    def recv(self, emu):
        frames, self.tx = self.tx, []
        return frames


class UsbFs:
    """stm32/usbfs.c: the usb full speed device controller (the EPnR toggle /
    clear-on-0 / read-write bits, ISTR, CNTR, the packet memory with its
    buffer table, double buffered bulk endpoints) for the UsbHost"""

    def __init__(self, cfg, chip):
        c = chip.const
        bit = lambda *names: next(c[x] for x in names if x in c)  # noqa: E731
        g0 = cfg.get("CONFIG_MACH_STM32G0")
        usb = "USB_DRD_FS" if g0 else "USB"
        self.epr = chip.ptr[usb]
        self.cntr, self.istr = chip.at(usb, "CNTR"), chip.at(usb, "ISTR")
        self.pma = c["USB_DRD_PMAADDR"] if g0 else c["USB_PMAADDR"]
        # (usbfs.c: f1 packet memory is 16 bit data in 32 bit words, f0 / l4 /
        # g4 16 bit words, g0 32 bit words)
        self.layout = 4 if cfg.get("CONFIG_MACH_STM32F1") else 32 if g0 else 2
        self.irq_num = bit("USB_LP_IRQn", "USB_UCPD1_2_IRQn", "USB_IRQn")
        self.CTR_RX, self.CTR_TX = bit("USB_EP_CTR_RX", "USB_EP_VTRX"), bit("USB_EP_CTR_TX", "USB_EP_VTTX")
        self.DTOG_RX, self.DTOG_TX = c["USB_EP_DTOG_RX"], c["USB_EP_DTOG_TX"]
        self.STAT_RX, self.STAT_TX = bit("USB_EPRX_STAT", "USB_EP_RX_STRX"), bit("USB_EPTX_STAT", "USB_EP_TX_STTX")
        self.SETUP, self.KIND, self.TYPE = c["USB_EP_SETUP"], c["USB_EP_KIND"], c["USB_EP_TYPE_MASK"]
        self.BULK = c["USB_EP_BULK"]
        self.RX_VALID, self.RX_NAK, self.RX_STALL = c["USB_EP_RX_VALID"], c["USB_EP_RX_NAK"], c["USB_EP_RX_STALL"]
        self.TX_VALID, self.TX_NAK, self.TX_STALL = c["USB_EP_TX_VALID"], c["USB_EP_TX_NAK"], c["USB_EP_TX_STALL"]
        self.ISTR_CTR, self.ISTR_RESET, self.ISTR_DIR = c["USB_ISTR_CTR"], c["USB_ISTR_RESET"], c["USB_ISTR_DIR"]
        self.CTRM, self.RESETM = c["USB_CNTR_CTRM"], c["USB_CNTR_RESETM"]
        self.T = self.DTOG_RX | self.STAT_RX | self.DTOG_TX | self.STAT_TX
        self.RC = self.CTR_RX | self.CTR_TX
        self.eps = [0] * 8
        self.flags = 0
        # (the double buffered in endpoints just enabled: they send without
        # the DTOG_TX == SW_BUF check)
        self.armed = set()
        self.host_side = UsbHost(self)

    # ---- the cpu's registers

    def read(self, emu, addr):
        if self.epr <= addr < self.epr + 32 and not addr & 3:
            return self.eps[(addr - self.epr) // 4]
        if addr == self.istr:
            v = self.flags
            for n, e in enumerate(self.eps):
                if e & (self.CTR_RX | self.CTR_TX):
                    v |= self.ISTR_CTR | n | (self.ISTR_DIR if e & self.CTR_RX else 0)
                    break
            return v
        return None

    def write(self, emu, addr, value):
        if self.epr <= addr < self.epr + 32 and not addr & 3:
            n = (addr - self.epr) // 4
            old = self.eps[n]
            rw = ~(self.T | self.RC | self.SETUP) & 0xFFFF
            new = self.eps[n] = ((value & rw) | ((old ^ value) & self.T) | (old & value & self.RC) | (old & self.SETUP))
            if new & self.STAT_TX == self.TX_VALID and old & self.STAT_TX != self.TX_VALID:
                self.armed.add(n)
            return True
        if addr == self.istr:
            # (the flags clear on 0; CTR, DIR, EP_ID are read only)
            self.flags &= value
            return True
        return False

    def irq(self, emu):
        cntr = emu.get(self.cntr, 4)
        ctr = any(e & (self.CTR_RX | self.CTR_TX) for e in self.eps)
        if (cntr & self.CTRM and ctr) or (cntr & self.RESETM and self.flags & self.ISTR_RESET):
            return self.irq_num
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the packet memory

    def pma_addr(self, off):
        """the cpu address of a packet memory byte offset (even)"""
        return self.pma + (off // 2 * 4 if self.layout == 4 else off)

    def pma_read(self, emu, off, n):
        out = b""
        for i in range(0, n, 2):
            out += struct.pack("<H", emu.get(self.pma_addr(off + i), 2))
        return out[:n]

    def pma_write(self, emu, off, data):
        data += b"\0" * (len(data) & 1)
        for i in range(0, len(data), 2):
            emu.put(self.pma_addr(off + i), 2, struct.unpack_from("<H", data, i)[0])

    def desc(self, emu, ep, bufnum):
        """(buffer address, count, count's address) of a buffer table entry"""
        o = (ep * 2 + bufnum) * 4
        if self.layout == 32:
            w = emu.get(self.pma + o, 4)
            return w & 0xFFFF, (w >> 16) & 0x3FF, None
        return emu.get(self.pma_addr(o), 2), emu.get(self.pma_addr(o + 2), 2) & 0x3FF, o + 2

    def set_count(self, emu, ep, bufnum, count):
        o = (ep * 2 + bufnum) * 4
        if self.layout == 32:
            w = emu.get(self.pma + o, 4)
            emu.put(self.pma + o, 4, (w & ~(0x3FF << 16)) | (count << 16))
        else:
            a = self.pma_addr(o + 2)
            emu.put(a, 2, (emu.get(a, 2) & ~0x3FF) | count)

    # ---- the host's transactions

    def dbuf(self, e):
        return e & self.TYPE == self.BULK and e & self.KIND

    def ready(self, emu):
        return emu.get(self.cntr, 4) & self.RESETM

    def reset(self, emu):
        self.eps = [0] * 8
        self.flags |= self.ISTR_RESET

    def setup(self, emu, data):
        e = self.eps[0]
        if e & self.STAT_RX == 0:
            return False
        addr, _, _ = self.desc(emu, 0, 1)
        self.pma_write(emu, addr, data)
        self.set_count(emu, 0, 1, len(data))
        # (a setup sets both directions to NAK, ending a stall)
        e &= ~(self.STAT_RX | self.STAT_TX)
        self.eps[0] = e | self.RX_NAK | self.TX_NAK | self.CTR_RX | self.SETUP
        return True

    def out(self, emu, ep, data):
        e = self.eps[ep]
        if e & self.STAT_RX == self.RX_STALL:
            return "stall"
        if self.dbuf(e):
            # (double buffered: the buffer DTOG_RX names, unless both are the
            # cpu's (DTOG_RX == SW_BUF, the DTOG_TX bit))
            if e & self.STAT_RX != self.RX_VALID or bool(e & self.DTOG_RX) == bool(e & self.DTOG_TX):
                return "nak"
            b = 1 if e & self.DTOG_RX else 0
            addr, _, _ = self.desc(emu, ep, b)
            self.pma_write(emu, addr, data)
            self.set_count(emu, ep, b, len(data))
            self.eps[ep] = (e ^ self.DTOG_RX) | self.CTR_RX
            return "ack"
        if e & self.STAT_RX != self.RX_VALID:
            return "nak"
        addr, _, _ = self.desc(emu, ep, 1)
        self.pma_write(emu, addr, data)
        self.set_count(emu, ep, 1, len(data))
        self.eps[ep] = ((e & ~(self.STAT_RX | self.SETUP)) | self.RX_NAK | self.CTR_RX) ^ self.DTOG_RX
        return "ack"

    def in_(self, emu, ep):
        e = self.eps[ep]
        if e & self.STAT_TX == self.TX_STALL:
            return "stall", b""
        if self.dbuf(e):
            # (double buffered: the buffer DTOG_TX names, unless both are the
            # cpu's (DTOG_TX == SW_BUF, the DTOG_RX bit) after a transfer; a
            # freshly enabled endpoint sends both, see usbfs.c BI_START)
            blocked = bool(e & self.DTOG_TX) == bool(e & self.DTOG_RX) and ep not in self.armed
            if e & self.STAT_TX != self.TX_VALID or blocked:
                return "nak", b""
            self.armed.discard(ep)
            b = 1 if e & self.DTOG_TX else 0
            addr, count, _ = self.desc(emu, ep, b)
            data = self.pma_read(emu, addr, count)
            self.eps[ep] = (e ^ self.DTOG_TX) | self.CTR_TX
            return "ack", data
        if e & self.STAT_TX != self.TX_VALID:
            return "nak", b""
        addr, count, _ = self.desc(emu, ep, 0)
        data = self.pma_read(emu, addr, count)
        self.eps[ep] = ((e & ~self.STAT_TX) | self.TX_NAK | self.CTR_TX) ^ self.DTOG_TX
        return "ack", data


class UsbOtg:
    """stm32/usbotg.c: the usb otg device controller in full speed device
    mode (the receive status / data fifos, the endpoint transmit fifos,
    DIEPCTL / DOEPCTL with EPENA, CNAK / SNAK, EPDIS, STALL, the transfer
    complete interrupts) for the UsbHost. usbotg.c does not use the bus
    reset interrupt: the host starts with a setup."""

    def __init__(self, cfg, chip):
        c = chip.const
        hs = cfg.get("CONFIG_STM32_USB_PB14_PB15") or cfg.get("CONFIG_MACH_STM32H723")
        base = c["USB_OTG_HS_PERIPH_BASE"] if hs else c["USB_OTG_FS_PERIPH_BASE"]
        self.irq_num = c["OTG_HS_IRQn"] if hs else c["OTG_FS_IRQn"]
        g = lambda f: base + chip.off[("USB_OTG_GlobalTypeDef", f)]  # noqa: E731
        d = lambda f: base + c["USB_OTG_DEVICE_BASE"] + chip.off[("USB_OTG_DeviceTypeDef", f)]  # noqa: E731
        self.gahbcfg, self.grstctl, self.gintsts = g("GAHBCFG"), g("GRSTCTL"), g("GINTSTS")
        self.gintmsk, self.grxstsr, self.grxstsp = g("GINTMSK"), g("GRXSTSR"), g("GRXSTSP")
        self.diepmsk, self.daint, self.daintmsk = d("DIEPMSK"), d("DAINT"), d("DAINTMSK")
        self.epin = base + c["USB_OTG_IN_ENDPOINT_BASE"]
        self.epout = base + c["USB_OTG_OUT_ENDPOINT_BASE"]
        self.fifo = base + c["USB_OTG_FIFO_BASE"]
        ie = lambda f: chip.off[("USB_OTG_INEndpointTypeDef", f)]  # noqa: E731
        oe = lambda f: chip.off[("USB_OTG_OUTEndpointTypeDef", f)]  # noqa: E731
        self.DIEPCTL, self.DIEPINT, self.DIEPTSIZ = ie("DIEPCTL"), ie("DIEPINT"), ie("DIEPTSIZ")
        self.DOEPCTL, self.DOEPINT = oe("DOEPCTL"), oe("DOEPINT")
        self.RXFLVL, self.IEPINT = c["USB_OTG_GINTSTS_RXFLVL"], c["USB_OTG_GINTSTS_IEPINT"]
        self.RXFLVLM, self.IEPINTM = c["USB_OTG_GINTMSK_RXFLVLM"], c["USB_OTG_GINTMSK_IEPINT"]
        self.GINT = c["USB_OTG_GAHBCFG_GINT"]
        self.AHBIDL, self.TXFFLSH = c["USB_OTG_GRSTCTL_AHBIDL"], c["USB_OTG_GRSTCTL_TXFFLSH"]
        self.TXFNUM_Pos = c["USB_OTG_GRSTCTL_TXFNUM_Pos"]
        self.EPENA, self.EPDIS = c["USB_OTG_DIEPCTL_EPENA"], c["USB_OTG_DIEPCTL_EPDIS"]
        self.CNAK, self.SNAK = c["USB_OTG_DIEPCTL_CNAK"], c["USB_OTG_DIEPCTL_SNAK"]
        self.STALL, self.NAKSTS = c["USB_OTG_DIEPCTL_STALL"], c["USB_OTG_DIEPCTL_NAKSTS"]
        self.XFRC, self.XFRCM = c["USB_OTG_DIEPINT_XFRC"], c["USB_OTG_DIEPMSK_XFRCM"]
        self.BCNT_Pos, self.PKTSTS_Pos = c["USB_OTG_GRXSTSP_BCNT_Pos"], c["USB_OTG_GRXSTSP_PKTSTS_Pos"]
        self.XFRSIZ = c["USB_OTG_DIEPTSIZ_XFRSIZ"]
        # (the receive fifo: status words with their data words; the data
        # of the status last popped; the transmit fifos; the nak states)
        self.rxq = []
        self.rxdata = []
        self.txf = [[] for _ in range(16)]
        self.in_nak = [True] * 16
        self.out_nak = [True] * 16
        self.host_side = UsbHost(self)

    def ep_reg(self, addr):
        """(direction, endpoint, offset) of an endpoint register"""
        for d, base in (("in", self.epin), ("out", self.epout)):
            if base <= addr < base + 16 * 0x20:
                return d, (addr - base) // 0x20, (addr - base) % 0x20
        return None

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.gintsts:
            v = emu.get(addr, 4) & ~(self.RXFLVL | self.IEPINT)
            if self.rxq:
                v |= self.RXFLVL
            if self.daint_value(emu) & emu.get(self.daintmsk, 4):
                v |= self.IEPINT
            return v
        if addr in (self.grxstsr, self.grxstsp):
            if not self.rxq:
                return 0
            if addr == self.grxstsr:
                return self.rxq[0][0]
            sts, data = self.rxq.pop(0)
            self.rxdata = list(data)
            return sts
        if self.fifo <= addr < self.fifo + 16 * 0x1000:
            return self.rxdata.pop(0) if self.rxdata else 0
        if addr == self.grstctl:
            return (emu.get(addr, 4) & ~self.TXFFLSH) | self.AHBIDL
        if addr == self.daint:
            return self.daint_value(emu)
        r = self.ep_reg(addr)
        if r and r[2] == 0:
            d, ep, _ = r
            nak = self.in_nak[ep] if d == "in" else self.out_nak[ep]
            return (emu.get(addr, 4) & ~self.NAKSTS) | (self.NAKSTS if nak else 0)
        return None

    def daint_value(self, emu):
        v = 0
        for ep in range(16):
            if emu.get(self.epin + ep * 0x20 + self.DIEPINT, 4) & emu.get(self.diepmsk, 4):
                v |= 1 << ep
        return v

    def write(self, emu, addr, value):
        if self.fifo <= addr < self.fifo + 16 * 0x1000:
            self.txf[(addr - self.fifo) // 0x1000].append(value)
            return True
        if addr == self.grstctl:
            if value & self.TXFFLSH:
                n = (value >> self.TXFNUM_Pos) & 0x1F
                for ep in range(16) if n == 0x10 else [n]:
                    self.txf[ep] = []
            emu.put(addr, 4, value & ~self.TXFFLSH)
            return True
        r = self.ep_reg(addr)
        if r is None:
            return False
        d, ep, off = r
        if off == self.DIEPINT or off == self.DOEPINT:
            # (cleared by writing 1)
            emu.put(addr, 4, emu.get(addr, 4) & ~value)
            return True
        if off == 0:
            naks = self.in_nak if d == "in" else self.out_nak
            if value & self.SNAK:
                naks[ep] = True
            if value & self.CNAK:
                naks[ep] = False
            v = value & ~(self.SNAK | self.CNAK | self.NAKSTS)
            if v & self.EPDIS:
                v &= ~(self.EPDIS | self.EPENA)
            emu.put(addr, 4, v)
            return True
        return False

    def irq(self, emu):
        if not emu.get(self.gahbcfg, 4) & self.GINT:
            return None
        msk = emu.get(self.gintmsk, 4)
        if (self.rxq and msk & self.RXFLVLM) or (msk & self.IEPINTM and self.daint_value(emu) & emu.get(self.daintmsk, 4)):
            return self.irq_num
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def status(self, pktsts, ep, n):
        return ep | (n << self.BCNT_Pos) | (pktsts << self.PKTSTS_Pos)

    @staticmethod
    def words(data):
        data += b"\0" * (-len(data) % 4)
        return [struct.unpack_from("<I", data, i)[0] for i in range(0, len(data), 4)]

    def ready(self, emu):
        return emu.get(self.gahbcfg, 4) & self.GINT and emu.get(self.epout + self.DOEPCTL, 4) & self.EPENA

    def reset(self, emu):
        pass

    def setup(self, emu, data):
        # (setup data received, setup stage done; a setup ends a stall)
        self.rxq.append((self.status(6, 0, len(data)), self.words(data)))
        self.rxq.append((self.status(4, 0, 0), []))
        a = self.epin + self.DIEPCTL
        emu.put(a, 4, emu.get(a, 4) & ~self.STALL)
        return True

    def out(self, emu, ep, data):
        a = self.epout + ep * 0x20 + self.DOEPCTL
        ctl = emu.get(a, 4)
        if ctl & self.STALL or (ep == 0 and emu.get(self.epin + self.DIEPCTL, 4) & self.STALL):
            return "stall"
        if not ctl & self.EPENA or self.out_nak[ep]:
            return "nak"
        # (data received, transfer done: the endpoint is disabled, naks)
        self.rxq.append((self.status(2, ep, len(data)), self.words(data)))
        self.rxq.append((self.status(3, ep, 0), []))
        emu.put(a, 4, ctl & ~self.EPENA)
        self.out_nak[ep] = True
        return "ack"

    def in_(self, emu, ep):
        a = self.epin + ep * 0x20
        ctl = emu.get(a + self.DIEPCTL, 4)
        if ctl & self.STALL:
            return "stall", b""
        if not ctl & self.EPENA or self.in_nak[ep]:
            return "nak", b""
        n = emu.get(a + self.DIEPTSIZ, 4) & self.XFRSIZ
        data = b"".join(struct.pack("<I", w) for w in self.txf[ep])[:n]
        self.txf[ep] = []
        emu.put(a + self.DIEPCTL, 4, ctl & ~self.EPENA)
        emu.put(a + self.DIEPINT, 4, emu.get(a + self.DIEPINT, 4) | self.XFRC)
        return "ack", data


class Lpc176x(Ported):
    """lpc176x: the uart or the usb device, DWT time; its gpio block sits at
    0x2009c000"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "lpc176x", "chip", "lpc17xx.rs"))
        self.cpu = A.UC_CPU_ARM_CORTEX_M3
        self.time = DwtTime(chip)
        self.mmio = [(chip.ptr["LPC_GPIO0"] & ~0xFFFF, 0x10000)]
        self.read_rules = {}
        if cfg.get("CONFIG_USBSERIAL"):
            self.port = LpcUsb(chip)
        elif cfg.get("CONFIG_CANSERIAL"):
            self.port = LpcCan(cfg, chip)
        else:
            self.port = LpcUart(cfg, chip)

    # (the boot rom's in application programming entry, chipid.c's serial
    # number read: CMD_SUCCESS, zeros)
    IAP = 0x1FFF1FF0

    def setup(self, uc):
        uc.mem_map(self.IAP & ~0xFFFF, 0x10000, unicorn.UC_PROT_READ | unicorn.UC_PROT_EXEC)
        uc.hook_add(unicorn.UC_HOOK_CODE, self.iap, begin=self.IAP, end=self.IAP)

    def iap(self, uc, address, size, user):
        uc.mem_write(uc.reg_read(A.UC_ARM_REG_R1), bytes(5 * 4))
        uc.reg_write(A.UC_ARM_REG_PC, uc.reg_read(A.UC_ARM_REG_LR))


class LpcCan:
    """lpc176x/can.c: the can controller (transmit buffer 1: sent at once
    on CMR TR, SR TBS1 always set; the receive buffer: SR RBS, RFS / RID /
    RDA / RDB, released by CMR RRB; ICR cleared by reading, IER) for the
    CanHost"""

    TR, RRB = 1 << 0, 1 << 2
    RBS, TBS1 = 1 << 0, 1 << 2
    RI, TI1 = 1 << 0, 1 << 1
    RIE, TIE1 = 1 << 0, 1 << 1

    def __init__(self, cfg, chip):
        n = "LPC_CAN2" if cfg.get("CONFIG_LPC_CANBUS_P0_4_P0_5") or cfg.get("CONFIG_LPC_CANBUS_P2_7_P2_8") else "LPC_CAN1"
        r = lambda f: chip.at(n, f)  # noqa: E731
        self.mod, self.cmr, self.icr, self.ier, self.sr = r("MOD"), r("CMR"), r("ICR"), r("IER"), r("SR")
        self.rfs, self.rid, self.rda, self.rdb = r("RFS"), r("RID"), r("RDA"), r("RDB")
        self.tfi1, self.tid1, self.tda1, self.tdb1 = r("TFI1"), r("TID1"), r("TDA1"), r("TDB1")
        self.irq_num = chip.const["CAN_IRQn"]
        self.sent = False
        self.rx = []
        self.tx = []
        self.host_side = CanHost(self)

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.sr:
            return self.TBS1 | (self.RBS if self.rx else 0)
        if addr == self.icr:
            v = (self.RI if self.rx else 0) | (self.TI1 if self.sent else 0)
            self.sent = False
            return v
        if addr in (self.rfs, self.rid, self.rda, self.rdb):
            if not self.rx:
                return 0
            fid, data = self.rx[0]
            words = (len(data) << 16, fid) + struct.unpack("<II", data.ljust(8, b"\0"))
            return words[(addr - self.rfs) // 4]
        return None

    def write(self, emu, addr, value):
        if addr != self.cmr:
            return False
        if value & self.RRB and self.rx:
            self.rx.pop(0)
        if value & self.TR:
            dlc = (emu.get(self.tfi1, 4) >> 16) & 0xF
            data = struct.pack("<II", emu.get(self.tda1, 4), emu.get(self.tdb1, 4))[:dlc]
            self.tx.append((emu.get(self.tid1, 4) & 0x7FF, data))
            self.sent = True
        return True

    def used_bits(self, emu, addr):
        # (TDA1 / TDB1 bytes past the TFI1 DLC are not sent; can.c copies the
        # whole message data words)
        if addr in (self.tda1, self.tdb1):
            left = min((emu.get(self.tfi1, 4) >> 16) & 0xF, 8) - (addr - self.tda1)
            return (1 << (8 * max(0, min(left, 4)))) - 1
        return 0xFFFFFFFF

    def irq(self, emu):
        ier = emu.get(self.ier, 4)
        if (self.rx and ier & self.RIE) or (self.sent and ier & self.TIE1):
            return self.irq_num
        return None

    def host(self, emu):
        return self.host_side.step(emu)

    # ---- the host's frames

    def ready(self, emu):
        return not emu.get(self.mod, 4) & 1 and emu.get(self.ier, 4) & self.RIE

    def send(self, emu, fid, data):
        if self.rx:
            return False
        self.rx.append((fid, bytes(data)))
        return True

    def recv(self, emu):
        frames, self.tx = self.tx, []
        return frames


class LpcUart:
    """lpc176x/serial.c: the uart (LSR THRE, IIR, IER)"""

    def __init__(self, cfg, chip):
        uart = "UART0" if cfg.get("CONFIG_LPC_SERIAL_UART0_P03_P02") else "UART3"
        self.irq_num = chip.const[uart + "_IRQn"]
        p = "LPC_" + uart
        self.rbr = chip.at(p, "RBR")
        self.ier, self.iir = chip.at(p, "IER"), chip.at(p, "IIR")
        self.lcr, self.lsr = chip.at(p, "LCR"), chip.at(p, "LSR")

    def host(self, emu):
        return False

    def read(self, emu, addr):
        if addr == self.lsr:
            # (THRE, and RDR when a byte is there)
            return (1 << 5) | (1 if emu.rx else 0)
        if addr == self.iir:
            ier = emu.get(self.ier, 4)
            return 0x04 if emu.rx and ier & 1 else 0x02 if ier & 2 else 0x01
        if addr == self.rbr and not emu.get(self.lcr, 1) & 0x80:
            return emu.rx_byte()
        return None

    def write(self, emu, addr, value):
        # (offset 0 is THR, or DLL with the DLAB bit set)
        if addr == self.rbr and not emu.get(self.lcr, 1) & 0x80:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def irq(self, emu):
        ier = emu.get(self.ier, 4)
        if (emu.rx and ier & 1) or ier & 2:
            return self.irq_num
        return None


class LpcUsb:
    """lpc176x/usbserial.c: the usb device controller, driven through its
    serial interface engine commands (USBCmdCode / USBCmdData with the
    CCEMPTY / CDFULL handshake: select, select and clear, set endpoint
    status, clear / validate buffer, ...), endpoint realisation, the USBCtrl
    selected USBRx / USBTx data and length registers and the endpoint
    interrupts (EP_SLOW), for the UsbHost. Physical endpoint 2n is endpoint
    n out, 2n + 1 endpoint n in."""

    EP_SLOW, DEV_STAT, CCEMPTY, CDFULL, EP_RLZED = 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 8
    RD_EN, WR_EN = 1 << 0, 1 << 1

    def __init__(self, chip):
        u = lambda f: chip.at("LPC_USB", f)  # noqa: E731
        self.devintst, self.devinten = u("USBDevIntSt"), u("USBDevIntEn")
        self.devintclr, self.devintset = u("USBDevIntClr"), u("USBDevIntSet")
        self.cmdcode, self.cmddata = u("USBCmdCode"), u("USBCmdData")
        self.rxdata, self.txdata, self.rxplen, self.txplen = u("USBRxData"), u("USBTxData"), u("USBRxPLen"), u("USBTxPLen")
        self.ctrl = u("USBCtrl")
        self.epintst, self.epinten, self.epintclr = u("USBEpIntSt"), u("USBEpIntEn"), u("USBEpIntClr")
        self.maxpsize = u("USBMaxPSize")
        self.clkctrl, self.clkst = u("USBClkCtrl"), u("USBClkSt")
        self.irq_num = chip.const["USB_IRQn"]
        self.flags = 0
        self.epint = 0
        self.cmd = 0
        self.selected = 0
        self.data = 0
        self.connected = False
        # (per physical endpoint: its buffer's packet (or None), stalled)
        self.buf = [None] * 32
        self.stall = [False] * 32
        self.rx_packet = b""
        self.rx = []
        self.tx = []
        self.host_side = UsbHost(self, "lpc176x")

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.devintst:
            return self.flags | (self.EP_SLOW if self.epint & emu.get(self.epinten, 4) else 0)
        if addr == self.epintst:
            return self.epint
        if addr == self.cmddata:
            self.flags &= ~self.CDFULL
            return self.data
        if addr == self.clkst:
            return emu.get(self.clkctrl, 4)
        if addr == self.rxplen:
            # (PKT_RDY, DV, the length)
            return (1 << 11) | (1 << 10) | len(self.rx_packet)
        if addr == self.rxdata:
            return self.rx.pop(0) if self.rx else 0
        return None

    def write(self, emu, addr, value):
        if addr == self.devintclr:
            self.flags &= ~value
            return True
        if addr == self.devintset:
            self.flags |= value
            return True
        if addr == self.maxpsize:
            self.flags |= self.EP_RLZED
            return False
        if addr == self.epintclr:
            # (also selects the endpoint: its status in USBCmdData)
            for n in range(32):
                if value >> n & 1:
                    self.epint &= ~(1 << n)
                    self.data = self.select_status(n)
            self.flags |= self.CDFULL
            return True
        if addr == self.cmdcode:
            self.command(emu, (value >> 8) & 0xFF, (value >> 16) & 0xFF)
            return True
        if addr == self.ctrl:
            ep = ((value >> 2) & 0xF) * 2
            if value & self.RD_EN:
                self.rx_packet = self.buf[ep] or b""
                words = self.rx_packet + bytes(-len(self.rx_packet) % 4)
                self.rx = [struct.unpack_from("<I", words, i)[0] for i in range(0, len(words), 4)]
            if value & self.WR_EN:
                self.tx = []
            emu.put(addr, 4, value)
            return True
        if addr == self.txdata:
            self.tx.append(value)
            return True
        return False

    def used_bits(self, emu, addr):
        # (a USBTxData word's bytes past USBTxPLen are not sent; usbserial.c
        # copies whole words from its buffer)
        if addr == self.txdata:
            left = (emu.get(self.txplen, 4) & 0x3FF) - 4 * len(self.tx)
            if 0 <= left < 4:
                return (1 << (8 * left)) - 1
        return 0xFFFFFFFF

    def select_status(self, ep):
        # (FE: buffer full, ST: stalled)
        return (1 if self.buf[ep] is not None else 0) | (2 if self.stall[ep] else 0)

    def command(self, emu, phase, code):
        if phase == 0x05:
            # (the command phase)
            self.cmd = code
            if code == 0xFA:
                # validate buffer: the selected in endpoint's packet written
                n = emu.get(self.txplen, 4) & 0x3FF
                self.buf[self.selected] = b"".join(struct.pack("<I", w) for w in self.tx)[:n]
                self.tx = []
            elif code < 0x20:
                self.selected = code
        elif phase == 0x01:
            # (a command's data written)
            cmd = self.cmd
            if cmd & 0xE0 == 0x40:
                # set endpoint status: ST, or bit 7 (ep0's conditional stall)
                ep = cmd & 0x1F
                st = bool(code & 0x81)
                for e in ((0, 1) if code & 0x80 else (ep,)):
                    self.stall[e] = st
            elif cmd == 0xFE:
                self.connected = bool(code & 1)
        elif phase == 0x02:
            # (a command's data read)
            cmd = self.cmd
            if cmd < 0x20:
                self.data = self.select_status(cmd)
            elif cmd == 0xF2:
                # clear buffer: the selected out endpoint's packet read
                self.buf[self.selected] = None
                self.data = 0
            else:
                self.data = 0
            self.flags |= self.CDFULL
            return
        self.flags |= self.CCEMPTY

    def irq(self, emu):
        st = self.flags | (self.EP_SLOW if self.epint & emu.get(self.epinten, 4) else 0)
        if st & emu.get(self.devinten, 4):
            return self.irq_num
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def ready(self, emu):
        return self.connected and emu.get(self.devinten, 4) & self.EP_SLOW

    def reset(self, emu):
        self.flags |= self.DEV_STAT

    def setup(self, emu, data):
        self.buf[0] = bytes(data)
        self.stall[0] = self.stall[1] = False
        self.epint |= 1 << 0
        return True

    def out(self, emu, ep, data):
        p = ep * 2
        if self.stall[p]:
            return "stall"
        if self.buf[p] is not None:
            return "nak"
        self.buf[p] = bytes(data)
        self.epint |= 1 << p
        return "ack"

    def in_(self, emu, ep):
        p = ep * 2 + 1
        if self.stall[p]:
            return "stall", b""
        if self.buf[p] is None:
            return "nak", b""
        data, self.buf[p] = self.buf[p], None
        self.epint |= 1 << p
        return "ack", data


class Atsam(Ported):
    """atsam: the uart or the usb device (sam3x / same70 UOTGHS / USBHS, sam4
    UDP), DWT time"""

    def __init__(self, cfg):
        mcu = cfg["CONFIG_MCU"]
        chip = self.chip = Chip(os.path.join(ROOT, "src", "atsam", "chip", mcu + ".rs"))
        self.cpu = A.UC_CPU_ARM_CORTEX_M3 if cfg.get("CONFIG_MACH_SAM3X") else (
            A.UC_CPU_ARM_CORTEX_M7 if cfg.get("CONFIG_MACH_SAME70") else A.UC_CPU_ARM_CORTEX_M4)
        self.time = DwtTime(chip)
        self.read_rules = {}
        if not cfg.get("CONFIG_USBSERIAL"):
            self.port = AtsamUart(cfg, chip)
        elif cfg.get("CONFIG_MACH_SAM4"):
            self.port = SamUdp(chip)
        else:
            self.port = SamUotghs(cfg, chip)
            self.mmio = [(self.port.ram, 10 * 0x8000)]
        self.eefc = Eefc(chip)
        # (same70_sysinit.c: the dma copy of the code to the itcm, its length)
        self.size_regs = {chip.ptr["REG_XDMAC_CUBC0"]} if cfg.get("CONFIG_MACH_SAME70") else set()

    def uart_read(self, emu, addr):
        v = self.eefc.read(emu, addr)
        return self.port.read(emu, addr) if v is None else v

    def uart_write(self, emu, addr, value):
        return self.eefc.write(emu, addr, value) or self.port.write(emu, addr, value)


class Eefc:
    """the (first) flash controller: atsam/chipid.c's unique identifier
    area, which the STUI command maps over the start of the flash until the
    SPUI command (FSR FRDY is clear meanwhile), and same70_sysinit.c's
    GPNVM bits (GGPB reads them to FRR, SGPB / CGPB set / clear one). The
    emulator cannot follow the reset same70_sysinit.c does after setting the
    TCM bits, so they start as a first boot left them: GPNVM1 (boot from
    flash), 7 and 8 (128KiB TCMs)."""

    UID = bytes(range(0x10, 0x20))
    GPNVM = (1 << 1) | (1 << 7) | (1 << 8)

    def __init__(self, chip):
        c = chip.const
        efc = "EFC0" if "EFC0" in chip.ptr else "EFC"
        self.fsr, self.fcr, self.frr = chip.at(efc, "EEFC_FSR"), chip.at(efc, "EEFC_FCR"), chip.at(efc, "EEFC_FRR")
        self.flash = c["IFLASH0_ADDR"] if "IFLASH0_ADDR" in c else c["IFLASH_ADDR"]
        self.FRDY, self.FCMD = c["EEFC_FSR_FRDY"], c["EEFC_FCR_FCMD_Msk"]
        self.STUI, self.SPUI = c["EEFC_FCR_FCMD_STUI"], c["EEFC_FCR_FCMD_SPUI"]
        self.GGPB, self.SGPB, self.CGPB = c["EEFC_FCR_FCMD_GGPB"], c["EEFC_FCR_FCMD_SGPB"], c["EEFC_FCR_FCMD_CGPB"]
        self.FARG_Pos, self.FARG_Msk = c["EEFC_FCR_FARG_Pos"], c["EEFC_FCR_FARG_Msk"]
        self.saved = None
        self.gpnvm = self.GPNVM
        self.result = 0

    def read(self, emu, addr):
        if addr == self.fsr:
            return 0 if self.saved is not None else self.FRDY
        if addr == self.frr:
            return self.result
        return None

    def write(self, emu, addr, value):
        if addr != self.fcr:
            return False
        cmd = value & self.FCMD
        arg = (value & self.FARG_Msk) >> self.FARG_Pos
        if cmd == self.GGPB:
            self.result = self.gpnvm
        elif cmd == self.SGPB:
            self.gpnvm |= 1 << arg
        elif cmd == self.CGPB:
            self.gpnvm &= ~(1 << arg)
        elif cmd == self.STUI and self.saved is None:
            self.saved = bytes(emu.uc.mem_read(self.flash, len(self.UID)))
            emu.uc.mem_write(self.flash, self.UID)
        elif cmd == self.SPUI and self.saved is not None:
            emu.uc.mem_write(self.flash, self.saved)
            self.saved = None
        return True


class AtsamUart:
    """atsam/serial.c: the uart (SR RXRDY / TXRDY, IER / IDR mask)"""

    def __init__(self, cfg, chip):
        c = chip.const
        # (serial.c: sam3x UART, sam4s UART1, sam4e UART0)
        port, irq = ("UART", "UART_IRQn") if cfg.get("CONFIG_MACH_SAM3X") else (
            ("UART1", "UART1_IRQn") if cfg.get("CONFIG_MACH_SAM4S") else ("UART0", "UART0_IRQn"))
        self.irq_num = c[irq]
        self.sr, self.rhr, self.thr = chip.at(port, "UART_SR"), chip.at(port, "UART_RHR"), chip.at(port, "UART_THR")
        self.ier, self.idr, self.imr = chip.at(port, "UART_IER"), chip.at(port, "UART_IDR"), chip.at(port, "UART_IMR")
        self.RXRDY, self.TXRDY, self.TXEMPTY = c["UART_SR_RXRDY"], c["UART_SR_TXRDY"], c["UART_SR_TXEMPTY"]
        self.mask = 0

    def host(self, emu):
        return False

    def read(self, emu, addr):
        if addr == self.sr:
            return self.TXRDY | self.TXEMPTY | (self.RXRDY if emu.rx else 0)
        if addr == self.rhr:
            return emu.rx_byte()
        if addr == self.imr:
            return self.mask
        return None

    def write(self, emu, addr, value):
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

    def irq(self, emu):
        if (emu.rx and self.mask & self.RXRDY) or self.mask & self.TXRDY:
            return self.irq_num
        return None


class SamUdp:
    """atsam/sam4_usb.c: the sam4 UDP device controller (UDP_CSR flags
    cleared by writing 0, TXPKTRDY set by writing 1, FORCESTALL; the UDP_FDR
    fifo registers; two banks for the bulk out endpoint; ISR / IMR) for the
    UsbHost"""

    def __init__(self, chip):
        c = chip.const
        u = lambda f: chip.at("UDP", f)  # noqa: E731
        self.ier, self.idr, self.imr = u("UDP_IER"), u("UDP_IDR"), u("UDP_IMR")
        self.isr, self.icr, self.txvc = u("UDP_ISR"), u("UDP_ICR"), u("UDP_TXVC")
        self.csr, self.fdr = u("UDP_CSR"), u("UDP_FDR")
        self.irq_num = c["UDP_IRQn"]
        self.TXCOMP, self.BK0, self.RXSETUP = c["UDP_CSR_TXCOMP"], c["UDP_CSR_RX_DATA_BK0"], c["UDP_CSR_RXSETUP"]
        self.STALLSENT, self.TXPKTRDY = c["UDP_CSR_STALLSENT"], c["UDP_CSR_TXPKTRDY"]
        self.FORCESTALL, self.BK1 = c["UDP_CSR_FORCESTALL"], c["UDP_CSR_RX_DATA_BK1"]
        self.CNT_Pos, self.CNT_Msk = c["UDP_CSR_RXBYTECNT_Pos"], c["UDP_CSR_RXBYTECNT_Msk"]
        self.ENDBUSRES, self.PUON = c["UDP_ISR_ENDBUSRES"], c["UDP_TXVC_PUON"]
        self.FLAGS = self.TXCOMP | self.BK0 | self.RXSETUP | self.STALLSENT | self.BK1
        self.csrs = [0] * 8
        self.mask = 0
        self.isr_flags = 0
        # (per endpoint: the received packets (bank order), the packet written)
        self.rxq = [[] for _ in range(8)]
        self.txq = [b""] * 8
        self.next_bank = [0] * 8
        self.host_side = UsbHost(self)

    def ep(self, addr, base):
        if base <= addr < base + 32 and not addr & 3:
            return (addr - base) // 4
        return None

    # ---- the cpu's registers

    def read(self, emu, addr):
        n = self.ep(addr, self.csr)
        if n is not None:
            cnt = len(self.rxq[n][0]) if self.rxq[n] else 0
            return (self.csrs[n] & ~self.CNT_Msk) | (cnt << self.CNT_Pos)
        n = self.ep(addr, self.fdr)
        if n is not None:
            if self.rxq[n] and self.rxq[n][0]:
                b, self.rxq[n][0] = self.rxq[n][0][0], self.rxq[n][0][1:]
                return b
            return 0
        if addr == self.isr:
            return self.isr_value()
        if addr == self.imr:
            return self.mask
        return None

    def isr_value(self):
        return self.isr_flags | sum(1 << n for n in range(8) if self.csrs[n] & self.FLAGS)

    def write(self, emu, addr, value):
        n = self.ep(addr, self.csr)
        if n is not None:
            old = self.csrs[n]
            # (the flags: cleared by 0; TXPKTRDY: set by 1; the rest written)
            for bank in (self.BK0, self.BK1):
                if old & bank and not value & bank and self.rxq[n]:
                    self.rxq[n].pop(0)
            if old & self.RXSETUP and not value & self.RXSETUP and self.rxq[n]:
                self.rxq[n].pop(0)
            keep = old & value & self.FLAGS
            rw = value & ~(self.FLAGS | self.TXPKTRDY | self.CNT_Msk)
            self.csrs[n] = keep | rw | ((old | value) & self.TXPKTRDY)
            return True
        n = self.ep(addr, self.fdr)
        if n is not None:
            self.txq[n] += bytes([value & 0xFF])
            return True
        if addr == self.ier:
            self.mask |= value
            return True
        if addr == self.idr:
            self.mask &= ~value
            return True
        if addr == self.icr:
            self.isr_flags &= ~value
            return True
        return False

    def irq(self, emu):
        # (end of bus reset is not maskable)
        if self.isr_value() & (self.mask | self.ENDBUSRES):
            return self.irq_num
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def ready(self, emu):
        return emu.get(self.txvc, 4) & self.PUON

    def reset(self, emu):
        self.isr_flags |= self.ENDBUSRES

    def setup(self, emu, data):
        # (after the device has handled the end of the bus reset)
        if self.isr_flags & self.ENDBUSRES:
            return False
        self.rxq[0] = [bytes(data)]
        self.csrs[0] = (self.csrs[0] & ~self.FORCESTALL) | self.RXSETUP
        return True

    def out(self, emu, ep, data):
        c = self.csrs[ep]
        if c & self.FORCESTALL:
            return "stall"
        banks = (self.BK0, self.BK1) if ep else (self.BK0,)
        b = banks[self.next_bank[ep] % len(banks)]
        if c & b or c & self.RXSETUP:
            return "nak"
        self.rxq[ep].append(bytes(data))
        self.csrs[ep] = c | b
        self.next_bank[ep] += 1
        return "ack"

    def in_(self, emu, ep):
        c = self.csrs[ep]
        if c & self.FORCESTALL:
            return "stall", b""
        if not c & self.TXPKTRDY:
            return "nak", b""
        data, self.txq[ep] = self.txq[ep], b""
        self.csrs[ep] = (c & ~self.TXPKTRDY) | self.TXCOMP
        return "ack", data


class SamUotghs:
    """atsam/sam3_usb.c: the sam3x UOTGHS / same70 USBHS device controller
    (DEVISR / DEVIMR with their clear / set / enable / disable registers,
    the endpoint status / mask registers likewise, the fifo windows at the
    usb ram address, one bank per endpoint here) for the UsbHost"""

    def __init__(self, cfg, chip):
        c = chip.const
        e70 = cfg.get("CONFIG_MACH_SAME70")
        n = "USBHS" if e70 else "UOTGHS"
        u = lambda f: chip.at(n, n + "_" + f)  # noqa: E731
        self.devisr, self.devicr, self.devifr = u("DEVISR"), u("DEVICR"), u("DEVIFR")
        self.devimr, self.devidr, self.devier = u("DEVIMR"), u("DEVIDR"), u("DEVIER")
        self.eptisr, self.epticr, self.eptifr = u("DEVEPTISR"), u("DEVEPTICR"), u("DEVEPTIFR")
        self.eptimr, self.eptier, self.eptidr = u("DEVEPTIMR"), u("DEVEPTIER"), u("DEVEPTIDR")
        self.ctrl = u("CTRL")
        self.irq_num = c[n + "_IRQn"]
        # (the usb ram: same70_usb.h's USBHS_RAM_ADDR on the same70)
        if e70:
            h = open(os.path.join(mcu_run.KALICO, "src", "atsam", "same70_usb.h")).read()
            self.ram = int(re.search(r"#define USBHS_RAM_ADDR (0x[0-9a-fA-F]+)", h).group(1), 16)
        else:
            self.ram = c["UOTGHS_RAM_ADDR"]
        self.EORST, self.PEP_0 = c[n + "_DEVISR_EORST"], c[n + "_DEVISR_PEP_0"]
        self.TXINI, self.RXOUTI, self.RXSTPI = c[n + "_DEVEPTISR_TXINI"], c[n + "_DEVEPTISR_RXOUTI"], c[n + "_DEVEPTISR_RXSTPI"]
        self.BYCT_Pos = c[n + "_DEVEPTISR_BYCT_Pos"]
        self.STALLRQ, self.USBE = c[n + "_DEVEPTIMR_STALLRQ"], c[n + "_CTRL_USBE"]
        self.dev_flags = 0
        self.dev_mask = 0
        # (per endpoint: status flags, masks, the received packet, written bytes)
        self.flags = [self.TXINI] * 10
        self.masks = [0] * 10
        self.rx = [b""] * 10
        self.tx = [b""] * 10
        self.host_side = UsbHost(self)

    def ep(self, addr, base):
        if base <= addr < base + 40 and not addr & 3:
            return (addr - base) // 4
        return None

    # ---- the cpu's registers

    def devisr_value(self):
        return self.dev_flags | sum(self.PEP_0 << n for n in range(10) if self.flags[n] & self.masks[n])

    def read(self, emu, addr):
        if addr == self.devisr:
            return self.devisr_value()
        if addr == self.devimr:
            return self.dev_mask
        n = self.ep(addr, self.eptisr)
        if n is not None:
            return self.flags[n] | (len(self.rx[n]) << self.BYCT_Pos)
        n = self.ep(addr, self.eptimr)
        if n is not None:
            return self.masks[n]
        if self.ram <= addr < self.ram + 10 * 0x8000:
            n = (addr - self.ram) // 0x8000
            if self.rx[n]:
                b, self.rx[n] = self.rx[n][0], self.rx[n][1:]
                return b
            return 0
        return None

    def write(self, emu, addr, value):
        if addr == self.devicr:
            self.dev_flags &= ~value
            return True
        if addr == self.devifr:
            self.dev_flags |= value
            return True
        if addr == self.devier:
            self.dev_mask |= value
            return True
        if addr == self.devidr:
            self.dev_mask &= ~value
            return True
        for base, op in ((self.epticr, "clr"), (self.eptifr, "set"), (self.eptier, "mset"), (self.eptidr, "mclr")):
            n = self.ep(addr, base)
            if n is not None:
                if op == "clr":
                    if value & (self.RXOUTI | self.RXSTPI) and self.flags[n] & (self.RXOUTI | self.RXSTPI):
                        self.rx[n] = b""
                    self.flags[n] &= ~value
                elif op == "set":
                    self.flags[n] |= value
                elif op == "mset":
                    self.masks[n] |= value
                else:
                    self.masks[n] &= ~value
                return True
        if self.ram <= addr < self.ram + 10 * 0x8000:
            n = (addr - self.ram) // 0x8000
            self.tx[n] += bytes([value & 0xFF])
            return True
        return False

    def irq(self, emu):
        if self.devisr_value() & self.dev_mask:
            return self.irq_num
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def ready(self, emu):
        return emu.get(self.ctrl, 4) & self.USBE and self.dev_mask & self.EORST

    def reset(self, emu):
        self.dev_flags |= self.EORST

    def setup(self, emu, data):
        self.rx[0] = bytes(data)
        self.flags[0] |= self.RXSTPI
        self.masks[0] &= ~self.STALLRQ
        return True

    def out(self, emu, ep, data):
        if self.masks[ep] & self.STALLRQ:
            return "stall"
        if self.flags[ep] & (self.RXOUTI | self.RXSTPI):
            return "nak"
        self.rx[ep] = bytes(data)
        self.flags[ep] |= self.RXOUTI
        return "ack"

    def in_(self, emu, ep):
        if self.masks[ep] & self.STALLRQ:
            return "stall", b""
        if self.flags[ep] & self.TXINI:
            return "nak", b""
        data, self.tx[ep] = self.tx[ep], b""
        self.flags[ep] |= self.TXINI
        return "ack", data


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


class Atsamd(Ported):
    """atsamd: the SERCOM0 usart or the usb device, the TC (samd21, samc21)
    or DWT (samd51, same5x) time; the nvm calibration rows (GET_FUSE: 0)"""

    def __init__(self, cfg):
        chip = self.chip = Chip(os.path.join(ROOT, "src", "atsamd", "chip", cfg["CONFIG_MCU"] + ".rs"))
        samx5 = cfg.get("CONFIG_MACH_SAMX5")
        self.cpu = A.UC_CPU_ARM_CORTEX_M4 if samx5 else A.UC_CPU_ARM_CORTEX_M0
        self.time = DwtTime(chip) if samx5 else TcTime(chip, "TC0" if cfg.get("CONFIG_MACH_SAMC21") else "TC4")
        self.read_rules = {}
        self.mmio = [(0x00800000, 0x10000)]
        self.port = SamdUsb(cfg, chip) if cfg.get("CONFIG_USBSERIAL") else AtsamdUsart(cfg, chip)
        self.pointer_regs = {self.port.descadd} if cfg.get("CONFIG_USBSERIAL") else set()


class AtsamdUsart:
    """atsamd/serial.c: the SERCOM0 usart (INTFLAG DRE / RXC, INTENSET /
    INTENCLR mask)"""

    def __init__(self, cfg, chip):
        c = chip.const
        samx5 = cfg.get("CONFIG_MACH_SAMX5")
        u = lambda f: chip.ptr["SERCOM0"] + chip.off[("SercomUsart", f)]  # noqa: E731
        self.intflag, self.data = u("INTFLAG"), u("DATA")
        self.intenset, self.intenclr = u("INTENSET"), u("INTENCLR")
        self.DRE, self.TXC, self.RXC = c["SERCOM_USART_INTFLAG_DRE"], c["SERCOM_USART_INTFLAG_TXC"], c["SERCOM_USART_INTFLAG_RXC"]
        # (samd51: one irq per flag, serial.c enables them all)
        self.dre_irq = c["SERCOM0_0_IRQn"] if samx5 else c["SERCOM0_IRQn"]
        self.rxc_irq = c["SERCOM0_2_IRQn"] if samx5 else c["SERCOM0_IRQn"]
        self.mask = 0

    def read(self, emu, addr):
        if addr == self.intflag:
            return self.DRE | self.TXC | (self.RXC if emu.rx else 0)
        if addr == self.data:
            return emu.rx_byte()
        if addr in (self.intenset, self.intenclr):
            return self.mask
        return None

    def write(self, emu, addr, value):
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

    def irq(self, emu):
        if emu.rx and self.mask & self.RXC:
            return self.rxc_irq
        if self.mask & self.DRE:
            return self.dre_irq
        return None

    def host(self, emu):
        return False


class SamdUsb:
    """atsamd/usbserial.c: the usb device controller (the endpoint
    descriptor table in ram with each bank's ADDR / PCKSIZE; EPSTATUS BKnRDY
    and STALLRQn, EPINTFLAG TRCPTn / RXSTP, EPINTSMRY; bank 0 out, bank 1
    in) for the UsbHost"""

    def __init__(self, cfg, chip):
        c = chip.const
        base = chip.ptr["USB"]
        d = lambda f: base + chip.off[("UsbDevice", f)]  # noqa: E731
        self.ctrla, self.intenset, self.intenclr = d("CTRLA"), d("INTENSET"), d("INTENCLR")
        self.intflag, self.epintsmry, self.descadd = d("INTFLAG"), d("EPINTSMRY"), d("DESCADD")
        self.ep = d("DeviceEndpoint")
        e = lambda f: chip.off[("UsbDeviceEndpoint", f)]  # noqa: E731
        self.EPSTATUSCLR, self.EPSTATUSSET, self.EPSTATUS = e("EPSTATUSCLR"), e("EPSTATUSSET"), e("EPSTATUS")
        self.EPINTFLAG, self.EPINTENCLR, self.EPINTENSET = e("EPINTFLAG"), e("EPINTENCLR"), e("EPINTENSET")
        self.EPSIZE = 32
        self.ADDR, self.PCKSIZE = chip.off[("UsbDeviceDescBank", "ADDR")], chip.off[("UsbDeviceDescBank", "PCKSIZE")]
        self.BANKSIZE = 16
        self.ENABLE, self.EORST = c["USB_CTRLA_ENABLE"], c["USB_DEVICE_INTFLAG_EORST"]
        self.BK0RDY, self.BK1RDY = c["USB_DEVICE_EPSTATUS_BK0RDY"], c["USB_DEVICE_EPSTATUS_BK1RDY"]
        self.STALLRQ0, self.STALLRQ1 = c["USB_DEVICE_EPSTATUS_STALLRQ0"], c["USB_DEVICE_EPSTATUS_STALLRQ1"]
        self.TRCPT0, self.TRCPT1 = c["USB_DEVICE_EPINTFLAG_TRCPT0"], c["USB_DEVICE_EPINTFLAG_TRCPT1"]
        self.RXSTP = c["USB_DEVICE_EPINTFLAG_RXSTP"]
        self.COUNT = c["USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk"]
        # (samd51: an irq per kind of event, usbserial.c enables them all)
        samx5 = cfg.get("CONFIG_MACH_SAMX5")
        self.irqs = [c["USB_%d_IRQn" % i] for i in range(4)] if samx5 else [c["USB_IRQn"]] * 4
        self.dev_mask = 0
        self.status = [0] * 8
        self.flags = [0] * 8
        self.masks = [0] * 8
        self.host_side = UsbHost(self)

    def ep_reg(self, addr):
        if self.ep <= addr < self.ep + 8 * self.EPSIZE:
            return (addr - self.ep) // self.EPSIZE, (addr - self.ep) % self.EPSIZE
        return None

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.intflag:
            return emu.get(addr, 2)
        if addr in (self.intenset, self.intenclr):
            return self.dev_mask
        if addr == self.epintsmry:
            return sum(1 << n for n in range(8) if self.flags[n] & self.masks[n])
        r = self.ep_reg(addr)
        if r:
            n, off = r
            if off == self.EPSTATUS:
                return self.status[n]
            if off == self.EPINTFLAG:
                return self.flags[n]
            if off in (self.EPINTENSET, self.EPINTENCLR):
                return self.masks[n]
        return None

    def write(self, emu, addr, value):
        if addr == self.intflag:
            emu.put(addr, 2, emu.get(addr, 2) & ~value)
            return True
        if addr == self.intenset:
            self.dev_mask |= value
            return True
        if addr == self.intenclr:
            self.dev_mask &= ~value
            return True
        r = self.ep_reg(addr)
        if r:
            n, off = r
            if off == self.EPSTATUSSET:
                self.status[n] |= value
                return True
            if off == self.EPSTATUSCLR:
                self.status[n] &= ~value
                return True
            if off == self.EPINTFLAG:
                self.flags[n] &= ~value
                return True
            if off == self.EPINTENSET:
                self.masks[n] |= value
                return True
            if off == self.EPINTENCLR:
                self.masks[n] &= ~value
                return True
        return False

    def irq(self, emu):
        if emu.get(self.intflag, 2) & self.dev_mask:
            return self.irqs[0]
        for n in range(8):
            f = self.flags[n] & self.masks[n]
            if f & self.RXSTP:
                return self.irqs[0]
            if f & self.TRCPT0:
                return self.irqs[2]
            if f & self.TRCPT1:
                return self.irqs[3]
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def bank(self, emu, ep, b):
        """(buffer address, PCKSIZE address) of an endpoint bank, from the
        descriptor table in ram"""
        desc = emu.get(self.descadd, 4) + ep * 2 * self.BANKSIZE + b * self.BANKSIZE
        addr = struct.unpack("<I", emu.uc.mem_read(desc + self.ADDR, 4))[0]
        return addr, desc + self.PCKSIZE

    def ready(self, emu):
        return emu.get(self.ctrla, 1) & self.ENABLE and self.dev_mask & self.EORST

    def reset(self, emu):
        emu.put(self.intflag, 2, emu.get(self.intflag, 2) | self.EORST)

    def setup(self, emu, data):
        buf, pck = self.bank(emu, 0, 0)
        emu.uc.mem_write(buf, data)
        size = struct.unpack("<I", emu.uc.mem_read(pck, 4))[0]
        emu.uc.mem_write(pck, struct.pack("<I", (size & ~self.COUNT) | len(data)))
        # (a setup ends a stall)
        self.status[0] = (self.status[0] & ~(self.STALLRQ0 | self.STALLRQ1)) | self.BK0RDY
        self.flags[0] |= self.RXSTP
        return True

    def out(self, emu, ep, data):
        if self.status[ep] & self.STALLRQ0:
            return "stall"
        if self.status[ep] & self.BK0RDY:
            return "nak"
        buf, pck = self.bank(emu, ep, 0)
        emu.uc.mem_write(buf, data)
        size = struct.unpack("<I", emu.uc.mem_read(pck, 4))[0]
        emu.uc.mem_write(pck, struct.pack("<I", (size & ~self.COUNT) | len(data)))
        self.status[ep] |= self.BK0RDY
        self.flags[ep] |= self.TRCPT0
        return "ack"

    def in_(self, emu, ep):
        if self.status[ep] & self.STALLRQ1:
            return "stall", b""
        if not self.status[ep] & self.BK1RDY:
            return "nak", b""
        buf, pck = self.bank(emu, ep, 1)
        n = struct.unpack("<I", emu.uc.mem_read(pck, 4))[0] & self.COUNT
        data = bytes(emu.uc.mem_read(buf, n))
        self.status[ep] &= ~self.BK1RDY
        self.flags[ep] |= self.TRCPT1
        return "ack", data


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
    """rp2040 / rp2350: the PL011 uart or the usb device; the rp2040's TIMER
    or the rp2350's DWT time (generic/armcm_timer.c); the SIO block, and the
    xor / set / clear aliases of the peripheral registers (hw_set_alias)"""

    def __init__(self, cfg):
        self.rp2350 = bool(cfg.get("CONFIG_MACH_RP2350"))
        chip = self.chip = Chip(os.path.join(ROOT, "src", "rp2040", "chip", cfg["CONFIG_MCU"] + ".rs"))
        c = chip.const
        self.mmio = [(chip.ptr["sio_hw"], 0x1000)]
        if self.rp2350:
            self.cpu = A.UC_CPU_ARM_CORTEX_M33
            self.time = DwtTime(chip)
            self.read_rules = {}
        else:
            self.cpu = A.UC_CPU_ARM_CORTEX_M0
            self.time = RpTime(chip)
            self.mmio.append((chip.ptr["ssi_hw"], 0x1000))
            # (the flash interface, bootrom.c's unique id read and the image's
            # boot2: transmit fifo not full and empty, receive fifo not empty)
            ready = c["SSI_SR_TFNF_BITS"] | c["SSI_SR_TFE_BITS"] | c["SSI_SR_RFNE_BITS"]
            # (and the flash's answers: 0, its status not busy)
            self.read_rules = {chip.at("ssi_hw", "sr"): lambda v: ready, chip.at("ssi_hw", "dr0"): lambda v: 0}
        self.port = RpUsb(chip) if cfg.get("CONFIG_USBSERIAL") else Rp2040Uart(cfg, chip)

    # (the boot rom bootrom.c / rp2350_bootrom.c use: the halfword pointer to
    # the lookup function (pico-sdk boot/bootrom_constants.h
    # BOOTROM_TABLE_LOOKUP_OFFSET: 0x18, rp2350 arm 0x16), on the rp2040 the
    # one to the function table at 0x14; the functions it gives
    # (connect_internal_flash, flash_exit_xip, flash_flush_cache,
    # reset_usb_boot, reboot) return at once, but the rp2350's get_sys_info,
    # which fills its buffer)
    ROM_TABLE, ROM_LOOKUP, ROM_STUBS, ROM_SYS_INFO = 0x100, 0x200, 0x300, 0x400
    GET_SYS_INFO = ord("G") | ord("S") << 8
    SYS_INFO = struct.pack("<9I", *range(0x11111111, 0x9999999A, 0x11111111))

    def setup(self, uc):
        uc.mem_map(0, 0x4000, unicorn.UC_PROT_READ | unicorn.UC_PROT_EXEC)
        if self.rp2350:
            uc.mem_write(0x16, struct.pack("<H", self.ROM_LOOKUP | 1))
        else:
            uc.mem_write(0x14, struct.pack("<H", self.ROM_TABLE))
            uc.mem_write(0x18, struct.pack("<H", self.ROM_LOOKUP | 1))
        uc.mem_write(self.ROM_STUBS, struct.pack("<H", 0x4770) * 64)  # bx lr
        uc.hook_add(unicorn.UC_HOOK_CODE, self.rom_lookup, begin=self.ROM_LOOKUP, end=self.ROM_LOOKUP)
        uc.hook_add(unicorn.UC_HOOK_CODE, self.rom_sys_info, begin=self.ROM_SYS_INFO, end=self.ROM_SYS_INFO)

    def rom_lookup(self, uc, address, size, user):
        # (rom_table_lookup(table, code) / rom_table_lookup(code, mask): a
        # stub per code)
        code = uc.reg_read(A.UC_ARM_REG_R0 if self.rp2350 else A.UC_ARM_REG_R1) & 0xFFFF
        stub = self.ROM_SYS_INFO if self.rp2350 and code == self.GET_SYS_INFO else self.ROM_STUBS + 2 * (code % 64)
        uc.reg_write(A.UC_ARM_REG_R0, stub | 1)
        uc.reg_write(A.UC_ARM_REG_PC, uc.reg_read(A.UC_ARM_REG_LR))

    def rom_sys_info(self, uc, address, size, user):
        # (get_sys_info(out, words, flags): fixed words, their count)
        n = min(uc.reg_read(A.UC_ARM_REG_R1), len(self.SYS_INFO) // 4)
        uc.mem_write(uc.reg_read(A.UC_ARM_REG_R0), self.SYS_INFO[: 4 * n])
        uc.reg_write(A.UC_ARM_REG_R0, n)
        uc.reg_write(A.UC_ARM_REG_PC, uc.reg_read(A.UC_ARM_REG_LR))

    def alias(self, addr):
        """(the register and operation of an alias address)"""
        if 0x40000000 <= addr < 0x60000000 and addr & 0x3000 and not 0x50100000 <= addr < 0x50110000:
            return addr & ~0x3000, (addr >> 12) & 3
        return None

    def uart_read(self, emu, addr):
        return self.port.read(emu, addr)

    def uart_write(self, emu, addr, value):
        return self.port.write(emu, addr, value)

    def uart_irq(self, emu):
        return self.port.irq(emu)

    def host(self, emu):
        return self.port.host(emu)


class Rp2040Uart:
    """rp2040/serial.c: the PL011 uart (fr RXFE, imsc, mis)"""

    def __init__(self, cfg, chip):
        c = chip.const
        n = 1 if any(k.startswith("CONFIG_RPXXXX_SERIAL_UART1") and v == 1 for k, v in cfg.items()) else 0
        p = "uart%d_hw" % n
        self.dr, self.fr = chip.at(p, "dr"), chip.at(p, "fr")
        self.imsc, self.mis = chip.at(p, "imsc"), chip.at(p, "mis")
        self.irq_num = c["UART%d_IRQ_IRQn" % n]
        self.RXFE = c["UART_UARTFR_RXFE_BITS"]
        self.RX = c["UART_UARTIMSC_RXIM_BITS"] | c["UART_UARTIMSC_RTIM_BITS"]

    def host(self, emu):
        return False

    def read(self, emu, addr):
        if addr == self.fr:
            return 0 if emu.rx else self.RXFE
        if addr == self.dr:
            return emu.rx_byte()
        if addr == self.mis:
            return emu.get(self.imsc, 4) & (self.RX if emu.rx else 0)
        return None

    def write(self, emu, addr, value):
        if addr == self.dr:
            emu.tx += bytes([value & 0xFF])
            return True
        return False

    def irq(self, emu):
        if emu.rx and emu.get(self.imsc, 4) & self.RX:
            return self.irq_num
        return None


class RpUsb:
    """rp2040/usbserial.c: the usb device controller (the dual port ram's
    setup packet, endpoint buffer controls FULL / AVAIL / LEN / STALL and
    buffers, double buffered bulk endpoints; sie_status, buf_status, intr /
    inte / ints) for the UsbHost"""

    def __init__(self, chip):
        c = chip.const
        h = lambda f: chip.at("usb_hw", f)  # noqa: E731
        self.intr, self.inte, self.ints = h("intr"), h("inte"), h("ints")
        self.buf_status, self.sie_status = h("buf_status"), h("sie_status")
        self.main_ctrl, self.sie_ctrl = h("main_ctrl"), h("sie_ctrl")
        self.ep_stall_arm = h("ep_stall_arm")
        dp = chip.ptr["usb_dpram"]
        self.setup_packet = dp + chip.off[("usb_device_dpram_t", "setup_packet")]
        self.ep_ctrl = dp + chip.off[("usb_device_dpram_t", "ep_ctrl")]
        self.ep_buf_ctrl = dp + chip.off[("usb_device_dpram_t", "ep_buf_ctrl")]
        self.out_off = chip.off[("usb_device_dpram_ep_buf_ctrl", "out")]
        self.dpram = dp
        self.irq_num = c["USBCTRL_IRQ_IRQn"]
        self.SETUP_REQ, self.BUFF_STATUS = c["USB_INTR_SETUP_REQ_BITS"], c["USB_INTR_BUFF_STATUS_BITS"]
        self.BUS_RESET = c["USB_INTR_BUS_RESET_BITS"]
        self.SETUP_REC, self.SIE_BUS_RESET = c["USB_SIE_STATUS_SETUP_REC_BITS"], c["USB_SIE_STATUS_BUS_RESET_BITS"]
        self.FULL, self.AVAIL, self.STALL = c["USB_BUF_CTRL_FULL"], c["USB_BUF_CTRL_AVAIL"], c["USB_BUF_CTRL_STALL"]
        self.LEN = c["USB_BUF_CTRL_LEN_MASK"]
        self.CONTROLLER_EN, self.PULLUP_EN = c["USB_MAIN_CTRL_CONTROLLER_EN_BITS"], c["USB_SIE_CTRL_PULLUP_EN_BITS"]
        self.DBUF = c["EP_CTRL_DOUBLE_BUFFERED_BITS"]
        # (the buffer the controller uses next, per endpoint and direction)
        self.next_buf = {}
        self.host_side = UsbHost(self)

    def bufctrl(self, ep, out):
        return self.ep_buf_ctrl + ep * 8 + (self.out_off if out else 0)

    # ---- the cpu's registers

    def read(self, emu, addr):
        if addr == self.intr:
            return self.intr_value(emu)
        if addr == self.ints:
            return self.intr_value(emu) & emu.get(self.inte, 4)
        return None

    def intr_value(self, emu):
        v = 0
        sie = emu.get(self.sie_status, 4)
        if sie & self.SETUP_REC:
            v |= self.SETUP_REQ
        if sie & self.SIE_BUS_RESET:
            v |= self.BUS_RESET
        if emu.get(self.buf_status, 4):
            v |= self.BUFF_STATUS
        return v

    def write(self, emu, addr, value):
        if addr in (self.sie_status, self.buf_status):
            # (cleared by writing 1)
            emu.put(addr, 4, emu.get(addr, 4) & ~value)
            return True
        if self.ep_buf_ctrl <= addr < self.ep_buf_ctrl + 16 * 8 and emu.write_size == 4:
            # (a whole buffer control word: the controller starts at buffer 0)
            self.next_buf[(addr - self.ep_buf_ctrl) // 4] = 0
        return False

    def irq(self, emu):
        if self.intr_value(emu) & emu.get(self.inte, 4):
            return self.irq_num
        return None

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def buffer(self, emu, ep, out):
        """(buffer control address, data address) of the endpoint's next buffer"""
        key = ep * 2 + (1 if out else 0)
        b = self.next_buf.get(key, 0)
        ctl = emu.get(self.ep_ctrl + (ep - 1) * 8 + (4 if out else 0), 4) if ep else 0
        if ep == 0:
            data = self.dpram + 0x100
        else:
            data = self.dpram + (ctl & 0xFFC0) + b * 64
        return self.bufctrl(ep, out) + 2 * b, data, key, ctl & self.DBUF

    def done(self, emu, key, dbuf):
        if dbuf:
            self.next_buf[key] = self.next_buf.get(key, 0) ^ 1
        emu.put(self.buf_status, 4, emu.get(self.buf_status, 4) | (1 << key))

    def ready(self, emu):
        return emu.get(self.main_ctrl, 4) & self.CONTROLLER_EN and emu.get(self.sie_ctrl, 4) & self.PULLUP_EN

    def reset(self, emu):
        emu.put(self.sie_status, 4, emu.get(self.sie_status, 4) | self.SIE_BUS_RESET)

    def setup(self, emu, data):
        for i, b in enumerate(data):
            emu.put(self.setup_packet + i, 1, b)
        emu.put(self.sie_status, 4, emu.get(self.sie_status, 4) | self.SETUP_REC)
        # (a setup disarms the ep0 stall)
        emu.put(self.ep_stall_arm, 4, 0)
        return True

    def stalled(self, emu, ep, out, epb):
        # (ep0 stalls only while armed: ep_stall_arm bit 0 in, bit 1 out)
        if ep == 0 and not emu.get(self.ep_stall_arm, 4) & (2 if out else 1):
            return False
        return epb & self.STALL

    def out(self, emu, ep, data):
        a, buf, key, dbuf = self.buffer(emu, ep, True)
        epb = emu.get(a, 2)
        if self.stalled(emu, ep, True, epb):
            return "stall"
        # (ep0's data stage after the device has read the setup: until then
        # its out buffer is the previous request's, which reading the setup
        # re-arms)
        if not ep and emu.get(self.sie_status, 4) & self.SETUP_REC:
            return "nak"
        if not epb & self.AVAIL or epb & self.FULL:
            return "nak"
        for i, b in enumerate(data):
            emu.put(buf + i, 1, b)
        emu.put(a, 2, (epb & ~(self.AVAIL | self.LEN)) | self.FULL | len(data))
        self.done(emu, key, dbuf)
        return "ack"

    def in_(self, emu, ep):
        a, buf, key, dbuf = self.buffer(emu, ep, False)
        epb = emu.get(a, 2)
        if self.stalled(emu, ep, False, epb):
            return "stall", b""
        if not (epb & self.AVAIL and epb & self.FULL):
            return "nak", b""
        n = epb & self.LEN
        data = bytes(emu.get(buf + i, 1) for i in range(n))
        emu.put(a, 2, epb & ~(self.AVAIL | self.FULL))
        self.done(emu, key, dbuf)
        return "ack", data


BOARDS = {"CONFIG_MACH_STM32": Stm32, "CONFIG_MACH_LPC176X": Lpc176x, "CONFIG_MACH_ATSAM": Atsam,
          "CONFIG_MACH_ATSAMD": Atsamd, "CONFIG_MACH_HC32F460": Hc32f460, "CONFIG_MACH_RPXXXX": Rp2040}


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
        # (a board's other memory: the rp2040 boot rom)
        if hasattr(b, "setup"):
            b.setup(uc)
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
        if not self.in_memory(addr):
            self.log.write("R %08x %d %s\n" % (addr, size, self.logged(addr, v)))
        return v

    def in_memory(self, addr):
        # (a port's memory, not registers: the fdcan message ram, which C
        # accesses through plain pointers, in the order and sizes gcc picks;
        # its frames are logged as they go out)
        port = getattr(self.board, "port", None)
        return any(a <= addr < a + n for a, n in getattr(port, "memory", ()))

    def logged(self, addr, value):
        # (a register holding a ram address: the images' ram layouts differ)
        if addr in getattr(self.board, "pointer_regs", ()):
            return "(ram address)"
        # (a register holding the image's code size: the images' sizes differ)
        if addr in getattr(self.board, "size_regs", ()):
            return "(code size)"
        # (the bytes a port does not use: past a packet's end)
        port = getattr(self.board, "port", None)
        if hasattr(port, "used_bits"):
            value &= port.used_bits(self, addr)
        return "%x" % value

    def mmio_write(self, uc, offset, size, value, base):
        addr = base + offset
        b = self.board
        if not self.in_memory(addr):
            self.log.write("W %08x %d %s\n" % (addr, size, self.logged(addr, value)))
        self.poll_addr = None
        self.write_size = size
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
        # (the usb host's transactions with the device)
        if hasattr(b, "host"):
            b.host(self)
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

        # (a number the board has none for: the one at that index, among the
        # numbered names: not rp2040's ADC_TEMPERATURE pseudo pin)
        def value(enum, n, default):
            names = sorted(((k, v) for k, v in enums.get(enum, {}).items() if re.search(r"\d", k)), key=lambda kv: kv[1])
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
