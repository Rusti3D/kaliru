#!/usr/bin/env python3
# The AVR board model of emu_run.py: the image runs on simavr's core
# (tests/avr_shim.c, built here into ~/rk/avr_shim.so) with every io register
# read and written by this model, the same way emu_run.py models the ARM
# boards (register addresses, bits and vector numbers come from the generated
# chip header src/avr/chip/<mcu>.rs):
# - time is timer1: it advances TICK cycles on each read of TCNT1L (TCNT1H
#   reads the byte latched then, as the chip's TEMP register does), and jumps
#   to the next OCR1A match when the cpu waits for an interrupt; the compare
#   and overflow flags are set when the count passes OCR1A / OCR1B / 0xffff;
# - interrupts are taken only at that wait (the nop of irq_wait's
#   "sei; nop; cli"), the lowest pending vector first, as the chip does;
# - the serial port has a transmitter that is always ready and a receiver fed
#   from the script; the usb controller (at90usb, atmega32u4) has emu_run.py's
#   usb host, which enumerates it and carries the serial data;
# - the other io registers are memory with emu_run.py's rule for a register
#   read POLL times in a row.
import ctypes
import os
import re
import subprocess
import sys
import tempfile

from emu_run import ROOT, SPIN, TICK, Scripted, UsbHost, autoconf

HERE = os.path.dirname(os.path.abspath(__file__))
SHIM = os.path.expanduser("~/rk/avr_shim.so")
READ = ctypes.CFUNCTYPE(ctypes.c_uint8, ctypes.c_void_p, ctypes.c_uint16, ctypes.c_void_p)
WRITE = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint8, ctypes.c_void_p)
WAIT = bytes([0x78, 0x94, 0x00, 0x00, 0xF8, 0x94])  # sei; nop; cli
# (the chips simavr has no core of: one with the same flash (so the same pc
# size and instructions), vector size and at least the ram; their io
# registers are this model's anyway)
STAND_IN = {"at90usb1286": "atmega1280", "at90usb646": "atmega644p"}


def shim():
    src = os.path.join(HERE, "avr_shim.c")
    if not os.path.exists(SHIM) or os.path.getmtime(SHIM) < os.path.getmtime(src):
        subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", SHIM, src, "-lsimavr"])
    lib = ctypes.CDLL(SHIM)
    lib.shim_new.restype = ctypes.c_void_p
    lib.shim_new.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_int, READ, WRITE]
    lib.shim_run.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int, ctypes.c_long]
    lib.shim_enter.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.shim_pc.argtypes = [ctypes.c_void_p]
    lib.shim_pc.restype = ctypes.c_uint32
    return lib


class AvrChip:
    """the generated chip header: register addresses (`addr::NAME`) and
    constants (bit numbers, `NAME_vect_num`)"""

    def __init__(self, mcu):
        text = open(os.path.join(ROOT, "src", "avr", "chip", mcu + ".rs")).read()
        mod = text[text.index("pub mod addr {"):]
        mod = mod[:mod.index("\n}\n")]
        self.addr = {m.group(1): int(m.group(2), 0) for m in re.finditer(r"pub const (\w+): u16 = (0x[0-9a-f]+);", mod)}
        rest = text.replace(mod, "")
        self.const = {m.group(1): int(m.group(2), 0) for m in re.finditer(r"pub const (\w+): u\d+ = (0x[0-9a-f]+);", rest)}

    def vector(self, *names):
        """the number of the first of `names` the chip has"""
        return next(self.const[n + "_vect_num"] for n in names if n + "_vect_num" in self.const)


class AvrEmu(Scripted):
    def __init__(self, elf, dictionary, config, log):
        cfg = autoconf(config)
        mcu = cfg["CONFIG_MCU"]
        c = self.chip = AvrChip(mcu)
        self.init_session(dictionary, log, cfg["CONFIG_CLOCK_FREQ"])
        # (timer1)
        a = c.addr
        self.tcnt, self.ocra, self.ocrb = a["TCNT1L"], a["OCR1AL"], a["OCR1BL"]
        self.tifr, self.timsk = a["TIFR1"], a["TIMSK1"]
        self.ocr = {self.ocra: 0, self.ocrb: 0}
        self.temp = 0
        k = c.const
        self.ocfa, self.ocfb, self.tov = 1 << k["OCF1A"], 1 << k["OCF1B"], 1 << k["TOV1"]
        self.ocie = 1 << k["OCIE1A"]
        self.timer_vector = c.vector("TIMER1_COMPA")
        self.flags = 0
        # (the serial port)
        n = cfg.get("CONFIG_SERIAL_PORT")
        self.uart = None
        if cfg.get("CONFIG_SERIAL") and n is not None:
            self.uart = {
                "a": a["UCSR%dA" % n], "b": a["UCSR%dB" % n], "udr": a["UDR%d" % n],
                "rxc": 1 << k["RXC%d" % n], "udre": 1 << k["UDRE%d" % n],
                "rxcie": 1 << k["RXCIE%d" % n], "udrie": 1 << k["UDRIE%d" % n],
                "rx_vector": c.vector("USART_RX", "USART%d_RX" % n),
                "udre_vector": c.vector("USART_UDRE", "USART%d_UDRE" % n),
            }
        self.usb = AvrUsb(c) if cfg.get("CONFIG_USBSERIAL") else None
        self.spin = {}
        # (the core: the image's flash (its .text and .data))
        with tempfile.NamedTemporaryFile() as f:
            subprocess.check_call(["avr-objcopy", "-O", "binary", "-R", ".eeprom", "-R", ".fuse", "-R", ".lock",
                                   "-R", ".signature", elf, f.name])
            flash = f.read()
        self.waits = [i + 2 for i in range(0, len(flash) - 5, 2) if flash[i:i + 6] == WAIT]
        if not self.waits:
            sys.exit("no irq_wait in the image")
        self.lib = shim()
        self.read_cb, self.write_cb = READ(self.io_read), WRITE(self.io_write)
        core = STAND_IN.get(mcu, mcu)
        if core != mcu:
            k, s = c.const, AvrChip(core).const
            if k["FLASHEND"] != s["FLASHEND"] or k["RAMEND"] > s["RAMEND"]:
                sys.exit("%s is no stand-in for %s" % (core, mcu))
        self.avr = self.lib.shim_new(core.encode(), flash, len(flash), c.const["RAMSTART"] - 1, self.read_cb,
                                     self.write_cb)
        if not self.avr:
            sys.exit("simavr has no core %s, or the image does not fit its flash" % core)
        self.error = None
        self.run()

    # ---- io registers

    def advance(self, new):
        """the time moves to `new`: the flags of the compare matches and the
        overflow it passes"""
        d = (new - self.time) & 0xFFFFFFFF
        cnt = self.time & 0xFFFF
        for reg, flag in ((self.ocra, self.ocfa), (self.ocrb, self.ocfb)):
            if d and (self.ocr[reg] - cnt - 1) & 0xFFFF < d:
                self.flags |= flag
        if cnt + d > 0xFFFF:
            self.flags |= self.tov
        self.time = new

    def io_read(self, avr, addr, param):
        try:
            v = self.read(addr)
            self.log.write("R %04x 1 %x\n" % (addr, v))
            return v
        except Exception as e:  # (an error in a callback: stop and report it)
            self.error = e
            self.lib.shim_stop()
            return 0

    def read(self, addr):
        u = self.uart
        if addr != self.tcnt:
            self.spun(addr)
        if self.usb:
            v = self.usb.read(self, addr)
            if v is not None:
                return v
        if addr == self.tcnt:
            self.advance((self.time + TICK) & 0xFFFFFFFF)
            self.temp = (self.time >> 8) & 0xFF
            return self.time & 0xFF
        if addr == self.tcnt + 1:
            return self.temp
        if addr == self.tifr:
            return self.flags
        if u and addr == u["a"]:
            return (self.get(addr, 1) & ~(u["rxc"] | u["udre"])) | (u["rxc"] if self.rx else 0) | u["udre"]
        if u and addr == u["udr"]:
            return self.rx_byte()
        if addr in self.ocr:
            return self.ocr[addr] & 0xFF
        if addr - 1 in self.ocr:
            return self.ocr[addr - 1] >> 8
        return self.polled(addr, 1, self.get(addr, 1)) & 0xFF

    def io_write(self, avr, addr, value, param):
        try:
            self.log.write("W %04x 1 %x\n" % (addr, value))
            self.write(addr, value)
        except Exception as e:
            self.error = e
            self.lib.shim_stop()

    def write(self, addr, value):
        u = self.uart
        self.poll_addr = None
        if self.usb and self.usb.write(self, addr, value):
            return
        if addr == self.tcnt + 1 or addr - 1 in self.ocr:
            self.temp = value
        elif addr == self.tcnt:
            self.time = (self.time & ~0xFFFF) | (self.temp << 8) | value
        elif addr in self.ocr:
            self.ocr[addr] = (self.temp << 8) | value
        elif addr == self.tifr:
            self.flags &= ~value
        elif u and addr == u["udr"]:
            self.tx += bytes([value])
        else:
            self.put(addr, 1, value)

    def spun(self, addr):
        """(emu_run.py's Emu.spun: the usb host's turn after SPIN reads of
        one register without a wait)"""
        n = self.spin.get(addr, 0) + 1
        self.spin[addr] = n
        if n >= SPIN:
            self.spin = {}
            if self.usb:
                self.usb.host(self)

    # ---- interrupts

    def expiry(self):
        """the time of the next timer interrupt (the next OCR1A match), if
        it is enabled"""
        if not self.get(self.timsk, 1) & self.ocie:
            return None
        return (self.time + ((self.ocr[self.ocra] - self.time - 1) & 0xFFFF) + 1) & 0xFFFFFFFF

    def pending(self, advance_until):
        """the vector to take now, if any (advancing the time to the next
        timer expiry if it is not after `advance_until`)"""
        self.spin = {}
        # (the usb host's transactions with the device)
        if self.usb:
            self.usb.host(self)
        timsk = self.get(self.timsk, 1)
        if advance_until is not None and not self.flags & self.ocfa and timsk & self.ocie:
            due = self.expiry()
            if (advance_until - due) & 0x80000000 == 0 and not self.port_pending():
                self.advance(due)
        vectors = []
        if self.flags & self.ocfa and timsk & self.ocie:
            vectors.append(self.timer_vector)
        vectors += self.port_pending()
        if not vectors:
            return None
        num = min(vectors)
        if num == self.timer_vector:
            # (the chip clears the flag as it enters the handler)
            self.flags &= ~self.ocfa
        return num

    def port_pending(self):
        """the serial port's or the usb controller's pending vectors"""
        u = self.uart
        if self.usb:
            return self.usb.pending(self)
        if not u:
            return []
        b = self.get(u["b"], 1)
        return ([u["rx_vector"]] if self.rx and b & u["rxcie"] else []) + ([u["udre_vector"]] if b & u["udrie"] else [])

    def run(self, advance_until=None):
        """run until the cpu waits with nothing to do (advancing the time
        through the timer expiries up to `advance_until`)"""
        if self.crashed:
            return
        waits = (ctypes.c_uint32 * len(self.waits))(*self.waits)
        for _ in range(1000000):
            r = self.lib.shim_run(self.avr, waits, len(self.waits), 50000000)
            if self.error:
                raise self.error
            if r < 0:
                # (the image crashed: an outcome to compare, the run ends)
                self.crashed = "cpu crash"
                self.log.write("# crash\n")
                return
            if r:
                sys.exit("no wait for interrupts in 50M instructions (pc %x)" % self.lib.shim_pc(self.avr))
            num = self.pending(advance_until)
            if num is None:
                # (the time the quiet period ends at)
                if advance_until is not None and (self.time - advance_until) & 0x80000000:
                    self.advance(advance_until)
                return
            self.lib.shim_enter(self.avr, num)


class AvrUsb:
    """avr/usbserial.c: the at90usb / atmega32u4 usb device controller for
    emu_run.py's UsbHost. The endpoint registers (UECONX, UECFGnX, UEIENX,
    UEINTX, UEBCLX, UEDATX) are those of the endpoint UENUM selects. UEINTX:
    RXSTPI is set while a setup packet is unread, RXOUTI while an out packet
    is, TXINI while an in bank is free; a flag written 0 hands the bank back
    (FIFOCON on the bulk endpoints, RXSTPI / RXOUTI / TXINI on the control
    endpoint). STALLRQ stalls the endpoint until STALLRQC or a setup packet;
    UDINT flags are cleared by writing 0."""

    def __init__(self, chip):
        a, k = chip.addr, chip.const
        for r in ("UENUM", "UECONX", "UECFG0X", "UECFG1X", "UEIENX", "UEINTX", "UEBCLX", "UEDATX", "UEINT",
                  "UDINT", "UDIEN", "USBCON", "UDCON"):
            setattr(self, r.lower(), a[r])
        bit = lambda n: 1 << k[n]  # noqa: E731
        self.TXINI, self.RXOUTI, self.RXSTPI, self.FIFOCON = bit("TXINI"), bit("RXOUTI"), bit("RXSTPI"), bit("FIFOCON")
        self.STALLRQ, self.STALLRQC, self.EPDIR, self.EPEN = bit("STALLRQ"), bit("STALLRQC"), bit("EPDIR"), bit("EPEN")
        self.EORSTI, self.USBE, self.FRZCLK, self.DETACH = bit("EORSTI"), bit("USBE"), bit("FRZCLK"), bit("DETACH")
        self.EPTYPE = bit("EPTYPE0") | bit("EPTYPE1")
        self.gen_vector, self.com_vector = chip.vector("USB_GEN"), chip.vector("USB_COM")
        self.regs = {}
        self.udint_flags = 0
        n = 7
        # (per endpoint: the unread setup packet, the out packets, the in
        # packet being written, the in packets sent, a stall)
        self.setup_pkt = [None] * n
        self.rxq = [[] for _ in range(n)]
        self.cur = [bytearray() for _ in range(n)]
        self.txq = [[] for _ in range(n)]
        self.stalled = [False] * n
        self.host_side = UsbHost(self)

    def reg(self, ep, addr):
        return self.regs.get((ep, addr), 0)

    def control(self, ep):
        return not self.reg(ep, self.uecfg0x) & self.EPTYPE

    def banks(self, ep):
        return 1 if self.control(ep) else 1 + ((self.reg(ep, self.uecfg1x) >> 2) & 3)

    def is_in(self, ep):
        return self.reg(ep, self.uecfg0x) & self.EPDIR

    def flags(self, ep):
        c = self.control(ep)
        v = 0
        if self.setup_pkt[ep] is not None:
            v |= self.RXSTPI
        if self.rxq[ep]:
            v |= self.RXOUTI | (0 if c else self.FIFOCON)
        if len(self.txq[ep]) < self.banks(ep) and (c or self.is_in(ep)):
            v |= self.TXINI | (0 if c else self.FIFOCON)
        return v

    def packet(self, ep):
        """the packet UEDATX reads"""
        if self.setup_pkt[ep] is not None:
            return self.setup_pkt[ep]
        return self.rxq[ep][0] if self.rxq[ep] else bytearray()

    # ---- the cpu's registers

    def read(self, emu, addr):
        ep = emu.get(self.uenum, 1) & 7
        if addr == self.ueintx:
            return self.flags(ep)
        if addr == self.uedatx:
            p = self.packet(ep)
            return p.pop(0) if p else 0
        if addr == self.uebclx:
            return len(self.packet(ep))
        if addr == self.ueconx:
            return self.reg(ep, addr) | (self.STALLRQ if self.stalled[ep] else 0)
        if addr in (self.uecfg0x, self.uecfg1x, self.ueienx):
            return self.reg(ep, addr)
        if addr == self.ueint:
            return sum(1 << e for e in range(7) if self.flags(e) & self.reg(e, self.ueienx))
        if addr == self.udint:
            return self.udint_flags
        return None

    def write(self, emu, addr, value):
        ep = emu.get(self.uenum, 1) & 7
        if addr == self.ueintx:
            cleared = self.flags(ep) & ~value
            if self.control(ep):
                if cleared & self.RXSTPI:
                    self.setup_pkt[ep] = None
                if cleared & self.RXOUTI:
                    self.rxq[ep].pop(0)
                if cleared & self.TXINI:
                    self.txq[ep].append(bytes(self.cur[ep]))
                    self.cur[ep] = bytearray()
            elif cleared & self.FIFOCON:
                if self.is_in(ep):
                    self.txq[ep].append(bytes(self.cur[ep]))
                    self.cur[ep] = bytearray()
                else:
                    self.rxq[ep].pop(0)
            return True
        if addr == self.uedatx:
            self.cur[ep].append(value)
            return True
        if addr == self.ueconx:
            if value & self.STALLRQ:
                self.stalled[ep] = True
            if value & self.STALLRQC:
                self.stalled[ep] = False
            self.regs[(ep, addr)] = value & ~(self.STALLRQ | self.STALLRQC)
            return True
        if addr in (self.uecfg0x, self.uecfg1x, self.ueienx):
            self.regs[(ep, addr)] = value
            return True
        if addr == self.udint:
            self.udint_flags &= value
            return True
        return False

    def pending(self, emu):
        v = []
        if self.udint_flags & emu.get(self.udien, 1):
            v.append(self.gen_vector)
        if any(self.flags(e) & self.reg(e, self.ueienx) for e in range(7)):
            v.append(self.com_vector)
        return v

    def host(self, emu):
        self.host_side.emu = emu
        return self.host_side.step(emu)

    # ---- the host's transactions

    def ready(self, emu):
        usbcon = emu.get(self.usbcon, 1)
        return usbcon & self.USBE and not usbcon & self.FRZCLK and not emu.get(self.udcon, 1) & self.DETACH

    def reset(self, emu):
        self.udint_flags |= self.EORSTI

    def setup(self, emu, data):
        # (after the device has handled the end of the bus reset and
        # enabled endpoint 0)
        if self.udint_flags & self.EORSTI or not self.reg(0, self.ueconx) & self.EPEN:
            return False
        self.setup_pkt[0] = bytearray(data)
        self.rxq[0], self.txq[0], self.cur[0] = [], [], bytearray()
        self.stalled[0] = False
        return True

    def out(self, emu, ep, data):
        if self.stalled[ep]:
            return "stall"
        if (self.setup_pkt[ep] is not None or len(self.rxq[ep]) >= self.banks(ep)
                or not self.reg(ep, self.ueconx) & self.EPEN):
            return "nak"
        self.rxq[ep].append(bytearray(data))
        return "ack"

    def in_(self, emu, ep):
        if self.stalled[ep]:
            return "stall", b""
        if not self.txq[ep]:
            return "nak", b""
        return "ack", self.txq[ep].pop(0)
