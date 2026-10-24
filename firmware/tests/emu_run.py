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
# - time is the DWT cycle counter; it advances TICK cycles on each read of it
#   (so it depends only on what the code does), and jumps to the next SysTick
#   expiry when the cpu waits for an interrupt;
# - interrupts are taken only at that wait (wfi), so the order of events does
#   not depend on instruction counts;
# - the serial port has a transmitter that is always ready and a receiver fed
#   from the script;
# - the other peripheral registers are memory, with the status bits the clock
#   setup waits for always set.
import json
import os
import re
import struct
import sys

import unicorn
import unicorn.arm_const as A

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcu_run  # noqa: E402

TICK = 16  # cycles per cycle counter read
POLL = 1000  # reads of a register in a row that end a wait for it
EXC_RETURN = 0xFFFFFFF9


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
        self.ptr = {m.group(1): int(m.group(2), 16) for m in re.finditer(r"^pub const (\w+): \*mut \w+ = (0x[0-9a-f]+) as", text, re.M)}
        self.off = {}
        for m in re.finditer(r"offset_of!\((\w+), (\w+)\) == (\d+)\)", text):
            self.off[(m.group(1), m.group(2))] = int(m.group(3))

    def reg(self, periph, struct_name, field):
        return self.ptr[periph] + self.off[(struct_name, field)]


def load_elf(uc, path):
    data = open(path, "rb").read()
    phoff, = struct.unpack_from("<I", data, 28)
    phentsize, phnum = struct.unpack_from("<HH", data, 42)
    for i in range(phnum):
        p_type, p_offset, p_vaddr, p_paddr, p_filesz = struct.unpack_from("<IIIII", data, phoff + i * phentsize)
        if p_type == 1 and p_filesz:
            uc.mem_write(p_paddr, data[p_offset : p_offset + p_filesz])


def wait_points(path):
    """the addresses of the wfi instructions (where the cpu waits for an
    interrupt: irq_wait), or on the Cortex-M7 the nop of irq_wait's
    `cpsie i; nop; cpsid i`"""
    import subprocess

    dis = subprocess.run(["arm-none-eabi-objdump", "-d", path], capture_output=True, text=True).stdout
    wfi = [int(m.group(1), 16) for m in re.finditer(r"^\s*([0-9a-f]+):\s+bf30\s+wfi", dis, re.M)]
    nop = [int(m.group(1), 16) for m in re.finditer(
        r"^\s*[0-9a-f]+:\s+b662\s+cpsie\s+i\s*\n\s*([0-9a-f]+):\s+bf00\s+nop\s*\n\s*[0-9a-f]+:\s+b672\s+cpsid\s+i", dis, re.M)]
    return wfi + nop


class Stm32:
    """the stm32 parts of the model: clock setup status bits, the usart"""

    def __init__(self, cfg, chip):
        self.chip = chip
        c = chip.const
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
        self.usart = n[0]
        self.usart_irq = c[self.usart + "_IRQn"]
        u = chip.ptr[self.usart]
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


class Emu(mcu_run.Mcu):
    mask_clocks = False

    def __init__(self, elf, dictionary, config, log):
        self.dictionary = dictionary
        cfg = autoconf(config)
        if not cfg.get("CONFIG_MACH_STM32"):
            sys.exit("only the stm32 boards are modelled")
        mcu = cfg["CONFIG_MCU"]
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.chip = Chip(os.path.join(root, "src", "stm32", "chip", mcu + ".rs"))
        self.board = Stm32(cfg, self.chip)
        cpu = {
            "CONFIG_MACH_STM32F0": A.UC_CPU_ARM_CORTEX_M0, "CONFIG_MACH_STM32G0": A.UC_CPU_ARM_CORTEX_M0,
            "CONFIG_MACH_STM32F1": A.UC_CPU_ARM_CORTEX_M3, "CONFIG_MACH_STM32F2": A.UC_CPU_ARM_CORTEX_M3,
            "CONFIG_MACH_STM32F7": A.UC_CPU_ARM_CORTEX_M7, "CONFIG_MACH_STM32H7": A.UC_CPU_ARM_CORTEX_M7,
        }
        model = next((v for k, v in cpu.items() if cfg.get(k)), A.UC_CPU_ARM_CORTEX_M4)
        uc = self.uc = unicorn.Uc(unicorn.UC_ARCH_ARM, unicorn.UC_MODE_THUMB | unicorn.UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(model)
        flash_start = 0x08000000
        uc.mem_map(flash_start, cfg["CONFIG_FLASH_SIZE"])
        uc.mem_map(cfg["CONFIG_RAM_START"], cfg["CONFIG_RAM_SIZE"])
        uc.mmio_map(0x40000000, 0x20000000, self.mmio_read, 0x40000000, self.mmio_write, 0x40000000)
        uc.mmio_map(0xE0000000, 0x00100000, self.mmio_read, 0xE0000000, self.mmio_write, 0xE0000000)
        load_elf(uc, elf)
        vt = cfg["CONFIG_FLASH_APPLICATION_ADDRESS"]
        sp, reset = struct.unpack("<II", uc.mem_read(vt, 8))
        self.vectors = vt
        uc.reg_write(A.UC_ARM_REG_SP, sp)
        self.pc = reset
        for a in wait_points(elf):
            uc.hook_add(unicorn.UC_HOOK_CODE, self.on_wait, begin=a, end=a)
        # (a branch to EXC_RETURN raises the cpu's exception exit)
        uc.hook_add(unicorn.UC_HOOK_INTR, self.on_intr)
        self.mem = {}
        self.log = open(log, "w")
        self.time = 0
        self.systick_due = None
        self.nvic_enabled = set()
        self.active = 0
        self.rx = b""
        self.tx = b""
        self.waiting = False
        self.poll_addr, self.poll_count = None, 0
        self.stopped = None
        c = self.chip
        self.cyccnt = c.reg("DWT", "DWT_Type", "CYCCNT")
        self.st_ctrl = c.reg("SysTick", "SysTick_Type", "CTRL")
        self.st_load = c.reg("SysTick", "SysTick_Type", "LOAD")
        self.st_val = c.reg("SysTick", "SysTick_Type", "VAL")
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

    def mmio_read(self, uc, offset, size, base):
        addr = base + offset
        b = self.board
        if addr == self.cyccnt:
            self.time = (self.time + TICK) & 0xFFFFFFFF
            v = self.time
        elif addr == b.sr:
            v = b.TXE | b.TC | (b.RXNE if self.rx else 0)
        elif addr == b.dr:
            v = self.rx[0] if self.rx else 0
            self.rx = self.rx[1:]
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
            # (a register read POLL times in a row, the cycle counter aside:
            # a wait for status bits the model does not set; it then reads
            # inverted, then each single bit, then 0, so the wait ends the
            # same way for any image)
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
        if addr == b.tdr:
            self.tx += bytes([value & 0xFF])
            return
        if addr == self.cyccnt:
            self.time = value
            return
        if self.iser <= addr < self.iser + 32:
            n = (addr - self.iser) * 8
            self.nvic_enabled |= {n + i for i in range(32) if value >> i & 1}
            return
        if self.icer <= addr < self.icer + 32:
            n = (addr - self.icer) * 8
            self.nvic_enabled -= {n + i for i in range(32) if value >> i & 1}
            return
        if addr == self.st_val:
            # (armcm_timer: LOAD = ticks; VAL = 0; LOAD = 0: one expiry)
            load = self.get(self.st_load, 4) & 0xFFFFFF
            self.systick_due = (self.time + load) if load else None
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

    def on_wait(self, uc, address, size, user):
        self.stopped = address
        uc.emu_stop()

    def pending(self, advance):
        """the exception to take now, if any (advancing the time to the next
        SysTick expiry when `advance`)"""
        b = self.board
        cr1 = self.get(b.cr1, 4)
        if b.usart_irq in self.nvic_enabled and ((self.rx and cr1 & b.RXNEIE) or cr1 & b.TXEIE):
            return 16 + b.usart_irq
        ctrl = self.get(self.st_ctrl, 4)
        if self.systick_due is not None and ctrl & 3 == 3:
            late = (self.time - self.systick_due) & 0xFFFFFFFF
            if late < 0x80000000 or advance:
                if late >= 0x80000000:
                    self.time = self.systick_due
                self.systick_due = None
                return 15
        return None

    def run(self, advance_until=None):
        """run until the cpu waits with nothing to do (advancing the time
        through the SysTick expiries up to `advance_until`)"""
        uc = self.uc
        for _ in range(1000000):
            self.stopped = None
            try:
                uc.emu_start(self.pc | 1, 0xFFFFFFFE, count=50000000)
            except unicorn.UcError as e:
                self.log.flush()
                sys.exit("%s at pc %x (lr %x)" % (e, uc.reg_read(A.UC_ARM_REG_PC), uc.reg_read(A.UC_ARM_REG_LR)))
            if self.stopped is None:
                sys.exit("no wait for interrupts in 50M instructions (pc %x)" % uc.reg_read(A.UC_ARM_REG_PC))
            self.pc = self.stopped
            adv = advance_until is not None and (
                self.systick_due is not None and ((advance_until - self.systick_due) & 0xFFFFFFFF) < 0x80000000)
            num = self.pending(adv)
            if num is None:
                # (the time the quiet period ends at)
                if advance_until is not None and ((advance_until - self.time) & 0xFFFFFFFF) < 0x80000000:
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

        def name(enum, value, default):
            return {v: k for k, v in enums.get(enum, {}).items()}.get(value, default)

        # (a number the board has no pin for: the pin at that index)
        pins = sorted(enums.get("pin", {}).items(), key=lambda kv: kv[1])

        def pin(n):
            return name("pin", n, pins[n % len(pins)][0])

        line = re.sub(r"\b(?:gpio|analog)(\d+)\b", lambda m: pin(int(m.group(1))), line)
        line = re.sub(r"\bspidev(\d+)\.\d+\b", lambda m: name("spi_bus", int(m.group(1)), m.group(0)), line)
        line = re.sub(r"\bi2c\.(\d+)\b", lambda m: name("i2c_bus", int(m.group(1)), m.group(0)), line)
        self.log.write("# %s\n" % line)
        return super().prepare(line)


def main():
    elf, dictionary, config, script, log = sys.argv[1:6]
    emu = Emu(elf, dictionary, config, log)
    out, err = mcu_run.run_script(emu, script)
    print("\n".join(out))


if __name__ == "__main__":
    main()
