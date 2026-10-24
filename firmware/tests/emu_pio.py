"""rp2040 PIO and a CAN bus for the emulator (emu_run.py): the four state
machines of a PIO block running the program the firmware loads, with their
FIFOs, IRQ flags and the block's registers, and a CAN bus wire between the
node's PIO pins and a host CAN controller (bit stuffing, CRC-15, arbitration,
ACK). For lib/can2040, which drives the bus through PIO state machines.

Time is counted in PIO cycles. All state machines execute in parallel: each
cycle they see the IRQ flags and pins of the cycle's start, and their IRQ
flag changes apply at its end."""

import os

OP_JMP, OP_WAIT, OP_IN, OP_OUT, OP_PUSHPULL, OP_MOV, OP_IRQ, OP_SET = range(8)
FIFO_DEPTH = 4
MASK32 = 0xFFFFFFFF


class Sm:
    """one state machine's registers and internal state"""

    def __init__(self):
        # (the registers' reset values)
        self.clkdiv = 1 << 16
        self.execctrl = 0x1F << 12
        self.shiftctrl = 0x000C0000
        self.pinctrl = 0x14000000
        self.pc = 0
        self.x = self.y = 0
        self.osr = 0
        self.txf = []
        self.rxf = []
        self.restart()

    def restart(self):
        # (SM_RESTART: the shift counters (the OSR empty), the ISR, the delay,
        # a stalled or pending instruction; not X, Y, the PC)
        self.isr = 0
        self.isr_cnt = 0
        self.osr_cnt = 32
        self.delay = 0
        self.exec_instr = None
        self.push_pending = False
        self.irq_waiting = None
        self.stalled = False

    def field(self, reg, lsb, bits):
        return (reg >> lsb) & ((1 << bits) - 1)

    def fifo_depth(self, rx):
        join_rx, join_tx = self.shiftctrl >> 31 & 1, self.shiftctrl >> 30 & 1
        if rx:
            return 0 if join_tx else 2 * FIFO_DEPTH if join_rx else FIFO_DEPTH
        return 0 if join_rx else 2 * FIFO_DEPTH if join_tx else FIFO_DEPTH

    def key(self):
        """(the state but X and Y)"""
        return (self.pc, self.isr, self.isr_cnt, self.osr, self.osr_cnt, self.delay, self.exec_instr,
                self.push_pending, self.irq_waiting, tuple(self.txf), tuple(self.rxf))


class Pio:
    """a PIO block: registers at `base` (the chip's pio_hw_t layout), its
    state machines, the pins' output values and directions; the pins' input
    levels come from `pins_in()`"""

    def __init__(self, chip, base, pins_in):
        o = lambda f: base + chip.off[("pio_hw_t", f)]  # noqa: E731
        self.base, self.end = base, o("ints1") + 4
        self.ctrl, self.fstat, self.fdebug_addr, self.flevel = o("ctrl"), o("fstat"), o("fdebug"), o("flevel")
        self.txf, self.rxf, self.irq_addr, self.irq_force = o("txf"), o("rxf"), o("irq"), o("irq_force")
        self.instr_mem_addr, self.sm_addr = o("instr_mem"), o("sm")
        self.intr, self.inte0, self.ints0 = o("intr"), o("inte0"), o("ints0")
        self.sm_size = (chip.off[("pio_hw_t", "intr")] - chip.off[("pio_hw_t", "sm")]) // 4
        self.sm_off = {chip.off[("pio_sm_hw_t", f)]: f
                       for f in ("clkdiv", "execctrl", "shiftctrl", "addr", "instr", "pinctrl")}
        self.pins_in = pins_in
        self.sms = [Sm() for _ in range(4)]
        self.instr_mem = [0] * 32
        self.enabled = 0
        self.irq = 0
        self.inte = 0
        self.fdebug = 0
        self.pin_out = 0
        self.pin_dir = 0
        # (debugging: called with the state machine's number at each `in pins`)
        self.debug = None

    # ---- the cpu's registers

    def owns(self, addr):
        return self.base <= addr < self.end

    def read(self, addr):
        if addr == self.ctrl:
            return self.enabled
        if addr == self.fdebug_addr:
            return self.fdebug
        if addr == self.flevel:
            return sum((min(len(sm.txf), 15) | min(len(sm.rxf), 15) << 4) << (8 * i) for i, sm in enumerate(self.sms))
        if addr == self.fstat:
            v = 0
            for i, sm in enumerate(self.sms):
                v |= (len(sm.rxf) >= sm.fifo_depth(True)) << i | (not sm.rxf) << (8 + i)
                v |= (len(sm.txf) >= sm.fifo_depth(False)) << (16 + i) | (not sm.txf) << (24 + i)
            return v
        if self.rxf <= addr < self.rxf + 16:
            i = (addr - self.rxf) // 4
            if not self.sms[i].rxf:
                self.fdebug |= 1 << (8 + i)
                return 0
            return self.sms[i].rxf.pop(0)
        if addr == self.irq_addr:
            return self.irq
        if addr == self.intr:
            return self.intr_value()
        if addr == self.inte0:
            return self.inte
        if addr == self.ints0:
            return self.intr_value() & self.inte
        n, f = self.sm_reg(addr)
        if f == "addr":
            return self.sms[n].pc
        if f == "instr":
            sm = self.sms[n]
            return sm.exec_instr if sm.exec_instr is not None else self.instr_mem[sm.pc]
        if f is not None:
            return getattr(self.sms[n], f)
        return 0

    def write(self, addr, value):
        if addr == self.ctrl:
            self.enabled = value & 0xF
            for i in range(4):
                if value >> (4 + i) & 1:
                    self.sms[i].restart()
        elif addr == self.fdebug_addr:
            self.fdebug &= ~value
        elif self.txf <= addr < self.txf + 16:
            i = (addr - self.txf) // 4
            sm = self.sms[i]
            if len(sm.txf) >= sm.fifo_depth(False):
                self.fdebug |= 1 << (16 + i)
            else:
                sm.txf.append(value & MASK32)
        elif addr == self.irq_addr:
            self.irq &= ~value & 0xFF
        elif addr == self.irq_force:
            self.irq |= value & 0xFF
        elif addr == self.inte0:
            self.inte = value & 0xFFF
        elif self.instr_mem_addr <= addr < self.instr_mem_addr + 128:
            self.instr_mem[(addr - self.instr_mem_addr) // 4] = value & 0xFFFF
        else:
            n, f = self.sm_reg(addr)
            if f is None or f == "addr":
                return
            sm = self.sms[n]
            if f == "instr":
                # (executed at once, even with the state machine disabled)
                sm.exec_instr = value & 0xFFFF
                sets, clears = self.step_sm(n, sm, self.irq)
                self.irq = (self.irq | sets) & ~clears
            elif f == "shiftctrl":
                # (a change of the fifo joins flushes the fifos)
                if (sm.shiftctrl ^ value) & 0xC0000000:
                    sm.txf, sm.rxf = [], []
                sm.shiftctrl = value
            else:
                setattr(sm, f, value)

    def sm_reg(self, addr):
        off = addr - self.sm_addr
        if 0 <= off < 4 * self.sm_size:
            return off // self.sm_size, self.sm_off.get(off % self.sm_size)
        return None, None

    def intr_value(self):
        v = 0
        for i, sm in enumerate(self.sms):
            v |= bool(sm.rxf) << i | (len(sm.txf) < sm.fifo_depth(False)) << (4 + i)
        return v | (self.irq & 0xF) << 8

    def irq_pending(self):
        return bool(self.intr_value() & self.inte)

    # ---- the state machines

    def step(self):
        """one cycle of the enabled state machines; True if one of them did
        something (not only a delay or a stall)"""
        irq = self.irq
        sets = clears = 0
        self.busy = False
        for i, sm in enumerate(self.sms):
            if self.enabled >> i & 1:
                s, c = self.step_sm(i, sm, irq)
                sets |= s
                clears |= c
        self.irq = (self.irq | sets) & ~clears
        return self.busy or sets or clears

    def idle_cycles(self):
        """(after a cycle no state machine did anything in: the cycles until
        one's delay ends, None if none is in a delay; 0 if one is ready)"""
        delays = []
        for i, sm in enumerate(self.sms):
            if self.enabled >> i & 1:
                if sm.delay:
                    delays.append(sm.delay)
                elif not sm.stalled:
                    return 0
        return min(delays) if delays else None

    def skip(self, k):
        for i, sm in enumerate(self.sms):
            if self.enabled >> i & 1 and sm.delay:
                sm.delay -= k

    def step_sm(self, i, sm, irq):
        """one cycle of state machine `i`: (irq flags set, cleared)"""
        if sm.delay:
            sm.delay -= 1
            return 0, 0
        exec_ = sm.exec_instr is not None
        ins = sm.exec_instr if exec_ else self.instr_mem[sm.pc]
        if sm.push_pending:
            # (an IN's autopush waiting for room in the rx fifo)
            if not self.push(i, sm):
                sm.stalled = True
                return 0, 0
            sm.push_pending = False
            done, jumped, sets, clears = True, False, 0, 0
        else:
            sm.exec_set = False
            done, jumped, sets, clears = self.execute(i, sm, ins, irq)
        sm.stalled = not done
        if not done:
            return sets, clears
        self.busy = True
        if exec_ and not sm.exec_set:
            sm.exec_instr = None
        if not jumped and not exec_:
            top, bottom = sm.field(sm.execctrl, 12, 5), sm.field(sm.execctrl, 7, 5)
            sm.pc = bottom if sm.pc == top else (sm.pc + 1) & 31
        sm.delay = (ins >> 8) & 0x1F
        return sets, clears

    def pin(self, n):
        return self.pins_in() >> n & 1

    def push(self, i, sm):
        if len(sm.rxf) >= sm.fifo_depth(True):
            self.fdebug |= 1 << i
            return False
        sm.rxf.append(sm.isr)
        sm.isr = sm.isr_cnt = 0
        return True

    def source(self, sm, src):
        if src == 0:
            # (the pins from IN_BASE up, wrapping)
            pins, base = self.pins_in(), sm.field(sm.pinctrl, 15, 5)
            return ((pins >> base) | (pins << (32 - base))) & MASK32
        return {1: sm.x, 2: sm.y, 3: 0, 6: sm.isr, 7: sm.osr}.get(src, 0)

    def execute(self, i, sm, ins, irq):
        """an instruction: (done, jumped, irq flags set, cleared)"""
        op = ins >> 13
        a = (ins >> 5) & 7
        idx = ins & 0x1F
        if op == OP_JMP:
            if a == 0:
                take = True
            elif a == 1:
                take = sm.x == 0
            elif a == 2:
                take = sm.x != 0
                sm.x = (sm.x - 1) & MASK32
            elif a == 3:
                take = sm.y == 0
            elif a == 4:
                take = sm.y != 0
                sm.y = (sm.y - 1) & MASK32
            elif a == 5:
                take = sm.x != sm.y
            elif a == 6:
                take = bool(self.pin(sm.field(sm.execctrl, 24, 5)))
            else:
                take = sm.osr_cnt < (sm.field(sm.shiftctrl, 25, 5) or 32)
            if take:
                sm.pc = idx
            return True, take, 0, 0
        if op == OP_WAIT:
            pol, src = ins >> 7 & 1, (ins >> 5) & 3
            if src == 2:
                n = self.irq_index(i, idx)
                if (irq >> n & 1) != pol:
                    return False, False, 0, 0
                # (a wait for a set flag clears it)
                return True, False, 0, (1 << n) if pol else 0
            pin = idx if src == 0 else (sm.field(sm.pinctrl, 15, 5) + idx) & 31
            return self.pin(pin) == pol, False, 0, 0
        if op == OP_IN:
            n = idx or 32
            v = self.source(sm, a) & ((1 << n) - 1)
            if a == 0 and self.debug is not None:
                self.debug(i)
            if sm.shiftctrl >> 18 & 1:
                sm.isr = ((sm.isr >> n) | (v << (32 - n))) & MASK32 if n < 32 else v
            else:
                sm.isr = ((sm.isr << n) | v) & MASK32
            sm.isr_cnt = min(32, sm.isr_cnt + n)
            if sm.shiftctrl >> 16 & 1 and sm.isr_cnt >= (sm.field(sm.shiftctrl, 20, 5) or 32):
                if not self.push(i, sm):
                    sm.push_pending = True
                    return False, False, 0, 0
            return True, False, 0, 0
        if op == OP_OUT:
            n = idx or 32
            autopull = sm.shiftctrl >> 17 & 1
            thresh = sm.field(sm.shiftctrl, 25, 5) or 32
            if autopull and sm.osr_cnt >= thresh:
                if not sm.txf:
                    self.fdebug |= 1 << (24 + i)
                    return False, False, 0, 0
                sm.osr, sm.osr_cnt = sm.txf.pop(0), 0
            if sm.shiftctrl >> 19 & 1:
                v = sm.osr & ((1 << n) - 1)
                sm.osr = sm.osr >> n if n < 32 else 0
            else:
                v = sm.osr >> (32 - n)
                sm.osr = (sm.osr << n) & MASK32 if n < 32 else 0
            sm.osr_cnt = min(32, sm.osr_cnt + n)
            if autopull and sm.osr_cnt >= thresh and sm.txf:
                sm.osr, sm.osr_cnt = sm.txf.pop(0), 0
            return self.write_dest(sm, a, v, n, sm.field(sm.pinctrl, 0, 5), sm.field(sm.pinctrl, 20, 6))
        if op == OP_PUSHPULL:
            block = ins >> 5 & 1
            if not ins >> 7 & 1:
                # (PUSH)
                if ins >> 6 & 1 and sm.isr_cnt < (sm.field(sm.shiftctrl, 20, 5) or 32):
                    return True, False, 0, 0
                if len(sm.rxf) >= sm.fifo_depth(True):
                    if block:
                        self.fdebug |= 1 << i
                        return False, False, 0, 0
                    sm.isr = sm.isr_cnt = 0
                    return True, False, 0, 0
                self.push(i, sm)
                return True, False, 0, 0
            # (PULL)
            if ins >> 6 & 1 and sm.osr_cnt < (sm.field(sm.shiftctrl, 25, 5) or 32):
                return True, False, 0, 0
            if not sm.txf:
                if block:
                    self.fdebug |= 1 << (24 + i)
                    return False, False, 0, 0
                # (a non blocking pull of an empty fifo: X)
                sm.osr, sm.osr_cnt = sm.x, 0
                return True, False, 0, 0
            sm.osr, sm.osr_cnt = sm.txf.pop(0), 0
            return True, False, 0, 0
        if op == OP_MOV:
            v = self.source(sm, ins & 7) & MASK32
            o = (ins >> 3) & 3
            if o == 1:
                v = ~v & MASK32
            elif o == 2:
                v = int("{:032b}".format(v)[::-1], 2)
            if a == 6:
                sm.isr, sm.isr_cnt = v, 0
                return True, False, 0, 0
            if a == 7:
                sm.osr, sm.osr_cnt = v, 0
                return True, False, 0, 0
            if a == 4:
                sm.exec_instr, sm.exec_set = v & 0xFFFF, True
                return True, False, 0, 0
            return self.write_dest(sm, {0: 0, 1: 1, 2: 2, 5: 5}.get(a, 3), v, 32,
                                   sm.field(sm.pinctrl, 0, 5), sm.field(sm.pinctrl, 20, 6))
        if op == OP_IRQ:
            n = self.irq_index(i, idx)
            if ins >> 6 & 1:
                return True, False, 0, 1 << n
            if ins >> 5 & 1:
                # (irq wait: set the flag, then wait for it to be cleared)
                if sm.irq_waiting is None:
                    sm.irq_waiting = n
                    return False, False, 1 << n, 0
                if irq >> n & 1:
                    return False, False, 0, 0
                sm.irq_waiting = None
                return True, False, 0, 0
            return True, False, 1 << n, 0
        # (SET)
        base, count = sm.field(sm.pinctrl, 5, 5), sm.field(sm.pinctrl, 26, 3)
        return self.write_dest(sm, {0: 0, 1: 1, 2: 2, 4: 4}.get(a, 3), idx, 5, base, count)

    def write_dest(self, sm, dest, v, n, base, count):
        """OUT / MOV / SET's destination (0 pins, 1 X, 2 Y, 3 null, 4
        pindirs, 5 PC, 6 ISR, 7 EXEC): (done, jumped, sets, clears)"""
        if dest == 0:
            self.pin_out = self.pins(self.pin_out, base, count, v)
        elif dest == 1:
            sm.x = v
        elif dest == 2:
            sm.y = v
        elif dest == 4:
            self.pin_dir = self.pins(self.pin_dir, base, count, v)
        elif dest == 5:
            sm.pc = v & 31
            return True, True, 0, 0
        elif dest == 6:
            sm.isr, sm.isr_cnt = v, n
        elif dest == 7:
            sm.exec_instr, sm.exec_set = v & 0xFFFF, True
        return True, False, 0, 0

    def pins(self, reg, base, count, v):
        for k in range(count):
            p = (base + k) & 31
            reg = (reg & ~(1 << p)) | ((v >> k & 1) << p)
        return reg

    def irq_index(self, i, idx):
        # (the REL bit: the index's low 2 bits plus the state machine number)
        if idx & 0x10:
            return (idx & 4) | ((idx + i) & 3)
        return idx & 7

    def key(self):
        """(the state but the state machines' X and Y)"""
        return (self.enabled, self.irq, self.fdebug, self.pin_out, self.pin_dir, tuple(self.instr_mem),
                tuple(sm.key() for sm in self.sms))

    def counters(self):
        return [v for sm in self.sms for v in (sm.x, sm.y)]

    def set_counters(self, values):
        for k, sm in enumerate(self.sms):
            sm.x, sm.y = values[2 * k], values[2 * k + 1]


def crc15(bits):
    crc = 0
    for b in bits:
        nxt = b ^ (crc >> 14 & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    return crc


def frame_bits(fid, data):
    """a standard data frame's bits from the start of frame through the CRC,
    without and with the stuff bits"""
    raw = [0] + [fid >> (10 - k) & 1 for k in range(11)] + [0, 0, 0] + [len(data) >> (3 - k) & 1 for k in range(4)]
    for byte in data:
        raw += [byte >> (7 - k) & 1 for k in range(8)]
    crc = crc15(raw)
    raw += [crc >> (14 - k) & 1 for k in range(15)]
    stuffed, run, last = [], 0, None
    for b in raw:
        stuffed.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            stuffed.append(1 - b)
            run, last = 1, 1 - b
    return raw, stuffed


class CanBus:
    """the wire (a wired AND of the node's tx pin, when it drives it, and the
    host's driver) and the host's CAN controller: it sends the frames queued
    with `send` (the bus idle for 11 bits first; arbitration; the node's ACK
    makes a frame sent), receives the frames on the bus (resyncing on each
    falling edge, sampling mid bit), ACKs the node's frames and keeps those
    in `received`. `log(text)` reports frames and errors."""

    BIT = 32  # (PIO cycles per bit: can2040's PIO_CLOCK_PER_BIT)
    IDLE_BITS = 11

    def __init__(self, pio, rx_pin, tx_pin, log):
        self.pio, self.rx_pin, self.tx_pin, self.log = pio, rx_pin, tx_pin, log
        pio.pins_in = self.pins_in
        self.cycle = 0
        self.drive = 1
        self.queue = []
        self.received = []
        self.idle = 0
        self.rx = None
        self.tx = None
        # (EMU_PIO_TRACE=1: the bus events and the node's drive changes)
        self.trace = bool(os.environ.get("EMU_PIO_TRACE"))
        self.node_was = 1

    def node_level(self):
        p = self.pio
        if p.pin_dir >> self.tx_pin & 1 and not p.pin_out >> self.tx_pin & 1:
            return 0
        return 1

    def level(self):
        return self.drive & self.node_level()

    def pins_in(self):
        return MASK32 & ~(1 << self.rx_pin) | self.level() << self.rx_pin

    def send(self, fid, data):
        self.queue.append((fid, bytes(data)))

    def quiet(self):
        """(no frame on the bus or to send)"""
        return self.rx is None and self.tx is None and not self.queue and self.level()

    def run(self, until, stop=lambda: False):
        """run to cycle `until`, or to the first cycle `stop()` is true at;
        the cycle reached. A quiet bus whose PIO state repeats each bit but
        for counters going down by the same amount (can2040's sync state
        machine idles in a `jmp y--` loop) is skipped through."""
        seen = []
        while self.cycle < until:
            if stop():
                return self.cycle
            if not self.quiet():
                seen = []
            elif not self.pio.enabled:
                # (nothing runs: the bus stays idle)
                self.idle += until - self.cycle
                self.cycle = until
                break
            elif self.cycle % self.BIT == 0:
                seen = (seen + [(self.pio.key(), self.pio.counters())])[-3:]
                if len(seen) == 3 and seen[0][0] == seen[1][0] == seen[2][0]:
                    self.skip(seen, until)
                    seen = []
                    continue
            if not self.step():
                # (every state machine in a delay or stalled: the cycles up to
                # the end of the shortest delay or the host's next event do
                # nothing either)
                k = until - self.cycle
                d, e = self.pio.idle_cycles(), self.next_event()
                if d is not None:
                    k = min(k, d)
                if e is not None:
                    k = min(k, e - self.cycle)
                if k > 0:
                    self.pio.skip(k)
                    if self.rx is None:
                        self.idle += k
                    self.cycle += k
        return self.cycle

    def skip(self, seen, until):
        (_, c0), (_, c1), (_, c2) = seen
        down = [(a - b) & MASK32 for a, b in zip(c1, c2)]
        if down != [(a - b) & MASK32 for a, b in zip(c0, c1)]:
            return
        # (whole bits, keeping each counter above zero: its loop unchanged)
        k = (until - self.cycle) // self.BIT - 1
        for v, d in zip(c2, down):
            if d:
                if d > 1 << 16:
                    return
                k = min(k, v // d - 2)
        if k <= 0:
            return
        self.pio.set_counters([(v - k * d) & MASK32 for v, d in zip(c2, down)])
        self.cycle += k * self.BIT
        self.idle += k * self.BIT

    def next_event(self):
        """(the next cycle the host does something at, None if none)"""
        ev = []
        r, t = self.rx, self.tx
        if r is not None:
            ev.append(r["next"])
            if r["ack_slot"] is not None:
                ev += [c for c in r["ack_slot"] if c > self.cycle]
        if t is not None:
            # (the next bit boundary, this cycle included)
            ev.append(t["start"] - (t["start"] - self.cycle) // self.BIT * self.BIT)
        if r is None and t is None and self.queue:
            ev.append(self.cycle + max(0, self.IDLE_BITS * self.BIT - self.idle - 1))
        return min(ev) if ev else None

    def step(self):
        """one PIO cycle of the bus and the host; True if the PIO did
        something"""
        if self.trace and self.node_level() != self.node_was:
            self.node_was = self.node_level()
            self.log("# bus %d node %d\n" % (self.cycle, self.node_was))
        lvl = self.level()
        if self.rx is None:
            if not lvl:
                self.start_rx()
            else:
                self.idle += 1
                if self.queue and self.tx is None and self.idle >= self.IDLE_BITS * self.BIT:
                    self.start_tx()
                    lvl = self.level()
                    self.start_rx()
        else:
            self.rx_cycle(lvl)
        if self.tx is not None:
            self.tx_cycle()
        busy = self.pio.step()
        self.cycle += 1
        return busy

    # ---- the host's transmitter

    def start_tx(self):
        fid, data = self.queue[0]
        _, stuffed = frame_bits(fid, data)
        # (the CRC delimiter, the ACK slot and delimiter, the end of frame)
        bits = stuffed + [1, 1, 1] + [1] * 7
        self.tx = {"bits": bits, "start": self.cycle, "arb": 12, "frame": (fid, data)}
        self.drive = 0

    def tx_cycle(self):
        t = self.tx
        k = (self.cycle - t["start"]) // self.BIT
        self.drive = t["bits"][k] if k < len(t["bits"]) else 1
        if k >= len(t["bits"]):
            self.tx = None

    # ---- the host's receiver

    def start_rx(self):
        if self.trace:
            self.log("# bus %d start of frame%s\n" % (self.cycle, " (host)" if self.tx else ""))
        self.rx = {"next": self.cycle + self.BIT // 2, "raw": [], "run": 0, "last": None, "stage": "frame",
                   "own": self.tx is not None, "ack": None, "level": 0, "ack_slot": None}

    def rx_cycle(self, lvl):
        r = self.rx
        # (resync: a recessive to dominant edge restarts the bit)
        if not lvl and r["level"]:
            r["next"] = self.cycle + self.BIT // 2
        r["level"] = lvl
        # (the host's ACK of a node frame: dominant through the ACK slot)
        if r["ack_slot"] is not None:
            start, end = r["ack_slot"]
            if self.cycle == start:
                self.drive = 0
            elif self.cycle == end:
                self.drive, r["ack_slot"] = 1, None
        if self.cycle != r["next"]:
            return
        r["next"] += self.BIT
        self.sample(r, lvl)

    def sample(self, r, b):
        stage = r["stage"]
        if self.trace and stage != "frame":
            self.log("# bus %d %s %d %s\n" % (self.cycle, stage, b, "own" if r["own"] else ""))
        if stage == "frame":
            # (stuffed: start of frame through the CRC, a stuff bit after the
            # CRC's last bit included)
            if r["run"] == 5:
                if b == r["last"]:
                    return self.rx_error("stuff error")
                r["run"], r["last"] = 1, b
                return
            raw = r["raw"]
            if len(raw) >= 19 and len(raw) == 19 + 8 * min(r["dlc"], 8) + 15:
                # (the CRC delimiter)
                if self.trace:
                    self.log("# bus %d crc_delim %d %s\n" % (self.cycle, b, "own" if r["own"] else ""))
                if not b:
                    return self.rx_error("crc delimiter")
                n = len(raw) - 15
                r["crc_ok"] = crc15(raw[:n]) == int("".join(map(str, raw[n:])), 2)
                r["stage"] = "ack"
                if not r["own"] and r["crc_ok"]:
                    # (the ACK slot: the next bit, from the end of this one)
                    start = self.cycle + self.BIT // 2
                    r["ack_slot"] = (start, start + self.BIT)
                return
            r["run"] = r["run"] + 1 if b == r["last"] else 1
            r["last"] = b
            if self.tx is not None and r["own"]:
                sent = self.tx["bits"][(self.cycle - self.tx["start"]) // self.BIT]
                if sent != b:
                    if sent == 1 and len(raw) <= self.tx["arb"]:
                        # (arbitration lost: the other frame goes on)
                        self.tx, self.drive, r["own"] = None, 1, False
                    else:
                        return self.rx_error("bit error")
            raw.append(b)
            if len(raw) == 19:
                r["dlc"] = int("".join(map(str, raw[15:19])), 2)
            return
        if stage == "ack":
            r["ack"] = not b
            r["stage"] = "ack_delim"
            return
        if stage == "ack_delim":
            r["stage"], r["eof"] = "eof", 0
            return
        r["eof"] += 1
        if r["eof"] < 7:
            return
        raw = r["raw"]
        fid = int("".join(map(str, raw[1:12])), 2)
        data = bytes(int("".join(map(str, raw[19 + 8 * k: 27 + 8 * k])), 2) for k in range(min(r["dlc"], 8)))
        if r["own"]:
            if r["ack"]:
                self.queue.pop(0)
            else:
                self.log("# can frame %x %s: no ack\n" % (fid, data.hex()))
        elif r["crc_ok"]:
            self.received.append((fid, data))
        else:
            self.log("# can frame %x %s: crc error\n" % (fid, data.hex()))
        self.rx, self.idle = None, 0

    def rx_error(self, what):
        self.log("# can bus %s\n" % what)
        self.rx, self.tx, self.drive, self.idle = None, None, 1, -self.IDLE_BITS * self.BIT
