#!/usr/bin/env python3
# Drives a linux MCU process (Kalico's C klipper.elf or kaliru-mcu) over its
# pty with a command script and prints what it answers, decoded with Kalico's
# msgproto. Clock values are masked; run it on both binaries and diff.
#
# usage: mcu_run.py <mcu binary> <script> [--kalico <kalico checkout>]
#                   [--fakehw <fakehw.so>]
#
# --fakehw runs the MCU under the fakehw.c shim (fake gpio / spi / i2c / adc)
# and appends its event log (pin writes, bus transfers) without the times.
#
# Script lines: a command in msgproto text form ("allocate_oids count=4"),
# "#raw <hex>" (bytes as they are), "#wait <seconds>", "#identify" (fetch
# and summarise the data dictionary), "# ..." comments. "{clock+N}" in a
# command is the last clock the MCU reported (get_clock / get_uptime) plus N.
import hashlib
import json
import os
import select
import subprocess
import sys
import termios
import time
import zlib

KALICO = os.path.expanduser("~/rk/kalico")
if "--kalico" in sys.argv:
    KALICO = sys.argv[sys.argv.index("--kalico") + 1]
sys.path.insert(0, os.path.join(KALICO, "klippy"))
import msgproto  # noqa: E402

# fields whose values depend on wall clock time
MASKED = {"clock", "high", "next_clock", "trigger_clock", "count_clock", "trigger_ticks", "report_clock", "query_ticks"}


class Mcu:
    def __init__(self, binary, fakehw=None):
        self.tty = "/tmp/kaliru_mcu_test_%d" % os.getpid()
        env = dict(os.environ)
        self.hwlog = None
        if fakehw:
            if not os.path.exists(fakehw):
                sys.exit("no fake hardware shim %s" % fakehw)
            self.hwlog = "/tmp/kaliru_mcu_hw_%d.log" % os.getpid()
            env["LD_PRELOAD"] = os.path.abspath(fakehw)
            env["FAKEHW_LOG"] = self.hwlog
        self.proc = subprocess.Popen([binary, "-I", self.tty], stderr=subprocess.PIPE, env=env)
        self.last_clock = 0
        for _ in range(200):
            if os.path.exists(self.tty):
                break
            time.sleep(0.01)
        self.fd = os.open(self.tty, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = attrs[1] = attrs[3] = 0  # raw
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.parser = msgproto.MessageParser()
        self.seq = 0
        self.buf = b""

    def close(self):
        self.proc.kill()
        self.proc.wait()
        os.close(self.fd)
        try:
            os.unlink(self.tty)
        except OSError:
            pass
        return self.proc.stderr.read().decode(errors="replace")

    def send_raw(self, data):
        os.write(self.fd, bytes(data))

    def send(self, cmd):
        block = [msgproto.MESSAGE_MIN + len(cmd), (self.seq & 0xF) | 0x10] + list(cmd)
        block += msgproto.crc16_ccitt(block) + [msgproto.MESSAGE_SYNC]
        self.seq += 1
        self.send_raw(block)

    def read_blocks(self, until_ack, quiet=0.01, timeout=2.0):
        """the message blocks received: until an ack (and then a quiet
        period) or a quiet period"""
        out = []
        end = time.time() + timeout
        acked = False
        while time.time() < end:
            wait = quiet if (acked or not until_ack) else end - time.time()
            r, _, _ = select.select([self.fd], [], [], max(wait, 0))
            if not r:
                if acked or not until_ack:
                    break
                continue
            self.buf += os.read(self.fd, 4096)
            while True:
                n = self.parser.check_packet(self.buf)
                if n == 0:
                    break
                if n < 0:
                    out.append(("garbage", self.buf[:1].hex()))
                    self.buf = self.buf[1:]
                    continue
                block, self.buf = self.buf[:n], self.buf[n:]
                out.append(("block", block))
                if n == msgproto.MESSAGE_MIN:
                    acked = True
        return out

    def format(self, block):
        if len(block) == msgproto.MESSAGE_MIN:
            return "ack seq=%02x" % block[msgproto.MESSAGE_POS_SEQ]
        params = self.parser.parse(block)
        name = params["#name"]
        if name in ("clock", "uptime"):
            self.last_clock = params["clock"]
        parts = [name]
        for k, v in params.items():
            if k.startswith("#"):
                continue
            if k in MASKED:
                v = "*"
            elif isinstance(v, bytes):
                v = v.hex()
            parts.append("%s=%s" % (k, v))
        return " ".join(parts)

    def identify(self):
        data = b""
        while True:
            cmd = self.parser.create_command("identify offset=%d count=40" % len(data))
            self.send(cmd)
            got = None
            for kind, block in self.read_blocks(True):
                if kind == "block" and len(block) > msgproto.MESSAGE_MIN:
                    params = self.parser.parse(block)
                    if params["#name"] == "identify_response":
                        got = params["data"]
            if not got:
                break
            data += got
        raw = zlib.decompress(data)
        self.parser.process_identify(raw, decompress=False)
        d = json.loads(raw)
        for k in ("version", "build_versions"):
            d.pop(k, None)
        return json.dumps(d, sort_keys=True), len(data)


def expand(mcu, line):
    # {clock+N}
    while "{clock+" in line:
        i = line.index("{clock+")
        j = line.index("}", i)
        v = (mcu.last_clock + int(line[i + 7 : j], 0)) & 0xFFFFFFFF
        line = line[:i] + str(v) + line[j + 1 :]
    return line


def main():
    binary, script = sys.argv[1], sys.argv[2]
    fakehw = sys.argv[sys.argv.index("--fakehw") + 1] if "--fakehw" in sys.argv else None
    mcu = Mcu(binary, fakehw)
    out = []
    try:
        # the commands are only known after an identify
        dictionary, size = mcu.identify()
        for line in open(script):
            line = line.strip()
            if not line or (line.startswith("#") and not line.startswith(("#raw", "#wait", "#identify"))):
                continue
            out.append("> " + line)
            if line == "#identify":
                sha = hashlib.sha1(dictionary.encode()).hexdigest()
                out.append("< dictionary %d bytes sha=%s" % (len(dictionary), sha))
                continue
            if line.startswith("#wait"):
                time.sleep(float(line.split()[1]))
                blocks = mcu.read_blocks(False)
            elif line.startswith("#raw"):
                mcu.send_raw(bytes.fromhex(line.split()[1]))
                blocks = mcu.read_blocks(False, quiet=0.2)
            else:
                mcu.send(mcu.parser.create_command(expand(mcu, line)))
                blocks = mcu.read_blocks(True)
            for kind, b in blocks:
                if kind == "garbage":
                    out.append("< garbage %s" % b)
                    continue
                f = mcu.format(b)
                if f.startswith("stats "):
                    continue
                out.append("< " + f)
    finally:
        err = mcu.close()
    if mcu.hwlog:
        out.append("hardware:")
        for ev in open(mcu.hwlog):
            out.append("  " + ev.split(" ", 1)[1].rstrip())
        os.unlink(mcu.hwlog)
    print("\n".join(out))
    if err.strip():
        print("stderr:")
        print(err.rstrip())


if __name__ == "__main__":
    main()
