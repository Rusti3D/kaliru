#!/usr/bin/env python3
# Live parity runs: a host (Kalico's python klippy, or kaliru) with real linux
# MCU processes (Kalico's C linux MCU under the firmware fakehw shim: fake
# gpio / spi / i2c / adc), driven through the gcode pseudo-tty by a scenario.
# Prints the transcript: the state the host reached, each command with what
# it answered, the messages that came while waiting, and the log lines the
# scenario asks for. Run it with both hosts and diff (tools/live_cmp.sh).
#
# usage: live_run.py <py|rs> <scenario.live> <workdir>
#
# A scenario (tests/live/*.live): the printer config, a line "---", then one
# command per line:
#   G28, QUERY_ENDSTOPS, ...      a gcode command: its answer up to "ok"
#   @sleep <s>                    wait, keeping what the host sends meanwhile
#   @stop <mcu> / @start <mcu>    kill / start the MCU process (a disconnect)
#   @pause <mcu> / @resume <mcu>  stop / continue the MCU process (SIGSTOP)
#   @wait_log <regex> [<s>]       wait for a log line (default 20 s)
#   @log <regex>                  the log lines so far that match
# In the config, "{mcu}" / "{mcu:<name>}" are the serial ports of the main /
# a named MCU (the harness starts one MCU process for each), and a line
# "@env NAME=VALUE" sets a variable for the MCU processes (the fakehw
# inputs: FAKEHW_GPIO_IN, FAKEHW_SPI_RX, FAKEHW_SPI_TMC). Lines starting
# with "#" in the command part are comments.
import os
import re
import select
import signal
import subprocess
import sys
import time

RK = os.path.expanduser("~/rk")
KALICO = os.path.join(RK, "kalico")
MCU = os.environ.get("LIVE_MCU", os.path.join(RK, "fwlinux", "klipper.elf"))
FAKEHW = os.path.join(RK, "kaliru_fakehw.so")
HOSTS = {
    "py": [os.path.join(KALICO, ".venv", "bin", "python"), "-m", "klippy"],
    "rs": [os.environ.get("LIVE_KALIRU", os.path.join(RK, "target", "release", "kaliru"))],
}
READY_TIMEOUT = 30.0
CMD_TIMEOUT = 20.0


class Mcu:
    """a linux MCU process on a pseudo-tty"""

    def __init__(self, name, workdir, env):
        self.name = name
        self.tty = os.path.join(workdir, "mcu_%s" % name)
        self.hwlog = os.path.join(workdir, "hw_%s.log" % name)
        self.env = env
        self.proc = None

    def start(self):
        env = dict(os.environ, LD_PRELOAD=FAKEHW, FAKEHW_LOG=self.hwlog, **self.env)
        self.proc = subprocess.Popen([MCU, "-I", self.tty], env=env, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        for _ in range(200):
            if os.path.exists(self.tty):
                return
            time.sleep(0.01)
        sys.exit("mcu %s did not start" % self.name)

    def stop(self):
        if self.proc:
            self.proc.kill()
            self.proc.wait()
            self.proc = None
        if os.path.lexists(self.tty):
            os.unlink(self.tty)


class Host:
    def __init__(self, kind, cfg, workdir):
        self.tty = os.path.join(workdir, "printer")
        self.logfile = os.path.join(workdir, "klippy.log")
        for f in (self.tty, self.logfile):
            if os.path.lexists(f):
                os.unlink(f)
        self.proc = subprocess.Popen(HOSTS[kind] + [cfg, "-I", self.tty, "-l", self.logfile], cwd=KALICO,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(1000):
            if os.path.exists(self.tty):
                break
            time.sleep(0.01)
        else:
            sys.exit("host did not open %s" % self.tty)
        self.fd = os.open(self.tty, os.O_RDWR | os.O_NOCTTY)
        self.buf = b""

    def read_line(self, end):
        """the next line the host sends before the time `end`, or None"""
        while b"\n" not in self.buf:
            left = end - time.monotonic()
            if left <= 0 or not select.select([self.fd], [], [], left)[0]:
                return None
            try:
                self.buf += os.read(self.fd, 4096)
            except OSError:
                return None
        line, self.buf = self.buf.split(b"\n", 1)
        return line.decode(errors="replace")

    def lines(self, timeout):
        """the lines the host sends within `timeout` seconds"""
        end = time.monotonic() + timeout
        out = []
        while (l := self.read_line(end)) is not None:
            out.append(l)
        return out

    def command(self, line, timeout=CMD_TIMEOUT):
        """send a gcode line: its answer, up to the "ok" (and what came
        before it)"""
        os.write(self.fd, (line + "\n").encode())
        end = time.monotonic() + timeout
        out = []
        while (l := self.read_line(end)) is not None:
            # (an answer ends with "ok", or "ok <text>": M105, M115)
            if l == "ok" or l.startswith("ok "):
                return out + ([l] if l != "ok" else [])
            out.append(l)
        out.append("(no ok in %.0f s)" % timeout)
        return out

    def log_lines(self):
        try:
            return open(self.logfile, errors="replace").read().splitlines()
        except OSError:
            return []

    def close(self):
        os.close(self.fd)
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()


def wait_ready(host):
    """poll STATUS until the host is past its startup (ready, or a config
    error or shutdown it stays in): the answer once two in a row, a second
    apart, are the same (a shutdown can follow the ready state closely)"""
    end = time.monotonic() + READY_TIMEOUT
    last = None
    while True:
        ans = host.command("STATUS", timeout=2.0)
        text = "\n".join(ans)
        settled = ans and "(no ok" not in text and "attempting to connect" not in text
        if (settled and ans == last) or time.monotonic() > end:
            return ans
        last = ans if settled else None
        time.sleep(1.0)


def main():
    kind, scenario, workdir = sys.argv[1:4]
    os.makedirs(workdir, exist_ok=True)
    text = open(scenario).read()
    cfg_text, _, commands = text.partition("\n---\n")
    env = dict(re.findall(r"^@env (\w+)=(.*)$", cfg_text, re.M))
    cfg_text = re.sub(r"^@env .*\n", "", cfg_text, flags=re.M)
    mcus = {}

    def port(m):
        name = m.group(1) or "mcu"
        if name not in mcus:
            mcus[name] = Mcu(name, workdir, env)
        return mcus[name].tty

    cfg_text = re.sub(r"\{mcu(?::(\w+))?\}", port, cfg_text)
    cfg = os.path.join(workdir, "printer.cfg")
    open(cfg, "w").write(cfg_text)
    for m in mcus.values():
        m.start()
    host = Host(kind, cfg, workdir)
    out = ["= state"] + wait_ready(host)
    try:
        for line in commands.splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            out.append("> " + line)
            word, _, arg = line.partition(" ")
            if word == "@sleep":
                out += host.lines(float(arg))
            elif word == "@stop":
                mcus[arg].stop()
            elif word == "@start":
                mcus[arg].start()
            elif word in ("@pause", "@resume"):
                mcus[arg].proc.send_signal(signal.SIGSTOP if word == "@pause" else signal.SIGCONT)
            elif word == "@wait_log":
                regex, _, t = arg.partition(" ")
                end = time.monotonic() + float(t or 20)
                while not any(re.search(regex, l) for l in host.log_lines()):
                    if time.monotonic() > end:
                        out.append("(no log line %s)" % regex)
                        break
                    out += host.lines(0.1)
            elif word == "@log":
                out += ["log: " + l for l in host.log_lines() if re.search(arg, l)]
            else:
                out += host.command(line)
    finally:
        host.close()
        for m in mcus.values():
            m.stop()
    print("\n".join(out))


if __name__ == "__main__":
    main()
