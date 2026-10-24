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
import json
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
    def __init__(self, kind, cfg, workdir, api=False):
        self.tty = os.path.join(workdir, "printer")
        self.logfile = os.path.join(workdir, "klippy.log")
        self.apisock = os.path.join(workdir, "api.sock")
        for f in (self.tty, self.logfile, self.apisock):
            if os.path.lexists(f):
                os.unlink(f)
        extra = ["-a", self.apisock] if api else []
        self.proc = subprocess.Popen(HOSTS[kind] + [cfg, "-I", self.tty, "-l", self.logfile] + extra, cwd=KALICO,
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

    def resources(self):
        """the host process's peak and current memory (VmHWM, VmRSS) and cpu
        time (not part of the transcript: workdir/resources.txt)"""
        status = open("/proc/%d/status" % self.proc.pid).read()
        mem = {k: v.split()[0] for k, v in re.findall(r"^(VmHWM|VmRSS|RssAnon|RssFile):\s+(.*)$", status, re.M)}
        # (own memory as Moonruster's benchmarks count it: anonymous pages;
        # program code and data mapped from files can be dropped by the kernel)
        rollup = open("/proc/%d/smaps_rollup" % self.proc.pid).read()
        pss_anon = re.search(r"^Pss_Anon:\s+(\d+)", rollup, re.M).group(1)
        stat = open("/proc/%d/stat" % self.proc.pid).read().rsplit(")", 1)[1].split()
        cpu = (int(stat[11]) + int(stat[12])) / os.sysconf("SC_CLK_TCK")
        return "peak_rss_kb=%s rss_kb=%s rss_anon_kb=%s rss_file_kb=%s pss_anon_kb=%s cpu_s=%.2f" % (
            mem["VmHWM"], mem["VmRSS"], mem["RssAnon"], mem["RssFile"], pss_anon, cpu)

    def close(self):
        os.close(self.fd)
        self.proc.send_signal(signal.SIGINT)
        try:
            # (LIVE_CLOSE_WAIT: longer for a host under valgrind)
            self.proc.wait(float(os.environ.get("LIVE_CLOSE_WAIT", 5)))
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()


class Api:
    """a client of the host's API socket (webhooks): requests and the
    status notifications of subscriptions"""

    def __init__(self, path):
        import socket
        for _ in range(500):
            try:
                self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                self.sock.connect(path)
                break
            except OSError:
                time.sleep(0.01)
        else:
            sys.exit("no api socket %s" % path)
        self.buf = b""
        self.next_id = 1
        self.notes = []

    def _read(self, end):
        """the next message (raw json text) before `end`, or None"""
        while b"" not in self.buf:
            left = end - time.monotonic()
            if left <= 0 or not select.select([self.sock], [], [], left)[0]:
                return None
            data = self.sock.recv(65536)
            if not data:
                return None
            self.buf += data
        msg, self.buf = self.buf.split(b"", 1)
        return msg.decode(errors="replace")

    def request(self, method, params, timeout=CMD_TIMEOUT):
        rid = self.next_id
        self.next_id += 1
        self.sock.sendall(json.dumps({"id": rid, "method": method, "params": params}).encode() + b"")
        end = time.monotonic() + timeout
        while (m := self._read(end)) is not None:
            if json.loads(m).get("id") == rid:
                return m
            self.notes.append(m)
        return "(no answer in %.0f s)" % timeout

    def notifications(self, timeout):
        end = time.monotonic() + timeout
        while (m := self._read(end)) is not None:
            self.notes.append(m)
        out, self.notes = self.notes, []
        return out


def mask_api(text):
    # (times differ run to run)
    return re.sub(r'"eventtime": ?[-0-9.e]+', '"eventtime": X', text)


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
    use_api = re.search(r"^@(api|sub|notify) ", commands, re.M) is not None
    host = Host(kind, cfg, workdir, api=use_api)
    out = ["= state"] + wait_ready(host)
    api = Api(host.apisock) if use_api else None
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
            elif word == "@api":
                # @api <method> <json params>
                method, _, params = arg.partition(" ")
                out.append("api: " + mask_api(api.request(method, json.loads(params or "{}"))))
            elif word == "@sub":
                # @sub <json objects>: objects/subscribe
                out.append("api: " + mask_api(api.request("objects/subscribe", {"objects": json.loads(arg), "response_template": {}})))
            elif word == "@notify":
                # @notify <seconds>: the status notifications that arrived
                out += ["notify: " + mask_api(n) for n in api.notifications(float(arg))]
            elif word == "@log":
                out += ["log: " + l for l in host.log_lines() if re.search(arg, l)]
            else:
                out += host.command(line)
    finally:
        with open(os.path.join(workdir, "resources.txt"), "w") as f:
            f.write(host.resources() + "\n")
        host.close()
        for m in mcus.values():
            m.stop()
    print("\n".join(out))


if __name__ == "__main__":
    main()
