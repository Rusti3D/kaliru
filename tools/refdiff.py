#!/usr/bin/env python3
# Compares two refrun.py output trees.
#
# Must match: the serialqueue trace (every message the host hands to
# serialqueue, with command queue, min_clock, req_clock and bytes, in call
# order), the exit code, and the log after masking volatile values.
#
# Only reported, not required: the raw serial output files. In batch mode
# serialqueue releases messages against a wall-clock driven fake MCU clock
# (clocksync.connect_file, serialqueue.c check_send_command) and drops what is
# not yet due at exit, so block framing, cross-queue order and the tail of
# the stream vary between runs of the reference itself.
#
# usage: refdiff.py A_DIR B_DIR [--no-log]
import argparse
import difflib
import pathlib
import re
import sys

# Values that legitimately change between runs (wall clock, pids, addresses)
LOG_MASKS = [
    # printer.py Printer.run: asctime, time.time(), reactor.monotonic()
    (re.compile(rb"Start printer at .*"), b"Start printer at <TIME>"),
    (re.compile(rb"\b\d{10}\.\d+\b"), b"<TIME>"),        # time.time()
    (re.compile(rb"0x[0-9a-f]{8,}"), b"<ADDR>"),
    # (repr() of an object: any address, a 32-bit host's are shorter)
    (re.compile(rb" object at 0x[0-9a-f]+>"), b" object at <ADDR>>"),
    (re.compile(rb"\bpid \d+"), b"pid <PID>"),
    # AFC's logger prefixes its lines with reactor.monotonic() ("%.3f [file:func():line]")
    (re.compile(rb"^((?:INFO|DEBUG|WARNING|ERROR):root:)\d+\.\d{3} \[", re.M), rb"\1<MONO> ["),
    # util.dump_mcu_build: paths relative to the install dir (klippy/ vs
    # the kaliru binary's dir)
    (re.compile(rb"No build file \S*?/\.\./"), b"No build file <DIR>/../"),
    # gcode.py dump_debug: "Read %f: %r" of the reactor.monotonic() read time
    (re.compile(rb"^Read \d+\.\d+: ", re.M), b"Read <MONO>: "),
    # reactor.py get_gc_stats in the shutdown dump: when python's gc last ran
    (re.compile(rb"^INFO:root:Reactor garbage collection: \([^\n]*\)$", re.M), b"INFO:root:Reactor garbage collection: <GC>"),
    # traceback frames (kaliru writes one placeholder frame line)
    (re.compile(rb"Traceback \(most recent call last\):\n(?:  [^\n]*\n)+"),
     b"Traceback (most recent call last):\n  <frames>\n"),
    # the same inside AFC's log lines (each line carries the logger's prefix)
    (re.compile(rb"^((?:INFO|DEBUG|WARNING|ERROR):root:<MONO> \[[^\]\n]*\] *- [A-Z]+: ?)([^\n]*)Traceback \(most recent call last\):\n(?:\1  [^\n]*\n)+", re.M),
     rb"\1\2Traceback (most recent call last):\n\1  <frames>\n"),
]


# homing.py _set_current_homing iterates a set() of rails, whose order follows
# object ids; compare each run of its "tmc <stepper>: " lines as a sorted block
TMC_RUN = re.compile(rb"(?:^INFO:root:tmc \S+: [^\n]*\n)+", re.M)


# AFC_ACE's serial thread logs "AFC_ACE: t=<monotonic> ... (<secs>s)" lines
# straight to the root logger, interleaved with the reactor's by thread timing:
# they compare as their own sequence, after the rest of the log
ACE_THREAD = re.compile(rb"^(?:INFO|DEBUG):root:AFC_ACE: [^\n]*\n", re.M)
# afcDeltaTime: wall-clock durations of AFC's loads and unloads
AFC_TIMES = [(re.compile(rb"\(\xce\x94t:\d+\.\d+s, t:\d+\.\d+\)"), "(Δt:<SECS>s, t:<SECS>)".encode()),
             (re.compile(rb"( (?:done|toolhead|unloaded|loaded)[^\n]* t:)\d+\.\d+$", re.M), rb"\1<SECS>")]
# on a shutdown AFC_ACE queues its stops for the serial thread and the process
# exits: how many go out before it does is a race
ACE_SHUTDOWN_BURST = re.compile(rb"(?:^INFO:root:AFC_ACE: t=\S+ (?:stop_feed_filament|stop_unwind_filament|stop_feed_assist|drying_stop) [^\n]*\n)+\Z", re.M)
ACE_TIMES = [(re.compile(rb"(AFC_ACE: (?:\S+ )*?t=)\d+\.\d+"), rb"\1<MONO>"),
             (re.compile(rb"\(\d+\.\d+s\)$", re.M), b"(<SECS>s)")]


def mask(log):
    # (the serial thread's lines first: one can land inside a traceback)
    thread = ACE_THREAD.findall(log)
    log = ACE_THREAD.sub(b"", log)
    for rx, rep in LOG_MASKS:
        log = rx.sub(rep, log)
    log = TMC_RUN.sub(lambda m: b"".join(sorted(m.group(0).splitlines(True))), log)
    for rx, rep in AFC_TIMES:
        log = rx.sub(rep, log)
    if thread:
        thread = b"".join(thread)
        if b"Transition to shutdown state" in log:
            thread = ACE_SHUTDOWN_BURST.sub(b"<shutdown stops>\n", thread)
        log += b"-- AFC_ACE serial thread --\n" + thread
        for rx, rep in ACE_TIMES:
            log = rx.sub(rep, log)
    return log


def lines(path, masked=False):
    data = path.read_bytes() if path.exists() else b""
    if masked:
        data = mask(data)
    return data.decode(errors="replace").splitlines()


def udiff(a, b, n):
    return list(difflib.unified_diff(a, b, "A", "B", n=n, lineterm=""))[2:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a", type=pathlib.Path)
    ap.add_argument("b", type=pathlib.Path)
    ap.add_argument("--no-log", action="store_true", help="skip log comparison")
    args = ap.parse_args()

    runs = sorted({p.parent.relative_to(args.a) for p in args.a.glob("*/*/exit")}
                  | {p.parent.relative_to(args.b) for p in args.b.glob("*/*/exit")})
    same = serial_differs = 0
    for r in runs:
        ra, rb = args.a / r, args.b / r
        problems = []
        if not rb.exists() or not ra.exists():
            problems.append("missing in " + ("B" if ra.exists() else "A"))
        else:
            if not (ra / "trace").exists() and (rb / "trace").exists():
                problems.append("no trace in A")
            ta, tb = lines(ra / "trace"), lines(rb / "trace")
            if ta != tb:
                problems.append(f"trace differs ({len(ta)} vs {len(tb)} messages):\n    "
                                + "\n    ".join(udiff(ta, tb, 2)[:30]))
            if (ra / "exit").read_text() != (rb / "exit").read_text():
                problems.append(f"exit {(ra / 'exit').read_text().strip()} vs "
                                f"{(rb / 'exit').read_text().strip()}")
            if not args.no_log:
                la, lb = lines(ra / "log", True), lines(rb / "log", True)
                if la != lb:
                    problems.append("log differs:\n    " + "\n    ".join(udiff(la, lb, 1)[:20]))
            for name in sorted({p.name for p in ra.glob("output*")}
                               | {p.name for p in rb.glob("output*")}):
                fa, fb = ra / name, rb / name
                if not fa.exists() or not fb.exists() or fa.read_bytes() != fb.read_bytes():
                    serial_differs += 1
        if problems:
            print(f"DIFF {r}")
            for p in problems:
                print("  " + p)
        else:
            same += 1
    print(f"{same}/{len(runs)} identical (info: {serial_differs} raw serial files differ)")
    sys.exit(0 if same == len(runs) else 1)


if __name__ == "__main__":
    main()
