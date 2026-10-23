#!/usr/bin/env python3
# Parity check for msgproto / parsedump: runs Python `python -m klippy.parsedump`
# and Rust `parsedump` on every serial output file of a refrun tree, plus
# copies with flipped bytes (seeded) to exercise resync / error paths, and
# compares stdout, exit code and the "Invalid data" stderr lines.
#
# usage: parsedump_cmp.py [--runs ~/rk/ref1] [--rust ~/rk/target/release/parsedump]
#                         [--corrupt N]
import argparse
import os
import pathlib
import random
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

H = pathlib.Path.home() / "rk"
PY = [str(H / "kalico/.venv/bin/python"), "-m", "klippy.parsedump"]


def dict_for(run, f):
    cmd = dict(l.split(" ", 1) for l in (run / "cmd").read_text().splitlines())
    dl = cmd["dicts"].split()
    name = f.name[len("output"):].lstrip("-")
    for x in dl[1:]:
        if x.split("=", 1)[0] == name:
            return x.split("=", 1)[1]
    return dl[0]


def run(cmd):
    p = subprocess.run(cmd, cwd=H / "kalico", capture_output=True)
    invalid = [l for l in p.stderr.splitlines() if l.endswith(b"Invalid data")]
    return p.stdout, p.returncode != 0, invalid


def check(args, d, f, label):
    py, rs = run(PY + [d, str(f)]), run([args.rust, d, str(f)])
    if py == rs:
        return None
    what = [n for n, a, b in zip(("stdout", "failed", "invalid-data lines"), py, rs) if a != b]
    return f"DIFF {label}: {', '.join(what)} (failed py={py[1]} rust={rs[1]})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=pathlib.Path, default=H / "ref1")
    ap.add_argument("--rust", default=str(H / "target/release/parsedump"))
    ap.add_argument("--corrupt", type=int, default=3, help="corrupted copies per file")
    args = ap.parse_args()
    tmp = pathlib.Path(tempfile.mkdtemp())
    jobs = []
    rnd = random.Random(1)
    for f in sorted(args.runs.glob("*/*/output*")):
        d = dict_for(f.parent, f)
        label = str(f.relative_to(args.runs))
        jobs.append((d, f, label))
        data = f.read_bytes()
        for i in range(args.corrupt if data else 0):
            b = bytearray(data)
            for _ in range(8):
                b[rnd.randrange(len(b))] = rnd.randrange(256)
            cf = tmp / f"{label.replace('/', '_')}.{i}"
            cf.write_bytes(bytes(b))
            jobs.append((d, cf, f"{label} corrupt#{i}"))
    with ThreadPoolExecutor(os.cpu_count()) as ex:
        res = list(ex.map(lambda j: check(args, *j), jobs))
    bad = [r for r in res if r]
    for r in bad:
        print(r)
    print(f"{len(jobs) - len(bad)}/{len(jobs)} identical")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
