#!/usr/bin/env python3
# Per-function flash sizes of the C and the Rust image of a config, matched by
# function name (Rust's v0 mangled names by their last path segment, C's LTO
# suffixes dropped), the largest Rust growth first: where to look when the
# Rust image is too big.
#
#   size_cmp.py <C klipper.elf> <Rust kaliru-mcu elf> [rows] [nm]
#   e.g. size_cmp.py ~/rk/fw_atmega168-serial/klipper.elf \
#        ~/rk/fwtarget-ctr-atmega168-serial/avr-none/mcu/kaliru-mcu.elf 40 avr-nm
import re
import subprocess
import sys


def sizes(elf, nm):
    out = subprocess.run([nm, "-S", "--size-sort", elf], capture_output=True, text=True, check=True).stdout
    d = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) != 4 or p[2] not in "tTrRwW":
            continue
        name = p[3]
        if name.startswith("_R"):
            # (v0 mangling: the last <length><identifier>)
            base = name.split(".")[0]
            for m in re.finditer(r"\d+", base):
                rest = base[m.end():]
                if len(rest) == int(m.group()):
                    name = rest
                    break
        name = re.sub(r"\.(lto_priv|part|constprop|isra)\.\d+", "", name).lower()
        d[name] = d.get(name, 0) + int(p[1], 16)
    return d


def main():
    nm = sys.argv[4] if len(sys.argv) > 4 else "avr-nm"
    c, r = sizes(sys.argv[1], nm), sizes(sys.argv[2], nm)
    print("C total %d  Rust total %d" % (sum(c.values()), sum(r.values())))
    rows = sorted(set(c) | set(r), key=lambda n: -(r.get(n, 0) - c.get(n, 0)))
    for n in rows[: int(sys.argv[3]) if len(sys.argv) > 3 else 40]:
        print("%6d %6d %+6d  %s" % (c.get(n, 0), r.get(n, 0), r.get(n, 0) - c.get(n, 0), n))


if __name__ == "__main__":
    main()
