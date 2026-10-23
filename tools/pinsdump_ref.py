#!/usr/bin/env python3
# Reference twin of kaliru's src/bin/pinsdump.rs: runs a pins.py operation
# script with Kalico's pins module. Run with PYTHONPATH=~/rk/kalico.
# usage: pinsdump_ref.py <script>
import sys

from klippy import pins


class Chip:
    def setup_pin(self, pin_type, pin_params):
        return object()


def show(p):
    return "chip_name=%r pin=%r invert=%d pullup=%d share_type=%r" % (
        p["chip_name"], p["pin"], p["invert"], p["pullup"], p.get("share_type"))


def main():
    pp = pins.PrinterPins()
    seen = []
    for line in open(sys.argv[1]).read().splitlines():
        op, _, rest = line.partition(" ")
        try:
            if op == "new":
                pp = pins.PrinterPins()
                seen = []
                out = "ok"
            elif op == "chip":
                pp.register_chip(rest, Chip())
                out = "ok"
            elif op == "parse":
                inv, pull, desc = (rest.split(" ", 2) + ["", ""])[:3]
                out = show(pp.parse_pin(desc, inv == "1", pull == "1"))
            elif op == "lookup":
                inv, pull, share, desc = (rest.split(" ", 3) + ["", ""])[:4]
                p = pp.lookup_pin(desc, inv == "1", pull == "1",
                                  None if share == "-" else share)
                for i, s in enumerate(seen):
                    if s is p:
                        break
                else:
                    seen.append(p)
                    i = len(seen) - 1
                out = "id=%d %s" % (i, show(p))
            elif op == "multi":
                pp.allow_multi_use_pin(rest)
                out = "ok"
            elif op == "reset":
                try:
                    pp.reset_pin_sharing(pp.parse_pin(rest))
                    out = "ok"
                except KeyError as e:
                    out = "ERR KeyError %s" % (e,)
            elif op in ("alias", "reserve", "update"):
                chip, _, args = rest.partition(" ")
                r = pp.get_pin_resolver(chip)
                if op == "alias":
                    a, _, p = args.partition(" ")
                    r.alias_pin(a, p)
                    out = "ok"
                elif op == "reserve":
                    p, _, n = args.partition(" ")
                    r.reserve_pin(p, n)
                    out = "ok"
                else:
                    out = r.update_command(args)
            else:
                continue
        except pins.error as e:
            out = "ERR %s" % (e,)
        print("%s -> %r" % (line, out))


if __name__ == "__main__":
    main()
