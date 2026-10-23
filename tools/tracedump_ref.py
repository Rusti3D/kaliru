#!/usr/bin/env python3
# Reference twin of kaliru's src/bin/tracedump.rs: decodes a serialqueue trace
# with Kalico's own msgproto, same output format, so the two can be diffed.
# Run with the Kalico venv python from ~/rk/kalico.
# usage: tracedump_ref.py <trace file> <mcu>=<dict> [<mcu>=<dict>...]
import sys

from klippy import msgproto


def decode(mp, msg):
    msgid, _ = msgproto.PT_int32().parse(msg, 0)
    mid = mp.messages_by_id.get(msgid)
    if not isinstance(mid, msgproto.MessageFormat):
        raise ValueError("unknown or output message id")
    params, pos = mid.parse(msg, 0)
    if pos != len(msg):
        raise ValueError("extra data")
    return mid.format_params(params)


def main():
    parsers = {}
    for a in sys.argv[2:]:
        name, path = a.split("=", 1)
        mp = msgproto.MessageParser()
        mp.process_identify(open(path, "rb").read(), decompress=False)
        parsers[("serialq " + name)[:15]] = mp
    out = []
    for line in open(sys.argv[1]):
        sq, q, mn, req, hexmsg = line.rstrip("\n").rsplit(" ", 4)
        try:
            text = decode(parsers[sq], bytes.fromhex(hexmsg))
        except Exception as e:
            text = "DECODE ERROR %s: %s" % (e, hexmsg)
        out.append("%s %s %s %s %s\n" % (sq, q, mn, req, text))
    sys.stdout.write("".join(out))


if __name__ == "__main__":
    main()
