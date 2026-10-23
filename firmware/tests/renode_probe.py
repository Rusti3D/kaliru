#!/usr/bin/env python3
# Debug aid: start an image under Renode (as mcu_run.py --renode does),
# send one identify and print the raw bytes received.
#   renode_probe.py <elf> <platform.resc>
import os
import select
import sys
import time

sys.argv += ["--kalico", os.path.expanduser("~/rk/kalico")]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mcu_run  # noqa: E402

mcu = mcu_run.Mcu(sys.argv[1], None, sys.argv[2])
try:
    mcu.send(mcu.parser.create_command("identify offset=0 count=40"))
    end = time.time() + 5
    data = b""
    while time.time() < end:
        r, _, _ = select.select([mcu.fd], [], [], 0.2)
        if r:
            data += os.read(mcu.fd, 4096)
    print("received %d bytes: %s" % (len(data), data[:200].hex()))
finally:
    print(mcu.close()[-2000:])
