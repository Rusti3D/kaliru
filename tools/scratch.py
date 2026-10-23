import sys, glob, json, os
sys.path.insert(0, ".")
from klippy import msgproto
d = glob.glob(os.path.expanduser("~/rk/kalico/test/klippy/*.dict")) + glob.glob(os.path.expanduser("~/rk/dict/*.dict"))
mp = msgproto.MessageParser()
for f in d:
    if "atmega2560" in f:
        mp.process_identify(open(f, "rb").read(), decompress=False)
        break
for h in sys.argv[1:]:
    data = bytes.fromhex(h)
    msg = bytearray([len(data) + 5, 0x10]) + data + b"\0\0\x7e"
    print(h, mp.dump(msg))
