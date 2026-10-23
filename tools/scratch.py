import sys
sys.path.insert(0, ".")
from klippy import msgproto
mp = msgproto.MessageParser()
mp.process_identify(open(sys.argv[1], "rb").read(), decompress=False)
for line in open(sys.argv[2]):
    parts = line.split()
    if len(parts) < 6 or not parts[0] == "serialq":
        continue
    data = bytes.fromhex(parts[5])
    msg = bytearray([len(data) + 5, 0x10]) + data + b"\0\0\x7e"
    out = mp.dump(msg)[1:]
    if any(" oid=15 " in o + " " or "oid=15" in o.split()[1:2] for o in out):
        print(parts[2], parts[3], parts[4], out)
