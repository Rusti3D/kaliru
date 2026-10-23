#!/usr/bin/env python3
# Reference twin of kaliru's src/bin/sqtest.rs: a scripted, synchronous
# exchange between serialqueue (Kalico's C chelper via cffi) and a fake mcu
# on a pty. Every step waits for the previous one, so the bytes are
# deterministic; timing values (srtt, rto, times) are not printed.
# Run with PYTHONPATH=~/rk/kalico from ~/rk/kalico.
import os
import select
import time
import tty

from klippy import chelper

ffi, lib = chelper.get_ffi()


def crc16(buf):
    crc = 0xFFFF
    for data in buf:
        data ^= crc & 0xFF
        data ^= (data & 0x0F) << 4
        crc = ((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3)
    return crc & 0xFFFF


def block(seq, payload):
    msg = [len(payload) + 5, 0x10 | (seq & 0x0F)] + list(payload)
    c = crc16(msg)
    return bytes(msg + [c >> 8, c & 0xFF, 0x7E])


def mcu_read(fd, first_timeout=2.0):
    # wait for data, then collect the rest of the burst
    out = b""
    if not select.select([fd], [], [], first_timeout)[0]:
        return out
    while select.select([fd], [], [], 0.02)[0]:
        out += os.read(fd, 4096)
    return out


def pull(sq):
    time.sleep(0.05)
    pqm = ffi.new("struct pull_queue_message *")
    lib.serialqueue_pull(sq, pqm)
    return "len=%d notify=%d msg=%s" % (pqm.len, pqm.notify_id,
                                        bytes(pqm.msg[0:max(pqm.len, 0)]).hex())


def stats(sq):
    buf = ffi.new("char[4096]")
    lib.serialqueue_get_stats(sq, buf, 4096)
    s = ffi.string(buf).decode()
    return " ".join(p for p in s.split() if p.split("=")[0] not in ("srtt", "rttvar", "rto"))


def main():
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    out = []
    emit = lambda s: print(s, flush=True)
    sq = lib.serialqueue_alloc(slave, b"u", 0, b"serialq mcu")
    lib.serialqueue_set_wire_frequency(sq, 250000.0)
    lib.serialqueue_set_receive_window(sq, 192)
    cq = lib.serialqueue_alloc_commandqueue()
    cq2 = lib.serialqueue_alloc_commandqueue()
    # est_freq 0: messages go out as soon as they are queued
    lib.serialqueue_set_clock_est(sq, 0.0, 0.0, 0, 0)

    def send(q, data, notify=0, minc=0, reqc=0):
        lib.serialqueue_send(sq, q, bytes(data), len(data), minc, reqc, notify)

    send(cq, [1, 2, 3])
    emit("write1 " + mcu_read(master).hex())
    os.write(master, block(2, []))  # ack seq 2
    time.sleep(0.05)
    send(cq, [4, 5], notify=7)
    emit("write2 " + mcu_read(master).hex())
    os.write(master, block(3, [9, 9, 9]))  # data message acking block 2
    emit("pull1 " + pull(sq))
    os.write(master, block(3, []))  # plain ack: notifies block 2's message
    emit("pull2 " + pull(sq))
    # two queues, both ready: one block, lowest req_clock first
    send(cq2, [0x20, 1], reqc=5)
    send(cq, [0x21, 1], reqc=3)
    emit("write3 " + mcu_read(master).hex())
    # no ack: wait for the timeout retransmit (sync byte + pending blocks)
    emit("retransmit " + mcu_read(master).hex())
    os.write(master, block(4, []))
    time.sleep(0.05)
    send(cq, [0x30])
    emit("write4 " + mcu_read(master).hex())
    os.write(master, block(4, []))  # duplicate ack = nak -> fast retransmit
    emit("nak retransmit " + mcu_read(master).hex())
    os.write(master, block(5, []))
    time.sleep(0.05)
    # garbage ending in a sync byte, then a data block
    os.write(master, b"\x01\x02garbage\x7e" + block(5, [1, 2]))
    emit("pull3 " + pull(sq))
    emit("stats " + stats(sq))
    for sentq in (1, 0):
        q = ffi.new("struct pull_queue_message[1024]")
        n = lib.serialqueue_extract_old(sq, sentq, q, 1024)
        emit("old%d %s" % (sentq, " ".join(
            bytes(q[i].msg[0:q[i].len]).hex() for i in range(n))))
    lib.serialqueue_exit(sq)
    emit("pull after exit " + pull(sq))
    lib.serialqueue_free(sq)
    for line in []:
        print(line)


main()
