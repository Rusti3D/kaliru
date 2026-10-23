#!/usr/bin/env python3
# Reference twin of kaliru's src/bin/reactortest.rs: reactor scenarios on
# Kalico's reactor, printing the order of events (not times). Deadlines are
# 10+ ms apart so the order is deterministic.
# Run with PYTHONPATH=~/rk/kalico.
import threading
import time

from klippy import reactor as R


def run(name, setup):
    r = R.Reactor(gc_checking=True)
    log = []
    setup(r, log)
    r.run()
    log.append("greenlets=%d" % len(r._all_greenlets))
    r.finalize()
    print("%s: %s" % (name, " | ".join(log)))


def at(r, t, fn):
    def cb(e):
        fn()
        return r.NEVER
    r.register_timer(cb, t)


def timers(r, log):
    t0 = r.monotonic()

    def mk(name, n, step):
        cnt = [0]

        def cb(e):
            cnt[0] += 1
            log.append("%s%d" % (name, cnt[0]))
            if cnt[0] < n:
                return e + step
            if name == "c":
                r.end()
            return r.NEVER
        return cb
    r.register_timer(mk("a", 1, 0), t0 + 0.06)
    r.register_timer(mk("b", 3, 0.025), t0 + 0.02)
    r.register_timer(mk("c", 1, 0), t0 + 0.10)


def nested(r, log):
    t0 = r.monotonic()

    def a():
        log.append("A1")
        r.pause(t0 + 0.08)
        log.append("A2")

    def b():
        log.append("B1")
        r.pause(t0 + 0.05)
        log.append("B2")
    at(r, t0 + 0.01, a)
    at(r, t0 + 0.03, b)
    at(r, t0 + 0.11, lambda: (log.append("C"), r.end()))


def repause(r, log):
    t0 = r.monotonic()

    def a():
        log.append("A1")
        r.pause(t0 + 0.03)
        log.append("A2")
        r.pause(t0 + 0.06)
        log.append("A3")
    at(r, t0 + 0.01, a)
    at(r, t0 + 0.045, lambda: log.append("B"))
    at(r, t0 + 0.09, lambda: (log.append("end"), r.end()))


def completion(r, log):
    t0 = r.monotonic()
    comp = r.completion()

    def w():
        log.append("W1")
        log.append("W got %s" % (comp.wait(),))

    def w2():
        log.append("W2 got %s" % (comp.wait(t0 + 0.03, "timeout"),))
    at(r, t0 + 0.01, w)
    at(r, t0 + 0.015, w2)
    at(r, t0 + 0.05, lambda: (log.append("X"), comp.complete("done")))
    at(r, t0 + 0.08, lambda: (log.append("test=%s" % comp.test()), r.end()))


def mutex(r, log):
    t0 = r.monotonic()
    m = r.mutex()

    def m1():
        m.lock()
        log.append("M1 locked")
        r.pause(t0 + 0.05)
        log.append("M1 unlock")
        m.unlock()

    def mk(name):
        def fn():
            log.append("%s wait" % name)
            m.lock()
            log.append("%s locked test=%s" % (name, m.test()))
            m.unlock()
        return fn
    at(r, t0 + 0.01, m1)
    at(r, t0 + 0.02, mk("M2"))
    at(r, t0 + 0.03, mk("M3"))
    at(r, t0 + 0.1, lambda: (log.append("end test=%s" % m.test()), r.end()))


def callback(r, log):
    t0 = r.monotonic()
    c = r.register_callback(lambda e: (log.append("cb"), "cb-result")[1])
    at(r, t0 + 0.02, lambda: log.append("got %s" % (c.wait(),)))
    at(r, t0 + 0.05, lambda: (log.append("end"), r.end()))


def async_cb(r, log):
    t0 = r.monotonic()

    def thread():
        time.sleep(0.02)
        r.register_async_callback(lambda e: log.append("async"))
    threading.Thread(target=thread).start()
    at(r, t0 + 0.08, lambda: (log.append("end"), r.end()))


def no_pause(r, log):
    t0 = r.monotonic()

    def p():
        with r.assert_no_pause():
            try:
                r.pause(r.monotonic() + 0.01)
                log.append("paused")
            except R.ReactorError as e:
                log.append("error: %s" % (e,))
        r.end()
    at(r, t0 + 0.01, p)


def sys_pause(r, log):
    t = r.monotonic()
    e = r.pause(t + 0.02)
    log.append("sys pause ok=%s" % (e >= t + 0.02,))
    at(r, r.NOW, r.end)


def delay(r, log):
    t0 = r.monotonic()

    def d():
        log.append("D1")
        r.delay_for_ms(30)
        log.append("D2")
    at(r, t0 + 0.01, d)
    at(r, t0 + 0.02, lambda: log.append("E"))
    at(r, t0 + 0.07, r.end)


for name, fn in [("timers", timers), ("nested", nested), ("repause", repause),
                 ("completion", completion), ("mutex", mutex),
                 ("callback", callback), ("async", async_cb),
                 ("no_pause", no_pause), ("sys_pause", sys_pause),
                 ("delay", delay)]:
    run(name, fn)
