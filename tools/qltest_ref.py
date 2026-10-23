#!/usr/bin/env python3
# Reference twin of kaliru's src/bin/qltest.rs: computeRollover() of python's
# TimedRotatingFileHandler as Kalico's queuelogger configures it.
# usage: qltest_ref.py <dir> <timestamp>...
import logging.handlers
import sys

log = sys.argv[1] + "/ql.log"
for name, kw in [("midnight", dict(when="midnight", backupCount=5)),
                 ("S", dict(when="S", interval=60 * 60 * 24, backupCount=5))]:
    h = logging.handlers.TimedRotatingFileHandler(log, **kw)
    for t in sys.argv[2:]:
        print("%s %s %d" % (name, t, h.computeRollover(int(t))))
    h.close()
