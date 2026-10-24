# Vendored crates

`rustpython-vm` and `rustpython-stdlib` 0.4.0 from crates.io, patched so python macros raise CPython 3.13's
exception messages (Kalico runs on CPython; its logs and console replies carry those texts). The first commit
adding each crate is the pristine crates.io copy; `git log -p vendor/` shows every change since.
Checked by `tools/gen_pysyntax_cases.py --exec`.

`corosensei` 0.3.4 (the greenlet coroutines), patched: its ARM context switch declares the callee-saved VFP
registers as d8-d15 instead of q4-q7, which only exist with NEON; on armv7 without NEON the switch lost the
floats held in d10-d15 (the reactor computed a negative poll timeout and hung). Pristine copy: the commit
adding it; the patch: `git log -p vendor/corosensei`.
