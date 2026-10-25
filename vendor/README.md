# Vendored crates

`corosensei` 0.3.4 (the greenlet coroutines), patched: its ARM context switch declares the callee-saved VFP
registers as d8-d15 instead of q4-q7, which only exist with NEON; on armv7 without NEON the switch lost the
floats held in d10-d15 (the reactor computed a negative poll timeout and hung). Pristine copy: the commit
adding it; the patch: `git log -p vendor/corosensei`.
