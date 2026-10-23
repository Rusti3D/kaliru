# Vendored crates

`rustpython-vm` and `rustpython-stdlib` 0.4.0 from crates.io, patched so python macros raise CPython 3.13's
exception messages (Kalico runs on CPython; its logs and console replies carry those texts). The first commit
adding each crate is the pristine crates.io copy; `git log -p vendor/` shows every change since.
Checked by `tools/gen_pysyntax_cases.py --exec`.
