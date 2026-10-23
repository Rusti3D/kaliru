# Parity status

Reference: Kalico `5b59427` in WSL `~/rk/kalico` (+ `tools/ref_trace.patch`), python 3.13 (uv venv). Reference outputs:
`~/rk/ref1` (PYTHONHASHSEED=1) and `~/rk/ref2` (=2); `refdiff.py ref1 ref2` = 88/88 identical (2026-10-23).
What must match per run: serialqueue trace, exit code, masked log (see `tools/refdiff.py` header for why the raw
`.serial` files are not compared).

## Modules

| Rust module | Python / C source | Check | Result |
|---|---|---|---|
| `src/msgproto.rs` | `klippy/msgproto.py` | `tools/parsedump_cmp.py --corrupt 10` (parsedump on all 91 ref outputs + 910 corrupted copies: stdout, exit, "Invalid data" lines) | 1001/1001 identical (2026-10-23) |
| | | `tools/trace_check.sh` (decode every traced message, Rust vs Kalico msgproto) | 88/88 traces, 277225 messages identical |
| | | unit tests: encode of 23 edge values, create_command 11 cases, lookup_output_params quirks (expected values from python) | pass |
| `src/chelper/msgblock.rs` | `chelper/msgblock.c` | unit tests (vlq round trip, resync); crc shared with msgproto checks above | pass; clock_* functions untested until serialqueue |

| `src/bin/parsedump.rs` | `klippy/parsedump.py` | parsedump_cmp.py | identical |
| `src/pyfmt.rs` | (python builtins: str/repr/int/float/strip, OSError text) | unit tests | pass |
| `src/pyval.rs` | (python objects: PyVal with str/repr/json.dumps) | unit tests; used by all configfile checks | pass |
| `src/configparser.rs` | python 3.13 `configparser.RawConfigParser` | via config_cmp.sh (below) + unit tests from python probes | pass |
| `src/pyglob.rs` | python 3.13 `glob`, `fnmatch`, `posixpath` | unit tests; include globs in config_cmp.sh | pass; exotic fnmatch sets ("a-c-e") approximated |
| `src/configfile.rs` | `klippy/configfile.py` | `tools/config_cmp.sh`: 307 configs (Kalico config/, test/klippy/, test/test_configs/, tools/config_cases/): sections, raw + interpolated values, tracking, every getter, autosave, log_config, get_status | 307/307 identical (2026-10-23) |
| | | `tools/save_cmp.sh`: 9 SAVE_CONFIG scenarios (set, remove_section, autosave_includes, include conflicts, backups); Kalico test_autosave assertions hold | 9/9 identical |
| `src/pins.rs` | `klippy/pins.py` | `tools/pins_cmp.sh`: 55 hand cases + 32349 ops from every pin option of the corpus | identical |
| `src/logging.rs` | python `logging` (root, basicConfig format) | used by the checks above (stderr compared) | pass; no tracebacks for logging.exception |
| `src/mathutil.rs` | `klippy/mathutil.py` (safe_float only) | config_cmp getfloatlist | rest ported with first user |
| `src/greenlet.rs` | python `greenlet` (subset) on corosensei: symmetric switch via trampoline, kill = unwind | unit tests (switching, kill unwinds, 600 KB deep stack); `tools/cross_test.sh` runs them under qemu: armv7, aarch64, armv6 | pass on all (armv6: corosensei asm warns about D16-D31, see NOTES) |
| `src/reactor.rs` | `klippy/reactor.py` (SelectReactor logic + PollReactor fds) | `tools/reactor_cmp.sh`: 10 scenarios (timers, nested pause, re-pause, completion, mutex, register_callback, async thread callback, assert_no_pause, sys pause, delay_for_ms), event order + greenlet count, 3 runs; also armv7 under qemu | identical |
| `src/chelper/pyhelper.rs` | `chelper/pyhelper.c` (all) | used by reactor (get_monotonic) | dump_string / fill_time untested until serialqueue |
| `src/util.rs` | `klippy/util.py` (all) | reactor_cmp (set_nonblock); rest used from printer.py on | get_git_version = tracked KALICO_VERSION, branch/remote/url "?"; dump_mcu_build looks one dir above the binary |
| `src/queuelogger.rs` | `klippy/queuelogger.py` + py3.13 `TimedRotatingFileHandler` | `tools/ql_cmp.sh`: computeRollover in 3 time zones across DST (90 cases); unit test: rotation names + backupCount deletion | identical; tracebacks of logging.exception not written |
| (none) | `klippy/compat.py` | n/a: python import-hook aliases for legacy plugins; a compiled binary has no imports (static registry instead) | n/a |
| `src/error.rs` | (python exception classes) | used everywhere | KError lacks the python class name needed for traceback last lines (next step) |
| `src/printer.rs` | `klippy/printer.py` (Printer, main, options, registry) | smoke: arg errors + config error runs match python (message, exit 2 / 255); full check = milestone traces | built, NOT parity-verified yet |
| `src/gcode.rs` | `klippy/gcode.py` | via milestone runs | built, NOT verified; request_restart ready branch waits for toolhead |
| `src/webhooks.rs` | `klippy/webhooks.py` | via milestone runs + a socket test (todo) | built, NOT verified |
| `src/extras/mod.rs` | (pkgutil scan of extras/) | registry order == code point order (checked) | 143 modules, 1 ported |
| `src/extras/danger_options.rs` | `extras/danger_options.py` | via milestone | built |
| `src/extras/probe.rs` | `extras/probe.py` (ProbeList only) | via milestone | add_probe_object + PrinterProbe todo |
| `src/chelper/pollreactor.rs` | `chelper/pollreactor.c` | via serialqueue checks | pass |
| `src/chelper/serialqueue.rs` | `chelper/serialqueue.c` (uart, can, debugfile modes; RK_SQ_TRACE trace) | `tools/sq_cmp.sh`: scripted pty exchange vs Kalico's C serialqueue (framing, seq, acks, data + notify delivery, cross-queue priority, timeout + nak retransmit, resync after garbage, stats counters, extract_old, pull after exit), 3 runs | identical; clock-est scheduling (min_clock stalls) only checked through later batch traces; CAN path untested |

Not ported on purpose: `MessageParser.encode_msgblock` (unused in Kalico, python result is not a valid block).
Known gaps: `process_identify` error *texts* for corrupt zlib/json data differ (python exception strings).
configfile: jinja2 error texts reproduced only for undefined names (include templates); jinja `do` extension not
enabled; not wired yet: get_printer(), SAVE_CONFIG gcode command, lookup_objects (wait for printer.py / gcode.py).
Run everything: `tools/check_all.sh` (~10 min).

## Stock test cases (`test/klippy/*.test`, 88 runs)

None run against Rust yet (no klippy binary).

## Kalico pytest suite (`test/test_*.py`)

Reference: 194 passed, 2 xfailed. Rust equivalents: not started.

## Firmware (data dictionary / responses)

Not started. Reference dicts: `/ci_build/dict/*.dict` (40, built by Kalico's `scripts/ci-build.sh`).
