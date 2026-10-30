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
| `src/logging.rs` | python `logging` (root, basicConfig format) | used by the checks above (stderr compared) | pass; tracebacks = one placeholder frame (masked by refdiff) + python's last line, chained exceptions included |
| `src/mathutil.rs` | `klippy/mathutil.py` + numpy helpers (np_std/mean/solve/lstsq/interp/i0/kaiser) | `tools/gen_numpy_cases.py`, `gen_angle_cases.py`, `gen_fft_cases.py` | bit-identical (lstsq: ill-conditioned systems excepted, NOTES.md) |
| `src/numpy_fft.rs` | numpy 2.2 pocketfft rfft (power-of-two lengths) | `tools/gen_fft_cases.py` | bit-identical |
| `src/scipy_signal.rs` | scipy 1.14.1 butter/iirnotch/tf2sos/sosfilt_zi | `tools/gen_sos_cases.py` (fixed point values) | 3000/3000 identical |
| `src/greenlet.rs` | python `greenlet` (subset) on corosensei: symmetric switch via trampoline, kill = unwind | unit tests (switching, kill unwinds, 600 KB deep stack); `tools/cross_test.sh` runs them under qemu: armv7, aarch64, armv6 | pass on all (armv6: corosensei asm warns about D16-D31, see NOTES) |
| `src/reactor.rs` | `klippy/reactor.py` (SelectReactor logic + PollReactor fds) | `tools/reactor_cmp.sh`: 10 scenarios (timers, nested pause, re-pause, completion, mutex, register_callback, async thread callback, assert_no_pause, sys pause, delay_for_ms), event order + greenlet count, 3 runs; also armv7 under qemu | identical |
| `src/chelper/pyhelper.rs` | `chelper/pyhelper.c` (all) | used by reactor (get_monotonic) | dump_string / fill_time untested until serialqueue |
| `src/util.rs` | `klippy/util.py` (all) | reactor_cmp (set_nonblock); rest used from printer.py on | get_git_version = tracked KALICO_VERSION, branch/remote/url "?"; dump_mcu_build looks one dir above the binary |
| `src/queuelogger.rs` | `klippy/queuelogger.py` + py3.13 `TimedRotatingFileHandler` | `tools/ql_cmp.sh`: computeRollover in 3 time zones across DST (90 cases); unit test: rotation names + backupCount deletion | identical; tracebacks of logging.exception not written |
| (none) | `klippy/compat.py` | n/a: python import-hook aliases for legacy plugins; a compiled binary has no imports (static registry instead) | n/a |
| `src/error.rs` | (python exception classes) | used everywhere | KError carries the python class (`Py`, `CommandSub`) and `__context__` chains (`Chained`) |
| `src/printer.rs` | `klippy/printer.py` (Printer, main, options, registry) | stock suite + custom cases | identical |
| `src/gcode.rs` | `klippy/gcode.py` | stock suite + custom cases | identical |
| `src/webhooks.rs` | `klippy/webhooks.py` | via milestone runs + a socket test (todo) | built, NOT verified |
| `src/extras/mod.rs` | (pkgutil scan of extras/) | registry order == code point order (checked) | every module ported (see below) |
| `src/chelper/pollreactor.rs` | `chelper/pollreactor.c` | via serialqueue checks | pass |
| `src/chelper/serialqueue.rs` | `chelper/serialqueue.c` (uart, can, debugfile modes; RK_SQ_TRACE trace) | `tools/sq_cmp.sh`: scripted pty exchange vs Kalico's C serialqueue (framing, seq, acks, data + notify delivery, cross-queue priority, timeout + nak retransmit, resync after garbage, stats counters, extract_old, pull after exit), 3 runs | identical; clock-est scheduling (min_clock stalls) only checked through later batch traces; CAN path untested |

Not ported on purpose: `MessageParser.encode_msgblock` (unused in Kalico, python result is not a valid block).
Known gaps: `process_identify` error *texts* for corrupt zlib/json data differ (python exception strings).
configfile: jinja2 error texts reproduced only for undefined names (include templates); jinja `do` extension not
enabled; not wired yet: get_printer(), SAVE_CONFIG gcode command, lookup_objects (wait for printer.py / gcode.py).
Run everything: `tools/check_all.sh` (~10 min).

## Stock test cases (`test/klippy/*.test`, 88 runs)

`tools/suite_cmp.sh`: 87/88 identical (2026-10-30). printers differs by wall-clock timer timing only (NOTES.md
"Parity harness").

## Python macros (`src/extras/gcode_python.rs`, `src/pysyntax.rs`, `vendor/rustpython-*`)

Stock gcode_python and macro_loading identical; custom pymacro, pymacro_misc, pymacro_bad identical (helpers,
status access, own_vars, in-place mutation of variables, error messages, syntax errors at config load).
`tools/gen_pysyntax_cases.py` (CPython 3.13 vs kaliru): SyntaxErrors 313/315 (no `match` statement in RustPython
0.4), runtime exception messages `--exec` 119/125 (NOTES.md "python macros" lists the rest).

## MCU firmware (`firmware/`, kaliru-mcu)

Linux board (Kalico `src/linux` + generic core): `compile_time_request.txt` identical to the C build's,
data dictionary byte-identical apart from `version` / `build_versions` (`firmware/tests/ref/`). Behaviour:
`firmware/tests/mcu_cmp.sh`, 7 scripts against the C `klipper.elf` under the `fakehw.c` shim (responses +
gpio/spi/i2c/adc event logs): all identical (basecmd, config + move_count, error paths, steppers + homing, gpio,
buses + sensors, drivers + load cell probe).

Micro-controller boards: stm32 (all families, AT32F403, N32G45x), rp2040 / rp2350 (incl. can2040), lpc176x,
atsam (sam3x, sam4s, sam4e, same70), atsamd (samd21, samc21, samd51, same51, same54), hc32f460, avr (atmega2560 / 1280 / 1284p / 644p / 328p / 328 / 168,
at90usb1286 / 646, atmega32u4: PROGMEM tables, rustc's avr-none target), the simulator, and the INDX
heater (Kalico's C++ `src/indx`) with newlib's powf / sqrtf / sin ported (`src/libm.rs`). `firmware/tests/ctr_check.sh`
builds every config in `firmware/configs/` (64: serial, USB, CAN / FDCAN, USB-to-CAN bridge, angle sensors, INDX
toolboard) and compares `compile_time_request.txt` (identical) and the data dictionary (byte-identical apart from the
version fields) with the C build's (`firmware/tests/ref/`): 64/64. The rp2040 boot2 block and `tools/elf2uf2` output
are byte-identical to the C build's; `src/libm.rs` gives newlib's bits on 700k inputs (qemu-arm). Emulator
(`firmware/tests/emu_cmp.sh`, Unicorn, deterministic board models): every config but linux (checked by
`mcu_cmp.sh`) and the simulator, 52 of the 54 ARM configs (62 with the AVR ones below), runs the 7 scripts with identical answers and identical peripheral
register accesses: every serial config (16), every USB config (20: stm32 usbfs / otg, rp2040, rp2350, samd, lpc176x,
sam3x / sam4 / same70, with a usb host that enumerates the device), the INDX toolboard, the CAN configs (9: bxcan,
stm32 fdcan, atsam / atsamd M_CAN, lpc176x, and can2040 on the rp2040 PIO, run instruction by instruction against a
bit-level CAN bus, with a can host that assigns the node id) and the USB-to-CAN bridges (6: gs_usb on the usb side),
apart from the documented C-side differences in `firmware/tests/emu_known/` (non-volatile register pointers gcc
reorders, a volatile read gcc moves past a call, an HC32 neopixel store gcc merges away, an F072 stack overflow,
uninitialised CAN frame bytes the bridge sends). Real chips: not run. Images (profile `mcu`): about 1.05-1.15x the C size (F103 43.4KB vs 37.8KB, rp2040 44.2KB vs
42.4KB); AVR (opt-level z) 1.07-1.09x (atmega168 16.3KB vs 15.3KB, in its 16KB flash).

AVR (`firmware/tests/emu_avr.py`, simavr's core with the same kind of deterministic model: timer1, usart, the
at90usb usb controller with the usb host): all 10 AVR configs give 7/7 identical scripts (it found
neopixel's AVR delay evaluating an argument C's macro drops, and C's jmp_buf in .bss, which sets the move_count).

Not ported: ar100 (or1k), pru (no Rust target). Kalico's `src/extras` (user firmware plugins found by
`scripts/find-firmware-extras.sh`) is empty in Kalico itself.

## Custom cases (`tests/cases/*.test`, `tools/case_cmp.sh`)

64 runs, all identical (2026-10-30), for modules and error paths the stock suite misses: accelerometers (accels), angle, chained
exception tracebacks (chains, parse_errors), canbus_ids, digipots, eddy current probes, fan, heater_bed, load cell
probes (lcp, lcp_bad; filter designs in `tests/cases/scipy/` against a scipy venv, `REF_KLIPPY`), misc extras,
nozzle_cleanup, palette2, python macros (pymacro*), pwm_clock, replicape, i2c/spi temperature sensors (sensors, sensor_status), resonance
tester / shaper_calibrate commands (shaper, shaper_cmds), smart_effector, tools_calibrate, skew/transforms.

Function level checks against python for code file-output runs cannot reach: shaper_calibrate
(`tools/gen_shaper_cases.py`: psds, fits, csv bit-identical), angle calibration, sos filter design.

**Example configs** (`tests/cases/configs/examples.test`, written by `tools/gen_config_cases.py`): every
config/*.cfg that printers.test does not load (186), with the mcu dictionary its header names (LPC176x, SAME54,
HC32 dictionaries copied from the firmware builds into `~/rk/ci_build/dict`), wrappers adding a virtual_sdcard path
/ tmc sense_resistor where the example leaves them out, running move.gcode: 186/186 identical (2026-10-31). Not
loaded: 15 sample fragments and BeagleBone PRU configs (no dictionary).

**Status probe** (`tests/plugins/kaliru_test_status.py`, `tests/cases/plugins/status.test`): `get_status()` of
every printer object (what Moonraker queries) on the api probe's 45 configs: identical but for idle_timeout's
`printing_time` (wall clock).

Fixed in the native sweep (2026-10-31): chelper errors log as Kalico's `logging.error(FFI_main.string(msg))` (the
bytes repr) once klippy starts; the flush handlers, telemetry, mcu reconnect and serial identify log the exception
with its traceback (`logging.exception`); a deep nested-macro chain raises RecursionError (an internal error, as in
Kalico) instead of overflowing kaliru's fixed greenlet stack; `rpi_usb` mcu restarts (hub-ctrl).

Not verified against Kalico: paths that need an mcu query answer (i2c/spi sensor reads, accelerometer and load cell
data, eddy homing, tap analysis) - python hangs on queries in file-output mode. Ported from source.

## Kalico pytest suite (`test/test_*.py`)

Reference: 194 passed, 2 xfailed (incl. the 67 `test/klippy/*.test` printer runs, compared by `tools/suite_cmp.sh`).
The unit tests poke python internals with fakes; kaliru checks the same behaviour end to end against the python host
(file-output cases, `tools/case_cmp.sh`; live runs with linux MCU processes, `tools/live_cmp.sh`) or with Rust unit
tests:

| pytest file | kaliru check |
|---|---|
| test_autosave | `tools/save_cmp.sh` (test_configs/autosave, SAVE_CONFIG with includes) |
| test_buildcommands_kconfig | `firmware/tests/ctr_check.sh`: the data dictionary's `kconfig` (64 configs, atmega2560's empty) |
| test_canbus_stats | reviewed: query_event keeps the stats on a serial / command error (no CAN on the linux MCU) |
| test_configfile_include_vars | `tools/config_cmp.sh` (test_configs/include_vars*) |
| test_extruder_overrides_simple | `tests/cases/extruder_smooth*.test` |
| test_gcode_shell_command | stock `gcode_shell_command.test` (PARAMS with `$HOME ~/x`) |
| test_imports | python imports: n/a |
| test_kconfig_defconfig_roundtrip | kconfiglib (menuconfig tooling, a vendored python library): not ported, configs are inputs |
| test_mcu_endstop, test_neopixel | `tests/live/noncritical.live` (endstop and leds on a disconnected toolboard, its reconnect) |
| test_mcu_recon | `tests/live/noncritical.live`, `tmc_noncritical.live` (disconnect, reconnect), `recon_shutdown.live` (an MCU in shutdown at reconnect: reset, retry) |
| test_msgproto_kconfig | `src/msgproto.rs` unit tests |
| test_pid_profile | `tests/cases/pid_profile_ext.test` (dual loop inner gains, watermark, mpc) |
| test_spi_temperature_clarify | `tests/live/thermocouple.live` (fault bits; the out-of-range path needs other fake readings) |
| test_tmc_homing | stock `tmc.test`, `tests/cases/tmc_homing.test` (TCOOLTHRS, home current, its maximum) |
| test_tmc_noncritical | `tests/live/tmc_noncritical.live`, `tmc_fault.live` |
| test_tools_calibrate | reviewed: every Kalico kinematics reports its axis limits, so the error is unreachable from a config |

Found this way: kaliru hung on a query to a non-critical MCU after its reconnect (a serialqueue's command queues kept
the messages queued before the disconnect: C frees them in `serialqueue_free`), and a `%` format error in PID_PROFILE
lost its python class (TypeError).

## Firmware (data dictionary / responses)

Not started. Reference dicts: `/ci_build/dict/*.dict` (40, built by Kalico's `scripts/ci-build.sh`).
