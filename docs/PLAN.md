# Kaliru port plan

**Kaliru** (Kalico in Rust): port of **Kalico** (Klipper fork, what the Centauri Carbon's COSMOS image runs) to Rust: host software (klippy)
and MCU firmware, full parity, byte-identical outputs. Updated 2026-10-23.

- Reference: `Originals_Base/kalico` = KalicoCrew/kalico `main` @ `5b59427` (2026-10-21). Read-only.
- Later layer: COSMOS changes (pinned to Kalico `26e46bd`, 58 commits behind our reference): 12 patches + 2 extras,
  ported after upstream parity (see "COSMOS layer").
- `Originals_Base/klipper` (upstream Klipper @ `ee2c6fb6b`) stays as a secondary reference: Kalico tracks it, so
  when a Kalico file equals Klipper's, Klipper's history/docs explain it.

## Scope (everything in Rust — user, 2026-10-23)

| Part | Size | Plan |
|---|---|---|
| `klippy/*.py` core (printer, reactor, mcu, serialhdl, msgproto, clocksync, configfile, gcode, toolhead, stepper, webhooks, pins, util, mathutil, queuelogger, compat, console, parsedump) | ~8.9k lines | port |
| `klippy/kinematics/` (16 incl. limited_cartesian / corexy / corexz, extruder, idex_modes) | ~3.1k | port |
| `klippy/extras/` (144 modules; folders `display/`, `indx/`, `load_cell/`) | ~50.7k | port, all of them |
| `klippy/plugins/` (user drop-in Python modules, loaded like extras) | — | see open question: Python plugins can't load into a Rust binary |
| `klippy/chelper/*.c` (serialqueue, stepcompress, steppersync, itersolve, trapq, trdispatch, msgblock, pollreactor, kin_*.c) | ~4.8k C incl. headers | port (no cffi / gcc at runtime, one static binary) |
| MCU firmware common `src/*.c`, `src/generic/` | ~10.7k C | port |
| MCU firmware platforms `src/<platform>/`: stm32 7.2k, atsam 2.9k, atsamd 2.7k, rp2040 2.1k, linux 1.9k, lpc176x 1.6k, avr 1.5k, pru 1.1k, hc32f460 0.9k, ar100 0.6k, indx 0.4k, simulator 0.2k | ~23k C | port, platform by platform |
| Firmware build: `Makefile`, Kconfig (`lib/kconfiglib`, Python), `scripts/buildcommands.py` (dictionary + command tables) | — | port: Kconfig options -> cargo features / config file, Rust build step emitting the same `klipper.dict` |
| `lib/` vendor code (CMSIS, pico-sdk, can2040, fatfs, chip headers) | — | Rust register crates (PACs) where they exist; anything missing gets written (user: "build anything that's missing") |
| `scripts/` (calibrate_shaper, graphs, flash tools, canbus_query, test_klippy) | — | after host + firmware; flash tools needed for installs come first |

Python libraries and replacements: `greenlet` -> stackful coroutines (see Reactor), `cffi` -> gone,
`pyserial` -> termios via `libc`, `python-can` -> SocketCAN via `libc`/`socket2`, `Jinja2`+`markupsafe` -> `minijinja`
(as Moonruster), **`numpy`** (webhooks JSON of numpy types, angle, shaper_calibrate, temperature_fan, z_tilt_ng,
load_cell/tap_analysis) -> plain Rust math; matching numpy's float results (summation order, linalg) is a test item.

## Parity method

1. **Batch mode serial output (primary).** `klippy cfg -i in.gcode -o out.serial -d mcu.dict [-d name=x.dict]`
   writes the exact bytes each MCU would receive, one file per MCU (`out.serial`, `out.serial-<name>`). Rust must
   produce byte-identical files; `parsedump` makes readable diffs. Batch mode stubs `estimated_print_time` to 0 and
   uses a fixed clock estimate, so output should not depend on wall time — the harness confirms this (run twice,
   different `PYTHONHASHSEED`).
2. **G-code responses and exit status** for the same input, incl. `SHOULD_FAIL` cases.
3. **Log text** with timestamps/pids masked.
4. **Webhooks API** (unix socket `-a`): same JSON for every endpoint and subscription (the Moonraker-facing
   contract). Diff-test style as Moonruster.
5. **Kalico's pytest suite** (`test/test_*.py`, `test/klippy_testing/`): unit-level tests that import klippy
   modules (autosave, include vars, mcu recon, tmc homing, buildcommands/kconfig...). Each becomes an equivalent
   Rust test or a black-box case with the same inputs/expected outputs.
6. **Firmware.** A binary can't be byte-identical, so: (a) identical generated data dictionary for the same Kconfig
   (commands, responses, ids, enumerations, constants; build-version strings excepted), (b) same host byte stream
   -> same response bytes and same pin/step activity. Check on the `linux` and `simulator` targets on a PC first,
   then real boards (step counts / logic analyser).
7. **Real-time behaviour** (clock sync, retransmits, timers, homing/trsync, non-critical MCU reconnect): run both
   hosts against the `linux` MCU process / simulavr and compare MCU-side logs; then the Carbon.

Test inputs: Kalico's 67 `test/klippy/*.test` cases (`printers.test`: 22 configs; 27 MCU dicts in total) + the pytest
suite + our own cfg+gcode cases for every module the stock tests don't cover (`kaliru/tests/cases/`).
A module is "done" only when every case touching it is identical.

Known parity risks:
- **Float arithmetic.** chelper is built `gcc -O2 -mfpmath=sse -msse2`; on x86_64 that is plain IEEE double, which
  Rust matches if operation order is kept. On ARM gcc may fuse multiply-adds (`-ffp-contract=fast` default), so
  references are made on x86_64; port expressions in the same order, never `mul_add`.
- **Python formatting** (`%.3f`, `repr(float)`, `str()` of containers): reuse `Moonruster/src/pyfmt.rs`.
- **Config parsing** (`configparser` + Kalico's includes with variables, `[constants]`, autosave): start from
  `Moonruster/src/configparser.rs`.
- **Ordering**: dicts keep insertion order (`IndexMap`); watch `set` iteration and `sorted()`.
- **Jinja**: minijinja vs jinja2 output differences in gcode_macro.
- **numpy** results (see above).

## Architecture decisions

- **Reactor.** klippy runs callbacks in greenlets; `reactor.pause()` / `completion.wait()` suspend mid-call-stack.
  Recommended: stackful coroutines (`corosensei`; confirm it builds and runs on armv7 first) so code keeps the
  Python shape for line-by-line review. Alternative: async/await everywhere (large structural divergence).
  Single thread, `Rc<RefCell<>>`, like Moonruster.
- **Background threads.** serialqueue's C thread (pollreactor + mutex) stays a thread in Rust, same structure.
- **Dynamic modules.** `printer.load_object` imports extras/plugins by section name -> static registry
  `name -> constructor` in `extras/mod.rs` (pattern: `Moonruster/src/components/mod.rs`).
- **Objects.** `lookup_object` -> `Rc<dyn PrinterObject>` with `as_any()` downcast, plus typed lookups for the
  common ones (toolhead, gcode, mcu).
- **Shared code with Moonruster.** Copy `pyfmt.rs` and `configparser.rs`; extract a shared crate only if a third
  project needs them.
- **Binary.** One static binary with Kalico's command line (`klippy.py` / `python -m klippy` options).

## Crate layout (`kaliru/`)

```
Cargo.toml
src/main.rs                 klippy.py / __main__ + printer.py
src/{reactor,mcu,serialhdl,msgproto,clocksync,configfile,gcode,toolhead,stepper,webhooks,pins,util,mathutil,queuelogger,compat}.rs
src/pyfmt.rs, src/configparser.rs   (from Moonruster)
src/chelper/*.rs
src/kinematics/*.rs
src/extras/mod.rs (registry) + one file per extra, folders like upstream (display/, indx/, load_cell/)
firmware/                   MCU firmware crate (no_std; common + src/<platform>/ like upstream; build.rs = buildcommands)
tests/cases/                our extra cfg+gcode parity cases
tools/                      harness: run reference + rust, diff
docs/PLAN.md, docs/PARITY.md (per-module status table, added with the harness)
```

## Order of work

Each step ends with its parity check passing; nothing moves on with a known diff.

1. **Harness**: reference outputs for all `test/klippy` cases (Kalico's runner `scripts/test_klippy.py`), run the
   pytest suite against the reference, determinism check, diff script. Needs the 27 MCU dicts.
2. **msgproto + msgblock + parsedump**: check Rust parsedump == Python parsedump on every reference `.serial` file.
3. **serialqueue (file mode) + reactor + util/queuelogger + configfile + pins + mcu/clocksync (file mode)**:
   minimal `[mcu]` + `kinematics: none` config, identical init bytes.
4. **Core motion**: gcode, webhooks, gcode_move, toolhead, trapq, itersolve, stepcompress, steppersync,
   motion_queuing, stepper, cartesian, extruder, heaters, stepper_enable + Kalico's default-on modules
   (force_move, respond, exclude_object). Check: `commands.test`, `out_of_bounds.test`, `printers.test`.
5. **Kinematics**: all 16 incl. limited_*, idex/dual_carriage, input shaper.
6. **Extras with stock tests**, one test file at a time.
7. **Remaining extras**: write a case per module, port, check.
8. **Webhooks API parity**, Moonruster + Mainsail/Fluidd against it.
9. **Real-time parity** (linux MCU / simulavr), then the Carbon (with the stock C HiFi4 firmware at first).
10. **Firmware** (can start after step 2): dictionary generation (identical `klipper.dict` for linux) -> common code
    on `linux` -> stm32 (F401 / F103: Carbon bed / toolhead / canvas) -> rp2040 / avr / rest -> HiFi4.
11. **COSMOS layer** (see below).
12. Phase 2: optimisation (memory first).

## COSMOS layer (after upstream Kalico parity)

Source: `D:\Carbon Centauri\cosmos\meta-opencentauri\recipes-apps\klipper` (`kalico_2026.02.00.inc` lists them).
12 patches (save-config subfile check, calibration tolerance, log rotate threshold, thermistor pulldown, probe home z
offset, forced input shaper, drv8833 motor control, config exclude, resonance tester max velocity, stm32 serial RS485
DE pin, **HiFi4 platform** (~9k lines incl. `lib/hifi4` HAL, rpmsg transport), HX711s load cell probe) + extras
`canvas_rfid.py`, `canvas_filament_sensor.py`. COSMOS also installs addons as extra Python modules: AFC,
dragonbreath, tmc-autotune (`recipes-data/klipper-*-addon`) — these need ports too to run the Carbon's stock config.
Main MCU on the Carbon is the HiFi4 Xtensa DSP: Rust has no upstream Xtensa target (Espressif's fork covers ESP32
cores only), so a Rust HiFi4 build may need a toolchain we build ourselves — check before porting.

## Open questions (user)

- **Python macros.** Kalico runs `gcode_macro` bodies whose lines start with `!` (or loaded from `.py` files) as real
  Python (`extras/gcode_macro.py` `TemplateWrapperPython`: `compile` + `exec`, helpers emit / wait_while / sleep /
  own_vars..., `docs/Command_Templates.md` "Python"; test `test/klippy/gcode_python.test`). Full parity needs a Python
  interpreter. Options: embed RustPython (a Python interpreter written in Rust, keeps the stack all-Rust, behind a
  cargo feature so the default binary stays small; CPython compatibility to be measured), or support a defined subset.
  **Decided (user 2026-10-23): RustPython behind a cargo feature, checked against CPython on the test macros.**
- **Python plugins / addons.** Kalico loads user Python modules from `klippy/plugins/` and extras dropped in by
  addons (AFC, tmc-autotune, ...). A Rust binary can't load them. Options: port the addons people use (COSMOS's set
  first) and drop runtime plugin loading; or embed a small Python for plugins only (against the no-Python goal).
  Recommended: port the addons, no embedded Python.
- **Decided (user 2026-10-23):** reactor uses stackful coroutines (`corosensei`).
- MCU dicts: Kalico/Klipper published dict tarball vs build all 27 ourselves. Recommended: build what our cross
  compilers allow (we need the firmware build working anyway), download the rest, cross-check overlaps.
- **Decided (user 2026-10-23):** use nightly Rust where it is needed (AVR).
