# Kaliru

> **Work in progress.** Kaliru is not ready for a printer yet. It is verified
> only against Kalico's own test suite and file-output test runs, not on real
> hardware; the MCU firmware's micro-controller boards build and match Kalico's
> data dictionary but have not run on a chip yet. Do not run it on a printer.

Kaliru is a port of [Kalico](https://github.com/KalicoCrew/kalico) (a Klipper
fork) to Rust, aimed at small printer hosts with little RAM (the first target
is the Elegoo Centauri Carbon: armv7, about 115 MB of RAM).

Phase 1 is a 1:1 port: for the same config and g-code, Kaliru must produce the
same MCU command stream, exit code and log as Kalico. Phase 2 (later) is
optimisation.

Reference: Kalico `5b59427`.

## Status

- **Kalico's stock test suite: 87 of 88 runs identical** to Kalico (MCU
  command trace, exit code and log). The remaining one differs only by
  wall-clock timer timing.
- **Custom parity cases** (`tests/cases/`, 62 runs covering modules the stock
  suite doesn't reach): all identical to Kalico.
- **Live runs** (`tools/live_cmp.sh`, `tests/live/`): Kalico and Kaliru each
  drive real Linux-process MCUs (with fake hardware) through a g-code session,
  including MCU shutdowns, reconnects and TMC faults; the transcripts match.
- **armv7 (the Centauri Carbon's CPU)**: `tools/build_armv7.sh` cross-builds
  it; under qemu the stock suite (88/88) and the custom cases (62/62) are
  identical to Kalico on x86.
- **Memory**: in the live runs Kaliru's own (anonymous) memory is 2.7 MB
  against Kalico's 34 MB, and CPU time 0.03-0.09 s against about 1.6 s
  (x86_64, `resources.txt` of `tools/live_run.py`).
- Ported: the whole klippy host (reactor, serial and msgproto, clock sync,
  config, pins, gcode, toolhead, steppers and rails, homing, mcu), the chelper
  C code, all kinematics, a Jinja2 port for g-code macros, and every Kalico
  `extras` module: heaters and all temperature sensors,
  TMC drivers, input shaper, probes (incl. eddy current, dockable, BLTouch,
  smart effector), bed mesh and leveling tools, displays and menus,
  accelerometers, load cells and load cell probes, angle sensors, filament sensors, MMUs
  (Palette 2, Trad Rack), Replicape, and more.
- The numpy and scipy pieces Kalico relies on (rfft, kaiser window, least
  squares, the load cell probe filter design) are ported and checked against
  numpy/scipy; input shaper calibration gives identical results.
- Python exception tracebacks are reproduced too, including chained ones
  ("During handling of the above exception ...").
- Python g-code macros (`gcode: !...`, `!!include file.py`) run on an
  embedded [RustPython](https://github.com/RustPython/RustPython) with
  Kalico's helpers (`emit`, `printer`, `own_vars`, `wait_moves`, ...). Syntax
  and runtime error messages follow CPython 3.13 (`vendor/` holds the patched
  RustPython crates). Known gap: no `match` statement.
- MCU firmware (`firmware/`, the `kaliru-mcu` crate): the generic core, every
  command module and sensor driver, and the Linux-process, simulator, STM32
  (F0, F1, F2, F4, F7, G0, G4, H7, L4, AT32F403, N32G45x), RP2040 / RP2350,
  LPC176x, SAM3/SAM4/SAME70, SAMD21/SAMC21/SAMD51/SAME5x, HC32F460 and AVR
  (atmega, at90usb) boards with serial, USB, CAN / FDCAN and the USB-to-CAN
  bridge, plus the Bondtech INDX heater code. For all 64 build configurations in `firmware/configs/` the data
  dictionary is byte-identical to Kalico's C build (version fields aside). For
  the linux board, scripted sessions against both, with fake gpio/spi/i2c/adc
  hardware, give the same answers and the same pin and bus activity. In an
  emulator the C and Rust images of every micro-controller config (serial,
  USB, CAN incl. can2040 on the rp2040 PIO, USB-to-CAN; AVR on simavr) give
  the same answers and the same peripheral register accesses; no micro-controller
  build has run on a chip yet. See
  [`firmware/README.md`](firmware/README.md).
- Not ported yet: the AR100 (or1k) and PRU boards (Rust has no target for
  them; planned through rustc_codegen_gcc).
- Planned: Python plugin support (Kalico's `klippy/extras` and
  `klippy/plugins` modules), so that a plugin that works on Kalico works on
  Kaliru, with no cost when none is configured. Design and measurements:
  [`docs/PLUGINS.md`](docs/PLUGINS.md); work happens on the `plugins` branch.

## Building

```sh
cargo build --release                        # with python macros (24.3 MB)
cargo build --release --no-default-features  # without them
tools/build_armv7.sh                         # armv7 (gcc linker, glibc)
```

The parity tools in `tools/` compare Kaliru against a Kalico checkout
(patched with `tools/ref_trace.patch` to trace the serialqueue). They expect
the layout described in the script headers (`tools/suite_cmp.sh`,
`tools/case_cmp.sh`, `tools/refdiff.py`).

## License

GPLv3, like Kalico and Klipper (see `COPYING`). `ref/` holds copies of the
Python 3.13 stdlib and Jinja2 sources used as porting references, under their
own licenses.
