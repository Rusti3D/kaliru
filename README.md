# Kaliru

> **Work in progress.** Kaliru is not ready for a printer yet. It is verified
> only against Kalico's own test suite and file-output test runs, not on real
> hardware, and the MCU firmware is not ported. Do not run it on a printer.

Kaliru is a port of [Kalico](https://github.com/KalicoCrew/kalico) (a Klipper
fork) to Rust, aimed at small printer hosts with little RAM (the first target
is the Elegoo Centauri Carbon: armv7, about 115 MB of RAM).

Phase 1 is a 1:1 port: for the same config and g-code, Kaliru must produce the
same MCU command stream, exit code and log as Kalico. Phase 2 (later) is
optimisation.

Reference: Kalico `5b59427`.

## Status

- **Kalico's stock test suite: 85 of 88 runs identical** to Kalico (MCU
  command trace, exit code and log). The other three: two need python g-code
  macros (planned through RustPython), and one differs only by wall-clock
  timer timing.
- **Custom parity cases** (`tests/cases/`, 44 runs covering modules the stock
  suite doesn't reach): all identical to Kalico.
- Ported: the whole klippy host (reactor, serial and msgproto, clock sync,
  config, pins, gcode, toolhead, steppers and rails, homing, mcu), the chelper
  C code, all kinematics, a Jinja2 port for g-code macros, and every Kalico
  `extras` module except the two below: heaters and all temperature sensors,
  TMC drivers, input shaper, probes (incl. eddy current, dockable, BLTouch,
  smart effector), bed mesh and leveling tools, displays and menus,
  accelerometers, load cells, angle sensors, filament sensors, MMUs
  (Palette 2, Trad Rack), Replicape, and more.
- Python exception tracebacks are reproduced too, including chained ones
  ("During handling of the above exception ...").
- Not ported yet:
  - `load_cell_probe` (in progress, including the scipy filter design it uses),
  - `shaper_calibrate` (needs a numpy-faithful FFT),
  - python g-code macros,
  - the MCU firmware.

  Loading an unported module fails with
  `Module 'x' is not ported to Kaliru yet`.
- Known limits: paths that read sensors over i2c/spi can't be exercised in
  Kalico's file-output test mode. They are ported from source but not yet
  verified against Kalico.

Per-module status and how each part is checked: [`docs/PARITY.md`](docs/PARITY.md)
(being brought up to date). Plan: [`docs/PLAN.md`](docs/PLAN.md).

## Building

```sh
cargo build --release
```

The parity tools in `tools/` compare Kaliru against a Kalico checkout
(patched with `tools/ref_trace.patch` to trace the serialqueue). They expect
the layout described in the script headers (`tools/suite_cmp.sh`,
`tools/case_cmp.sh`, `tools/refdiff.py`).

## License

GPLv3, like Kalico and Klipper (see `COPYING`). `ref/` holds copies of the
Python 3.13 stdlib and Jinja2 sources used as porting references, under their
own licenses.
