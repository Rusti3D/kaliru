# Kaliru

> **Work in progress.** Kaliru is not usable on a printer yet. Large parts of
> Kalico are still unported, and the parts that are ported are only verified
> against the reference test suite. Do not run it on real hardware.

Kaliru is a port of [Kalico](https://github.com/KalicoCrew/kalico) (a Klipper
fork) to Rust, aimed at small printer hosts with little RAM (the first target
is the Elegoo Centauri Carbon: armv7, about 115 MB of RAM).

Phase 1 is a 1:1 port: for the same config and g-code, Kaliru must produce the
same MCU command stream, exit code and log as Kalico. Phase 2 (later) is
optimisation.

Reference: Kalico `5b59427`.

## Status

- Ported: the klippy core (reactor, serial and msgproto handling, clock sync,
  config parsing, pins, gcode, toolhead, stepper/rails, homing, mcu), the
  chelper C code used so far (serialqueue, stepcompress, steppersync, trapq,
  itersolve, cartesian and extruder kinematics), a Jinja2 port for g-code
  macros, and a growing set of `extras` modules (heaters, thermistors, fans,
  probe, bed_mesh, virtual_sdcard, ...).
- Not ported yet: most other kinematics, TMC drivers, input shaper, displays,
  many `extras` modules, python g-code macros, the MCU firmware.
  Loading an unported module fails with
  `Module 'x' is not ported to Kaliru yet`.
- Kalico's stock test suite (88 runs): 19 identical to Kalico so far; every
  remaining difference is an unported module or python macros.

Per-module status and how each part is checked: [`docs/PARITY.md`](docs/PARITY.md).
Plan: [`docs/PLAN.md`](docs/PLAN.md).

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
