# kaliru-mcu

Kalico's micro-controller firmware (`src/` of Kalico `5b59427`) in Rust. Ported so far: the generic core (scheduler,
command parsing, all command modules and sensor drivers) and the **linux** board (the MCU code running as a Linux
process, Kalico's `src/linux`). STM32 boards come next.

## Building

```sh
cargo build --release                                  # configs/linux
KALIRU_MCU_CONFIG=<dir> cargo build --release          # another configuration
```

A configuration directory holds the output of Kalico's Kconfig tooling for one board: `autoconf.h` (every symbol,
unset ones as `0` / `""`), `config` (the `.config`, which tells bools from ints) and `defconfig` (the minimal config,
embedded in the data dictionary). Make them with Kalico's `make menuconfig` / `make olddefconfig`.

Run: `kaliru-mcu -I /tmp/klipper_host_mcu` (same options as Kalico's `klipper.elf`: `-w` watchdog, `-r` realtime,
`-g` / `-m` pty group and mode).

## How it maps to the C code

- One Rust file per C file, same names, functions and declaration order.
- Compile time requests: `decl_command!`, `decl_init!`, `decl_task!`, `decl_shutdown!`, `decl_constant!`,
  `decl_enumeration!`, `sendf!`, `output!`, `shutdown!`, `try_shutdown!` (`src/ctr.rs`). The build script
  (`build/`) reads them from the source text, as the C build reads each object's `.compile_time_request` section,
  and runs a port of `scripts/buildcommands.py`. Per file the requests are taken in reverse source order, the order
  gcc emits them; `#[cfg(...)]` items are left out like `#if` blocks, a plain `if CONFIG_X` keeps its strings like
  C's `if (CONFIG_X)`.
- `sched_shutdown()` longjmps in C; here it is a `Shutdown` error passed up with `?` to `sched_main()`, so the same
  frames are left. Shutdown handlers return it too: a shutdown inside one runs them again, like the C longjmp.
- Objects are `#[repr(C)]` structs in the same 20 KB pool with the same layout, so `move_count` (and every
  allocation failure) matches the C firmware.

## Checks

- `$OUT_DIR/compile_time_request.txt` must equal `tests/ref/linux_compile_time_request.txt` (it does), and the data
  dictionary `tests/ref/linux_klipper.dict` apart from `version` / `build_versions` (it does, byte for byte).
- `tests/mcu_cmp.sh` runs `tests/scripts/*.script` against Kalico's C linux MCU and kaliru-mcu under the
  `tests/fakehw.c` shim (fake gpio / spi / i2c / adc hardware, every pin write and transfer logged) and diffs the
  answers and the hardware logs. All scripts are identical.
