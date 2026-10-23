# kaliru-mcu

Kalico's micro-controller firmware (`src/` of Kalico `5b59427`) in Rust. Ported so far: the generic core (scheduler,
command parsing, all command modules and sensor drivers), the **linux** board (the MCU code running as a Linux
process, Kalico's `src/linux`) and the **STM32F1 / STM32F4** boards (`src/stm32`: clock setup, gpio, adc, spi,
i2c, hardware pwm, sdio, USART serial, USB OTG CDC, watchdog, DFU / HID / stm32duino bootloader requests, and the
ARM Cortex-M `src/generic/armcm_*` code).

## Building

```sh
cargo build --release                                  # configs/linux
KALIRU_MCU_CONFIG=<dir> cargo build --release          # another linux configuration

# STM32: the target matches the chip (Kalico builds them soft-float)
KALIRU_MCU_CONFIG=configs/stm32f401-usb cargo build --profile mcu --target thumbv7em-none-eabi
KALIRU_MCU_CONFIG=configs/stm32f103-serial cargo build --profile mcu --target thumbv7m-none-eabi
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabi/mcu/kaliru-mcu klipper.bin
```

A configuration directory holds the output of Kalico's Kconfig tooling for one board: `autoconf.h` (every symbol,
unset ones as `0` / `""`), `config` (the `.config`, which tells bools from ints) and `defconfig` (the minimal config,
embedded in the data dictionary). Make them with Kalico's `make menuconfig` / `make olddefconfig`
(`configs/src/*.config` are the inputs of the ones here).

The `mcu` profile is fat LTO (Kalico's `-flto -fwhole-program`) at opt-level `s`: at opt-level 2 the image is
twice the C one and does not fit a 64KB STM32F103 (`CARGO_PROFILE_MCU_OPT_LEVEL=z` makes it smaller still).

An STM32 chip needs its CMSIS device header as Rust, `src/stm32/chip/<CONFIG_MCU>.rs`, made from Kalico's `lib/`
with `tools/gen_stm32_regs.py <kalico>/lib <family> <mcu> <out.rs>` (stm32f401xc and stm32f103xe are here).

Run (linux): `kaliru-mcu -I /tmp/klipper_host_mcu` (same options as Kalico's `klipper.elf`: `-w` watchdog, `-r`
realtime, `-g` / `-m` pty group and mode). STM32: flash `klipper.bin` like Kalico's, at the configured
application address.

## How it maps to the C code

- One Rust file per C file, same names, functions and declaration order.
- Compile time requests: `decl_command!`, `decl_init!`, `decl_task!`, `decl_shutdown!`, `decl_constant!`,
  `decl_enumeration!`, `decl_armcm_irq!`, `armcm_enable_irq!`, `sendf!`, `output!`, `shutdown!`, `try_shutdown!`
  (`src/ctr.rs`). The build script (`build/`) reads them from the source text, as the C build reads each object's
  `.compile_time_request` section, and runs a port of `scripts/buildcommands.py` (including the ARM vector table
  of `DECL_ARMCM_IRQ`). Per file the requests are taken in reverse source order, the order gcc emits them;
  `#[cfg(...)]` items are left out like `#if` blocks, a plain `if CONFIG_X` keeps its strings like C's
  `if (CONFIG_X)`; `#[cfg(has_X)]` is the C `#ifdef X` on a name of the chip header.
- `sched_shutdown()` longjmps in C; here it is a `Shutdown` error passed up with `?` to `sched_main()`, so the same
  frames are left. Shutdown handlers return it too: a shutdown inside one runs them again, like the C longjmp. A
  shutdown inside an irq handler (the SysTick timer dispatch) resets the stack and re-enters the `sched_main()`
  loop from the handler, as the longjmp does (`generic/armcm_irq.rs`).
- Objects are `#[repr(C)]` structs in the same dynamic memory pool with the same layout, so `move_count` (and every
  allocation failure) matches the C firmware on linux. (On STM32 the pool is the RAM after `.bss`; the Rust `.bss`
  is a little smaller than the C one, so the pool is a little larger.)
- Registers: the CMSIS structs with volatile `Reg<T>` fields (`regs.MODER.write(..)`), names as in C.

## Checks

- `$OUT_DIR/compile_time_request.txt` must equal `tests/ref/<config>_compile_time_request.txt`, and the data
  dictionary `tests/ref/<config>_klipper.dict` apart from `version` / `build_versions`. Both hold, byte for byte, for
  linux, stm32f401-usb and stm32f103-serial.
- `tests/mcu_cmp.sh` runs `tests/scripts/*.script` against Kalico's C linux MCU and kaliru-mcu under the
  `tests/fakehw.c` shim (fake gpio / spi / i2c / adc hardware, every pin write and transfer logged) and diffs the
  answers and the hardware logs. All scripts are identical.
- STM32 behaviour is not checked yet (no hardware run; the Renode set-up in `tests/renode/` does not yet run
  Kalico's own image, see NOTES "Kaliru MCU firmware").
