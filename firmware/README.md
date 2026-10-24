# kaliru-mcu

Kalico's micro-controller firmware (`src/` of Kalico `5b59427`) in Rust. Ported: the generic core (scheduler,
command parsing, all command modules and sensor drivers) and these boards:

- **linux** (the MCU code as a Linux process, `src/linux`) and the **simulator** (`src/simulator`);
- **stm32** (`src/stm32`): F0, F1, F2, F4, F7, G0, G4, H7, L4, AT32F403, N32G45x: clock setup, gpio, adc, spi, i2c,
  hardware pwm, sdio, USART serial, USB (fs / otg), bxCAN, FDCAN, the USB-to-CAN bridge, watchdog and the
  bootloader requests;
- **rp2040 / rp2350** (`src/rp2040`): USB, serial, spi, i2c, adc, pwm, can2040 (the PIO CAN library, ported), the
  boot2 stage (`tools/gen_rp2040_stage2.py`) and `tools/elf2uf2` (byte-identical `.uf2` files);
- **lpc176x** (`src/lpc176x`), **atsam** (`src/atsam`: sam3x, sam4s, sam4e, same70) and **atsamd**
  (`src/atsamd`: samd21, samc21, samd51, same51, same54): USB, serial, CAN, spi, i2c, adc, pwm;
- the ARM Cortex-M code of `src/generic/armcm_*`, `usb_cdc`, `serial_irq`, `canbus`, `canserial`, `usb_canbus`;
- the **INDX heater** (`src/indx`, C++ in Kalico: the Bondtech INDX induction heater on a SAME51J19 toolboard), with
  the newlib math it uses (`src/libm.rs`: powf, sqrtf, sin, bit for bit newlib 4.4.0's).

Not ported: hc32f460, avr, ar100, pru.

## Building

```sh
cargo build --release                                  # configs/linux
KALIRU_MCU_CONFIG=<dir> cargo build --release          # another linux configuration

# micro-controllers: the target matches the chip (Kalico builds them all soft-float)
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

Targets: `thumbv6m-none-eabi` (Cortex-M0/M0+: stm32 F0 / G0, rp2040, samd21, samc21), `thumbv7m-none-eabi` (M3:
stm32 F1 / F2, sam3x, lpc176x), `thumbv7em-none-eabi` (M4 / M7: the other stm32, AT32, N32, sam4, same70, samd51 /
same5x), `thumbv8m.main-none-eabi` (M33: rp2350); linux and the simulator build for the host.

A chip needs its device header as Rust, `src/<board>/chip/<mcu>.rs`, made from Kalico's `lib/` with
`tools/gen_stm32_regs.py` (modes: stm32 CMSIS headers, `--pico` for the pico-sdk, `--cmsis` for other vendors'
CMSIS headers incl. Atmel's ASF unions, `--header` for one header); register values come from gcc, struct offsets
are asserted.

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
- `decl_ctr!("...")` is C's raw `DECL_CTR` (a compile_time_request.txt line as written, used by the INDX code).
- `sched_shutdown()` longjmps in C; here it is a `Shutdown` error passed up with `?` to `sched_main()`, so the same
  frames are left. Shutdown handlers return it too: a shutdown inside one runs them again, like the C longjmp. A
  shutdown inside an irq handler (the SysTick timer dispatch) resets the stack and re-enters the `sched_main()`
  loop from the handler, as the longjmp does (`generic/armcm_irq.rs`).
- Objects are `#[repr(C)]` structs in the same dynamic memory pool with the same layout, so `move_count` (and every
  allocation failure) matches the C firmware on linux. (On STM32 the pool is the RAM after `.bss`; the Rust `.bss`
  is a little smaller than the C one, so the pool is a little larger.)
- Registers: the CMSIS structs with volatile `Reg<T>` fields (`regs.MODER.write(..)`), names as in C.

## Checks

- `tests/ctr_check.sh [config ...]` builds every configuration in `configs/` (52: linux, simulator, 25 stm32 incl. AT32 / N32, 5
  rp2040 / rp2350, 4 lpc176x, 7 atsam, 9 atsamd incl. the INDX toolboard) and compares
  `$OUT_DIR/compile_time_request.txt` with the C build's (`tests/ref/<config>_compile_time_request.txt`) and the
  data dictionary with `tests/ref/<config>_klipper.dict` apart from `version` / `build_versions`. New configs:
  `configs/src/<name>.in` (Kconfig choices), `../tools/mkref.sh <name>` builds the C reference.
- `tests/mcu_cmp.sh` runs `tests/scripts/*.script` against Kalico's C linux MCU and kaliru-mcu under the
  `tests/fakehw.c` shim (fake gpio / spi / i2c / adc hardware, every pin write and transfer logged) and diffs the
  answers and the hardware logs. All scripts are identical.
- `src/libm.rs` against newlib: 700k powf / sqrtf / sin inputs (temperatures, radiances, random bit patterns,
  huge and NaN arguments) give identical bits to the arm-none-eabi libm under qemu-arm.
- Behaviour on a chip is not checked for any micro-controller board yet (no hardware run; the Renode set-up in `tests/renode/` does not yet run
  Kalico's own image, see NOTES "Kaliru MCU firmware").
