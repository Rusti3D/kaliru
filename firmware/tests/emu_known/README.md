# Known emulator differences

`emu_cmp.sh <config>` compares the C and Rust builds of a config in the
emulator (`emu_run.py`): their answers (`<script>.txt.diff`) and their
peripheral register accesses (`<script>.diff`). A difference recorded here,
byte for byte, is expected; anything else is reported as `DIFF`.

Every one of them comes from Kalico's C code doing something whose effect the
C compiler is free to choose (non-volatile pointers to hardware registers,
uninitialised memory) or from a C defect; the Rust port does what the source
says, with volatile register accesses in source order.

Memory that is not registers is not logged: the stm32 system memory and the
atsamd nvm rows (unique id, GET_FUSE calibration), the stm32 fdcan message
ram, the atsam / atsamd M_CAN message ram (the image's own ram). Kalico reads
and writes those through plain pointers, in an order and with access sizes
gcc picks; what they carry is checked through the answers and the frames the
can models log.

- `at32f403-serial/*.diff` (every script, the clock setup): `stm32/stm32f1.c`
  `clock_setup()` sets the AT32 ACC_CTRL1, CRM_MISC1 and CRM_MISC3 registers
  through `*((uint32_t *)REG_..._ADDR) |= ...`, which are not volatile; gcc
  moves those read-modify-writes after the volatile `RCC->CFGR = cfgr`. Rust
  writes them first, in source order. Same final register values.
- `n32g455-serial/bus.diff` (the adc setup): `lib/n32g45x/include/n32g45x_adc.h`
  defines `ADC_RCC_AHBPCLKEN` and `ADC_RCC_CFG2` as `*((uint32_t *)(...))`,
  not volatile; gcc merges `n32g45x_adc.c`'s two read-modify-writes of CFG2
  into one and moves it before the AHBPCLKEN one. Rust does both, in order.
  Same final register values.
- `hc32f460-serial/*.diff`, `hc32f460-ph2/*.diff` (every script, the usart
  setup): `lib/hc32f460/driver/src/hc32f460_usart.c` `USART_UART_Init()`
  ends with the struct copies `USARTx->PR_f = PR_f; ... CR1_f = CR1_f;`,
  which gcc does not treat as volatile accesses and emits as CR3, PR, CR1,
  CR2; Rust writes PR, CR2, CR3, CR1. Same final register values.
- `hc32f460-serial/drivers.diff`, `hc32f460-ph2/drivers.diff` (also the
  neopixel): `hc32f460/gpio.c` `gpio_out_toggle_noirq()` writes the port's
  POTR through a non-volatile `uint16_t *`; in `neopixel.c` a short pulse is
  two identical toggles with only (volatile) timer reads between them, so gcc
  keeps one store and the C firmware sends no pulse for a 0 bit. Rust makes
  both toggles (a C defect, not reproduced).
- `samd21e15-usb/{config,gpio,stepper}.txt.diff`: `move_count`. The Rust
  image's ram (.data + .bss) is smaller, so its dynamic memory pool, and the
  moves it holds, are larger; on this 4KB ram chip the count stays below its
  1024 cap, so the answers show it (see NOTES "Kaliru MCU firmware", size).
- `lpc1768-usbcan/*.diff`, `rp2040-usbcan/*.diff`, `same70q20b-usbcan/*.diff`,
  `stm32f446-usbcan/*.diff`, `stm32h723-usbcan/*.diff` (every script, the
  usb bulk in data register or buffer only): `generic/canserial.c`
  `canserial_tx_task()` fills only `dlc` bytes of its uninitialised
  `struct canbus_msg msg`, and `usb_canbus.c` `send_frame()` copies both
  `data32` words into the `gs_host_frame` it sends, so the C frames' bytes
  past the DLC are stack contents. Rust's message starts zeroed. The host
  ignores those bytes; the answers are identical. (same54p20-usbcan sends
  through usb dma descriptors in ram, which is not logged: identical.)
- `rp2350-usb/drivers.txt.diff`, `drivers.diff` (the I2C timeout and all
  times after it): `sensor_lis2dw.c` `command_query_lis2dw_status()` reads
  `time2 = timer_read_time()` (the volatile `DWT->CYCCNT`) before
  `i2c_shutdown_on_err(ret)`; gcc moves that load after the call, which
  does not return on the timeout, so the C firmware never reads `time2` and
  its shutdown clock (and every time after it) is one clock read (16 ticks
  in the emulator) earlier. Rust reads it in source order. The rp2040
  (Cortex-M0+) C build of the same source keeps the load before the call:
  identical.
- `stm32f072-serial/stepper.txt.diff`, `stepper.diff`: the C image's stack
  (CONFIG_STACK_SIZE 512) overflows. With LTO `run_tasks` has a 300 byte frame,
  the idle loop already uses 416 bytes, and `config_stepper` -> `oid_alloc` ->
  `alloc_chunk` -> `memset` goes 52 bytes past the stack into the heap, which
  that memset clears: the C image crashes (`< crash`). The Rust image uses
  less stack there and goes on (a C defect, not reproduced).
