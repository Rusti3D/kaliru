# Reference outputs of Kalico's C firmware

Built from Kalico `550d21fb` with `firmware/configs/linux` (`make OUT=... KCONFIG_CONFIG=...`):

- `linux_compile_time_request.txt`: `out/compile_time_request.txt`, the compile time requests in link order.
- `linux_klipper.dict`: `out/klipper.dict`, the data dictionary.

The Rust build must produce the same request stream (`$OUT_DIR/compile_time_request.txt`) and, apart from
`version` / `build_versions`, the same dictionary bytes. `KALIRU_MCU_CTR_INPUT=<txt>` makes the build script read a
request file instead of the sources (a check of the generator alone).
