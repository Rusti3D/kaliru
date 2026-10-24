cd /mnt/d/RustyKlipper/kaliru/firmware
touch src/main.rs
KALIRU_MCU_CONFIG=configs/stm32f103-angle CARGO_TARGET_DIR=~/rk/fwtarget-ctr-stm32f103-angle ~/.cargo/bin/cargo build --profile mcu --target thumbv7m-none-eabi 2>&1 | grep -E "^warning|^error" -A5 | head -30
