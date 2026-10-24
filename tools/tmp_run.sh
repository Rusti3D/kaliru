cd /mnt/d/RustyKlipper/kaliru/firmware
touch src/main.rs
KALIRU_MCU_CONFIG=configs/n32g455-serial CARGO_TARGET_DIR=~/rk/fwtarget-ctr-n32g455-serial ~/.cargo/bin/cargo build --profile mcu --target thumbv7em-none-eabi 2>&1 | grep -E "^warning|^error" -A4 | head -30
arm-none-eabi-size ~/rk/fwtarget-ctr-n32g455-serial/thumbv7em-none-eabi/mcu/kaliru-mcu
