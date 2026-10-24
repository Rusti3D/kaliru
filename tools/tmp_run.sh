cd /mnt/d/RustyKlipper/kaliru/firmware
KALIRU_MCU_CONFIG=configs/simulator CARGO_TARGET_DIR=~/rk/fwtarget-ctr-simulator ~/.cargo/bin/cargo build --release 2>&1 | grep -E "^(error|warning)" -A7 | head -60
bash tests/ctr_check.sh simulator
