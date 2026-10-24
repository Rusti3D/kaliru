cd /mnt/d/RustyKlipper/kaliru/firmware
for c in rp2040-can rp2040-usbcan; do
KALIRU_MCU_CONFIG=configs/$c CARGO_TARGET_DIR=~/rk/fwtarget-ctr-$c ~/.cargo/bin/cargo build --profile mcu --target thumbv6m-none-eabi 2>&1 | grep -E "^(error|warning)" -A7 | head -40
done
bash tests/ctr_check.sh rp2040-can rp2040-usbcan
