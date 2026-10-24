cd /mnt/d/RustyKlipper/kaliru/firmware
bash tests/ctr_check.sh lpc1768-usb lpc1769-serial lpc1768-can lpc1768-usbcan 2>&1 | tail -30
for c in lpc1769-serial lpc1768-can lpc1768-usbcan; do KALIRU_MCU_CONFIG=configs/$c CARGO_TARGET_DIR=~/rk/fwtarget-ctr-$c ~/.cargo/bin/cargo build --profile mcu --target thumbv7m-none-eabi 2>&1 | grep -E "^warning" -A6 | head -20; done
arm-none-eabi-size ~/rk/fwtarget-ctr-lpc1768-usb/thumbv7m-none-eabi/mcu/kaliru-mcu ~/rk/fw_lpc1768-usb/klipper.elf
