cd /mnt/d/RustyKlipper/kaliru/firmware
python3 tools/gen_rp2040_stage2.py ~/rk/kalico src/rp2040/stage2.rs
KALIRU_MCU_CONFIG=configs/rp2040-usb CARGO_TARGET_DIR=~/rk/fwtarget-ctr-rp2040-usb ~/.cargo/bin/cargo build --profile mcu --target thumbv6m-none-eabi 2>&1 | grep -E "^(error|warning)" -A7 | head -30
R=~/rk/fwtarget-ctr-rp2040-usb/thumbv6m-none-eabi/mcu/kaliru-mcu
arm-none-eabi-nm $R | grep -E " (_text_vectortable_start|ResetHandler)$"
arm-none-eabi-objcopy -O binary -j .text $R /tmp/r.bin; arm-none-eabi-objcopy -O binary -j .text ~/rk/fw_rp2040-usb/klipper.elf /tmp/c.bin
cmp -n 256 /tmp/r.bin /tmp/c.bin && echo boot2-identical
