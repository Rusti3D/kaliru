bash /mnt/d/RustyKlipper/kaliru/tools/mkref.sh samd21e15-usb 2>&1 | tail -5
cd /mnt/d/RustyKlipper/kaliru/firmware && bash tests/ctr_check.sh samd21e15-usb
arm-none-eabi-size ~/rk/fw_samd21e15-usb/klipper.elf ~/rk/fwtarget-ctr-samd21e15-usb/thumbv6m-none-eabi/mcu/kaliru-mcu
