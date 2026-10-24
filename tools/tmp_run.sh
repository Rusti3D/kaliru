cd /mnt/d/RustyKlipper/kaliru/firmware
bash tests/ctr_check.sh rp2350-usb
R=~/rk/fwtarget-ctr-rp2350-usb/thumbv8m.main-none-eabi/mcu/kaliru-mcu; C=~/rk/fw_rp2350-usb/klipper.elf
for f in $R $C; do arm-none-eabi-nm $f | grep -E " (_text_vectortable_start|ResetHandler)$"; arm-none-eabi-size $f | tail -1; done
