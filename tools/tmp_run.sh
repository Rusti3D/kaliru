R=~/rk/fwtarget-ctr-same70q20b-usb/thumbv7em-none-eabi/mcu/kaliru-mcu; C=~/rk/fw_same70q20b-usb/klipper.elf
for f in $R $C; do echo "== $f"; arm-none-eabi-objdump -h $f | grep -E "^ +[0-9]" | grep -v debug; arm-none-eabi-nm $f | grep -E " (_text_vectortable_start|_text_size|_dtcm_bss_start|_dtcm_bss_end|ResetHandler|_data_flash)$"; done
