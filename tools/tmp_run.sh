cd /mnt/d/RustyKlipper/kaliru/firmware
bash ../tools/mkref.sh at32f403-serial n32g455-serial 2>&1 | grep "==\|fail\|error" | head
grep "CONFIG_MCU=\|MACH_STM32F1=\|N32G45x=" configs/at32f403-serial/config configs/n32g455-serial/config
