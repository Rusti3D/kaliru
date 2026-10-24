#!/bin/bash
# Build Kalico's C firmware for configs/src/<name>.in (Kconfig choices) and
# keep its configuration and its ctr txt + dictionary:
#   tools/mkref.sh <name> ...
F=/mnt/d/RustyKlipper/kaliru/firmware
for N in "$@"; do
O=$HOME/rk/fw_$N
mkdir -p $O && cp $F/configs/src/$N.in $O/.config
cd ~/rk/kalico
make OUT=$O/ KCONFIG_CONFIG=$O/.config olddefconfig > /dev/null
if ! make OUT=$O/ KCONFIG_CONFIG=$O/.config -j8 > $O/build.log 2>&1; then
    # (avr: binutils 2.26 checks the memory regions before --gc-sections drops
    # the LTO objects' compile time request sections; link again without
    # stopping there, and keep the image if it fits the chip)
    mcu=$(sed -n 's/^CONFIG_MCU="\(.*\)"/\1/p' $O/.config)
    if grep -q "^CONFIG_MACH_AVR=y" $O/.config && grep -q "not within region" $O/build.log; then
        cmd=$(make OUT=$O/ KCONFIG_CONFIG=$O/.config V=1 2>&1 | grep -- "-o $O/klipper.elf" | head -1)
        eval "$cmd -Wl,--noinhibit-exec" > /dev/null 2>&1
        if ! avr-size -C --mcu=$mcu $O/klipper.elf | grep -q "Full)" || avr-size -C --mcu=$mcu $O/klipper.elf | grep -qE "\(([1-9][0-9]{2,}|100)\.[0-9]% Full"; then
            echo "C build of $N failed (too big)"; avr-size -C --mcu=$mcu $O/klipper.elf; continue
        fi
    else
        echo "C build of $N failed"; tail -20 $O/build.log; continue
    fi
fi
mkdir -p $F/configs/$N
cp $O/autoconf.h $F/configs/$N/autoconf.h; cp $O/.config $F/configs/$N/config; cp $O/defconfig $F/configs/$N/defconfig
cp $O/compile_time_request.txt $F/tests/ref/${N}_compile_time_request.txt; cp $O/klipper.dict $F/tests/ref/${N}_klipper.dict
echo "== $N: $(grep 'CONFIG_MCU=' $O/.config)"
done
