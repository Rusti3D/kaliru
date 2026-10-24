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
make OUT=$O/ KCONFIG_CONFIG=$O/.config -j8 > $O/build.log 2>&1 || { echo "C build of $N failed"; tail -20 $O/build.log; continue; }
mkdir -p $F/configs/$N
cp $O/autoconf.h $F/configs/$N/autoconf.h; cp $O/.config $F/configs/$N/config; cp $O/defconfig $F/configs/$N/defconfig
cp $O/compile_time_request.txt $F/tests/ref/${N}_compile_time_request.txt; cp $O/klipper.dict $F/tests/ref/${N}_klipper.dict
echo "== $N: $(grep 'CONFIG_MCU=' $O/.config)"
done
