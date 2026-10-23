#!/bin/bash
# Runs each script of tests/scripts (or the given ones) on Kalico's C linux
# MCU and on kaliru-mcu and diffs what they answer.
# usage: mcu_cmp.sh [script...]   (build both first: see tests/ref/README.md)
cd "$(dirname "$0")"
C_MCU=${C_MCU:-~/rk/fwlinux/klipper.elf}
RS_MCU=${RS_MCU:-~/rk/fwtarget/release/kaliru-mcu}
# the fake hardware shim
gcc -shared -fPIC -O2 -o $HOME/rk/kaliru_fakehw.so fakehw.c -ldl || exit 1
scripts=("$@")
[ ${#scripts[@]} -eq 0 ] && scripts=(scripts/*.script)
same=0
for s in "${scripts[@]}"; do
    name=$(basename "$s" .script)
    if ! python3 mcu_run.py "$C_MCU" "$s" --fakehw $HOME/rk/kaliru_fakehw.so > /tmp/mcu_c_$name.txt 2>&1         || ! python3 mcu_run.py "$RS_MCU" "$s" --fakehw $HOME/rk/kaliru_fakehw.so > /tmp/mcu_rs_$name.txt 2>&1; then
        echo "ERROR $name (mcu_run.py failed)"
        tail -3 /tmp/mcu_c_$name.txt /tmp/mcu_rs_$name.txt
        continue
    fi
    if diff -q /tmp/mcu_c_$name.txt /tmp/mcu_rs_$name.txt > /dev/null; then
        echo "SAME $name"
        same=$((same + 1))
    else
        echo "DIFF $name"
        diff /tmp/mcu_c_$name.txt /tmp/mcu_rs_$name.txt | head -40
    fi
done
echo "$same/${#scripts[@]} identical"
