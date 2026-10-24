#!/bin/bash
# Runs each script of tests/scripts on the C build (~/rk/fw_<config>/klipper.elf,
# tools/mkref.sh) and the Rust build (~/rk/fwtarget-ctr-<config>, ctr_check.sh)
# of a micro-controller config in the emulator (emu_run.py) and diffs the
# answers and the peripheral register accesses.
# usage: emu_cmp.sh <config> [script...]
cd "$(dirname "$0")"
c=$1
shift
C_ELF=~/rk/fw_$c/klipper.elf
RS_ELF=$(ls ~/rk/fwtarget-ctr-$c/*/mcu/kaliru-mcu 2>/dev/null | head -1)
C_DICT=~/rk/fw_$c/klipper.dict
RS_DICT=$(ls -td ~/rk/fwtarget-ctr-$c/*/mcu/build/kaliru-mcu-*/out 2>/dev/null | head -1)/klipper.dict
[ -f "$C_ELF" ] && [ -n "$RS_ELF" ] || { echo "build $c first (tools/mkref.sh, tests/ctr_check.sh)"; exit 1; }
scripts=("$@")
[ ${#scripts[@]} -eq 0 ] && scripts=(scripts/*.script)
out=~/rk/emu_cmp/$c
mkdir -p $out
same=0
for s in "${scripts[@]}"; do
    name=$(basename "$s" .script)
    if ! timeout 600 python3 emu_run.py "$C_ELF" $C_DICT ../configs/$c "$s" $out/c_$name.log > $out/c_$name.txt 2>&1 \
        || ! timeout 600 python3 emu_run.py "$RS_ELF" $RS_DICT ../configs/$c "$s" $out/rs_$name.log > $out/rs_$name.txt 2>&1; then
        echo "ERROR $c $name (emu_run.py failed)"
        tail -n 3 $out/c_$name.txt $out/rs_$name.txt
        continue
    fi
    if diff -q $out/c_$name.txt $out/rs_$name.txt > /dev/null && diff -q $out/c_$name.log $out/rs_$name.log > /dev/null; then
        echo "SAME $c $name"
        same=$((same + 1))
    else
        echo "DIFF $c $name"
        diff $out/c_$name.txt $out/rs_$name.txt | head -20
        diff $out/c_$name.log $out/rs_$name.log | head -20
    fi
done
echo "$same/${#scripts[@]} identical"
