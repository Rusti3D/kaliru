#!/bin/bash
# Runs each script of tests/scripts on the C build (~/rk/fw_<config>/klipper.elf,
# tools/mkref.sh) and the Rust build (~/rk/fwtarget-ctr-<config>, ctr_check.sh)
# of a micro-controller config in the emulator (emu_run.py) and diffs the
# answers and the peripheral register accesses. A register access difference
# recorded in emu_known/<config>/<script>.diff (see emu_known/README.md) is
# expected.
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
    # (the answers' and the register accesses' differences: none, or the
    # known ones)
    diff $out/c_$name.txt $out/rs_$name.txt > $out/$name.txt.diff
    diff $out/c_$name.log $out/rs_$name.log > $out/$name.diff
    known=
    bad=
    for d in $name.txt.diff $name.diff; do
        [ -s $out/$d ] || continue
        if [ -f emu_known/$c/$d ] && diff -q $out/$d emu_known/$c/$d > /dev/null; then
            known="$known emu_known/$c/$d"
        else
            bad="$bad $d"
        fi
    done
    # (no answer to a command: the board model never carried the session)
    if ! grep -q "^< ack" $out/c_$name.txt; then
        echo "NOANSWER $c $name (the model carried no command; see $out/c_$name.log)"
    elif [ -n "$bad" ]; then
        echo "DIFF $c $name"
        for d in $bad; do head -20 $out/$d; done
    else
        echo "SAME $c $name${known:+ (known differences:$known)}"
        same=$((same + 1))
    fi
done
echo "$same/${#scripts[@]} identical"
