#!/bin/bash
# Live parity: runs each scenario (tests/live/*.live, or the given ones) with
# Kalico's python klippy and with kaliru on the same linux MCU processes
# (tools/live_run.py) and diffs the transcripts. Needs ~/rk/kalico (venv),
# ~/rk/target/release/kaliru, ~/rk/fwlinux/klipper.elf and the fakehw shim
# (firmware/tests/mcu_cmp.sh builds it).
# usage: live_cmp.sh [scenario...]
cd "$(dirname "$0")/.."
scenarios=("$@")
[ ${#scenarios[@]} -eq 0 ] && scenarios=(tests/live/*.live)
out=~/rk/live
mkdir -p $out
same=0
for s in "${scenarios[@]}"; do
    name=$(basename "$s" .live)
    for h in py rs; do
        # (one work directory for both: the same paths in the transcripts)
        timeout 600 python3 tools/live_run.py $h "$s" $out/$name > $out/$name-$h.txt 2>&1
    done
    if diff $out/$name-py.txt $out/$name-rs.txt > $out/$name.diff; then
        echo "SAME $name"
        same=$((same + 1))
    else
        echo "DIFF $name"
        head -30 $out/$name.diff
    fi
done
echo "$same/${#scenarios[@]} identical"
