# pins parity: tools/pins_cases.txt plus ops generated from every *pin* option
# of the config corpus, run through pinsdump_ref.py (Kalico) and pinsdump.
TOOLS=$(dirname "$(readlink -f "$0")")
K=~/rk/kalico
OPS=~/rk/pins_corpus.txt
# per config: fresh pins, chips mcu/probe + every [mcu X], then each pin value
# looked up (shared, so repeats error like a real config) and parsed 4 ways
for f in $K/config/*.cfg $K/test/klippy/*.cfg; do
    echo new; echo chip mcu; echo chip probe
    sed -n 's/^\[mcu \(.*\)\]/chip \1/p' $f
    sed -n 's/^\([a-z_]*pin[a-z_]*\) *[:=] *\([^#;]*\)/\2/p' $f | while read -r v; do
        echo "lookup 1 1 - $v"; for m in "0 0" "1 0" "0 1" "1 1"; do echo "parse $m $v"; done
    done
done > $OPS
cd $K
fail=0
for s in $TOOLS/pins_cases.txt $OPS; do
    PYTHONPATH=$K .venv/bin/python $TOOLS/pinsdump_ref.py $s > ~/rk/pins_py.txt 2>&1
    ~/rk/target/release/pinsdump $s > ~/rk/pins_rust.txt 2>&1
    if cmp -s ~/rk/pins_py.txt ~/rk/pins_rust.txt; then
        echo "identical: $s ($(wc -l < ~/rk/pins_py.txt) ops)"
    else
        echo "DIFF $s"; diff ~/rk/pins_py.txt ~/rk/pins_rust.txt | head -20; fail=1
    fi
done
exit $fail
