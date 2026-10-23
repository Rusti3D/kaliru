# Decodes every traced message of a refrun tree with the Rust msgproto
# (tracedump) and with Kalico's msgproto (tracedump_ref.py); outputs must be
# identical and nothing may fail to decode. usage: trace_check.sh [~/rk/ref1]
RUNS=${1:-~/rk/ref1}
TOOLS=$(dirname "$(readlink -f "$0")")
fail=0; total=0
cd ~/rk/kalico
for t in $RUNS/*/*/trace; do
    d=$(dirname $t)
    # cmd: "dicts <main> [name=path...]" -> mcu=<main> name=path...
    set -- $(sed -n 's/^dicts //p' $d/cmd)
    main=$1; shift
    ~/rk/target/release/tracedump --check $t mcu=$main "$@" > /tmp/td_rust.txt 2> /tmp/td_rust.err
    rc=$?
    PYTHONPATH=~/rk/kalico .venv/bin/python $TOOLS/tracedump_ref.py $t mcu=$main "$@" > /tmp/td_py.txt
    if [ $rc != 0 ] || ! cmp -s /tmp/td_rust.txt /tmp/td_py.txt; then
        echo "FAIL $d (rust rc=$rc: $(cat /tmp/td_rust.err))"
        diff /tmp/td_py.txt /tmp/td_rust.txt | head -6
        fail=$((fail+1))
    fi
    total=$((total+1))
done
echo "$((total-fail))/$total traces decode identically ($(cat $RUNS/*/*/trace | wc -l) messages)"
