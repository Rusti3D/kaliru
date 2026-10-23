# reactor parity: event order of the scenarios in reactortest_ref.py (Kalico's
# reactor) vs kaliru's reactortest. Runs each side 3 times to catch flakiness.
TOOLS=$(dirname "$(readlink -f "$0")")
K=~/rk/kalico
cd $K
fail=0
for i in 1 2 3; do
    PYTHONPATH=$K .venv/bin/python $TOOLS/reactortest_ref.py > ~/rk/reactor_py.txt 2>&1
    ${REACTORTEST:-~/rk/target/release/reactortest} > ~/rk/reactor_rust.txt 2>&1
    if ! cmp -s ~/rk/reactor_py.txt ~/rk/reactor_rust.txt; then
        echo "DIFF run $i"; diff ~/rk/reactor_py.txt ~/rk/reactor_rust.txt; fail=1
    fi
done
cat ~/rk/reactor_py.txt
[ $fail = 0 ] && echo "reactor scenarios identical (3 runs)"
exit $fail
