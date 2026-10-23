# serialqueue parity: scripted pty exchange through Kalico's C serialqueue
# (sqtest_ref.py) and kaliru's (sqtest), 3 runs each.
TOOLS=$(dirname "$(readlink -f "$0")")
K=~/rk/kalico
cd $K
fail=0
for i in 1 2 3; do
    PYTHONPATH=$K .venv/bin/python $TOOLS/sqtest_ref.py > ~/rk/sq_py.txt 2>&1
    ~/rk/target/release/sqtest > ~/rk/sq_rust.txt 2>&1
    if ! cmp -s ~/rk/sq_py.txt ~/rk/sq_rust.txt; then
        echo "DIFF run $i"; diff ~/rk/sq_py.txt ~/rk/sq_rust.txt; fail=1
    fi
done
cat ~/rk/sq_py.txt
[ $fail = 0 ] && echo "serialqueue exchange identical (3 runs)"
exit $fail
