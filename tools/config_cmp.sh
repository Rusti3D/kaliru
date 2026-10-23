# configfile parity: runs tools/configdump_ref.py (Kalico's configfile) and
# kaliru's configdump on every config of the corpus, compares stdout+stderr.
# usage: config_cmp.sh [config files...]   (default: whole corpus)
TOOLS=$(dirname "$(readlink -f "$0")")
K=~/rk/kalico
if [ $# -gt 0 ]; then
    FILES="$@"
else
    FILES="$(ls $K/config/*.cfg $K/test/klippy/*.cfg $K/test/test_configs/*/printer.cfg $TOOLS/config_cases/*.cfg)"
fi
cd $K
fail=0; total=0
for f in $FILES; do
    PYTHONPATH=$K .venv/bin/python $TOOLS/configdump_ref.py $f > /tmp/cd_py.txt 2>&1
    ~/rk/target/release/configdump $f > /tmp/cd_rust.txt 2>&1
    if ! cmp -s /tmp/cd_py.txt /tmp/cd_rust.txt; then
        echo "DIFF $f"
        diff /tmp/cd_py.txt /tmp/cd_rust.txt | head -${DIFFLINES:-10}
        fail=$((fail+1))
    fi
    total=$((total+1))
done
echo "$((total-fail))/$total configs read identically"
