# SAVE_CONFIG parity: each scenario runs configdump_ref.py --save and
# kaliru's configdump --save on a fresh copy of a config folder (same path)
# and compares what they print, including every file left in the folder.
TOOLS=$(dirname "$(readlink -f "$0")")
K=~/rk/kalico
W=~/rk/save_work
fail=0; total=0
run() {  # run <src dir> <cfg name> <autosave_includes> <changes...>
    src=$1; cfg=$2; shift 2
    out=()
    for side in py rust; do
        rm -rf $W; cp -r $src $W
        if [ $side = py ]; then
            (cd $K && PYTHONPATH=$K .venv/bin/python $TOOLS/configdump_ref.py --save $W/$cfg "$@") > ~/rk/save_$side.txt 2>&1
        else
            (cd $K && ~/rk/target/release/configdump --save $W/$cfg "$@") > ~/rk/save_$side.txt 2>&1
        fi
    done
    total=$((total+1))
    if ! cmp -s ~/rk/save_py.txt ~/rk/save_rust.txt; then
        echo "DIFF $src $cfg $*"; diff ~/rk/save_py.txt ~/rk/save_rust.txt | head -20; fail=$((fail+1))
    fi
}
run $K/test/test_configs/autosave printer.cfg 1 danger_options:temp_ignore_limits=False
run $TOOLS/config_cases autosave.cfg 0 extruder:pid_kp=30.1 "-bed_mesh default" "bed_mesh x:version=1"
run $TOOLS/config_cases autosave.cfg 1 extruder:rotation_distance=7.5
run $TOOLS/config_cases values.cfg 0
run $TOOLS/config_cases values.cfg 0 "new:opt=1" "-new"
run $TOOLS/config_cases include_glob.cfg 1 sub_b:k=changed after:k=2
run $TOOLS/save_cases/conflict printer.cfg 0 extruder:pid_ki=2
run $TOOLS/save_cases/conflict printer.cfg 1 extruder:pid_ki=2
run $TOOLS/save_cases/conflict printer.cfg 0 "-extruder"
echo "$((total-fail))/$total SAVE_CONFIG scenarios identical"
