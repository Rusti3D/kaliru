# Runs test cases with the Python reference and with kaliru, then diffs them
# (trace + exit + masked log). Build kaliru first (tools/build.sh).
# usage: case_cmp.sh <test files...>   (paths relative to kaliru/ or absolute)
cd "$(dirname "$0")/.."
tests=()
for t in "$@"; do tests+=("$(realpath "$t")"); done
rm -rf ~/rk/case_py ~/rk/case_rs
python3 tools/refrun.py --out ~/rk/case_py "${tests[@]}" < /dev/null | tail -1
python3 tools/refrun.py --out ~/rk/case_rs --klippy ~/rk/target/release/kaliru "${tests[@]}" < /dev/null | tail -1
python3 tools/refdiff.py ~/rk/case_py ~/rk/case_rs
