# KALIRU_BIN="qemu-arm-static <armv7 build>" runs another build.
# Runs test cases with the Python reference and with kaliru, then diffs them
# (trace + exit + masked log). Build kaliru first (tools/build.sh).
# usage: case_cmp.sh <test files...>   (paths relative to kaliru/ or absolute)
# REF_KLIPPY="~/rk/scipyenv/bin/python -m klippy" picks another reference
# interpreter (tests/cases/scipy/lcp_filters.test needs scipy)
cd "$(dirname "$0")/.."
# python macros and plugins run on the host's CPython 3.15 (kaliru has no
# embedded python): Kalico's klippy directory and the uv build's libpython,
# as plugin_cmp.sh
export KALIRU_KLIPPY_DIR=${KALIRU_KLIPPY_DIR:-$HOME/rk/kalico/klippy}
export KALIRU_LIBPYTHON=${KALIRU_LIBPYTHON:-$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0}
tests=()
for t in "$@"; do tests+=("$(realpath "$t")"); done
rm -rf ~/rk/case_py ~/rk/case_rs
python3 tools/refrun.py --out ~/rk/case_py ${REF_KLIPPY:+--klippy "$REF_KLIPPY"} "${tests[@]}" < /dev/null | tail -1
python3 tools/refrun.py --out ~/rk/case_rs --klippy "${KALIRU_BIN:-$HOME/rk/target/release/kaliru}" "${tests[@]}" < /dev/null | tail -1
python3 tools/refdiff.py ~/rk/case_py ~/rk/case_rs
