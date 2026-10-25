# KALIRU_BIN="qemu-arm-static <armv7 build>" runs another build.
# Runs the stock Kalico test cases (or the given ones) with kaliru into
# ~/rk/rs and diffs them against the Python reference run ~/rk/ref1
# (made by run_ref.sh ref1). Build kaliru first (tools/build.sh).
# The full diff goes to ~/rk/rs.diff; the summary is printed.
# usage: suite_cmp.sh [test names without .test...]
cd "$(dirname "$0")"
# python macros and plugins run on the host's CPython 3.15 (kaliru has no
# embedded python): Kalico's klippy directory and the uv build's libpython,
# as plugin_cmp.sh
export KALIRU_KLIPPY_DIR=${KALIRU_KLIPPY_DIR:-$HOME/rk/kalico/klippy}
export KALIRU_LIBPYTHON=${KALIRU_LIBPYTHON:-$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0}
tests=()
for t in "$@"; do tests+=(~/rk/kalico/test/klippy/$t.test); done
rm -rf ~/rk/rs
python3 refrun.py --out ~/rk/rs --klippy "${KALIRU_BIN:-$HOME/rk/target/release/kaliru}" "${tests[@]}" < /dev/null > ~/rk/rs.out 2>&1
tail -1 ~/rk/rs.out
if [ ${#tests[@]} -eq 0 ]; then
    python3 refdiff.py ~/rk/ref1 ~/rk/rs > ~/rk/rs.diff 2>&1
else
    # only the selected tests
    mkdir -p ~/rk/ref_sel && rm -rf ~/rk/ref_sel/*
    for t in "$@"; do cp -r ~/rk/ref1/$t ~/rk/ref_sel/; done
    python3 refdiff.py ~/rk/ref_sel ~/rk/rs > ~/rk/rs.diff 2>&1
fi
grep -E "^(DIFF|SAME|ONLY)" ~/rk/rs.diff | awk '{print $1}' | sort | uniq -c
tail -1 ~/rk/rs.diff
