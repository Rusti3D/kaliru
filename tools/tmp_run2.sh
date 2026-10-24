cd /mnt/d/RustyKlipper/kaliru && . ~/.cargo/env
export KALIRU_KLIPPY_DIR=~/rk/kalico/klippy KALIRU_LIBPYTHON=$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0
echo "== nopy cases"; KALIRU_BIN=~/rk/target-nopy/release/kaliru bash tools/case_cmp.sh tests/cases/*.test 2>&1 | grep -v "^ \|^-\|^+" | tail -3
echo "== nopy suite"; KALIRU_BIN=~/rk/target-nopy/release/kaliru bash tools/suite_cmp.sh 2>&1 | tail -2
echo "== nopy plugins"; KALIRU_BIN=~/rk/target-nopy/release/kaliru bash tools/plugin_cmp.sh 2>&1 | tail -1
unset KALIRU_KLIPPY_DIR KALIRU_LIBPYTHON
echo "== default build"; CARGO_TARGET_DIR=~/rk/target cargo build --release 2>&1 | grep -E "^error" -A10; echo "build ${PIPESTATUS[0]}"
bash tools/case_cmp.sh tests/cases/pymacro*.test 2>&1 | tail -1
bash tools/plugin_cmp.sh 2>&1 | tail -1
