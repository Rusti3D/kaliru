cd /mnt/d/RustyKlipper/kaliru && . ~/.cargo/env
CARGO_TARGET_DIR=~/rk/target cargo build --release 2>&1 | grep -E "^error" -A10; bash tools/plugin_cmp.sh 2>&1 | tail -1; bash tools/case_cmp.sh tests/cases/pymacro*.test 2>&1 | tail -1
