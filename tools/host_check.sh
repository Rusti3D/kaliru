cd /mnt/d/RustyKlipper/kaliru
CARGO_TARGET_DIR=~/rk/target ~/.cargo/bin/cargo build --release 2>&1 | grep -E '^error' -A14 | head -30
bash tools/case_cmp.sh tests/cases/*.test 2>&1 | tail -4
