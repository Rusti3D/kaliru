~/rk/kalico/.venv/bin/python /mnt/d/RustyKlipper/kaliru/tools/scratch.py > ~/rk/hypot_cases.txt
cd /mnt/d/RustyKlipper/kaliru && export CARGO_TARGET_DIR=~/rk/target
RK_HYPOT_CASES=~/rk/hypot_cases.txt ~/.cargo/bin/cargo test --release --lib hypot 2>&1 | tail -15
