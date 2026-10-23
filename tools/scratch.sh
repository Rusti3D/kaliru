cd /mnt/d/RustyKlipper/kaliru
export CARGO_TARGET_DIR=~/rk/target
touch src/extras/probe.rs
~/.cargo/bin/cargo build --release 2>&1 | tail -5
ls -la ~/rk/target/release/kaliru
