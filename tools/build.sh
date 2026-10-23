# Builds kaliru (release) and runs its unit tests, target dir ~/rk/target
# usage: build.sh [test]
cd "$(dirname "$0")/.."
export CARGO_TARGET_DIR=~/rk/target
~/.cargo/bin/cargo build --release 2>&1 | grep -E "^(warning|error)" -A7 | head -80
if [ "$1" = test ]; then
    ~/.cargo/bin/cargo test --release 2>&1 | grep -E "test result|FAILED|panicked|error" | head -40
fi
ls -la ~/rk/target/release/parsedump 2>/dev/null
