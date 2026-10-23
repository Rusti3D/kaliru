# Cross-builds kaliru's unit tests for the printer architectures and runs them
# under qemu-user. usage: cross_test.sh [target...]
# needs: rustup targets, cargo-zigbuild, zig (Moonruster's ~/mr/venv has it), qemu-user-static
cd "$(dirname "$0")/.."
export PATH=~/mr/venv/bin:~/.cargo/bin:$PATH
export CARGO_TARGET_DIR=~/rk/target_cross
TARGETS=${@:-armv7-unknown-linux-musleabihf aarch64-unknown-linux-musl arm-unknown-linux-musleabihf}
for t in $TARGETS; do
    case $t in aarch64-*) q=qemu-aarch64-static;; *) q=qemu-arm-static;; esac
    echo "### $t"
    cargo zigbuild --release --target $t --tests 2>&1 | grep -E "^(error|warning)" -A6 | head -30
    for b in $(ls -t $CARGO_TARGET_DIR/$t/release/deps/kaliru-* | grep -v '\.d$' | head -1); do
        $q $b 2>&1 | grep -E "test result|FAILED|panicked|error|Segmentation" | head -10
    done
done
