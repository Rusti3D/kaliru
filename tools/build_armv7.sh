#!/bin/bash
# Builds kaliru for armv7 Linux hosts with glibc (the Centauri Carbon:
# glibc 2.43) into ~/rk/target-arm/armv7-unknown-linux-gnueabihf/release/.
# Linked by the gcc cross toolchain (apt gcc-arm-linux-gnueabihf), not zig:
# zig's linker swaps glibc's memcpy and sin / cos / exp / log ... for its own
# in an LTO build, which changes float results (NOTES "Kaliru on armv7").
# Run it with qemu: qemu-arm-static -L /usr/arm-linux-gnueabihf <binary>
cd "$(dirname "$0")/.."
CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=arm-linux-gnueabihf-gcc \
CARGO_TARGET_DIR=${CARGO_TARGET_DIR:-$HOME/rk/target-arm} \
    ~/.cargo/bin/cargo build --release --target armv7-unknown-linux-gnueabihf --bin kaliru "$@"
