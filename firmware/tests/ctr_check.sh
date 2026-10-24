#!/bin/bash
# Build kaliru-mcu for every configuration in configs/ (or the ones named)
# and compare its compile_time_request.txt and data dictionary with the C
# build's in tests/ref/ (the dictionary without version / build_versions).
#   tests/ctr_check.sh [config ...]
cd "$(dirname "$0")/.."
CARGO=${CARGO:-~/.cargo/bin/cargo}
configs="$*"
[ -z "$configs" ] && configs=$(ls configs | grep -v '^src$')
pass=0; total=0
for c in $configs; do
    total=$((total + 1))
    a=configs/$c/autoconf.h
    # (avr: a tier 3 target, its core built by a nightly cargo; optimised
    # for size further, -Oz, to fit the smaller flashes)
    nightly=; buildstd=; rustflags=; optlevel=s
    if grep -q "CONFIG_MACH_AVR 1" $a; then
        target=avr-none; tdir=$target/mcu; profile="--profile mcu"
        nightly=+nightly; buildstd="-Z build-std=core"; optlevel=z
        rustflags="-C target-cpu=$(sed -n 's/^#define CONFIG_MCU "\(.*\)"/\1/p' $a)"
    elif grep -q "CONFIG_MACH_LINUX 1\|CONFIG_MACH_SIMU 1" $a; then
        target=""; tdir=release; profile=--release
    elif grep -q "CONFIG_MACH_STM32F4 1\|CONFIG_MACH_STM32F7 1\|CONFIG_MACH_STM32G4 1\|CONFIG_MACH_STM32L4 1\|CONFIG_MACH_STM32H7 1\|CONFIG_MACH_N32G45x 1\|CONFIG_MACH_SAM4 1\|CONFIG_MACH_SAME70 1\|CONFIG_MACH_SAMX5 1\|CONFIG_MACH_HC32F460 1" $a; then
        target=thumbv7em-none-eabi; tdir=$target/mcu; profile="--profile mcu"
    elif grep -q "CONFIG_MACH_RP2350 1" $a; then
        target=thumbv8m.main-none-eabi; tdir=$target/mcu; profile="--profile mcu"
    elif grep -q "CONFIG_MACH_STM32F0 1\|CONFIG_MACH_STM32G0 1\|CONFIG_MACH_RP2040 1\|CONFIG_MACH_SAMX2 1" $a; then
        target=thumbv6m-none-eabi; tdir=$target/mcu; profile="--profile mcu"
    else
        target=thumbv7m-none-eabi; tdir=$target/mcu; profile="--profile mcu"
    fi
    dir=~/rk/fwtarget-ctr-$c
    log=$(KALIRU_MCU_CONFIG=configs/$c CARGO_TARGET_DIR=$dir RUSTFLAGS="$rustflags" CARGO_PROFILE_MCU_OPT_LEVEL=$optlevel $CARGO $nightly build $profile $buildstd ${target:+--target $target} 2>&1)
    status=$?
    size=
    if [ $status -ne 0 ]; then
        # (an image too big for the chip's flash: its requests and dictionary
        # are still made, before the link)
        if ! echo "$log" | grep -q "region .text. overflowed"; then
            echo "FAIL $c: build"; echo "$log" | grep -E "^error|panicked" -A6 | head -20; continue
        fi
        size=" (but the image does not fit: $(echo "$log" | grep -o "overflowed by [0-9]* bytes" | head -1))"
    fi
    # (a nightly cargo: build/kaliru-mcu/<hash>/out)
    out=$(ls -td $dir/$tdir/build/kaliru-mcu-*/out $dir/$tdir/build/kaliru-mcu/*/out 2>/dev/null | while read d; do [ -f $d/compile_time_request.txt ] && echo $d && break; done)
    if ! diff -q $out/compile_time_request.txt tests/ref/${c}_compile_time_request.txt > /dev/null; then
        echo "FAIL $c: compile_time_request.txt"; diff $out/compile_time_request.txt tests/ref/${c}_compile_time_request.txt | head; continue
    fi
    if ! python3 - $out/klipper.dict tests/ref/${c}_klipper.dict <<'EOF'
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
for d in (a, b):
    d.pop("version", None)
    d.pop("build_versions", None)
sys.exit(a != b)
EOF
    then
        echo "FAIL $c: dictionary"; continue
    fi
    echo "SAME $c$size"; pass=$((pass + 1))
done
echo "$pass/$total identical"
