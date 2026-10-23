#!/bin/bash
# Sets up the WSL reference environment the way Kalico's CI image does
# (Originals_Base/kalico/scripts/Dockerfile-build), then runs Kalico's own
# scripts/ci-build.sh: builds every test/configs/*.config dictionary and runs
# the py.test suite. Log: /tmp/rk_setup.out
# usage (detached): nohup setsid bash setup_env.sh < /dev/null > /tmp/rk_setup.out 2>&1 &
set -eux
KALICO_REV=5b5942799ba33e50cf2b67b5258acff7214b3001
OR1K_ARCHIVE=or1k-elf-12.0.1-20220210-20220304.tar.xz
OR1K_RELEASE=or1k-12.0.1-20220210-20220304
RK=~/rk

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y git curl python3 python3-dev python3-venv libffi-dev build-essential \
    gcc-avr avr-libc libnewlib-arm-none-eabi gcc-arm-none-eabi binutils-arm-none-eabi

# ci-build.sh expects its tools under /ci_build
mkdir -p $RK/ci_build
ln -sfn $RK/ci_build /ci_build
if [ ! -d /ci_build/or1k-elf ]; then
    curl -L https://github.com/openrisc/or1k-gcc/releases/download/$OR1K_RELEASE/$OR1K_ARCHIVE \
        -o $RK/$OR1K_ARCHIVE
    tar -xJ -C /ci_build -f $RK/$OR1K_ARCHIVE
fi

# uv (the image copies it from ghcr.io/astral-sh/uv)
if [ ! -x $RK/uvenv/bin/uv ]; then
    python3 -m venv $RK/uvenv
    $RK/uvenv/bin/pip install uv
fi
export PATH=$RK/uvenv/bin:$PATH

# Reference checkout, same commit as Originals_Base/kalico
if [ ! -d $RK/kalico ]; then
    git clone https://github.com/KalicoCrew/kalico.git $RK/kalico
fi
cd $RK/kalico
git -c advice.detachedHead=false checkout $KALICO_REV

export DICTDIR=/ci_build/dict
uv run ./scripts/ci-build.sh
echo SETUP_DONE
