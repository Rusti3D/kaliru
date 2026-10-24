#!/bin/bash
# Webhooks / API parity: tests/live_api/*.live (or the given ones) through
# live_cmp.sh, with the test plugin (tests/plugins) in Kalico's klippy/plugins
# for the run and kaliru pointed at the same klippy directory (as
# plugin_cmp.sh does).
cd "$(dirname "$0")/.."
K=~/rk/kalico/klippy
installed=()
for f in tests/plugins/*.py; do cp "$f" $K/plugins/; installed+=("$K/plugins/$(basename "$f")"); done
trap 'rm -rf "${installed[@]}"' EXIT
export KALIRU_KLIPPY_DIR=$K
export KALIRU_LIBPYTHON=${KALIRU_LIBPYTHON:-$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0}
[ $# -eq 0 ] && set -- tests/live_api/*.live
bash tools/live_cmp.sh "$@"
