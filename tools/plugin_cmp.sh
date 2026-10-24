# Python plugin parity: runs tests/cases/plugins/*.test (or the given ones)
# with Kalico and with kaliru. The test plugins (tests/plugins/*.py) are
# copied into the reference's klippy/plugins (git-ignored there) for the
# run, and kaliru is pointed at that klippy directory.
# Needs CPython 3.15+ with a shared libpython: KALIRU_LIBPYTHON (default: the
# uv build in ~/.local/share/uv/python).
# usage: plugin_cmp.sh [test files...]
cd "$(dirname "$0")/.."
K=~/rk/kalico/klippy
cp tests/plugins/*.py $K/plugins/
trap 'for f in tests/plugins/*.py; do rm -f "$K/plugins/$(basename "$f")"; done' EXIT
export KALIRU_KLIPPY_DIR=$K
export KALIRU_LIBPYTHON=${KALIRU_LIBPYTHON:-$HOME/.local/share/uv/python/cpython-3.15-linux-x86_64-gnu/lib/libpython3.15.so.1.0}
[ $# -eq 0 ] && set -- tests/cases/plugins/*.test
bash tools/case_cmp.sh "$@"
