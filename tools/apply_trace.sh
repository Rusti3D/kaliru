# Applies ref_trace.patch to the WSL reference checkout (~/rk/kalico) and
# rebuilds c_helper.so. Safe to rerun.
set -e
cd ~/rk/kalico
if ! grep -q rk_trace_id klippy/chelper/serialqueue.c; then
    git apply --recount "$(dirname "$(readlink -f "$0")")/ref_trace.patch"
fi
git diff --stat
.venv/bin/python -c "import klippy.chelper as c; c.get_ffi(); print('c_helper ok')"
