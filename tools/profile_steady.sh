#!/bin/bash
# The steady-state profile (docs/RUSTIFY.md, "How to measure"): the resource
# gate's tests/perf/steady.live (every object subscribed, idle) under
# callgrind. Instruction counts are near-deterministic where cpu time spreads
# ~10 %; the build keeps symbols and turns function merging off (otherwise
# callgrind names one merged function for another's cost).
#   tools/profile_steady.sh [SECONDS]   (default 30: measured after a 20 s startup)
# Output: ~/rk/prof/callgrind.out; the top inclusive costs are printed.
cd "$(dirname "$0")/.."
# (shared WSL with Moonruster's gates: see rustify_gate.sh)
if [ -z "$RUSTIFY_GATE_LOCKED" ]; then
    RUSTIFY_GATE_LOCKED=1 exec flock -o /tmp/rustify_gates.lock bash "$PWD/tools/$(basename "$0")" "$@"
fi
set -e
SECS=${1:-30}
P=~/rk/prof
mkdir -p $P
rm -f $P/callgrind.out*
RUSTFLAGS="-Z merge-functions=disabled" CARGO_PROFILE_RELEASE_STRIP=false CARGO_PROFILE_RELEASE_DEBUG=line-tables-only \
    CARGO_TARGET_DIR=~/rk/target-prof ~/.cargo/bin/cargo +nightly build --release --bin kaliru -q
# (live_run kills the host: the wrapper dumps the data itself, for its own
# pid, which exec keeps for valgrind)
cat > $P/wrap.sh <<EOF
#!/bin/bash
(sleep 20; callgrind_control -i on \$\$ > /dev/null; sleep $SECS; callgrind_control -d \$\$ > /dev/null) &
exec valgrind --tool=callgrind --instr-atstart=no --callgrind-out-file=$P/callgrind.out \\
    $HOME/rk/target-prof/release/kaliru "\$@" 2> $P/valgrind.err
EOF
chmod +x $P/wrap.sh
LIVE_KALIRU=$P/wrap.sh python3 tools/live_run.py rs tests/perf/steady.live $P/run > /dev/null 2>&1 || true
f=$(ls -t $P/callgrind.out* | head -1)
callgrind_annotate --inclusive=yes "$f" 2> /dev/null | sed -n '1,70p'
