#!/bin/bash
# Rustify's resource gate (docs/RUSTIFY.md): kaliru's resource use on fixed
# workloads, compared with tools/resource_baseline.txt. Lower is better for
# every metric; one that grows beyond its noise tolerance fails the gate.
#   suite_*   the Kalico test suite (tools/refrun.py, every test): cpu time
#             summed over its runs, the largest peak RSS of one run
#   steady_*  tests/perf/steady.live (tools/live_run.py): a printer with every
#             object subscribed, idling 60 s; cpu time (ms, from schedstat),
#             peak RSS, own memory (anonymous PSS)
#   size_*    the binaries: x86 release (default features), and with ARMV7=1
#             the armv7 deploy build (no RustPython, as on a printer host)
# Each workload runs RUNS times (default 3); the smallest value counts.
# usage: resource_gate.sh             measure and compare
#        resource_gate.sh --baseline  measure and store as the baseline
# Build kaliru first (tools/build.sh).
cd "$(dirname "$0")/.."
B=${KALIRU_BIN:-$HOME/rk/target/release/kaliru}
RUNS=${RUNS:-3}
W=~/rk/resource_gate
rm -rf $W && mkdir -p $W
results=$W/results.txt
: > $results

for i in $(seq $RUNS); do
    t=$W/suite_time_$i.txt
    rm -rf $W/suite
    python3 tools/refrun.py --out $W/suite --klippy "/usr/bin/time -a -o $t -f %U,%S,%M $B" ~/rk/kalico/test/klippy/*.test \
        < /dev/null > /dev/null 2>&1
    awk -F, '/^[0-9.]+,[0-9.]+,[0-9]+$/ { cpu += $1 + $2; if ($3 > rss) rss = $3 } END { printf "suite_cpu_s %.2f\nsuite_peak_rss_kb %d\n", cpu, rss }' $t >> $results
    LIVE_KALIRU=$B python3 tools/live_run.py rs tests/perf/steady.live $W/steady > /dev/null 2>&1
    sed -E 's/.*peak_rss_kb=([0-9]+).*rss_anon_kb=([0-9]+).*pss_anon_kb=([0-9]+) cpu_s=[0-9.]+ cpu_ns=([0-9]+).*/steady_cpu_ms \4\nsteady_peak_rss_kb \1\nsteady_rss_anon_kb \2\nsteady_pss_anon_kb \3/' $W/steady/resources.txt \
        | awk '$1 == "steady_cpu_ms" { printf "%s %.1f\n", $1, $2 / 1e6; next } { print }' >> $results
done
echo "size_x86_bytes $(stat -c %s $B)" >> $results
if [ -n "$ARMV7" ]; then
    CARGO_TARGET_DIR=$HOME/rk/target-rg-arm bash tools/build_armv7.sh --no-default-features --features plugins > /dev/null 2>&1
    echo "size_armv7_bytes $(stat -c %s $HOME/rk/target-rg-arm/armv7-unknown-linux-gnueabihf/release/kaliru)" >> $results
fi

python3 - "$results" tools/resource_baseline.txt "$1" <<'EOF'
import sys
results, base_path, mode = sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else ""
vals = {}
for line in open(results):
    k, v = line.split()
    vals.setdefault(k, []).append(float(v))
cur = {k: min(v) for k, v in vals.items()}
spread = {k: (max(v) - min(v)) / min(v) if min(v) else 0.0 for k, v in vals.items()}
# noise tolerances (calibrated from repeated baseline runs: docs/RUSTIFY.md)
# (spread over 5 runs of the baseline, 2026-10-25: suite cpu 6.7 %, suite peak
# RSS 2.6 %, steady cpu 10.7 %, steady peak RSS 0.9 %, PSS 1.3 %; the minimum
# of the runs varies less than that. Peak RSS counts the program's file pages
# too, which move with the page cache: 11.0-11.4 MB over 5 runs of one
# build, the anonymous part constant; own memory is steady_rss_anon_kb /
# steady_pss_anon_kb, code memory the binary sizes)
tol = {"suite_cpu_s": 0.05, "suite_peak_rss_kb": 0.03, "steady_cpu_ms": 0.08, "steady_peak_rss_kb": 0.03,
       "steady_rss_anon_kb": 0.0,
       "steady_pss_anon_kb": 0.02, "size_x86_bytes": 0.0, "size_armv7_bytes": 0.0}
if mode == "--baseline":
    with open(base_path, "w") as f:
        for k, v in cur.items():
            f.write("%s %s\n" % (k, ("%d" % v) if v == int(v) else ("%.2f" % v)))
    for k, v in cur.items():
        print("%-20s %12.2f  (runs spread %.1f%%)" % (k, v, spread[k] * 100))
    print("baseline stored")
    sys.exit(0)
base = {l.split()[0]: float(l.split()[1]) for l in open(base_path)}
fail = False
for k, v in cur.items():
    if k not in base:
        print("%-20s %12.2f  (no baseline)" % (k, v))
        continue
    b = base[k]
    d = (v - b) / b if b else 0.0
    bad = d > tol.get(k, 0.0)
    fail |= bad
    print("%-20s %12.2f  base %12.2f  %+6.2f%%%s" % (k, v, b, d * 100, "  WORSE" if bad else ""))
print("resource gate: " + ("FAIL" if fail else "pass"))
sys.exit(1 if fail else 0)
EOF
