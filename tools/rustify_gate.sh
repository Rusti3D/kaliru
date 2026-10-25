#!/bin/bash
# Rustify's parity gate (docs/RUSTIFY.md): builds kaliru and runs every parity
# harness; each summary must equal tools/rustify_gate.expected (the known
# state: suite printers/13 is a timing case that differs in the reference
# run too). Exits non-zero on any difference. Takes ~40 min.
# usage: rustify_gate.sh
cd "$(dirname "$0")/.."
out=~/rk/rustify_gate.out
{
    bash tools/check_all.sh 2>&1 | grep -E "^(test result: FAILED|error|FAIL|DIFF)|identical" | sort -u
    echo "== cases";   bash tools/case_cmp.sh tests/cases/*.test 2>&1 | tail -1
    echo "== suite";   bash tools/suite_cmp.sh 2>&1 | tail -1
    grep "^DIFF" ~/rk/rs.diff | sort -u
    echo "== plugins"; bash tools/plugin_cmp.sh 2>&1 | tail -1
    echo "== live";    bash tools/live_cmp.sh 2>&1 | tail -1
    echo "== api";     bash tools/api_cmp.sh 2>&1 | tail -1
} > $out
# (the counts in the "info: raw serial files differ" notes vary run to run)
sed -i -E 's/ \(info: [0-9]+ raw serial files differ\)//' $out
if diff -u tools/rustify_gate.expected $out; then
    echo "rustify gate: pass"
else
    echo "rustify gate: FAIL"
    exit 1
fi
