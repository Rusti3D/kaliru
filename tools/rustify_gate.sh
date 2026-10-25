#!/bin/bash
# Rustify's parity gate (docs/RUSTIFY.md): builds kaliru and runs every parity
# harness; each summary must equal tools/rustify_gate.expected (the known
# state: suite printers/13 is a timing case that differs in the reference
# run too). Exits non-zero on any difference. Takes ~40 min.
# usage: rustify_gate.sh
cd "$(dirname "$0")/.."
# (one gate run at a time across kaliru and Moonruster, which share this
# WSL: a resource measurement must not share the cpu with the other
# project's suites. The flock process holds the lock (-o: nothing this
# script starts inherits it, so a process left running can't keep it))
if [ -z "$RUSTIFY_GATE_LOCKED" ]; then
    RUSTIFY_GATE_LOCKED=1 exec flock -o /tmp/rustify_gates.lock bash "$PWD/tools/$(basename "$0")" "$@"
fi
out=~/rk/rustify_gate.out
{
    bash tools/check_all.sh 2>&1 | grep -E "^(test result: FAILED|error|FAIL|DIFF)|identical" | sort -u
    echo "clippy: $(CARGO_TARGET_DIR=~/rk/target-clippy ~/.cargo/bin/cargo clippy --release --all-targets --message-format short 2>&1 | grep -cE '^(src|tests)/.*(warning|error)') warnings"
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
