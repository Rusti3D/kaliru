# queuelogger parity: computeRollover() for timestamps around the 2026 DST
# changes and ordinary times, in two time zones, python vs kaliru.
TOOLS=$(dirname "$(readlink -f "$0")")
D=~/rk/qltest; rm -rf $D; mkdir -p $D
# 2026-03-29 01:00 UTC (EU DST start), 2026-10-25 01:00 UTC (EU DST end), +-1h/2h, midnight edges
TS="1774746000 1774742400 1774749600 1774738800 1774753200 1792890000 1792886400 1792893600 1792882800 1792897200 1792800000 1792799999 1792886399 1792836000 1700000000"
fail=0
for tz in Europe/Copenhagen UTC America/New_York; do
    TZ=$tz ~/rk/kalico/.venv/bin/python $TOOLS/qltest_ref.py $D $TS > $D/py.txt
    TZ=$tz ~/rk/target/release/qltest $D $TS > $D/rust.txt
    if cmp -s $D/py.txt $D/rust.txt; then echo "identical: $tz ($(wc -l < $D/py.txt) cases)"; else echo "DIFF $tz"; diff $D/py.txt $D/rust.txt; fail=1; fi
done
exit $fail
