# Lists the first "not ported" blocker of each test in the last suite_cmp run
# and the tests that differ without one
# usage: pertest.sh
cd ~/rk/rs || exit 1
pat="Module '[a-z_0-9]+' is not ported|[A-Za-z_ ']+ is not ported[^\"]*|not supported here by Kaliru[^\"]*"
for t in $(grep '^DIFF' ~/rk/rs.diff | awk '{print $2}' | cut -d/ -f1 | sort -u); do
    m=$(grep -hoE "$pat" $t/*/log 2>/dev/null | head -1)
    echo "$t: ${m:-<no blocker>}"
done
