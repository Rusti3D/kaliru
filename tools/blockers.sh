# Counts the "not ported" errors in the last suite_cmp run (~/rk/rs logs)
# usage: blockers.sh
grep -hoE "Module '[a-z_0-9]+' is not ported|[A-Za-z ']+ is not ported[^\"]*|not supported here by Kaliru[^\"]*|are not ported" ~/rk/rs/*/*/log | sort | uniq -c | sort -rn | head -50
