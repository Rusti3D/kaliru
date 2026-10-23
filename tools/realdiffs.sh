# Lists the suite runs (last suite_cmp.sh) that differ for a reason other
# than an unported module, with the start of their diff
# usage: realdiffs.sh [lines per run]
n=${1:-12}
for d in ~/rk/rs/*/*/; do
    r=${d#$HOME/rk/rs/}; r=${r%/}
    grep -q "not ported\|not supported here by Kaliru" "$d/log" 2>/dev/null && continue
    awk -v r="DIFF $r" '$0==r{p=1;print;next} /^DIFF /{p=0} p' ~/rk/rs.diff | head -$n
done
