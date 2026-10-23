# Shows what the reference run of each custom case did: exit code, traced
# message count and the last log lines (after tools/case_cmp.sh)
# usage: case_show.sh <case name...>
for t in "$@"; do
    d=~/rk/case_py/$t/0
    echo "=== $t: exit $(cat "$d/exit" 2>/dev/null), $(grep -c serialq "$d/trace" 2>/dev/null) traced messages"
    grep -v 'DEBUG:\|Stats\|^MCU\|Loaded MCU\|^Start printer\|^=====\|Starting Klippy' "$d/log" | tail -12
done
