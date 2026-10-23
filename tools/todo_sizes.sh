# Lists the extras modules still marked todo() in src/extras/mod.rs with the
# line count of their python source (a package counts all its files)
# usage: todo_sizes.sh
cd "$(dirname "$0")/.." || exit 1
ext=~/rk/kalico/klippy/extras
for m in $(grep -o 'todo("[a-z_0-9]*")' src/extras/mod.rs | sed 's/todo("//;s/")//'); do
    if [ -f "$ext/$m.py" ]; then
        echo "$m $(wc -l < "$ext/$m.py")"
    else
        echo "$m $(cat "$ext/$m"/*.py 2>/dev/null | wc -l) (pkg)"
    fi
done | sort -k2 -n
