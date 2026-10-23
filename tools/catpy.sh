# Prints Kalico python sources (extras module names) with a header each
# usage: catpy.sh name...
ext=~/rk/kalico/klippy/extras
for m in "$@"; do
    echo "=== $m"
    cat "$ext/$m.py"
done
