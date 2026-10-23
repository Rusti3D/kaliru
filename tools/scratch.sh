cd ~/rk/rs
for t in dual_carriage hybrid_corexy_dual_carriage limited_cartesian polar rotary_delta_calibrate linuxtest temperature; do
    echo "== $t"
    grep -hE 'rror|ported|Traceback|panick' $t/*/log | head -4
done
