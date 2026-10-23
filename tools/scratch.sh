# wall time of one printers run: python reference vs kaliru
cfg=~/rk/kalico/config/printer-artillery-sidewinder-x2-2022.cfg
gc=~/rk/kalico/test/klippy/move.gcode
d=~/rk/ci_build/dict/stm32f401.dict
cd ~/rk/kalico
/usr/bin/time -f "python %e s" .venv/bin/python -m klippy $cfg -i $gc -o /tmp/p.out -v -d $d > /dev/null 2>/tmp/p.err; tail -1 /tmp/p.err
/usr/bin/time -f "kaliru %e s" ~/rk/target/release/kaliru $cfg -i $gc -o /tmp/k.out -v -d $d > /dev/null 2>/tmp/k.err; tail -1 /tmp/k.err
