cd /mnt/d/RustyKlipper/kaliru
timeout 300 bash tools/case_cmp.sh tests/cases/chains.test
for d in ~/rk/case_py/chains/*/; do echo "== $d"; grep -h 'Error\|During\|error' $d/log | head -6; done
