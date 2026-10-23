cd ~/rk/renode/renode_1.15.3_portable
rm -f /tmp/renode_fn.log
grep -v "^start$" /mnt/d/RustyKlipper/kaliru/firmware/tests/renode/stm32f103.resc | sed 's/^logLevel 3$/logLevel 1/' > /tmp/dbg.resc
printf 'logFile @/tmp/renode_fn.log\nsysbus.cpu LogFunctionNames true false\nstart\n' >> /tmp/dbg.resc
(sleep 20; echo quit) | timeout 40 ./renode --disable-xwt --console -e '$elf=@/root/rk/fw_stm32f103-serial/klipper.elf; $uart="/tmp/uart_x"; include @/tmp/dbg.resc' > /dev/null 2>&1
grep -o "Entering function [^ ]*" /tmp/renode_fn.log | sort | uniq -c | sort -rn | head -20
grep -o "Entering function [^ ]*" /tmp/renode_fn.log | tail -8
