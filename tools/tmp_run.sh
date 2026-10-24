T=~/rk/elf2uf2-target/release/elf2uf2
for c in rp2040-usb rp2040-serial rp2040-usbcan rp2350-usb; do
  $T ~/rk/fw_$c/klipper.elf /tmp/r.uf2; if cmp -s /tmp/r.uf2 ~/rk/fw_$c/klipper.uf2; then echo "SAME uf2 of C elf: $c"; else echo "DIFF $c"; fi
done
$T ~/rk/fwtarget-ctr-rp2040-usb/thumbv6m-none-eabi/mcu/kaliru-mcu /tmp/k.uf2
~/rk/fw_rp2040-usb/lib/elf2uf2/elf2uf2 ~/rk/fwtarget-ctr-rp2040-usb/thumbv6m-none-eabi/mcu/kaliru-mcu /tmp/kc.uf2
if cmp -s /tmp/k.uf2 /tmp/kc.uf2; then echo "SAME uf2 of Rust elf (C tool vs Rust tool)"; else echo DIFF-rust-elf; fi
ls -la /tmp/k.uf2
