cd /mnt/d/RustyKlipper/kaliru/firmware
K=~/rk/kalico/lib
G="python3 tools/gen_stm32_regs.py --cmsis $K"
$G sam3x sam3x/include/sam3xa.h src/atsam/chip/sam3x8e.rs __SAM3X8E__ | tail -1
$G sam3x sam3x/include/sam3xa.h src/atsam/chip/sam3x8c.rs __SAM3X8C__ | tail -1
$G sam4s sam4s/include/sam4s.h src/atsam/chip/sam4s8c.rs __SAM4S8C__ | tail -1
$G sam4e sam4e/include/sam4e.h src/atsam/chip/sam4e8e.rs __SAM4E8E__ | tail -1
$G same70b same70b/include/sam.h src/atsam/chip/same70q20b.rs __SAME70Q20B__ | tail -1
for m in samd21g18a samd21e18a samd21j18a samd21e15a; do $G samd21 samd21/samd21a/include/sam.h src/atsamd/chip/$m.rs __$(echo $m | tr a-z A-Z)__ | tail -1; done
$G samc21 samc21/samc21/include/sam.h src/atsamd/chip/samc21g18a.rs __SAMC21G18A__ | tail -1
for m in samd51g19a samd51j19a samd51n19a samd51p20a; do $G samd51 samd51/samd51a/include/sam.h src/atsamd/chip/$m.rs __$(echo $m | tr a-z A-Z)__ same54/include | tail -1; done
for m in same51g19a same51j19a same51n19a; do $G same51 same51/include/sam.h src/atsamd/chip/$m.rs __$(echo $m | tr a-z A-Z)__ same54/include | tail -1; done
$G same54 same54/include/sam.h src/atsamd/chip/same54p20a.rs __SAME54P20A__ | tail -1
