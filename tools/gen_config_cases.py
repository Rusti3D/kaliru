#!/usr/bin/env python3
# Writes tests/cases/configs/examples.test: every Kalico config/*.cfg that
# test/klippy/printers.test does not load, with the mcu dictionary its header
# names ("compiled for the STM32F103"), running Kalico's move.gcode. Configs
# with no known mcu are listed as skipped (an extra [mcu NAME] gets the
# linux host build or the main board's dictionary). A
# config missing options every printer must set (virtual_sdcard path, tmc
# sense_resistor) runs through a wrapper (tests/cases/configs/wrap/) that
# includes it and adds them.
#
# usage: gen_config_cases.py KALICO_DIR [DICT_DIR]
import pathlib
import re
import sys

kalico = pathlib.Path(sys.argv[1])
dicts = {p.stem.lower(): p.name for p in pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else "/root/rk/ci_build/dict").glob("*.dict")}
covered = set(re.findall(r"^CONFIG \S*/([^/\s]+)$", (kalico / "test/klippy/printers.test").read_text(), re.M))
# header names -> dictionary stem (longest first)
NAMES = [
    ("atmega2560", "atmega2560"), ("atmega1280", "atmega1280"), ("atmega1284p", "atmega1284p"),
    ("atmega644p", "atmega644p"), ("at90usb1286", "at90usb1286"), ("sam3x8c", "sam3x8c"),
    ("sam3x8e", "sam3x8e"), ("sam4s8c", "sam4s8c"), ("sam4e8e", "sam4e8e"), ("samd21g18", "samd21g18"),
    ("samd51p20", "samd51p20"), ("same70q20b", "same70q20b"), ("stm32f103", "stm32f103"),
    ("stm32f070", "stm32f070"), ("stm32f072", "stm32f072"), ("stm32f031", "stm32f031"), ("stm32f042", "stm32f042"),
    ("stm32f401", "stm32f401"), ("stm32f405", "stm32f405"), ("stm32f407", "stm32f407"), ("stm32f411", "stm32f411"),
    ("stm32f429", "stm32f429"), ("stm32f446", "stm32f446"), ("stm32f765", "stm32f765"), ("stm32g0b1", "stm32g0b1"),
    ("stm32g431", "stm32g431"), ("stm32g474", "stm32g474"), ("stm32h723", "stm32h723"), ("stm32h743", "stm32h743"),
    ("stm32l412", "stm32l412"), ("rp2040", "rp2040"), ("rp2350", "rp2350"), ("at32f403", "at32f403"),
    ("lpc1768", "lpc1768"), ("lpc1769", "lpc1769"), ("lpc176x", "lpc1769"), ("same54p20", "same54p20"),
    ("hc32f460", "hc32f460-serial-pa3pa2"), ("arduino # due", "sam3x8e"), ("arduino due", "sam3x8e"),
]
dest_dir = pathlib.Path(__file__).resolve().parent.parent / "tests/cases/configs"
groups, skipped = {}, []
for cfg in sorted((kalico / "config").glob("*.cfg")):
    if cfg.name in covered:
        continue
    text = cfg.read_text(errors="replace")
    head = " ".join(l for l in text.splitlines()[:40] if l.startswith("#")).lower()
    found = next((d for n, d in NAMES if n in head and d in dicts), None)
    # configs whose header names no mcu: the boards their text describes
    found = found or {"kit-voron2-250mm.cfg": "atmega2560"}.get(cfg.name)  # dual RAMPS
    # (the kinematics examples run on an atmega2560, as in printers.test)
    if found is None and cfg.name.startswith("example-"):
        found = "atmega2560"
    if found is None:
        skipped.append((cfg.name, "no known mcu in the header"))
        continue
    # a board using the usb pins for something else needs the serial build
    if found + "-serial" in dicts and re.search(r"\bPA1[12]\b", text):
        found += "-serial"
    fixes = []
    # (also where another module loads virtual_sdcard: the lcd menu)
    if not re.search(r"^\[virtual_sdcard\][^\[]*^path\s*[:=]", text, re.M | re.S):
        fixes.append("[virtual_sdcard]\npath: /tmp/kaliru_vsd\n")
    for sec in re.findall(r"^\[(tmc\w+ [^\]]+)\]", text, re.M):
        body = re.search(r"^\[" + re.escape(sec) + r"\](.*?)(?=^\[|\Z)", text, re.M | re.S).group(1)
        if not re.search(r"^sense_resistor\s*[:=]", body, re.M):
            fixes.append(f"[{sec}]\nsense_resistor: 0.110\n")
    name = cfg.name
    if fixes:
        wrap = dest_dir / "wrap" / cfg.name
        wrap.parent.mkdir(parents=True, exist_ok=True)
        wrap.write_text(f"[include {cfg}]\n\n" + "\n".join(fixes))
        name = str(wrap)
    # an extra [mcu NAME]: the linux host build for the host mcu, else the
    # main board's
    extra = []
    for mname, body in re.findall(r"^\[mcu (\S+)\](.*?)(?=^\[|\Z)", text, re.M | re.S):
        host = re.search(r"^serial\s*[:=]\s*/tmp/klipper_host_mcu", body, re.M)
        extra.append(f"{mname}={'linuxprocess.dict' if host else dicts[found]}")
    groups.setdefault(" ".join([dicts[found]] + extra), []).append(name)
out = ["# every Kalico example / board / printer config the stock printers.test",
       "# does not load (tools/gen_config_cases.py)",
       f"GCODE {kalico}/test/klippy/move.gcode", ""]
for d, cfgs in sorted(groups.items()):
    out.append(f"DICTIONARY {d}")
    out += [f"CONFIG {c if c.startswith('/') else f'{kalico}/config/{c}'}" for c in cfgs]
    out.append("")
out += [f"# skipped {n}: {why}" for n, why in skipped]
dest = dest_dir / "examples.test"
dest.parent.mkdir(parents=True, exist_ok=True)
dest.write_text("\n".join(out) + "\n")
print(f"{sum(len(c) for c in groups.values())} configs, {len(skipped)} skipped -> {dest}")
