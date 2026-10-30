#!/usr/bin/env python3
# Mirror IDs (src/plugins/mirror_ids.rs) with no handler in src/plugins/impls:
# a python plugin touching one gets "kaliru: X is not mirrored yet". Handled =
# named in impls, verbatim (mirror_verbatim.txt) or generic get_status /
# stats / get_temp (same matching as plugin_gaps.py).
#
# usage: mirror_stubs.py [KALIRU_DIR]          per-module bound/total
#        mirror_stubs.py [KALIRU_DIR] --list   every unbound name
#        --hand: count only the hand-written impls (not generated.rs)
import collections
import pathlib
import re
import sys

args = [a for a in sys.argv[1:] if a not in ("--list", "--hand")]
root = pathlib.Path(args[0] if args else pathlib.Path(__file__).parent.parent)
ids = re.findall(r'^    "([^"]+)",$', (root / "src/plugins/mirror_ids.rs").read_text(), re.M)
handled = set()
for p in (root / "src/plugins/impls").glob("*.rs"):
    if "--hand" in sys.argv and p.name == "generated.rs":
        continue
    t = p.read_text()
    handled |= set(re.findall(r'"(klippy\.[A-Za-z0-9_.]+)"', t))
    for cls, name in re.findall(r'(?:method|function|cell_attr|ro_attr|vec_attr)!\(r, "(klippy\.[A-Za-z0-9_.]+)"(?:\.[^,]*)?, (?:[^,(]+, )?(?:r#)?([A-Za-z_0-9]+)', t):
        handled.add(f"{cls}.{name}")
    # a file's own macros registering concat!($class, ".name") per class
    for _indent, mac, body in re.findall(r'(?m)^([ \t]*)macro_rules! (\w+) \{(.*?)\n\1\}', t, re.S):
        suffixes = re.findall(r'concat!\(\$class, "\.(\w+)"\)', body)
        # ... a nested macro given the class and a field: m!($r, $class, field)
        suffixes += re.findall(r'\w+!\(\$r, \$class, (\w+)\)', body)
        # ... concat!($class, ".", stringify!($name)) per (class, name) given
        if 'concat!($class, ".", stringify!($name))' in body:
            for cls, name in re.findall(mac + r'!\(r, "(klippy\.[A-Za-z0-9_.]+)", (\w+)\)', t):
                handled.add(f"{cls}.{name}")
        # ... concat!("klippy.x.", $module, ".name") per module
        for prefix, suffix in re.findall(r'concat!\("(klippy\.[A-Za-z0-9_.]+\.)", \$\w+, "\.(\w+)"\)', body):
            for mod in re.findall(mac + r'!\(r, "(\w+)"\)', t):
                handled.add(f"{prefix}{mod}.{suffix}")
        for cls in re.findall(mac + r'!\(r, "(klippy\.[A-Za-z0-9_.]+)"', t):
            handled |= {f"{cls}.{s}" for s in suffixes}
        # ... and concat!("klippy.x.Class.", stringify!($field)) per listed field
        for prefix in re.findall(r'concat!\("(klippy\.[A-Za-z0-9_.]+\.)", stringify!\(\$\w+\)\)', body):
            for args in re.findall(mac + r'!\(r, ([^;]*?)\);', t, re.S):
                handled |= {prefix + f.strip() for f in args.split(",") if f.strip()}
for line in (root / "tools/mirror_verbatim.txt").read_text().splitlines():
    if line.strip() and not line.startswith("#"):
        handled.add(line.strip())
# impls/traits.rs: a method name kaliru implements through a trait, bound
# where the class's module has a Rust type implementing that trait;
# self.printer / self.reactor everywhere
TRAIT_NAMES = {
    "TempSensor": ["setup_minmax", "setup_callback", "get_report_time_delta"],
    "PwmPin": ["get_mcu", "setup_max_duration", "setup_start_value", "setup_cycle_time", "set_pwm"],
    "DigitalOutPin": ["get_mcu", "setup_max_duration", "setup_start_value", "set_digital"],
    "AdcPin": ["get_mcu", "setup_adc_sample", "setup_adc_callback", "get_last_value"],
    "Endstop": ["get_mcu", "get_position_endstop"],
    "McuProbe": ["multi_probe_begin", "multi_probe_end", "probe_prepare", "probe_finish", "probing_move"],
    "LcdChip": ["get_mcu", "init", "flush", "clear", "write_text", "write_graphics", "write_glyph", "set_glyphs",
                "get_dimensions"],
    "BulkAdcSensor": ["get_mcu", "get_samples_per_second", "get_range", "get_channel_count", "add_client",
                      "attach_load_cell_probe"],
}
trait_files = collections.defaultdict(set)
for p in (root / "src").rglob("*.rs"):
    if "plugins" in p.parts:
        continue
    for tr in re.findall(r"impl (?:crate::[\w:]+::)?(\w+) for", p.read_text(errors="replace")):
        if tr in TRAIT_NAMES:
            trait_files[tr].add(p.relative_to(root / "src").with_suffix("").as_posix().removesuffix("/mod"))
for i in ids if "--hand" not in sys.argv else []:
    parts = i.split(".")
    # ... and cmd_X(gcmd): the handler registered for g-code command X
    if parts[-1] in ("printer", "reactor", "gcode") or (parts[-1].startswith("cmd_") and len(parts[-1]) > 4
                                               and parts[-1][4:] == parts[-1][4:].upper()):
        handled.add(i)
        continue
    # (a lowercase class, e.g. hd44780_spi.hd44780_spi: the second-last part)
    k = next((j for j, s in enumerate(parts) if j > 1 and s[:1].isupper()), len(parts) - 2)
    modfile = "/".join(parts[1:k])
    # (a Kalico module kaliru implements in another's file)
    modfile = {"extras/mpc_ambient_temperature": "extras/mpc_block_temperature"}.get(modfile, modfile)
    if any(parts[-1] in names and modfile in trait_files[tr] for tr, names in TRAIT_NAMES.items()):
        handled.add(i)

# native.rs ALIASES: a subclass's names served by its base class's handlers
for sub, base in re.findall(r'\("(klippy\.[\w.]+\.)", "(klippy\.[\w.]+\.)"\)', (root / "src/plugins/native.rs").read_text()):
    for i in ids:
        if i.startswith(sub) and base + i[len(sub):] in handled:
            handled.add(i)

# classes only python ever builds (verbatim __init__, never a kaliru
# object): their attributes live in the instance's __dict__ (prelude.py Attr)
PYTHON_ONLY = ("klippy.reactor.ReactorCallback.", "klippy.extras.indx.compat.ConfigWrapper.",
               "klippy.extras.indx.compat.ConfigContents.", "klippy.extras.indx.compat._ResponseHandle.", "klippy.extras.gcode_macro.TemplateWrapperPython.",
               "klippy.extras.gcode_macro.GetStatusWrapperPython.",
               "klippy.extras.gcode_macro.TemplateVariableWrapperPython.")
handled |= {i for i in ids if i.startswith(PYTHON_ONLY)}
stubs = [i for i in ids if i not in handled and not i.endswith((".get_status", ".stats", ".get_temp"))]
if "--list" in sys.argv:
    print("\n".join(stubs))
    sys.exit()


def module(name):
    parts = name.split(".")
    k = next((j for j, s in enumerate(parts) if j > 1 and s[:1].isupper()), len(parts) - 1)
    return ".".join(parts[:k])


total = collections.Counter(map(module, ids))
unbound = collections.Counter(map(module, stubs))
print(f"{len(ids)} ids, {len(ids) - len(stubs)} bound, {len(stubs)} stubs "
      f"({sum(1 for s in stubs if '._' not in s)} public)")
for m in sorted(total):
    print(f"  {m}: {total[m] - unbound[m]}/{total[m]}")
