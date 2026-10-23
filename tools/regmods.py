# usage: regmods.py name:lc:lp ...   (lc/lp = 1 when the module has it)
import re, sys
p = 'mod.rs'
s = open(p).read()
specs = [a.split(':') for a in sys.argv[1:]]
mods = re.findall(r"^pub mod (\w+);", s, re.M)
allm = sorted(set(mods + [m for m, _, _ in specs]))
start = s.index("pub mod ")
end = s.index("\n\n", start)
s = s[:start] + "\n".join(f"pub mod {m};" for m in allm) + s[end:]
for m, lc, lp in specs:
    lcs = f"Some({m}::load_config)" if lc == '1' else "None"
    lps = f"Some({m}::load_config_prefix)" if lp == '1' else "None"
    entry = f'PrinterModule {{ name: "{m}", ported: true, load_config: {lcs}, load_config_prefix: {lps}, register_components: None }},'
    pat = re.compile(r'^(\s*)todo\("' + m + r'"\),$', re.M)
    if not pat.search(s):
        print("no todo entry for", m)
        continue
    s = pat.sub(lambda mm: mm.group(1) + entry, s)
open(p, 'w').write(s)
