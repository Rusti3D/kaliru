#!/usr/bin/env python3
# Generate src/stm32/chip/<mcu>.rs from the CMSIS device headers in Kalico's lib/:
# every typedef struct (as a #[repr(C)] struct of volatile registers, offsets checked
# against gcc), every integer macro and IRQn_Type value, every peripheral pointer.
# Values are computed by gcc from the headers, never typed in by hand.
#
#   gen_stm32_regs.py <kalico>/lib <family> <mcu> <out.rs>
#   e.g. gen_stm32_regs.py ~/kalico/lib stm32f4 stm32f401xc src/stm32/chip/stm32f401xc.rs
import os, re, subprocess, sys, tempfile

lib, family, mcu, out = sys.argv[1:5]
define = mcu.upper().replace('X', 'x')          # as the stm32 Makefile's MCU_UPPER
incs = ['-I', os.path.join(lib, family, 'include'), '-I', os.path.join(lib, 'cmsis-core')]
top = '%sxx.h' % family
# the -mcpu of Kalico's src/stm32/Makefile (the CMSIS headers test the target)
MCPU = {'stm32f0': 'cortex-m0', 'stm32f1': 'cortex-m3', 'stm32f2': 'cortex-m3', 'stm32f4': 'cortex-m4',
        'stm32f7': 'cortex-m7', 'stm32g0': 'cortex-m0plus', 'stm32g4': 'cortex-m4', 'stm32h7': 'cortex-m7',
        'stm32l4': 'cortex-m4'}[family]
CTYPES = {'uint32_t': 'u32', 'uint16_t': 'u16', 'uint8_t': 'u8', 'int32_t': 'i32'}


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def strip_comments(s):
    s = re.sub(r'/\*.*?\*/', ' ', s, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', s)


# The headers the device include pulls in (gcc -M), and the macro names they define
dep = run(['gcc', '-M', '-D' + define] + incs + ['-x', 'c', os.path.join(lib, family, 'include', top)])
hdrs = [h for h in dep.stdout.replace('\\\n', ' ').split()[1:]
        if os.path.abspath(h).startswith(os.path.abspath(lib))]
text = ''.join(strip_comments(open(h, encoding='latin-1').read()) for h in hdrs)
names = set(re.findall(r'#\s*define\s+(\w+)', text))

# Structs: only plain register lists; anything else (unions, bitfields) is left out
structs = {}
for m in re.finditer(r'typedef\s+struct\s*\{([^{}]*)\}\s*(\w+)\s*;', text):
    body, sname = m.groups()
    fields = []
    for decl in body.split(';'):
        decl = decl.strip()
        if not decl:
            continue
        f = re.fullmatch(r'(?:__IOM|__IM|__OM|__IO|__I|__O|volatile|const|\s)*(\w+)\s+(\w+)\s*(?:\[\s*(\w+)\s*\])?', decl)
        if not f or (f.group(1) not in CTYPES and f.group(1) not in structs):
            fields = None
            break
        fields.append(f.groups())
    if fields:
        structs[sname] = fields
enums = []
for m in re.finditer(r'typedef\s+enum\s*\{([^{}]*)\}\s*IRQn_Type\s*;', text):
    enums += [e.split('=')[0].strip() for e in m.group(1).split(',') if e.strip()]

# Macro expansions (object-like only)
dm = run(['gcc', '-E', '-dM', '-D' + define] + incs + ['-x', 'c', os.path.join(lib, family, 'include', top)])
macros = {}
for line in dm.stdout.splitlines():
    m = re.match(r'#define (\w+) (.*)', line)
    if m and m.group(1) in names:
        macros[m.group(1)] = m.group(2)
ptrs = {}
for n, v in macros.items():
    p = re.fullmatch(r'\(\(\s*(\w+)\s*\*\s*\)\s*(.+)\)', v.strip())
    if p and p.group(1) in structs:
        ptrs[n] = p.group(1)
ints = sorted(n for n in macros if n not in ptrs and macros[n].strip())


# Evaluate with arm-none-eabi-gcc: each value becomes a const uint32_t whose .word is read from
# the assembly. Items gcc rejects (not integer constants, structs it doesn't define) are dropped.
def evaluate():
    items = [('I', n, '(uint32_t)(%s)' % n) for n in ints]
    items += [('P', n, '(uint32_t)(%s)' % n) for n in ptrs]
    items += [('E', n, '(uint32_t)(%s)' % n) for n in enums]
    for s, fl in structs.items():
        items.append(('S', s, 'sizeof(%s)' % s))
        items += [('F', s + '.' + f, 'offsetof(%s, %s)' % (s, f)) for t, f, dim in fl]
    while True:
        pre = ['#include "%s"' % top, '#include <stddef.h>', '#include <stdint.h>']
        src = pre + ['__attribute__((used)) const uint32_t v%d = %s;' % (i, e) for i, (k, n, e) in enumerate(items)]
        with tempfile.TemporaryDirectory() as d:
            c = os.path.join(d, 'v.c')
            open(c, 'w').write('\n'.join(src) + '\n')
            r = run(['arm-none-eabi-gcc', '-w', '-mcpu=' + MCPU, '-mthumb', '-fno-zero-initialized-in-bss', '-S', '-D' + define]
                    + incs + ['-o', os.path.join(d, 'v.s'), c])
            if r.returncode == 0:
                asm = open(os.path.join(d, 'v.s')).read()
                vals = {}
                for m in re.finditer(r'^v(\d+):\s*\n\s*(?:\.word\s+(\S+)|(\.space\s+4))$', asm, re.M):
                    k, n, e = items[int(m.group(1))]
                    if m.group(3):
                        vals[(k, n)] = 0
                    elif re.fullmatch(r'-?\d+', m.group(2)):
                        vals[(k, n)] = int(m.group(2)) & 0xffffffff
                    elif k != 'I':
                        sys.exit('not a number: %s %s' % (n, m.group(2)))
                    # else: the macro names a function (NVIC_SetPriority -> __NVIC_SetPriority)
                return vals
            bad = {int(e.group(1)) - len(pre) - 1 for e in re.finditer(r'v\.c:(\d+):', r.stderr)}
            bad = {i for i in bad if 0 <= i < len(items)}
            if not bad:
                sys.exit(r.stderr[:3000])
            drop = {items[i][1].split('.')[0] for i in bad if items[i][0] in 'SF'}
            for s in drop:
                del structs[s]
            items = [it for i, it in enumerate(items)
                     if i not in bad and not (it[0] in 'SF' and it[1].split('.')[0] in drop)]


vals = evaluate()
ptrs = {n: s for n, s in ptrs.items() if ('P', n) in vals and s in structs}

o = ['// Generated by tools/gen_stm32_regs.py from Kalico lib/%s (%s, -D%s): do not edit.' % (family, top, define),
     'use crate::stm32::reg::Reg;', '', 'pub type IRQn_Type = i32;', '']
for s, fl in structs.items():
    o.append('#[repr(C)]')
    o.append('pub struct %s {' % s)
    for t, f, dim in fl:
        ty = 'Reg<%s>' % CTYPES[t] if t in CTYPES else t
        if dim:
            lit = re.fullmatch(r'(\d+|0x[0-9a-fA-F]+)[uUlL]*', dim)
            n = int(lit.group(1), 0) if lit else vals[('I', dim)]
            ty = '[%s; %d]' % (ty, n)
        o.append('    pub %s: %s,' % (f, ty))
    o.append('}')
    o.append('const _: () = assert!(core::mem::size_of::<%s>() == %d);' % (s, vals[('S', s)]))
    for t, f, dim in fl:
        o.append('const _: () = assert!(core::mem::offset_of!(%s, %s) == %d);' % (s, f, vals[('F', s + '.' + f)]))
o.append('')
for n in enums:
    v = vals[('E', n)]
    o.append('pub const %s: i32 = %d;' % (n, v - (1 << 32) if v >= 1 << 31 else v))
for n in sorted(ptrs):
    o.append('pub const %s: *mut %s = 0x%08x as *mut %s;' % (n, ptrs[n], vals[('P', n)], ptrs[n]))
for n in ints:
    if ('I', n) in vals and n not in enums:
        o.append('pub const %s: u32 = 0x%x;' % (n, vals[('I', n)] & 0xffffffff))
open(out, 'w').write('\n'.join(o) + '\n')
print('%s: %d structs, %d irqs, %d pointers, %d values' % (out, len(structs), len(enums), len(ptrs),
                                                         sum(1 for n in ints if ('I', n) in vals)))
