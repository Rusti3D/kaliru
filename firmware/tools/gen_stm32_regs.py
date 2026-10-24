#!/usr/bin/env python3
# Generate src/stm32/chip/<mcu>.rs from the CMSIS device headers in Kalico's lib/:
# every typedef struct (as a #[repr(C)] struct of volatile registers, offsets checked
# against gcc), every integer macro and IRQn_Type value, every peripheral pointer.
# Values are computed by gcc from the headers, never typed in by hand.
#
#   gen_stm32_regs.py <kalico>/lib <family> <mcu> <out.rs>
#   e.g. gen_stm32_regs.py ~/kalico/lib stm32f4 stm32f401xc src/stm32/chip/stm32f401xc.rs
#
# or one stand alone header (Kalico's lib/n32g45x/include/n32g45x_adc.h):
#   gen_stm32_regs.py --header <kalico>/lib <family> <header.h> <out.rs>
#
# or the pico-sdk headers of an rp2040 / rp2350 build (the cmsis device header and every
# hardware/structs and hardware/regs header; also every enum, not only IRQn_Type):
#   gen_stm32_regs.py --pico <kalico>/lib <mcu> <out.rs>
#
# or another vendor's CMSIS device header (with cmsis-core), optionally with a -D of the build:
#   gen_stm32_regs.py --cmsis <kalico>/lib <family> <header, from lib/> <out.rs> [define [include dir ...]]
#   e.g. gen_stm32_regs.py --cmsis ~/kalico/lib lpc176x lpc176x/device/LPC17xx.h src/lpc176x/chip/lpc17xx.rs
import os, re, subprocess, sys, tempfile

pico = sys.argv[1] == '--pico'
if sys.argv[1] == '--header':
    lib, family, header, out = sys.argv[2:6]
    define = None
    incs = ['-I', os.path.dirname(os.path.abspath(header))]
    top = os.path.basename(header)
    header_path = os.path.abspath(header)
elif pico:
    lib, mcu, out = sys.argv[2:5]
    family = mcu
    define = 'PICO_' + mcu.upper()                  # as the rp2040 Makefile's -DPICO_$(MCU_UPPER)
    sdk = os.path.join(lib, 'pico-sdk')
    incs = ['-I', os.path.join(lib, 'cmsis-core'), '-I', os.path.join(sdk, mcu), '-I', sdk,
            '-I', os.path.join(sdk, mcu, 'cmsis_include')]
    hw = os.path.join(sdk, mcu, 'hardware')
    top = os.path.abspath(out) + '.h'
    with open(top, 'w') as f:
        f.write('#include "%s.h"\n' % mcu.upper())
        for d in ('regs', 'structs'):
            for h in sorted(os.listdir(os.path.join(hw, d))):
                f.write('#include "hardware/%s/%s"\n' % (d, h))
        if mcu == 'rp2350':
            # (the rom api constants of rp2040/rp2350_bootrom.c)
            f.write('#include "boot/bootrom_constants.h"\n#include "boot/picoboot_constants.h"\n')
    header_path = top
elif sys.argv[1] == '--cmsis':
    lib, family, rel, out = sys.argv[2:6]
    define = sys.argv[6] if len(sys.argv) > 6 else None
    header_path = os.path.join(lib, rel)
    incs = ['-I', os.path.dirname(header_path), '-I', os.path.join(lib, 'cmsis-core')]
    # (more include directories, from lib/: samd51's component headers are same54's)
    for d in sys.argv[7:]:
        incs += ['-I', os.path.join(lib, d)]
    top = os.path.basename(rel)
else:
    lib, family, mcu, out = sys.argv[1:5]
    define = mcu.upper().replace('X', 'x')          # as the stm32 Makefile's MCU_UPPER
    incs = ['-I', os.path.join(lib, family, 'include'), '-I', os.path.join(lib, 'cmsis-core')]
    top = '%sxx.h' % family
    header_path = os.path.join(lib, family, 'include', top)
defs = ['-D' + define] if define else []
# the -mcpu of Kalico's src/stm32/Makefile (the CMSIS headers test the target)
MCPU = {'rp2040': 'cortex-m0plus', 'rp2350': 'cortex-m33', 'stm32f0': 'cortex-m0', 'stm32f1': 'cortex-m3', 'stm32f2': 'cortex-m3', 'stm32f4': 'cortex-m4',
        'stm32f7': 'cortex-m7', 'stm32g0': 'cortex-m0plus', 'stm32g4': 'cortex-m4', 'stm32h7': 'cortex-m7',
        'stm32l4': 'cortex-m4', 'lpc176x': 'cortex-m3', 'sam3x': 'cortex-m3', 'sam4s': 'cortex-m4', 'sam4e': 'cortex-m4',
        'same70b': 'cortex-m7', 'samd21': 'cortex-m0plus', 'samc21': 'cortex-m0plus', 'samd51': 'cortex-m4',
        'same51': 'cortex-m4', 'same54': 'cortex-m4'}[family]
CTYPES = {'uint32_t': 'u32', 'uint16_t': 'u16', 'uint8_t': 'u8', 'int32_t': 'i32',
          # pico-sdk hardware/address_mapped.h
          'io_rw_32': 'u32', 'io_ro_32': 'u32', 'io_wo_32': 'u32',
          'io_rw_16': 'u16', 'io_ro_16': 'u16', 'io_wo_16': 'u16',
          'io_rw_8': 'u8', 'io_ro_8': 'u8', 'io_wo_8': 'u8',
          # Atmel (sam3x, sam4, same70, samd) headers
          'RoReg': 'u32', 'WoReg': 'u32', 'RwReg': 'u32', 'RoReg16': 'u16', 'WoReg16': 'u16', 'RwReg16': 'u16',
          'RoReg8': 'u8', 'WoReg8': 'u8', 'RwReg8': 'u8'}


# a C name that is a Rust keyword as a raw identifier (the `in` of usb_dpram)
KEYWORDS = set('as async await break const continue crate dyn else enum extern false fn for if impl in let loop '
               'match mod move mut pub ref return self static struct super trait true type unsafe use where '
               'while abstract become box do final gen macro override priv try typeof unsized virtual yield'.split())


def rust_name(n):
    return 'r#' + n if n in KEYWORDS else n


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def strip_comments(s):
    s = re.sub(r'/\*.*?\*/', ' ', s, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', s)


# The headers the device include pulls in (gcc -M), and the macro names they define
dep = run(['gcc', '-M'] + defs + incs + ['-x', 'c', header_path])
hdrs = [h for h in dep.stdout.replace('\\\n', ' ').split()[1:]
        if os.path.abspath(h).startswith(os.path.abspath(lib))]
text = ''.join(strip_comments(open(h, encoding='latin-1').read()) for h in hdrs)
names = set(re.findall(r'#\s*define\s+(\w+)', text))
# the structs and enums as the compiler sees them (#if blocks resolved)
gcc = ['arm-none-eabi-gcc', '-mcpu=' + MCPU, '-mthumb'] + defs + incs
text = run(gcc + ['-E', '-P', '-x', 'c', header_path]).stdout

# (pico: an anonymous `union { struct { fields }; T array[N]; };` is its struct's fields, the names
# the C code uses; an inline `struct tag { fields } name[N];` is a struct of its own)
inner = []        # (the C names of these are `struct tag`)
hoisted = []
if pico:
    def hoist(m):
        inner.append(m.group(1))
        hoisted.append('typedef struct {%s} %s;' % (m.group(2), m.group(1)))
        return '%s %s' % (m.group(1), m.group(3))
    while True:
        t = re.sub(r'union\s*\{\s*struct\s*\{([^{}]*)\}\s*;[^{}]*\}\s*;', r'\1', text)
        t = re.sub(r'struct\s+(\w+)\s*\{([^{}]*)\}\s*(\w+\s*(?:\[[^\]]*\])?\s*;)', hoist, t)
        if t == text:
            break
        text = t
    text = '\n'.join(hoisted) + '\n' + text

# An anonymous union in a struct (`union { __I uint8_t RBR; __O uint8_t THR; uint32_t RESERVED0; };`,
# `union { __IO uint32_t SR; stc_usart_sr_field_t SR_f; };`): its widest plain register is the
# struct field, its other plain registers (also those of anonymous structs in it) are accessor
# methods at their own offsets; bitfield struct members are left out.
unions = []
text = re.sub(r'union\s*\{((?:[^{}]|\{[^{}]*\})*)\}\s*;',
              lambda m: unions.append(m.group(1)) or '__union_%d __union_%d;' % (len(unions) - 1, len(unions) - 1), text)
DECL = r'(?:__IOM|__IM|__OM|__IO|__I|__O|volatile|const|\s)*(\w+)\s+(\w+)((?:\s*\[[^\]]+\])*)'
CSIZE = {'u8': 1, 'u16': 2, 'u32': 4, 'i32': 4}


def parse_decl(decl):
    f = re.fullmatch(DECL, decl.strip())
    if not f:
        return None
    # (the array dimensions, outermost first)
    dims = tuple(d.strip() for d in re.findall(r'\[([^\]]+)\]', f.group(3)))
    return f.group(1), f.group(2), dims


aliases = {}      # struct: [(type, name, dims)] (the other union members)

# Structs: only plain register lists; anything else (bitfields, ...) is left out
structs = {}
# (Atmel ASF: a register type `typedef union { struct {..} bit; uintN_t reg; } X_Type;` is a
# struct of its `reg` register)
for m in re.finditer(r'typedef\s+union\s*\{((?:[^{}]|\{[^{}]*\})*)\}\s*(\w+)\s*;', text):
    utop = re.sub(r'struct\s*\{[^{}]*\}\s*\w*\s*;', '', m.group(1))
    r = re.search(r'(?:__IO|__I|__O|volatile|const|\s)*\b(uint(?:8|16|32)_t)\s+reg\s*;', utop)
    if r:
        structs[m.group(2)] = [(r.group(1), 'reg', ())]
for m in re.finditer(r'typedef\s+struct\s*\{([^{}]*)\}\s*(\w+)\s*;', text):
    body, sname = m.groups()
    fields = []
    for decl in body.split(';'):
        decl = decl.strip()
        if not decl:
            continue
        u = re.fullmatch(r'__union_(\d+)\s+__union_\d+', decl)
        if u:
            ubody = unions[int(u.group(1))]
            plain = [d for d in (parse_decl(x) for x in re.sub(r'struct\s*\{[^{}]*\}\s*;', '', ubody).split(';') if x.strip())
                     if d and d[0] in CTYPES]
            inner_decls = [d for s_ in re.findall(r'struct\s*\{([^{}]*)\}\s*;', ubody)
                           for d in (parse_decl(x) for x in s_.split(';') if x.strip()) if d and d[0] in CTYPES]

            def size(d):
                n = CSIZE[CTYPES[d[0]]]
                for x in d[2]:
                    lit = re.fullmatch(r'(\d+|0x[0-9a-fA-F]+)[uUlL]*', x)
                    n *= int(lit.group(1), 0) if lit else 1
                return n
            if not plain:
                fields = None
                break
            primary = max(plain, key=size)         # (the first of the widest)
            fields.append(primary)
            aliases.setdefault(sname, []).extend(d for d in plain + inner_decls if d is not primary)
            continue
        f = parse_decl(decl)
        if not f or (f[0] not in CTYPES and f[0] not in structs):
            fields = None
            break
        fields.append(f)
    if fields:
        structs[sname] = fields
    else:
        aliases.pop(sname, None)
# (Atmel ASF: a union of struct views of a peripheral, `typedef union { SercomI2cm I2CM;
# SercomSpi SPI; ... } Sercom;`: the widest view is the field, all views are accessors)
union_views = []
for m in re.finditer(r'typedef\s+union\s*\{([^{}]*)\}\s*(\w+)\s*;', text):
    views = [parse_decl(x) for x in m.group(1).split(';') if x.strip()]
    if views and all(v and v[0] in structs and not v[2] for v in views) and m.group(2) not in structs:
        structs[m.group(2)] = [views[0]]
        aliases[m.group(2)] = views
        union_views.append(m.group(2))
enums = []
for m in re.finditer(r'typedef\s+enum\s*\w*\s*\{([^{}]*)\}\s*IRQn_Type\s*;', text):
    enums += [e.split('=')[0].strip() for e in m.group(1).split(',') if e.strip()]
# (pico: the other enums too, as u32 values: gpio_function_t, ...)
uenums = []
if pico:
    for m in re.finditer(r'\benum\s*\w*\s*\{([^{}]*)\}', text):
        uenums += [e.split('=')[0].strip() for e in m.group(1).split(',')
                   if e.strip() and e.split('=')[0].strip() not in enums]

# Macro expansions (object-like only)
dm = run(gcc + ['-E', '-dM', '-x', 'c', header_path])
macros = {}
for line in dm.stdout.splitlines():
    m = re.match(r'#define (\w+)(?: (.*))?$', line)
    if m and m.group(1) in names:
        macros[m.group(1)] = m.group(2) or ''
ptrs = {}
for n, v in macros.items():
    p = re.fullmatch(r'\(\(\s*(\w+)\s*\*\s*\)\s*(.+)\)', v.strip())
    if p and p.group(1) in structs:
        ptrs[n] = p.group(1)
# a register as an lvalue macro: *((uint32_t *)(ADDR)) or (*(uint32_t *)(ADDR))
regs = sorted(n for n, v in macros.items()
              if re.fullmatch(r'\(?\s*\*\s*\(*\s*(?:__IO|__I|__O|volatile|const|\s)*uint32_t\s*\*\s*\).+', v.strip()))
# (pico: a pointer macro named by another one, iobank0_hw -> io_bank0_hw)
for n, v in macros.items():
    if pico and v.strip() in ptrs:
        ptrs[n] = ptrs[v.strip()]
ints = sorted(n for n in macros if n not in ptrs and n not in regs and macros[n].strip())
# Field macros `NAME(value)` that are `((NAME_Msk & ((value) << NAME_Pos)))` (Atmel): const fns,
# checked against gcc for a few values
fields_fn = {}
for line in dm.stdout.splitlines():
    m = re.match(r'#define (\w+)\((\w+)\) (.*)$', line)
    if m and m.group(1) in names:
        v = re.escape(m.group(2))
        f = re.fullmatch(r'\(*\s*(\w+)\s*&\s*\(\s*\(\s*%s\s*\)\s*<<\s*(\w+)\s*\)\s*\)*' % v, m.group(3).strip())
        if f and m.group(3).count('(') == m.group(3).count(')'):
            fields_fn[m.group(1)] = (f.group(1), f.group(2))
FN_TESTS = (0, 1, 3, 0x5a5a5a5a)


# Evaluate with arm-none-eabi-gcc: each value becomes a const uint32_t whose .word is read from
# the assembly. Items gcc rejects (not integer constants, structs it doesn't define) are dropped.
def evaluate():
    items = [('I', n, '(uint32_t)(%s)' % n) for n in ints]
    items += [('P', n, '(uint32_t)(%s)' % n) for n in ptrs]
    items += [('R', n, '(uint32_t)&(%s)' % n) for n in regs]
    items += [('E', n, '(uint32_t)(%s)' % n) for n in enums + uenums]
    items += [('M', '%s.%d' % (n, k), '(uint32_t)(%s(%du))' % (n, k)) for n in fields_fn for k in FN_TESTS]
    for s, fl in structs.items():
        cs = 'struct ' + s if s in inner else s
        items.append(('S', s, 'sizeof(%s)' % cs))
        # (a field named like a macro, COMP1 of DWT_Type on chips with COMP1
        # peripherals, cannot be named here: its offset is covered by the size)
        items += [('F', s + '.' + f, 'offsetof(%s, %s)' % (cs, f)) for t, f, dim in fl if f not in macros]
        # (an array dimension, when it is not a plain number)
        for t, f, dims in fl:
            for k, d in enumerate(dims):
                if f not in macros and not re.fullmatch(r'\w+', d):
                    a = '((%s *)0)->%s' % (cs, f) + '[0]' * k
                    items.append(('D', '%s.%s.%d' % (s, f, k), 'sizeof(%s) / sizeof(%s[0])' % (a, a)))
        # (the offsets of the other union members)
        items += [('A', s + '.' + f, 'offsetof(%s, %s)' % (cs, f)) for t, f, dims in aliases.get(s, []) if f not in macros]
    while True:
        pre = ['#include <stddef.h>', '#include <stdint.h>', '#include "%s"' % top]
        src = pre + ['__attribute__((used)) const uint32_t v%d = %s;' % (i, e) for i, (k, n, e) in enumerate(items)]
        with tempfile.TemporaryDirectory() as d:
            c = os.path.join(d, 'v.c')
            open(c, 'w').write('\n'.join(src) + '\n')
            r = run(['arm-none-eabi-gcc', '-w', '-mcpu=' + MCPU, '-mthumb', '-fno-zero-initialized-in-bss', '-S'] + defs
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
            drop = {items[i][1].split('.')[0] for i in bad if items[i][0] in 'SFDA'}
            for i in bad:
                if items[i][0] == 'M':
                    fields_fn.pop(items[i][1].split('.')[0], None)
            for s in drop:
                del structs[s]
            items = [it for i, it in enumerate(items)
                     if i not in bad and not (it[0] in 'SFDA' and it[1].split('.')[0] in drop)]


vals = evaluate()
# (a union of views: its field is the view as wide as the union)
for u in union_views:
    if u in structs:
        wide = [v for v in aliases[u] if vals.get(('S', v[0])) == vals.get(('S', u))]
        # (named apart from the views, which are all accessors)
        structs[u] = [(wide[0][0] if wide else structs[u][0][0], '_view', ())]
ptrs = {n: s for n, s in ptrs.items() if ('P', n) in vals and s in structs}

if pico:
    src = 'pico-sdk/%s (%s.h, hardware/regs/*.h, hardware/structs/*.h, -D%s)' % (mcu, mcu.upper(), define)
elif define is None:
    src = os.path.relpath(header_path, lib)
elif sys.argv[1] == '--cmsis':
    src = '%s, -D%s' % (os.path.relpath(header_path, lib), define)
else:
    src = '%s (%s, -D%s)' % (family, top, define)
o = ['// Generated by tools/gen_stm32_regs.py from Kalico lib/%s: do not edit.' % src,
     'use crate::reg::Reg;', '', 'pub type IRQn_Type = i32;',
     ]
if define:
    o += ['// the -D%s of the build (the headers test it)' % define, 'pub const %s: u32 = 1;' % define]
o.append('')
def rust_type(s, t, f, dim, plain=False):
    ty = CTYPES[t] if plain and t in CTYPES else 'Reg<%s>' % CTYPES[t] if t in CTYPES else t
    for k, d in reversed(list(enumerate(dim))):
        lit = re.fullmatch(r'(\d+|0x[0-9a-fA-F]+)[uUlL]*', d)
        n = int(lit.group(1), 0) if lit else vals.get(('I', d), vals.get(('D', '%s.%s.%d' % (s, f, k))))
        if n is None:
            return None
        ty = '[%s; %d]' % (ty, n)
    return ty


for s, fl in structs.items():
    # (a stand alone header's struct that no peripheral points at is a plain
    # value: ADC_InitType)
    plain = sys.argv[1] == '--header' and s not in ptrs.values()
    o.append('#[repr(C)]')
    o.append('pub struct %s {' % s)
    for t, f, dim in fl:
        o.append('    pub %s: %s,' % (rust_name(f), rust_type(s, t, f, dim, plain)))
    o.append('}')
    al = [(t, f, d) for t, f, d in aliases.get(s, [])
          if ('A', s + '.' + f) in vals and rust_type(s, t, f, d) and f not in [x[1] for x in fl]]
    if al:
        # (the other members of the anonymous unions)
        o.append('impl %s {' % s)
        for t, f, d in al:
            ty = rust_type(s, t, f, d)
            o.append('    #[inline(always)]')
            o.append('    pub fn %s(&self) -> &%s {' % (rust_name(f), ty))
            o.append('        unsafe { &*((self as *const Self as *const u8).add(%d) as *const %s) }' % (vals[('A', s + '.' + f)], ty))
            o.append('    }')
        o.append('}')
    o.append('const _: () = assert!(core::mem::size_of::<%s>() == %d);' % (s, vals[('S', s)]))
    for t, f, dim in fl:
        if ('F', s + '.' + f) in vals:
            o.append('const _: () = assert!(core::mem::offset_of!(%s, %s) == %d);' % (s, rust_name(f), vals[('F', s + '.' + f)]))
o.append('')
for n in enums:
    v = vals[('E', n)]
    o.append('pub const %s: i32 = %d;' % (n, v - (1 << 32) if v >= 1 << 31 else v))
for n in uenums:
    if ('E', n) in vals:
        o.append('pub const %s: u32 = 0x%x;' % (n, vals[('E', n)]))
for n in sorted(n for n in macros if not macros[n].strip()):
    # a macro defined empty (the C #ifdef tests it: has_NAME)
    o.append('pub const %s: () = ();' % n)
for n in regs:
    # (the C register macro *((uint32_t *)(ADDR)) as its address)
    o.append('pub const %s: *mut u32 = 0x%08x as *mut u32;' % (n, vals[('R', n)]))
for n in sorted(ptrs):
    o.append('pub const %s: *mut %s = 0x%08x as *mut %s;' % (n, ptrs[n], vals[('P', n)], ptrs[n]))
for n, (msk, pos) in sorted(fields_fn.items()):
    # (only when the formula gives gcc's values)
    if ('I', msk) not in vals or ('I', pos) not in vals or vals[('I', pos)] >= 32:
        continue
    if all(vals.get(('M', '%s.%d' % (n, k))) == vals[('I', msk)] & ((k << vals[('I', pos)]) & 0xffffffff)
           for k in FN_TESTS):
        o.append('pub const fn %s(value: u32) -> u32 {' % n)
        o.append('    %s & (value << %s)' % (msk, pos))
        o.append('}')
for n in ints:
    if ('I', n) in vals and n not in enums and n not in uenums:
        v = vals[('I', n)] & 0xffffffff
        if macros[n].strip() in enums:
            # an alias of an irq number (USB_LP_IRQn -> USB_LP_CAN1_RX0_IRQn)
            o.append('pub const %s: IRQn_Type = %d;' % (n, v - (1 << 32) if v >= 1 << 31 else v))
        else:
            o.append('pub const %s: u32 = 0x%x;' % (n, v))
open(out, 'w').write('\n'.join(o) + '\n')
if pico:
    os.remove(top)
print('%s: %d structs, %d irqs, %d pointers, %d values' % (out, len(structs), len(enums), len(ptrs),
                                                         sum(1 for n in ints if ('I', n) in vals)))
