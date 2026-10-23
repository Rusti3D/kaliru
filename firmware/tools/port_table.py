#!/usr/bin/env python3
# Convert C code whose items sit in #if / #ifdef / #elif / #else blocks into
# cfg'd Rust (the C #if as #[cfg]: CONFIG_X, !, &&, ||, defined(X) / #ifdef X
# as has_X).
#
# A table initializer: each run of rows under the same condition becomes a
# `const NAME_n: &[T]` part (an empty one when its cfg is false) and the table
# is their ctr::concat(). The rows are converted by a regex substitution.
#
#   port_table.py <file.c> <start regex> <NAME> <T> <row regex> <row replacement> [<extra cfg>]
#
# e.g. the pwm table of stm32/hard_pwm.c:
#   port_table.py hard_pwm.c 'pwm_regs\[\] = \{' PWM_REGS GpioPwmInfo \
#     "\{(\w+),\s*GPIO\('(\w)',\s*(\d+)\),\s*(\d+),\s*GPIO_FUNCTION\((\d+)\)\}" "p(\1, '\2', \3, \4, \5)"
#
# Declarations: the DECL_*() lines and #defines from the start line to the
# end line, as cfg'd decl_*!() items and consts, in source order.
#
#   port_table.py --decls <file.c> <start regex> <end regex>
import re
import sys

decls = sys.argv[1] == '--decls'
if decls:
    path, start, end = sys.argv[2:5]
    extra = None
else:
    path, start, name, typ, row_re, row_sub = sys.argv[1:7]
    extra = sys.argv[7] if len(sys.argv) > 7 else None
lines = open(path).read().split('\n')
i = next(n for n, l in enumerate(lines) if re.search(start, l)) + (0 if decls else 1)


def cond(expr):
    """a C #if expression as a cfg predicate"""
    toks = re.findall(r'\|\||&&|!|\(|\)|defined|\w+', expr)
    pos = [0]

    def peek():
        return toks[pos[0]] if pos[0] < len(toks) else None

    def take():
        pos[0] += 1
        return toks[pos[0] - 1]

    def atom():
        t = take()
        if t == '!':
            return 'not(%s)' % atom()
        if t == '(':
            e = orx()
            take()
            return e
        if t == 'defined':
            paren = peek() == '('
            if paren:
                take()
            n = take()
            if paren:
                take()
            return 'has_' + n
        return t

    def andx():
        parts = [atom()]
        while peek() == '&&':
            take()
            parts.append(atom())
        return parts[0] if len(parts) == 1 else 'all(%s)' % ', '.join(parts)

    def orx():
        parts = [andx()]
        while peek() == '||':
            take()
            parts.append(andx())
        return parts[0] if len(parts) == 1 else 'any(%s)' % ', '.join(parts)

    return orx()


def pred(cfgs):
    return cfgs[0] if len(cfgs) == 1 else 'all(%s)' % ', '.join(cfgs)


# stack of [conditions of this #if chain so far, current condition]
stack = []
rows = []  # (cfg list, rust item, comment)
pending_comment = None


def current():
    cfgs = [extra] if extra else []
    for prev, cur in stack:
        cfgs += ['not(%s)' % p for p in prev]
        if cur is not None:
            cfgs.append(cur)
    return cfgs


for line in lines[i:]:
    t = line.strip()
    if decls and re.search(end, t):
        break
    if not decls and t.startswith('};'):
        break
    m = re.match(r'#\s*(if|ifdef|ifndef|elif|else|endif)\b\s*(.*)', t)
    if m:
        kw, rest = m.groups()
        rest = re.sub(r'//.*', '', rest).strip()
        if kw == 'if':
            stack.append([[], cond(rest)])
        elif kw == 'ifdef':
            stack.append([[], 'has_' + rest])
        elif kw == 'ifndef':
            stack.append([[], 'not(has_%s)' % rest])
        elif kw == 'elif':
            stack[-1][0].append(stack[-1][1])
            stack[-1][1] = cond(rest)
        elif kw == 'else':
            stack[-1][0].append(stack[-1][1])
            stack[-1][1] = None
        else:
            stack.pop()
        continue
    if t.startswith('//'):
        pending_comment = t
        continue
    if decls:
        dm = re.match(r'DECL_(\w+)\((.*)\);', t)
        fm = re.match(r'#\s*define\s+(\w+)\s+(.*)', t)
        if dm:
            item = 'decl_%s!(%s);' % (dm.group(1).lower(), dm.group(2))
        elif fm:
            item = '#[allow(dead_code)]\nconst %s: u32 = %s;' % fm.groups()
        else:
            continue
        rows.append((current(), item, pending_comment))
        pending_comment = None
        continue
    for rm in re.finditer(row_re, t):
        rows.append((current(), rm.expand(row_sub), pending_comment))
        pending_comment = None

if decls:
    out = []
    for cfgs, item, comment in rows:
        if comment:
            out.append(comment)
        out.append(('#[cfg(%s)]\n' % pred(cfgs) if cfgs else '') + item)
    print('\n'.join(out))
    sys.exit(0)

# group consecutive rows with the same condition
parts = []
for cfgs, r, comment in rows:
    if parts and parts[-1][0] == cfgs and not comment:
        parts[-1][1].append(r)
    else:
        parts.append((cfgs, [r], comment))

out = ['// (the C table, its #if blocks as parts: tools/port_table.py)']
names = []
for n, (cfgs, rs, comment) in enumerate(parts):
    pn = '%s_%d' % (name, n)
    names.append(pn)
    if comment:
        out.append(comment)
    body = '&[\n    %s,\n]' % ',\n    '.join(rs)
    if not cfgs:
        out.append('const %s: &[%s] = %s;' % (pn, typ, body))
        continue
    out.append('#[cfg(%s)]' % pred(cfgs))
    out.append('const %s: &[%s] = %s;' % (pn, typ, body))
    out.append('#[cfg(not(%s))]' % pred(cfgs))
    out.append('const %s: &[%s] = &[];' % (pn, typ))
out.append('const %s_PARTS: &[&[%s]] = &[%s];' % (name, typ, ', '.join(names)))
out.append('static %s: [%s; concat_len(%s_PARTS)] = concat(%s_PARTS);' % (name, typ, name, name))
print('\n'.join(out))
