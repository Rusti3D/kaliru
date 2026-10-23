#!/usr/bin/env python3
# Compares the SyntaxErrors of python macro sources: CPython (the python
# running this script, 3.13 like the reference) against kaliru's pysyntax
# tool (src/pysyntax.rs). Prints the differing cases and a summary.
# usage: gen_pysyntax_cases.py <path to pysyntax binary> [--exec]
import json
import subprocess
import sys

CASES = [
    # brackets
    "x = (1,\ny = 2\n",
    "x = [1, 2\n",
    "x = {1: 2\nemit('a')\n",
    "x = (1, 2))\n",
    "x = [1, 2)\n",
    "x = (1,\n 2]\n",
    "emit('G1 X%d' % (1)\n",
    "respond_info(str(1)\nx = 1\n",
    "}\n",
    # strings
    "x = 'abc\n",
    'x = "abc\n',
    "x = '''abc\n",
    'x = """abc\ny\n',
    "x = f'abc\n",
    "emit(\"G1 X1)\n",
    # indentation
    "if True:\nx = 2\n",
    "if True:\n    x = 1\nelse:\nx = 2\n",
    "if True:\n    x = 1\nelif False:\nx = 2\n",
    "for i in range(3):\nemit('a')\n",
    "while True:\nbreak\n",
    "def f():\nreturn 1\n",
    "class A:\npass\n",
    "try:\nx = 1\nexcept Exception:\n    pass\n",
    "try:\n    x = 1\nexcept Exception:\npass\n",
    "try:\n    x = 1\nfinally:\npass\n",
    "with open('x') as f:\npass\n",
    "if True:\n",
    "x = 1\n    y = 2\n",
    "if True:\n        x = 1\n    y = 2\n",
    "if True:\n\tx = 1\n        y = 2\n",
    # generic and specific parser errors
    "x = 1 +* 2\n",
    "x = = 1\n",
    "if x == 1\n    pass\n",
    "for i in range(3)\n    pass\n",
    "def f()\n    pass\n",
    "while True\n    pass\n",
    "if x = 1:\n    pass\n",
    "print(a b)\n",
    "x = [1 2]\n",
    "f(x for x in y, 1)\n",
    "1 = x\n",
    "f() = 1\n",
    "x + 1 = 2\n",
    "del f()\n",
    "import\n",
    "from x import\n",
    "return\n" + "x =\n",
    "x =\n",
    "emit(\n",
    "else:\n    pass\n",
    "x = 1 if 2\n",
    "lambda x: (yield)\n",
    "def f(x, x): pass\n",
    "x = 0777\n",
    "x = 1_\n",
    "x = 1.2.3\n",
    "x = $\n",
    "x = 5 ++\n",
    "a, b += 1\n",
    "f(**x, *y)\n",
    "f(a=1, 2)\n",
    "x = {'a' 1}\n",
    "nonlocal x\n",
    "async def f():\n    pass\nawait x\n",
    "break\n",
    "continue\n",
    "x = (yield)\n",
    "True = 1\n",
    "None = 1\n",
    "def f(*): pass\n",
    "x = 1;;\n",
    "x := 1\n",
    "print 'hello'\n",
    "exec 'x'\n",
    "x = 1 \\ 2\n",
    "x = '\\N{foo}'\n",
    "if True:\n    pass\n  else:\n    pass\n",
    "match x:\ncase 1:\n    pass\n",
    "emit('G1')\n  emit('G2')\n",
    "x = [i for i in]\n",
    "x = {**}\n",
    "f'{}'\n",
    "f'{x'\n",
    "f'{x!z}'\n",
    "x.1\n",
    "import a.b as\n",
    "assert\n",
    "raise from x\n",
    "global\n",
    "for x in:\n    pass\n",
    "with:\n    pass\n",
    "try:\n    pass\n",
    "try:\n    pass\nx = 1\n",
    # held out: written after the rules
    "emit(f'G1 X{x} Y{y}'\nemit('G28')\n",
    "for i in range(own_vars.count)\n    emit('G4 P10')\n",
    "if printer['extruder']['temperature'] > 200\n    respond_info('hot')\n",
    "respond_info('a' 'b' c)\n",
    "emit('G1 X' + str(10) 'Y5')\n",
    "t = printer['extruder'['target']\n",
    "x = {'a': 1, 'b': 2\nrespond_info(x)\n",
    "def helper(a, b):\n    return a + b\nrespond_info(helper(1, 2)\n",
    "if a:\n    pass\nelif b\n    pass\n",
    "while own_vars.count < 3:\nown_vars.count += 1\n",
    "x = 10mm\n",
    "x = 0x1g\n",
    "x = 1e\n",
    "y = 'it\\'s\n",
    'z = """never\nclosed\n',
    "a = [1, 2, 3]]\n",
    "if x:\n  a = 1\n    b = 2\n",
    "if x:\n    a = 1\n   b = 2\n",
    "emit(\u201cG28\u201d)\n",
    "x = 1 +\n",
    "x = (1 +\n",
    "def f(:\n    pass\n",
    "class A(:\n    pass\n",
    "lambda: = 1\n",
    "for x in range(3): pass\nelse\n    pass\n",
    "with open('a') as f\n    pass\n",
    "try\n    pass\nexcept:\n    pass\n",
    "x = [1, 2, 3\n    4]\n",
    'respond_info(f"{printer.toolhead.position}" x)\n',
    "a = b = \n",
    "print('x'\n",
    "x = y z\n",
    "if x == 1 or\n    pass\n",
    "f(1, x for x in y)\n",
    "except:\n    pass\n",
    "return 5\n",
    "yield 1\n",
    # valid code: no error
    "x = f\"{d['a']} {y!r:>10} {z:.2f}\"\n",
    "x = f'{\"a\"}'\n",
    "s = sum(x for x in range(3))\n",
    "f(a, b=1, *c, **d)\n",
    "x = {k: v for k, v in d.items() if v}\n",
    "x = 1e5 + 0x1F + 1_000 + 1j + 0.5 + 0o17 + 0b101 + .5\n",
    "x = 1if y else 2\n",
    "x = '''multi\nline'''\n",
    "x = 1 + \\\n    2\n",
    "@dec\ndef f():\n    pass\n",
    "g = lambda a, *b, c=1, **d: a\n",
    "if x:\n    pass\nelif y:\n    pass\nelse:\n    pass\n",
    "try:\n    pass\nexcept (A, B) as e:\n    pass\nelse:\n    pass\nfinally:\n    pass\n",
    "x = [\n    1,\n    2,\n]\n",
    "# only a comment\n",
    "",
    "x = r'\\d+' + b'abc'.decode() + rb'x'.decode()\n",
    "async def f():\n    await g()\n",
    "match x:\n    case 1:\n        pass\n    case _:\n        pass\n",
    "x = not a and b or c in d is not e\n",
    "x = a if b else c if d else e\n",
    "def f(a, /, b, *, c):\n    return a\n",
    "x: int = 5\n",
    "x = [*a, *b]\n",
    "print(*args, sep='')\n",
    "if x:\n\tpass\n",
    "x = f'{a}' f\"{b}\" 'c'\n",
    "emit('G1 X%.3f Y%.3f' % (x, y))\n",
]



# runtime errors (--exec): the exception a macro script raises
EXEC_CASES = [
    "d = {}\nd['a']",
    "[1, 2][5]",
    "(1,)[3]",
    "'abc'[10]",
    "1 + 'a'",
    "'a' + 1",
    "'a' * 'b'",
    "[] + ()",
    "None + 1",
    "1 < 'a'",
    "None()",
    "1()",
    "len(5)",
    "for x in 5: pass",
    "a, b = [1]",
    "a, b = [1, 2, 3]",
    "a, = []",
    "int('x')",
    "int('1.5')",
    "float('abc')",
    "int(None)",
    "1 / 0",
    "1.0 / 0",
    "1 // 0",
    "1 % 0",
    "1.5 % 0",
    "divmod(1, 0)",
    "2 ** -1 / 0",
    "0 ** -1",
    "import math\nmath.sqrt(-1)",
    "import math\nmath.log(0)",
    "import math\nmath.log(-1)",
    "import math\nmath.acos(2)",
    "import math\nmath.exp(1000)",
    "10.0 ** 400",
    "{}.x",
    "None.x",
    "(1).foo",
    "'a'.foo()",
    "[].pop()",
    "{}.pop('a')",
    "[1].index(2)",
    "'abc'.index('z')",
    "[1].remove(2)",
    "set().pop()",
    "undefined",
    "def f():\n    x += 1\nf()",
    "def f(a): pass\nf()",
    "def f(a): pass\nf(1, 2)",
    "def f(a): pass\nf(b=1)",
    "def f(): pass\nf(1)",
    "'%d' % 'x'",
    "'%d %d' % (1,)",
    "'%s' % (1, 2)",
    "'{}'.format()",
    "'{0} {2}'.format(1, 2)",
    "'{a}'.format(b=1)",
    "f'{1:x>}{\"a\":d}'",
    "format(1.5, 'd')",
    "'{:d}'.format('a')",
    "round('a')",
    "abs('a')",
    "sum(['a'])",
    "max([])",
    "min()",
    "sorted([1, 'a'])",
    "dict([1])",
    "list(1)",
    "range('a')",
    "range(1, 2, 0)",
    "chr(-1)",
    "ord('ab')",
    "hash([])",
    "{[]: 1}",
    "x = {}\nx[[1]] = 2",
    "'a'.join([1])",
    "str.upper(1)",
    "raise ValueError('bad')",
    "raise Exception",
    "raise KeyError('k')",
    "assert False",
    "assert 1 == 2, 'nope'",
    "import nonexistent_module",
    "from math import nope",
    "def f(): return f()\nf()",
    "x = [1, 2]\nx[1:2] = 5",
    "'abc'[1] = 'x'",
    "(1, 2)[0] = 5",
    "del undefined_name",
    "int('99999999999999999999999') + 'a'",
    "float('inf') // 0",
    "isinstance(1, 2)",
    "getattr(1, 'x')",
    "iter(5)",
    "next(iter([]))",
    "import json\njson.loads('{bad')",
    "import json\njson.dumps(object())",
    "import re\nre.compile('(')",
    "'abc'.split('')",
    "bytes('abc')",
    "b'a' + 'b'",
    "[1, 2, 3][1.5]",
    "{'a': 1}.keys()[0]",
    "print(1, sep=1)",
    "class A:\n    pass\nA().missing",
    "class A:\n    __slots__ = ()\nA().x = 1",
    "lambda: 1\n(lambda x: x)()",
    "import random\nrandom.choice([])",
    "import random\nrandom.randint(5, 1)",
    "import time\ntime.sleep(-1)",
    "int('12', 50)",
    "complex('x')",
    "1 + 1j > 2",
    "x = 5\nx.append(1)",
    "[1, 2].sort(key=1)",
    "''.join(None)",
    "zip(1)",
    "enumerate(None)",
    "str(b'a', 'nope')",
    "'%(a)s' % {}",
    "'%z' % 1",
    "'{:>5.2q}'.format(1.0)",
    "'{' .format(1)",
    "'}' .format(1)",
    "own = None\nown.count += 1",
]


def cpython(src):
    try:
        compile(src, "<test>", "exec")
    except SyntaxError as e:
        return [type(e).__name__, str(e.msg), e.lineno or 0, e.offset or 0]
    return None


def cpython_exec(src):
    # traceback.format_exception_only(type(e), e)[-1] as class, message
    try:
        exec(compile(src, "<test>", "exec"), {})
    except Exception as e:
        t = type(e)
        qualname = t.__qualname__
        if t.__module__ not in ("__main__", "builtins"):
            qualname = t.__module__ + "." + qualname
        return [qualname, str(e)]
    return None


def main():
    exec_mode = "--exec" in sys.argv
    if exec_mode:
        cases, ref = EXEC_CASES, cpython_exec
    else:
        # config scripts have no final newline: every case both ways
        cases = CASES + [c.rstrip("\n") for c in CASES if c.endswith("\n")]
        ref = cpython
    args = [sys.argv[1]] + (["--exec"] if exec_mode else [])
    rs = subprocess.run(
        args, input=json.dumps(cases), capture_output=True, text=True, check=True
    ).stdout.splitlines()
    same = 0
    for src, line in zip(cases, rs):
        py = ref(src)
        k = json.loads(line)
        if py == k:
            same += 1
        else:
            print("DIFF %r\n  cpython %r\n  kaliru  %r" % (src, py, k))
    print("%d/%d identical" % (same, len(cases)))


if __name__ == "__main__":
    main()
