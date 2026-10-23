import ast, json
tests = ["abc", "1 2", "[1,", "", " 1", "\t1", "1 # c", "'a' 'b'", "-1", "+1.5", "--1", "1+2j", "(1,)", "()", "{1,2}", "{'a':[1,(2,3)]}",
 "b'x'", "f'x'", "0x10", "1_000", "1e3", ".5", "5.", "None", "True", "[1, 2,]", "{", "'abc", "1 +", "a.b", "x()", "[x]", "1.2.3",
 "'\\x4'", "r'\\n'", "'''a\nb'''", "'a\\\nb'", "[\n1,\n2]", "1\n2", "nan", "-inf", "1e999", "- 1", "not 1", "1 if 1 else 2", "{**{}}", "[*a]",
 "set()", "0b11", "0o7", "07", "1__0", "u'x'", "12345678901234567890123", "'\\d'", "1j", "-1j", "(1)", "((1,2))", "[1,[2,[3]]]",
 "{'a':1,'a':2}", "{1:2}", "{(1,2):3}", "True + 1", "1 - 2", "1+2", "[1]+[2]", "'a' + 'b'", "~1", "1e", "0x", "abc def", "1)", "(1", "]"]
for t in tests:
    try:
        v = ast.literal_eval(t)
        try:
            json.dumps(v, separators=(",", ":"))
            print(repr(t), "->", repr(v))
        except Exception as e:
            print(repr(t), "JSON!!", type(e).__name__, str(e))
    except Exception as e:
        print(repr(t), "!!", type(e).__name__, str(e))
