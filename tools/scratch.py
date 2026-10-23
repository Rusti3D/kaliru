# one-off: probe CPython sum() semantics with mixed ints/floats
def neu(items, int_comp):
    f, c = 0.0, 0.0
    for x in items:
        if isinstance(x, int) and not int_comp:
            f += float(x)
            continue
        x = float(x)
        t = f + x
        if abs(f) >= abs(x):
            c += (f - t) + x
        else:
            c += (x - t) + f
        f = t
    if c:
        f += c
    return f

cases = [
    [0.1] * 10,
    [1e16, 1.0, -1e16],
    [0.1, 1, 0.2, 1, 0.3],
    [1e16, 1, -1e16, 0.1],
    [0.1, 10**17, 0.2, -10**17],
    [0.3, 1e100, 7, -1e100],
]
for c in cases:
    print(repr(sum(c)), repr(neu(c, True)), repr(neu(c, False)))
