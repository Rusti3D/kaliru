# Generates reference cases for kaliru's numpy helpers (mathutil np_*):
# lines "kind args... = results..." with floats as float.hex()
# usage (reference venv): .venv/bin/python gen_numpy_cases.py > cases.txt
# then: RK_NUMPY_CASES=cases.txt cargo test --release numpy_vs_python
import random

import numpy as np

random.seed(1)
h = float.hex


def rnd():
    k = random.random()
    if k < 0.4:
        return random.uniform(-5.0, 5.0)
    if k < 0.8:
        return random.uniform(-300.0, 300.0)
    return random.uniform(-1.0, 1.0) * 10 ** random.randint(-8, 8)


out = []
for _ in range(3000):
    n = random.randint(1, 20)
    v = [rnd() for _ in range(n)]
    out.append("std1 %s = %s" % (" ".join(map(h, v)), h(float(np.std(v)))))
    out.append("mean1 %s = %s" % (" ".join(map(h, v)), h(float(np.mean(v)))))
for _ in range(2000):
    m, n = random.randint(1, 20), random.randint(1, 7)
    rows = [[rnd() for _ in range(n)] for _ in range(m)]
    flat = [x for r in rows for x in r]
    std0 = np.std(rows, axis=0).tolist()
    mean0 = np.mean(rows, axis=0).tolist()
    args = "%d %d %s" % (m, n, " ".join(map(h, flat)))
    out.append("std0 %s = %s" % (args, " ".join(map(h, std0))))
    out.append("mean0 %s = %s" % (args, " ".join(map(h, mean0))))
for _ in range(3000):
    a = [[rnd() for _ in range(3)] for _ in range(3)]
    b = [rnd() for _ in range(3)]
    try:
        sol = np.linalg.solve(np.array(a), np.array(b)).tolist()
        res = " ".join(map(h, sol))
    except np.linalg.LinAlgError:
        res = "err"
    flat = [x for r in a for x in r] + b
    out.append("solve %s = %s" % (" ".join(map(h, flat)), res))
for _ in range(3000):
    xa, ya, za, ox, oy = [rnd() for _ in range(5)]
    v = np.array([ox, oy, za])
    r = np.array([1, 0, xa])
    s = np.array([0, 1, ya])
    cp = np.cross(r, s)
    nf = np.append(cp, np.dot(cp, v)).tolist()
    out.append("normal %s = %s" % (" ".join(map(h, [xa, ya, za, ox, oy])), " ".join(map(h, nf))))
print("\n".join(out))
