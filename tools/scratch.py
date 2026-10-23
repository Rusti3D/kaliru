# one-off: print math.hypot results for kaliru's py_hypot test
import math, random, struct
random.seed(5)
vals = []
for i in range(2000):
    a = random.uniform(-500, 500) * 10 ** random.randint(-8, 3)
    b = random.uniform(-500, 500) * 10 ** random.randint(-8, 3)
    vals.append((a, b))
vals += [(3.0, 4.0), (1e-310, 3e-310), (0.0, 5.0), (1e300, 1e300)]
for a, b in vals:
    print(a.hex(), b.hex(), math.hypot(a, b).hex())
