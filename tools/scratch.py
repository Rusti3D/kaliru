# Which LU/solve variant reproduces numpy.linalg.solve (OpenBLAS dgesv) for 3x3?
import math
import random

import numpy as np

random.seed(2)
fma = math.fma


def rnd():
    k = random.random()
    if k < 0.4:
        return random.uniform(-5.0, 5.0)
    if k < 0.8:
        return random.uniform(-300.0, 300.0)
    return random.uniform(-1.0, 1.0) * 10 ** random.randint(-8, 8)


def solve(a, b, use_fma, left_looking, recip_scal, dot_fma_order):
    n = 3
    A = [row[:] for row in a]  # row major A[i][j]
    b = b[:]
    ipiv = [0] * n
    mac = (lambda acc, x, y: fma(x, y, acc)) if use_fma else (lambda acc, x, y: acc + x * y)
    msub = (lambda acc, x, y: fma(-x, y, acc)) if use_fma else (lambda acc, x, y: acc - x * y)
    if left_looking:
        for j in range(n):
            # apply previous interchanges to column j
            for i in range(j):
                ip = ipiv[i]
                if ip != i:
                    A[i][j], A[ip][j] = A[ip][j], A[i][j]
            # solve L11 * x = column (unit lower) via dots
            for i in range(1, j):
                d = 0.0
                for k in range(i):
                    d = mac(d, A[i][k], A[k][j])
                A[i][j] -= d
            # gemv: b[j:] -= A[j:, :j] * col[:j]
            for i in range(j, n):
                if dot_fma_order:
                    t = 0.0
                    for k in range(j):
                        t = mac(t, A[i][k], A[k][j])
                    A[i][j] -= t
                else:
                    for k in range(j):
                        A[i][j] = msub(A[i][j], A[i][k], A[k][j])
            jp = max(range(j, n), key=lambda i: (abs(A[i][j]), -i))
            ipiv[j] = jp
            t1 = A[jp][j]
            if t1 == 0:
                raise ZeroDivisionError
            if jp != j:
                A[j][: j + 1], A[jp][: j + 1] = A[jp][: j + 1], A[j][: j + 1]
            if recip_scal:
                r = 1.0 / t1
                for i in range(j + 1, n):
                    A[i][j] *= r
            else:
                for i in range(j + 1, n):
                    A[i][j] /= t1
    else:
        for j in range(n):
            jp = max(range(j, n), key=lambda i: (abs(A[i][j]), -i))
            ipiv[j] = jp
            if A[jp][j] == 0:
                raise ZeroDivisionError
            A[j], A[jp] = A[jp], A[j]
            r = 1.0 / A[j][j]
            for i in range(j + 1, n):
                A[i][j] = A[i][j] * r if recip_scal else A[i][j] / A[j][j]
            for i in range(j + 1, n):
                for k in range(j + 1, n):
                    A[i][k] = msub(A[i][k], A[i][j], A[j][k])
    # left-looking getf2 applies later swaps to the earlier columns only
    # through the row swaps; finish: swap the remaining columns
    for j in range(n):
        if ipiv[j] != j:
            b[j], b[ipiv[j]] = b[ipiv[j]], b[j]
    for k in range(n):
        for i in range(k + 1, n):
            b[i] = msub(b[i], b[k], A[i][k])
    for k in range(n - 1, -1, -1):
        b[k] /= A[k][k]
        for i in range(k):
            b[i] = msub(b[i], b[k], A[i][k])
    return b


cases = []
for _ in range(3000):
    a = [[rnd() for _ in range(3)] for _ in range(3)]
    bb = [rnd() for _ in range(3)]
    try:
        want = np.linalg.solve(np.array(a), np.array(bb)).tolist()
    except np.linalg.LinAlgError:
        continue
    cases.append((a, bb, want))

for use_fma in (False, True):
    for left in (False, True):
        for recip in (True, False):
            for dot_order in ((False, True) if left else (False,)):
                bad = 0
                for a, bb, want in cases:
                    try:
                        got = solve(a, bb, use_fma, left, recip, dot_order)
                    except ZeroDivisionError:
                        bad += 1
                        continue
                    if got != want:
                        bad += 1
                print("fma=%d left=%d recip=%d dotorder=%d: %d/%d" % (use_fma, left, recip, dot_order, bad, len(cases)))
