# Reference cases for kaliru's numpy.fft.rfft / kaiser / i0 ports:
# "rfft x_hex... = re_hex,im_hex ...", "kaiser M beta = ...", "i0 x = y"
# usage: ~/rk/kalico/.venv/bin/python gen_fft_cases.py > fft_cases.txt
# then RK_FFT_CASES=fft_cases.txt cargo test --release fft_vs_numpy
import random
import numpy as np
random.seed(3)
h = float.hex
out = []
for k in range(0, 13):
    for _ in range(4):
        n = 1 << k
        x = [random.gauss(0, random.choice([1e-3, 1, 1e3])) for _ in range(n)]
        r = np.fft.rfft(np.array(x))
        out.append("rfft %s = %s" % (" ".join(map(h, x)), " ".join("%s,%s" % (h(c.real), h(c.imag)) for c in r)))
for m in [1, 2, 3, 8, 64, 256, 1024, 2048, 4096]:
    for beta in [6.0, 0.5, 14.0]:
        w = np.kaiser(m, beta)
        out.append("kaiser %d %s = %s" % (m, h(beta), " ".join(map(h, w))))
for _ in range(3000):
    x = random.uniform(-30, 30)
    out.append("i0 %s = %s" % (h(x), h(float(np.i0(x)))))
print("\n".join(out))
