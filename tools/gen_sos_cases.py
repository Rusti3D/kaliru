# Generates reference cases for kaliru's sos_filter design
# (extras::load_cell::sos_filter, scipy.signal based): lines
# "sps hp hp_order lp lp_order notches... quality = sections states" with the
# fixed point Q2 sections / Q16 states as python lists, or "err <class>"
# usage (a venv with kalico's requirements + scipy, from the kalico dir):
#   PYTHONPATH=. ~/rk/scipyenv/bin/python gen_sos_cases.py > sos_cases.txt
# then: RK_SOS_CASES=sos_cases.txt cargo test --release sos_filter_vs_scipy
import math
import random

from klippy.extras.load_cell import sos_filter

random.seed(1)
h = float.hex


def opt(v):
    return "None" if v is None else h(v)


out = []
for _ in range(3000):
    sps = random.choice([80.0, 320.0, 400.0, 600.0, 800.0, 1000.0, 1200.0, 2000.0, random.uniform(50, 4000)])
    maxf = math.floor(sps / 2.0)
    hp = random.choice([None, round(random.uniform(0.1, 20.0), random.choice([1, 3, 6]))])
    lp = random.choice([None, random.uniform(min(80.0, maxf - 1.0), maxf - 0.01)])
    if random.random() < 0.05:
        lp = random.uniform(maxf, sps)  # invalid
    notches = [random.uniform(1.0, maxf - 1.0) for _ in range(random.choice([0, 0, 1, 2]))]
    q = random.uniform(0.5, 6.0)
    ho, lo = random.choice([1, 2]), random.choice([1, 2])
    try:
        d = sos_filter.DigitalFilter(sps, None, hp, ho, lp, lo, notches, q)
        f = sos_filter.FixedPointSosFilter(d.get_filter_sections(), d.get_initial_state(), 2, 16)
        res = "%s %s" % (f.get_filter_sections(), f.get_initial_state())
    except Exception as e:
        res = "err %s" % (type(e).__name__,)
    args = [h(sps), opt(hp), str(ho), opt(lp), str(lo)] + [h(n) for n in notches] + [h(q)]
    out.append("%s = %s" % (" ".join(args), res))
print("\n".join(out))
