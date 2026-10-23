# Reference cases for kaliru's shaper_calibrate port: json lines with the
# synthetic accelerometer samples and python's results (psds as float.hex,
# find_best_shaper logger lines, best shaper, csv text)
# usage (from the kalico dir): PYTHONPATH=. ~/rk/kalico/.venv/bin/python gen_shaper_cases.py > shaper_cases.jsonl
# then RK_SHAPER_CASES=shaper_cases.jsonl cargo test --release shaper_vs_python
import json, math, os, random, tempfile
import numpy as np
from klippy.extras import shaper_calibrate

random.seed(7)
h = float.hex


def samples(rate, secs, freqs):
    out = []
    t = 100.0 + random.random()
    for i in range(int(rate * secs)):
        t += 1.0 / rate * random.uniform(0.98, 1.02)
        vals = []
        for ax in range(3):
            v = random.gauss(0, 30)
            for f, a in freqs[ax]:
                v += a * math.sin(2 * math.pi * f * t)
            vals.append(round(v, 2))
        out.append((round(t, 6), vals[0], vals[1], vals[2]))
    return out


helper = shaper_calibrate.ShaperCalibrate(None)
for case in range(6):
    rate = random.choice([3200.0, 1600.0, 4000.0, 1344.0])
    secs = random.uniform(1.5, 3.0)
    fr = [[(random.uniform(20, 90), random.uniform(50, 2000)) for _ in range(2)] for _ in range(3)]
    s1 = samples(rate, secs, fr)
    s2 = samples(rate, secs * 0.8, fr) if case % 2 else None
    d = helper.calc_freq_response(np.array(s1))
    d.set_numpy(np)
    rec = {"s1": [[h(v) for v in r] for r in s1], "s2": None if s2 is None else [[h(v) for v in r] for r in s2]}
    rec["psd_x"] = [h(v) for v in d.psd_x]
    rec["freqs"] = [h(v) for v in d.freq_bins]
    if s2 is not None:
        d2 = helper.calc_freq_response(np.array(s2))
        d.add_data(d2)
    d.normalize_to_frequencies()
    rec["psd_sum"] = [h(v) for v in d.psd_sum]
    scv = random.choice([5.0, 3.0])
    max_sm = random.choice([None, 0.2])
    max_freq = random.choice([None, 150.0, 300.0])
    logs = []
    best, all_shapers = helper.find_best_shaper(d, scv=scv, max_smoothing=max_sm, max_freq=max_freq, logger=logs.append)
    rec.update(scv=scv, max_smoothing=max_sm, max_freq=max_freq, logs=logs, best=[best.name, h(float(best.freq))])
    fn = os.path.join(tempfile.gettempdir(), "kaliru_shaper.csv")
    helper.save_calibration_data(fn, d, all_shapers, max_freq, 75.0)
    rec["csv"] = open(fn).read()
    rec["noise"] = "%.6f" % (helper.calc_freq_response(np.array(s1)).psd_x.mean(),)
    print(json.dumps(rec))
