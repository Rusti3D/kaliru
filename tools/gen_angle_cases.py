# Generates reference cases for kaliru's angle calibration solve
# (extras::angle::calc_calibration, numpy.linalg.lstsq based): lines
# "real|rand|illc angles_hex... = calibration_ints... phase_offset_hex reversed"
# usage (reference venv, from the kalico dir):
#   .venv/bin/python /path/to/gen_angle_cases.py > angle_cases.txt
# then: RK_ANGLE_CASES=angle_cases.txt cargo test --release angle_calibration_vs_python
import math
import random
import types

import numpy

from klippy.extras import angle

random.seed(1)


def cond(angles):
    # numpy's condition number of load_calibration's equations
    full_steps = len(angles)
    nominal_step = 65536.0 / full_steps
    if angles[-2] > angles[-1]:
        angles = list(reversed(angles))
    first = angles.index(min(angles))
    angles = angles[first:] + angles[:first]
    eqs = numpy.zeros((full_steps, 64))
    for step, a in enumerate(angles):
        bucket = int((int(a + 0.5) % 65536) / 1024)
        per = (a - bucket * 1024) / 1024
        eqs[step][bucket] = 1.0 - per
        eqs[step][(bucket + 1) % 64] = per
    return numpy.linalg.cond(eqs)


def load(angles):
    obj = types.SimpleNamespace()
    angle.AngleCalibration.load_calibration(obj, list(angles))
    return obj


out = []
for _ in range(400):
    full_steps = random.choice([200, 200, 400, 100, 64, 50, 48, 32])
    kind = random.random()
    nominal = 65536.0 / full_steps
    if kind < 0.8:
        # a calibrate run: near linear with magnet distortion and noise
        offset = random.uniform(0, 65535)
        sign = random.choice([1, -1])
        amp = random.uniform(0, 400)
        ph = random.uniform(0, 2 * math.pi)
        angs = [
            (offset + sign * i * nominal + amp * math.sin(ph + 2 * math.pi * i / full_steps)
             + random.gauss(0, 3)) % 0xFFFF
            for i in range(full_steps)
        ]
        angs = [round(a, 1) for a in angs]
    else:
        # arbitrary data (rank deficient systems)
        angs = [round(random.uniform(0, 65535), 1) for _ in range(full_steps)]
    obj = load(angs)
    out.append(
        "%s %s = %s %s %d"
        % (
            "illc" if cond(angs) > 1e8 else ("real" if kind < 0.8 else "rand"),
            " ".join(map(float.hex, angs)),
            " ".join(map(str, obj.calibration)),
            float.hex(obj.angle_phase_offset),
            obj.calibration_reversed,
        )
    )
print("\n".join(out))
