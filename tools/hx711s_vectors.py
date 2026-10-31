#!/usr/bin/env python3
"""Generate decode vectors from Pellcorp's unmodified HX711s host driver.

Usage: python3 tools/hx711s_vectors.py /path/to/pellcorp/kalico > tests/data/hx711s.json
Reference: pellcorp/kalico 070a3066f2ffe1f0fdad291798263418b1c86004.
Only imports are stubbed; _convert_samples runs from the reference source.
No printer, MCU, NumPy or SciPy is needed.
"""
import importlib.util
import json
import logging
import pathlib
import random
import sys
import types

logging.disable(logging.CRITICAL)
for name in ('klippy', 'klippy.extras', 'klippy.extras.load_cell',
             'klippy.extras.bulk_sensor', 'klippy.mcu',
             'klippy.extras.load_cell.interfaces'):
    module = types.ModuleType(name)
    module.__path__ = []
    sys.modules[name] = module
sys.modules['klippy.mcu'].MCU = object
interfaces = sys.modules['klippy.extras.load_cell.interfaces']
interfaces.BulkAdcData = dict
interfaces.BulkAdcDataCallback = object
interfaces.LoadCellSensor = object
path = pathlib.Path(sys.argv[1]) / 'klippy/extras/load_cell/hx711s.py'
spec = importlib.util.spec_from_file_location('klippy.extras.load_cell.hx711s', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

cases = [('empty', []), ('signed_limits', [[1.123456789, [-8388608, -1, 0, 8388607]]])]
for count in range(1, 5):
    rng = random.Random(count)
    samples = [[i / 80 + 0.0000005, [rng.randrange(-8388608, 8388608) for _ in range(count)]] for i in range(4)]
    cases.append((f'{count}_channels', samples))
    for error in (-2147483648, 1073741824):
        cases.append((f'{count}_channels_error_{error}', samples[:2] + [[1.5, [error] * count]] + samples[2:]))
result = []
for name, samples in cases:
    sensor = object.__new__(module.HX711SBase)
    sensor.name = 'test'
    sensor.last_error_count = 0
    rows = [[time, *channels] for time, channels in samples]
    sensor._convert_samples(rows)
    result.append(dict(name=name, samples=samples, expected=rows, errors=sensor.last_error_count))
print("[\n" + ",\n".join("  " + json.dumps(case) for case in result) + "\n]")
