#!/usr/bin/env python3
"""File-output host protocol/config tests against Pellcorp's Python driver.

Run with a Python environment containing Pellcorp's Klippy requirements + SciPy:
  python tools/hx711s_compare.py --reference /path/to/pellcorp/kalico \
    --kaliru /path/to/kaliru --out /tmp/hx711s-comparison
The reference C helper needs tools/ref_trace.patch (older serialqueues without
sq->name can use a constant string for that trace label). No MCU is accessed.
A synthetic dictionary adds the five HX711s command formats to the checked-in
STM32 dictionary. This tests host command encoding, not MCU implementation.
"""
import argparse
import json
import os
import pathlib
import socket
import time
import subprocess
import sys

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--reference', type=pathlib.Path, required=True)
ap.add_argument('--kaliru', type=pathlib.Path, required=True)
ap.add_argument('--out', type=pathlib.Path, required=True)
args = ap.parse_args()
args.reference = args.reference.resolve()
args.kaliru = args.kaliru.resolve()
args.out = args.out.resolve()
args.out.mkdir(parents=True, exist_ok=True)
root = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(args.reference))
from klippy import msgproto

dictionary = json.loads((root/'firmware/tests/ref/stm32f407-usb_klipper.dict').read_text())
formats = ['config_hx711s oid=%c sensor_count=%c gain_channel=%c',
           'add_hx711s oid=%c index=%c dout_pin=%u sclk_pin=%u',
           'query_hx711s oid=%c rest_ticks=%u',
           'query_hx711s_status oid=%c',
           'hx711s_attach_load_cell_probe oid=%c load_cell_probe_oid=%c']
used = set(dictionary['commands'].values()) | set(dictionary['responses'].values())
next_id = max(used) + 1
for fmt in formats:
    dictionary['commands'][fmt] = next_id
    next_id += 1
dictpath = args.out/'hx711s.dict'
dictpath.write_text(json.dumps(dictionary))
parser = msgproto.MessageParser()
parser.process_identify(dictpath.read_bytes(), decompress=False)

def commands(path):
    result = []
    for line in path.read_text().splitlines():
        raw = bytes.fromhex(line.split()[-1])
        pos = 0
        while pos < len(raw):
            mid, _ = parser.msgid_parser.parse(raw, pos)
            fmt = parser.messages_by_id[mid]
            values, pos = fmt.parse(raw, pos)
            if 'hx711s' in fmt.name:
                result.append(dict(command=fmt.name, **values))
    return result

def secondary_api(label, cmd, config, folder):
    # File-input mode disables webhooks. Keep an interactive test host open
    # with a private G-code PTY and file-output MCU; no serial MCU is used.
    uds, tty = folder/(label+'.sock'), folder/(label+'.pty')
    uds.unlink(missing_ok=True)
    tty.unlink(missing_ok=True)
    env = dict(os.environ)
    env.pop('PYTHONHOME', None)
    env.pop('KALIRU_LIBPYTHON', None)
    env.pop('KALIRU_KLIPPY_DIR', None)
    proc = subprocess.Popen(cmd+[str(config), '-I', str(tty), '-o', str(folder/(label+'-api.bin')), '-d', str(dictpath), '-a', str(uds)],
                            cwd=args.reference, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    def request(method, params=None):
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(3)
            client.connect(str(uds))
            client.sendall(json.dumps(dict(id=1, method=method, params=params or {})).encode()+b'\x03')
            data = b''
            while b'\x03' not in data:
                chunk = client.recv(65536)
                assert chunk, 'API socket closed before response'
                data += chunk
            response = json.loads(data.split(b'\x03')[0])
            assert 'error' not in response, response
            return response['result']
    try:
        deadline = time.monotonic()+20
        while True:
            assert proc.poll() is None, proc.communicate()[0]
            try:
                if request('info')['state'] == 'ready': break
            except (OSError, KeyError): pass
            assert time.monotonic()<deadline, 'API failed to become ready'
            time.sleep(.05)
        help = request('gcode/help')
        for command in (['PROBE', 'QUERY_PROBE', 'LOAD_CELL_PROBE', 'LOAD_CELL_QUERY_PROBE'] if label == 'python' else ['PROBE', 'QUERY_PROBE', 'PROBE_ACCURACY']):
            assert command in help, (label, command, help)
        (folder/(label+'-help.json')).write_text(json.dumps(help, indent=2))
        if label == 'rust':
            assert 'libpython' not in pathlib.Path(f'/proc/{proc.pid}/maps').read_text(), 'Native driver loaded Python'
        request('gcode/script', {'script': 'DRIVER_STATUS'})
        proc.terminate()
        log = proc.communicate(timeout=20)[0]
        (folder/(label+'-api.log')).write_text(log)
        assert 'HX711S_STATUS valid=False' in log and 'Traceback' not in log, log
    finally:
        if proc.poll() is None:
            proc.terminate()
            log = proc.communicate(timeout=5)[0]
            (folder/(label+'-api.log')).write_text(log)
        tty.unlink(missing_ok=True)

prefix = '[mcu]\nserial: /tmp/not-a-real-mcu\n[printer]\nkinematics: none\nmax_velocity: 100\nmax_accel: 100\n'
douts = ['PA0', 'PA1', 'PA2', 'PA3', 'PA8']
sclks = ['PB0', 'PB1', 'PB2', 'PB3', 'PB8']

def sensor(count=4, sensor_type='hx711s', section='load_cell'):
    return f'[{section}]\nsensor_type: {sensor_type}\ndout_pins: {", ".join(douts[:count])}\nsclk_pins: {", ".join(sclks[:count])}\n'

cases = []
for count in range(1, 5):
    for kind in ('hx711s', 'hx717s'):
        cases.append((f'{kind}-{count}', prefix+sensor(count, kind), None, 'G4 P100\n'))
# All gain/rate choices exercise the reported config, not just construction.
for kind, rates, gains in [('hx711s', [10, 80], ['A-128', 'B-32', 'A-64']),
                           ('hx717s', [10, 20, 80, 320], ['A-128', 'B-64', 'A-64', 'B-8'])]:
    for rate in rates:
        for gain in gains:
            cases.append((f'{kind}-{rate}-{gain}', prefix+sensor(2, kind)+f'sample_rate: {rate}\ngain: {gain}\n', None, 'G4 P100\n'))
probe = sensor(4, section='load_cell_probe')+'z_offset: 0\n'
cases.append(('primary-probe', prefix+probe, None, 'HELP\nG4 P100\n'))
secondary = prefix+probe+'register_as_probe: False\n[probe]\npin: ^PC9\nz_offset: 0\n[gcode_macro DRIVER_STATUS]\ngcode:\n  { action_respond_info("HX711S_STATUS valid=%s" % printer.load_cell_probe.is_last_tap_valid) }\n'
cases.append(('secondary-probe', secondary, None, 'HELP\nDRIVER_STATUS\nG4 P100\n'))
# SE/KE use a single hx711, but need the same secondary-probe commands/status.
single = secondary.replace('sensor_type: hx711s', 'sensor_type: hx711').replace('dout_pins: PA0, PA1, PA2, PA3', 'dout_pin: PA0').replace('sclk_pins: PB0, PB1, PB2, PB3', 'sclk_pin: PB0')
cases.append(('se-single-secondary', single, None, 'HELP\nDRIVER_STATUS\nG4 P100\n'))
cases.extend([
    ('mismatch', prefix+sensor().replace('sclk_pins: PB0, PB1, PB2, PB3', 'sclk_pins: PB0'), 'must have the same number of entries', ''),
    ('too-many', prefix+sensor(5), 'must specify 1 to 4 sensor pin pairs', ''),
    ('mixed-mcu', prefix+sensor().replace('PA1,', 'second:PA1,')+'[mcu second]\nserial: /tmp/not-a-real-second-mcu\n', 'all pins must be on the same MCU', ''),
    ('rate', prefix+sensor()+'sample_rate: 320\n', 'sample_rate', ''),
    ('gain', prefix+sensor()+'gain: B-8\n', 'gain', ''),
    ('duplicate', prefix+sensor().replace('PA1,', 'PA0,'), 'used multiple times', ''),
])
report = []
for name, config, error, gcode in cases:
    folder = args.out/name
    folder.mkdir(exist_ok=True)
    cfg, gc = folder/'printer.cfg', folder/'input.gcode'
    cfg.write_text(config)
    gc.write_text(gcode)
    decoded = {}
    for label, cmd in [('python', [sys.executable, '-m', 'klippy']), ('rust', [str(args.kaliru)])]:
        trace = folder/(label+'.trace')
        trace.unlink(missing_ok=True)
        env = dict(os.environ, RK_SQ_TRACE=str(trace))
        env.pop('PYTHONHOME', None)
        env.pop('KALIRU_LIBPYTHON', None)
        env.pop('KALIRU_KLIPPY_DIR', None)
        cmd += [str(cfg), '-i', str(gc), '-o', str(folder/(label+'.bin')), '-d', str(dictpath)]
        if '[mcu second]' in config:
            cmd += ['-d', 'second='+str(dictpath)]
        proc = subprocess.run(cmd, cwd=args.reference, env=env, capture_output=True, text=True, timeout=30)
        log = proc.stdout+proc.stderr
        (folder/(label+'.log')).write_text(log)
        if error:
            assert error in log, (name, label, log[-5000:])
            assert 'Exiting (print time' not in log, (name, label, 'unexpected successful startup')
        else:
            assert proc.returncode == 0 and 'Exiting (print time' in log, (name, label, log[-5000:])
            assert 'Traceback' not in log and 'Unknown command' not in log, (name, label, log[-5000:])
            if 'secondary' in name:
                assert 'HX711S_STATUS valid=False' in log, (name, label, log[-5000:])
            decoded[label] = commands(trace)
    if not error:
        assert decoded['python'] == decoded['rust'], (name, decoded)
        if 'single' not in name:
            assert decoded['rust'], (name, 'no HX711s config commands captured')
    if 'secondary' in name:
        for label, cmd in [('python', [sys.executable, '-m', 'klippy']), ('rust', [str(args.kaliru)])]:
            secondary_api(label, cmd, cfg, folder)
    report.append(dict(name=name, result='passed', error=error, commands=decoded.get('rust')))
    print('PASS', name, flush=True)
(args.out/'results.json').write_text(json.dumps(report, indent=2)+'\n')
print(f'{len(report)} differential host cases passed')
