#!/usr/bin/env python3
# Runs Kalico's test/klippy/*.test cases with a given klippy command and keeps
# every output, so two runs (reference vs Rust, or reference vs reference) can
# be compared with refdiff.py. Test files are parsed exactly like Kalico's
# test/klippy/conftest.py (KlippyTest.collect / KlippyTestItem.runtest).
#
# Per run, <out>/<test>/<n>/ holds: trace (every message handed to
# serialqueue, see ref_trace.patch / RK_SQ_TRACE), output* (serial bytes per
# MCU), log (klippy stdout+stderr), exit (exit code), cmd (config, dicts).
#
# usage: refrun.py --out DIR [--klippy "python -m klippy"] [--dictdir DIR]
#                  [--cwd DIR] [-j N] [test files...]
import argparse
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

KALICO = pathlib.Path.home() / "rk/kalico"


def collect(path, dict_path):
    # Mirrors test/klippy/conftest.py KlippyTest.collect
    def rel(*parts, root=None):
        return (root or path.parent).joinpath(*parts).resolve()

    items = []
    multi_test = should_fail = False
    gcode, config_file, dictionaries = [], None, []

    def item():
        return dict(config=config_file, dicts=list(dictionaries),
                    gcode=list(gcode), should_fail=should_fail)

    for line in path.open("r", encoding="utf-8"):
        parts = line.strip().split()
        if not parts or line.strip().startswith("#"):
            continue
        elif parts[0] == "SHOULD_FAIL":
            should_fail = True
        elif parts[0] == "GCODE":
            gcode = rel(parts[1]).open("r", encoding="utf-8").readlines()
        elif parts[0] == "DICTIONARY":
            dictionaries = [str(rel(parts[1], root=dict_path))]
            for mcu_path in parts[2:]:
                mcu, fname = mcu_path.split("=", maxsplit=1)
                dictionaries.append(f"{mcu}={rel(fname, root=dict_path)}")
        elif parts[0] == "CONFIG":
            if config_file and not multi_test:
                multi_test = True
                items.append(item())
            config_file = rel(parts[1])
            if multi_test:
                items.append(item())
        else:
            gcode.append(line.strip())
    if not multi_test:
        items.append(item())
    return items


def run_one(args, test, n, it):
    dest = args.out / test.stem / str(n)
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    tmp = pathlib.Path(tempfile.mkdtemp())
    try:
        gcode_file = tmp / "_test_.gcode"
        gcode_file.write_text("\n".join(it["gcode"]) + "\n")
        cmd = shlex.split(args.klippy) + [
            str(it["config"]), "-i", str(gcode_file),
            "-o", str(tmp / "_test_.output"), "-v"]
        for df in it["dicts"]:
            cmd += ["-d", df]
        # a fresh HOME per run: modules keep files under ~ (save_variables'
        # ~/variables.cfg), which would carry state between runs
        home = tmp / "home"
        home.mkdir()
        env = dict(os.environ, RK_SQ_TRACE=str(tmp / "_test_.trace"),
                   HOME=str(home))
        p = subprocess.run(cmd, cwd=args.cwd, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=args.timeout,
                           env=env)
        rc = p.returncode
        log = p.stdout
        for f in tmp.iterdir():
            if f.name.startswith(("_test_.output", "_test_.trace")):
                shutil.copy(f, dest / f.name.replace("_test_.", ""))
        # tmp paths differ per run
        log = log.replace(str(tmp).encode(), b"<TMP>")
    except subprocess.TimeoutExpired as e:
        # keep what was logged before the hang
        rc, log = "timeout", (e.output or b"").replace(str(tmp).encode(), b"<TMP>")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    (dest / "log").write_bytes(log)
    (dest / "exit").write_text(f"{rc}\n")
    (dest / "cmd").write_text(
        f"config {it['config']}\ndicts {' '.join(it['dicts'])}\n"
        f"should_fail {it['should_fail']}\n")
    ok = (rc != 0) if it["should_fail"] else (rc == 0)
    return f"{'ok  ' if ok else 'FAIL'} {test.stem}/{n} rc={rc}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--klippy", default=f"{KALICO}/.venv/bin/python -m klippy")
    ap.add_argument("--cwd", default=str(KALICO))
    ap.add_argument("--dictdir", type=pathlib.Path, default=pathlib.Path("/ci_build/dict"))
    ap.add_argument("--timeout", type=int, default=600)
    ap.add_argument("-j", type=int, default=os.cpu_count())
    ap.add_argument("tests", nargs="*", type=pathlib.Path)
    args = ap.parse_args()
    tests = args.tests or sorted((KALICO / "test/klippy").glob("*.test"))
    jobs = [(t, n, it) for t in tests
            for n, it in enumerate(collect(t.resolve(), args.dictdir))]
    with ThreadPoolExecutor(args.j) as ex:
        results = list(ex.map(lambda j: run_one(args, *j), jobs))
    for r in results:
        print(r)
    bad = sum(r.startswith("FAIL") for r in results)
    print(f"{len(results) - bad}/{len(results)} runs as expected")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
