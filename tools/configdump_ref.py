#!/usr/bin/env python3
# Reference twin of kaliru's src/bin/configdump.rs: reads a config with
# Kalico's configfile.PrinterConfig (fake printer: no gcode, no objects) and
# prints the same dump. Run with the Kalico venv, PYTHONPATH=~/rk/kalico.
# usage: configdump_ref.py <config file>
import json
import sys

from klippy import configfile


class FakeGcode:
    ready_gcode_handlers = {}

    def register_command(self, *args, **kwargs):
        pass


class FakePrinter:
    def __init__(self, path):
        self.path = path

    def lookup_object(self, name, default=None):
        return FakeGcode()

    def get_start_args(self):
        return {"config_file": self.path}

    def lookup_objects(self, module=None):
        return []


def show(what, fn):
    try:
        print("  %s %s" % (what, json.dumps(fn())))
    except Exception as e:
        print("  %s ERR %s" % (what, e))


def dump_parser(name, fc):
    print("== %s sections" % (name,))
    for k, v in fc._defaults.items():
        print("D %s = %r" % (k, v))
    for s in fc.sections():
        print("[%s]" % (s,))
        for k, v in fc._sections[s].items():
            try:
                got = repr(fc.get(s, k))
            except Exception as e:
                got = "ERR %s" % (e,)
            print("  %s = %r -> %s" % (k, v, got))


class FakeGcmd:
    def get_int(self, name, default, minval=None, maxval=None):
        return 0  # RESTART=0


class SaveGcode(FakeGcode):
    class error(Exception):
        pass

    def respond_info(self, msg):
        print("info %s" % (msg,))


class SavePrinter(FakePrinter):
    gcode = SaveGcode()

    def lookup_object(self, name, default=None):
        return self.gcode


def save(path, autosave_includes, changes):
    import glob, os, re
    from klippy.extras import danger_options

    class DO:
        pass

    danger_options.DANGER_OPTIONS = DO()
    danger_options.DANGER_OPTIONS.autosave_includes = autosave_includes == "1"
    printer = SavePrinter(path)
    pconfig = configfile.PrinterConfig(printer)
    try:
        pconfig.read_main_config()
    except configfile.error as e:
        print("ERROR %s" % (e,))
        return
    for change in changes:
        if change.startswith("-"):
            pconfig.remove_section(change[1:])
        else:
            so, value = change.split("=", 1)
            section, option = so.split(":", 1)
            try:
                pconfig.set(section, option, value)
            except Exception as e:
                print("set ERR %s" % (e,))
    print("pending %s %s" % (str(pconfig.save_config_pending).lower(),
                             json.dumps(pconfig.status_save_pending)))
    try:
        pconfig.cmd_SAVE_CONFIG(FakeGcmd())
    except Exception as e:
        print("SAVE ERR %s" % (e,))
    print("pending %s" % (str(pconfig.save_config_pending).lower(),))
    d = os.path.dirname(path)
    for f in sorted(glob.glob(d + "/**", recursive=True)):
        if os.path.isfile(f):
            name = re.sub(r"-\d{8}_\d{6}", "-<DATE>", f[len(d):], count=1)
            print("== file %s\n%s" % (name, open(f).read()))


def main():
    if sys.argv[1] == "--save":
        return save(sys.argv[2], sys.argv[3], sys.argv[4:])
    path = sys.argv[1]
    pconfig = configfile.PrinterConfig(FakePrinter(path))
    try:
        cfg = pconfig.read_main_config()
    except configfile.error as e:
        print("ERROR %s" % (e,))
        return
    dump_parser("main", cfg.fileconfig)
    dump_parser("autosave", pconfig.autosave.fileconfig)
    print("== log_config")
    print("\n".join(["===== Config file =====", pconfig._build_config_string(cfg),
                     "======================="]))
    print("== tracking after interpolation")
    for (s, o), v in cfg.access_tracking.items():
        print("T %r %r = %s" % (s, o, json.dumps(v)))
    print("== getters")
    for s in cfg.fileconfig.sections():
        sec = cfg.getsection(s)
        for o in cfg.fileconfig.options(s):
            print("[%s] %s" % (s, o))
            show("getint", lambda: sec.getint(o))
            show("getfloat", lambda: sec.getfloat(o))
            show("getfloat_min0", lambda: sec.getfloat(o, minval=0.0))
            show("getfloat_above1", lambda: sec.getfloat(o, above=1))
            show("getboolean", lambda: sec.getboolean(o))
            show("getlist", lambda: sec.getlist(o))
            show("getfloatlist", lambda: sec.getfloatlist(o))
            show("getintlist2", lambda: sec.getintlist(o, count=2))
            show("getlists", lambda: sec.getlists(o, seps=(",", "\n")))
            show("getscript", lambda: sec.getscript(o))
        show("missing", lambda: sec.get("no_such_option"))
        show("default_int", lambda: sec.getfloat("no_such_float", 300))
    try:
        pconfig.check_unused_options(cfg, False)
        print("== status")
        print(json.dumps(pconfig.get_status(0.0)))
    except configfile.error as e:
        print("== status ERROR %s" % (e,))


if __name__ == "__main__":
    main()
