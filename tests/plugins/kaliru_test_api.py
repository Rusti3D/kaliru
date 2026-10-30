# kaliru plugin layer API probe (tests/cases/plugins/api.test): TEST_API
# walks every printer object, reads each attribute and calls each get_* /
# is_* method that needs no arguments, as the Kalico class has them (names
# from kaliru's mirror stubs, through the object's MRO), one level into the
# objects found. Values are logged with objects as their class names, so a
# Kalico and a kaliru run can be diffed.
import logging
import os
import re
from collections import abc

# objects not descended into (reached from almost every object)
NO_DESCEND = ("printer", "reactor", "gcode", "toolhead", "config", "mcu")


def show(v, depth=0):
    if v is None or isinstance(v, (bool, int, float, str, bytes)):
        r = repr(v)
        return r if len(r) <= 200 else r[:200] + "..."
    # kaliru's live list / dict views as the list / dict they mirror
    seq = isinstance(v, abc.Sequence)
    if seq or isinstance(v, (abc.Set, abc.Mapping)):
        kind = "tuple" if isinstance(v, tuple) else "list" if seq else "set"
        if depth >= 3:
            return "%s[%d]" % (kind, len(v))
        if isinstance(v, abc.Mapping):
            keys = sorted(v, key=repr)
            s = ", ".join(
                "%s: %s" % (show(k, depth + 1), show(v[k], depth + 1))
                for k in keys[:30]
            )
            return "{%s%s}" % (s, ", ..." if len(keys) > 30 else "")
        items = list(v)
        if not seq:
            items.sort(key=repr)
        s = ", ".join(show(x, depth + 1) for x in items[:30])
        more = ", ..." if len(items) > 30 else ""
        return "%s(%s%s)" % (kind, s, more)
    # kaliru hands python native callables for Kalico's bound methods
    if callable(v) and not isinstance(v, type):
        return "<fn>"
    return "<%s>" % type(v).__name__


def plain(v):
    return v is None or isinstance(
        v, (bool, int, float, str, bytes, abc.Sequence, abc.Set, abc.Mapping)
    ) or callable(v)


class TestApi:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mirror = config.get("mirror")
        self.names = {}
        self.printer.lookup_object("gcode").register_command(
            "TEST_API", self.cmd_TEST_API
        )

    def class_names(self, cls):
        # (attributes, argument-free get_/is_ methods) of one class
        key = (cls.__module__, cls.__name__)
        if key not in self.names:
            mod = cls.__module__
            if not mod.startswith("klippy."):
                mod = "klippy." + mod
            try:
                with open(os.path.join(self.mirror, mod + ".py")) as f:
                    text = f.read()
            except OSError:
                text = ""
            m = re.search(
                r"^class %s\b[^\n]*\n((?:[ \t]+[^\n]*\n|\n)*)" % cls.__name__,
                text,
                re.M,
            )
            body = m.group(1) if m else ""
            attrs = re.findall(r"^    (\w+) = _k\.Attr\(", body, re.M)
            methods = [
                name
                for name, params in re.findall(
                    r"^    def ((?:get|is)_\w+)\(self((?:, [^)]*)?)\):", body,
                    re.M,
                )
                if name != "get_status"
                and all("=" in p for p in params.split(",")[1:])
            ]
            self.names[key] = (attrs, methods)
        return self.names[key]

    def probe(self, obj, path, depth):
        attrs, methods = [], []
        for cls in type(obj).__mro__:
            a, m = self.class_names(cls)
            attrs += [n for n in a if n not in attrs]
            methods += [n for n in m if n not in methods]
        found = []
        for name in sorted(attrs):
            try:
                v = getattr(obj, name)
                s = show(v)
            except Exception as e:
                logging.info("api %s.%s ! %s: %s", path, name,
                             type(e).__name__, str(e)[:200])
                continue
            logging.info("api %s.%s = %s", path, name, s)
            found.append((name, v))
        for name in sorted(methods):
            try:
                v = getattr(obj, name)()
                s = show(v)
            except Exception as e:
                logging.info("api %s.%s() ! %s: %s", path, name,
                             type(e).__name__, str(e)[:200])
                continue
            logging.info("api %s.%s() = %s", path, name, s)
            found.append((name + "()", v))
        if depth < 1:
            for name, v in found:
                if not plain(v) and name.rstrip("()") not in NO_DESCEND:
                    self.probe(v, "%s.%s" % (path, name), depth + 1)

    def cmd_TEST_API(self, gcmd):
        for name, obj in self.printer.lookup_objects():
            if obj is not self:
                self.probe(obj, name, 0)
        gcmd.respond_info("api probe done")


def load_config(config):
    return TestApi(config)
