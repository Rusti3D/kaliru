#!/usr/bin/env python3
# Generates plugin-layer handlers (src/plugins/impls/generated.rs) for the
# mirrored Kalico names that map one to one onto kaliru's Rust port: a
# method with the same name (leading "_" dropped, cmd_FOO -> cmd_foo) and
# arity, a constructor (`new`), a module function, or a field of the same
# name, whose argument and return types the layer converts (convert.rs).
# Names already handled by the hand-written impls are left to them. Rust
# methods / fields it uses that are private are made `pub` in place.
#
# usage: gen_bindings.py [--report] [module ...]   (default: all mirror modules)
import ast
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
MIRROR = ROOT / "src/plugins/mirror"
OUT = ROOT / "src/plugins/impls/generated.rs"

# --------------------------------------------------------------- the mirror


def mirror_modules():
    return sorted(p.stem for p in MIRROR.glob("klippy*.py"))


def parse_mirror(mod):
    """[(kind, class or None, name, args, id)]: kind method / ctor / func / attr"""
    tree = ast.parse((MIRROR / f"{mod}.py").read_bytes().decode())
    out = []

    def call_id(fn):
        for n in ast.walk(fn):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name) \
                    and n.func.value.id == "_k" and n.func.attr in ("call", "new", "fcall"):
                return n.func.attr
        return None

    def args_of(fn, skip_self):
        a = fn.args
        if a.vararg or a.kwarg or a.kwonlyargs:
            return None
        names = [x.arg for x in a.args]
        return names[1:] if skip_self else names

    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and call_id(node) == "fcall":
            args = args_of(node, False)
            if args is not None:
                out.append(("func", None, node.name, args))
        elif isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, ast.FunctionDef):
                    if any(isinstance(d, ast.Name) and d.id in ("staticmethod", "classmethod") for d in item.decorator_list):
                        continue
                    kind = call_id(item)
                    args = args_of(item, True)
                    if kind is None or args is None:
                        continue
                    out.append(("ctor" if kind == "new" else "method", node.name, item.name, args))
                elif isinstance(item, ast.Assign) and isinstance(item.value, ast.Call) \
                        and isinstance(item.value.func, ast.Attribute) and item.value.func.attr == "Attr":
                    out.append(("attr", node.name, item.targets[0].id, None))
    return out


def handled_names():
    stubs = subprocess.run([sys.executable, "-I", str(ROOT / "tools/mirror_stubs.py"), str(ROOT), "--list", "--hand"],
                           capture_output=True, text=True).stdout.split()
    return set(stubs)

# ---------------------------------------------------------------- rust source


# Kalico modules kaliru keeps elsewhere
MODULE_FILES = {
    "klippy.extras.display.display": "src/extras/display/mod.rs",
}


def rust_file(mod):
    if mod in MODULE_FILES:
        return ROOT / MODULE_FILES[mod]
    parts = mod.split(".")[1:]
    base = ROOT / "src" / pathlib.Path(*parts)
    for cand in (base.with_suffix(".rs"), base / "mod.rs"):
        if cand.exists() and cand.name in {x.name for x in cand.parent.iterdir()}:
            return cand
    return None


def strip_code(text):
    """the text with comments and string / char literal contents blanked
    (same length: offsets stay valid)"""
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            for k in range(i, j):
                out[k] = " "
            i = j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                if out[k] != "\n":
                    out[k] = " "
            i = j
        elif c == '"' or (c == "r" and re.match(r'r#*"', text[i:i + 4]) and (i == 0 or not text[i - 1].isalnum())):
            if c == "r":
                m = re.match(r'r(#*)"', text[i:])
                end = '"' + m.group(1)
                j = text.find(end, i + len(m.group(0)))
                j = n if j < 0 else j + len(end)
                for k in range(i + len(m.group(0)), j - len(end)):
                    if out[k] != "\n":
                        out[k] = " "
                i = j
            else:
                j = i + 1
                while j < n and text[j] != '"':
                    j += 2 if text[j] == "\\" else 1
                for k in range(i + 1, min(j, n)):
                    if out[k] != "\n":
                        out[k] = " "
                i = j + 1
        elif c == "'" and re.match(r"'(\\.|[^\\'])'", text[i:i + 4]):
            m = re.match(r"'(\\.|[^\\'])'", text[i:i + 4])
            for k in range(i + 1, i + len(m.group(0)) - 1):
                out[k] = " "
            i += len(m.group(0))
        else:
            i += 1
    return "".join(out)


def match_brace(s, i):
    """index after the brace block starting at s[i] == '{'"""
    depth = 0
    for j in range(i, len(s)):
        if s[j] == "{":
            depth += 1
        elif s[j] == "}":
            depth -= 1
            if depth == 0:
                return j + 1
    return len(s)


def split_top(s, sep=","):
    parts, depth, cur = [], 0, ""
    for ch in s:
        if ch in "<([":
            depth += 1
        elif ch in ">)]":
            depth -= 1
        if ch == sep and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        parts.append(cur)
    return [p.strip() for p in parts]


class RustFile:
    def __init__(self, path, modpath):
        self.path = path
        self.text = path.read_bytes().decode()
        self.code = strip_code(self.text)
        self.modpath = modpath  # crate::extras::foo
        self.uses = self.parse_uses()
        self.struct_pub = {}
        self.structs = self.parse_structs()
        self.fns = self.parse_fns()

    def parse_uses(self):
        uses = {}

        def add(prefix, body):
            for item in split_top(body):
                item = item.strip()
                if not item:
                    continue
                if "{" in item:
                    head, rest = item.split("{", 1)
                    add(prefix + head.rstrip(":").rstrip(":") + "::", rest.rsplit("}", 1)[0])
                    continue
                m = re.match(r"([\w:]+)(?:\s+as\s+(\w+))?$", item)
                if not m:
                    continue
                path = prefix + m.group(1)
                name = m.group(2) or path.split("::")[-1]
                uses[name] = path

        for m in re.finditer(r"(?m)^use\s+(.*?);", self.code, re.S):
            body = m.group(1).replace("\n", " ")
            body = re.sub(r"^super::", self.modpath.rsplit("::", 1)[0] + "::", body)
            if "{" in body:
                head, rest = body.split("{", 1)
                add(head, rest.rsplit("}", 1)[0])
            else:
                add("", body)
        return uses

    def parse_structs(self):
        structs = {}
        for m in re.finditer(r"(?m)^(pub(?:\([\w]+\))? )?struct (\w+)(<[^>]*>)?\s*\{", self.code):
            if m.group(3):
                continue
            # (offset of `struct` when private: made pub when bound)
            self.struct_pub[m.group(2)] = None if m.group(1) else m.start()
            start = m.end() - 1
            end = match_brace(self.code, start)
            body = self.code[start + 1:end - 1]
            fields = {}
            # each top-level item of the body with its offset (a field name
            # can occur inside another's: no text search)
            depth, item_start = 0, 0
            items = []
            for k, ch in enumerate(body + ","):
                if ch in "<([{":
                    depth += 1
                elif ch in ">)]}":
                    depth -= 1
                elif ch == "," and depth == 0:
                    items.append((item_start, body[item_start:k]))
                    item_start = k + 1
            for pos, f in items:
                lead = len(f) - len(f.lstrip())
                fm = re.match(r"(pub(?:\([\w]+\))?\s+)?(\w+)\s*:\s*(.+)$", f.strip(), re.S)
                if fm:
                    fields[fm.group(2)] = (" ".join(fm.group(3).split()), bool(fm.group(1)), start + 1 + pos + lead)
            structs[m.group(2)] = fields
        # unit structs (a printer object with no state of its own)
        for m in re.finditer(r"(?m)^(pub(?:\([\w]+\))? )?struct (\w+);", self.code):
            self.struct_pub[m.group(2)] = None if m.group(1) else m.start()
            structs[m.group(2)] = {}
        return structs

    def parse_fns(self):
        """{(struct or None, name): [(params, ret, is_pub, offset)]}"""
        fns = {}
        code = self.code
        # inherent impls
        for m in re.finditer(r"(?m)^impl\s+(\w+)\s*\{", code):
            start = m.end() - 1
            end = match_brace(code, start)
            self._fns_in(code, start + 1, end - 1, m.group(1), fns, depth_check=True)
        # module functions
        self._fns_in(code, 0, len(code), None, fns, depth_check=False)
        return fns

    def _fns_in(self, code, a, b, struct, fns, depth_check):
        pat = re.compile(r"(?m)^(    )?(pub(?:\([\w]+\))? )?fn (\w+)\s*(<[^>]*>)?\s*\(")
        for m in pat.finditer(code, a, b):
            indented = bool(m.group(1))
            if struct is None and indented:
                continue
            if struct is not None and not indented:
                continue
            if struct is None:
                # top level only: not inside any brace block
                if code[:m.start()].count("{") != code[:m.start()].count("}"):
                    continue
            if m.group(4):
                continue
            pstart = m.end()
            depth, j = 1, pstart
            while depth and j < len(code):
                if code[j] == "(":
                    depth += 1
                elif code[j] == ")":
                    depth -= 1
                j += 1
            params = " ".join(code[pstart:j - 1].split())
            rest = code[j:code.find("{", j)]
            if "where" in rest:
                continue
            ret = rest.strip()
            ret = " ".join(ret[2:].split()) if ret.startswith("->") else "()"
            fns.setdefault((struct, m.group(3)), []).append((params, ret, bool(m.group(2)), m.start()))

# -------------------------------------------------------------- conversions

INTS = {"i64", "i32", "u32", "u64", "usize", "u8", "i8", "u16", "i16"}
SIMPLE = {"f64", "bool", "String"} | INTS

# kaliru's handle types and their converters (impls/*): value -> KResult<PyRef>
SPECIAL_TO = {
    "TrapQRef": "super::chelper::trapq_to_py(Some({x}))",
    "SkRef": "super::chelper::sk_to_py(Some({x}))",
    "EndstopRef": "super::pins::endstop_to_py(&{x})",
    "TmcMcuRef": "super::tmc::mcu_tmc_to_py(&{x})",
    "CommandQueue": "super::mcu::cq_to_py({x})",
    "RailRef": "super::toolhead::rail_to_py({x})",
    "Params": "super::mcu::params_to_py(&{x})",
    "CommandWrapper": "objects::wrap(Rc::new({x}))",
    "CommandQueryWrapper": "objects::wrap(Rc::new({x}))",
    "TempCallback": "super::traits::temp_callback_to_py({x})",
    "Rc<dyn TempSensor>": "super::heaters::sensor_to_py({x})",
    "MoveTransformRef": "super::extras_state::transform_to_py({x})",
    "Rc<PyVal>": "Ok(from_pyval(&{x}))",
    "GCodeCommand": "super::gcode::gcmd_to_py(&{x})",
    "Rc<dyn PwmPin>": "super::pins::pwm_pin_to_py({x})",
    "Rc<dyn DigitalOutPin>": "super::pins::digital_out_to_py({x})",
    "Rc<dyn AdcPin>": "super::pins::adc_pin_to_py({x})",
    "Rc<dyn BulkAdcSensor>": "super::traits::bulk_adc_to_py({x})",
    "Rc<dyn LcdChip>": "super::traits::lcd_chip_to_py({x})",
    "Rc<ReactorCompletion<bool>>": "objects::wrap({x})",
    "Rc<ReactorCompletion<()>>": "objects::wrap({x})",
}
# python -> kaliru handle: (expression of the python object {a}, KResult)
SPECIAL_FROM = {
    "TrapQRef": "super::chelper::trapq_from_py({a})?.ok_or_else(|| KError::Py(\"TypeError\".into(), \"kaliru: NULL trapq\".into()))",
    "SkRef": "super::chelper::sk_from_py({a})?.ok_or_else(|| KError::Py(\"TypeError\".into(), \"kaliru: NULL stepper_kinematics\".into()))",
    "EndstopRef": "super::pins::endstop_from_py({a})",
    "TmcMcuRef": "super::tmc::mcu_tmc_from_py({a})",
    "CommandQueue": "super::mcu::cq_from_py({a})?.ok_or_else(|| KError::Py(\"TypeError\".into(), \"kaliru: expected a command queue\".into()))",
    "Params": "super::mcu::params_from_py({a})",
}
# type aliases that are an Rc of a class python knows
RC_ALIASES = {"Timer"}


def simple_type(t):
    """types FromPy / ToPy convert without naming a crate path"""
    t = t.strip()
    if t in SIMPLE:
        return True
    m = re.match(r"^(Option|Vec)<(.+)>$", t)
    if m:
        return simple_type(m.group(2))
    if t.startswith("(") and t.endswith(")"):
        parts = split_top(t[1:-1])
        return 2 <= len(parts) <= 3 and all(simple_type(p) for p in parts)
    return False


class Gen:
    def __init__(self, rf):
        self.rf = rf
        self.need_pub_fns = []
        self.need_pub_fields = []

    def resolve(self, ident):
        if ident in self.rf.structs:
            return f"{self.rf.modpath}::{ident}"
        if ident in self.rf.uses:
            p = self.rf.uses[ident]
            return p if p.startswith("crate::") else None
        return None

    # ------------------------------------------------------------- to python

    def direct(self, t):
        """ToPy converts t as it is"""
        t = t.strip()
        if t in SIMPLE or t in ("()", "&str", "&'static str", "PyVal", "PyDict", "PObj") or t in RC_ALIASES:
            return True
        m = re.match(r"^(Option|Vec|KResult)<(.+)>$", t)
        if m:
            return self.direct(m.group(2))
        m = re.match(r"^Rc<(\w+)>$", t)
        if m:
            # Rc<PyVal> is a value (SPECIAL_TO), not an object to wrap
            return m.group(1) != "PyVal" and self.resolve(m.group(1)) is not None
        if t.startswith("(") and t.endswith(")"):
            parts = split_top(t[1:-1])
            return 2 <= len(parts) <= 5 and all(self.direct(p) for p in parts)
        return False

    def to_py(self, x, t):
        """an expression of KResult<PyRef> for value x of type t, or None"""
        t = t.strip()
        if self.direct(t):
            return f"{x}.to_py()"
        if t in SPECIAL_TO:
            return SPECIAL_TO[t].format(x=x)
        m = re.match(r"^KResult<(.+)>$", t)
        if m:
            inner = self.to_py("v", m.group(1))
            return None if inner is None else f"{x}.and_then(|v| {inner})"
        m = re.match(r"^Option<(.+)>$", t)
        if m:
            inner = self.to_py("v", m.group(1))
            return None if inner is None else f"(match {x} {{ Some(v) => {inner}, None => Ok(py_none()) }})"
        m = re.match(r"^(?:Vec<(.+)>|\[(.+);\s*\d+\])$", t)
        if m:
            item = m.group(1) or m.group(2)
            if self.direct(item):
                return f"{x}.to_vec().to_py()" if m.group(2) else f"{x}.to_py()"
            inner = self.to_py("v", item)
            if inner is None:
                return None
            return f"{x}.into_iter().map(|v| {inner}).collect::<KResult<Vec<_>>>().map(py_list)"
        m = re.match(r"^IndexMap<String,\s*(.+)>$", t)
        if m:
            inner = self.to_py("v", m.group(1))
            if inner is None:
                return None
            return ("(|| -> KResult<PyRef> { let d = lookup(\"builtins\", \"dict\")?.call(&[])?; "
                    f"for (k, v) in {x} {{ d.call_method(\"__setitem__\", &[&py_str(&k), &{inner}?])?; }} Ok(d) }})()")
        return None

    # ----------------------------------------------------------- from python

    def fromable(self, t):
        """FromPy converts to t (type inferred at the use)"""
        t = t.strip()
        if t in SIMPLE or t in ("PyVal", "PObj"):
            return True
        m = re.match(r"^(Option|Vec)<(.+)>$", t)
        if m:
            return self.fromable(m.group(2))
        m = re.match(r"^Rc<(\w+)>$", t)
        if m:
            # Rc<PyVal> is a value (SPECIAL_TO), not an object to wrap
            return m.group(1) != "PyVal" and self.resolve(m.group(1)) is not None
        if t.startswith("(") and t.endswith(")"):
            parts = split_top(t[1:-1])
            return 2 <= len(parts) <= 3 and all(self.fromable(p) for p in parts)
        return False

    def param(self, i, ptype):
        """(binding statement, argument expression) or None"""
        t = ptype.strip()
        arg = f"a.arg({i})"
        if t in ("&GCodeCommand", "&crate::gcode::GCodeCommand"):
            return f"let p{i} = super::gcode::gcmd_from_py({arg})?;", f"&p{i}"
        if t == "Option<&GCodeCommand>":
            return (f"let p{i} = if {arg}.is_none() {{ None }} else {{ Some(super::gcode::gcmd_from_py({arg})?) }};",
                    f"p{i}.as_deref()")
        if t in ("&ConfigWrapper", "&crate::configfile::ConfigWrapper"):
            return f"let p{i} = super::configfile::config_from_py({arg})?;", f"&p{i}"
        if t == "&str":
            return f"let p{i} = a.string({i})?;", f"&p{i}"
        if t == "Option<&str>":
            return f"let p{i}: Option<String> = FromPy::from_py({arg})?;", f"p{i}.as_deref()"
        if t in ("&PyVal", "&PObj"):
            conv = f"to_pyval({arg})?" if t == "&PyVal" else f"crate::plugins::py_to_pobj({arg})"
            return f"let p{i} = {conv};", f"&p{i}"
        if t == "&[RailRef]":
            return f"let p{i} = super::homing::rails_from_py({arg})?;", f"&p{i}"
        if t == "&IndexMap<String, f64>":
            return f"let p{i} = super::homing::f64_map_from_py({arg})?;", f"&p{i}"
        if t == "&[Color]":
            return f"let p{i}: Vec<Vec<f64>> = FromPy::from_py({arg})?;", f"&p{i}"
        if t in ("TempCallback", "ClientCallback", "crate::extras::bulk_sensor::ClientCallback"):
            conv = "temp_callback_from_py" if t == "TempCallback" else "client_callback_from_py"
            return f"let p{i} = super::traits::{conv}({arg});", f"p{i}"
        if t == "Rc<dyn AdcPin>":
            return f"let p{i} = super::pins::adc_pin_from_py({arg})?;", f"p{i}"
        m = re.match(r"^(&)?(\w+)$", t)
        if m and m.group(2) in SPECIAL_FROM:
            return f"let p{i} = {SPECIAL_FROM[m.group(2)].format(a=arg)}?;", (f"&p{i}" if m.group(1) else f"p{i}")
        m = re.match(r"^&\[(.+)\]$", t)
        if m and simple_type(m.group(1)):
            return f"let p{i}: Vec<{m.group(1)}> = FromPy::from_py({arg})?;", f"&p{i}"
        m = re.match(r"^Option<&\[(.+)\]>$", t)
        if m and simple_type(m.group(1)):
            return f"let p{i}: Option<Vec<{m.group(1)}>> = FromPy::from_py({arg})?;", f"p{i}.as_deref()"
        if t in INTS:
            return f"let p{i} = {arg}.as_i64()? as {t};", f"p{i}"
        if self.fromable(t):
            if simple_type(t):
                return f"let p{i}: {t} = FromPy::from_py({arg})?;", f"p{i}"
            return f"let p{i} = FromPy::from_py({arg})?;", f"p{i}"
        m = re.match(r"^&Rc<(\w+)>$", t)
        if m and self.resolve(m.group(1)):
            return f"let p{i} = objects::unwrap_as::<{self.resolve(m.group(1))}>({arg})?;", f"&p{i}"
        m = re.match(r"^&(\w+)$", t)
        if m and self.resolve(m.group(1)):
            return f"let p{i} = objects::unwrap_as::<{self.resolve(m.group(1))}>({arg})?;", f"&p{i}"
        m = re.match(r"^Option<&(\w+)>$", t)
        if m and self.resolve(m.group(1)):
            path = self.resolve(m.group(1))
            return (f"let p{i} = if {arg}.is_none() {{ None }} else {{ Some(objects::unwrap_as::<{path}>({arg})?) }};",
                    f"p{i}.as_deref()")
        return None

    # --------------------------------------------------------------- handlers

    def call_handler(self, kind, qual, lines, call, ret, is_pub, off):
        expr = self.to_py(call, ret)
        if expr is None:
            return None, f"return type {ret}"
        if not is_pub:
            self.need_pub_fns.append(off)
        return f'    r.method("{qual}", |a| {{\n        ' + "\n        ".join(lines + [expr]) + "\n    });\n", None

    def convert_params(self, ps):
        lines, call_args = [], []
        for i, p in enumerate(ps):
            ptype = p.split(":", 1)[1]
            conv = self.param(i, ptype)
            if conv is None:
                return None, f"param type {ptype.strip()}"
            lines.append(conv[0])
            call_args.append(conv[1])
        return lines, call_args

    def method(self, cls, pyname, args, qual):
        rname = rust_name(pyname)
        cands = self.rf.fns.get((cls, rname), [])
        if len(cands) != 1:
            return None, "no Rust method" if not cands else "several Rust methods"
        params, ret, is_pub, off = cands[0]
        ps = split_top(params)
        if not ps or not (ps[0] in ("&self", "self: &Rc<Self>")):
            return None, f"receiver {ps[0] if ps else '(none)'}"
        ps = ps[1:]
        if len(ps) != len(args):
            return None, f"arity rust {len(ps)} python {len(args)}"
        lines, call_args = self.convert_params(ps)
        if lines is None:
            return None, call_args
        lines = [f"let s = a.this::<{self.rf.modpath}::{cls}>()?;"] + lines
        return self.call_handler("method", qual, lines, f"s.{rname}({', '.join(call_args)})", ret, is_pub, off)

    def func(self, pyname, args, qual):
        cands = self.rf.fns.get((None, pyname), [])
        if len(cands) != 1:
            return None, "no Rust function"
        params, ret, is_pub, off = cands[0]
        ps = split_top(params)
        if len(ps) != len(args):
            return None, f"arity rust {len(ps)} python {len(args)}"
        lines, call_args = self.convert_params(ps)
        if lines is None:
            return None, call_args
        return self.call_handler("func", qual, lines, f"{self.rf.modpath}::{pyname}({', '.join(call_args)})", ret, is_pub, off)

    def ctor(self, cls, args, qual):
        cands = self.rf.fns.get((cls, "new"), [])
        if len(cands) != 1:
            return None, "no Rust new()"
        params, ret, is_pub, off = cands[0]
        ps = split_top(params)
        if len(ps) != len(args):
            return None, f"arity rust {len(ps)} python {len(args)}"
        lines, call_args = self.convert_params(ps)
        if lines is None:
            return None, call_args
        call = f"{self.rf.modpath}::{cls}::new({', '.join(call_args)})"
        r = ret.replace(" ", "")
        if r in ("KResult<Rc<Self>>", f"KResult<Rc<{cls}>>"):
            val = f"{call}?"
        elif r in ("Rc<Self>", f"Rc<{cls}>"):
            val = call
        elif r in ("KResult<Self>", f"KResult<{cls}>"):
            val = f"Rc::new({call}?)"
        elif r in ("Self", cls):
            val = f"Rc::new({call})"
        else:
            return None, f"new() returns {ret}"
        if not is_pub:
            self.need_pub_fns.append(off)
        body = lines + [f"let o: Rc<dyn Any> = {val};", "Ok(o)"]
        return f'    r.ctor("{qual}", |a| {{\n        ' + "\n        ".join(body) + "\n    });\n", None

    def attr(self, cls, pyname, qual):
        fields = self.rf.structs.get(cls, {})
        fname = pyname.lstrip("_") if pyname not in fields else pyname
        if fname not in fields:
            return None, "no Rust field"
        ftype, is_pub, off = fields[fname]
        path = f"{self.rf.modpath}::{cls}"
        obj = f"objects::unwrap_as::<{path}>(o)?.{fname}"
        get = set_ = None
        if m := re.match(r"^Cell<(.+)>$", ftype):
            get = self.to_py(f"{obj}.get()", m.group(1))
            if self.fromable(m.group(1)):
                set_ = f"{obj}.set(FromPy::from_py(v)?);\n        Ok(())"
        elif (m := re.match(r"^RefCell<Vec<(.+)>>$", ftype)) and self.direct(m.group(1)) and self.fromable(m.group(1)):
            if not is_pub:
                self.need_pub_fields.append(off)
            return f'    crate::vec_attr!(r, "{qual}", {path}, {fname});\n', None
        elif m := re.match(r"^RefCell<(.+)>$", ftype):
            get = self.to_py(f"{obj}.borrow().clone()", m.group(1))
            if self.fromable(m.group(1)):
                set_ = f"*{obj}.borrow_mut() = FromPy::from_py(v)?;\n        Ok(())"
        elif m := re.match(r"^Weak<(\w+)>$", ftype):
            get = self.to_py(f"{obj}.upgrade()", f"Option<Rc<{m.group(1)}>>")
        elif ftype in ("&str", "&'static str"):
            get = f"{obj}.to_py()"
        else:
            get = self.to_py(f"{obj}.clone()", ftype)
        if get is None:
            return None, f"field type {ftype}"
        if not is_pub:
            self.need_pub_fields.append(off)
        s = "None" if set_ is None else f"Some(|o, v| {{\n        {set_}\n    }})"
        return f'    r.attr("{qual}", |o| {get}, {s});\n', None


# Kalico classes whose Rust struct has another name or lives elsewhere
# (module.Class -> (module of the Rust file, struct)); same-module renames
# are found by name (case and "_" ignored)
ALIASES = {
    "klippy.reactor.SelectReactor": ("klippy.reactor", "Reactor"),
    "klippy.reactor.PollReactor": ("klippy.reactor", "Reactor"),
    "klippy.reactor.EPollReactor": ("klippy.reactor", "Reactor"),
    "klippy.configfile.PrinterConfig": ("klippy.printer", "ConfigFileObject"),
    "klippy.extras.display.menu.MenuElement": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuContainer": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuDisabled": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuCommand": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuInput": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuList": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuVSDList": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuDialog": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.extras.display.menu.MenuFileBrowser": ("klippy.extras.display.menu", "MenuItem"),
    "klippy.clocksync.SecondarySync": ("klippy.clocksync", "Secondary"),
    "klippy.extras.z_tilt_ng.ZAdjustHelper": ("klippy.extras.z_tilt", "ZAdjustHelper"),
    "klippy.extras.z_tilt_ng.ZAdjustStatus": ("klippy.extras.z_tilt", "ZAdjustStatus"),
    "klippy.extras.z_tilt_ng.RetryHelper": ("klippy.extras.z_tilt", "RetryHelper"),
    "klippy.extras.display.display.DisplayTemplate": ("klippy.extras.display_templates", "DisplayTemplate"),
    "klippy.extras.display.display.DisplayGroup": ("klippy.extras.display_templates", "DisplayGroup"),
    "klippy.extras.display.display.PrinterDisplayTemplate": ("klippy.extras.display_templates", "PrinterDisplayTemplate"),
    "klippy.mcu.MCURestartHelper": ("klippy.mcu", "RestartHelper"),
    "klippy.mcu.MCUConnectHelper": ("klippy.mcu", "ConnHelper"),
    "klippy.mcu.MCUStatsHelper": ("klippy.mcu", "StatsHelper"),
    "klippy.mcu.MCUConfigHelper": ("klippy.mcu", "ConfigHelper"),
    "klippy.extras.tmc2208.TMC2208": ("klippy.extras.tmc", "TmcDriver"),
    "klippy.extras.tmc5262.TMC5262": ("klippy.extras.tmc", "TmcDriver"),
    "klippy.extras.telemetry.KalicoTelementry": ("klippy.extras.telemetry", "KalicoTelemetry"),
    "klippy.extras.buttons.BaseRotaryEncoder": ("klippy.extras.buttons", "RotaryEncoder"),
    "klippy.extras.mixing_extruder.GradientMixingExtruder": ("klippy.extras.mixing_extruder", "MixingExtruder"),
    "klippy.extras.mpc_block_temperature.MPC_BLOCK_TEMP_WRAPPER": ("klippy.extras.mpc_block_temperature", "MpcTempWrapper"),
    "klippy.extras.mpc_ambient_temperature.MPC_AMBIENT_TEMP_WRAPPER": ("klippy.extras.mpc_block_temperature", "MpcTempWrapper"),
    "klippy.extras.aht10.AHT1x": ("klippy.extras.aht10", "AHTBase"),
    "klippy.extras.aht10.AHT2x": ("klippy.extras.aht10", "AHTBase"),
    "klippy.extras.aht10.AHT3x": ("klippy.extras.aht10", "AHTBase"),
    "klippy.extras.spi_temperature.MAX6675": ("klippy.extras.spi_temperature", "SensorBase"),
    "klippy.extras.spi_temperature.MAX31855": ("klippy.extras.spi_temperature", "SensorBase"),
    "klippy.extras.spi_temperature.MAX31856": ("klippy.extras.spi_temperature", "SensorBase"),
    "klippy.extras.spi_temperature.MAX31865": ("klippy.extras.spi_temperature", "SensorBase"),
    "klippy.extras.display.st7920.ST7920": ("klippy.extras.display.st7920", "DisplayBase"),
    "klippy.extras.display.st7920.EmulatedST7920": ("klippy.extras.display.st7920", "DisplayBase"),
    "klippy.extras.display.uc1701.UC1701": ("klippy.extras.display.uc1701", "DisplayBase"),
    "klippy.extras.display.uc1701.SSD1306": ("klippy.extras.display.uc1701", "DisplayBase"),
    "klippy.extras.display.uc1701.SH1106": ("klippy.extras.display.uc1701", "DisplayBase"),
    "klippy.extras.load_cell.hx71x.HX711": ("klippy.extras.load_cell.hx71x", "HX71xBase"),
    "klippy.extras.load_cell.hx71x.HX717": ("klippy.extras.load_cell.hx71x", "HX71xBase"),
    "klippy.extras.load_cell.ads131m0x.ADS131M02": ("klippy.extras.load_cell.ads131m0x", "ADS131MxBase"),
    "klippy.extras.load_cell.ads131m0x.ADS131M04": ("klippy.extras.load_cell.ads131m0x", "ADS131MxBase"),
}


def norm(s):
    return s.lower().replace("_", "")


def hand_registered():
    """struct names the hand-written impls register a class for"""
    names = set()
    for p in (ROOT / "src/plugins/impls").glob("*.rs"):
        if p.name == "generated.rs":
            continue
        names |= set(re.findall(r"register_class(?:_fn)?::<(?:[\w:]+::)?(\w+)>", p.read_bytes().decode()))
    return names


def rust_name(py):
    n = py.lstrip("_")
    if n.startswith("cmd_"):
        n = "cmd_" + n[4:].lower()
    return n


def apply_pub(rf, fn_offs, field_offs):
    """make the used private items pub (from the end: offsets stay valid)"""
    text = rf.text
    edits = []
    for off in fn_offs:
        m = re.match(r"(\s*)(fn|struct) ", text[off:])
        if m:
            edits.append((off + len(m.group(1)), "pub "))
    for off in field_offs:
        edits.append((off, "pub "))
    for off, s in sorted(set(edits), reverse=True):
        text = text[:off] + s + text[off:]
    if edits:
        rf.path.write_bytes(text.encode())


def main():
    report = "--report" in sys.argv
    mods = [m for m in sys.argv[1:] if not m.startswith("--")] or mirror_modules()
    unbound = handled_names()
    out = ["// generated by tools/gen_bindings.py: do not edit (hand-written handlers live",
           "// in the other impls files and take precedence: their names are skipped)",
           "#![allow(unused_imports, unused_variables, unused_parens, noop_method_call, clippy::all)]",
           "use super::*;",
           "use crate::plugins::convert::{FromPy, ToPy};",
           "",
           "pub fn register(r: &mut Registry) {"]
    if OUT.exists() and "--fresh" not in sys.argv:
        # keep previously generated handlers of modules not regenerated now
        prev = OUT.read_bytes().decode()
        for block in re.findall(r"    // ---- (\S+)\n(.*?)(?=    // ---- |\}\n\Z)", prev, re.S):
            if block[0] not in mods:
                out.append(f"    // ---- {block[0]}\n{block[1]}")
    skipped = []
    total = 0
    hand_regs = hand_registered()
    gens = {}

    def gen_for(mod):
        rpath = rust_file(mod)
        if rpath not in gens:
            modpath = "crate::" + "::".join(rpath.relative_to(ROOT / "src").with_suffix("").parts).replace("::mod", "")
            rf = RustFile(rpath, modpath)
            gens[rpath] = (rf, Gen(rf))
        return gens[rpath]

    def find_struct(mod, cls, rf):
        if f"{mod}.{cls}" in ALIASES:
            amod, acls = ALIASES[f"{mod}.{cls}"]
            arf, ag = gen_for(amod)
            return (arf, ag, acls) if acls in arf.structs else None
        if cls in rf.structs:
            return rf, gens[rf.path][1], cls
        same = [s for s in rf.structs if norm(s) == norm(cls)]
        return (rf, gens[rf.path][1], same[0]) if len(same) == 1 else None

    for mod in mods:
        rpath = rust_file(mod)
        names = parse_mirror(mod)
        todo = [n for n in names if (f"{mod}.{n[1]}.{n[2]}" if n[1] else f"{mod}.{n[2]}") in unbound]
        if not todo:
            continue
        if rpath is None:
            skipped += [(f"{mod}.{n[1]}.{n[2]}" if n[1] else f"{mod}.{n[2]}", "no Rust module") for n in todo]
            continue
        rf, g = gen_for(mod)
        block = []
        registered = set()
        for kind, cls, name, args in todo:
            qual = f"{mod}.{cls}.{name}" if cls else f"{mod}.{name}"
            crf, cg, rcls = rf, g, cls
            if cls:
                found = find_struct(mod, cls, rf)
                if found is None:
                    skipped.append((qual, f"no Rust struct {cls}"))
                    continue
                crf, cg, rcls = found
            if kind == "method":
                code, why = cg.method(rcls, name, args, qual)
            elif kind == "ctor":
                code, why = cg.ctor(rcls, args, qual)
            elif kind == "func":
                code, why = cg.func(name, args, qual)
            else:
                code, why = cg.attr(rcls, name, qual)
            if code:
                block.append(code)
                if cls and crf.struct_pub.get(rcls) is not None:
                    cg.need_pub_fns.append(crf.struct_pub[rcls])
                # a renamed struct: python finds its class by registration
                if cls and rcls != cls and f"{mod}.{cls}" not in ALIASES and rcls not in hand_regs \
                        and (rcls, cls) not in registered:
                    registered.add((rcls, cls))
                    block.insert(0, f'    objects::register_class::<{crf.modpath}::{rcls}>("{mod}", "{cls}");\n')
            else:
                skipped.append((qual, why))
        if block:
            out.append(f"    // ---- {mod}\n" + "".join(block))
            total += len(block)
    for rf, g in gens.values():
        apply_pub(rf, g.need_pub_fns, g.need_pub_fields)
    out.append("}")
    OUT.write_bytes(("\n".join(out) + "\n").encode())
    print(f"generated {total} handlers; skipped {len(skipped)}")
    if report:
        for q, why in skipped:
            print(f"  {q}: {why}")


if __name__ == "__main__":
    main()
