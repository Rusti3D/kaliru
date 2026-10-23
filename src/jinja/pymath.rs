// python's math module (CPython 3.13 Modules/mathmodule.c behaviour) for
// templates; the C library supplies the functions CPython takes from libm
use super::filters::bind;
use super::runtime::{builtin, Module};
use super::value::{CallCtx, Exc, VResult, Value};
use indexmap::IndexMap;

extern "C" {
    fn erf(x: f64) -> f64;
    fn erfc(x: f64) -> f64;
    fn tgamma(x: f64) -> f64;
    fn lgamma(x: f64) -> f64;
    fn nextafter(x: f64, y: f64) -> f64;
    fn remainder(x: f64, y: f64) -> f64;
    fn fma(x: f64, y: f64, z: f64) -> f64;
}

fn domain() -> Exc {
    Exc::value_error("math domain error")
}

fn range() -> Exc {
    Exc::new("OverflowError", "math range error")
}

/// a float argument (PyFloat_AsDouble)
fn real(v: &Value) -> VResult<f64> {
    v.as_f64().ok_or_else(|| Exc::type_error(format!("must be real number, not {}", v.type_name())))
}

fn int_arg(v: &Value) -> VResult<i64> {
    v.as_int().ok_or_else(|| Exc::type_error(format!("'{}' object cannot be interpreted as an integer", v.type_name())))
}

fn one(fname: &str, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    if !kwargs.is_empty() {
        return Err(Exc::type_error(format!("math.{}() takes no keyword arguments", fname)));
    }
    if args.len() != 1 {
        return Err(Exc::type_error(format!("math.{}() takes exactly one argument ({} given)", fname, args.len())));
    }
    Ok(args.into_iter().next().unwrap())
}

fn two(fname: &str, args: Vec<Value>) -> VResult<(Value, Value)> {
    if args.len() != 2 {
        return Err(Exc::type_error(format!("{} expected 2 arguments, got {}", fname, args.len())));
    }
    let mut it = args.into_iter();
    Ok((it.next().unwrap(), it.next().unwrap()))
}

/// math_1(): apply f and map nan / inf results to python errors
fn math_1(x: f64, f: fn(f64) -> f64, can_overflow: bool) -> VResult<f64> {
    let r = f(x);
    if r.is_nan() && !x.is_nan() {
        return Err(domain());
    }
    if r.is_infinite() && x.is_finite() {
        return Err(if can_overflow { range() } else { domain() });
    }
    Ok(r)
}

macro_rules! unary {
    ($name:ident, $py:expr, $f:expr, $ovf:expr) => {
        fn $name(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
            let x = real(&one($py, args, kwargs)?)?;
            Ok(Value::Float(math_1(x, $f, $ovf)?))
        }
    };
}

unary!(m_sqrt, "sqrt", f64::sqrt, false);
unary!(m_exp, "exp", f64::exp, true);
unary!(m_exp2, "exp2", f64::exp2, true);
unary!(m_expm1, "expm1", f64::exp_m1, true);
unary!(m_sin, "sin", f64::sin, false);
unary!(m_cos, "cos", f64::cos, false);
unary!(m_tan, "tan", f64::tan, false);
unary!(m_asin, "asin", f64::asin, false);
unary!(m_acos, "acos", f64::acos, false);
unary!(m_atan, "atan", f64::atan, false);
unary!(m_sinh, "sinh", f64::sinh, true);
unary!(m_cosh, "cosh", f64::cosh, true);
unary!(m_tanh, "tanh", f64::tanh, false);
unary!(m_asinh, "asinh", f64::asinh, false);
unary!(m_acosh, "acosh", f64::acosh, false);
unary!(m_atanh, "atanh", f64::atanh, false);
unary!(m_cbrt, "cbrt", f64::cbrt, false);
unary!(m_fabs, "fabs", f64::abs, false);
unary!(m_erf, "erf", |x| unsafe { erf(x) }, false);
unary!(m_erfc, "erfc", |x| unsafe { erfc(x) }, false);
unary!(m_log10, "log10", |x| if x <= 0.0 { f64::NAN } else { x.log10() }, false);
unary!(m_log2, "log2", |x| if x <= 0.0 { f64::NAN } else { x.log2() }, false);
unary!(m_log1p, "log1p", |x| if x <= -1.0 { f64::NAN } else { x.ln_1p() }, false);
unary!(m_degrees, "degrees", |x| x * (180.0 / std::f64::consts::PI), true);
unary!(m_radians, "radians", |x| x * (std::f64::consts::PI / 180.0), false);

fn m_gamma(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let x = real(&one("gamma", args, kwargs)?)?;
    if x.is_finite() && x <= 0.0 && x.fract() == 0.0 {
        return Err(domain());
    }
    Ok(Value::Float(math_1(x, |x| unsafe { tgamma(x) }, true)?))
}

fn m_lgamma(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let x = real(&one("lgamma", args, kwargs)?)?;
    if x.is_finite() && x <= 0.0 && x.fract() == 0.0 {
        return Err(domain());
    }
    Ok(Value::Float(math_1(x, |x| unsafe { lgamma(x) }, true)?))
}

fn to_int(f: f64) -> VResult<Value> {
    if f.is_nan() {
        return Err(Exc::value_error("cannot convert float NaN to integer"));
    }
    if f.is_infinite() {
        return Err(Exc::new("OverflowError", "cannot convert float infinity to integer"));
    }
    Ok(Value::Int(f as i64))
}

fn m_floor(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let v = one("floor", args, kwargs)?;
    if let Some(i) = v.as_int() {
        return Ok(Value::Int(i));
    }
    to_int(real(&v)?.floor())
}

fn m_ceil(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let v = one("ceil", args, kwargs)?;
    if let Some(i) = v.as_int() {
        return Ok(Value::Int(i));
    }
    to_int(real(&v)?.ceil())
}

fn m_trunc(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let v = one("trunc", args, kwargs)?;
    if let Some(i) = v.as_int() {
        return Ok(Value::Int(i));
    }
    match v {
        Value::Float(f) => to_int(f.trunc()),
        other => Err(Exc::type_error(format!("type {} doesn't define __trunc__ method", other.type_name()))),
    }
}

fn m_log(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    if !kwargs.is_empty() {
        return Err(Exc::type_error("log() takes no keyword arguments"));
    }
    if args.is_empty() || args.len() > 2 {
        return Err(Exc::type_error(format!("log expected at least 1 argument, got {}", args.len())));
    }
    let ln = |v: &Value| -> VResult<f64> {
        let x = real(v)?;
        if x.is_nan() {
            return Ok(x);
        }
        if x <= 0.0 {
            return Err(domain());
        }
        Ok(x.ln())
    };
    let num = ln(&args[0])?;
    if args.len() == 2 {
        let den = ln(&args[1])?;
        if den == 0.0 {
            return Err(Exc::zero_division("float division by zero"));
        }
        return Ok(Value::Float(num / den));
    }
    Ok(Value::Float(num))
}

fn m_pow(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("pow", args)?;
    let (x, y) = (real(&a)?, real(&b)?);
    if x.is_finite() && y.is_finite() {
        if x == 0.0 && y < 0.0 {
            return Err(domain());
        }
        if x < 0.0 && y.fract() != 0.0 {
            return Err(domain());
        }
    }
    let r = x.powf(y);
    if r.is_infinite() && x.is_finite() && y.is_finite() {
        return Err(range());
    }
    Ok(Value::Float(r))
}

fn m_atan2(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("atan2", args)?;
    Ok(Value::Float(real(&a)?.atan2(real(&b)?)))
}

fn m_fmod(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("fmod", args)?;
    let (x, y) = (real(&a)?, real(&b)?);
    if y.is_infinite() && x.is_finite() {
        return Ok(Value::Float(x));
    }
    let r = x % y;
    if r.is_nan() && !x.is_nan() && !y.is_nan() {
        return Err(domain());
    }
    Ok(Value::Float(r))
}

fn m_remainder(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("remainder", args)?;
    let (x, y) = (real(&a)?, real(&b)?);
    let r = unsafe { remainder(x, y) };
    if r.is_nan() && !x.is_nan() && !y.is_nan() {
        return Err(domain());
    }
    Ok(Value::Float(r))
}

fn m_copysign(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("copysign", args)?;
    Ok(Value::Float(real(&a)?.copysign(real(&b)?)))
}

fn m_ldexp(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("ldexp", args)?;
    let x = real(&a)?;
    let e = b.as_int().ok_or_else(|| Exc::type_error("Expected an int as second argument to ldexp."))?;
    if x == 0.0 || !x.is_finite() {
        return Ok(Value::Float(x));
    }
    let r = x * 2f64.powi(e.clamp(-2000, 2000) as i32);
    if r.is_infinite() {
        return Err(range());
    }
    Ok(Value::Float(r))
}

fn m_frexp(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let x = real(&one("frexp", args, kwargs)?)?;
    if x == 0.0 || !x.is_finite() {
        return Ok(Value::tuple(vec![Value::Float(x), Value::Int(0)]));
    }
    let mut e = x.abs().log2().floor() as i64 + 1;
    let mut m = x / 2f64.powi(e as i32);
    if m.abs() >= 1.0 {
        m /= 2.0;
        e += 1;
    } else if m.abs() < 0.5 {
        m *= 2.0;
        e -= 1;
    }
    Ok(Value::tuple(vec![Value::Float(m), Value::Int(e)]))
}

fn m_modf(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let x = real(&one("modf", args, kwargs)?)?;
    if x.is_infinite() {
        return Ok(Value::tuple(vec![Value::Float(0.0f64.copysign(x)), Value::Float(x)]));
    }
    Ok(Value::tuple(vec![Value::Float(x.fract()), Value::Float(x.trunc())]))
}

fn m_isnan(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    Ok(Value::Bool(real(&one("isnan", args, kwargs)?)?.is_nan()))
}

fn m_isinf(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    Ok(Value::Bool(real(&one("isinf", args, kwargs)?)?.is_infinite()))
}

fn m_isfinite(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    Ok(Value::Bool(real(&one("isfinite", args, kwargs)?)?.is_finite()))
}

fn m_isclose(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let (a, _, _) = bind(
        "isclose",
        &[("a", None), ("b", None), ("rel_tol", Some(Value::Float(1e-09))), ("abs_tol", Some(Value::Float(0.0)))],
        args,
        kwargs,
        false,
        false,
    )?;
    let (x, y, rel, abs) = (real(&a[0])?, real(&a[1])?, real(&a[2])?, real(&a[3])?);
    if rel < 0.0 || abs < 0.0 {
        return Err(Exc::value_error("tolerances must be non-negative"));
    }
    if x == y {
        return Ok(Value::Bool(true));
    }
    if x.is_infinite() || y.is_infinite() {
        return Ok(Value::Bool(false));
    }
    let diff = (y - x).abs();
    Ok(Value::Bool((diff <= (rel * y).abs()) || (diff <= (rel * x).abs()) || (diff <= abs)))
}

fn m_hypot(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let mut coords = vec![];
    for a in &args {
        coords.push(real(a)?);
    }
    Ok(Value::Float(crate::mathutil::py_hypot(&coords)))
}

fn m_dist(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (p, q) = two("dist", args)?;
    let (p, q) = (p.iter_items()?, q.iter_items()?);
    if p.len() != q.len() {
        return Err(Exc::value_error("both points must have the same number of dimensions"));
    }
    let mut diffs = vec![];
    for (a, b) in p.iter().zip(&q) {
        diffs.push(real(a)? - real(b)?);
    }
    Ok(Value::Float(crate::mathutil::py_hypot(&diffs)))
}

/// math.fsum: exact partial sums (msum)
fn m_fsum(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let items = one("fsum", args, kwargs)?.iter_items()?;
    let mut partials: Vec<f64> = vec![];
    for it in &items {
        let mut x = real(it)?;
        let mut i = 0;
        for j in 0..partials.len() {
            let mut y = partials[j];
            if x.abs() < y.abs() {
                std::mem::swap(&mut x, &mut y);
            }
            let hi = x + y;
            let lo = y - (hi - x);
            if lo != 0.0 {
                partials[i] = lo;
                i += 1;
            }
            x = hi;
        }
        partials.truncate(i);
        partials.push(x);
    }
    Ok(Value::Float(partials.iter().sum()))
}

fn m_prod(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let (a, _, _) = bind("prod", &[("iterable", None), ("start", Some(Value::Int(1)))], args, kwargs, false, false)?;
    let mut acc = a[1].clone();
    for it in a[0].iter_items()? {
        acc = acc.mul(&it)?;
    }
    Ok(acc)
}

fn m_sumprod(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (p, q) = two("sumprod", args)?;
    let (p, q) = (p.iter_items()?, q.iter_items()?);
    if p.len() != q.len() {
        return Err(Exc::value_error("Inputs are not the same length"));
    }
    let mut acc = Value::Int(0);
    for (a, b) in p.iter().zip(&q) {
        acc = acc.add(&a.mul(b)?)?;
    }
    Ok(acc)
}

fn m_factorial(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let n = int_arg(&one("factorial", args, kwargs)?)?;
    if n < 0 {
        return Err(Exc::value_error("factorial() not defined for negative values"));
    }
    let mut acc = Value::Int(1);
    for i in 2..=n {
        acc = acc.mul(&Value::Int(i))?;
    }
    Ok(acc)
}

fn gcd(a: i64, b: i64) -> i64 {
    let (mut a, mut b) = (a.unsigned_abs(), b.unsigned_abs());
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a as i64
}

fn m_gcd(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let mut acc = 0;
    for a in &args {
        acc = gcd(acc, int_arg(a)?);
    }
    Ok(Value::Int(acc))
}

fn m_lcm(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let mut acc: i64 = 1;
    for a in &args {
        let x = int_arg(a)?;
        if x == 0 || acc == 0 {
            acc = 0;
            continue;
        }
        acc = (acc / gcd(acc, x) * x).abs();
    }
    Ok(Value::Int(acc))
}

fn m_isqrt(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let n = int_arg(&one("isqrt", args, kwargs)?)?;
    if n < 0 {
        return Err(Exc::value_error("isqrt() argument must be nonnegative"));
    }
    let mut r = (n as f64).sqrt() as i64;
    while r * r > n {
        r -= 1;
    }
    while (r + 1) * (r + 1) <= n {
        r += 1;
    }
    Ok(Value::Int(r))
}

fn m_comb(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("comb", args)?;
    let (n, k) = (int_arg(&a)?, int_arg(&b)?);
    if n < 0 {
        return Err(Exc::value_error("n must be a non-negative integer"));
    }
    if k < 0 {
        return Err(Exc::value_error("k must be a non-negative integer"));
    }
    if k > n {
        return Ok(Value::Int(0));
    }
    let k = k.min(n - k);
    let mut acc: i128 = 1;
    for i in 0..k {
        acc = acc * (n - i) as i128 / (i + 1) as i128;
    }
    Ok(Value::Int(acc as i64))
}

fn m_perm(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    if args.len() == 1 {
        return m_factorial(&mut NoCtx, args, IndexMap::new());
    }
    let (a, b) = two("perm", args)?;
    let (n, k) = (int_arg(&a)?, int_arg(&b)?);
    if n < 0 {
        return Err(Exc::value_error("n must be a non-negative integer"));
    }
    if k < 0 {
        return Err(Exc::value_error("k must be a non-negative integer"));
    }
    if k > n {
        return Ok(Value::Int(0));
    }
    let mut acc = Value::Int(1);
    for i in 0..k {
        acc = acc.mul(&Value::Int(n - i))?;
    }
    Ok(acc)
}

fn m_nextafter(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    let (a, b) = two("nextafter", args)?;
    Ok(Value::Float(unsafe { nextafter(real(&a)?, real(&b)?) }))
}

fn m_ulp(_: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> VResult<Value> {
    let x = real(&one("ulp", args, kwargs)?)?.abs();
    if x.is_nan() || x.is_infinite() {
        return Ok(Value::Float(x));
    }
    let next = unsafe { nextafter(x, f64::INFINITY) };
    if next.is_infinite() {
        let prev = unsafe { nextafter(x, 0.0) };
        return Ok(Value::Float(x - prev));
    }
    Ok(Value::Float(next - x))
}

fn m_fma(_: &mut dyn CallCtx, args: Vec<Value>, _kw: IndexMap<String, Value>) -> VResult<Value> {
    if args.len() != 3 {
        return Err(Exc::type_error(format!("fma expected 3 arguments, got {}", args.len())));
    }
    let (x, y, z) = (real(&args[0])?, real(&args[1])?, real(&args[2])?);
    Ok(Value::Float(unsafe { fma(x, y, z) }))
}

struct NoCtx;

impl CallCtx for NoCtx {
    fn call_macro(&mut self, _m: &super::eval::MacroObj, _a: Vec<Value>, _k: IndexMap<String, Value>) -> VResult<Value> {
        Err(Exc::runtime("no template context"))
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// the math module object
pub fn math_module() -> Value {
    let mut attrs: IndexMap<&'static str, Value> = IndexMap::new();
    let f = |name: &'static str, p: super::runtime::BuiltinFnPtr| builtin(name, "builtin_function_or_method", p);
    let funcs: &[(&'static str, super::runtime::BuiltinFnPtr)] = &[
        ("acos", m_acos),
        ("acosh", m_acosh),
        ("asin", m_asin),
        ("asinh", m_asinh),
        ("atan", m_atan),
        ("atan2", m_atan2),
        ("atanh", m_atanh),
        ("cbrt", m_cbrt),
        ("ceil", m_ceil),
        ("comb", m_comb),
        ("copysign", m_copysign),
        ("cos", m_cos),
        ("cosh", m_cosh),
        ("degrees", m_degrees),
        ("dist", m_dist),
        ("erf", m_erf),
        ("erfc", m_erfc),
        ("exp", m_exp),
        ("exp2", m_exp2),
        ("expm1", m_expm1),
        ("fabs", m_fabs),
        ("factorial", m_factorial),
        ("floor", m_floor),
        ("fma", m_fma),
        ("fmod", m_fmod),
        ("frexp", m_frexp),
        ("fsum", m_fsum),
        ("gamma", m_gamma),
        ("gcd", m_gcd),
        ("hypot", m_hypot),
        ("isclose", m_isclose),
        ("isfinite", m_isfinite),
        ("isinf", m_isinf),
        ("isnan", m_isnan),
        ("isqrt", m_isqrt),
        ("lcm", m_lcm),
        ("ldexp", m_ldexp),
        ("lgamma", m_lgamma),
        ("log", m_log),
        ("log10", m_log10),
        ("log1p", m_log1p),
        ("log2", m_log2),
        ("modf", m_modf),
        ("nextafter", m_nextafter),
        ("perm", m_perm),
        ("pow", m_pow),
        ("prod", m_prod),
        ("radians", m_radians),
        ("remainder", m_remainder),
        ("sin", m_sin),
        ("sinh", m_sinh),
        ("sqrt", m_sqrt),
        ("sumprod", m_sumprod),
        ("tan", m_tan),
        ("tanh", m_tanh),
        ("trunc", m_trunc),
        ("ulp", m_ulp),
    ];
    for (name, p) in funcs {
        attrs.insert(name, f(name, *p));
    }
    attrs.insert("e", Value::Float(std::f64::consts::E));
    attrs.insert("inf", Value::Float(f64::INFINITY));
    attrs.insert("nan", Value::Float(f64::NAN));
    attrs.insert("pi", Value::Float(std::f64::consts::PI));
    attrs.insert("tau", Value::Float(std::f64::consts::TAU));
    attrs.sort_keys();
    Value::object(Module { name: "math", attrs })
}
