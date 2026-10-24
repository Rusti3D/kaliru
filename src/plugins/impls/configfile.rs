// klippy/configfile.py: ConfigWrapper
use super::*;
use crate::configfile::ConfigWrapper;
use crate::plugins::native::Args;

pub fn register(r: &mut Registry) {
    objects::register_class::<ConfigWrapper>("klippy.configfile", "ConfigWrapper");
    r.method("klippy.configfile.ConfigWrapper.get_printer", |a| {
        let p = a.this::<ConfigWrapper>()?.get_printer();
        objects::wrap(p)
    });
    r.method("klippy.configfile.ConfigWrapper.get_name", |a| Ok(py_str(a.this::<ConfigWrapper>()?.get_name())));
    r.method("klippy.configfile.ConfigWrapper.get", get);
    r.method("klippy.configfile.ConfigWrapper.getint", getint);
    r.method("klippy.configfile.ConfigWrapper.getfloat", getfloat);
    r.method("klippy.configfile.ConfigWrapper.getboolean", getboolean);
}

fn sentinel() -> KResult<PyRef> {
    lookup("klippy.configfile", "sentinel")
}

// get(option, default=sentinel, note_valid=True)
fn get(a: &Args) -> KResult<PyRef> {
    let c = a.this::<ConfigWrapper>()?;
    let option = a.string(0)?;
    let (dflt, orig) = dflt(a.arg(1), &sentinel()?)?;
    let missing = !c.fileconfig_has_option(&option);
    let v = c.get_value(&option, dflt, a.bool(2)?)?;
    Ok(value_or_default(v, missing, orig))
}

// getint(option, default=sentinel, minval=None, maxval=None, note_valid=True)
fn getint(a: &Args) -> KResult<PyRef> {
    let c = a.this::<ConfigWrapper>()?;
    let option = a.string(0)?;
    let (dflt, orig) = dflt(a.arg(1), &sentinel()?)?;
    let none = py_none();
    let lim = lim([a.arg(2), a.arg(3), &none, &none], a.bool(4)?)?;
    let missing = !c.fileconfig_has_option(&option);
    let v = c.getint_value(&option, &dflt, &lim)?;
    Ok(value_or_default(v, missing, orig))
}

// getfloat(option, default=sentinel, minval, maxval, above, below, note_valid=True)
fn getfloat(a: &Args) -> KResult<PyRef> {
    let c = a.this::<ConfigWrapper>()?;
    let option = a.string(0)?;
    let (dflt, orig) = dflt(a.arg(1), &sentinel()?)?;
    let lim = lim([a.arg(2), a.arg(3), a.arg(4), a.arg(5)], a.bool(6)?)?;
    let missing = !c.fileconfig_has_option(&option);
    let v = c.getfloat_value(&option, &dflt, &lim)?;
    Ok(value_or_default(v, missing, orig))
}

// getboolean(option, default=sentinel, note_valid=True)
fn getboolean(a: &Args) -> KResult<PyRef> {
    let c = a.this::<ConfigWrapper>()?;
    let option = a.string(0)?;
    let (dflt, orig) = dflt(a.arg(1), &sentinel()?)?;
    let missing = !c.fileconfig_has_option(&option);
    let v = c.getboolean_value(&option, &dflt, a.bool(2)?)?;
    Ok(value_or_default(v, missing, orig))
}
