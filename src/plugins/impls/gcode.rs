// klippy/gcode.py: GCodeDispatch, GCodeCommand
use super::*;
use crate::gcode::{GCodeCommand, GCodeDispatch, Handler};
use crate::plugins::callable::py_function;
use crate::plugins::native::Args;

pub fn register(r: &mut Registry) {
    objects::register_class::<GCodeDispatch>("klippy.gcode", "GCodeDispatch");
    objects::register_class::<GCodeCommand>("klippy.gcode", "GCodeCommand");
    r.method("klippy.gcode.GCodeDispatch.register_command", register_command);
    r.method("klippy.gcode.GCodeDispatch.respond_info", |a| {
        a.this::<GCodeDispatch>()?.respond_info(&a.string(0)?, a.bool(1)?);
        Ok(py_none())
    });
    r.method("klippy.gcode.GCodeDispatch.respond_raw", |a| {
        a.this::<GCodeDispatch>()?.respond_raw(&a.string(0)?);
        Ok(py_none())
    });
    r.method("klippy.gcode.GCodeCommand.get_command", |a| Ok(py_str(&a.this::<GCodeCommand>()?.get_command())));
    r.method("klippy.gcode.GCodeCommand.get_commandline", |a| Ok(py_str(a.this::<GCodeCommand>()?.get_commandline())));
    r.method("klippy.gcode.GCodeCommand.get", get);
    r.method("klippy.gcode.GCodeCommand.get_int", get_int);
    r.method("klippy.gcode.GCodeCommand.get_float", get_float);
    // self.respond_info = gcode.respond_info (a bound method of the dispatch)
    r.attr("klippy.gcode.GCodeCommand.respond_info", |o| {
        let g = crate::plugins::objects::unwrap_as::<GCodeCommand>(o)?;
        py_function(Rc::new(move |args: &[PyRef]| {
            let log = match args.get(1) { Some(l) => l.is_true()?, None => true };
            g.respond_info(&args[0].as_string()?, log);
            Ok(py_none())
        }))
    }, None);
    r.attr("klippy.gcode.GCodeCommand.respond_raw", |o| {
        let g = crate::plugins::objects::unwrap_as::<GCodeCommand>(o)?;
        py_function(Rc::new(move |args: &[PyRef]| {
            g.respond_raw(&args[0].as_string()?);
            Ok(py_none())
        }))
    }, None);
}

/// The python object of a command handed to a handler: the command itself
/// while the dispatch runs it, else a copy
pub fn gcmd_to_py(gcmd: &GCodeCommand) -> KResult<PyRef> {
    match crate::gcode::running_command(gcmd) {
        Some(rc) => wrap(rc),
        None => wrap(Rc::new(gcmd.clone())),
    }
}

/// A python gcode handler as kaliru's
pub fn handler_of(func: PyRef) -> Handler {
    Rc::new(move |gcmd: &GCodeCommand| {
        let g = gcmd_to_py(gcmd)?;
        func.call(&[&g])?;
        Ok(())
    })
}

// register_command(cmd, func, when_not_ready=False, desc=None)
fn register_command(a: &Args) -> KResult<PyRef> {
    let g = a.this::<GCodeDispatch>()?;
    let func = if a.arg(1).is_none() { None } else { Some(handler_of(a.arg(1).clone())) };
    let old = g.register_command(&a.string(0)?, func, a.bool(2)?, a.opt_string(3)?.as_deref())?;
    match old {
        None => Ok(py_none()),
        Some(h) => py_function(Rc::new(move |args: &[PyRef]| {
            let gcmd = crate::plugins::objects::unwrap_as::<GCodeCommand>(&args[0])?;
            h(&gcmd)?;
            Ok(py_none())
        })),
    }
}

fn sentinel() -> KResult<PyRef> {
    lookup("klippy.gcode", "GCodeCommand")?.getattr("sentinel")
}

/// The raw parameter, or the default (Err when required and missing)
fn param(g: &GCodeCommand, name: &str, default: &PyRef) -> KResult<Result<String, PyRef>> {
    match g.get_opt(name) {
        Some(v) => Ok(Ok(v)),
        None if default.is(sentinel()?.as_ptr()) => Err(g.get(name).unwrap_err()),
        None => Ok(Err(default.clone())),
    }
}

// get(name, default=sentinel, parser=str, minval, maxval, above, below)
fn get(a: &Args) -> KResult<PyRef> {
    let g = a.this::<GCodeCommand>()?;
    let name = a.string(0)?;
    match param(&g, &name, a.arg(1))? {
        Err(d) => Ok(d),
        Ok(v) => {
            let parser = a.arg(2);
            if parser.is(lookup("builtins", "str")?.as_ptr()) && [3, 4, 5, 6].iter().all(|i| a.arg(*i).is_none()) {
                return Ok(py_str(&v));
            }
            Err(KError::Py("NotImplementedError".into(), "kaliru: GCodeCommand.get with a parser is not mirrored yet".into()))
        }
    }
}

// get_int(name, default=sentinel, minval=None, maxval=None)
fn get_int(a: &Args) -> KResult<PyRef> {
    let g = a.this::<GCodeCommand>()?;
    let name = a.string(0)?;
    if let Err(d) = param(&g, &name, a.arg(1))? {
        return Ok(d);
    }
    let none = py_none();
    let v = g.get_int(&name, None, lim([a.arg(2), a.arg(3), &none, &none], true)?)?;
    Ok(py_int(v))
}

// get_float(name, default=sentinel, minval, maxval, above, below)
fn get_float(a: &Args) -> KResult<PyRef> {
    let g = a.this::<GCodeCommand>()?;
    let name = a.string(0)?;
    if let Err(d) = param(&g, &name, a.arg(1))? {
        return Ok(d);
    }
    let v = g.get_float(&name, None, lim([a.arg(2), a.arg(3), a.arg(4), a.arg(5)], true)?)?;
    Ok(py_float(v))
}
