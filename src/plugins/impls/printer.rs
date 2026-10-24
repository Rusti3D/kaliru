// klippy/printer.py: Printer
use super::*;
use crate::plugins::native::Args;
use crate::plugins::{objects, py_to_pobj};

pub fn register(r: &mut Registry) {
    objects::register_class::<Printer>("klippy.printer", "Printer");
    r.method("klippy.printer.Printer.get_reactor", |a| wrap(a.this::<Printer>()?.get_reactor()));
    r.method("klippy.printer.Printer.lookup_object", lookup_object);
    r.method("klippy.printer.Printer.add_object", |a| {
        a.this::<Printer>()?.add_object(&a.string(0)?, py_to_pobj(a.arg(1)))?;
        Ok(py_none())
    });
    r.method("klippy.printer.Printer.register_event_handler", register_event_handler);
}

// lookup_object(name, default=configfile.sentinel)
fn lookup_object(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let name = a.string(0)?;
    match p.lookup_object_opt(&name) {
        Some(o) => pobj_to_py(&o),
        None if a.arg(1).is(lookup("klippy.configfile", "sentinel")?.as_ptr()) => {
            Err(KError::config(format!("Unknown config object '{}'", name)))
        }
        None => Ok(a.arg(1).clone()),
    }
}

// register_event_handler(event, callback)
fn register_event_handler(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let cb = a.arg(1).clone();
    p.register_event_handler(
        &a.string(0)?,
        Rc::new(move |args: &[&dyn Any]| {
            let py_args = args.iter().map(|x| any_to_py(*x)).collect::<KResult<Vec<_>>>()?;
            let refs: Vec<&PyRef> = py_args.iter().collect();
            cb.call(&refs)?;
            Ok(())
        }),
    );
    Ok(py_none())
}
