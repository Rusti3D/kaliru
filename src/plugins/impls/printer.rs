// klippy/printer.py: Printer
use super::*;
use crate::plugins::convert::FromPy;
use crate::plugins::native::Args;
use crate::plugins::objects;
use crate::method;

/// printer.components (identity-stable: one view per printer)
pub struct ComponentsRef(pub Rc<Printer>);

thread_local! {
    static COMPONENTS: std::cell::RefCell<std::collections::HashMap<usize, Rc<ComponentsRef>>> = Default::default();
}

/// The collection behind a python SubsystemComponentCollection: the
/// printer's, or one a plugin built
fn with_components<R>(o: &PyRef, f: impl FnOnce(&crate::printer::SubsystemComponentCollection) -> R) -> KResult<R> {
    if let Ok(c) = objects::unwrap_as::<ComponentsRef>(o) {
        return Ok(f(&c.0.components));
    }
    Ok(f(&*objects::unwrap_as::<crate::printer::SubsystemComponentCollection>(o)?))
}

/// A component as python sees it: a plugin's object as registered; a
/// kaliru one (a factory) as the class Kalico registers under that name
fn component_to_py(subsystem: &str, name: &str, c: &Rc<dyn Any>) -> KResult<PyRef> {
    if let Some(p) = c.downcast_ref::<PyRef>() {
        return Ok(p.clone());
    }
    let class = match (subsystem, name) {
        ("eddy_current_sensors", "ldc1612") => lookup("klippy.extras.ldc1612", "LDC1612"),
        ("load_cell_probe_tap_classifiers", "empty") => lookup("klippy.extras.load_cell.tap_analysis", "TapClassifierModule"),
        ("load_cell_probe_tap_classifiers", "tap_quality") => {
            lookup("klippy.extras.load_cell.tap_quality_classifier", "TapQualityClassifier")
        }
        ("load_cell_sensors", _) | ("load_cell_probe_sensors", _) => {
            let tables = [
                ("klippy.extras.load_cell.hx71x", "HX71X_SENSOR_TYPES"),
                ("klippy.extras.load_cell.ads1220", "ADS1220_SENSOR_TYPE"),
                ("klippy.extras.load_cell.ads131m0x", "ADS131M0X_SENSOR_TYPES"),
                ("klippy.extras.load_cell.cs1237", "CS1237_SENSOR_TYPE"),
            ];
            let mut found = None;
            for (m, d) in tables {
                let v = lookup(m, d)?.call_method("get", &[&py_str(name)])?;
                if !v.is_none() {
                    found = Some(v);
                    break;
                }
            }
            found.ok_or_else(|| KError::Py("KeyError".into(), py_str(name).repr().unwrap_or_default()))
        }
        _ => Err(KError::Py("NotImplementedError".into(), format!("kaliru: component {}:{} as a python object", subsystem, name))),
    };
    class
}

fn components_dict(subsystem: &str, comps: &crate::IndexMap<String, Rc<dyn Any>>) -> KResult<PyRef> {
    let d = lookup("builtins", "dict")?.call(&[])?;
    for (name, c) in comps {
        d.call_method("__setitem__", &[&py_str(name), &component_to_py(subsystem, name, c)?])?;
    }
    Ok(d)
}

pub fn register(r: &mut Registry) {
    objects::register_class::<ComponentsRef>("klippy.printer", "SubsystemComponentCollection");
    r.method("klippy.printer.Printer.lookup_components", |a| {
        let sub = a.string(0)?;
        components_dict(&sub, &a.this::<Printer>()?.lookup_components(&sub))
    });
    r.attr("klippy.printer.Printer.components", |o| {
        let p = objects::unwrap_as::<Printer>(o)?;
        let key = Rc::as_ptr(&p) as usize;
        let c = COMPONENTS.with(|m| m.borrow_mut().entry(key).or_insert_with(|| Rc::new(ComponentsRef(p.clone()))).clone());
        objects::wrap(c)
    }, None);
    // SubsystemComponentCollection(config_error): a plugin's own collection
    r.ctor("klippy.printer.SubsystemComponentCollection.__init__", |_a| {
        let c: Rc<dyn Any> = Rc::new(crate::printer::SubsystemComponentCollection::default());
        Ok(c)
    });
    r.method("klippy.printer.SubsystemComponentCollection.get_components", |a| {
        let sub = a.string(0)?;
        let comps = with_components(a.slf.as_ref().expect("method"), |c| c.get_components(&sub))?;
        components_dict(&sub, &comps)
    });
    // register_component(subsystem_name, component_name, component): the python object itself
    r.method("klippy.printer.SubsystemComponentCollection.register_component", |a| {
        let (sub, name) = (a.string(0)?, a.string(1)?);
        let comp: Rc<dyn Any> = Rc::new(a.arg(2).clone());
        with_components(a.slf.as_ref().expect("method"), |c| c.register_component(&sub, &name, comp))??;
        Ok(py_none())
    });
    r.attr("klippy.printer.SubsystemComponentCollection._config_error", |_o| lookup("klippy.configfile", "error"), None);
    r.attr("klippy.printer.SubsystemComponentCollection._subsystems", |o| {
        let subs = with_components(o, |c| c.subsystems.borrow().clone())?;
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (sub, comps) in &subs {
            d.call_method("__setitem__", &[&py_str(sub), &components_dict(sub, comps)?])?;
        }
        Ok(d)
    }, None);
    // event_handlers: {event: [callback, ...]} (python callables of kaliru's handlers)
    r.attr("klippy.printer.Printer.event_handlers", |o| {
        let handlers = objects::unwrap_as::<Printer>(o)?.event_handlers.borrow().clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (event, cbs) in handlers {
            let items = cbs
                .into_iter()
                .map(|cb| {
                    crate::plugins::callable::py_function(Rc::new(move |args: &[PyRef]| {
                        let refs: Vec<&dyn Any> = args.iter().map(|x| x as &dyn Any).collect();
                        cb(&refs)?;
                        Ok(py_none())
                    }))
                })
                .collect::<KResult<Vec<_>>>()?;
            d.call_method("__setitem__", &[&py_str(&event), &py_list(items)])?;
        }
        Ok(d)
    }, None);
    method!(r, "klippy.printer.Printer", Printer, get_reactor(p) => p.get_reactor());
    method!(r, "klippy.printer.Printer", Printer, get_start_args(p) => start_args(&p));
    method!(r, "klippy.printer.Printer", Printer, get_state_message(p) => p.get_state_message());
    method!(r, "klippy.printer.Printer", Printer, is_shutdown(p) => p.is_shutdown());
    method!(r, "klippy.printer.Printer", Printer, update_error_msg(p, old: String, new: String) => p.update_error_msg(&old, &new));
    method!(r, "klippy.printer.Printer", Printer, add_object(p, name: String, obj: PObj) => p.add_object(&name, obj));
    r.method("klippy.printer.Printer.lookup_object", lookup_object);
    method!(r, "klippy.printer.Printer", Printer, lookup_objects(p, module: Option<String>) => p.lookup_objects(module.as_deref()));
    r.method("klippy.printer.Printer.load_object", load_object);
    method!(r, "klippy.printer.Printer", Printer, set_rollover_info(p, name: String, info: Option<String>, log: bool) =>
        p.set_rollover_info(&name, info.as_deref(), log));
    method!(r, "klippy.printer.Printer", Printer, invoke_shutdown(p, msg: String, details: Option<PyVal>) => p.invoke_shutdown(&msg, details));
    method!(r, "klippy.printer.Printer", Printer, invoke_async_shutdown(p, msg: String, details: Option<PyVal>) =>
        p.invoke_async_shutdown(&msg, details));
    r.method("klippy.printer.Printer.register_event_handler", register_event_handler);
    r.method("klippy.printer.Printer.send_event", send_event);
    method!(r, "klippy.printer.Printer", Printer, request_exit(p, result: String) => p.request_exit(&result));
    r.method("klippy.printer.Printer.wait_while", wait_while);
    // attributes
    r.attr("klippy.printer.Printer.reactor", |o| objects::unwrap_as::<Printer>(o)?.get_reactor().to_py(), None);
    r.attr("klippy.printer.Printer.objects", |o| lookup("_k", "ObjectsView")?.call(&[o]), None);
    r.attr("klippy.printer.Printer.start_args", |o| {
        let p = objects::unwrap_as::<Printer>(o)?;
        start_args(&p)
    }, None);
    r.attr("klippy.printer.Printer.state_message", |o| Ok(py_str(&objects::unwrap_as::<Printer>(o)?.get_state_message().0)), None);
    // printer.objects (prelude.py ObjectsView)
    fn pr(a: &Args) -> KResult<Rc<Printer>> {
        objects::unwrap_as::<Printer>(a.arg(0))
    }
    r.ext("objects_get", |a| {
        let key = a.string(1)?;
        match pr(a)?.lookup_object_opt(&key) {
            Some(o) => pobj_to_py(&o),
            None => Err(KError::Py("KeyError".into(), py_str(&key).repr()?)),
        }
    });
    r.ext("objects_set", |a| {
        pr(a)?.objects_set(&a.string(1)?, crate::plugins::py_to_pobj(a.arg(2)));
        Ok(py_none())
    });
    r.ext("objects_del", |a| {
        let key = a.string(1)?;
        match pr(a)?.objects_remove(&key) {
            Some(_) => Ok(py_none()),
            None => Err(KError::Py("KeyError".into(), py_str(&key).repr()?)),
        }
    });
    r.ext("objects_keys", |a| pr(a)?.object_names().to_py());
    r.attr("klippy.printer.Printer.in_shutdown_state", |o| Ok(py_bool(objects::unwrap_as::<Printer>(o)?.is_shutdown())), None);
}

use crate::plugins::convert::ToPy;

/// get_start_args(): the dict (a copy: kaliru keeps its own)
fn start_args(p: &Printer) -> KResult<PyRef> {
    Ok(from_pyval(&PyVal::Dict(p.start_args.borrow().clone())))
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

// load_object(config, section, default=configfile.sentinel)
fn load_object(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let config = super::configfile::config_from_py(a.arg(0))?;
    let section = a.string(1)?;
    let required = a.arg(2).is(lookup("klippy.configfile", "sentinel")?.as_ptr());
    match p.load_object(&config, &section, required)? {
        Some(o) => pobj_to_py(&o),
        None => Ok(a.arg(2).clone()),
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

// send_event(event, *params): python handlers get the objects as given
fn send_event(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let params: Vec<PyRef> = Vec::from_py(a.arg(1))?;
    let refs: Vec<&dyn Any> = params.iter().map(|x| x as &dyn Any).collect();
    p.send_event(&a.string(0)?, &refs)?;
    Ok(py_none())
}

// wait_while(condition_cb, error_on_cancel=True, interval=1.0)
fn wait_while(a: &Args) -> KResult<PyRef> {
    let p = a.this::<Printer>()?;
    let cb = a.arg(0).clone();
    p.wait_while(|eventtime| cb.call(&[&py_float(eventtime)])?.is_true(), a.bool(1)?, a.f64(2)?)?;
    Ok(py_none())
}
