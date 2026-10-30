// klippy/configfile.py: ConfigWrapper runs as Kalico's own code
// (tools/mirror_verbatim.txt) on its attributes; for kaliru's configs those
// are live views of kaliru's parser (prelude.py FileConfig) and access
// tracking (AccessTracking). A ConfigWrapper a plugin builds is plain python
// until kaliru needs it (config_from_py).
use super::*;
use crate::configfile::{AccessTracking, ConfigWrapper};
use crate::printer::ConfigFileObject;
use crate::configparser::RawConfigParser;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::Args;
use std::cell::RefCell;

type Parser = RefCell<RawConfigParser>;
type Tracking = RefCell<AccessTracking>;

pub fn register(r: &mut Registry) {
    objects::register_class::<crate::printer::ConfigFileObject>("klippy.configfile", "PrinterConfig");
    objects::register_class::<Parser>("_k", "FileConfig");
    objects::register_class::<Tracking>("_k", "AccessTracking");
    fn cw(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
        objects::unwrap_as::<ConfigWrapper>(o)
    }
    r.attr("klippy.configfile.ConfigWrapper.printer", |o| cw(o)?.get_printer().to_py(), None);
    r.attr("klippy.configfile.ConfigWrapper.fileconfig", |o| cw(o)?.fileconfig.clone().to_py(), None);
    r.attr("klippy.configfile.ConfigWrapper.access_tracking", |o| cw(o)?.access_tracking.clone().to_py(), None);
    r.attr("klippy.configfile.ConfigWrapper.section", |o| Ok(py_str(cw(o)?.get_name())), None);

    // ------------------------------------------------------- PrinterConfig
    // (on the "configfile" object; its printer is the running one)
    fn pc(o: &PyRef) -> KResult<Rc<ConfigFileObject>> {
        objects::unwrap_as::<ConfigFileObject>(o)
    }
    fn the_printer() -> KResult<Rc<Printer>> {
        super::printer().ok_or_else(|| KError::Other("no printer".into()))
    }
    fn wrap_cfg(c: ConfigWrapper) -> KResult<PyRef> {
        objects::wrap(Rc::new(c.with_printer(&the_printer()?)))
    }
    // PrinterConfig(printer): SAVE_CONFIG is kaliru's already
    r.ctor("klippy.configfile.PrinterConfig.__init__", |a| {
        let printer: Rc<Printer> = FromPy::from_py(a.arg(0))?;
        let config_file = printer.start_arg("config_file").map(|v| v.str()).unwrap_or_default();
        let c: Rc<dyn Any> = Rc::new(ConfigFileObject { pconfig: RefCell::new(crate::configfile::PrinterConfig::new(&config_file)) });
        Ok(c)
    });
    r.method("klippy.configfile.PrinterConfig.get_printer", |_a| the_printer()?.to_py());
    r.attr("klippy.configfile.PrinterConfig.printer", |_o| the_printer()?.to_py(), None);
    r.method("klippy.configfile.PrinterConfig._read_config_file", |a| {
        Ok(py_str(&a.this::<ConfigFileObject>()?.pconfig.borrow().read_config_file(&a.string(0)?)?))
    });
    r.method("klippy.configfile.PrinterConfig._find_autosave_data", |a| {
        a.this::<ConfigFileObject>()?.pconfig.borrow().find_autosave_data(&a.string(0)?).to_py()
    });
    r.method("klippy.configfile.PrinterConfig._strip_duplicates", |a| {
        let config = config_from_py(a.arg(1))?;
        Ok(py_str(&a.this::<ConfigFileObject>()?.pconfig.borrow().strip_duplicates(&a.string(0)?, &config)))
    });
    r.method("klippy.configfile.PrinterConfig._build_config_wrapper", |a| {
        let c = a.this::<ConfigFileObject>()?.pconfig.borrow().build_config_wrapper(&a.string(0)?, &a.string(1)?)?;
        wrap_cfg(c)
    });
    r.method("klippy.configfile.PrinterConfig._build_config_string", |a| {
        let config = config_from_py(a.arg(0))?;
        Ok(py_str(&a.this::<ConfigFileObject>()?.pconfig.borrow().build_config_string(&config)))
    });
    r.method("klippy.configfile.PrinterConfig.read_config", |a| {
        let c = a.this::<ConfigFileObject>()?.pconfig.borrow().read_config(&a.string(0)?)?;
        wrap_cfg(c)
    });
    r.method("klippy.configfile.PrinterConfig.read_main_config", |a| {
        let c = a.this::<ConfigFileObject>()?.pconfig.borrow_mut().read_main_config()?;
        wrap_cfg(c)
    });
    r.method("klippy.configfile.PrinterConfig.check_unused_options", |a| {
        let config = config_from_py(a.arg(0))?;
        let names = the_printer()?.object_names();
        a.this::<ConfigFileObject>()?.pconfig.borrow_mut().check_unused_options(&config, a.bool(1)?, &names)?;
        Ok(py_none())
    });
    r.method("klippy.configfile.PrinterConfig.log_config", |a| {
        let config = config_from_py(a.arg(0))?;
        let text = a.this::<ConfigFileObject>()?.pconfig.borrow().log_config(&config);
        the_printer()?.set_rollover_info("config", Some(&text), true);
        Ok(py_none())
    });
    r.method("klippy.configfile.PrinterConfig.runtime_warning", |a| {
        a.this::<ConfigFileObject>()?.pconfig.borrow_mut().runtime_warning(&a.string(0)?);
        Ok(py_none())
    });
    // deprecate(section, option, value=None, msg=None): kaliru keeps msg a str
    r.method("klippy.configfile.PrinterConfig.deprecate", |a| {
        let (value, msg) = (a.opt_string(2)?, a.opt_string(3)?.unwrap_or_default());
        a.this::<ConfigFileObject>()?.pconfig.borrow().deprecate(&a.string(0)?, &a.string(1)?, value.as_deref(), &msg);
        Ok(py_none())
    });
    r.method("klippy.configfile.PrinterConfig.warn", |a| {
        let (section, option, value) = (a.opt_string(2)?, a.opt_string(3)?, a.opt_string(4)?);
        a.this::<ConfigFileObject>()?.pconfig.borrow_mut().warn(
            &a.string(0)?,
            &a.string(1)?,
            section.as_deref(),
            option.as_deref(),
            value.as_deref(),
        );
        Ok(py_none())
    });
    r.method("klippy.configfile.PrinterConfig._build_status", |a| {
        let config = config_from_py(a.arg(0))?;
        a.this::<ConfigFileObject>()?.pconfig.borrow_mut().build_status(&config)?;
        Ok(py_none())
    });
    // set(section, option, value): python str(value)
    r.method("klippy.configfile.PrinterConfig.set", |a| {
        let svalue = lookup("builtins", "str")?.call(&[a.arg(2)])?.as_string()?;
        a.this::<ConfigFileObject>()?.pconfig.borrow_mut().set(&a.string(0)?, &a.string(1)?, &svalue)?;
        Ok(py_none())
    });
    r.method("klippy.configfile.PrinterConfig.remove_section", |a| {
        a.this::<ConfigFileObject>()?.pconfig.borrow_mut().remove_section(&a.string(0)?);
        Ok(py_none())
    });
    r.method("klippy.configfile.PrinterConfig.cmd_SAVE_CONFIG", |a| {
        let gcmd = super::gcode::gcmd_from_py(a.arg(0))?;
        a.this::<ConfigFileObject>()?.cmd_save_config(&*the_printer()?, &gcmd)?;
        Ok(py_none())
    });
    r.attr("klippy.configfile.PrinterConfig.autosave", |o| match pc(o)?.pconfig.borrow().autosave.clone() {
        Some(c) => wrap_cfg(c),
        None => Ok(py_none()),
    }, None);
    // {(section, option, value): msg}
    r.attr("klippy.configfile.PrinterConfig.deprecated", |o| {
        let deprecated = pc(o)?.pconfig.borrow().deprecated.borrow().clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for ((section, option, value), msg) in deprecated {
            let key = py_tuple(vec![py_str(&section), py_str(&option), value.as_deref().map_or_else(py_none, py_str)]);
            d.call_method("__setitem__", &[&key, &py_str(&msg)])?;
        }
        Ok(d)
    }, None);
    // the environment Kalico builds (kaliru renders includes itself)
    r.attr("klippy.configfile.PrinterConfig.jinja_env", |o| {
        thread_local!(static ENV: RefCell<Option<PyRef>> = const { RefCell::new(None) });
        pc(o)?;
        if let Some(e) = ENV.with(|e| e.borrow().clone()) {
            return Ok(e);
        }
        let env = lookup("_k", "config_jinja_env")?.call(&[])?;
        ENV.with(|e| *e.borrow_mut() = Some(env.clone()));
        Ok(env)
    }, None);
    macro_rules! pc_val {
        ($r:ident, $($field:ident),*) => {$(
            $r.attr(concat!("klippy.configfile.PrinterConfig.", stringify!($field)), |o| {
                Ok(from_pyval(&pc(o)?.pconfig.borrow().$field))
            }, Some(|o, v| {
                pc(o)?.pconfig.borrow_mut().$field = PyVal::shared(to_pyval(v)?);
                Ok(())
            }));
        )*};
    }
    pc_val!(r, status_raw_config, status_save_pending, status_settings, status_warnings);
    r.attr("klippy.configfile.PrinterConfig.runtime_warnings", |o| pc(o)?.pconfig.borrow().runtime_warnings.clone().to_py(), None);
    r.attr("klippy.configfile.PrinterConfig.deprecate_warnings", |o| pc(o)?.pconfig.borrow().deprecate_warnings.clone().to_py(), None);
    r.attr("klippy.configfile.PrinterConfig.unused_sections", |o| pc(o)?.pconfig.borrow().unused_sections.clone().to_py(), None);
    r.attr("klippy.configfile.PrinterConfig.unused_options", |o| pc(o)?.pconfig.borrow().unused_options.clone().to_py(), None);
    r.attr("klippy.configfile.PrinterConfig.save_config_pending", |o| Ok(py_bool(pc(o)?.pconfig.borrow().save_config_pending)), Some(|o, v| {
        pc(o)?.pconfig.borrow_mut().save_config_pending = v.is_true()?;
        Ok(())
    }));

    // FileConfig primitives (python names its errors itself, prelude.py)
    fn fc(a: &Args) -> KResult<Rc<Parser>> {
        objects::unwrap_as::<Parser>(a.arg(0))
    }
    // the object table's size (tests/cases/plugins/objects_release)
    r.ext("objects_len", |_| Ok(py_int(objects::table_len() as i64)));
    r.ext("fc_sections", |a| fc(a)?.borrow().sections().to_py());
    r.ext("fc_has_section", |a| Ok(py_bool(fc(a)?.borrow().has_section(&a.string(1)?))));
    r.ext("fc_add_section", |a| {
        fc(a)?.borrow_mut().add_section(&a.string(1)?)?;
        Ok(py_none())
    });
    r.ext("fc_remove_section", |a| Ok(py_bool(fc(a)?.borrow_mut().remove_section(&a.string(1)?))));
    r.ext("fc_options", |a| fc(a)?.borrow().options(&a.string(1)?)?.to_py());
    r.ext("fc_defaults", |a| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in fc(a)?.borrow().defaults() {
            d.call_method("__setitem__", &[&py_str(k), &py_str(v)])?;
        }
        Ok(d)
    });
    r.ext("fc_has_option", |a| Ok(py_bool(fc(a)?.borrow().has_option(&a.string(1)?, &a.string(2)?))));
    r.ext("fc_get", |a| {
        let v = fc(a)?.borrow().get_raw(&a.string(1)?, &a.string(2)?, a.bool(3)?)?;
        Ok(py_str(&v))
    });
    r.ext("fc_set", |a| {
        fc(a)?.borrow_mut().set(&a.string(1)?, &a.string(2)?, &a.string(3)?)?;
        Ok(py_none())
    });
    r.ext("fc_remove_option", |a| Ok(py_bool(fc(a)?.borrow_mut().remove_option(&a.string(1)?, &a.string(2)?)?)));

    // AccessTracking: {(section, option): value}
    fn at(a: &Args) -> KResult<Rc<Tracking>> {
        objects::unwrap_as::<Tracking>(a.arg(0))
    }
    r.ext("at_get", |a| {
        let key: (String, String) = FromPy::from_py(a.arg(1))?;
        match at(a)?.borrow().get(&key) {
            Some(v) => Ok(from_pyval(v)),
            None => Err(KError::Py("KeyError".into(), a.arg(1).repr()?)),
        }
    });
    r.ext("at_set", |a| {
        let key: (String, String) = FromPy::from_py(a.arg(1))?;
        at(a)?.borrow_mut().insert(key, to_pyval(a.arg(2))?);
        Ok(py_none())
    });
    r.ext("at_del", |a| {
        let key: (String, String) = FromPy::from_py(a.arg(1))?;
        match at(a)?.borrow_mut().shift_remove(&key) {
            Some(_) => Ok(py_none()),
            None => Err(KError::Py("KeyError".into(), a.arg(1).repr()?)),
        }
    });
    r.ext("at_keys", |a| at(a)?.borrow().keys().cloned().collect::<Vec<_>>().to_py());
}

/// The kaliru config behind a python ConfigWrapper: kaliru's own, or one a
/// plugin built (its parser copied: kaliru objects read their config when
/// they are created)
pub fn config_from_py(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    if o.getattr("_k_h").map(|h| !h.is_none()).unwrap_or(false) {
        return objects::unwrap_as::<ConfigWrapper>(o);
    }
    let printer: Rc<Printer> = FromPy::from_py(&o.getattr("printer")?)?;
    let pyfc = o.getattr("fileconfig")?;
    let fileconfig = match objects::unwrap_as::<Parser>(&pyfc) {
        Ok(p) => p,
        Err(_) => Rc::new(RefCell::new(copy_parser(&pyfc)?)),
    };
    let access_tracking = match objects::unwrap_as::<Tracking>(&o.getattr("access_tracking")?) {
        Ok(t) => t,
        Err(_) => Rc::new(RefCell::new(AccessTracking::default())),
    };
    let section = o.getattr("section")?.as_string()?;
    Ok(Rc::new(ConfigWrapper::from_parts(&printer, fileconfig, access_tracking, &section)))
}

/// A python RawConfigParser's contents as a kaliru parser (raw values; a
/// None value - set(section, option) - becomes "": kaliru's parser has no
/// valueless options, docs/PLUGINS.md)
fn copy_parser(fc: &PyRef) -> KResult<RawConfigParser> {
    let mut out = ConfigWrapper::empty_fileconfig();
    let sections: Vec<String> = FromPy::from_py(&fc.call_method("sections", &[])?)?;
    for s in sections {
        out.add_section(&s)?;
        let section = fc.getattr("_sections")?.call_method("__getitem__", &[&py_str(&s)])?;
        let items: Vec<(String, Option<String>)> = FromPy::from_py(&section.call_method("items", &[])?)?;
        for (k, v) in items {
            out.set(&s, &k, v.as_deref().unwrap_or(""))?;
        }
    }
    Ok(out)
}
