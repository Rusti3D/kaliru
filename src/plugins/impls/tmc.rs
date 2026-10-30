// klippy/extras/tmc.py and the driver modules (tmc2130, tmc2208, tmc2209,
// tmc2240, tmc2660, tmc5160, tmc5262, tmc_uart): the driver objects, their
// FieldHelper, register access (mcu_tmc), TMCCommandHelper, the current
// helpers and the setup helpers. Used by klipper_tmc_autotune and
// shaketune (tmc_object.get_status.__self__ is the command helper, as in
// Kalico where the driver's get_status is the helper's bound method).
use super::*;
use crate::configfile::{ConfigWrapper, Dflt};
use crate::extras::tmc::{FieldHelper, TMCCommandHelper, TmcCurrent, TmcDriver, TmcMcu};
use crate::gcode::GCodeCommand;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::DictField;
use crate::{function, method, ro_attr};

fn cfg(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

fn fh(o: &PyRef) -> KResult<Rc<FieldHelper>> {
    objects::unwrap_as::<FieldHelper>(o)
}

fn ch(o: &PyRef) -> KResult<Rc<TMCCommandHelper>> {
    objects::unwrap_as::<TMCCommandHelper>(o)
}

fn drv(o: &PyRef) -> KResult<Rc<TmcDriver>> {
    objects::unwrap_as::<TmcDriver>(o)
}

pub fn mcu_tmc_to_py(m: &Rc<dyn TmcMcu>) -> KResult<PyRef> {
    let any: Rc<dyn Any> = m.clone();
    let name = m.type_name();
    objects::wrap_view(any, m.clone(), name)
}

/// a driver's register access object (kaliru's own: MCU_TMC_uart / _SPI)
pub fn mcu_tmc_from_py(o: &PyRef) -> KResult<Rc<dyn TmcMcu>> {
    objects::unwrap_view::<dyn TmcMcu>(o).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: expected a TMC register access object".into()))
}

fn current_to_py(c: &Rc<dyn TmcCurrent>) -> KResult<PyRef> {
    let any: Rc<dyn Any> = c.clone();
    let name = c.type_name();
    objects::wrap_view(any, c.clone(), name)
}

fn current_of(o: &PyRef) -> KResult<Rc<dyn TmcCurrent>> {
    objects::unwrap_view::<dyn TmcCurrent>(o).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: expected a TMC current helper".into()))
}

fn slf(a: &crate::plugins::native::Args) -> &PyRef {
    a.slf.as_ref().expect("method")
}

fn opt_i64(o: &PyRef) -> KResult<Option<i64>> {
    FromPy::from_py(o)
}

fn opt_str(o: &PyRef) -> KResult<Option<String>> {
    FromPy::from_py(o)
}

/// the mcu_tmc methods of a register access class
macro_rules! mcu_tmc_methods {
    ($r:expr, $class:literal) => {
        $r.method(concat!($class, ".get_fields"), |a| mcu_tmc_from_py(slf(a))?.get_fields().to_py());
        $r.method(concat!($class, ".get_register_raw"), |a| {
            let raw = mcu_tmc_from_py(slf(a))?.get_register_raw(&a.string(0)?)?;
            let d = lookup("builtins", "dict")?.call(&[])?;
            d.call_method("__setitem__", &[&py_str("data"), &py_int(raw.data)])?;
            d.call_method("__setitem__", &[&py_str("#receive_time"), &py_float(raw.receive_time)])?;
            if let Some(s) = raw.spi_status {
                d.call_method("__setitem__", &[&py_str("spi_status"), &py_int(s)])?;
            }
            Ok(d)
        });
        $r.method(concat!($class, ".get_register"), |a| Ok(py_int(mcu_tmc_from_py(slf(a))?.get_register(&a.string(0)?)?)));
        // set_register(reg_name, val, print_time=None)
        $r.method(concat!($class, ".set_register"), |a| {
            mcu_tmc_from_py(slf(a))?.set_register(&a.string(0)?, a.arg(1).as_i64()?, a.opt_f64(2)?)?;
            Ok(py_none())
        });
        $r.method(concat!($class, ".get_tmc_frequency"), |a| mcu_tmc_from_py(slf(a))?.get_tmc_frequency().to_py());
        $r.method(concat!($class, ".get_mcu"), |a| mcu_tmc_from_py(slf(a))?.get_mcu().to_py());
    };
}

/// get_current / apply_current of a current helper class
macro_rules! current_methods {
    ($r:expr, $class:literal) => {
        $r.method(concat!($class, ".get_current"), |a| current_of(slf(a))?.get_current().to_py());
        $r.method(concat!($class, ".apply_current"), |a| {
            current_of(slf(a))?.apply_current(a.opt_f64(0)?)?;
            Ok(py_none())
        });
    };
}

/// a driver class: fields, mcu_tmc and the command helper's bound methods
macro_rules! driver_class {
    ($r:expr, $class:literal, $module:ident) => {
        $r.ctor(concat!($class, ".__init__"), |a| {
            let pobj = crate::extras::$module::load_config_prefix(&*cfg(a.arg(0))?)?;
            let d: Rc<dyn Any> = crate::printer::downcast_rc::<TmcDriver>(pobj)
                .ok_or_else(|| KError::Other("kaliru: the driver is not a TMC driver".into()))?;
            Ok(d)
        });
        $r.attr(concat!($class, ".fields"), |o| drv(o)?.fields.clone().to_py(), None);
        $r.attr(concat!($class, ".mcu_tmc"), |o| mcu_tmc_to_py(&drv(o)?.mcu_tmc), None);
        // self.get_status = cmdhelper.get_status (likewise get_phase_offset)
        $r.attr(concat!($class, ".get_status"), |o| objects::wrap(drv(o)?.cmdhelper.clone())?.getattr("get_status"), None);
        $r.attr(concat!($class, ".get_phase_offset"), |o| objects::wrap(drv(o)?.cmdhelper.clone())?.getattr("get_phase_offset"), None);
    };
}

pub fn register(r: &mut Registry) {
    objects::register_class_fn::<TmcDriver>(|a| match a.downcast_ref::<TmcDriver>().map(|d| d.class) {
        Some("TMC2130") => ("klippy.extras.tmc2130", "TMC2130"),
        Some("TMC2208") => ("klippy.extras.tmc2208", "TMC2208"),
        Some("TMC2240") => ("klippy.extras.tmc2240", "TMC2240"),
        Some("TMC2660") => ("klippy.extras.tmc2660", "TMC2660"),
        Some("TMC5160") => ("klippy.extras.tmc5160", "TMC5160"),
        Some("TMC5262") => ("klippy.extras.tmc5262", "TMC5262"),
        _ => ("klippy.extras.tmc2209", "TMC2209"),
    });
    objects::register_class::<crate::extras::tmc_uart::McuTmcUart>("klippy.extras.tmc_uart", "MCU_TMC_uart");
    objects::register_class::<crate::extras::tmc2130::McuTmcSpi>("klippy.extras.tmc2130", "MCU_TMC_SPI");
    objects::register_class::<crate::extras::tmc2660::McuTmc2660Spi>("klippy.extras.tmc2660", "MCU_TMC2660_SPI");

    function!(r, "klippy.extras.tmc", ffs(mask: i64) => crate::extras::tmc::ffs(mask) as i64);

    // -------------------------------------------------------- FieldHelper
    // FieldHelper(all_fields, signed_fields=None, field_formatters=None, registers=None)
    r.ctor("klippy.extras.tmc.FieldHelper.__init__", |a| {
        let all: Vec<(String, PyRef)> = FromPy::from_py(&a.arg(0).call_method("items", &[])?)?;
        let mut all_fields = crate::IndexMap::default();
        for (reg, fields) in all {
            let f: Vec<(String, i64)> = FromPy::from_py(&fields.call_method("items", &[])?)?;
            all_fields.insert(reg, f.into_iter().collect());
        }
        let signed: Vec<String> = if a.arg(1).is_none() { vec![] } else { FromPy::from_py(a.arg(1))? };
        let signed_refs: Vec<&str> = signed.iter().map(|s| s.as_str()).collect();
        let h = FieldHelper::new(all_fields, &signed_refs, None);
        if !a.arg(3).is_none() {
            let regs: Vec<(String, i64)> = FromPy::from_py(&a.arg(3).call_method("items", &[])?)?;
            *h.registers.borrow_mut() = regs.into_iter().collect();
        }
        let h: Rc<dyn Any> = Rc::new(h);
        Ok(h)
    });
    // lookup_register(field_name, default=None)
    r.method("klippy.extras.tmc.FieldHelper.lookup_register", |a| match a.this::<FieldHelper>()?.lookup_register(&a.string(0)?) {
        Some(reg) => Ok(py_str(&reg)),
        None => Ok(a.arg(1).clone()),
    });
    // get_field(field_name, reg_value=None, reg_name=None)
    r.method("klippy.extras.tmc.FieldHelper.get_field", |a| {
        let h = a.this::<FieldHelper>()?;
        Ok(py_int(h.get_field_full(&a.string(0)?, opt_i64(a.arg(1))?, opt_str(a.arg(2))?.as_deref())?))
    });
    // set_field(field_name, field_value, reg_value=None, reg_name=None)
    r.method("klippy.extras.tmc.FieldHelper.set_field", |a| {
        let h = a.this::<FieldHelper>()?;
        Ok(py_int(h.set_field_full(&a.string(0)?, a.arg(1).as_i64()?, opt_i64(a.arg(2))?, opt_str(a.arg(3))?.as_deref())?))
    });
    // set_config_field(config, field_name, default)
    r.method("klippy.extras.tmc.FieldHelper.set_config_field", |a| {
        let default = if a.arg(2).is_none() { Dflt::None } else { Dflt::Val(to_pyval(a.arg(2))?) };
        a.this::<FieldHelper>()?.set_config_field(&*cfg(a.arg(0))?, &a.string(1)?, default).to_py()
    });
    method!(r, "klippy.extras.tmc.FieldHelper", FieldHelper, pretty_format(h, reg_name: String, reg_value: i64) => h.pretty_format(&reg_name, reg_value));
    r.method("klippy.extras.tmc.FieldHelper.get_reg_fields", |a| {
        let fields = a.this::<FieldHelper>()?.get_reg_fields(&a.string(0)?, a.arg(1).as_i64()?);
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in fields {
            d.call_method("__setitem__", &[&py_str(&k), &py_int(v)])?;
        }
        Ok(d)
    });
    // all_fields: {reg_name: {field_name: mask}}
    r.attr("klippy.extras.tmc.FieldHelper.all_fields", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (reg, fields) in &fh(o)?.all_fields {
            let f = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in fields {
                f.call_method("__setitem__", &[&py_str(k), &py_int(*v)])?;
            }
            d.call_method("__setitem__", &[&py_str(reg), &f])?;
        }
        Ok(d)
    }, None);
    ro_attr!(r, "klippy.extras.tmc.FieldHelper.signed_fields", FieldHelper, |h| h.signed_fields.clone());
    r.attr("klippy.extras.tmc.FieldHelper.field_to_register", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in &fh(o)?.field_to_register {
            d.call_method("__setitem__", &[&py_str(k), &py_str(v)])?;
        }
        Ok(d)
    }, None);
    // field_formatters: the drivers' formatters are kaliru functions (an empty dict here)
    r.attr("klippy.extras.tmc.FieldHelper.field_formatters", |_o| lookup("builtins", "dict")?.call(&[]), None);
    r.dict("klippy.extras.tmc.FieldHelper.registers", DictField {
        get: |o, _f, key| match fh(o)?.registers.borrow().get(&key.as_string()?) {
            Some(v) => Ok(py_int(*v)),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        set: |o, _f, key, value| {
            fh(o)?.registers.borrow_mut().insert(key.as_string()?, value.as_i64()?);
            Ok(())
        },
        del: |o, _f, key| match fh(o)?.registers.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(fh(o)?.registers.borrow().keys().map(|k| py_str(k)).collect()),
    });

    // ------------------------------------------------ register access
    mcu_tmc_methods!(r, "klippy.extras.tmc_uart.MCU_TMC_uart");
    mcu_tmc_methods!(r, "klippy.extras.tmc2130.MCU_TMC_SPI");
    mcu_tmc_methods!(r, "klippy.extras.tmc2660.MCU_TMC2660_SPI");

    // --------------------------------------------------- TMCCommandHelper
    // TMCCommandHelper(config, mcu_tmc, current_helper, temp_from_adc=None, init_callback=None)
    r.ctor("klippy.extras.tmc.TMCCommandHelper.__init__", |a| {
        if !a.arg(3).is_none() {
            return Err(KError::Py("NotImplementedError".into(), "kaliru: a python temp_from_adc".into()));
        }
        let init_callback: Option<Rc<dyn Fn() -> KResult<()>>> = if a.arg(4).is_none() {
            None
        } else {
            let cb = a.arg(4).clone();
            Some(Rc::new(move || cb.call(&[]).map(|_| ())))
        };
        let h: Rc<dyn Any> = TMCCommandHelper::new(&*cfg(a.arg(0))?, &mcu_tmc_from_py(a.arg(1))?, current_of(a.arg(2))?, None, init_callback)?;
        Ok(h)
    });
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _init_registers(h, print_time: Option<f64>) => h.init_registers(print_time));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, cmd_INIT_TMC(h, g: PyRef) => h.cmd_init_tmc(&*gcmd(&g)?));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, cmd_SET_TMC_FIELD(h, g: PyRef) => h.cmd_set_tmc_field(&*gcmd(&g)?));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, cmd_SET_TMC_CURRENT(h, g: PyRef) => h.cmd_set_tmc_current(&*gcmd(&g)?));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, cmd_DUMP_TMC(h, g: PyRef) => h.cmd_dump_tmc(&*gcmd(&g)?));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _get_phases(h) => h.get_phases());
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, clear_mcu_phase_offset(h) => h.clear_mcu_phase_offset());
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, get_phase_offset(h) => h.get_phase_offset());
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _query_phase(h) => h.query_phase());
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _handle_sync_mcu_pos(h, stepper: Rc<crate::stepper::McuStepper>) =>
        h.handle_sync_mcu_pos(&stepper));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _do_enable(h, print_time: f64) => h.do_enable(print_time));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _do_disable(h, print_time: f64) => h.do_disable(print_time));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _handle_stepper_enable(h, print_time: f64, is_enable: bool) =>
        h.handle_stepper_enable(print_time, is_enable));
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _handle_mcu_identify(h) => h.handle_mcu_identify());
    method!(r, "klippy.extras.tmc.TMCCommandHelper", TMCCommandHelper, _handle_connect(h) => h.handle_connect());
    // get_status(eventtime=None)
    r.method("klippy.extras.tmc.TMCCommandHelper.get_status", |a| Ok(from_pyval(&a.this::<TMCCommandHelper>()?.get_status())));
    // setup_register_dump(read_registers, read_translate=None): read_translate(reg_name, val) -> (reg_name, val)
    r.method("klippy.extras.tmc.TMCCommandHelper.setup_register_dump", |a| {
        let regs: Vec<String> = FromPy::from_py(a.arg(0))?;
        let refs: Vec<&str> = regs.iter().map(|s| s.as_str()).collect();
        let translate: Option<crate::extras::tmc::ReadTranslate> = if a.arg(1).is_none() {
            None
        } else {
            let f = a.arg(1).clone();
            Some(Rc::new(move |reg: &str, val: i64| {
                let r = f.call(&[&py_str(reg), &py_int(val)]).and_then(|t| <(String, i64)>::from_py(&t));
                r.unwrap_or_else(|e| {
                    report_unhandled(&e);
                    (reg.to_string(), val)
                })
            }))
        };
        a.this::<TMCCommandHelper>()?.setup_register_dump(&refs, translate)?;
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.printer", TMCCommandHelper, |h| h.printer.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.name", TMCCommandHelper, |h| h.name.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.stepper_name", TMCCommandHelper, |h| h.stepper_name.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.fields", TMCCommandHelper, |h| h.fields.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.stepper", TMCCommandHelper, |h| h.stepper.borrow().clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.stepper_enable", TMCCommandHelper, |h| h.stepper_enable.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.enable_mutex", TMCCommandHelper, |h| h.enable_mutex.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.echeck_helper", TMCCommandHelper, |h| h.echeck_helper.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.record_helper", TMCCommandHelper, |h| h.record_helper.clone());
    ro_attr!(r, "klippy.extras.tmc.TMCCommandHelper.read_registers", TMCCommandHelper, |h| h.read_registers.borrow().clone());
    crate::cell_attr!(r, "klippy.extras.tmc.TMCCommandHelper.mcu_phase_offset", TMCCommandHelper, mcu_phase_offset);
    crate::cell_attr!(r, "klippy.extras.tmc.TMCCommandHelper.toff", TMCCommandHelper, toff);
    r.attr("klippy.extras.tmc.TMCCommandHelper.mcu_tmc", |o| mcu_tmc_to_py(&ch(o)?.mcu_tmc), None);
    r.attr("klippy.extras.tmc.TMCCommandHelper.current_helper", |o| current_to_py(&ch(o)?.current_helper), None);
    r.attr("klippy.extras.tmc.TMCCommandHelper.init_callback", |o| match ch(o)?.init_callback.clone() {
        Some(cb) => crate::plugins::callable::py_function(Rc::new(move |_a: &[PyRef]| {
            cb()?;
            Ok(py_none())
        })),
        None => Ok(py_none()),
    }, None);
    r.attr("klippy.extras.tmc.TMCCommandHelper.read_translate", |o| match ch(o)?.read_translate.borrow().clone() {
        Some(f) => crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| f(&a[0].as_string()?, a[1].as_i64()?).to_py())),
        None => Ok(py_none()),
    }, None);

    // --------------------------------------------------- current helpers
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.needs_home_current_change", |a| Ok(py_bool(current_of(slf(a))?.base().needs_home_current_change())));
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.needs_run_current_change", |a| Ok(py_bool(current_of(slf(a))?.base().needs_run_current_change())));
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.needs_hold_current_change", |a| {
        Ok(py_bool(current_of(slf(a))?.base().needs_hold_current_change(a.f64(0)?)))
    });
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.set_home_current", |a| {
        current_of(slf(a))?.base().set_home_current(a.f64(0)?);
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.set_run_current", |a| {
        current_of(slf(a))?.base().set_run_current(a.f64(0)?);
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.set_hold_current", |a| {
        current_of(slf(a))?.base().set_hold_current(a.f64(0)?);
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.set_actual_current", |a| {
        current_of(slf(a))?.base().set_actual_current(a.f64(0)?);
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.set_current_for_homing", |a| {
        Ok(py_float(current_of(slf(a))?.set_current_for_homing_impl(a.f64(0)?, a.bool(1)?)?))
    });
    // needs_current_changes(run_current, hold_current, force=False)
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.needs_current_changes", |a| {
        Ok(py_bool(current_of(slf(a))?.base().needs_current_changes(a.f64(0)?, a.f64(1)?, a.bool(2)?)))
    });
    // set_current(new_current, hold_current, print_time, force=False)
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.set_current", |a| {
        current_of(slf(a))?.set_current(a.f64(0)?, a.f64(1)?, a.opt_f64(2)?, a.bool(3)?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.BaseTMCCurrentHelper.apply_current", |a| {
        current_of(slf(a))?.apply_current(a.opt_f64(0)?)?;
        Ok(py_none())
    });
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.printer", |o| current_of(o)?.base().printer.clone().to_py(), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.name", |o| Ok(py_str(&current_of(o)?.base().name)), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.mcu_tmc", |o| mcu_tmc_to_py(&current_of(o)?.base().mcu_tmc), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.fields", |o| current_of(o)?.base().fields.clone().to_py(), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.sense_resistor", |o| Ok(py_float(current_of(o)?.base().sense_resistor)), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.config_run_current", |o| Ok(py_float(current_of(o)?.base().config_run_current)), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.config_hold_current", |o| Ok(py_float(current_of(o)?.base().config_hold_current)), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.config_home_current", |o| Ok(py_float(current_of(o)?.base().config_home_current)), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.current_change_dwell_time", |o| {
        Ok(py_float(current_of(o)?.base().current_change_dwell_time))
    }, None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.max_current", |o| Ok(py_float(current_of(o)?.base().max_current)), None);
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.req_run_current", |o| Ok(py_float(current_of(o)?.base().req_run_current.get())), Some(|o, v| {
        current_of(o)?.base().req_run_current.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.req_hold_current", |o| Ok(py_float(current_of(o)?.base().req_hold_current.get())), Some(|o, v| {
        current_of(o)?.base().req_hold_current.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.req_home_current", |o| Ok(py_float(current_of(o)?.base().req_home_current.get())), Some(|o, v| {
        current_of(o)?.base().req_home_current.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.tmc.BaseTMCCurrentHelper.actual_current", |o| Ok(py_float(current_of(o)?.base().actual_current.get())), Some(|o, v| {
        current_of(o)?.base().actual_current.set(v.as_f64()?);
        Ok(())
    }));
    current_methods!(r, "klippy.extras.tmc2130.TMC2130CurrentHelper");
    current_methods!(r, "klippy.extras.tmc2240.TMC2240CurrentHelper");
    current_methods!(r, "klippy.extras.tmc5160.TMC5160CurrentHelper");
    current_methods!(r, "klippy.extras.tmc5262.TMC5262CurrentHelper");
    r.method("klippy.extras.tmc2660.TMC2660CurrentHelper.get_current", |a| current_of(slf(a))?.get_current().to_py());
    // (TMC2660's set_current(run_current, hold_current, print_time, force=False))
    r.method("klippy.extras.tmc2660.TMC2660CurrentHelper.set_current", |a| {
        current_of(slf(a))?.set_current(a.f64(0)?, a.f64(1)?, a.opt_f64(2)?, a.bool(3)?)?;
        Ok(py_none())
    });

    // ----------------------------------------------------- setup helpers
    r.method("klippy.extras.tmc.TMCWaveTableHelper", |a| {
        crate::extras::tmc::tmc_wave_table_helper(&*cfg(a.arg(0))?, &mcu_tmc_from_py(a.arg(1))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.TMCMicrostepHelper", |a| {
        crate::extras::tmc::tmc_microstep_helper(&*cfg(a.arg(0))?, &mcu_tmc_from_py(a.arg(1))?)?;
        Ok(py_none())
    });
    // TMCtstepHelper(mcu_tmc, velocity, pstepper=None, config=None)
    r.method("klippy.extras.tmc.TMCtstepHelper", |a| {
        let m = mcu_tmc_from_py(a.arg(0))?;
        let stepper = if a.arg(2).is_none() { None } else { Some(objects::unwrap_as::<crate::stepper::McuStepper>(a.arg(2))?) };
        let config = if a.arg(3).is_none() { None } else { Some(cfg(a.arg(3))?) };
        Ok(py_int(crate::extras::tmc::tmc_tstep_helper(&*m, a.f64(1)?, stepper.as_ref(), config.as_deref())?))
    });
    r.method("klippy.extras.tmc.TMCStealthchopHelper", |a| {
        crate::extras::tmc::tmc_stealthchop_helper(&*cfg(a.arg(0))?, &mcu_tmc_from_py(a.arg(1))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.TMCVcoolthrsHelper", |a| {
        crate::extras::tmc::tmc_vcoolthrs_helper(&*cfg(a.arg(0))?, &mcu_tmc_from_py(a.arg(1))?)?;
        Ok(py_none())
    });
    r.method("klippy.extras.tmc.TMCVhighHelper", |a| {
        crate::extras::tmc::tmc_vhigh_helper(&*cfg(a.arg(0))?, &mcu_tmc_from_py(a.arg(1))?)?;
        Ok(py_none())
    });

    // ------------------------------------------------------- the drivers
    driver_class!(r, "klippy.extras.tmc2130.TMC2130", tmc2130);
    driver_class!(r, "klippy.extras.tmc2208.TMC2208", tmc2208);
    driver_class!(r, "klippy.extras.tmc2209.TMC2209", tmc2209);
    driver_class!(r, "klippy.extras.tmc2240.TMC2240", tmc2240);
    driver_class!(r, "klippy.extras.tmc2660.TMC2660", tmc2660);
    driver_class!(r, "klippy.extras.tmc5160.TMC5160", tmc5160);
    driver_class!(r, "klippy.extras.tmc5262.TMC5262", tmc5262);
    r.method("klippy.extras.tmc2130.load_config_prefix", |a| crate::extras::tmc2130::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.tmc2208.load_config_prefix", |a| crate::extras::tmc2208::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.tmc2209.load_config_prefix", |a| crate::extras::tmc2209::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.tmc2240.load_config_prefix", |a| crate::extras::tmc2240::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.tmc2660.load_config_prefix", |a| crate::extras::tmc2660::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.tmc5160.load_config_prefix", |a| crate::extras::tmc5160::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
    r.method("klippy.extras.tmc5262.load_config_prefix", |a| crate::extras::tmc5262::load_config_prefix(&*cfg(a.arg(0))?)?.to_py());
}
