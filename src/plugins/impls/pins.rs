// klippy/pins.py: PrinterPins; a python plugin's pin chip (register_chip)
use super::*;
use crate::plugins::callable::py_function;
use crate::plugins::native::DictField;
use crate::IndexMap;
use std::cell::RefCell;

use crate::mcu::{AdcCallback, AdcPin, DigitalOutPin, Endstop, EndstopRef, Mcu, McuAdc, McuDigitalOut, McuEndstop, McuPwm, PwmPin, StepperRef};
use crate::pins::ForeignPin;
use crate::reactor::{ReactorCompletion, NEVER, NOW};
use crate::method;
use crate::pins::{PinChip, PinParams, PinsObject, SharedPinParams};

pub fn register(r: &mut Registry) {
    objects::register_class::<PinsObject>("klippy.pins", "PrinterPins");
    objects::register_class::<crate::mcu::Mcu>("klippy.mcu", "MCU");
    r.method("klippy.pins.PrinterPins.register_chip", |a| {
        let chip: Rc<dyn PinChip> = Rc::new(PyPinChip { obj: a.arg(1).clone() });
        a.this::<PinsObject>()?.0.borrow_mut().register_chip(&a.string(0)?, chip).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
    // parse_pin(pin_desc, can_invert=False, can_pullup=False)
    r.method("klippy.pins.PrinterPins.parse_pin", |a| {
        let p = a.this::<PinsObject>()?.0.borrow().parse_pin(&a.string(0)?, a.bool(1)?, a.bool(2)?).map_err(|e| KError::Pins(e.0))?;
        pin_params_dict(&p, false)
    });
    // lookup_pin(pin_desc, can_invert=False, can_pullup=False, share_type=None)
    r.method("klippy.pins.PrinterPins.lookup_pin", |a| {
        let shared: SharedPinParams = a
            .this::<PinsObject>()?
            .0
            .borrow_mut()
            .lookup_pin(&a.string(0)?, a.bool(1)?, a.bool(2)?, a.opt_string(3)?.as_deref())
            .map_err(|e| KError::Pins(e.0))?;
        let p = shared.borrow();
        pin_params_dict(&p, true)
    });
    // PinResolver: kaliru's are shared with the chip's MCU
    objects::register_class::<Resolver>("klippy.pins", "PinResolver");
    r.ctor("klippy.pins.PinResolver.__init__", |a| {
        let r: Rc<dyn Any> = Rc::new(RefCell::new(crate::pins::PinResolver::new(a.bool(0)?)));
        Ok(r)
    });
    r.method("klippy.pins.PinResolver.reserve_pin", |a| {
        resolver(a.slf.as_ref().expect("method"))?.borrow_mut().reserve_pin(&a.string(0)?, &a.string(1)?).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
    r.method("klippy.pins.PinResolver.alias_pin", |a| {
        resolver(a.slf.as_ref().expect("method"))?.borrow_mut().alias_pin(&a.string(0)?, &a.string(1)?).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
    r.method("klippy.pins.PinResolver.update_command", |a| {
        let cmd = resolver(a.slf.as_ref().expect("method"))?.borrow_mut().update_command(&a.string(0)?).map_err(|e| KError::Pins(e.0))?;
        Ok(py_str(&cmd))
    });
    r.attr("klippy.pins.PinResolver.validate_aliases", |o| Ok(py_bool(resolver(o)?.borrow().validate_aliases)), Some(|o, v| {
        resolver(o)?.borrow_mut().validate_aliases = v.is_true()?;
        Ok(())
    }));
    r.dict("klippy.pins.PinResolver.reserved", resolver_dict());
    r.dict("klippy.pins.PinResolver.aliases", resolver_dict());
    r.dict("klippy.pins.PinResolver.active_pins", resolver_dict());
    r.method("klippy.pins.PrinterPins.get_pin_resolver", |a| {
        let r = a.this::<PinsObject>()?.0.borrow().get_pin_resolver(&a.string(0)?).map_err(|e| KError::Pins(e.0))?;
        objects::wrap(r)
    });
    // reset_pin_sharing(pin_params): del active_pins["chip_name:pin"]
    r.method("klippy.pins.PrinterPins.reset_pin_sharing", |a| {
        let get = |k: &str| a.arg(0).call_method("__getitem__", &[&py_str(k)])?.as_string();
        let share_name = format!("{}:{}", get("chip_name")?, get("pin")?);
        match a.this::<PinsObject>()?.0.borrow_mut().active_pins.shift_remove(&share_name) {
            Some(_) => Ok(py_none()),
            None => Err(KError::Py("KeyError".into(), crate::pyfmt::py_str_repr(&share_name))),
        }
    });
    // (snapshots of kaliru's tables)
    r.attr("klippy.pins.PrinterPins.chips", |o| {
        let chips = objects::unwrap_as::<PinsObject>(o)?.0.borrow().chips.clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, c) in chips {
            d.call_method("__setitem__", &[&py_str(&name), &chip_to_py(c)?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.pins.PrinterPins.pin_resolvers", |o| {
        let resolvers = objects::unwrap_as::<PinsObject>(o)?.0.borrow().pin_resolvers.clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, r) in resolvers {
            d.call_method("__setitem__", &[&py_str(&name), &objects::wrap(r)?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.pins.PrinterPins.active_pins", |o| {
        let active = objects::unwrap_as::<PinsObject>(o)?.0.borrow().active_pins.clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, p) in active {
            d.call_method("__setitem__", &[&py_str(&name), &pin_params_dict(&p.borrow(), true)?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.pins.PrinterPins.allow_multi_use_pins", |o| {
        let allowed = objects::unwrap_as::<PinsObject>(o)?.0.borrow().allow_multi_use_pins.clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, v) in allowed {
            d.call_method("__setitem__", &[&py_str(&name), &py_bool(v)])?;
        }
        Ok(d)
    }, None);
    r.method("klippy.pins.PrinterPins.allow_multi_use_pin", |a| {
        a.this::<PinsObject>()?.0.borrow_mut().allow_multi_use_pin(&a.string(0)?).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
    // setup_pin(pin_type, pin_desc): the chip's pin object (AFC_canvas's
    // enable pins, its lanes' LED pins, AFC's espoolers and buffer adc)
    r.method("klippy.pins.PrinterPins.setup_pin", |a| {
        let printer = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
        let pin = crate::pins::setup_pin(&printer, &a.string(0)?, &a.string(1)?)?;
        pin_to_py(pin, &a.string(0)?)
    });
    objects::register_class::<McuDigitalOut>("klippy.mcu", "MCU_digital_out");
    objects::register_class::<McuPwm>("klippy.mcu", "MCU_pwm");
    objects::register_class::<McuAdc>("klippy.mcu", "MCU_adc");
    objects::register_class::<McuEndstop>("klippy.mcu", "MCU_endstop");
    objects::register_class::<crate::stepper::McuStepper>("klippy.stepper", "MCU_stepper");
    method!(r, "klippy.mcu.MCU_endstop", McuEndstop, get_mcu(e) => e.get_mcu());
    r.method("klippy.mcu.MCU_endstop.query_endstop", |a| {
        let e = a.this::<McuEndstop>()?;
        Ok(py_int(crate::mcu::Endstop::query_endstop(&*e, a.f64(0)?)?))
    });
    r.method("klippy.mcu.MCU_endstop.get_steppers", |a| {
        let steppers = a.this::<McuEndstop>()?.get_steppers();
        let list: KResult<Vec<PyRef>> = steppers.into_iter().map(objects::wrap).collect();
        Ok(py_list(list?))
    });
    // QueryEndstops.register_endstop(mcu_endstop, name): an endstop a python
    // module set up (AFC's lane / hub / toolhead sensors)
    r.method("klippy.extras.query_endstops.QueryEndstops.register_endstop", |a| {
        let endstop = crate::mcu::any_to_endstop(objects::unwrap(a.arg(0))?)
            .ok_or_else(|| KError::Py("TypeError".into(), "kaliru: register_endstop of an endstop kaliru did not make".into()))?;
        a.this::<crate::extras::query_endstops::QueryEndstops>()?.register_endstop(endstop, &a.string(1)?);
        Ok(py_none())
    });
    method!(r, "klippy.mcu.MCU_digital_out", McuDigitalOut, get_mcu(p) => p.get_mcu());
    r.method("klippy.mcu.MCU_digital_out.setup_max_duration", |a| {
        a.this::<McuDigitalOut>()?.setup_max_duration(a.f64(0)?);
        Ok(py_none())
    });
    // (start / shutdown values: truthiness, as `not not value`)
    r.method("klippy.mcu.MCU_digital_out.setup_start_value", |a| {
        a.this::<McuDigitalOut>()?.setup_start_value(a.bool(0)?, a.bool(1)?);
        Ok(py_none())
    });
    r.method("klippy.mcu.MCU_digital_out.set_digital", |a| {
        a.this::<McuDigitalOut>()?.set_digital(a.f64(0)?, a.bool(1)?)?;
        Ok(py_none())
    });
    method!(r, "klippy.mcu.MCU_pwm", McuPwm, get_mcu(p) => p.get_mcu());
    r.method("klippy.mcu.MCU_pwm.setup_max_duration", |a| {
        a.this::<McuPwm>()?.setup_max_duration(a.f64(0)?);
        Ok(py_none())
    });
    r.method("klippy.mcu.MCU_pwm.setup_cycle_time", |a| {
        a.this::<McuPwm>()?.setup_cycle_time(a.f64(0)?, a.bool(1)?);
        Ok(py_none())
    });
    r.method("klippy.mcu.MCU_pwm.setup_start_value", |a| {
        a.this::<McuPwm>()?.setup_start_value(a.f64(0)?, a.f64(1)?);
        Ok(py_none())
    });
    r.method("klippy.mcu.MCU_pwm.set_pwm", |a| {
        a.this::<McuPwm>()?.set_pwm(a.f64(0)?, a.f64(1)?)?;
        Ok(py_none())
    });
    method!(r, "klippy.mcu.MCU_adc", McuAdc, get_mcu(p) => p.get_mcu());
    r.method("klippy.mcu.MCU_adc.setup_adc_sample", |a| {
        let range_check_count = a.arg(4).as_i64()?;
        a.this::<McuAdc>()?.setup_adc_sample(a.f64(0)?, a.arg(1).as_i64()?, a.f64(2)?, a.f64(3)?, range_check_count);
        Ok(py_none())
    });
    r.method("klippy.mcu.MCU_adc.setup_adc_callback", |a| {
        let cb = a.arg(1).clone();
        a.this::<McuAdc>()?.setup_adc_callback(
            a.f64(0)?,
            Rc::new(move |read_time, read_value| cb.call(&[&py_float(read_time), &py_float(read_value)]).map(|_| ())),
        );
        Ok(py_none())
    });
    r.method("klippy.mcu.MCU_adc.get_last_value", |a| {
        let (t, v) = a.this::<McuAdc>()?.get_last_value();
        Ok(py_tuple(vec![py_float(t), py_float(v)]))
    });
}

/// The pin_params dict python gets (a copy: kaliru keeps the shared one).
/// lookup_pin's (`shared`) has share_type, and "class" once a user set it.
fn pin_params_dict(p: &PinParams, shared: bool) -> KResult<PyRef> {
    let mut d = crate::pydict! {
        "chip_name" => PyVal::Str(p.chip_name.clone()),
        "pin" => PyVal::Str(p.pin.clone()),
        "invert" => PyVal::Int(p.invert),
        "pullup" => PyVal::Int(p.pullup),
    };
    if shared {
        d.insert("share_type".into(), p.share_type.clone().map_or(PyVal::None, PyVal::Str));
    }
    let dict = from_pyval(&PyVal::Dict(d));
    dict.call_method("__setitem__", &[&py_str("chip"), &chip_object(p)?])?;
    if let (true, Some(c)) = (shared, &p.class) {
        dict.call_method("__setitem__", &[&py_str("class"), &objects::wrap_any(c.clone(), "pin class")?])?;
    }
    Ok(dict)
}

/// pin_params["chip"]: the chip registered for the pin's chip name
fn chip_object(p: &PinParams) -> KResult<PyRef> {
    chip_to_py(p.chip.clone())
}

/// A chip as python sees it: a plugin's own object, else kaliru's (the
/// object it is registered as: MCU, probe, sx1509, ...)
fn chip_to_py(c: Rc<dyn PinChip>) -> KResult<PyRef> {
    if let Some(o) = c.python_object() {
        if let Ok(py) = o.downcast::<PyRef>() {
            return Ok((*py).clone());
        }
    }
    let any: Rc<dyn Any> = c.clone();
    let name = c.type_name();
    objects::wrap_view(any, c, name)
}

/// A PinResolver: kaliru's (one per chip) or one a plugin built
type Resolver = RefCell<crate::pins::PinResolver>;

fn resolver(o: &PyRef) -> KResult<Rc<Resolver>> {
    objects::unwrap_as::<Resolver>(o)
}

/// PinResolver.reserved / aliases / active_pins: {name: name}
fn resolver_map<R>(o: &PyRef, f: &str, op: impl FnOnce(&mut IndexMap<String, String>) -> R) -> KResult<R> {
    let r = resolver(o)?;
    let mut r = r.borrow_mut();
    let map = match f.rsplit('.').next() {
        Some("reserved") => &mut r.reserved,
        Some("aliases") => &mut r.aliases,
        _ => &mut r.active_pins,
    };
    Ok(op(map))
}

fn resolver_dict() -> DictField {
    DictField {
        get: |o, f, key| {
            let k = key.as_string()?;
            match resolver_map(o, f, |m| m.get(&k).cloned())? {
                Some(v) => Ok(py_str(&v)),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            }
        },
        set: |o, f, key, value| {
            let (k, v) = (key.as_string()?, value.as_string()?);
            resolver_map(o, f, |m| m.insert(k, v))?;
            Ok(())
        },
        del: |o, f, key| {
            let k = key.as_string()?;
            match resolver_map(o, f, |m| m.shift_remove(&k))? {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            }
        },
        keys: |o, f| resolver_map(o, f, |m| m.keys().map(|k| py_str(k)).collect()),
    }
}

/// A python plugin's pin chip (pins.register_chip(name, obj))
struct PyPinChip {
    obj: PyRef,
}

impl PinChip for PyPinChip {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn std::any::Any>, String> {
        let params = pin_params_dict(&pin_params.borrow(), true).map_err(|e| e.to_string())?;
        let pin = self.obj.call_method("setup_pin", &[&py_str(pin_type), &params]).map_err(|e| e.to_string())?;
        // kaliru's own users of the pin call it through the native pin
        // interface its type has (python duck types these)
        let native: Rc<dyn std::any::Any> = match pin_type {
            "digital_out" => Rc::new(Rc::new(PyPin(pin.clone())) as Rc<dyn DigitalOutPin>),
            "pwm" => Rc::new(Rc::new(PyPin(pin.clone())) as Rc<dyn PwmPin>),
            "adc" => Rc::new(Rc::new(PyPin(pin.clone())) as Rc<dyn AdcPin>),
            "endstop" => Rc::new(Rc::new(PyPin(pin.clone())) as EndstopRef),
            _ => Rc::new(pin.clone()),
        };
        Ok(Rc::new(ForeignPin { obj: Rc::new(pin), native }))
    }

    fn register_config_callback(&self, cb: Rc<dyn Fn() -> KResult<()>>) -> Option<KResult<()>> {
        let f = py_function(Rc::new(move |_a: &[PyRef]| {
            cb()?;
            Ok(py_none())
        }));
        Some(f.and_then(|f| self.obj.call_method("register_config_callback", &[&f])).map(|_| ()))
    }

    fn python_object(&self) -> Option<Rc<dyn std::any::Any>> {
        Some(Rc::new(self.obj.clone()))
    }
}


/// pin_params (a dict, as lookup_pin returns) as kaliru's: parsed again
/// from its chip, pin and flags
pub fn pin_params_from_py(o: &PyRef) -> KResult<crate::pins::SharedPinParams> {
    let chip_name = o.call_method("__getitem__", &[&py_str("chip_name")])?.as_string()?;
    let pin = o.call_method("__getitem__", &[&py_str("pin")])?.as_string()?;
    let invert = o.call_method("get", &[&py_str("invert"), &py_int(0)])?.as_i64()?;
    let pullup = o.call_method("get", &[&py_str("pullup"), &py_int(0)])?.as_i64()?;
    let prefix = match (invert != 0, pullup) {
        (true, _) => "!",
        (false, p) if p > 0 => "^",
        (false, p) if p < 0 => "~",
        _ => "",
    };
    let printer = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
    let ppins = printer.lookup::<crate::pins::PinsObject>("pins")?;
    let desc = format!("{}{}:{}", prefix, chip_name, pin);
    let p = ppins.0.borrow().parse_pin(&desc, true, true).map_err(|e| KError::Pins(e.0))?;
    Ok(Rc::new(RefCell::new(p)))
}

/// A pin a chip's setup_pin() made, as python sees it
pub fn pin_to_py(pin: Rc<dyn std::any::Any>, pin_type: &str) -> KResult<PyRef> {
    // a python plugin's chip made a python object itself
    let pin = match pin.downcast::<ForeignPin>() {
        Ok(f) => match f.obj.clone().downcast::<PyRef>() {
            Ok(py) => return Ok((*py).clone()),
            Err(_) => return Err(KError::Other("kaliru: a foreign pin without its python object".into())),
        },
        Err(p) => p,
    };
    let pin = match pin.downcast::<McuDigitalOut>() {
        Ok(p) => return objects::wrap(p),
        Err(p) => p,
    };
    let pin = match pin.downcast::<McuPwm>() {
        Ok(p) => return objects::wrap(p),
        Err(p) => p,
    };
    let pin = match pin.downcast::<McuEndstop>() {
        Ok(p) => return objects::wrap(p),
        Err(p) => p,
    };
    match pin.downcast::<McuAdc>() {
        Ok(p) => objects::wrap(p),
        Err(_) => Err(KError::Py("NotImplementedError".into(), format!("kaliru: a {} pin of this chip is not mirrored yet", pin_type))),
    }
}

/// An endstop as python sees it: a plugin's own object, else kaliru's
pub fn endstop_to_py(e: &EndstopRef) -> KResult<PyRef> {
    let any: Rc<dyn std::any::Any> = e.clone();
    let any = match any.downcast::<PyPin>() {
        Ok(p) => return Ok(p.0.clone()),
        Err(a) => a,
    };
    let name = e.type_name();
    objects::wrap_view(any, e.clone(), name)
}

/// An endstop python hands kaliru: kaliru's own, else called through
pub fn endstop_from_py(o: &PyRef) -> KResult<EndstopRef> {
    if is_mirror_instance(o)? {
        if let Some(e) = objects::unwrap_view::<dyn Endstop>(o) {
            return Ok(e);
        }
        if let Some(e) = objects::unwrap(o).ok().and_then(crate::mcu::any_to_endstop) {
            return Ok(e);
        }
    }
    Ok(Rc::new(PyPin(o.clone())))
}

/// A python plugin's pin object (a virtual_pins pin, ...) behind kaliru's
/// native pin interfaces: each method calls the python method of that name
struct PyPin(PyRef);

/// log a failed call kaliru's interface cannot return (python would raise)
fn report(r: KResult<PyRef>) {
    if let Err(e) = r {
        crate::logging::exception_err("python pin", &e);
    }
}

impl PyPin {
    /// get_mcu(): the MCU the python object names (virtual_pins: the main
    /// mcu), else the main mcu
    fn mcu(&self) -> Rc<Mcu> {
        let named = self.0.call_method("get_mcu", &[]).and_then(|m| objects::unwrap(&m)).ok().and_then(|m| m.downcast::<Mcu>().ok());
        named.unwrap_or_else(|| {
            let printer = printer().expect("kaliru: no printer");
            crate::mcu::get_printer_mcu(&printer, "mcu").expect("kaliru: no main mcu")
        })
    }
}

impl DigitalOutPin for PyPin {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        report(self.0.call_method("setup_max_duration", &[&py_float(max_duration)]));
    }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        // (python's output_pin passes its values as floats)
        report(self.0.call_method("setup_start_value", &[&py_float(start_value as i64 as f64), &py_float(shutdown_value as i64 as f64)]));
    }
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        self.0.call_method("set_digital", &[&py_float(print_time), &py_float(value as i64 as f64)]).map(|_| ())
    }
}

impl PwmPin for PyPin {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        report(self.0.call_method("setup_max_duration", &[&py_float(max_duration)]));
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()> {
        self.0.call_method("setup_cycle_time", &[&py_float(cycle_time), &py_bool(hardware_pwm)]).map(|_| ())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        report(self.0.call_method("setup_start_value", &[&py_float(start_value), &py_float(shutdown_value)]));
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        self.0.call_method("set_pwm", &[&py_float(print_time), &py_float(value)]).map(|_| ())
    }
}

impl AdcPin for PyPin {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu()
    }
    fn setup_adc_sample(&self, sample_time: f64, sample_count: i64, minval: f64, maxval: f64, range_check_count: i64) {
        report(self.0.call_method(
            "setup_adc_sample",
            &[&py_float(sample_time), &py_int(sample_count), &py_float(minval), &py_float(maxval), &py_int(range_check_count)],
        ));
    }
    fn setup_adc_callback(&self, report_time: f64, callback: AdcCallback) {
        let f = py_function(Rc::new(move |a: &[PyRef]| {
            let t = a.first().map_or(Ok(0.0), |v| v.as_f64())?;
            let v = a.get(1).map_or(Ok(0.0), |v| v.as_f64())?;
            callback(t, v)?;
            Ok(py_none())
        }));
        report(f.and_then(|f| self.0.call_method("setup_adc_callback", &[&py_float(report_time), &f])));
    }
    fn get_last_value(&self) -> (f64, f64) {
        let r = self.0.call_method("get_last_value", &[]).and_then(|t| {
            let a = t.call_method("__getitem__", &[&py_int(0)])?.as_f64()?;
            let b = t.call_method("__getitem__", &[&py_int(1)])?.as_f64()?;
            Ok((a, b))
        });
        r.unwrap_or_else(|e| {
            crate::logging::exception_err("python pin", &e);
            (0.0, 0.0)
        })
    }
}

impl Endstop for PyPin {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu()
    }
    fn add_stepper(&self, stepper: &StepperRef) -> KResult<()> {
        self.0.call_method("add_stepper", &[&objects::wrap(stepper.clone())?]).map(|_| ())
    }
    fn get_steppers(&self) -> Vec<StepperRef> {
        let r = self.0.call_method("get_steppers", &[]).and_then(|l| {
            let n = l.call_method("__len__", &[])?.as_i64()?;
            let mut out = vec![];
            for i in 0..n {
                let s = objects::unwrap(&l.call_method("__getitem__", &[&py_int(i)])?)?;
                if let Ok(s) = s.downcast::<crate::stepper::McuStepper>() {
                    out.push(s);
                }
            }
            Ok(out)
        });
        r.unwrap_or_else(|e| {
            crate::logging::exception_err("python pin", &e);
            vec![]
        })
    }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) -> KResult<Rc<ReactorCompletion<bool>>> {
        let r = self.0.call_method(
            "home_start",
            &[&py_float(print_time), &py_float(sample_time), &py_int(sample_count), &py_float(rest_time), &py_bool(triggered)],
        )?;
        let pyc = objects::unwrap(&r)?
            .downcast::<ReactorCompletion<PyRef>>()
            .map_err(|_| KError::Py("TypeError".into(), "kaliru: home_start did not return a reactor completion".into()))?;
        // the native homing waits on a native completion of the python one
        let printer = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
        let reactor = printer.get_reactor();
        let done = reactor.completion::<bool>();
        // (already done, as virtual_pins' is: done now, as python's caller sees it)
        if pyc.test() {
            if let Ok(Some(v)) = pyc.wait(NEVER, None) {
                done.complete(v.is_true()?);
                return Ok(done);
            }
        }
        let d = done.clone();
        reactor.register_callback(
            move |_| {
                let v = match pyc.wait(NEVER, None) {
                    Ok(Some(v)) => v.is_true().unwrap_or(false),
                    _ => false,
                };
                d.complete(v);
            },
            NOW,
        );
        Ok(done)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.0.call_method("home_wait", &[&py_float(home_end_time)])?.as_f64()
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        let v = self.0.call_method("query_endstop", &[&py_float(print_time)])?;
        Ok(v.is_true()? as i64)
    }
}

/// A pin trait object as python sees it: a plugin's own pin, else kaliru's
fn dyn_pin_to_py<D: ?Sized + 'static>(any: Rc<dyn std::any::Any>, view: Rc<D>, name: &str) -> KResult<PyRef> {
    match any.downcast::<PyPin>() {
        Ok(p) => Ok(p.0.clone()),
        Err(any) => objects::wrap_view(any, view, name),
    }
}

pub fn pwm_pin_to_py(p: Rc<dyn PwmPin>) -> KResult<PyRef> {
    let name = p.type_name();
    dyn_pin_to_py(p.clone(), p, name)
}

pub fn digital_out_to_py(p: Rc<dyn DigitalOutPin>) -> KResult<PyRef> {
    let name = p.type_name();
    dyn_pin_to_py(p.clone(), p, name)
}

pub fn adc_pin_to_py(p: Rc<dyn AdcPin>) -> KResult<PyRef> {
    let name = p.type_name();
    dyn_pin_to_py(p.clone(), p, name)
}

/// An adc pin python hands kaliru: kaliru's own, else called through
pub fn adc_pin_from_py(o: &PyRef) -> KResult<Rc<dyn AdcPin>> {
    if is_mirror_instance(o)? {
        if let Some(p) = objects::unwrap_view::<dyn AdcPin>(o) {
            return Ok(p);
        }
    }
    Ok(Rc::new(PyPin(o.clone())))
}
