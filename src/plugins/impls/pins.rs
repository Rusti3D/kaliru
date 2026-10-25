// klippy/pins.py: PrinterPins; a python plugin's pin chip (register_chip)
use super::*;
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;

use crate::mcu::{McuAdc, McuDigitalOut, McuEndstop, McuPwm};
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
        pin_params_dict(&p)
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
        pin_params_dict(&p)
    });
    r.method("klippy.pins.PrinterPins.allow_multi_use_pin", |a| {
        a.this::<PinsObject>()?.0.borrow_mut().allow_multi_use_pin(&a.string(0)?).map_err(|e| KError::Pins(e.0))?;
        Ok(py_none())
    });
    // setup_pin(pin_type, pin_desc): the chip's pin object (AFC_canvas's
    // enable pins, its lanes' LED pins, AFC's espoolers and buffer adc)
    r.method("klippy.pins.PrinterPins.setup_pin", |a| {
        let printer = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
        let pin = crate::pins::setup_pin(&printer, &a.string(0)?, &a.string(1)?)?;
        // a python plugin's chip made a python object itself
        let pin = match pin.downcast::<PyRef>() {
            Ok(py) => return Ok((*py).clone()),
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
            Err(_) => Err(KError::Py(
                "NotImplementedError".into(),
                format!("kaliru: a {} pin of this chip is not mirrored yet", a.string(0)?),
            )),
        }
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

/// The pin_params dict python gets (a copy: kaliru keeps the shared one)
fn pin_params_dict(p: &PinParams) -> KResult<PyRef> {
    let mut d = crate::pydict! {
        "chip_name" => PyVal::Str(p.chip_name.clone()),
        "pin" => PyVal::Str(p.pin.clone()),
        "invert" => PyVal::Int(p.invert),
        "pullup" => PyVal::Int(p.pullup),
    };
    if let Some(s) = &p.share_type {
        d.insert("share_type".into(), PyVal::Str(s.clone()));
    }
    let dict = from_pyval(&PyVal::Dict(d));
    dict.call_method("__setitem__", &[&py_str("chip"), &chip_object(p)?])?;
    Ok(dict)
}

/// pin_params["chip"]: the plugin's own object, or kaliru's MCU
fn chip_object(p: &PinParams) -> KResult<PyRef> {
    if let Some(o) = p.chip.python_object() {
        if let Ok(py) = o.downcast::<PyRef>() {
            return Ok((*py).clone());
        }
    }
    let name = if p.chip_name == "mcu" { "mcu".to_string() } else { format!("mcu {}", p.chip_name) };
    match printer().and_then(|pr| pr.lookup_object_opt(&name)) {
        Some(o) => o.to_py(),
        None => Err(KError::Py(
            "NotImplementedError".into(),
            format!("kaliru: pin_params['chip'] of chip '{}' is not mirrored yet", p.chip_name),
        )),
    }
}

/// A python plugin's pin chip (pins.register_chip(name, obj))
struct PyPinChip {
    obj: PyRef,
}

impl PinChip for PyPinChip {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn std::any::Any>, String> {
        let params = pin_params_dict(&pin_params.borrow()).map_err(|e| e.to_string())?;
        let pin = self.obj.call_method("setup_pin", &[&py_str(pin_type), &params]).map_err(|e| e.to_string())?;
        Ok(Rc::new(pin))
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

