// klippy/extras/heaters.py: PrinterHeaters, Heater, DualSensorHeater and the
// controls. Plugins put their own heaters into PrinterHeaters.heaters and its
// name lists (AFC_ACE's dryer): kaliru keeps them (HeaterSlot::Plugin) and
// calls them by name. A sensor or control a plugin hands kaliru (a
// sensor_factory's result, set_control) is called through as the python
// object it is (PySensor, PyControl); kaliru's own travel as their Kalico
// classes, and a profile as the dict both sides share (prelude.py
// ProfileDict).
use super::*;
use crate::configfile::ConfigWrapper;
use crate::extras::heaters::{
    Control, ControlBangBang, ControlDualLoopPID, ControlPID, ControlRef, ControlVelocityPID, Heater, HeaterSlot,
    MpcSensor, PrinterHeaters, ProfVal, Profile, SensorFactory, TempCallback, TempSensor,
};
use crate::gcode::GCodeCommand;
use crate::plugins::callable::py_function;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::{Args, DictField, ListField};
use crate::plugins::py_to_pobj;
use crate::{cell_attr, function, method, ro_attr};
use std::cell::OnceCell;

type ProfileMap = RefCell<crate::IndexMap<String, ProfVal>>;

fn ph(o: &PyRef) -> KResult<Rc<PrinterHeaters>> {
    objects::unwrap_as::<PrinterHeaters>(o)
}

fn ht(o: &PyRef) -> KResult<Rc<Heater>> {
    objects::unwrap_as::<Heater>(o)
}

fn cfg(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

// ------------------------------------------------------------------ sensors

/// A plugin's temperature sensor (what its sensor_factory returned)
struct PySensor {
    obj: PyRef,
}

impl TempSensor for PySensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.obj.call_method("setup_minmax", &[&py_float(min_temp), &py_float(max_temp)])?;
        Ok(())
    }
    fn setup_callback(&self, cb: TempCallback) -> KResult<()> {
        let f = py_function(Rc::new(move |a: &[PyRef]| {
            cb(a[0].as_f64()?, a[1].as_f64()?)?;
            Ok(py_none())
        }))?;
        self.obj.call_method("setup_callback", &[&f])?;
        Ok(())
    }
    fn get_report_time_delta(&self) -> KResult<f64> {
        self.obj.call_method("get_report_time_delta", &[])?.as_f64()
    }
}

/// A sensor as python sees it
pub fn sensor_to_py(s: Rc<dyn TempSensor>) -> KResult<PyRef> {
    if let Some(p) = s.downcast_ref::<PySensor>() {
        return Ok(p.obj.clone());
    }
    let any: Rc<dyn Any> = s.clone();
    let name = s.type_name();
    objects::wrap_view(any, s, name)
}

/// A sensor python hands kaliru: kaliru's own, else called through
pub fn sensor_from_py(o: &PyRef) -> KResult<Rc<dyn TempSensor>> {
    if is_mirror_instance(o)? {
        if let Some(s) = objects::unwrap_view::<dyn TempSensor>(o) {
            return Ok(s);
        }
    }
    Ok(Rc::new(PySensor { obj: o.clone() }))
}

/// A python sensor_factory as kaliru's
fn factory_from_py(f: PyRef) -> SensorFactory {
    Rc::new(move |config: &ConfigWrapper| {
        let c = objects::wrap(Rc::new(config.clone()))?;
        sensor_from_py(&f.call(&[&c])?)
    })
}

/// A sensor factory as python sees it (kaliru's own: a callable)
fn factory_to_py(f: SensorFactory) -> KResult<PyRef> {
    py_function(Rc::new(move |a: &[PyRef]| sensor_to_py(f(&*cfg(&a[0])?)?)))
}

// ----------------------------------------------------------------- profiles

fn profval_to_py(v: &ProfVal) -> KResult<PyRef> {
    match v {
        ProfVal::V(v) => Ok(from_pyval(v)),
        ProfVal::Obj(Some(o)) => pobj_to_py(o),
        ProfVal::Obj(None) => Ok(py_none()),
    }
}

/// A profile value: a plain value, else an object (ambient sensor, fan)
fn profval_from_py(v: &PyRef) -> ProfVal {
    if objects::unwrap(v).is_ok() {
        return ProfVal::Obj(Some(py_to_pobj(v)));
    }
    match to_pyval(v) {
        Ok(pv) => ProfVal::V(pv),
        Err(_) => ProfVal::Obj(Some(py_to_pobj(v))),
    }
}

pub fn profile_to_py(p: &Profile) -> KResult<PyRef> {
    objects::wrap(p.clone())
}

/// A profile python hands kaliru: a ProfileDict is kaliru's own; a plain
/// dict (a plugin building a control) becomes a profile with its items
pub fn profile_from_py(o: &PyRef) -> KResult<Profile> {
    if let Ok(p) = objects::unwrap_as::<ProfileMap>(o) {
        return Ok(p);
    }
    let items: Vec<(String, PyRef)> = FromPy::from_py(&o.call_method("items", &[])?)?;
    let p = Profile::default();
    for (k, v) in items {
        p.borrow_mut().insert(k, profval_from_py(&v));
    }
    Ok(p)
}

fn pf(a: &Args) -> KResult<Rc<ProfileMap>> {
    objects::unwrap_as::<ProfileMap>(a.arg(0))
}

// ----------------------------------------------------------------- controls

/// A plugin's control (set_control with an object of its own class)
struct PyControl {
    obj: PyRef,
    kind: OnceCell<&'static str>,
}

impl Control for PyControl {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64, secondary_temp: Option<f64>) -> KResult<()> {
        let mut args = vec![py_float(read_time), py_float(temp), py_float(target_temp)];
        if let Some(s) = secondary_temp {
            args.push(py_float(s));
        }
        self.obj.call_method("temperature_update", &args.iter().collect::<Vec<_>>())?;
        Ok(())
    }
    fn check_busy(&self, eventtime: f64, smoothed_temp: f64, target_temp: f64) -> KResult<bool> {
        self.obj.call_method("check_busy", &[&py_float(eventtime), &py_float(smoothed_temp), &py_float(target_temp)])?.is_true()
    }
    fn update_smooth_time(&self) -> KResult<()> {
        self.obj.call_method("update_smooth_time", &[])?;
        Ok(())
    }
    // ponytail: get_profile / get_type cannot fail in kaliru's Control; a
    // plugin control raising there is reported as an unhandled error
    fn get_profile(&self) -> Profile {
        match self.obj.call_method("get_profile", &[]).and_then(|p| profile_from_py(&p)) {
            Ok(p) => p,
            Err(e) => {
                report_unhandled(&e);
                Profile::default()
            }
        }
    }
    fn get_type(&self) -> &'static str {
        self.kind.get_or_init(|| match self.obj.call_method("get_type", &[]).and_then(|t| t.as_string()) {
            Ok(t) => match t.as_str() {
                "watermark" => "watermark",
                "pid" => "pid",
                "pid_v" => "pid_v",
                "mpc" => "mpc",
                "dual_loop_pid" => "dual_loop_pid",
                _ => &*Box::leak(t.into_boxed_str()),
            },
            Err(e) => {
                report_unhandled(&e);
                ""
            }
        })
    }
    // hasattr(self.control, "get_status") / "check_valid" in Kalico
    fn get_status(&self, eventtime: f64) -> Option<PyVal> {
        if !self.obj.hasattr("get_status").unwrap_or(false) {
            return None;
        }
        match self.obj.call_method("get_status", &[&py_float(eventtime)]).and_then(|s| to_pyval(&s)) {
            Ok(PyVal::None) => None,
            Ok(v) => Some(v),
            Err(e) => {
                report_unhandled(&e);
                None
            }
        }
    }
    fn check_valid(&self) -> KResult<()> {
        if self.obj.hasattr("check_valid")? {
            self.obj.call_method("check_valid", &[])?;
        }
        Ok(())
    }
}

/// A control as python sees it
pub fn control_to_py(c: ControlRef) -> KResult<PyRef> {
    if let Some(p) = c.downcast_ref::<PyControl>() {
        return Ok(p.obj.clone());
    }
    let any: Rc<dyn Any> = c.clone();
    let name = c.type_name();
    objects::wrap_view(any, c, name)
}

/// A control python hands kaliru: kaliru's own, else called through
pub fn control_from_py(o: &PyRef) -> KResult<ControlRef> {
    if is_mirror_instance(o)? {
        if let Some(c) = objects::unwrap_view::<dyn Control>(o) {
            return Ok(c);
        }
    }
    Ok(Rc::new(PyControl { obj: o.clone(), kind: OnceCell::new() }))
}

/// The kaliru control behind a mirrored control object
fn ctl(o: &PyRef) -> KResult<ControlRef> {
    objects::unwrap_view::<dyn Control>(o).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: expected a heater control".into()))
}

// args (profile, heater, load_clean) of the control constructors
fn ctl_args(a: &Args) -> KResult<(Profile, Rc<Heater>, bool)> {
    Ok((profile_from_py(a.arg(0))?, ht(a.arg(1))?, a.bool(2)?))
}

/// registered only where the Kalico class has the name
fn known_method(r: &mut Registry, name: &'static str, f: crate::plugins::native::Method) {
    if crate::plugins::native::known(name) {
        r.method(name, f);
    }
}

/// A control's common methods: temperature_update, check_busy,
/// update_smooth_time, get_profile, get_type
macro_rules! control_methods {
    ($r:expr, $class:literal) => {
        known_method($r, concat!($class, ".temperature_update"), |a| {
            let secondary = a.args.get(3).map(|s| s.as_f64()).transpose()?;
            ctl(a.slf.as_ref().expect("method"))?.temperature_update(a.f64(0)?, a.f64(1)?, a.f64(2)?, secondary)?;
            Ok(py_none())
        });
        known_method($r, concat!($class, ".check_busy"), |a| {
            ctl(a.slf.as_ref().expect("method"))?.check_busy(a.f64(0)?, a.f64(1)?, a.f64(2)?).to_py()
        });
        known_method($r, concat!($class, ".update_smooth_time"), |a| ctl(a.slf.as_ref().expect("method"))?.update_smooth_time().to_py());
        known_method($r, concat!($class, ".get_profile"), |a| profile_to_py(&ctl(a.slf.as_ref().expect("method"))?.get_profile()));
        known_method($r, concat!($class, ".get_type"), |a| Ok(py_str(ctl(a.slf.as_ref().expect("method"))?.get_type())));
    };
}

/// The attributes every control class has: profile, heater, heater_max_power
macro_rules! control_attrs {
    ($r:expr, $class:literal, $ty:ty) => {
        $r.attr(concat!($class, ".profile"), |o| profile_to_py(&objects::unwrap_as::<$ty>(o)?.profile), None);
        $r.attr(concat!($class, ".heater"), |o| objects::unwrap_as::<$ty>(o)?.heater.clone().to_py(), None);
        $r.attr(
            concat!($class, ".heater_max_power"),
            |o| Ok(py_float(objects::unwrap_as::<$ty>(o)?.heater_max_power.get())),
            Some(|o, v| {
                objects::unwrap_as::<$ty>(o)?.heater_max_power.set(v.as_f64()?);
                Ok(())
            }),
        );
    };
}

// -------------------------------------------------------------- mpc sensors

/// A plugin's mpc sensor (heater.add_mpc_sensor(self))
struct PyMpcSensor {
    obj: PyRef,
}

impl MpcSensor for PyMpcSensor {
    fn process_temp_update(&self, control: &ControlRef, read_time: f64) -> KResult<()> {
        self.obj.call_method("process_temp_update", &[&control_to_py(control.clone())?, &py_float(read_time)])?;
        Ok(())
    }
}

fn mpc_to_py(s: Rc<dyn MpcSensor>) -> KResult<PyRef> {
    if let Some(p) = (s.as_ref() as &dyn Any).downcast_ref::<PyMpcSensor>() {
        return Ok(p.obj.clone());
    }
    let any: Rc<dyn Any> = s.clone();
    let name = s.type_name();
    objects::wrap_view(any, s, name)
}

fn mpc_from_py(o: &PyRef) -> KResult<Rc<dyn MpcSensor>> {
    if is_mirror_instance(o)? {
        if let Some(s) = objects::unwrap_view::<dyn MpcSensor>(o) {
            return Ok(s);
        }
    }
    Ok(Rc::new(PyMpcSensor { obj: o.clone() }))
}

// ------------------------------------------------------------------ heaters

/// The heater a python object is: kaliru's own or a plugin's
fn heater_check_busy(o: &PyRef, eventtime: f64) -> KResult<bool> {
    match ht(o) {
        Ok(h) => h.check_busy(eventtime),
        Err(_) => o.call_method("check_busy", &[&py_float(eventtime)])?.is_true(),
    }
}

pub fn register(r: &mut Registry) {
    // temperature_fan controls: kaliru's control holds no fan, python's
    // control.temperature_fan is the fan whose control it is
    {
        use crate::extras::temperature_fan::{TemperatureFan, TfControl};
        fn control_to_py(c: Rc<dyn TfControl>) -> KResult<PyRef> {
            let name = c.type_name();
            objects::wrap_view(c.clone(), c, name)
        }
        fn control(a: &PyRef) -> KResult<Rc<dyn TfControl>> {
            objects::unwrap_view::<dyn TfControl>(a).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: not a temperature fan control".into()))
        }
        fn owner(c: &Rc<dyn TfControl>) -> KResult<Rc<TemperatureFan>> {
            let p = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
            for (_, o) in p.lookup_objects(Some("temperature_fan")) {
                if let Some(tf) = crate::printer::downcast_rc::<TemperatureFan>(o) {
                    if tf.control.borrow().as_ref().is_some_and(|x| Rc::ptr_eq(x, c)) {
                        return Ok(tf);
                    }
                }
            }
            Err(KError::Py("AttributeError".into(), "temperature_fan".into()))
        }
        r.attr("klippy.extras.temperature_fan.TemperatureFan.control", |o| {
            match objects::unwrap_as::<TemperatureFan>(o)?.control.borrow().clone() {
                Some(c) => control_to_py(c),
                None => Err(KError::Py("AttributeError".into(), "'TemperatureFan' object has no attribute 'control'".into())),
            }
        }, None);
        macro_rules! tf_control {
            ($r:ident, $class:literal, $ty:ty) => {
                objects::register_view::<$ty>(|a| -> Box<dyn Any> { Box::new(a.downcast::<$ty>().expect("type") as Rc<dyn TfControl>) });
                $r.method(concat!($class, ".temperature_callback"), |a| {
                    let c = control(a.slf.as_ref().expect("method"))?;
                    c.temperature_callback(&*owner(&c)?, a.f64(0)?, a.f64(1)?)?;
                    Ok(py_none())
                });
                $r.method(concat!($class, ".get_type"), |a| Ok(py_str(control(a.slf.as_ref().expect("method"))?.get_type())));
                $r.attr(concat!($class, ".temperature_fan"), |o| objects::wrap(owner(&control(o)?)?), None);
            };
        }
        tf_control!(r, "klippy.extras.temperature_fan.ControlBangBang", crate::extras::temperature_fan::ControlBangBang);
        tf_control!(r, "klippy.extras.temperature_fan.ControlPID", crate::extras::temperature_fan::ControlPID);
        tf_control!(r, "klippy.extras.temperature_fan.ControlCurve", crate::extras::temperature_fan::ControlCurve);
        r.attr("klippy.extras.temperature_fan.ControlPID.Kp", |o| Ok(py_float(objects::unwrap_as::<crate::extras::temperature_fan::ControlPID>(o)?.kp)), None);
        r.attr("klippy.extras.temperature_fan.ControlPID.Ki", |o| Ok(py_float(objects::unwrap_as::<crate::extras::temperature_fan::ControlPID>(o)?.ki)), None);
        r.attr("klippy.extras.temperature_fan.ControlPID.Kd", |o| Ok(py_float(objects::unwrap_as::<crate::extras::temperature_fan::ControlPID>(o)?.kd)), None);
    }
    // temperature_fan.TemperatureFan: its sensor and printer
    r.attr("klippy.extras.temperature_fan.TemperatureFan.sensor", |o| {
        sensor_to_py(objects::unwrap_as::<crate::extras::temperature_fan::TemperatureFan>(o)?.sensor.clone())
    }, None);
    r.attr("klippy.extras.temperature_fan.TemperatureFan.printer", |o| {
        objects::unwrap_as::<crate::extras::temperature_fan::TemperatureFan>(o)?;
        printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.to_py()
    }, None);
    objects::register_pobj::<Heater>();
    objects::register_pobj::<crate::mcu::Mcu>();
    objects::register_class::<ProfileMap>("_k", "ProfileDict");
    // a heater with an inner sensor is Kalico's DualSensorHeater, a dual
    // loop's inner pid its ControlInnerPID
    objects::register_class_fn::<Heater>(|a| match a.downcast_ref::<Heater>() {
        Some(h) if h.secondary_sensor.is_some() => ("klippy.extras.heaters", "DualSensorHeater"),
        _ => ("klippy.extras.heaters", "Heater"),
    });
    objects::register_class_fn::<ControlPID>(|a| match a.downcast_ref::<ControlPID>() {
        Some(c) if c.kind == "inner" => ("klippy.extras.heaters", "ControlInnerPID"),
        _ => ("klippy.extras.heaters", "ControlPID"),
    });
    objects::register_view::<ControlBangBang>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ControlBangBang>().expect("type") as ControlRef) });
    objects::register_view::<ControlPID>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ControlPID>().expect("type") as ControlRef) });
    objects::register_view::<ControlVelocityPID>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ControlVelocityPID>().expect("type") as ControlRef) });
    objects::register_view::<ControlDualLoopPID>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ControlDualLoopPID>().expect("type") as ControlRef) });

    // ProfileDict
    r.ext("profile_get", |a| {
        let key = a.string(1)?;
        match pf(a)?.borrow().get(&key) {
            Some(v) => profval_to_py(v),
            None => Err(KError::Py("KeyError".into(), py_str(&key).repr()?)),
        }
    });
    r.ext("profile_set", |a| {
        let v = profval_from_py(a.arg(2));
        pf(a)?.borrow_mut().insert(a.string(1)?, v);
        Ok(py_none())
    });
    r.ext("profile_del", |a| {
        let key = a.string(1)?;
        match pf(a)?.borrow_mut().shift_remove(&key) {
            Some(_) => Ok(py_none()),
            None => Err(KError::Py("KeyError".into(), py_str(&key).repr()?)),
        }
    });
    r.ext("profile_keys", |a| Ok(py_list(pf(a)?.borrow().keys().map(|k| py_str(k)).collect())));

    function!(r, "klippy.extras.heaters", lookup_dual_loop_pid_inner_target_temp(config: PyRef) =>
        crate::extras::heaters::lookup_dual_loop_pid_inner_target_temp(&*cfg(&config)?));
    r.method("klippy.extras.heaters.load_config", |a| PrinterHeaters::new(&*cfg(a.arg(0))?)?.to_py());

    // ------------------------------------------------------------ Heater
    r.ctor("klippy.extras.heaters.Heater.__init__", |a| {
        let h: Rc<dyn Any> = Heater::new(&*cfg(a.arg(0))?, sensor_from_py(a.arg(1))?, None)?;
        Ok(h)
    });
    r.ctor("klippy.extras.heaters.DualSensorHeater.__init__", |a| {
        let secondary = if a.arg(2).is_none() { None } else { Some(py_to_pobj(a.arg(2))) };
        let has_secondary = secondary.is_some();
        let h = Heater::new(&*cfg(a.arg(0))?, sensor_from_py(a.arg(1))?, secondary)?;
        if !has_secondary && h.get_control().is::<ControlDualLoopPID>() {
            return Err(KError::config("dual_loop_pid requires a secondary sensor"));
        }
        let h: Rc<dyn Any> = h;
        Ok(h)
    });
    r.method("klippy.extras.heaters.Heater.lookup_control", |a| {
        let h = a.this::<Heater>()?;
        control_to_py(h.lookup_control(&profile_from_py(a.arg(0))?, a.bool(1)?)?)
    });
    method!(r, "klippy.extras.heaters.Heater", Heater, set_pwm(h, read_time: f64, value: f64) => h.set_pwm(read_time, value));
    method!(r, "klippy.extras.heaters.Heater", Heater, temperature_callback(h, read_time: f64, temp: f64) =>
        h.temperature_callback(read_time, temp));
    method!(r, "klippy.extras.heaters.DualSensorHeater", Heater, temperature_callback(h, read_time: f64, primary_temp: f64) =>
        h.temperature_callback(read_time, primary_temp));
    method!(r, "klippy.extras.heaters.Heater", Heater, _handle_shutdown(h) => h.verify_mainthread_time.set(-999.0));
    method!(r, "klippy.extras.heaters.Heater", Heater, set_inv_smooth_time(h, v: f64) => h.set_inv_smooth_time(v));
    r.method("klippy.extras.heaters.Heater.set_control", |a| {
        let h = a.this::<Heater>()?;
        control_to_py(h.set_control(control_from_py(a.arg(0))?, a.bool(1)?))
    });
    r.method("klippy.extras.heaters.Heater.get_control", |a| control_to_py(a.this::<Heater>()?.get_control()));
    method!(r, "klippy.extras.heaters.Heater", Heater, is_adc_faulty(h) => h.is_adc_faulty());
    r.method("klippy.extras.heaters.Heater.set_cold_extrude", |a| {
        // (cold_extrude: Kalico passes gcmd.get_int's int or None)
        let cold = if a.arg(0).is_none() { None } else { Some(a.arg(0).as_i64()?) };
        a.this::<Heater>()?.set_cold_extrude(cold, a.opt_f64(1)?)?;
        Ok(py_none())
    });
    method!(r, "klippy.extras.heaters.Heater", Heater, cmd_SET_HEATER_TEMPERATURE(h, g: PyRef) => h.cmd_set_heater_temperature(&*gcmd(&g)?));
    method!(r, "klippy.extras.heaters.Heater", Heater, cmd_COLD_EXTRUDE(h, g: PyRef) => h.cmd_cold_extrude(&*gcmd(&g)?));
    method!(r, "klippy.extras.heaters.Heater", Heater, cmd_SET_SMOOTH_TIME(h, g: PyRef) => h.cmd_set_smooth_time(&*gcmd(&g)?));
    method!(r, "klippy.extras.heaters.Heater", Heater, cmd_SET_HEATER_PID(h, g: PyRef) => h.cmd_set_heater_pid(&*gcmd(&g)?));
    r.method("klippy.extras.heaters.Heater.add_mpc_sensor", |a| {
        a.this::<Heater>()?.add_mpc_sensor(mpc_from_py(a.arg(0))?);
        Ok(py_none())
    });
    // attributes
    cell_attr!(r, "klippy.extras.heaters.Heater.cold_extrude", Heater, cold_extrude);
    cell_attr!(r, "klippy.extras.heaters.Heater.config_smooth_time", Heater, config_smooth_time);
    cell_attr!(r, "klippy.extras.heaters.Heater.inv_smooth_time", Heater, inv_smooth_time);
    cell_attr!(r, "klippy.extras.heaters.Heater.last_pwm_value", Heater, last_pwm_value);
    cell_attr!(r, "klippy.extras.heaters.Heater.last_temp_time", Heater, last_temp_time);
    cell_attr!(r, "klippy.extras.heaters.Heater.lost_update_tolerance", Heater, lost_update_tolerance);
    cell_attr!(r, "klippy.extras.heaters.Heater.max_power", Heater, max_power);
    cell_attr!(r, "klippy.extras.heaters.Heater.min_pwm_change", Heater, min_pwm_change);
    cell_attr!(r, "klippy.extras.heaters.Heater.next_pwm_time", Heater, next_pwm_time);
    cell_attr!(r, "klippy.extras.heaters.Heater.pwm_delay", Heater, pwm_delay);
    cell_attr!(r, "klippy.extras.heaters.Heater.smooth_time", Heater, smooth_time);
    cell_attr!(r, "klippy.extras.heaters.Heater.verify_mainthread_time", Heater, verify_mainthread_time);
    ro_attr!(r, "klippy.extras.heaters.Heater.config", Heater, |h| Rc::new(h.config.clone()));
    ro_attr!(r, "klippy.extras.heaters.Heater.configfile", Heater, |h| h.configfile.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.printer", Heater, |h| h.printer.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.reactor", Heater, |h| h.reactor.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.gcode", Heater, |h| h.printer.lookup::<crate::gcode::GCodeDispatch>("gcode"));
    r.attr("klippy.extras.heaters.Heater.sensor", |o| sensor_to_py(ht(o)?.sensor()), None);
    r.attr("klippy.extras.heaters.Heater.mcu_pwm", |o| {
        let p = ht(o)?.mcu_pwm();
        let any: Rc<dyn Any> = p.clone();
        let name = p.type_name();
        objects::wrap_view(any, p, name)
    }, None);
    r.attr("klippy.extras.heaters.Heater.control", |o| control_to_py(ht(o)?.get_control()), Some(|o, v| {
        ht(o)?.set_control(control_from_py(v)?, true);
        Ok(())
    }));
    // self.lock = threading.Lock(): a python lock (kaliru's callbacks run on
    // the reactor thread; a plugin's `with heater.lock` still works)
    r.attr("klippy.extras.heaters.Heater.lock", |o| {
        let d = o.getattr("__dict__")?;
        let lock = lookup("threading", "Lock")?.call(&[])?;
        d.call_method("setdefault", &[&py_str("_k_lock"), &lock])
    }, None);
    r.list("klippy.extras.heaters.Heater.mpc_sensors", ListField {
        get: |o, _f| ht(o)?.mpc_sensors.borrow().clone().into_iter().map(mpc_to_py).collect(),
        set: |o, _f, items| {
            let v = items.iter().map(mpc_from_py).collect::<KResult<Vec<_>>>()?;
            *ht(o)?.mpc_sensors.borrow_mut() = v;
            Ok(())
        },
    });
    // self.pmgr = ProfileManager(self): Kalico's own class on kaliru's state
    r.attr("klippy.extras.heaters.Heater.pmgr", |o| {
        let d = o.getattr("__dict__")?;
        if let Ok(p) = d.call_method("__getitem__", &[&py_str("_k_pmgr")]) {
            return Ok(p);
        }
        let cls = lookup("klippy.extras.heaters", "Heater")?.getattr("ProfileManager")?;
        let pm = cls.getattr("__new__")?.call(&[&cls])?;
        pm.setattr("outer_instance", o)?;
        pm.setattr("profiles", &lookup("_k", "DictView")?.call(&[o, &py_str("_k.Heater.profiles")])?)?;
        pm.setattr("incompatible_profiles", &lookup("_k", "ListView")?.call(&[o, &py_str("_k.Heater.incompatible_profiles")])?)?;
        d.call_method("__setitem__", &[&py_str("_k_pmgr"), &pm])?;
        Ok(pm)
    }, None);
    r.dict(
        "_k.Heater.profiles",
        DictField {
            get: |o, _f, key| match ht(o)?.profiles.borrow().get(&key.as_string()?) {
                Some(p) => profile_to_py(p),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                let p = profile_from_py(value)?;
                ht(o)?.profiles.borrow_mut().insert(key.as_string()?, p);
                Ok(())
            },
            del: |o, _f, key| match ht(o)?.profiles.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(ht(o)?.profiles.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
    r.list("_k.Heater.incompatible_profiles", ListField {
        get: |o, _f| Ok(ht(o)?.incompatible_profiles.borrow().iter().map(|s| py_str(s)).collect()),
        set: |o, _f, items| {
            let v = items.iter().map(|i| i.as_string()).collect::<KResult<Vec<_>>>()?;
            *ht(o)?.incompatible_profiles.borrow_mut() = v;
            Ok(())
        },
    });
    // DualSensorHeater
    r.attr("klippy.extras.heaters.DualSensorHeater.secondary_sensor", |o| match &ht(o)?.secondary_sensor {
        Some(s) => pobj_to_py(s),
        None => Ok(py_none()),
    }, None);
    cell_attr!(r, "klippy.extras.heaters.DualSensorHeater.can_extrude", Heater, can_extrude);
    ro_attr!(r, "klippy.extras.heaters.DualSensorHeater.last_temp", Heater, |h| h.last_temp.get());
    cell_attr!(r, "klippy.extras.heaters.DualSensorHeater.last_temp_time", Heater, last_temp_time);
    ro_attr!(r, "klippy.extras.heaters.DualSensorHeater.smoothed_temp", Heater, |h| h.smoothed_temp.get());

    // ----------------------------------------------------------- controls
    r.ctor("klippy.extras.heaters.ControlBangBang.__init__", |a| {
        let (profile, heater, _) = ctl_args(a)?;
        let c: Rc<dyn Any> = Rc::new(ControlBangBang::new(&profile, &heater)?);
        Ok(c)
    });
    r.ctor("klippy.extras.heaters.ControlPID.__init__", |a| {
        let (profile, heater, load_clean) = ctl_args(a)?;
        let c: Rc<dyn Any> = Rc::new(ControlPID::new(&profile, &heater, load_clean)?);
        Ok(c)
    });
    r.ctor("klippy.extras.heaters.ControlInnerPID.__init__", |a| {
        let (profile, heater, load_clean) = ctl_args(a)?;
        let c: Rc<dyn Any> = Rc::new(ControlPID::new_inner(&profile, &heater, load_clean)?);
        Ok(c)
    });
    r.ctor("klippy.extras.heaters.ControlVelocityPID.__init__", |a| {
        let (profile, heater, load_clean) = ctl_args(a)?;
        let c: Rc<dyn Any> = Rc::new(ControlVelocityPID::new(&profile, &heater, load_clean)?);
        Ok(c)
    });
    r.ctor("klippy.extras.heaters.ControlDualLoopPID.__init__", |a| {
        let (profile, heater, load_clean) = ctl_args(a)?;
        let c: Rc<dyn Any> = Rc::new(ControlDualLoopPID::new(&profile, &heater, load_clean)?);
        Ok(c)
    });
    control_methods!(r, "klippy.extras.heaters.ControlBangBang");
    control_methods!(r, "klippy.extras.heaters.ControlPID");
    control_methods!(r, "klippy.extras.heaters.ControlVelocityPID");
    control_methods!(r, "klippy.extras.heaters.ControlDualLoopPID");
    // the MPC, its tuning control and PID_CALIBRATE's control
    {
        use crate::extras::control_mpc::{ControlMPC, TuningControl};
        use crate::extras::pid_calibrate::ControlAutoTune;
        objects::register_view::<ControlMPC>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ControlMPC>().expect("type") as ControlRef) });
        objects::register_view::<TuningControl>(|a| -> Box<dyn Any> { Box::new(a.downcast::<TuningControl>().expect("type") as ControlRef) });
        objects::register_view::<ControlAutoTune>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ControlAutoTune>().expect("type") as ControlRef) });
        control_methods!(r, "klippy.extras.control_mpc.ControlMPC");
        control_methods!(r, "klippy.extras.control_mpc.TuningControl");
        control_methods!(r, "klippy.extras.pid_calibrate.ControlAutoTune");
        r.method("klippy.extras.control_mpc.ControlMPC.check_valid", |a| {
            ctl(a.slf.as_ref().expect("method"))?.check_valid()?;
            Ok(py_none())
        });
        r.attr("klippy.extras.control_mpc.ControlMPC.profile", |o| profile_to_py(&objects::unwrap_as::<ControlMPC>(o)?.profile), None);
    }
    control_attrs!(r, "klippy.extras.heaters.ControlBangBang", ControlBangBang);
    control_attrs!(r, "klippy.extras.heaters.ControlPID", ControlPID);
    control_attrs!(r, "klippy.extras.heaters.ControlVelocityPID", ControlVelocityPID);
    control_attrs!(r, "klippy.extras.heaters.ControlDualLoopPID", ControlDualLoopPID);
    // ControlBangBang
    cell_attr!(r, "klippy.extras.heaters.ControlBangBang.heating", ControlBangBang, heating);
    cell_attr!(r, "klippy.extras.heaters.ControlBangBang.max_delta", ControlBangBang, max_delta);
    r.attr("klippy.extras.heaters.ControlBangBang.smooth_time", |o| {
        match objects::unwrap_as::<ControlBangBang>(o)?.smooth_time.get() {
            Some(t) => Ok(py_float(t)),
            None => Err(KError::Py("AttributeError".into(), "'ControlBangBang' object has no attribute 'smooth_time'".into())),
        }
    }, Some(|o, v| {
        objects::unwrap_as::<ControlBangBang>(o)?.smooth_time.set(Some(v.as_f64()?));
        Ok(())
    }));
    // ControlPID (and ControlInnerPID: the same Rust type)
    method!(r, "klippy.extras.heaters.ControlPID", ControlPID, calculate_output(c, read_time: f64, temp: f64, target_temp: f64) =>
        c.calculate_output(read_time, temp, target_temp));
    cell_attr!(r, "klippy.extras.heaters.ControlPID.Kp", ControlPID, kp);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.Ki", ControlPID, ki);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.Kd", ControlPID, kd);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.min_deriv_time", ControlPID, min_deriv_time);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.temp_integ_max", ControlPID, temp_integ_max);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.prev_temp", ControlPID, prev_temp);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.prev_temp_time", ControlPID, prev_temp_time);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.prev_temp_deriv", ControlPID, prev_temp_deriv);
    cell_attr!(r, "klippy.extras.heaters.ControlPID.prev_temp_integ", ControlPID, prev_temp_integ);
    r.attr("klippy.extras.heaters.ControlPID.smooth_time", |o| {
        match objects::unwrap_as::<ControlPID>(o)?.smooth_time.get() {
            Some(t) => Ok(py_float(t)),
            None => Err(KError::Py("AttributeError".into(), "'ControlPID' object has no attribute 'smooth_time'".into())),
        }
    }, Some(|o, v| {
        objects::unwrap_as::<ControlPID>(o)?.smooth_time.set(Some(v.as_f64()?));
        Ok(())
    }));
    cell_attr!(r, "klippy.extras.heaters.ControlInnerPID.Kp", ControlPID, kp);
    cell_attr!(r, "klippy.extras.heaters.ControlInnerPID.Ki", ControlPID, ki);
    cell_attr!(r, "klippy.extras.heaters.ControlInnerPID.Kd", ControlPID, kd);
    cell_attr!(r, "klippy.extras.heaters.ControlInnerPID.temp_integ_max", ControlPID, temp_integ_max);
    // ControlVelocityPID
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.Kp", ControlVelocityPID, kp);
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.Ki", ControlVelocityPID, ki);
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.Kd", ControlVelocityPID, kd);
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.smooth_time", ControlVelocityPID, smooth_time);
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.d1", ControlVelocityPID, d1);
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.d2", ControlVelocityPID, d2);
    cell_attr!(r, "klippy.extras.heaters.ControlVelocityPID.pwm", ControlVelocityPID, pwm);
    crate::vec_attr!(r, "klippy.extras.heaters.ControlVelocityPID.temps", ControlVelocityPID, temps);
    crate::vec_attr!(r, "klippy.extras.heaters.ControlVelocityPID.times", ControlVelocityPID, times);
    // ControlDualLoopPID
    ro_attr!(r, "klippy.extras.heaters.ControlDualLoopPID.primary_pid", ControlDualLoopPID, |c| c.primary_pid.clone());
    ro_attr!(r, "klippy.extras.heaters.ControlDualLoopPID.secondary_pid", ControlDualLoopPID, |c| c.secondary_pid.clone());
    cell_attr!(r, "klippy.extras.heaters.ControlDualLoopPID.inner_target_temp", ControlDualLoopPID, inner_target_temp);
    r.attr("klippy.extras.heaters.ControlDualLoopPID.smooth_time", |o| {
        match objects::unwrap_as::<ControlDualLoopPID>(o)?.smooth_time.get() {
            Some(t) => Ok(py_float(t)),
            None => Err(KError::Py("AttributeError".into(), "'ControlDualLoopPID' object has no attribute 'smooth_time'".into())),
        }
    }, Some(|o, v| {
        objects::unwrap_as::<ControlDualLoopPID>(o)?.smooth_time.set(Some(v.as_f64()?));
        Ok(())
    }));

    // ---------------------------------------------------- PrinterHeaters
    r.ctor("klippy.extras.heaters.PrinterHeaters.__init__", |a| {
        let p: Rc<dyn Any> = PrinterHeaters::new(&*cfg(a.arg(0))?)?;
        Ok(p)
    });
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, load_config(p, config: PyRef) => p.load_config(&*cfg(&config)?));
    r.method("klippy.extras.heaters.PrinterHeaters.add_sensor_factory", |a| {
        a.this::<PrinterHeaters>()?.add_sensor_factory(&a.string(0)?, factory_from_py(a.arg(1).clone()));
        Ok(py_none())
    });
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, setup_heater(p, config: PyRef, gcode_id: Option<String>) =>
        p.setup_heater(&*cfg(&config)?, gcode_id.as_deref()));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, lookup_heater(p, name: String) => p.lookup_heater_obj(&name));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, get_all_heaters(p) => p.get_all_heaters());
    r.method("klippy.extras.heaters.PrinterHeaters.setup_sensor", |a| sensor_to_py(a.this::<PrinterHeaters>()?.setup_sensor(&*cfg(a.arg(0))?)?));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, register_sensor(p, config: PyRef, psensor: PObj, gcode_id: Option<String>) =>
        p.register_sensor(&*cfg(&config)?, psensor, gcode_id.as_deref()));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, register_monitor(p, config: PyRef) => p.register_monitor(&*cfg(&config)?));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, turn_off_all_heaters(p) => p.turn_off_all_heaters());
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, cmd_TURN_OFF_HEATERS(p, _g: PyRef) => p.turn_off_all_heaters());
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, _handle_ready(p) => p.has_started.set(true));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, _get_temp(p, eventtime: f64) => p.get_temp_msg(eventtime));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, cmd_M105(p, g: PyRef) => p.cmd_m105(&*gcmd(&g)?));
    method!(r, "klippy.extras.heaters.PrinterHeaters", PrinterHeaters, cmd_TEMPERATURE_WAIT(p, g: PyRef) => p.cmd_temperature_wait(&*gcmd(&g)?));
    r.method("klippy.extras.heaters.PrinterHeaters._wait_for_temperature", |a| {
        let heater = a.arg(0).clone();
        a.this::<PrinterHeaters>()?.wait_for_temperature_with(&|eventtime| heater_check_busy(&heater, eventtime))?;
        Ok(py_none())
    });
    // set_temperature(heater, temp, wait=False)
    r.method("klippy.extras.heaters.PrinterHeaters.set_temperature", |a| {
        let p = a.this::<PrinterHeaters>()?;
        let (temp, wait) = (a.f64(1)?, a.bool(2)?);
        if let Ok(h) = ht(a.arg(0)) {
            p.set_temperature(&h, temp, wait)?;
            return Ok(py_none());
        }
        // a python plugin's heater: Kalico's steps, on the object
        let toolhead = p.printer.lookup::<crate::toolhead::ToolHead>("toolhead")?;
        toolhead.register_lookahead_callback(Rc::new(|_| Ok(())))?;
        a.arg(0).call_method("set_temp", &[&py_float(temp)])?;
        if wait && temp != 0.0 {
            let heater = a.arg(0).clone();
            p.wait_for_temperature_with(&|eventtime| heater_check_busy(&heater, eventtime))?;
        }
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.heaters.PrinterHeaters.printer", PrinterHeaters, |p| p.printer.clone());
    ro_attr!(r, "klippy.extras.heaters.PrinterHeaters.gcode", PrinterHeaters, |p| p.gcode.clone());
    cell_attr!(r, "klippy.extras.heaters.PrinterHeaters.has_started", PrinterHeaters, has_started);
    cell_attr!(r, "klippy.extras.heaters.PrinterHeaters.have_load_sensors", PrinterHeaters, have_load_sensors);
    r.dict(
        "klippy.extras.heaters.PrinterHeaters.sensor_factories",
        DictField {
            get: |o, _f, key| match ph(o)?.sensor_factories.borrow().get(&key.as_string()?) {
                Some(f) => factory_to_py(f.clone()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                ph(o)?.add_sensor_factory(&key.as_string()?, factory_from_py(value.clone()));
                Ok(())
            },
            del: |o, _f, key| match ph(o)?.sensor_factories.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(ph(o)?.sensor_factories.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
    r.dict(
        "klippy.extras.heaters.PrinterHeaters.gcode_id_to_sensor",
        DictField {
            get: |o, _f, key| match ph(o)?.gcode_id_to_sensor.borrow().get(&key.as_string()?) {
                Some(s) => pobj_to_py(s),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                ph(o)?.gcode_id_to_sensor.borrow_mut().insert(key.as_string()?, py_to_pobj(value));
                Ok(())
            },
            del: |o, _f, key| match ph(o)?.gcode_id_to_sensor.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(ph(o)?.gcode_id_to_sensor.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
    let names = ListField {
        get: |o, field| {
            let p = ph(o)?;
            let which = field.rsplit('.').next().unwrap_or("");
            let list = p.name_list(which).ok_or_else(|| KError::Other(format!("kaliru: no list {}", field)))?;
            let out = list.borrow().iter().map(|s| py_str(s)).collect();
            Ok(out)
        },
        set: |o, field, items| {
            let p = ph(o)?;
            let which = field.rsplit('.').next().unwrap_or("");
            let list = p.name_list(which).ok_or_else(|| KError::Other(format!("kaliru: no list {}", field)))?;
            *list.borrow_mut() = items.iter().map(|i| i.as_string()).collect::<KResult<_>>()?;
            Ok(())
        },
    };
    r.list("klippy.extras.heaters.PrinterHeaters.available_heaters", names);
    r.list("klippy.extras.heaters.PrinterHeaters.available_sensors", names);
    r.list("klippy.extras.heaters.PrinterHeaters.available_monitors", names);
    r.dict(
        "klippy.extras.heaters.PrinterHeaters.heaters",
        DictField {
            get: |o, _f, key| match ph(o)?.heaters.borrow().get(&key.as_string()?) {
                Some(slot) => slot.as_pobj().to_py(),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                let pobj = py_to_pobj(value);
                let slot = match crate::printer::downcast_rc::<Heater>(pobj.clone()) {
                    Some(h) => HeaterSlot::Kaliru(h),
                    None => HeaterSlot::Plugin(pobj),
                };
                ph(o)?.heaters.borrow_mut().insert(key.as_string()?, slot);
                Ok(())
            },
            del: |o, _f, key| match ph(o)?.heaters.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => Ok(()),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(ph(o)?.heaters.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
}
