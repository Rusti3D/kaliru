// klippy/extras/filament_switch_sensor.py: SwitchSensor, RunoutHelper. AFC
// rewires these: it sets runout_helper fields to its own values and
// replaces note_filament_present / _runout_event_handler (kaliru's methods
// dispatch to the replacement: plugins::dispatch).
use super::*;
use crate::extras::filament_motion_sensor::EncoderSensor;
use crate::extras::filament_switch_sensor::{DefinedSensor, GcodeSlot, RunoutHelper, SwitchSensor};
use crate::extras::hall_filament_width_sensor::HallFilamentWidthSensor;
use crate::gcode::GCodeCommand;
use crate::plugins::gil::Gil;
use crate::plugins::native::Args;
use crate::pyval::PyDict;
use std::collections::HashMap;
use crate::{cell_attr, method, ro_attr};
use crate::plugins::convert::ToPy;
use crate::plugins::py_to_pobj;

fn rh(o: &PyRef) -> KResult<Rc<RunoutHelper>> {
    objects::unwrap_as::<RunoutHelper>(o)
}

/// A plugin's sensor behind a RunoutHelper it built (python's defined_sensor)
struct PyDefinedSensor {
    obj: PyRef,
}

impl DefinedSensor for PyDefinedSensor {
    fn get_extruder_pos(&self, eventtime: Option<f64>) -> KResult<f64> {
        let t = eventtime.map_or_else(py_none, py_float);
        self.obj.call_method("get_extruder_pos", &[&t])?.as_f64()
    }
    fn sensor_get_status(&self, eventtime: f64) -> PyDict {
        // (kaliru's sensors cannot fail here; a python error is reported)
        match self.obj.call_method("sensor_get_status", &[&py_float(eventtime)]).and_then(|d| to_pyval(&d)) {
            Ok(PyVal::Dict(d)) => d,
            Ok(_) => PyDict::default(),
            Err(e) => {
                crate::logging::exception_err("python filament sensor sensor_get_status", &e);
                PyDict::default()
            }
        }
    }
    fn get_info(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        self.obj.call_method("get_info", &[&super::gcode::gcmd_to_py(gcmd)?])?.is_true()
    }
    fn reset_needed(&self, enable: Option<i64>, always_fire_events: Option<i64>) -> bool {
        let (e, f) = (enable.map_or_else(py_none, py_int), always_fire_events.map_or_else(py_none, py_int));
        match self.obj.call_method("reset_needed", &[&e, &f]).and_then(|v| v.is_true()) {
            Ok(v) => v,
            Err(e) => {
                crate::logging::exception_err("python filament sensor reset_needed", &e);
                false
            }
        }
    }
    fn set_filament_sensor(&self, gcmd: &GCodeCommand) -> KResult<bool> {
        self.obj.call_method("set_filament_sensor", &[&super::gcode::gcmd_to_py(gcmd)?])?.is_true()
    }
    fn reset(&self) -> KResult<()> {
        self.obj.call_method("reset", &[])?;
        Ok(())
    }
}

// the plugin sensors of the helpers plugins built (python's helper holds
// its sensor; kaliru's holds a weak reference)
static PY_SENSORS: Gil<RefCell<HashMap<usize, Rc<dyn DefinedSensor>>>> = Gil::new(|| RefCell::new(HashMap::new()));

/// A defined sensor as python sees it
fn defined_sensor_to_py(s: Rc<dyn DefinedSensor>) -> KResult<PyRef> {
    if let Some(p) = s.downcast_ref::<PyDefinedSensor>() {
        return Ok(p.obj.clone());
    }
    let any: Rc<dyn Any> = s.clone();
    let name = s.type_name();
    objects::wrap_view(any, s, name)
}

fn this_sensor(a: &Args) -> KResult<Rc<dyn DefinedSensor>> {
    objects::unwrap_view::<dyn DefinedSensor>(a.slf.as_ref().expect("method"))
        .ok_or_else(|| KError::Py("TypeError".into(), "kaliru: not a filament sensor".into()))
}

fn opt_int(o: &PyRef) -> KResult<Option<i64>> {
    if o.is_none() { Ok(None) } else { o.as_i64().map(Some) }
}

// the DefinedSensor methods of a kaliru sensor class
macro_rules! defined_sensor {
    ($r:ident, $class:literal) => {
        $r.method(concat!($class, ".get_extruder_pos"), |a| {
            let t = if a.arg(0).is_none() { None } else { Some(a.f64(0)?) };
            Ok(py_float(this_sensor(a)?.get_extruder_pos(t)?))
        });
        // (hall_filament_width_sensor: eventtime=None)
        $r.method(concat!($class, ".sensor_get_status"), |a| {
            let t = match a.opt_f64(0)? {
                Some(t) => t,
                None => printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.get_reactor().monotonic(),
            };
            Ok(from_pyval(&PyVal::Dict(this_sensor(a)?.sensor_get_status(t))))
        });
        $r.method(concat!($class, ".get_info"), |a| Ok(py_bool(this_sensor(a)?.get_info(&*super::gcode::gcmd_from_py(a.arg(0))?)?)));
        $r.method(concat!($class, ".reset_needed"), |a| Ok(py_bool(this_sensor(a)?.reset_needed(opt_int(a.arg(0))?, opt_int(a.arg(1))?))));
        $r.method(concat!($class, ".set_filament_sensor"), |a| {
            Ok(py_bool(this_sensor(a)?.set_filament_sensor(&*super::gcode::gcmd_from_py(a.arg(0))?)?))
        });
        $r.method(concat!($class, ".reset"), |a| {
            this_sensor(a)?.reset()?;
            Ok(py_none())
        });
        // self.estimated_print_time = mcu.estimated_print_time
        $r.attr(concat!($class, ".estimated_print_time"), |_o| {
            let p = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
            p.lookup_object("mcu")?.to_py()?.getattr("estimated_print_time")
        }, None);
    };
}

pub fn register(r: &mut Registry) {
    // RunoutHelper(config, defined_sensor, runout_distance=0): a plugin's
    // own sensor
    r.ctor("klippy.extras.filament_switch_sensor.RunoutHelper.__init__", |a| {
        let config = super::configfile::config_from_py(a.arg(0))?;
        let h = RunoutHelper::new(&config, a.f64(2)?)?;
        let sensor: Rc<dyn DefinedSensor> = match objects::unwrap_view::<dyn DefinedSensor>(a.arg(1)) {
            Some(s) if is_mirror_instance(a.arg(1))? => s,
            _ => {
                let s: Rc<dyn DefinedSensor> = Rc::new(PyDefinedSensor { obj: a.arg(1).clone() });
                PY_SENSORS.with(|m| m.borrow_mut().insert(Rc::as_ptr(&h) as usize, s.clone()));
                s
            }
        };
        h.set_defined_sensor(Rc::downgrade(&sensor));
        let any: Rc<dyn Any> = h;
        Ok(any)
    });
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.defined_sensor", |o| match rh(o)?.defined_sensor() {
        Some(s) => defined_sensor_to_py(s),
        None => Ok(py_none()),
    }, None);
    objects::register_view::<SwitchSensor>(|a| -> Box<dyn Any> { Box::new(a.downcast::<SwitchSensor>().expect("type") as Rc<dyn DefinedSensor>) });
    objects::register_view::<EncoderSensor>(|a| -> Box<dyn Any> { Box::new(a.downcast::<EncoderSensor>().expect("type") as Rc<dyn DefinedSensor>) });
    objects::register_view::<HallFilamentWidthSensor>(|a| -> Box<dyn Any> {
        Box::new(a.downcast::<HallFilamentWidthSensor>().expect("type") as Rc<dyn DefinedSensor>)
    });
    defined_sensor!(r, "klippy.extras.filament_switch_sensor.SwitchSensor");
    defined_sensor!(r, "klippy.extras.filament_motion_sensor.EncoderSensor");
    defined_sensor!(r, "klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor");
    r.method("klippy.extras.filament_motion_sensor.EncoderSensor.encoder_event", |a| {
        a.this::<EncoderSensor>()?.encoder_event(a.f64(0)?)?;
        Ok(py_none())
    });
    r.attr("klippy.extras.filament_motion_sensor.EncoderSensor.gcode", |_o| {
        printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup_object("gcode")?.to_py()
    }, None);
    r.attr("klippy.extras.filament_switch_sensor.SwitchSensor.runout_helper", |o| {
        objects::unwrap_as::<SwitchSensor>(o)?.runout_helper.clone().to_py()
    }, None);
    method!(r, "klippy.extras.filament_switch_sensor.RunoutHelper", RunoutHelper, note_filament_present(
        h, eventtime: f64, is_filament_present: Option<bool>, force: bool, immediate: bool) =>
        h.note_filament_present(eventtime, is_filament_present, force, immediate));
    method!(r, "klippy.extras.filament_switch_sensor.RunoutHelper", RunoutHelper, get_status(h, eventtime: f64) => h.get_status(eventtime));
    method!(r, "klippy.extras.filament_switch_sensor.RunoutHelper", RunoutHelper, reset_runout_distance_info(h) => h.reset_runout_distance_info());

    // fields
    ro_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.name", RunoutHelper, |h| h.name.clone());
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.smart", RunoutHelper, smart);
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.always_fire_events", RunoutHelper, always_fire_events);
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.runout_distance", RunoutHelper, runout_distance);
    cell_attr!(r, "klippy.extras.filament_switch_sensor.RunoutHelper.runout_elapsed", RunoutHelper, runout_elapsed);
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.sensor_enabled", |o| Ok(py_bool(rh(o)?.sensor_enabled.get())), Some(|o, v| {
        rh(o)?.sensor_enabled.set(v.is_true()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.filament_present", |o| Ok(py_bool(rh(o)?.filament_present.get())), Some(|o, v| {
        rh(o)?.filament_present.set(v.is_true()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.runout_pause", |o| Ok(py_bool(rh(o)?.runout_pause.get())), Some(|o, v| {
        rh(o)?.runout_pause.set(v.is_true()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.event_delay", |o| Ok(py_float(rh(o)?.event_delay.get())), Some(|o, v| {
        rh(o)?.event_delay.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.min_event_systime", |o| Ok(py_float(rh(o)?.min_event_systime.get())), Some(|o, v| {
        rh(o)?.min_event_systime.set(v.as_f64()?);
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.runout_gcode", |o| slot_to_py(&rh(o)?.runout_gcode.borrow()), Some(|o, v| {
        *rh(o)?.runout_gcode.borrow_mut() = slot_from_py(v)?;
        Ok(())
    }));
    r.attr("klippy.extras.filament_switch_sensor.RunoutHelper.insert_gcode", |o| slot_to_py(&rh(o)?.insert_gcode.borrow()), Some(|o, v| {
        *rh(o)?.insert_gcode.borrow_mut() = slot_from_py(v)?;
        Ok(())
    }));
}

fn slot_to_py(slot: &Option<GcodeSlot>) -> KResult<PyRef> {
    match slot {
        None => Ok(py_none()),
        Some(GcodeSlot::Template(t)) => t.clone().to_py(),
        Some(GcodeSlot::Plugin(p)) => p.clone().to_py(),
        Some(GcodeSlot::Native) | Some(GcodeSlot::Render(_)) => Ok(py_int(1)),
    }
}

fn slot_from_py(v: &PyRef) -> KResult<Option<GcodeSlot>> {
    if v.is_none() {
        return Ok(None);
    }
    if let Ok(t) = objects::unwrap_as::<crate::extras::gcode_macro::Template>(v) {
        return Ok(Some(GcodeSlot::Template(t)));
    }
    Ok(Some(GcodeSlot::Plugin(py_to_pobj(v))))
}
