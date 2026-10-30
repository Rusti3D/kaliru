// Fields of klippy/extras objects plugins read and write: gcode_move,
// idle_timeout, heaters.Heater
use super::*;
use crate::extras::gcode_move::{GCodeMove, MoveTransform, MoveTransformRef, SavedState};
use crate::plugins::convert::ToPy;
use crate::extras::exclude_object::ExcludeObject;
use crate::plugins::native::{DictField, ListField};
use crate::toolhead::ToolHead;
use crate::extras::heaters::Heater;
use crate::extras::idle_timeout::IdleTimeout;
use crate::plugins::callable::py_function;
use crate::plugins::convert::FromPy;
use crate::{cell_attr, method, ro_attr, vec_attr};

pub fn register(r: &mut Registry) {
    // gcode_move.GCodeMove
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.absolute_coord", GCodeMove, absolute_coord);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.absolute_extrude", GCodeMove, absolute_extrude);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.speed", GCodeMove, speed);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.speed_factor", GCodeMove, speed_factor);
    cell_attr!(r, "klippy.extras.gcode_move.GCodeMove.extrude_factor", GCodeMove, extrude_factor);
    vec_attr!(r, "klippy.extras.gcode_move.GCodeMove.base_position", GCodeMove, base_position);
    vec_attr!(r, "klippy.extras.gcode_move.GCodeMove.last_position", GCodeMove, last_position);
    vec_attr!(r, "klippy.extras.gcode_move.GCodeMove.homing_position", GCodeMove, homing_position);
    // self.move_with_transform / position_with_transform: the transform's methods
    r.attr("klippy.extras.gcode_move.GCodeMove.move_with_transform", |o| {
        let g = objects::unwrap_as::<GCodeMove>(o)?;
        py_function(Rc::new(move |a: &[PyRef]| {
            let pos: Vec<f64> = FromPy::from_py(&a[0])?;
            g.move_with_transform(&pos, a[1].as_f64()?)?;
            Ok(py_none())
        }))
    }, None);
    r.attr("klippy.extras.gcode_move.GCodeMove.position_with_transform", |o| {
        let g = objects::unwrap_as::<GCodeMove>(o)?;
        py_function(Rc::new(move |_a: &[PyRef]| crate::plugins::convert::ToPy::to_py(g.position_with_transform())))
    }, None);

    // set_move_transform(transform, force=False): the old transform
    r.method("klippy.extras.gcode_move.GCodeMove.set_move_transform", |a| {
        let g = a.this::<GCodeMove>()?;
        match g.set_move_transform(transform_from_py(a.arg(0))?, a.bool(1)?)? {
            Some(t) => transform_to_py(t),
            None => Ok(py_none()),
        }
    });
    r.attr("klippy.extras.gcode_move.GCodeMove.move_transform", |o| {
        match objects::unwrap_as::<GCodeMove>(o)?.move_transform.borrow().clone() {
            Some(t) => transform_to_py(t),
            None => Ok(py_none()),
        }
    }, None);
    r.attr("klippy.extras.gcode_move.GCodeMove.Coord", |_o| lookup("klippy.gcode", "Coord"), None);
    r.dict("klippy.extras.gcode_move.GCodeMove.saved_states", DictField {
        get: |o, _f, key| {
            let name = key.as_string()?;
            match objects::unwrap_as::<GCodeMove>(o)?.saved_states.borrow().get(&name) {
                Some(s) => saved_state_to_py(s),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            }
        },
        set: |o, _f, key, value| {
            let state = saved_state_from_py(value)?;
            objects::unwrap_as::<GCodeMove>(o)?.saved_states.borrow_mut().insert(key.as_string()?, state);
            Ok(())
        },
        del: |o, _f, key| match objects::unwrap_as::<GCodeMove>(o)?.saved_states.borrow_mut().shift_remove(&key.as_string()?) {
            Some(_) => Ok(()),
            None => Err(KError::Py("KeyError".into(), key.repr()?)),
        },
        keys: |o, _f| Ok(objects::unwrap_as::<GCodeMove>(o)?.saved_states.borrow().keys().map(|k| py_str(k)).collect()),
    });
    // move(newpos, speed) / get_position() of kaliru's transforms
    macro_rules! transform {
        ($r:ident, $class:literal, $ty:ty) => {
            $r.method(concat!($class, ".move"), |a| {
                let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
                MoveTransform::move_to(&*a.this::<$ty>()?, &newpos, a.f64(1)?)?;
                Ok(py_none())
            });
            $r.method(concat!($class, ".get_position"), |a| MoveTransform::get_position(&*a.this::<$ty>()?).to_py());
            objects::register_view::<$ty>(|a| -> Box<dyn Any> { Box::new(a.downcast::<$ty>().expect("type") as MoveTransformRef) });
        };
    }
    transform!(r, "klippy.extras.bed_tilt.BedTilt", crate::extras::bed_tilt::BedTilt);
    transform!(r, "klippy.extras.exclude_object.ExcludeObject", crate::extras::exclude_object::ExcludeObject);
    transform!(r, "klippy.extras.firmware_retraction.FirmwareRetraction", crate::extras::firmware_retraction::FirmwareRetraction);
    transform!(r, "klippy.extras.mixing_extruder.GradientMixingExtruder", crate::extras::mixing_extruder::MixingExtruder);
    transform!(r, "klippy.extras.skew_correction.PrinterSkew", crate::extras::skew_correction::PrinterSkew);
    transform!(r, "klippy.extras.tuning_tower.TuningTower", crate::extras::tuning_tower::TuningTower);
    transform!(r, "klippy.extras.z_thermal_adjust.ZThermalAdjuster", crate::extras::z_thermal_adjust::ZThermalAdjuster);
    objects::register_view::<crate::extras::bed_mesh::BedMesh>(|a| -> Box<dyn Any> {
        Box::new(a.downcast::<crate::extras::bed_mesh::BedMesh>().expect("type") as MoveTransformRef)
    });
    objects::register_view::<ToolHead>(|a| -> Box<dyn Any> { Box::new(a.downcast::<ToolHead>().expect("type") as MoveTransformRef) });

    // exclude_object.ExcludeObject: its state (lists read and written whole)
    fn eo(o: &PyRef) -> KResult<Rc<ExcludeObject>> {
        objects::unwrap_as::<ExcludeObject>(o)
    }
    macro_rules! eo_list {
        ($r:ident, $field:ident, $conv:expr) => {
            $r.list(concat!("klippy.extras.exclude_object.ExcludeObject.", stringify!($field)), ListField {
                get: |o, _f| eo(o)?.st.borrow().$field.iter().map(|v| v.clone().to_py()).collect(),
                set: |o, _f, items| {
                    let v: KResult<Vec<_>> = items.iter().map($conv).collect();
                    eo(o)?.st.borrow_mut().$field = v?;
                    Ok(())
                },
            });
        };
    }
    // (objects: dicts)
    eo_list!(r, objects, |v: &PyRef| match to_pyval(v)? {
        PyVal::Dict(d) => Ok(d),
        _ => Err(KError::Py("TypeError".into(), "kaliru: exclude_object objects are dicts".into())),
    });
    eo_list!(r, excluded_objects, String::from_py);
    eo_list!(r, last_position, f64::from_py);
    eo_list!(r, last_position_extruded, f64::from_py);
    eo_list!(r, last_position_excluded, f64::from_py);
    macro_rules! eo_value {
        ($r:ident, $field:ident, $ty:ty) => {
            $r.attr(concat!("klippy.extras.exclude_object.ExcludeObject.", stringify!($field)), |o| eo(o)?.st.borrow().$field.clone().to_py(), Some(|o, v| {
                let v: $ty = FromPy::from_py(v)?;
                eo(o)?.st.borrow_mut().$field = v;
                Ok(())
            }));
        };
    }
    eo_value!(r, current_object, Option<String>);
    eo_value!(r, in_excluded_region, bool);
    eo_value!(r, max_position_extruded, f64);
    eo_value!(r, max_position_excluded, f64);
    eo_value!(r, extruder_adj, f64);
    eo_value!(r, initial_extrusion_moves, i64);
    // {extruder: [offsets]} (a copy)
    r.attr("klippy.extras.exclude_object.ExcludeObject.extrusion_offsets", |o| {
        let offsets = eo(o)?.st.borrow().extrusion_offsets.clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in offsets {
            d.call_method("__setitem__", &[&py_str(&k), &v.to_py()?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.extras.exclude_object.ExcludeObject.last_speed", |o| match eo(o)?.st.borrow().last_speed {
        Some(s) => Ok(py_float(s)),
        None => Err(KError::Py("AttributeError".into(), "'ExcludeObject' object has no attribute 'last_speed'".into())),
    }, None);
    r.attr("klippy.extras.exclude_object.ExcludeObject.next_transform", |o| match eo(o)?.st.borrow().next_transform.clone() {
        Some(t) => transform_to_py(t),
        None => Ok(py_none()),
    }, None);

    // firmware_retraction.FirmwareRetraction: its settings
    fn fr(o: &PyRef) -> KResult<Rc<crate::extras::firmware_retraction::FirmwareRetraction>> {
        objects::unwrap_as::<crate::extras::firmware_retraction::FirmwareRetraction>(o)
    }
    macro_rules! fr_param {
        ($r:ident, $field:ident) => {
            $r.attr(concat!("klippy.extras.firmware_retraction.FirmwareRetraction.", stringify!($field)), |o| fr(o)?.p.get().$field.to_py(), Some(|o, v| {
                let f = fr(o)?;
                let mut p = f.p.get();
                p.$field = FromPy::from_py(v)?;
                f.p.set(p);
                Ok(())
            }));
        };
    }
    fr_param!(r, retract_length);
    fr_param!(r, retract_speed);
    fr_param!(r, unretract_extra_length);
    fr_param!(r, unretract_speed);
    fr_param!(r, z_hop_height);
    fr_param!(r, clear_zhop);
    fr_param!(r, unretract_length);
    r.attr("klippy.extras.firmware_retraction.FirmwareRetraction.config_ref", |o| objects::wrap(Rc::new(fr(o)?.config_ref.clone())), None);

    // input_shaper: the axis shapers and their (n, A, T)
    {
        use crate::extras::input_shaper::{AxisInputShaper, InputShaper};
        fn shapers_list(s: &InputShaper) -> KResult<PyRef> {
            let l: KResult<Vec<PyRef>> = s.get_shapers().iter().cloned().map(objects::wrap).collect();
            Ok(py_list(l?))
        }
        r.method("klippy.extras.input_shaper.InputShaper.get_shapers", |a| shapers_list(&*a.this::<InputShaper>()?));
        r.attr("klippy.extras.input_shaper.InputShaper.shapers", |o| shapers_list(&*objects::unwrap_as::<InputShaper>(o)?), None);
        fn ais(o: &PyRef) -> KResult<Rc<AxisInputShaper>> {
            objects::unwrap_as::<AxisInputShaper>(o)
        }
        r.method("klippy.extras.input_shaper.AxisInputShaper.get_shaper", |a| a.this::<AxisInputShaper>()?.state.borrow().clone().to_py());
        r.attr("klippy.extras.input_shaper.AxisInputShaper.n", |o| Ok(py_int(ais(o)?.state.borrow().0 as i64)), None);
        r.attr("klippy.extras.input_shaper.AxisInputShaper.A", |o| ais(o)?.state.borrow().1.clone().to_py(), None);
        r.attr("klippy.extras.input_shaper.AxisInputShaper.T", |o| ais(o)?.state.borrow().2.clone().to_py(), None);
        r.attr("klippy.extras.input_shaper.AxisInputShaper.params", |o| objects::wrap(ais(o)?.params.clone()), None);
        r.attr("klippy.extras.input_shaper.AxisInputShaper.saved", |o| ais(o)?.saved.borrow().clone().to_py(), None);
    }

    // idle_timeout.IdleTimeout
    cell_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.idle_timeout", IdleTimeout, idle_timeout);
    cell_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.last_print_start_systime", IdleTimeout, last_print_start_systime);
    ro_attr!(r, "klippy.extras.idle_timeout.IdleTimeout.state", IdleTimeout, |s| s.state.borrow().to_string());

    // heaters.Heater
    ro_attr!(r, "klippy.extras.heaters.Heater.name", Heater, |h| h.name.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.short_name", Heater, |h| h.short_name.clone());
    ro_attr!(r, "klippy.extras.heaters.Heater.min_temp", Heater, |h| h.min_temp);
    ro_attr!(r, "klippy.extras.heaters.Heater.max_temp", Heater, |h| h.max_temp);
    cell_attr!(r, "klippy.extras.heaters.Heater.min_extrude_temp", Heater, min_extrude_temp);
    cell_attr!(r, "klippy.extras.heaters.Heater.can_extrude", Heater, can_extrude);
    ro_attr!(r, "klippy.extras.heaters.Heater.last_temp", Heater, |h| h.last_temp.get());
    ro_attr!(r, "klippy.extras.heaters.Heater.smoothed_temp", Heater, |h| h.smoothed_temp.get());
    ro_attr!(r, "klippy.extras.heaters.Heater.target_temp", Heater, |h| h.target_temp.get());
    method!(r, "klippy.extras.heaters.Heater", Heater, get_name(h) => h.get_name().to_string());
    method!(r, "klippy.extras.heaters.Heater", Heater, set_temp(h, degrees: f64) => h.set_temp(degrees));
    method!(r, "klippy.extras.heaters.Heater", Heater, check_busy(h, eventtime: f64) => h.check_busy(eventtime));
    method!(r, "klippy.extras.heaters.Heater", Heater, get_max_power(h) => h.get_max_power());
    method!(r, "klippy.extras.heaters.Heater", Heater, get_smooth_time(h) => h.get_smooth_time());
    method!(r, "klippy.extras.heaters.Heater", Heater, get_pwm_delay(h) => h.get_pwm_delay());
    method!(r, "klippy.extras.heaters.Heater", Heater, alter_target(h, target_temp: f64) => h.alter_target(target_temp));
}

/// A plugin's move transform (what it handed set_move_transform)
struct PyMoveTransform {
    obj: PyRef,
}

impl MoveTransform for PyMoveTransform {
    fn move_to(&self, newpos: &[f64], speed: f64) -> KResult<()> {
        self.obj.call_method("move", &[&newpos.to_vec().to_py()?, &py_float(speed)])?;
        Ok(())
    }
    fn get_position(&self) -> Vec<f64> {
        // (kaliru's transforms cannot fail here; a python error is reported)
        match self.obj.call_method("get_position", &[]).and_then(|p| FromPy::from_py(&p)) {
            Ok(p) => p,
            Err(e) => {
                crate::logging::exception_err("python move transform get_position", &e);
                vec![]
            }
        }
    }
}

/// A transform as python sees it
pub fn transform_to_py(t: MoveTransformRef) -> KResult<PyRef> {
    if let Some(p) = t.downcast_ref::<PyMoveTransform>() {
        return Ok(p.obj.clone());
    }
    let any: Rc<dyn Any> = t.clone();
    let name = t.type_name();
    objects::wrap_view(any, t, name)
}

/// A transform python hands kaliru: kaliru's own, else called through
pub fn transform_from_py(o: &PyRef) -> KResult<MoveTransformRef> {
    if is_mirror_instance(o)? {
        if let Some(t) = objects::unwrap_view::<dyn MoveTransform>(o) {
            return Ok(t);
        }
    }
    Ok(Rc::new(PyMoveTransform { obj: o.clone() }))
}

/// saved_states[name]: python's dict of the state
fn saved_state_to_py(s: &SavedState) -> KResult<PyRef> {
    let d = lookup("builtins", "dict")?.call(&[])?;
    let items: [(&str, PyRef); 8] = [
        ("absolute_coord", py_bool(s.absolute_coord)),
        ("absolute_extrude", py_bool(s.absolute_extrude)),
        ("base_position", s.base_position.clone().to_py()?),
        ("last_position", s.last_position.clone().to_py()?),
        ("homing_position", s.homing_position.clone().to_py()?),
        ("speed", py_float(s.speed)),
        ("speed_factor", py_float(s.speed_factor)),
        ("extrude_factor", py_float(s.extrude_factor)),
    ];
    for (k, v) in items {
        d.call_method("__setitem__", &[&py_str(k), &v])?;
    }
    Ok(d)
}

fn saved_state_from_py(d: &PyRef) -> KResult<SavedState> {
    let get = |k: &str| d.call_method("__getitem__", &[&py_str(k)]);
    Ok(SavedState {
        absolute_coord: get("absolute_coord")?.is_true()?,
        absolute_extrude: get("absolute_extrude")?.is_true()?,
        base_position: FromPy::from_py(&get("base_position")?)?,
        last_position: FromPy::from_py(&get("last_position")?)?,
        homing_position: FromPy::from_py(&get("homing_position")?)?,
        speed: get("speed")?.as_f64()?,
        speed_factor: get("speed_factor")?.as_f64()?,
        extrude_factor: get("extrude_factor")?.as_f64()?,
    })
}
