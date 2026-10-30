// klippy/kinematics/*.py: the kinematics classes, through kaliru's
// Kinematics trait (one Rust type can be several Kalico classes:
// CartKinematics is cartesian, corexy, corexz, the hybrids and the limited
// variants)
use super::toolhead::{kin_to_py, rail_to_py};
use super::*;
use crate::extras::homing::Homing;
use crate::kinematics::cartesian::{CartKinematics, Kind};
use crate::kinematics::limited::Limited;
use crate::kinematics::{Kinematics, KINEMATICS};
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::{known, Args, Getter, ListField, Method, Setter};
use crate::stepper::PrinterRail;
use crate::toolhead::{Move, ToolHead};

fn kin(o: &PyRef) -> KResult<Rc<dyn Kinematics>> {
    objects::unwrap_view::<dyn Kinematics>(o).ok_or_else(|| KError::Py("TypeError".into(), "kaliru: not a kinematics object".into()))
}

fn this(a: &Args) -> KResult<Rc<dyn Kinematics>> {
    kin(a.slf.as_ref().expect("method"))
}

/// homing_axes / clear_axes: a string of axis names (or any iterable of them)
fn axes(o: &PyRef) -> KResult<String> {
    if let Ok(s) = o.as_string() {
        return Ok(s);
    }
    let names: Vec<String> = FromPy::from_py(o)?;
    Ok(names.concat())
}

/// toolhead.Coord(*values)
fn coord(v: &[f64; 4]) -> KResult<PyRef> {
    let args: Vec<PyRef> = v.iter().map(|x| py_float(*x)).collect();
    lookup("klippy.gcode", "Coord")?.call(&args.iter().collect::<Vec<_>>())
}

fn cart(o: &PyRef) -> KResult<Rc<CartKinematics>> {
    objects::unwrap_as::<CartKinematics>(o)
}

/// f(the limits) of a limited_* kinematics (only its Kalico class has the fields)
fn limited<R>(o: &PyRef, f: impl FnOnce(&Limited) -> R) -> KResult<R> {
    match &cart(o)?.limited {
        Some(l) => Ok(f(l)),
        None => Err(KError::Py("AttributeError".into(), "kaliru: not a limited kinematics".into())),
    }
}

// registered only where the Kalico class has the name
fn method(r: &mut Registry, name: &'static str, f: Method) {
    if known(name) {
        r.method(name, f);
    }
}

fn attr(r: &mut Registry, name: &'static str, get: Getter, set: Option<Setter>) {
    if known(name) {
        r.attr(name, get, set);
    }
}

macro_rules! kin_class {
    ($r:ident, $class:literal) => {
        method($r, concat!($class, ".get_steppers"), |a| {
            let steppers: KResult<Vec<PyRef>> = this(a)?.get_steppers().into_iter().map(objects::wrap).collect();
            Ok(py_list(steppers?))
        });
        // calc_position(stepper_positions): {stepper name: position}
        method($r, concat!($class, ".calc_position"), |a| {
            this(a)?.calc_position(&super::homing::f64_map_from_py(a.arg(0))?).to_py()
        });
        method($r, concat!($class, ".set_position"), |a| {
            let newpos: Vec<f64> = FromPy::from_py(a.arg(0))?;
            this(a)?.set_position(&newpos, &axes(a.arg(1))?)?;
            Ok(py_none())
        });
        method($r, concat!($class, ".clear_homing_state"), |a| {
            this(a)?.clear_homing_state(&axes(a.arg(0))?);
            Ok(py_none())
        });
        method($r, concat!($class, ".note_z_not_homed"), |a| {
            this(a)?.clear_homing_state("z");
            Ok(py_none())
        });
        method($r, concat!($class, ".home"), |a| {
            this(a)?.home(&objects::unwrap_as::<Homing>(a.arg(0))?)?;
            Ok(py_none())
        });
        // check_move(move): limits the python move itself
        method($r, concat!($class, ".check_move"), |a| {
            let m = objects::unwrap_as::<RefCell<Move>>(a.arg(0))?;
            this(a)?.check_move(&mut m.borrow_mut())?;
            Ok(py_none())
        });
        method($r, concat!($class, ".update_limits"), |a| {
            let range: (f64, f64) = FromPy::from_py(a.arg(1))?;
            this(a)?.update_limits(a.arg(0).as_i64()? as usize, range);
            Ok(py_none())
        });
        method($r, concat!($class, ".home_axis"), |a| {
            let rail = objects::unwrap_as::<PrinterRail>(a.arg(2))?;
            this(a)?.home_axis(&objects::unwrap_as::<Homing>(a.arg(0))?, a.arg(1).as_i64()? as usize, &rail)?;
            Ok(py_none())
        });
        attr($r, concat!($class, ".supports_dual_carriage"), |o| Ok(py_bool(kin(o)?.supports_dual_carriage())), None);
        attr($r, concat!($class, ".rails"), |o| {
            let rails: KResult<Vec<PyRef>> = kin(o)?.rails().into_iter().map(rail_to_py).collect();
            Ok(py_list(rails?))
        }, None);
        attr($r, concat!($class, ".printer"), |o| {
            kin(o)?;
            printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.to_py()
        }, None);
        kin_limit!($r, $class, max_z_velocity);
        kin_limit!($r, $class, max_z_accel);
        kin_limit!($r, $class, max_x_accel);
        kin_limit!($r, $class, max_y_accel);
    };
}

// max_z_velocity / max_z_accel / ...: the kinematics' own value
macro_rules! kin_limit {
    ($r:ident, $class:literal, $name:ident) => {
        attr($r, concat!($class, ".", stringify!($name)), |o| match kin(o)?.limit(stringify!($name)) {
            Some(c) => Ok(py_float(c.get())),
            None => Err(KError::Py("AttributeError".into(), stringify!($name).into())),
        }, Some(|o, v| match kin(o)?.limit(stringify!($name)) {
            Some(c) => {
                c.set(v.as_f64()?);
                Ok(())
            }
            None => Err(KError::Py("AttributeError".into(), stringify!($name).into())),
        }));
    };
}

// the cartesian family's own fields
macro_rules! cart_class {
    ($r:ident, $class:literal) => {
        attr($r, concat!($class, ".axes_min"), |o| coord(&cart(o)?.axes_min), None);
        attr($r, concat!($class, ".axes_max"), |o| coord(&cart(o)?.axes_max), None);
        attr($r, concat!($class, ".dual_carriage_axis"), |o| match &cart(o)?.dc_module {
            Some(dc) => Ok(py_int(dc.axis as i64)),
            None => Ok(py_none()),
        }, None);
        // set to [] and never filled in Kalico
        attr($r, concat!($class, ".dual_carriage_rails"), |o| {
            cart(o)?;
            Ok(py_list(vec![]))
        }, None);
        // the limited variants' limits (snapshots)
        attr($r, concat!($class, ".config_max_velocity"), |o| Ok(py_float(limited(o, |l| l.config_max_velocity)?)), None);
        attr($r, concat!($class, ".config_max_accel"), |o| Ok(py_float(limited(o, |l| l.config_max_accel)?)), None);
        attr($r, concat!($class, ".max_velocities"), |o| limited(o, |l| l.max_velocities.borrow().to_vec())?.to_py(), None);
        attr($r, concat!($class, ".max_accels"), |o| limited(o, |l| l.max_accels.borrow().to_vec())?.to_py(), None);
        attr($r, concat!($class, ".xy_hypot_accel"), |o| Ok(py_float(limited(o, |l| l.xy_hypot_accel.get())?)), None);
        attr($r, concat!($class, ".scale_per_axis"), |o| Ok(py_bool(limited(o, |l| l.scale_per_axis.get())?)), None);
        // limits: [(min, max), ...] (item assignment writes the list back)
        if known(concat!($class, ".limits")) {
            $r.list(concat!($class, ".limits"), ListField {
                get: |o, _f| cart(o)?.limits.borrow().iter().map(|l| l.to_py()).collect(),
                set: |o, _f, items| {
                    let k = cart(o)?;
                    let mut limits = k.limits.borrow_mut();
                    if items.len() != limits.len() {
                        return Err(KError::Py("NotImplementedError".into(), "kaliru: limits keeps one range per axis".into()));
                    }
                    for (l, item) in limits.iter_mut().zip(&items) {
                        *l = FromPy::from_py(item)?;
                    }
                    Ok(())
                },
            });
        }
    };
}

// load_kinematics(toolhead, config) of a kinematics module
macro_rules! load_kinematics {
    ($r:ident, $module:literal) => {
        $r.method(concat!("klippy.kinematics.", $module, ".load_kinematics"), |a| {
            let toolhead = objects::unwrap_as::<ToolHead>(a.arg(0))?;
            let config = super::configfile::config_from_py(a.arg(1))?;
            let load = KINEMATICS.iter().find(|(n, _)| *n == $module).and_then(|(_, l)| *l).expect("kinematics module");
            kin_to_py(load(&toolhead, &config)?)
        });
    };
}

pub fn register(r: &mut Registry) {
    // CartKinematics is the Kalico class of its kind
    objects::register_class_fn::<CartKinematics>(|a| {
        let k = a.downcast_ref::<CartKinematics>().expect("type");
        match (&k.kind, k.limited.is_some()) {
            (Kind::Cartesian, false) => ("klippy.kinematics.cartesian", "CartKinematics"),
            (Kind::Cartesian, true) => ("klippy.kinematics.limited_cartesian", "LimitedCartKinematics"),
            (Kind::CoreXY, false) => ("klippy.kinematics.corexy", "CoreXYKinematics"),
            (Kind::CoreXY, true) => ("klippy.kinematics.limited_corexy", "LimitedCoreXYKinematics"),
            (Kind::CoreXZ, false) => ("klippy.kinematics.corexz", "CoreXZKinematics"),
            (Kind::CoreXZ, true) => ("klippy.kinematics.limited_corexz", "LimitedCoreXZKinematics"),
            (Kind::HybridCoreXY { .. }, _) => ("klippy.kinematics.hybrid_corexy", "HybridCoreXYKinematics"),
            (Kind::HybridCoreXZ { .. }, _) => ("klippy.kinematics.hybrid_corexz", "HybridCoreXZKinematics"),
        }
    });
    macro_rules! view {
        ($($ty:ty),*) => {$(
            objects::register_view::<$ty>(|a| -> Box<dyn Any> { Box::new(a.downcast::<$ty>().expect("type") as Rc<dyn Kinematics>) });
        )*};
    }
    view!(
        CartKinematics,
        crate::kinematics::delta::DeltaKinematics,
        crate::kinematics::deltesian::DeltesianKinematics,
        crate::kinematics::none::NoneKinematics,
        crate::kinematics::polar::PolarKinematics,
        crate::kinematics::rotary_delta::RotaryDeltaKinematics,
        crate::kinematics::winch::WinchKinematics,
        crate::extras::trad_rack::TradRackKinematics
    );

    kin_class!(r, "klippy.kinematics.cartesian.CartKinematics");
    kin_class!(r, "klippy.kinematics.corexy.CoreXYKinematics");
    kin_class!(r, "klippy.kinematics.corexz.CoreXZKinematics");
    kin_class!(r, "klippy.kinematics.hybrid_corexy.HybridCoreXYKinematics");
    kin_class!(r, "klippy.kinematics.hybrid_corexz.HybridCoreXZKinematics");
    kin_class!(r, "klippy.kinematics.limited_cartesian.LimitedCartKinematics");
    kin_class!(r, "klippy.kinematics.limited_corexy.LimitedCoreXYKinematics");
    kin_class!(r, "klippy.kinematics.limited_corexz.LimitedCoreXZKinematics");
    kin_class!(r, "klippy.kinematics.delta.DeltaKinematics");
    kin_class!(r, "klippy.kinematics.deltesian.DeltesianKinematics");
    kin_class!(r, "klippy.kinematics.none.NoneKinematics");
    // min_z: minimum_z_position as python read it (an int default stays int);
    // axes_min's z keeps that value
    fn coord_z(c: &crate::pyval::PyVal) -> KResult<PyRef> {
        match c {
            crate::pyval::PyVal::NamedTuple(_, _, v) => Ok(from_pyval(&v[2])),
            _ => Err(KError::Other("kaliru: axes_min is not a Coord".into())),
        }
    }
    r.attr("klippy.kinematics.delta.DeltaKinematics.min_z", |o| {
        coord_z(&objects::unwrap_as::<crate::kinematics::delta::DeltaKinematics>(o)?.axes_min)
    }, None);
    r.attr("klippy.kinematics.rotary_delta.RotaryDeltaKinematics.min_z", |o| {
        coord_z(&objects::unwrap_as::<crate::kinematics::rotary_delta::RotaryDeltaKinematics>(o)?.axes_min)
    }, None);
    // Coord(0.0, 0.0, 0.0, 0.0) in Kalico, never changed
    r.attr("klippy.kinematics.none.NoneKinematics.axes_minmax", |o| {
        objects::unwrap_as::<crate::kinematics::none::NoneKinematics>(o)?;
        coord(&[0.0; 4])
    }, None);
    kin_class!(r, "klippy.kinematics.polar.PolarKinematics");
    kin_class!(r, "klippy.kinematics.rotary_delta.RotaryDeltaKinematics");
    kin_class!(r, "klippy.kinematics.winch.WinchKinematics");
    kin_class!(r, "klippy.extras.trad_rack.TradRackKinematics");
    cart_class!(r, "klippy.kinematics.cartesian.CartKinematics");
    cart_class!(r, "klippy.kinematics.corexy.CoreXYKinematics");
    cart_class!(r, "klippy.kinematics.corexz.CoreXZKinematics");
    cart_class!(r, "klippy.kinematics.hybrid_corexy.HybridCoreXYKinematics");
    cart_class!(r, "klippy.kinematics.hybrid_corexz.HybridCoreXZKinematics");
    cart_class!(r, "klippy.kinematics.limited_cartesian.LimitedCartKinematics");
    cart_class!(r, "klippy.kinematics.limited_corexy.LimitedCoreXYKinematics");
    cart_class!(r, "klippy.kinematics.limited_corexz.LimitedCoreXZKinematics");
    load_kinematics!(r, "cartesian");
    load_kinematics!(r, "corexy");
    load_kinematics!(r, "corexz");
    load_kinematics!(r, "delta");
    load_kinematics!(r, "deltesian");
    load_kinematics!(r, "hybrid_corexy");
    load_kinematics!(r, "hybrid_corexz");
    load_kinematics!(r, "limited_cartesian");
    load_kinematics!(r, "limited_corexy");
    load_kinematics!(r, "limited_corexz");
    load_kinematics!(r, "none");
    load_kinematics!(r, "polar");
    load_kinematics!(r, "rotary_delta");
    load_kinematics!(r, "winch");

    // ---- delta / rotary delta calibration objects
    fn calibration(o: &PyRef) -> KResult<Option<Rc<dyn crate::kinematics::delta::Calibration>>> {
        Ok(kin(o)?.get_calibration())
    }
    fn calibration_to_py(c: Rc<dyn crate::kinematics::delta::Calibration>) -> KResult<PyRef> {
        let name = c.type_name();
        let any: Rc<dyn Any> = c;
        objects::wrap_any(any, name)
    }
    method(r, "klippy.kinematics.delta.DeltaKinematics.get_calibration", |a| match calibration(a.slf.as_ref().expect("method"))? {
        Some(c) => calibration_to_py(c),
        None => Ok(py_none()),
    });
    method(r, "klippy.kinematics.rotary_delta.RotaryDeltaKinematics.get_calibration", |a| match calibration(a.slf.as_ref().expect("method"))? {
        Some(c) => calibration_to_py(c),
        None => Ok(py_none()),
    });
    // the towers' absolute endstop positions (the calibration's)
    attr(r, "klippy.kinematics.delta.DeltaKinematics.abs_endstops", |o| {
        let c = calibration(o)?.ok_or_else(|| KError::Py("AttributeError".into(), "abs_endstops".into()))?;
        let any: Rc<dyn Any> = c;
        match any.downcast::<crate::kinematics::delta::DeltaCalibration>() {
            Ok(d) => d.abs_endstops.clone().to_py(),
            Err(_) => Err(KError::Py("AttributeError".into(), "abs_endstops".into())),
        }
    }, None);

    // ---- hybrid corexy / corexz: corexy_mode (bytes signs), inverted, dc_module
    fn hybrid_inverted(o: &PyRef) -> KResult<bool> {
        use crate::kinematics::cartesian::Kind;
        match cart(o)?.kind {
            Kind::HybridCoreXY { inverted } | Kind::HybridCoreXZ { inverted } => Ok(inverted),
            _ => Err(KError::Py("AttributeError".into(), "inverted".into())),
        }
    }
    fn corexy_mode(o: &PyRef) -> KResult<PyRef> {
        let (a, b) = if hybrid_inverted(o)? { ("+", "-") } else { ("-", "+") };
        let bytes = lookup("builtins", "bytes")?;
        Ok(py_tuple(vec![bytes.call(&[&py_str(a), &py_str("ascii")])?, bytes.call(&[&py_str(b), &py_str("ascii")])?]))
    }
    attr(r, "klippy.kinematics.hybrid_corexy.HybridCoreXYKinematics.inverted", |o| Ok(py_bool(hybrid_inverted(o)?)), None);
    attr(r, "klippy.kinematics.hybrid_corexz.HybridCoreXZKinematics.inverted", |o| Ok(py_bool(hybrid_inverted(o)?)), None);
    attr(r, "klippy.kinematics.hybrid_corexy.HybridCoreXYKinematics.corexy_mode", corexy_mode, None);
    attr(r, "klippy.kinematics.hybrid_corexz.HybridCoreXZKinematics.corexy_mode", corexy_mode, None);
    attr(r, "klippy.kinematics.hybrid_corexy.HybridCoreXYKinematics.dc_module", |o| cart(o)?.dc_module.clone().to_py(), None);
    attr(r, "klippy.kinematics.hybrid_corexz.HybridCoreXZKinematics.dc_module", |o| cart(o)?.dc_module.clone().to_py(), None);

    // ---- idex_modes
    {
        use crate::kinematics::idex_modes::{mode_of, DualCarriages, DualCarriagesRail};
        // {name: {"carriage_modes": [...], "axes_positions": [...]}}
        attr(r, "klippy.kinematics.idex_modes.DualCarriages.saved_states", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (name, s) in objects::unwrap_as::<DualCarriages>(o)?.saved_states.borrow().iter() {
                let e = lookup("builtins", "dict")?.call(&[])?;
                e.call_method("__setitem__", &[&py_str("carriage_modes"), &s.carriage_modes.iter().map(|m| m.to_string()).collect::<Vec<_>>().to_py()?])?;
                e.call_method("__setitem__", &[&py_str("axes_positions"), &s.axes_positions.clone().to_py()?])?;
                d.call_method("__setitem__", &[&py_str(name), &e])?;
            }
            Ok(d)
        }, None);
        method(r, "klippy.kinematics.idex_modes.DualCarriages.activate_dc_mode", |a| {
            a.this::<DualCarriages>()?.activate_dc_mode(a.arg(0).as_i64()? as usize, mode_of(&a.string(1)?))?;
            Ok(py_none())
        });
        // activate(mode, position, old_position=None)
        method(r, "klippy.kinematics.idex_modes.DualCarriagesRail.activate", |a| {
            let position: Vec<f64> = FromPy::from_py(a.arg(1))?;
            let old: Option<Vec<f64>> = if a.arg(2).is_none() { None } else { Some(FromPy::from_py(a.arg(2))?) };
            a.this::<DualCarriagesRail>()?.activate(mode_of(&a.string(0)?), &position, old.as_deref());
            Ok(py_none())
        });
    }

    // ---- rotary delta calibration (kaliru's chelper math)
    {
        use crate::kinematics::delta::Calibration;
        use crate::kinematics::rotary_delta::RotaryDeltaCalibration;
        method(r, "klippy.kinematics.rotary_delta.RotaryDeltaCalibration.get_position_from_stable", |a| {
            let sp: Vec<f64> = FromPy::from_py(a.arg(0))?;
            a.this::<RotaryDeltaCalibration>()?.get_position_from_stable(&sp).map_err(crate::kinematics::delta::math_kerror)?.to_vec().to_py()
        });
        method(r, "klippy.kinematics.rotary_delta.RotaryDeltaCalibration.calc_stable_position", |a| {
            let coord: Vec<f64> = FromPy::from_py(a.arg(0))?;
            a.this::<RotaryDeltaCalibration>()?.calc_stable_position(&coord).map_err(crate::kinematics::delta::math_kerror)?.to_py()
        });
        // chelper's ffi_lib
        attr(r, "klippy.kinematics.rotary_delta.RotaryDeltaCalibration.ffi_lib", |o| {
            objects::unwrap_as::<RotaryDeltaCalibration>(o)?;
            lookup("klippy.chelper", "get_ffi")?.call(&[])?.call_method("__getitem__", &[&py_int(1)])
        }, None);
    }
}
