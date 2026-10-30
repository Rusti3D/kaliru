// klippy/chelper: get_ffi(). Plugins call chelper through cffi (AFC's and
// Happy Hare's manual steppers: trapq_alloc / trapq_append /
// cartesian_stepper_alloc; beacon / cartographer: trapq_extract_old). Here
// ffi_lib's functions are kaliru's chelper (prelude.py FFI / FFILib), and
// its pointers the chelper objects themselves (prelude.py CData): a trapq,
// a stepper_kinematics.
use super::*;
use crate::chelper::itersolve::{self, SkRef, StepperKinematics};
use crate::chelper::trapq::{TrapQ, TrapQRef};
use crate::chelper::{kin_cartesian, kin_corexy, kin_corexz, kin_delta, kin_deltesian, kin_extruder, kin_idex, kin_polar,
    kin_rotary_delta, kin_shaper, kin_winch};
use crate::plugins::callable::py_function;

/// ffi_lib's functions: (name, argument types, result type)
const SIGS: &[(&str, &[&str], &str)] = &[
    ("trapq_alloc", &[], "struct trapq *"),
    ("trapq_free", &["struct trapq *"], "void"),
    ("trapq_append", &["struct trapq *", "double", "double", "double", "double", "double", "double", "double",
        "double", "double", "double", "double", "double", "double"], "void"),
    ("trapq_finalize_moves", &["struct trapq *", "double", "double"], "void"),
    ("trapq_set_position", &["struct trapq *", "double", "double", "double", "double"], "void"),
    ("trapq_extract_old", &["struct trapq *", "struct pull_move *", "int", "double", "double"], "int"),
    ("cartesian_stepper_alloc", &["char"], "struct stepper_kinematics *"),
    ("corexy_stepper_alloc", &["char"], "struct stepper_kinematics *"),
    ("corexz_stepper_alloc", &["char"], "struct stepper_kinematics *"),
    ("polar_stepper_alloc", &["char"], "struct stepper_kinematics *"),
    ("delta_stepper_alloc", &["double", "double", "double"], "struct stepper_kinematics *"),
    ("deltesian_stepper_alloc", &["double", "double"], "struct stepper_kinematics *"),
    ("rotary_delta_stepper_alloc", &["double", "double", "double", "double", "double"], "struct stepper_kinematics *"),
    ("winch_stepper_alloc", &["double", "double", "double"], "struct stepper_kinematics *"),
    ("extruder_stepper_alloc", &[], "struct stepper_kinematics *"),
    ("extruder_stepper_free", &["struct stepper_kinematics *"], "void"),
    ("extruder_set_pressure_advance", &["struct stepper_kinematics *", "double", "double", "double"], "void"),
    ("input_shaper_alloc", &[], "struct stepper_kinematics *"),
    ("dual_carriage_alloc", &[], "struct stepper_kinematics *"),
    ("itersolve_check_active", &["struct stepper_kinematics *", "double"], "double"),
    ("itersolve_is_active_axis", &["struct stepper_kinematics *", "char"], "int32_t"),
    ("itersolve_set_trapq", &["struct stepper_kinematics *", "struct trapq *", "double"], "void"),
    ("itersolve_get_trapq", &["struct stepper_kinematics *"], "struct trapq *"),
    ("itersolve_calc_position_from_coord", &["struct stepper_kinematics *", "double", "double", "double"], "double"),
    ("itersolve_set_position", &["struct stepper_kinematics *", "double", "double", "double"], "void"),
    ("itersolve_get_commanded_pos", &["struct stepper_kinematics *"], "double"),
    ("itersolve_get_gen_steps_pre_active", &["struct stepper_kinematics *"], "double"),
    ("itersolve_get_gen_steps_post_active", &["struct stepper_kinematics *"], "double"),
    ("trdispatch_alloc", &[], "struct trdispatch *"),
    ("trdispatch_free", &["struct trdispatch *"], "void"),
    ("trdispatch_start", &["struct trdispatch *", "uint32_t"], "void"),
    ("trdispatch_stop", &["struct trdispatch *"], "void"),
    ("get_monotonic", &[], "double"),
    ("free", &["void *"], "void"),
];

fn f(a: &[PyRef], i: usize) -> KResult<f64> {
    a[i].as_f64()
}

/// a trapq argument (None: NULL)
pub fn trapq_from_py(o: &PyRef) -> KResult<Option<TrapQRef>> {
    if o.is_none() {
        return Ok(None);
    }
    objects::unwrap_as::<RefCell<TrapQ>>(o).map(Some)
}

pub fn trapq_to_py(tq: Option<TrapQRef>) -> KResult<PyRef> {
    match tq {
        Some(t) => objects::wrap(t),
        None => Ok(py_none()),
    }
}

/// a stepper_kinematics argument (None: NULL)
pub fn sk_from_py(o: &PyRef) -> KResult<Option<SkRef>> {
    if o.is_none() {
        return Ok(None);
    }
    objects::unwrap_as::<RefCell<StepperKinematics>>(o).map(Some)
}

pub fn sk_to_py(sk: Option<SkRef>) -> KResult<PyRef> {
    match sk {
        Some(s) => objects::wrap(s),
        None => Ok(py_none()),
    }
}

fn tq(a: &[PyRef], i: usize) -> KResult<TrapQRef> {
    trapq_from_py(&a[i])?.ok_or_else(|| KError::Py("TypeError".into(), "kaliru: NULL trapq".into()))
}

fn sk(a: &[PyRef], i: usize) -> KResult<SkRef> {
    sk_from_py(&a[i])?.ok_or_else(|| KError::Py("TypeError".into(), "kaliru: NULL stepper_kinematics".into()))
}

/// a `char` argument: cffi takes a one-byte bytes object (b'x')
fn char_arg(o: &PyRef) -> KResult<char> {
    let s = if type_name(o)? == "bytes" { o.call_method("decode", &[])?.as_string()? } else { o.as_string()? };
    let mut chars = s.chars();
    match (chars.next(), chars.next()) {
        (Some(c), None) => Ok(c),
        _ => Err(KError::Py("TypeError".into(), format!("initializer for ctype 'char' must be a bytes of length 1, not {}", type_name(o)?))),
    }
}

/// ffi_lib.<name>: the function object get_ffi() hands out
pub fn lib_fn(name: &str) -> KResult<PyRef> {
    lookup("_k", "get_ffi")?.call(&[])?.call_method("__getitem__", &[&py_int(1)])?.getattr(name)
}

/// The chelper function `name` as a python callable
pub fn ffi_fn(name: &str) -> KResult<PyRef> {
    let nargs = SIGS
        .iter()
        .find(|(n, _, _)| *n == name)
        .map(|(_, args, _)| args.len())
        .ok_or_else(|| KError::Py("NotImplementedError".into(), format!("kaliru: ffi_lib.{} is not mirrored yet", name)))?;
    let name: &'static str = SIGS.iter().find(|(n, _, _)| *n == name).map(|(n, _, _)| *n).expect("listed");
    py_function(Rc::new(move |a: &[PyRef]| {
        if a.len() != nargs {
            return Err(KError::Py("TypeError".into(), format!("{}() takes {} arguments ({} given)", name, nargs, a.len())));
        }
        call(name, a)
    }))
}

fn call(name: &str, a: &[PyRef]) -> KResult<PyRef> {
    Ok(match name {
        "trapq_alloc" => objects::wrap(TrapQ::alloc())?,
        "trapq_free" | "extruder_stepper_free" | "trdispatch_free" | "free" => py_none(),
        "trdispatch_alloc" => objects::wrap(Rc::new(crate::chelper::trdispatch::Trdispatch::alloc()))?,
        "trdispatch_start" => {
            objects::unwrap_as::<crate::chelper::trdispatch::Trdispatch>(&a[0])?.start(a[1].as_i64()? as u32);
            py_none()
        }
        "trdispatch_stop" => {
            objects::unwrap_as::<crate::chelper::trdispatch::Trdispatch>(&a[0])?.stop();
            py_none()
        }
        "trapq_append" => {
            tq(a, 0)?.borrow_mut().append(
                f(a, 1)?, f(a, 2)?, f(a, 3)?, f(a, 4)?, f(a, 5)?, f(a, 6)?, f(a, 7)?, f(a, 8)?, f(a, 9)?, f(a, 10)?, f(a, 11)?,
                f(a, 12)?, f(a, 13)?,
            );
            py_none()
        }
        "trapq_finalize_moves" => {
            tq(a, 0)?.borrow_mut().finalize_moves(f(a, 1)?, f(a, 2)?);
            py_none()
        }
        "trapq_set_position" => {
            tq(a, 0)?.borrow_mut().set_position(f(a, 1)?, f(a, 2)?, f(a, 3)?, f(a, 4)?);
            py_none()
        }
        // trapq_extract_old(tq, p, max, start_time, end_time): fills p
        "trapq_extract_old" => {
            let max = a[2].as_i64()?.max(0) as usize;
            let moves = tq(a, 0)?.borrow().extract_old(max, f(a, 3)?, f(a, 4)?);
            for (i, m) in moves.iter().enumerate() {
                let p = a[1].call_method("__getitem__", &[&py_int(i as i64)])?;
                for (field, v) in [
                    ("print_time", m.print_time),
                    ("move_t", m.move_t),
                    ("start_v", m.start_v),
                    ("accel", m.accel),
                    ("start_x", m.start_x),
                    ("start_y", m.start_y),
                    ("start_z", m.start_z),
                    ("x_r", m.x_r),
                    ("y_r", m.y_r),
                    ("z_r", m.z_r),
                ] {
                    p.setattr(field, &py_float(v))?;
                }
            }
            py_int(moves.len() as i64)
        }
        "cartesian_stepper_alloc" => sk_to_py(kin_cartesian::cartesian_stepper_alloc(char_arg(&a[0])?))?,
        "corexy_stepper_alloc" => sk_to_py(kin_corexy::corexy_stepper_alloc(char_arg(&a[0])?))?,
        "corexz_stepper_alloc" => sk_to_py(kin_corexz::corexz_stepper_alloc(char_arg(&a[0])?))?,
        "polar_stepper_alloc" => sk_to_py(kin_polar::polar_stepper_alloc(char_arg(&a[0])?))?,
        "delta_stepper_alloc" => objects::wrap(kin_delta::delta_stepper_alloc(f(a, 0)?, f(a, 1)?, f(a, 2)?))?,
        "deltesian_stepper_alloc" => objects::wrap(kin_deltesian::deltesian_stepper_alloc(f(a, 0)?, f(a, 1)?))?,
        "rotary_delta_stepper_alloc" => {
            objects::wrap(kin_rotary_delta::rotary_delta_stepper_alloc(f(a, 0)?, f(a, 1)?, f(a, 2)?, f(a, 3)?, f(a, 4)?))?
        }
        "winch_stepper_alloc" => objects::wrap(kin_winch::winch_stepper_alloc(f(a, 0)?, f(a, 1)?, f(a, 2)?))?,
        "extruder_stepper_alloc" => objects::wrap(kin_extruder::extruder_stepper_alloc())?,
        "extruder_set_pressure_advance" => {
            kin_extruder::extruder_set_pressure_advance(&sk(a, 0)?, f(a, 1)?, f(a, 2)?, f(a, 3)?);
            py_none()
        }
        "input_shaper_alloc" => objects::wrap(kin_shaper::input_shaper_alloc())?,
        "dual_carriage_alloc" => objects::wrap(kin_idex::dual_carriage_alloc())?,
        "itersolve_check_active" => py_float(itersolve::check_active_time(&sk(a, 0)?.borrow(), f(a, 1)?)),
        "itersolve_is_active_axis" => py_int(itersolve::is_active_axis(&sk(a, 0)?.borrow(), char_arg(&a[1])? as u8) as i64),
        "itersolve_set_trapq" => {
            itersolve::set_trapq(&mut sk(a, 0)?.borrow_mut(), trapq_from_py(&a[1])?, f(a, 2)?);
            py_none()
        }
        "itersolve_get_trapq" => trapq_to_py(itersolve::get_trapq(&sk(a, 0)?.borrow()))?,
        "itersolve_calc_position_from_coord" => {
            py_float(itersolve::calc_position_from_coord(&sk(a, 0)?.borrow(), f(a, 1)?, f(a, 2)?, f(a, 3)?))
        }
        "itersolve_set_position" => {
            itersolve::set_position(&mut sk(a, 0)?.borrow_mut(), f(a, 1)?, f(a, 2)?, f(a, 3)?);
            py_none()
        }
        "itersolve_get_commanded_pos" => py_float(itersolve::get_commanded_pos(&sk(a, 0)?.borrow())),
        "itersolve_get_gen_steps_pre_active" => py_float(itersolve::get_gen_steps_pre_active(&sk(a, 0)?.borrow())),
        "itersolve_get_gen_steps_post_active" => py_float(itersolve::get_gen_steps_post_active(&sk(a, 0)?.borrow())),
        "get_monotonic" => py_float(crate::chelper::pyhelper::get_monotonic()),
        _ => unreachable!("listed in SIGS"),
    })
}

pub fn register(r: &mut Registry) {
    objects::register_class::<RefCell<TrapQ>>("_k", "CData");
    objects::register_class::<RefCell<StepperKinematics>>("_k", "CData");
    r.method("klippy.chelper.get_ffi", |_a| lookup("_k", "get_ffi")?.call(&[]));
    r.ext("ffi_fn", |a| ffi_fn(&a.string(0)?));
    r.ext("ffi_sig", |a| {
        let name = a.string(0)?;
        let (_, args, result) = SIGS
            .iter()
            .find(|(n, _, _)| *n == name)
            .ok_or_else(|| KError::Py("NotImplementedError".into(), format!("kaliru: ffi_lib.{} is not mirrored yet", name)))?;
        Ok(py_tuple(vec![py_list(args.iter().map(|s| py_str(s)).collect()), py_str(result)]))
    });
    r.ext("cdata_type", |a| {
        let o = objects::unwrap(a.arg(0))?;
        Ok(py_str(if o.is::<RefCell<TrapQ>>() {
            "struct trapq *"
        } else if o.is::<crate::chelper::steppersync::SyncEmitter>() {
            "struct syncemitter *"
        } else if o.is::<crate::chelper::trdispatch::Trdispatch>() {
            "struct trdispatch *"
        } else if o.is::<crate::plugins::impls::mcu::CqHandle>() {
            "struct command_queue *"
        } else if o.is::<RefCell<crate::chelper::steppersync::StepperSync>>() {
            "struct steppersync *"
        } else {
            "struct stepper_kinematics *"
        }))
    });

    // run_hub_ctrl(enable_power): the hub-ctrl tool through sudo
    r.method("klippy.chelper.run_hub_ctrl", |a| {
        crate::chelper::hub_ctrl::run_hub_ctrl(a.arg(0).as_i64()?);
        Ok(py_none())
    });
}
