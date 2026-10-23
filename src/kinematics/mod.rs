// Ports of klippy/kinematics
//
// python loads the kinematics module named by [printer] kinematics with
// importlib; KINEMATICS lists the ported ones (and the known unported ones,
// which raise like a load failure).
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::pyval::PyVal;
use crate::extras::homing::Homing;
use crate::stepper::{McuStepper, RailRef};
use crate::toolhead::{Move, ToolHead};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::Cell;
use std::rc::Rc;

pub mod cartesian;
pub mod extruder;
pub mod none;

/// what toolhead.py uses of a kinematics object
pub trait Kinematics {
    fn get_steppers(&self) -> Vec<Rc<McuStepper>>;
    /// calc_position(stepper_positions) with positions by stepper name
    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> Vec<f64>;
    fn check_move(&self, mv: &mut Move) -> KResult<()>;
    fn home(&self, homing_state: &Rc<Homing>) -> KResult<()>;
    /// self.rails (python raises AttributeError for kinematics without it)
    fn rails(&self) -> Vec<RailRef> {
        vec![]
    }
    fn set_position(&self, newpos: &[f64], homing_axes: &str) -> KResult<()>;
    fn clear_homing_state(&self, clear_axes: &str);
    fn get_status(&self, eventtime: f64) -> IndexMap<String, PyVal>;
    /// self.supports_dual_carriage
    fn supports_dual_carriage(&self) -> bool {
        false
    }
    /// the kinematics' max_x_velocity / max_x_accel / ... attribute, None
    /// where python's hasattr() is false
    fn limit(&self, _name: &str) -> Option<&Cell<f64>> {
        None
    }
    fn as_any(&self) -> &dyn Any;
}

pub type LoadKinematics = fn(&Rc<ToolHead>, &ConfigWrapper) -> KResult<Rc<dyn Kinematics>>;

/// klippy/kinematics/*.py that have load_kinematics(), None where not
/// ported yet
pub static KINEMATICS: &[(&str, Option<LoadKinematics>)] = &[
    ("cartesian", Some(cartesian::load_kinematics)),
    ("corexy", None),
    ("corexz", None),
    ("delta", None),
    ("deltesian", None),
    ("hybrid_corexy", None),
    ("hybrid_corexz", None),
    ("limited_cartesian", None),
    ("limited_corexy", None),
    ("limited_corexz", None),
    ("none", Some(none::load_kinematics)),
    ("polar", None),
    ("rotary_delta", None),
    ("winch", None),
];
