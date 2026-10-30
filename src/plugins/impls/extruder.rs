// klippy/kinematics/extruder.py: ExtruderStepper, PrinterExtruder,
// DummyExtruder. Plugins build their own ExtruderStepper (AFC's lanes) and
// sync it to an extruder.
use super::chelper::{lib_fn, trapq_to_py};
use super::*;
use crate::configfile::ConfigWrapper;
use crate::gcode::GCodeCommand;
use crate::kinematics::extruder::{DummyExtruder, ExtruderStepper, PrinterExtruder};
use crate::toolhead::{ExtraAxis, Move};
use crate::{cell_attr, function, method, ro_attr};

fn cfg(o: &PyRef) -> KResult<Rc<ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn gcmd(o: &PyRef) -> KResult<Rc<GCodeCommand>> {
    super::gcode::gcmd_from_py(o)
}

/// check_move(move, ea_index) and calc_junction(prev_move, move, ea_index)
macro_rules! move_methods {
    ($r:expr, $class:literal, $ty:ty) => {
        $r.method(concat!($class, ".check_move"), |a| {
            let cell = objects::unwrap_as::<RefCell<Move>>(a.arg(0))?;
            let mut m = cell.borrow().clone();
            let res = ExtraAxis::check_move(&*a.this::<$ty>()?, &mut m, a.arg(1).as_i64()? as usize);
            *cell.borrow_mut() = m;
            res?;
            Ok(py_none())
        });
        $r.method(concat!($class, ".calc_junction"), |a| {
            let prev = objects::unwrap_as::<RefCell<Move>>(a.arg(0))?.borrow().clone();
            let m = objects::unwrap_as::<RefCell<Move>>(a.arg(1))?.borrow().clone();
            Ok(py_float(ExtraAxis::calc_junction(&*a.this::<$ty>()?, &prev, &m, a.arg(2).as_i64()? as usize)?))
        });
    };
}

pub fn register(r: &mut Registry) {
    // ---------------------------------------------------- ExtruderStepper
    r.ctor("klippy.kinematics.extruder.ExtruderStepper.__init__", |a| {
        let e: Rc<dyn Any> = ExtruderStepper::new(&*cfg(a.arg(0))?)?;
        Ok(e)
    });
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, _handle_connect(e) =>
        e.set_pressure_advance(e.config_pa, e.config_smooth_time));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, get_status(e, eventtime: f64) =>
        PyVal::Dict(e.get_status(eventtime)));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, find_past_position(e, print_time: f64) =>
        e.find_past_position(print_time));
    // sync_to_extruder(extruder_name): None or "" unsyncs
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, sync_to_extruder(e, extruder_name: Option<String>) =>
        e.sync_to_extruder(&extruder_name.unwrap_or_default()));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, set_rotation_distance(e, rotation_dist: f64) =>
        e.set_rotation_distance(rotation_dist));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, get_rotation_distance(e) => e.get_rotation_distance());
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, _set_pressure_advance(e, pressure_advance: f64, smooth_time: f64) =>
        e.set_pressure_advance(pressure_advance, smooth_time));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, cmd_default_SET_PRESSURE_ADVANCE(e, g: PyRef) =>
        e.cmd_default_set_pressure_advance(&*gcmd(&g)?));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, cmd_SET_PRESSURE_ADVANCE(e, g: PyRef) =>
        e.cmd_set_pressure_advance(&*gcmd(&g)?));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, cmd_SET_E_ROTATION_DISTANCE(e, g: PyRef) =>
        e.cmd_set_e_rotation_distance(&*gcmd(&g)?));
    method!(r, "klippy.kinematics.extruder.ExtruderStepper", ExtruderStepper, cmd_SYNC_EXTRUDER_MOTION(e, g: PyRef) =>
        e.cmd_sync_extruder_motion(&*gcmd(&g)?));
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.config_pa", ExtruderStepper, |e| e.config_pa);
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.config_smooth_time", ExtruderStepper, |e| e.config_smooth_time);
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.name", ExtruderStepper, |e| e.name.clone());
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.per_move_pressure_advance", ExtruderStepper, |e| e.per_move_pressure_advance);
    cell_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.pressure_advance", ExtruderStepper, pressure_advance);
    cell_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.pressure_advance_smooth_time", ExtruderStepper, pressure_advance_smooth_time);
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.printer", ExtruderStepper, |e| e.printer.clone());
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.stepper", ExtruderStepper, |e| e.stepper.clone());
    ro_attr!(r, "klippy.kinematics.extruder.ExtruderStepper.sk_extruder", ExtruderStepper, |e| e.sk_extruder.clone());
    r.attr("klippy.kinematics.extruder.ExtruderStepper.motion_queue", |o| {
        Ok(match objects::unwrap_as::<ExtruderStepper>(o)?.motion_queue.borrow().as_deref() {
            Some(n) => py_str(n),
            None => py_none(),
        })
    }, None);

    // ---------------------------------------------------- PrinterExtruder
    r.ctor("klippy.kinematics.extruder.PrinterExtruder.__init__", |a| {
        let e: Rc<dyn Any> = PrinterExtruder::new(&*cfg(a.arg(0))?, a.arg(1).as_i64()? as usize)?;
        Ok(e)
    });
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, get_name(e) => e.name.clone());
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, get_heater(e) => e.heater.clone());
    r.method("klippy.kinematics.extruder.PrinterExtruder.get_trapq", |a| trapq_to_py(Some(a.this::<PrinterExtruder>()?.get_trapq_ref())));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, find_past_position(e, print_time: f64) =>
        ExtraAxis::find_past_position(&*e, print_time));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, get_axis_gcode_id(e) => ExtraAxis::get_axis_gcode_id(&*e));
    // cmd_M104(gcmd, wait=False)
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, cmd_M104(e, g: PyRef, wait: bool) => e.cmd_m104(&*gcmd(&g)?, wait));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, cmd_M109(e, g: PyRef) => e.cmd_m104(&*gcmd(&g)?, true));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, cmd_M302(e, g: PyRef) => e.cmd_m302(&*gcmd(&g)?));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, cmd_ACTIVATE_EXTRUDER(e, g: PyRef) =>
        e.cmd_activate_extruder(&*gcmd(&g)?));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, cmd_default_SET_NOZZLE_DIAMETER(e, g: PyRef) =>
        e.cmd_default_set_nozzle_diameter(&*gcmd(&g)?));
    method!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder, cmd_SET_NOZZLE_DIAMETER(e, g: PyRef) =>
        e.cmd_set_nozzle_diameter(&*gcmd(&g)?));
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.name", PrinterExtruder, |e| e.name.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.heater", PrinterExtruder, |e| e.heater.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.extruder_stepper", PrinterExtruder, |e| e.extruder_stepper.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.filament_area", PrinterExtruder, |e| e.filament_area);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.config_max_cross_section", PrinterExtruder, |e| e.config_max_cross_section);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_e_velocity", PrinterExtruder, |e| e.max_e_velocity);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_e_accel", PrinterExtruder, |e| e.max_e_accel);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_e_dist", PrinterExtruder, |e| e.max_e_dist);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.instant_corner_v", PrinterExtruder, |e| e.instant_corner_v);
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.printer", PrinterExtruder, |e| e.printer.clone());
    ro_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.motion_queuing", PrinterExtruder, |e| {
        e.printer.lookup::<crate::extras::motion_queuing::PrinterMotionQueuing>("motion_queuing")
    });
    r.attr("klippy.kinematics.extruder.PrinterExtruder.trapq", |o| trapq_to_py(Some(objects::unwrap_as::<PrinterExtruder>(o)?.get_trapq_ref())), None);
    // self.trapq_append = ffi_lib.trapq_append
    r.attr("klippy.kinematics.extruder.PrinterExtruder.trapq_append", |_o| lib_fn("trapq_append"), None);
    cell_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.last_position", PrinterExtruder, last_position);
    cell_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.nozzle_diameter", PrinterExtruder, nozzle_diameter);
    cell_attr!(r, "klippy.kinematics.extruder.PrinterExtruder.max_extrude_ratio", PrinterExtruder, max_extrude_ratio);

    // ------------------------------------------------------ DummyExtruder
    r.ctor("klippy.kinematics.extruder.DummyExtruder.__init__", |a| {
        let printer: Rc<Printer> = objects::unwrap_as::<Printer>(a.arg(0))?;
        let e: Rc<dyn Any> = Rc::new(DummyExtruder::new(&printer));
        Ok(e)
    });
    method!(r, "klippy.kinematics.extruder.DummyExtruder", DummyExtruder, find_past_position(e, print_time: f64) =>
        ExtraAxis::find_past_position(&*e, print_time));
    method!(r, "klippy.kinematics.extruder.DummyExtruder", DummyExtruder, get_name(e) => ExtraAxis::get_name(&*e));
    method!(r, "klippy.kinematics.extruder.DummyExtruder", DummyExtruder, get_heater(e) => ExtraAxis::get_heater(&*e));
    r.method("klippy.kinematics.extruder.DummyExtruder.get_trapq", |_a| Ok(py_none()));
    method!(r, "klippy.kinematics.extruder.DummyExtruder", DummyExtruder, get_axis_gcode_id(e) => ExtraAxis::get_axis_gcode_id(&*e));
    ro_attr!(r, "klippy.kinematics.extruder.DummyExtruder.printer", DummyExtruder, |e| e._printer.clone());

    // check_move / calc_junction / process_move(..., move, ea_index): the
    // python Move (impls/toolhead.rs) is the move kaliru works on
    move_methods!(r, "klippy.kinematics.extruder.PrinterExtruder", PrinterExtruder);
    move_methods!(r, "klippy.kinematics.extruder.DummyExtruder", DummyExtruder);
    r.method("klippy.kinematics.extruder.PrinterExtruder.process_move", |a| {
        let m = objects::unwrap_as::<RefCell<Move>>(a.arg(1))?;
        let m = m.borrow().clone();
        ExtraAxis::process_move(&*a.this::<PrinterExtruder>()?, a.f64(0)?, &m, a.arg(2).as_i64()? as usize)?;
        Ok(py_none())
    });

    function!(r, "klippy.kinematics.extruder", add_printer_objects(config: PyRef) =>
        crate::kinematics::extruder::add_printer_objects(&*cfg(&config)?));
}
