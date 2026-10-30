// klippy/extras/indx: the toolboard heater's settings and commands, and the
// toolboard the helper objects belong to (kaliru finds it by its mcu)
use super::*;
use crate::extras::indx::heater::{IndxBracketTempSensor, IndxToolboardHeater};
use crate::extras::indx::toolboard::IndxToolboard;
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;
use crate::mcu::Mcu;

/// the [indx] toolboard on this mcu
fn toolboard_of(printer: &Printer, mcu: &Rc<Mcu>) -> KResult<PyRef> {
    for (_, obj) in printer.lookup_objects(None) {
        if let Some(tb) = crate::printer::downcast_rc::<IndxToolboard>(obj) {
            if Rc::ptr_eq(&tb.mcu, mcu) {
                return tb.to_py();
            }
        }
    }
    Ok(py_none())
}

fn heater(o: &PyRef) -> KResult<Rc<IndxToolboardHeater>> {
    objects::unwrap_as::<IndxToolboardHeater>(o)
}

fn cmd<T: 'static + Clone>(c: &Option<T>) -> KResult<PyRef> {
    match c {
        Some(c) => Rc::new(c.clone()).to_py(),
        None => Err(KError::Py("AttributeError".into(), "kaliru: looked up at build_config".into())),
    }
}

pub fn register(r: &mut Registry) {
    // pid_kp, pid_ti, pid_td, pid_b, pid_i_window, pid_i_limit
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.pid_kp", |o| Ok(py_float(heater(o)?.pid.borrow()[0])), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.pid_ti", |o| Ok(py_float(heater(o)?.pid.borrow()[1])), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.pid_td", |o| Ok(py_float(heater(o)?.pid.borrow()[2])), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.pid_b", |o| Ok(py_float(heater(o)?.pid.borrow()[3])), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.pid_i_window", |o| Ok(py_float(heater(o)?.pid.borrow()[4])), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.pid_i_limit", |o| Ok(py_float(heater(o)?.pid.borrow()[5])), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.toolboard", |o| {
        let h = heater(o)?;
        toolboard_of(&h.printer, &h.mcu)
    }, None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.vin_adc", |o| super::pins::adc_pin_to_py(heater(o)?._vin_adc.clone()), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.bracket_temp_update", |o| heater(o)?._bracket_temp_update.borrow().clone().to_py(), None);
    // (time, charge, power, extruder_pos) once reported
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.last_report", |o| heater(o)?.last_report.get().to_py(), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.cmd_debug_stream_raw_ir", |o| cmd(&heater(o)?.cmds.borrow().debug_stream_raw_ir), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.tune_coil_cmd", |o| cmd(&heater(o)?.cmds.borrow().tune_coil), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.query_coil_tune_cmd", |o| cmd(&heater(o)?.cmds.borrow().query_coil_tune), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.cmd_set_control_params", |o| cmd(&heater(o)?.cmds.borrow().set_control_params), None);
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.cmd_set_target", |o| cmd(&heater(o)?.cmds.borrow().set_target), None);
    // the heater's own set_temp (python wraps heater.set_temp with a check)
    r.attr("klippy.extras.indx.heater.IndxToolboardHeater.heater_raw_set_temp", |o| {
        let h = heater(o)?.heater.borrow().clone().ok_or_else(|| KError::Py("AttributeError".into(), "heater_raw_set_temp".into()))?;
        py_function(Rc::new(move |a: &[PyRef]| {
            h.set_temp(a[0].as_f64()?)?;
            Ok(py_none())
        }))
    }, None);
    r.attr("klippy.extras.indx.heater.IndxBracketTempSensor.heater", |o| objects::unwrap_as::<IndxBracketTempSensor>(o)?.heater.borrow().upgrade().to_py(), None);
    r.attr("klippy.extras.indx.heater.IndxBracketTempSensor.toolboard", |o| {
        let s = objects::unwrap_as::<IndxBracketTempSensor>(o)?;
        let p = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
        toolboard_of(&p, &s.mcu)
    }, None);
    r.attr("klippy.extras.indx.heatsink_fan.IndxHeatsinkFan.toolboard", |o| {
        let f = objects::unwrap_as::<crate::extras::indx::heatsink_fan::IndxHeatsinkFan>(o)?;
        toolboard_of(&f.printer, &f.mcu)
    }, None);
    r.attr("klippy.extras.indx.toolboard.IndxToolboardPins.ppins", |o| {
        objects::unwrap_as::<crate::extras::indx::toolboard::IndxToolboardPins>(o)?;
        printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup_object("pins")?.to_py()
    }, None);
    r.method("klippy.extras.indx.toolboard.IndxToolboardPins.setup_pin", |a| {
        use crate::pins::PinChip;
        let pin_type = a.string(0)?;
        let params = super::pins::pin_params_from_py(a.arg(1))?;
        let pin = a.this::<crate::extras::indx::toolboard::IndxToolboardPins>()?.setup_pin(&pin_type, &params).map_err(KError::Pins)?;
        super::pins::pin_to_py(pin, &pin_type)
    });
    // the toolboard whose dock measurement this is
    r.attr("klippy.extras.indx.toolboard.IndxDockMeasurement.toolboard", |o| {
        let d = objects::unwrap_as::<crate::extras::indx::toolboard::IndxDockMeasurement>(o)?;
        for (_, obj) in d.printer.lookup_objects(None) {
            if let Some(tb) = crate::printer::downcast_rc::<IndxToolboard>(obj) {
                if tb.dock_measurement.borrow().as_ref().map(|m| Rc::ptr_eq(m, &d)).unwrap_or(false) {
                    return tb.to_py();
                }
            }
        }
        Ok(py_none())
    }, None);
}
