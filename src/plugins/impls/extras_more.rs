// klippy/extras: Kalico attributes and methods of the smaller modules that
// kaliru keeps as fields / closures (bulk_sensor, accelerometers, buttons,
// belay, endstop_phase, dockable_probe, ...): python sees Kalico's values
use super::*;
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;
use crate::{method, ro_attr};

fn no_attr(name: &str) -> KError {
    KError::Py("AttributeError".into(), name.into())
}

/// a button callback(eventtime, state) as a python callable
fn button_cb_to_py(cb: crate::extras::buttons::ButtonCallback) -> KResult<PyRef> {
    py_function(Rc::new(move |a: &[PyRef]| {
        cb(a[0].as_f64()?, a[1].as_i64()?)?;
        Ok(py_none())
    }))
}

/// an encoder callback(eventtime) as a python callable
fn push_cb_to_py(cb: crate::extras::buttons::PushCallback) -> KResult<PyRef> {
    py_function(Rc::new(move |a: &[PyRef]| {
        cb(a[0].as_f64()?)?;
        Ok(py_none())
    }))
}

/// a FixedFreqReader as python sees it
fn ffreader(r: &Rc<crate::extras::bulk_sensor::FixedFreqReader>) -> KResult<PyRef> {
    r.clone().to_py()
}

/// an accelerometer axes_map: [(axis index, scale)] * 3
fn axes_map(m: &crate::extras::adxl345::AxisMap) -> KResult<PyRef> {
    m.iter().map(|&(i, s)| (i as i64, s)).collect::<Vec<_>>().to_py()
}

pub fn register(r: &mut Registry) {
    // ---- bulk_sensor
    {
        use crate::extras::bulk_sensor::{BatchBulkHelper, BulkDataQueue, ClockSyncRegression, FixedFreqReader};
        r.attr("klippy.extras.bulk_sensor.BatchBulkHelper.batch_cb", |o| {
            let cb = objects::unwrap_as::<BatchBulkHelper>(o)?.batch_cb.clone();
            py_function(Rc::new(move |a: &[PyRef]| Ok(from_pyval(&cb(a[0].as_f64()?)?))))
        }, None);
        // (a no-op when none was given)
        fn start_stop(cb: Option<crate::extras::bulk_sensor::StartStopCallback>) -> KResult<PyRef> {
            py_function(Rc::new(move |_a: &[PyRef]| {
                if let Some(cb) = &cb {
                    cb()?;
                }
                Ok(py_none())
            }))
        }
        r.attr("klippy.extras.bulk_sensor.BatchBulkHelper.start_cb", |o| start_stop(objects::unwrap_as::<BatchBulkHelper>(o)?.start_cb.clone()), None);
        r.attr("klippy.extras.bulk_sensor.BatchBulkHelper.stop_cb", |o| start_stop(objects::unwrap_as::<BatchBulkHelper>(o)?.stop_cb.clone()), None);
        // [client_cb(msg) -> keep receiving] (a snapshot)
        r.attr("klippy.extras.bulk_sensor.BatchBulkHelper.client_cbs", |o| {
            let cbs = objects::unwrap_as::<BatchBulkHelper>(o)?.client_cbs.borrow().clone();
            let items: KResult<Vec<PyRef>> = cbs
                .into_iter()
                .map(|cb| py_function(Rc::new(move |a: &[PyRef]| Ok(py_bool(cb(&to_pyval(&a[0])?))))))
                .collect();
            Ok(py_list(items?))
        }, None);
        // the queued sensor_bulk_data params (a snapshot)
        r.attr("klippy.extras.bulk_sensor.BulkDataQueue.raw_samples", |o| {
            let q = objects::unwrap_as::<BulkDataQueue>(o)?;
            let items: KResult<Vec<PyRef>> = q.raw_samples.borrow().iter().map(super::mcu::params_to_py).collect();
            Ok(py_list(items?))
        }, None);
        r.attr("klippy.extras.bulk_sensor.ClockSyncRegression.last_mcu_clock", |o| {
            objects::unwrap_as::<ClockSyncRegression>(o)?.last_mcu_clock.get().ok_or_else(|| no_attr("last_mcu_clock"))?.to_py()
        }, None);
        r.method("klippy.extras.bulk_sensor.FixedFreqReader.setup_query_command", |a| {
            let cq = super::mcu::cq_from_py(a.arg(2))?;
            a.this::<FixedFreqReader>()?.setup_query_command(&a.string(0)?, a.arg(1).as_i64()?, cq)?;
            Ok(py_none())
        });
        // ([(ptime, field, ...)], overflows)
        r.method("klippy.extras.bulk_sensor.FixedFreqReader.pull_samples", |a| {
            let (samples, overflows) = a.this::<FixedFreqReader>()?.pull_samples()?;
            let samples = py_list(samples
                .into_iter()
                .map(|(t, fields)| py_tuple(std::iter::once(py_float(t)).chain(fields.into_iter().map(py_int)).collect()))
                .collect());
            Ok(py_tuple(vec![samples, py_int(overflows)]))
        });
        r.attr("klippy.extras.bulk_sensor.FixedFreqReader.bulk_queue", |o| {
            objects::unwrap_as::<FixedFreqReader>(o)?.bulk_queue.borrow().clone().to_py()
        }, None);
        ro_attr!(r, "klippy.extras.bulk_sensor.FixedFreqReader.clock_sync", FixedFreqReader, |s| s.clock_sync.clone());
        // struct.Struct(fmt).unpack_from
        r.attr("klippy.extras.bulk_sensor.FixedFreqReader.unpack_from", |o| {
            let fmt = objects::unwrap_as::<FixedFreqReader>(o)?.unpack_fmt.clone();
            lookup("struct", "Struct")?.call(&[&py_str(&fmt)])?.getattr("unpack_from")
        }, None);
    }

    // ---- accelerometers
    {
        use crate::extras::adxl345::{AccelCommandHelper, AccelQueryHelper, ADXL345};
        // [Accel_Measurement(time, accel_x, accel_y, accel_z)]
        fn measurements(v: Vec<crate::extras::adxl345::AccelMeasurement>) -> KResult<PyRef> {
            let nt = lookup("klippy.extras.adxl345", "Accel_Measurement")?;
            let items: KResult<Vec<PyRef>> =
                v.iter().map(|m| nt.call(&[&py_float(m.time), &py_float(m.accel_x), &py_float(m.accel_y), &py_float(m.accel_z)])).collect();
            Ok(py_list(items?))
        }
        r.method("klippy.extras.adxl345.AccelQueryHelper.get_samples", |a| measurements(a.this::<AccelQueryHelper>()?.get_samples()));
        r.attr("klippy.extras.adxl345.AccelQueryHelper.samples", |o| {
            measurements(objects::unwrap_as::<AccelQueryHelper>(o)?.samples.borrow().clone())
        }, None);
        r.attr("klippy.extras.adxl345.AccelCommandHelper.chip", |o| {
            match objects::unwrap_as::<AccelCommandHelper>(o)?.chip.borrow().upgrade() {
                Some(c) => {
                    let name = c.type_name();
                    let any: Rc<dyn Any> = c;
                    objects::wrap_any(any, name)
                }
                None => Ok(py_none()),
            }
        }, None);
        r.attr("klippy.extras.adxl345.ADXL345.axes_map", |o| axes_map(&objects::unwrap_as::<ADXL345>(o)?.axes_map), None);
        r.attr("klippy.extras.adxl345.ADXL345.ffreader", |o| ffreader(&objects::unwrap_as::<ADXL345>(o)?.ffreader), None);
        use crate::extras::icm20948::ICM20948;
        r.attr("klippy.extras.icm20948.ICM20948.axes_map", |o| axes_map(&objects::unwrap_as::<ICM20948>(o)?.axes_map), None);
        r.attr("klippy.extras.icm20948.ICM20948.ffreader", |o| ffreader(&objects::unwrap_as::<ICM20948>(o)?.ffreader), None);
        use crate::extras::mpu9250::MPU9250;
        r.attr("klippy.extras.mpu9250.MPU9250.axes_map", |o| axes_map(&objects::unwrap_as::<MPU9250>(o)?.axes_map), None);
        r.attr("klippy.extras.mpu9250.MPU9250.ffreader", |o| ffreader(&objects::unwrap_as::<MPU9250>(o)?.ffreader), None);
        use crate::extras::lis2dw::{Bus, LIS2DW};
        r.attr("klippy.extras.lis2dw.LIS2DW.axes_map", |o| axes_map(&objects::unwrap_as::<LIS2DW>(o)?.axes_map), None);
        r.attr("klippy.extras.lis2dw.LIS2DW.ffreader", |o| ffreader(&objects::unwrap_as::<LIS2DW>(o)?.ffreader), None);
        r.attr("klippy.extras.lis2dw.LIS2DW.bus", |o| match &objects::unwrap_as::<LIS2DW>(o)?.bus {
            Bus::Spi(b) => b.clone().to_py(),
            Bus::I2c(b) => b.clone().to_py(),
        }, None);
        // SPI_SERIAL_TYPE / I2C_SERIAL_TYPE
        r.attr("klippy.extras.lis2dw.LIS2DW.bus_type", |o| match &objects::unwrap_as::<LIS2DW>(o)?.bus {
            Bus::Spi(_) => Ok(py_str("spi")),
            Bus::I2c(_) => Ok(py_str("i2c")),
        }, None);
        r.attr("klippy.extras.ldc1612.LDC1612.ffreader", |o| ffreader(&objects::unwrap_as::<crate::extras::ldc1612::LDC1612>(o)?.ffreader), None);
    }

    // ---- buttons: the registered callbacks (snapshots)
    {
        use crate::extras::buttons::{DebounceButton, McuAdcButtons, McuButtons, RotaryEncoder};
        r.attr("klippy.extras.buttons.MCU_buttons.callbacks", |o| {
            let cbs = objects::unwrap_as::<McuButtons>(o)?.callbacks.borrow().clone();
            let items: KResult<Vec<PyRef>> = cbs
                .into_iter()
                .map(|(mask, shift, cb)| Ok(py_tuple(vec![py_int(mask), py_int(shift), button_cb_to_py(cb)?])))
                .collect();
            Ok(py_list(items?))
        }, None);
        r.attr("klippy.extras.buttons.MCU_ADC_buttons.buttons", |o| {
            let bs = objects::unwrap_as::<McuAdcButtons>(o)?.buttons.borrow().clone();
            let items: KResult<Vec<PyRef>> = bs
                .into_iter()
                .map(|(min, max, cb)| Ok(py_tuple(vec![py_float(min), py_float(max), button_cb_to_py(cb)?])))
                .collect();
            Ok(py_list(items?))
        }, None);
        r.attr("klippy.extras.buttons.MCU_ADC_buttons.mcu_adc", |o| super::pins::adc_pin_to_py(objects::unwrap_as::<McuAdcButtons>(o)?.mcu_adc.clone()), None);
        ro_attr!(r, "klippy.extras.buttons.MCU_ADC_buttons.pin", McuAdcButtons, |s| s.pin.clone());
        r.attr("klippy.extras.buttons.BaseRotaryEncoder.cw_callback", |o| push_cb_to_py(objects::unwrap_as::<RotaryEncoder>(o)?.cw_callback.clone()), None);
        r.attr("klippy.extras.buttons.BaseRotaryEncoder.ccw_callback", |o| push_cb_to_py(objects::unwrap_as::<RotaryEncoder>(o)?.ccw_callback.clone()), None);
        r.attr("klippy.extras.buttons.DebounceButton.button_action", |o| button_cb_to_py(objects::unwrap_as::<DebounceButton>(o)?.button_action.clone()), None);
    }

    // ---- belay
    {
        use crate::extras::belay::{Belay, BelayType, SetMultiplier};
        // [lambda: self.enabled] / [lambda: not self.enabled]
        fn condition(o: &PyRef, want: bool) -> KResult<PyRef> {
            let b = objects::unwrap_as::<Belay>(o)?;
            let f = py_function(Rc::new(move |_a: &[PyRef]| Ok(py_bool(b.enabled.get() == want))))?;
            Ok(py_list(vec![f]))
        }
        r.attr("klippy.extras.belay.Belay.disable_conditions", |o| condition(o, true), None);
        r.attr("klippy.extras.belay.Belay.enable_conditions", |o| condition(o, false), None);
        r.attr("klippy.extras.belay.Belay.set_multiplier", |o| {
            match objects::unwrap_as::<Belay>(o)?.set_multiplier.borrow().clone() {
                Some(SetMultiplier::TradRack(tr)) => tr.to_py()?.getattr("set_fil_driver_multiplier"),
                Some(SetMultiplier::Stepper(stepper, base)) => py_function(Rc::new(move |a: &[PyRef]| {
                    stepper.set_rotation_distance(base / a[0].as_f64()?);
                    Ok(py_none())
                })),
                None => Ok(py_none()),
            }
        }, None);
        ro_attr!(r, "klippy.extras.belay.Belay.type", Belay, |s| match s.kind {
            BelayType::TradRack => "trad_rack",
            BelayType::ExtruderStepper => "extruder_stepper",
        });
    }

    // ---- probes: wait_for_trigger(eventtime)
    method!(r, "klippy.extras.bltouch.BLTouchEndstopWrapper", crate::extras::bltouch::BLTouchEndstopWrapper, wait_for_trigger(s, _eventtime: PyRef) => s.wait_for_trigger());
    method!(r, "klippy.extras.dockable_probe.DockableProbe", crate::extras::dockable_probe::DockableProbe, wait_for_trigger(s, _eventtime: PyRef) => s.wait_for_trigger());
    {
        use crate::extras::dockable_probe::{PinQuery, ProbeState};
        // helper.query_pin / helper.query_pin_inv
        fn pin_query(q: &PinQuery) -> KResult<PyRef> {
            q.helper.clone().to_py()?.getattr(if q.inverted { "query_pin_inv" } else { "query_pin" })
        }
        r.attr("klippy.extras.dockable_probe.ProbeState.probe_sense_pin", |o| pin_query(&objects::unwrap_as::<ProbeState>(o)?.probe_sense_pin), None);
        r.attr("klippy.extras.dockable_probe.ProbeState.dock_sense_pin", |o| match &objects::unwrap_as::<ProbeState>(o)?.dock_sense_pin {
            Some(q) => pin_query(q),
            None => Ok(py_none()),
        }, None);
    }

    // ---- canbus_stats / control_mpc / delayed_gcode / dotstar / fans / gcode_button / shell
    r.attr("klippy.extras.canbus_stats.PrinterCANBusStats.status", |o| {
        let s = objects::unwrap_as::<crate::extras::canbus_stats::PrinterCANBusStats>(o)?;
        Ok(from_pyval(&crate::printer::PrinterObject::get_status(&*s, 0.0).unwrap_or(PyVal::None)))
    }, None);
    // (set to 0.0 in __init__ and never updated)
    r.attr("klippy.extras.control_mpc.ControlMPC.last_time", |_o| Ok(py_float(0.0)), None);
    ro_attr!(r, "klippy.extras.delayed_gcode.DelayedGcode.name", crate::extras::delayed_gcode::DelayedGcode, |s| s.name.clone());
    ro_attr!(r, "klippy.extras.delayed_gcode.DelayedGcode.cmd_desc", crate::extras::delayed_gcode::DelayedGcode, |s| s.cmd_desc.clone());
    {
        use crate::extras::dotstar::PrinterDotstar;
        ro_attr!(r, "klippy.extras.dotstar.PrinterDotstar.chain_count", PrinterDotstar, |s| s.chain_count);
        // [(r, g, b, w)] once sent
        r.attr("klippy.extras.dotstar.PrinterDotstar.prev_data", |o| {
            match objects::unwrap_as::<PrinterDotstar>(o)?.prev_data.borrow().as_ref() {
                Some(d) => Ok(py_list(d.iter().map(|c| py_tuple(c.iter().map(|v| py_float(*v)).collect())).collect())),
                None => Ok(py_none()),
            }
        }, None);
    }
    // tachometer_ppr (only with a tachometer_pin)
    r.attr("klippy.extras.fan.FanTachometer.ppr", |o| {
        match &objects::unwrap_as::<crate::extras::fan::FanTachometer>(o)?.freq_counter {
            Some((_, ppr)) => Ok(py_int(*ppr)),
            None => Err(no_attr("ppr")),
        }
    }, None);
    ro_attr!(r, "klippy.extras.fan_generic.PrinterFanGeneric.fan_name", crate::extras::fan_generic::PrinterFanGeneric, |s| s.fan_name.clone());
    ro_attr!(r, "klippy.extras.gcode_button.GCodeButton.pin", crate::extras::gcode_button::GCodeButton, |s| s.pin.clone());
    ro_attr!(r, "klippy.extras.gcode_shell_command.ShellCommand.proc_fd", crate::extras::gcode_shell_command::ShellCommand, |s| s.proc_fd.get().map(|fd| fd as i64));

    // ---- endstop_phase
    {
        use crate::extras::endstop_phase::{EndstopPhase, PhaseCalc};
        method!(r, "klippy.extras.endstop_phase.PhaseCalc", PhaseCalc, calc_phase(s, _stepper: PyRef, trig_mcu_pos: i64) => s.calc_phase(trig_mcu_pos));
        method!(r, "klippy.extras.endstop_phase.EndstopPhase", EndstopPhase, get_homed_offset(s, _stepper: PyRef, trig_mcu_pos: i64) => s.get_homed_offset(trig_mcu_pos));
        ro_attr!(r, "klippy.extras.endstop_phase.EndstopPhase.endstop_accuracy", EndstopPhase, |s| s.endstop_accuracy);
    }

    // ---- filament width sensors
    {
        use crate::extras::hall_filament_width_sensor::HallFilamentWidthSensor as H;
        // round(read_value * 10000)
        method!(r, "klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor", H, adc_callback(s, _read_time: PyRef, read_value: f64) => {
            s.last_reading.set((read_value * 10000.0).round_ties_even() as i64)
        });
        method!(r, "klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor", H, adc2_callback(s, _read_time: PyRef, read_value: f64) => s.adc2_callback(read_value));
        r.method("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.cmd_ClearFilamentArray", |a| {
            a.this::<H>()?.cmd_clear_filament_array(&*super::gcode::gcmd_from_py(a.arg(0))?)?;
            Ok(py_none())
        });
        use crate::extras::tsl1401cl_filament_width_sensor::FilamentWidthSensor as T;
        // round(read_value * 5, 2)
        method!(r, "klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor", T, adc_callback(s, _read_time: PyRef, read_value: f64) => {
            s.last_filament_width_reading.set(crate::pyfmt::py_round(read_value * 5.0, 2))
        });
        r.method("klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor.cmd_ClearFilamentArray", |a| {
            a.this::<T>()?.cmd_clear_filament_array(&*super::gcode::gcmd_from_py(a.arg(0))?)?;
            Ok(py_none())
        });
        ro_attr!(r, "klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor.lastFilamentWidthReading", T, |s| s.last_filament_width_reading.get());
        r.attr("klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor.mcu_adc", |o| super::pins::adc_pin_to_py(objects::unwrap_as::<T>(o)?.mcu_adc.clone()), None);
        ro_attr!(r, "klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor.measurement_max_difference", T, |s| s.measurement_max_difference);
        ro_attr!(r, "klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor.pin", T, |s| s.pin.clone());
        r.attr("klippy.extras.tsl1401cl_filament_width_sensor.FilamentWidthSensor.ppins", |o| {
            objects::unwrap_as::<T>(o)?.printer.lookup_object("pins")?.to_py()
        }, None);
    }

    // ---- ldc1612 / led / manual_probe / multi_pin / output_pin / pca9632 / pid_calibrate
    r.attr("klippy.extras.ldc1612.DriveCurrentCalibrate.sensor", |o| {
        objects::unwrap_as::<crate::extras::ldc1612::DriveCurrentCalibrate>(o)?.sensor.borrow().upgrade().to_py()
    }, None);
    // [template update callback(text)] per led
    r.attr("klippy.extras.led.LEDHelper.tcallbacks", |o| {
        let cbs = objects::unwrap_as::<crate::extras::led::LEDHelper>(o)?.tcallbacks.borrow().clone();
        let items: KResult<Vec<PyRef>> = cbs
            .into_iter()
            .map(|cb| py_function(Rc::new(move |a: &[PyRef]| {
                cb(&a[0].as_string()?)?;
                Ok(py_none())
            })))
            .collect();
        Ok(py_list(items?))
    }, None);
    // finalize_callback(kin_pos or None)
    r.attr("klippy.extras.manual_probe.ManualProbeHelper.finalize_callback", |o| {
        let cb = objects::unwrap_as::<crate::extras::manual_probe::ManualProbeHelper>(o)?.finalize_callback.clone();
        py_function(Rc::new(move |a: &[PyRef]| {
            let pos: Option<Vec<f64>> = if a[0].is_none() { None } else { Some(crate::plugins::convert::FromPy::from_py(&a[0])?) };
            cb(pos)?;
            Ok(py_none())
        }))
    }, None);
    r.attr("klippy.extras.multi_pin.PrinterMultiPin.mcu_pins", |o| {
        use crate::extras::multi_pin::McuPins;
        let s = objects::unwrap_as::<crate::extras::multi_pin::PrinterMultiPin>(o)?;
        let pins = s.mcu_pins.borrow();
        let items: KResult<Vec<PyRef>> = match &*pins {
            McuPins::None => Ok(vec![]),
            McuPins::Pwm(v) => v.iter().map(|p| super::pins::pin_to_py(p.clone() as Rc<dyn Any>, "pwm")).collect(),
            McuPins::Digital(v) => v.iter().map(|p| super::pins::pin_to_py(p.clone() as Rc<dyn Any>, "digital_out")).collect(),
        };
        Ok(py_list(items?))
    }, None);
    // callback(print_time, value) -> None or ("discard" / "delay", value)
    r.attr("klippy.extras.output_pin.GCodeRequestQueue.callback", |o| {
        let cb = objects::unwrap_as::<crate::extras::output_pin::GCodeRequestQueue>(o)?.callback.clone();
        py_function(Rc::new(move |a: &[PyRef]| {
            match cb(a[0].as_f64()?, a[1].as_f64()?)? {
                Some((action, v)) => {
                    let name = match action {
                        crate::extras::output_pin::Action::Discard => "discard",
                        crate::extras::output_pin::Action::Delay => "delay",
                    };
                    Ok(py_tuple(vec![py_str(name), py_float(v)]))
                }
                None => Ok(py_none()),
            }
        }))
    }, None);
    // gcode_macro.create_template_context
    r.attr("klippy.extras.output_pin.PrinterTemplateEvaluator.create_template_context", |o| {
        let s = objects::unwrap_as::<crate::extras::output_pin::PrinterTemplateEvaluator>(o)?;
        s.printer.lookup_object("gcode_macro")?.to_py()?.getattr("create_template_context")
    }, None);
    r.attr("klippy.extras.pca9632.PCA9632.prev_regs", |o| {
        let regs = objects::unwrap_as::<crate::extras::pca9632::PCA9632>(o)?.prev_regs.borrow().clone();
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in regs {
            d.call_method("__setitem__", &[&py_int(k as i64), &py_int(v as i64)])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.extras.pid_calibrate.ControlAutoTune.control", |o| {
        super::heaters::control_to_py(objects::unwrap_as::<crate::extras::pid_calibrate::ControlAutoTune>(o)?.control.clone())
    }, None);

    // ---- pause_resume / pulse_counter / pwm tools
    r.method("klippy.extras.pause_resume.PauseResume.handle_connect", |a| {
        let p = a.this::<crate::extras::pause_resume::PauseResume>()?;
        *p.v_sd.borrow_mut() = p.printer.lookup_object_opt("virtual_sdcard").and_then(crate::printer::downcast_rc);
        Ok(py_none())
    });
    // setup_callback(cb): cb(time, count, count_time)
    r.method("klippy.extras.pulse_counter.MCU_counter.setup_callback", |a| {
        let cb = a.arg(0).clone();
        *a.this::<crate::extras::pulse_counter::McuCounter>()?.callback.borrow_mut() = Some(Rc::new(move |t, count, count_time| {
            cb.call(&[&py_float(t), &py_int(count), &py_float(count_time)])?;
            Ok(())
        }));
        Ok(py_none())
    });
    ro_attr!(r, "klippy.extras.pwm_cycle_time.PrinterOutputPWMCycle.shutdown_value", crate::extras::pwm_cycle_time::PrinterOutputPWMCycle, |s| s.shutdown_value);
    ro_attr!(r, "klippy.extras.pwm_tool.PrinterOutputPin.shutdown_value", crate::extras::pwm_tool::PrinterOutputPin, |s| s.shutdown_value);

    // ---- probe_eddy_current
    {
        use crate::extras::probe::ProbeDistanceCollector;
        use crate::extras::probe_eddy_current::EddyDistanceCollector as C;
        // [ProbeDistanceResult(time, distance)]
        fn results(v: Vec<(f64, f64)>) -> KResult<PyRef> {
            let nt = lookup("klippy.extras.probe", "ProbeDistanceResult")?;
            let items: KResult<Vec<PyRef>> = v.iter().map(|(t, d)| nt.call(&[&py_float(*t), &py_float(*d)])).collect();
            Ok(py_list(items?))
        }
        r.method("klippy.extras.probe_eddy_current.EddyDistanceCollector.finish", |a| results(a.this::<C>()?.finish(a.f64(0)?)?));
        r.attr("klippy.extras.probe_eddy_current.EddyDistanceCollector.samples", |o| results(objects::unwrap_as::<C>(o)?.samples.borrow().clone()), None);
    }

    // ---- probe_finalize(offsets, positions) of the probing helpers
    {
        use crate::extras::probe::FinalizeResult;
        fn args(a: &crate::plugins::native::Args) -> KResult<((f64, f64, f64), Vec<Vec<f64>>)> {
            let o: Vec<f64> = crate::plugins::convert::FromPy::from_py(a.arg(0))?;
            let positions: Vec<Vec<f64>> = crate::plugins::convert::FromPy::from_py(a.arg(1))?;
            Ok(((o[0], o[1], o[2]), positions))
        }
        // z_tilt's check_retry_result passes "done" / the error through
        fn retry_to_py(f: FinalizeResult) -> PyRef {
            match f {
                FinalizeResult::Num(n) => py_float(n),
                FinalizeResult::Retry => py_str("retry"),
                FinalizeResult::Done => py_str("done"),
            }
        }
        fn plain_to_py(f: FinalizeResult) -> PyRef {
            match f {
                FinalizeResult::Num(n) => py_float(n),
                FinalizeResult::Retry => py_str("retry"),
                FinalizeResult::Done => py_none(),
            }
        }
        r.method("klippy.extras.z_tilt.ZTilt.probe_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(retry_to_py(a.this::<crate::extras::z_tilt::ZTilt>()?.probe_finalize(o, &p)?))
        });
        r.method("klippy.extras.quad_gantry_level.QuadGantryLevel.probe_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(retry_to_py(a.this::<crate::extras::quad_gantry_level::QuadGantryLevel>()?.probe_finalize(o, &p)?))
        });
        r.method("klippy.extras.z_tilt_ng.ZTilt.probe_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(retry_to_py(a.this::<crate::extras::z_tilt_ng::ZTilt>()?.probe_finalize(o, &p)?))
        });
        r.method("klippy.extras.z_tilt_ng.ZTilt.cal_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(plain_to_py(a.this::<crate::extras::z_tilt_ng::ZTilt>()?.cal_finalize(o, &p)?))
        });
        r.method("klippy.extras.z_tilt_ng.ZTilt.ad_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(plain_to_py(a.this::<crate::extras::z_tilt_ng::ZTilt>()?.ad_finalize(o, &p)?))
        });
        r.method("klippy.extras.screws_tilt_adjust.ScrewsTiltAdjust.probe_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(plain_to_py(a.this::<crate::extras::screws_tilt_adjust::ScrewsTiltAdjust>()?.probe_finalize(o, &p)?))
        });
        r.method("klippy.extras.delta_calibrate.DeltaCalibrate.probe_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(plain_to_py(a.this::<crate::extras::delta_calibrate::DeltaCalibrate>()?.probe_finalize(o, &p)?))
        });
        r.method("klippy.extras.bed_tilt.BedTiltCalibrate.probe_finalize", |a| {
            let (o, p) = args(a)?;
            Ok(plain_to_py(a.this::<crate::extras::bed_tilt::BedTiltCalibrate>()?.probe_finalize(o, &p)?))
        });
        ro_attr!(r, "klippy.extras.z_tilt.ZTilt.retry_helper", crate::extras::z_tilt::ZTilt, |s| s.retry_helper.clone());
        ro_attr!(r, "klippy.extras.quad_gantry_level.QuadGantryLevel.retry_helper", crate::extras::quad_gantry_level::QuadGantryLevel, |s| s.retry_helper.clone());
        ro_attr!(r, "klippy.extras.z_tilt_ng.ZTilt.retry_helper", crate::extras::z_tilt_ng::ZTilt, |s| s.retry_helper.clone());
    }
    // RetryHelper.check_retry(z_positions) -> "done" / error;
    // ZAdjustStatus.check_retry_result(retry_result) -> retry_result
    {
        use crate::extras::z_tilt::{RetryHelper, RetryResult, ZAdjustStatus};
        fn check_retry(a: &crate::plugins::native::Args) -> KResult<PyRef> {
            let z: Vec<f64> = crate::plugins::convert::FromPy::from_py(&lookup("builtins", "list")?.call(&[a.arg(0)])?)?;
            Ok(match a.this::<RetryHelper>()?.check_retry(&z)? {
                RetryResult::Done => py_str("done"),
                RetryResult::Value(v) => py_float(v),
            })
        }
        fn check_retry_result(a: &crate::plugins::native::Args) -> KResult<PyRef> {
            let r = a.arg(0);
            let done = (r.is_instance(&lookup("builtins", "str")?)? && r.as_string()? == "done")
                || (r.is_instance(&lookup("builtins", "float")?)? && r.as_f64()? == 0.0);
            if done {
                a.this::<ZAdjustStatus>()?.applied.set(true);
            }
            Ok(r.clone())
        }
        r.method("klippy.extras.z_tilt.RetryHelper.check_retry", check_retry);
        r.method("klippy.extras.z_tilt_ng.RetryHelper.check_retry", check_retry);
        r.method("klippy.extras.z_tilt.ZAdjustStatus.check_retry_result", check_retry_result);
        r.method("klippy.extras.z_tilt_ng.ZAdjustStatus.check_retry_result", check_retry_result);
    }
    {
        use crate::extras::screws_tilt_adjust::ScrewsTiltAdjust;
        // [(coord, name)]
        r.attr("klippy.extras.screws_tilt_adjust.ScrewsTiltAdjust.screws", |o| {
            let s = objects::unwrap_as::<ScrewsTiltAdjust>(o)?;
            let items: Vec<PyRef> =
                s.screws.iter().map(|(c, n)| py_tuple(vec![py_tuple(vec![py_float(c[0]), py_float(c[1])]), py_str(n)])).collect();
            Ok(py_list(items))
        }, None);
        // the module's THREADS table as Kalico builds it
        r.attr("klippy.extras.screws_tilt_adjust.ScrewsTiltAdjust.threads", |_o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (i, n) in ["CW-M3", "CCW-M3", "CW-M4", "CCW-M4", "CW-M5", "CCW-M5", "CW-M6", "CCW-M6", "CW-M8", "CCW-M8"].iter().enumerate() {
                d.call_method("__setitem__", &[&py_str(n), &py_int(i as i64)])?;
            }
            Ok(d)
        }, None);
    }

    // ---- chips: setup_pin(pin_type, pin_params)
    {
        use crate::pins::PinChip;
        fn chip_setup_pin<T: PinChip>(a: &crate::plugins::native::Args) -> KResult<PyRef> {
            let pin_type = a.string(0)?;
            let params = super::pins::pin_params_from_py(a.arg(1))?;
            let pin = a.this::<T>()?.setup_pin(&pin_type, &params).map_err(KError::Pins)?;
            super::pins::pin_to_py(pin, &pin_type)
        }
        r.method("klippy.extras.sx1509.SX1509.setup_pin", chip_setup_pin::<crate::extras::sx1509::SX1509>);
        r.method("klippy.extras.replicape.Replicape.setup_pin", chip_setup_pin::<crate::extras::replicape::Replicape>);
        r.method("klippy.extras.tmc.TMCVirtualPinHelper.setup_pin", chip_setup_pin::<crate::extras::tmc::TMCVirtualPinHelper>);
        // the "multi_pin <name>" object of the pin sets it up
        r.method("klippy.extras.multi_pin.PrinterMultiPin.setup_pin", |a| {
            let pin_type = a.string(0)?;
            let params = super::pins::pin_params_from_py(a.arg(1))?;
            let name = params.borrow().pin.clone();
            let s = a.this::<crate::extras::multi_pin::PrinterMultiPin>()?;
            let target = s
                .printer
                .lookup_object_opt(&format!("multi_pin {}", name))
                .and_then(crate::printer::downcast_rc::<crate::extras::multi_pin::PrinterMultiPin>)
                .ok_or_else(|| KError::Pins(format!("multi_pin {} not configured", name)))?;
            let pin = target.setup_pin(&pin_type, &params).map_err(KError::Pins)?;
            super::pins::pin_to_py(pin, &pin_type)
        });
    }
    {
        use crate::extras::sx1509::SX1509;
        fn regs(m: &crate::IndexMap<i64, i64>) -> KResult<PyRef> {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in m {
                d.call_method("__setitem__", &[&py_int(*k), &py_int(*v)])?;
            }
            Ok(d)
        }
        r.attr("klippy.extras.sx1509.SX1509.reg_dict", |o| regs(&objects::unwrap_as::<SX1509>(o)?.reg_dict.borrow()), None);
        r.attr("klippy.extras.sx1509.SX1509.reg_i_on_dict", |o| regs(&objects::unwrap_as::<SX1509>(o)?.reg_i_on_dict.borrow()), None);
        method!(r, "klippy.extras.sx1509.SX1509", SX1509, get_oid(s) => s.i2c.get_oid());
    }
    {
        use crate::extras::replicape::{PinClass, Replicape};
        // {name: (pca9685_pwm / ReplicapeDACEnable, channel)}
        r.attr("klippy.extras.replicape.Replicape.pins", |o| {
            let s = objects::unwrap_as::<Replicape>(o)?;
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (name, (cls, ch)) in &s.pins {
                let c = lookup("klippy.extras.replicape", match cls {
                    PinClass::Pca9685 => "pca9685_pwm",
                    PinClass::DacEnable => "ReplicapeDACEnable",
                })?;
                d.call_method("__setitem__", &[&py_str(name), &py_tuple(vec![c, py_int(*ch)])])?;
            }
            Ok(d)
        }, None);
        r.attr("klippy.extras.replicape.Replicape.servo_pins", |_o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            d.call_method("__setitem__", &[&py_str("servo0"), &py_int(3)])?;
            d.call_method("__setitem__", &[&py_str("servo1"), &py_int(2)])?;
            Ok(d)
        }, None);
        r.attr("klippy.extras.replicape.Replicape.stepper_dacs", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (ch, v) in &objects::unwrap_as::<Replicape>(o)?.stepper_dacs {
                d.call_method("__setitem__", &[&py_int(*ch), &py_float(*v)])?;
            }
            Ok(d)
        }, None);
        r.attr("klippy.extras.replicape.Replicape.enabled_channels", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (ch, v) in objects::unwrap_as::<Replicape>(o)?.enabled_channels.borrow().iter() {
                d.call_method("__setitem__", &[&py_int(*ch), &py_bool(*v)])?;
            }
            Ok(d)
        }, None);
    }

    // ---- safe_z_home / samd_sercom / save_variables / skew / statistics / stepper_enable
    ro_attr!(r, "klippy.extras.safe_z_home.SafeZHoming.max_z", crate::extras::safe_z_home::SafeZHoming, |s| s.max_z);
    r.attr("klippy.extras.safe_z_home.SafeZHoming.prev_G28", |o| {
        match objects::unwrap_as::<crate::extras::safe_z_home::SafeZHoming>(o)?.prev_g28.borrow().clone() {
            Some(h) => super::gcode::handler_to_py(h),
            None => Ok(py_none()),
        }
    }, None);
    {
        use crate::extras::samd_sercom::SamdSERCOM;
        ro_attr!(r, "klippy.extras.samd_sercom.SamdSERCOM.sercom", SamdSERCOM, |s| s.sercom.clone());
        ro_attr!(r, "klippy.extras.samd_sercom.SamdSERCOM.tx_pin", SamdSERCOM, |s| s.tx_pin.clone());
        ro_attr!(r, "klippy.extras.samd_sercom.SamdSERCOM.rx_pin", SamdSERCOM, |s| s.rx_pin.clone());
        ro_attr!(r, "klippy.extras.samd_sercom.SamdSERCOM.clk_pin", SamdSERCOM, |s| s.clk_pin.clone());
        ro_attr!(r, "klippy.extras.samd_sercom.SamdSERCOM.mcu", SamdSERCOM, |s| s.mcu.clone());
    }
    method!(r, "klippy.extras.save_variables.SaveVariables", crate::extras::save_variables::SaveVariables, loadVariables(s) => s.load_variables());
    r.attr("klippy.extras.skew_correction.PrinterSkew.toolhead", |o| {
        objects::unwrap_as::<crate::extras::skew_correction::PrinterSkew>(o)?.printer.lookup_object_opt("toolhead").to_py()
    }, None);
    // python's file object over kaliru's open /proc/meminfo (same descriptor)
    r.attr("klippy.extras.statistics.PrinterSysStats.mem_file", |o| {
        use std::os::fd::AsRawFd;
        let fd = match objects::unwrap_as::<crate::extras::statistics::PrinterSysStats>(o)?.mem_file.borrow().as_ref() {
            Some(f) => f.as_raw_fd(),
            None => return Ok(py_none()),
        };
        let n = py_none();
        lookup("os", "fdopen")?.call(&[&py_int(fd as i64), &py_str("r"), &py_int(-1), &n, &n, &n, &py_bool(false)])
    }, None);
    {
        use crate::extras::stepper_enable::EnableTracking;
        // register_state_callback(callback): callback(print_time, enabled)
        r.method("klippy.extras.stepper_enable.EnableTracking.register_state_callback", |a| {
            let cb = a.arg(0).clone();
            a.this::<EnableTracking>()?.register_state_callback(Rc::new(move |t, enabled| {
                cb.call(&[&py_float(t), &py_bool(enabled)])?;
                Ok(())
            }));
            Ok(py_none())
        });
        r.attr("klippy.extras.stepper_enable.EnableTracking.callbacks", |o| {
            let cbs = objects::unwrap_as::<EnableTracking>(o)?.callbacks.borrow().clone();
            let items: KResult<Vec<PyRef>> = cbs
                .into_iter()
                .map(|cb| py_function(Rc::new(move |a: &[PyRef]| {
                    cb(a[0].as_f64()?, a[1].is_true()?)?;
                    Ok(py_none())
                })))
                .collect();
            Ok(py_list(items?))
        }, None);
    }

    // ---- tmc internals
    {
        use crate::extras::tmc::{RawRead, RegInfo, TMCErrorCheck};
        // [last_value, reg_name, mask, err_mask, cs_actual_mask]
        fn reg_info(i: &RegInfo) -> PyRef {
            py_list(vec![py_int(i.last_value), py_str(&i.reg_name), py_int(i.mask), py_int(i.err_mask), py_int(i.cs_actual_mask)])
        }
        r.attr("klippy.extras.tmc.TMCErrorCheck.drv_status_reg_info", |o| Ok(reg_info(&objects::unwrap_as::<TMCErrorCheck>(o)?.drv_status_reg_info.borrow())), None);
        r.attr("klippy.extras.tmc.TMCErrorCheck.gstat_reg_info", |o| {
            Ok(objects::unwrap_as::<TMCErrorCheck>(o)?.gstat_reg_info.borrow().as_ref().map(reg_info).unwrap_or_else(py_none))
        }, None);
        r.attr("klippy.extras.tmc.TMCErrorCheck.temp_from_adc", |o| match objects::unwrap_as::<TMCErrorCheck>(o)?.temp_from_adc {
            Some(f) => py_function(Rc::new(move |a: &[PyRef]| Ok(py_float(f(a[0].as_i64()?))))),
            None => Ok(py_none()),
        }, None);
        // {"spi_status", "data", "#receive_time"} / {"data", "#receive_time"}
        fn raw_read(r: &RawRead) -> KResult<PyRef> {
            let d = lookup("builtins", "dict")?.call(&[])?;
            if let Some(s) = r.spi_status {
                d.call_method("__setitem__", &[&py_str("spi_status"), &py_int(s)])?;
            }
            d.call_method("__setitem__", &[&py_str("data"), &py_int(r.data)])?;
            d.call_method("__setitem__", &[&py_str("#receive_time"), &py_float(r.receive_time)])?;
            Ok(d)
        }
        use crate::extras::tmc2130::{McuTmcSpi, McuTmcSpiChain};
        r.method("klippy.extras.tmc2130.MCU_TMC_SPI_chain.reg_read", |a| raw_read(&a.this::<McuTmcSpiChain>()?.reg_read(a.arg(0).as_i64()?, a.arg(1).as_i64()?)?));
        ro_attr!(r, "klippy.extras.tmc2130.MCU_TMC_SPI.mcu", McuTmcSpi, |s| s.tmc_spi.spi.get_mcu());
        ro_attr!(r, "klippy.extras.tmc2130.MCU_TMC_SPI.mutex", McuTmcSpi, |s| s.tmc_spi.mutex.clone());
        ro_attr!(r, "klippy.extras.tmc2660.MCU_TMC2660_SPI.mcu", crate::extras::tmc2660::McuTmc2660Spi, |s| s.spi.get_mcu());
        ro_attr!(r, "klippy.extras.tmc2240.TMC2240CurrentHelper.Rref", crate::extras::tmc2240::TMC2240CurrentHelper, |s| s.rref);
        ro_attr!(r, "klippy.extras.tmc2240.TMC2240CurrentHelper.current_range", crate::extras::tmc2240::TMC2240CurrentHelper, |s| s.current_range);
        use crate::extras::tmc::TmcDriver;
        ro_attr!(r, "klippy.extras.tmc5262.TMC5262.name", TmcDriver, |s| s.cmdhelper.name.clone());
        r.attr("klippy.extras.tmc5262.TMC5262.clear_mcu_phase_offset", |o| {
            objects::unwrap_as::<TmcDriver>(o)?.cmdhelper.clone().to_py()?.getattr("clear_mcu_phase_offset")
        }, None);
        // tmc_uart: instance ids are None or the select pin values
        use crate::extras::tmc_uart::{InstanceId, McuTmcUart, McuTmcUartBitbang};
        fn instance_to_py(i: &InstanceId) -> KResult<PyRef> {
            match i {
                Some(v) => Ok(py_tuple(v.iter().map(|x| py_int(*x)).collect())),
                None => Ok(py_none()),
            }
        }
        fn instance_from_py(o: &PyRef) -> KResult<InstanceId> {
            if o.is_none() {
                return Ok(None);
            }
            Ok(Some(crate::plugins::convert::FromPy::from_py(&lookup("builtins", "list")?.call(&[o])?)?))
        }
        r.attr("klippy.extras.tmc_uart.MCU_TMC_uart.instance_id", |o| instance_to_py(&objects::unwrap_as::<McuTmcUart>(o)?.instance_id), None);
        ro_attr!(r, "klippy.extras.tmc_uart.MCU_TMC_uart.mcu", McuTmcUart, |s| s.mcu_uart.mcu.clone());
        ro_attr!(r, "klippy.extras.tmc_uart.MCU_TMC_uart.mutex", McuTmcUart, |s| s.mcu_uart.mutex.clone());
        r.method("klippy.extras.tmc_uart.MCU_TMC_uart_bitbang.reg_read", |a| {
            let (data, receive_time) = a.this::<McuTmcUartBitbang>()?.reg_read(&instance_from_py(a.arg(0))?, a.arg(1).as_i64()?, a.arg(2).as_i64()?)?;
            let d = lookup("builtins", "dict")?.call(&[])?;
            d.call_method("__setitem__", &[&py_str("data"), &data.to_py()?])?;
            d.call_method("__setitem__", &[&py_str("#receive_time"), &py_float(receive_time)])?;
            Ok(d)
        });
        r.method("klippy.extras.tmc_uart.MCU_TMC_uart_bitbang.reg_write", |a| {
            let print_time = if a.arg(4).is_none() { None } else { Some(a.f64(4)?) };
            a.this::<McuTmcUartBitbang>()?.reg_write(&instance_from_py(a.arg(0))?, a.arg(1).as_i64()?, a.arg(2).as_i64()?, a.arg(3).as_i64()?, print_time)?;
            Ok(py_none())
        });
        r.method("klippy.extras.tmc_uart.MCU_TMC_uart_bitbang.register_instance", |a| {
            let rx = super::pins::pin_params_from_py(a.arg(0))?;
            let tx = super::pins::pin_params_from_py(a.arg(1))?;
            let spd: Option<Vec<String>> = if a.arg(2).is_none() { None } else { Some(crate::plugins::convert::FromPy::from_py(&lookup("builtins", "list")?.call(&[a.arg(2)])?)?) };
            instance_to_py(&a.this::<McuTmcUartBitbang>()?.register_instance(&rx, &tx, spd.as_deref(), a.arg(3).as_i64()?)?)
        });
        // {(instance_id, addr): True}
        r.attr("klippy.extras.tmc_uart.MCU_TMC_uart_bitbang.instances", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (i, addr) in objects::unwrap_as::<McuTmcUartBitbang>(o)?.instances.borrow().iter() {
                d.call_method("__setitem__", &[&py_tuple(vec![instance_to_py(i)?, py_int(*addr)]), &py_bool(true)])?;
            }
            Ok(d)
        }, None);
        // (instance_id, addr, mcu_uart)
        r.method("klippy.extras.tmc_uart.lookup_tmc_uart_bitbang", |a| {
            let config = super::configfile::config_from_py(a.arg(0))?;
            let (i, addr, uart) = crate::extras::tmc_uart::lookup_tmc_uart_bitbang(&config, a.arg(1).as_i64()?)?;
            Ok(py_tuple(vec![instance_to_py(&i)?, py_int(addr), uart.to_py()?]))
        });
    }

    // ---- angle: the sensor_type helpers' state
    {
        use crate::extras::angle::{Angle, AngleSensor, HelperA1333, HelperAS5047D, HelperMT6816, HelperMT6826S, HelperTLE5012B};
        fn helper<T: AngleSensor>(o: &PyRef) -> KResult<Rc<T>> {
            objects::unwrap_as::<T>(o)
        }
        r.attr("klippy.extras.angle.HelperA1333.is_tcode_absolute", |o| Ok(py_bool(helper::<HelperA1333>(o)?.is_tcode_absolute())), None);
        r.attr("klippy.extras.angle.HelperAS5047D.is_tcode_absolute", |o| Ok(py_bool(helper::<HelperAS5047D>(o)?.is_tcode_absolute())), None);
        r.attr("klippy.extras.angle.HelperTLE5012B.is_tcode_absolute", |o| Ok(py_bool(helper::<HelperTLE5012B>(o)?.is_tcode_absolute())), None);
        r.attr("klippy.extras.angle.HelperMT6816.is_tcode_absolute", |o| Ok(py_bool(helper::<HelperMT6816>(o)?.is_tcode_absolute())), None);
        r.attr("klippy.extras.angle.HelperMT6826S.is_tcode_absolute", |o| Ok(py_bool(helper::<HelperMT6826S>(o)?.is_tcode_absolute())), None);
        r.attr("klippy.extras.angle.HelperA1333.last_temperature", |o| helper::<HelperA1333>(o)?.last_temperature().to_py(), None);
        r.attr("klippy.extras.angle.HelperAS5047D.last_temperature", |o| helper::<HelperAS5047D>(o)?.last_temperature().to_py(), None);
        r.attr("klippy.extras.angle.HelperMT6816.last_temperature", |o| helper::<HelperMT6816>(o)?.last_temperature().to_py(), None);
        r.attr("klippy.extras.angle.HelperMT6826S.last_temperature", |o| helper::<HelperMT6826S>(o)?.last_temperature().to_py(), None);
        r.method("klippy.extras.angle.HelperTLE5012B.start", |a| {
            a.this::<HelperTLE5012B>()?.start()?;
            Ok(py_none())
        });
        r.method("klippy.extras.angle.HelperTLE5012B.update_clock", |a| {
            a.this::<HelperTLE5012B>()?.update_clock()?;
            Ok(py_none())
        });
        r.attr("klippy.extras.angle.HelperMT6826S.status_map", |_o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for code in 0..4 {
                d.call_method("__setitem__", &[&py_int(code), &py_str(HelperMT6826S::status_map(code))])?;
            }
            Ok(d)
        }, None);
        r.attr("klippy.extras.angle.Angle.sensor_helper", |o| {
            let h = objects::unwrap_as::<Angle>(o)?.sensor_helper.clone();
            let name = h.type_name();
            let any: Rc<dyn Any> = h;
            objects::wrap_any(any, name)
        }, None);
        ro_attr!(r, "klippy.extras.angle.Angle.bulk_queue", Angle, |s| s.bulk_queue.clone());
    }

    // ---- resonance_tester
    {
        use crate::extras::resonance_tester::{ResonanceTester, SweepingVibrationsTestGenerator};
        ro_attr!(r, "klippy.extras.resonance_tester.ResonanceTester.executor", ResonanceTester, |s| s.executor.clone());
        ro_attr!(r, "klippy.extras.resonance_tester.ResonanceTester.generator", ResonanceTester, |s| s.generator.clone());
        ro_attr!(r, "klippy.extras.resonance_tester.SweepingVibrationsTestGenerator.vibration_generator", SweepingVibrationsTestGenerator, |s| s.vibration_generator.clone());
    }

    // ---- input_shaper: {name: the shaper_defs function}
    r.attr("klippy.extras.input_shaper.InputShaperParams.shapers", |o| {
        objects::unwrap_as::<crate::extras::input_shaper::InputShaperParams>(o)?;
        let d = lookup("builtins", "dict")?.call(&[])?;
        for s in crate::extras::shaper_defs::INPUT_SHAPERS {
            d.call_method("__setitem__", &[&py_str(s.name), &lookup("klippy.extras.shaper_defs", &format!("get_{}_shaper", s.name))?])?;
        }
        Ok(d)
    }, None);

    // ---- bed_screws: {"adjust": screws, "fine": fine_adjust} of (coord, name)
    {
        use crate::extras::bed_screws::{BedScrews, Screw};
        fn screws(v: &[Screw]) -> PyRef {
            py_list(v.iter().map(|((x, y), n)| py_tuple(vec![py_tuple(vec![py_float(*x), py_float(*y)]), py_str(n)])).collect())
        }
        r.attr("klippy.extras.bed_screws.BedScrews.states", |o| {
            let s = objects::unwrap_as::<BedScrews>(o)?;
            let d = lookup("builtins", "dict")?.call(&[])?;
            d.call_method("__setitem__", &[&py_str("adjust"), &screws(&s.adjust)])?;
            d.call_method("__setitem__", &[&py_str("fine"), &screws(&s.fine)])?;
            Ok(d)
        }, None);
        r.method("klippy.extras.bed_screws.BedScrews.move_to_screw", |a| {
            let state = match a.string(0)?.as_str() {
                "adjust" => "adjust",
                "fine" => "fine",
                s => return Err(KError::Py("KeyError".into(), crate::pyfmt::py_str_repr(s))),
            };
            a.this::<BedScrews>()?.move_to_screw(state, a.arg(1).as_i64()? as usize)?;
            Ok(py_none())
        });
    }
    r.method("klippy.extras.bed_tilt.BedTilt.handle_connect", |a| {
        let b = a.this::<crate::extras::bed_tilt::BedTilt>()?;
        *b.toolhead.borrow_mut() = Some(b.printer.lookup::<crate::toolhead::ToolHead>("toolhead")?);
        Ok(py_none())
    });
    {
        use crate::extras::axis_twist_compensation::Calibrater;
        ro_attr!(r, "klippy.extras.axis_twist_compensation.Calibrater.probe_x_offset", Calibrater, |s| s.probe_x_offset.get());
        ro_attr!(r, "klippy.extras.axis_twist_compensation.Calibrater.probe_y_offset", Calibrater, |s| s.probe_y_offset.get());
    }

    // ---- z_calibration: the sites as [x, y, z] (None where unset)
    {
        use crate::extras::z_calibration::ZCalibrationHelper as Z;
        fn site(s: Option<Vec<Option<f64>>>) -> KResult<PyRef> {
            s.to_py()
        }
        r.attr("klippy.extras.z_calibration.ZCalibrationHelper.nozzle_site", |o| site(objects::unwrap_as::<Z>(o)?.nozzle_site.borrow().clone()), None);
        r.attr("klippy.extras.z_calibration.ZCalibrationHelper.switch_site", |o| site(objects::unwrap_as::<Z>(o)?.switch_site.borrow().clone()), None);
        r.attr("klippy.extras.z_calibration.ZCalibrationHelper.bed_site", |o| site(objects::unwrap_as::<Z>(o)?.bed_site.borrow().clone()), None);
        r.attr("klippy.extras.z_calibration.ZCalibrationHelper.switch_xy_offsets", |o| site(objects::unwrap_as::<Z>(o)?.switch_xy_offsets.clone()), None);
        r.attr("klippy.extras.z_calibration.ZCalibrationHelper.wiggle_offsets", |o| site(objects::unwrap_as::<Z>(o)?.wiggle_offsets.clone()), None);
        ro_attr!(r, "klippy.extras.z_calibration.ZCalibrationHelper.config", Z, |s| s.config.clone());
    }

    // ---- tools_calibrate
    {
        use crate::extras::tools_calibrate::{PrinterProbeMultiAxis, ProbeEndstopWrapper, ToolsCalibrate};
        // Position(x, y, z)
        fn position(p: Option<[f64; 3]>) -> KResult<PyRef> {
            match p {
                Some(p) => lookup("klippy.extras.tools_calibrate", "Position")?.call(&[&py_float(p[0]), &py_float(p[1]), &py_float(p[2])]),
                None => Ok(py_none()),
            }
        }
        r.attr("klippy.extras.tools_calibrate.ToolsCalibrate.last_result", |o| position(objects::unwrap_as::<ToolsCalibrate>(o)?.last_result.get()), None);
        r.attr("klippy.extras.tools_calibrate.ToolsCalibrate.sensor_location", |o| position(objects::unwrap_as::<ToolsCalibrate>(o)?.sensor_location.get()), None);
        ro_attr!(r, "klippy.extras.tools_calibrate.ToolsCalibrate.name", ToolsCalibrate, |s| s.name.clone());
        ro_attr!(r, "klippy.extras.tools_calibrate.ToolsCalibrate.trigger_to_bottom_z", ToolsCalibrate, |s| s.trigger_to_bottom_z);
        r.attr("klippy.extras.tools_calibrate.ToolsCalibrate.gcode_move", |o| objects::unwrap_as::<ToolsCalibrate>(o)?.printer.lookup_object("gcode_move")?.to_py(), None);
        r.method("klippy.extras.tools_calibrate.ToolsCalibrate.locate_sensor", |a| {
            position(Some(a.this::<ToolsCalibrate>()?.locate_sensor(&*super::gcode::gcmd_from_py(a.arg(0))?)?))
        });
        ro_attr!(r, "klippy.extras.tools_calibrate.PrinterProbeMultiAxis.name", PrinterProbeMultiAxis, |s| s.name.clone());
        ro_attr!(r, "klippy.extras.tools_calibrate.PrinterProbeMultiAxis.max_travel", PrinterProbeMultiAxis, |s| s.max_travel);
        r.attr("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.gcode_move", |o| objects::unwrap_as::<PrinterProbeMultiAxis>(o)?.printer.lookup_object("gcode_move")?.to_py(), None);
        // (set in __init__ and never changed)
        r.attr("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.last_state", |_o| Ok(py_bool(false)), None);
        r.attr("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.last_result", |_o| Ok(py_list(vec![py_float(0.0), py_float(0.0), py_float(0.0)])), None);
        r.attr("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.last_x_result", |_o| Ok(py_float(0.0)), None);
        r.attr("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.last_y_result", |_o| Ok(py_float(0.0)), None);
        r.attr("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.last_z_result", |_o| Ok(py_float(0.0)), None);
        // run_probe(direction, gcmd, speed_ratio=1.0, samples=None, max_distance=100.0)
        r.method("klippy.extras.tools_calibrate.PrinterProbeMultiAxis.run_probe", |a| {
            let samples = if a.arg(3).is_none() { None } else { Some(a.arg(3).as_i64()?) };
            let p = a.this::<PrinterProbeMultiAxis>()?.run_probe(&a.string(0)?, &*super::gcode::gcmd_from_py(a.arg(1))?, a.f64(2)?, samples, a.f64(4)?)?;
            position(Some(p))
        });
        r.attr("klippy.extras.tools_calibrate.ProbeEndstopWrapper.axis", |o| Ok(py_str(&objects::unwrap_as::<ProbeEndstopWrapper>(o)?.axis.to_string())), None);
    }

    // ---- shaper_calibrate (its methods run as Kalico's numpy code): numpy,
    // error (printer.command_error)
    r.attr("klippy.extras.shaper_calibrate.ShaperCalibrate.numpy", |o| {
        objects::unwrap_as::<crate::extras::shaper_calibrate::ShaperCalibrate>(o)?;
        import("numpy")
    }, None);
    r.attr("klippy.extras.shaper_calibrate.ShaperCalibrate.error", |o| {
        objects::unwrap_as::<crate::extras::shaper_calibrate::ShaperCalibrate>(o)?.printer.clone().to_py()?.getattr("command_error")
    }, None);

    // ---- palette2: the omega state (snapshots), the p2cmd_* handlers
    {
        use crate::extras::palette2::Palette2;
        fn st<R>(o: &PyRef, f: impl FnOnce(&crate::extras::palette2::Omega) -> R) -> KResult<R> {
            Ok(f(&objects::unwrap_as::<Palette2>(o)?.st.borrow()))
        }
        // [{"number": n, "percent": p}]
        fn pings(v: &[(i64, f64)]) -> KResult<PyRef> {
            let items: KResult<Vec<PyRef>> = v
                .iter()
                .map(|(n, p)| {
                    let d = lookup("builtins", "dict")?.call(&[])?;
                    d.call_method("__setitem__", &[&py_str("number"), &py_int(*n)])?;
                    d.call_method("__setitem__", &[&py_str("percent"), &py_float(*p)])?;
                    Ok(d)
                })
                .collect();
            Ok(py_list(items?))
        }
        r.attr("klippy.extras.palette2.Palette2.files", |o| st(o, |s| s.files.clone())?.to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.is_setup_complete", |o| Ok(py_bool(st(o, |s| s.is_setup_complete)?)), None);
        r.attr("klippy.extras.palette2.Palette2.is_splicing", |o| Ok(py_bool(st(o, |s| s.is_splicing)?)), None);
        r.attr("klippy.extras.palette2.Palette2.is_loading", |o| Ok(py_bool(st(o, |s| s.is_loading)?)), None);
        r.attr("klippy.extras.palette2.Palette2.remaining_load_length", |o| st(o, |s| s.remaining_load_length)?.to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.omega_algorithms", |o| st(o, |s| s.algorithms.clone())?.to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.omega_algorithms_counter", |o| Ok(py_int(st(o, |s| s.algorithms_counter)? as i64)), None);
        r.attr("klippy.extras.palette2.Palette2.omega_splices", |o| st(o, |s| s.splices.clone())?.to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.omega_splices_counter", |o| Ok(py_int(st(o, |s| s.splices_counter)? as i64)), None);
        r.attr("klippy.extras.palette2.Palette2.omega_pings", |o| pings(&st(o, |s| s.pings.clone())?), None);
        r.attr("klippy.extras.palette2.Palette2.omega_pongs", |o| pings(&st(o, |s| s.pongs.clone())?), None);
        r.attr("klippy.extras.palette2.Palette2.omega_current_ping", |o| st(o, |s| s.current_ping.clone())?.to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.omega_header", |o| st(o, |s| s.header.to_vec())?.to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.omega_header_counter", |o| Ok(py_int(st(o, |s| s.header_counter)? as i64)), None);
        r.attr("klippy.extras.palette2.Palette2.omega_last_command", |o| Ok(py_str(&st(o, |s| s.last_command.clone())?)), None);
        r.attr("klippy.extras.palette2.Palette2.omega_drivers", |o| objects::unwrap_as::<Palette2>(o)?.omega_drives.borrow().clone().to_py(), None);
        r.attr("klippy.extras.palette2.Palette2.gcode_move", |o| objects::unwrap_as::<Palette2>(o)?.printer.lookup_object("gcode_move")?.to_py(), None);
        // cmd_Disconnect(gcmd=None) / cmd_OmegaDefault(gcmd)
        r.method("klippy.extras.palette2.Palette2.cmd_Disconnect", |a| {
            a.this::<Palette2>()?.cmd_disconnect()?;
            Ok(py_none())
        });
        r.method("klippy.extras.palette2.Palette2.cmd_OmegaDefault", |a| {
            a.this::<Palette2>()?.cmd_omega_default(&*super::gcode::gcmd_from_py(a.arg(0))?)?;
            Ok(py_none())
        });
        // p2cmd_O*(params): params is the list of the message's fields
        fn params(a: &crate::plugins::native::Args) -> KResult<Vec<String>> {
            crate::plugins::convert::FromPy::from_py(&lookup("builtins", "list")?.call(&[a.arg(0)])?)
        }
        fn p2(a: &crate::plugins::native::Args, f: fn(&Palette2, &[&str]) -> KResult<()>) -> KResult<PyRef> {
            let p = params(a)?;
            let refs: Vec<&str> = p.iter().map(|s| s.as_str()).collect();
            f(&*a.this::<Palette2>()?, &refs)?;
            Ok(py_none())
        }
        r.method("klippy.extras.palette2.Palette2.p2cmd_O20", |a| p2(a, Palette2::p2cmd_o20));
        r.method("klippy.extras.palette2.Palette2.p2cmd_O34", |a| p2(a, Palette2::p2cmd_o34));
        r.method("klippy.extras.palette2.Palette2.p2cmd_O50", |a| p2(a, Palette2::p2cmd_o50));
        r.method("klippy.extras.palette2.Palette2.p2cmd_O53", |a| p2(a, Palette2::p2cmd_o53));
        r.method("klippy.extras.palette2.Palette2.p2cmd_O97", |a| p2(a, Palette2::p2cmd_o97));
        r.method("klippy.extras.palette2.Palette2.p2cmd_O100", |a| {
            a.this::<Palette2>()?.p2cmd_o100()?;
            Ok(py_none())
        });
        r.method("klippy.extras.palette2.Palette2.p2cmd_O102", |a| {
            a.this::<Palette2>()?.p2cmd_o102()?;
            Ok(py_none())
        });
    }

    // ---- delta_calibrate: positions as (height, stable_position) /
    // (distance, stable_position1, stable_position2)
    {
        use crate::extras::delta_calibrate::{DeltaCalibrate, Stable};
        fn stable(s: &Stable) -> KResult<PyRef> {
            let items: Vec<PyRef> = s.v.iter().map(|x| py_float(*x)).collect();
            Ok(if s.is_tuple { py_tuple(items) } else { py_list(items) })
        }
        fn heights(v: &[(f64, Stable)]) -> KResult<PyRef> {
            let items: KResult<Vec<PyRef>> = v.iter().map(|(h, s)| Ok(py_tuple(vec![py_float(*h), stable(s)?]))).collect();
            Ok(py_list(items?))
        }
        r.attr("klippy.extras.delta_calibrate.DeltaCalibrate.last_probe_positions", |o| heights(&objects::unwrap_as::<DeltaCalibrate>(o)?.last_probe_positions.borrow()), None);
        r.attr("klippy.extras.delta_calibrate.DeltaCalibrate.manual_heights", |o| heights(&objects::unwrap_as::<DeltaCalibrate>(o)?.manual_heights.borrow()), None);
        r.attr("klippy.extras.delta_calibrate.DeltaCalibrate.last_distances", |o| {
            let items: KResult<Vec<PyRef>> = objects::unwrap_as::<DeltaCalibrate>(o)?
                .last_distances
                .iter()
                .map(|(d, a, b)| Ok(py_tuple(vec![py_float(*d), stable(a)?, stable(b)?])))
                .collect();
            Ok(py_list(items?))
        }, None);
    }

    // ---- z_thermal_adjust: handle_homing_move_end(homing_state, rails)
    {
        use crate::extras::z_thermal_adjust::ZThermalAdjuster;
        r.method("klippy.extras.z_thermal_adjust.ZThermalAdjuster.handle_homing_move_end", |a| {
            let axes: Vec<i64> = crate::plugins::convert::FromPy::from_py(&a.arg(0).call_method("get_axes", &[])?)?;
            if axes.contains(&2) {
                let z = a.this::<ZThermalAdjuster>()?;
                z.ref_temperature.set(z.smoothed_temp.get());
                z.ref_temp_override.set(false);
                z.z_adjust_mm.set(0.0);
            }
            Ok(py_none())
        });
        r.attr("klippy.extras.z_thermal_adjust.ZThermalAdjuster.toolhead", |o| {
            objects::unwrap_as::<ZThermalAdjuster>(o)?;
            printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup_object_opt("toolhead").to_py()
        }, None);
    }

    // ---- tmc stallguard dump
    {
        use crate::extras::tmc::TMCStallguardDump;
        ro_attr!(r, "klippy.extras.tmc.TMCStallguardDump.stepper_name", TMCStallguardDump, |s| s.stepper_name.clone());
        ro_attr!(r, "klippy.extras.tmc.TMCStallguardDump.sg2_supp", TMCStallguardDump, |s| s.sg2_supp);
    }

    // ---- util: get_git_version (kaliru reports the Kalico commit it tracks), create_pty
    r.method("klippy.util.get_git_version", |_a| {
        let g = crate::util::get_git_version();
        let d = lookup("builtins", "dict")?.call(&[])?;
        d.call_method("__setitem__", &[&py_str("version"), &py_str(&g.version)])?;
        d.call_method("__setitem__", &[&py_str("file_status"), &g.file_status.to_py()?])?;
        d.call_method("__setitem__", &[&py_str("branch"), &py_str(&g.branch)])?;
        d.call_method("__setitem__", &[&py_str("remote"), &py_str(&g.remote)])?;
        d.call_method("__setitem__", &[&py_str("url"), &py_str(&g.url)])?;
        Ok(d)
    });
    r.method("klippy.util.create_pty", |a| {
        let fd = crate::util::create_pty(&a.string(0)?).map_err(|e| KError::os(&e, "create_pty"))?;
        Ok(py_int(fd as i64))
    });
    r.attr("klippy.gcode.GCodeIO.fd_handle", |o| objects::unwrap_as::<crate::gcode::GCodeIO>(o)?.fd_handle.borrow().clone().to_py(), None);
}
