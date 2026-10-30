// klippy/extras: pause_resume.PauseResume, gcode_macro.GCodeMacro
// (variables) / PrinterGCodeMacro / Template / TemplateWrapperJinja,
// stepper_enable.PrinterStepperEnable / EnableTracking
use super::*;
use crate::extras::gcode_macro::{GCodeMacro, GetStatusWrapperJinja, PrinterGCodeMacro, Template, TemplateWrapperJinja};
use crate::extras::pause_resume::PauseResume;
use crate::extras::stepper_enable::{EnableTracking, PrinterStepperEnable};
use crate::jinja::value::Value;
use crate::IndexMap;
use crate::{method, ro_attr};
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::DictField;

pub fn register(r: &mut Registry) {
    // statistics: python's list holds each object's bound stats method
    r.attr("klippy.extras.statistics.PrinterStats.stats_cb", |o| {
        let objs = objects::unwrap_as::<crate::extras::statistics::PrinterStats>(o)?.stats_cb.borrow().clone();
        let l = lookup("builtins", "list")?.call(&[])?;
        for obj in objs {
            l.call_method("append", &[&objects::wrap_pobj(obj)?.getattr("stats")?])?;
        }
        Ok(l)
    }, None);
    // error_mcu: clarify callbacks cb(msg, details) -> message or None
    {
        use crate::extras::error_mcu::{ClarifyCallback, PrinterMcuError};
        r.method("klippy.extras.error_mcu.PrinterMCUError.add_clarify", |a| {
            let cb = a.arg(1).clone();
            // kaliru's clarify callbacks can't fail: a raising one is logged
            // and adds nothing
            let f: ClarifyCallback = Rc::new(move |msg: &str, details: &PyVal| {
                let r = cb.call(&[&py_str(msg), &from_pyval(details)]).and_then(|m| {
                    if m.is_none() { Ok(None) } else { m.as_string().map(Some) }
                });
                r.unwrap_or_else(|e| {
                    crate::logging::exception_err("clarify callback error", &e);
                    None
                })
            });
            a.this::<PrinterMcuError>()?.add_clarify(&a.string(0)?, f);
            Ok(py_none())
        });
        // {msg: [callback]} (a snapshot)
        r.attr("klippy.extras.error_mcu.PrinterMCUError.clarify_callbacks", |o| {
            let cbs = objects::unwrap_as::<PrinterMcuError>(o)?.clarify_callbacks.borrow().clone();
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (msg, list) in cbs {
                let items: KResult<Vec<PyRef>> = list
                    .into_iter()
                    .map(|cb| {
                        crate::plugins::callable::py_function(Rc::new(move |args: &[PyRef]| {
                            let details = to_pyval(&args[1])?;
                            Ok(match cb(&args[0].as_string()?, &details) {
                                Some(m) => py_str(&m),
                                None => py_none(),
                            })
                        }))
                    })
                    .collect();
                d.call_method("__setitem__", &[&py_str(&msg), &py_list(items?)])?;
            }
            Ok(d)
        }, None);
    }
    // hall_filament_width_sensor: Kalico's names for kaliru's fields
    {
        use crate::extras::hall_filament_width_sensor::HallFilamentWidthSensor as H;
        fn hs(o: &PyRef) -> KResult<Rc<H>> {
            objects::unwrap_as::<H>(o)
        }
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.pin1", |o| Ok(py_str(&hs(o)?.pin1)), None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.pin2", |o| Ok(py_str(&hs(o)?.pin2)), None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.MEASUREMENT_INTERVAL_MM", |o| {
            Ok(py_int(hs(o)?.measurement_interval_mm))
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.measurement_max_difference", |o| {
            Ok(py_float(hs(o)?.measurement_max_difference))
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.lastFilamentWidthReading", |o| {
            Ok(py_int(hs(o)?.last_reading.get()))
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.lastFilamentWidthReading2", |o| {
            Ok(py_int(hs(o)?.last_reading2.get()))
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.firstExtruderUpdatePosition", |o| {
            Ok(py_float(hs(o)?.first_extruder_update_position.get()))
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.mcu_adc", |o| {
            super::pins::adc_pin_to_py(hs(o)?.mcu_adc.clone())
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.mcu_adc2", |o| {
            super::pins::adc_pin_to_py(hs(o)?.mcu_adc2.clone())
        }, None);
        r.attr("klippy.extras.hall_filament_width_sensor.HallFilamentWidthSensor.ppins", |o| {
            hs(o)?.printer.lookup_object("pins")?.to_py()
        }, None);
    }
    // reactor timer: its callback, callable from python
    r.attr("klippy.reactor.ReactorTimer.callback", |o| {
        let t = objects::unwrap_as::<crate::reactor::ReactorTimer>(o)?;
        crate::plugins::callable::py_function(Rc::new(move |args: &[PyRef]| match t.call(args[0].as_f64()?) {
            Some(w) => Ok(py_float(w)),
            None => Err(KError::Py("RuntimeError".into(), "kaliru: the timer callback is running".into())),
        }))
    }, None);
    // gcode io: the input fd and the last 50 inputs, deque([(eventtime, data)], maxlen=50)
    r.attr("klippy.gcode.GCodeIO.fd", |o| Ok(py_int(objects::unwrap_as::<crate::gcode::GCodeIO>(o)?.fd as i64)), None);
    r.attr("klippy.gcode.GCodeIO.input_log", |o| {
        let io = objects::unwrap_as::<crate::gcode::GCodeIO>(o)?;
        let items: Vec<PyRef> = io.input_log.borrow().iter().map(|(t, d)| py_tuple(vec![py_float(*t), py_str(d)])).collect();
        lookup("collections", "deque")?.call(&[&py_list(items), &py_int(50)])
    }, None);
    // output_pin: its mcu pin (pwm or digital_out)
    r.attr("klippy.extras.output_pin.PrinterOutputPin.mcu_pin", |o| {
        use crate::extras::output_pin::{OutPin, PrinterOutputPin};
        match &objects::unwrap_as::<PrinterOutputPin>(o)?.mcu_pin {
            OutPin::Pwm(p) => super::pins::pwm_pin_to_py(p.clone()),
            OutPin::Digital(p) => super::pins::digital_out_to_py(p.clone()),
        }
    }, None);
    // led: colors are (red, green, blue, white) tuples
    {
        use crate::extras::led::{Color, LEDHelper, PrinterPWMLED};
        fn color(c: &Color) -> PyRef {
            py_tuple(c.iter().map(|v| py_float(*v)).collect())
        }
        r.attr("klippy.extras.led.LEDHelper.led_state", |o| {
            Ok(py_list(objects::unwrap_as::<LEDHelper>(o)?.led_state.borrow().iter().map(color).collect()))
        }, None);
        // update_func(led_state, print_time)
        r.attr("klippy.extras.led.LEDHelper.update_func", |o| {
            let f = objects::unwrap_as::<LEDHelper>(o)?.update_func.clone();
            crate::plugins::callable::py_function(Rc::new(move |args: &[PyRef]| {
                let state: Vec<Color> = FromPy::from_py(&lookup("builtins", "list")?.call(&[&args[0]])?)?;
                let print_time = if args[1].is_none() { None } else { Some(args[1].as_f64()?) };
                f(&state, print_time)?;
                Ok(py_none())
            }))
        }, None);
        r.attr("klippy.extras.led.PrinterPWMLED.prev_color", |o| Ok(color(&objects::unwrap_as::<PrinterPWMLED>(o)?.prev_color.borrow())), None);
        // [(color index, pwm pin)]
        r.attr("klippy.extras.led.PrinterPWMLED.pins", |o| {
            let items: KResult<Vec<PyRef>> = objects::unwrap_as::<PrinterPWMLED>(o)?
                .pins
                .iter()
                .map(|(i, p)| Ok(py_tuple(vec![py_int(*i as i64), super::pins::pwm_pin_to_py(p.clone())?])))
                .collect();
            Ok(py_list(items?))
        }, None);
    }
    // adc_scaled: the vref / vssa pins, its pin chip and the smoothing callbacks
    {
        use crate::extras::adc_scaled::PrinterADCScaled as S;
        r.attr("klippy.extras.adc_scaled.PrinterADCScaled.mcu_vref", |o| super::pins::adc_pin_to_py(objects::unwrap_as::<S>(o)?.mcu_vref.clone()), None);
        r.attr("klippy.extras.adc_scaled.PrinterADCScaled.mcu_vssa", |o| super::pins::adc_pin_to_py(objects::unwrap_as::<S>(o)?.mcu_vssa.clone()), None);
        r.method("klippy.extras.adc_scaled.PrinterADCScaled.setup_pin", |a| {
            use crate::pins::PinChip;
            let pin_params = super::pins::pin_params_from_py(a.arg(1))?;
            let pin = a.this::<S>()?.setup_pin(&a.string(0)?, &pin_params).map_err(KError::Pins)?;
            let adc = pin.downcast::<Rc<dyn crate::mcu::AdcPin>>().map_err(|_| KError::Other("kaliru: adc_scaled pin of an unexpected type".into()))?;
            super::pins::adc_pin_to_py((*adc).clone())
        });
        r.method("klippy.extras.adc_scaled.PrinterADCScaled.vref_callback", |a| {
            let s = a.this::<S>()?;
            s.last_vref.set(s.calc_smooth(a.f64(0)?, a.f64(1)?, s.last_vref.get()));
            Ok(py_none())
        });
        r.method("klippy.extras.adc_scaled.PrinterADCScaled.vssa_callback", |a| {
            let s = a.this::<S>()?;
            s.last_vssa.set(s.calc_smooth(a.f64(0)?, a.f64(1)?, s.last_vssa.get()));
            Ok(py_none())
        });
    }
    // SecondarySync's own fields (kaliru: ClockSync.secondary)
    {
        use crate::clocksync::{ClockSync, Secondary};
        fn sec<R>(o: &PyRef, f: impl FnOnce(&Secondary) -> KResult<R>) -> KResult<R> {
            match &objects::unwrap_as::<ClockSync>(o)?.secondary {
                Some(s) => f(s),
                None => Err(KError::Py("AttributeError".into(), "kaliru: not a SecondarySync".into())),
            }
        }
        r.attr("klippy.clocksync.SecondarySync.main_sync", |o| sec(o, |s| objects::wrap(s.main_sync.clone())), None);
        r.attr("klippy.clocksync.SecondarySync.clock_adj", |o| sec(o, |s| s.clock_adj.get().to_py()), Some(|o, v| {
            sec(o, |s| {
                s.clock_adj.set(FromPy::from_py(v)?);
                Ok(())
            })
        }));
        r.attr("klippy.clocksync.SecondarySync.last_sync_time", |o| sec(o, |s| Ok(py_float(s.last_sync_time.get()))), Some(|o, v| {
            sec(o, |s| {
                s.last_sync_time.set(v.as_f64()?);
                Ok(())
            })
        }));
    }
    // servo: the pulse width conversions and _set_pwm
    {
        use crate::extras::servo::PrinterServo;
        macro_rules! servo_value {
            ($r:ident, $($name:ident),*) => {$(
                $r.attr(concat!("klippy.extras.servo.PrinterServo.", stringify!($name)), |o| {
                    Ok(py_float(objects::unwrap_as::<PrinterServo>(o)?.calc.$name))
                }, None);
            )*};
        }
        servo_value!(r, min_width, max_width, max_angle, angle_to_width, width_to_value);
        // _set_pwm(print_time, value): None, or ("discard", 0.0)
        r.method("klippy.extras.servo.PrinterServo._set_pwm", |a| {
            use crate::extras::output_pin::Action;
            match a.this::<PrinterServo>()?.set_pwm(a.f64(0)?, a.f64(1)?)? {
                None => Ok(py_none()),
                Some((action, wait)) => {
                    let s = match action {
                        Action::Discard => "discard",
                        Action::Delay => "delay",
                    };
                    Ok(py_tuple(vec![py_str(s), py_float(wait)]))
                }
            }
        });
    }
    // trad_rack.fil_homing_lengths: {name: length} (kaliru keeps them in key order)
    r.attr("klippy.extras.trad_rack.TradRack.fil_homing_lengths", |o| {
        let tr = objects::unwrap_as::<crate::extras::trad_rack::TradRack>(o)?;
        let d = lookup("builtins", "dict")?.call(&[])?;
        let names = ["user load lane", "load selector", "bowden load", "bowden unload", "unload toolhead"];
        for (k, v) in names.iter().zip(tr.fil_homing_lengths) {
            d.call_method("__setitem__", &[&py_str(k), &py_float(v)])?;
        }
        Ok(d)
    }, None);
    // homing_override.prev_G28: the G28 handler it replaced
    r.attr("klippy.extras.homing_override.HomingOverride.prev_G28", |o| {
        match objects::unwrap_as::<crate::extras::homing_override::HomingOverride>(o)?.prev_g28.borrow().clone() {
            Some(h) => super::gcode::handler_to_py(h),
            None => Ok(py_none()),
        }
    }, None);
    // menu: Kalico's _root is the menu_root name (root the loaded container)
    r.attr("klippy.extras.display.menu.MenuManager._root", |o| {
        Ok(py_str(&objects::unwrap_as::<crate::extras::display::menu::MenuManager>(o)?.root_name))
    }, None);
    // query_endstops.endstops: [(endstop, name)] (a snapshot)
    r.attr("klippy.extras.query_endstops.QueryEndstops.endstops", |o| {
        let q = objects::unwrap_as::<crate::extras::query_endstops::QueryEndstops>(o)?;
        let items: KResult<Vec<PyRef>> = q
            .endstops
            .borrow()
            .iter()
            .map(|(e, name)| Ok(py_tuple(vec![super::pins::endstop_to_py(e)?, py_str(name)])))
            .collect();
        Ok(py_list(items?))
    }, None);
    // save_variables.allVariables (a snapshot)
    r.attr("klippy.extras.save_variables.SaveVariables.allVariables", |o| {
        let sv = objects::unwrap_as::<crate::extras::save_variables::SaveVariables>(o)?;
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in sv.all_variables.borrow().iter() {
            d.call_method("__setitem__", &[&py_str(k), &crate::plugins::values::value_to_py(v)?])?;
        }
        Ok(d)
    }, None);
    // virtual_sdcard.current_file: None, or the file python would have open
    // (its own handle at kaliru's read position: reads don't move kaliru's)
    r.attr("klippy.extras.virtual_sdcard.VirtualSD.current_file", |o| {
        let vsd = objects::unwrap_as::<crate::extras::virtual_sdcard::VirtualSD>(o)?;
        let Some(name) = vsd.current_file_name() else { return Ok(py_none()) };
        let f = lookup("io", "open")?.call(&[&py_str(&name), &py_str("r"), &py_int(-1), &py_none(), &py_none(), &py_str("")])?;
        f.call_method("seek", &[&py_int(vsd.file_position.get() as i64)])?;
        Ok(f)
    }, None);
    // display.display: the lcd chip, the menu and the display templates
    {
        use crate::extras::display::PrinterLCD;
        use crate::extras::display_templates::PrinterDisplayTemplate;
        r.attr("klippy.extras.display.display.PrinterLCD.lcd_chip", |o| {
            super::traits::lcd_chip_to_py(objects::unwrap_as::<PrinterLCD>(o)?.lcd_chip.clone())
        }, None);
        r.attr("klippy.extras.display.display.PrinterLCD.menu", |o| match objects::unwrap_as::<PrinterLCD>(o)?.menu.borrow().clone() {
            Some(m) => objects::wrap(m),
            None => Ok(py_none()),
        }, None);
        fn templates_dict(t: &PrinterDisplayTemplate) -> KResult<PyRef> {
            let d = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in &t.display_templates {
                d.call_method("__setitem__", &[&py_str(k), &objects::wrap(v.clone())?])?;
            }
            Ok(d)
        }
        r.attr("klippy.extras.display.display.PrinterLCD.display_templates", |o| {
            templates_dict(&*objects::unwrap_as::<PrinterLCD>(o)?.templates()?)
        }, None);
        r.method("klippy.extras.display.display.PrinterDisplayTemplate.get_display_templates", |a| {
            templates_dict(&*a.this::<PrinterDisplayTemplate>()?)
        });
        r.attr("klippy.extras.display.display.PrinterDisplayTemplate.display_templates", |o| {
            templates_dict(&*objects::unwrap_as::<PrinterDisplayTemplate>(o)?)
        }, None);
        // display groups {name: DisplayGroup} and glyphs
        // {name: {"icon16x16": (icon1, icon2), "icon5x8": (slot, rows)}}
        fn groups_dict(t: &PrinterDisplayTemplate) -> KResult<PyRef> {
            let d = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in &t.display_data_groups {
                d.call_method("__setitem__", &[&py_str(k), &objects::wrap(v.clone())?])?;
            }
            Ok(d)
        }
        fn glyphs_dict(t: &PrinterDisplayTemplate) -> KResult<PyRef> {
            let d = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
            for (k, g) in &t.display_glyphs {
                let icons = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
                if let Some((icon1, icon2)) = &g.icon16x16 {
                    icons.call_method("__setitem__", &[&py_str("icon16x16"), &py_tuple(vec![icon1.clone().to_py()?, icon2.clone().to_py()?])])?;
                }
                if let Some((slot, rows)) = &g.icon5x8 {
                    icons.call_method("__setitem__", &[&py_str("icon5x8"), &py_tuple(vec![py_int(*slot), rows.clone().to_py()?])])?;
                }
                d.call_method("__setitem__", &[&py_str(k), &icons])?;
            }
            Ok(d)
        }
        r.attr("klippy.extras.display.display.PrinterDisplayTemplate.display_data_groups", |o| {
            groups_dict(&*objects::unwrap_as::<PrinterDisplayTemplate>(o)?)
        }, None);
        r.method("klippy.extras.display.display.PrinterDisplayTemplate.get_display_data_groups", |a| {
            groups_dict(&*a.this::<PrinterDisplayTemplate>()?)
        });
        r.attr("klippy.extras.display.display.PrinterDisplayTemplate.display_glyphs", |o| {
            glyphs_dict(&*objects::unwrap_as::<PrinterDisplayTemplate>(o)?)
        }, None);
        r.method("klippy.extras.display.display.PrinterDisplayTemplate.get_display_glyphs", |a| {
            glyphs_dict(&*a.this::<PrinterDisplayTemplate>()?)
        });
        r.attr("klippy.extras.display.display.PrinterLCD.display_data_groups", |o| {
            groups_dict(&*objects::unwrap_as::<PrinterLCD>(o)?.templates()?)
        }, None);
        // show_data_group: the DisplayGroup shown (kaliru keeps its name)
        r.attr("klippy.extras.display.display.PrinterLCD.show_data_group", |o| {
            let lcd = objects::unwrap_as::<PrinterLCD>(o)?;
            let name = lcd.show_data_group.borrow().clone();
            match lcd.templates()?.display_data_groups.get(&name) {
                Some(g) => objects::wrap(g.clone()),
                None => Ok(py_none()),
            }
        }, None);
        // DisplayTemplate.get_params(): {name: value}
        r.method("klippy.extras.display.display.DisplayTemplate.get_params", |a| {
            let t = a.this::<crate::extras::display_templates::DisplayTemplate>()?;
            let d = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in t.get_params() {
                d.call_method("__setitem__", &[&py_str(k), &crate::plugins::values::value_to_py(v)?])?;
            }
            Ok(d)
        });
    }
    method!(r, "klippy.extras.pause_resume.PauseResume", PauseResume, send_pause_command(p) => p.send_pause_command());
    method!(r, "klippy.extras.pause_resume.PauseResume", PauseResume, send_resume_command(p) => p.send_resume_command());
    method!(r, "klippy.extras.pause_resume.PauseResume", PauseResume, is_sd_active(p) => p.is_sd_active());
    r.method("klippy.extras.pause_resume.PauseResume.cmd_PAUSE", |a| {
        let g = super::gcode::gcmd_from_py(a.arg(0))?;
        a.this::<PauseResume>()?.cmd_pause(&g)?;
        Ok(py_none())
    });
    r.method("klippy.extras.pause_resume.PauseResume.cmd_RESUME", |a| {
        let g = super::gcode::gcmd_from_py(a.arg(0))?;
        a.this::<PauseResume>()?.cmd_resume(&g)?;
        Ok(py_none())
    });

    // templates: Template delegates to its function (TemplateWrapperJinja /
    // TemplateWrapperPython) through __getattr__, as in Kalico
    r.method("klippy.extras.gcode_macro.PrinterGCodeMacro.load_template", |a| {
        let config = super::configfile::config_from_py(a.arg(0))?;
        let default = a.opt_string(2)?;
        a.this::<PrinterGCodeMacro>()?.load_template(&config, &a.string(1)?, default.as_deref())?.to_py()
    });
    ro_attr!(r, "klippy.extras.gcode_macro.PrinterGCodeMacro.printer", PrinterGCodeMacro, |m| m.printer.clone());
    r.attr("klippy.extras.gcode_macro.Template.function", template_function, None);
    r.method("klippy.extras.gcode_macro.Template.__getattr__", |a| template_function(a.slf.as_ref().unwrap())?.getattr(&a.string(0)?));
    // Template(context=None): self.function(context)
    r.method("klippy.extras.gcode_macro.Template.__call__", |a| template_function(a.slf.as_ref().unwrap())?.call(&[a.arg(0)]));
    // reload(script_type, script) with the gcode_macro environment
    r.method("klippy.extras.gcode_macro.Template.reload", |a| {
        let t = a.this::<Template>()?;
        let gm = t.printer.lookup::<PrinterGCodeMacro>("gcode_macro")?;
        t.reload(gm.env(), &a.string(0)?, &a.string(1)?)?;
        Ok(py_none())
    });
    // env: the jinja2 environment Kalico builds (kaliru renders its own)
    r.attr("klippy.extras.gcode_macro.Template.env", |_o| macro_env(), None);
    r.attr("klippy.extras.gcode_macro.PrinterGCodeMacro.env", |_o| macro_env(), None);
    ro_attr!(r, "klippy.extras.gcode_macro.PrinterGCodeMacro.gcode", PrinterGCodeMacro, |m| m.printer.lookup::<crate::gcode::GCodeDispatch>("gcode"));
    ro_attr!(r, "klippy.extras.gcode_macro.GCodeMacro.printer", GCodeMacro, |m| m.template.printer.clone());
    // the gcode_macro object's create_template_context
    r.attr("klippy.extras.gcode_macro.TemplateWrapperJinja.create_template_context", |o| {
        let t = objects::unwrap_as::<TemplateWrapperJinja>(o)?;
        t.printer.lookup::<PrinterGCodeMacro>("gcode_macro")?.to_py()?.getattr("create_template_context")
    }, None);
    // cache: {name: status}
    r.attr("klippy.extras.gcode_macro.GetStatusWrapperJinja.cache", |o| {
        let cache = objects::unwrap_as::<GetStatusWrapperJinja>(o)?.cache.borrow().clone();
        let d = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in &cache {
            d.call_method("__setitem__", &[&py_str(k), &crate::plugins::values::value_to_py(v)?])?;
        }
        Ok(d)
    }, None);
    method!(r, "klippy.extras.gcode_macro.TemplateWrapperJinja", TemplateWrapperJinja, render(t, context: Option<PyRef>) =>
        t.render(template_context(context)?));
    method!(r, "klippy.extras.gcode_macro.TemplateWrapperJinja", TemplateWrapperJinja, run_gcode_from_command(t, context: Option<PyRef>) =>
        t.run_gcode_from_command(template_context(context)?));

    // template lists / dicts as python objects paired with them (values.rs)
    objects::register_class::<std::cell::RefCell<Vec<Value>>>("_k", "ValueList");
    objects::register_class::<std::cell::RefCell<IndexMap<crate::jinja::value::Key, Value>>>("_k", "ValueDict");
    r.ext("vl_store", |a| crate::plugins::values::store_list(a.arg(0)).map(|_| py_none()));
    r.ext("vd_store", |a| crate::plugins::values::store_dict(a.arg(0)).map(|_| py_none()));
    // get_status(): self.variables, its values the macro's own objects
    r.method("klippy.extras.gcode_macro.GCodeMacro.get_status", |a| {
        let m = a.this::<GCodeMacro>()?;
        let d = crate::plugins::py::lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in m.variables.borrow().iter() {
            d.call_method("__setitem__", &[&py_str(k), &crate::plugins::values::value_to_py(v)?])?;
        }
        Ok(d)
    });

    // GCodeMacro.variables: {name: value}, values kaliru's jinja values
    r.dict(
        "klippy.extras.gcode_macro.GCodeMacro.variables",
        DictField {
            get: |o, _f, key| match objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow().get(&key.as_string()?) {
                Some(v) => crate::plugins::values::value_to_py(v),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |o, _f, key, value| {
                let v = crate::plugins::values::py_to_value(value)?;
                let m = objects::unwrap_as::<GCodeMacro>(o)?;
                m.variables.borrow_mut().insert(key.as_string()?, v);
                m.variables_changed();
                Ok(())
            },
            del: |o, _f, key| match objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow_mut().shift_remove(&key.as_string()?) {
                Some(_) => {
                    objects::unwrap_as::<GCodeMacro>(o)?.variables_changed();
                    Ok(())
                }
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            keys: |o, _f| Ok(objects::unwrap_as::<GCodeMacro>(o)?.variables.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );

    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, lookup_enable(s, name: String) => s.lookup_enable(&name));
    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, get_steppers(s) => s.get_steppers());
    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, motor_off(s) => s.motor_off());
    method!(r, "klippy.extras.stepper_enable.PrinterStepperEnable", PrinterStepperEnable, set_motors_enable(s, names: Vec<String>, enable: bool) =>
        s.set_motors_enable(&names, enable));
    r.dict(
        "klippy.extras.stepper_enable.PrinterStepperEnable.enable_lines",
        DictField {
            get: |o, _f, key| match objects::unwrap_as::<PrinterStepperEnable>(o)?.enable_lines.borrow().get(&key.as_string()?) {
                Some(e) => e.clone().to_py(),
                None => Err(KError::Py("KeyError".into(), key.repr()?)),
            },
            set: |_o, f, _k, _v| Err(KError::Py("NotImplementedError".into(), format!("kaliru: assigning into {}", f))),
            del: |_o, f, _k| Err(KError::Py("NotImplementedError".into(), format!("kaliru: deleting from {}", f))),
            keys: |o, _f| Ok(objects::unwrap_as::<PrinterStepperEnable>(o)?.enable_lines.borrow().keys().map(|k| py_str(k)).collect()),
        },
    );
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, motor_enable(e, print_time: f64) => e.motor_enable(print_time));
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, motor_disable(e, print_time: f64) => e.motor_disable(print_time));
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, is_motor_enabled(e) => e.is_motor_enabled());
    method!(r, "klippy.extras.stepper_enable.EnableTracking", EnableTracking, has_dedicated_enable(e) => e.has_dedicated_enable());
}

fn template_function(o: &PyRef) -> KResult<PyRef> {
    let t = objects::unwrap_as::<Template>(o)?;
    if let Some(obj) = t.host_python() {
        return Ok(obj);
    }
    let (f, type_name) = t.function();
    objects::wrap_any(f, type_name)
}

/// a render context python passes (a dict), as kaliru's jinja values
pub(super) fn template_context(context: Option<PyRef>) -> KResult<Option<IndexMap<String, Value>>> {
    let Some(c) = context.filter(|c| !c.is_none()) else { return Ok(None) };
    match to_pyval(&c)? {
        crate::pyval::PyVal::Dict(d) => Ok(Some(d.iter().map(|(k, v)| (k.to_string(), Value::from_pyval(v))).collect())),
        _ => Err(KError::Py("TypeError".into(), "kaliru: a template context must be a dict".into())),
    }
}

/// PrinterGCodeMacro.env (one, as Kalico's single gcode_macro object has)
fn macro_env() -> KResult<PyRef> {
    thread_local!(static ENV: std::cell::RefCell<Option<PyRef>> = const { std::cell::RefCell::new(None) });
    if let Some(e) = ENV.with(|e| e.borrow().clone()) {
        return Ok(e);
    }
    let env = crate::plugins::py::lookup("_k", "macro_jinja_env")?.call(&[])?;
    ENV.with(|e| *e.borrow_mut() = Some(env.clone()));
    Ok(env)
}
