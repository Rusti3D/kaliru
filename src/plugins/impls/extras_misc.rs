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
use crate::plugins::convert::ToPy;
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
fn template_context(context: Option<PyRef>) -> KResult<Option<IndexMap<String, Value>>> {
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
