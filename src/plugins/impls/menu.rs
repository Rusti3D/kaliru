// klippy/extras/display/menu: what kaliru's menu (one MenuItem type for
// every Kalico menu class) exposes of Kalico's MenuElement / MenuContainer /
// MenuInput / MenuDialog / MenuFileBrowser / MenuManager, and MenuKeys
use super::*;
use crate::extras::display::menu::{Ctx, MenuItem, MenuManager};
use crate::plugins::convert::ToPy;
use crate::plugins::values::value_to_py;

fn ctx_to_py(c: &Ctx) -> KResult<PyRef> {
    let d = lookup("builtins", "dict")?.call(&[])?;
    for (k, v) in c {
        d.call_method("__setitem__", &[&py_str(k), &value_to_py(v)?])?;
    }
    Ok(d)
}

/// a context python passes: a dict or None
fn ctx_from_py(o: &PyRef) -> KResult<Option<Ctx>> {
    super::extras_misc::template_context(Some(o.clone()))
}

fn path_to_py(p: &Option<std::path::PathBuf>) -> KResult<PyRef> {
    match p {
        Some(p) => lookup("pathlib", "Path")?.call(&[&py_str(&p.to_string_lossy())]),
        None => Ok(py_none()),
    }
}

pub fn register(r: &mut Registry) {
    // get_context(cxt=None)
    fn item_context(a: &crate::plugins::native::Args) -> KResult<PyRef> {
        let cxt = ctx_from_py(a.arg(0))?;
        ctx_to_py(&a.this::<MenuItem>()?.get_context(cxt.as_ref())?)
    }
    r.method("klippy.extras.display.menu.MenuElement.get_context", item_context);
    r.method("klippy.extras.display.menu.MenuInput.get_context", item_context);
    r.method("klippy.extras.display.menu.MenuManager.get_context", |a| {
        let cxt = ctx_from_py(a.arg(0))?;
        ctx_to_py(&a.this::<MenuManager>()?.get_context(cxt.as_ref()))
    });
    r.attr("klippy.extras.display.menu.MenuManager.context", |o| ctx_to_py(&objects::unwrap_as::<MenuManager>(o)?.context.borrow()), None);
    r.attr("klippy.extras.display.menu.MenuManager.pconfig", |o| {
        objects::unwrap_as::<MenuManager>(o)?.printer.lookup_object("configfile")?.to_py()
    }, None);
    r.method("klippy.extras.display.menu.MenuManager.timeout_check", |a| {
        a.this::<MenuManager>()?.timeout_check()?;
        Ok(py_none())
    });
    r.method("klippy.extras.display.menu.MenuElement.eval_enable", |a| {
        let ctx = ctx_from_py(a.arg(0))?.unwrap_or_default();
        Ok(py_bool(a.this::<MenuItem>()?.eval_enable(&ctx)))
    });
    // run_script(name, event=None, context=None, render_only=False)
    r.method("klippy.extras.display.menu.MenuElement.run_script", |a| {
        let event = a.opt_string(1)?;
        let ctx = ctx_from_py(a.arg(2))?;
        Ok(py_str(&a.this::<MenuItem>()?.run_script(&a.string(0)?, event.as_deref(), ctx.as_ref(), a.bool(3)?)?))
    });
    // (set to None in __init__ and never changed)
    r.attr("klippy.extras.display.menu.MenuElement.dialog", |_o| Ok(py_none()), None);
    r.attr("klippy.extras.display.menu.MenuContainer.selected", |o| objects::unwrap_as::<MenuItem>(o)?.container.selected.get().to_py(), None);
    r.method("klippy.extras.display.menu.MenuInput.inc_value", |a| {
        a.this::<MenuItem>()?.step_value(a.bool(0)?, true);
        Ok(py_none())
    });
    r.method("klippy.extras.display.menu.MenuInput.dec_value", |a| {
        a.this::<MenuItem>()?.step_value(a.bool(0)?, false);
        Ok(py_none())
    });
    // the current dialog's value for this input
    r.method("klippy.extras.display.menu.MenuInput.handle_script_change", |a| {
        let s = a.this::<MenuItem>()?;
        if let Some(d) = s.manager().current_dialog() {
            let v = s.input.as_ref().and_then(|i| i.input_value.get());
            d.dialog_set_value(&s.id, crate::jinja::value::Value::from_pyval(&v.map(crate::pyval::PyVal::Float).unwrap_or(crate::pyval::PyVal::None)))?;
        }
        Ok(py_none())
    });
    r.method("klippy.extras.display.menu.MenuDialog.get_values", |a| ctx_to_py(&a.this::<MenuItem>()?.dialog_values()));
    // get_value(ns, default=None)
    r.method("klippy.extras.display.menu.MenuDialog.get_value", |a| {
        match a.this::<MenuItem>()?.dialog_values().get(&a.string(0)?) {
            Some(v) => value_to_py(v),
            None => Ok(a.arg(1).clone()),
        }
    });
    r.method("klippy.extras.display.menu.MenuDialog.set_value", |a| {
        a.this::<MenuItem>()?.dialog_set_value(&a.string(0)?, crate::plugins::values::py_to_value(a.arg(1))?)?;
        Ok(py_none())
    });
    r.attr("klippy.extras.display.menu.MenuFileBrowser.path", |o| path_to_py(&objects::unwrap_as::<MenuItem>(o)?.fb.as_ref().and_then(|f| f.path.clone())), None);
    r.attr("klippy.extras.display.menu.MenuFileBrowser.root_dir", |o| path_to_py(&objects::unwrap_as::<MenuItem>(o)?.fb.as_ref().and_then(|f| f.root_dir.clone())), None);
    // "" once a path came in the context
    r.attr("klippy.extras.display.menu.MenuFileBrowser.prefix", |o| {
        let s = objects::unwrap_as::<MenuItem>(o)?;
        match &s.context {
            Some(c) if c.contains_key("path") => Ok(py_str("")),
            _ => Err(KError::Py("AttributeError".into(), "prefix".into())),
        }
    }, None);

    // menu_keys: callback(key, eventtime)
    {
        use crate::extras::display::menu_keys::MenuKeys;
        r.attr("klippy.extras.display.menu_keys.MenuKeys.callback", |o| {
            let cb = objects::unwrap_as::<MenuKeys>(o)?.callback.clone();
            crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
                cb(&a[0].as_string()?, a[1].as_f64()?)?;
                Ok(py_none())
            }))
        }, None);
        fn key(a: &crate::plugins::native::Args, key: &str) -> KResult<PyRef> {
            let k = a.this::<MenuKeys>()?;
            (k.callback)(key, a.f64(0)?)?;
            Ok(py_none())
        }
        r.method("klippy.extras.display.menu_keys.MenuKeys.back_callback", |a| key(a, "back"));
        r.method("klippy.extras.display.menu_keys.MenuKeys.up_callback", |a| key(a, "up"));
        r.method("klippy.extras.display.menu_keys.MenuKeys.down_callback", |a| key(a, "down"));
    }
}
