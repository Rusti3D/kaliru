// Python g-code macros on the host CPython (kaliru built without
// RustPython): Kalico's own gcode_macro.TemplateWrapperPython, emitted
// verbatim in the mirror with GetStatusWrapperPython and
// TemplateVariableWrapperPython (tools/mirror_verbatim.txt), so compile and
// run errors, helpers and logging are Kalico's.
use super::py::{self, PyRef};
use super::{ensure_started, objects};
use crate::error::KResult;
use crate::jinja::value::Value;
use crate::printer::Printer;
use crate::IndexMap;
use std::rc::Rc;

pub struct PyMacro {
    pub obj: PyRef,
}

impl PyMacro {
    /// TemplateWrapperPython(printer, env, name, script)
    pub fn new(printer: &Rc<Printer>, name: &str, script: &str) -> KResult<PyMacro> {
        ensure_started()?;
        let cls = py::lookup("klippy.extras.gcode_macro", "TemplateWrapperPython")?;
        let p = objects::wrap(printer.clone())?;
        let obj = cls.call(&[&p, &py::py_none(), &py::py_str(name), &py::py_str(script)])?;
        Ok(PyMacro { obj })
    }

    /// run_gcode_from_command(context): the macro's variables and params
    /// come from kaliru; the entries of create_template_context() (printer
    /// wrapper, action helpers) are python's own, as GCodeMacro.cmd builds
    /// the context in Kalico
    pub fn run_gcode_from_command(&self, context: Option<IndexMap<String, Value>>) -> KResult<()> {
        let ctx = match context {
            None => py::py_none(),
            Some(c) => {
                let template_ctx = self.obj.getattr("gcode_macro")?.call_method("create_template_context", &[])?;
                let d = py::lookup("builtins", "dict")?.call(&[])?;
                for (k, v) in &c {
                    let key = py::py_str(k);
                    let value = match template_ctx.call_method("get", &[&key])? {
                        t if !t.is_none() => t,
                        _ => super::values::value_to_py(v)?,
                    };
                    d.call_method("__setitem__", &[&key, &value])?;
                }
                d
            }
        };
        self.obj.call_method("run_gcode_from_command", &[&ctx])?;
        Ok(())
    }
}
