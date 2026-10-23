// Save arbitrary variables so that values can be kept across restarts
// (port of klippy/extras/save_variables.py)
//
// Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
// Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::configparser::{BasicInterpolation, RawConfigParser};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::jinja::value::Value;
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use crate::pyglob::expanduser;
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::Rc;

const CMD_SAVE_VARIABLE_HELP: &str = "Save arbitrary variables to disk";

pub struct SaveVariables {
    filename: String,
    pub all_variables: RefCell<IndexMap<String, Value>>,
}

/// a configparser.ConfigParser()
fn new_parser() -> RawConfigParser {
    RawConfigParser::new(true, &[], Box::new(BasicInterpolation))
}

impl SaveVariables {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let filename = expanduser(&config.get("filename", Dflt::Required)?);
        let s = Rc::new(SaveVariables { filename, all_variables: RefCell::new(IndexMap::new()) });
        let r = (|| -> KResult<()> {
            if !std::path::Path::new(&s.filename).exists() {
                std::fs::File::create(&s.filename).map_err(|e| KError::os(&e, &s.filename))?;
            }
            s.load_variables()
        })();
        if let Err(e) = r {
            if e.is_command_error() {
                return Err(KError::config(e.message().to_string()).with_context(e));
            }
            return Err(e);
        }
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let ss = s.clone();
        gcode.register_command("SAVE_VARIABLE", Some(Rc::new(move |g| ss.cmd_save_variable(g))), false, Some(CMD_SAVE_VARIABLE_HELP))?;
        Ok(s)
    }

    fn load_variables(&self) -> KResult<()> {
        let r = (|| -> Result<IndexMap<String, Value>, String> {
            let mut allvars = IndexMap::new();
            let mut varfile = new_parser();
            // read() skips a file it cannot open
            if let Ok(text) = std::fs::read_to_string(&self.filename) {
                varfile.read_string(&text, &self.filename).map_err(|e| e.to_string())?;
            }
            if varfile.has_section("Variables") {
                for (name, val) in varfile.items("Variables").map_err(|e| e.to_string())? {
                    let v = crate::pyliteral::literal_eval_json(&val).map_err(|(c, m)| format!("{}: {}", c, m))?;
                    allvars.insert(name, v);
                }
            }
            Ok(allvars)
        })();
        match r {
            Ok(allvars) => {
                *self.all_variables.borrow_mut() = allvars;
                Ok(())
            }
            Err(e) => {
                let msg = "Unable to parse existing variable file";
                logging::exception_tb(msg, "Exception", &e);
                Err(KError::command(msg))
            }
        }
    }

    fn cmd_save_variable(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let varname = gcmd.get("VARIABLE")?;
        let value = gcmd.get("VALUE")?;
        let value = match crate::pyliteral::literal_eval_json(&value) {
            Ok(v) => v,
            Err(("ValueError", _)) => return Err(gcmd.error(format!("Unable to parse '{}' as a literal", value))),
            // a SyntaxError is not caught by the python handler
            Err((cls, m)) => return Err(KError::Py(cls.to_string(), m.to_string())),
        };
        let mut newvars = self.all_variables.borrow().clone();
        newvars.insert(varname, value);
        // Write file
        let mut varfile = new_parser();
        varfile.add_section("Variables").map_err(|e| KError::Other(e.to_string()))?;
        let mut items: Vec<(String, Value)> = newvars.into_iter().collect();
        items.sort_by(|a, b| a.0.cmp(&b.0));
        for (name, val) in items {
            // ConfigParser.set raises ValueError on a stray '%'
            varfile.set("Variables", &name, &val.repr()).map_err(|e| KError::Other(e.to_string()))?;
        }
        if let Err(e) = std::fs::write(&self.filename, varfile.write()) {
            let msg = "Unable to save variable";
            logging::exception_tb(msg, "OSError", &e.to_string());
            return Err(gcmd.error(msg));
        }
        self.load_variables()
    }
}

impl PrinterObject for SaveVariables {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let vars: IndexMap<String, PyVal> = self.all_variables.borrow().iter().map(|(k, v)| (k.clone(), v.to_pyval())).collect();
        let mut d = IndexMap::new();
        d.insert("variables".to_string(), PyVal::Dict(vars));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(SaveVariables::new(config)?)
}
