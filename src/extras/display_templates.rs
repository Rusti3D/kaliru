// Display templates: the [display_template] / [display_data] /
// [display_glyph] handling of klippy/extras/display/display.py
// (DisplayTemplate, DisplayGroup, PrinterDisplayTemplate,
// lookup_display_templates). The LCD part (PrinterLCD, the chip drivers,
// menu) is the "display" module, not ported yet.
//
// Copyright (C) 2018-2022  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018  Aleph Objects, Inc <marcio@alephobjects.com>
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::gcode_macro::{load_gcode_macro, Template};
use crate::jinja::value::Value;
use crate::printer::{downcast_rc, ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyfmt::py_strip;
use indexmap::IndexMap;
use std::any::Any;
use std::rc::Rc;

/// klippy/extras/display/display.cfg (python reads it next to display.py)
const DISPLAY_CFG: &str = include_str!("display.cfg");
const DISPLAY_CFG_FILENAME: &str = "klippy/extras/display/display.cfg";

/// ast.literal_eval(s) as a jinja value; Err is (class, message)
pub fn literal_eval(src: &str) -> Result<Value, (String, String)> {
    crate::pyliteral::literal_eval_json(src).map_err(|(c, m)| (c.to_string(), m))
}

// Storage of [display_template my_template] config sections
pub struct DisplayTemplate {
    pub name: String,
    params: IndexMap<String, Value>,
    template: Rc<Template>,
}

impl DisplayTemplate {
    fn new(config: &ConfigWrapper) -> KResult<Self> {
        let name_parts: Vec<&str> = config.get_name().split_whitespace().collect();
        if name_parts.len() != 2 {
            return Err(KError::config(format!("Section name '{}' is not valid", config.get_name())));
        }
        let name = name_parts[1].to_string();
        let mut params = IndexMap::new();
        for option in config.get_prefix_options("param_")? {
            match literal_eval(&config.get(&option, Dflt::Required)?) {
                Ok(v) => {
                    params.insert(option, v);
                }
                Err((cls, msg)) => {
                    if cls == "ValueError" {
                        return Err(KError::config(format!(
                            "Option '{}' in section '{}' is not a valid literal",
                            option,
                            config.get_name()
                        )));
                    }
                    return Err(KError::Other(format!("{}: {}", cls, msg)));
                }
            }
        }
        let gcode_macro = load_gcode_macro(config)?;
        let template = gcode_macro.load_template(config, "text", None)?;
        Ok(DisplayTemplate { name, params, template })
    }

    pub fn get_params(&self) -> &IndexMap<String, Value> {
        &self.params
    }

    pub fn render(&self, context: &IndexMap<String, Value>, kwargs: &IndexMap<String, Value>) -> KResult<String> {
        let mut params = self.params.clone();
        for (k, v) in kwargs {
            params.insert(k.clone(), v.clone());
        }
        if params.len() != self.params.len() {
            return Err(KError::command(format!("Invalid parameter to display_template {}", self.name)));
        }
        let mut context = context.clone();
        context.extend(params);
        self.template.render(Some(context))
    }
}

// Store [display_data my_group my_item] sections (one instance per group name)
pub struct DisplayGroup {
    pub data_items: Vec<(i64, i64, Rc<Template>)>,
}

impl DisplayGroup {
    fn new(config: &ConfigWrapper, _name: &str, data_configs: &[ConfigWrapper]) -> KResult<Self> {
        // Load and parse the position of display_data items
        let mut items = vec![];
        for c in data_configs {
            let pos = c.get("position", Dflt::Required)?;
            let parsed: Option<Vec<i64>> = pos.split(',').map(|v| crate::pyfmt::py_int(py_strip(v))).collect();
            match parsed.as_deref() {
                Some([row, col]) => items.push((*row, *col, c.get_name().to_string())),
                _ => return Err(KError::config(format!("Unable to parse 'position' in section '{}'", c.get_name()))),
            }
        }
        // Load all templates and store sorted by display position
        let configs_by_name: IndexMap<String, &ConfigWrapper> = data_configs.iter().map(|c| (c.get_name().to_string(), c)).collect();
        let gcode_macro = load_gcode_macro(config)?;
        let mut data_items = vec![];
        items.sort();
        for (row, col, name) in items {
            let c = configs_by_name[&name];
            if !c.get("text", Dflt::Required)?.is_empty() {
                let template = gcode_macro.load_template(c, "text", None)?;
                data_items.push((row, col, template));
            }
        }
        Ok(DisplayGroup { data_items })
    }
}

/// the icons of a [display_glyph]: icon16x16 (two byte columns) and
/// icon5x8 (hd44780 slot, rows)
#[derive(Default, Clone)]
pub struct Glyph {
    pub icon16x16: Option<(Vec<i64>, Vec<i64>)>,
    pub icon5x8: Option<(i64, Vec<i64>)>,
}

// Global cache of DisplayTemplate, DisplayGroup, and glyphs
pub struct PrinterDisplayTemplate {
    pub display_templates: IndexMap<String, Rc<DisplayTemplate>>,
    pub display_data_groups: IndexMap<String, DisplayGroup>,
    pub display_glyphs: IndexMap<String, Glyph>,
}

fn parse_glyph(glyph_name: &str, data: &str, width: usize, height: usize) -> KResult<Vec<i64>> {
    let mut glyph_data = vec![];
    for line in data.split('\n') {
        let line = py_strip(line).replace('.', "0").replace('*', "1");
        if line.is_empty() {
            continue;
        }
        if line.chars().count() != width || !line.replace('0', "").replace('1', "").is_empty() {
            return Err(KError::config(format!("Invalid glyph line in {}", glyph_name)));
        }
        glyph_data.push(i64::from_str_radix(&line, 2).unwrap_or(0));
    }
    if glyph_data.len() != height {
        return Err(KError::config(format!("Glyph {} incorrect lines", glyph_name)));
    }
    Ok(glyph_data)
}

impl PrinterDisplayTemplate {
    fn new(config: &ConfigWrapper) -> KResult<Self> {
        let printer = config.get_printer();
        // Load default display config file
        let pconfig = printer.lookup::<ConfigFileObject>("configfile")?;
        let dconfig = pconfig
            .pconfig
            .borrow()
            .build_config_wrapper(DISPLAY_CFG, DISPLAY_CFG_FILENAME)
            .map_err(|_| KError::config(format!("Cannot load config '{}'", DISPLAY_CFG_FILENAME)))?
            .with_printer(&printer);
        // sections of the main config, then the defaults not overridden
        let merged = |prefix: &str| -> Vec<ConfigWrapper> {
            let main = config.get_prefix_sections(prefix);
            let names: Vec<String> = main.iter().map(|c| c.get_name().to_string()).collect();
            let mut out = main;
            out.extend(dconfig.get_prefix_sections(prefix).into_iter().filter(|c| !names.iter().any(|n| n == c.get_name())));
            out
        };
        // Load display_template sections
        let mut display_templates = IndexMap::new();
        for c in merged("display_template ") {
            let dt = DisplayTemplate::new(&c)?;
            display_templates.insert(dt.name.clone(), Rc::new(dt));
        }
        // Load display_data sections
        let mut groups: IndexMap<String, Vec<ConfigWrapper>> = IndexMap::new();
        for c in merged("display_data ") {
            let name_parts: Vec<String> = c.get_name().split_whitespace().map(|s| s.to_string()).collect();
            if name_parts.len() != 3 {
                return Err(KError::config(format!("Section name '{}' is not valid", c.get_name())));
            }
            groups.entry(name_parts[1].clone()).or_default().push(c);
        }
        let mut display_data_groups = IndexMap::new();
        for (group_name, data_configs) in &groups {
            display_data_groups.insert(group_name.clone(), DisplayGroup::new(config, group_name, data_configs)?);
        }
        // Load display glyphs
        let dg_prefix = "display_glyph ";
        let mut icons: IndexMap<String, Glyph> = IndexMap::new();
        for dg in merged(dg_prefix) {
            let glyph_name = dg.get_name()[dg_prefix.len()..].to_string();
            if let Some(data) = dg.get_opt("data")? {
                let idata = parse_glyph(&glyph_name, &data, 16, 16)?;
                let icon1 = idata.iter().map(|bits| (bits >> 8) & 0xFF).collect();
                let icon2 = idata.iter().map(|bits| bits & 0xFF).collect();
                icons.entry(glyph_name.clone()).or_default().icon16x16 = Some((icon1, icon2));
            }
            if let Some(data) = dg.get_opt("hd44780_data")? {
                let slot = dg.getint("hd44780_slot", Dflt::Required, Lim::new().min(0).max(7))?;
                let idata = parse_glyph(&glyph_name, &data, 5, 8)?;
                icons.entry(glyph_name.clone()).or_default().icon5x8 = Some((slot, idata));
            }
        }
        Ok(PrinterDisplayTemplate { display_templates, display_data_groups, display_glyphs: icons })
    }
}

impl PrinterObject for PrinterDisplayTemplate {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

pub fn lookup_display_templates(config: &ConfigWrapper) -> KResult<Rc<PrinterDisplayTemplate>> {
    let printer: Rc<Printer> = config.get_printer();
    if let Some(o) = printer.lookup_object_opt("display_template") {
        return downcast_rc::<PrinterDisplayTemplate>(o).ok_or_else(|| KError::Other("display_template has an unexpected type".into()));
    }
    let dt = Rc::new(PrinterDisplayTemplate::new(config)?);
    let obj: PObj = dt.clone();
    printer.add_object("display_template", obj)?;
    Ok(dt)
}
