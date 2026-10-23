// Basic LCD menu support (port of klippy/extras/display/menu.py)
//
// Copyright (C) 2020  Janar Sööt <janar.soot@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The python class hierarchy (MenuElement > MenuCommand > MenuInput,
// MenuContainer > MenuList > MenuDialog / MenuVSDList / MenuFileBrowser,
// MenuDisabled) is one MenuItem with a Kind; the subclass state lives in
// optional parts.
use super::menu_keys::MenuKeys;
use super::PrinterLCD;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::display_templates::literal_eval;
use crate::extras::gcode_macro::{create_template_context, load_gcode_macro, PrinterGCodeMacro, Template};
use crate::extras::virtual_sdcard::{VirtualSD, VALID_GCODE_EXTS};
use crate::gcode::GCodeDispatch;
use crate::jinja::value::{CallCtx, Key, Object, VResult, Value};
use crate::logging;
use crate::printer::{downcast_rc, ConfigFileObject, Printer, PrinterObject};
use crate::pyval::PyVal;
use crate::reactor::{NEVER, NOW};
use indexmap::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::path::PathBuf;
use std::rc::{Rc, Weak};

/// klippy/extras/display/menu.cfg (python reads it next to menu.py)
const MENU_CFG: &str = include_str!("menu.cfg");
const MENU_CFG_FILENAME: &str = "klippy/extras/display/menu.cfg";

const TIMER_DELAY: f64 = 1.0;

/// a jinja template context
pub type Ctx = IndexMap<String, Value>;

fn error(msg: impl Into<String>) -> KError {
    KError::Other(msg.into())
}

thread_local! {
    static ITEM_IDS: Cell<usize> = const { Cell::new(0) };
}

/// a unique id for a generated menu item (python uses id(self), an address)
fn next_item_id() -> usize {
    ITEM_IDS.with(|c| {
        c.set(c.get() + 1);
        c.get()
    })
}

/// a menu script: a template, a callback, or None (no caller passes the
/// static string python also allows)
#[derive(Clone)]
enum Script {
    None,
    Template(Rc<Template>),
    Callback(Rc<dyn Fn(&Rc<MenuItem>, &Ctx) -> KResult<()>>),
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Kind {
    Disabled,
    Command,
    Input,
    List,
    Dialog,
    VsdList,
    FileBrowser,
}

const MENU_ITEMS: [(&str, Kind); 7] = [
    ("disabled", Kind::Disabled),
    ("command", Kind::Command),
    ("input", Kind::Input),
    ("list", Kind::List),
    ("dialog", Kind::Dialog),
    ("vsdlist", Kind::VsdList),
    ("file_browser", Kind::FileBrowser),
];

fn kind_of(type_name: &str) -> KResult<Kind> {
    MENU_ITEMS.iter().find(|(n, _)| *n == type_name).map(|(_, k)| *k).ok_or_else(|| {
        let names: Vec<String> = MENU_ITEMS.iter().map(|(n, _)| format!("'{}': <class '{}'>", n, n)).collect();
        error(format!("Choice '{}' for option '{{{}}}' is not a valid choice", type_name, names.join(", ")))
    })
}

impl Kind {
    fn is_container(self) -> bool {
        matches!(self, Kind::List | Kind::Dialog | Kind::VsdList | Kind::FileBrowser)
    }
    /// isinstance(item, MenuList)
    fn is_list(self) -> bool {
        self.is_container()
    }
    /// isinstance(item, MenuCommand)
    fn is_command(self) -> bool {
        matches!(self, Kind::Command | Kind::Input)
    }
    fn default_suffix(self) -> &'static str {
        match self {
            Kind::List | Kind::Dialog | Kind::VsdList => ">",
            Kind::FileBrowser => "~folder~",
            _ => "",
        }
    }
}

/// the keyword arguments of menuitem_from()
#[derive(Default, Clone)]
pub struct Kwargs {
    name: Option<String>,
    suffix: Option<String>,
    gcode: Option<Script>,
    context: Option<Ctx>,
}

/// MenuContainer state
#[derive(Default)]
struct Container {
    selected: Cell<Option<i64>>,
    allitems: RefCell<Vec<(Rc<MenuItem>, String)>>,
    names: RefCell<Vec<String>>,
    items: RefCell<Vec<Rc<MenuItem>>>,
    parents: RefCell<Vec<Rc<MenuItem>>>,
    // MenuList
    viewport_top: Cell<i64>,
    item_back: RefCell<Option<Rc<MenuItem>>>,
}

/// MenuInput state
struct Input {
    input_min: Cell<Option<f64>>,
    input_max: Cell<Option<f64>>,
    input_step: f64,
    realtime: bool,
    input_tpl: Option<Rc<Template>>,
    input_min_tpl: Option<Rc<Template>>,
    input_max_tpl: Option<Rc<Template>>,
    is_dirty: Cell<bool>,
    last_change: Cell<Option<f64>>,
    input_value: Cell<Option<f64>>,
}

/// MenuDialog state
struct Dialog {
    values: RefCell<IndexMap<String, Value>>,
    title_item: Rc<MenuItem>,
    confirm_item: Rc<MenuItem>,
    cancel_item: Rc<MenuItem>,
}

/// MenuFileBrowser state
struct FileBrowser {
    sort_by: String,
    root_dir: Option<PathBuf>,
    path: Option<PathBuf>,
}

pub struct MenuItem {
    me: Weak<MenuItem>,
    manager: Weak<MenuManager>,
    pub kind: Kind,
    index: Option<i64>,
    enable: bool,
    name: Option<String>,
    context: Option<Ctx>,
    enable_tpl: Option<Rc<Template>>,
    name_tpl: RefCell<Option<Rc<Template>>>,
    suffix: String,
    suffix_tpl: Option<Rc<Template>>,
    id: String,
    ns: String,
    last_heartbeat: Cell<Option<f64>>,
    scroll_pos: Cell<Option<i64>>,
    /// True / False / None
    scroll_request_pending: Cell<Option<bool>>,
    scroll_next: Cell<f64>,
    scripts: RefCell<IndexMap<String, Script>>,
    container: Container,
    input: Option<Input>,
    dialog: RefCell<Option<Dialog>>,
    fb: Option<FileBrowser>,
}

/// the config source of a menu item: a config section or kwargs
enum Source<'a> {
    Config(&'a ConfigWrapper),
    Kwargs(&'a Kwargs),
}

fn load_tpl(gm: &PrinterGCodeMacro, config: &ConfigWrapper, option: &str, default: Option<&str>) -> KResult<Rc<Template>> {
    gm.load_template(config, option, default)
}

impl MenuItem {
    fn new(manager: &Rc<MenuManager>, kind: Kind, src: Source) -> KResult<Rc<MenuItem>> {
        let gm = manager.gcode_macro.clone();
        let kw = match &src {
            Source::Kwargs(k) => (*k).clone(),
            Source::Config(_) => Kwargs::default(),
        };
        // MenuDisabled passes name=""
        let name = if kind == Kind::Disabled { Some(String::new()) } else { kw.name.clone() };
        let mut index = None;
        let mut enable = true;
        let mut enable_tpl = None;
        let mut name_tpl = None;
        let mut suffix_tpl = None;
        let suffix = kw.suffix.clone().unwrap_or_else(|| kind.default_suffix().to_string());
        let (id, ns) = match &src {
            Source::Config(config) => {
                // overwrite class attributes from config
                index = config.getint_opt("index", Lim::new())?;
                name_tpl = Some(load_tpl(&gm, config, "name", name.as_deref())?);
                match config.getboolean("enable", true) {
                    Ok(v) => enable = v,
                    // (except config.error)
                    Err(_) => enable_tpl = Some(load_tpl(&gm, config, "enable", None)?),
                }
                suffix_tpl = Some(load_tpl(&gm, config, "suffix", Some(&suffix))?);
                // item namespace - used in relative paths
                let parts: Vec<&str> = config.get_name().split(' ').collect();
                (parts[parts.len() - 1].to_string(), parts[1..].join(" ").trim().to_string())
            }
            Source::Kwargs(k) => {
                // ns - item namespace key, used in item relative paths
                // $__id - generated id text variable (python: hex(id(self)))
                let id = format!("__menu_{:x}", next_item_id());
                let ns_kw = k.context.as_ref().and_then(|c| c.get("ns")).map(|v| v.to_str()).unwrap_or_else(|| id.clone());
                let ns = format!("menu {}", ns_kw).replace("$__id", &id);
                (id, ns)
            }
        };
        let mut input = None;
        if kind == Kind::Input {
            let (mut realtime, mut input_step) = (false, 1.0);
            let (mut input_tpl, mut input_min_tpl, mut input_max_tpl) = (None, None, None);
            if let Source::Config(config) = &src {
                realtime = config.getboolean("realtime", realtime)?;
                input_tpl = Some(load_tpl(&gm, config, "input", None)?);
                input_min_tpl = Some(load_tpl(&gm, config, "input_min", Some("-999999.0"))?);
                input_max_tpl = Some(load_tpl(&gm, config, "input_max", Some("999999.0"))?);
                input_step = config.getfloat("input_step", input_step, Lim::new().above(0.0))?;
            }
            input = Some(Input {
                input_min: Cell::new(Some(-999999.0)),
                input_max: Cell::new(Some(999999.0)),
                input_step,
                realtime,
                input_tpl,
                input_min_tpl,
                input_max_tpl,
                is_dirty: Cell::new(false),
                last_change: Cell::new(None),
                input_value: Cell::new(None),
            });
        }
        let mut fb = None;
        if kind == Kind::FileBrowser {
            let sort_by = match &src {
                Source::Config(config) => config.get("sort_by", "last_modified")?,
                Source::Kwargs(k) => k.context.as_ref().and_then(|c| c.get("sort_by")).map(|v| v.to_str()).unwrap_or_else(|| "last_modified".into()),
            }
            .to_lowercase();
            let mut f = FileBrowser { sort_by, root_dir: None, path: None };
            if let Some(sdcard) = manager.virtual_sdcard.clone() {
                let root = PathBuf::from(&sdcard.sdcard_dirname);
                f.path = Some(root.clone());
                f.root_dir = Some(root);
                if let Some(p) = kw.context.as_ref().and_then(|c| c.get("path")) {
                    f.path = Some(PathBuf::from(p.to_str()));
                }
            }
            fb = Some(f);
        }
        let item = Rc::new_cyclic(|me: &Weak<MenuItem>| MenuItem {
            me: me.clone(),
            manager: Rc::downgrade(manager),
            kind,
            index,
            enable,
            name,
            context: kw.context.clone(),
            enable_tpl,
            name_tpl: RefCell::new(name_tpl),
            suffix,
            suffix_tpl,
            id,
            ns,
            last_heartbeat: Cell::new(None),
            scroll_pos: Cell::new(None),
            scroll_request_pending: Cell::new(Some(false)),
            scroll_next: Cell::new(0.0),
            scripts: RefCell::new(IndexMap::new()),
            container: Container::default(),
            input,
            dialog: RefCell::new(None),
            fb,
        });
        // MenuCommand gcode script
        if kind.is_command() {
            item.load_script(&src, "gcode")?;
        }
        if kind.is_list() {
            // create back item
            let back = match (&item.fb, kw.context.as_ref().and_then(|c| c.get("path"))) {
                (Some(f), Some(_)) if f.root_dir.is_some() => manager.menuitem_from(
                    "command",
                    Kwargs {
                        name: Some(f.path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().into_owned()).unwrap_or_default()),
                        suffix: Some("~folder_up~".into()),
                        gcode: Some(Script::Callback(Rc::new(|el: &Rc<MenuItem>, _ctx: &Ctx| el.manager().back(false, true)))),
                        context: None,
                    },
                )?,
                _ => manager.menuitem_from(
                    "command",
                    Kwargs {
                        name: Some("..".into()),
                        gcode: Some(Script::Callback(Rc::new(|el: &Rc<MenuItem>, _ctx: &Ctx| el.manager().back(false, true)))),
                        ..Default::default()
                    },
                )?,
            };
            *item.container.item_back.borrow_mut() = Some(back);
        }
        if kind == Kind::Dialog {
            item.load_script(&src, "gcode")?;
            let title_text = item.name.clone().unwrap_or_else(|| "None".into());
            let mk = |name: &str, gcode: Option<Script>| manager.menuitem_from("command", Kwargs { name: Some(name.to_string()), gcode, ..Default::default() });
            let w = item.me.clone();
            let title_item = mk(&title_text, None)?;
            let confirm_item = mk(
                "Confirm",
                Some(Script::Callback(Rc::new(move |_el: &Rc<MenuItem>, _ctx: &Ctx| match w.upgrade() {
                    Some(d) => d.run_script("gcode", None, None, false).map(|_| ()),
                    None => Ok(()),
                }))),
            )?;
            let cancel_item = mk("Cancel", Some(Script::Callback(Rc::new(|el: &Rc<MenuItem>, _ctx: &Ctx| el.manager().back(false, true)))))?;
            if let Source::Config(config) = &src {
                if config.get_opt("title")?.map(|t| !t.is_empty()).unwrap_or(false) {
                    *title_item.name_tpl.borrow_mut() = Some(load_tpl(&gm, config, "title", Some(&title_text))?);
                } else {
                    *title_item.name_tpl.borrow_mut() = item.name_tpl.borrow().clone();
                }
                *confirm_item.name_tpl.borrow_mut() = Some(load_tpl(&gm, config, "confirm_text", Some("Confirm"))?);
                *cancel_item.name_tpl.borrow_mut() = Some(load_tpl(&gm, config, "cancel_text", Some("Cancel"))?);
            }
            *item.dialog.borrow_mut() = Some(Dialog { values: RefCell::new(IndexMap::new()), title_item, confirm_item, cancel_item });
        }
        Ok(item)
    }

    fn manager(&self) -> Rc<MenuManager> {
        self.manager.upgrade().expect("menu manager")
    }

    fn rc(&self) -> Rc<MenuItem> {
        self.me.upgrade().expect("menu item")
    }

    // override
    fn init(&self) {
        if self.kind.is_container() {
            // recursive guard
            self.container.parents.borrow_mut().clear();
        }
        if let Some(i) = &self.input {
            i.is_dirty.set(false);
            i.last_change.set(None);
            i.input_value.set(None);
        }
    }

    fn render_name_raw(&self) -> KResult<String> {
        let tpl = self.name_tpl.borrow().clone();
        if let Some(tpl) = tpl {
            let context = self.get_context(None)?;
            return Ok(MenuManager::asflat(&tpl.render(Some(context))?));
        }
        Ok(MenuManager::asflat(self.name.as_deref().unwrap_or("None")))
    }

    fn render_suffix(&self) -> KResult<String> {
        if let Some(tpl) = &self.suffix_tpl {
            let context = self.get_context(None)?;
            return Ok(MenuManager::asflat(&tpl.render(Some(context))?));
        }
        Ok(MenuManager::asflat(&self.suffix))
    }

    /// Load script template from config or callback from dict
    fn load_script(&self, src: &Source, name: &str) -> KResult<()> {
        if self.scripts.borrow().contains_key(name) {
            logging::info(&format!("Declaration of '{}' hides previous script declaration", name));
        }
        let script = match src {
            Source::Kwargs(k) => k.gcode.clone().unwrap_or(Script::None),
            Source::Config(config) => Script::Template(self.manager().gcode_macro.load_template(config, name, Some(""))?),
        };
        self.scripts.borrow_mut().insert(name.to_string(), script);
        Ok(())
    }

    fn is_editing(&self) -> bool {
        if self.kind.is_container() {
            return self.container.items.borrow().iter().any(|i| i.is_editing());
        }
        if let Some(i) = &self.input {
            return i.input_value.get().is_some();
        }
        false
    }

    fn is_scrollable(&self) -> bool {
        self.kind != Kind::Input
    }

    fn is_enabled(&self) -> bool {
        match self.kind {
            Kind::Disabled => false,
            Kind::FileBrowser => self.manager().virtual_sdcard.is_some() && self.eval_enable_ctx(),
            _ => self.eval_enable_ctx(),
        }
    }

    fn eval_enable_ctx(&self) -> bool {
        // (MenuInput.is_enabled uses the element context, without input)
        match self.element_context(None) {
            Ok(context) => self.eval_enable(&context),
            Err(e) => {
                logging::exception_err("Menu item enable valuation error", &e);
                false
            }
        }
    }

    fn stop_editing(&self) -> KResult<()> {
        if self.kind.is_container() {
            let items = self.container.items.borrow().clone();
            for item in items {
                if item.is_editing() {
                    item.stop_editing()?;
                }
            }
            return Ok(());
        }
        let Some(i) = &self.input else { return Ok(()) };
        if !self.is_editing() {
            return Ok(());
        }
        if i.is_dirty.get() {
            self.run_script("change", None, None, false)?;
        }
        i.input_value.set(None);
        Ok(())
    }

    fn start_editing(&self) -> KResult<()> {
        if self.input.is_none() || self.is_editing() {
            return Ok(());
        }
        self.init_value()
    }

    /// MenuElement.get_context(cxt)
    fn element_context(&self, cxt: Option<&Ctx>) -> KResult<Ctx> {
        // get default menu context
        let mut context = self.manager().get_context(cxt);
        if let Some(c) = &self.context {
            context.extend(c.clone());
        }
        let menu = context.get("menu").cloned().ok_or_else(|| KError::Other("'menu'".into()))?;
        dict_set(&menu, "ns", Value::str(&self.get_ns(".")));
        Ok(context)
    }

    pub fn get_context(&self, cxt: Option<&Ctx>) -> KResult<Ctx> {
        let context = self.element_context(cxt)?;
        if let Some(i) = &self.input {
            let value = match i.input_value.get() {
                None => self.get_value_ctx(&context).map(Value::Float).unwrap_or(Value::None),
                Some(v) => Value::Float(v),
            };
            if let Some(menu) = context.get("menu") {
                dict_set(menu, "input", value);
            }
        }
        Ok(context)
    }

    /// _eval_tpl(tpl, context, coerce)
    fn eval_tpl(&self, tpl: &Template, context: &Ctx) -> KResult<Value> {
        let stmt = match tpl.render(Some(context.clone())) {
            Ok(s) => s,
            Err(e) => {
                logging::exception_err(&format!("{} Failed to render template {}", self.ns, "<klippy.extras.gcode_macro.Template object>"), &e);
                return Err(e);
            }
        };
        if stmt.is_empty() {
            return Ok(Value::None);
        }
        literal_eval(&stmt).map_err(|(cls, msg)| {
            let e = KError::Py(cls.to_string(), msg.to_string());
            logging::exception_tb(&format!("{} Failed to evaluate {}", self.ns, crate::pyfmt::py_str_repr(&stmt)), &cls, &msg);
            e
        })
    }

    fn eval_tpl_float(&self, tpl: &Template, context: &Ctx) -> KResult<f64> {
        let val = self.eval_tpl(tpl, context)?;
        let f = match &val {
            Value::Int(i) => Some(*i as f64),
            Value::Float(f) => Some(*f),
            Value::Bool(b) => Some(*b as i64 as f64),
            Value::Str(s) => crate::pyfmt::py_float(s),
            _ => None,
        };
        f.ok_or_else(|| {
            let msg = format!("{} Failed to coerce {} as <class 'float'>", self.ns, val.repr());
            logging::exception_tb(&msg, "TypeError", "float() argument must be a string or a real number");
            KError::Other("float() argument must be a string or a real number".into())
        })
    }

    fn eval_enable(&self, context: &Ctx) -> bool {
        if let Some(tpl) = &self.enable_tpl {
            return match self.eval_tpl(tpl, context) {
                Ok(v) => v.truthy(),
                Err(e) => {
                    logging::exception_err("Menu item enable valuation error", &e);
                    false
                }
            };
        }
        self.enable
    }

    // Called when a item is selected
    fn select(&self) {
        self.reset_scroller();
    }

    fn heartbeat(&self, eventtime: f64) -> KResult<()> {
        self.last_heartbeat.set(Some(eventtime));
        if eventtime >= self.scroll_next.get() {
            self.scroll_next.set(eventtime + 0.5);
            if !self.is_editing() {
                self.update_scroller();
            }
        }
        if let Some(i) = &self.input {
            if i.is_dirty.get()
                && i.last_change.get().is_some()
                && i.input_value.get().is_some()
                && (eventtime - i.last_change.get().unwrap_or(0.0)) > 0.250
            {
                if i.realtime {
                    self.run_script("gcode", Some("change"), None, false)?;
                    self.run_script("change", None, None, false)?;
                }
                i.is_dirty.set(false);
            }
        }
        Ok(())
    }

    fn update_scroller(&self) {
        match (self.scroll_pos.get(), self.scroll_request_pending.get()) {
            (None, Some(true)) => self.scroll_pos.set(Some(0)),
            (Some(p), Some(true)) => {
                self.scroll_pos.set(Some(p + 1));
                self.scroll_request_pending.set(Some(false));
            }
            (_, Some(false)) => {} // hold scroll position
            (_, None) => self.reset_scroller(),
        }
    }

    fn reset_scroller(&self) {
        self.scroll_pos.set(None);
        self.scroll_request_pending.set(Some(false));
    }

    /// Allows to control the scroller: True - inc. scroll pos. on next
    /// update, False - hold scroll pos., None - reset the scroller
    fn need_scroller(&self, value: Option<bool>) {
        self.scroll_request_pending.set(value);
    }

    fn slice_name(name: &str, index: i64) -> String {
        // re.split(r"(\~.*?\~)", name): text chunks count per character,
        // glyph placeholders as one
        let mut chunks: Vec<String> = vec![];
        let mut rest = name;
        loop {
            match rest.find('~').and_then(|s| rest[s + 1..].find('~').map(|e| (s, s + 1 + e))) {
                Some((s, e)) => {
                    chunks.extend(rest[..s].chars().map(|c| c.to_string()));
                    chunks.push(rest[s..=e].to_string());
                    rest = &rest[e + 1..];
                }
                None => {
                    chunks.extend(rest.chars().map(|c| c.to_string()));
                    break;
                }
            }
        }
        let start = if index < 0 { (chunks.len() as i64 + index).max(0) as usize } else { (index as usize).min(chunks.len()) };
        chunks[start..].concat()
    }

    fn render_name(&self, selected: bool) -> KResult<String> {
        let mut name = self.render_name_raw()?;
        match (selected, self.scroll_pos.get()) {
            (true, Some(pos)) => name = Self::slice_name(&name, pos),
            _ => self.reset_scroller(),
        }
        Ok(name)
    }

    pub fn get_ns(&self, name: &str) -> String {
        let name = name.trim();
        let name = if let Some(rest) = name.strip_prefix("..") {
            let parts: Vec<&str> = self.ns.split(' ').collect();
            format!("{} {}", parts[..parts.len() - 1].join(" "), rest)
        } else if let Some(rest) = name.strip_prefix('.') {
            format!("{} {}", self.ns, rest)
        } else {
            name.to_string()
        };
        name.trim().to_string()
    }

    fn send_event(&self, event: &str) -> KResult<()> {
        let me = self.rc();
        self.manager().send_event(&format!("{}:{}", self.get_ns("."), event), &[&me])
    }

    fn run_one_script(&self, name: &str, context: &Ctx) -> KResult<String> {
        let script = self.scripts.borrow().get(name).cloned().unwrap_or(Script::None);
        match script {
            // check template
            Script::Template(t) => t.render(Some(context.clone())),
            // check callback
            Script::Callback(cb) => {
                cb(&self.rc(), context)?;
                Ok(String::new())
            }
            Script::None => Ok(String::new()),
        }
    }

    fn run_script(&self, name: &str, event: Option<&str>, context: Option<&Ctx>, render_only: bool) -> KResult<String> {
        let mut result = String::new();
        // init context
        if self.scripts.borrow().contains_key(name) {
            let context = self.get_context(context)?;
            if let Some(menu) = context.get("menu") {
                dict_set(menu, "event", Value::str(event.unwrap_or(name)));
            }
            result = self.run_one_script(name, &context)?;
        }
        if !render_only {
            // run result as gcode
            self.manager().queue_gcode(&result);
            // default behaviour
            match name {
                "change" if self.kind == Kind::Input => {
                    if let Some(dialog) = self.manager().current_dialog() {
                        let v = self.input.as_ref().and_then(|i| i.input_value.get()).map(Value::Float).unwrap_or(Value::None);
                        dialog.dialog_set_value(&self.id, v)?;
                    }
                }
                "click" if self.kind == Kind::Input => {
                    if !self.is_editing() {
                        self.start_editing()?;
                    } else {
                        self.stop_editing()?;
                    }
                }
                "gcode" if self.kind == Kind::Dialog => self.manager().back(false, true)?,
                _ => {}
            }
        }
        Ok(result)
    }

    fn cursor(&self) -> &'static str {
        ">"
    }

    // MenuInput

    fn eval_bound(&self, tpl: &Option<Rc<Template>>, fallback: Option<f64>, context: &Ctx, what: &str) -> Option<f64> {
        let r = match tpl {
            Some(t) => self.eval_tpl_float(t, context),
            None => Ok(fallback.unwrap_or(0.0)),
        };
        match r {
            Ok(v) => Some(v),
            // ponytail: python only catches ValueError / command_error here
            Err(e) => {
                let msg = if what.is_empty() { "Input value evaluation error".to_string() } else { format!("Input {} value evaluation error", what) };
                logging::exception_err(&msg, &e);
                None
            }
        }
    }

    fn get_value_ctx(&self, context: &Ctx) -> Option<f64> {
        if let Some(Value::Dict(d)) = context.get("dialog") {
            if let Some(v) = d.borrow().get(&Key::str(&self.id)) {
                if v.truthy() {
                    return v.as_f64();
                }
            }
        }
        let i = self.input.as_ref()?;
        self.eval_bound(&i.input_tpl, None, context, "")
    }

    pub fn get_value(&self) -> KResult<Option<f64>> {
        let context = self.get_context(None)?;
        Ok(self.get_value_ctx(&context))
    }

    fn value_changed(&self) {
        if let Some(i) = &self.input {
            i.last_change.set(self.last_heartbeat.get());
            i.is_dirty.set(true);
        }
    }

    fn init_value(&self) -> KResult<()> {
        let Some(i) = &self.input else { return Ok(()) };
        let context = self.element_context(None)?;
        i.input_value.set(None);
        i.input_min.set(self.eval_bound(&i.input_min_tpl, Some(-999999.0), &context, "min"));
        i.input_max.set(self.eval_bound(&i.input_max_tpl, Some(999999.0), &context, "max"));
        let value = self.get_value_ctx(&context);
        let (Some(lo), Some(hi), Some(v)) = (i.input_min.get(), i.input_max.get(), value) else {
            return Err(KError::Other("'<' not supported between instances of 'NoneType' and 'float'".into()));
        };
        i.input_value.set(Some(crate::pyfmt::py_min(hi, crate::pyfmt::py_max(lo, v))));
        self.value_changed();
        Ok(())
    }

    fn get_input_step(&self, fast_rate: bool) -> f64 {
        let i = self.input.as_ref().expect("input");
        let (lo, hi) = (i.input_min.get().unwrap_or(0.0), i.input_max.get().unwrap_or(0.0));
        if fast_rate && ((hi - lo) / i.input_step > 100.0) {
            10.0 * i.input_step
        } else {
            i.input_step
        }
    }

    fn step_value(&self, fast_rate: bool, up: bool) {
        let Some(i) = &self.input else { return };
        let last_value = i.input_value.get();
        let Some(v) = last_value else { return };
        let input_step = self.get_input_step(fast_rate).abs();
        let v = if up { v + input_step } else { v - input_step };
        let v = crate::pyfmt::py_min(i.input_max.get().unwrap_or(f64::NAN), crate::pyfmt::py_max(i.input_min.get().unwrap_or(f64::NAN), v));
        i.input_value.set(Some(v));
        if last_value != Some(v) {
            self.value_changed();
        }
    }

    // MenuContainer

    fn names_aslist(&self) -> Vec<String> {
        if self.kind.is_list() {
            return self.manager().lookup_children(&self.get_ns("."));
        }
        vec![]
    }

    /// is_accepted(item)
    fn is_accepted(&self, item: &MenuItem) -> bool {
        if self.kind == Kind::Dialog {
            return item.kind.is_command();
        }
        true
    }

    fn lookup_item_name(&self, name: &str) -> KResult<(Option<Rc<MenuItem>>, String)> {
        let name = name.trim().to_string();
        let ns = self.get_ns(&name);
        Ok((Some(self.manager().lookup_menuitem(&ns)?), name))
    }

    fn index_of(&self, item: &Rc<MenuItem>, look_inside: bool) -> Option<i64> {
        let own = |c: &MenuItem, item: &Rc<MenuItem>| c.container.items.borrow().iter().position(|i| Rc::ptr_eq(i, item)).map(|p| p as i64);
        let mut index = own(self, item);
        if index.is_none() && look_inside {
            let items = self.container.items.borrow().clone();
            for con in items {
                // (python: `con._index_of(item)` is falsy for index 0)
                if con.kind.is_container() && own(&con, item).map(|i| i != 0).unwrap_or(false) {
                    index = own(self, &con);
                }
            }
        }
        index
    }

    fn add_parents(&self, parents: &[Rc<MenuItem>]) {
        self.container.parents.borrow_mut().extend_from_slice(parents);
    }

    fn assert_recursive_relation(&self) -> KResult<()> {
        let me = self.rc();
        if self.container.parents.borrow().iter().any(|p| Rc::ptr_eq(p, &me)) {
            return Err(KError::Other(format!("Recursive relation of '{}' container", self.get_ns("."))));
        }
        Ok(())
    }

    fn insert_item(&self, item: Option<Rc<MenuItem>>, name: String, index: Option<usize>) -> KResult<()> {
        let Some(item) = item else { return Ok(()) };
        if !self.is_accepted(&item) {
            return Err(error(format!("Menu item '{}'is not accepted!", item.class_repr())));
        }
        item.init();
        if item.kind.is_container() {
            let parents = self.container.parents.borrow().clone();
            item.add_parents(&parents);
            item.add_parents(&[self.rc()]);
            item.assert_recursive_relation()?;
        }
        let mut all = self.container.allitems.borrow_mut();
        match index {
            None => all.push((item, name)),
            Some(i) => {
                let i = i.min(all.len());
                all.insert(i, (item, name))
            }
        }
        Ok(())
    }

    fn insert_item_ref(&self, item: &Rc<MenuItem>, index: Option<usize>) -> KResult<()> {
        self.insert_item(Some(item.clone()), item.get_ns("."), index)
    }

    fn class_repr(&self) -> String {
        let c = match self.kind {
            Kind::Disabled => "MenuDisabled",
            Kind::Command => "MenuCommand",
            Kind::Input => "MenuInput",
            Kind::List => "MenuList",
            Kind::Dialog => "MenuDialog",
            Kind::VsdList => "MenuVSDList",
            Kind::FileBrowser => "MenuFileBrowser",
        };
        format!("<class 'klippy.extras.display.menu.{}'>", c)
    }

    fn populate(&self) -> KResult<()> {
        self.container.allitems.borrow_mut().clear(); // empty list
        for name in self.names_aslist() {
            let (item, name) = self.lookup_item_name(&name)?;
            self.insert_item(item, name, None)?;
        }
        // populate successor items
        self.populate_successors()?;
        // send populate event
        self.send_event("populate")
    }

    fn populate_successors(&self) -> KResult<()> {
        match self.kind {
            Kind::Dialog => {
                // (super(MenuList, self)._populate(): no back item)
                self.container.viewport_top.set(0);
                let (title, confirm, cancel) = {
                    let d = self.dialog.borrow();
                    let d = d.as_ref().expect("dialog");
                    (d.title_item.clone(), d.confirm_item.clone(), d.cancel_item.clone())
                };
                // Add title as the first item
                self.insert_item_ref(&title, Some(0))?;
                // Append confirm and cancel
                self.insert_item_ref(&confirm, None)?;
                self.insert_item_ref(&cancel, None)?;
                self.populate_values()
            }
            Kind::List | Kind::VsdList | Kind::FileBrowser => {
                self.container.viewport_top.set(0);
                //  add back as first item
                let back = self.container.item_back.borrow().clone();
                if let Some(back) = back {
                    self.insert_item_ref(&back, Some(0))?;
                }
                match self.kind {
                    Kind::VsdList => self.populate_vsd(),
                    Kind::FileBrowser => self.populate_files(),
                    _ => Ok(()),
                }
            }
            _ => Ok(()),
        }
    }

    fn print_file_cb() -> Script {
        Script::Callback(Rc::new(|el: &Rc<MenuItem>, context: &Ctx| {
            if let Some(g) = context.get("gcode") {
                el.manager().queue_gcode(&g.to_str());
            }
            el.manager().exit(false)
        }))
    }

    fn populate_vsd(&self) -> KResult<()> {
        let manager = self.manager();
        let Some(sdcard) = manager.printer.lookup_object_opt("virtual_sdcard").and_then(downcast_rc::<VirtualSD>) else { return Ok(()) };
        let files = sdcard.get_file_list(sdcard.with_subdirs)?;
        for (fname, _fsize) in files {
            let mut context = Ctx::new();
            context.insert("gcode".into(), Value::str(&format!("SDCARD_PRINT_FILE FILENAME='/{}'", fname)));
            let item = manager.menuitem_from(
                "command",
                Kwargs { name: Some(crate::pyfmt::py_str_repr(&fname)), gcode: Some(Self::print_file_cb()), context: Some(context), ..Default::default() },
            )?;
            self.insert_item_ref(&item, None)?;
        }
        Ok(())
    }

    fn dir_contains_gcode(path: &std::path::Path) -> bool {
        fn walk(p: &std::path::Path) -> bool {
            let Ok(rd) = std::fs::read_dir(p) else { return false };
            for e in rd.flatten() {
                let path = e.path();
                if path.is_dir() {
                    if walk(&path) {
                        return true;
                    }
                } else if path.is_file() {
                    let ext = path.extension().map(|e| e.to_string_lossy().into_owned()).unwrap_or_default();
                    if VALID_GCODE_EXTS.contains(&ext.as_str()) {
                        return true;
                    }
                }
            }
            false
        }
        walk(path)
    }

    fn populate_files(&self) -> KResult<()> {
        let manager = self.manager();
        let f = self.fb.as_ref().expect("file browser");
        let (Some(path), Some(root_dir)) = (&f.path, &f.root_dir) else {
            return Err(KError::Other("'MenuFileBrowser' object has no attribute 'path'".into()));
        };
        let reverse = f.sort_by == "last_modified";
        let mut items: Vec<PathBuf> = std::fs::read_dir(path).map_err(|e| KError::Other(e.to_string()))?.flatten().map(|e| e.path()).collect();
        let key = |p: &PathBuf| -> (bool, f64, String) {
            let is_dir = p.is_dir();
            if reverse {
                let mtime = std::fs::metadata(p)
                    .and_then(|m| m.modified())
                    .ok()
                    .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0);
                (is_dir == reverse, mtime, String::new())
            } else {
                (is_dir == reverse, 0.0, p.file_name().map(|n| n.to_string_lossy().to_lowercase()).unwrap_or_default())
            }
        };
        items.sort_by(|a, b| {
            let (ka, kb) = (key(a), key(b));
            let o = ka.0.cmp(&kb.0).then(ka.1.partial_cmp(&kb.1).unwrap_or(std::cmp::Ordering::Equal)).then(ka.2.cmp(&kb.2));
            if reverse {
                o.reverse()
            } else {
                o
            }
        });

        for item in items {
            let name = item.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
            if item.is_dir() {
                if !Self::dir_contains_gcode(&item) {
                    continue;
                }
                let mut context = Ctx::new();
                context.insert("path".into(), Value::str(&item.to_string_lossy()));
                let sub = manager.menuitem_from("file_browser", Kwargs { name: Some(name), context: Some(context), ..Default::default() })?;
                self.insert_item_ref(&sub, None)?;
            } else {
                let ext = item.extension().map(|e| e.to_string_lossy().into_owned()).unwrap_or_default();
                if !VALID_GCODE_EXTS.contains(&ext.as_str()) {
                    continue;
                }
                let stem = item.file_stem().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
                let rel = item.strip_prefix(root_dir).unwrap_or(&item).to_string_lossy().into_owned();
                let mut context = Ctx::new();
                context.insert("sort_by".into(), Value::str(&f.sort_by));
                context.insert("gcode".into(), Value::str(&format!("SDCARD_PRINT_FILE FILENAME='/{}'", rel)));
                let cmd = manager.menuitem_from("command", Kwargs { name: Some(stem), gcode: Some(Self::print_file_cb()), context: Some(context), ..Default::default() })?;
                self.insert_item_ref(&cmd, None)?;
            }
        }
        Ok(())
    }

    fn update_items(&self, keep_pointer: bool) -> KResult<()> {
        let selection = if keep_pointer { self.selected_item() } else { None };
        let all = self.container.allitems.borrow().clone();
        let mut items = vec![];
        let mut names = vec![];
        for (item, name) in all {
            if item.is_enabled() {
                items.push(item);
                names.push(name);
            }
        }
        if items.is_empty() {
            // (python unpacks an empty zip())
            return Err(KError::Other("not enough values to unpack (expected 2, got 0)".into()));
        }
        *self.container.items.borrow_mut() = items;
        *self.container.names.borrow_mut() = names;

        match selection {
            Some(sel) if keep_pointer && self.container.items.borrow().iter().any(|i| Rc::ptr_eq(i, &sel)) => {
                self.select_item(&sel);
            }
            _ => self.init_selection(),
        }
        if self.kind == Kind::Dialog {
            // Refresh values for newly disabled/enabled items
            self.populate_values()?;
        }
        Ok(())
    }

    // select methods
    fn init_selection(&self) {
        self.select_at(Some(0));
    }

    fn select_at(&self, index: Option<i64>) -> Option<Rc<MenuItem>> {
        let mut index = index;
        if self.kind == Kind::Dialog {
            if index == Some(0) {
                self.container.viewport_top.set(0);
                index = Some(1);
            }
            index = Some(crate::pyfmt::py_max(1.0, index.unwrap_or(0) as f64) as i64);
        }
        self.container.selected.set(index);
        // select element
        let item = self.selected_item();
        if let Some(i) = &item {
            i.select();
        }
        item
    }

    fn select_item(&self, needle: &Rc<MenuItem>) {
        let is_selected = self.selected_item().map(|s| Rc::ptr_eq(&s, needle)).unwrap_or(false);
        if !is_selected {
            if let Some(index) = self.index_of(needle, false) {
                self.select_at(Some(index));
            }
        }
    }

    fn len(&self) -> i64 {
        self.container.items.borrow().len() as i64
    }

    pub fn selected_item(&self) -> Option<Rc<MenuItem>> {
        let s = self.container.selected.get()?;
        if 0 <= s && s < self.len() {
            return Some(self.container.items.borrow()[s as usize].clone());
        }
        None
    }

    fn select_next(&self) {
        let index = match self.container.selected.get() {
            None => {
                if self.len() > 0 {
                    Some(0)
                } else {
                    None
                }
            }
            Some(s) if 0 <= s && s < self.len() - 1 => Some(s + 1),
            s => s,
        };
        self.select_at(index);
    }

    fn select_prev(&self) {
        let index = match self.container.selected.get() {
            None => {
                if self.len() > 0 {
                    Some(0)
                } else {
                    None
                }
            }
            Some(s) if 0 < s && s < self.len() => Some(s - 1),
            s => s,
        };
        self.select_at(index);
    }

    fn draw_container(&self, nrows: i64, eventtime: f64) {
        if !self.kind.is_list() {
            return;
        }
        let manager = self.manager();
        let Some(display) = manager.display.upgrade() else { return };
        let selected_row = self.container.selected.get();
        // adjust viewport
        let vt = &self.container.viewport_top;
        match selected_row {
            Some(s) => {
                if s >= vt.get() + nrows {
                    vt.set((s - nrows) + 1);
                }
                if s < vt.get() {
                    vt.set(s);
                }
            }
            None => vt.set(0),
        }
        // clamps viewport
        vt.set(0.max(vt.get().min(self.len() - nrows)));
        let r = (|| -> KResult<()> {
            let mut y = 0;
            let mut selected = false;
            let mut current: Option<Rc<MenuItem>> = None;
            for row in vt.get()..vt.get() + nrows {
                let mut text = String::new();
                let mut prefix = String::new();
                let mut suffix = String::new();
                if row < self.len() {
                    let cur = self.container.items.borrow()[row as usize].clone();
                    selected = Some(row) == selected_row;
                    if selected {
                        cur.heartbeat(eventtime)?;
                    }
                    text = cur.render_name(selected)?;
                    // add prefix (selection indicator)
                    prefix = if selected && !cur.is_editing() {
                        cur.cursor().to_string()
                    } else if selected && cur.is_editing() {
                        "*".into()
                    } else {
                        " ".into()
                    };
                    // add suffix (indicators, typically folder)
                    suffix += &cur.render_suffix()?;
                    current = Some(cur);
                }
                // draw to display
                let plen = prefix.chars().count() as i64;
                let slen = re_glyphs_sub(&suffix).chars().count() as i64;
                let width = manager.cols - plen - slen;
                // draw item prefix (cursor)
                let ppos = display.draw_text(y, 0, &prefix, eventtime);
                // draw item name
                let tpos = display.draw_text(y, ppos, &ljust(&text, width), eventtime);
                let Some(cur) = &current else {
                    return Err(KError::Other("cannot access local variable 'current' where it is not associated with a value".into()));
                };
                // check scroller
                if selected && tpos > manager.cols && cur.is_scrollable() {
                    // scroll next
                    cur.need_scroller(Some(true));
                } else {
                    // reset scroller
                    cur.need_scroller(None);
                }
                // draw item suffix
                if !suffix.is_empty() {
                    display.draw_text(y, manager.cols - slen, &suffix, eventtime);
                }
                // next display row
                y += 1;
            }
            Ok(())
        })();
        if let Err(e) = r {
            logging::exception_err("List drawing error", &e);
        }
    }

    // MenuDialog

    fn dialog_values(&self) -> IndexMap<String, Value> {
        self.dialog.borrow().as_ref().map(|d| d.values.borrow().clone()).unwrap_or_default()
    }

    fn dialog_set_value(&self, ns: &str, value: Value) -> KResult<()> {
        if let Some(d) = self.dialog.borrow().as_ref() {
            d.values.borrow_mut().insert(ns.to_string(), value);
        }
        self.update_items(true)
    }

    fn populate_values(&self) -> KResult<()> {
        let all = self.container.allitems.borrow().clone();
        for (item, _) in all {
            if item.kind != Kind::Input {
                continue;
            }
            let in_items = self.container.items.borrow().iter().any(|i| Rc::ptr_eq(i, &item));
            let current = self.dialog.borrow().as_ref().and_then(|d| d.values.borrow().get(&item.id).cloned());
            if in_items {
                let Some(current) = current else {
                    return Err(KError::Other(crate::pyfmt::py_str_repr(&item.id)));
                };
                if matches!(current, Value::None) {
                    let v = item.get_value()?.map(Value::Float).unwrap_or(Value::None);
                    if let Some(d) = self.dialog.borrow().as_ref() {
                        d.values.borrow_mut().insert(item.id.clone(), v);
                    }
                }
            } else if let Some(d) = self.dialog.borrow().as_ref() {
                d.values.borrow_mut().insert(item.id.clone(), Value::None);
            }
        }
        Ok(())
    }

    fn teardown(&self) {
        self.container.allitems.borrow_mut().clear();
        self.container.items.borrow_mut().clear();
        self.container.parents.borrow_mut().clear();
        *self.container.item_back.borrow_mut() = None;
        self.scripts.borrow_mut().clear();
        *self.dialog.borrow_mut() = None;
    }
}

/// RE_GLYPHS.sub("~~", s)
fn re_glyphs_sub(s: &str) -> String {
    let mut out = String::new();
    let mut rest = s;
    while let Some(start) = rest.find('~') {
        let after = &rest[start + 1..];
        let word_len = after.find(|c: char| !(c.is_alphanumeric() || c == '_')).unwrap_or(after.len());
        if word_len > 0 && after[word_len..].starts_with('~') {
            out += &rest[..start];
            out += "~~";
            rest = &after[word_len + 1..];
        } else {
            out += &rest[..start + 1];
            rest = after;
        }
    }
    out + rest
}

/// str.ljust(width)
fn ljust(s: &str, width: i64) -> String {
    let n = s.chars().count() as i64;
    if n >= width {
        return s.to_string();
    }
    format!("{}{}", s, " ".repeat((width - n) as usize))
}

/// dict.update({key: value}) of a jinja dict value
fn dict_set(d: &Value, key: &str, value: Value) {
    if let Value::Dict(d) = d {
        d.borrow_mut().insert(Key::str(key), value);
    }
}

pub struct MenuManager {
    me: Weak<MenuManager>,
    running: Cell<bool>,
    menuitems: RefCell<IndexMap<String, Rc<MenuItem>>>,
    menustack: RefCell<Vec<Rc<MenuItem>>>,
    dialog_stack: RefCell<Vec<Rc<MenuItem>>>,
    children: RefCell<IndexMap<String, Vec<String>>>,
    display: Weak<PrinterLCD>,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    gcode_queue: RefCell<Vec<String>>,
    context: RefCell<Ctx>,
    root: RefCell<Option<Rc<MenuItem>>>,
    root_name: String,
    cols: i64,
    rows: i64,
    timeout: i64,
    timer: Cell<i64>,
    reverse_navigation: bool,
    gcode_macro: Rc<PrinterGCodeMacro>,
    virtual_sdcard: Option<Rc<VirtualSD>>,
    _keys: RefCell<Option<Rc<MenuKeys>>>,
}

impl MenuManager {
    pub fn new(config: &ConfigWrapper, display: &Rc<PrinterLCD>) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let root_name = config.get("menu_root", "__main")?;
        let (cols, rows) = display.get_dimensions();
        let timeout = config.getint("menu_timeout", 0, Lim::new())?;
        // reverse container navigation
        let reverse_navigation = config.getboolean("menu_reverse_navigation", false)?;
        // load printer objects
        let gcode_macro = load_gcode_macro(config)?;
        let virtual_sdcard = printer.load_object(config, "virtual_sdcard", false)?.and_then(downcast_rc::<VirtualSD>);
        let m = Rc::new_cyclic(|me| MenuManager {
            me: me.clone(),
            running: Cell::new(false),
            menuitems: RefCell::new(IndexMap::new()),
            menustack: RefCell::new(vec![]),
            dialog_stack: RefCell::new(vec![]),
            children: RefCell::new(IndexMap::new()),
            display: Rc::downgrade(display),
            printer: printer.clone(),
            gcode: printer.lookup::<GCodeDispatch>("gcode").expect("gcode"),
            gcode_queue: RefCell::new(vec![]),
            context: RefCell::new(Ctx::new()),
            root: RefCell::new(None),
            root_name,
            cols,
            rows,
            timeout,
            timer: Cell::new(0),
            reverse_navigation,
            gcode_macro,
            virtual_sdcard,
            _keys: RefCell::new(None),
        });
        // register itself for printer callbacks
        printer.add_object("menu", Rc::new(MenuObject(Rc::downgrade(&m))))?;
        let w = m.me.clone();
        printer.register_event_handler(
            "klippy:ready",
            Rc::new(move |_| {
                if let Some(m) = w.upgrade() {
                    m.handle_ready();
                }
                Ok(())
            }),
        );
        // register other printer event handlers
        for event in [
            "filament:insert",
            "filament:runout",
            "print_stats:reset",
            "print_stats:start_printing",
            "print_stats:paused_printing",
            "print_stats:error_printing",
            "print_stats:complete_printing",
            "print_stats:cancelled_printing",
            "idle_timeout:idle",
            "idle_timeout:ready",
            "idle_timeout:printing",
            "virtual_sdcard:load_file",
            "virtual_sdcard:reset_file",
        ] {
            let w = m.me.clone();
            printer.register_event_handler(
                event,
                Rc::new(move |_| match w.upgrade() {
                    Some(m) => m.handle_printer_event(),
                    None => Ok(()),
                }),
            );
        }

        // register for key events
        let w = m.me.clone();
        *m._keys.borrow_mut() = Some(MenuKeys::new(
            config,
            Rc::new(move |key, eventtime| match w.upgrade() {
                Some(m) => m.key_event(key, eventtime),
                None => Ok(()),
            }),
        )?);
        // Load local config file in same directory as current module
        m.load_menu_cfg()?;
        // Load items from main config
        m.load_menuitems(config)?;
        // Load menu root
        *m.root.borrow_mut() = Some(m.lookup_menuitem(&m.root_name)?);
        // send init event
        m.send_event("init", &[])?;
        Ok(m)
    }

    fn rc(&self) -> Rc<MenuManager> {
        self.me.upgrade().expect("menu manager")
    }

    pub fn teardown(&self) {
        for item in self.menuitems.borrow().values() {
            item.teardown();
        }
        self.menuitems.borrow_mut().clear();
        self.menustack.borrow_mut().clear();
        self.dialog_stack.borrow_mut().clear();
        *self.root.borrow_mut() = None;
        self.context.borrow_mut().clear();
        *self._keys.borrow_mut() = None;
    }

    fn handle_ready(&self) {
        // start timer
        let reactor = self.printer.get_reactor();
        let w = self.me.clone();
        reactor.register_timer(
            Box::new(move |eventtime| match w.upgrade() {
                Some(m) => m.timer_event(eventtime),
                None => NEVER,
            }),
            NOW,
        );
    }

    fn handle_printer_event(&self) -> KResult<()> {
        if self.is_running() {
            // iterate the stack from lowest to highest (starting with the root)
            //   if the entry is enabled, update it
            //   else, clear the stack from this point and break
            let mut changed = false;
            for idx in 0..self.stack_size() {
                let entry = self.menustack.borrow()[idx].clone();
                if !entry.is_enabled() {
                    while self.stack_size() > idx {
                        self.stack_pop(true)?;
                    }
                    changed = true;
                    break;
                }
            }
            if !changed {
                // No menu changes were needed
                if let Some(top) = self.stack_peek(0) {
                    top.update_items(true)?;
                }
            }
            if let Some(d) = self.display.upgrade() {
                d.request_redraw();
            }
        }
        Ok(())
    }

    fn timer_event(&self, eventtime: f64) -> f64 {
        if let Err(e) = self.timeout_check() {
            logging::exception_err("menu timer", &e);
        }
        eventtime + TIMER_DELAY
    }

    fn timeout_check(&self) -> KResult<()> {
        let root_is_container = self.root.borrow().as_ref().map(|r| r.kind.is_container()).unwrap_or(false);
        if self.is_running() && self.timeout > 0 && root_is_container {
            if self.timer.get() >= self.timeout {
                self.exit(false)?;
            } else {
                self.timer.set(self.timer.get() + 1);
            }
        } else {
            self.timer.set(0);
        }
        Ok(())
    }

    fn send_event(&self, event: &str, args: &[&dyn Any]) -> KResult<()> {
        let me = self.rc();
        if args.is_empty() {
            return self.printer.send_event(&format!("menu:{}", event), &[&me]);
        }
        self.printer.send_event(&format!("menu:{}", event), args)
    }

    pub fn is_running(&self) -> bool {
        self.running.get()
    }

    fn begin(&self, eventtime: f64) -> KResult<()> {
        self.menustack.borrow_mut().clear();
        self.timer.set(0);
        let root = self.root.borrow().clone();
        match root {
            Some(root) if root.kind.is_container() => {
                // send begin event
                self.send_event("begin", &[])?;
                self.update_context(eventtime);
                root.init_selection();
                self.stack_push(&root)?;
                self.running.set(true);
                return Ok(());
            }
            Some(_) => logging::error("Invalid root, menu stopped!"),
            None => {}
        }
        self.running.set(false);
        Ok(())
    }

    fn back_action(&self, force: bool, update: bool) -> KResult<String> {
        self.back(force, update)?;
        Ok(String::new())
    }

    fn exit_action(&self, force: bool) -> KResult<String> {
        self.exit(force)?;
        Ok(String::new())
    }

    fn get_context(&self, cxt: Option<&Ctx>) -> Ctx {
        let mut context = self.context.borrow().clone();
        if let Some(c) = cxt {
            context.extend(c.clone());
        }
        let stack = self.dialog_stack.borrow().clone();
        if !stack.is_empty() {
            let mut d: IndexMap<Key, Value> = IndexMap::new();
            for dialog in stack {
                for (k, v) in dialog.dialog_values() {
                    d.insert(Key::str(&k), v);
                }
            }
            context.insert("dialog".into(), Value::dict(d));
        }
        context
    }

    fn update_context(&self, eventtime: f64) {
        // menu default jinja2 context
        let mut context = create_template_context(&self.printer, Some(eventtime));
        let mut menu: IndexMap<Key, Value> = IndexMap::new();
        menu.insert(Key::str("eventtime"), Value::Float(eventtime));
        menu.insert(Key::str("back"), Value::object(MenuAction { manager: self.me.clone(), exit: false }));
        menu.insert(Key::str("exit"), Value::object(MenuAction { manager: self.me.clone(), exit: true }));
        context.insert("menu".into(), Value::dict(menu));
        *self.context.borrow_mut() = context;
    }

    fn current_dialog(&self) -> Option<Rc<MenuItem>> {
        self.dialog_stack.borrow().last().cloned()
    }

    fn stack_push(&self, container: &Rc<MenuItem>) -> KResult<()> {
        if !container.kind.is_container() {
            return Err(error("Wrong type, expected MenuContainer"));
        }
        container.populate()?;
        if let Some(top) = self.stack_peek(0) {
            if top.kind.is_list() {
                top.run_script("leave", None, None, false)?;
            }
        }
        if container.kind.is_list() {
            container.run_script("enter", None, None, false)?;
        }
        if container.kind == Kind::Dialog {
            self.dialog_stack.borrow_mut().push(container.clone());
        }
        if !container.is_editing() {
            container.update_items(false)?;
            container.init_selection();
        }
        self.menustack.borrow_mut().push(container.clone());
        Ok(())
    }

    fn stack_pop(&self, update: bool) -> KResult<Option<Rc<MenuItem>>> {
        if self.stack_size() == 0 {
            return Ok(None);
        }
        let container = self.menustack.borrow_mut().pop().expect("stack");
        if !container.kind.is_container() {
            return Err(error("Wrong type, expected MenuContainer"));
        }
        if container.kind == Kind::Dialog {
            let mut ds = self.dialog_stack.borrow_mut();
            if let Some(pos) = ds.iter().position(|d| Rc::ptr_eq(d, &container)) {
                ds.remove(pos);
            }
        }
        match self.stack_peek(0) {
            Some(top) => {
                if !top.is_editing() && update {
                    top.update_items(false)?;
                    top.init_selection();
                }
                if container.kind.is_list() {
                    container.run_script("leave", None, None, false)?;
                }
                if top.kind.is_list() {
                    top.run_script("enter", None, None, false)?;
                }
            }
            None => {
                if container.kind.is_list() {
                    container.run_script("leave", None, None, false)?;
                }
            }
        }
        Ok(Some(container))
    }

    fn stack_size(&self) -> usize {
        self.menustack.borrow().len()
    }

    fn stack_peek(&self, lvl: usize) -> Option<Rc<MenuItem>> {
        let stack = self.menustack.borrow();
        if stack.len() > lvl {
            return Some(stack[stack.len() - lvl - 1].clone());
        }
        None
    }

    pub fn screen_update_event(&self, eventtime: f64) -> KResult<bool> {
        // screen update
        if !self.is_running() {
            return Ok(false);
        }
        // draw menu
        self.update_context(eventtime);
        if let Some(container) = self.stack_peek(0) {
            if self.running.get() && container.kind.is_container() {
                container.heartbeat(eventtime)?;
                container.draw_container(self.rows, eventtime);
            }
        }
        Ok(true)
    }

    fn running_container(&self) -> Option<Rc<MenuItem>> {
        let c = self.stack_peek(0)?;
        if self.running.get() && c.kind.is_container() {
            return Some(c);
        }
        None
    }

    fn up(&self, fast_rate: bool) {
        let Some(container) = self.running_container() else { return };
        self.timer.set(0);
        match container.selected_item() {
            Some(current) if current.kind == Kind::Input && current.is_editing() => current.step_value(fast_rate, false),
            _ => {
                if self.reverse_navigation {
                    container.select_next(); // reverse
                } else {
                    container.select_prev(); // normal
                }
            }
        }
    }

    fn down(&self, fast_rate: bool) {
        let Some(container) = self.running_container() else { return };
        self.timer.set(0);
        match container.selected_item() {
            Some(current) if current.kind == Kind::Input && current.is_editing() => current.step_value(fast_rate, true),
            _ => {
                if self.reverse_navigation {
                    container.select_prev(); // reverse
                } else {
                    container.select_next(); // normal
                }
            }
        }
    }

    pub fn back(&self, force: bool, update: bool) -> KResult<()> {
        let Some(container) = self.running_container() else { return Ok(()) };
        self.timer.set(0);
        if let Some(current) = container.selected_item() {
            if current.kind == Kind::Input && current.is_editing() {
                if force {
                    current.stop_editing()?;
                } else {
                    return Ok(());
                }
            }
        }
        match self.stack_peek(1).filter(|p| p.kind.is_container()) {
            Some(parent) => {
                self.stack_pop(update)?;
                match parent.index_of(&container, true) {
                    Some(index) => {
                        parent.select_at(Some(index));
                    }
                    None => {
                        if parent.selected_item().is_none() {
                            parent.init_selection();
                        }
                    }
                }
            }
            None => {
                self.stack_pop(true)?;
                self.running.set(false);
            }
        }
        Ok(())
    }

    pub fn exit(&self, force: bool) -> KResult<()> {
        let Some(container) = self.running_container() else { return Ok(()) };
        self.timer.set(0);
        if let Some(current) = container.selected_item() {
            if !force && current.kind == Kind::Input && current.is_editing() {
                return Ok(());
            }
        }
        if container.kind.is_list() {
            container.run_script("leave", None, None, false)?;
        }
        self.send_event("exit", &[])?;
        self.running.set(false);
        Ok(())
    }

    pub fn push_container(&self, menu: &Rc<MenuItem>) -> KResult<bool> {
        if let Some(container) = self.running_container() {
            if menu.kind.is_container() && !container.is_editing() && !Rc::ptr_eq(menu, &container) {
                self.stack_push(menu)?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn press(&self, event: &str) -> KResult<()> {
        let Some(container) = self.running_container() else { return Ok(()) };
        self.timer.set(0);
        match container.selected_item() {
            Some(current) if current.kind.is_container() => self.stack_push(&current)?,
            Some(current) if current.kind == Kind::Input => {
                if current.is_editing() {
                    current.run_script("gcode", Some(event), None, false)?;
                }
                current.run_script(event, None, None, false)?;
            }
            Some(current) if current.kind == Kind::Command => {
                current.run_script("gcode", Some(event), None, false)?;
                current.run_script(event, None, None, false)?;
            }
            // current is None, no selection. passthru to container
            _ => {
                container.run_script(event, None, None, false)?;
            }
        }
        Ok(())
    }

    pub fn queue_gcode(&self, script: &str) {
        if script.is_empty() {
            return;
        }
        if self.gcode_queue.borrow().is_empty() {
            let reactor = self.printer.get_reactor();
            let w = self.me.clone();
            reactor.register_callback(
                move |eventtime| {
                    if let Some(m) = w.upgrade() {
                        m.dispatch_gcode(eventtime);
                    }
                },
                NOW,
            );
        }
        self.gcode_queue.borrow_mut().push(script.to_string());
    }

    fn dispatch_gcode(&self, _eventtime: f64) {
        loop {
            let Some(script) = self.gcode_queue.borrow().first().cloned() else { break };
            if let Err(e) = self.gcode.run_script(&script) {
                logging::exception_err("Script running error", &e);
            }
            self.gcode_queue.borrow_mut().remove(0);
        }
    }

    fn menuitem_from(&self, type_name: &str, kwargs: Kwargs) -> KResult<Rc<MenuItem>> {
        let kind = kind_of(type_name)?;
        MenuItem::new(&self.rc(), kind, Source::Kwargs(&kwargs))
    }

    fn add_menuitem(&self, name: &str, item: Rc<MenuItem>) {
        let mut existing_item = false;
        if self.menuitems.borrow().contains_key(name) {
            existing_item = true;
            logging::info(&format!("Declaration of '{}' hides previous menuitem declaration", name));
        }
        self.menuitems.borrow_mut().insert(name.to_string(), item.clone());
        let parent = item.get_ns("..");
        if !parent.is_empty() && !existing_item {
            let mut children = self.children.borrow_mut();
            let list = children.entry(parent).or_default();
            match item.index {
                Some(index) => {
                    // list.insert(index, x)
                    let len = list.len() as i64;
                    let i = if index < 0 { (len + index).max(0) } else { index.min(len) };
                    list.insert(i as usize, item.get_ns("."));
                }
                None => list.push(item.get_ns(".")),
            }
        }
    }

    pub fn lookup_menuitem(&self, name: &str) -> KResult<Rc<MenuItem>> {
        self.menuitems.borrow().get(name).cloned().ok_or_else(|| KError::config(format!("Unknown menuitem '{}'", name)))
    }

    fn lookup_children(&self, ns: &str) -> Vec<String> {
        self.children.borrow().get(ns).cloned().unwrap_or_default()
    }

    fn load_menu_cfg(&self) -> KResult<()> {
        let pconfig = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let cfg = pconfig
            .pconfig
            .borrow()
            .build_config_wrapper(MENU_CFG, MENU_CFG_FILENAME)
            .map_err(|_| KError::config(format!("Cannot load config '{}'", MENU_CFG_FILENAME)))?
            .with_printer(&self.printer);
        self.load_menuitems(&cfg)
    }

    fn load_menuitems(&self, config: &ConfigWrapper) -> KResult<()> {
        for cfg in config.get_prefix_sections("menu ") {
            let type_name = cfg.get("type", Dflt::Required)?;
            let kind = kind_of(&type_name)?;
            let item = MenuItem::new(&self.rc(), kind, Source::Config(&cfg))?;
            self.add_menuitem(&item.get_ns("."), item);
        }
        Ok(())
    }

    fn click_callback(&self, eventtime: f64, event: &str) -> KResult<()> {
        if self.is_running() {
            self.press(event)
        } else {
            // lets start and populate the menu items
            self.begin(eventtime)
        }
    }

    fn key_event(&self, key: &str, eventtime: f64) -> KResult<()> {
        match key {
            "click" | "long_click" => self.click_callback(eventtime, key)?,
            "up" => self.up(false),
            "fast_up" => self.up(true),
            "down" => self.down(false),
            "fast_down" => self.down(true),
            "back" => self.back(false, true)?,
            _ => {}
        }
        if let Some(d) = self.display.upgrade() {
            d.request_redraw();
        }
        Ok(())
    }

    // Collection of manager class helper methods

    /// Literals are beginning or ending by the double or single quotes
    fn stripliterals(s: &str) -> String {
        if s.len() >= 2 && ((s.starts_with('"') && s.ends_with('"')) || (s.starts_with('\'') && s.ends_with('\''))) {
            return s[1..s.len() - 1].to_string();
        }
        // (a lone quote: python's s[1:-1] of a 1 char string is "")
        if s == "\"" || s == "'" {
            return String::new();
        }
        s.to_string()
    }

    pub fn asflat(s: &str) -> String {
        Self::stripliterals(&crate::jinja::methods::str_splitlines(s, false).concat())
    }

    pub fn get_status(&self) -> PyVal {
        let mut d = IndexMap::new();
        d.insert("timeout".to_string(), PyVal::Int(self.timeout));
        d.insert("running".to_string(), PyVal::Bool(self.running.get()));
        d.insert("rows".to_string(), PyVal::Int(self.rows));
        d.insert("cols".to_string(), PyVal::Int(self.cols));
        PyVal::Dict(d)
    }
}

/// menu.back / menu.exit of the menu template context
struct MenuAction {
    manager: Weak<MenuManager>,
    exit: bool,
}

impl Object for MenuAction {
    fn type_name(&self) -> String {
        "method".into()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn call(&self, _ctx: &mut dyn CallCtx, args: Vec<Value>, kwargs: IndexMap<String, Value>) -> Option<VResult<Value>> {
        let m = self.manager.upgrade()?;
        let arg = |i: usize, name: &str, default: bool| args.get(i).or_else(|| kwargs.get(name)).map(|v| v.truthy()).unwrap_or(default);
        let r = if self.exit { m.exit_action(arg(0, "force", false)) } else { m.back_action(arg(0, "force", false), arg(1, "update", true)) };
        Some(r.map(|s| Value::str(&s)).map_err(crate::extras::gcode_macro::kerror_exc))
    }
}

/// the "menu" printer object (the MenuManager)
struct MenuObject(Weak<MenuManager>);

impl PrinterObject for MenuObject {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        self.0.upgrade().map(|m| m.get_status())
    }
}
