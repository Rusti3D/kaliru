// The build configuration: a Kconfig `.config` (as Kalico's `make
// menuconfig` / `olddefconfig` write it) and its minimal `defconfig`.
use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i64),
    Str(String),
}

pub struct Config {
    pub values: BTreeMap<String, Value>,
    /// the savedefconfig text (the data dictionary's "kconfig")
    pub defconfig: String,
}

impl Config {
    /// `autoconf.h` holds every symbol (unset ones as 0 / ""), `config`
    /// tells bools (y / is not set) from ints
    pub fn load(dir: &std::path::Path) -> Config {
        let dotconfig = Self::load_dotconfig(dir);
        let defconfig = std::fs::read_to_string(dir.join("defconfig")).expect("defconfig");
        let autoconf = std::fs::read_to_string(dir.join("autoconf.h")).expect("autoconf.h");
        let mut values = BTreeMap::new();
        for line in autoconf.lines() {
            let Some(rest) = line.strip_prefix("#define ") else { continue };
            let Some((name, v)) = rest.split_once(' ') else { continue };
            let value = if let Some(s) = v.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
                Value::Str(s.replace("\\\"", "\"").replace("\\\\", "\\"))
            } else if let Some(h) = v.strip_prefix("0x") {
                // a hex symbol (unset ones are a bare "0x")
                Value::Int(if h.is_empty() { 0 } else { i64::from_str_radix(h, 16).expect("hex config value") })
            } else {
                let i: i64 = v.parse().expect("int config value");
                match dotconfig.get(name) {
                    Some(Value::Int(_)) => Value::Int(i),
                    _ if i == 0 || i == 1 => Value::Bool(i == 1),
                    _ => Value::Int(i),
                }
            };
            values.insert(name.to_string(), value);
        }
        Config { values, defconfig }
    }

    fn load_dotconfig(dir: &std::path::Path) -> BTreeMap<String, Value> {
        let text = std::fs::read_to_string(dir.join("config")).expect("config");
        let mut values = BTreeMap::new();
        for line in text.lines() {
            if let Some(rest) = line.strip_prefix("# ") {
                if let Some(name) = rest.strip_suffix(" is not set") {
                    values.insert(name.to_string(), Value::Bool(false));
                }
                continue;
            }
            let Some((name, v)) = line.split_once('=') else { continue };
            let value = if v == "y" {
                Value::Bool(true)
            } else if let Some(s) = v.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
                Value::Str(s.replace("\\\"", "\"").replace("\\\\", "\\"))
            } else if let Some(h) = v.strip_prefix("0x") {
                Value::Int(i64::from_str_radix(h, 16).expect("hex config value"))
            } else {
                Value::Int(v.parse().expect("int config value"))
            };
            values.insert(name.to_string(), value);
        }
        values
    }

    pub fn enabled(&self, name: &str) -> bool {
        self.values.get(name) == Some(&Value::Bool(true))
    }

    pub fn int(&self, name: &str) -> Option<i64> {
        match self.values.get(name) {
            Some(Value::Int(i)) => Some(*i),
            Some(Value::Bool(b)) => Some(*b as i64),
            _ => None,
        }
    }

    pub fn string(&self, name: &str) -> Option<&str> {
        match self.values.get(name) {
            Some(Value::Str(s)) => Some(s),
            _ => None,
        }
    }

    /// `config.rs`: the values as constants (autoconf.h)
    pub fn rust_consts(&self) -> String {
        let mut out = String::new();
        for (name, v) in &self.values {
            out += &match v {
                Value::Bool(b) => format!("pub const {}: bool = {};\n", name, b),
                Value::Int(i) => format!("pub const {}: u32 = {};\n", name, *i as u32),
                Value::Str(s) => format!("pub const {}: &str = {:?};\n", name, s),
            };
        }
        out
    }
}
