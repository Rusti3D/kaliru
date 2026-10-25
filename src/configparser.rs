// Port of python 3.13's configparser.RawConfigParser (kaliru/ref/
// configparser_py313.py), the parts klippy uses: reading with strict on/off,
// full and inline comment prefixes, empty lines in values, a pluggable
// interpolation, get/getint/getfloat/getboolean, set, remove_*, write.
// Not ported: allow_no_value (klippy never enables it), unnamed sections,
// read_dict, converters, the mapping protocol.
use crate::pyfmt::{py_float, py_int, py_isspace, py_rstrip, py_str_repr, py_strip};
use crate::{IndexMap, IndexSet};

pub const DEFAULTSECT: &str = "DEFAULT";
pub const MAX_INTERPOLATION_DEPTH: usize = 10;

#[derive(Clone, Debug, PartialEq)]
pub enum ErrorKind {
    NoSection,
    NoOption,
    DuplicateSection,
    DuplicateOption,
    Parsing,
    MissingSectionHeader,
    Interpolation,
    /// a plain configparser.Error (eg raised by klippy's configfile)
    Error,
}

/// configparser.Error and subclasses; Display is str(e)
#[derive(Clone, Debug)]
pub struct Error {
    pub kind: ErrorKind,
    pub message: String,
    /// the python exception (class, message) this was raised while handling
    pub context: Option<Box<(String, String)>>,
}

impl Error {
    pub fn new(msg: impl Into<String>) -> Self {
        Error { kind: ErrorKind::Error, message: msg.into(), context: None }
    }
    fn kind(kind: ErrorKind, message: String) -> Self {
        Error { kind, message, context: None }
    }
    /// raise self while handling a python exception (cls, msg)
    pub fn with_context(mut self, cls: &str, msg: &str) -> Self {
        self.context = Some(Box::new((cls.to_string(), msg.to_string())));
        self
    }
    pub fn no_section(section: &str) -> Self {
        Self::kind(ErrorKind::NoSection, format!("No section: {}", py_str_repr(section)))
    }
    pub fn no_option(option: &str, section: &str) -> Self {
        Self::kind(ErrorKind::NoOption, format!("No option {} in section: {}", py_str_repr(option), py_str_repr(section)))
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

/// configparser.Interpolation hooks (all default to the identity)
pub trait Interpolation {
    fn before_get(&self, _parser: &RawConfigParser, _section: &str, _option: &str, value: &str) -> Result<String> {
        Ok(value.to_string())
    }
    /// Err is python's ValueError (BasicInterpolation's syntax check)
    fn before_set(&self, _parser: &RawConfigParser, _section: &str, _option: &str, value: &str) -> std::result::Result<String, String> {
        Ok(value.to_string())
    }
    fn before_read(&self, _parser: &RawConfigParser, _section: &str, _option: &str, value: String) -> String {
        value
    }
    fn before_write(&self, _parser: &RawConfigParser, _section: &str, _option: &str, value: &str) -> String {
        value.to_string()
    }
}

pub struct NoInterpolation;
impl Interpolation for NoInterpolation {}

/// configparser.BasicInterpolation ("%(name)s" references, "%%" escapes)
pub struct BasicInterpolation;

/// _KEYCRE.match(rest) for r"%\(([^)]+)\)s": (name, match length)
fn basic_keycre_match(rest: &str) -> Option<(String, usize)> {
    let body = rest.strip_prefix("%(")?;
    let close = body.find(')')?;
    if close == 0 || !body[close + 1..].starts_with('s') {
        return None;
    }
    Some((body[..close].to_string(), 2 + close + 2))
}

impl BasicInterpolation {
    fn interpolate_some(
        &self,
        option: &str,
        accum: &mut String,
        mut rest: &str,
        section: &str,
        map: &Section,
        depth: usize,
    ) -> Result<()> {
        if depth > MAX_INTERPOLATION_DEPTH {
            return Err(Error::new(format!("Recursion limit exceeded in value substitution: option {} in section {}", py_repr_str(option), py_repr_str(section))));
        }
        while !rest.is_empty() {
            let Some(p) = rest.find('%') else {
                accum.push_str(rest);
                return Ok(());
            };
            if p > 0 {
                accum.push_str(&rest[..p]);
                rest = &rest[p..];
            }
            let c = rest.get(1..2).unwrap_or("");
            if c == "%" {
                accum.push('%');
                rest = &rest[2..];
            } else if c == "(" {
                let Some((name, len)) = basic_keycre_match(rest) else {
                    return Err(Error::new(format!("bad interpolation variable reference {}", py_repr_str(rest))));
                };
                let var = optionxform(&name);
                rest = &rest[len..];
                let Some(v) = map.get(&var) else {
                    return Err(Error::new(format!(
                        "Bad value substitution: option {} in section {} contains an interpolation key {} which is not a valid option name.",
                        py_repr_str(option),
                        py_repr_str(section),
                        py_repr_str(&var)
                    )));
                };
                if v.contains('%') {
                    self.interpolate_some(option, accum, v, section, map, depth + 1)?;
                } else {
                    accum.push_str(v);
                }
            } else {
                return Err(Error::new(format!("'%' must be followed by '%' or '(', found: {}", py_repr_str(rest))));
            }
        }
        Ok(())
    }
}

fn py_repr_str(s: &str) -> String {
    crate::pyfmt::py_str_repr(s)
}

impl Interpolation for BasicInterpolation {
    fn before_get(&self, parser: &RawConfigParser, section: &str, option: &str, value: &str) -> Result<String> {
        // the interpolation map: defaults updated with the section
        let mut map = parser.defaults().clone();
        if let Some(sect) = parser.raw_section(section) {
            for (k, v) in sect {
                map.insert(k.clone(), v.clone());
            }
        }
        let mut accum = String::new();
        self.interpolate_some(option, &mut accum, value, section, &map, 1)?;
        Ok(accum)
    }

    fn before_set(&self, _parser: &RawConfigParser, _section: &str, _option: &str, value: &str) -> std::result::Result<String, String> {
        // escaped percent signs, then valid syntax
        let escaped = value.replace("%%", "");
        // _KEYCRE.sub("", tmp_value)
        let mut tmp_value = String::new();
        let mut i = 0;
        while i < escaped.len() {
            if let Some((_, len)) = basic_keycre_match(&escaped[i..]) {
                i += len;
                continue;
            }
            let c = escaped[i..].chars().next().unwrap_or('\0');
            tmp_value.push(c);
            i += c.len_utf8();
        }
        if let Some(pos) = tmp_value.find('%') {
            return Err(format!("invalid interpolation syntax in {} at position {}", py_repr_str(value), tmp_value[..pos].chars().count()));
        }
        Ok(value.to_string())
    }
}

pub type Section = IndexMap<String, String>;

pub struct RawConfigParser {
    sections: IndexMap<String, Section>,
    defaults: Section,
    comment_prefixes: Vec<String>,
    inline_comment_prefixes: Vec<String>,
    strict: bool,
    empty_lines_in_values: bool,
    pub default_section: String,
    interpolation: Box<dyn Interpolation>,
}

// _ReadState.cursect: a section, the defaults, or none yet
#[derive(Clone, PartialEq, Eq, Hash)]
enum Cur {
    Section(String),
    Defaults,
}

pub const BOOLEAN_STATES: [(&str, bool); 8] = [
    ("1", true),
    ("yes", true),
    ("true", true),
    ("on", true),
    ("0", false),
    ("no", false),
    ("false", false),
    ("off", false),
];

pub fn optionxform(optionstr: &str) -> String {
    optionstr.to_lowercase()
}

impl RawConfigParser {
    /// RawConfigParser(strict=..., inline_comment_prefixes=..., interpolation=...)
    /// with the other arguments at their defaults
    pub fn new(strict: bool, inline_comment_prefixes: &[&str], interpolation: Box<dyn Interpolation>) -> Self {
        RawConfigParser {
            sections: IndexMap::default(),
            defaults: Section::default(),
            comment_prefixes: vec!["#".into(), ";".into()],
            inline_comment_prefixes: inline_comment_prefixes.iter().map(|s| s.to_string()).collect(),
            strict,
            empty_lines_in_values: true,
            default_section: DEFAULTSECT.into(),
            interpolation,
        }
    }

    pub fn defaults(&self) -> &Section {
        &self.defaults
    }

    pub fn sections(&self) -> Vec<String> {
        self.sections.keys().cloned().collect()
    }

    pub fn add_section(&mut self, section: &str) -> Result<()> {
        if section == self.default_section {
            // python raises ValueError
            return Err(Error::new(format!("Invalid section name: {}", py_str_repr(section))));
        }
        if self.sections.contains_key(section) {
            return Err(Error::kind(ErrorKind::DuplicateSection, format!("Section {} already exists", py_str_repr(section))));
        }
        self.sections.insert(section.to_string(), Section::default());
        Ok(())
    }

    pub fn has_section(&self, section: &str) -> bool {
        self.sections.contains_key(section)
    }

    pub fn options(&self, section: &str) -> Result<Vec<String>> {
        let mut opts = self.sections.get(section).ok_or_else(|| Error::no_section(section))?.clone();
        for (k, v) in &self.defaults {
            opts.insert(k.clone(), v.clone());
        }
        Ok(opts.into_keys().collect())
    }

    /// read_file(f, source) of the text f yields
    pub fn read_string(&mut self, string: &str, source: &str) -> Result<()> {
        let mut pending: IndexMap<(Cur, String), Vec<String>> = IndexMap::default();
        let res = self.read_inner(string, source, &mut pending);
        // finally: _join_multiline_values()
        for ((cur, name), val) in pending {
            let joined = py_rstrip(&val.join("\n")).to_string();
            let sect = match &cur {
                Cur::Section(s) => s.clone(),
                Cur::Defaults => self.default_section.clone(),
            };
            let v = self.interpolation.before_read(self, &sect, &name, joined);
            match cur {
                Cur::Section(s) => self.sections.get_mut(&s).unwrap().insert(name, v),
                Cur::Defaults => self.defaults.insert(name, v),
            };
        }
        let errors = res?;
        if let Some((first, rest)) = errors.split_first() {
            // ParsingError._raise_all: the first error combined with the others
            let mut msg = format!("Source contains parsing errors: {}", py_str_repr(source));
            for (lineno, line) in std::iter::once(first).chain(rest) {
                msg.push_str(&format!("\n\t[line {:2}]: {}", lineno, py_str_repr(line)));
            }
            return Err(Error::kind(ErrorKind::Parsing, msg));
        }
        Ok(())
    }

    // _Line.clean: "" for full-line comments, else the text before the
    // earliest inline prefix at the line start or after whitespace, stripped
    fn clean(&self, line: &str) -> String {
        let stripped = py_strip(line);
        if self.comment_prefixes.iter().any(|p| stripped.starts_with(p.as_str())) {
            return String::new();
        }
        let chars: Vec<(usize, char)> = line.char_indices().collect();
        for (n, &(pos, c)) in chars.iter().enumerate() {
            for prefix in &self.inline_comment_prefixes {
                if n == 0 && line.starts_with(prefix.as_str()) {
                    return String::new();
                }
                if py_isspace(c) && line[pos + c.len_utf8()..].starts_with(prefix.as_str()) {
                    return py_strip(&line[..pos]).to_string();
                }
            }
        }
        stripped.to_string()
    }

    fn read_inner(
        &mut self,
        data: &str,
        fpname: &str,
        pending: &mut IndexMap<(Cur, String), Vec<String>>,
    ) -> Result<Vec<(usize, String)>> {
        let mut elements_added: IndexSet<String> = IndexSet::default();
        let mut options_added: IndexSet<(String, String)> = IndexSet::default();
        let mut cursect: Option<Cur> = None;
        let mut sectname = String::new();
        let mut optname: Option<String> = None;
        let mut indent_level: usize = 0;
        let mut errors = vec![];
        for (idx, line) in data.split_inclusive('\n').enumerate() {
            let lineno = idx + 1;
            let clean = self.clean(line);
            if clean.is_empty() {
                if self.empty_lines_in_values {
                    let has_comments = py_strip(line) != clean;
                    if let (false, Some(cur), Some(opt)) = (has_comments, &cursect, &optname) {
                        if !opt.is_empty() {
                            if let Some(v) = pending.get_mut(&(cur.clone(), opt.clone())) {
                                v.push(String::new());
                            }
                        }
                    }
                } else {
                    indent_level = usize::MAX;
                }
                continue;
            }
            let cur_indent_level = line.chars().position(|c| !py_isspace(c)).unwrap_or(0);
            // continuation line?
            if let (Some(cur), Some(opt)) = (&cursect, &optname) {
                if !opt.is_empty() && cur_indent_level > indent_level {
                    if let Some(v) = pending.get_mut(&(cur.clone(), opt.clone())) {
                        v.push(clean);
                    }
                    continue;
                }
            }
            // a section header or option header?
            indent_level = cur_indent_level;
            let header = match_sectcre(&clean);
            if header.is_none() && cursect.is_none() {
                return Err(Error::kind(
                    ErrorKind::MissingSectionHeader,
                    format!(
                        "File contains no section headers.\nfile: {}, line: {}\n{}",
                        py_str_repr(fpname),
                        lineno,
                        py_str_repr(line)
                    ),
                ));
            }
            if let Some(header) = header {
                sectname = header.clone();
                if self.sections.contains_key(&header) {
                    if self.strict && elements_added.contains(&header) {
                        return Err(Error::kind(
                            ErrorKind::DuplicateSection,
                            format!(
                                "While reading from {} [line {:2}]: section {} already exists",
                                py_str_repr(fpname),
                                lineno,
                                py_str_repr(&header)
                            ),
                        ));
                    }
                    cursect = Some(Cur::Section(header.clone()));
                    elements_added.insert(header);
                } else if header == self.default_section {
                    cursect = Some(Cur::Defaults);
                } else {
                    self.sections.insert(header.clone(), Section::default());
                    cursect = Some(Cur::Section(header.clone()));
                    elements_added.insert(header);
                }
                // So sections can't start with a continuation line
                optname = None;
                continue;
            }
            // an option line?
            let Some((name, optval)) = match_optcre(&clean) else {
                errors.push((lineno, line.to_string()));
                continue;
            };
            if name.is_empty() {
                errors.push((lineno, line.to_string()));
            }
            let name = optionxform(py_rstrip(&name));
            if self.strict && options_added.contains(&(sectname.clone(), name.clone())) {
                return Err(Error::kind(
                    ErrorKind::DuplicateOption,
                    format!(
                        "While reading from {} [line {:2}]: option {} in section {} already exists",
                        py_str_repr(fpname),
                        lineno,
                        py_str_repr(&name),
                        py_str_repr(&sectname)
                    ),
                ));
            }
            options_added.insert((sectname.clone(), name.clone()));
            let cur = cursect.clone().unwrap();
            // keep the option's position in its section (python dict order)
            match &cur {
                Cur::Section(s) => self.sections.get_mut(s).unwrap().entry(name.clone()).or_default(),
                Cur::Defaults => self.defaults.entry(name.clone()).or_default(),
            };
            pending.insert((cur, name.clone()), vec![py_strip(&optval).to_string()]);
            optname = Some(name);
        }
        Ok(errors)
    }

    // ChainMap(section, defaults) lookup
    fn unify_get(&self, section: &str, option: &str) -> Result<Option<&String>> {
        let sectiondict = match self.sections.get(section) {
            Some(s) => Some(s),
            None if section == self.default_section => None,
            None => return Err(Error::no_section(section)),
        };
        Ok(sectiondict.and_then(|s| s.get(option)).or_else(|| self.defaults.get(option)))
    }

    pub fn get(&self, section: &str, option: &str) -> Result<String> {
        self.get_raw(section, option, false)
    }

    pub fn get_raw(&self, section: &str, option: &str, raw: bool) -> Result<String> {
        let option = optionxform(option);
        let value = self.unify_get(section, &option)?.ok_or_else(|| Error::no_option(&option, section))?;
        if raw {
            return Ok(value.clone());
        }
        self.interpolation.before_get(self, section, &option, value)
    }

    /// Err(Ok(e)): a configparser error; Err(Err(msg)): a ValueError from the
    /// conversion (python raises those as-is)
    pub fn getint(&self, section: &str, option: &str) -> std::result::Result<i64, std::result::Result<Error, String>> {
        let v = self.get(section, option).map_err(Ok)?;
        py_int(&v).ok_or_else(|| Err(format!("invalid literal for int() with base 10: {}", py_str_repr(&v))))
    }

    pub fn getfloat(&self, section: &str, option: &str) -> std::result::Result<f64, std::result::Result<Error, String>> {
        let v = self.get(section, option).map_err(Ok)?;
        py_float(&v).ok_or_else(|| Err(format!("could not convert string to float: {}", py_str_repr(&v))))
    }

    pub fn getboolean(&self, section: &str, option: &str) -> std::result::Result<bool, std::result::Result<Error, String>> {
        let v = self.get(section, option).map_err(Ok)?;
        convert_to_boolean(&v).map_err(Err)
    }

    /// items(section): (option, interpolated value) with defaults merged
    pub fn items(&self, section: &str) -> Result<Vec<(String, String)>> {
        let mut d = self.defaults.clone();
        match self.sections.get(section) {
            Some(s) => {
                for (k, v) in s {
                    d.insert(k.clone(), v.clone());
                }
            }
            None if section != self.default_section => return Err(Error::no_section(section)),
            None => {}
        }
        d.iter()
            .map(|(k, v)| Ok((k.clone(), self.interpolation.before_get(self, section, k, v)?)))
            .collect()
    }

    pub fn has_option(&self, section: &str, option: &str) -> bool {
        let option = optionxform(option);
        if section.is_empty() || section == self.default_section {
            return self.defaults.contains_key(&option);
        }
        match self.sections.get(section) {
            None => false,
            Some(s) => s.contains_key(&option) || self.defaults.contains_key(&option),
        }
    }

    pub fn set(&mut self, section: &str, option: &str, value: &str) -> Result<()> {
        let value = if value.is_empty() {
            value.to_string()
        } else {
            self.interpolation.before_set(self, section, option, value).map_err(Error::new)?
        };
        let sectdict = if section.is_empty() || section == self.default_section {
            &mut self.defaults
        } else {
            self.sections.get_mut(section).ok_or_else(|| Error::no_section(section))?
        };
        sectdict.insert(optionxform(option), value);
        Ok(())
    }

    /// write(fp, space_around_delimiters=True) into a string
    pub fn write(&self) -> String {
        let mut out = String::new();
        if !self.defaults.is_empty() {
            self.write_section(&mut out, &self.default_section, &self.defaults);
        }
        for (name, items) in &self.sections {
            self.write_section(&mut out, name, items);
        }
        out
    }

    fn write_section(&self, out: &mut String, section_name: &str, items: &Section) {
        out.push_str(&format!("[{}]\n", section_name));
        for (key, value) in items {
            let value = self.interpolation.before_write(self, section_name, key, value);
            // Convert all possible line-endings into '\n\t'
            let value = value.replace("\r\n", "\n").replace('\r', "\n").replace('\n', "\n\t");
            out.push_str(&format!("{} = {}\n", key, value));
        }
        out.push('\n');
    }

    pub fn remove_option(&mut self, section: &str, option: &str) -> Result<bool> {
        let sectdict = if section.is_empty() || section == self.default_section {
            &mut self.defaults
        } else {
            self.sections.get_mut(section).ok_or_else(|| Error::no_section(section))?
        };
        Ok(sectdict.shift_remove(&optionxform(option)).is_some())
    }

    pub fn remove_section(&mut self, section: &str) -> bool {
        self.sections.shift_remove(section).is_some()
    }

    /// raw (uninterpolated) values of a section, for dumps
    pub fn raw_section(&self, section: &str) -> Option<&Section> {
        self.sections.get(section)
    }
}

pub fn convert_to_boolean(value: &str) -> std::result::Result<bool, String> {
    let lower = value.to_lowercase();
    BOOLEAN_STATES
        .iter()
        .find(|(k, _)| *k == lower)
        .map(|(_, v)| *v)
        .ok_or_else(|| format!("Not a boolean: {}", value))
}

/// SECTCRE.match(): r"\[(?P<header>.+)\]" (greedy, so up to the last ']')
pub fn match_sectcre(value: &str) -> Option<String> {
    let rest = value.strip_prefix('[')?;
    let pos = rest.rfind(']')?;
    if pos == 0 {
        return None;
    }
    // '.' does not match a newline
    let header = &rest[..pos];
    if header.contains('\n') {
        return None;
    }
    Some(header.to_string())
}

/// OPTCRE.match() for delimiters '=' / ':': (option, value) where option is
/// the text before the first delimiter without trailing whitespace and value
/// the rest without leading whitespace
pub fn match_optcre(value: &str) -> Option<(String, String)> {
    let pos = value.find(['=', ':'])?;
    let name = py_rstrip(&value[..pos]).to_string();
    let val = value[pos + 1..].trim_start_matches(py_isspace).to_string();
    // `.*$`: no newline in the value
    if val.contains('\n') {
        return None;
    }
    Some((name, val))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(text: &str) -> (RawConfigParser, Option<String>) {
        let mut cp = RawConfigParser::new(false, &[";", "#"], Box::new(NoInterpolation));
        let err = cp.read_string(text, "f").err().map(|e| e.message);
        (cp, err)
    }

    // expected values from python 3.13 configparser (kaliru tools/scratch.sh probes)
    #[test]
    fn read_like_python() {
        let (cp, err) = p("[a]\n= x\n b ; c\n");
        assert_eq!(err.unwrap(), "Source contains parsing errors: 'f'\n\t[line  2]: '= x\\n'\n\t[line  3]: ' b ; c\\n'");
        assert_eq!(cp.raw_section("a").unwrap().get("").unwrap(), "x");

        let (cp, err) = p("[a]\nk=v;w\nk2 = v ;w\nk3 = v\t;w\n[a]\nk = 2\n\n[DEFAULT]\nd = 1\n");
        assert!(err.is_none());
        let a = cp.raw_section("a").unwrap();
        assert_eq!(a.iter().map(|(k, v)| format!("{}={}", k, v)).collect::<Vec<_>>(), ["k=2", "k2=v", "k3=v"]);
        assert_eq!(cp.defaults().get("d").unwrap(), "1");

        let (_, err) = p("junk\n");
        assert_eq!(err.unwrap(), "File contains no section headers.\nfile: 'f', line: 1\n'junk\\n'");

        let (cp, err) = p("[s]\nnovalue\nok=1\n  more\n\n  after blank\n;c\n  after comment\nalso bad\n");
        assert_eq!(err.unwrap(), "Source contains parsing errors: 'f'\n\t[line  2]: 'novalue\\n'\n\t[line  9]: 'also bad\\n'");
        assert_eq!(cp.raw_section("s").unwrap().get("ok").unwrap(), "1\nmore\n\nafter blank\nafter comment");

        let (cp, _) = p("[s]\nk:1\nk : 2\n[ s2 ]\n[x]y]\nv=a=b\n");
        assert_eq!(cp.sections(), ["s", " s2 ", "x]y"]);
        assert_eq!(cp.raw_section("x]y").unwrap().get("v").unwrap(), "a=b");

        let (cp, _) = p("[s]\nv=\n  x\n\n\n");
        assert_eq!(cp.raw_section("s").unwrap().get("v").unwrap(), "\nx");
        let (cp, _) = p("  [s]\nk=1\n");
        assert_eq!(cp.raw_section("s").unwrap().get("k").unwrap(), "1");
        assert_eq!(cp.write(), "[s]\nk = 1\n\n");
    }
}
