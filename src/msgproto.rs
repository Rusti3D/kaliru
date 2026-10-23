// Protocol definitions for firmware communication (port of klippy/msgproto.py)
//
// Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::msgblock;
use crate::pyfmt::{py_bytes_repr, py_float, py_int, py_int_base, py_int_digits, py_str, py_str_repr};
use indexmap::IndexMap;
use serde_json::Value;
use std::collections::HashMap;
use std::fmt;
use std::rc::Rc;

pub const DEFAULT_MESSAGES: [(&str, i64); 2] = [
    ("identify_response offset=%u data=%.*s", 0),
    ("identify offset=%u count=%c", 1),
];

pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_MAX: usize = 64;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_POS_LEN: usize = 0;
pub const MESSAGE_POS_SEQ: usize = 1;
pub const MESSAGE_TRAILER_CRC: usize = 3;
pub const MESSAGE_TRAILER_SYNC: usize = 1;
pub const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;
pub const MESSAGE_SEQ_MASK: u8 = 0x0F;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SYNC: u8 = 0x7E;

/// msgproto.error; `enum_params` is set for enumeration_error
#[derive(Clone, Debug)]
pub struct Error {
    pub msg: String,
    pub enum_params: Option<(String, String)>,
}

impl Error {
    pub fn new(msg: impl Into<String>) -> Self {
        Error { msg: msg.into(), enum_params: None }
    }
    fn enumeration(enum_name: &str, value: &Val) -> Self {
        let value = value.py_str();
        Error {
            msg: format!("Unknown value '{}' in enumeration '{}'", value, enum_name),
            enum_params: Some((enum_name.to_string(), value)),
        }
    }
    pub fn get_enum_params(&self) -> Option<&(String, String)> {
        self.enum_params.as_ref()
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(&self.msg)
    }
}

pub type Result<T> = std::result::Result<T, Error>;

pub fn crc16_ccitt(buf: &[u8]) -> [u8; 2] {
    let crc = msgblock::msgblock_crc16_ccitt(buf);
    [(crc >> 8) as u8, (crc & 0xFF) as u8]
}

/// A message parameter value: python int, bytes, or str (enumeration names)
#[derive(Clone, Debug, PartialEq)]
pub enum Val {
    Int(i64),
    Bytes(Vec<u8>),
    Str(String),
    /// the "#sent_time" / "#receive_time" entries serialhdl adds
    Float(f64),
}

impl Val {
    /// python str()
    pub fn py_str(&self) -> String {
        match self {
            Val::Int(i) => i.to_string(),
            Val::Bytes(b) => py_bytes_repr(b),
            Val::Str(s) => s.clone(),
            Val::Float(f) => crate::pyfmt::py_float_repr(*f),
        }
    }
    /// python repr()
    pub fn py_repr(&self) -> String {
        match self {
            Val::Str(s) => py_str_repr(s),
            v => v.py_str(),
        }
    }
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Val::Int(i) => Some(*i),
            _ => None,
        }
    }
}

pub type Params = IndexMap<String, Val>;

/// python repr() of a params dict
pub fn params_repr(params: &Params) -> String {
    let items: Vec<String> = params.iter().map(|(k, v)| format!("{}: {}", py_str_repr(k), v.py_repr())).collect();
    format!("{{{}}}", items.join(", "))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BaseType {
    Uint32,
    Int32,
    Uint16,
    Int16,
    Byte,
    String,
    ProgmemBuffer,
    Buffer,
}

#[derive(Clone, Debug)]
pub struct Enumeration {
    pub pt: BaseType,
    pub enum_name: String,
    pub enums: IndexMap<String, i64>,
    pub reverse_enums: HashMap<i64, String>,
}

/// PT_* classes and Enumeration
#[derive(Clone, Debug)]
pub enum ParamType {
    Base(BaseType),
    Enum(Rc<Enumeration>),
}

impl BaseType {
    pub fn is_int(self) -> bool {
        !self.is_dynamic_string()
    }
    pub fn is_dynamic_string(self) -> bool {
        matches!(self, BaseType::String | BaseType::ProgmemBuffer | BaseType::Buffer)
    }
    pub fn max_length(self) -> usize {
        match self {
            BaseType::Uint32 | BaseType::Int32 => 5,
            BaseType::Uint16 | BaseType::Int16 => 3,
            BaseType::Byte => 2,
            _ => 64,
        }
    }
    pub fn signed(self) -> bool {
        matches!(self, BaseType::Int32 | BaseType::Int16)
    }

    fn encode_int(out: &mut Vec<u8>, v: i64) {
        if v >= 0xC000000 || v < -0x4000000 {
            out.push((((v >> 28) & 0x7F) | 0x80) as u8);
        }
        if v >= 0x180000 || v < -0x80000 {
            out.push((((v >> 21) & 0x7F) | 0x80) as u8);
        }
        if v >= 0x3000 || v < -0x1000 {
            out.push((((v >> 14) & 0x7F) | 0x80) as u8);
        }
        if v >= 0x60 || v < -0x20 {
            out.push((((v >> 7) & 0x7F) | 0x80) as u8);
        }
        out.push((v & 0x7F) as u8);
    }

    pub fn encode(self, out: &mut Vec<u8>, v: &Val) -> Result<()> {
        match (self.is_dynamic_string(), v) {
            (false, Val::Int(i)) => BaseType::encode_int(out, *i),
            (true, Val::Bytes(b)) => {
                out.push(b.len() as u8);
                out.extend_from_slice(b);
            }
            // python raises TypeError (eg bytearray(str))
            _ => return Err(Error::new(format!("TypeError: can't encode {} as {:?}", v.py_repr(), self))),
        }
        Ok(())
    }

    pub fn parse(self, s: &[u8], pos: usize) -> Result<(Val, usize)> {
        let at = |i: usize| s.get(i).copied().ok_or_else(|| Error::new("index out of range"));
        let mut pos = pos;
        if self.is_dynamic_string() {
            let l = at(pos)? as usize;
            let end = (pos + l + 1).min(s.len());
            return Ok((Val::Bytes(s[(pos + 1).min(end)..end].to_vec()), pos + l + 1));
        }
        let mut c = at(pos)?;
        pos += 1;
        let mut v = (c & 0x7F) as i64;
        if (c & 0x60) == 0x60 {
            v |= -0x20;
        }
        while c & 0x80 != 0 {
            c = at(pos)?;
            pos += 1;
            v = (v << 7) | (c & 0x7F) as i64;
        }
        if !self.signed() {
            v &= 0xFFFFFFFF;
        }
        Ok((Val::Int(v), pos))
    }
}

impl ParamType {
    pub fn is_int(&self) -> bool {
        match self {
            ParamType::Base(b) => b.is_int(),
            ParamType::Enum(_) => false,
        }
    }
    pub fn is_dynamic_string(&self) -> bool {
        match self {
            ParamType::Base(b) => b.is_dynamic_string(),
            ParamType::Enum(_) => false,
        }
    }
    pub fn max_length(&self) -> usize {
        match self {
            ParamType::Base(b) => b.max_length(),
            ParamType::Enum(e) => e.pt.max_length(),
        }
    }
    pub fn encode(&self, out: &mut Vec<u8>, v: &Val) -> Result<()> {
        match self {
            ParamType::Base(b) => b.encode(out, v),
            ParamType::Enum(e) => {
                let tv = match v {
                    Val::Str(s) => e.enums.get(s),
                    _ => None,
                };
                match tv {
                    Some(tv) => e.pt.encode(out, &Val::Int(*tv)),
                    None => Err(Error::enumeration(&e.enum_name, v)),
                }
            }
        }
    }
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Val, usize)> {
        match self {
            ParamType::Base(b) => b.parse(s, pos),
            ParamType::Enum(e) => {
                let (v, pos) = e.pt.parse(s, pos)?;
                let i = v.as_int().unwrap_or(0);
                let tv = match e.reverse_enums.get(&i) {
                    Some(name) => name.clone(),
                    None => format!("?{}", i),
                };
                Ok((Val::Str(tv), pos))
            }
        }
    }
}

pub fn message_type(fmt: &str) -> Option<BaseType> {
    Some(match fmt {
        "%u" => BaseType::Uint32,
        "%i" => BaseType::Int32,
        "%hu" => BaseType::Uint16,
        "%hi" => BaseType::Int16,
        "%c" => BaseType::Byte,
        "%s" => BaseType::String,
        "%.*s" => BaseType::ProgmemBuffer,
        "%*s" => BaseType::Buffer,
        _ => return None,
    })
}

pub type Enumerations = IndexMap<String, IndexMap<String, i64>>;

// Lookup the message types for a format string
pub fn lookup_params(msgformat: &str, enumerations: &Enumerations) -> Result<Vec<(String, ParamType)>> {
    let mut out = vec![];
    for arg in msgformat.split_whitespace().skip(1) {
        let parts: Vec<&str> = arg.split('=').collect();
        // python: "for name, fmt in argparts" -> ValueError text (python 3.13, the reference venv)
        if parts.len() < 2 {
            return Err(Error::new(format!("not enough values to unpack (expected 2, got {})", parts.len())));
        } else if parts.len() > 2 {
            return Err(Error::new("too many values to unpack (expected 2)"));
        }
        let (name, fmt) = (parts[0], parts[1]);
        // python: KeyError, str() of it is the quoted key
        let base = message_type(fmt).ok_or_else(|| Error::new(format!("'{}'", fmt)))?;
        let mut pt = ParamType::Base(base);
        for (enum_name, enums) in enumerations {
            if name == enum_name || name.ends_with(&format!("_{}", enum_name)) {
                let mut reverse_enums = HashMap::new();
                for (k, v) in enums {
                    reverse_enums.insert(*v, k.clone());
                }
                pt = ParamType::Enum(Rc::new(Enumeration {
                    pt: base,
                    enum_name: enum_name.clone(),
                    enums: enums.clone(),
                    reverse_enums,
                }));
                break;
            }
        }
        out.push((name.to_string(), pt));
    }
    Ok(out)
}

// Lookup the message types for a debugging "output()" format string
pub fn lookup_output_params(msgformat: &str) -> Result<Vec<BaseType>> {
    let mut param_types = vec![];
    let mut args = msgformat;
    loop {
        let pos = match args.find('%') {
            Some(p) => p,
            None => break,
        };
        if pos + 1 >= args.len() || args.as_bytes()[pos + 1] != b'%' {
            let mut found = None;
            for i in 0..4 {
                if let Some(t) = args.get(pos..(pos + 1 + i).min(args.len())).and_then(message_type) {
                    found = Some(t);
                    break;
                }
            }
            match found {
                Some(t) => param_types.push(t),
                None => return Err(Error::new(format!("Invalid output format for '{}'", msgformat))),
            }
        }
        args = &args[pos + 1..];
    }
    Ok(param_types)
}

// Update the message format to be compatible with python's % operator
pub fn convert_msg_format(msgformat: &str) -> String {
    let mut msgformat = msgformat.to_string();
    for c in ["%u", "%i", "%hu", "%hi", "%c", "%.*s", "%*s"] {
        msgformat = msgformat.replace(c, "%s");
    }
    msgformat
}

/// python `fmt % tuple(args)` for formats holding only %s and %%
fn percent_format(fmt: &str, args: &[String]) -> Result<String> {
    let mut out = String::with_capacity(fmt.len() + 16);
    let mut args = args.iter();
    let mut chars = fmt.chars();
    while let Some(c) = chars.next() {
        if c != '%' {
            out.push(c);
            continue;
        }
        match chars.next() {
            Some('s') => out.push_str(args.next().ok_or_else(|| Error::new("not enough arguments for format string"))?),
            Some('%') => out.push('%'),
            _ => return Err(Error::new("unsupported format character")),
        }
    }
    if args.next().is_some() {
        return Err(Error::new("not all arguments converted during string formatting"));
    }
    Ok(out)
}

#[derive(Debug)]
pub struct MessageFormat {
    pub msgid_bytes: Vec<u8>,
    pub msgformat: String,
    pub debugformat: String,
    pub name: String,
    pub param_names: Vec<(String, ParamType)>,
    pub name_to_type: HashMap<String, ParamType>,
}

impl MessageFormat {
    pub fn new(msgid_bytes: Vec<u8>, msgformat: &str, enumerations: &Enumerations) -> Result<Self> {
        let param_names = lookup_params(msgformat, enumerations)?;
        Ok(MessageFormat {
            msgid_bytes,
            msgformat: msgformat.to_string(),
            debugformat: convert_msg_format(msgformat),
            name: msgformat.split_whitespace().next().unwrap_or("").to_string(),
            name_to_type: param_names.iter().cloned().collect(),
            param_names,
        })
    }
    pub fn param_types(&self) -> impl Iterator<Item = &ParamType> {
        self.param_names.iter().map(|(_, t)| t)
    }
    pub fn encode(&self, params: &[Val]) -> Result<Vec<u8>> {
        let mut out = self.msgid_bytes.clone();
        for (i, t) in self.param_types().enumerate() {
            t.encode(&mut out, params.get(i).ok_or_else(|| Error::new("tuple index out of range"))?)?;
        }
        Ok(out)
    }
    pub fn encode_by_name(&self, params: &Params) -> Result<Vec<u8>> {
        let mut out = self.msgid_bytes.clone();
        for (name, t) in &self.param_names {
            t.encode(&mut out, params.get(name).ok_or_else(|| Error::new(format!("'{}'", name)))?)?;
        }
        Ok(out)
    }
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        let mut pos = pos + self.msgid_bytes.len();
        let mut out = Params::new();
        for (name, t) in &self.param_names {
            let (v, p) = t.parse(s, pos)?;
            pos = p;
            out.insert(name.clone(), v);
        }
        Ok((out, pos))
    }
    pub fn format_params(&self, params: &Params) -> Result<String> {
        let mut out = vec![];
        for (name, t) in &self.param_names {
            let v = params.get(name).ok_or_else(|| Error::new(format!("'{}'", name)))?;
            // dynamic strings are repr()'d, the rest str()'d by %s
            out.push(if t.is_dynamic_string() { v.py_repr() } else { v.py_str() });
        }
        percent_format(&self.debugformat, &out)
    }
}

#[derive(Debug)]
pub struct OutputFormat {
    pub msgid_bytes: Vec<u8>,
    pub msgformat: String,
    pub debugformat: String,
    pub param_types: Vec<BaseType>,
}

impl OutputFormat {
    pub const NAME: &'static str = "#output";
    pub fn new(msgid_bytes: Vec<u8>, msgformat: &str) -> Result<Self> {
        Ok(OutputFormat {
            msgid_bytes,
            msgformat: msgformat.to_string(),
            debugformat: convert_msg_format(msgformat),
            param_types: lookup_output_params(msgformat)?,
        })
    }
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        let mut pos = pos + self.msgid_bytes.len();
        let mut out = vec![];
        for t in &self.param_types {
            let (v, p) = t.parse(s, pos)?;
            pos = p;
            out.push(if t.is_dynamic_string() { v.py_repr() } else { v.py_str() });
        }
        let outmsg = percent_format(&self.debugformat, &out)?;
        let mut params = Params::new();
        params.insert("#msg".into(), Val::Str(outmsg));
        Ok((params, pos))
    }
    pub fn format_params(&self, params: &Params) -> String {
        format!("#output {}", params.get("#msg").map(|v| v.py_str()).unwrap_or_default())
    }
}

pub struct UnknownFormat;

impl UnknownFormat {
    pub const NAME: &'static str = "#unknown";
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        let (msgid, _param_pos) = BaseType::Int32.parse(s, pos)?;
        let mut params = Params::new();
        params.insert("#msgid".into(), msgid);
        params.insert("#msg".into(), Val::Bytes(s.to_vec()));
        Ok((params, s.len().saturating_sub(MESSAGE_TRAILER_SIZE)))
    }
    pub fn format_params(&self, params: &Params) -> String {
        format!("#unknown {}", params.get("#msg").map(|v| v.py_repr()).unwrap_or_default())
    }
}

/// An entry of messages_by_id
#[derive(Clone, Debug)]
pub enum MsgFormat {
    Message(Rc<MessageFormat>),
    Output(Rc<OutputFormat>),
}

impl MsgFormat {
    pub fn name(&self) -> &str {
        match self {
            MsgFormat::Message(m) => &m.name,
            MsgFormat::Output(_) => OutputFormat::NAME,
        }
    }
    pub fn parse(&self, s: &[u8], pos: usize) -> Result<(Params, usize)> {
        match self {
            MsgFormat::Message(m) => m.parse(s, pos),
            MsgFormat::Output(o) => o.parse(s, pos),
        }
    }
    pub fn format_params(&self, params: &Params) -> Result<String> {
        match self {
            MsgFormat::Message(m) => m.format_params(params),
            MsgFormat::Output(o) => Ok(o.format_params(params)),
        }
    }
}

pub struct MessageParser {
    pub warn_prefix: String,
    unknown: UnknownFormat,
    pub enumerations: Enumerations,
    pub messages: Vec<(i64, &'static str, String)>,
    pub messages_by_id: HashMap<i64, MsgFormat>,
    pub messages_by_name: HashMap<String, Rc<MessageFormat>>,
    pub msgid_by_format: HashMap<String, i64>,
    pub config: serde_json::Map<String, Value>,
    pub kconfig: Value,
    pub app: Value,
    pub version: Value,
    pub build_versions: Value,
    pub raw_identify_data: Vec<u8>,
}

/// default for get_constant*() ("sentinel" = no default)
pub enum Default<T> {
    Sentinel,
    Value(T),
}

impl MessageParser {
    pub fn new(warn_prefix: &str) -> Self {
        let mut mp = MessageParser {
            warn_prefix: warn_prefix.to_string(),
            unknown: UnknownFormat,
            enumerations: Enumerations::new(),
            messages: vec![],
            messages_by_id: HashMap::new(),
            messages_by_name: HashMap::new(),
            msgid_by_format: HashMap::new(),
            config: serde_json::Map::new(),
            kconfig: Value::Null,
            app: Value::Null,
            version: Value::String(String::new()),
            build_versions: Value::String(String::new()),
            raw_identify_data: vec![],
        };
        let defaults: Vec<(String, i64)> = DEFAULT_MESSAGES.iter().map(|(f, i)| (f.to_string(), *i)).collect();
        mp.init_messages(&defaults, &[], &[]).ok().expect("default messages");
        mp
    }

    fn error(&self, msg: String) -> Error {
        Error::new(format!("{}{}", self.warn_prefix, msg))
    }

    pub fn check_packet(&self, s: &[u8]) -> isize {
        if s.len() < MESSAGE_MIN {
            return 0;
        }
        let msglen = s[MESSAGE_POS_LEN] as usize;
        if msglen < MESSAGE_MIN || msglen > MESSAGE_MAX {
            return -1;
        }
        let msgseq = s[MESSAGE_POS_SEQ];
        if (msgseq & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
            return -1;
        }
        if s.len() < msglen {
            // Need more data
            return 0;
        }
        if s[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
            return -1;
        }
        let msgcrc = &s[msglen - MESSAGE_TRAILER_CRC..msglen - MESSAGE_TRAILER_CRC + 2];
        let crc = crc16_ccitt(&s[..msglen - MESSAGE_TRAILER_SIZE]);
        if crc[..] != *msgcrc {
            return -1;
        }
        msglen as isize
    }

    fn lookup_id(&self, msgid: &Val) -> Option<&MsgFormat> {
        msgid.as_int().and_then(|i| self.messages_by_id.get(&i))
    }

    pub fn dump(&self, s: &[u8]) -> Result<Vec<String>> {
        let msgseq = s[MESSAGE_POS_SEQ];
        let mut out = vec![format!("seq: {:02x}", msgseq)];
        let mut pos = MESSAGE_HEADER_SIZE;
        loop {
            let (msgid, _param_pos) = BaseType::Int32.parse(s, pos)?;
            let (params, p) = match self.lookup_id(&msgid) {
                Some(mid) => {
                    let (params, p) = mid.parse(s, pos)?;
                    out.push(mid.format_params(&params)?);
                    (params, p)
                }
                None => {
                    let (params, p) = self.unknown.parse(s, pos)?;
                    out.push(self.unknown.format_params(&params));
                    (params, p)
                }
            };
            let _ = params;
            pos = p;
            if pos >= s.len().saturating_sub(MESSAGE_TRAILER_SIZE) {
                break;
            }
        }
        Ok(out)
    }

    pub fn format_params(&self, params: &Params) -> Result<String> {
        let name = params.get("#name");
        if let Some(Val::Str(n)) = name {
            if let Some(mid) = self.messages_by_name.get(n) {
                return mid.format_params(params);
            }
        }
        if let Some(msg) = params.get("#msg") {
            let name = name.map(|n| n.py_str()).unwrap_or_else(|| "None".into());
            return Ok(format!("{} {}", name, msg.py_str()));
        }
        Ok(params_repr(params))
    }

    pub fn parse(&self, s: &[u8]) -> Result<Params> {
        let (msgid, _param_pos) = BaseType::Int32.parse(s, MESSAGE_HEADER_SIZE)?;
        let (mut params, pos, name) = match self.lookup_id(&msgid) {
            Some(mid) => {
                let (p, pos) = mid.parse(s, MESSAGE_HEADER_SIZE)?;
                (p, pos, mid.name().to_string())
            }
            None => {
                let (p, pos) = self.unknown.parse(s, MESSAGE_HEADER_SIZE)?;
                (p, pos, UnknownFormat::NAME.to_string())
            }
        };
        if pos != s.len().saturating_sub(MESSAGE_TRAILER_SIZE) {
            return Err(self.error("Extra data at end of message".into()));
        }
        params.insert("#name".into(), Val::Str(name));
        Ok(params)
    }

    // encode_msgblock() is not ported: nothing in Kalico calls it (blocks are
    // built by serialqueue) and the python version appends the crc as a nested
    // list, so its result is not a valid message block.

    fn parse_buffer(value: &str) -> Option<Vec<u8>> {
        if value.is_empty() {
            return Some(vec![]);
        }
        // tval = int(value, 16); the low len(value)//2 bytes, big endian
        let (neg, _, digits) = py_int_digits(value, 16)?;
        let n = value.chars().count() / 2;
        let mut out = vec![0u8; n];
        let mut i = n;
        let mut it = digits.iter().rev();
        while i > 0 {
            let lo = it.next().copied().unwrap_or(0);
            let hi = it.next().copied().unwrap_or(0);
            i -= 1;
            out[i] = (hi << 4) | lo;
        }
        if neg {
            // python's `tval & 0xFF; tval >>= 8` on a negative int: the low
            // bytes of the two's complement
            let mut carry = true;
            for b in out.iter_mut().rev() {
                let (v, c) = (!*b).overflowing_add(carry as u8);
                *b = v;
                carry = c;
            }
        }
        Some(out)
    }

    pub fn lookup_command(&self, msgformat: &str) -> Result<Rc<MessageFormat>> {
        let msgname = msgformat.split_whitespace().next().unwrap_or("");
        let mp = self
            .messages_by_name
            .get(msgname)
            .ok_or_else(|| self.error(format!("Unknown command: {}", msgname)))?;
        if msgformat != mp.msgformat {
            return Err(self.error(format!("Command format mismatch: {} vs {}", msgformat, mp.msgformat)));
        }
        Ok(mp.clone())
    }

    pub fn lookup_msgid(&self, msgformat: &str) -> Result<i64> {
        self.msgid_by_format
            .get(msgformat)
            .copied()
            .ok_or_else(|| self.error(format!("Unknown command: {}", msgformat)))
    }

    pub fn create_command(&self, msg: &str) -> Result<Vec<u8>> {
        let parts: Vec<&str> = msg.split_whitespace().collect();
        if parts.is_empty() {
            // python returns "" here
            return Ok(vec![]);
        }
        let msgname = parts[0];
        let mp = self
            .messages_by_name
            .get(msgname)
            .ok_or_else(|| self.error(format!("Unknown command: {}", msgname)))?;
        let extract_err = || self.error(format!("Unable to extract params from: {}", msgname));
        let mut argparts = Params::new();
        for arg in &parts[1..] {
            let (name, value) = arg.split_once('=').ok_or_else(extract_err)?;
            argparts.insert(name.to_string(), Val::Str(value.to_string()));
        }
        for (name, v) in argparts.iter_mut() {
            let t = mp.name_to_type.get(name).ok_or_else(extract_err)?;
            let value = match v {
                Val::Str(s) => s.clone(),
                _ => unreachable!(),
            };
            *v = if t.is_dynamic_string() {
                Val::Bytes(Self::parse_buffer(&value).ok_or_else(extract_err)?)
            } else if t.is_int() {
                Val::Int(py_int_base(&value, 0).ok_or_else(extract_err)?)
            } else {
                Val::Str(value)
            };
        }
        mp.encode_by_name(&argparts).map_err(|e| {
            if e.enum_params.is_some() {
                e
            } else {
                self.error(format!("Unable to encode: {}", msgname))
            }
        })
    }

    pub fn fill_enumerations(&mut self, enumerations: &serde_json::Map<String, Value>) -> Result<()> {
        for (add_name, add_enums) in enumerations {
            let add_enums = add_enums
                .as_object()
                .ok_or_else(|| Error::new(format!("'{}' object has no attribute 'items'", py_type(add_enums))))?;
            let enums = self.enumerations.entry(add_name.clone()).or_default();
            for (enum_, value) in add_enums {
                if let Some(v) = value.as_i64().filter(|_| !value.is_f64()) {
                    // Simple enumeration
                    enums.insert(enum_.clone(), v);
                    continue;
                }
                // Enumeration range
                let enum_root = enum_.trim_end_matches(|c: char| c.is_ascii_digit());
                let mut start_enum = 0;
                if enum_root.len() != enum_.len() {
                    start_enum = enum_[enum_root.len()..]
                        .parse::<i64>()
                        .map_err(|e| Error::new(e.to_string()))?;
                }
                let range = value.as_array().filter(|a| a.len() == 2);
                let (start_value, count) = match range.map(|a| (a[0].as_i64(), a[1].as_i64())) {
                    Some((Some(s), Some(c))) => (s, c),
                    _ => return Err(Error::new("cannot unpack enumeration range")),
                };
                for i in 0..count.max(0) {
                    enums.insert(format!("{}{}", enum_root, start_enum + i), start_value + i);
                }
            }
        }
        Ok(())
    }

    // Err(Ok(e)): a msgproto.error (python re-raises those unchanged from
    // process_identify); Err(Err(s)): any other python exception text
    fn init_messages(
        &mut self,
        messages: &[(String, i64)],
        command_ids: &[i64],
        output_ids: &[i64],
    ) -> std::result::Result<(), std::result::Result<Error, String>> {
        for (msgformat, msgid) in messages {
            let msgid = *msgid;
            let msgtype = if command_ids.contains(&msgid) {
                "command"
            } else if output_ids.contains(&msgid) {
                "output"
            } else {
                "response"
            };
            self.messages.push((msgid, msgtype, msgformat.clone()));
            self.msgid_by_format.insert(msgformat.clone(), msgid);
            let mut msgid_bytes = vec![];
            BaseType::encode_int(&mut msgid_bytes, msgid);
            if msgtype == "output" {
                let f = OutputFormat::new(msgid_bytes, msgformat).map_err(Ok)?;
                self.messages_by_id.insert(msgid, MsgFormat::Output(Rc::new(f)));
            } else {
                let msg = MessageFormat::new(msgid_bytes, msgformat, &self.enumerations).map_err(|e| Err(e.msg))?;
                let msg = Rc::new(msg);
                self.messages_by_id.insert(msgid, MsgFormat::Message(msg.clone()));
                self.messages_by_name.insert(msg.name.clone(), msg);
            }
        }
        Ok(())
    }

    pub fn process_identify(&mut self, data: &[u8], decompress: bool) -> Result<()> {
        // ponytail: python reports str(e) of the zlib / json exception (and
        // logs a traceback); those texts come from the rust decoders and differ
        let res = (|| -> std::result::Result<(), std::result::Result<Error, String>> {
            let data = if decompress {
                let mut out = vec![];
                use std::io::Read;
                flate2::read::ZlibDecoder::new(data).read_to_end(&mut out).map_err(|e| Err(e.to_string()))?;
                out
            } else {
                data.to_vec()
            };
            self.raw_identify_data = data.clone();
            let data: Value = serde_json::from_slice(&data).map_err(|e| Err(e.to_string()))?;
            let empty = serde_json::Map::new();
            let get_obj = |k: &str| data.get(k).and_then(|v| v.as_object());
            self.fill_enumerations(get_obj("enumerations").unwrap_or(&empty)).map_err(|e| Err(e.msg))?;
            let not_iterable = || Err("'NoneType' object is not iterable".to_string());
            let commands = get_obj("commands").ok_or_else(not_iterable)?;
            let responses = get_obj("responses").ok_or_else(not_iterable)?;
            let output = get_obj("output").unwrap_or(&empty);
            let mut all_messages: IndexMap<String, i64> = IndexMap::new();
            let id = |v: &Value| v.as_i64().ok_or_else(|| Err("invalid message id".to_string()));
            for m in [commands, responses, output] {
                for (k, v) in m {
                    all_messages.insert(k.clone(), id(v)?);
                }
            }
            let ids = |m: &serde_json::Map<String, Value>| m.values().map(id).collect::<std::result::Result<Vec<_>, _>>();
            let all: Vec<(String, i64)> = all_messages.into_iter().collect();
            self.init_messages(&all, &ids(commands)?, &ids(output)?)?;
            if let Some(c) = get_obj("config") {
                for (k, v) in c {
                    self.config.insert(k.clone(), v.clone());
                }
            }
            self.kconfig = data.get("kconfig").cloned().unwrap_or(Value::Null);
            self.app = data.get("app").cloned().unwrap_or(Value::String(String::new()));
            self.version = data.get("version").cloned().unwrap_or(Value::String(String::new()));
            self.build_versions = data.get("build_versions").cloned().unwrap_or(Value::String(String::new()));
            Ok(())
        })();
        res.map_err(|e| match e {
            Ok(e) => e,
            Err(s) => self.error(format!("Error during identify: {}", s)),
        })
    }

    pub fn get_raw_data_dictionary(&self) -> &[u8] {
        &self.raw_identify_data
    }
    pub fn get_app_info(&self) -> &Value {
        &self.app
    }
    pub fn get_version_info(&self) -> (&Value, &Value) {
        (&self.version, &self.build_versions)
    }
    pub fn get_messages(&self) -> Vec<(i64, &'static str, String)> {
        self.messages.clone()
    }
    pub fn get_enumerations(&self) -> Enumerations {
        self.enumerations.clone()
    }
    pub fn get_constants(&self) -> serde_json::Map<String, Value> {
        self.config.clone()
    }
    pub fn get_kconfig(&self) -> &Value {
        &self.kconfig
    }

    fn constant<T>(&self, name: &str, default: Default<T>, parser: impl Fn(&Value) -> Option<T>) -> Result<T> {
        let value = match self.config.get(name) {
            Some(v) => v,
            None => {
                return match default {
                    Default::Value(d) => Ok(d),
                    Default::Sentinel => Err(self.error(format!("Firmware constant '{}' not found", name))),
                }
            }
        };
        parser(value)
            .ok_or_else(|| self.error(format!("Unable to parse firmware constant {}: {}", name, py_str(value))))
    }

    /// get_constant(name, default) with parser=str
    pub fn get_constant(&self, name: &str, default: Default<String>) -> Result<String> {
        self.constant(name, default, |v| Some(py_str(v)))
    }

    pub fn get_constant_float(&self, name: &str, default: Default<f64>) -> Result<f64> {
        self.constant(name, default, |v| match v {
            Value::String(s) => py_float(s),
            Value::Number(n) => n.as_f64(),
            Value::Bool(b) => Some(*b as i64 as f64),
            _ => None,
        })
    }

    pub fn get_constant_int(&self, name: &str, default: Default<i64>) -> Result<i64> {
        self.constant(name, default, |v| match v {
            Value::String(s) => py_int(s),
            // int(float) truncates toward zero
            Value::Number(n) => n.as_i64().or_else(|| n.as_f64().filter(|f| f.is_finite()).map(|f| f.trunc() as i64)),
            Value::Bool(b) => Some(*b as i64),
            _ => None,
        })
    }
}

fn py_type(v: &Value) -> &'static str {
    crate::pyfmt::py_type_name(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DICT: &str = r#"{"commands": {"set_pin oid=%c pin=%u": 5, "send buf=%*s": 6},
        "responses": {"result oid=%c value=%i": 7},
        "output": {"debug %u %s": 8},
        "enumerations": {"pin": {"PA0": 0, "PB": [16, 3]}},
        "config": {"CLOCK_FREQ": 16000000, "MCU": "atmega2560"}}"#;

    fn mp() -> MessageParser {
        let mut mp = MessageParser::new("");
        mp.process_identify(DICT.as_bytes(), false).unwrap();
        mp
    }

    #[test]
    fn create_and_parse() {
        let mp = mp();
        let cmd = mp.create_command("set_pin oid=3 pin=PB2").unwrap();
        assert_eq!(cmd, vec![5, 3, 18]);
        let err = mp.create_command("set_pin oid=3 pin=PC9").unwrap_err();
        assert_eq!(err.msg, "Unknown value 'PC9' in enumeration 'pin'");
        assert_eq!(mp.create_command("set_pin oid=x pin=PB2").unwrap_err().msg, "Unable to extract params from: set_pin");
        assert_eq!(mp.create_command("send buf=0x0102").unwrap(), vec![6, 3, 0, 1, 2]);
        // frame it and dump
        let mut s = vec![0, 0x10];
        s.extend(&cmd);
        s.extend([0, 0, MESSAGE_SYNC]);
        s[0] = s.len() as u8;
        let n = s.len();
        let crc = crc16_ccitt(&s[..n - 3]);
        s[n - 3..n - 1].copy_from_slice(&crc);
        assert_eq!(mp.check_packet(&s), n as isize);
        assert_eq!(mp.dump(&s).unwrap(), vec!["seq: 10", "set_pin oid=3 pin=PB2"]);
        assert_eq!(mp.get_constant_float("CLOCK_FREQ", Default::Sentinel).unwrap(), 16000000.0);
        assert_eq!(mp.get_constant("MCU", Default::Sentinel).unwrap(), "atmega2560");
        assert!(mp.get_constant_int("MCU", Default::Sentinel).is_err());
    }

    #[test]
    fn encode_like_python() {
        // expected bytes from python msgproto.PT_uint32().encode (Kalico 5b59427)
        let cases: &[(i64, &[u8])] = &[(-1, &[0x7f]), (-32, &[0x60]), (-33, &[0xff, 0x5f]), (95, &[0x5f]), (96, &[0x80, 0x60]), (-4096, &[0xe0, 0x00]), (-4097, &[0xff, 0xdf, 0x7f]), (12287, &[0xdf, 0x7f]), (12288, &[0x80, 0xe0, 0x00]), (-524288, &[0xe0, 0x80, 0x00]), (-524289, &[0xff, 0xdf, 0xff, 0x7f]), (1572863, &[0xdf, 0xff, 0x7f]), (1572864, &[0x80, 0xe0, 0x80, 0x00]), (-67108864, &[0xe0, 0x80, 0x80, 0x00]), (-67108865, &[0xff, 0xdf, 0xff, 0xff, 0x7f]), (201326591, &[0xdf, 0xff, 0xff, 0x7f]), (201326592, &[0x80, 0xe0, 0x80, 0x80, 0x00]), (4294967295, &[0x8f, 0xff, 0xff, 0xff, 0x7f]), (4294967296, &[0x90, 0x80, 0x80, 0x80, 0x00]), (34359738373, &[0x80, 0x80, 0x80, 0x80, 0x05]), (1099511627783, &[0x80, 0x80, 0x80, 0x80, 0x07]), (-8589934592, &[0xe0, 0x80, 0x80, 0x80, 0x00]), (4611686018427387904, &[0x80, 0x80, 0x80, 0x80, 0x00])];
        for (v, want) in cases {
            let mut out = vec![];
            BaseType::Uint32.encode(&mut out, &Val::Int(*v)).unwrap();
            assert_eq!(&out[..], *want, "value {}", v);
        }
        // expected results from python MessageParser.create_command
        let mut mp = MessageParser::new("");
        mp.process_identify(br#"{"commands": {"send oid=%c buf=%*s": 6}, "responses": {}}"#, false).unwrap();
        let ok: &[(&str, &[u8])] = &[
            ("send oid=1 buf=0x123", &[6, 1, 2, 1, 35]),
            ("send oid=1 buf=abc", &[6, 1, 1, 188]),
            ("send oid=1 buf=1_2", &[6, 1, 1, 18]),
            ("send oid=1 buf=", &[6, 1, 0]),
            ("send oid=0x10 buf=00ff", &[6, 16, 2, 0, 255]),
            ("send oid=-1 buf=ff", &[6, 127, 1, 255]),
            ("send oid=1 buf=-1", &[6, 1, 1, 255]),
        ];
        for (c, want) in ok {
            assert_eq!(&mp.create_command(c).unwrap()[..], *want, "{}", c);
        }
        let err = [
            ("send oid=1", "Unable to encode: send"),
            ("send oid=1 buf=ff extra=3", "Unable to extract params from: send"),
            ("send oid=1 buf=zz", "Unable to extract params from: send"),
            ("send oid=1 buf=0x", "Unable to extract params from: send"),
        ];
        for (c, want) in err {
            assert_eq!(mp.create_command(c).unwrap_err().msg, want, "{}", c);
        }
    }

    #[test]
    fn output_params_like_python() {
        // expected values from python msgproto.lookup_output_params
        assert!(lookup_output_params("debug %u %s%%").is_err());
        assert!(lookup_output_params("a %% b %u").is_err());
        assert_eq!(lookup_output_params("x %%u").unwrap(), vec![BaseType::Uint32]);
        assert_eq!(lookup_output_params("debug %u %s").unwrap(), vec![BaseType::Uint32, BaseType::String]);
    }
}
