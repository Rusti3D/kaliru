// scripts/buildcommands.py: from the compile time requests to the data
// dictionary and the generated tables (ctr_generated.rs; the C version
// writes out/compile_time_request.c).
use crate::eval::{ctr_int, Eval};
use crate::json::Json;
use crate::scan::Decl;
use std::collections::BTreeMap;

const STATIC_STRING_MIN: usize = 2;
const MESSAGE_MIN: usize = 5;
const MESSAGE_MAX: usize = 64;

/// a compile time request of a source file, ready for the handlers: the
/// C text line plus the module path its functions live in
pub struct Request {
    pub decl: Decl,
    pub module: String,
}

/// msgproto.MessageTypes: (name, max_length)
fn param_type(fmt: &str) -> Option<(&'static str, usize)> {
    Some(match fmt {
        "%u" => ("PT_uint32", 5),
        "%i" => ("PT_int32", 5),
        "%hu" => ("PT_uint16", 3),
        "%hi" => ("PT_int16", 3),
        "%c" => ("PT_byte", 2),
        "%s" => ("PT_string", 64),
        "%.*s" => ("PT_progmem_buffer", 64),
        "%*s" => ("PT_buffer", 64),
        _ => return None,
    })
}

/// msgproto.lookup_params
fn lookup_params(msgformat: &str) -> Vec<(&'static str, usize)> {
    msgformat
        .split_whitespace()
        .skip(1)
        .map(|arg| {
            let (_, fmt) = arg.split_once('=').unwrap_or_else(|| panic!("bad message format {:?}", msgformat));
            param_type(fmt).unwrap_or_else(|| panic!("unknown type {} in {:?}", fmt, msgformat))
        })
        .collect()
}

/// msgproto.lookup_output_params
fn lookup_output_params(msgformat: &str) -> Vec<(&'static str, usize)> {
    let mut out = vec![];
    let mut args = msgformat;
    while let Some(pos) = args.find('%') {
        if args.as_bytes().get(pos + 1) != Some(&b'%') {
            let t = (0..4).find_map(|i| args.get(pos..pos + 2 + i).and_then(param_type));
            out.push(t.unwrap_or_else(|| panic!("Invalid output format for '{}'", msgformat)));
        }
        args = &args[pos + 1..];
    }
    out
}

/// HandleCommandGeneration.convert_encoded_msgid: PT_int32 parse
fn convert_encoded_msgid(encoded_msgid: i64) -> i64 {
    let data: Vec<i64> = if encoded_msgid >= 0x80 {
        vec![(encoded_msgid >> 7) | 0x80, encoded_msgid & 0x7f]
    } else {
        vec![encoded_msgid]
    };
    let mut c = data[0];
    let mut v = c & 0x7f;
    if (c & 0x60) == 0x60 {
        v |= -0x20;
    }
    let mut i = 1;
    while c & 0x80 != 0 {
        c = data[i];
        i += 1;
        v = (v << 7) | (c & 0x7f);
    }
    v
}

/// an insertion ordered map (python dict)
struct Ordered<V> {
    keys: Vec<String>,
    map: BTreeMap<String, V>,
}

impl<V> Ordered<V> {
    fn new() -> Self {
        Ordered { keys: vec![], map: BTreeMap::new() }
    }
    fn insert(&mut self, k: &str, v: V) {
        if self.map.insert(k.to_string(), v).is_none() {
            self.keys.push(k.to_string());
        }
    }
    fn get(&self, k: &str) -> Option<&V> {
        self.map.get(k)
    }
    fn iter(&self) -> impl Iterator<Item = (&String, &V)> {
        self.keys.iter().map(|k| (k, &self.map[k]))
    }
}

pub struct Output {
    /// compile_time_request.txt
    pub txt: String,
    /// the data dictionary (json)
    pub dict: String,
    /// ctr_generated.rs
    pub code: String,
}

enum EnumValue {
    Int(i64),
    Range(i64, i64),
}

pub struct Versions {
    pub version: String,
    pub build_versions: String,
}

pub fn generate(requests: &[Request], eval: &Eval, versions: &Versions, kconfig: &str) -> Output {
    let mut txt = String::new();
    // HandleCallList (ctr_run_initfuncs is always defined)
    let mut call_lists: Ordered<Vec<String>> = Ordered::new();
    call_lists.insert("ctr_run_initfuncs", vec![]);
    // HandleEnumerations
    let mut static_strings: Vec<String> = vec![];
    let mut enumerations: Ordered<Ordered<EnumValue>> = Ordered::new();
    // HandleConstants
    let mut constants: Ordered<Json> = Ordered::new();
    // HandleInitialPins
    let mut initial_pins: Vec<String> = vec![];
    // HandleCommandGeneration
    let mut commands: Ordered<(String, String)> = Ordered::new();
    let mut encoders: Vec<(Option<String>, String)> = vec![];
    let mut msg_to_encid: Ordered<i64> = Ordered::new();
    msg_to_encid.insert("identify_response offset=%u data=%.*s", 0);
    msg_to_encid.insert("identify offset=%u count=%c", 1);
    let mut messages_by_name: Ordered<String> = Ordered::new();
    for (m, _) in msg_to_encid.iter() {
        messages_by_name.insert(m.split_whitespace().next().unwrap(), m.clone());
    }

    let add_enum = |enumerations: &mut Ordered<Ordered<EnumValue>>, e: &str, name: &str, v: EnumValue| {
        if enumerations.get(e).is_none() {
            enumerations.insert(e, Ordered::new());
        }
        let enums = enumerations.map.get_mut(e).unwrap();
        if let Some(old) = enums.get(name) {
            let same = match (old, &v) {
                (EnumValue::Int(a), EnumValue::Int(b)) => a == b,
                (EnumValue::Range(a, b), EnumValue::Range(c, d)) => a == c && b == d,
                _ => false,
            };
            if !same {
                panic!("Conflicting definition for enumeration '{} {}'", e, name);
            }
        }
        enums.insert(name, v);
    };
    let set_constant = |constants: &mut Ordered<Json>, name: &str, v: Json| {
        if let Some(old) = constants.get(name) {
            if crate::json::dumps(old) != crate::json::dumps(&v) {
                panic!("Conflicting definition for constant '{}'", name);
            }
        }
        constants.insert(name, v);
    };

    for r in requests {
        let line = match &r.decl {
            Decl::Command { func, flags, msg } => {
                let msgname = msg.split_whitespace().next().unwrap().to_string();
                if commands.get(&msgname).is_some() {
                    panic!("Multiple definitions for command '{}'", msgname);
                }
                commands.insert(&msgname, (format!("{}::{}", r.module, func), flags.clone()));
                if let Some(m) = messages_by_name.get(&msgname) {
                    if m != msg {
                        panic!("Conflicting definition for command '{}'", msgname);
                    }
                }
                messages_by_name.insert(&msgname, msg.clone());
                format!("DECL_COMMAND_FLAGS {} {} {}", func, flags, msg)
            }
            Decl::Encoder(msg) => {
                let msgname = msg.split_whitespace().next().unwrap().to_string();
                if let Some(m) = messages_by_name.get(&msgname) {
                    if m != msg {
                        panic!("Conflicting definition for message '{}'", msgname);
                    }
                }
                messages_by_name.insert(&msgname, msg.clone());
                encoders.push((Some(msgname), msg.clone()));
                format!("_DECL_ENCODER {}", msg)
            }
            Decl::Output(msg) => {
                encoders.push((None, msg.clone()));
                format!("_DECL_OUTPUT {}", msg)
            }
            Decl::StaticStr(msg) => {
                if !static_strings.contains(msg) {
                    static_strings.push(msg.clone());
                }
                format!("_DECL_STATIC_STR {}", msg)
            }
            Decl::CallList { list, func } => {
                if call_lists.get(list).is_none() {
                    call_lists.insert(list, vec![]);
                }
                call_lists.map.get_mut(*list).unwrap().push(format!("{}::{}", r.module, func));
                format!("_DECL_CALLLIST {} {}", list, func)
            }
            Decl::Constant { name, expr } => {
                let v = eval.eval(expr);
                set_constant(&mut constants, name, Json::Int(v));
                format!("DECL_CONSTANT {}{}", name, ctr_int(v))
            }
            Decl::ConstantStr { name, value } => {
                let v = match crate::scan::literal(value) {
                    Some(s) => s,
                    None => eval.config.string(value).unwrap_or_else(|| panic!("constant string {}", value)).to_string(),
                };
                set_constant(&mut constants, name, Json::Str(v.clone()));
                format!("DECL_CONSTANT_STR {} {}", name, v)
            }
            Decl::Enumeration { enumeration, name, expr } => {
                let v = eval.eval(expr);
                add_enum(&mut enumerations, enumeration, name, EnumValue::Int(v));
                format!("DECL_ENUMERATION {} {}{}", enumeration, name, ctr_int(v))
            }
            Decl::EnumerationRange { enumeration, name, expr, count } => {
                let (v, n) = (eval.eval(expr), eval.eval(count));
                add_enum(&mut enumerations, enumeration, name, EnumValue::Range(v, n));
                format!("DECL_ENUMERATION_RANGE {} {}{}{}", enumeration, name, ctr_int(v), ctr_int(n))
            }
            Decl::InitialPins(value) => {
                let pins = eval.config.string(value).unwrap_or_else(|| panic!("initial pins {}", value)).to_string();
                if !pins.is_empty() {
                    initial_pins = pins.split(',').map(|p| p.trim().to_string()).collect();
                    set_constant(&mut constants, "INITIAL_PINS", Json::Str(initial_pins.join(",")));
                }
                format!("DECL_INITIAL_PINS \"{}\"", pins)
            }
        };
        txt += &line;
        txt.push('\n');
    }

    // create_message_ids
    let mut encoded_msgid = msg_to_encid.iter().map(|(_, v)| *v).max().unwrap();
    let mlist: Vec<String> = commands.keys.iter().cloned().chain(encoders.iter().map(|(_, m)| m.clone())).collect();
    for msgname in &mlist {
        let msg = messages_by_name.get(msgname).cloned().unwrap_or_else(|| msgname.clone());
        if msg_to_encid.get(&msg).is_none() {
            encoded_msgid += 1;
            msg_to_encid.insert(&msg, encoded_msgid);
        }
    }
    if encoded_msgid >= 1 << 14 {
        panic!("Too many message ids");
    }

    // the data dictionary
    let mut data: BTreeMap<String, Json> = BTreeMap::new();
    let mut enums_json: BTreeMap<String, Json> = BTreeMap::new();
    for (i, s) in static_strings.iter().enumerate() {
        add_enum(&mut enumerations, "static_string_id", s, EnumValue::Int((i + STATIC_STRING_MIN) as i64));
    }
    for (e, enums) in enumerations.iter() {
        let mut m = BTreeMap::new();
        for (name, v) in enums.iter() {
            m.insert(
                name.clone(),
                match v {
                    EnumValue::Int(i) => Json::Int(*i),
                    EnumValue::Range(a, b) => Json::List(vec![Json::Int(*a), Json::Int(*b)]),
                },
            );
        }
        enums_json.insert(e.clone(), Json::Dict(m));
    }
    data.insert("enumerations".into(), Json::Dict(enums_json));
    let mut config = BTreeMap::new();
    for (k, v) in constants.iter() {
        config.insert(k.clone(), match v {
            Json::Int(i) => Json::Int(*i),
            Json::Str(s) => Json::Str(s.clone()),
            _ => unreachable!(),
        });
    }
    data.insert("config".into(), Json::Dict(config));
    let msgid = |msg: &str| convert_encoded_msgid(*msg_to_encid.get(msg).unwrap());
    let command_ids: Vec<i64> = messages_by_name.iter().filter(|(n, _)| commands.get(n).is_some()).map(|(_, m)| msgid(m)).collect();
    let response_ids: Vec<i64> = messages_by_name.iter().filter(|(n, _)| commands.get(n).is_none()).map(|(_, m)| msgid(m)).collect();
    let (mut cmds, mut resps, mut outs) = (BTreeMap::new(), BTreeMap::new(), BTreeMap::new());
    for (msg, _) in msg_to_encid.iter() {
        let id = msgid(msg);
        if command_ids.contains(&id) {
            cmds.insert(msg.clone(), Json::Int(id));
        } else if response_ids.contains(&id) {
            resps.insert(msg.clone(), Json::Int(id));
        } else {
            outs.insert(msg.clone(), Json::Int(id));
        }
    }
    data.insert("commands".into(), Json::Dict(cmds));
    data.insert("responses".into(), Json::Dict(resps));
    if !outs.is_empty() {
        data.insert("output".into(), Json::Dict(outs));
    }
    data.insert("version".into(), Json::Str(versions.version.clone()));
    data.insert("build_versions".into(), Json::Str(versions.build_versions.clone()));
    data.insert("app".into(), Json::Str("Kalico".into()));
    data.insert("license".into(), Json::Str("GNU GPLv3".into()));
    data.insert("kconfig".into(), Json::Str(kconfig.to_string()));
    let dict = crate::json::dumps(&Json::Dict(data));

    // the generated code
    let mut code = String::from("// Generated by build/commands.rs (scripts/buildcommands.py). Do not edit.\n\n");
    let mut param_ids: Vec<Vec<&'static str>> = vec![];
    let mut params_ref = |types: &[(&'static str, usize)]| -> String {
        let names: Vec<&'static str> = types.iter().map(|t| t.0).collect();
        if names.is_empty() {
            return "&[]".to_string();
        }
        let id = match param_ids.iter().position(|p| *p == names) {
            Some(i) => i,
            None => {
                param_ids.push(names);
                param_ids.len() - 1
            }
        };
        format!("&COMMAND_PARAMETERS{}", id)
    };
    // encoders and outputs (generate_responses_code)
    let mut encoder_defs = vec![];
    let mut encoder_lookup = vec![];
    let mut output_lookup = vec![];
    let mut done: Vec<i64> = vec![];
    for (msgname, msg) in &encoders {
        let enc = *msg_to_encid.get(msg).unwrap();
        if done.contains(&enc) {
            continue;
        }
        done.push(enc);
        let types = if msgname.is_none() { lookup_output_params(msg) } else { lookup_params(msg) };
        let msgid_size = if enc >= 0x80 { 2 } else { 1 };
        let max_size = MESSAGE_MAX.min(MESSAGE_MIN + msgid_size + types.iter().map(|t| t.1).sum::<usize>());
        let idx = encoder_defs.len();
        encoder_defs.push(format!(
            "    // {}{}\n    CommandEncoder {{ encoded_msgid: {}, max_size: {}, num_params: {}, param_types: {} }},\n",
            if msgname.is_none() { "Output: " } else { "" },
            msg,
            enc,
            max_size,
            types.len(),
            params_ref(&types)
        ));
        let entry = format!("    if streq(_s, {:?}) {{\n        return {};\n    }}\n", msg, idx);
        if msgname.is_none() {
            output_lookup.push(entry);
        } else {
            encoder_lookup.push(entry);
        }
    }
    // commands (generate_commands_code)
    let mut cmd_by_encid: BTreeMap<i64, (String, String, String)> = BTreeMap::new();
    for (msgname, (func, flags)) in commands.iter() {
        let msg = messages_by_name.get(msgname).unwrap().clone();
        cmd_by_encid.insert(*msg_to_encid.get(&msg).unwrap(), (func.clone(), flags.clone(), msg));
    }
    let max_cmd_encid = *cmd_by_encid.keys().max().unwrap();
    let mut index = vec![];
    for enc in 0..=max_cmd_encid {
        let Some((func, flags, msg)) = cmd_by_encid.get(&enc) else {
            index.push("    CommandParser::EMPTY,\n".to_string());
            continue;
        };
        let types = lookup_params(msg);
        let num_args = types.len() + types.iter().filter(|t| t.0 == "PT_progmem_buffer" || t.0 == "PT_buffer").count();
        index.push(format!(
            "    // {}\n    CommandParser {{ encoded_msgid: {}, num_args: {}, flags: {}, num_params: {}, param_types: {}, func: Some({}) }},\n",
            msg,
            enc,
            num_args,
            flags,
            types.len(),
            params_ref(&types),
            func
        ));
    }
    for (i, p) in param_ids.iter().enumerate() {
        code += &format!("static COMMAND_PARAMETERS{}: [u8; {}] = [{}];\n", i, p.len(), p.join(", "));
    }
    code += &format!("\npub static ENCODERS: [CommandEncoder; {}] = [\n{}];\n", encoder_defs.len(), encoder_defs.concat());
    code += &format!(
        "\npub const fn ctr_lookup_encoder(_s: &str) -> usize {{\n{}    panic!(\"undeclared message\")\n}}\n",
        encoder_lookup.concat()
    );
    code += &format!(
        "\npub const fn ctr_lookup_output(_s: &str) -> usize {{\n{}    panic!(\"undeclared output\")\n}}\n",
        output_lookup.concat()
    );
    let mut statics = String::new();
    for (i, s) in static_strings.iter().enumerate() {
        statics += &format!("    if streq(_s, {:?}) {{\n        return {};\n    }}\n", s, i + STATIC_STRING_MIN);
    }
    code += &format!("\npub const fn ctr_lookup_static_string(_s: &str) -> u8 {{\n{}    0xff\n}}\n", statics);
    code += &format!("\npub static COMMAND_INDEX: [CommandParser; {}] = [\n{}];\n", index.len(), index.concat());
    // call lists (HandleCallList)
    for (name, funcs) in call_lists.iter() {
        let calls: Vec<String> = funcs
            .iter()
            .map(|f| {
                if name == "ctr_run_taskfuncs" {
                    format!("    crate::board::irq_poll()?;\n    {}()?;\n", f)
                } else if name == "ctr_run_shutdownfuncs" {
                    format!("    {}();\n", f)
                } else {
                    format!("    {}()?;\n", f)
                }
            })
            .collect();
        let tail = if name == "ctr_run_taskfuncs" { "    crate::board::irq_poll()?;\n" } else { "" };
        if name == "ctr_run_shutdownfuncs" {
            code += &format!("\npub fn {}() {{\n{}}}\n", name, calls.concat());
        } else {
            code += &format!("\npub fn {}() -> R {{\n{}{}    Ok(())\n}}\n", name, calls.concat(), tail);
        }
    }
    // initial pins (HandleInitialPins.map_pins)
    let mut pins_code = vec![];
    if !initial_pins.is_empty() {
        let pinmap = enumerations.get("pin");
        for p in &initial_pins {
            let (flag, name) = match p.strip_prefix('!') {
                Some(n) => ("0", n.trim()),
                None => ("IP_OUT_HIGH", p.as_str()),
            };
            // msgproto fill_enumerations: ranges expand to name prefix + index
            let pin = pinmap.and_then(|m| lookup_pin(m, name)).unwrap_or_else(|| panic!("Unknown initial pin '{}'", name));
            pins_code.push(format!("    InitialPin {{ pin: {}, flags: {} }}, // {}\n", pin, flag, p));
        }
    }
    code += &format!("\npub static INITIAL_PINS: [InitialPin; {}] = [\n{}];\n", pins_code.len(), pins_code.concat());
    // the identify data (HandleIdentify)
    let z = {
        use std::io::Write;
        let mut e = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::new(9));
        e.write_all(dict.as_bytes()).unwrap();
        e.finish().unwrap()
    };
    let bytes: Vec<String> = z.iter().map(|b| format!("0x{:02x}", b)).collect();
    code += &format!(
        "\n// Identify size = {} ({} uncompressed)\npub static COMMAND_IDENTIFY_DATA: [u8; {}] = [{}];\n",
        z.len(),
        dict.len(),
        z.len(),
        bytes.join(", ")
    );
    code += &format!("\n// version: {}\n// build_versions: {}\n", versions.version, versions.build_versions);
    Output { txt, dict, code }
}

/// msgproto MessageParser.fill_enumerations, then a lookup of `name`
fn lookup_pin(enums: &Ordered<EnumValue>, name: &str) -> Option<i64> {
    let mut all: BTreeMap<String, i64> = BTreeMap::new();
    for (key, v) in enums.iter() {
        match v {
            EnumValue::Int(i) => {
                all.insert(key.clone(), *i);
            }
            EnumValue::Range(start, count) => {
                let root = key.trim_end_matches(|c: char| c.is_ascii_digit());
                let first: i64 = if root.len() != key.len() { key[root.len()..].parse().unwrap() } else { 0 };
                for i in 0..*count {
                    all.insert(format!("{}{}", root, first + i), start + i);
                }
            }
        }
    }
    all.get(name).copied()
}
