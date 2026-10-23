// python json.dumps(data, separators=(",", ":"), sort_keys=True)
use std::collections::BTreeMap;

pub enum Json {
    Int(i64),
    Str(String),
    List(Vec<Json>),
    Dict(BTreeMap<String, Json>),
}

fn string(out: &mut String, s: &str) {
    // ensure_ascii: non-ASCII as \uXXXX (UTF-16 surrogate pairs)
    out.push('"');
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 || (c as u32) > 0x7e => {
                let mut buf = [0u16; 2];
                for u in c.encode_utf16(&mut buf) {
                    out.push_str(&format!("\\u{:04x}", u));
                }
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

pub fn dumps(v: &Json) -> String {
    let mut out = String::new();
    write(&mut out, v);
    out
}

fn write(out: &mut String, v: &Json) {
    match v {
        Json::Int(i) => out.push_str(&i.to_string()),
        Json::Str(s) => string(out, s),
        Json::List(l) => {
            out.push('[');
            for (i, x) in l.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write(out, x);
            }
            out.push(']');
        }
        Json::Dict(d) => {
            out.push('{');
            for (i, (k, x)) in d.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                string(out, k);
                out.push(':');
                write(out, x);
            }
            out.push('}');
        }
    }
}
