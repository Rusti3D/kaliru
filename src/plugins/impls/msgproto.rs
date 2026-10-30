// klippy/msgproto.py: MessageParser. Its getters run as Kalico's own code
// (tools/mirror_verbatim.txt) on these attributes.
use super::mcu::{params_from_py, params_to_py};
use super::*;
use crate::msgproto::{MessageFormat, MessageParser, MsgFormat, OutputFormat};
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::Args;

fn mp(o: &PyRef) -> KResult<Rc<MessageParser>> {
    objects::unwrap_as::<MessageParser>(o)
}

fn this(a: &Args) -> KResult<Rc<MessageParser>> {
    mp(a.slf.as_ref().expect("method"))
}

/// identify data values (json) as python's json.loads gives them
fn json_to_py(v: &serde_json::Value) -> KResult<PyRef> {
    lookup("json", "loads")?.call(&[&py_str(&v.to_string())])
}

/// a message python passes (bytes, bytearray or a list of ints)
fn bytes_of(o: &PyRef) -> KResult<Vec<u8>> {
    FromPy::from_py(&lookup("builtins", "list")?.call(&[o])?)
}

fn bytes_to_py(b: &[u8]) -> KResult<PyRef> {
    lookup("builtins", "bytes")?.call(&[&b.to_vec().to_py()?])
}

fn format_to_py(f: &MsgFormat) -> KResult<PyRef> {
    match f {
        MsgFormat::Message(m) => objects::wrap(m.clone()),
        MsgFormat::Output(o) => objects::wrap(o.clone()),
    }
}

pub fn register(r: &mut Registry) {
    objects::register_class::<MessageFormat>("klippy.msgproto", "MessageFormat");
    objects::register_class::<OutputFormat>("klippy.msgproto", "OutputFormat");
    r.attr("klippy.msgproto.MessageParser.config", |o| json_to_py(&serde_json::Value::Object(mp(o)?.config.clone())), None);
    r.attr("klippy.msgproto.MessageParser.kconfig", |o| json_to_py(&mp(o)?.kconfig), None);
    r.attr("klippy.msgproto.MessageParser.app", |o| json_to_py(&mp(o)?.app), None);
    r.attr("klippy.msgproto.MessageParser.version", |o| json_to_py(&mp(o)?.version), None);
    r.attr("klippy.msgproto.MessageParser.build_versions", |o| json_to_py(&mp(o)?.build_versions), None);
    r.attr("klippy.msgproto.MessageParser.raw_identify_data", |o| bytes_to_py(&mp(o)?.raw_identify_data), None);
    // {name: {enum: value}}
    r.attr("klippy.msgproto.MessageParser.enumerations", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, table) in mp(o)?.enumerations.iter() {
            let enums = lookup("builtins", "dict")?.call(&[])?;
            for (e, v) in &table.enums {
                enums.call_method("__setitem__", &[&py_str(e), &py_int(*v)])?;
            }
            d.call_method("__setitem__", &[&py_str(name), &enums])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.msgproto.MessageParser.msgid_by_format", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (f, id) in mp(o)?.msgid_by_format.iter() {
            d.call_method("__setitem__", &[&py_str(f), &py_int(*id)])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.msgproto.MessageParser.messages_by_name", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, f) in mp(o)?.messages_by_name.iter() {
            d.call_method("__setitem__", &[&py_str(name), &objects::wrap(f.clone())?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.msgproto.MessageParser.messages_by_id", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (id, f) in mp(o)?.messages_by_id.iter() {
            d.call_method("__setitem__", &[&py_int(*id), &format_to_py(f)?])?;
        }
        Ok(d)
    }, None);
    r.method("klippy.msgproto.MessageParser.check_packet", |a| Ok(py_int(this(a)?.check_packet(&bytes_of(a.arg(0))?) as i64)));
    r.method("klippy.msgproto.MessageParser.dump", |a| this(a)?.dump(&bytes_of(a.arg(0))?)?.to_py());
    r.method("klippy.msgproto.MessageParser.format_params", |a| Ok(py_str(&this(a)?.format_params(&params_from_py(a.arg(0))?)?)));
    r.method("klippy.msgproto.MessageParser.parse", |a| params_to_py(&this(a)?.parse(&bytes_of(a.arg(0))?)?));
    r.method("klippy.msgproto.MessageParser.lookup_command", |a| objects::wrap(this(a)?.lookup_command(&a.string(0)?)?));
    // create_command(msg): "" for an empty message, as python returns
    r.method("klippy.msgproto.MessageParser.create_command", |a| {
        let msg = a.string(0)?;
        if msg.split_whitespace().next().is_none() {
            return Ok(py_str(""));
        }
        this(a)?.create_command(&msg)?.to_py()
    });
}
