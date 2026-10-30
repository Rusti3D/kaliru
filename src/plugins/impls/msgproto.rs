// klippy/msgproto.py: MessageParser. Its getters run as Kalico's own code
// (tools/mirror_verbatim.txt) on these attributes.
use super::mcu::{data_from_py, params_from_py, params_to_py};
use super::*;
use crate::msgproto::{MessageFormat, MessageParser, MsgFormat, OutputFormat, Params};
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
    // the formats: encode(params) / encode_by_name(**params) -> [byte],
    // parse(s, pos) -> (params, pos), format_params(params) -> str
    fn parsed(r: crate::msgproto::Result<(Params, usize)>) -> KResult<PyRef> {
        let (params, pos) = r?;
        Ok(py_tuple(vec![params_to_py(&params)?, py_int(pos as i64)]))
    }
    r.method("klippy.msgproto.MessageFormat.encode", |a| {
        let f = a.this::<MessageFormat>()?;
        f.encode(&data_from_py(a.arg(0))?)?.to_py()
    });
    r.method("klippy.msgproto.MessageFormat.encode_by_name", |a| {
        let f = a.this::<MessageFormat>()?;
        f.encode_by_name(&params_from_py(a.arg(0))?)?.to_py()
    });
    r.method("klippy.msgproto.MessageFormat.parse", |a| parsed(a.this::<MessageFormat>()?.parse(&bytes_of(a.arg(0))?, a.arg(1).as_i64()? as usize)));
    r.method("klippy.msgproto.MessageFormat.format_params", |a| {
        Ok(py_str(&a.this::<MessageFormat>()?.format_params(&params_from_py(a.arg(0))?)?))
    });
    r.method("klippy.msgproto.OutputFormat.parse", |a| parsed(a.this::<OutputFormat>()?.parse(&bytes_of(a.arg(0))?, a.arg(1).as_i64()? as usize)));
    r.method("klippy.msgproto.UnknownFormat.parse", |a| {
        parsed(crate::msgproto::UnknownFormat.parse(&bytes_of(a.arg(0))?, a.arg(1).as_i64()? as usize))
    });
    // create_command(msg): "" for an empty message, as python returns
    r.method("klippy.msgproto.MessageParser.create_command", |a| {
        let msg = a.string(0)?;
        if msg.split_whitespace().next().is_none() {
            return Ok(py_str(""));
        }
        this(a)?.create_command(&msg)?.to_py()
    });

    // ---- the param types: MessageTypes' PT_* objects, or the Enumeration
    {
        use crate::msgproto::{BaseType, Enumeration, ParamType};
        fn base_to_py(b: BaseType) -> KResult<PyRef> {
            let fmt = match b {
                BaseType::Uint32 => "%u",
                BaseType::Int32 => "%i",
                BaseType::Uint16 => "%hu",
                BaseType::Int16 => "%hi",
                BaseType::Byte => "%c",
                BaseType::String => "%s",
                BaseType::ProgmemBuffer => "%.*s",
                BaseType::Buffer => "%*s",
            };
            lookup("klippy.msgproto", "MessageTypes")?.call_method("__getitem__", &[&py_str(fmt)])
        }
        fn pt_to_py(p: &ParamType) -> KResult<PyRef> {
            match p {
                ParamType::Base(b) => base_to_py(*b),
                ParamType::Enum(e) => objects::wrap(e.clone()),
            }
        }
        r.attr("klippy.msgproto.Enumeration.pt", |o| base_to_py(objects::unwrap_as::<Enumeration>(o)?.pt), None);
        r.attr("klippy.msgproto.Enumeration.max_length", |o| Ok(py_int(objects::unwrap_as::<Enumeration>(o)?.pt.max_length() as i64)), None);
        r.attr("klippy.msgproto.Enumeration.enums", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in &objects::unwrap_as::<Enumeration>(o)?.table.enums {
                d.call_method("__setitem__", &[&py_str(k), &py_int(*v)])?;
            }
            Ok(d)
        }, None);
        r.attr("klippy.msgproto.Enumeration.reverse_enums", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in &objects::unwrap_as::<Enumeration>(o)?.table.enums {
                d.call_method("__setitem__", &[&py_int(*v), &py_str(k)])?;
            }
            Ok(d)
        }, None);
        // [(name, type)] / {name: type} / [type]
        r.attr("klippy.msgproto.MessageFormat.param_names", |o| {
            let items: KResult<Vec<PyRef>> =
                objects::unwrap_as::<MessageFormat>(o)?.param_names.iter().map(|(n, p)| Ok(py_tuple(vec![py_str(n), pt_to_py(p)?]))).collect();
            Ok(py_list(items?))
        }, None);
        r.attr("klippy.msgproto.MessageFormat.param_types", |o| {
            let items: KResult<Vec<PyRef>> = objects::unwrap_as::<MessageFormat>(o)?.param_names.iter().map(|(_, p)| pt_to_py(p)).collect();
            Ok(py_list(items?))
        }, None);
        r.attr("klippy.msgproto.MessageFormat.name_to_type", |o| {
            let m = objects::unwrap_as::<MessageFormat>(o)?;
            let d = lookup("builtins", "dict")?.call(&[])?;
            // (in the format's parameter order, as python builds it)
            for (n, p) in &m.param_names {
                d.call_method("__setitem__", &[&py_str(n), &pt_to_py(p)?])?;
            }
            Ok(d)
        }, None);
        r.attr("klippy.msgproto.OutputFormat.param_types", |o| {
            let items: KResult<Vec<PyRef>> = objects::unwrap_as::<OutputFormat>(o)?.param_types.iter().map(|b| base_to_py(*b)).collect();
            Ok(py_list(items?))
        }, None);
    }

    // unknown: UnknownFormat(); msgid_parser: PT_int32()
    r.attr("klippy.msgproto.MessageParser.unknown", |o| {
        mp(o)?;
        objects::wrap(Rc::new(crate::msgproto::UnknownFormat))
    }, None);
    r.attr("klippy.msgproto.MessageParser.msgid_parser", |o| {
        mp(o)?;
        lookup("klippy.msgproto", "PT_int32")?.call(&[])
    }, None);
    // the clock queries' command queue (allocated at connect)
    r.attr("klippy.clocksync.ClockSync.cmd_queue", |o| {
        match objects::unwrap_as::<crate::clocksync::ClockSync>(o)?.cmd_queue.borrow().clone() {
            Some(cq) => super::mcu::cq_to_py(cq),
            None => Ok(py_none()),
        }
    }, None);
}
