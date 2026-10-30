// klippy/mcu.py: MCU, CommandWrapper, CommandQueryWrapper, MCU_trsync,
// TriggerDispatch. Probes with their own mcu (beacon, cartographer) look
// up commands, register responses and run trsyncs here; a command queue is
// a chelper object (prelude.py CData).
use super::*;
use crate::chelper::serialqueue::CommandQueue;
use crate::chelper::trdispatch::Trdispatch;
use crate::mcu::{CommandQueryWrapper, CommandWrapper, Mcu, McuAdc, McuDigitalOut, McuEndstop, McuPwm, McuTrsync, TriggerDispatch};
use crate::msgproto::{Params, Val};
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::gil::Gil;
use crate::stepper::McuStepper;
use crate::{cell_attr, method, ro_attr};
use std::collections::HashMap;

// --------------------------------------------------------- command queues

/// A command queue as python holds it (one per queue, kept for `is`)
pub struct CqHandle(pub CommandQueue);

static CQS: Gil<RefCell<HashMap<usize, Rc<CqHandle>>>> = Gil::new(|| RefCell::new(HashMap::new()));

pub fn cq_to_py(cq: CommandQueue) -> KResult<PyRef> {
    let key = std::sync::Arc::as_ptr(&cq) as usize;
    let h = CQS.with(|m| m.borrow_mut().entry(key).or_insert_with(|| Rc::new(CqHandle(cq.clone()))).clone());
    objects::wrap(h)
}

/// a command queue argument (None: the default queue)
pub fn cq_from_py(o: &PyRef) -> KResult<Option<CommandQueue>> {
    if o.is_none() {
        return Ok(None);
    }
    Ok(Some(objects::unwrap_as::<CqHandle>(o)?.0.clone()))
}

// ----------------------------------------------------- messages and params

/// command data: ints, bytes / bytearray (buffers), str
pub fn data_from_py(o: &PyRef) -> KResult<Vec<Val>> {
    let items: Vec<PyRef> = FromPy::from_py(o)?;
    items
        .iter()
        .map(|v| {
            Ok(match type_name(v)?.as_str() {
                "bytes" | "bytearray" => {
                    let b: Vec<i64> = FromPy::from_py(v)?;
                    Val::Bytes(b.into_iter().map(|x| x as u8).collect())
                }
                "str" => Val::Str(v.as_string()?),
                _ => Val::Int(v.as_i64()?),
            })
        })
        .collect()
}

pub fn val_to_py(v: &Val) -> KResult<PyRef> {
    Ok(match v {
        Val::Int(i) => py_int(*i),
        Val::Float(f) => py_float(*f),
        Val::Str(s) => py_str(s),
        Val::Bytes(b) => lookup("builtins", "bytes")?.call(&[&py_list(b.iter().map(|x| py_int(*x as i64)).collect())])?,
    })
}

/// a response's params as python's dict
pub fn params_to_py(p: &Params) -> KResult<PyRef> {
    let d = lookup("builtins", "dict")?.call(&[])?;
    for (k, v) in p {
        d.call_method("__setitem__", &[&py_str(k), &val_to_py(v)?])?;
    }
    Ok(d)
}

pub fn params_from_py(o: &PyRef) -> KResult<Params> {
    let items: Vec<(String, PyRef)> = FromPy::from_py(&o.call_method("items", &[])?)?;
    items
        .into_iter()
        .map(|(k, v)| {
            let val = match type_name(&v)?.as_str() {
                "float" => Val::Float(v.as_f64()?),
                "str" => Val::Str(v.as_string()?),
                "bytes" | "bytearray" => {
                    let b: Vec<i64> = FromPy::from_py(&v)?;
                    Val::Bytes(b.into_iter().map(|x| x as u8).collect())
                }
                _ => Val::Int(v.as_i64()?),
            };
            Ok((k, val))
        })
        .collect()
}

/// a python response callback (None: unregister)
fn response_handler(cb: &PyRef) -> Option<crate::serialhdl::ResponseHandler> {
    if cb.is_none() {
        return None;
    }
    let cb = cb.clone();
    Some(Rc::new(move |p: &Params| {
        cb.call(&[&params_to_py(p)?])?;
        Ok(())
    }))
}

fn json_to_py(v: &serde_json::Value) -> PyRef {
    match v {
        serde_json::Value::String(s) => py_str(s),
        serde_json::Value::Bool(b) => py_bool(*b),
        serde_json::Value::Number(n) => match n.as_i64() {
            Some(i) => py_int(i),
            None => py_float(n.as_f64().unwrap_or(0.0)),
        },
        serde_json::Value::Null => py_none(),
        other => py_str(&other.to_string()),
    }
}

fn mc(o: &PyRef) -> KResult<Rc<Mcu>> {
    objects::unwrap_as::<Mcu>(o)
}

fn ts(o: &PyRef) -> KResult<Rc<McuTrsync>> {
    objects::unwrap_as::<McuTrsync>(o)
}

fn td(o: &PyRef) -> KResult<Rc<TriggerDispatch>> {
    objects::unwrap_as::<TriggerDispatch>(o)
}

fn steppers_to_py(steppers: Vec<Rc<McuStepper>>) -> KResult<PyRef> {
    Ok(py_list(steppers.into_iter().map(objects::wrap).collect::<KResult<Vec<_>>>()?))
}

fn opt_cmd_to_py<T: 'static + Clone>(c: &RefCell<Option<T>>) -> KResult<PyRef> {
    match c.borrow().clone() {
        Some(c) => objects::wrap(Rc::new(c)),
        None => Ok(py_none()),
    }
}

/// conn_helper.get_serial(), conn_helper.get_mcu().is_fileoutput()
fn conn_helper_of(o: &PyRef) -> KResult<(Rc<crate::serialhdl::SerialReader>, bool)> {
    let serial = objects::unwrap_as::<crate::serialhdl::SerialReader>(&o.call_method("get_serial", &[])?)?;
    let fileoutput = o.call_method("get_mcu", &[])?.call_method("is_fileoutput", &[])?.is_true()?;
    Ok((serial, fileoutput))
}

pub fn register(r: &mut Registry) {
    objects::register_class::<CqHandle>("_k", "CData");
    objects::register_class::<Trdispatch>("_k", "CData");
    objects::register_class::<McuTrsync>("klippy.mcu", "MCU_trsync");

    // ---------------------------------------------------- CommandWrapper
    // (conn_helper, msgformat, cmd_queue=None)
    r.ctor("klippy.mcu.CommandWrapper.__init__", |a| {
        let (serial, fileoutput) = conn_helper_of(a.arg(0))?;
        let c: Rc<dyn Any> = Rc::new(CommandWrapper::new(&serial, &a.string(1)?, cq_from_py(a.arg(2))?, fileoutput)?);
        Ok(c)
    });
    // send(data=(), minclock=0, reqclock=0)
    r.method("klippy.mcu.CommandWrapper.send", |a| {
        a.this::<CommandWrapper>()?.send(&data_from_py(a.arg(0))?, a.arg(1).as_i64()? as u64, a.arg(2).as_i64()? as u64)?;
        Ok(py_none())
    });
    r.method("klippy.mcu.CommandWrapper.send_wait_ack", |a| {
        a.this::<CommandWrapper>()?.send_wait_ack(&data_from_py(a.arg(0))?, a.arg(1).as_i64()? as u64, a.arg(2).as_i64()? as u64)?;
        Ok(py_none())
    });
    method!(r, "klippy.mcu.CommandWrapper", CommandWrapper, get_command_tag(c) => c.get_command_tag());
    // self._error: the printer's command_error
    r.attr("klippy.mcu.CommandWrapper._error", |_o| lookup("klippy.gcode", "CommandError"), None);
    r.attr("klippy.mcu.CommandQueryWrapper._error", |_o| lookup("klippy.gcode", "CommandError"), None);

    // ----------------------------------------------------- DummyResponse
    // (serial, name, oid=None)
    r.ctor("klippy.mcu.DummyResponse.__init__", |a| {
        let serial = objects::unwrap_as::<crate::serialhdl::SerialReader>(a.arg(0))?;
        let oid: Option<i64> = FromPy::from_py(a.arg(2))?;
        let d: Rc<dyn Any> = Rc::new(crate::mcu::DummyResponse::new(&serial, &a.string(1)?, oid)?);
        Ok(d)
    });
    // get_response(cmds, cmd_queue, minclock=0, reqclock=0, retry=True)
    r.method("klippy.mcu.DummyResponse.get_response", |a| params_to_py(&a.this::<crate::mcu::DummyResponse>()?.get_response()));
    r.attr("klippy.mcu.DummyResponse._response", |o| params_to_py(&objects::unwrap_as::<crate::mcu::DummyResponse>(o)?.response), None);
    ro_attr!(r, "klippy.mcu.CommandWrapper._msgtag", CommandWrapper, |c| c.msgtag);
    ro_attr!(r, "klippy.mcu.CommandWrapper._serial", CommandWrapper, |c| c.serial.clone());
    r.attr("klippy.mcu.CommandWrapper._cmd_queue", |o| cq_to_py(objects::unwrap_as::<CommandWrapper>(o)?.cmd_queue.clone()), None);

    // ----------------------------------------------- CommandQueryWrapper
    // (conn_helper, msgformat, respformat, oid=None, cmd_queue=None, is_async=False)
    r.ctor("klippy.mcu.CommandQueryWrapper.__init__", |a| {
        let (serial, fileoutput) = conn_helper_of(a.arg(0))?;
        let oid: Option<i64> = FromPy::from_py(a.arg(3))?;
        let c: Rc<dyn Any> =
            Rc::new(CommandQueryWrapper::new(&serial, &a.string(1)?, &a.string(2)?, oid, cq_from_py(a.arg(4))?, a.bool(5)?, fileoutput)?);
        Ok(c)
    });
    // send(data=(), minclock=0, reqclock=0, retry=True)
    r.method("klippy.mcu.CommandQueryWrapper.send", |a| {
        let p = a.this::<CommandQueryWrapper>()?.send(
            &data_from_py(a.arg(0))?,
            a.arg(1).as_i64()? as u64,
            a.arg(2).as_i64()? as u64,
            a.bool(3)?,
        )?;
        params_to_py(&p)
    });
    // send_with_preface(preface_cmd, preface_data=(), data=(), minclock=0, reqclock=0, retry=True)
    r.method("klippy.mcu.CommandQueryWrapper.send_with_preface", |a| {
        let preface = objects::unwrap_as::<CommandWrapper>(a.arg(0))?;
        let p = a.this::<CommandQueryWrapper>()?.send_with_preface(
            &preface,
            &data_from_py(a.arg(1))?,
            &data_from_py(a.arg(2))?,
            a.arg(3).as_i64()? as u64,
            a.arg(4).as_i64()? as u64,
            a.bool(5)?,
        )?;
        params_to_py(&p)
    });
    ro_attr!(r, "klippy.mcu.CommandQueryWrapper._oid", CommandQueryWrapper, |c| c.oid);
    ro_attr!(r, "klippy.mcu.CommandQueryWrapper._response", CommandQueryWrapper, |c| c.response.clone());
    ro_attr!(r, "klippy.mcu.CommandQueryWrapper._serial", CommandQueryWrapper, |c| c.serial.clone());
    r.attr("klippy.mcu.CommandQueryWrapper._cmd_queue", |o| cq_to_py(objects::unwrap_as::<CommandQueryWrapper>(o)?.cmd_queue.clone()), None);

    // --------------------------------------------------------------- MCU
    method!(r, "klippy.mcu.MCU", Mcu, get_name(m) => m.get_name().to_string());
    method!(r, "klippy.mcu.MCU", Mcu, get_printer(m) => m.get_printer());
    method!(r, "klippy.mcu.MCU", Mcu, is_fileoutput(m) => m.is_fileoutput());
    method!(r, "klippy.mcu.MCU", Mcu, create_oid(m) => m.create_oid());
    method!(r, "klippy.mcu.MCU", Mcu, seconds_to_clock(m, time: f64) => m.seconds_to_clock(time));
    method!(r, "klippy.mcu.MCU", Mcu, min_schedule_time(m) => m.min_schedule_time());
    method!(r, "klippy.mcu.MCU", Mcu, max_nominal_duration(m) => m.max_nominal_duration());
    method!(r, "klippy.mcu.MCU", Mcu, get_constant_float(m, name: String) => m.get_constant_float(&name));
    method!(r, "klippy.mcu.MCU", Mcu, print_time_to_clock(m, print_time: f64) => m.print_time_to_clock(print_time));
    method!(r, "klippy.mcu.MCU", Mcu, clock_to_print_time(m, clock: f64) => m.clock_to_print_time(clock));
    method!(r, "klippy.mcu.MCU", Mcu, estimated_print_time(m, eventtime: f64) => m.estimated_print_time(eventtime));
    method!(r, "klippy.mcu.MCU", Mcu, clock32_to_clock64(m, clock32: i64) => m.clock32_to_clock64(clock32));
    method!(r, "klippy.mcu.MCU", Mcu, calibrate_clock(m, print_time: f64, eventtime: f64) => m.calibrate_clock(print_time, eventtime));
    method!(r, "klippy.mcu.MCU", Mcu, handle_non_critical_disconnect(m) => m.handle_non_critical_disconnect());
    method!(r, "klippy.mcu.MCU", Mcu, schedule_non_critical_reconnect(m) => m.schedule_non_critical_reconnect());
    method!(r, "klippy.mcu.MCU", Mcu, non_critical_recon_event(m, eventtime: f64) => m.non_critical_recon_event(eventtime));
    method!(r, "klippy.mcu.MCU", Mcu, recon_mcu(m) => m.recon_mcu());
    method!(r, "klippy.mcu.MCU", Mcu, _abort_recon_attempt(m) => m.abort_recon_attempt());
    method!(r, "klippy.mcu.MCU", Mcu, _mcu_identify(m) => m.mcu_identify());
    method!(r, "klippy.mcu.MCU", Mcu, _identify_helpers(m) => m.identify_helpers());
    method!(r, "klippy.mcu.MCU", Mcu, get_non_critical_reconnect_event_name(m) => m.get_non_critical_reconnect_event_name().to_string());
    method!(r, "klippy.mcu.MCU", Mcu, get_non_critical_disconnect_event_name(m) => m.get_non_critical_disconnect_event_name().to_string());
    // setup_pin(pin_type, pin_params)
    r.method("klippy.mcu.MCU.setup_pin", |a| {
        let m = a.this::<Mcu>()?;
        let pin_type = a.string(0)?;
        let params = super::pins::pin_params_from_py(a.arg(1))?;
        let pin = crate::pins::PinChip::setup_pin(&*m, &pin_type, &params).map_err(KError::Pins)?;
        super::pins::pin_to_py(pin, &pin_type)
    });
    // register_config_callback(cb): cb()
    r.method("klippy.mcu.MCU.register_config_callback", |a| {
        let cb = a.arg(0).clone();
        a.this::<Mcu>()?.register_config_callback(Rc::new(move || {
            cb.call(&[])?;
            Ok(())
        }));
        Ok(py_none())
    });
    method!(r, "klippy.mcu.MCU", Mcu, add_config_cmd(m, cmd: String, is_init: bool, on_restart: bool) => m.add_config_cmd(&cmd, is_init, on_restart));
    method!(r, "klippy.mcu.MCU", Mcu, request_move_queue_slot(m) => m.request_move_queue_slot());
    method!(r, "klippy.mcu.MCU", Mcu, get_query_slot(m, oid: i64) => m.get_query_slot(oid));
    method!(r, "klippy.mcu.MCU", Mcu, dump_debug(m) => m.dump_debug());
    // lookup_command(msgformat, cq=None)
    r.method("klippy.mcu.MCU.lookup_command", |a| objects::wrap(Rc::new(a.this::<Mcu>()?.lookup_command(&a.string(0)?, cq_from_py(a.arg(1))?)?)));
    // lookup_query_command(msgformat, respformat, oid=None, cq=None, is_async=False)
    r.method("klippy.mcu.MCU.lookup_query_command", |a| {
        let oid: Option<i64> = FromPy::from_py(a.arg(2))?;
        let c = a.this::<Mcu>()?.lookup_query_command(&a.string(0)?, &a.string(1)?, oid, cq_from_py(a.arg(3))?, a.bool(4)?)?;
        objects::wrap(Rc::new(c))
    });
    r.method("klippy.mcu.MCU.try_lookup_command", |a| match a.this::<Mcu>()?.try_lookup_command(&a.string(0)?) {
        Some(c) => objects::wrap(Rc::new(c)),
        None => Ok(py_none()),
    });
    // register_response(cb, msg, oid=None)
    r.method("klippy.mcu.MCU.register_response", |a| {
        let oid: Option<i64> = FromPy::from_py(a.arg(2))?;
        a.this::<Mcu>()?.register_response(response_handler(a.arg(0)), &a.string(1)?, oid);
        Ok(py_none())
    });
    r.method("klippy.serialhdl.SerialReader.register_response", |a| {
        let oid: Option<i64> = FromPy::from_py(a.arg(2))?;
        a.this::<crate::serialhdl::SerialReader>()?.register_response(response_handler(a.arg(0)), &a.string(1)?, oid);
        Ok(py_none())
    });
    // self.danger_options = printer.lookup_object("danger_options")
    r.attr("klippy.mcu.MCU.danger_options", |o| mc(o)?.printer.lookup_object("danger_options")?.to_py(), None);
    r.method("klippy.mcu.MCU.alloc_command_queue", |a| cq_to_py(a.this::<Mcu>()?.alloc_command_queue()));
    // get_enumerations(): {name: {value_name: value}}
    r.method("klippy.mcu.MCU.get_enumerations", |a| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (name, table) in a.this::<Mcu>()?.get_enumerations() {
            let e = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in &table.enums {
                e.call_method("__setitem__", &[&py_str(k), &py_int(*v)])?;
            }
            d.call_method("__setitem__", &[&py_str(&name), &e])?;
        }
        Ok(d)
    });
    r.method("klippy.mcu.MCU.get_constants", |a| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (k, v) in a.this::<Mcu>()?.get_constants() {
            d.call_method("__setitem__", &[&py_str(&k), &json_to_py(&v)])?;
        }
        Ok(d)
    });
    // get_status(eventtime=None)
    r.method("klippy.mcu.MCU.get_status", |a| {
        let m = a.this::<Mcu>()?;
        match crate::printer::PrinterObject::get_status(&*m, 0.0) {
            Some(v) => Ok(from_pyval(&v)),
            None => Ok(py_none()),
        }
    });
    ro_attr!(r, "klippy.mcu.MCU._name", Mcu, |m| m.name.clone());
    ro_attr!(r, "klippy.mcu.MCU._printer", Mcu, |m| m.printer.clone());
    ro_attr!(r, "klippy.mcu.MCU._reactor", Mcu, |m| m.reactor.clone());
    ro_attr!(r, "klippy.mcu.MCU._serial", Mcu, |m| m.serial.clone());
    ro_attr!(r, "klippy.mcu.MCU._clocksync", Mcu, |m| m.clocksync.clone());
    ro_attr!(r, "klippy.mcu.MCU.gcode", Mcu, |m| m.gcode.clone());
    ro_attr!(r, "klippy.mcu.MCU._non_critical_disconnect_event_name", Mcu, |m| m.non_critical_disconnect_event_name.clone());
    ro_attr!(r, "klippy.mcu.MCU._non_critical_reconnect_event_name", Mcu, |m| m.non_critical_reconnect_event_name.clone());
    ro_attr!(r, "klippy.mcu.MCU.non_critical_recon_timer", Mcu, |m| m.non_critical_recon_timer.borrow().clone());
    cell_attr!(r, "klippy.mcu.MCU.is_non_critical", Mcu, is_non_critical);
    cell_attr!(r, "klippy.mcu.MCU.non_critical_disconnected", Mcu, non_critical_disconnected);
    cell_attr!(r, "klippy.mcu.MCU._reconnecting", Mcu, reconnecting);
    cell_attr!(r, "klippy.mcu.MCU.reconnect_interval", Mcu, reconnect_interval);
    r.method("klippy.mcu.get_printer_mcu", |a| {
        crate::mcu::get_printer_mcu(&*objects::unwrap_as::<Printer>(a.arg(0))?, &a.string(1)?)?.to_py()
    });
    r.method("klippy.mcu.add_printer_objects", |a| {
        let c = super::configfile::config_from_py(a.arg(0))?;
        crate::mcu::add_printer_objects(&c.get_printer(), &c)?;
        Ok(py_none())
    });

    // -------------------------------------------------------- MCU_trsync
    r.ctor("klippy.mcu.MCU_trsync.__init__", |a| {
        let t: Rc<dyn Any> = McuTrsync::new(&mc(a.arg(0))?, &objects::unwrap_as::<Trdispatch>(a.arg(1))?);
        Ok(t)
    });
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, get_mcu(t) => t.get_mcu());
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, get_oid(t) => t.get_oid());
    r.method("klippy.mcu.MCU_trsync.get_command_queue", |a| cq_to_py(a.this::<McuTrsync>()?.get_command_queue()));
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, add_stepper(t, stepper: Rc<McuStepper>) => t.add_stepper(&stepper));
    r.method("klippy.mcu.MCU_trsync.get_steppers", |a| steppers_to_py(a.this::<McuTrsync>()?.get_steppers()));
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, _build_config(t) => t.build_config());
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, _shutdown(t) => t.shutdown());
    r.method("klippy.mcu.MCU_trsync._handle_trsync_state", |a| {
        a.this::<McuTrsync>()?.handle_trsync_state(&params_from_py(a.arg(0))?)?;
        Ok(py_none())
    });
    // start(print_time, report_offset, trigger_completion, expire_timeout)
    r.method("klippy.mcu.MCU_trsync.start", |a| {
        let tc = objects::unwrap_as::<crate::reactor::ReactorCompletion<bool>>(a.arg(2))?;
        a.this::<McuTrsync>()?.start(a.f64(0)?, a.f64(1)?, &tc, a.f64(3)?)?;
        Ok(py_none())
    });
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, set_home_end_time(t, home_end_time: f64) => t.set_home_end_time(home_end_time));
    method!(r, "klippy.mcu.MCU_trsync", McuTrsync, stop(t) => t.stop());
    ro_attr!(r, "klippy.mcu.MCU_trsync._mcu", McuTrsync, |t| t.mcu.clone());
    ro_attr!(r, "klippy.mcu.MCU_trsync._oid", McuTrsync, |t| t.oid);
    ro_attr!(r, "klippy.mcu.MCU_trsync._reactor", McuTrsync, |t| t.reactor.clone());
    ro_attr!(r, "klippy.mcu.MCU_trsync._trdispatch", McuTrsync, |t| t.trdispatch.clone());
    r.attr("klippy.mcu.MCU_trsync._cmd_queue", |o| cq_to_py(ts(o)?.cmd_queue.clone()), None);
    r.attr("klippy.mcu.MCU_trsync._steppers", |o| steppers_to_py(ts(o)?.get_steppers()), None);
    r.attr("klippy.mcu.MCU_trsync._home_end_clock", |o| ts(o)?.home_end_clock.get().to_py(), None);
    r.attr("klippy.mcu.MCU_trsync._trigger_completion", |o| ts(o)?.trigger_completion.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.MCU_trsync._trsync_start_cmd", |o| opt_cmd_to_py(&ts(o)?.trsync_start_cmd), None);
    r.attr("klippy.mcu.MCU_trsync._trsync_set_timeout_cmd", |o| opt_cmd_to_py(&ts(o)?.trsync_set_timeout_cmd), None);
    r.attr("klippy.mcu.MCU_trsync._trsync_trigger_cmd", |o| opt_cmd_to_py(&ts(o)?.trsync_trigger_cmd), None);
    r.attr("klippy.mcu.MCU_trsync._trsync_query_cmd", |o| opt_cmd_to_py(&ts(o)?.trsync_query_cmd), None);
    r.attr("klippy.mcu.MCU_trsync._stepper_stop_cmd", |o| opt_cmd_to_py(&ts(o)?.stepper_stop_cmd), None);

    // ------------------------------------------- MCU_endstop and the pins
    // (MCU_x(mcu, pin_params): the pin_params dict as lookup_pin returns it)
    r.ctor("klippy.mcu.MCU_endstop.__init__", |a| {
        let e: Rc<dyn Any> = McuEndstop::new(&mc(a.arg(0))?, &super::pins::pin_params_from_py(a.arg(1))?);
        Ok(e)
    });
    method!(r, "klippy.mcu.MCU_endstop", McuEndstop, add_stepper(e, stepper: Rc<McuStepper>) => e.add_stepper(&stepper));
    method!(r, "klippy.mcu.MCU_endstop", McuEndstop, _build_config(e) => e.build_config());
    method!(r, "klippy.mcu.MCU_endstop", McuEndstop, _check_connected(e) => e.check_connected());
    // home_start(print_time, sample_time, sample_count, rest_time, triggered=True)
    method!(r, "klippy.mcu.MCU_endstop", McuEndstop, home_start(e, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: bool) =>
        e.home_start(print_time, sample_time, sample_count, rest_time, triggered));
    method!(r, "klippy.mcu.MCU_endstop", McuEndstop, home_wait(e, home_end_time: f64) => e.home_wait(home_end_time));
    ro_attr!(r, "klippy.mcu.MCU_endstop._mcu", McuEndstop, |e| e.mcu.clone());
    ro_attr!(r, "klippy.mcu.MCU_endstop._pin", McuEndstop, |e| e.pin.clone());
    ro_attr!(r, "klippy.mcu.MCU_endstop._pullup", McuEndstop, |e| e.pullup);
    ro_attr!(r, "klippy.mcu.MCU_endstop._invert", McuEndstop, |e| e.invert);
    ro_attr!(r, "klippy.mcu.MCU_endstop._oid", McuEndstop, |e| e.oid);
    ro_attr!(r, "klippy.mcu.MCU_endstop._dispatch", McuEndstop, |e| e.dispatch.clone());
    cell_attr!(r, "klippy.mcu.MCU_endstop._rest_ticks", McuEndstop, rest_ticks);
    r.attr("klippy.mcu.MCU_endstop._home_cmd", |o| opt_cmd_to_py(&objects::unwrap_as::<McuEndstop>(o)?.home_cmd), None);
    r.attr("klippy.mcu.MCU_endstop._query_cmd", |o| opt_cmd_to_py(&objects::unwrap_as::<McuEndstop>(o)?.query_cmd), None);

    r.ctor("klippy.mcu.MCU_digital_out.__init__", |a| {
        let d: Rc<dyn Any> = McuDigitalOut::new(&mc(a.arg(0))?, &super::pins::pin_params_from_py(a.arg(1))?);
        Ok(d)
    });
    method!(r, "klippy.mcu.MCU_digital_out", McuDigitalOut, _build_config(d) => d.build_config());
    ro_attr!(r, "klippy.mcu.MCU_digital_out._mcu", McuDigitalOut, |d| d.mcu.clone());
    ro_attr!(r, "klippy.mcu.MCU_digital_out._pin", McuDigitalOut, |d| d.pin.clone());
    ro_attr!(r, "klippy.mcu.MCU_digital_out._invert", McuDigitalOut, |d| d.invert);
    ro_attr!(r, "klippy.mcu.MCU_digital_out._printer", McuDigitalOut, |d| d.mcu.get_printer());
    cell_attr!(r, "klippy.mcu.MCU_digital_out._oid", McuDigitalOut, oid);
    cell_attr!(r, "klippy.mcu.MCU_digital_out._start_value", McuDigitalOut, start_value);
    cell_attr!(r, "klippy.mcu.MCU_digital_out._shutdown_value", McuDigitalOut, shutdown_value);
    cell_attr!(r, "klippy.mcu.MCU_digital_out._max_duration", McuDigitalOut, max_duration);
    cell_attr!(r, "klippy.mcu.MCU_digital_out._last_clock", McuDigitalOut, last_clock);
    r.attr("klippy.mcu.MCU_digital_out._set_cmd", |o| opt_cmd_to_py(&objects::unwrap_as::<McuDigitalOut>(o)?.set_cmd), None);

    r.ctor("klippy.mcu.MCU_pwm.__init__", |a| {
        let p: Rc<dyn Any> = McuPwm::new(&mc(a.arg(0))?, &super::pins::pin_params_from_py(a.arg(1))?);
        Ok(p)
    });
    method!(r, "klippy.mcu.MCU_pwm", McuPwm, _build_config(p) => p.build_config());
    ro_attr!(r, "klippy.mcu.MCU_pwm._mcu", McuPwm, |p| p.mcu.clone());
    ro_attr!(r, "klippy.mcu.MCU_pwm._pin", McuPwm, |p| p.pin.clone());
    ro_attr!(r, "klippy.mcu.MCU_pwm._invert", McuPwm, |p| p.invert);
    cell_attr!(r, "klippy.mcu.MCU_pwm._oid", McuPwm, oid);
    cell_attr!(r, "klippy.mcu.MCU_pwm._hardware_pwm", McuPwm, hardware_pwm);
    cell_attr!(r, "klippy.mcu.MCU_pwm._cycle_time", McuPwm, cycle_time);
    cell_attr!(r, "klippy.mcu.MCU_pwm._max_duration", McuPwm, max_duration);
    cell_attr!(r, "klippy.mcu.MCU_pwm._start_value", McuPwm, start_value);
    cell_attr!(r, "klippy.mcu.MCU_pwm._shutdown_value", McuPwm, shutdown_value);
    cell_attr!(r, "klippy.mcu.MCU_pwm._last_clock", McuPwm, last_clock);
    cell_attr!(r, "klippy.mcu.MCU_pwm._pwm_max", McuPwm, pwm_max);
    r.attr("klippy.mcu.MCU_pwm._set_cmd", |o| opt_cmd_to_py(&objects::unwrap_as::<McuPwm>(o)?.set_cmd), None);

    r.ctor("klippy.mcu.MCU_adc.__init__", |a| {
        let p: Rc<dyn Any> = McuAdc::new(&mc(a.arg(0))?, &super::pins::pin_params_from_py(a.arg(1))?);
        Ok(p)
    });
    method!(r, "klippy.mcu.MCU_adc", McuAdc, _build_config(p) => p.build_config());
    r.method("klippy.mcu.MCU_adc._handle_analog_in_state", |a| {
        a.this::<McuAdc>()?.handle_analog_in_state(&params_from_py(a.arg(0))?)?;
        Ok(py_none())
    });
    ro_attr!(r, "klippy.mcu.MCU_adc._mcu", McuAdc, |p| p.mcu.clone());
    ro_attr!(r, "klippy.mcu.MCU_adc._pin", McuAdc, |p| p.pin.clone());
    cell_attr!(r, "klippy.mcu.MCU_adc._oid", McuAdc, oid);
    cell_attr!(r, "klippy.mcu.MCU_adc._min_sample", McuAdc, min_sample);
    cell_attr!(r, "klippy.mcu.MCU_adc._max_sample", McuAdc, max_sample);
    cell_attr!(r, "klippy.mcu.MCU_adc._sample_time", McuAdc, sample_time);
    cell_attr!(r, "klippy.mcu.MCU_adc._report_time", McuAdc, report_time);
    cell_attr!(r, "klippy.mcu.MCU_adc._sample_count", McuAdc, sample_count);
    cell_attr!(r, "klippy.mcu.MCU_adc._range_check_count", McuAdc, range_check_count);
    cell_attr!(r, "klippy.mcu.MCU_adc._report_clock", McuAdc, report_clock);
    cell_attr!(r, "klippy.mcu.MCU_adc._inv_max_adc", McuAdc, inv_max_adc);
    r.attr("klippy.mcu.MCU_adc._last_state", |o| objects::unwrap_as::<McuAdc>(o)?.last_state.get().to_py(), None);
    // self._callback: the callback setup_adc_callback() was given
    r.attr("klippy.mcu.MCU_adc._callback", |o| match objects::unwrap_as::<McuAdc>(o)?.callback.borrow().clone() {
        Some(cb) => crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
            cb(a[0].as_f64()?, a[1].as_f64()?)?;
            Ok(py_none())
        })),
        None => Ok(py_none()),
    }, None);

    // --------------------------------------------------- TriggerDispatch
    r.ctor("klippy.mcu.TriggerDispatch.__init__", |a| {
        let t: Rc<dyn Any> = Rc::new(TriggerDispatch::new(&mc(a.arg(0))?));
        Ok(t)
    });
    method!(r, "klippy.mcu.TriggerDispatch", TriggerDispatch, get_oid(t) => t.get_oid());
    r.method("klippy.mcu.TriggerDispatch.get_command_queue", |a| cq_to_py(a.this::<TriggerDispatch>()?.get_command_queue()));
    method!(r, "klippy.mcu.TriggerDispatch", TriggerDispatch, add_stepper(t, stepper: Rc<McuStepper>) => t.add_stepper(&stepper));
    r.method("klippy.mcu.TriggerDispatch.get_steppers", |a| steppers_to_py(a.this::<TriggerDispatch>()?.get_steppers()));
    method!(r, "klippy.mcu.TriggerDispatch", TriggerDispatch, start(t, print_time: f64) => t.start(print_time));
    method!(r, "klippy.mcu.TriggerDispatch", TriggerDispatch, wait_end(t, end_time: f64) => t.wait_end(end_time));
    method!(r, "klippy.mcu.TriggerDispatch", TriggerDispatch, stop(t) => t.stop());
    ro_attr!(r, "klippy.mcu.TriggerDispatch._mcu", TriggerDispatch, |t| t.mcu.clone());
    ro_attr!(r, "klippy.mcu.TriggerDispatch._trdispatch", TriggerDispatch, |t| t.trdispatch.clone());
    r.attr("klippy.mcu.TriggerDispatch._trigger_completion", |o| td(o)?.trigger_completion.borrow().clone().to_py(), None);
    r.attr("klippy.mcu.TriggerDispatch._trsyncs", |o| {
        let ts = td(o)?.trsyncs.borrow().clone();
        Ok(py_list(ts.into_iter().map(objects::wrap).collect::<KResult<Vec<_>>>()?))
    }, None);
}
