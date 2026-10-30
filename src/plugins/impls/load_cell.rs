// klippy/extras/load_cell: ApiClientHelper, the sample collector, force
// clients (messages are Kalico's {"data": [[time, force, counts,
// tare_counts, ...]], "errors", "overflows"} dicts)
use super::*;
use crate::extras::load_cell::load_cell::{ApiClientHelper, ForceMsg, ForceSample, LoadCell, LoadCellSampleCollector};
use crate::plugins::callable::py_function;
use crate::plugins::convert::ToPy;

fn msg_to_py(m: &ForceMsg) -> PyRef {
    use crate::extras::load_cell::load_cell::ApiMsg;
    from_pyval(&m.to_pyval())
}

fn msg_from_py(o: &PyRef) -> KResult<ForceMsg> {
    let get = |k: &str| -> KResult<PyVal> { Ok(if o.call_method("__contains__", &[&py_str(k)])?.is_true()? { to_pyval(&dict_get(o, k)?)? } else { PyVal::None }) };
    let data = match get("data")? {
        PyVal::List(rows) | PyVal::Tuple(rows) => rows
            .iter()
            .map(|r| ForceSample::from_pyval(r).ok_or_else(|| KError::Py("ValueError".into(), "kaliru: a load cell sample row".into())))
            .collect::<KResult<Vec<_>>>()?,
        _ => vec![],
    };
    Ok(ForceMsg { data, errors: get("errors")?, overflows: get("overflows")? })
}

/// a client callback(msg) -> keep receiving, from python
fn client_from_py(cb: PyRef) -> Rc<dyn Fn(&ForceMsg) -> bool> {
    Rc::new(move |m: &ForceMsg| match cb.call(&[&msg_to_py(m)]).and_then(|r| r.is_true()) {
        Ok(keep) => keep,
        Err(e) => {
            report_unhandled(&e);
            false
        }
    })
}

/// (samples, (errors, overflows) or 0)
fn collected(r: (Vec<ForceSample>, Option<(i64, i64)>)) -> KResult<PyRef> {
    let rows: Vec<PyRef> = r.0.iter().map(|s| from_pyval(&s.to_pyval())).collect();
    let errors = match r.1 {
        Some((e, o)) => py_tuple(vec![py_int(e), py_int(o)]),
        None => py_int(0),
    };
    Ok(py_tuple(vec![py_list(rows), errors]))
}

pub fn register(r: &mut Registry) {
    type H = ApiClientHelper;
    r.method("klippy.extras.load_cell.load_cell.ApiClientHelper.send", |a| {
        a.this::<H>()?.send(&msg_from_py(a.arg(0))?);
        Ok(py_none())
    });
    r.method("klippy.extras.load_cell.load_cell.ApiClientHelper.add_client", |a| {
        a.this::<H>()?.add_client(client_from_py(a.arg(0).clone()));
        Ok(py_none())
    });
    // add_mux_endpoint(path, key, value, webhooks_start_resp)
    r.method("klippy.extras.load_cell.load_cell.ApiClientHelper.add_mux_endpoint", |a| {
        a.this::<H>()?.add_mux_endpoint(&a.string(0)?, &a.string(1)?, &a.string(2)?, to_pyval(a.arg(3))?)?;
        Ok(py_none())
    });
    r.attr("klippy.extras.load_cell.load_cell.ApiClientHelper.client_cbs", |o| {
        let cbs = objects::unwrap_as::<H>(o)?.client_cbs.borrow().clone();
        let items: KResult<Vec<PyRef>> = cbs
            .into_iter()
            .map(|cb| py_function(Rc::new(move |a: &[PyRef]| Ok(py_bool(cb(&msg_from_py(&a[0])?))))))
            .collect();
        Ok(py_list(items?))
    }, None);
    r.attr("klippy.extras.load_cell.load_cell.ApiClientHelper.webhooks_start_resp", |o| {
        Ok(from_pyval(&objects::unwrap_as::<H>(o)?.webhooks_start_resp.borrow()))
    }, None);
    r.attr("klippy.extras.load_cell.load_cell.ApiClientHelper.printer", |o| objects::unwrap_as::<H>(o)?.printer.clone().to_py(), None);
    r.method("klippy.extras.load_cell.load_cell.LoadCell.add_client", |a| {
        a.this::<LoadCell>()?.add_client(client_from_py(a.arg(0).clone()));
        Ok(py_none())
    });
    r.attr("klippy.extras.load_cell.load_cell.LoadCell.clients", |o| objects::unwrap_as::<LoadCell>(o)?.clients.clone().to_py(), None);
    r.attr("klippy.extras.load_cell.load_cell.LoadCellCommandHelper.name", |o| {
        Ok(py_str(&objects::unwrap_as::<crate::extras::load_cell::load_cell::LoadCellCommandHelper>(o)?.name))
    }, None);
    r.method("klippy.extras.load_cell.load_cell.LoadCellSampleCollector.stop_collecting", |a| collected(a.this::<LoadCellSampleCollector>()?.stop_collecting()));
    r.method("klippy.extras.load_cell.load_cell.LoadCellSampleCollector.collect_min", |a| {
        collected(a.this::<LoadCellSampleCollector>()?.collect_min(a.arg(0).as_i64()?)?)
    });
    r.method("klippy.extras.load_cell.load_cell.LoadCellSampleCollector.collect_until", |a| {
        collected(a.this::<LoadCellSampleCollector>()?.collect_until(a.f64(0)?)?)
    });
    // ads1220's option tables: {name: value}
    {
        use crate::extras::load_cell::ads1220 as a;
        fn table(t: &[(&str, i64)]) -> KResult<PyRef> {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in t {
                d.call_method("__setitem__", &[&py_str(k), &py_int(*v)])?;
            }
            Ok(d)
        }
        r.attr("klippy.extras.load_cell.ads1220.ADS1220.gain_options", |_o| table(&a::GAIN_OPTIONS), None);
        r.attr("klippy.extras.load_cell.ads1220.ADS1220.vref_options", |_o| table(&a::VREF_OPTIONS), None);
        r.attr("klippy.extras.load_cell.ads1220.ADS1220.sps_normal", |_o| table(&a::SPS_NORMAL), None);
        r.attr("klippy.extras.load_cell.ads1220.ADS1220.sps_turbo", |_o| table(&a::SPS_TURBO), None);
        r.attr("klippy.extras.load_cell.ads1220.ADS1220.sps_options", |_o| table(&a::sps_options()), None);
        // "<" + "i" * channels
        r.attr("klippy.extras.load_cell.ads131m0x.ADS131MxBase.unpack_format", |o| {
            let s = objects::unwrap_as::<crate::extras::load_cell::ads131m0x::ADS131MxBase>(o)?;
            Ok(py_str(&format!("<{}", "i".repeat(s.channels.len()))))
        }, None);
    }
}
