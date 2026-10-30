// klippy/webhooks.py: WebHooks, WebRequest
use super::*;
use crate::method;
use crate::pyval::PyDict;
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::Args;
use crate::plugins::callable::py_function;
use crate::webhooks::{ClientConnection, ServerSocket, Ty, WebHooks, WebRequest};
use crate::IndexMap;

pub fn register(r: &mut Registry) {
    register_servers(r);
    r.method("klippy.webhooks.WebHooks.register_endpoint", |a| {
        let cb = a.arg(1).clone();
        a.this::<WebHooks>()?.register_endpoint(&a.string(0)?, endpoint_of(cb))?;
        Ok(py_none())
    });
    r.method("klippy.webhooks.WebHooks.register_mux_endpoint", |a| {
        let cb = a.arg(3).clone();
        a.this::<WebHooks>()?.register_mux_endpoint(&a.string(0)?, &a.string(1)?, a.opt_string(2)?.as_deref(), endpoint_of(cb))?;
        Ok(py_none())
    });
    // call_remote_method(method, **kwargs)
    r.method("klippy.webhooks.WebHooks.call_remote_method", |a| {
        let kwargs: PyVal = FromPy::from_py(a.arg(1))?;
        let PyVal::Dict(kw) = kwargs else { return Err(KError::Other("kaliru: kwargs".into())) };
        a.this::<WebHooks>()?.call_remote_method(&a.string(0)?, kw)?;
        Ok(py_none())
    });
    // _remote_methods: {method: {...}}; plugins test membership
    r.attr("klippy.webhooks.WebHooks._remote_methods", |o| {
        let names = objects::unwrap_as::<WebHooks>(o)?.remote_method_names();
        let d: PyDict = names.into_iter().map(|n| (n.into(), PyVal::Dict(IndexMap::default()))).collect();
        Ok(from_pyval(&PyVal::Dict(d)))
    }, None);
    // get_callback(path): the endpoint as a callable taking the request
    r.method("klippy.webhooks.WebHooks.get_callback", |a| {
        let cb = a.this::<WebHooks>()?.get_callback(&a.string(0)?)?;
        py_function(Rc::new(move |args: &[PyRef]| {
            cb(&*objects::unwrap_as::<WebRequest>(&args[0])?)?;
            Ok(py_none())
        }))
    });
    // WebRequest(client_conn, request): json.loads raises python's own error
    r.ctor("klippy.webhooks.WebRequest.__init__", |a| {
        let request = a.string(1)?;
        lookup("json", "loads")?.call(&[a.arg(1)])?;
        let conn = if a.arg(0).is_none() { None } else { Some(objects::unwrap_as::<ClientConnection>(a.arg(0))?) };
        let w: Rc<dyn Any> = Rc::new(WebRequest::new(conn, &request).map_err(|e| KError::Py("ValueError".into(), e))?);
        Ok(w)
    });
    r.attr("klippy.webhooks.ClientConnection.reactor", |o| objects::unwrap_as::<ClientConnection>(o)?.printer.get_reactor().to_py(), None);
    r.attr("klippy.webhooks.ServerSocket.reactor", |o| objects::unwrap_as::<ServerSocket>(o)?.printer.get_reactor().to_py(), None);
    r.attr("klippy.webhooks.GCodeHelper.printer", |o| {
        objects::unwrap_as::<crate::webhooks::GCodeHelper>(o)?;
        printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.to_py()
    }, None);

    method!(r, "klippy.webhooks.WebRequest", WebRequest, get_method(w) => w.get_method().to_string());
    r.method("klippy.webhooks.WebRequest.get", |a| {
        let types = types_of(a.arg(2))?;
        get_item(a, |w, item, d| w.get(item, d, types.as_deref()))
    });
    r.method("klippy.webhooks.WebRequest.get_str", |a| get_item(a, |w, item, d| w.get_str(item, d)));
    r.method("klippy.webhooks.WebRequest.get_int", |a| get_item(a, |w, item, d| w.get_int(item, d)));
    r.method("klippy.webhooks.WebRequest.get_float", |a| get_item(a, |w, item, d| w.get_float(item, d).map(PyVal::Float)));
    r.method("klippy.webhooks.WebRequest.get_dict", |a| get_item(a, |w, item, d| w.get_dict(item, d)));
    method!(r, "klippy.webhooks.WebRequest", WebRequest, set_error(w, error: PyRef) => w.set_error(&error.str()?));
    method!(r, "klippy.webhooks.WebRequest", WebRequest, send(w, data: PyVal) => w.send(data));
}

/// A python endpoint callback as kaliru's: the request object itself
fn endpoint_of(cb: PyRef) -> crate::webhooks::Endpoint {
    Rc::new(move |w: &WebRequest| {
        let req = match crate::running::running(w) {
            Some(rc) => rc.to_py()?,
            None => return Err(KError::Other("kaliru: web request outside its dispatch".into())),
        };
        cb.call(&[&req])?;
        Ok(())
    })
}

/// get*(item, default=Sentinel): the python default comes back as given
fn get_item(a: &Args, get: impl FnOnce(&WebRequest, &str, Option<PyVal>) -> KResult<PyVal>) -> KResult<PyRef> {
    let w = a.this::<WebRequest>()?;
    let item = a.string(0)?;
    let sentinel = lookup("klippy.webhooks", "Sentinel")?;
    if !w.params.contains_key(item.as_str()) && !a.arg(1).is(sentinel.as_ptr()) {
        return Ok(a.arg(1).clone());
    }
    Ok(from_pyval(&get(&w, &item, None)?))
}

/// types=(str, int, ...) as kaliru's Ty list
fn types_of(o: &PyRef) -> KResult<Option<Vec<Ty>>> {
    if o.is_none() {
        return Ok(None);
    }
    let items: Vec<PyRef> = FromPy::from_py(o)?;
    let mut out = vec![];
    for t in items {
        let name = t.getattr("__name__")?.as_string()?;
        out.push(match name.as_str() {
            "str" => Ty::Str,
            "int" => Ty::Int,
            "float" => Ty::Float,
            "dict" => Ty::Dict,
            _ => return Err(KError::Py("NotImplementedError".into(), format!("kaliru: WebRequest.get types={}", name))),
        });
    }
    Ok(Some(out))
}

// the api server socket and its client connections
fn register_servers(r: &mut Registry) {
    r.attr("klippy.webhooks.ServerSocket.fd_handle", |o| objects::unwrap_as::<ServerSocket>(o)?.fd_handle.borrow().clone().to_py(), None);
    r.attr("klippy.webhooks.ServerSocket.webhooks", |o| objects::unwrap_as::<ServerSocket>(o)?.printer.lookup_object("webhooks")?.to_py(), None);
    // {client_id: ClientConnection}
    r.attr("klippy.webhooks.ServerSocket.clients", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (uid, c) in objects::unwrap_as::<ServerSocket>(o)?.clients.borrow().iter() {
            d.call_method("__setitem__", &[&py_int(*uid as i64), &c.clone().to_py()?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.webhooks.ClientConnection.fd_handle", |o| objects::unwrap_as::<ClientConnection>(o)?.fd_handle.borrow().clone().to_py(), None);
    r.attr("klippy.webhooks.ClientConnection.server", |o| objects::unwrap_as::<ClientConnection>(o)?.server.upgrade().to_py(), None);
    r.attr("klippy.webhooks.ClientConnection.webhooks", |o| objects::unwrap_as::<ClientConnection>(o)?.webhooks.upgrade().to_py(), None);
    // deque([(time, request bytes)], maxlen=20) (a snapshot)
    r.attr("klippy.webhooks.ClientConnection.request_log", |o| {
        let items: KResult<Vec<PyRef>> = objects::unwrap_as::<ClientConnection>(o)?
            .request_log
            .borrow()
            .iter()
            .map(|(t, d)| Ok(py_tuple(vec![py_float(*t), lookup("builtins", "bytes")?.call(&[&d.clone().to_py()?])?])))
            .collect();
        lookup("collections", "deque")?.call(&[&py_list(items?), &py_int(crate::webhooks::REQUEST_LOG_SIZE as i64)])
    }, None);
    // {ClientConnection: template}
    r.attr("klippy.webhooks.GCodeHelper.clients", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (c, template) in objects::unwrap_as::<crate::webhooks::GCodeHelper>(o)?.clients.borrow().values() {
            d.call_method("__setitem__", &[&c.clone().to_py()?, &from_pyval(template)])?;
        }
        Ok(d)
    }, None);

    // _endpoints {path: callback(web_request)} / _mux_endpoints {path: (key,
    // {value: callback})} (snapshots)
    fn endpoint_to_py(cb: crate::webhooks::Endpoint) -> KResult<PyRef> {
        py_function(Rc::new(move |args: &[PyRef]| {
            cb(&*objects::unwrap_as::<WebRequest>(&args[0])?)?;
            Ok(py_none())
        }))
    }
    r.attr("klippy.webhooks.WebHooks._endpoints", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (path, cb) in objects::unwrap_as::<WebHooks>(o)?.endpoints.borrow().iter() {
            d.call_method("__setitem__", &[&py_str(path), &endpoint_to_py(cb.clone())?])?;
        }
        Ok(d)
    }, None);
    r.attr("klippy.webhooks.WebHooks._mux_endpoints", |o| {
        let d = lookup("builtins", "dict")?.call(&[])?;
        for (path, (key, values)) in objects::unwrap_as::<WebHooks>(o)?.mux_endpoints.borrow().iter() {
            let vd = lookup("builtins", "dict")?.call(&[])?;
            for (v, cb) in values {
                let k = match v {
                    Some(v) => py_str(v),
                    None => py_none(),
                };
                vd.call_method("__setitem__", &[&k, &endpoint_to_py(cb.clone())?])?;
            }
            d.call_method("__setitem__", &[&py_str(path), &py_tuple(vec![py_str(key), vd])])?;
        }
        Ok(d)
    }, None);
}
