// Klippy WebHooks registration and server connection
// (port of klippy/webhooks.py)
//
// Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license
//
// Differences: "klipper_path" in the info response is the directory above
// KALIRU_KLIPPY_DIR, else above the binary's directory, and "python_path"
// the binary itself; client ids are rust
// pointer values like python's id(); numpy conversion in send() is not
// needed (PyVal is always serializable).
use crate::error::{KError, KResult};
use crate::gcode::GCodeDispatch;
use crate::logging;
use crate::printer::{Printer, PrinterObject, APP_NAME};
use crate::pyfmt::py_bytes_repr;
use crate::pyval::{PyDict, PyKey, PyVal};
use crate::reactor::{FileHandler, ReactorCompletion, Timer, NEVER, NOW};
use crate::IndexMap;
use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::os::fd::RawFd;
use std::rc::Rc;

pub const REQUEST_LOG_SIZE: usize = 20;

/// WebRequestError (a gcode.CommandError)
fn web_error(msg: impl Into<String>) -> KError {
    KError::Command(msg.into())
}

fn error_dict(message: &str) -> PyVal {
    let d = crate::pydict! {
        "error" => PyVal::Str("WebRequestError".into()),
        "message" => PyVal::Str(message.to_string()),
    };
    PyVal::Dict(d)
}

pub type Endpoint = Rc<dyn Fn(&WebRequest) -> KResult<()>>;

pub struct WebRequest {
    pub client_conn: Option<Rc<ClientConnection>>,
    pub id: PyVal,
    pub method: String,
    pub params: PyDict,
    pub response: RefCell<Option<PyVal>>,
    pub is_error: Cell<bool>,
}

/// expected python types for WebRequest.get(..., types=...)
#[derive(Clone, Copy)]
pub enum Ty {
    Str,
    Int,
    Float,
    Dict,
}

fn type_ok(v: &PyVal, types: &[Ty]) -> bool {
    types.iter().any(|t| matches!((t, v), (Ty::Str, PyVal::Str(_)) | (Ty::Int, PyVal::Int(_)) | (Ty::Float, PyVal::Float(_)) | (Ty::Dict, PyVal::Dict(_))))
}

impl WebRequest {
    /// WebRequest(client_conn, request); Err is the python exception text
    pub fn new(client_conn: Option<Rc<ClientConnection>>, request: &str) -> Result<WebRequest, String> {
        let base_request = PyVal::json_loads(request)?;
        let PyVal::Dict(base) = base_request else {
            return Err("Not a top-level dictionary".into());
        };
        let id = base.get("id").cloned().unwrap_or(PyVal::None);
        let method = base.get("method").cloned().unwrap_or(PyVal::None);
        let params = base.get("params").cloned().unwrap_or_else(PyVal::dict);
        let (PyVal::Str(method), PyVal::Dict(params)) = (method, params) else {
            return Err("Invalid request type".into());
        };
        Ok(WebRequest { client_conn, id, method, params, response: RefCell::new(None), is_error: Cell::new(false) })
    }

    pub fn get_client_connection(&self) -> Option<Rc<ClientConnection>> {
        self.client_conn.clone()
    }

    /// get(item, default=Sentinel, types=None); default None = required
    pub fn get(&self, item: &str, default: Option<PyVal>, types: Option<&[Ty]>) -> KResult<PyVal> {
        let value = match (self.params.get(item), default) {
            (Some(v), _) => v.clone(),
            (None, Some(d)) => return Ok(d),
            (None, None) => return Err(web_error(format!("Missing Argument [{}]", item))),
        };
        if let Some(types) = types {
            if !type_ok(&value, types) {
                return Err(web_error(format!("Invalid Argument Type [{}]", item)));
            }
        }
        Ok(value)
    }

    pub fn get_str(&self, item: &str, default: Option<PyVal>) -> KResult<PyVal> {
        self.get(item, default, Some(&[Ty::Str]))
    }

    pub fn get_int(&self, item: &str, default: Option<PyVal>) -> KResult<PyVal> {
        self.get(item, default, Some(&[Ty::Int]))
    }

    /// float(get(item, default, types=(int, float)))
    pub fn get_float(&self, item: &str, default: Option<PyVal>) -> KResult<f64> {
        let v = self.get(item, default, Some(&[Ty::Int, Ty::Float]))?;
        v.as_f64().ok_or_else(|| KError::Other(format!("float() argument must be a string or a real number, not '{}'", v.repr())))
    }

    pub fn get_dict(&self, item: &str, default: Option<PyVal>) -> KResult<PyVal> {
        self.get(item, default, Some(&[Ty::Dict]))
    }

    pub fn get_method(&self) -> &str {
        &self.method
    }

    pub fn set_error(&self, message: &str) {
        self.is_error.set(true);
        *self.response.borrow_mut() = Some(error_dict(message));
    }

    pub fn send(&self, data: PyVal) -> KResult<()> {
        if self.response.borrow().is_some() {
            return Err(web_error("Multiple calls to send not allowed"));
        }
        *self.response.borrow_mut() = Some(data);
        Ok(())
    }

    pub fn finish(&self) -> Option<PyVal> {
        if self.id == PyVal::None {
            return None;
        }
        let rtype = if self.is_error.get() { "error" } else { "result" };
        // No error was set and the user never executed send, default
        // response is {}
        let response = self.response.borrow().clone().unwrap_or_else(PyVal::dict);
        let mut d = IndexMap::default();
        d.insert("id".into(), self.id.clone());
        d.insert(rtype.to_string().into(), response);
        Some(PyVal::Dict(d))
    }
}

pub struct ServerSocket {
    pub printer: Rc<Printer>,
    sock: Cell<RawFd>,
    fd_handle: RefCell<Option<FileHandler>>,
    clients: RefCell<IndexMap<usize, Rc<ClientConnection>>>,
}

impl ServerSocket {
    pub fn new(webhooks: &Rc<WebHooks>, printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let ss = Rc::new(ServerSocket { printer: printer.clone(), sock: Cell::new(-1), fd_handle: RefCell::new(None), clients: RefCell::new(IndexMap::default()) });
        let server_address = printer.start_arg("apiserver_file").map(|v| v.str());
        let is_fileinput = printer.start_arg("debuginput").is_some();
        let Some(server_address) = server_address.filter(|s| !s.is_empty() && !is_fileinput) else {
            // Do not enable server
            return Ok(ss);
        };
        ss.remove_socket_file(&server_address)?;
        // (python sockets are non-inheritable: shell commands do not get them)
        // SAFETY: a plain syscall with constant arguments; the fd is checked below
        let sock = unsafe { libc::socket(libc::AF_UNIX, libc::SOCK_STREAM | libc::SOCK_CLOEXEC, 0) };
        if sock < 0 {
            return Err(KError::Other(std::io::Error::last_os_error().to_string()));
        }
        crate::util::set_nonblock(sock);
        // SAFETY: sockaddr_un is plain data: all zero is valid (and NUL-terminates the path)
        let mut addr: libc::sockaddr_un = unsafe { std::mem::zeroed() };
        addr.sun_family = libc::AF_UNIX as libc::sa_family_t;
        let bytes = server_address.as_bytes();
        if bytes.len() >= addr.sun_path.len() {
            return Err(KError::Other("AF_UNIX path too long".into()));
        }
        for (i, b) in bytes.iter().enumerate() {
            addr.sun_path[i] = *b as libc::c_char;
        }
        // SAFETY: addr is a live sockaddr_un and the length is its size
        let r = unsafe {
            libc::bind(sock, &addr as *const libc::sockaddr_un as *const libc::sockaddr, std::mem::size_of::<libc::sockaddr_un>() as libc::socklen_t)
        };
        if r != 0 {
            return Err(KError::Other(crate::pyfmt::py_oserror(&std::io::Error::last_os_error(), &server_address)));
        }
        apply_permission_options_to_socket(&server_address, printer)?;
        // SAFETY: sock is the open socket created above
        unsafe { libc::listen(sock, 1) };
        ss.sock.set(sock);
        let (s, w) = (ss.clone(), Rc::downgrade(webhooks));
        let h = printer.reactor.register_fd(
            sock,
            Box::new(move |_| {
                if let Some(wh) = w.upgrade() {
                    s.handle_accept(&wh);
                }
            }),
            None,
        );
        *ss.fd_handle.borrow_mut() = Some(h);
        let s = ss.clone();
        crate::events::KlippyDisconnect::on(printer, move || {
            s.handle_disconnect();
            Ok(())
        });
        let s = ss.clone();
        crate::events::KlippyAnalyzeShutdown::on(printer, move |_, _| {
            for client in s.clients.borrow().values() {
                client.dump_request_log();
            }
            Ok(())
        });
        Ok(ss)
    }

    pub fn handle_accept(self: &Rc<Self>, webhooks: &Rc<WebHooks>) {
        // SAFETY: null address pointers are allowed (the peer address is not wanted)
        let fd = unsafe { libc::accept4(self.sock.get(), std::ptr::null_mut(), std::ptr::null_mut(), libc::SOCK_CLOEXEC) };
        if fd < 0 {
            return;
        }
        crate::util::set_nonblock(fd);
        let client = ClientConnection::new(self, webhooks, fd);
        self.clients.borrow_mut().insert(client.uid, client);
    }

    pub fn handle_disconnect(&self) {
        let clients: Vec<_> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            client.close();
        }
        let sock = self.sock.replace(-1);
        if sock >= 0 {
            if let Some(h) = self.fd_handle.borrow_mut().take() {
                self.printer.reactor.unregister_fd(&h);
            }
            // SAFETY: the server's own fd, taken out of self.sock: closed once
            unsafe { libc::close(sock) };
        }
    }

    pub fn remove_socket_file(&self, file_path: &str) -> KResult<()> {
        if let Err(e) = std::fs::remove_file(file_path) {
            if std::path::Path::new(file_path).exists() {
                logging::exception(&format!("webhooks: Unable to delete socket file '{}'", file_path));
                return Err(KError::Other(crate::pyfmt::py_oserror(&e, file_path)));
            }
        }
        Ok(())
    }

    pub fn pop_client(&self, client_id: usize) {
        self.clients.borrow_mut().shift_remove(&client_id);
    }

    pub fn stats(&self, _eventtime: f64) -> (bool, String) {
        // Called once per second - check for idle clients
        let clients: Vec<_> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            if client.is_blocking.get() {
                client.blocking_count.set(client.blocking_count.get() - 1);
                if client.blocking_count.get() < 0 {
                    logging::info(&format!("Closing unresponsive client {}", client.uid));
                    client.close();
                }
            }
        }
        (false, String::new())
    }
}

fn apply_permission_options_to_socket(socket_path: &str, printer: &Printer) -> KResult<()> {
    let user = printer.start_arg("apiserver_user").map(|v| v.str());
    let group = printer.start_arg("apiserver_group").map(|v| v.str());
    let file_mode = printer.start_arg("apiserver_file_mode").map(|v| v.str());
    // (os.chown / os.chmod raise OSError, as the bind above)
    let oserror = |e: std::io::Error| KError::Other(crate::pyfmt::py_oserror(&e, socket_path));
    if user.is_some() || group.is_some() {
        let uid = match &user {
            None => None,
            Some(u) => {
                let c = std::ffi::CString::new(u.as_str()).unwrap();
                // SAFETY: getpwnam reads a NUL-terminated name and returns
                // null or a pointer to static storage, read before any other
                // passwd call (single-threaded here)
                let pw = unsafe { libc::getpwnam(c.as_ptr()) };
                if pw.is_null() {
                    return Err(KError::Other(format!("getpwnam(): name not found: {}", crate::pyfmt::py_str_repr(u))));
                }
                // SAFETY: non-null, see above
                Some(unsafe { (*pw).pw_uid })
            }
        };
        let gid = match &group {
            None => None,
            Some(g) => {
                let c = std::ffi::CString::new(g.as_str()).unwrap();
                // SAFETY: as getpwnam above
                let gr = unsafe { libc::getgrnam(c.as_ptr()) };
                if gr.is_null() {
                    return Err(KError::Other(format!("getgrnam(): name not found: {}", crate::pyfmt::py_str_repr(g))));
                }
                // SAFETY: non-null, see above
                Some(unsafe { (*gr).gr_gid })
            }
        };
        std::os::unix::fs::chown(socket_path, uid, gid).map_err(oserror)?;
    }
    if let Some(mode) = file_mode {
        let mode_int = crate::pyfmt::py_int_digits(&mode, 8)
            .and_then(|(neg, _, d)| if neg { None } else { Some(d.iter().fold(0u32, |a, x| a * 8 + *x as u32)) })
            .ok_or_else(|| KError::Other(format!("invalid literal for int() with base 8: {}", crate::pyfmt::py_str_repr(&mode))))?;
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(socket_path, std::fs::Permissions::from_mode(mode_int)).map_err(oserror)?;
    }
    Ok(())
}

pub struct ClientConnection {
    pub printer: Rc<Printer>,
    webhooks: std::rc::Weak<WebHooks>,
    server: std::rc::Weak<ServerSocket>,
    pub uid: usize,
    sock: RawFd,
    fd_handle: RefCell<Option<FileHandler>>,
    pub partial_data: RefCell<Vec<u8>>,
    pub send_buffer: RefCell<Vec<u8>>,
    /// the reply being serialized (kept between replies)
    json_scratch: RefCell<String>,
    pub is_blocking: Cell<bool>,
    pub blocking_count: Cell<i64>,
    request_log: RefCell<VecDeque<(f64, Vec<u8>)>>,
}

impl ClientConnection {
    fn new(server: &Rc<ServerSocket>, webhooks: &Rc<WebHooks>, sock: RawFd) -> Rc<Self> {
        let c = Rc::new_cyclic(|me: &std::rc::Weak<ClientConnection>| ClientConnection {
            printer: server.printer.clone(),
            webhooks: Rc::downgrade(webhooks),
            server: Rc::downgrade(server),
            // id(self)
            uid: me.as_ptr() as usize,
            sock,
            fd_handle: RefCell::new(None),
            partial_data: RefCell::new(vec![]),
            send_buffer: RefCell::new(vec![]),
            json_scratch: RefCell::new(String::new()),
            is_blocking: Cell::new(false),
            blocking_count: Cell::new(0),
            request_log: RefCell::new(VecDeque::new()),
        });
        let (r, w) = (c.clone(), c.clone());
        let h = c.printer.reactor.register_fd(sock, Box::new(move |e| r.process_received(e)), Some(Box::new(move |_| w.do_send())));
        *c.fd_handle.borrow_mut() = Some(h);
        c.set_client_info(Some(&PyVal::Str("?".into())), Some("New connection"));
        c
    }

    pub fn dump_request_log(&self) {
        let log = self.request_log.borrow();
        let mut out = vec![format!("Dumping {} requests for client {}", log.len(), self.uid)];
        for (eventtime, request) in log.iter() {
            out.push(format!("Received {:.6}: {}", eventtime, py_bytes_repr(request)));
        }
        logging::info(&out.join("\n"));
    }

    pub fn set_client_info(&self, client_info: Option<&PyVal>, state_msg: Option<&str>) {
        let state_msg = match state_msg {
            Some(s) => s.to_string(),
            None => format!("Client info {}", client_info.map(|c| c.repr()).unwrap_or_else(|| "None".into())),
        };
        logging::info(&format!("webhooks client {}: {}", self.uid, state_msg));
        let log_id = format!("webhooks {}", self.uid);
        let Some(client_info) = client_info else {
            self.printer.set_rollover_info(&log_id, None, false);
            return;
        };
        let rollover_msg = format!("webhooks client {}: {}", self.uid, client_info.repr());
        self.printer.set_rollover_info(&log_id, Some(&rollover_msg), false);
    }

    pub fn close(&self) {
        let Some(h) = self.fd_handle.borrow_mut().take() else { return };
        self.set_client_info(None, Some("Disconnected"));
        self.printer.reactor.unregister_fd(&h);
        // SAFETY: the client's own fd; close() runs once (fd_handle was taken above)
        unsafe { libc::close(self.sock) };
        if let Some(s) = self.server.upgrade() {
            s.pop_client(self.uid);
        }
    }

    pub fn is_closed(&self) -> bool {
        self.fd_handle.borrow().is_none()
    }

    pub fn process_received(self: &Rc<Self>, eventtime: f64) {
        let mut buf = [0u8; 4096];
        // SAFETY: buf is a live buffer of buf.len() bytes
        let n = unsafe { libc::recv(self.sock, buf.as_mut_ptr() as *mut libc::c_void, buf.len(), 0) };
        let data: &[u8] = if n < 0 {
            // If bad file descriptor allow connection to be closed by the
            // data check
            if std::io::Error::last_os_error().raw_os_error() == Some(libc::EBADF) {
                &[]
            } else {
                return;
            }
        } else {
            &buf[..n as usize]
        };
        if data.is_empty() {
            // Socket Closed
            self.close();
            return;
        }
        let mut requests: Vec<Vec<u8>> = data.split(|b| *b == 0x03).map(|r| r.to_vec()).collect();
        let first = [self.partial_data.borrow().as_slice(), requests[0].as_slice()].concat();
        requests[0] = first;
        *self.partial_data.borrow_mut() = requests.pop().unwrap();
        for req in requests {
            {
                let mut log = self.request_log.borrow_mut();
                log.push_back((eventtime, req.clone()));
                if log.len() > REQUEST_LOG_SIZE {
                    log.pop_front();
                }
            }
            let web_request = std::str::from_utf8(&req)
                .map_err(|e| e.to_string())
                .and_then(|s| WebRequest::new(Some(self.clone()), s));
            let web_request = match web_request {
                Ok(w) => w,
                Err(_) => {
                    logging::exception(&format!("webhooks: Error decoding Server Request {}", py_bytes_repr(&req)));
                    continue;
                }
            };
            let s = self.clone();
            self.printer.reactor.register_callback(move |_| s.process_request(web_request), NOW);
        }
    }

    fn process_request(self: &Rc<Self>, web_request: WebRequest) {
        let web_request = Rc::new(web_request);
        let res = (|| -> KResult<()> {
            let wh = self.webhooks.upgrade().ok_or_else(|| KError::Other("webhooks gone".into()))?;
            let func = wh.get_callback(web_request.get_method())?;
            crate::running::run(&web_request, |w| func(w))
        })();
        if let Err(e) = res {
            if e.is_command_error() {
                web_request.set_error(&e.to_string());
            } else {
                let msg = format!("Internal Error on WebRequest: {}", web_request.get_method());
                logging::exception(&msg);
                web_request.set_error(&e.to_string());
                self.printer.invoke_shutdown(&msg, None);
            }
        }
        let Some(result) = web_request.finish() else { return };
        self.send(&result);
    }

    pub fn send(&self, data: &PyVal) {
        {
            // (serialized into a reused string: no allocation per message)
            let mut jmsg = self.json_scratch.borrow_mut();
            jmsg.clear();
            data.json_compact_into(&mut jmsg);
            let mut buf = self.send_buffer.borrow_mut();
            buf.extend_from_slice(jmsg.as_bytes());
            buf.push(0x03);
            // (the first, large replies are not kept: the steady ones are small)
            if jmsg.capacity() > 16384 {
                *jmsg = String::new();
            }
        }
        if !self.is_blocking.get() {
            self.do_send();
        }
    }

    fn do_send(&self) {
        let Some(h) = self.fd_handle.borrow().clone() else { return };
        let (sent, len) = {
            let buf = self.send_buffer.borrow();
            // SAFETY: buf is borrowed for the call and buf.len() bytes long
            (unsafe { libc::send(self.sock, buf.as_ptr() as *const libc::c_void, buf.len(), libc::MSG_NOSIGNAL) }, buf.len())
        };
        let sent = if sent < 0 {
            let err = std::io::Error::last_os_error().raw_os_error();
            if err != Some(libc::EAGAIN) && err != Some(libc::EWOULDBLOCK) {
                logging::info(&format!("webhooks: socket write error {}", self.uid));
                self.close();
                return;
            }
            0
        } else {
            sent as usize
        };
        if sent < len {
            if !self.is_blocking.get() {
                self.printer.reactor.set_fd_wake(&h, false, true);
                self.is_blocking.set(true);
                self.blocking_count.set(5);
            }
        } else if self.is_blocking.get() {
            self.printer.reactor.set_fd_wake(&h, true, false);
            self.is_blocking.set(false);
        }
        let mut buf = self.send_buffer.borrow_mut();
        buf.drain(..sent);
        // (a large reply's buffer is not kept once it is sent)
        if buf.is_empty() && buf.capacity() > 16384 {
            *buf = Vec::new();
        }
    }
}

type MuxValues = IndexMap<Option<String>, Endpoint>;

pub struct WebHooks {
    pub printer: Rc<Printer>,
    endpoints: RefCell<IndexMap<String, Endpoint>>,
    remote_methods: RefCell<IndexMap<String, IndexMap<usize, (Rc<ClientConnection>, PyVal)>>>,
    mux_endpoints: RefCell<IndexMap<String, (String, MuxValues)>>,
    pub sconn: RefCell<Option<Rc<ServerSocket>>>,
}

impl PrinterObject for WebHooks {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let (state_message, state) = self.printer.get_state_message();
        let d = crate::pydict! {
            "state" => PyVal::Str(state.into()),
            "state_message" => PyVal::Str(state_message),
        };
        Some(PyVal::Dict(d))
    }
    fn has_stats(&self) -> bool {
        true
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        self.sconn.borrow().as_ref().map(|s| s.stats(eventtime))
    }
    fn teardown(&self) {
        self.endpoints.borrow_mut().clear();
        self.mux_endpoints.borrow_mut().clear();
        self.remote_methods.borrow_mut().clear();
        if let Some(s) = self.sconn.borrow_mut().take() {
            s.clients.borrow_mut().clear();
            s.fd_handle.borrow_mut().take();
        }
    }
}

impl WebHooks {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let wh = Rc::new(WebHooks {
            printer: printer.clone(),
            endpoints: RefCell::new(IndexMap::default()),
            remote_methods: RefCell::new(IndexMap::default()),
            mux_endpoints: RefCell::new(IndexMap::default()),
            sconn: RefCell::new(None),
        });
        let w = Rc::downgrade(&wh);
        wh.endpoints.borrow_mut().insert(
            "list_endpoints".into(),
            Rc::new(move |wr: &WebRequest| {
                let wh = w.upgrade().unwrap();
                let names: Vec<PyVal> = wh.endpoints.borrow().keys().map(|k| PyVal::Str(k.clone())).collect();
                let mut d = IndexMap::default();
                d.insert("endpoints".into(), PyVal::List(names));
                wr.send(PyVal::Dict(d))
            }),
        );
        let w = Rc::downgrade(&wh);
        wh.register_endpoint("info", Rc::new(move |wr| w.upgrade().unwrap().handle_info_request(wr)))?;
        let w = Rc::downgrade(&wh);
        wh.register_endpoint(
            "emergency_stop",
            Rc::new(move |_| {
                w.upgrade().unwrap().printer.invoke_shutdown("Shutdown due to webhooks request", None);
                Ok(())
            }),
        )?;
        let w = Rc::downgrade(&wh);
        wh.register_endpoint("register_remote_method", Rc::new(move |wr| w.upgrade().unwrap().handle_rpc_registration(wr)))?;
        let sconn = ServerSocket::new(&wh, printer)?;
        *wh.sconn.borrow_mut() = Some(sconn);
        Ok(wh)
    }

    pub fn register_endpoint(&self, path: &str, callback: Endpoint) -> KResult<()> {
        if self.endpoints.borrow().contains_key(path) {
            return Err(web_error("Path already registered to an endpoint"));
        }
        self.endpoints.borrow_mut().insert(path.to_string(), callback);
        Ok(())
    }

    /// register_mux_endpoint(path, key, value, callback); value None is
    /// python's None key
    pub fn register_mux_endpoint(self: &Rc<Self>, path: &str, key: &str, value: Option<&str>, callback: Endpoint) -> KResult<()> {
        if !self.mux_endpoints.borrow().contains_key(path) {
            let w = Rc::downgrade(self);
            self.register_endpoint(path, Rc::new(move |wr| w.upgrade().unwrap().handle_mux(wr)))?;
            self.mux_endpoints.borrow_mut().insert(path.to_string(), (key.to_string(), IndexMap::default()));
        }
        let mut mux = self.mux_endpoints.borrow_mut();
        let (prev_key, prev_values) = mux.get_mut(path).unwrap();
        let show = |v: Option<&str>| v.map(|s| s.to_string()).unwrap_or_else(|| "None".into());
        if prev_key != key {
            return Err(KError::config(format!("mux endpoint {} {} {} may have only one key ({})", path, key, show(value), prev_key)));
        }
        let k = value.map(|s| s.to_string());
        if prev_values.contains_key(&k) {
            let keys: Vec<String> = prev_values.keys().map(|k| k.as_deref().map(crate::pyfmt::py_str_repr).unwrap_or_else(|| "None".into())).collect();
            return Err(KError::config(format!("mux endpoint {} {} {} already registered ({{{}}})", path, key, show(value), keys.join(", "))));
        }
        prev_values.insert(k, callback);
        Ok(())
    }

    pub fn handle_mux(&self, web_request: &WebRequest) -> KResult<()> {
        let func = {
            let mux = self.mux_endpoints.borrow();
            let (key, values) = mux.get(web_request.get_method()).unwrap();
            let key_param = if values.contains_key(&None) {
                web_request.get(key, Some(PyVal::None), None)?
            } else {
                web_request.get(key, None, None)?
            };
            let lookup = match &key_param {
                PyVal::None => Some(None),
                PyVal::Str(s) => Some(Some(s.clone())),
                _ => None,
            };
            match lookup.and_then(|k| values.get(&k).cloned()) {
                Some(f) => f,
                None => return Err(web_error(format!("The value '{}' is not valid for {}", key_param.str(), key))),
            }
        };
        func(web_request)
    }

    pub fn handle_info_request(&self, web_request: &WebRequest) -> KResult<()> {
        let client_info = web_request.get_dict("client_info", Some(PyVal::None))?;
        if client_info != PyVal::None {
            if let Some(c) = web_request.get_client_connection() {
                c.set_client_info(Some(&client_info), None);
            }
        }
        let (state_message, state) = self.printer.get_state_message();
        let exe = std::env::current_exe().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
        // klippy/.. as python: the klippy tree kaliru reads its python from
        // (KALIRU_KLIPPY_DIR) when there is one; Moonraker registers
        // klipper_path/config and /docs from it
        let src_path = std::env::var("KALIRU_KLIPPY_DIR").unwrap_or_else(|_| crate::pyglob::dirname(&exe));
        let klipper_path = crate::pyglob::normpath(&crate::pyglob::join(&src_path, &[".."]));
        let mut hostname = [0u8; 256];
        // SAFETY: hostname is a live buffer of hostname.len() bytes
        unsafe { libc::gethostname(hostname.as_mut_ptr() as *mut libc::c_char, hostname.len()) };
        let hostname = String::from_utf8_lossy(&hostname[..hostname.iter().position(|b| *b == 0).unwrap_or(0)]).into_owned();
        let mut response = crate::pydict! {
            "app" => PyVal::Str(APP_NAME.into()),
            "state" => PyVal::Str(state.into()),
            "state_message" => PyVal::Str(state_message),
            "hostname" => PyVal::Str(hostname),
            "klipper_path" => PyVal::Str(klipper_path),
            "python_path" => PyVal::Str(exe),
            "process_id" => PyVal::Int(std::process::id() as i64),
        };
        // SAFETY: getuid / getgid cannot fail and take no pointers
        response.insert("user_id".into(), PyVal::Int(unsafe { libc::getuid() } as i64));
        response.insert("group_id".into(), PyVal::Int(unsafe { libc::getgid() } as i64));
        for sa in ["log_file", "config_file", "software_version", "cpu_info"] {
            response.insert(sa.to_string().into(), self.printer.start_arg(sa).unwrap_or(PyVal::None));
        }
        web_request.send(PyVal::Dict(response))
    }

    pub fn handle_rpc_registration(&self, web_request: &WebRequest) -> KResult<()> {
        let template = web_request.get_dict("response_template", None)?;
        let method = web_request.get_str("remote_method", None)?.str();
        let Some(new_conn) = web_request.get_client_connection() else { return Ok(()) };
        if crate::extras::danger_options::get_danger_options().log_webhook_method_register_messages {
            logging::info(&format!("webhooks: registering remote method '{}' for connection id: {}", method, new_conn.uid));
        }
        self.remote_methods.borrow_mut().entry(method).or_default().insert(new_conn.uid, (new_conn, template));
        Ok(())
    }

    pub fn get_connection(&self) -> Option<Rc<ServerSocket>> {
        self.sconn.borrow().clone()
    }

    /// the methods clients registered (python plugins read `_remote_methods`)
    pub fn remote_method_names(&self) -> Vec<String> {
        self.remote_methods.borrow().keys().cloned().collect()
    }

    pub fn get_callback(&self, path: &str) -> KResult<Endpoint> {
        match self.endpoints.borrow().get(path) {
            Some(cb) => Ok(cb.clone()),
            None => {
                let msg = format!("webhooks: No registered callback for path '{}'", path);
                logging::info(&msg);
                Err(web_error(msg))
            }
        }
    }

    /// call_remote_method(method, **kwargs)
    pub fn call_remote_method(&self, method: &str, kwargs: PyDict) -> KResult<()> {
        let conn_map = self
            .remote_methods
            .borrow()
            .get(method)
            .cloned()
            .ok_or_else(|| KError::command(format!("Remote method '{}' not registered", method)))?;
        let mut valid_conns = IndexMap::default();
        for (uid, (conn, template)) in conn_map {
            if !conn.is_closed() {
                let mut out = IndexMap::default();
                out.insert("params".into(), PyVal::Dict(kwargs.clone()));
                if let PyVal::Dict(t) = &template {
                    for (k, v) in t {
                        out.insert(k.clone(), v.clone());
                    }
                }
                conn.send(&PyVal::Dict(out));
                valid_conns.insert(uid, (conn, template));
            }
        }
        if valid_conns.is_empty() {
            self.remote_methods.borrow_mut().shift_remove(method);
            return Err(KError::command(format!("No active connections for method '{}'", method)));
        }
        self.remote_methods.borrow_mut().insert(method.to_string(), valid_conns);
        Ok(())
    }
}

/// an output callback webhooks registered for its API clients
/// (isinstance(cb.__self__, GCodeHelper))
pub fn is_gcode_helper(origin: &crate::gcode::CallbackOrigin) -> bool {
    origin.1 == std::any::type_name::<GCodeHelper>()
}

pub struct GCodeHelper {
    pub gcode: Rc<GCodeDispatch>,
    pub is_output_registered: Cell<bool>,
    clients: RefCell<IndexMap<usize, (Rc<ClientConnection>, PyVal)>>,
}

impl GCodeHelper {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let gh = Rc::new(GCodeHelper {
            gcode: printer.lookup::<GCodeDispatch>("gcode")?,
            is_output_registered: Cell::new(false),
            clients: RefCell::new(IndexMap::default()),
        });
        // Register webhooks
        let wh = printer.lookup::<WebHooks>("webhooks")?;
        let g = gh.clone();
        wh.register_endpoint(
            "gcode/help",
            Rc::new(move |wr| {
                let help: PyDict = g.gcode.get_command_help().into_iter().map(|(k, v)| (k.into(), PyVal::Str(v))).collect();
                wr.send(PyVal::Dict(help))
            }),
        )?;
        let g = gh.clone();
        wh.register_endpoint("gcode/script", Rc::new(move |wr| g.gcode.run_script(&wr.get_str("script", None)?.str())))?;
        let g = gh.clone();
        wh.register_endpoint("gcode/restart", Rc::new(move |_| g.gcode.run_script("restart")))?;
        let g = gh.clone();
        wh.register_endpoint("gcode/firmware_restart", Rc::new(move |_| g.gcode.run_script("firmware_restart")))?;
        let g = gh.clone();
        wh.register_endpoint("gcode/subscribe_output", Rc::new(move |wr| g.handle_subscribe_output(wr)))?;
        Ok(gh)
    }

    pub fn output_callback(&self, msg: &str) {
        let clients: Vec<_> = self.clients.borrow().iter().map(|(k, v)| (*k, v.clone())).collect();
        for (uid, (cconn, template)) in clients {
            if cconn.is_closed() {
                self.clients.borrow_mut().shift_remove(&uid);
                continue;
            }
            let mut tmp = match template {
                PyVal::Dict(d) => d,
                _ => IndexMap::default(),
            };
            let mut params = IndexMap::default();
            params.insert("response".into(), PyVal::Str(msg.to_string()));
            tmp.insert("params".into(), PyVal::Dict(params));
            cconn.send(&PyVal::Dict(tmp));
        }
    }

    pub fn handle_subscribe_output(self: &Rc<Self>, web_request: &WebRequest) -> KResult<()> {
        let template = web_request.get_dict("response_template", Some(PyVal::dict()))?;
        if let Some(cconn) = web_request.get_client_connection() {
            self.clients.borrow_mut().insert(cconn.uid, (cconn, template));
        }
        if !self.is_output_registered.get() {
            let g = self.clone();
            let origin: crate::gcode::CallbackOrigin = (self.clone(), std::any::type_name::<GCodeHelper>());
            self.gcode.register_output_handler_from(Rc::new(move |msg| g.output_callback(msg)), origin);
            self.is_output_registered.set(true);
        }
        Ok(())
    }
}

pub const SUBSCRIPTION_REFRESH_TIME: f64 = 0.25;

/// a subscription dict (shared: python keeps the same dict in the pending
/// query and in the client subscription)
type Subscription = Rc<RefCell<PyDict>>;

enum SendFunc {
    Complete(Rc<ReactorCompletion<PyVal>>),
    Conn(Rc<ClientConnection>),
}

/// One printer object's status in a subscription poll: kaliru's value, or a
/// python plugin's dict as python returned it. Only the fields that changed
/// (python's own != for a python one, as Kalico compares) are converted.
enum Status {
    Val(PyVal),
    #[cfg(feature = "plugins")]
    Py(crate::plugins::py::PyRef),
}

impl Status {
    fn of(po: &dyn PrinterObject, eventtime: f64) -> Status {
        #[cfg(feature = "plugins")]
        if let Some(r) = po.get_status_py(eventtime) {
            // (a get_status that raises reads as {}, as on the converting path)
            return r.map(Status::Py).unwrap_or_else(|_| Status::Val(PyVal::dict()));
        }
        Status::Val(po.get_status(eventtime).unwrap_or_else(PyVal::dict))
    }

    fn keys(&self) -> Vec<String> {
        match self {
            Status::Val(v) => match v.get() {
                PyVal::Dict(d) => d.keys().map(|k| k.to_string()).collect(),
                _ => vec![],
            },
            #[cfg(feature = "plugins")]
            Status::Py(d) => crate::plugins::py::dict_keys(d).unwrap_or_default(),
        }
    }

    /// res[key] != last.get(key)
    fn changed(&self, key: &str, last: Option<&Status>) -> bool {
        match self {
            Status::Val(v) => {
                let none = PyVal::None;
                let a = match v.get() {
                    PyVal::Dict(d) => d.get(key).unwrap_or(&none),
                    _ => &none,
                };
                match last {
                    Some(Status::Val(l)) => match l.get() {
                        PyVal::Dict(l) => !a.py_eq(l.get(key).unwrap_or(&none)),
                        _ => !a.py_eq(&none),
                    },
                    #[cfg(feature = "plugins")]
                    Some(Status::Py(_)) => true,
                    None => !a.py_eq(&none),
                }
            }
            #[cfg(feature = "plugins")]
            Status::Py(d) => {
                use crate::plugins::py;
                let b = match last {
                    Some(Status::Py(l)) => py::dict_get(l, key),
                    Some(Status::Val(_)) => return true,
                    None => Ok(py::py_none()),
                };
                match (py::dict_get(d, key), b) {
                    (Ok(a), Ok(b)) => py::py_ne(&a, &b).unwrap_or(true),
                    _ => true,
                }
            }
        }
    }

    /// (key, res[key]) for the response (None when missing); the key is the
    /// status dict's own (static field names cost no allocation)
    fn entry(&self, key: &str) -> (PyKey, PyVal) {
        match self {
            Status::Val(v) => match v.get() {
                PyVal::Dict(d) => match d.get_key_value(key) {
                    Some((k, v)) => (k.clone(), v.clone()),
                    None => (key.to_string().into(), PyVal::None),
                },
                _ => (key.to_string().into(), PyVal::None),
            },
            #[cfg(feature = "plugins")]
            Status::Py(d) => (
                key.to_string().into(),
                crate::plugins::py::dict_get(d, key)
                    .and_then(|v| crate::plugins::py::to_pyval(&v))
                    .unwrap_or(PyVal::None),
            ),
        }
    }
}

pub struct QueryStatusHelper {
    pub printer: Rc<Printer>,
    clients: RefCell<IndexMap<usize, (Rc<ClientConnection>, Subscription, PyVal)>>,
    pending_queries: RefCell<Vec<(Option<Rc<ClientConnection>>, Subscription, SendFunc, PyVal)>>,
    pub query_timer: RefCell<Option<Timer>>,
    last_query: RefCell<IndexMap<PyKey, Status>>,
}

impl QueryStatusHelper {
    pub fn new(printer: &Rc<Printer>) -> KResult<Rc<Self>> {
        let q = Rc::new(QueryStatusHelper {
            printer: printer.clone(),
            clients: RefCell::new(IndexMap::default()),
            pending_queries: RefCell::new(vec![]),
            query_timer: RefCell::new(None),
            last_query: RefCell::new(IndexMap::default()),
        });
        // Register webhooks
        let wh = printer.lookup::<WebHooks>("webhooks")?;
        let qq = q.clone();
        wh.register_endpoint(
            "objects/list",
            Rc::new(move |wr| {
                let objects: Vec<PyVal> = qq
                    .printer
                    .lookup_objects(None)
                    .into_iter()
                    .filter(|(_, o)| o.has_status())
                    .map(|(n, _)| PyVal::Str(n))
                    .collect();
                let mut d = IndexMap::default();
                d.insert("objects".into(), PyVal::List(objects));
                wr.send(PyVal::Dict(d))
            }),
        )?;
        let qq = q.clone();
        wh.register_endpoint("objects/query", Rc::new(move |wr| qq.handle_query(wr, false)))?;
        let qq = q.clone();
        wh.register_endpoint("objects/subscribe", Rc::new(move |wr| qq.handle_query(wr, true)))?;
        Ok(q)
    }

    pub fn do_query(self: &Rc<Self>, eventtime: f64) -> f64 {
        let last_query = std::mem::take(&mut *self.last_query.borrow_mut());
        let mut query: IndexMap<PyKey, Status> = IndexMap::default();
        let mut msglist = std::mem::take(&mut *self.pending_queries.borrow_mut());
        for (_, (cconn, sub, template)) in self.clients.borrow().iter() {
            msglist.push((Some(cconn.clone()), sub.clone(), SendFunc::Conn(cconn.clone()), template.clone()));
        }
        // Generate get_status() info for each client
        for (cconn, subscription, send_func, template) in msglist {
            let is_query = cconn.is_none();
            if let Some(c) = &cconn {
                if c.is_closed() {
                    self.clients.borrow_mut().shift_remove(&c.uid);
                    continue;
                }
            }
            // Query each requested printer object (borrowed: python iterates
            // the subscription dict; a None field list becomes the keys)
            let mut cquery = IndexMap::default();
            let mut filled: Vec<(PyKey, PyVal)> = vec![];
            {
                let sub = subscription.borrow();
                for (obj_name, req) in sub.iter() {
                    if !query.contains_key(obj_name) {
                        let res = match self.printer.lookup_object_opt(obj_name) {
                            Some(po) if po.has_status() => Status::of(&*po, eventtime),
                            _ => Status::Val(PyVal::dict()),
                        };
                        query.insert(obj_name.clone(), res);
                    }
                    let res = &query[obj_name];
                    let keys_list;
                    let req_items = if *req == PyVal::None {
                        keys_list = PyVal::List(res.keys().into_iter().map(PyVal::Str).collect());
                        if keys_list.items().is_some_and(|i| !i.is_empty()) {
                            filled.push((obj_name.clone(), keys_list.clone()));
                        }
                        &keys_list
                    } else {
                        req
                    };
                    let lres = last_query.get(obj_name);
                    let mut cres = IndexMap::default();
                    for ri in req_items.items().unwrap_or(&[]) {
                        let key = match ri.as_str() {
                            Some(k) => std::borrow::Cow::Borrowed(k),
                            None => std::borrow::Cow::Owned(ri.str()),
                        };
                        if is_query || res.changed(&key, lres) {
                            let (k, v) = res.entry(&key);
                            cres.insert(k, v);
                        }
                    }
                    if !cres.is_empty() || is_query {
                        cquery.insert(obj_name.clone(), PyVal::Dict(cres));
                    }
                }
            }
            if !filled.is_empty() {
                let mut sub = subscription.borrow_mut();
                for (k, v) in filled {
                    sub.insert(k, v);
                }
            }
            // Send data
            if !cquery.is_empty() || is_query {
                let mut tmp = match template {
                    PyVal::Dict(d) => d,
                    _ => IndexMap::default(),
                };
                let params = crate::pydict! {
                    "eventtime" => PyVal::Float(eventtime),
                    "status" => PyVal::Dict(cquery),
                };
                tmp.insert("params".into(), PyVal::Dict(params));
                let msg = PyVal::Dict(tmp);
                match send_func {
                    SendFunc::Complete(c) => c.complete(msg),
                    SendFunc::Conn(c) => c.send(&msg),
                }
            }
        }
        let empty = query.is_empty();
        *self.last_query.borrow_mut() = query;
        if empty {
            // Unregister timer if there are no longer any subscriptions
            if let Some(t) = self.query_timer.borrow_mut().take() {
                self.printer.reactor.unregister_timer(&t);
            }
            return NEVER;
        }
        eventtime + SUBSCRIPTION_REFRESH_TIME
    }

    pub fn handle_query(self: &Rc<Self>, web_request: &WebRequest, is_subscribe: bool) -> KResult<()> {
        let PyVal::Dict(objects) = web_request.get_dict("objects", None)? else { unreachable!() };
        // Validate subscription format
        for v in objects.values() {
            match v {
                PyVal::None => {}
                PyVal::List(items) => {
                    if items.iter().any(|ri| !matches!(ri, PyVal::Str(_))) {
                        return Err(web_error("Invalid argument"));
                    }
                }
                _ => return Err(web_error("Invalid argument")),
            }
        }
        let objects: Subscription = Rc::new(RefCell::new(objects));
        // Add to pending queries
        let cconn = web_request.get_client_connection();
        let template = web_request.get_dict("response_template", Some(PyVal::dict()))?;
        if is_subscribe {
            if let Some(c) = &cconn {
                self.clients.borrow_mut().shift_remove(&c.uid);
            }
        }
        let reactor = self.printer.get_reactor();
        let complete = reactor.completion::<PyVal>();
        self.pending_queries.borrow_mut().push((None, objects.clone(), SendFunc::Complete(complete.clone()), PyVal::dict()));
        // Start timer if needed
        if self.query_timer.borrow().is_none() {
            let q = self.clone();
            let qt = reactor.register_timer(Box::new(move |e| q.do_query(e)), NOW);
            *self.query_timer.borrow_mut() = Some(qt);
        }
        // Wait for data to be queried
        let msg = complete.wait(NEVER, None)?.unwrap_or(PyVal::None);
        let params = match &msg {
            PyVal::Dict(d) => d.get("params").cloned().unwrap_or(PyVal::None),
            _ => PyVal::None,
        };
        web_request.send(params)?;
        if is_subscribe {
            if let Some(c) = cconn {
                self.clients.borrow_mut().insert(c.uid, (c, objects, template));
            }
        }
        Ok(())
    }
}

pub fn add_early_printer_objects(printer: &Rc<Printer>) -> KResult<()> {
    printer.add_object("webhooks", WebHooks::new(printer)?)?;
    GCodeHelper::new(printer)?;
    QueryStatusHelper::new(printer)?;
    Ok(())
}
