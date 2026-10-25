// Run a shell command via gcode (port of klippy/extras/gcode_shell_command.py)
//
// Copyright (C) 2019  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::py_shlex_split;
use crate::pyglob::{expanduser, expandvars};
use std::cell::{Cell, RefCell};
use std::os::fd::{FromRawFd, OwnedFd, RawFd};
use std::rc::Rc;

const CMD_RUN_SHELL_COMMAND_HELP: &str = "Run a linux shell command";

pub struct ShellCommand {
    name: String,
    printer: Rc<Printer>,
    gcode: Rc<GCodeDispatch>,
    command: Vec<String>,
    timeout: f64,
    verbose: bool,
    proc_fd: Cell<Option<RawFd>>,
    partial_output: RefCell<String>,
}

impl ShellCommand {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let printer = config.get_printer();
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let cmd = config.get("command", Dflt::Required)?;
        let cmd = expandvars(&expanduser(&cmd));
        let command = py_shlex_split(&cmd, "").map_err(KError::Other)?;
        let timeout = config.getfloat("timeout", 2.0, Lim::new().above(0.0))?;
        let verbose = config.getboolean("verbose", true)?;
        let sc = Rc::new(ShellCommand {
            name: name.clone(),
            printer,
            gcode: gcode.clone(),
            command,
            timeout,
            verbose,
            proc_fd: Cell::new(None),
            partial_output: RefCell::new(String::new()),
        });
        let s = sc.clone();
        gcode.register_mux_command(
            "RUN_SHELL_COMMAND",
            "CMD",
            Some(&name),
            Rc::new(move |g| s.cmd_run_shell_command(g)),
            Some(CMD_RUN_SHELL_COMMAND_HELP),
        )?;
        Ok(sc)
    }

    fn process_output(&self, _eventtime: f64) {
        let Some(fd) = self.proc_fd.get() else { return };
        let mut buf = [0u8; 4096];
        // SAFETY: reading the pipe this command owns
        let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        let read = if n < 0 { vec![] } else { buf[..n as usize].to_vec() };
        let mut data = format!("{}{}", self.partial_output.borrow(), String::from_utf8_lossy(&read));
        if !data.contains('\n') {
            *self.partial_output.borrow_mut() = data;
            return;
        } else if !data.ends_with('\n') {
            let split = data.rfind('\n').unwrap() + 1;
            *self.partial_output.borrow_mut() = data[split..].to_string();
            data.truncate(split);
        } else {
            self.partial_output.borrow_mut().clear();
        }
        self.gcode.respond_info(&data, true);
    }

    fn cmd_run_shell_command(self: &Rc<Self>, gcmd: &GCodeCommand) -> KResult<()> {
        let gcode_params = gcmd.get_or("PARAMS", "");
        let gcode_params: Vec<String> = py_shlex_split(&gcode_params, "")
            .map_err(KError::Other)?
            .iter()
            .map(|p| expandvars(&expanduser(p)))
            .collect();
        let reactor = self.printer.get_reactor();
        let mut argv = self.command.clone();
        argv.extend(gcode_params);
        // stdout=PIPE, stderr=STDOUT
        let spawned = (|| -> std::io::Result<(std::process::Child, RawFd)> {
            let mut fds = [0 as RawFd; 2];
            // SAFETY: plain pipe(2)
            if unsafe { libc::pipe2(fds.as_mut_ptr(), libc::O_CLOEXEC) } != 0 {
                return Err(std::io::Error::last_os_error());
            }
            // SAFETY: fresh descriptors from pipe2
            let (r, w) = unsafe { (OwnedFd::from_raw_fd(fds[0]), OwnedFd::from_raw_fd(fds[1])) };
            let w2 = w.try_clone()?;
            let program = argv.first().ok_or_else(|| std::io::Error::other("list index out of range"))?;
            let child = std::process::Command::new(program).args(&argv[1..]).stdout(w).stderr(w2).spawn()?;
            let rfd = std::os::fd::IntoRawFd::into_raw_fd(r);
            Ok((child, rfd))
        })();
        let (mut proc, rfd) = match spawned {
            Ok(v) => v,
            Err(e) => {
                logging::exception_err(&format!("shell_command: Command {{{}}} failed", self.name), &KError::Other(e.to_string()));
                return Err(KError::command(format!("Error running command {{{}}}", self.name)));
            }
        };
        let mut hdl = None;
        if self.verbose {
            self.proc_fd.set(Some(rfd));
            self.gcode.respond_info(&format!("Running Command {{{}}}...:", self.name), true);
            let s = self.clone();
            hdl = Some(reactor.register_fd(rfd, Box::new(move |e| s.process_output(e)), None));
        }
        let mut eventtime = reactor.monotonic();
        let endtime = eventtime + self.timeout;
        let mut complete = false;
        while eventtime < endtime {
            eventtime = reactor.pause(eventtime + 0.05)?;
            if let Ok(Some(_)) = proc.try_wait() {
                complete = true;
                break;
            }
        }
        if !complete {
            // proc.terminate()
            // SAFETY: signalling our child
            unsafe { libc::kill(proc.id() as libc::pid_t, libc::SIGTERM) };
        }
        if self.verbose {
            let partial = std::mem::take(&mut *self.partial_output.borrow_mut());
            if !partial.is_empty() {
                self.gcode.respond_info(&partial, true);
            }
            let msg = if complete {
                format!("Command {{{}}} finished\n", self.name)
            } else {
                format!("Command {{{}}} timed out", self.name)
            };
            self.gcode.respond_info(&msg, true);
            if let Some(h) = hdl {
                reactor.unregister_fd(&h);
            }
            self.proc_fd.set(None);
        }
        // (python keeps the pipe open until the Popen object is collected)
        // SAFETY: our read end
        unsafe { libc::close(rfd) };
        Ok(())
    }
}

impl PrinterObject for ShellCommand {
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(ShellCommand::new(config)?)
}
