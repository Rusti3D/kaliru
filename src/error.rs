// The python exception classes klippy code raises and catches by type, as
// one error enum (no python counterpart file). Display is python's str(e).
//
//   Config     configfile.error (configparser.Error)  printer.config_error
//   Pins       pins.error
//   Protocol   msgproto.error
//   Enumeration msgproto.enumeration_error (a msgproto.error subclass)
//   Serial     serialhdl.error
//   Mcu        mcu.error
//   Command    gcode.CommandError                     printer.command_error
//              (and its subclasses that are caught as CommandError:
//               WebRequestError, homing.CommandError, ...)
//   WaitInterruption  printer.WaitInterruption (a CommandError subclass)
//   Reactor    reactor.ReactorError
//   Other      any other exception (str(e) only)
use std::fmt;

#[derive(Clone, Debug)]
pub enum KError {
    Config(String),
    Pins(String),
    Protocol(String),
    Enumeration { msg: String, enum_name: String, value: String },
    Serial(String),
    Mcu(String),
    Command(String),
    WaitInterruption(String),
    Reactor(String),
    Other(String),
}

impl KError {
    pub fn message(&self) -> &str {
        match self {
            KError::Config(m)
            | KError::Pins(m)
            | KError::Protocol(m)
            | KError::Enumeration { msg: m, .. }
            | KError::Serial(m)
            | KError::Mcu(m)
            | KError::Command(m)
            | KError::WaitInterruption(m)
            | KError::Reactor(m)
            | KError::Other(m) => m,
        }
    }

    /// isinstance(e, gcode.CommandError)
    pub fn is_command_error(&self) -> bool {
        matches!(self, KError::Command(_) | KError::WaitInterruption(_))
    }

    /// the python exception class as traceback.format_exception_only()
    /// names it ("Other" errors carry no class: "Exception")
    pub fn py_class(&self) -> &'static str {
        match self {
            KError::Config(_) => "configparser.Error",
            KError::Pins(_) => "klippy.pins.error",
            KError::Protocol(_) => "klippy.msgproto.error",
            KError::Enumeration { .. } => "klippy.msgproto.enumeration_error",
            KError::Serial(_) => "klippy.serialhdl.error",
            KError::Mcu(_) => "klippy.mcu.error",
            KError::Command(_) => "klippy.gcode.CommandError",
            KError::WaitInterruption(_) => "klippy.printer.WaitInterruption",
            KError::Reactor(_) => "klippy.reactor.ReactorError",
            KError::Other(_) => "Exception",
        }
    }

    /// isinstance(e, msgproto.error)
    pub fn is_protocol_error(&self) -> bool {
        matches!(self, KError::Protocol(_) | KError::Enumeration { .. })
    }

    pub fn config(msg: impl Into<String>) -> Self {
        KError::Config(msg.into())
    }

    pub fn command(msg: impl Into<String>) -> Self {
        KError::Command(msg.into())
    }
}

impl fmt::Display for KError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(self.message())
    }
}

impl From<crate::configparser::Error> for KError {
    fn from(e: crate::configparser::Error) -> Self {
        KError::Config(e.message)
    }
}

impl From<crate::pins::Error> for KError {
    fn from(e: crate::pins::Error) -> Self {
        KError::Pins(e.0)
    }
}

impl From<crate::msgproto::Error> for KError {
    fn from(e: crate::msgproto::Error) -> Self {
        match e.enum_params {
            Some((enum_name, value)) => KError::Enumeration { msg: e.msg, enum_name, value },
            None => KError::Protocol(e.msg),
        }
    }
}

impl From<crate::reactor::ReactorError> for KError {
    fn from(e: crate::reactor::ReactorError) -> Self {
        KError::Reactor(e.0)
    }
}

pub type KResult<T> = Result<T, KError>;
