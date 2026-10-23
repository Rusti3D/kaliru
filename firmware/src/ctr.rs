// Compile time requests (ctr.h, command.h, sched.h). The decl_*! macros
// expand to nothing: build/scan.rs reads them from the source text, as
// the C build reads the .compile_time_request sections, and generates
// the tables included below.
use core::cell::UnsafeCell;

/// firmware global state. The firmware is single threaded (irqs are
/// disabled around shared updates, as in the C code); `get()` hands out
/// short lived references like the C globals are used.
#[repr(transparent)]
pub struct Global<T>(UnsafeCell<T>);

unsafe impl<T> Sync for Global<T> {}

impl<T> Global<T> {
    pub const fn new(v: T) -> Self {
        Global(UnsafeCell::new(v))
    }
    pub const fn ptr(&self) -> *mut T {
        self.0.get()
    }
    #[allow(clippy::mut_from_ref)]
    pub fn get(&self) -> &mut T {
        unsafe { &mut *self.0.get() }
    }
}

/// an entry of the vector table that DECL_ARMCM_IRQ builds (armcm_boot.h):
/// the initial stack pointer, then the handlers
#[repr(C)]
pub union Vector {
    pub handler: unsafe extern "C" fn(),
    pub stack: *const u32,
}
unsafe impl Sync for Vector {}

/// a C table whose `#if` blocks add entries: the concatenation of its
/// parts (not all empty)
pub const fn concat<T: Copy, const N: usize>(parts: &[&[T]]) -> [T; N] {
    let mut first = 0;
    while parts[first].is_empty() {
        first += 1;
    }
    let mut out = [parts[first][0]; N];
    let (mut n, mut p) = (0, 0);
    while p < parts.len() {
        let mut i = 0;
        while i < parts[p].len() {
            out[n] = parts[p][i];
            n += 1;
            i += 1;
        }
        p += 1;
    }
    assert!(n == N);
    out
}

/// the total length of the parts of a concat()
pub const fn concat_len<T>(parts: &[&[T]]) -> usize {
    let (mut n, mut p) = (0, 0);
    while p < parts.len() {
        n += parts[p].len();
        p += 1;
    }
    n
}

/// a pin set at startup (initial_pins.h)
pub struct InitialPin {
    pub pin: i32,
    pub flags: u8,
}
pub const IP_OUT_HIGH: u8 = 1;

/// string equality usable in constants (the generated lookups)
pub const fn streq(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        if a[i] != b[i] {
            return false;
        }
        i += 1;
    }
    true
}

#[allow(unused_imports)]
use crate::command::{
    CommandEncoder, CommandParser, PT_buffer, PT_byte, PT_int16, PT_int32, PT_progmem_buffer, PT_string, PT_uint16, PT_uint32,
};
use crate::sched::R;

include!(concat!(env!("OUT_DIR"), "/ctr_generated.rs"));

// Declare a function to run when the specified command is received
#[macro_export]
macro_rules! decl_command {
    ($($t:tt)*) => {};
}
// Declare an init function (called at firmware startup)
#[macro_export]
macro_rules! decl_init {
    ($($t:tt)*) => {};
}
// Declare a task function (called periodically during normal runtime)
#[macro_export]
macro_rules! decl_task {
    ($($t:tt)*) => {};
}
// Declare a shutdown function (called on an emergency stop)
#[macro_export]
macro_rules! decl_shutdown {
    ($($t:tt)*) => {};
}
// Declare a constant exported to the host
#[macro_export]
macro_rules! decl_constant {
    ($($t:tt)*) => {};
}
#[macro_export]
macro_rules! decl_constant_str {
    ($($t:tt)*) => {};
}
// Declare an enumeration
#[macro_export]
macro_rules! decl_enumeration {
    ($($t:tt)*) => {};
}
#[macro_export]
macro_rules! decl_enumeration_range {
    ($($t:tt)*) => {};
}
#[macro_export]
macro_rules! decl_initial_pins {
    ($($t:tt)*) => {};
}
// Declare an IRQ handler (an entry of the generated VectorTable)
#[macro_export]
macro_rules! decl_armcm_irq {
    ($($t:tt)*) => {};
}
// Statically declare an IRQ handler and run-time enable it
#[macro_export]
macro_rules! armcm_enable_irq {
    ($func:path, $num:expr, $prio:expr) => {{
        $crate::generic::cmsis::NVIC_SetPriority($num, $prio);
        $crate::generic::cmsis::NVIC_EnableIRQ($num);
    }};
}

// Declare a message type and transmit it.
#[macro_export]
macro_rules! sendf {
    ($fmt:expr $(, $arg:expr)* $(,)?) => {{
        const IDX: usize = $crate::ctr::ctr_lookup_encoder($fmt);
        $crate::command::command_sendf(&$crate::ctr::ENCODERS[IDX], &[$($crate::command::SendArg::from($arg)),*])
    }};
}

// Send an output message (and declare a static message type for it)
#[macro_export]
macro_rules! output {
    ($fmt:expr $(, $arg:expr)* $(,)?) => {{
        const IDX: usize = $crate::ctr::ctr_lookup_output($fmt);
        $crate::command::command_sendf(&$crate::ctr::ENCODERS[IDX], &[$($crate::command::SendArg::from($arg)),*])
    }};
}

// The id of a static string (also declares it)
#[macro_export]
macro_rules! static_string {
    ($msg:expr) => {{
        const ID: u8 = $crate::ctr::ctr_lookup_static_string($msg);
        ID
    }};
}

// Shut down the machine (also declares a static string to transmit)
#[macro_export]
macro_rules! shutdown {
    ($msg:expr) => {
        return Err($crate::sched::sched_shutdown($crate::static_string!($msg)))
    };
}
#[macro_export]
macro_rules! try_shutdown {
    ($msg:expr) => {
        $crate::sched::sched_try_shutdown($crate::static_string!($msg))?
    };
}
