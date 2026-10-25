// Typed klippy events: printer.send_event() / register_event_handler() with
// the event's name and argument types in one declaration, checked by the
// compiler at both ends.
//
// The name stays the dispatch key: handlers run in registration order,
// python plugins' handlers among them, and python plugins send and receive
// the same events. A typed handler takes its arguments as values; a python
// plugin's arguments are converted as python would see them (a float where
// a float is expected), anything else is python's TypeError.
use crate::error::{KError, KResult};
use std::any::Any;

/// An event's name
pub trait Event {
    const NAME: &'static str;
}

/// A value an event carries, taken from the `&dyn Any` the dispatcher passes
pub trait EventArg: Sized + 'static {
    fn from_arg(a: &dyn Any) -> Option<Self>;
}

/// the argument types sent as rust values (an Rc, a String, ...)
#[macro_export]
macro_rules! event_arg {
    ($($t:ty),* $(,)?) => {$(
        impl $crate::events::EventArg for $t {
            #[inline]
            fn from_arg(a: &dyn std::any::Any) -> Option<Self> {
                a.downcast_ref::<$t>().cloned()
            }
        }
    )*};
}

impl EventArg for f64 {
    fn from_arg(a: &dyn Any) -> Option<Self> {
        if let Some(v) = a.downcast_ref::<f64>() {
            return Some(*v);
        }
        #[cfg(feature = "plugins")]
        if let Some(v) = a.downcast_ref::<crate::plugins::py::PyRef>() {
            return v.as_f64().ok();
        }
        None
    }
}

event_arg!(String, crate::pyval::PyVal);

/// argument `n` of an event, or python's TypeError
#[inline]
pub fn arg<T: EventArg>(name: &str, args: &[&dyn Any], n: usize) -> KResult<T> {
    args.get(n).and_then(|a| T::from_arg(*a)).ok_or_else(|| {
        KError::Py("TypeError".into(), format!("{}: argument {} has an unexpected type", name, n))
    })
}

/// Declares events: `events! { pub IdlePrinting = "idle_timeout:printing" (print_time: f64); }`
/// gives `IdlePrinting::emit(&printer, print_time)` and
/// `IdlePrinting::on(&printer, |print_time| ...)`.
#[macro_export]
macro_rules! events {
    ($( $(#[$m:meta])* $vis:vis $name:ident = $s:literal ( $($arg:ident : $ty:ty),* $(,)? ); )*) => {$(
        $(#[$m])*
        $vis struct $name;
        impl $crate::events::Event for $name {
            const NAME: &'static str = $s;
        }
        impl $name {
            /// the event's name (for code that registers several by name)
            #[allow(dead_code)]
            pub const NAME: &'static str = $s;
            /// send_event(): every handler, in registration order
            #[allow(clippy::too_many_arguments, dead_code)]
            pub fn emit(printer: &$crate::printer::Printer $(, $arg: &$ty)*) -> $crate::error::KResult<()> {
                printer.send_event($s, &[$($arg as &dyn std::any::Any),*])
            }
            /// register_event_handler()
            #[allow(dead_code)]
            pub fn on(printer: &$crate::printer::Printer, f: impl Fn($($ty),*) -> $crate::error::KResult<()> + 'static) {
                printer.register_event_handler($s, std::rc::Rc::new(move |args: &[&dyn std::any::Any]| {
                    let _ = args;
                    #[allow(unused_mut, unused_variables)]
                    let mut n = 0usize;
                    $( let $arg: $ty = $crate::events::arg($s, args, { n += 1; n - 1 })?; )*
                    f($($arg),*)
                }));
            }
        }
    )*};
}

event_arg!(
    std::rc::Rc<crate::extras::homing::Homing>,
    std::rc::Rc<crate::extras::homing::HomingMove>,
    Vec<crate::stepper::RailRef>,
    std::rc::Rc<crate::stepper::McuStepper>,
    std::rc::Rc<crate::extras::load_cell::load_cell::LoadCell>,
);

// Kalico's printer events (the names python plugins use too)
events! {
    /// all MCUs identified (before klippy:connect)
    pub KlippyMcuIdentify = "klippy:mcu_identify" ();
    /// the printer is connecting: handlers configure their MCU objects
    pub KlippyConnect = "klippy:connect" ();
    /// the printer is ready
    pub KlippyReady = "klippy:ready" ();
    /// the printer entered the shutdown state
    pub KlippyShutdown = "klippy:shutdown" ();
    /// after klippy:shutdown, with the shutdown message and its details
    pub KlippyAnalyzeShutdown = "klippy:analyze_shutdown" (msg: String, details: crate::pyval::PyVal);
    /// the host is disconnecting from the MCUs
    pub KlippyDisconnect = "klippy:disconnect" ();
    /// FIRMWARE_RESTART
    pub KlippyFirmwareRestart = "klippy:firmware_restart" ();
    /// an MCU error while connecting
    pub KlippyNotifyMcuError = "klippy:notify_mcu_error" (msg: String, details: crate::pyval::PyVal);
    /// RESTART / FIRMWARE_RESTART requested, at this print time
    pub GcodeRequestRestart = "gcode:request_restart" (print_time: f64);
    /// a g-code command failed
    pub GcodeCommandError = "gcode:command_error" ();
    /// an unknown g-code command
    pub GcodeUnknownCommand = "gcode:unknown_command" (cmd: String);
    /// homing: the rails are about to home
    pub HomingHomeRailsBegin = "homing:home_rails_begin" (homing: std::rc::Rc<crate::extras::homing::Homing>, rails: Vec<crate::stepper::RailRef>);
    /// homing: the rails homed
    pub HomingHomeRailsEnd = "homing:home_rails_end" (homing: std::rc::Rc<crate::extras::homing::Homing>, rails: Vec<crate::stepper::RailRef>);
    /// a homing / probing move starts
    pub HomingMoveBegin = "homing:homing_move_begin" (hmove: std::rc::Rc<crate::extras::homing::HomingMove>);
    /// a homing / probing move ended
    pub HomingMoveEnd = "homing:homing_move_end" (hmove: std::rc::Rc<crate::extras::homing::HomingMove>);
    /// idle_timeout: the printer went idle
    pub IdleTimeoutIdle = "idle_timeout:idle" (print_time: f64);
    /// idle_timeout: the printer is printing
    pub IdleTimeoutPrinting = "idle_timeout:printing" (print_time: f64);
    /// idle_timeout: the printer is ready (not printing)
    pub IdleTimeoutReady = "idle_timeout:ready" (print_time: f64);
    /// a stepper's direction inversion changed
    pub StepperSetDirInverted = "stepper:set_dir_inverted" (stepper: std::rc::Rc<crate::stepper::McuStepper>);
    /// a stepper's MCU position was resynchronized
    pub StepperSyncMcuPosition = "stepper:sync_mcu_position" (stepper: std::rc::Rc<crate::stepper::McuStepper>);
    /// the motors were turned off
    pub StepperEnableMotorOff = "stepper_enable:motor_off" ();
    /// the toolhead synchronized its print time
    pub ToolheadSyncPrintTime = "toolhead:sync_print_time" (curtime: f64, est_print_time: f64, print_time: f64);
    /// the toolhead position was set
    pub ToolheadSetPosition = "toolhead:set_position" ();
    /// a manual move was queued
    pub ToolheadManualMove = "toolhead:manual_move" ();
    /// the toolhead's extra axes changed
    pub ToolheadUpdateExtraAxes = "toolhead:update_extra_axes" ();
    /// the active extruder changed
    pub ExtruderActivateExtruder = "extruder:activate_extruder" ();
    /// a load cell was calibrated
    pub LoadCellCalibrate = "load_cell:calibrate" (load_cell: std::rc::Rc<crate::extras::load_cell::load_cell::LoadCell>);
    /// a load cell was tared
    pub LoadCellTare = "load_cell:tare" (load_cell: std::rc::Rc<crate::extras::load_cell::load_cell::LoadCell>);
    /// print_stats state changes
    pub PrintStatsReset = "print_stats:reset" ();
    pub PrintStatsStartPrinting = "print_stats:start_printing" ();
    pub PrintStatsPausedPrinting = "print_stats:paused_printing" ();
    pub PrintStatsCompletePrinting = "print_stats:complete_printing" ();
    pub PrintStatsCancelledPrinting = "print_stats:cancelled_printing" ();
    pub PrintStatsErrorPrinting = "print_stats:error_printing" ();
    /// virtual_sdcard: a file was loaded / reset
    pub VirtualSdcardLoadFile = "virtual_sdcard:load_file" ();
    pub VirtualSdcardResetFile = "virtual_sdcard:reset_file" ();
    /// trad_rack lane / load / unload events
    pub TradRackForcedActiveLane = "trad_rack:forced_active_lane" ();
    pub TradRackResetActiveLane = "trad_rack:reset_active_lane" ();
    pub TradRackLoadStarted = "trad_rack:load_started" ();
    pub TradRackLoadComplete = "trad_rack:load_complete" ();
    pub TradRackUnloadStarted = "trad_rack:unload_started" ();
    pub TradRackUnloadComplete = "trad_rack:unload_complete" ();
    pub TradRackSyncedToExtruder = "trad_rack:synced_to_extruder" ();
    pub TradRackUnsyncingFromExtruder = "trad_rack:unsyncing_from_extruder" ();
}
