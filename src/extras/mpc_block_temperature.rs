// MPC block temperature sensor (port of klippy/extras/mpc_block_temperature.py)
//
// The wrapper also serves mpc_ambient_temperature.py, which is the same code
// reporting the ambient state instead.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt};
use crate::error::{KError, KResult};
use crate::extras::control_mpc::ControlMPC;
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, ControlRef, Heater, MpcSensor, PrinterHeaters, TempCallback, TempSensor, AMBIENT_TEMP};
use crate::gcode::GCodeDispatch;
use crate::printer::{none_object, PObj, Printer};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

#[derive(Clone, Copy, PartialEq)]
pub enum Kind {
    Block,
    Ambient,
}

pub struct MpcTempWrapper {
    kind: Kind,
    printer: Rc<Printer>,
    name: String,
    heater: RefCell<Option<Rc<Heater>>>,
    heater_name: String,
    temperature_callback: RefCell<Option<TempCallback>>,
    temp: Cell<f64>,
    min_temp: Cell<f64>,
    max_temp: Cell<f64>,
    ignore: bool,
    echo_limits_to_console: bool,
}

impl MpcTempWrapper {
    pub fn new(config: &ConfigWrapper, kind: Kind) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let heater_name = config.get("heater_name", Dflt::Required)?;
        let ignore = config.getboolean("ignore_limits", false)? || get_danger_options().temp_ignore_limits;
        let echo_limits_to_console = config.getboolean("echo_limits_to_console", false)?;
        let w = Rc::new(MpcTempWrapper {
            kind,
            printer: printer.clone(),
            name: config.get_name().split_whitespace().last().unwrap_or("").to_string(),
            heater: RefCell::new(None),
            heater_name,
            temperature_callback: RefCell::new(None),
            temp: Cell::new(0.0),
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            ignore,
            echo_limits_to_console,
        });
        let ww = w.clone();
        printer.register_event_handler("klippy:ready", Rc::new(move |_| ww.handle_ready()));
        Ok(w)
    }

    fn handle_ready(self: &Rc<Self>) -> KResult<()> {
        let pheaters = self.printer.lookup::<PrinterHeaters>("heaters")?;
        let heater = pheaters.lookup_heater(&self.heater_name)?;
        heater.add_mpc_sensor(self.clone());
        *self.heater.borrow_mut() = Some(heater);
        Ok(())
    }

    fn heater(&self) -> KResult<Rc<Heater>> {
        self.heater.borrow().clone().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'sensor'".into()))
    }
}

impl MpcSensor for MpcTempWrapper {
    fn process_temp_update(&self, control: &ControlRef, read_time: f64) -> KResult<()> {
        let heater = self.heater()?;
        let attr = if self.kind == Kind::Block { "state_block_temp" } else { "state_ambient_temp" };
        let temp = match control.get_type() {
            "mpc" | "tuning" => {
                let hc = heater.get_control();
                let mpc = hc.downcast_ref::<ControlMPC>().ok_or_else(|| {
                    KError::Other(format!("'{}' object has no attribute '{}'", control_class(hc.get_type()), attr))
                })?;
                if self.kind == Kind::Block {
                    mpc.state_block_temp.get()
                } else {
                    mpc.state_ambient_temp.get()
                }
            }
            _ if self.kind == Kind::Block => heater.smoothed_temp(),
            _ => AMBIENT_TEMP,
        };
        self.temp.set(temp);
        let (min_temp, max_temp) = (self.min_temp.get(), self.max_temp.get());
        if !heater.pwm_mcu().non_critical_disconnected.get() && (temp < min_temp || temp > max_temp) {
            let label = if self.kind == Kind::Block { "MPC Heater Block" } else { "Ambient MPC" };
            let msg = format!("{} {}\nTemperature {:.1} outside range of {:.1}-{:.1}", label, self.name, temp, min_temp, max_temp);
            if !self.ignore {
                self.printer.invoke_shutdown(&msg, None);
            } else if self.echo_limits_to_console {
                self.printer.lookup::<GCodeDispatch>("gcode")?.respond_error(&msg);
            }
        }
        let cb = self.temperature_callback.borrow().clone();
        match cb {
            Some(cb) => cb(read_time, temp),
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }
}

/// the python class name of a control type
fn control_class(t: &str) -> &'static str {
    match t {
        "tuning" => "TuningControl",
        "mpc" => "ControlMPC",
        _ => "object",
    }
}

impl TempSensor for MpcTempWrapper {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }

    fn setup_callback(&self, cb: TempCallback) {
        *self.temperature_callback.borrow_mut() = Some(cb);
    }

    /// self.heater.sensor.get_report_time_delta() (python raises before
    /// klippy:ready; 0.0 here)
    fn get_report_time_delta(&self) -> f64 {
        self.heater.borrow().as_ref().map(|h| h.sensor().get_report_time_delta()).unwrap_or(0.0)
    }

}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(
        "mpc_block_temperature",
        Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(MpcTempWrapper::new(config, Kind::Block)?) }),
    );
    Ok(none_object())
}
