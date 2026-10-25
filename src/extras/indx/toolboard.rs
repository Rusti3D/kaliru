// INDX toolboard (port of klippy/extras/indx/toolboard.py)
use super::heater::IndxToolboardHeater;
use super::{compat_config, CVal};
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::homing::Homing;
use crate::extras::stepper_enable::PrinterStepperEnable;
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::mcu::{Mcu, PwmPin};
use crate::msgproto::Val;
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{downcast_rc, Printer, PrinterObject};
use crate::pyfmt::py_float;
use crate::pyval::PyVal;
use crate::toolhead::ToolHead;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

const PIN_ALIASES: [(&str, &str); 30] = [
    ("vin_mon", "PA2"),
    ("led", "PA8"),
    ("endstop", "PA9"),
    ("part_cooling", "PA1"),
    ("part_cooling_tacho", "PA0"),
    ("motor_step", "PB12"),
    ("motor_dir", "PB23"),
    ("motor_enable", "PB4"),
    ("motor_cs", "PA10"),
    ("motor_sclk", "PA5"),
    ("motor_mosi", "PA4"),
    ("motor_miso", "PA7"),
    ("ldc_int", "PB10"),
    ("ldc_clk", "PB11"),
    ("fan0_out", "PA21"),
    ("fan0_tacho", "PA20"),
    ("temp_bracket", "PA11"),
    ("temp_board", "PB8"),
    ("temp_ldc", "PB9"),
    ("i2c0_sda", "PA22"),
    ("i2c0_scl", "PA23"),
    ("encoder_cs", "PB0"),
    ("encoder_sclk", "PB3"),
    ("encoder_mosi", "PB2"),
    ("encoder_miso", "PB1"),
    ("loadcell_cs", "PA18"),
    ("loadcell_sclk", "PA17"),
    ("loadcell_mosi", "PA16"),
    ("loadcell_miso", "PA19"),
    ("loadcell_drdy", "PB22"),
];

pub struct IndxToolboard {
    pub printer: Rc<Printer>,
    pub name: String,
    pub mcu: Rc<Mcu>,
    heater: RefCell<Option<Rc<IndxToolboardHeater>>>,
    dock_measurement: RefCell<Option<Rc<IndxDockMeasurement>>>,
}

impl IndxToolboard {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().to_string();

        let mcu_name = config.get("mcu", Dflt::Required)?;
        let mcu = printer
            .lookup_object_opt(&format!("mcu {}", mcu_name))
            .and_then(downcast_rc::<Mcu>)
            .ok_or_else(|| KError::config(format!("[{}] could not find MCU {}", name, mcu_name)))?;

        let tb = Rc::new(IndxToolboard {
            printer: printer.clone(),
            name: name.clone(),
            mcu: mcu.clone(),
            heater: RefCell::new(None),
            dock_measurement: RefCell::new(None),
        });

        let ppins = printer.lookup::<PinsObject>("pins")?;
        {
            let mut pins = ppins.0.borrow_mut();
            let chip: Rc<dyn PinChip> = Rc::new(IndxToolboardPins { toolboard: Rc::downgrade(&tb) });
            pins.register_chip(&name, chip).map_err(|e| KError::Pins(e.0))?;
            let resolver = pins.get_pin_resolver(&mcu.get_name()).map_err(|e| KError::Pins(e.0))?;
            for (alias, pin) in PIN_ALIASES {
                resolver.alias_pin(alias, pin).map_err(|e| KError::Pins(e.0))?;
            }
        }

        let m = mcu.get_name();
        let sercoms: [(&str, &str, Vec<(&str, String)>); 3] = [
            ("indx_periph", "sercom3", vec![("tx_pin", format!("{}:i2c0_sda", m)), ("clk_pin", format!("{}:i2c0_scl", m))]),
            (
                "indx_encoder",
                "sercom5",
                vec![("tx_pin", format!("{}:encoder_mosi", m)), ("rx_pin", format!("{}:encoder_miso", m)), ("clk_pin", format!("{}:encoder_sclk", m))],
            ),
            (
                "indx_loadcell",
                "sercom1",
                vec![("tx_pin", format!("{}:loadcell_mosi", m)), ("rx_pin", format!("{}:loadcell_miso", m)), ("clk_pin", format!("{}:loadcell_sclk", m))],
            ),
        ];
        for (suffix, sercom, pins) in sercoms {
            let section = format!("samd_sercom {}_{}", name, suffix);
            let mut contents = vec![("sercom", CVal::S(sercom.to_string()))];
            contents.extend(pins.into_iter().map(|(k, v)| (k, CVal::S(v))));
            let sercom_config = compat_config(config, &section, &contents);
            printer.load_object(&sercom_config, &section, true)?;
        }

        *tb.heater.borrow_mut() = Some(IndxToolboardHeater::new(&tb, config)?);
        *tb.dock_measurement.borrow_mut() = Some(IndxDockMeasurement::new(&tb)?);

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        type TCmd = fn(&IndxToolboard, &GCodeCommand) -> KResult<()>;
        let cmds: [(&str, TCmd); 2] =
            [("INDX_LED_FORCE_COLOR", IndxToolboard::cmd_led_force_color), ("INDX_LED_SET_CURRENT", IndxToolboard::cmd_led_set_current)];
        for (cmd, f) in cmds {
            let w = Rc::downgrade(&tb);
            gcode.register_command(
                cmd,
                Some(Rc::new(move |g| match w.upgrade() {
                    Some(t) => f(&t, g),
                    None => Ok(()),
                })),
                false,
                None,
            )?;
        }
        Ok(tb)
    }

    pub fn heater(&self) -> Rc<IndxToolboardHeater> {
        self.heater.borrow().clone().expect("indx heater")
    }

    fn rgb(gcmd: &GCodeCommand) -> KResult<[i64; 3]> {
        let lim = || Lim::new().min(0.0).max(255.0);
        Ok([gcmd.get_int("RED", None, lim())?, gcmd.get_int("GREEN", None, lim())?, gcmd.get_int("BLUE", None, lim())?])
    }

    fn cmd_led_force_color(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = self.mcu.lookup_command("indx_led_force_color r=%c g=%c b=%c force=%c", None)?;
        if gcmd.get_opt("CLEAR").is_none() {
            let [r, g, b] = Self::rgb(gcmd)?;
            cmd.send(&[Val::Int(r), Val::Int(g), Val::Int(b), Val::Int(1)], 0, 0)
        } else {
            cmd.send(&[Val::Int(0), Val::Int(0), Val::Int(0), Val::Int(0)], 0, 0)
        }
    }

    fn cmd_led_set_current(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let cmd = self.mcu.lookup_command("indx_led_set_current r=%c g=%c b=%c", None)?;
        let [r, g, b] = Self::rgb(gcmd)?;
        cmd.send(&[Val::Int(r), Val::Int(g), Val::Int(b)], 0, 0)
    }
}

impl PrinterObject for IndxToolboard {
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let last = self.dock_measurement.borrow().as_ref().and_then(|d| d.last_dock_measurement.borrow().clone()).unwrap_or(PyVal::None);
        let mut d = IndexMap::new();
        d.insert("last_dock_measurement".to_string(), last);
        Some(PyVal::Dict(d))
    }
    fn teardown(&self) {
        if let Some(h) = self.heater.borrow_mut().take() {
            h.teardown();
        }
        *self.dock_measurement.borrow_mut() = None;
    }
}

struct IndxToolboardPins {
    toolboard: Weak<IndxToolboard>,
}

impl PinChip for IndxToolboardPins {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let pin = pin_params.borrow().pin.clone();
        if pin_type == "pwm" && pin == "heater" {
            let tb = self.toolboard.upgrade().ok_or("INDX toolboard is gone")?;
            let heater: Rc<dyn PwmPin> = tb.heater();
            return Ok(Rc::new(heater));
        }
        Err(format!("unknown INDX virtual pin {}", pin))
    }
}

const CMD_INDX_DOCK_MEASURE_HELP: &str = "Measure INDX dock X/Y position by energizing XY motors and homing.";

struct IndxDockMeasurement {
    printer: Rc<Printer>,
    last_dock_measurement: RefCell<Option<PyVal>>,
}

impl IndxDockMeasurement {
    fn new(toolboard: &Rc<IndxToolboard>) -> KResult<Rc<Self>> {
        let printer = toolboard.printer.clone();
        let d = Rc::new(IndxDockMeasurement { printer: printer.clone(), last_dock_measurement: RefCell::new(None) });
        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let w = Rc::downgrade(&d);
        gcode.register_command(
            "INDX_DOCK_MEASURE",
            Some(Rc::new(move |g| match w.upgrade() {
                Some(d) => d.cmd_indx_dock_measure(g),
                None => Ok(()),
            })),
            false,
            Some(CMD_INDX_DOCK_MEASURE_HELP),
        )?;
        Ok(d)
    }

    fn cmd_indx_dock_measure(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let x_first = gcmd.get_int("X_FIRST", Some(0), Lim::new().min(0.0).max(1.0))?;
        let axes = if x_first != 0 { [0, 1] } else { [1, 0] };
        let retreat: Option<Vec<f64>> = gcmd.get_or("RETREAT", "0,0").split(',').map(|v| py_float(v.trim())).collect();
        let Some(retreat) = retreat else {
            return Err(gcmd.error("Unable to parse RETREAT parameter. Should be of format '<x>,<y>'."));
        };
        let retreat_speed = gcmd.get_float("RETREAT_SPEED", Some(10.0), Lim::new().above(0.0))?;
        let (x, y) = match self.measure(&axes, &retreat, retreat_speed) {
            Ok(m) => m,
            Err(e) => {
                if e.is_command_error() {
                    self.printer.lookup::<PrinterStepperEnable>("stepper_enable")?.motor_off()?;
                }
                return Err(e);
            }
        };
        gcmd.respond_info(&format!("INDX_DOCK_MEASURE: measured dock position X={:.3} Y={:.3}", x, y), true);
        Ok(())
    }

    fn enable_motors(&self, toolhead: &ToolHead) -> KResult<()> {
        let Some(stepper_enable) = self.printer.lookup_object_opt("stepper_enable").and_then(downcast_rc::<PrinterStepperEnable>) else {
            return Ok(());
        };
        let print_time = toolhead.get_last_move_time()?;
        for stepper in toolhead.get_kinematics().get_steppers() {
            if !(stepper.is_active_axis('x') || stepper.is_active_axis('y')) {
                continue;
            }
            stepper_enable.lookup_enable(&stepper.get_name(false))?.motor_enable(print_time)?;
        }
        toolhead.dwell(0.100)
    }

    fn get_kinematic_position(toolhead: &ToolHead) -> Vec<f64> {
        let kin = toolhead.get_kinematics();
        let stepper_positions: IndexMap<String, f64> =
            kin.get_steppers().iter().map(|s| (s.get_name(false), s.get_mcu_position(None) as f64 * s.get_step_dist())).collect();
        kin.calc_position(&stepper_positions)
    }

    fn measure(&self, axes: &[usize; 2], retreat: &[f64], retreat_speed: f64) -> KResult<(f64, f64)> {
        let toolhead = self.printer.lookup::<ToolHead>("toolhead")?;
        let kin = toolhead.get_kinematics();
        self.enable_motors(&toolhead)?;

        let kinematic_position_before = Self::get_kinematic_position(&toolhead);

        let r = |i: usize| retreat.get(i).copied().ok_or_else(|| KError::Other("list index out of range".into()));
        if r(0)? != 0.0 || r(1)? != 0.0 {
            let curpos = toolhead.get_position();
            toolhead.set_position(&[0.0, 0.0, curpos[2], curpos[3]], "xy")?;
            let coord: Vec<Option<f64>> = retreat.iter().map(|v| Some(*v)).collect();
            toolhead.manual_move(&coord, retreat_speed)?;
            toolhead.wait_moves()?;
            kin.clear_homing_state("xy");
        }

        let mut start_position = [0.0, 0.0];

        for &axis in axes {
            let homing_state = Homing::new(&self.printer)?;
            homing_state.set_axes(vec![axis]);
            kin.home(&homing_state)?;
            let final_position = toolhead.get_position();
            let kinematic_position_after = Self::get_kinematic_position(&toolhead);
            start_position[axis] = final_position[axis] + kinematic_position_before[axis] - kinematic_position_after[axis];
        }

        let mut m = IndexMap::new();
        m.insert("position".to_string(), PyVal::List(start_position.iter().map(|v| PyVal::Float(*v)).collect()));
        m.insert("x".to_string(), PyVal::Float(start_position[0]));
        m.insert("y".to_string(), PyVal::Float(start_position[1]));
        m.insert("axis_order".to_string(), PyVal::Str(axes.iter().map(|a| ['X', 'Y'][*a]).collect()));
        *self.last_dock_measurement.borrow_mut() = Some(PyVal::Dict(m));
        Ok((start_position[0], start_position[1]))
    }
}
