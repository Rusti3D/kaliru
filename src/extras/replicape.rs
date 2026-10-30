// Code to configure miscellaneous chips
// (port of klippy/extras/replicape.py)
//
// Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::bus::McuSpi;
use crate::logging;
use crate::mcu::{get_printer_mcu, CommandWrapper, DigitalOutPin, Mcu, McuPwm, PwmPin};
use crate::msgproto::Val;
use crate::pins::{setup_digital_out_pin, PinChip, PinParams, PinsObject, SharedPinParams};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::{py_max, py_min};
use crate::reactor::NOW;
use crate::IndexMap;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const REPLICAPE_MAX_CURRENT: f64 = 3.84;
const REPLICAPE_PCA9685_BUS: i64 = 2;
const REPLICAPE_PCA9685_ADDRESS: i64 = 0x70;
const REPLICAPE_PCA9685_CYCLE_TIME: f64 = 0.001;
const PIN_MIN_TIME: f64 = 0.100;

pub struct Pca9685Pwm {
    pub replicape: Rc<Replicape>,
    pub channel: i64,
    pub mcu: Rc<Mcu>,
    pub bus: i64,
    pub address: i64,
    pub cycle_time: f64,
    pub max_duration: Cell<f64>,
    pub oid: Cell<Option<i64>>,
    pub invert: i64,
    pub start_value: Cell<f64>,
    pub shutdown_value: Cell<f64>,
    pub is_enable: Cell<bool>,
    pub last_clock: Cell<i64>,
    pub pwm_max: Cell<f64>,
    pub set_cmd: RefCell<Option<CommandWrapper>>,
}

impl Pca9685Pwm {
    fn new(replicape: &Rc<Replicape>, channel: i64, pin_type: &str, pin_params: &PinParams) -> Result<Rc<Self>, String> {
        if pin_type != "digital_out" && pin_type != "pwm" {
            return Err("Pin type not supported on replicape".into());
        }
        let mcu = replicape.host_mcu.clone();
        let invert = pin_params.invert;
        let p = Rc::new(Pca9685Pwm {
            replicape: replicape.clone(),
            channel,
            mcu: mcu.clone(),
            bus: REPLICAPE_PCA9685_BUS,
            address: REPLICAPE_PCA9685_ADDRESS,
            cycle_time: REPLICAPE_PCA9685_CYCLE_TIME,
            max_duration: Cell::new(2.0),
            oid: Cell::new(None),
            invert,
            start_value: Cell::new(invert as f64),
            shutdown_value: Cell::new(invert as f64),
            is_enable: Cell::new(invert != 0),
            last_clock: Cell::new(0),
            pwm_max: Cell::new(0.0),
            set_cmd: RefCell::new(None),
        });
        let pp = p.clone();
        mcu.register_config_callback(Rc::new(move || pp.build_config()));
        Ok(p)
    }

    pub fn build_config(&self) -> KResult<()> {
        self.pwm_max.set(self.mcu.get_constant_float("PCA9685_MAX")?);
        let cycle_ticks = self.mcu.seconds_to_clock(self.cycle_time);
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid();
        self.oid.set(Some(oid));
        self.mcu.add_config_cmd(
            &format!(
                "config_pca9685 oid={} bus={} addr={} channel={} cycle_ticks={} value={} default_value={} max_duration={}",
                oid,
                self.bus,
                self.address,
                self.channel,
                cycle_ticks,
                (self.start_value.get() * self.pwm_max.get()) as i64,
                (self.shutdown_value.get() * self.pwm_max.get()) as i64,
                self.mcu.seconds_to_clock(self.max_duration.get())
            ),
            false,
            false,
        );
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.set_cmd.borrow_mut() = Some(self.mcu.lookup_command("queue_pca9685_out oid=%c clock=%u value=%hu", Some(cmd_queue))?);
        Ok(())
    }
}

impl PwmPin for Pca9685Pwm {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()> {
        if hardware_pwm {
            return Err(KError::Pins("pca9685 does not support hardware_pwm parameter".into()));
        }
        if cycle_time != self.cycle_time {
            logging::info(&format!("Ignoring pca9685 cycle time of {:.6} (using {:.6})", cycle_time, self.cycle_time));
        }
        Ok(())
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let (mut start_value, mut shutdown_value) = (start_value, shutdown_value);
        if self.invert != 0 {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        self.start_value.set(py_max(0.0, py_min(1.0, start_value)));
        self.shutdown_value.set(py_max(0.0, py_min(1.0, shutdown_value)));
        self.replicape.note_pwm_start_value(self.channel, self.start_value.get(), self.shutdown_value.get());
        self.is_enable.set(self.start_value.get() != 0.0);
    }

    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        let mut value = value;
        if self.invert != 0 {
            value = 1.0 - value;
        }
        let value = (py_max(0.0, py_min(1.0, value)) * self.pwm_max.get() + 0.5) as i64;
        let is_enable = value != 0;
        if is_enable != self.is_enable.get() {
            self.is_enable.set(is_enable);
            let (r, channel) = (self.replicape.clone(), self.channel);
            self.mcu.get_printer().get_reactor().register_async_callback_local(
                move |_| {
                    if let Err(e) = r.note_pwm_enable(print_time, channel, is_enable) {
                        logging::exception_err("replicape pwm enable", &e);
                    }
                },
                NOW,
            );
        }
        let cmd = self.set_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        cmd.send(&[Val::Int(self.oid.get().unwrap_or(0)), Val::Int(clock), Val::Int(value)], self.last_clock.get() as u64, clock as u64)?;
        self.last_clock.set(clock);
        Ok(())
    }
}

impl DigitalOutPin for Pca9685Pwm {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        PwmPin::setup_start_value(self, start_value as i64 as f64, shutdown_value as i64 as f64)
    }
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        if value {
            self.set_pwm(print_time, 1.0)
        } else {
            self.set_pwm(print_time, 0.0)
        }
    }
}

pub struct ReplicapeDACEnable {
    pub mcu: Rc<Mcu>,
    pub value: f64,
    pub pwm: Rc<Pca9685Pwm>,
}

impl ReplicapeDACEnable {
    fn new(replicape: &Rc<Replicape>, channel: i64, pin_type: &str, pin_params: &PinParams) -> Result<Rc<Self>, String> {
        if pin_type != "digital_out" {
            return Err("Replicape virtual enable pin must be digital_out".into());
        }
        if pin_params.invert != 0 {
            return Err("Replicape virtual enable pin can not be inverted".into());
        }
        let value = replicape.stepper_dacs[&channel];
        let pwm = Pca9685Pwm::new(replicape, channel, pin_type, pin_params)?;
        Ok(Rc::new(ReplicapeDACEnable { mcu: replicape.host_mcu.clone(), value, pwm }))
    }
}

impl DigitalOutPin for ReplicapeDACEnable {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        PwmPin::setup_max_duration(&*self.pwm, max_duration)
    }
    /// python's ReplicapeDACEnable has no setup_start_value (only stepper
    /// enable lines, which never call it, use these pins)
    fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        if value {
            self.pwm.set_pwm(print_time, self.value)
        } else {
            self.pwm.set_pwm(print_time, 0.0)
        }
    }
}

/// SERVO_PINS[name]: (pwm pin suffix, reserved pins)
fn servo_pins(name: &str) -> Option<(&'static str, &'static str, &'static str)> {
    match name {
        "servo0" => Some(("/pwm0", "gpio0_30", "gpio1_18")), // P9_11, P9_14
        "servo1" => Some(("/pwm1", "gpio3_17", "gpio1_19")), // P9_28, P9_16
        _ => None,
    }
}

pub struct ServoPwm {
    pub mcu_pwm: Rc<McuPwm>,
}

impl ServoPwm {
    fn new(replicape: &Rc<Replicape>, pin_params: &PinParams) -> Result<Rc<Self>, String> {
        let config_name = pin_params.pin.clone();
        let mut pwmchip = "pwmchip0".to_string();
        if !replicape.host_mcu.is_fileoutput() {
            // Determine the pwmchip number for the servo channels
            // /sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm
            // should be stable on the beagle bone black.
            // It contains only a "pwmchipX" directory. The entry in
            // /sys/class/pwm/ used by the Linux MCU should be a symlink
            // to this directory.
            let found = std::fs::read_dir("/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/").ok().and_then(|d| {
                d.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().to_string()).find(|pc| pc.starts_with("pwmchip"))
            });
            pwmchip = found.ok_or_else(|| "Replicape unable to determine pwmchip".to_string())?;
        }
        let (pwm_pin, resv1, resv2) = servo_pins(&config_name).expect("servo pin");
        let params = PinParams {
            chip: pin_params.chip.clone(),
            chip_name: pin_params.chip_name.clone(),
            pin: pwmchip + pwm_pin,
            invert: pin_params.invert,
            pullup: pin_params.pullup,
            share_type: pin_params.share_type.clone(),
            class: pin_params.class.clone(),
        };
        // Setup actual pwm pin using linux hardware pwm on host
        let mcu_pwm = PinChip::setup_pin(&*replicape.host_mcu, "pwm", &Rc::new(RefCell::new(params)))?
            .downcast::<McuPwm>()
            .map_err(|_| "host mcu pwm pin of an unexpected type".to_string())?;
        // Reserve pins to warn user of conflicts
        let pru_mcu = replicape.mcu_pwm_enable.get_mcu();
        let printer = pru_mcu.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins").map_err(|e| e.to_string())?;
        let pins = ppins.0.borrow_mut();
        let pin_resolver = pins.get_pin_resolver(pru_mcu.get_name()).map_err(|e| e.0)?;
        let mut pin_resolver = pin_resolver.borrow_mut();
        pin_resolver.reserve_pin(resv1, &config_name).map_err(|e| e.0)?;
        pin_resolver.reserve_pin(resv2, &config_name).map_err(|e| e.0)?;
        Ok(Rc::new(ServoPwm { mcu_pwm }))
    }
}

impl PwmPin for ServoPwm {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_pwm.get_mcu()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.mcu_pwm.setup_max_duration(max_duration)
    }
    fn setup_cycle_time(&self, cycle_time: f64, _hardware_pwm: bool) -> KResult<()> {
        self.mcu_pwm.setup_cycle_time(cycle_time, true);
        Ok(())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        self.mcu_pwm.setup_start_value(start_value, shutdown_value)
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        self.mcu_pwm.set_pwm(print_time, value)
    }
}

/// ReplicapeStepConfig: the microstep mode shift register bits
const REPLICAPE_STEP_CONFIG: [(&str, Option<i64>); 10] = [
    ("disable", None),
    ("1", Some((1 << 7) | (1 << 5))),
    ("2", Some((1 << 7) | (1 << 5) | (1 << 6))),
    ("spread2", Some(1 << 5)),
    ("4", Some((1 << 7) | (1 << 5) | (1 << 4))),
    ("16", Some((1 << 7) | (1 << 5) | (1 << 6) | (1 << 4))),
    ("spread4", Some((1 << 5) | (1 << 4))),
    ("spread16", Some(1 << 7)),
    ("stealth4", Some((1 << 7) | (1 << 6))),
    ("stealth16", Some(0)),
];

#[derive(Clone, Copy)]
pub enum PinClass {
    Pca9685,
    DacEnable,
}

pub struct Replicape {
    me: Weak<Replicape>,
    pub host_mcu: Rc<Mcu>,
    pub mcu_pwm_enable: Rc<dyn DigitalOutPin>,
    pub mcu_pwm_start_value: Cell<bool>,
    pub mcu_pwm_shutdown_value: Cell<bool>,
    pub last_pwm_enable_time: Cell<f64>,
    pub pins: IndexMap<String, (PinClass, i64)>,
    pub stepper_dacs: IndexMap<i64, f64>,
    pub enabled_channels: RefCell<IndexMap<i64, bool>>,
    pub sr_disabled: RefCell<Vec<u8>>,
    pub sr_enabled: RefCell<Vec<u8>>,
    pub sr_spi: Rc<McuSpi>,
}

impl Replicape {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer: Rc<Printer> = config.get_printer();
        config.getchoice("revision", &[("B3", "B3")], Dflt::Required)?;
        let host_mcu = get_printer_mcu(&printer, &config.get("host_mcu", Dflt::Required)?)?;
        // Setup enable pin
        let enable_pin = config.get("enable_pin", "!gpio0_20")?;
        let mcu_pwm_enable = setup_digital_out_pin(&printer, &enable_pin)?;
        mcu_pwm_enable.setup_max_duration(0.0);
        // Setup power pins
        let mut pins: IndexMap<String, (PinClass, i64)> = [
            ("power_e", 5),
            ("power_h", 3),
            ("power_hotbed", 4),
            ("power_fan0", 7),
            ("power_fan1", 8),
            ("power_fan2", 9),
            ("power_fan3", 10),
        ]
        .iter()
        .map(|(n, c)| (n.to_string(), (PinClass::Pca9685, *c)))
        .collect();
        // Setup stepper config
        let mut stepper_dacs = IndexMap::default();
        let mut shift_registers: Vec<i64> = vec![1, 0, 0, 1, 1];
        for (port, name) in "xyzeh".chars().enumerate() {
            let prefix = format!("stepper_{}_", name);
            let Some(mut sc) = config.getchoice(&format!("{}microstep_mode", prefix), &REPLICAPE_STEP_CONFIG, "disable")? else {
                continue;
            };
            sc |= shift_registers[port];
            if config.getboolean(&format!("{}chopper_off_time_high", prefix), false)? {
                sc |= 1 << 3;
            }
            if config.getboolean(&format!("{}chopper_hysteresis_high", prefix), false)? {
                sc |= 1 << 2;
            }
            if config.getboolean(&format!("{}chopper_blank_time_high", prefix), true)? {
                sc |= 1 << 1;
            }
            shift_registers[port] = sc;
            let channel = port as i64 + 11;
            let cur = config.getfloat(&format!("{}current", prefix), Dflt::Required, Lim::new().above(0.0).max(REPLICAPE_MAX_CURRENT))?;
            stepper_dacs.insert(channel, cur / REPLICAPE_MAX_CURRENT);
            pins.insert(format!("{}enable", prefix), (PinClass::DacEnable, channel));
        }
        let enabled_channels: IndexMap<i64, bool> = pins.values().map(|(_, ch)| (*ch, false)).collect();
        let bytes = |sr: &[i64]| sr.iter().rev().map(|v| *v as u8).collect::<Vec<u8>>();
        let sr_disabled = bytes(&shift_registers);
        if (0..3).any(|i| stepper_dacs.contains_key(&(11 + i))) {
            // Enable xyz steppers
            shift_registers[0] &= !1;
        }
        if (3..5).any(|i| stepper_dacs.contains_key(&(11 + i))) {
            // Enable eh steppers
            shift_registers[3] &= !1;
        }
        if config.getboolean("standstill_power_down", false)? && !stepper_dacs.is_empty() {
            shift_registers[4] &= !1;
        }
        let sr_enabled = bytes(&shift_registers);
        let mut sr_spi_bus = "spidev1.1";
        if !host_mcu.is_fileoutput() && std::path::Path::new("/sys/devices/platform/ocp/481a0000.spi/spi_master/spi2").exists() {
            sr_spi_bus = "spidev2.1";
        }
        let sr_spi = McuSpi::new(&host_mcu, Some(sr_spi_bus.to_string()), None, 0, 50000000, None, false);
        sr_spi.setup_shutdown_msg(&sr_disabled);
        sr_spi.spi_send(&sr_disabled, 0, 0)?;
        let r = Rc::new_cyclic(|me| Replicape {
            me: me.clone(),
            host_mcu,
            mcu_pwm_enable,
            mcu_pwm_start_value: Cell::new(false),
            mcu_pwm_shutdown_value: Cell::new(false),
            last_pwm_enable_time: Cell::new(0.0),
            pins,
            stepper_dacs,
            enabled_channels: RefCell::new(enabled_channels),
            sr_disabled: RefCell::new(sr_disabled),
            sr_enabled: RefCell::new(sr_enabled),
            sr_spi,
        });
        let ppins = printer.lookup::<PinsObject>("pins")?;
        ppins.0.borrow_mut().register_chip("replicape", r.clone()).map_err(|e| KError::Pins(e.0))?;
        Ok(r)
    }

    pub fn note_pwm_start_value(&self, channel: i64, start_value: f64, shutdown_value: f64) {
        self.mcu_pwm_start_value.set(self.mcu_pwm_start_value.get() | (start_value != 0.0));
        self.mcu_pwm_shutdown_value.set(self.mcu_pwm_shutdown_value.get() | (shutdown_value != 0.0));
        self.mcu_pwm_enable.setup_start_value(self.mcu_pwm_start_value.get(), self.mcu_pwm_shutdown_value.get());
        self.enabled_channels.borrow_mut().insert(channel, start_value != 0.0);
    }

    pub fn note_pwm_enable(&self, print_time: f64, channel: i64, is_enable: bool) -> KResult<()> {
        self.enabled_channels.borrow_mut().insert(channel, is_enable);
        // Check if need to set the pca9685 enable pin
        let pe_time = py_max(print_time, self.last_pwm_enable_time.get() + PIN_MIN_TIME);
        let on_channels = self.enabled_channels.borrow().values().filter(|e| **e).count();
        if on_channels == 0 {
            self.mcu_pwm_enable.set_digital(pe_time, false)?;
            self.last_pwm_enable_time.set(pe_time);
        } else if is_enable && on_channels == 1 {
            self.mcu_pwm_enable.set_digital(pe_time, true)?;
            self.last_pwm_enable_time.set(pe_time);
        }
        // Check if need to set the stepper enable lines
        if !self.stepper_dacs.contains_key(&channel) {
            return Ok(());
        }
        let on_dacs = {
            let enabled = self.enabled_channels.borrow();
            self.stepper_dacs.keys().filter(|c| enabled[*c]).count()
        };
        let sr = if on_dacs == 0 {
            self.sr_disabled.borrow().clone()
        } else if is_enable && on_dacs == 1 {
            self.sr_enabled.borrow().clone()
        } else {
            return Ok(());
        };
        let clock = self.host_mcu.print_time_to_clock(print_time);
        self.sr_spi.spi_send(&sr, clock as u64, clock as u64)
    }
}

impl PinChip for Replicape {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        let me = self.me.upgrade().expect("replicape");
        let p = pin_params.borrow();
        let pin = p.pin.clone();
        if let Some((pclass, channel)) = self.pins.get(&pin).copied() {
            return match pclass {
                PinClass::Pca9685 => {
                    let pwm = Pca9685Pwm::new(&me, channel, pin_type, &p)?;
                    Ok(match pin_type {
                        "pwm" => Rc::new(pwm as Rc<dyn PwmPin>) as Rc<dyn Any>,
                        _ => Rc::new(pwm as Rc<dyn DigitalOutPin>) as Rc<dyn Any>,
                    })
                }
                PinClass::DacEnable => Ok(Rc::new(ReplicapeDACEnable::new(&me, channel, pin_type, &p)? as Rc<dyn DigitalOutPin>)),
            };
        } else if pin == "servo0" || pin == "servo1" {
            // enable servo pins via shift registers
            let index = if pin == "servo0" { 3 } else { 2 };
            self.sr_enabled.borrow_mut()[index] |= 1;
            self.sr_disabled.borrow_mut()[index] |= 1;
            let sr = self.sr_disabled.borrow().clone();
            self.sr_spi.spi_send(&sr, 0, 0).map_err(|e| e.to_string())?;
            return Ok(Rc::new(ServoPwm::new(&me, &p)? as Rc<dyn PwmPin>));
        }
        Err(format!("Unknown replicape pin {}", pin))
    }
}

impl PrinterObject for Replicape {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Replicape::new(config)?)
}
