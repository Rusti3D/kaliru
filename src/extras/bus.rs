// Helper code for SPI and I2C bus communication (port of klippy/extras/bus.py)
//
// Copyright (C) 2018,2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::chelper::serialqueue::CommandQueue;
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::mcu::{get_printer_mcu, CommandQueryWrapper, CommandWrapper, Mcu};
use crate::msgproto::{Params, Val};
use crate::pins::{PinsObject, SharedPinParams};
use std::cell::RefCell;
use std::rc::Rc;

pub fn same_rc<A: ?Sized, B: ?Sized>(a: &Rc<A>, b: &Rc<B>) -> bool {
    Rc::as_ptr(a) as *const () == Rc::as_ptr(b) as *const ()
}

fn pins_err(e: crate::pins::Error) -> KError {
    KError::Pins(e.0)
}

fn hex(data: &[u8]) -> String {
    data.iter().map(|x| format!("{:02x}", x)).collect()
}

pub fn resolve_bus_name(mcu: &Mcu, param: &str, bus: Option<&str>) -> KResult<String> {
    // Find enumerations for the given bus
    let enumerations = mcu.get_enumerations();
    let Some(enums) = enumerations.get(param).or_else(|| enumerations.get("bus")).map(|t| &t.enums) else {
        return Ok(bus.unwrap_or("0").to_string());
    };
    // Verify bus is a valid enumeration
    let printer = mcu.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let mcu_name = mcu.get_name().to_string();
    let bus = match bus {
        Some(b) => b.to_string(),
        None => {
            // rev_enums = {v: k}: the last name of a value wins
            let rev0 = enums.iter().filter(|(_, v)| **v == 0).last().map(|(k, _)| k.clone());
            match rev0 {
                Some(b) => b,
                None => return Err(KError::Pins(format!("Must specify {} on mcu '{}'", param, mcu_name))),
            }
        }
    };
    if !enums.contains_key(&bus) {
        return Err(KError::Pins(format!("Unknown {} '{}'", param, bus)));
    }
    // Check for reserved bus pins
    let constants = mcu.get_constants();
    let reserve_pins = constants.get(&format!("BUS_PINS_{}", bus)).and_then(|v| v.as_str().map(|s| s.to_string()));
    let mut pins = ppins.0.borrow_mut();
    let pin_resolver = pins.get_pin_resolver(&mcu_name).map_err(pins_err)?;
    if let Some(reserve_pins) = reserve_pins {
        for pin in reserve_pins.split(',') {
            pin_resolver.reserve_pin(pin, &bus).map_err(pins_err)?;
        }
    }
    Ok(bus)
}

// ----------------------------------------------------------------------
// SPI
// ----------------------------------------------------------------------

// Helper code for working with devices connected to an MCU via an SPI bus
pub struct McuSpi {
    mcu: Rc<Mcu>,
    bus: Option<String>,
    speed: i64,
    oid: i64,
    config_fmt: RefCell<String>,
    config_fmt_ticks: Option<String>,
    cmd_queue: CommandQueue,
    spi_send_cmd: RefCell<Option<CommandWrapper>>,
    spi_transfer_cmd: RefCell<Option<CommandQueryWrapper>>,
}

impl McuSpi {
    pub fn new(
        mcu: &Rc<Mcu>,
        bus: Option<String>,
        pin: Option<&str>,
        mode: i64,
        speed: i64,
        sw_pins: Option<[String; 3]>,
        cs_active_high: bool,
    ) -> Rc<Self> {
        // Config SPI object (set all CS pins high before spi_set_bus commands)
        let oid = mcu.create_oid();
        match pin {
            None => mcu.add_config_cmd(&format!("config_spi_without_cs oid={}", oid), false, false),
            Some(pin) => mcu.add_config_cmd(&format!("config_spi oid={} pin={} cs_active_high={}", oid, pin, cs_active_high as i64), false, false),
        }
        // Generate SPI bus config message
        let (config_fmt, config_fmt_ticks) = match &sw_pins {
            Some(p) => (
                format!("spi_set_software_bus oid={} miso_pin={} mosi_pin={} sclk_pin={} mode={} rate={}", oid, p[0], p[1], p[2], mode, speed),
                Some(format!("spi_set_sw_bus oid={} miso_pin={} mosi_pin={} sclk_pin={} mode={} pulse_ticks=%d", oid, p[0], p[1], p[2], mode)),
            ),
            None => (format!("spi_set_bus oid={} spi_bus=%s mode={} rate={}", oid, mode, speed), None),
        };
        let cmd_queue = mcu.alloc_command_queue();
        let s = Rc::new(McuSpi {
            mcu: mcu.clone(),
            bus,
            speed,
            oid,
            config_fmt: RefCell::new(config_fmt),
            config_fmt_ticks,
            cmd_queue,
            spi_send_cmd: RefCell::new(None),
            spi_transfer_cmd: RefCell::new(None),
        });
        let ss = s.clone();
        mcu.register_config_callback(Rc::new(move || ss.build_config()));
        s
    }

    pub fn setup_shutdown_msg(&self, shutdown_seq: &[u8]) {
        let shutdown_msg = hex(shutdown_seq);
        self.mcu.add_config_cmd(
            &format!("config_spi_shutdown oid={} spi_oid={} shutdown_msg={}", self.mcu.create_oid(), self.oid, shutdown_msg),
            false,
            false,
        );
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn get_command_queue(&self) -> CommandQueue {
        self.cmd_queue.clone()
    }

    fn build_config(&self) -> KResult<()> {
        if self.config_fmt.borrow().contains('%') {
            let bus = resolve_bus_name(&self.mcu, "spi_bus", self.bus.as_deref())?;
            let f = self.config_fmt.borrow().replace("%s", &bus);
            *self.config_fmt.borrow_mut() = f;
        }
        if let Some(ticks_fmt) = &self.config_fmt_ticks {
            if self.mcu.try_lookup_command("spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u").is_some() {
                let pulse_ticks = self.mcu.seconds_to_clock(1.0 / self.speed as f64);
                *self.config_fmt.borrow_mut() = ticks_fmt.replace("%d", &pulse_ticks.to_string());
            }
        }
        self.mcu.add_config_cmd(&self.config_fmt.borrow(), false, false);
        *self.spi_send_cmd.borrow_mut() = Some(self.mcu.lookup_command("spi_send oid=%c data=%*s", Some(self.cmd_queue.clone()))?);
        *self.spi_transfer_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "spi_transfer oid=%c data=%*s",
            "spi_transfer_response oid=%c response=%*s",
            Some(self.oid),
            Some(self.cmd_queue.clone()),
            false,
        )?);
        Ok(())
    }

    pub fn spi_send(&self, data: &[u8], minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = self.spi_send_cmd.borrow();
        let Some(cmd) = cmd.as_ref() else {
            // Send setup message via mcu initialization
            self.mcu.add_config_cmd(&format!("spi_send oid={} data={}", self.oid, hex(data)), true, false);
            return Ok(());
        };
        cmd.send(&[Val::Int(self.oid), Val::Bytes(data.to_vec())], minclock, reqclock)
    }

    pub fn spi_send_wait_ack(&self, data: &[u8], minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = self.spi_send_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send_wait_ack'".into()))?;
        cmd.send_wait_ack(&[Val::Int(self.oid), Val::Bytes(data.to_vec())], minclock, reqclock)
    }

    pub fn spi_transfer(&self, data: &[u8], minclock: u64, reqclock: u64) -> KResult<Params> {
        let cmd = self.spi_transfer_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        cmd.send(&[Val::Int(self.oid), Val::Bytes(data.to_vec())], minclock, reqclock, true)
    }

    pub fn spi_transfer_with_preface(&self, preface_data: &[u8], data: &[u8], minclock: u64, reqclock: u64) -> KResult<Params> {
        let tcmd = self.spi_transfer_cmd.borrow();
        let tcmd = tcmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send_with_preface'".into()))?;
        let scmd = self.spi_send_cmd.borrow();
        let scmd = scmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'cmd'".into()))?;
        tcmd.send_with_preface(
            scmd,
            &[Val::Int(self.oid), Val::Bytes(preface_data.to_vec())],
            &[Val::Int(self.oid), Val::Bytes(data.to_vec())],
            minclock,
            reqclock,
            true,
        )
    }
}

/// the mcu a pin is on (pin_params["chip"])
pub fn pin_mcu(config: &ConfigWrapper, pin_params: &SharedPinParams) -> KResult<Rc<Mcu>> {
    let chip_name = pin_params.borrow().chip_name.clone();
    get_printer_mcu(&config.get_printer(), &chip_name)
}

// Helper to setup an spi bus from settings in a config section
pub fn mcu_spi_from_config(
    config: &ConfigWrapper,
    mode: i64,
    pin_option: &str,
    default_speed: i64,
    share_type: Option<&str>,
    cs_active_high: bool,
) -> KResult<Rc<McuSpi>> {
    // Determine pin from config
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let cs_pin = config.get(pin_option, Dflt::Required)?;
    let cs_pin_params = ppins.0.borrow_mut().lookup_pin(&cs_pin, false, false, share_type).map_err(pins_err)?;
    let mut pin = Some(cs_pin_params.borrow().pin.clone());
    if pin.as_deref() == Some("None") {
        ppins.0.borrow_mut().reset_pin_sharing(&cs_pin_params.borrow());
        pin = None;
    }
    // Load bus parameters
    let mcu = pin_mcu(config, &cs_pin_params)?;
    let speed = config.getint("spi_speed", default_speed, Lim::new().min(100000))?;
    let (bus, sw_pins) = if config.get_opt("spi_software_sclk_pin")?.is_some() {
        let mut sw = vec![];
        for name in ["miso", "mosi", "sclk"] {
            let opt = format!("spi_software_{}_pin", name);
            let desc = config.get(&opt, Dflt::Required)?;
            let pp = ppins.0.borrow_mut().lookup_pin(&desc, false, false, Some(&opt)).map_err(pins_err)?;
            sw.push(pp);
        }
        for pp in &sw {
            if !same_rc(&pp.borrow().chip, &mcu) {
                return Err(KError::Pins(format!("{}: spi pins must be on same mcu", config.get_name())));
            }
        }
        let p: Vec<String> = sw.iter().map(|pp| pp.borrow().pin.clone()).collect();
        (None, Some([p[0].clone(), p[1].clone(), p[2].clone()]))
    } else {
        (config.get_opt("spi_bus")?, None)
    };
    // Create MCU_SPI object
    Ok(McuSpi::new(&mcu, bus, pin.as_deref(), mode, speed, sw_pins, cs_active_high))
}

// ----------------------------------------------------------------------
// I2C
// ----------------------------------------------------------------------

// Helper code for working with devices connected to an MCU via an I2C bus
pub struct McuI2c {
    mcu: Rc<Mcu>,
    bus: Option<String>,
    i2c_address: i64,
    oid: i64,
    speed: i64,
    config_fmt: RefCell<String>,
    config_fmt_ticks: Option<String>,
    cmd_queue: CommandQueue,
    i2c_write_cmd: RefCell<Option<CommandWrapper>>,
    i2c_read_cmd: RefCell<Option<CommandQueryWrapper>>,
    // backward support i2c_write inside the init section
    to_write: RefCell<Vec<Vec<u8>>>,
}

impl McuI2c {
    pub fn new(mcu: &Rc<Mcu>, bus: Option<String>, addr: i64, speed: i64, sw_pins: Option<[String; 2]>) -> Rc<Self> {
        let oid = mcu.create_oid();
        mcu.add_config_cmd(&format!("config_i2c oid={}", oid), false, false);
        // Generate I2C bus config message
        let (config_fmt, config_fmt_ticks) = match &sw_pins {
            Some(p) => (
                format!("i2c_set_software_bus oid={} scl_pin={} sda_pin={} rate={} address={}", oid, p[0], p[1], speed, addr),
                Some(format!("i2c_set_sw_bus oid={} scl_pin={} sda_pin={} pulse_ticks=%d address={}", oid, p[0], p[1], addr)),
            ),
            None => (format!("i2c_set_bus oid={} i2c_bus=%s rate={} address={}", oid, speed, addr), None),
        };
        let cmd_queue = mcu.alloc_command_queue();
        let s = Rc::new(McuI2c {
            mcu: mcu.clone(),
            bus,
            i2c_address: addr,
            oid,
            speed,
            config_fmt: RefCell::new(config_fmt),
            config_fmt_ticks,
            cmd_queue,
            i2c_write_cmd: RefCell::new(None),
            i2c_read_cmd: RefCell::new(None),
            to_write: RefCell::new(vec![]),
        });
        let ss = s.clone();
        mcu.register_config_callback(Rc::new(move || ss.build_config()));
        let printer = mcu.get_printer();
        let ss = s.clone();
        crate::events::KlippyConnect::on(&printer, move || {
            let to_write = ss.to_write.borrow().clone();
            for data in to_write {
                ss.i2c_write(&data, 0, 0)?;
            }
            Ok(())
        });
        s
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn get_i2c_address(&self) -> i64 {
        self.i2c_address
    }

    pub fn get_command_queue(&self) -> CommandQueue {
        self.cmd_queue.clone()
    }

    fn build_config(&self) -> KResult<()> {
        if self.config_fmt.borrow().contains('%') {
            let bus = resolve_bus_name(&self.mcu, "i2c_bus", self.bus.as_deref())?;
            let f = self.config_fmt.borrow().replace("%s", &bus);
            *self.config_fmt.borrow_mut() = f;
        }
        if let Some(ticks_fmt) = &self.config_fmt_ticks {
            if self.mcu.try_lookup_command("i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u").is_some() {
                let pulse_ticks = self.mcu.seconds_to_clock(1.0 / self.speed as f64 / 2.0);
                *self.config_fmt.borrow_mut() = ticks_fmt.replace("%d", &pulse_ticks.to_string());
            }
        }
        self.mcu.add_config_cmd(&self.config_fmt.borrow(), false, false);
        *self.i2c_write_cmd.borrow_mut() = Some(self.mcu.lookup_command("i2c_write oid=%c data=%*s", Some(self.cmd_queue.clone()))?);
        *self.i2c_read_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "i2c_read oid=%c reg=%*s read_len=%u",
            "i2c_read_response oid=%c response=%*s",
            Some(self.oid),
            Some(self.cmd_queue.clone()),
            false,
        )?);
        Ok(())
    }

    pub fn i2c_write_noack(&self, data: &[u8], minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = self.i2c_write_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        cmd.send(&[Val::Int(self.oid), Val::Bytes(data.to_vec())], minclock, reqclock)
    }

    pub fn i2c_write(&self, data: &[u8], minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = self.i2c_write_cmd.borrow();
        let Some(cmd) = cmd.as_ref() else {
            self.to_write.borrow_mut().push(data.to_vec());
            return Ok(());
        };
        cmd.send_wait_ack(&[Val::Int(self.oid), Val::Bytes(data.to_vec())], minclock, reqclock)
    }

    pub fn i2c_read(&self, write: &[u8], read_len: i64, retry: bool) -> KResult<Params> {
        let cmd = self.i2c_read_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        // python passes retry as minclock (send(data, retry))
        cmd.send(&[Val::Int(self.oid), Val::Bytes(write.to_vec()), Val::Int(read_len)], retry as u64, 0, true)
    }
}

pub fn mcu_i2c_from_config(config: &ConfigWrapper, default_addr: Option<i64>, default_speed: i64) -> KResult<Rc<McuI2c>> {
    // Load bus parameters
    let printer = config.get_printer();
    let i2c_mcu = get_printer_mcu(&printer, &config.get("i2c_mcu", "mcu")?)?;
    let speed = config.getint("i2c_speed", default_speed, Lim::new().min(100000))?;
    let addr = match default_addr {
        None => config.getint("i2c_address", Dflt::Required, Lim::new().min(0).max(127))?,
        Some(d) => config.getint("i2c_address", d, Lim::new().min(0).max(127))?,
    };
    // Determine pin from config
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let (bus, sw_pins) = if config.get_opt("i2c_software_scl_pin")?.is_some() {
        let mut sw = vec![];
        for name in ["scl", "sda"] {
            let opt = format!("i2c_software_{}_pin", name);
            let desc = config.get(&opt, Dflt::Required)?;
            sw.push(ppins.0.borrow_mut().lookup_pin(&desc, false, false, Some(&opt)).map_err(pins_err)?);
        }
        for pp in &sw {
            if !same_rc(&pp.borrow().chip, &i2c_mcu) {
                return Err(KError::Pins(format!("{}: i2c pins must be on same mcu", config.get_name())));
            }
        }
        let p: Vec<String> = sw.iter().map(|pp| pp.borrow().pin.clone()).collect();
        (None, Some([p[0].clone(), p[1].clone()]))
    } else {
        (config.get_opt("i2c_bus")?, None)
    };
    // Create MCU_I2C object
    Ok(McuI2c::new(&i2c_mcu, bus, addr, speed, sw_pins))
}

// ----------------------------------------------------------------------
// Bus synchronized digital outputs
// ----------------------------------------------------------------------

// Helper code for a gpio that updates on a cmd_queue
pub struct McuBusDigitalOut {
    mcu: Rc<Mcu>,
    oid: i64,
    cmd_queue: CommandQueue,
    update_pin_cmd: RefCell<Option<CommandWrapper>>,
}

impl McuBusDigitalOut {
    pub fn new(mcu: &Rc<Mcu>, pin_desc: &str, cmd_queue: Option<CommandQueue>, value: i64) -> KResult<Rc<Self>> {
        let oid = mcu.create_oid();
        let printer = mcu.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let pin_params = ppins.0.borrow_mut().lookup_pin(pin_desc, false, false, None).map_err(pins_err)?;
        if !same_rc(&pin_params.borrow().chip, mcu) {
            return Err(KError::Pins(format!("Pin {} must be on mcu {}", pin_desc, mcu.get_name())));
        }
        mcu.add_config_cmd(
            &format!(
                "config_digital_out oid={} pin={} value={} default_value={} max_duration={}",
                oid,
                pin_params.borrow().pin,
                value,
                value,
                0
            ),
            false,
            false,
        );
        let cmd_queue = cmd_queue.unwrap_or_else(|| mcu.alloc_command_queue());
        let d = Rc::new(McuBusDigitalOut { mcu: mcu.clone(), oid, cmd_queue, update_pin_cmd: RefCell::new(None) });
        let dd = d.clone();
        mcu.register_config_callback(Rc::new(move || {
            *dd.update_pin_cmd.borrow_mut() =
                Some(dd.mcu.lookup_command("update_digital_out oid=%c value=%c", Some(dd.cmd_queue.clone()))?);
            Ok(())
        }));
        Ok(d)
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    pub fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu.clone()
    }

    pub fn get_command_queue(&self) -> CommandQueue {
        self.cmd_queue.clone()
    }

    pub fn update_digital_out(&self, value: i64, minclock: u64, reqclock: u64) -> KResult<()> {
        let value = (value != 0) as i64;
        let cmd = self.update_pin_cmd.borrow();
        let Some(cmd) = cmd.as_ref() else {
            // Send setup message via mcu initialization ("%c" of an int)
            let c = char::from_u32(self.oid as u32).unwrap_or('\0');
            let v = char::from_u32(value as u32).unwrap_or('\0');
            self.mcu.add_config_cmd(&format!("update_digital_out oid={} value={}", c, v), false, false);
            return Ok(());
        };
        cmd.send(&[Val::Int(self.oid), Val::Int(value)], minclock, reqclock)
    }
}
