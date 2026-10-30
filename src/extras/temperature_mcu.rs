// Support for micro-controller chip based temperature sensors
// (port of klippy/extras/temperature_mcu.py)
//
// Copyright (C) 2020-2024  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::adc_temperature::HelperTemperatureDiagnostics;
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::mcu::{CommandQueryWrapper, McuAdc};
use crate::msgproto::Val;
use crate::printer::{none_object, PObj};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const SAMPLE_TIME: f64 = 0.001;
const SAMPLE_COUNT: i64 = 8;
const REPORT_TIME: f64 = 0.300;
const RANGE_CHECK_COUNT: i64 = 4;

pub struct PrinterTemperatureMCU {
    pub base_temperature: Cell<f64>,
    pub slope: Cell<f64>,
    pub temp1: Option<f64>,
    pub adc1: f64,
    pub temp2: Option<f64>,
    pub adc2: f64,
    pub min_temp: Cell<f64>,
    pub max_temp: Cell<f64>,
    pub debug_read_cmd: RefCell<Option<CommandQueryWrapper>>,
    pub reference_voltage: f64,
    pub mcu_adc: Rc<McuAdc>,
    pub diag_helper: RefCell<Option<Rc<HelperTemperatureDiagnostics>>>,
    pub danger_check_count: i64,
    pub temperature_callback: RefCell<Option<TempCallback>>,
    pub mcu_type: RefCell<String>,
}

impl PrinterTemperatureMCU {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        // Read config
        let mcu_name = config.get("sensor_mcu", "mcu")?;
        let reference_voltage = config.getfloat("reference_voltage", 3.3, Lim::new())?;
        let temp1 = config.getfloat_opt("sensor_temperature1", Lim::new())?;
        let (mut adc1, mut temp2, mut adc2) = (0.0, None, 0.0);
        if temp1.is_some() {
            adc1 = config.getfloat("sensor_adc1", Dflt::Required, Lim::new().min(0.0).max(1.0))?;
            temp2 = config.getfloat_opt("sensor_temperature2", Lim::new())?;
            if temp2.is_some() {
                adc2 = config.getfloat("sensor_adc2", Dflt::Required, Lim::new().min(0.0).max(1.0))?;
            }
        }
        // Setup ADC port
        let mcu_adc = crate::pins::setup_pin_as::<McuAdc>(&printer, "adc", &format!("{}:ADC_TEMPERATURE", mcu_name))?;
        let danger_check_count = if get_danger_options().temp_ignore_limits { 0 } else { RANGE_CHECK_COUNT };
        let s = Rc::new(PrinterTemperatureMCU {
            base_temperature: Cell::new(f64::NAN),
            slope: Cell::new(f64::NAN),
            temp1,
            adc1,
            temp2,
            adc2,
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            debug_read_cmd: RefCell::new(None),
            reference_voltage,
            mcu_adc: mcu_adc.clone(),
            diag_helper: RefCell::new(None),
            danger_check_count,
            temperature_callback: RefCell::new(None),
            mcu_type: RefCell::new(String::new()),
        });
        let ss = s.clone();
        mcu_adc.setup_adc_callback(REPORT_TIME, Rc::new(move |read_time, read_value| ss.adc_callback(read_time, read_value)));
        let ss = s.clone();
        let adc_pin: Rc<dyn crate::mcu::AdcPin> = mcu_adc.clone();
        let diag = HelperTemperatureDiagnostics::new(config, &adc_pin, Rc::new(move |adc| Ok(ss.calc_temp(adc))))?;
        *s.diag_helper.borrow_mut() = Some(diag);
        // Register callbacks
        if printer.start_arg("debugoutput").is_some() {
            mcu_adc.setup_adc_sample(SAMPLE_TIME, SAMPLE_COUNT, 0.0, 1.0, 0);
            return Ok(s);
        }
        let ss = s.clone();
        mcu_adc.get_mcu().register_config_callback(Rc::new(move || ss.build_config()));
        Ok(s)
    }

    // Internal code
    pub fn adc_callback(&self, read_time: f64, read_value: f64) -> KResult<()> {
        let temp = self.base_temperature.get() + read_value * self.slope.get();
        let cb = self.temperature_callback.borrow().clone();
        match cb {
            Some(cb) => cb(read_time + SAMPLE_COUNT as f64 * SAMPLE_TIME, temp),
            None => Err(KError::Other("'PrinterTemperatureMCU' object has no attribute 'temperature_callback'".into())),
        }
    }

    pub fn calc_temp(&self, adc: f64) -> f64 {
        self.base_temperature.get() + adc * self.slope.get()
    }

    pub fn calc_adc(&self, temp: f64) -> f64 {
        (temp - self.base_temperature.get()) / self.slope.get()
    }

    pub fn calc_base(&self, temp: f64, adc: f64) -> f64 {
        temp - adc * self.slope.get()
    }

    pub fn build_config(&self) -> KResult<()> {
        // Obtain mcu information
        let mcu = self.mcu_adc.get_mcu();
        *self.debug_read_cmd.borrow_mut() = Some(mcu.lookup_query_command("debug_read order=%c addr=%u", "debug_result val=%u", None, None, false)?);
        let mcu_type = mcu.get_constants().get("MCU").and_then(|v| v.as_str().map(|s| s.to_string())).unwrap_or_default();
        *self.mcu_type.borrow_mut() = mcu_type.clone();
        // Run MCU specific configuration
        type Cfg = fn(&PrinterTemperatureMCU) -> KResult<()>;
        let cfg_funcs: [(&str, Cfg); 22] = [
            ("rp2", Self::config_rp2040),
            ("sam3", Self::config_sam3),
            ("sam4", Self::config_sam4),
            ("same70", Self::config_same70),
            ("samd21", Self::config_samd21),
            ("samd51", Self::config_samd51),
            ("same5", Self::config_samd51),
            ("stm32f1", Self::config_stm32f1),
            ("stm32f2", Self::config_stm32f2),
            ("stm32f4", Self::config_stm32f4),
            ("stm32f042", Self::config_stm32f0x2),
            ("stm32f070", Self::config_stm32f070),
            ("stm32f072", Self::config_stm32f0x2),
            ("stm32g0", Self::config_stm32g0),
            ("stm32g4", Self::config_stm32g0),
            ("stm32l4", Self::config_stm32g0),
            ("stm32h723", Self::config_stm32h723),
            ("stm32h7", Self::config_stm32h7),
            ("gd32e230x8", Self::config_gd32e230x8),
            ("gd32f303xe", Self::config_gd32f303xe),
            ("gd32f303xb", Self::config_gd32f303xb),
            ("", Self::config_unknown),
        ];
        for (name, func) in cfg_funcs {
            if mcu_type.starts_with(name) {
                func(self)?;
                break;
            }
        }
        logging::info(&format!(
            "mcu_temperature '{}' nominal base={:.6} slope={:.6}",
            mcu.get_name(),
            self.base_temperature.get(),
            self.slope.get()
        ));
        // Setup manual base/slope override
        if let Some(temp1) = self.temp1 {
            if let Some(temp2) = self.temp2 {
                self.slope.set((temp2 - temp1) / (self.adc2 - self.adc1));
            }
            self.base_temperature.set(self.calc_base(temp1, self.adc1));
        }
        // Setup min/max checks
        let mut arange = [self.calc_adc(self.min_temp.get()), self.calc_adc(self.max_temp.get())];
        arange.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let (min_adc, max_adc) = (arange[0], arange[1]);
        self.mcu_adc.setup_adc_sample(SAMPLE_TIME, SAMPLE_COUNT, min_adc, max_adc, self.danger_check_count);
        if let Some(d) = self.diag_helper.borrow().as_ref() {
            d.setup_diag_minmax(self.min_temp.get(), self.max_temp.get(), min_adc, max_adc);
        }
        self.mcu_adc.build_config()
    }

    pub fn config_unknown(&self) -> KResult<()> {
        Err(KError::config(format!("MCU temperature not supported on {}", self.mcu_type.borrow())))
    }

    fn fixed(&self, slope: f64, temp: f64, adc: f64) -> KResult<()> {
        self.slope.set(slope);
        self.base_temperature.set(self.calc_base(temp, adc));
        Ok(())
    }

    pub fn config_gd32e230x8(&self) -> KResult<()> {
        self.fixed(3.3 / -0.004300, 25.0, 1.45 / 3.3)
    }

    pub fn config_gd32f303xe(&self) -> KResult<()> {
        self.fixed(3.3 / -0.004100, 25.0, 1.45 / 3.3)
    }

    pub fn config_gd32f303xb(&self) -> KResult<()> {
        self.fixed(3.3 / -0.004100, 25.0, 1.45 / 3.3)
    }

    pub fn config_rp2040(&self) -> KResult<()> {
        let rv = self.reference_voltage;
        self.fixed(rv / -0.001721, 27.0, 0.706 / rv)
    }

    pub fn config_sam3(&self) -> KResult<()> {
        let rv = self.reference_voltage;
        self.fixed(rv / 0.002650, 27.0, 0.8 / rv)
    }

    pub fn config_sam4(&self) -> KResult<()> {
        let rv = self.reference_voltage;
        self.fixed(rv / 0.004700, 27.0, 1.44 / rv)
    }

    pub fn config_same70(&self) -> KResult<()> {
        let rv = self.reference_voltage;
        self.fixed(rv / 0.002330, 25.0, 0.72 / rv)
    }

    fn config_samd21_at(&self, addr: i64) -> KResult<()> {
        let get1v = |val: i64| {
            let val = if val & 0x80 != 0 { 0x100 - val } else { val };
            1.0 - val as f64 / 1000.0
        };
        let cal1 = self.read32(addr)?;
        let cal2 = self.read32(addr + 4)?;
        let room_temp = (cal1 & 0xFF) as f64 + ((cal1 >> 8) & 0xF) as f64 / 10.0;
        let hot_temp = ((cal1 >> 12) & 0xFF) as f64 + ((cal1 >> 20) & 0xF) as f64 / 10.0;
        let room_1v = get1v((cal1 >> 24) & 0xFF);
        let hot_1v = get1v(cal2 & 0xFF);
        let rv = self.reference_voltage;
        let room_adc = ((cal2 >> 8) & 0xFFF) as f64 * room_1v / (rv * 4095.0);
        let hot_adc = ((cal2 >> 20) & 0xFFF) as f64 * hot_1v / (rv * 4095.0);
        self.slope.set((hot_temp - room_temp) / (hot_adc - room_adc));
        self.base_temperature.set(self.calc_base(room_temp, room_adc));
        Ok(())
    }

    fn config_samd21(&self) -> KResult<()> {
        self.config_samd21_at(0x00806030)
    }

    pub fn config_samd51(&self) -> KResult<()> {
        self.config_samd21_at(0x00800100)
    }

    pub fn config_stm32f1(&self) -> KResult<()> {
        let rv = self.reference_voltage;
        self.fixed(rv / -0.004300, 25.0, 1.43 / rv)
    }

    pub fn config_stm32f2(&self) -> KResult<()> {
        let rv = self.reference_voltage;
        self.fixed(rv / 0.002500, 25.0, 0.76 / rv)
    }

    /// slope/base from two factory calibration reads
    fn calibrated(&self, addr1: i64, addr2: i64, vcal: f64, adc_max: f64, t1: f64, t2: f64) -> KResult<()> {
        let rv = self.reference_voltage;
        let cal_adc_1 = self.read16(addr1)? as f64 * vcal / (rv * adc_max);
        let cal_adc_2 = self.read16(addr2)? as f64 * vcal / (rv * adc_max);
        self.slope.set((t2 - t1) / (cal_adc_2 - cal_adc_1));
        self.base_temperature.set(self.calc_base(t1, cal_adc_1));
        Ok(())
    }

    fn config_stm32f4(&self) -> KResult<()> {
        self.calibrated(0x1FFF7A2C, 0x1FFF7A2E, 3.3, 4095.0, 30.0, 110.0)
    }

    pub fn config_stm32f0x2(&self) -> KResult<()> {
        self.calibrated(0x1FFFF7B8, 0x1FFFF7C2, 3.3, 4095.0, 30.0, 110.0)
    }

    pub fn config_stm32f070(&self) -> KResult<()> {
        self.slope.set(self.reference_voltage / -0.004300);
        let cal_adc_30 = self.read16(0x1FFFF7B8)? as f64 / 4095.0;
        self.base_temperature.set(self.calc_base(30.0, cal_adc_30));
        Ok(())
    }

    pub fn config_stm32g0(&self) -> KResult<()> {
        self.calibrated(0x1FFF75A8, 0x1FFF75CA, 3.0, 4095.0, 30.0, 130.0)
    }

    pub fn config_stm32h723(&self) -> KResult<()> {
        self.calibrated(0x1FF1E820, 0x1FF1E840, 3.3, 4095.0, 30.0, 130.0)
    }

    pub fn config_stm32h7(&self) -> KResult<()> {
        self.calibrated(0x1FF1E820, 0x1FF1E840, 3.3, 65535.0, 30.0, 110.0)
    }

    fn debug_read(&self, order: i64, addr: i64) -> KResult<i64> {
        let cmd = self.debug_read_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        let params = cmd.send(&[Val::Int(order), Val::Int(addr)], 0, 0, true)?;
        match params.get("val") {
            Some(Val::Int(v)) => Ok(*v),
            _ => Err(KError::Other("'val'".into())),
        }
    }

    pub fn read16(&self, addr: i64) -> KResult<i64> {
        self.debug_read(1, addr)
    }

    pub fn read32(&self, addr: i64) -> KResult<i64> {
        self.debug_read(2, addr)
    }
}

impl TempSensor for PrinterTemperatureMCU {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }

    fn setup_callback(&self, cb: TempCallback) -> KResult<()> {
        *self.temperature_callback.borrow_mut() = Some(cb);
        Ok(())
    }

    fn get_report_time_delta(&self) -> KResult<f64> {
        Ok(REPORT_TIME)
    }

}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(
        "temperature_mcu",
        Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(PrinterTemperatureMCU::new(config)?) }),
    );
    Ok(none_object())
}
