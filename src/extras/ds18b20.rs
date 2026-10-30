// Support for 1-wire based temperature sensors (port of klippy/extras/ds18b20.py)
//
// Copyright (C) 2020 Alan Lord <alanslists@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::danger_options::get_danger_options;
use crate::extras::heaters::{load_heaters, TempCallback, TempSensor};
use crate::logging;
use crate::mcu::{get_printer_mcu, Mcu};
use crate::msgproto::{Params, Val};
use crate::printer::{none_object, PObj};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

const DS18_REPORT_TIME: f64 = 3.0;
// Temperature can be sampled at any time but conversion time is ~750ms, so
// setting the time too low will not make the reports come faster.
const DS18_MIN_REPORT_TIME: f64 = 1.0;
const DS18_MAX_CONSECUTIVE_ERRORS: i64 = 4;

pub struct DS18B20 {
    pub name: String,
    pub sensor_id: Vec<u8>,
    pub min_temp: Cell<f64>,
    pub max_temp: Cell<f64>,
    pub report_clock: Cell<i64>,
    pub report_time: f64,
    pub mcu: Rc<Mcu>,
    pub oid: i64,
    pub callback: RefCell<Option<TempCallback>>,
}

fn int_of(p: &Params, k: &str) -> i64 {
    match p.get(k) {
        Some(Val::Int(v)) => *v,
        _ => 0,
    }
}

impl DS18B20 {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let sensor_id = config.get("serial_no", Dflt::Required)?.into_bytes();
        let report_time = config.getfloat("ds18_report_time", DS18_REPORT_TIME, Lim::new().min(DS18_MIN_REPORT_TIME))?;
        let mcu = get_printer_mcu(&printer, &config.get("sensor_mcu", Dflt::Required)?)?;
        let oid = mcu.create_oid();
        let s = Rc::new(DS18B20 {
            name: config.get_name().split_whitespace().last().unwrap_or_default().to_string(),
            sensor_id,
            min_temp: Cell::new(0.0),
            max_temp: Cell::new(0.0),
            report_clock: Cell::new(0),
            report_time,
            mcu: mcu.clone(),
            oid,
            callback: RefCell::new(None),
        });
        let ss = s.clone();
        mcu.register_response(Some(Rc::new(move |params: &Params| ss.handle_ds18b20_response(params))), "ds18b20_result", Some(oid));
        let ss = s.clone();
        mcu.register_config_callback(Rc::new(move || ss.build_config()));
        Ok(s)
    }

    pub fn build_config(&self) -> KResult<()> {
        let sid: String = self.sensor_id.iter().map(|x| format!("{:02x}", x)).collect();
        self.mcu.add_config_cmd(
            &format!(
                "config_ds18b20 oid={} serial={} max_error_count={} ignore_limits={}",
                self.oid,
                sid,
                DS18_MAX_CONSECUTIVE_ERRORS,
                get_danger_options().temp_ignore_limits as i64
            ),
            false,
            false,
        );
        let clock = self.mcu.get_query_slot(self.oid);
        self.report_clock.set(self.mcu.seconds_to_clock(self.report_time));
        // "%d" of a float truncates
        self.mcu.add_config_cmd(
            &format!(
                "query_ds18b20 oid={} clock={} rest_ticks={} min_value={} max_value={}",
                self.oid,
                clock,
                self.report_clock.get(),
                (self.min_temp.get() * 1000.0).trunc() as i64,
                (self.max_temp.get() * 1000.0).trunc() as i64
            ),
            true,
            false,
        );
        Ok(())
    }

    pub fn handle_ds18b20_response(&self, params: &Params) -> KResult<()> {
        let temp = int_of(params, "value") as f64 / 1000.0;
        let fault = int_of(params, "fault");
        if fault != 0 {
            logging::info(&format!("ds18b20 reports fault {} (temp={:.1})", fault, temp));
            return Ok(());
        }
        let next_clock = self.mcu.clock32_to_clock64(int_of(params, "next_clock"));
        let last_read_clock = next_clock - self.report_clock.get();
        let last_read_time = self.mcu.clock_to_print_time(last_read_clock as f64);
        let cb = self.callback.borrow().clone();
        match cb {
            Some(cb) => cb(last_read_time, temp),
            None => Err(KError::Other("'DS18B20' object has no attribute '_callback'".into())),
        }
    }
}

impl TempSensor for DS18B20 {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
        Ok(())
    }

    fn setup_callback(&self, cb: TempCallback) -> KResult<()> {
        *self.callback.borrow_mut() = Some(cb);
        Ok(())
    }

    fn get_report_time_delta(&self) -> KResult<f64> {
        Ok(self.report_time)
    }

}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    // Register sensor
    let pheaters = load_heaters(config)?;
    pheaters.add_sensor_factory(
        "DS18B20",
        Rc::new(|config: &ConfigWrapper| -> KResult<Rc<dyn TempSensor>> { Ok(DS18B20::new(config)?) }),
    );
    Ok(none_object())
}
