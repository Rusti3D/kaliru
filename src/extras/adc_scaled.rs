// Support for scaling ADC values based on measured VREF and VSSA
// (port of klippy/extras/adc_scaled.py)
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::query_adc::QueryADC;
use crate::mcu::{AdcCallback, AdcPin, Mcu};
use crate::pins::{PinChip, PinsObject, SharedPinParams};
use crate::printer::{PObj, Printer, PrinterObject};
use crate::pyfmt::py_min;
use std::any::Any;
use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};

const SAMPLE_TIME: f64 = 0.001;
const SAMPLE_COUNT: i64 = 8;
const REPORT_TIME: f64 = 0.300;

pub struct McuScaledAdc {
    me: Weak<McuScaledAdc>,
    pub main: Weak<PrinterADCScaled>,
    pub last_state: Cell<(f64, f64)>,
    pub mcu_adc: Rc<dyn AdcPin>,
    callback: RefCell<Option<AdcCallback>>,
}

impl McuScaledAdc {
    fn new(main: &Rc<PrinterADCScaled>, pin_params: &SharedPinParams) -> Result<Rc<Self>, String> {
        let obj = PinChip::setup_pin(&*main.mcu, "adc", pin_params)?;
        let mcu_adc: Rc<dyn AdcPin> = obj.downcast::<crate::mcu::McuAdc>().map_err(|_| "adc pin of an unexpected type".to_string())?;
        let query_adc = main.printer.lookup::<QueryADC>("query_adc").map_err(|e| e.message().to_string())?;
        let qname = format!("{}:{}", main.name, pin_params.borrow().pin);
        query_adc.register_adc(&qname, mcu_adc.clone());
        Ok(Rc::new_cyclic(|me| McuScaledAdc {
            me: me.clone(),
            main: Rc::downgrade(main),
            last_state: Cell::new((0.0, 0.0)),
            mcu_adc,
            callback: RefCell::new(None),
        }))
    }

    pub fn handle_callback(&self, read_time: f64, read_value: f64) -> KResult<()> {
        let main = self.main.upgrade().ok_or_else(|| KError::Other("adc_scaled gone".into()))?;
        let max_adc = main.last_vref.get().1;
        let min_adc = main.last_vssa.get().1;
        let scaled_val = (read_value - min_adc) / (max_adc - min_adc);
        self.last_state.set((scaled_val, read_time));
        let cb = self.callback.borrow().clone();
        match cb {
            Some(cb) => cb(read_time, scaled_val),
            None => Err(KError::Other("'NoneType' object is not callable".into())),
        }
    }
}

impl AdcPin for McuScaledAdc {
    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_adc.get_mcu()
    }
    fn setup_adc_sample(&self, sample_time: f64, sample_count: i64, minval: f64, maxval: f64, range_check_count: i64) {
        self.mcu_adc.setup_adc_sample(sample_time, sample_count, minval, maxval, range_check_count)
    }
    fn setup_adc_callback(&self, report_time: f64, callback: AdcCallback) {
        *self.callback.borrow_mut() = Some(callback);
        let me = self.me.clone();
        self.mcu_adc.setup_adc_callback(
            report_time,
            Rc::new(move |read_time, read_value| match me.upgrade() {
                Some(s) => s.handle_callback(read_time, read_value),
                None => Ok(()),
            }),
        );
    }
    fn get_last_value(&self) -> (f64, f64) {
        self.last_state.get()
    }
}

pub struct PrinterADCScaled {
    me: Weak<PrinterADCScaled>,
    pub printer: Rc<Printer>,
    pub name: String,
    pub last_vref: Cell<(f64, f64)>,
    pub last_vssa: Cell<(f64, f64)>,
    pub inv_smooth_time: f64,
    pub mcu: Rc<Mcu>,
    scaled: RefCell<Vec<Rc<McuScaledAdc>>>,
}

impl PrinterADCScaled {
    pub fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().nth(1).unwrap_or("").to_string();
        // Configure vref and vssa pins (the callbacks find the object
        // through a slot filled below)
        let slot: Rc<RefCell<Weak<PrinterADCScaled>>> = Rc::new(RefCell::new(Weak::new()));
        let mcu_vref = Self::config_pin(&printer, config, &name, "vref", &slot, true)?;
        let mcu_vssa = Self::config_pin(&printer, config, &name, "vssa", &slot, false)?;
        let smooth_time = config.getfloat("smooth_time", 2.0, Lim::new().above(0.0))?;
        let mcu = mcu_vref.get_mcu();
        if !Rc::ptr_eq(&mcu, &mcu_vssa.get_mcu()) {
            return Err(KError::config("vref and vssa must be on same mcu"));
        }
        let a = Rc::new_cyclic(|me| PrinterADCScaled {
            me: me.clone(),
            printer: printer.clone(),
            name: name.clone(),
            last_vref: Cell::new((0.0, 0.0)),
            last_vssa: Cell::new((0.0, 0.0)),
            inv_smooth_time: 1.0 / smooth_time,
            mcu,
            scaled: RefCell::new(vec![]),
        });
        *slot.borrow_mut() = Rc::downgrade(&a);
        // Register setup_pin
        let ppins = printer.lookup::<PinsObject>("pins")?;
        ppins.0.borrow_mut().register_chip(&name, a.clone()).map_err(|e| KError::Pins(e.0))?;
        Ok(a)
    }

    fn config_pin(
        printer: &Rc<Printer>,
        config: &ConfigWrapper,
        main_name: &str,
        name: &str,
        slot: &Rc<RefCell<Weak<PrinterADCScaled>>>,
        is_vref: bool,
    ) -> KResult<Rc<dyn AdcPin>> {
        let pin_name = config.get(&format!("{}_pin", name), Dflt::Required)?;
        let mcu_adc = crate::pins::setup_adc_pin(printer, &pin_name)?;
        let s = slot.clone();
        mcu_adc.setup_adc_callback(
            REPORT_TIME,
            Rc::new(move |read_time, read_value| {
                if let Some(a) = s.borrow().upgrade() {
                    let cell = if is_vref { &a.last_vref } else { &a.last_vssa };
                    cell.set(a.calc_smooth(read_time, read_value, cell.get()));
                }
                Ok(())
            }),
        );
        mcu_adc.setup_adc_sample(SAMPLE_TIME, SAMPLE_COUNT, 0.0, 1.0, 0);
        let query_adc = printer.load_as::<QueryADC>(config, "query_adc")?;
        query_adc.register_adc(&format!("{}:{}", main_name, name), mcu_adc.clone());
        Ok(mcu_adc)
    }

    pub fn calc_smooth(&self, read_time: f64, read_value: f64, last: (f64, f64)) -> (f64, f64) {
        let (last_time, last_value) = last;
        let time_diff = read_time - last_time;
        let value_diff = read_value - last_value;
        let adj_time = py_min(time_diff * self.inv_smooth_time, 1.0);
        let smoothed_value = last_value + value_diff * adj_time;
        (read_time, smoothed_value)
    }
}

impl PinChip for PrinterADCScaled {
    fn setup_pin(&self, pin_type: &str, pin_params: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        if pin_type != "adc" {
            return Err("adc_scaled only supports adc pins".into());
        }
        let me = self.me.upgrade().expect("adc_scaled");
        let s = McuScaledAdc::new(&me, pin_params)?;
        self.scaled.borrow_mut().push(s.clone());
        let p: Rc<dyn AdcPin> = s;
        Ok(Rc::new(p))
    }
}

impl PrinterObject for PrinterADCScaled {
    fn teardown(&self) {
        self.scaled.borrow_mut().clear();
    }
}

pub fn load_config_prefix(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(PrinterADCScaled::new(config)?)
}
