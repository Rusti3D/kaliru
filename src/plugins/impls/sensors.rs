// klippy/extras temperature sensors: adc_temperature, thermistor, the i2c /
// spi / 1-wire chips, temperature_host / _mcu / _combined / _sensor,
// mpc_*_temperature, temperature_fan.ControlCurve; and the digipot / dac
// chips (ad5206, dac084S085, mcp4018, mcp4451, mcp4728)
use super::*;
use crate::extras::adc_temperature::{
    AdcConvert, CustomLinearResistance, CustomLinearVoltage, HelperTemperatureDiagnostics, LinearInterpolate,
    LinearResistance, LinearVoltage, PrinterADCtoTemperature,
};
use crate::extras::thermistor::{CustomThermistor, Thermistor, ThermistorParams};
use crate::plugins::callable::py_function;
use crate::plugins::convert::{FromPy, ToPy};
use crate::{method, ro_attr};

fn pycfg_of(o: &PyRef) -> KResult<Rc<crate::configfile::ConfigWrapper>> {
    super::configfile::config_from_py(o)
}

fn pycfg(a: &crate::plugins::native::Args, i: usize) -> KResult<Rc<crate::configfile::ConfigWrapper>> {
    pycfg_of(a.arg(i))
}

fn byte(v: i64) -> KResult<u8> {
    crate::extras::ad5206::byte(v)
}

fn bytes(o: &PyRef) -> KResult<Vec<u8>> {
    let v: Vec<i64> = FromPy::from_py(o)?;
    v.into_iter().map(byte).collect()
}

/// a converter (Thermistor, LinearVoltage, LinearResistance) as python sees it
fn adc_convert_to_py(c: Rc<dyn AdcConvert>) -> KResult<PyRef> {
    let name = c.type_name();
    let any: Rc<dyn Any> = c;
    objects::wrap_any(any, name)
}

/// li.interpolate / li.reverse_interpolate as a bound method
fn li_method(li: Rc<LinearInterpolate>, reverse: bool) -> KResult<PyRef> {
    py_function(Rc::new(move |a: &[PyRef]| {
        let v = a[0].as_f64()?;
        if reverse {
            Ok(py_float(li.reverse_interpolate(v)))
        } else {
            Ok(py_float(li.interpolate(v)?))
        }
    }))
}

/// PrinterThermistor's params: {"t1", "r1", "beta"} or {"t1".."r3"}
fn thermistor_params_to_py(p: &ThermistorParams) -> KResult<PyRef> {
    let d = lookup("builtins", "dict")?.call(&[])?;
    let items: Vec<(&str, f64)> = match *p {
        ThermistorParams::Beta { t1, r1, beta } => vec![("t1", t1), ("r1", r1), ("beta", beta)],
        ThermistorParams::Points { t1, r1, t2, r2, t3, r3 } => {
            vec![("t1", t1), ("r1", r1), ("t2", t2), ("r2", r2), ("t3", t3), ("r3", r3)]
        }
    };
    for (k, v) in items {
        d.call_method("__setitem__", &[&py_str(k), &py_float(v)])?;
    }
    Ok(d)
}

fn thermistor_params_from_py(o: &PyRef) -> KResult<ThermistorParams> {
    let get = |k: &str| -> KResult<f64> { o.call_method("__getitem__", &[&py_str(k)])?.as_f64() };
    if o.call_method("__contains__", &[&py_str("beta")])?.is_true()? {
        return Ok(ThermistorParams::Beta { t1: get("t1")?, r1: get("r1")?, beta: get("beta")? });
    }
    Ok(ThermistorParams::Points { t1: get("t1")?, r1: get("r1")?, t2: get("t2")?, r2: get("r2")?, t3: get("t3")?, r3: get("r3")? })
}

pub fn register(r: &mut Registry) {
    // ---- digipots / dacs: set_register over their bus
    {
        use crate::extras::ad5206::Ad5206;
        use crate::extras::dac084s085::Dac084S085;
        use crate::extras::mcp4018::Mcp4018;
        use crate::extras::mcp4451::Mcp4451;
        use crate::extras::mcp4728::Mcp4728;
        method!(r, "klippy.extras.ad5206.ad5206", Ad5206, set_register(s, reg: i64, value: i64) => {
            s.spi.spi_send(&[byte(reg)?, byte(value)?], 0, 0)
        });
        ro_attr!(r, "klippy.extras.ad5206.ad5206.spi", Ad5206, |s| s.spi.clone());
        method!(r, "klippy.extras.dac084S085.dac084S085", Dac084S085, set_register(s, chan: i64, value: i64) => s.set_register(chan, value));
        ro_attr!(r, "klippy.extras.dac084S085.dac084S085.spi", Dac084S085, |s| s.spi.clone());
        r.method("klippy.extras.dac084S085.load_config_prefix", |a| {
            crate::extras::dac084s085::load_config_prefix(&*pycfg(a, 0)?).to_py()
        });
        method!(r, "klippy.extras.mcp4018.mcp4018", Mcp4018, handle_connect(s) => s.set_dac(s.start_value));
        method!(r, "klippy.extras.mcp4451.mcp4451", Mcp4451, set_register(s, reg: i64, value: i64) => {
            s.i2c.i2c_write(&[byte((reg << 4) | ((value >> 8) & 0x03))?, byte(value)?], 0, 0)
        });
        ro_attr!(r, "klippy.extras.mcp4451.mcp4451.i2c", Mcp4451, |s| s.i2c.clone());
        method!(r, "klippy.extras.mcp4728.mcp4728", Mcp4728, set_dac(s, dac: i64, value: i64) => {
            s.i2c.i2c_write(&[byte(0x40 | (dac << 1))?, byte(((value >> 8) & 0x0F) | 0x80)?, byte(value & 0xFF)?], 0, 0)
        });
        ro_attr!(r, "klippy.extras.mcp4728.mcp4728.i2c", Mcp4728, |s| s.i2c.clone());
    }

    // ---- adc_temperature
    r.attr("klippy.extras.adc_temperature.PrinterADCtoTemperature.adc_convert", |o| {
        adc_convert_to_py(objects::unwrap_as::<PrinterADCtoTemperature>(o)?.adc_convert.clone())
    }, None);
    r.attr("klippy.extras.adc_temperature.HelperTemperatureDiagnostics.calc_temp_cb", |o| {
        let cb = objects::unwrap_as::<HelperTemperatureDiagnostics>(o)?.calc_temp_cb.clone();
        py_function(Rc::new(move |a: &[PyRef]| Ok(py_float(cb(a[0].as_f64()?)?))))
    }, None);
    // (None until setup_diag_minmax)
    fn diag(o: &PyRef, i: usize) -> KResult<PyRef> {
        objects::unwrap_as::<HelperTemperatureDiagnostics>(o)?.minmax.get().map(|m| [m.0, m.1, m.2, m.3][i]).to_py()
    }
    r.attr("klippy.extras.adc_temperature.HelperTemperatureDiagnostics.min_temp", |o| diag(o, 0), None);
    r.attr("klippy.extras.adc_temperature.HelperTemperatureDiagnostics.max_temp", |o| diag(o, 1), None);
    r.attr("klippy.extras.adc_temperature.HelperTemperatureDiagnostics.min_adc", |o| diag(o, 2), None);
    r.attr("klippy.extras.adc_temperature.HelperTemperatureDiagnostics.max_adc", |o| diag(o, 3), None);
    // LinearVoltage: calc_temp / calc_adc are li's bound methods
    r.attr("klippy.extras.adc_temperature.LinearVoltage.calc_temp", |o| li_method(objects::unwrap_as::<LinearVoltage>(o)?.li.clone(), false), None);
    r.attr("klippy.extras.adc_temperature.LinearVoltage.calc_adc", |o| li_method(objects::unwrap_as::<LinearVoltage>(o)?.li.clone(), true), None);
    method!(r, "klippy.extras.adc_temperature.LinearResistance", LinearResistance, calc_temp(s, adc: f64) => s.calc_temp(adc));
    method!(r, "klippy.extras.adc_temperature.LinearResistance", LinearResistance, calc_adc(s, temp: f64) => s.calc_adc(temp));
    ro_attr!(r, "klippy.extras.adc_temperature.LinearResistance.li", LinearResistance, |s| s.li.clone());
    r.ctor("klippy.extras.adc_temperature.CustomLinearVoltage.__init__", |a| {
        let o: Rc<dyn Any> = Rc::new(CustomLinearVoltage::new(&*pycfg(a, 0)?)?);
        Ok(o)
    });
    ro_attr!(r, "klippy.extras.adc_temperature.CustomLinearVoltage.name", CustomLinearVoltage, |s| s.name.clone());
    ro_attr!(r, "klippy.extras.adc_temperature.CustomLinearVoltage.params", CustomLinearVoltage, |s| s.params.clone());
    method!(r, "klippy.extras.adc_temperature.CustomLinearVoltage", CustomLinearVoltage, create(s, config: PyRef) => s.create(&*pycfg_of(&config)?));
    r.ctor("klippy.extras.adc_temperature.CustomLinearResistance.__init__", |a| {
        let o: Rc<dyn Any> = Rc::new(CustomLinearResistance::new(&*pycfg(a, 0)?)?);
        Ok(o)
    });
    ro_attr!(r, "klippy.extras.adc_temperature.CustomLinearResistance.name", CustomLinearResistance, |s| s.name.clone());
    ro_attr!(r, "klippy.extras.adc_temperature.CustomLinearResistance.samples", CustomLinearResistance, |s| s.samples.clone());
    method!(r, "klippy.extras.adc_temperature.CustomLinearResistance", CustomLinearResistance, create(s, config: PyRef) => s.create(&*pycfg_of(&config)?));

    // ---- thermistor
    method!(r, "klippy.extras.thermistor.Thermistor", Thermistor, calc_temp(s, adc: f64) => s.calc_temp(adc));
    method!(r, "klippy.extras.thermistor.Thermistor", Thermistor, calc_adc(s, temp: f64) => s.calc_adc(temp));
    r.method("klippy.extras.thermistor.PrinterThermistor", |a| {
        let config = pycfg(a, 0)?;
        super::heaters::sensor_to_py(crate::extras::thermistor::printer_thermistor(&config, &thermistor_params_from_py(a.arg(1))?)?)
    });
    r.ctor("klippy.extras.thermistor.CustomThermistor.__init__", |a| {
        let o: Rc<dyn Any> = Rc::new(CustomThermistor::new(&*pycfg(a, 0)?)?);
        Ok(o)
    });
    ro_attr!(r, "klippy.extras.thermistor.CustomThermistor.name", CustomThermistor, |s| s.name.clone());
    r.attr("klippy.extras.thermistor.CustomThermistor.params", |o| thermistor_params_to_py(&objects::unwrap_as::<CustomThermistor>(o)?.params), None);
    r.method("klippy.extras.thermistor.CustomThermistor.create", |a| {
        let config = pycfg(a, 0)?;
        super::heaters::sensor_to_py(a.this::<CustomThermistor>()?.create(&config)?)
    });

    // ---- i2c / 1-wire chips
    ro_attr!(r, "klippy.extras.aht10.AHTBase.mcu", crate::extras::aht10::AHTBase, |s| s.i2c.get_mcu());
    {
        use crate::extras::bme280::BME280;
        // {name: register} of the detected chip
        r.attr("klippy.extras.bme280.BME280.chip_registers", |o| {
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in objects::unwrap_as::<BME280>(o)?.chip_registers.get() {
                d.call_method("__setitem__", &[&py_str(k), &py_int(*v as i64)])?;
            }
            Ok(d)
        }, None);
        // None until connected; the BMP388 values are floats (scaled), the
        // other chips' ints
        r.attr("klippy.extras.bme280.BME280.dig", |o| {
            let s = objects::unwrap_as::<BME280>(o)?;
            let dig = s.dig.borrow();
            if dig.is_empty() {
                return Ok(py_none());
            }
            let floats = *s.chip_type.borrow() == "BMP388";
            let d = lookup("builtins", "dict")?.call(&[])?;
            for (k, v) in dig.iter() {
                let v = if floats { py_float(*v) } else { py_int(*v as i64) };
                d.call_method("__setitem__", &[&py_str(k), &v])?;
            }
            Ok(d)
        }, None);
        // (set to 0.0 in __init__ and never changed)
        r.attr("klippy.extras.bme280.BME280.range_switching_error", |_o| Ok(py_float(0.0)), None);
    }
    {
        use crate::extras::ds18b20::DS18B20;
        ro_attr!(r, "klippy.extras.ds18b20.DS18B20.name", DS18B20, |s| s.name.clone());
        // (set to 0.0 in __init__ and never updated)
        r.attr("klippy.extras.ds18b20.DS18B20.temp", |_o| Ok(py_float(0.0)), None);
        r.method("klippy.extras.ds18b20.DS18B20.fault", |a| {
            let p = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
            p.invoke_async_shutdown(&a.string(0)?, None);
            Ok(py_none())
        });
    }
    {
        use crate::extras::htu21d::HTU21D;
        ro_attr!(r, "klippy.extras.htu21d.HTU21D.deviceId", HTU21D, |s| s.device_id.clone());
        ro_attr!(r, "klippy.extras.htu21d.HTU21D.mcu", HTU21D, |s| s.i2c.get_mcu());
        ro_attr!(r, "klippy.extras.htu21d.HTU21D.name", HTU21D, |s| s.name.clone());
    }
    {
        use crate::extras::sht3x::SHT3X;
        ro_attr!(r, "klippy.extras.sht3x.SHT3X.deviceId", SHT3X, |s| s.device_id.clone());
        ro_attr!(r, "klippy.extras.sht3x.SHT3X.mcu", SHT3X, |s| s.i2c.get_mcu());
        ro_attr!(r, "klippy.extras.sht3x.SHT3X.name", SHT3X, |s| s.name.clone());
    }
    {
        use crate::extras::lm75::LM75;
        ro_attr!(r, "klippy.extras.lm75.LM75.name", LM75, |s| s.name.clone());
        r.method("klippy.extras.lm75.LM75.degrees_from_sample", |a| Ok(py_float(LM75::degrees_from_sample(&bytes(a.arg(0))?))));
        // write_register(reg_name, data): data is a value or a list of them
        r.method("klippy.extras.lm75.LM75.write_register", |a| {
            let s = a.this::<LM75>()?;
            let regs = lookup("klippy.extras.lm75", "LM75_REGS")?;
            let reg = regs.call_method("__getitem__", &[a.arg(0)])?.as_i64()?;
            let data = if a.arg(1).is_instance(&lookup("builtins", "list")?)? { bytes(a.arg(1))? } else { vec![byte(a.arg(1).as_i64()?)?] };
            let mut msg = vec![byte(reg)?];
            msg.extend(data);
            s.i2c.i2c_write(&msg, 0, 0)?;
            Ok(py_none())
        });
    }

    // ---- spi_temperature
    {
        use crate::extras::spi_temperature::SensorBase;
        // (None until setup_minmax)
        r.attr("klippy.extras.spi_temperature.SensorBase.min_temp", |o| objects::unwrap_as::<SensorBase>(o)?.minmax_temp.get().map(|m| m.0).to_py(), None);
        r.attr("klippy.extras.spi_temperature.SensorBase.max_temp", |o| objects::unwrap_as::<SensorBase>(o)?.minmax_temp.get().map(|m| m.1).to_py(), None);
        // handle_fault(adc, fault)
        macro_rules! handle_fault {
            ($r:expr, $class:literal) => {
                $r.method(concat!($class, ".handle_fault"), |a| {
                    a.this::<SensorBase>()?.handle_fault(a.arg(1).as_i64()?)?;
                    Ok(py_none())
                });
            };
        }
        handle_fault!(r, "klippy.extras.spi_temperature.MAX31856");
        handle_fault!(r, "klippy.extras.spi_temperature.MAX31855");
        handle_fault!(r, "klippy.extras.spi_temperature.MAX6675");
        handle_fault!(r, "klippy.extras.spi_temperature.MAX31865");
        method!(r, "klippy.extras.spi_temperature.MAX31856", SensorBase, build_spi_init(_s, config: PyRef) => {
            crate::extras::spi_temperature::max31856_spi_init(&*pycfg_of(&config)?)
        });
        method!(r, "klippy.extras.spi_temperature.MAX31865", SensorBase, build_spi_init(_s, config: PyRef) => {
            crate::extras::spi_temperature::max31865_spi_init(&*pycfg_of(&config)?)
        });
        r.attr("klippy.extras.spi_temperature.MAX31865.adc_to_resist_div_nominal", |o| {
            match objects::unwrap_as::<SensorBase>(o)?.chip {
                crate::extras::spi_temperature::Chip::Max31865 { adc_to_resist_div_nominal } => Ok(py_float(adc_to_resist_div_nominal)),
                _ => Err(KError::Py("AttributeError".into(), "adc_to_resist_div_nominal".into())),
            }
        }, None);
    }

    // ---- temperature_host / _mcu / _combined / _sensor
    {
        use crate::extras::temperature_host::TemperatureHost;
        ro_attr!(r, "klippy.extras.temperature_host.Temperature_HOST.name", TemperatureHost, |s| s.name.clone());
        ro_attr!(r, "klippy.extras.temperature_host.Temperature_HOST.path", TemperatureHost, |s| s.path.clone());
        // python's file object over kaliru's open file (same descriptor)
        r.attr("klippy.extras.temperature_host.Temperature_HOST.file_handle", |o| {
            use std::os::fd::AsRawFd;
            let s = objects::unwrap_as::<TemperatureHost>(o)?;
            let fd = match s.file_handle.borrow().as_ref() {
                Some(f) => f.as_raw_fd(),
                None => return Err(KError::Py("AttributeError".into(), "file_handle".into())),
            };
            // fdopen(fd, "r", -1, None, None, None, closefd=False)
            let n = py_none();
            lookup("os", "fdopen")?.call(&[&py_int(fd as i64), &py_str("r"), &py_int(-1), &n, &n, &n, &py_bool(false)])
        }, None);
    }
    {
        use crate::extras::temperature_mcu::PrinterTemperatureMCU as T;
        method!(r, "klippy.extras.temperature_mcu.PrinterTemperatureMCU", T, config_samd21(s, addr: i64) => s.config_samd21_at(addr));
        method!(r, "klippy.extras.temperature_mcu.PrinterTemperatureMCU", T, config_stm32f4(s, addr1: i64, addr2: i64) => {
            s.calibrated(addr1, addr2, 3.3, 4095.0, 30.0, 110.0)
        });
    }
    {
        use crate::extras::temperature_combined::PrinterSensorCombined as C;
        ro_attr!(r, "klippy.extras.temperature_combined.PrinterSensorCombined.name", C, |s| s.name.clone());
        r.attr("klippy.extras.temperature_combined.PrinterSensorCombined.sensor", |o| Ok(o.clone()), None);
        // the algorithm: builtins min / max or the module's mean
        r.attr("klippy.extras.temperature_combined.PrinterSensorCombined.apply_mode", |o| {
            match objects::unwrap_as::<C>(o)?.apply_mode_name() {
                "mean" => lookup("klippy.extras.temperature_combined", "mean"),
                m => lookup("builtins", m),
            }
        }, None);
        // mean(values): None for no values
        r.method("klippy.extras.temperature_combined.mean", |a| {
            let v: Vec<f64> = FromPy::from_py(&lookup("builtins", "list")?.call(&[a.arg(0)])?)?;
            if v.is_empty() {
                return Ok(py_none());
            }
            Ok(py_float(crate::pyfmt::py_sum(v.iter().copied()) / v.len() as f64))
        });
    }
    r.attr("klippy.extras.temperature_sensor.PrinterSensorGeneric.sensor", |o| {
        super::heaters::sensor_to_py(objects::unwrap_as::<crate::extras::temperature_sensor::PrinterSensorGeneric>(o)?.sensor.clone())
    }, None);

    // ---- mpc_*_temperature (one Rust type)
    {
        use crate::extras::heaters::MpcSensor;
        use crate::extras::mpc_block_temperature::MpcTempWrapper as W;
        macro_rules! mpc_wrapper {
            ($r:expr, $class:literal) => {
                $r.method(concat!($class, ".process_temp_update"), |a| {
                    a.this::<W>()?.process_temp_update(&super::heaters::control_from_py(a.arg(0))?, a.arg(1).as_f64()?)?;
                    Ok(py_none())
                });
                // (a no-op)
                $r.method(concat!($class, ".set_report_time"), |_a| Ok(py_none()));
            };
        }
        mpc_wrapper!(r, "klippy.extras.mpc_ambient_temperature.MPC_AMBIENT_TEMP_WRAPPER");
        mpc_wrapper!(r, "klippy.extras.mpc_block_temperature.MPC_BLOCK_TEMP_WRAPPER");
    }

    // ---- temperature_fan.ControlCurve: numpy's array([temps, speeds])
    r.attr("klippy.extras.temperature_fan.ControlCurve.curve_standard", |o| {
        let c = objects::unwrap_as::<crate::extras::temperature_fan::ControlCurve>(o)?;
        let rows = c.curve_standard.clone().to_vec().to_py()?;
        lookup("numpy", "array")?.call(&[&rows])
    }, None);
}
