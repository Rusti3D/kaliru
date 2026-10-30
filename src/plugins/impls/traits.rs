// Methods Kalico defines on many classes that kaliru implements through one
// trait (temperature sensors, pins, probes, lcd chips, bulk adc chips): a
// handler by method name for every class without its own, through the
// object's trait view (generic_methods), and the views of every
// implementor. self.printer / self.reactor are the running printer's.
use super::*;
use crate::extras::display::LcdChip;
use crate::extras::heaters::TempSensor;
use crate::extras::homing::HomingMove;
use crate::extras::load_cell::BulkAdcSensor;
use crate::extras::probe::McuProbe;
use crate::mcu::{AdcPin, DigitalOutPin, Endstop, PwmPin};
use crate::plugins::convert::{FromPy, ToPy};
use crate::plugins::native::{Args, Getter, Method};

fn view<D: ?Sized + 'static>(a: &Args) -> Option<Rc<D>> {
    objects::unwrap_view::<D>(a.slf.as_ref().expect("method"))
}

fn not_mirrored(a: &Args, _name: &str) -> KError {
    KError::Py("NotImplementedError".into(), format!("kaliru: {} is not mirrored yet", a.name()))
}

fn arg_or(a: &Args, i: usize) -> Option<&PyRef> {
    a.args.get(i).filter(|v| !v.is_none())
}

fn hmove(o: &PyRef) -> KResult<Rc<HomingMove>> {
    objects::unwrap_as::<HomingMove>(o)
}

/// The handler of a method name kaliru implements through a trait
pub fn generic(name: &str) -> Option<Method> {
    let m: Method = match name.rsplit('.').next()? {
        // TempSensor
        "setup_minmax" => |a| match view::<dyn TempSensor>(a) {
            Some(s) => {
                s.setup_minmax(a.f64(0)?, a.f64(1)?)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "setup_minmax")),
        },
        "setup_callback" => |a| match view::<dyn TempSensor>(a) {
            Some(s) => {
                s.setup_callback(temp_callback_from_py(a.arg(0)))?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "setup_callback")),
        },
        "get_report_time_delta" => |a| match view::<dyn TempSensor>(a) {
            Some(s) => Ok(py_float(s.get_report_time_delta()?)),
            None => Err(not_mirrored(a, "get_report_time_delta")),
        },
        // the pins (PwmPin before DigitalOutPin: a pin that is both is a pwm)
        "get_mcu" => |a| {
            if let Some(e) = view::<dyn Endstop>(a) {
                return e.get_mcu().to_py();
            }
            if let Some(p) = view::<dyn PwmPin>(a) {
                return p.get_mcu().to_py();
            }
            if let Some(p) = view::<dyn DigitalOutPin>(a) {
                return p.get_mcu().to_py();
            }
            if let Some(p) = view::<dyn AdcPin>(a) {
                return p.get_mcu().to_py();
            }
            if let Some(c) = view::<dyn LcdChip>(a) {
                return c.get_mcu().to_py();
            }
            if let Some(s) = view::<dyn BulkAdcSensor>(a) {
                return s.get_mcu().to_py();
            }
            Err(not_mirrored(a, "get_mcu"))
        },
        "setup_max_duration" => |a| {
            if let Some(p) = view::<dyn PwmPin>(a) {
                p.setup_max_duration(a.f64(0)?);
            } else if let Some(p) = view::<dyn DigitalOutPin>(a) {
                p.setup_max_duration(a.f64(0)?);
            } else {
                return Err(not_mirrored(a, "setup_max_duration"));
            }
            Ok(py_none())
        },
        "setup_start_value" => |a| {
            if let Some(p) = view::<dyn PwmPin>(a) {
                p.setup_start_value(a.f64(0)?, a.f64(1)?);
            } else if let Some(p) = view::<dyn DigitalOutPin>(a) {
                p.setup_start_value(a.bool(0)?, a.bool(1)?);
            } else {
                return Err(not_mirrored(a, "setup_start_value"));
            }
            Ok(py_none())
        },
        "setup_cycle_time" => |a| match view::<dyn PwmPin>(a) {
            Some(p) => {
                let hw = match arg_or(a, 1) {
                    Some(v) => v.is_true()?,
                    None => false,
                };
                p.setup_cycle_time(a.f64(0)?, hw)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "setup_cycle_time")),
        },
        "set_pwm" => |a| match view::<dyn PwmPin>(a) {
            Some(p) => {
                p.set_pwm(a.f64(0)?, a.f64(1)?)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "set_pwm")),
        },
        "set_digital" => |a| match view::<dyn DigitalOutPin>(a) {
            Some(p) => {
                p.set_digital(a.f64(0)?, a.bool(1)?)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "set_digital")),
        },
        "setup_adc_sample" => |a| match view::<dyn AdcPin>(a) {
            Some(p) => {
                let n = |i: usize, d: f64| -> KResult<f64> { arg_or(a, i).map_or(Ok(d), |v| v.as_f64()) };
                p.setup_adc_sample(a.f64(0)?, n(1, 8.0)? as i64, n(2, 0.0)?, n(3, 1.0)?, n(4, 0.0)? as i64);
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "setup_adc_sample")),
        },
        "setup_adc_callback" => |a| match view::<dyn AdcPin>(a) {
            Some(p) => {
                let cb = a.arg(1).clone();
                p.setup_adc_callback(a.f64(0)?, Rc::new(move |t, v| cb.call(&[&py_float(t), &py_float(v)]).map(|_| ())));
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "setup_adc_callback")),
        },
        "get_last_value" => |a| match view::<dyn AdcPin>(a) {
            Some(p) => p.get_last_value().to_py(),
            None => Err(not_mirrored(a, "get_last_value")),
        },
        // the probes
        "get_position_endstop" => |a| match view::<dyn Endstop>(a).and_then(|e| e.get_position_endstop()) {
            Some(p) => Ok(py_float(p)),
            None => Err(not_mirrored(a, "get_position_endstop")),
        },
        "multi_probe_begin" => |a| match view::<dyn McuProbe>(a) {
            Some(p) => {
                let restore = match arg_or(a, 0) {
                    Some(v) => v.is_true()?,
                    None => false,
                };
                p.multi_probe_begin(restore)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "multi_probe_begin")),
        },
        "multi_probe_end" => |a| match view::<dyn McuProbe>(a) {
            Some(p) => {
                p.multi_probe_end()?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "multi_probe_end")),
        },
        "probe_prepare" => |a| match view::<dyn McuProbe>(a) {
            Some(p) => {
                p.probe_prepare(&hmove(a.arg(0))?)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "probe_prepare")),
        },
        "probe_finish" => |a| match view::<dyn McuProbe>(a) {
            Some(p) => {
                p.probe_finish(&hmove(a.arg(0))?)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "probe_finish")),
        },
        // probing_move(pos, speed, gcmd) -> (epos, is_good)
        "probing_move" => |a| match view::<dyn McuProbe>(a) {
            Some(p) if a.args.len() == 3 => {
                let pos: Vec<f64> = FromPy::from_py(a.arg(0))?;
                let gcmd = super::gcode::gcmd_from_py(a.arg(2))?;
                let (epos, good) = p.probing_move(&pos, a.f64(1)?, &gcmd)?;
                Ok(py_tuple(vec![epos.to_py()?, good.to_py()?]))
            }
            _ => Err(not_mirrored(a, "probing_move")),
        },
        // the lcd chips
        "init" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => {
                c.init()?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "init")),
        },
        "flush" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => {
                c.flush()?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "flush")),
        },
        "clear" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => {
                c.clear();
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "clear")),
        },
        "write_text" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => {
                c.write_text(a.arg(0).as_i64()?, a.arg(1).as_i64()?, &lcd_data(a.arg(2))?);
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "write_text")),
        },
        "write_graphics" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => {
                c.write_graphics(a.arg(0).as_i64()?, a.arg(1).as_i64()?, &lcd_data(a.arg(2))?);
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "write_graphics")),
        },
        "write_glyph" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => Ok(py_int(c.write_glyph(a.arg(0).as_i64()?, a.arg(1).as_i64()?, &a.string(2)?))),
            None => Err(not_mirrored(a, "write_glyph")),
        },
        "get_dimensions" => |a| match view::<dyn LcdChip>(a) {
            Some(c) => c.get_dimensions().to_py(),
            None => Err(not_mirrored(a, "get_dimensions")),
        },
        // the bulk adc chips
        "get_samples_per_second" => |a| match view::<dyn BulkAdcSensor>(a) {
            Some(s) => Ok(py_int(s.get_samples_per_second())),
            None => Err(not_mirrored(a, "get_samples_per_second")),
        },
        "get_range" => |a| match view::<dyn BulkAdcSensor>(a) {
            Some(s) => s.get_range().to_py(),
            None => Err(not_mirrored(a, "get_range")),
        },
        "get_channel_count" => |a| match view::<dyn BulkAdcSensor>(a) {
            Some(s) => Ok(py_int(s.get_channel_count() as i64)),
            None => Err(not_mirrored(a, "get_channel_count")),
        },
        "attach_load_cell_probe" => |a| match view::<dyn BulkAdcSensor>(a) {
            Some(s) => {
                s.attach_load_cell_probe(a.arg(0).as_i64()?)?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "attach_load_cell_probe")),
        },
        // add_client(callback): callback(msg) returns whether to stay attached
        "add_client" => |a| match view::<dyn BulkAdcSensor>(a) {
            Some(s) => {
                s.add_client(client_callback_from_py(a.arg(0)))?;
                Ok(py_none())
            }
            None => Err(not_mirrored(a, "add_client")),
        },
        // cmd_X(gcmd) of a g-code command X: its registered handler
        n if n.starts_with("cmd_") && n.len() > 4 && n[4..] == n[4..].to_uppercase() => |a| {
            let cmd = &a.name().rsplit('.').next().unwrap_or("")[4..];
            let p = printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?;
            let gcode = p.lookup::<crate::gcode::GCodeDispatch>("gcode")?;
            match gcode.handler_in("base", cmd).or_else(|| gcode.handler_in("active", cmd)) {
                Some(h) => {
                    h(&*super::gcode::gcmd_from_py(a.arg(0))?)?;
                    Ok(py_none())
                }
                None => Err(not_mirrored(a, "")),
            }
        },
        _ => return None,
    };
    Some(m)
}

/// write_text / write_graphics data: a str or bytes
fn lcd_data(o: &PyRef) -> KResult<Vec<u8>> {
    if let Ok(s) = o.as_string() {
        return Ok(s.chars().map(|c| c as u32 as u8).collect());
    }
    FromPy::from_py(&lookup("builtins", "list")?.call(&[o])?)
}

/// self.printer / self.reactor / self.gcode of a class without its own handler
pub fn generic_attr(name: &str) -> Option<Getter> {
    let g: Getter = match name.rsplit('.').next()? {
        "printer" => |_o| printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.to_py(),
        "reactor" => |_o| printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.get_reactor().to_py(),
        // (self.gcode is the g-code dispatcher in every Kalico class)
        "gcode" => |_o| printer().ok_or_else(|| KError::Other("kaliru: no printer".into()))?.lookup_object("gcode")?.to_py(),
        _ => return None,
    };
    Some(g)
}

pub fn register(_r: &mut Registry) {
    use crate::extras as x;
    macro_rules! views {
        ($tr:ty: $($ty:ty),*) => {$(
            objects::register_view::<$ty>(|a| -> Box<dyn Any> { Box::new(a.downcast::<$ty>().expect("type") as Rc<$tr>) });
        )*};
    }
    views!(dyn TempSensor: x::aht10::AHTBase, x::bme280::BME280, x::ds18b20::DS18B20, x::htu21d::HTU21D,
        x::indx::sensors::IndxHeaterTemperatureSensor, x::indx::sensors::IndxTempSensorWrapper, x::lm75::LM75,
        x::mpc_block_temperature::MpcTempWrapper, x::adc_temperature::PrinterADCtoTemperature,
        x::temperature_combined::PrinterSensorCombined, x::temperature_mcu::PrinterTemperatureMCU, x::sht3x::SHT3X,
        x::spi_temperature::SensorBase, x::temperature_host::TemperatureHost);
    views!(dyn PwmPin: x::indx::heater::IndxToolboardHeater, crate::mcu::McuPwm, x::replicape::Pca9685Pwm,
        x::multi_pin::PrinterMultiPin, x::sx1509::SX1509Pwm, x::replicape::ServoPwm);
    views!(dyn DigitalOutPin: crate::mcu::McuDigitalOut, x::replicape::Pca9685Pwm, x::multi_pin::PrinterMultiPin,
        x::replicape::ReplicapeDACEnable, x::sx1509::SX1509DigitalOut);
    views!(dyn AdcPin: crate::mcu::McuAdc, x::adc_scaled::McuScaledAdc);
    views!(dyn McuProbe: x::bltouch::BLTouchEndstopWrapper, x::dockable_probe::DockableProbe,
        x::probe_eddy_current::EddyEndstopWrapper, x::load_cell::load_cell_probe::LoadCellEndstopWrapper,
        x::smart_effector::SmartEffectorEndstopWrapper);
    views!(dyn LcdChip: x::display::aip31068_spi::Aip31068Spi, x::display::st7920::DisplayBase,
        x::display::uc1701::DisplayBase, x::display::hd44780::HD44780, x::display::hd44780_spi::HD44780Spi);
    views!(dyn BulkAdcSensor: x::load_cell::ads1220::ADS1220, x::load_cell::ads131m0x::ADS131MxBase,
        x::load_cell::cs1237::CS1237, x::load_cell::hx71x::HX71xBase);
}

/// A temperature callback as a python callable cb(read_time, temp)
pub fn temp_callback_to_py(cb: crate::extras::heaters::TempCallback) -> KResult<PyRef> {
    crate::plugins::callable::py_function(Rc::new(move |a: &[PyRef]| {
        cb(a[0].as_f64()?, a[1].as_f64()?)?;
        Ok(py_none())
    }))
}

pub fn bulk_adc_to_py(s: Rc<dyn BulkAdcSensor>) -> KResult<PyRef> {
    let name = s.type_name();
    objects::wrap_view(s.clone(), s, name)
}

pub fn lcd_chip_to_py(c: Rc<dyn LcdChip>) -> KResult<PyRef> {
    let name = c.type_name();
    objects::wrap_view(c.clone(), c, name)
}

/// A python callable as a temperature callback
pub fn temp_callback_from_py(o: &PyRef) -> crate::extras::heaters::TempCallback {
    let cb = o.clone();
    Rc::new(move |t, v| cb.call(&[&py_float(t), &py_float(v)]).map(|_| ()))
}

/// A python client callback(msg) -> keep attached
pub fn client_callback_from_py(o: &PyRef) -> crate::extras::bulk_sensor::ClientCallback {
    let cb = o.clone();
    Rc::new(move |msg: &PyVal| match cb.call(&[&from_pyval(msg)]).and_then(|r| r.is_true()) {
        Ok(keep) => keep,
        Err(e) => {
            crate::logging::exception_err("python bulk sensor client", &e);
            false
        }
    })
}
