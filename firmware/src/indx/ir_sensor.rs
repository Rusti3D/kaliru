// INDX nozzle IR temperature sensor (ir_sensor.h, ir_sensor.c++)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::i2c_bus::I2cBus;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::ctr::Global;
use crate::sched::R;
use crate::shutdown;
use crate::libm::powf;

// (C: a double)
pub const KELVIN_OFFSET: f64 = 273.15;

pub static EEPROM: Global<[u8; 32]> = Global::new([0; 32]);

#[derive(Clone, Copy, Default)]
pub struct IrSensorRawReading {
    pub tp_object: u32,
    pub tp_ambient: u32,
}

#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum IrSensorError {
    #[default]
    Ok,
    NoRadiance,
}

#[derive(Clone, Copy, Default)]
pub struct IrSensorReading {
    pub error: IrSensorError,
    pub object_temperature: f32,
    pub sensor_ambient_temperature: f32,
    pub raw: IrSensorRawReading,
}

impl IrSensorReading {
    // (C: the error argument is not used)
    pub fn new_error(_error: IrSensorError) -> IrSensorReading {
        // Return high temperature for errors as that would make any calculated
        // error terms negative.
        let t = (1000.0f32 as f64 + KELVIN_OFFSET) as f32;
        IrSensorReading {
            error: IrSensorError::NoRadiance,
            object_temperature: t,
            sensor_ambient_temperature: t,
            raw: IrSensorRawReading { tp_object: 0, tp_ambient: 0 },
        }
    }
}

const SENSOR_ADDRESS: u8 = 12;

// Default calibration constants, relative to the ones provided in the sensor
// EEPROM.
const IR_EXPONENT: f32 = 3.9916;
const IR_OBJ_GAIN: f32 = 2.2280;
const IR_BRACKET_GAIN: f32 = -2.4739;

fn delay(us: u32) {
    let end = timer_read_time().wrapping_add(timer_from_us(us));
    while timer_is_before(timer_read_time(), end) {}
}

#[derive(Clone, Copy, Default)]
pub struct IrSensorModel {
    pub ptat25: i32,
    pub inv_m: f32,
    pub u0: i32,
    pub inv_k: f32,
    pub exponent: f32,
    pub inv_exponent: f32,
    pub obj_gain: f32,
    pub bracket_gain: f32,
    pub cal_tobj1: f32,
    pub cal_du: f32,
}

pub struct IrSensor {
    pub bus: &'static I2cBus,
    pub model: IrSensorModel,
}

pub fn ir_sensor_create(i2c: &'static I2cBus) -> R<IrSensor> {
    let mut sensor = IrSensor { bus: i2c, model: IrSensorModel::default() };
    let eeprom = EEPROM.get();

    // Initialize sensor
    if !sensor.i2c_transfer(0, &[0x04], &mut [])? {
        shutdown!("IR sensor didn't reply to general call");
    }

    let mut attempts = 4;
    while attempts > 0 {
        attempts -= 1;
        delay(100);

        // Configure ECR for readout
        if !sensor.i2c_transfer(SENSOR_ADDRESS, &[0x1F, 0x80], &mut [])? {
            if attempts == 0 {
                shutdown!("Could not read IR sensor EEPROM");
            } else {
                continue;
            }
        }

        if !sensor.i2c_transfer(SENSOR_ADDRESS, &[0x20], eeprom)? {
            if attempts == 0 {
                shutdown!("Could not read IR sensor EEPROM");
            } else {
                continue;
            }
        }

        // Disable ECR for readout. Ignore result.
        sensor.i2c_transfer(SENSOR_ADDRESS, &[0x1F, 0x00], &mut [])?;

        let mut actual_checksum = eeprom[0] as u16;
        for &b in &eeprom[3..] {
            actual_checksum = actual_checksum.wrapping_add(b as u16);
        }
        let expected_checksum = ((eeprom[1] as u16) << 8) | eeprom[2] as u16;

        if !(eeprom[0] == 3 && eeprom[31] == (SENSOR_ADDRESS | 0x80) && actual_checksum == expected_checksum) {
            if attempts == 0 {
                shutdown!("IR sensor EEPROM is invalid");
            } else {
                continue;
            }
        }

        sensor.model = IrSensorModel::from_eeprom(eeprom);
        break;
    }
    Ok(sensor)
}

impl IrSensor {
    pub fn i2c_transfer(&self, address: u8, send: &[u8], recv: &mut [u8]) -> R<bool> {
        self.bus.transfer(address, send, recv)
    }

    pub fn read_sensor_raw(&self) -> R<IrSensorRawReading> {
        let send = [0x01];
        let mut recv = [0u8; 4];
        let mut read = false;
        for _ in 0..5 {
            if self.i2c_transfer(SENSOR_ADDRESS, &send, &mut recv)? {
                read = true;
                break;
            }
        }
        if !read {
            shutdown!("Could not read IR sensor value");
        }

        let tp_object = ((recv[0] as u32) << 9) | ((recv[1] as u32) << 1) | ((recv[2] as u32) >> 7);
        let t_ambient = ((recv[2] as u32) & 0x7F) << 8 | recv[3] as u32;

        Ok(IrSensorRawReading { tp_object, tp_ambient: t_ambient })
    }

    pub fn read_sensor(&self, ambient_temperature: f32) -> R<IrSensorReading> {
        let raw_reading = self.read_sensor_raw()?;
        Ok(self.model.translate(raw_reading, ambient_temperature))
    }
}

impl IrSensorModel {
    pub fn from_eeprom(eeprom: &[u8; 32]) -> IrSensorModel {
        let m = ((eeprom[12] as u16) << 8) | eeprom[13] as u16;
        let u0 = ((((eeprom[14] as u32) << 8) | eeprom[15] as u32) + 32768) as i32;
        let uout1 = ((((eeprom[16] as u32) << 8) | eeprom[17] as u32) * 2) as i32;

        let mut model = IrSensorModel {
            ptat25: (((eeprom[10] as u32) << 8) | eeprom[11] as u32) as i32,
            inv_m: 100.0 / m as f32,
            u0,
            cal_tobj1: eeprom[18] as f32,
            cal_du: uout1.wrapping_sub(u0) as f32,
            ..Default::default()
        };
        // Sets exponent/obj_gain/bracket_gain and derives inv_k from the cal point.
        model.set_tuning(IR_EXPONENT, IR_OBJ_GAIN, IR_BRACKET_GAIN);
        model
    }

    pub fn set_tuning(&mut self, exponent: f32, obj_gain: f32, bracket_gain: f32) {
        self.exponent = exponent;
        self.inv_exponent = 1.0 / exponent;
        self.obj_gain = obj_gain;
        self.bracket_gain = bracket_gain;
        // (C: the temperatures in double, then to float for powf)
        self.inv_k = (powf((self.cal_tobj1 as f64 + KELVIN_OFFSET) as f32, exponent)
            - powf((25.0f32 as f64 + KELVIN_OFFSET) as f32, exponent))
            / self.cal_du;
    }

    pub fn translate(&self, raw_reading: IrSensorRawReading, ambient_temp: f32) -> IrSensorReading {
        // Calculate sensor ambient temperature, see datasheet section 9.4
        let sensor_ambient_temperature = ((25.0 + KELVIN_OFFSET)
            + ((raw_reading.tp_ambient as i32).wrapping_sub(self.ptat25) as f32 * self.inv_m) as f64)
            as f32;

        // Calculate radiances
        let rad_ambient = powf(sensor_ambient_temperature, self.exponent);
        let rad_received = self.obj_gain * (raw_reading.tp_object as i32).wrapping_sub(self.u0) as f32 * self.inv_k;
        let rad_bracket = self.bracket_gain * (powf(ambient_temp, self.exponent) - rad_ambient);

        let object_radiance = rad_ambient + rad_received + rad_bracket;

        if object_radiance < 0.0 {
            return IrSensorReading::new_error(IrSensorError::NoRadiance);
        }

        // Convert sum of radiances to temperature
        let mut object_temperature = powf(object_radiance, self.inv_exponent);

        // If object temperature is below ambient temperature, take the mean
        // instead.
        if object_temperature < sensor_ambient_temperature {
            object_temperature = (object_temperature + sensor_ambient_temperature) / 2.0;
        }

        IrSensorReading {
            error: IrSensorError::Ok,
            object_temperature,
            sensor_ambient_temperature,
            raw: raw_reading,
        }
    }
}
