// INDX heater control loop and host commands (heater.h, heater.c++)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::coil_driver::{coil_driver_create, CoilDriver};
use super::i2c_bus::I2cBus;
use super::ir_sensor::{ir_sensor_create, IrSensor, IrSensorError, IrSensorReading, EEPROM, KELVIN_OFFSET};
use super::pid::{PidController, PidParams};
use super::status_led_driver::*;
use crate::atsamd::internal::*;
use crate::board::{timer_from_us, timer_is_before, timer_read_time};
use crate::command::Args;
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::sched::{
    sched_add_timer, sched_check_wake, sched_del_timer, sched_is_shutdown, sched_wake_task, TaskWake, Timer, R, SF_DONE,
    SF_RESCHEDULE,
};
use crate::{decl_command, decl_constant_str, decl_shutdown, decl_task, sendf, shutdown};
use crate::libm::sqrtf;

const PIN_SDA: u32 = GPIO('A', 13);
const PIN_SCL: u32 = GPIO('A', 12);
decl_constant_str!("RESERVE_PINS_INDX", "PA13,PA12");

const LIVENESS_CHECK_INTERVAL: u32 = CONFIG_CLOCK_FREQ;
const HEATER_TIMEOUT_US: f32 = 3000000.0;
const UPDATE_RATE_HZ: u32 = 100;
const HEATER_MAX_POWER_CYCLES: u32 = UPDATE_RATE_HZ * 15;
// (C: a double)
const HEATER_MAX_TEMPERATURE: f64 = KELVIN_OFFSET + 315.0;

#[derive(Default)]
pub struct IndxHeaterState {
    pub ambient_temperature: f32,
    pub nozzle_temperature: IrSensorReading,
    pub update_count: u32,
    pub max_power_cycles: u32,
    pub power_sum: f32,
    pub updates_since_report: u32,
}

pub struct IndxHeater {
    pub i2c: &'static I2cBus,
    pub nozzle_temp_sensor: IrSensor,
    pub coil_driver_inst: CoilDriver,
    pub pid: PidController,

    pub status_led: StatusLedDriver,
    pub status_pattern: Option<&'static Pattern>,
    pub cold_timeout: u32,

    pub update_interval: f32,
    pub update_interval_ticks: u32,
    pub update_timer: Timer,
    pub want_update: bool,
    pub debug_stream_raw_ir: bool,

    pub timeout_timer: Timer,
    pub liveness_timer: Timer,

    pub next_temperature_report: u32,
    pub temperature_report_interval: u32,

    pub state: IndxHeaterState,
}

impl IndxHeater {
    pub fn run(&mut self) -> R {
        let now = timer_read_time();

        let mut output: f32 = 0.0;

        self.state.nozzle_temperature = self.nozzle_temp_sensor.read_sensor(self.state.ambient_temperature)?;

        match self.state.nozzle_temperature.error {
            IrSensorError::Ok => {
                if self.state.nozzle_temperature.object_temperature as f64 > HEATER_MAX_TEMPERATURE {
                    shutdown!("Heater maximum safe temperature passed");
                }

                output = self.pid.step(self.state.nozzle_temperature.object_temperature, self.update_interval);
            }
            IrSensorError::NoRadiance => {
                if self.pid.set_point().is_some() {
                    shutdown!("IR sensor calculated no radiance");
                }
            }
        }

        if output >= 1.0 {
            self.state.max_power_cycles += 1;
            if self.state.max_power_cycles >= HEATER_MAX_POWER_CYCLES && self.coil_driver_inst.duty_limit == 0 {
                shutdown!("Heater was at maximum power for too long");
            }
        } else {
            self.state.max_power_cycles = 0;
        }
        self.coil_driver_inst.set_duty(output)?;

        self.state.power_sum += output;
        self.state.updates_since_report += 1;
        if timer_is_before(self.next_temperature_report, now) {
            self.report_temperature(now)?;
            self.next_temperature_report = self.next_temperature_report.wrapping_add(self.temperature_report_interval);
        }

        // Determine status pattern to use

        let new_pattern: &'static Pattern;

        if let Some(set_point) = self.pid.set_point() {
            let error = set_point - self.state.nozzle_temperature.object_temperature;
            if error > 1.0 {
                new_pattern = &PATTERN_HEATING;
            } else {
                new_pattern = &PATTERN_TARGET;
            }
            // Reset the cold timeout so we can show the hot->cold transition
            // effect when reaching cold.
            self.cold_timeout = 100 * 60;
        } else {
            let cold = (self.state.nozzle_temperature.object_temperature as f64) < KELVIN_OFFSET + 50.0f32 as f64;
            if !cold {
                new_pattern = &PATTERN_COOLING_DOWN;
            } else if self.cold_timeout > 0 {
                self.cold_timeout -= 1;
                new_pattern = &PATTERN_COOLED_DOWN;
            } else {
                new_pattern = &PATTERN_COLD;
            }
        }

        if !self.status_pattern.is_some_and(|p| core::ptr::eq(p, new_pattern)) {
            self.status_pattern = Some(new_pattern);
            self.status_led.set_pattern(*new_pattern);
        }

        // Update status LED after the main PID loop, as it's less timing sensitive.
        // This also ensures that a status LED update doesn't hold back IR sensor
        // readout.
        self.status_led.tick()?;

        self.state.update_count += 1;
        Ok(())
    }

    pub fn report_temperature(&mut self, clock: u32) -> R {
        let updates_f = self.state.updates_since_report as f32;
        let power = self.state.power_sum / updates_f;

        // (C: the scaling in double)
        sendf!(
            "indx_nozzle_temp clock=%u nozzle=%u sensor=%u power=%u charge=%u overvoltage=%u",
            clock,
            (self.state.nozzle_temperature.object_temperature as f64 * 100.0) as i32,
            (self.state.nozzle_temperature.sensor_ambient_temperature as f64 * 100.0) as i32,
            (power as f64 * 1000000.0) as i32,
            self.coil_driver_inst.get_total_charge(),
            self.coil_driver_inst.get_ov_count()
        )?;

        if self.debug_stream_raw_ir {
            sendf!(
                "indx_debug_raw_ir object=%u ambient=%u",
                self.state.nozzle_temperature.raw.tp_object,
                self.state.nozzle_temperature.raw.tp_ambient
            )?;
        }

        self.state.power_sum = 0.0;
        self.state.updates_since_report = 0;
        Ok(())
    }

    pub fn shutdown_(&self) {
        self.coil_driver_inst.shutdown_();
    }
}

static I2C: Global<Option<I2cBus>> = Global::new(None);
static INDX_HEATER_INSTANCE: Global<Option<IndxHeater>> = Global::new(None);
static INDX_HEATER_WAKE: Global<TaskWake> = Global::new(TaskWake::new());

fn instance() -> &'static mut Option<IndxHeater> {
    INDX_HEATER_INSTANCE.get()
}

unsafe fn indx_heater_update_wake(timer: *mut Timer) -> R<u8> {
    // (C: the instance is dereferenced unchecked; the timer runs only with one)
    let h = instance().as_mut().unwrap_unchecked();
    h.want_update = true;
    sched_wake_task(INDX_HEATER_WAKE.ptr());
    (*timer).waketime = (*timer).waketime.wrapping_add(h.update_interval_ticks);
    Ok(SF_RESCHEDULE)
}

unsafe fn indx_heater_timeout(_timer: *mut Timer) -> R<u8> {
    shutdown!("INDX heater watchdog timeout");
    #[allow(unreachable_code)]
    Ok(SF_DONE)
}

static LAST_UPDATE_COUNT: Global<u32> = Global::new(0);

unsafe fn indx_heater_liveness_check(timer: *mut Timer) -> R<u8> {
    let new_update_count = instance().as_ref().unwrap_unchecked().state.update_count;
    if new_update_count == *LAST_UPDATE_COUNT.get() {
        shutdown!("INDX heater liveness check failed");
    }
    *LAST_UPDATE_COUNT.get() = new_update_count;
    (*timer).waketime = (*timer).waketime.wrapping_add(LIVENESS_CHECK_INTERVAL);
    Ok(SF_RESCHEDULE)
}

pub fn indx_heater_create(bus: &'static I2cBus) -> R<IndxHeater> {
    let mut status_led = StatusLedDriver::new(bus)?;
    status_led.set_currents(0xFF, 0xCC, 0xCC)?;
    let nozzle_temp_sensor = ir_sensor_create(bus)?;
    let coil_driver = coil_driver_create()?;
    let pid = PidController::new(PidParams::zero());

    let update_interval = 1.0 / UPDATE_RATE_HZ as f32;

    Ok(IndxHeater {
        i2c: bus,
        nozzle_temp_sensor,
        coil_driver_inst: coil_driver,
        pid,
        status_led,
        status_pattern: None,
        cold_timeout: 0,

        update_interval,
        update_interval_ticks: timer_from_us((update_interval * 1000000.0) as u32),
        update_timer: Timer::new(Some(indx_heater_update_wake)),
        want_update: true,
        debug_stream_raw_ir: false,

        timeout_timer: Timer::new(Some(indx_heater_timeout)),
        liveness_timer: Timer::new(Some(indx_heater_liveness_check)),

        next_temperature_report: timer_read_time(),
        temperature_report_interval: timer_from_us(100000),

        state: IndxHeaterState { ambient_temperature: (25.0 + KELVIN_OFFSET) as f32, ..Default::default() },
    })
}

fn indx_init() -> R {
    if instance().is_some() {
        shutdown!("INDX heater already active");
    }

    gpio_peripheral(PIN_SDA, 'D', 0);
    gpio_peripheral(PIN_SCL, 'D', 0);

    *I2C.get() = Some(I2cBus::new(unsafe { &*SERCOM4 }.I2CM(), SERCOM4_GCLK_ID_CORE, ID_SERCOM4));

    let bus: &'static I2cBus = I2C.get().as_ref().unwrap();
    *instance() = Some(indx_heater_create(bus)?);
    let h = instance().as_mut().unwrap();

    h.update_timer.waketime = timer_read_time().wrapping_add(h.update_interval_ticks);
    sched_add_timer(&mut h.update_timer)?;
    sched_wake_task(INDX_HEATER_WAKE.ptr());

    h.liveness_timer.waketime = timer_read_time().wrapping_add(LIVENESS_CHECK_INTERVAL);
    sched_add_timer(&mut h.liveness_timer)
}

pub fn command_indx_set_target(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    let set_point = f32::from_bits(args[0]);
    if set_point.is_nan() {
        h.pid.update_set_point(None);
        sched_del_timer(&mut h.timeout_timer);
    } else {
        h.pid.update_set_point(Some(set_point));
        h.timeout_timer.waketime = timer_read_time().wrapping_add(timer_from_us(HEATER_TIMEOUT_US as u32));
        sched_del_timer(&mut h.timeout_timer);
        sched_add_timer(&mut h.timeout_timer)?;
    }
    Ok(())
}
decl_command!(command_indx_set_target, "indx_set_target target=%u");

pub fn command_indx_set_bracket_temp(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    let temp = f32::from_bits(args[0]);
    if temp.is_nan() {
        h.state.ambient_temperature = (KELVIN_OFFSET + 25.0) as f32; // Fallback value
    } else {
        h.state.ambient_temperature = temp;
    }
    Ok(())
}
decl_command!(command_indx_set_bracket_temp, "indx_set_bracket_temp temp=%u");

pub fn command_indx_set_ir_sensor_params(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    let exponent = f32::from_bits(args[0]);
    let obj_gain = f32::from_bits(args[1]);
    let bracket_gain = f32::from_bits(args[2]);
    if !exponent.is_nan() && !obj_gain.is_nan() && !bracket_gain.is_nan() {
        h.nozzle_temp_sensor.model.set_tuning(exponent, obj_gain, bracket_gain);
    }
    Ok(())
}
decl_command!(
    command_indx_set_ir_sensor_params,
    "indx_set_ir_sensor_params exponent=%u obj_gain=%u bracket_gain=%u"
);

pub fn command_indx_set_control_params(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    let kp = f32::from_bits(args[0]);
    let ti = f32::from_bits(args[1]);
    let td = f32::from_bits(args[2]);
    let b = f32::from_bits(args[3]);
    let i_window = f32::from_bits(args[4]);
    let i_limit = f32::from_bits(args[5]);
    // (C: the td test in double)
    let tt = if (td as f64) < 0.000001 { ti / 8.0 } else { sqrtf(ti * td) };
    let params = PidParams { kp, ti, td, b, tt, i_window, i_limit };

    h.pid.update_params(params);
    Ok(())
}
decl_command!(
    command_indx_set_control_params,
    "indx_set_control_params kp=%u ti=%u td=%u b=%u i_window=%u i_limit=%u"
);

pub fn command_indx_set_coil_driver_params(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    let time_on = f32::from_bits(args[0]);
    let time_off = f32::from_bits(args[1]);
    let time_on_first = f32::from_bits(args[2]);
    h.coil_driver_inst.set_timings(time_on, time_off, time_on_first);
    Ok(())
}
decl_command!(
    command_indx_set_coil_driver_params,
    "indx_set_coil_driver_params time_on=%u time_off=%u time_on_first=%u"
);

pub fn command_indx_cycle_limit(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    h.coil_driver_inst.set_cycle_limit(args[0]);
    Ok(())
}
decl_command!(command_indx_cycle_limit, "indx_cycle_limit limit=%u");

// Start the on-device coil-driver auto-tune (commissioning). start_tune()
// faults if the coil is being driven; tuning and heating are mutually
// exclusive. Advanced to completion by tune_step() from indx_heater_task.
pub fn command_indx_tune_coil(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    h.coil_driver_inst.start_tune(args[0] != 0)
}
decl_command!(command_indx_tune_coil, "indx_tune_coil debug=%c");

// Poll the latched tune outcome (replies indx_coil_tune_result). Reliable
// because the host can re-query; a dropped completion notification is fine.
pub fn command_indx_query_coil_tune(_args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    h.coil_driver_inst.report_tune_status()
}
decl_command!(command_indx_query_coil_tune, "indx_query_coil_tune");

// Read back the drive timings currently in effect (replies
// indx_coil_driver_params).
pub fn command_indx_query_coil_driver_params(_args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    h.coil_driver_inst.report_params()
}
decl_command!(command_indx_query_coil_driver_params, "indx_query_coil_driver_params");

pub fn command_indx_led_force_color(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    if args[3] != 0 {
        h.status_led.force_color(Some(Color::new(args[0] as u8, args[1] as u8, args[2] as u8)))
    } else {
        h.status_led.force_color(None)
    }
}
decl_command!(command_indx_led_force_color, "indx_led_force_color r=%c g=%c b=%c force=%c");

pub fn command_indx_led_set_current(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    h.status_led.set_currents(args[0] as u8, args[1] as u8, args[2] as u8)
}
decl_command!(command_indx_led_set_current, "indx_led_set_current r=%c g=%c b=%c");

pub fn command_indx_debug_stream_raw_ir_sensor(args: &Args) -> R {
    let Some(h) = instance() else { return Ok(()) };

    h.debug_stream_raw_ir = args[0] != 0;
    Ok(())
}
decl_command!(command_indx_debug_stream_raw_ir_sensor, "indx_debug_stream_raw_ir_sensor enable=%c");

pub fn command_indx_debug_ir_sensor_eeprom(_args: &Args) -> R {
    sendf!("indx_debug_ir_sensor_eeprom_data data=%*s", &*EEPROM.get())
}
decl_command!(command_indx_debug_ir_sensor_eeprom, "indx_debug_ir_sensor_eeprom");

pub fn indx_shutdown() -> R {
    if let Some(h) = instance() {
        h.shutdown_();
        h.pid.update_set_point(None);

        h.update_timer.waketime = timer_read_time().wrapping_add(h.update_interval_ticks);
        sched_add_timer(&mut h.update_timer)?;
    }
    Ok(())
}
decl_shutdown!(indx_shutdown);

pub fn indx_heater_task() -> R {
    if !sched_is_shutdown() && instance().is_none() {
        indx_init()?;
    }
    let Some(h) = instance() else { return Ok(()) };

    // Pump the auto-tuner every scheduler pass while it runs, so bounded test
    // bursts fire back to back. Cheap flag check otherwise.
    let coil = &mut h.coil_driver_inst;
    if coil.tune_active() && !sched_is_shutdown() {
        coil.tune_step()?;
    }

    if !sched_check_wake(INDX_HEATER_WAKE.ptr()) {
        return Ok(());
    }
    if h.want_update {
        h.run()?;
        h.want_update = false;
    }
    Ok(())
}
decl_task!(indx_heater_task);
