// Ports of klippy/extras and the module registry that replaces python's
// pkgutil scan of klippy/extras (+ the empty klippy/plugins).
//
// MODULES lists every module of Kalico's extras directory, sorted like
// pkgutil.iter_modules (the order register_components runs in). Entries with
// `ported: false` exist in Kalico but not in Kaliru yet: loading their
// config sections fails with "Module 'x' is not ported to Kaliru yet".
// Regenerate the names with: ls klippy/extras (*.py and packages), sorted.
use crate::printer::PrinterModule;

pub mod adc_temperature;
pub mod aht10;
pub mod bed_mesh;
pub mod bed_tilt;
pub mod bltouch;
pub mod bme280;
pub mod board_pins;
pub mod bulk_sensor;
pub mod buttons;
pub mod control_mpc;
pub mod danger_options;
pub mod delta_calibrate;
pub mod display_status;
pub mod display_templates;
pub mod ds18b20;
pub mod error_mcu;
pub mod exclude_object;
pub mod extruder_stepper;
pub mod fan;
pub mod fan_generic;
pub mod filament_motion_sensor;
pub mod filament_switch_sensor;
pub mod force_move;
pub mod garbage_collection;
pub mod gcode_macro;
pub mod gcode_move;
pub mod heater_bed;
pub mod heater_fan;
pub mod heater_generic;
pub mod heaters;
pub mod homing;
pub mod homing_override;
pub mod htu21d;
pub mod idle_timeout;
pub mod led;
pub mod lm75;
pub mod manual_probe;
pub mod motion_queuing;
pub mod motion_report;
pub mod mpc_ambient_temperature;
pub mod mpc_block_temperature;
pub mod output_pin;
pub mod pause_resume;
pub mod pid_calibrate;
pub mod print_stats;
pub mod probe;
pub mod pulse_counter;
pub mod quad_gantry_level;
pub mod query_adc;
pub mod query_endstops;
pub mod respond;
pub mod safe_z_home;
pub mod sht3x;
pub mod spi_temperature;
pub mod statistics;
pub mod stepper_enable;
pub mod telemetry;
pub mod temperature_combined;
pub mod temperature_fan;
pub mod temperature_host;
pub mod temperature_mcu;
pub mod temperature_sensor;
pub mod thermistor;
pub mod tuning_tower;
pub mod verify_heater;
pub mod virtual_sdcard;
pub mod z_tilt;

const fn todo(name: &'static str) -> PrinterModule {
    PrinterModule { name, ported: false, load_config: None, load_config_prefix: None, register_components: None }
}

pub static MODULES: &[PrinterModule] = &[
    todo("ad5206"),
    todo("adc_scaled"),
    PrinterModule { name: "adc_temperature", ported: true, load_config: Some(adc_temperature::load_config), load_config_prefix: Some(adc_temperature::load_config_prefix), register_components: None },
    todo("adxl345"),
    PrinterModule { name: "aht10", ported: true, load_config: Some(aht10::load_config), load_config_prefix: None, register_components: None },
    todo("angle"),
    todo("axis_twist_compensation"),
    PrinterModule { name: "bed_mesh", ported: true, load_config: Some(bed_mesh::load_config), load_config_prefix: None, register_components: None },
    todo("bed_screws"),
    PrinterModule { name: "bed_tilt", ported: true, load_config: Some(bed_tilt::load_config), load_config_prefix: None, register_components: None },
    todo("belay"),
    PrinterModule { name: "bltouch", ported: true, load_config: Some(bltouch::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "bme280", ported: true, load_config: Some(bme280::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "board_pins", ported: true, load_config: Some(board_pins::load_config), load_config_prefix: Some(board_pins::load_config_prefix), register_components: None },
    PrinterModule { name: "bulk_sensor", ported: true, load_config: None, load_config_prefix: None, register_components: None },
    todo("bus"),
    PrinterModule { name: "buttons", ported: true, load_config: Some(buttons::load_config), load_config_prefix: None, register_components: None },
    todo("canbus_ids"),
    todo("canbus_stats"),
    PrinterModule { name: "control_mpc", ported: true, load_config: None, load_config_prefix: None, register_components: None },
    todo("controller_fan"),
    todo("dac084S085"),
    PrinterModule { name: "danger_options", ported: true, load_config: Some(danger_options::load_config), load_config_prefix: None, register_components: None },
    todo("delayed_gcode"),
    PrinterModule { name: "delta_calibrate", ported: true, load_config: Some(delta_calibrate::load_config), load_config_prefix: None, register_components: None },
    todo("display"),
    PrinterModule { name: "display_status", ported: true, load_config: Some(display_status::load_config), load_config_prefix: None, register_components: None },
    todo("dockable_probe"),
    todo("dotstar"),
    PrinterModule { name: "ds18b20", ported: true, load_config: Some(ds18b20::load_config), load_config_prefix: None, register_components: None },
    todo("duplicate_pin_override"),
    todo("endstop_phase"),
    PrinterModule { name: "error_mcu", ported: true, load_config: Some(error_mcu::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "exclude_object", ported: true, load_config: Some(exclude_object::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "extruder_stepper", ported: true, load_config: None, load_config_prefix: Some(extruder_stepper::load_config_prefix), register_components: None },
    PrinterModule { name: "fan", ported: true, load_config: Some(fan::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "fan_generic", ported: true, load_config: None, load_config_prefix: Some(fan_generic::load_config_prefix), register_components: None },
    PrinterModule { name: "filament_motion_sensor", ported: true, load_config: None, load_config_prefix: Some(filament_motion_sensor::load_config_prefix), register_components: None },
    PrinterModule { name: "filament_switch_sensor", ported: true, load_config: None, load_config_prefix: Some(filament_switch_sensor::load_config_prefix), register_components: None },
    todo("firmware_retraction"),
    PrinterModule { name: "force_move", ported: true, load_config: Some(force_move::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "garbage_collection", ported: true, load_config: Some(garbage_collection::load_config), load_config_prefix: None, register_components: None },
    todo("gcode_arcs"),
    todo("gcode_button"),
    PrinterModule { name: "gcode_macro", ported: true, load_config: Some(gcode_macro::load_config), load_config_prefix: Some(gcode_macro::load_config_prefix), register_components: None },
    PrinterModule { name: "gcode_move", ported: true, load_config: Some(gcode_move::load_config), load_config_prefix: None, register_components: None },
    todo("gcode_shell_command"),
    todo("hall_filament_width_sensor"),
    todo("heated_fan"),
    PrinterModule { name: "heater_bed", ported: true, load_config: Some(heater_bed::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "heater_fan", ported: true, load_config: None, load_config_prefix: Some(heater_fan::load_config_prefix), register_components: None },
    PrinterModule { name: "heater_generic", ported: true, load_config: None, load_config_prefix: Some(heater_generic::load_config_prefix), register_components: None },
    PrinterModule { name: "heaters", ported: true, load_config: Some(heaters::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "homing", ported: true, load_config: Some(homing::load_config), load_config_prefix: None, register_components: None },
    todo("homing_heaters"),
    PrinterModule { name: "homing_override", ported: true, load_config: Some(homing_override::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "htu21d", ported: true, load_config: Some(htu21d::load_config), load_config_prefix: None, register_components: None },
    todo("icm20948"),
    PrinterModule { name: "idle_timeout", ported: true, load_config: Some(idle_timeout::load_config), load_config_prefix: None, register_components: None },
    todo("indx"),
    todo("input_shaper"),
    todo("ldc1612"),
    PrinterModule { name: "led", ported: true, load_config: None, load_config_prefix: Some(led::load_config_prefix), register_components: None },
    todo("lis2dw"),
    todo("lis3dh"),
    PrinterModule { name: "lm75", ported: true, load_config: Some(lm75::load_config), load_config_prefix: None, register_components: None },
    todo("load_cell"),
    todo("load_cell_probe"),
    PrinterModule { name: "manual_probe", ported: true, load_config: Some(manual_probe::load_config), load_config_prefix: None, register_components: None },
    todo("manual_stepper"),
    todo("mcp4018"),
    todo("mcp4451"),
    todo("mcp4728"),
    todo("mixing_extruder"),
    PrinterModule { name: "motion_queuing", ported: true, load_config: Some(motion_queuing::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "motion_report", ported: true, load_config: Some(motion_report::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "mpc_ambient_temperature", ported: true, load_config: Some(mpc_ambient_temperature::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "mpc_block_temperature", ported: true, load_config: Some(mpc_block_temperature::load_config), load_config_prefix: None, register_components: None },
    todo("mpu9250"),
    todo("multi_pin"),
    todo("neopixel"),
    todo("nozzle_cleanup"),
    PrinterModule { name: "output_pin", ported: true, load_config: None, load_config_prefix: Some(output_pin::load_config_prefix), register_components: None },
    todo("palette2"),
    PrinterModule { name: "pause_resume", ported: true, load_config: Some(pause_resume::load_config), load_config_prefix: None, register_components: None },
    todo("pca9533"),
    todo("pca9632"),
    PrinterModule { name: "pid_calibrate", ported: true, load_config: Some(pid_calibrate::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "print_stats", ported: true, load_config: Some(print_stats::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "probe", ported: true, load_config: Some(probe::load_config), load_config_prefix: Some(probe::load_config_prefix), register_components: None },
    todo("probe_eddy_current"),
    PrinterModule { name: "pulse_counter", ported: true, load_config: None, load_config_prefix: None, register_components: None },
    todo("pwm_cycle_time"),
    todo("pwm_tool"),
    PrinterModule { name: "quad_gantry_level", ported: true, load_config: Some(quad_gantry_level::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "query_adc", ported: true, load_config: Some(query_adc::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "query_endstops", ported: true, load_config: Some(query_endstops::load_config), load_config_prefix: None, register_components: None },
    todo("replicape"),
    todo("resonance_tester"),
    PrinterModule { name: "respond", ported: true, load_config: Some(respond::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "safe_z_home", ported: true, load_config: Some(safe_z_home::load_config), load_config_prefix: None, register_components: None },
    todo("samd_sercom"),
    todo("save_variables"),
    todo("screws_tilt_adjust"),
    todo("sdcard_loop"),
    todo("servo"),
    todo("shaper_calibrate"),
    todo("shaper_defs"),
    PrinterModule { name: "sht3x", ported: true, load_config: Some(sht3x::load_config), load_config_prefix: None, register_components: None },
    todo("skew_correction"),
    todo("smart_effector"),
    PrinterModule { name: "spi_temperature", ported: true, load_config: Some(spi_temperature::load_config), load_config_prefix: None, register_components: None },
    todo("static_digital_output"),
    todo("static_pwm_clock"),
    PrinterModule { name: "statistics", ported: true, load_config: Some(statistics::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "stepper_enable", ported: true, load_config: Some(stepper_enable::load_config), load_config_prefix: None, register_components: None },
    todo("sx1509"),
    PrinterModule { name: "telemetry", ported: true, load_config: Some(telemetry::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "temperature_combined", ported: true, load_config: Some(temperature_combined::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "temperature_fan", ported: true, load_config: None, load_config_prefix: Some(temperature_fan::load_config_prefix), register_components: None },
    PrinterModule { name: "temperature_host", ported: true, load_config: Some(temperature_host::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "temperature_mcu", ported: true, load_config: Some(temperature_mcu::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "temperature_sensor", ported: true, load_config: None, load_config_prefix: Some(temperature_sensor::load_config_prefix), register_components: None },
    PrinterModule { name: "thermistor", ported: true, load_config: None, load_config_prefix: Some(thermistor::load_config_prefix), register_components: None },
    todo("tmc"),
    todo("tmc2130"),
    todo("tmc2208"),
    todo("tmc2209"),
    todo("tmc2240"),
    todo("tmc2660"),
    todo("tmc5160"),
    todo("tmc5262"),
    todo("tmc_uart"),
    todo("tools_calibrate"),
    todo("trad_rack"),
    todo("tsl1401cl_filament_width_sensor"),
    PrinterModule { name: "tuning_tower", ported: true, load_config: Some(tuning_tower::load_config), load_config_prefix: None, register_components: None },
    PrinterModule { name: "verify_heater", ported: true, load_config: None, load_config_prefix: Some(verify_heater::load_config_prefix), register_components: None },
    PrinterModule { name: "virtual_sdcard", ported: true, load_config: Some(virtual_sdcard::load_config), load_config_prefix: None, register_components: None },
    todo("z_calibration"),
    todo("z_thermal_adjust"),
    PrinterModule { name: "z_tilt", ported: true, load_config: Some(z_tilt::load_config), load_config_prefix: None, register_components: None },
    todo("z_tilt_ng"),
];
