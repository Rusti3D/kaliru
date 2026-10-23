// Kalico "danger options" (port of klippy/extras/danger_options.py)
use crate::configfile::{ConfigWrapper, Lim};
use crate::error::KResult;
use crate::printer::{PObj, PrinterObject};
use std::any::Any;
use std::cell::RefCell;
use std::rc::Rc;

pub struct DangerOptions {
    pub minimal_logging: bool,
    pub log_statistics: bool,
    pub log_config_file_at_startup: bool,
    pub log_bed_mesh_at_startup: bool,
    pub log_velocity_limit_changes: bool,
    pub log_pressure_advance_changes: bool,
    pub log_shutdown_info: bool,
    pub log_serial_reader_warnings: bool,
    pub log_startup_info: bool,
    pub log_webhook_method_register_messages: bool,
    pub error_on_unused_config_options: bool,
    pub allow_plugin_override: bool,
    pub single_mcu_trsync_timeout: f64,
    pub multi_mcu_trsync_timeout: f64,
    pub homing_elapsed_distance_tolerance: f64,
    pub temp_ignore_limits: bool,
    pub autosave_includes: bool,
    pub bgflush_extra_time: f64,
    pub homing_start_delay: f64,
    pub endstop_sample_time: f64,
    pub endstop_sample_count: i64,
    // Extruder safety limit overrides
    pub override_pressure_advance_smooth_time_max: f64,
}

impl PrinterObject for DangerOptions {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl DangerOptions {
    pub fn new(config: &ConfigWrapper) -> KResult<Self> {
        let minimal_logging = config.getboolean("minimal_logging", false)?;
        let verbose = !minimal_logging;
        let log_statistics = config.getboolean("log_statistics", verbose)?;
        let log_config_file_at_startup = config.getboolean("log_config_file_at_startup", verbose)?;
        let log_bed_mesh_at_startup = config.getboolean("log_bed_mesh_at_startup", verbose)?;
        let log_velocity_limit_changes = config.getboolean("log_velocity_limit_changes", verbose)?;
        let log_pressure_advance_changes = config.getboolean("log_pressure_advance_changes", verbose)?;
        let log_shutdown_info = config.getboolean("log_shutdown_info", verbose)?;
        let log_serial_reader_warnings = config.getboolean("log_serial_reader_warnings", verbose)?;
        let log_startup_info = config.getboolean("log_startup_info", verbose)?;
        let log_webhook_method_register_messages = config.getboolean("log_webhook_method_register_messages", verbose)?;
        let error_on_unused_config_options = config.getboolean("error_on_unused_config_options", true)?;
        let allow_plugin_override = config.getboolean("allow_plugin_override", false)?;
        let single_mcu_trsync_timeout = config.getfloat("single_mcu_trsync_timeout", 0.25, Lim::new().min(0.0))?;
        let multi_mcu_trsync_timeout = config.getfloat("multi_mcu_trsync_timeout", 0.025, Lim::new().min(0.0))?;
        let homing_elapsed_distance_tolerance = config.getfloat("homing_elapsed_distance_tolerance", 0.5, Lim::new().min(0.0))?;

        let mut temp_ignore_limits = false;
        if config.getboolean_opt("temp_ignore_limits")?.is_none() {
            if let Some(adc_ignore_limits) = config.getboolean_opt("adc_ignore_limits")? {
                config.deprecate("adc_ignore_limits", None);
                temp_ignore_limits = adc_ignore_limits;
            }
        }
        let temp_ignore_limits = config.getboolean("temp_ignore_limits", temp_ignore_limits)?;

        Ok(DangerOptions {
            minimal_logging,
            log_statistics,
            log_config_file_at_startup,
            log_bed_mesh_at_startup,
            log_velocity_limit_changes,
            log_pressure_advance_changes,
            log_shutdown_info,
            log_serial_reader_warnings,
            log_startup_info,
            log_webhook_method_register_messages,
            error_on_unused_config_options,
            allow_plugin_override,
            single_mcu_trsync_timeout,
            multi_mcu_trsync_timeout,
            homing_elapsed_distance_tolerance,
            temp_ignore_limits,
            autosave_includes: config.getboolean("autosave_includes", false)?,
            bgflush_extra_time: config.getfloat("bgflush_extra_time", 0.250, Lim::new().min(0.0))?,
            homing_start_delay: config.getfloat("homing_start_delay", 0.001, Lim::new().min(0.0))?,
            endstop_sample_time: config.getfloat("endstop_sample_time", 0.000015, Lim::new().min(0i64))?,
            endstop_sample_count: config.getint("endstop_sample_count", 4, Lim::new().min(1i64))?,
            override_pressure_advance_smooth_time_max: config.getfloat(
                "override_pressure_advance_smooth_time_max",
                0.200,
                Lim::new().above(0.0),
            )?,
        })
    }
}

thread_local! {
    static DANGER_OPTIONS: RefCell<Option<Rc<DangerOptions>>> = const { RefCell::new(None) };
}

/// get_danger_options() (python raises "DangerOptions has not been loaded yet!")
pub fn get_danger_options() -> Rc<DangerOptions> {
    DANGER_OPTIONS.with(|d| d.borrow().clone()).expect("DangerOptions has not been loaded yet!")
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let d = Rc::new(DangerOptions::new(config)?);
    DANGER_OPTIONS.with(|g| *g.borrow_mut() = Some(d.clone()));
    Ok(d)
}
