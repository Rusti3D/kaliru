// Load Cell Probe (port of klippy/extras/load_cell_probe.py)
//
// Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::extras::load_cell::load_cell_probe::{empty_classifier, load, ClassifierFactory};
use crate::extras::load_cell::sensor_types;
use crate::extras::load_cell::tap_quality_classifier::TapQualityClassifier;
use crate::printer::{PObj, SubsystemComponentCollection};
use std::rc::Rc;

// register sensors that implement LoadCellSensor
pub fn register_components(subsystem: &SubsystemComponentCollection) -> KResult<()> {
    for (name, sensor) in sensor_types() {
        subsystem.register_component("load_cell_probe_sensors", name, Rc::new(sensor))?;
    }
    let key = "load_cell_probe_tap_classifiers";
    subsystem.register_component(key, "empty", Rc::new(empty_classifier as ClassifierFactory))?;
    subsystem.register_component(key, "tap_quality", Rc::new(TapQualityClassifier::new as ClassifierFactory))?;
    Ok(())
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    load(config)
}
