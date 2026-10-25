// Garbage collection optimizations
// (port of klippy/extras/garbage_collection.py)
//
// Copyright (C) 2025  Branden Cash <ammmze@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// Rust has no garbage collector: the handlers keep python's log messages
// (gc.freeze / gc.unfreeze exist on every supported python).
use crate::configfile::ConfigWrapper;
use crate::error::KResult;
use crate::logging;
use crate::printer::{PObj, PrinterObject};
use std::rc::Rc;

pub struct GarbageCollection;

impl PrinterObject for GarbageCollection {
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    let printer = config.get_printer();
    printer.register_event_handler(
        "klippy:ready",
        Rc::new(|_| {
            logging::debug("Running full garbage collection and freezing");
            Ok(())
        }),
    );
    printer.register_event_handler(
        "klippy:disconnect",
        Rc::new(|_| {
            logging::debug("Unfreezing garbage collection");
            Ok(())
        }),
    );
    Ok(Rc::new(GarbageCollection))
}
