// Support for tracking canbus node ids (port of klippy/extras/canbus_ids.py)
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::printer::{PObj, PrinterObject};
use crate::IndexMap;
use std::cell::RefCell;
use std::rc::Rc;

const NODEID_FIRST: i64 = 4;

pub struct PrinterCANBus {
    pub ids: RefCell<IndexMap<String, i64>>,
}

impl PrinterCANBus {
    pub fn add_uuid(&self, _config: &ConfigWrapper, canbus_uuid: &str, _canbus_iface: &str) -> KResult<i64> {
        if self.ids.borrow().contains_key(canbus_uuid) {
            return Err(KError::config("Duplicate canbus_uuid"));
        }
        let new_id = self.ids.borrow().len() as i64 + NODEID_FIRST;
        self.ids.borrow_mut().insert(canbus_uuid.to_string(), new_id);
        Ok(new_id)
    }

    pub fn get_nodeid(&self, canbus_uuid: &str) -> KResult<i64> {
        self.ids.borrow().get(canbus_uuid).copied().ok_or_else(|| KError::config(format!("Unknown canbus_uuid {}", canbus_uuid)))
    }
}

impl PrinterObject for PrinterCANBus {
}

pub fn load_config(_config: &ConfigWrapper) -> KResult<PObj> {
    Ok(Rc::new(PrinterCANBus { ids: RefCell::new(IndexMap::default()) }))
}
