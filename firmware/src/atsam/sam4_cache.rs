// SAM4 cache enable
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::internal::*;
use crate::decl_init;
use crate::sched::R;

pub fn sam4_cache_init() -> R {
    let cmcc = unsafe { &*CMCC };
    if cmcc.CMCC_SR.read() & CMCC_SR_CSTS == 0 {
        cmcc.CMCC_CTRL.write(CMCC_CTRL_CEN);
    }
    Ok(())
}
decl_init!(sam4_cache_init);
