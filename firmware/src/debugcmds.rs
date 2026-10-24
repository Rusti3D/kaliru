// Debugging commands.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::board::{irq_restore, irq_save};
use crate::command::{command_decode_buffer, command_decode_ptr, Args};
use crate::sched::R;
use crate::{decl_command, sendf};
use core::ptr::{read_volatile, write_volatile};

pub fn command_debug_read(args: &Args) -> R {
    let order = args[0] as u8;
    let ptr = command_decode_ptr(args[1]);
    let flag = irq_save();
    let v = unsafe {
        match order {
            1 => read_volatile(ptr as *const u16) as u32,
            2 => read_volatile(ptr as *const u32),
            _ => read_volatile(ptr) as u32,
        }
    };
    irq_restore(flag);
    sendf!("debug_result val=%u", v)
}
decl_command!(command_debug_read, HF_IN_SHUTDOWN, "debug_read order=%c addr=%u");

pub fn command_debug_write(args: &Args) -> R {
    let order = args[0] as u8;
    let ptr = command_decode_ptr(args[1]);
    let v = args[2];
    let flag = irq_save();
    unsafe {
        match order {
            1 => write_volatile(ptr as *mut u16, v as u16),
            2 => write_volatile(ptr as *mut u32, v),
            _ => write_volatile(ptr, v as u8),
        }
    }
    irq_restore(flag);
    Ok(())
}
decl_command!(command_debug_write, HF_IN_SHUTDOWN, "debug_write order=%c addr=%u val=%u");

pub fn command_debug_ping(args: &Args) -> R {
    let data = command_decode_buffer(args[0] as u8 as u32, args[1]);
    sendf!("pong data=%*s", &*data)
}
decl_command!(command_debug_ping, HF_IN_SHUTDOWN, "debug_ping data=%*s");

pub fn command_debug_nop(_args: &Args) -> R {
    Ok(())
}
decl_command!(command_debug_nop, HF_IN_SHUTDOWN, "debug_nop");
