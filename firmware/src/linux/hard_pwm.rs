// HW PWM upport via Linux PWM sysfs interface
//
// Copyright (C) 2019  Janne Grunau <janne-3d@jannau.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::console::report_errno;
use super::timer::NSECS_PER_TICK;
use crate::sched::R;
use crate::{decl_constant, decl_enumeration_range, shutdown};
use std::ffi::CString;

const MAX_PWM: u32 = 1 << 15;
decl_constant!("PWM_MAX", MAX_PWM);

const HARD_PWM_START: u32 = 1 << 16;
// used by the declarations (a C macro)
#[allow(dead_code)]
const fn hard_pwm(chip: u32, pin: u32) -> u32 {
    (chip << 4) + pin + HARD_PWM_START
}

decl_enumeration_range!("pin", "pwmchip0/pwm0", hard_pwm(0, 0), 16);
decl_enumeration_range!("pin", "pwmchip1/pwm0", hard_pwm(1, 0), 16);
decl_enumeration_range!("pin", "pwmchip2/pwm0", hard_pwm(2, 0), 16);
decl_enumeration_range!("pin", "pwmchip3/pwm0", hard_pwm(3, 0), 16);
decl_enumeration_range!("pin", "pwmchip4/pwm0", hard_pwm(4, 0), 16);
decl_enumeration_range!("pin", "pwmchip5/pwm0", hard_pwm(5, 0), 16);
decl_enumeration_range!("pin", "pwmchip6/pwm0", hard_pwm(6, 0), 16);
decl_enumeration_range!("pin", "pwmchip7/pwm0", hard_pwm(7, 0), 16);

#[derive(Clone, Copy)]
#[repr(C)]
pub struct GpioPwm {
    duty_fd: libc::c_int,
    enable_fd: libc::c_int,
    period: u32,
}

fn errno() -> libc::c_int {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

fn open_wronly(path: &str) -> libc::c_int {
    let c = CString::new(path).unwrap();
    unsafe { libc::open(c.as_ptr(), libc::O_WRONLY | libc::O_CLOEXEC) }
}

/// PWM_PATH "/sys/class/pwm/pwmchip%u/pwm%u/%s", PWM_PATH_BB "/sys/class/pwm/pwm-%u:%u/%s"
fn pwm_file(bb: bool, chip_id: u8, pwm_id: u8, name: &str) -> String {
    if bb {
        format!("/sys/class/pwm/pwm-{}:{}/{}", chip_id, pwm_id, name)
    } else {
        format!("/sys/class/pwm/pwmchip{}/pwm{}/{}", chip_id, pwm_id, name)
    }
}

pub fn gpio_pwm_setup(pin: u32, cycle_time: u32, val: u16) -> R<GpioPwm> {
    let chip_id = (pin.wrapping_sub(HARD_PWM_START) >> 4) as u8;
    let pwm_id = (pin.wrapping_sub(HARD_PWM_START) & 0xf) as u8;
    let mut bb = false;

    let mut g = GpioPwm { duty_fd: 0, enable_fd: 0, period: 0 };
    g.period = cycle_time.wrapping_mul(NSECS_PER_TICK as u32);

    let mut fd;
    'fail: {
        // configure period/cycle time. Always in nanoseconds
        fd = open_wronly(&pwm_file(bb, chip_id, pwm_id, "period"));
        if fd == -1 && errno() == libc::ENOENT {
            // upstream pwm control file does not exists try BeagleBoard scheme
            bb = true;
            fd = open_wronly(&pwm_file(bb, chip_id, pwm_id, "period"));
            if fd == -1 {
                report_errno("pwm /sys/class/pwm/pwm-%u:%u/%speriod for pin", pin as i32);
                break 'fail;
            }
        } else if fd == -1 {
            report_errno("pwm /sys/class/pwm/pwmchip%u/pwm%u/%speriod for pin", pin as i32);
            break 'fail;
        }
        let scratch = format!("{}", cycle_time.wrapping_mul(NSECS_PER_TICK as u32));
        unsafe {
            libc::write(fd, scratch.as_ptr() as *const libc::c_void, scratch.len());
            libc::close(fd);
        }

        // write duty cycle
        fd = open_wronly(&pwm_file(bb, chip_id, pwm_id, "duty_cycle"));
        if fd == -1 {
            report_errno("pwm duty_cycle", fd);
            break 'fail;
        }
        g.duty_fd = fd;

        // enable PWM
        fd = open_wronly(&pwm_file(bb, chip_id, pwm_id, "enable"));
        if fd == -1 {
            report_errno("pwm enable", fd);
            unsafe { libc::close(g.duty_fd) };
            break 'fail;
        }
        g.enable_fd = fd;
        gpio_pwm_write(g, val);

        return Ok(g);
    }
    if fd >= 0 {
        unsafe { libc::close(fd) };
    }
    shutdown!("Unable to config pwm device");
}

pub fn gpio_pwm_write(g: GpioPwm, val: u16) {
    if val == 0 {
        unsafe { libc::write(g.enable_fd, b"0\0".as_ptr() as *const libc::c_void, 2) };
        return;
    }
    let duty_cycle = (g.period as u64 * val as u64 / MAX_PWM as u64) as u32;
    // the digits and their nul
    let scratch = format!("{}\0", duty_cycle);
    unsafe {
        libc::write(g.duty_fd, scratch.as_ptr() as *const libc::c_void, scratch.len());
        libc::write(g.enable_fd, b"1\0".as_ptr() as *const libc::c_void, 2);
    }
}
