// INDX status LED (status_led_driver.h, status_led_driver.c++)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::i2c_bus::I2cBus;
use crate::sched::R;
use crate::libm::sin;

const TICK_RATE: f32 = 1.0 / 100.0;

#[derive(Clone, Copy)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub const fn new(r: u8, g: u8, b: u8) -> Color {
        Color { r, g, b }
    }

    pub fn scale(self, scale: f32) -> Color {
        Color::new((self.r as f32 * scale) as u8, (self.g as f32 * scale) as u8, (self.b as f32 * scale) as u8)
    }

    pub fn blend(a: Color, b: Color, b_ratio: f32) -> Color {
        let a_ratio = 1.0 - b_ratio;
        Color::new(
            (b.r as f32 * b_ratio + a.r as f32 * a_ratio) as u8,
            (b.g as f32 * b_ratio + a.g as f32 * a_ratio) as u8,
            (b.b as f32 * b_ratio + a.b as f32 * a_ratio) as u8,
        )
    }
}

#[derive(Clone, Copy)]
pub struct Pattern {
    pub color: Color,
    pub blinks_fast: u8,
    pub blinks_slow: u8,
    pub breathing_freq: f32,
    pub blend_time: f32,
}

const fn pattern(color: Color, blinks_fast: u8, blinks_slow: u8, breathing_freq: f32, blend_time: f32) -> Pattern {
    Pattern { color, blinks_fast, blinks_slow, breathing_freq, blend_time }
}

pub const COL_BLACK: Color = Color::new(0, 0, 0);
pub const COL_RED: Color = Color::new(255, 0, 0);
pub const COL_GREEN: Color = Color::new(0, 255, 0);
pub const COL_BLUE: Color = Color::new(0, 0, 255);
pub const COL_PURPLE: Color = Color::new(255, 0, 255);

// (statics: the heater tells the patterns apart by their address)
pub static PATTERN_OFF: Pattern = pattern(COL_BLACK, 0, 0, 0.0, 0.0);
pub static PATTERN_ERROR_TEMP_MIN: Pattern = pattern(COL_RED, 1, 1, 0.0, 0.0);
pub static PATTERN_ERROR_TEMP_MAX: Pattern = pattern(COL_RED, 1, 2, 0.0, 0.0);
pub static PATTERN_ERROR_TEMP_AMBIENT_MIN: Pattern = pattern(COL_RED, 1, 3, 0.0, 0.0);
pub static PATTERN_ERROR_TEMP_AMBIENT_MAX: Pattern = pattern(COL_RED, 1, 4, 0.0, 0.0);
pub static PATTERN_ERROR_TEMP_DIFF: Pattern = pattern(COL_RED, 1, 5, 0.0, 0.0);
pub static PATTERN_ERROR_HEAT_MAX: Pattern = pattern(COL_RED, 2, 1, 0.0, 0.0);
pub static PATTERN_ERROR_HEAT_INPUT: Pattern = pattern(COL_RED, 2, 2, 0.0, 0.0);
pub static PATTERN_ERROR_INTERNAL_TEMP: Pattern = pattern(COL_RED, 5, 3, 0.0, 0.0);
pub static PATTERN_HEATING: Pattern = pattern(COL_RED, 0, 0, 0.5, 0.0);
pub static PATTERN_COLD: Pattern = pattern(COL_GREEN, 0, 0, 0.0, 0.5);
pub static PATTERN_TARGET: Pattern = pattern(COL_RED, 0, 0, 0.0, 0.5);
pub static PATTERN_COOLING_DOWN: Pattern = pattern(COL_RED, 0, 0, 0.25, 0.5);
pub static PATTERN_COOLED_DOWN: Pattern = pattern(COL_BLUE, 0, 0, 0.25, 0.5);
pub static PATTERN_DFU: Pattern = pattern(COL_BLUE, 1, 0, 0.0, 0.0);
pub static PATTERN_TRANSIENT: Pattern = pattern(COL_PURPLE, 1, 0, 0.0, 0.0);

#[derive(Clone, Copy)]
pub struct Blending {
    pub color: Color,
    pub time: f32,
}

impl Blending {
    pub fn blend(&self, t: f32, new_color: Color) -> Color {
        Color::blend(self.color, new_color, t / self.time)
    }

    pub fn is_finished(&self, t: f32) -> bool {
        t >= self.time
    }
}

pub struct PatternExecutor {
    pub pattern: Pattern,
    pub blend: Option<Blending>,
    pub timestamp: u32,
}

impl PatternExecutor {
    pub fn new(pattern: Pattern, blend: Option<Blending>) -> PatternExecutor {
        PatternExecutor { pattern, blend, timestamp: 0 }
    }

    pub fn sample(&self) -> Color {
        let t = self.timestamp;
        let mut color = self.pattern.color;

        if self.pattern.breathing_freq != 0.0 {
            let tf = t as f32 * TICK_RATE;
            // (C: in double, std::numbers::pi)
            let scale = (sin(2.0 * core::f64::consts::PI * tf as f64 * self.pattern.breathing_freq as f64) + 1.0) * 0.5;
            color = color.scale(scale as f32);
        }

        color
    }

    pub fn tick(&mut self) -> Color {
        let color = self.sample();
        self.timestamp += 1;
        color
    }
}

pub struct StatusLedDriver {
    bus: &'static I2cBus,
    good: bool,
    current_pattern: PatternExecutor,
    override_color: Option<Color>,
}

impl StatusLedDriver {
    pub fn new(bus: &'static I2cBus) -> R<StatusLedDriver> {
        let mut d =
            StatusLedDriver { bus, good: false, current_pattern: PatternExecutor::new(PATTERN_OFF, None), override_color: None };

        d.good = true;
        d.good &= d.write_u8(0x0E, 0xCC)?; // Chip reset
        d.good &= d.write_u8(0x00, 0x01)?; // Chip enable
        d.good &= d.write_u8(0x01, 0x00)?; // Max currrent = 25.5mA
        d.good &= d.write_u8(0x02, 0x07)?; // Enable all LEDs
        d.good &= d.write_u8(0x0F, 0x55)?; // Update command

        // Verify write was good
        let mut readback = 0;
        d.good &= d.read_u8_into(0x02, &mut readback)?;
        d.good &= readback == 0x07;
        Ok(d)
    }

    pub fn set_currents(&mut self, red: u8, green: u8, blue: u8) -> R {
        if !self.good {
            return Ok(());
        }
        self.good &= self.write_u8(0x14, red)?; // Red current
        self.good &= self.write_u8(0x15, green)?; // Green current
        self.good &= self.write_u8(0x16, blue)?; // Blue current
        Ok(())
    }

    pub fn force_color(&mut self, color: Option<Color>) -> R {
        self.override_color = color;
        if let Some(c) = self.override_color {
            self.good &= self.write_color(c)?;
        } else {
            let last_color = self.current_pattern.sample();
            self.good &= self.write_color(last_color)?;
        }
        Ok(())
    }

    fn write_u8(&self, reg: u8, val: u8) -> R<bool> {
        self.bus.transfer(0x2D, &[reg, val], &mut [])
    }

    fn read_u8_into(&self, reg: u8, out: &mut u8) -> R<bool> {
        self.bus.transfer(0x2D, &[reg], core::slice::from_mut(out))
    }

    fn write_color(&self, color: Color) -> R<bool> {
        Ok(self.write_u8(0x18, color.r)? && self.write_u8(0x19, color.g)? && self.write_u8(0x1A, color.b)?)
    }

    pub fn set_color(&mut self, color: Color) {
        self.set_pattern(pattern(color, 0, 0, 0.0, 0.0));
    }

    pub fn set_pattern(&mut self, pattern: Pattern) {
        if !self.good {
            return;
        }
        let mut blend = None;
        if self.current_pattern.pattern.blend_time != 0.0 {
            let last_color = self.current_pattern.sample();
            blend = Some(Blending { color: last_color, time: self.current_pattern.pattern.blend_time });
        }
        self.current_pattern = PatternExecutor::new(pattern, blend);
    }

    pub fn tick(&mut self) -> R {
        let color = self.current_pattern.tick();
        if self.override_color.is_none() {
            self.good &= self.write_color(color)?;
        }
        Ok(())
    }
}
