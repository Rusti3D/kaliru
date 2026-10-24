// src/hc32f460: the Huada HC32F460 boards. The board interface (board/misc.h,
// board/irq.h, board/gpio.h, ...) as re-exports.
#[allow(non_upper_case_globals, non_camel_case_types, non_snake_case, dead_code, clippy::all)]
pub mod chip {
    // hc32f460.h and the driver headers, made by tools/gen_stm32_regs.py --hc32
    include!(concat!(env!("OUT_DIR"), "/chip.rs"));
}
pub mod internal;
// the lib/hc32f460 driver functions Kalico's code calls
pub mod lib;

#[cfg(CONFIG_WANT_ADC)]
pub mod adc;
pub mod gpio;
pub mod gpio_h;
#[cfg(CONFIG_WANT_HARD_PWM)]
pub mod hard_pwm;
pub mod interrupts;
pub mod main;
#[cfg(CONFIG_SERIAL)]
pub mod serial;

pub use crate::generic::armcm_boot::{dynmem_end, dynmem_start};
pub use crate::generic::armcm_irq::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, IrqStatus};
pub use crate::generic::armcm_timer::{timer_from_us, timer_is_before, timer_kick, timer_read_time};
pub use crate::generic::crc16_ccitt::crc16_ccitt;
#[cfg(CONFIG_SERIAL)]
pub use crate::generic::serial_irq::console_sendf;
#[cfg(CONFIG_SERIAL)]
pub use serial::serial_enable_tx_irq;
#[cfg(CONFIG_WANT_ADC)]
pub use adc::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup};
pub use gpio::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write,
};
pub use gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, I2cConfig, SpiConfig};
#[cfg(CONFIG_WANT_HARD_PWM)]
pub use hard_pwm::{gpio_pwm_setup, gpio_pwm_write};
pub use main::armcm_main;
