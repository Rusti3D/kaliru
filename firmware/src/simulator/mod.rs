// src/simulator: the firmware compiled for and run on the host, for
// simulation purposes. The board interface as re-exports.
pub mod gpio;
pub mod main;
pub mod serial;
pub mod timer;

pub use crate::generic::alloc::{dynmem_end, dynmem_start};
pub use crate::generic::crc16_ccitt::crc16_ccitt;
pub use crate::generic::gpio_h::{GpioAdc, GpioIn, GpioOut, GpioOutVal, GpioPullUp, GpioPwm, I2cConfig, SpiConfig};
pub use crate::generic::serial_irq::console_sendf;
pub use crate::generic::timer_irq::{timer_from_us, timer_is_before};
pub use gpio::{
    gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup, gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset,
    gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write, gpio_pwm_setup, gpio_pwm_write, spi_prepare, spi_setup,
    spi_transfer,
};
pub use serial::{console_receive_buffer, serial_enable_tx_irq};
pub use timer::{irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, timer_kick, timer_read_time, IrqStatus};
pub use crate::generic::pgm;
