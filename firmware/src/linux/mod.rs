// src/linux: the micro-controller code running in a Linux process. The
// board interface (board/misc.h, board/irq.h, board/gpio.h) as re-exports.
pub mod analog;
pub mod console;
pub mod gpio;
pub mod hard_pwm;
pub mod i2c;
pub mod main;
pub mod pca9685;
pub mod sensor_ds18b20;
pub mod spidev;
pub mod timer;
pub mod watchdog;

pub use crate::generic::alloc::{dynmem_end, dynmem_start};
pub use crate::generic::crc16_ccitt::crc16_ccitt;
pub use analog::{gpio_adc_cancel_sample, gpio_adc_read, gpio_adc_sample, gpio_adc_setup, GpioAdc};
pub use console::{console_receive_buffer, console_sendf};
pub use gpio::{
    gpio_in_read, gpio_in_reset, gpio_in_setup, gpio_out_reset, gpio_out_setup, gpio_out_toggle, gpio_out_toggle_noirq, gpio_out_write,
    GpioIn, GpioOut, GpioOutVal, GpioPullUp,
};
pub use hard_pwm::{gpio_pwm_setup, gpio_pwm_write, GpioPwm};
pub use i2c::{i2c_read, i2c_setup, i2c_write, I2cConfig};
pub use spidev::{spi_prepare, spi_setup, spi_transfer, SpiConfig};
pub use timer::{
    irq_disable, irq_enable, irq_poll, irq_restore, irq_save, irq_wait, timer_from_us, timer_is_before, timer_kick,
    timer_read_time, IrqStatus,
};
