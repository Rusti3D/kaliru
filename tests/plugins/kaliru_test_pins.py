# kaliru plugin layer test plugin (tests/cases/plugins/pins.test): pins from
# PrinterPins.setup_pin as AFC uses them (AFC_canvas's enable pin, its lanes'
# LED pwm pins, AFC_buffer's adc), driven from a command
import logging


class TestPins:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")
        self.enable = ppins.setup_pin("digital_out", config.get("enable_pin"))
        self.enable.setup_start_value(1.0, 1.0)
        self.led = ppins.setup_pin("pwm", config.get("led_pin"))
        self.led.setup_cycle_time(0.01, False)
        self.led.setup_start_value(0.0, 0.0)
        self.led.setup_max_duration(0.0)
        self.led.last_set_time = 0.0
        self.adc = ppins.setup_pin("adc", config.get("adc_pin"))
        self.adc.setup_adc_sample(0.001, 8)
        self.adc.setup_adc_callback(0.3, self._adc_callback)
        self.printer.lookup_object("gcode").register_command("TEST_PINS", self.cmd_TEST_PINS)
        logging.info("test pins: %s %s %s", type(self.enable).__name__,
                     type(self.led).__name__, type(self.adc).__name__)

    def _adc_callback(self, read_time, read_value):
        logging.info("test pins adc: %.3f", read_value)

    def cmd_TEST_PINS(self, gcmd):
        mcu = self.led.get_mcu()
        print_time = mcu.estimated_print_time(self.printer.get_reactor().monotonic()) + 0.1
        print_time = max(print_time, self.led.last_set_time + 0.2)
        self.led.last_set_time = print_time
        self.led.set_pwm(print_time, gcmd.get_float("VALUE", 0.5))
        self.enable.set_digital(print_time, gcmd.get_int("ON", 0))
        gcmd.respond_info("pins on %s, adc %s" % (mcu.get_name(), self.adc.get_last_value()))


def load_config(config):
    return TestPins(config)
