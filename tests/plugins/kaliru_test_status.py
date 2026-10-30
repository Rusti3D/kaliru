# kaliru status probe (tests/cases/plugins/status.test): TEST_STATUS logs
# get_status(eventtime) of every printer object that has one (what
# Moonraker's objects/query reads), formatted as the api probe formats
# values, so a Kalico and a kaliru run can be diffed.
import logging

from . import kaliru_test_api


class TestStatus:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.lookup_object("gcode").register_command(
            "TEST_STATUS", self.cmd_TEST_STATUS
        )

    def cmd_TEST_STATUS(self, gcmd):
        eventtime = self.printer.get_reactor().monotonic()
        for name, obj in sorted(self.printer.lookup_objects()):
            if obj is self or not hasattr(obj, "get_status"):
                continue
            try:
                status = obj.get_status(eventtime)
            except Exception as e:
                logging.info("status %s ! %s: %s", name, type(e).__name__,
                             str(e)[:200])
                continue
            if not isinstance(status, dict):
                logging.info("status %s = %s", name,
                             kaliru_test_api.show(status))
                continue
            for key in sorted(status):
                logging.info("status %s.%s = %s", name, key,
                             kaliru_test_api.show(status[key]))
        gcmd.respond_info("status probe done")


def load_config(config):
    return TestStatus(config)
