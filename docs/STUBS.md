# Stubs

What exists in kaliru only as a stub (an entry that fails when used). Checked 2026-10-29 at 0e4b4fd.

## Native (Rust) side

Nearly complete. All Kalico extras are `ported: true` (`src/extras/mod.rs`) and every kinematics in
`src/kinematics/mod.rs` has a loader. Stubs left:

| Where | What fails |
|---|---|
| `src/pins.rs:332-371` `setup_pin_as` etc. | a pin object kaliru does not know as that type: "not supported here by Kaliru yet" |
| `src/kinematics/winch.rs:84-99` | homing, boundary checks, speed limits, homed checks (same `XXX` gaps as Kalico's winch.py) |

`restart_method: rpi_usb` runs hub-ctrl (`src/chelper/hub_ctrl.rs`, a port of lib/hub-ctrl/hub-ctrl.c over Linux
usbdevfs) as `sudo kaliru --hub-ctrl -h 0 -P 2 -p N`, as Kalico runs its compiled hub-ctrl; hubs are numbered in
libusb's order (udev scan, sorted by sysfs path). Not tried on real hardware yet.

## Python plugin layer

Every Kalico name has a Python stub in `src/plugins/mirror/`, but only names with a handler in
`src/plugins/impls/` work. The rest raise `NotImplementedError: kaliru: X is not mirrored yet`.

**Coverage: 6,753 of 7,489 IDs bound, 736 stubs (152 of them public names).**
Done: extras.heaters, stepper (all but `_get_position_cmd`, `_stepqueue`), chelper `get_ffi` (an emulated cffi:
prelude.py FFI / FFILib / CData, impls/chelper.rs), extras.motion_queuing (all but `setup_mcu_movequeue`,
`steppersyncmgr`, `steppersyncmgr_gen_steps`), toolhead, kinematics.extruder, extras.homing, extras.force_move,
extras.bus, mcu (all but the MCU*Helper classes, RetryAsyncCommand and a few private attributes); event handlers
receive Homing / HomingMove / rails / MCU_stepper / LoadCell objects.
Also: extras.probe (all but ProbeRetryState, GcodeNozzleScrubber(Config) and a few RetrySession internals),
extras.bed_mesh (all but `_init_mesh_config`), the tmc surface plugins use (driver objects, FieldHelper, register
access, TMCCommandHelper, current helpers, setup helpers; not yet: tmc_uart / SPI chain internals, per-chip current
maths, TMCErrorCheck / TMCStallguardDump / TMCVirtualPinHelper).
Also: configfile.PrinterConfig, pins (PinResolver objects are the chips' own), gcode.GCodeDispatch builtin commands
and `mux_commands`, gcode_move move transforms (`set_move_transform` with a plugin's transform, `saved_states`),
gcode_macro Template (`__call__`, `reload`, `env`), webhooks (`get_callback`, `WebRequest(...)`), the kinematics
classes (impls/kinematics.rs, through kaliru's Kinematics trait), manual_stepper, filament sensors (a plugin's
`RunoutHelper(config, its_sensor)`), the endstop wrappers' delegated methods (probe, bltouch, dockable_probe,
load_cell_probe, probe_eddy_current, smart_effector, tools_calibrate), msgproto.MessageParser, buttons
(`setup_buttons` / `setup_button`), exclude_object state, temperature_fan fields (not `control`),
`SerialReader.register_response`, statistics `stats_cb`, virtual_sdcard `current_file`, save_variables
`allVariables`, query_endstops `endstops`, fan `off_below` / `tachometer`, display groups / glyphs, dockable_probe
positions and states, bltouch `multi`, homing_override `prev_G28`, the limited kinematics' limits, servo
widths, error_mcu clarify callbacks, hall_filament_width_sensor fields, `ReactorTimer.callback`, GCodeIO `fd` /
`input_log`, the MPC / tuning / PID-autotune controls, output_pin `mcu_pin` / `shutdown_value`, LED state and pins,
z_thermal_adjust limits, msgproto MessageFormat `encode` / `encode_by_name` / `parse` / `format_params`, adc_scaled,
SecondarySync (kaliru's secondary ClockSync).
Also: the temperature sensors (adc_temperature / thermistor custom sensors as Kalico's classes, i2c / spi / 1-wire
chips, temperature_host / _mcu / _combined, the MPC wrappers), the digipots / dacs, accelerometers and bulk_sensor
(FixedFreqReader, ClockSyncRegression, BatchBulkHelper callbacks), buttons and encoders, belay, endstop_phase,
the z_tilt family and screws_tilt / bed_tilt / delta_calibrate `probe_finalize`, tmc internals (TMCErrorCheck,
SPI chain reads, uart bitbang), display chips (framebuffers, io commands, `set_glyphs`), the menu (contexts, scripts,
inputs, dialogs, keys), load_cell (chips, ApiClientHelper, sample collector, force clients), trad_rack (its selector
toolhead is Kalico's TradRackToolHead: native.rs ALIASES serves the ToolHead names), palette2's omega state and
`p2cmd_*`, indx's toolboard heater, delta / rotary delta calibration objects, hybrid corexy, idex, the four MCU
helper objects (`MCU._conn_helper` / `_config_helper` / `_stats_helper`, one per mcu: kaliru's Mcu keeps their state),
msgproto param types, reactor file handler callbacks, the api server sockets' clients, tools_calibrate, z_calibration.

**Verbatim** (tools/mirror_verbatim.txt, 295 names): pure Kalico helpers and methods built only on bound names run as
Kalico's own code in the mirror (shaper_defs, msgproto PT_* / Enumeration / lookup_params, menu helpers, angle chip
helpers, gcode_arcs `planArc`, delta / rotary calibration maths, resonance generators, shaper_calibrate's numpy
methods, indx calibration fits and compat config, load_cell getters, ...). `ReactorCallback` and indx's compat
ConfigWrapper / ConfigContents / _ResponseHandle are python-only classes.

**API probe** (`tests/plugins/kaliru_test_api.py`, `tests/cases/plugins/api.test`): reads every attribute and calls
every argument-free `get_*` / `is_*` method of every printer object (names from the mirror, through the MRO) on 45
stock and example Kalico configs, one level into the objects found, and logs the values; `plugin_cmp.sh` diffs Kalico against
kaliru. What still differs is stubs, module internals and the deviations below.

**Generic handlers** (impls/traits.rs, used where a class has no binding of its own): `self.printer`,
`self.reactor`, `self.gcode`; `cmd_X(gcmd)` runs the handler registered for g-code command `X`; the methods kaliru
implements through a trait, by name, through the object's trait view: temperature sensors (`setup_minmax`,
`setup_callback`, `get_report_time_delta`), pins (`get_mcu`, `set_pwm`, `set_digital`, `setup_*`), adc pins,
probes (`multi_probe_*`, `probe_prepare/finish`, `probing_move`), lcd chips, bulk adc chips. `mirror_stubs.py`
counts a trait name as bound where the class's module has a Rust type implementing the trait.
**Class mapping** (impls/classes.rs, gen_bindings.py ALIASES): one Rust type for several Kalico classes gets the
class of its kind (AHT1x/2x/3x, MAX6675/31855/31856/31865, ST7920/EmulatedST7920, UC1701/SSD1306/SH1106,
HX711/HX717, ADS131M02/04, the MPC wrappers, the menu item classes, the MCU helpers, z_tilt_ng's helpers);
`objects::register_redirect` makes the "menu" object python's MenuManager.

Left (152 public names) is what kaliru has no counterpart for, or keeps in a shape python cannot share:
- no equivalent: cffi / C helpers (`chelper.logging_callback`, `run_hub_ctrl`, motion_queuing's steppersync,
  `RotaryDeltaCalibration.ffi_lib` aside), threads and locks (`BulkDataQueue.lock`, `ZThermalAdjuster.lock`,
  `SerialReader.lock` / `background_thread`), raw sockets and pyserial objects (`ServerSocket.sock`,
  `ClientConnection.sock`, palette2 `serial` and its `queue.Queue`s), python's module loader (`PrinterModule`,
  `Printer.printer_modules`, `printer.import_test`, `parsedump.main`, `queuelogger.setup_bg_logging`)
- held per call, not stored: a `gcmd` (axis_twist Calibrater, z_calibration CalibrationState, load_cell
  ForceReporter), python generators (`trad_rack.tr_next_generator`), completions of non-python values
  (`EddyDistanceCollector.stop_notify`)
- kaliru's own shape (mutable state owned in place, no shared object): load_cell tap_analysis (TapAnalysis,
  ForceGraph, TapAnalysisHelper, TapQualityClassifier), load_cell_probe HomingMove / ParamHelper, sos_filter
  `change_filter`, indx ThermalModel and its log files, menu container `insert_item` / `add_parents`, display / menu
  `load_config`, z_calibration EndstopWrapper / CalibrationState, trad_rack TradRackHoming / RunIfNoActivity /
  `resume_stack`, output_pin `set_template` / `active_templates`, mcu RetryAsyncCommand / serialhdl
  SerialRetryCommand, SerialReader internals, QueryStatusHelper subscriptions, MessageParser `process_identify` /
  `fill_enumerations`
- unreachable: uc1701's SPI4wire / I2C (python gets `DisplayBase.send` as a function).

**Order of work** (by what the plugin corpus in `Originals_Base/addons` uses): led / neopixel / fan / output_pin -> gcode_move / gcode / webhooks / configfile / printer / reactor -> the rest.
`tools/mirror_stubs.py` prints per-module counts; `--list` prints every unbound name.

**Generated bindings.** `tools/gen_bindings.py` writes `src/plugins/impls/generated.rs`: every unbound Kalico method,
constructor, module function or attribute whose Rust counterpart has the same name (leading `_` dropped,
`cmd_FOO` -> `cmd_foo`), the same arity and types the layer converts (convert.rs, plus the handle converters of
impls/: trapq, endstop, mcu_tmc, command queue, rail, params). It makes the Rust items it uses `pub`. Hand-written
impls take precedence (their names are skipped). Rerun after changing Rust signatures: `python tools/gen_bindings.py
--fresh` (`--report` lists what it skipped and why). The match is by name, arity and type only; a sample was
checked against Kalico, not every binding.

Partly bound core modules (bound/total):

| Module | Bound |
|---|---|
| klippy.printer | 38/56 |
| klippy.configfile | 55/63 |
| klippy.reactor | 84/98 |
| klippy.gcode | 85/86 |
| klippy.extras.gcode_macro | 79/80 |
| klippy.extras.filament_switch_sensor | 44/50 |
| klippy.webhooks | 74/95 |
| klippy.pins | 21/21 |
| klippy.extras.gcode_move | 42/45 |
| klippy.extras.stepper_enable | 28/30 |
| klippy.clocksync | 43/44 |
| klippy.mcu | 260/314 |
| klippy.serialhdl | 37/56 |
| klippy.msgproto | 69/75 |

Not applicable (kaliru's own loader does these): `printer.PrinterModule.*`, `Printer.__init__` /
`printer_modules`, `printer.import_test`, `printer.arg_dictionary`, `pins.add_printer_objects`.

### Bound, but with paths that fail on purpose

| Where | What fails |
|---|---|
| `impls/gcode.rs:153`, `impls/extras_misc.rs:97` | assigning into / deleting from handler dicts |
| `impls/accel.rs:14` | `set_reg` with a minclock |
| `impls/pins.rs:68` | `setup_pin` of a pin type other than endstop / adc on that chip |
| `impls/toolhead.rs` | `ManualStepper.drip_move` with a completion kaliru did not make |
| `impls/webhooks.rs:85` | `WebRequest.get` with types other than the bound ones |
| `objects.rs:81` | a kaliru object whose Kalico class is not registered |

### Known deviations in bound code

| Where | What differs from Kalico |
|---|---|
| `impls/heaters.rs` PyControl | a plugin control raising in `get_profile` / `get_type` / `get_status` is reported as an unhandled error (kaliru's `Control` returns those without a `KResult`) |
| `impls/heaters.rs` | `heater.config` is a new python object per access (`is` does not hold) |
| `impls/heaters.rs` | a profile a plugin builds as a plain dict is copied into kaliru's profile; `get_profile()` returns a `ProfileDict`, not that dict |
| `impls/toolhead.rs` PyExtraAxis | a plugin extra axis raising in `find_past_position` / `get_name` / `get_trapq` / `get_axis_gcode_id` is reported as unhandled |
| `impls/toolhead.rs` | a `Move` python sees is a copy: changes reach kaliru only where kaliru hands the move to python (extra-axis calls) |
| `impls/toolhead.rs` | `toolhead.lookahead.get_last()` / `.queue` / `.flush()` return copies of the moves |
| `impls/chelper.rs` | `ffi_main.gc` ignores the destructor (kaliru frees its objects with the last reference); `ffi.NULL` is `None` |
| `impls/stepper.rs` | `MCU_stepper(...)` built by a plugin re-parses its pin_params dicts (kaliru's lookup_pin hands out copies) |
| `impls/homing.rs` PyHomingToolhead | a plugin toolhead raising in `get_position` / `get_kinematics().get_steppers()` / `calc_position` is reported as unhandled |
| `impls/homing.rs` | `hmove.stepper_positions` are copies |
| `impls/probe.rs` PyMcuProbe | a plugin probe raising in `get_position_endstop` is reported as unhandled |
| `impls/tmc.rs` | `FieldHelper(..., field_formatters=...)` from a plugin ignores the formatters; `field_formatters` reads as `{}` |
| `impls/tmc.rs` | `TMCCommandHelper(..., temp_from_adc=f)` from a plugin raises NotImplementedError; a plugin's own mcu_tmc / current helper object is not accepted |
| `impls/configfile.rs` | `PrinterConfig.deprecate(..., msg=None)` stores `''`; the warning lists, `unused_*` and `deprecated` read as copies |
| `impls/configfile.rs` | `jinja_env` / `PrinterGCodeMacro.env` / `Template.env` are python jinja2 environments built as Kalico builds them; kaliru renders configs and macros with its own engine |
| `impls/extras_misc.rs` | `TemplateWrapperJinja.template` has no python object (kaliru's compiled template) |
| `impls/gcode.rs` | `mux_commands` is a snapshot |
| `impls/pins.rs` | `PrinterPins.chips` / `active_pins` / `pin_resolvers` / `allow_multi_use_pins` are snapshots (the resolvers in them are live) |
| `impls/extras_state.rs` PyMoveTransform | a plugin transform raising in `get_position` is reported as unhandled |
| `impls/webhooks.rs` | `WebRequest(...)` raises ValueError for any request kaliru's parser rejects after python's `json.loads` accepted it |
| `impls/filament_switch_sensor.rs` PyDefinedSensor | a plugin sensor raising in `sensor_get_status` / `reset_needed` is reported as unhandled |
| `impls/msgproto.rs` | `MessageParser.config` / `enumerations` / `messages_by_*` / `msgid_by_format` are rebuilt per access (copies) |
| `impls/extras_state.rs` | `ExcludeObject.extrusion_offsets` is a copy |
| `impls/traits.rs` | `cmd_X(gcmd)` without its own binding goes through the g-code handler of `X`: a mux command picks its instance from the command's parameters, not the object it was called on |
| `impls/traits.rs` | `self.gcode` / `self.printer` / `self.reactor` are read-only where kaliru has no field for them |
| `impls/heaters.rs` | a temperature fan control finds its fan by searching the printer's temperature fans |
| `impls/*` | values Kalico reads with `config.getlist(s)` are tuples there and lists in kaliru (stepper names, probe points, `z_positions`) |
| `impls/*` | attributes Kalico only sets later (exclude_object, tuning_tower, z_tilt_ng, bed_mesh's MoveSplitter, firmware_retraction) read as their default value instead of raising `AttributeError` |
| `impls/*` | a few ints are floats (bed_mesh `fade_start` / `fade_end`, dockable_probe `last_z`) |
| `impls/*` | kaliru's native callables (`<fn kaliru_function>`) stand for Kalico's bound methods: no `__name__` / `__self__` of the original |
| `impls/heaters.rs` | `sensor_factories` holds every sensor type from the start (Kalico adds the i2c / spi ones as their modules load) |
| `impls/extras_misc.rs` | `virtual_sdcard.current_file` is a python file of its own at kaliru's read position: reading it does not move kaliru's |
| `impls/extras_misc.rs` | `PrinterLCD.show_data_group` is read-only |
| `impls/*` | kaliru reuses types where Kalico has its own: `z_calibration.z_endstop` is the MCU_endstop |
| `src/extras/output_pin.rs` | one `klippy:connect` handler for all g-code request queues (Kalico registers one per queue) |
| `impls/extras_misc.rs` | a python clarify callback (`error_mcu.add_clarify`) that raises is logged and adds nothing |
| `impls/extras_misc.rs` | `timer.callback(eventtime)` raises `RuntimeError` while the reactor is running that timer (kaliru's callbacks are not reentrant) |
| `src/extras/quad_gantry_level.rs` and others | an int config default (`max_adjust`, `frequency`, `max_travel`, `temp_coeff`) is int in `configfile.settings` as in Kalico, but the object's attribute is a float |
| `impls/display.rs` and others | Kalico's bytearrays / deques / callback lists read as snapshots: lcd framebuffers, `MovingAverageFilter.queue`, `ClientConnection.request_log`, the buttons' / bulk sensors' / load cell clients' callback lists, `BulkDataQueue.raw_samples` |
| `impls/sensors.rs`, `impls/extras_more.rs` | `Temperature_HOST.file_handle` / `PrinterSysStats.mem_file` are python files over kaliru's descriptor (a new wrapper per access) |
| `impls/extras_more.rs` | `util.get_git_version()` reports the Kalico commit kaliru tracks (util.rs) |
| `impls/display.rs`, `impls/menu.rs` | `DisplayTemplate.render` / menu `get_context` / `run_script` contexts are values only (python callables in a context are not supported, as for `TemplateWrapperJinja.render`) |
| `mirror_verbatim.txt` | shaper_calibrate's `CalibrationData` methods work on objects python builds; one kaliru made has list attributes, not numpy arrays |
| `impls/extras_more.rs` | tsl1401cl `lastFilamentWidthReading` is `0.0` before the first reading (Kalico's is the int `0`) |
