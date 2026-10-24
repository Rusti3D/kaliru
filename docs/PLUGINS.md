# Python plugins in kaliru — design (2026-10-24, draft)

Goal (user): kaliru can run Python klippy plugins *in general*, as Kalico can, even though the user means to move
off them over time. Their printer needs AFC (AFC-Klipper-Add-On) and the AFC_ACE plugin. Memory rule:
- no cost when no plugin is configured;
- no more than Python Kalico costs when one is.

## What Kalico offers a plugin (the contract to match)

Source: Kalico 5b59427 `klippy/printer.py` (`_load_modules`, `PrinterModule`, `load_object`), `klippy/compat.py`,
`docs/Code_Overview.md` "Adding a host module", `docs/Kalico_Additions.md` "Plugins".

1. **Discovery.** At startup Kalico lists every module in `klippy/extras/` and then `klippy/plugins/` (pkgutil). A
   name found in both is a config error, unless `[danger_options] allow_plugin_override: True`; then the plugin
   replaces the built-in extra.
2. **Loading.** A section `[name]` or `[name suffix]` imports the module on first use and calls `load_config(config)`
   or `load_config_prefix(config)`. The returned object goes into `printer.objects`. The import is lazy, and an
   import failure is logged and re-raised only when the section is used.
3. **Components.** `register_components(collector)` is called for any module whose source contains that name
   (`printer.lookup_components()`, used for load-cell and eddy sensors).
4. **Legacy imports.** `compat.install()` adds a meta-path finder, so `import mcu`, `from configfile import error`
   and `from extras.x import y` resolve to the `klippy.*` modules. The same module objects are reused, so `is` and
   `isinstance` checks hold.
5. **Everything else is unrestricted.** There is no API boundary. A plugin gets the real Python objects and can:
   - call any method (the documented set: config getters, `get_printer`, `lookup_object`, `register_event_handler`,
     `register_command`/`register_mux_command`, `pins.setup_pin`, the reactor timers/pause/fd/completions,
     `get_status`, webhooks endpoints);
   - read and write private attributes, and monkeypatch methods;
   - construct core classes itself;
   - call the C helper (`chelper.get_ffi()`) directly.
   
   Code_Overview only *asks* plugins to avoid internals.
6. **Firmware side.** `src/extras/<module>/` with its own Kconfig and Makefile is auto-included in the MCU build.
   This is out of scope for the host bridge and is tracked separately.

What the user's plugins actually do (static scan of AFC@CC1 and AFC_ACE@CC1, copies in
`Originals_Base/addons/`). AFC is the stress case for point 5:
- 62 `register_mux_command`, 34 `register_command`, 34 `register_event_handler`, 17 `register_timer`,
  12 `register_buttons`, 8 `setup_pin`, `register_endstop`, `register_lookahead_callback`, `alloc_command_queue`,
  `register_response`.
- `chelper.get_ffi()`: `trapq_alloc`/`trapq_append`/`trapq_finalize_moves`, `cartesian_stepper_alloc`,
  `itersolve_get_commanded_pos` and `set_thread_name`. AFC drives its own steppers like `manual_stepper`.
- It builds a `filament_switch_sensor` from Python (`configfile.ConfigWrapper(printer, RawConfigParser, {}, name)` then
  `printer.load_object`) and then rewires that core object:
  - `runout_helper.note_filament_present` is replaced, after `inspect.signature` on the original;
  - `runout_helper._runout_event_handler` is replaced;
  - `runout_gcode = 1` and `insert_gcode = None` are set;
  - `printer.objects["_" + name] = printer.objects.pop(name)`.
- Other internals it uses: `toolhead._advance_move_time`, `webhooks._remote_methods`, `gcode.ready_gcode_handlers`,
  `config.fileconfig.sections/options/set`, `motion_queuing.allocate_trapq/lookup_trapq_append/wipe_trapq/
  note_mcu_movequeue_activity/drip_update_time`.
- Imports of Kalico Python code it runs itself: `extras.led`, `force_move.calc_move_time`,
  `queuelogger.QueueListener/QueueHandler`, `webhooks.GCodeHelper`.
- Python features: threads (AFC logger, ACE serial), pyserial, `urllib`, `subprocess`, `inspect`, `typing`, `enum`.

## Interpreter: CPython, loaded only when a plugin is configured

| | CPython (system libpython) | RustPython (already in kaliru for python macros) |
|---|---|---|
| Compatibility | the real thing: C extensions (cffi, pyserial's termios, numpy), `inspect.signature`, threads, `match` | no `match` (0.4), older stdlib, no C extensions → no cffi, no numpy; plugins like shaketune impossible |
| Memory | measured below | heavier object model (unmeasured with AFC: it cannot import it) |
| On the CC1 | `/usr/lib/libpython3.15.so.1.0` (4.7 MB, shared) + cffi 2.0.0, pyserial 3.5, greenlet 3.4.0 installed | built in |
| Zero cost when unused | yes: bridge is a separate `.so` dlopen'ed on the first plugin section | interpreter is created lazily already |

**Decision (recommended): CPython via a bridge library** (`libkaliru_py.so`, Rust + the CPython C API, linked against
the host's libpython). kaliru `dlopen`s it only when a config section names a module that is not built in. Hosts
without Python keep working, minus plugins.

## Memory budget (measured, x86_64, CPython 3.13, module import only)

| step | own (anon) memory |
|---|---|
| bare interpreter | 4.1 MB |
| + Kalico's Python core (configfile, gcode, reactor, pins, mcu, stepper, toolhead, webhooks, queuelogger, chelper, printer) | 21.1 MB (+17.0) |
| + extras led/force_move/heaters | 26.1 MB (+5.0) |
| + AFC, 32 modules | 34.9 MB (+8.8) |
| + AFC_ACE | 36.4 MB (+1.5) |

For comparison, Python Kalico in the live scenario is 34 MB anon and kaliru is 2.7 MB. Consequences:
- The bridge must **not** load Kalico's Python core: that alone costs what Python klippy costs. Plugins talk to
  kaliru's Rust objects through thin Python-visible objects.
- Target with AFC + AFC_ACE: kaliru (about 3) + interpreter (4) + plugin code (10.3) + bridge, roughly 18-20 MB.
  That is under even plugin-free Python Kalico (34 MB); the fair comparison is Python Kalico *with* AFC, about
  34 + 10. To re-measure on armv7, where pointers are half the size.

## Architecture

1. **Module namespace.** The bridge provides a `klippy` package that matches Kalico's layout: `klippy.printer`,
   `klippy.mcu`, `klippy.extras.<name>` and so on, with `compat.install()` semantics. Each core module is a thin
   *native* module that exposes the Rust class, function or exception under its Kalico name:
   - `configfile.error`, `gcode.CommandError`, `printer.command_error`;
   - `ConfigWrapper`, `PrinterStepper`, `calc_move_time`, `LEDHelper`, and so on.
   
   Pure-Python stdlib-only helpers (queuelogger's handlers) can be Kalico's own source, imported only when a plugin
   imports them.
2. **Objects.** Each Rust printer object is exposed as a Python object of a class named like Kalico's, with the
   same methods and attributes. Identity is stable: one Python wrapper per Rust object, so `is` holds.
   Python-created objects (plugin objects) live in `printer.objects` next to the Rust ones. On the Rust side they
   implement `PrinterObject` (`get_status`, `stats`, events) by calling into Python.
3. **Calls both ways.** Rust calls Python for:
   - gcode handlers, event handlers, timers, fd callbacks, webhooks endpoints;
   - `get_status`, button callbacks, `load_config`.
   
   Python calls Rust through the wrapper methods. Errors map both ways: the Python exception classes *are* kaliru's
   classes (`KError::Py(class)` ↔ the same Python class objects).
4. **Hook points for rewiring.** Rust objects stay Rust, so monkeypatching a method only takes effect where kaliru
   dispatches through an overridable slot. The slots are a fixed list, grown as plugins need them. The first ones,
   from AFC:
   - `RunoutHelper.note_filament_present` and `_runout_event_handler`;
   - `RunoutHelper.runout_gcode`/`insert_gcode` assignment;
   - `printer.objects` as a mutable mapping.
   
   A write to an attribute that is not mapped raises a clear error naming the attribute, rather than being silently
   ignored.
5. **Greenlets.** kaliru's reactor greenlets are corosensei coroutines. Python frames can be live on several of them
   (a plugin gcode handler that waits, while a timer runs another plugin callback). Plan:
   - one `PyThreadState` per coroutine that runs Python, swapped with `PyThreadState_Swap` on every switch;
   - on the same switch, `PyUnstable_ThreadState_SetStackProtection(tstate, stack, size)` (CPython ≥ 3.15; docs:
     "When the stack is changed, for example using context switching techniques like ... boost::context, you must
     call" it).
   
   **Unverified, so it is the first prototype.** Before 3.15, CPython cannot be told about the stack. The fallback
   is to do what greenlet does or to require 3.15. The CC1 has 3.15.
6. **GIL and threads.** The reactor thread holds the GIL only while it runs Python. Plugin threads (AFC logger, ACE
   serial reader) run in between and return to the reactor via `register_async_callback`, which kaliru already has.
7. **chelper.** `chelper.get_ffi()` returns a real cffi `(ffi, lib)`, with Kalico's cdefs, over C-ABI exports of
   kaliru's chelper (`trapq_alloc`, `itersolve_*`, ...). The pointers are handles that the Rust wrappers
   (`set_trapq`, `set_stepper_kinematics`) map back to Rust objects.
8. **Override of built-in extras** (`allow_plugin_override`). Kaliru looks up core extras by concrete Rust type in
   ~400 places (`lookup::<GCodeDispatch>` 122, `ToolHead` 117, `PrinterHeaters` 12, `GCodeMove` 10, ...). A Python
   replacement of an extra that other Rust modules look up by type cannot satisfy those lookups. Support override
   for extras nothing in Rust looks up by type, and give a clear config error for the others. gcode, toolhead,
   pins, mcu and configfile are core files in Kalico too, so plugins cannot override them there either.

## Honest limits

Matching every private attribute of every Kalico class is not possible with Rust objects. The coverage target is:
1. the documented module API, in full;
2. every attribute and method that a plugin corpus uses, found by tracing.

Everything else fails loudly, never silently.

## How the surface is found and verified

- **Corpus:** AFC + AFC_ACE first. Then widely used plugins: Happy Hare, led_effect, klipper_tmc_autotune,
  Klippain shaketune, gcode_shell_command, z_calibration, beacon, cartographer. Each pinned to a commit in
  `Originals_Base/addons/`.
- **Trace:** run each plugin under Python Kalico with an instrumented `__getattribute__`/`__setattr__` proxy around
  every core object handed to plugin code. That lists the exact surface used, including dynamic access that static
  grep misses. The list becomes the bridge's checklist.
- **Parity:** the existing harnesses (live_cmp with linux MCU processes, case/suite) run each plugin config on
  Python Kalico and on kaliru+bridge. The serialqueue trace and the masked log must match.

## Steps

1. Prototype the risky core and verify it before anything else (WSL, CPython 3.15 built locally):
   - dlopen'ed bridge;
   - a plugin gcode command that pauses while a timer runs Python in another coroutine (tstate swap + stack
     protection);
   - a thread calling `register_async_callback`;
   - memory delta.
2. Loader parity: discovery of extras and plugins, `allow_plugin_override`, lazy import, `register_components`,
   compat finder. Cases against Kalico.
3. The documented API surface (config, printer, reactor, gcode, pins/MCU objects, webhooks, status), with a test
   plugin per area run on both sides.
4. Trace AFC + AFC_ACE and fill their surface (hook points, chelper ffi, steppers). Parity runs with the user's AFC
   config (copied read-only from the CC1).
5. Corpus plugins one by one.
6. Later, decide whether python *macros* move from RustPython to the same bridge. Without RustPython the binary
   shrinks from 29.7 to 9.1 MB, but macros would then need libpython.
