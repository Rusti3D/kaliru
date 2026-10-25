# Rustify

kaliru is Kalico ported to Rust: Kalico's logic, and Kalico's (Python's) shape. Rustify keeps the logic and changes
the shape: the dynamic object model Python gives Kalico, rebuilt by hand in Rust, becomes Rust's own types, traits and
ownership.

## Rules

1. **It cannot break.** Nothing visible from outside changes: g-code replies, the API's JSON, the logs, config error
   messages, the MCU traffic, the python plugin API. Every commit passes all the parity harnesses byte-identical
   (`tools/rustify_gate.sh`).
2. **Resource usage only gets better.** Every commit is measured against the baseline (CPU time, peak memory, heap,
   binary size; `tools/resource_gate.sh`); a regression beyond the noise does not land. Milestones are measured on
   the CC1 as well.
3. **Crates where they give a real, measured gain**, checked against binary size and memory.
4. **Small steps**: one change per commit, both gates green, no big-bang rewrite.
5. **Python semantics only at the edges**: `PyVal`, python float formatting and python exception classes stay where
   they are output (config text, the API, Jinja, plugins); inside, Rust types.

## Where kaliru stands (2026-10-25, 110k lines, one crate, 283 files)

| python's shape, rebuilt in Rust | count | cost |
|---|---|---|
| `RefCell` fields, `.borrow()` / `.borrow_mut()` | 707 / 4,115 | a runtime flag per field, runtime panics instead of compile errors |
| `Rc<dyn PrinterObject>`, `as_any`, downcasts | 288 / 231 / 198 | type errors at runtime; 231 copies of the same function |
| string-named events, `&[&dyn Any]` arguments | 149 handlers, 62 senders, 44 events | unchecked payloads, a downcast in each handler |
| `PyVal` dynamic values (status, config tracking) | 1,022 | a value tree built per status poll: the top CPU item measured on the CC1 |
| config getters (`getfloat` / `getint` / ...) | 2,836 | hand-written parsing in every module |
| `.clone()` / `.unwrap()` | 2,573 / 1,228 | allocations, panic paths |
| `#[test]` | 27 | correctness lives in the external parity harnesses only |

## Phases

**0. Guardrails**
- `tools/rustify_gate.sh`: every parity harness (check_all, case_cmp, suite_cmp, plugin_cmp, live_cmp, api_cmp).
- `tools/resource_gate.sh` with a stored baseline: CPU time of the suite, peak RSS and heap on a real printer config,
  startup time, x86 and armv7 binary sizes.
- Unit and property tests for the pure modules (math, kinematics, stepcompress, msgproto, python formatting).

**1. Structure**
- A cargo workspace, leaf crates first (python compatibility, jinja, the MCU protocol, motion math): layering the
  compiler enforces, faster builds. Only where the resource gate allows it: Moonruster measured +0.75 % .text for two
  split crates (generics monomorphized per crate under lto + codegen-units = 1); otherwise modules with `pub(crate)`
  boundaries give the layering without the cost.
- Lints (clippy baseline), a SAFETY note on each `unsafe`.

**2. Typed core APIs**
- a. Object model: one blanket impl instead of 231 `as_any`; typed lookups instead of downcasts; capability traits
  (temperature sensor, status, ...) instead of special cases on `PrinterObject`.
- b. Events: a type per event with a checked payload; handler order kept; names only for the plugin bridge.
- c. Config: `#[derive(FromConfig)]` typed config structs making the same getter calls in the same order (access
  tracking and error text stay identical).
- d. Status: typed status with per-field change detection, written straight to JSON (no `PyVal` tree per poll).
- e. Errors: `KError` at the python boundary; typed errors with context inside.

**3. State and ownership**
- Immutable config apart from mutable state; one `RefCell<State>` per object instead of a cell per field; fewer
  clones (borrowing, interned names, `Cow`).

**4. Hot paths** (profiled on the CC1)
- Allocation-free stepcompress / itersolve / trapq, zero-copy msgproto, a timer heap in the reactor; each with a
  benchmark and a CC1 A/B.

**5. Concurrency (spike, decision gate)**
- Greenlets are stackful coroutines with a stack each; a single-threaded async executor could use less memory.
  Adopted only on a measured win.

Out of scope: behaviour changes; the MCU firmware (its own crate, later). First run: kaliru only; Rusticc's patch
series is updated afterwards.

## Progress

| step | what | parity | resources vs main (min of runs) |
|---|---|---|---|
| 0 | gates: `tools/rustify_gate.sh`, `tools/resource_gate.sh` (b788f9c) | baseline | baseline |
| 2a-1 | trait upcasting instead of 230 `as_any` (975b669) | identical | steady cpu -3.9 %, anon PSS -0.2 %, x86 -29.6 kB, armv7 -16 kB |
| 2b | typed events, `src/events.rs` (788cdda) | identical | steady cpu -4.8 %, anon RSS -4.2 %, x86 -11.7 kB |
| 4 + 2d-1 | FxHasher maps, allocation-free float / JSON formatting, RustPython dropped, mcu status shared, CPython-table python compatibility (batch 1) | identical | with batches 2-3 below |
| 2d-2 + 1 | static status keys (`PyKey`), `pydict!` sized dicts, motion_report status shared as python's, reply keys reused, no send-buffer copy; close-on-exec fds and raising chown / chmod as python; SAFETY notes, `[lints.clippy]`, clippy clean in the gate (batch 2) | identical | |
| 2d + fixes | gcode_macro status shared as python's, klipper_path from KALIRU_KLIPPY_DIR, mirrored list fields assigned whole (AFC restore_pos), gettempdir as CPython, `tools/profile_steady.sh` (batch 3) | identical | batches 1-3: steady cpu -3.8 %, steady peak RSS -25.5 %, anon RSS -25 %, anon PSS -22.4 %, suite cpu -2.1 %, suite peak RSS -30 %, x86 -66 % (RustPython), armv7 -0.9 % |

Evaluated and kept as they are (no measurable gain, or a parity cost):
- 1, workspace crates: +0.75 % .text for two crates (generics instantiated per crate); modules give the layering free.
- 2a-2, capability traits: the optional `PrinterObject` methods returning `Option` are python's `hasattr` duck
  typing at no cost; separate traits would still need an `as_*()` query on `dyn PrinterObject`.
- 2c, config derive: Kalico's reads depend on earlier values (`above(min_temp)`), interleave with logic and their
  order is observable (access tracking, error text); `config.getfloat(name, default, Lim)` is already typed.
- 2e, errors: `KError` is already one variant per python exception class; `Py`'s class stays a string because
  plugins and Jinja raise arbitrary classes. Errors are cold paths.
- 3, one `RefCell<State>` per object: `Cell<f64>` has no overhead and a `RefCell` one word; merging them saves
  nothing measurable and loses the per-field borrow scope.

## How to measure (lessons)

- Run both gates under the shared lock (`/tmp/rustify_gates.lock`, taken by the scripts): Moonruster's rustify
  gates run in the same WSL, and a measurement must not share the cpu with them. Heavy builds too.
- Wall-clock cpu of the steady workload spreads ~10 % run to run on this machine: a change worth less than that needs
  a deterministic count. Callgrind instruction counts over the same window are near-deterministic:
  `tools/live_run.py` with `LIVE_KALIRU` = a wrapper running `valgrind --tool=callgrind --instr-atstart=no` and
  `callgrind_control -i on` after startup (forward SIGINT to valgrind so it writes its data).
- Profile with `RUSTFLAGS="-Z merge-functions=disabled"` (nightly) and debug info: otherwise identical functions are
  merged and callgrind charges one name with another's cost (a "config map lookup" that was the status dicts').
  `tools/profile_steady.sh` does all of this for the steady workload.
- The steady-state profile (main, every object subscribed at 250 ms): the status poll is 82 % of the instructions;
  in it malloc/free ~29 %, SipHash ~19 %, PyVal JSON serialization 17.5 %, Mcu get_status / stats ~10 %.

## Design: typed status (2d)

Today: each of the ~100 `get_status()` implementations builds a fresh `PyVal::Dict` (String keys, boxed values) on
every 250 ms poll of every subscribed object; the poll then looks up each requested key, compares it with the last
poll's copy (`py_eq`) and copies the changed ones into the response, which is serialized to JSON.

Target, in steps that each keep the JSON byte-identical:
1. Values that rarely change are built once and handed out shared (`PyVal::Shared`: an Rc clone; an unchanged field
   compares by identity, as python's `!=` does for the same object). Done for configfile, gcode; mcu in batch 1.
2. Static keys: a status is a list of `(&'static str, value)` fields instead of a map with allocated String keys
   (python plugins and the Jinja context still see a dict: converted at that edge).
3. A `Status` trait with a typed struct per object and a derive that writes the fields in Kalico's order straight to
   the JSON writer and compares field by field with the previous struct (`PartialEq` with python's float semantics),
   so the poll allocates only the response.
The order of keys, python's float repr and the change rules are what the parity harnesses (api_cmp, live) check.
