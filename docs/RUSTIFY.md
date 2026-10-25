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
  compiler enforces, faster builds.
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
