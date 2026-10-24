// Kalico Telemetry (port of klippy/extras/telemetry.py)
//
// Submit anonymized information about your printer to the Kalico project
//
// Difference: python posts with urllib from a thread. Kaliru has no HTTP
// client; the same JSON and headers are posted by a `curl` child process
// (a missing curl is logged like a failed request).
use crate::configfile::ConfigWrapper;
use crate::error::{KError, KResult};
use crate::gcode::{GCodeCommand, GCodeDispatch};
use crate::logging;
use crate::printer::{ConfigFileObject, PObj, Printer, PrinterObject};
use crate::pyval::PyVal;
use indexmap::IndexMap;
use std::any::Any;
use std::path::{Path, PathBuf};
use std::rc::Rc;

pub const TELEMETRY_ENDPOINT: &str = "https://telemetry.kalico.gg/collect";

pub struct KalicoTelemetry {
    printer: Rc<Printer>,
    enabled: Option<bool>,
}

impl KalicoTelemetry {
    fn new(config: &ConfigWrapper) -> KResult<Rc<Self>> {
        let printer = config.get_printer();
        let enabled = config.getboolean_opt("enabled")?;
        let t = Rc::new(KalicoTelemetry { printer: printer.clone(), enabled });

        let gcode = printer.lookup::<GCodeDispatch>("gcode")?;
        let c = t.clone();
        gcode.register_command("TELEMETRY_EXAMPLE", Some(Rc::new(move |g| c.cmd_telemetry_example(g))), true, None)?;

        if enabled != Some(true) {
            let c = t.clone();
            gcode.register_command("ENABLE_TELEMETRY", Some(Rc::new(move |g| c.cmd_set_telemetry(g, true))), false, None)?;
        }

        if enabled != Some(false) {
            let c = t.clone();
            gcode.register_command("DISABLE_TELEMETRY", Some(Rc::new(move |g| c.cmd_set_telemetry(g, false))), false, None)?;
        }

        if enabled.is_none() {
            let c = t.clone();
            printer.register_event_handler(
                "klippy:ready",
                Rc::new(move |_| {
                    c.telemetry_prompt()?;
                    Ok(())
                }),
            );
        }

        if enabled == Some(true) {
            let c = t.clone();
            printer.register_event_handler(
                "klippy:ready",
                Rc::new(move |_| {
                    c.handle_ready();
                    Ok(())
                }),
            );
        }
        Ok(t)
    }

    fn telemetry_prompt(&self) -> KResult<()> {
        if self.printer.start_arg("debuginput").map(|v| v.truthy()).unwrap_or(false) {
            return Ok(());
        }

        let gcode = self.printer.lookup::<GCodeDispatch>("gcode")?;

        gcode.respond_info(
            "Welcome to Kalico\n\
             \n\
             Kalico is starting an opt-in analytics campaign, collecting anonymous data about our users\n\
             Submitted data will contain an anonymized copy of your current configuration, enabled \
             Kalico modules and plugins, and some basic information about the host machine.\n\
             \n\
             `ENABLE_TELEMETRY` will opt-in to the campaign\n\
             `DISABLE_TELEMETRY` will permanently opt-out\n\
             \n\
             `TELEMETRY_EXAMPLE` will save a full example of what would be \
             submitted as 'telemetry.json' in your configuration folder\n\
             \n\
             For more information, visit https://kalico.gg/telemetry",
            true,
        );
        Ok(())
    }

    // cmd_ENABLE_TELEMETRY / cmd_DISABLE_TELEMETRY
    fn cmd_set_telemetry(&self, gcmd: &GCodeCommand, enable: bool) -> KResult<()> {
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        configfile.pconfig.borrow_mut().set("telemetry", "enabled", crate::pyfmt::py_bool(enable))?;

        let what = if enable { "enabled" } else { "disabled" };
        gcmd.respond_info(
            &format!(
                "Kalico telemetry {}\n\n`SAVE_CONFIG` will update your printer config\nfile with this setting and restart the printer.",
                what
            ),
            true,
        );
        Ok(())
    }

    fn handle_ready(&self) {
        match self.collect_telemetry() {
            Ok(data) => send(data),
            Err(_) => {
                logging::exception("exception collecting telemetry");
                // python then fails on the unbound 'data' (UnboundLocalError)
            }
        }
    }

    fn config_dir(&self) -> PathBuf {
        let config_file = self.printer.start_arg("config_file").map(|v| v.str()).unwrap_or_default();
        Path::new(&config_file).parent().map(|p| p.to_path_buf()).unwrap_or_default()
    }

    fn cmd_telemetry_example(&self, gcmd: &GCodeCommand) -> KResult<()> {
        let filename = self.config_dir().join("telemetry.json");
        let data = self.collect_telemetry()?;
        let text = json_indent(&data, 2, 0);
        std::fs::write(&filename, text)
            .map_err(|e| KError::os(&e, &filename.to_string_lossy()))?;
        gcmd.respond_info(&format!("Example telemetry saved to {}", filename.display()), true);
        Ok(())
    }

    // Get the freedesktop machine-id if available, otherwise generate and save our own
    fn get_machine_id(&self) -> KResult<String> {
        let machine_id = Path::new("/etc/machine-id");
        let kalico_machine_id = self.config_dir().join(".machine-id");
        let read = |p: &Path| {
            std::fs::read_to_string(p)
                .map(|s| crate::pyfmt::py_strip(&s).to_string())
                .map_err(|e| KError::os(&e, &p.to_string_lossy()))
        };
        if machine_id.exists() {
            return read(machine_id);
        }
        if kalico_machine_id.exists() {
            return read(&kalico_machine_id);
        }
        let generated_id = format!("kalico-{}", uuid4());
        std::fs::write(&kalico_machine_id, &generated_id)
            .map_err(|e| KError::os(&e, &kalico_machine_id.to_string_lossy()))?;
        Ok(generated_id)
    }

    // Generate an anonymized version of the current printer configuration.
    // Only section and option names are submitted
    fn collect_anonymized_config(&self) -> KResult<PyVal> {
        let configfile = self.printer.lookup::<ConfigFileObject>("configfile")?;
        let pconfig = configfile.pconfig.borrow();
        let mut res = IndexMap::new();
        let raw_config = match pconfig.status_raw_config.get() {
            PyVal::Dict(d) => d.clone(),
            _ => IndexMap::new(),
        };
        for (section, options) in &raw_config {
            let keys = match options {
                PyVal::Dict(d) => d.keys().map(|k| PyVal::Str(k.clone())).collect(),
                _ => vec![],
            };
            res.insert(section.clone(), PyVal::List(keys));
        }
        Ok(PyVal::Dict(res))
    }

    // Collect basic platform information
    fn collect_platform(&self) -> PyVal {
        let mut uts: libc::utsname = unsafe { std::mem::zeroed() };
        unsafe { libc::uname(&mut uts) };
        let field = |f: &[libc::c_char]| unsafe { std::ffi::CStr::from_ptr(f.as_ptr()) }.to_string_lossy().into_owned();
        let mut d = IndexMap::new();
        d.insert("machine".to_string(), PyVal::Str(field(&uts.machine)));
        d.insert("os_release".to_string(), collect_os_release());
        d.insert("version".to_string(), PyVal::Str(field(&uts.version)));
        // sys.version: kaliru is not python
        d.insert("python".to_string(), PyVal::Str(format!("Kaliru {}", env!("CARGO_PKG_VERSION"))));
        PyVal::Dict(d)
    }

    // Collect the full telemetry for submission
    fn collect_telemetry(&self) -> KResult<PyVal> {
        let machine_id = self.get_machine_id()?;
        let arg = |k: &str| self.printer.start_arg(k).unwrap_or_else(|| PyVal::from("unknown"));
        let mut d = IndexMap::new();
        d.insert("id".to_string(), PyVal::Str(machine_id));
        d.insert("version".to_string(), arg("software_version"));
        d.insert("branch".to_string(), arg("git_branch"));
        d.insert("cpu_info".to_string(), arg("cpu_info"));
        d.insert("platform".to_string(), self.collect_platform());
        d.insert("config".to_string(), self.collect_anonymized_config()?);
        let objects = self.printer.lookup_objects(None).into_iter().map(|(n, _)| PyVal::Str(n)).collect();
        d.insert("objects".to_string(), PyVal::List(objects));
        Ok(PyVal::Dict(d))
    }
}

// Collect the freedesktop OS-RELEASE information.
fn collect_os_release() -> PyVal {
    let paths = ["/etc/os-release", "/usr/lib/os-release"];
    let Some(path) = paths.iter().find(|p| Path::new(p).exists()) else { return PyVal::None };
    let Ok(text) = std::fs::read_to_string(path) else { return PyVal::None };
    let mut result = IndexMap::new();
    for line in text.split_inclusive('\n') {
        let Some((key, value)) = line.split_once('=') else { continue };
        if let Ok(parts) = crate::pyfmt::py_shlex_split(value, "") {
            if let Some(first) = parts.into_iter().next() {
                result.insert(key.to_string(), PyVal::Str(first));
            }
        }
    }
    PyVal::Dict(result)
}

/// uuid.uuid4()
fn uuid4() -> String {
    let mut b = [0u8; 16];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
        use std::io::Read;
        let _ = f.read_exact(&mut b);
    }
    b[6] = (b[6] & 0x0f) | 0x40;
    b[8] = (b[8] & 0x3f) | 0x80;
    let h: String = b.iter().map(|x| format!("{:02x}", x)).collect();
    format!("{}-{}-{}-{}-{}", &h[..8], &h[8..12], &h[12..16], &h[16..20], &h[20..])
}

/// json.dump(data, fp, indent=n)
fn json_indent(v: &PyVal, indent: usize, level: usize) -> String {
    let pad = |l: usize| " ".repeat(indent * l);
    match v {
        PyVal::List(a) | PyVal::Tuple(a) | PyVal::NamedTuple(_, _, a) if !a.is_empty() => {
            let items: Vec<String> = a.iter().map(|x| format!("{}{}", pad(level + 1), json_indent(x, indent, level + 1))).collect();
            format!("[\n{}\n{}]", items.join(",\n"), pad(level))
        }
        PyVal::Dict(d) if !d.is_empty() => {
            let items: Vec<String> = d
                .iter()
                .map(|(k, x)| format!("{}{}: {}", pad(level + 1), PyVal::Str(k.clone()).json(), json_indent(x, indent, level + 1)))
                .collect();
            format!("{{\n{}\n{}}}", items.join(",\n"), pad(level))
        }
        v => v.json(),
    }
}

fn send(collected_data: PyVal) {
    let data = collected_data.json();
    let version = match &collected_data {
        PyVal::Dict(d) => d.get("version").map(|v| v.str()).unwrap_or_default(),
        _ => String::new(),
    };
    std::thread::spawn(move || {
        use std::io::Write;
        let child = std::process::Command::new("curl")
            .args(["-sSf", "-X", "POST", "-H", "Content-Type: application/json", "-H"])
            .arg(format!("User-Agent: Kalico {}", version))
            .args(["--data-binary", "@-", TELEMETRY_ENDPOINT])
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::null())
            .spawn();
        let ok = match child {
            Ok(mut c) => {
                if let Some(mut stdin) = c.stdin.take() {
                    let _ = stdin.write_all(data.as_bytes());
                }
                c.wait().map(|s| s.success()).unwrap_or(false)
            }
            Err(_) => false,
        };
        if ok {
            logging::info(&format!("Telemetry sent to {}", TELEMETRY_ENDPOINT));
        } else {
            logging::error("Exception in telemetry submission thread");
        }
    });
}

impl PrinterObject for KalicoTelemetry {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn has_status(&self) -> bool {
        true
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyVal> {
        let mut d = IndexMap::new();
        d.insert("enabled".to_string(), self.enabled.map(PyVal::Bool).unwrap_or(PyVal::None));
        Some(PyVal::Dict(d))
    }
}

pub fn load_config(config: &ConfigWrapper) -> KResult<PObj> {
    Ok(KalicoTelemetry::new(config)?)
}
