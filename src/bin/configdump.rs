// Dumps how a config file is read (configfile.PrinterConfig.read_main_config)
// for parity testing; twin of tools/configdump_ref.py. Development tool.
//
// usage: configdump <config file>
use kaliru::configfile::{ConfigWrapper, Lim, ListParser, PrinterConfig, REQUIRED};
use kaliru::configparser::RawConfigParser;
use kaliru::pyfmt::py_str_repr;
use kaliru::pyval::PyVal;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(|a| a == "--save").unwrap_or(false) {
        return save(&args[1..]);
    }
    let path = args.first().expect("usage: configdump <config file>").clone();
    let mut pconfig = PrinterConfig::new(&path);
    let cfg = match pconfig.read_main_config() {
        Ok(c) => c,
        Err(e) => {
            println!("ERROR {}", e);
            return;
        }
    };
    dump_parser("main", &cfg.fileconfig.borrow(), &cfg);
    let autosave = pconfig.autosave.clone().unwrap();
    dump_parser("autosave", &autosave.fileconfig.borrow(), &autosave);
    println!("== log_config\n{}", pconfig.log_config(&cfg));
    println!("== tracking after interpolation");
    for ((s, o), v) in cfg.access_tracking.borrow().iter() {
        println!("T {} {} = {}", py_str_repr(s), py_str_repr(o), v.json());
    }
    println!("== getters");
    let sections = cfg.fileconfig.borrow().sections();
    for s in &sections {
        let sec = cfg.getsection(s);
        let options = cfg.fileconfig.borrow().options(s).unwrap();
        for o in &options {
            println!("[{}] {}", s, o);
            show("getint", sec.getint(o, REQUIRED, Lim::new()).map(PyVal::from));
            show("getfloat", sec.getfloat(o, REQUIRED, Lim::new()).map(float_json));
            show("getfloat_min0", sec.getfloat(o, REQUIRED, Lim::new().min(0.)).map(float_json));
            show("getfloat_above1", sec.getfloat(o, REQUIRED, Lim::new().above(1)).map(float_json));
            show("getboolean", sec.getboolean(o, REQUIRED).map(PyVal::from));
            show("getlist", sec.getlists(o, REQUIRED, &[","], None, ListParser::Str, true).map(|v| v.unwrap()));
            show("getfloatlist", sec.getlists(o, REQUIRED, &[","], None, ListParser::SafeFloat, true).map(|v| v.unwrap()));
            show("getintlist2", sec.getlists(o, REQUIRED, &[","], Some(2), ListParser::Int, true).map(|v| v.unwrap()));
            show("getlists", sec.getlists(o, REQUIRED, &[",", "\n"], None, ListParser::Str, true).map(|v| v.unwrap()));
            show("getscript", sec.getscript(o, REQUIRED).map(|(k, t)| PyVal::Tuple(vec![k.into(), t.into()])));
        }
        show("missing", sec.get("no_such_option", REQUIRED).map(PyVal::from));
        // python getfloat returns an int default unchanged
        show("default_int", sec.getfloat_value("no_such_float", &300i64.into(), &Lim::new()).map(|v| v.unwrap()));
    }
    match pconfig.check_unused_options(&cfg, false, &[]) {
        Ok(()) => println!("== status\n{}", pconfig.get_status(0.).json()),
        Err(e) => println!("== status ERROR {}", e),
    }
}

// --save <config> <autosave_includes 0|1> [section:option=value | -section]...
// applies the changes, runs SAVE_CONFIG RESTART=0, dumps the config folder
fn save(args: &[String]) {
    let path = &args[0];
    let autosave_includes = args[1] == "1";
    let mut pconfig = PrinterConfig::new(path);
    if let Err(e) = pconfig.read_main_config() {
        println!("ERROR {}", e);
        return;
    }
    for change in &args[2..] {
        if let Some(section) = change.strip_prefix('-') {
            pconfig.remove_section(section);
        } else {
            let (so, value) = change.split_once('=').unwrap();
            let (section, option) = so.split_once(':').unwrap();
            if let Err(e) = pconfig.set(section, option, value) {
                println!("set ERR {}", e);
            }
        }
    }
    println!("pending {} {}", pconfig.save_config_pending, pconfig.status_save_pending.json());
    match pconfig.save_config(autosave_includes) {
        Ok(true) => {
            // cmd_SAVE_CONFIG with RESTART=0
            pconfig.save_config_pending = false;
            println!("info Config update without restart successful");
        }
        Ok(false) => {}
        Err(e) => println!("SAVE ERR {}", e),
    }
    println!("pending {}", pconfig.save_config_pending);
    dump_tree(&kaliru::pyglob::dirname(path));
}

fn dump_tree(dir: &str) {
    let mut files = kaliru::pyglob::glob(&format!("{}/**", dir), true);
    files.sort();
    for f in files {
        if std::path::Path::new(&f).is_file() {
            // backup names carry time.strftime("-%Y%m%d_%H%M%S")
            let name = mask_date(&f[dir.len()..]);
            // open(f).read(): universal newlines
            let text = std::fs::read_to_string(&f).unwrap_or_default().replace("\r\n", "\n").replace('\r', "\n");
            println!("== file {}\n{}", name, text);
        }
    }
}

fn mask_date(s: &str) -> String {
    let b = s.as_bytes();
    for i in 0..b.len() {
        if i + 16 <= b.len()
            && b[i] == b'-'
            && b[i + 1..i + 9].iter().all(u8::is_ascii_digit)
            && b[i + 9] == b'_'
            && b[i + 10..i + 16].iter().all(u8::is_ascii_digit)
        {
            return format!("{}-<DATE>{}", &s[..i], &s[i + 16..]);
        }
    }
    s.to_string()
}

fn float_json(f: f64) -> PyVal {
    PyVal::Float(f)
}

fn show(what: &str, r: Result<PyVal, kaliru::configparser::Error>) {
    match r {
        Ok(v) => println!("  {} {}", what, v.json()),
        Err(e) => println!("  {} ERR {}", what, e),
    }
}

fn dump_parser(name: &str, fc: &RawConfigParser, cfg: &ConfigWrapper) {
    println!("== {} sections", name);
    for (k, v) in fc.defaults() {
        println!("D {} = {}", k, py_str_repr(v));
    }
    for s in fc.sections() {
        println!("[{}]", s);
        for (k, v) in fc.raw_section(&s).unwrap() {
            let got = match fc.get(&s, k) {
                Ok(g) => py_str_repr(&g),
                Err(e) => format!("ERR {}", e),
            };
            println!("  {} = {} -> {}", k, py_str_repr(v), got);
        }
    }
    let _ = cfg;
}
