// Build script: the compile time request processing of Kalico's Makefile
// and scripts/buildcommands.py. Reads the Kconfig output (KALIRU_MCU_CONFIG,
// a directory with `config` and `defconfig`; default configs/linux), scans
// the sources the config selects, in the Makefile's order, and writes
// $OUT_DIR/{config.rs,ctr_generated.rs,compile_time_request.txt,klipper.dict}.
mod commands;
mod eval;
mod json;
mod kconfig;
mod scan;

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

/// src/Makefile, src/linux/Makefile: (config option or "", source file)
fn sources(config: &kconfig::Config) -> Vec<&'static str> {
    let main: &[(&str, &str)] = &[
        ("", "sched.rs"),
        ("", "command.rs"),
        ("", "basecmd.rs"),
        ("", "debugcmds.rs"),
        ("CONFIG_HAVE_GPIO", "initial_pins.rs"),
        ("CONFIG_HAVE_GPIO", "gpiocmds.rs"),
        ("CONFIG_HAVE_GPIO", "stepper.rs"),
        ("CONFIG_HAVE_GPIO", "endstop.rs"),
        ("CONFIG_HAVE_GPIO", "trsync.rs"),
        ("CONFIG_WANT_ADC", "adccmds.rs"),
        ("CONFIG_WANT_SPI", "spicmds.rs"),
        ("CONFIG_WANT_I2C", "i2ccmds.rs"),
        ("CONFIG_WANT_HARD_PWM", "pwmcmds.rs"),
        ("CONFIG_HAVE_GPIO_SDIO", "sdiocmds.rs"),
        ("CONFIG_WANT_BUTTONS", "buttons.rs"),
        ("CONFIG_WANT_TMCUART", "tmcuart.rs"),
        ("CONFIG_WANT_NEOPIXEL", "neopixel.rs"),
        ("CONFIG_WANT_PULSE_COUNTER", "pulse_counter.rs"),
        ("CONFIG_WANT_ST7920", "lcd_st7920.rs"),
        ("CONFIG_WANT_HD44780", "lcd_hd44780.rs"),
        ("CONFIG_WANT_SOFTWARE_SPI", "spi_software.rs"),
        ("CONFIG_WANT_SOFTWARE_I2C", "i2c_software.rs"),
        ("CONFIG_WANT_THERMOCOUPLE", "thermocouple.rs"),
        ("CONFIG_WANT_ADXL345", "sensor_adxl345.rs"),
        ("CONFIG_WANT_LIS2DW", "sensor_lis2dw.rs"),
        ("CONFIG_WANT_LDC1612", "sensor_ldc1612.rs"),
        ("CONFIG_WANT_MPU9250", "sensor_mpu9250.rs"),
        ("CONFIG_WANT_ICM20948", "sensor_icm20948.rs"),
        ("CONFIG_WANT_HX71X", "sensor_hx71x.rs"),
        ("CONFIG_WANT_ADS1220", "sensor_ads1220.rs"),
        ("CONFIG_WANT_ADS131M0X", "sensor_ads131m0x.rs"),
        ("CONFIG_WANT_CS1237", "sensor_cs1237.rs"),
        ("CONFIG_WANT_SENSOR_ANGLE", "sensor_angle.rs"),
        ("CONFIG_NEED_SENSOR_BULK", "sensor_bulk.rs"),
        ("CONFIG_NEED_SOS_FILTER", "sos_filter.rs"),
        ("CONFIG_NEED_LOAD_CELL_PROBE", "load_cell_probe.rs"),
    ];
    let linux: &[&str] = &[
        "linux/main.rs",
        "linux/timer.rs",
        "linux/console.rs",
        "linux/watchdog.rs",
        "linux/pca9685.rs",
        "linux/spidev.rs",
        "linux/analog.rs",
        "linux/hard_pwm.rs",
        "linux/i2c.rs",
        "linux/gpio.rs",
        "generic/crc16_ccitt.rs",
        "generic/alloc.rs",
        "linux/sensor_ds18b20.rs",
    ];
    let on = |c: &str| config.enabled(c);
    let mut out: Vec<&'static str> = main.iter().filter(|(c, _)| c.is_empty() || on(c)).map(|(_, f)| *f).collect();
    if on("CONFIG_MACH_LINUX") {
        out.extend(linux);
    } else if on("CONFIG_MACH_STM32") {
        // src/stm32/Makefile
        if !(on("CONFIG_MACH_STM32F1") || on("CONFIG_MACH_STM32F4")) {
            panic!("only the stm32f1 / stm32f4 boards are ported");
        }
        if on("CONFIG_CANSERIAL") || on("CONFIG_USBCANBUS") || on("CONFIG_HAVE_STM32_USBFS") {
            panic!("stm32 canbus / usbfs are not ported yet");
        }
        // (internal.h: no requests, its GPIO() is used in theirs)
        out.push("stm32/internal.rs");
        out.extend(["stm32/watchdog.rs", "stm32/clockline.rs", "stm32/dfu_reboot.rs"]);
        out.push("generic/crc16_ccitt.rs");
        out.extend(["generic/armcm_boot.rs", "generic/armcm_irq.rs", "generic/armcm_reset.rs"]);
        if on("CONFIG_MACH_STM32F1") {
            out.extend(["stm32/stm32f1.rs", "stm32/system_stm32f1xx.rs"]);
        } else {
            out.extend(["stm32/stm32f4.rs", "stm32/system_stm32f4xx.rs"]);
        }
        out.push("generic/armcm_timer.rs");
        if on("CONFIG_MACH_STM32F1") {
            out.push("stm32/gpio.rs");
        } else {
            out.extend(["stm32/gpio.rs", "stm32/gpioperiph.rs"]);
        }
        let opt: &[(&str, &[&'static str])] = &[
            ("CONFIG_WANT_ADC", &["stm32/adc.rs"]),
            ("CONFIG_WANT_GPIO_SPI", &["stm32/spi.rs"]),
            ("CONFIG_WANT_GPIO_I2C", &["stm32/i2c.rs"]),
            ("CONFIG_SERIAL", &["stm32/serial.rs", "generic/serial_irq.rs"]),
            ("CONFIG_USBSERIAL", &["stm32/usbotg.rs", "stm32/chipid.rs", "generic/usb_cdc.rs"]),
            ("CONFIG_WANT_HARD_PWM", &["stm32/hard_pwm.rs"]),
            ("CONFIG_HAVE_GPIO_SDIO", &["stm32/sdio.rs"]),
        ];
        for (c, files) in opt {
            if on(c) {
                out.extend(*files);
            }
        }
    } else {
        panic!("only the linux and stm32 boards are ported");
    }
    out
}

/// The CMSIS device header of an stm32 build (src/stm32/chip/<mcu>.rs,
/// made by tools/gen_stm32_regs.py): its path and the names it defines
/// (the C `#ifdef NAME`, here `#[cfg(has_NAME)]`)
fn stm32_chip(root: &Path, config: &kconfig::Config) -> Option<(PathBuf, BTreeSet<String>)> {
    if !config.enabled("CONFIG_MACH_STM32") {
        return None;
    }
    let mcu = config.string("CONFIG_MCU").unwrap();
    let path = root.join("src/stm32/chip").join(format!("{}.rs", mcu));
    let text = read(&path);
    let names = text
        .lines()
        .filter_map(|l| l.strip_prefix("pub const ").and_then(|r| r.split(':').next()))
        .map(|n| n.to_string())
        .collect();
    Some((path, names))
}

/// generic/armcm_link.lds.S through the C preprocessor: its `#if CONFIG_X`
/// blocks and CONFIG_X values from the configuration
fn armcm_link(src: &str, config: &kconfig::Config) -> String {
    let mut out = String::new();
    let mut skip = vec![];
    for line in src.lines() {
        let t = line.trim();
        if let Some(c) = t.strip_prefix("#if ") {
            skip.push(!config.enabled(c.trim()));
            continue;
        }
        if t.starts_with("#endif") {
            skip.pop();
            continue;
        }
        if t.starts_with("#include") || t.starts_with("//") || skip.contains(&true) {
            continue;
        }
        let mut l = line.to_string();
        while let Some(i) = l.find("CONFIG_") {
            let name: String = l[i..].chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '_').collect();
            let v = config.int(&name).unwrap_or_else(|| panic!("linker script: {}", name));
            l = format!("{}{}{}", &l[..i], v, &l[i + name.len()..]);
        }
        out += &l;
        out.push('\n');
    }
    out
}

/// the module path of a source file (linux/timer.rs -> crate::linux::timer)
fn module_path(file: &str) -> String {
    format!("crate::{}", file.trim_end_matches(".rs").replace('/', "::"))
}

fn run(cmd: &str, args: &[&str]) -> String {
    std::process::Command::new(cmd)
        .args(args)
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_default()
}

/// buildcommands.py build_version / tool_versions, for the Rust toolchain
fn versions() -> commands::Versions {
    let rustc = std::env::var("RUSTC").unwrap_or_else(|_| "rustc".into());
    let toolver = run(&rustc, &["--version"]);
    let build_versions = format!("rustc: {}", toolver.strip_prefix("rustc ").unwrap_or(&toolver));
    let mut version = run("git", &["describe", "--always", "--tags", "--long", "--dirty"]);
    let mut clean = !toolver.is_empty();
    if version.is_empty() {
        clean = false;
        version = "?".into();
    } else if version.contains("dirty") {
        clean = false;
    }
    if !clean {
        let btime = run("date", &["+%Y%m%d_%H%M%S"]);
        let host = run("hostname", &[]);
        version = format!("{}-{}-{}", version, btime, host);
    }
    commands::Versions { version, build_versions }
}

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let config_dir = std::env::var("KALIRU_MCU_CONFIG").map(PathBuf::from).unwrap_or_else(|_| root.join("configs/linux"));
    println!("cargo:rerun-if-env-changed=KALIRU_MCU_CONFIG");
    println!("cargo:rerun-if-changed={}", config_dir.display());
    println!("cargo:rerun-if-changed=src");
    let config = kconfig::Config::load(&config_dir);

    // cfg flags for the enabled bool options; check-cfg for all of them
    for (name, v) in &config.values {
        if let kconfig::Value::Bool(_) = v {
            println!("cargo::rustc-check-cfg=cfg({})", name);
        }
        if *v == kconfig::Value::Bool(true) {
            println!("cargo:rustc-cfg={}", name);
        }
    }
    // and for the symbols the sources name that this config lacks
    // (another board's)
    for name in config_names(&root.join("src"), "CONFIG_") {
        if !config.values.contains_key(&name) {
            println!("cargo::rustc-check-cfg=cfg({})", name);
        }
    }
    std::fs::write(out.join("config.rs"), config.rust_consts()).unwrap();

    // stm32: the chip header, has_NAME cfgs, the linker script
    let chip = stm32_chip(&root, &config);
    let chip_names = chip.as_ref().map(|c| c.1.clone()).unwrap_or_default();
    for name in config_names(&root.join("src"), "has_") {
        println!("cargo::rustc-check-cfg=cfg({})", name);
        if chip_names.contains(&name["has_".len()..]) {
            println!("cargo:rustc-cfg={}", name);
        }
    }
    let has = |n: &str| config.enabled(n) || n.strip_prefix("has_").is_some_and(|c| chip_names.contains(c));
    if let Some((path, _)) = &chip {
        std::fs::write(out.join("chip.rs"), format!("include!({:?});\n", path.display().to_string())).unwrap();
        let lds = armcm_link(&read(&root.join("build/armcm_link.lds.S")), &config);
        std::fs::write(out.join("armcm_link.ld"), lds).unwrap();
        println!("cargo:rustc-link-arg=-T{}", out.join("armcm_link.ld").display());
        // the C link has no entry symbol either: ld then uses the start of
        // .text, the vector table
        println!("cargo:rustc-link-arg=--entry=_text_vectortable_start");
    }

    // scan: per file, the requests come out in reverse source order (as
    // gcc emits a file's .compile_time_request section)
    let src = root.join("src");
    let files: Vec<&str> = sources(&config);
    let mut consts = scan::Consts::default();
    if let Some((path, _)) = &chip {
        // the register / irq number constants
        scan::scan(&read(path), &mut consts, &has);
    }
    let mut scanned = vec![];
    for f in &files {
        let path = src.join(f);
        let text = read(&path);
        let s = scan::scan(&text, &mut consts, &has);
        scanned.push((*f, s));
    }
    let mut requests = vec![];
    for (f, s) in scanned {
        let module = module_path(f);
        for decl in s.decls.into_iter().rev() {
            requests.push(commands::Request { decl, module: module.clone() });
        }
    }
    // a check of the generator: the requests of a C build's
    // compile_time_request.txt instead (the firmware then won't link)
    println!("cargo:rerun-if-env-changed=KALIRU_MCU_CTR_INPUT");
    if let Ok(path) = std::env::var("KALIRU_MCU_CTR_INPUT") {
        requests = read(Path::new(&path)).lines().filter(|l| !l.trim().is_empty()).map(|l| parse_request(l.trim_start())).collect();
    }
    let eval = eval::Eval { config: &config, consts: &consts };
    let o = commands::generate(&requests, &eval, &versions(), &config.defconfig);
    std::fs::write(out.join("compile_time_request.txt"), &o.txt).unwrap();
    std::fs::write(out.join("klipper.dict"), &o.dict).unwrap();
    std::fs::write(out.join("ctr_generated.rs"), &o.code).unwrap();
}

/// a compile_time_request.txt line as a request
fn parse_request(line: &str) -> commands::Request {
    use scan::Decl;
    let (cmd, rest) = line.split_once(' ').unwrap_or((line, ""));
    let words: Vec<&str> = rest.split_whitespace().collect();
    // CTR_INT values back to expressions the evaluator reads
    let int = |w: &str| w.replace("+0x", "0x").replace("-0x", "-0x");
    let decl = match cmd {
        "DECL_COMMAND_FLAGS" => {
            let mut p = rest.splitn(3, ' ');
            Decl::Command { func: p.next().unwrap().into(), flags: p.next().unwrap().into(), msg: p.next().unwrap().into() }
        }
        "_DECL_ENCODER" => Decl::Encoder(rest.into()),
        "_DECL_OUTPUT" => Decl::Output(rest.into()),
        "_DECL_STATIC_STR" => Decl::StaticStr(rest.into()),
        "_DECL_CALLLIST" => {
            let list = match words[0] {
                "ctr_run_initfuncs" => "ctr_run_initfuncs",
                "ctr_run_taskfuncs" => "ctr_run_taskfuncs",
                _ => "ctr_run_shutdownfuncs",
            };
            Decl::CallList { list, func: words[1].into() }
        }
        "DECL_CONSTANT" => Decl::Constant { name: words[0].into(), expr: int(words[1]) },
        "DECL_CONSTANT_STR" => {
            let (name, value) = rest.split_once(' ').unwrap();
            Decl::ConstantStr { name: name.into(), value: format!("{:?}", value) }
        }
        "DECL_ENUMERATION" => Decl::Enumeration { enumeration: words[0].into(), name: words[1].into(), expr: int(words[2]) },
        "DECL_ENUMERATION_RANGE" => Decl::EnumerationRange {
            enumeration: words[0].into(),
            name: words[1].into(),
            expr: int(words[2]),
            count: int(words[3]),
        },
        "DECL_INITIAL_PINS" => Decl::InitialPins("CONFIG_INITIAL_PINS".into()),
        other => panic!("Unknown build time command '{}'", other),
    };
    commands::Request { decl, module: "crate".into() }
}

/// the names starting with `prefix` (CONFIG_*, has_*) in the sources
/// under `dir` (the generated chip headers aside)
fn config_names(dir: &Path, prefix: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for e in std::fs::read_dir(dir).unwrap().flatten() {
        let p = e.path();
        if p.is_dir() {
            if !p.ends_with("chip") {
                out.extend(config_names(&p, prefix));
            }
        } else if p.extension().is_some_and(|x| x == "rs") {
            let text = std::fs::read_to_string(&p).unwrap();
            let mut rest = text.as_str();
            while let Some(i) = rest.find(prefix) {
                let name: String = rest[i..].chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '_').collect();
                // a whole word only (not the has_ of flash_x)
                let word = i == 0 || !rest[..i].ends_with(|c: char| c.is_ascii_alphanumeric() || c == '_');
                rest = &rest[i + name.len()..];
                if word {
                    out.insert(name);
                }
            }
        }
    }
    out
}

fn read(path: &Path) -> String {
    match std::fs::read_to_string(path) {
        Ok(t) => t,
        // a source not ported yet
        Err(_) => {
            println!("cargo:warning=firmware source {} missing", path.display());
            String::new()
        }
    }
}
