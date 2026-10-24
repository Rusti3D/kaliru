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
        let fam = |f: &str| on(&format!("CONFIG_MACH_STM32{}", f));
        // (internal.h: no requests, its GPIO() is used in theirs)
        out.push("stm32/internal.rs");
        out.extend(["stm32/watchdog.rs", "stm32/clockline.rs", "stm32/dfu_reboot.rs"]);
        out.push("generic/crc16_ccitt.rs");
        out.extend(["generic/armcm_boot.rs", "generic/armcm_irq.rs", "generic/armcm_reset.rs"]);
        let chip: &[&'static str] = if fam("F0") {
            &["stm32/stm32f0.rs", "stm32/system_stm32f0xx.rs"]
        } else if fam("F1") {
            &["stm32/stm32f1.rs", "stm32/system_stm32f1xx.rs"]
        } else if fam("F2") {
            &["stm32/stm32f4.rs", "stm32/system_stm32f2xx.rs"]
        } else if fam("F4") {
            &["stm32/stm32f4.rs", "stm32/system_stm32f4xx.rs"]
        } else if fam("F7") {
            &["stm32/stm32f7.rs", "stm32/system_stm32f7xx.rs"]
        } else if fam("G0") {
            &["stm32/stm32g0.rs"]
        } else if fam("G4") {
            &["stm32/stm32g4.rs", "stm32/system_stm32g4xx.rs"]
        } else if fam("H7") {
            &["stm32/stm32h7.rs", "stm32/system_stm32h7xx.rs"]
        } else if fam("L4") {
            &["stm32/stm32l4.rs", "stm32/system_stm32l4xx.rs"]
        } else {
            panic!("stm32 family not ported (n32g45x, at32f403)")
        };
        out.extend(chip);
        // timer-src-y, gpio-src-y
        if fam("F0") || fam("G0") {
            out.extend(["generic/timer_irq.rs", "stm32/stm32f0_timer.rs"]);
        } else {
            out.push("generic/armcm_timer.rs");
        }
        if fam("F1") {
            out.push("stm32/gpio.rs");
        } else if fam("H7") {
            out.extend(["stm32/stm32h7_gpio.rs", "stm32/gpioperiph.rs"]);
        } else {
            out.extend(["stm32/gpio.rs", "stm32/gpioperiph.rs"]);
        }
        let adc: &[&'static str] = if fam("F0") || fam("G0") {
            &["stm32/stm32f0_adc.rs"]
        } else if on("CONFIG_MACH_N32G45x") {
            // (../lib/n32g45x/n32g45x_adc.c, its header and stm32/n32g45x_adc.c)
            &["stm32/n32g45x_adc_lib.rs", "stm32/n32g45x_adc.rs"]
        } else if fam("G4") || fam("H7") || fam("L4") {
            &["stm32/stm32h7_adc.rs"]
        } else {
            &["stm32/adc.rs"]
        };
        let spi: &'static str = if fam("H7") { "stm32/stm32h7_spi.rs" } else { "stm32/spi.rs" };
        let i2c: &'static str = if fam("F1") || fam("F2") || fam("F4") { "stm32/i2c.rs" } else { "stm32/stm32f0_i2c.rs" };
        let serial: &'static str =
            if fam("F0") || fam("G0") || fam("G4") || fam("H7") { "stm32/stm32f0_serial.rs" } else { "stm32/serial.rs" };
        let usb: &[&'static str] = if on("CONFIG_HAVE_STM32_USBFS") {
            &["stm32/usbfs.rs"]
        } else if on("CONFIG_HAVE_STM32_USBOTG") {
            &["stm32/usbotg.rs"]
        } else {
            &[]
        };
        let can: &[&'static str] = if on("CONFIG_HAVE_STM32_CANBUS") {
            &["generic/canserial.rs", "generic/fasthash.rs", "stm32/can.rs"]
        } else if on("CONFIG_HAVE_STM32_FDCANBUS") {
            &["generic/canserial.rs", "generic/fasthash.rs", "stm32/fdcan.rs"]
        } else {
            &["generic/canserial.rs", "generic/fasthash.rs"]
        };
        // (canbus.h: no requests, its constants are used in theirs)
        if on("CONFIG_CANSERIAL") || on("CONFIG_USBCANBUS") {
            out.push("generic/canbus_h.rs");
        }
        let mut opt: Vec<(&str, Vec<&'static str>)> = vec![
            ("CONFIG_WANT_ADC", adc.to_vec()),
            ("CONFIG_WANT_GPIO_SPI", vec![spi]),
            ("CONFIG_WANT_GPIO_I2C", vec![i2c]),
            ("CONFIG_SERIAL", vec![serial, "generic/serial_irq.rs"]),
        ];
        let mut usbser = usb.to_vec();
        usbser.extend(["stm32/chipid.rs", "generic/usb_cdc.rs"]);
        opt.push(("CONFIG_USBSERIAL", usbser));
        let mut canser = can.to_vec();
        canser.extend(["generic/canbus.rs", "stm32/chipid.rs"]);
        opt.push(("CONFIG_CANSERIAL", canser));
        let mut usbcan = usb.to_vec();
        usbcan.extend(can);
        usbcan.extend(["stm32/chipid.rs", "generic/usb_canbus.rs"]);
        opt.push(("CONFIG_USBCANBUS", usbcan));
        opt.push(("CONFIG_WANT_HARD_PWM", vec!["stm32/hard_pwm.rs"]));
        opt.push(("CONFIG_HAVE_GPIO_SDIO", vec!["stm32/sdio.rs"]));
        for (c, files) in opt {
            if on(c) {
                out.extend(files);
            }
        }
    } else if on("CONFIG_MACH_RPXXXX") {
        // src/rp2040/Makefile
        out.extend(["rp2040/internal.rs", "rp2040/main.rs", "rp2040/watchdog.rs", "rp2040/gpio.rs", "rp2040/adc.rs"]);
        out.extend(["generic/armcm_boot.rs", "generic/armcm_irq.rs", "generic/armcm_reset.rs", "generic/crc16_ccitt.rs"]);
        if on("CONFIG_MACH_RP2040") {
            out.extend(["rp2040/timer.rs", "generic/timer_irq.rs", "rp2040/bootrom.rs"]);
        }
        if on("CONFIG_MACH_RP2350") {
            out.extend(["generic/armcm_timer.rs", "rp2040/rp2350_bootrom.rs"]);
        }
        // (canbus.h: no requests, its constants are used in theirs)
        if on("CONFIG_CANSERIAL") || on("CONFIG_USBCANBUS") {
            out.push("generic/canbus_h.rs");
        }
        let opt: &[(&str, &[&'static str])] = &[
            ("CONFIG_USBSERIAL", &["rp2040/usbserial.rs", "generic/usb_cdc.rs", "rp2040/chipid.rs"]),
            ("CONFIG_SERIAL", &["rp2040/serial.rs", "generic/serial_irq.rs"]),
            ("CONFIG_CANSERIAL", &["rp2040/can.rs", "rp2040/chipid.rs", "rp2040/can2040.rs"]),
            ("CONFIG_CANSERIAL", &["generic/canserial.rs", "generic/canbus.rs"]),
            ("CONFIG_CANSERIAL", &["generic/fasthash.rs"]),
            ("CONFIG_USBCANBUS", &["rp2040/can.rs", "rp2040/chipid.rs", "rp2040/can2040.rs"]),
            ("CONFIG_USBCANBUS", &["generic/canserial.rs", "generic/usb_canbus.rs"]),
            ("CONFIG_USBCANBUS", &["generic/fasthash.rs", "rp2040/usbserial.rs"]),
            ("CONFIG_WANT_HARD_PWM", &["rp2040/hard_pwm.rs"]),
            ("CONFIG_WANT_GPIO_SPI", &["rp2040/spi.rs"]),
            ("CONFIG_WANT_GPIO_I2C", &["rp2040/i2c.rs"]),
        ];
        for (c, files) in opt {
            if on(c) {
                out.extend(*files);
            }
        }
    } else if on("CONFIG_MACH_LPC176X") {
        // src/lpc176x/Makefile
        out.extend(["lpc176x/internal.rs", "lpc176x/main.rs", "lpc176x/gpio.rs"]);
        out.extend(["generic/armcm_boot.rs", "generic/armcm_irq.rs", "generic/armcm_timer.rs"]);
        out.extend(["generic/armcm_reset.rs", "generic/crc16_ccitt.rs"]);
        out.push("lpc176x/system_LPC17xx.rs");
        // (canbus.h: no requests, its constants are used in theirs)
        if on("CONFIG_CANSERIAL") || on("CONFIG_USBCANBUS") {
            out.push("generic/canbus_h.rs");
        }
        let canbus: &[&'static str] = &["lpc176x/can.rs", "lpc176x/chipid.rs", "generic/canserial.rs", "generic/fasthash.rs"];
        let opt: &[(&str, &[&'static str])] = &[
            ("CONFIG_WANT_ADC", &["lpc176x/adc.rs"]),
            ("CONFIG_WANT_GPIO_I2C", &["lpc176x/i2c.rs"]),
            ("CONFIG_WANT_GPIO_SPI", &["lpc176x/spi.rs"]),
            ("CONFIG_USBSERIAL", &["lpc176x/usbserial.rs", "lpc176x/chipid.rs"]),
            ("CONFIG_USBSERIAL", &["generic/usb_cdc.rs"]),
            ("CONFIG_SERIAL", &["lpc176x/serial.rs", "generic/serial_irq.rs"]),
            ("CONFIG_CANSERIAL", canbus),
            ("CONFIG_CANSERIAL", &["generic/canbus.rs"]),
            ("CONFIG_USBCANBUS", canbus),
            ("CONFIG_USBCANBUS", &["lpc176x/usbserial.rs"]),
            ("CONFIG_USBCANBUS", &["generic/usb_canbus.rs"]),
            ("CONFIG_WANT_HARD_PWM", &["lpc176x/hard_pwm.rs"]),
        ];
        for (c, files) in opt {
            if on(c) {
                out.extend(*files);
            }
        }
    } else if on("CONFIG_MACH_ATSAM") {
        // src/atsam/Makefile
        out.extend(["atsam/internal.rs", "atsam/main.rs", "atsam/gpio.rs", "atsam/i2c.rs", "atsam/spi.rs"]);
        out.extend(["generic/armcm_boot.rs", "generic/armcm_irq.rs", "generic/armcm_timer.rs"]);
        out.push("generic/crc16_ccitt.rs");
        // (canbus.h: no requests, its constants are used in theirs)
        if on("CONFIG_CANSERIAL") || on("CONFIG_USBCANBUS") {
            out.push("generic/canbus_h.rs");
        }
        let usb: &'static str = if on("CONFIG_MACH_SAM4") { "atsam/sam4_usb.rs" } else { "atsam/sam3_usb.rs" };
        let canbus: &[&'static str] = &["generic/canserial.rs", "generic/fasthash.rs", "atsam/fdcan.rs", "atsam/chipid.rs"];
        let opt: &[(&str, Vec<&'static str>)] = &[
            ("CONFIG_USBSERIAL", vec![usb, "atsam/chipid.rs", "generic/usb_cdc.rs"]),
            ("CONFIG_SERIAL", vec!["atsam/serial.rs", "generic/serial_irq.rs"]),
            ("CONFIG_USBCANBUS", [canbus, &[usb, "generic/usb_canbus.rs"]].concat()),
            ("CONFIG_CANSERIAL", [canbus, &["generic/canbus.rs"]].concat()),
            ("CONFIG_MACH_SAM3X", vec!["atsam/adc.rs", "atsam/hard_pwm.rs"]),
            ("CONFIG_MACH_SAM4", vec!["atsam/hard_pwm.rs"]),
            ("CONFIG_MACH_SAM4S", vec!["atsam/adc.rs"]),
            ("CONFIG_MACH_SAM4E", vec!["atsam/sam4e_afec.rs", "atsam/sam4_cache.rs"]),
            ("CONFIG_MACH_SAM3X", vec!["atsam/system_sam3xa.rs"]),
            ("CONFIG_MACH_SAM4S", vec!["atsam/sam4s_sysinit.rs"]),
            ("CONFIG_MACH_SAM4E", vec!["atsam/system_sam4e.rs"]),
            ("CONFIG_MACH_SAME70", vec!["atsam/same70_sysinit.rs", "atsam/sam4e_afec.rs"]),
        ];
        for (c, files) in opt {
            if on(c) {
                out.extend(files.iter().copied());
            }
        }
    } else if on("CONFIG_MACH_ATSAMD") {
        // src/atsamd/Makefile
        out.extend(["atsamd/internal.rs", "atsamd/main.rs", "atsamd/gpio.rs", "generic/crc16_ccitt.rs"]);
        out.extend(["generic/armcm_boot.rs", "generic/armcm_irq.rs", "generic/armcm_reset.rs"]);
        // (canbus.h: no requests, its constants are used in theirs)
        if on("CONFIG_CANSERIAL") || on("CONFIG_USBCANBUS") {
            out.push("generic/canbus_h.rs");
        }
        let canbus: &[&'static str] = &["generic/canserial.rs", "generic/fasthash.rs", "atsamd/fdcan.rs", "atsamd/chipid.rs"];
        let opt: &[(&str, Vec<&'static str>)] = &[
            ("CONFIG_USBSERIAL", vec!["atsamd/usbserial.rs", "atsamd/chipid.rs", "generic/usb_cdc.rs"]),
            ("CONFIG_SERIAL", vec!["atsamd/serial.rs", "generic/serial_irq.rs"]),
            ("CONFIG_USBCANBUS", [canbus, &["atsamd/usbserial.rs", "generic/usb_canbus.rs"]].concat()),
            ("CONFIG_CANSERIAL", [canbus, &["generic/canbus.rs"]].concat()),
            ("CONFIG_WANT_GPIO_ADC", vec!["atsamd/adc.rs"]),
            ("CONFIG_WANT_GPIO_I2C", vec!["atsamd/i2c.rs"]),
            ("CONFIG_WANT_GPIO_SPI", vec!["atsamd/spi.rs"]),
            ("CONFIG_HAVE_SERCOM", vec!["atsamd/sercom.rs"]),
            ("CONFIG_HAVE_GPIO_HARD_PWM", vec!["atsamd/hard_pwm.rs"]),
            ("CONFIG_MACH_SAMC21", vec!["atsamd/samd51_watchdog.rs"]),
            ("CONFIG_MACH_SAMC21", vec!["atsamd/samc21_clock.rs", "atsamd/timer.rs", "generic/timer_irq.rs"]),
            ("CONFIG_MACH_SAMD21", vec!["atsamd/watchdog.rs"]),
            ("CONFIG_MACH_SAMD21", vec!["atsamd/clock.rs", "atsamd/timer.rs", "generic/timer_irq.rs"]),
            ("CONFIG_MACH_SAMX5", vec!["atsamd/samd51_watchdog.rs"]),
            ("CONFIG_MACH_SAMX5", vec!["atsamd/samd51_clock.rs", "generic/armcm_timer.rs"]),
        ];
        for (c, files) in opt {
            if on(c) {
                out.extend(files.iter().copied());
            }
        }
    } else if on("CONFIG_MACH_SIMU") {
        // src/simulator/Makefile
        out.extend(["simulator/main.rs", "simulator/gpio.rs", "simulator/timer.rs", "simulator/serial.rs"]);
        out.extend(["generic/crc16_ccitt.rs", "generic/alloc.rs"]);
        out.extend(["generic/timer_irq.rs", "generic/serial_irq.rs"]);
    } else {
        panic!("only the linux, stm32, rp2040, lpc176x, atsam and simulator boards are ported");
    }
    // src/indx/Makefile (its c++ objects link after all the c ones)
    if on("CONFIG_WANT_INDX_HEATER") {
        out.extend([
            "indx/ir_sensor.rs",
            "indx/heater.rs",
            "indx/coil_driver.rs",
            "indx/pid.rs",
            "indx/i2c_bus.rs",
            "indx/status_led_driver.rs",
            "indx/external_clocks.rs",
        ]);
    }
    // (make: a source listed twice is built once, at its first place)
    let mut seen = BTreeSet::new();
    out.retain(|f| seen.insert(*f));
    out
}

/// The device header of an stm32 / rp2040 / lpc176x / atsam build (src/<board>/chip/<mcu>.rs,
/// made by tools/gen_stm32_regs.py): its path and the names it defines
/// (the C `#ifdef NAME`, here `#[cfg(has_NAME)]`)
fn chip_header(root: &Path, config: &kconfig::Config) -> Option<(PathBuf, BTreeSet<String>)> {
    let mcu = config.string("CONFIG_MCU").unwrap();
    let (board, file) = if config.enabled("CONFIG_MACH_STM32") {
        ("stm32", mcu.to_string())
    } else if config.enabled("CONFIG_MACH_RPXXXX") {
        ("rp2040", mcu.to_string())
    } else if config.enabled("CONFIG_MACH_ATSAM") || config.enabled("CONFIG_MACH_ATSAMD") {
        (if config.enabled("CONFIG_MACH_ATSAMD") { "atsamd" } else { "atsam" }, mcu.to_string())
    } else if config.enabled("CONFIG_MACH_LPC176X") {
        // (LPC17xx.h, for both lpc1768 and lpc1769)
        ("lpc176x", "lpc17xx".to_string())
    } else {
        return None;
    };
    let path = root.join("src").join(board).join("chip").join(format!("{}.rs", file));
    if !path.exists() {
        panic!("no {}: make it with tools/gen_stm32_regs.py", path.display());
    }
    let text = read(&path);
    let names = text
        .lines()
        .filter_map(|l| l.strip_prefix("pub const ").and_then(|r| r.split(':').next()))
        .map(|n| n.to_string())
        .collect();
    Some((path, names))
}

/// A linker script (generic/armcm_link.lds.S, rp2040/rpxxxx_link.lds.S)
/// through the C preprocessor: its `#if CONFIG_X` / `#else` blocks, its
/// `#define`s, CONFIG_X values from the configuration, `//` comments removed
fn armcm_link(src: &str, config: &kconfig::Config) -> String {
    let mut out = String::new();
    // (per open #if: skipping, and whether a branch was taken)
    let mut skip: Vec<(bool, bool)> = vec![];
    let mut defines: Vec<(String, String)> = vec![];
    let skipping = |skip: &Vec<(bool, bool)>| skip.iter().any(|s| s.0);
    for line in src.lines() {
        let t = line.trim();
        if let Some(c) = t.strip_prefix("#if ") {
            let on = config.enabled(c.trim());
            skip.push((!on, on));
            continue;
        }
        if t.starts_with("#else") {
            let (_, taken) = skip.pop().unwrap();
            skip.push((taken, true));
            continue;
        }
        if t.starts_with("#endif") {
            skip.pop();
            continue;
        }
        if t.starts_with("#include") || skipping(&skip) {
            continue;
        }
        if let Some(d) = t.strip_prefix("#define ") {
            let (name, value) = d.trim().split_once(char::is_whitespace).unwrap();
            defines.push((name.to_string(), value.trim().to_string()));
            continue;
        }
        let mut l = match line.find("//") {
            Some(i) => line[..i].trim_end().to_string(),
            None => line.to_string(),
        };
        if l.trim().is_empty() && !line.trim().is_empty() {
            continue;
        }
        for (name, value) in &defines {
            l = replace_word(&l, name, value);
        }
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

/// `text` with the whole word `word` replaced
fn replace_word(text: &str, word: &str, by: &str) -> String {
    let is_id = |c: char| c.is_ascii_alphanumeric() || c == '_';
    let mut out = String::new();
    let mut rest = text;
    while let Some(i) = rest.find(word) {
        let before = rest[..i].chars().next_back();
        let after = rest[i + word.len()..].chars().next();
        out += &rest[..i];
        if before.is_some_and(is_id) || after.is_some_and(is_id) {
            out += word;
        } else {
            out += by;
        }
        rest = &rest[i + word.len()..];
    }
    out + rest
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

    // stm32 / rp2040: the chip header, has_NAME cfgs, the linker script
    let chip = chip_header(&root, &config);
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
        let script = if config.enabled("CONFIG_MACH_RPXXXX") {
            "rpxxxx_link.lds.S"
        } else if config.enabled("CONFIG_MACH_SAME70") {
            "same70_link.lds.S"
        } else {
            "armcm_link.lds.S"
        };
        let lds = armcm_link(&read(&root.join("build").join(script)), &config);
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
    commands::Request { decl: parse_decl(line), module: "crate".into() }
}

/// a compile_time_request.txt line as the request it makes
fn parse_decl(line: &str) -> scan::Decl {
    use scan::Decl;
    let (cmd, rest) = line.split_once(' ').unwrap_or((line, ""));
    let words: Vec<&str> = rest.split_whitespace().collect();
    // CTR_INT values back to expressions the evaluator reads
    let int = |w: &str| w.replace("+0x", "0x").replace("-0x", "-0x");
    match cmd {
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
        "DECL_ARMCM_IRQ" => Decl::ArmcmIrq { func: words[0].into(), expr: int(words[1]) },
        other => panic!("Unknown build time command '{}'", other),
    }
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
        Err(_) => panic!("firmware source {} missing (not ported yet)", path.display()),
    }
}
