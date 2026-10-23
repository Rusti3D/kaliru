// Runs a pins.py operation script for parity testing; twin of
// tools/pinsdump_ref.py. Development tool. Script lines:
//   chip <name> | parse <inv> <pull> <desc> | lookup <inv> <pull> <share|-> <desc>
//   multi <desc> | reset <desc> | alias <chip> <alias> <pin>
//   reserve <chip> <pin> <name> | update <chip> <cmd...> | new   (fresh PrinterPins)
// usage: pinsdump <script>
use kaliru::pins::{PinChip, PinParams, PrinterPins, SharedPinParams};
use std::any::Any;
use std::rc::Rc;

struct Chip;
impl PinChip for Chip {
    fn setup_pin(&self, _t: &str, _p: &SharedPinParams) -> Result<Rc<dyn Any>, String> {
        Ok(Rc::new(()))
    }
}

fn show(p: &PinParams) -> String {
    format!(
        "chip_name={} pin={} invert={} pullup={} share_type={}",
        kaliru::pyfmt::py_str_repr(&p.chip_name),
        kaliru::pyfmt::py_str_repr(&p.pin),
        p.invert,
        p.pullup,
        p.share_type.as_deref().map(kaliru::pyfmt::py_str_repr).unwrap_or_else(|| "None".into())
    )
}

fn main() {
    let script = std::fs::read_to_string(std::env::args().nth(1).expect("usage: pinsdump <script>")).unwrap();
    let mut pins = PrinterPins::new();
    // identity of returned pin_params objects, numbered in first-seen order
    let mut seen: Vec<SharedPinParams> = vec![];
    for line in script.lines() {
        let (op, rest) = line.split_once(' ').unwrap_or((line, ""));
        let out = match op {
            "new" => {
                pins = PrinterPins::new();
                seen.clear();
                "ok".to_string()
            }
            "chip" => pins.register_chip(rest, Rc::new(Chip)).map(|_| "ok".into()).unwrap_or_else(|e| format!("ERR {}", e)),
            "parse" => {
                let mut it = rest.splitn(3, ' ');
                let (inv, pull, desc) = (it.next().unwrap() == "1", it.next().unwrap() == "1", it.next().unwrap_or(""));
                pins.parse_pin(desc, inv, pull).map(|p| show(&p)).unwrap_or_else(|e| format!("ERR {}", e))
            }
            "lookup" => {
                let mut it = rest.splitn(4, ' ');
                let (inv, pull, share) = (it.next().unwrap() == "1", it.next().unwrap() == "1", it.next().unwrap());
                let desc = it.next().unwrap_or("");
                let share = if share == "-" { None } else { Some(share) };
                match pins.lookup_pin(desc, inv, pull, share) {
                    Ok(p) => {
                        let id = match seen.iter().position(|s| Rc::ptr_eq(s, &p)) {
                            Some(i) => i,
                            None => {
                                seen.push(p.clone());
                                seen.len() - 1
                            }
                        };
                        let s = show(&p.borrow());
                        format!("id={} {}", id, s)
                    }
                    Err(e) => format!("ERR {}", e),
                }
            }
            "multi" => pins.allow_multi_use_pin(rest).map(|_| "ok".into()).unwrap_or_else(|e| format!("ERR {}", e)),
            "reset" => match pins.parse_pin(rest, false, false) {
                Ok(p) => {
                    let key = format!("{}:{}", p.chip_name, p.pin);
                    if pins.active_pins.contains_key(&key) {
                        pins.reset_pin_sharing(&p);
                        "ok".into()
                    } else {
                        format!("ERR KeyError {}", kaliru::pyfmt::py_str_repr(&key))
                    }
                }
                Err(e) => format!("ERR {}", e),
            },
            "alias" | "reserve" | "update" => {
                let (chip, args) = rest.split_once(' ').unwrap_or((rest, ""));
                match pins.get_pin_resolver(chip) {
                    Err(e) => format!("ERR {}", e),
                    Ok(r) => {
                        let res = match op {
                            "alias" => {
                                let (a, p) = args.split_once(' ').unwrap_or((args, ""));
                                r.alias_pin(a, p).map(|_| "ok".to_string())
                            }
                            "reserve" => {
                                let (p, n) = args.split_once(' ').unwrap_or((args, ""));
                                r.reserve_pin(p, n).map(|_| "ok".to_string())
                            }
                            _ => r.update_command(args),
                        };
                        res.unwrap_or_else(|e| format!("ERR {}", e))
                    }
                }
            }
            _ => continue,
        };
        println!("{} -> {}", line, kaliru::pyfmt::py_str_repr(&out));
    }
}
