// Prints TimedRotatingFileHandler.computeRollover() results for timestamps
// given on the command line; twin of tools/qltest_ref.py. Development tool.
// usage: qltest <dir> <timestamp>...
use kaliru::queuelogger::{RotatingFile, When};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let log = format!("{}/ql.log", args[0]);
    for (name, when) in [("midnight", When::Midnight), ("S", When::Seconds)] {
        let f = RotatingFile::new(&log, when).unwrap();
        for t in &args[1..] {
            println!("{} {} {}", name, t, f.compute_rollover(t.parse().unwrap()));
        }
    }
}
