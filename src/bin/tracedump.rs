// Decodes a serialqueue trace (tools/ref_trace.patch, RK_SQ_TRACE) into
// readable messages. Development tool, no python counterpart.
//   <serialqueue name> q<queue #> <min_clock> <req_clock> <decoded message>
// With --check undecodable messages make the exit code non-zero.
//
// usage: tracedump [--check] <trace file> <mcu>=<dict> [<mcu>=<dict>...]
use kaliru::msgproto::{MessageParser, MsgFormat};
use std::collections::HashMap;
use std::io::Write;

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let check = args.first().map(|a| a == "--check").unwrap_or(false);
    if check {
        args.remove(0);
    }
    if args.len() < 2 {
        eprintln!("usage: tracedump [--check] <trace file> <mcu>=<dict> [<mcu>=<dict>...]");
        std::process::exit(2);
    }
    let mut parsers = HashMap::new();
    for a in &args[1..] {
        let (name, path) = a.split_once('=').expect("<mcu>=<dict>");
        let mut mp = MessageParser::new("");
        mp.process_identify(&std::fs::read(path).expect("read dict"), false).expect("dict");
        // serialqueue names are char[16] in C: at most 15 bytes
        let mut sq = format!("serialq {}", name);
        sq.truncate(15);
        parsers.insert(sq, mp);
    }
    let trace = std::fs::read_to_string(&args[0]).expect("read trace");
    let mut out = std::io::BufWriter::new(std::io::stdout().lock());
    let (mut n, mut bad) = (0, 0);
    for line in trace.lines() {
        // the serialqueue name holds a space: split the fixed fields from the right
        let f: Vec<&str> = line.rsplitn(5, ' ').collect();
        let (hex, req, min, q, sq) = (f[0], f[1], f[2], f[3], f[4]);
        let msg: Vec<u8> = (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
        let mp = parsers.get(sq).unwrap_or_else(|| panic!("no dict for '{}'", sq));
        let text = decode(mp, &msg).unwrap_or_else(|e| {
            bad += 1;
            format!("DECODE ERROR {}: {}", e, hex)
        });
        writeln!(out, "{} {} {} {} {}", sq, q, min, req, text).unwrap();
        n += 1;
    }
    out.flush().unwrap();
    if check {
        eprintln!("{} messages, {} failed to decode", n, bad);
        std::process::exit(if bad > 0 { 1 } else { 0 });
    }
}

// Re-encoding is not an identity: the host may pass values a parse can't
// return (eg -1 for %u, 64bit clocks), so only decoding is checked here.
fn decode(mp: &MessageParser, msg: &[u8]) -> Result<String, String> {
    let (msgid, _) = kaliru::msgproto::BaseType::Int32.parse(msg, 0).map_err(|e| e.msg)?;
    let mid = match mp.messages_by_id.get(&msgid.as_int().unwrap_or(-1)) {
        Some(MsgFormat::Message(m)) => m.clone(),
        _ => return Err("unknown or output message id".into()),
    };
    let (params, pos) = mid.parse(msg, 0).map_err(|e| e.msg)?;
    if pos != msg.len() {
        return Err("extra data".into());
    }
    mid.format_params(&params).map_err(|e| e.msg)
}
