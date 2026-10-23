// Script to parse a serial port data dump (port of klippy/parsedump.py)
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use kaliru::msgproto;
use std::io::Write;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() != 2 {
        eprintln!("usage: parsedump <dictionary> <data file>");
        std::process::exit(1);
    }
    let dictionary = std::fs::read(&args[0]).unwrap_or_else(|e| fail(&e.to_string()));

    let mut mp = msgproto::MessageParser::new("");
    mp.process_identify(&dictionary, false).unwrap_or_else(|e| fail(&e.msg));

    // python reads 4096 byte chunks; check_packet() waits for more data on a
    // partial message, so parsing the whole file at once is equivalent
    let data = std::fs::read(&args[1]).unwrap_or_else(|e| fail(&e.to_string()));
    let mut data = &data[..];
    let mut out = std::io::BufWriter::new(std::io::stdout().lock());
    loop {
        let l = mp.check_packet(data);
        if l == 0 {
            break;
        }
        if l < 0 {
            // logging.error(); importing the klippy package configures the
            // default "LEVEL:logger:msg" format (verified against python)
            eprintln!("ERROR:root:Invalid data");
            data = &data[(-l) as usize..];
            continue;
        }
        let msgs = mp.dump(&data[..l as usize]).unwrap_or_else(|e| fail(&e.msg));
        writeln!(out, "{}", msgs[1..].join("\n")).unwrap();
        data = &data[l as usize..];
    }
}

fn fail(msg: &str) -> ! {
    eprintln!("{}", msg);
    std::process::exit(1);
}
