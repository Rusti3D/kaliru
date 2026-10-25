// Scripted serialqueue exchange with a fake mcu on a pty; twin of
// tools/sqtest_ref.py (same script, same output). Development tool.
use kaliru::chelper::msgblock::msgblock_crc16_ccitt;
use kaliru::chelper::serialqueue::{alloc_commandqueue, CommandQueue, SerialQueue, SQT_UART};
use std::os::fd::RawFd;
use std::sync::Arc;
use std::time::Duration;

fn block(seq: u8, payload: &[u8]) -> Vec<u8> {
    let mut msg = vec![payload.len() as u8 + 5, 0x10 | (seq & 0x0f)];
    msg.extend_from_slice(payload);
    let c = msgblock_crc16_ccitt(&msg);
    msg.extend_from_slice(&[(c >> 8) as u8, (c & 0xff) as u8, 0x7e]);
    msg
}

fn wait_readable(fd: RawFd, timeout: f64) -> bool {
    let mut p = libc::pollfd { fd, events: libc::POLLIN, revents: 0 };
    // SAFETY: p is one live pollfd
    unsafe { libc::poll(&mut p, 1, (timeout * 1000.) as i32) > 0 }
}

fn mcu_read(fd: RawFd) -> String {
    // wait for data, then collect the rest of the burst
    let mut out = vec![];
    if !wait_readable(fd, 2.0) {
        return String::new();
    }
    while wait_readable(fd, 0.02) {
        let mut buf = [0u8; 4096];
        // SAFETY: buf is a live buffer of buf.len() bytes
        let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n <= 0 {
            break;
        }
        out.extend_from_slice(&buf[..n as usize]);
    }
    hex(&out)
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{:02x}", x)).collect()
}

fn write(fd: RawFd, data: &[u8]) {
    // SAFETY: data is a live slice of data.len() bytes
    unsafe { libc::write(fd, data.as_ptr() as *const libc::c_void, data.len()) };
}

fn sleep() {
    std::thread::sleep(Duration::from_millis(50));
}

fn pull(sq: &SerialQueue) -> String {
    sleep();
    let p = sq.pull();
    format!("len={} notify={} msg={}", p.len, p.notify_id, hex(&p.msg))
}

fn set_raw(fd: RawFd) {
    // SAFETY: termios is plain data (zeroed is valid) and lives across the calls
    unsafe {
        let mut t: libc::termios = std::mem::zeroed();
        libc::tcgetattr(fd, &mut t);
        libc::cfmakeraw(&mut t);
        libc::tcsetattr(fd, libc::TCSAFLUSH, &t);
    }
}

fn main() {
    let (mut master, mut slave) = (0, 0);
    // SAFETY: master / slave are live ints; name, termios and winsize may be null
    unsafe { libc::openpty(&mut master, &mut slave, std::ptr::null_mut(), std::ptr::null(), std::ptr::null()) };
    set_raw(master);
    set_raw(slave);

    let sq: Arc<SerialQueue> = SerialQueue::alloc(slave, SQT_UART, 0, "serialq mcu").unwrap();
    sq.set_wire_frequency(250000.0);
    sq.set_receive_window(192);
    let cq = alloc_commandqueue();
    let cq2 = alloc_commandqueue();
    // est_freq 0: messages go out as soon as they are queued
    sq.set_clock_est(0.0, 0.0, 0, 0);
    let send = |q: &CommandQueue, data: &[u8], notify: u64, reqc: u64| sq.send(q, data, 0, reqc, notify);

    send(&cq, &[1, 2, 3], 0, 0);
    println!("write1 {}", mcu_read(master));
    write(master, &block(2, &[]));
    sleep();
    send(&cq, &[4, 5], 7, 0);
    println!("write2 {}", mcu_read(master));
    write(master, &block(3, &[9, 9, 9]));
    println!("pull1 {}", pull(&sq));
    write(master, &block(3, &[]));
    println!("pull2 {}", pull(&sq));
    // two queues, both ready: one block, lowest req_clock first
    send(&cq2, &[0x20, 1], 0, 5);
    send(&cq, &[0x21, 1], 0, 3);
    println!("write3 {}", mcu_read(master));
    // no ack: wait for the timeout retransmit (sync byte + pending blocks)
    println!("retransmit {}", mcu_read(master));
    write(master, &block(4, &[]));
    sleep();
    send(&cq, &[0x30], 0, 0);
    println!("write4 {}", mcu_read(master));
    write(master, &block(4, &[])); // duplicate ack = nak -> fast retransmit
    println!("nak retransmit {}", mcu_read(master));
    write(master, &block(5, &[]));
    sleep();
    // garbage ending in a sync byte, then a data block
    let mut garbage = b"\x01\x02garbage\x7e".to_vec();
    garbage.extend(block(5, &[1, 2]));
    write(master, &garbage);
    println!("pull3 {}", pull(&sq));
    let stats: Vec<String> = sq
        .get_stats()
        .split_whitespace()
        .filter(|p| !matches!(p.split('=').next(), Some("srtt" | "rttvar" | "rto")))
        .map(|s| s.to_string())
        .collect();
    println!("stats {}", stats.join(" "));
    for sentq in [true, false] {
        let old = sq.extract_old(sentq, 1024);
        let msgs: Vec<String> = old.iter().map(|m| hex(&m.msg)).collect();
        println!("old{} {}", sentq as i32, msgs.join(" "));
    }
    sq.exit();
    println!("pull after exit {}", pull(&sq));
    drop(sq);
    for line in Vec::<String>::new() {
        println!("{}", line);
    }
}
