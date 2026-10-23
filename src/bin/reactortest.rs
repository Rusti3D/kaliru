// Reactor scenarios printing the order of events; twin of
// tools/reactortest_ref.py (same scenarios, same output). Development tool.
use kaliru::reactor::{Reactor, NEVER, NOW};
use std::cell::RefCell;
use std::rc::Rc;

thread_local! {
    static LOG: RefCell<Vec<String>> = const { RefCell::new(vec![]) };
}

fn log(s: impl Into<String>) {
    LOG.with(|l| l.borrow_mut().push(s.into()));
}

type R = Rc<Reactor>;

fn run(name: &str, setup: fn(&R)) {
    let r = Reactor::new(true);
    LOG.with(|l| l.borrow_mut().clear());
    setup(&r);
    r.run();
    log(format!("greenlets={}", r.greenlet_count()));
    r.finalize();
    println!("{}: {}", name, LOG.with(|l| l.borrow().join(" | ")));
}

fn at(r: &R, t: f64, f: impl FnMut() + 'static) {
    let f = RefCell::new(f);
    r.register_timer(
        Box::new(move |_| {
            (f.borrow_mut())();
            NEVER
        }),
        t,
    );
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn timers(r: &R) {
    let t0 = r.monotonic();
    let mk = |name: &'static str, n: u32, step: f64| {
        let r = r.clone();
        let mut cnt = 0;
        Box::new(move |e: f64| {
            cnt += 1;
            log(format!("{}{}", name, cnt));
            if cnt < n {
                return e + step;
            }
            if name == "c" {
                r.end();
            }
            NEVER
        })
    };
    r.register_timer(mk("a", 1, 0.), t0 + 0.06);
    r.register_timer(mk("b", 3, 0.025), t0 + 0.02);
    r.register_timer(mk("c", 1, 0.), t0 + 0.10);
}

fn nested(r: &R) {
    let t0 = r.monotonic();
    let ra = r.clone();
    at(r, t0 + 0.01, move || {
        log("A1");
        ra.pause(t0 + 0.08).unwrap();
        log("A2");
    });
    let rb = r.clone();
    at(r, t0 + 0.03, move || {
        log("B1");
        rb.pause(t0 + 0.05).unwrap();
        log("B2");
    });
    let rc = r.clone();
    at(r, t0 + 0.11, move || {
        log("C");
        rc.end();
    });
}

fn repause(r: &R) {
    let t0 = r.monotonic();
    let ra = r.clone();
    at(r, t0 + 0.01, move || {
        log("A1");
        ra.pause(t0 + 0.03).unwrap();
        log("A2");
        ra.pause(t0 + 0.06).unwrap();
        log("A3");
    });
    at(r, t0 + 0.045, || log("B"));
    let re = r.clone();
    at(r, t0 + 0.09, move || {
        log("end");
        re.end();
    });
}

fn completion(r: &R) {
    let t0 = r.monotonic();
    let comp = r.completion::<String>();
    let c = comp.clone();
    at(r, t0 + 0.01, move || {
        log("W1");
        let res = c.wait(NEVER, None).unwrap();
        log(format!("W got {}", res.unwrap_or_else(|| "None".into())));
    });
    let c = comp.clone();
    at(r, t0 + 0.015, move || {
        let res = c.wait(t0 + 0.03, Some("timeout".into())).unwrap();
        log(format!("W2 got {}", res.unwrap_or_else(|| "None".into())));
    });
    let c = comp.clone();
    at(r, t0 + 0.05, move || {
        log("X");
        c.complete("done".into());
    });
    let (c, re) = (comp.clone(), r.clone());
    at(r, t0 + 0.08, move || {
        log(format!("test={}", py_bool(c.test())));
        re.end();
    });
}

fn mutex(r: &R) {
    let t0 = r.monotonic();
    let m = r.mutex(false);
    let (m1, r1) = (m.clone(), r.clone());
    at(r, t0 + 0.01, move || {
        m1.lock().unwrap();
        log("M1 locked");
        r1.pause(t0 + 0.05).unwrap();
        log("M1 unlock");
        m1.unlock();
    });
    for (t, name) in [(0.02, "M2"), (0.03, "M3")] {
        let mm = m.clone();
        at(r, t0 + t, move || {
            log(format!("{} wait", name));
            mm.lock().unwrap();
            log(format!("{} locked test={}", name, py_bool(mm.test())));
            mm.unlock();
        });
    }
    let (me, re) = (m.clone(), r.clone());
    at(r, t0 + 0.1, move || {
        log(format!("end test={}", py_bool(me.test())));
        re.end();
    });
}

fn callback(r: &R) {
    let t0 = r.monotonic();
    let c = r.register_callback(
        |_| {
            log("cb");
            "cb-result".to_string()
        },
        NOW,
    );
    at(r, t0 + 0.02, move || {
        let res = c.wait(NEVER, None).unwrap();
        log(format!("got {}", res.unwrap_or_else(|| "None".into())));
    });
    let re = r.clone();
    at(r, t0 + 0.05, move || {
        log("end");
        re.end();
    });
}

fn async_cb(r: &R) {
    let t0 = r.monotonic();
    let h = r.async_handle();
    std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(20));
        // runs on the reactor thread, so it may use the thread local log
        h.register_async_callback(Box::new(|_| log("async")), NOW);
    });
    let re = r.clone();
    at(r, t0 + 0.08, move || {
        log("end");
        re.end();
    });
}

fn no_pause(r: &R) {
    let t0 = r.monotonic();
    let rp = r.clone();
    at(r, t0 + 0.01, move || {
        {
            let _guard = rp.assert_no_pause();
            match rp.pause(rp.monotonic() + 0.01) {
                Ok(_) => log("paused"),
                Err(e) => log(format!("error: {}", e)),
            }
        }
        rp.end();
    });
}

fn sys_pause(r: &R) {
    let t = r.monotonic();
    let e = r.pause(t + 0.02).unwrap();
    log(format!("sys pause ok={}", py_bool(e >= t + 0.02)));
    let re = r.clone();
    at(r, NOW, move || re.end());
}

fn delay(r: &R) {
    let t0 = r.monotonic();
    let rd = r.clone();
    at(r, t0 + 0.01, move || {
        log("D1");
        rd.delay_for_ms(30.).unwrap();
        log("D2");
    });
    at(r, t0 + 0.02, || log("E"));
    let re = r.clone();
    at(r, t0 + 0.07, move || re.end());
}

fn main() {
    let scenarios: [(&str, fn(&R)); 10] = [
        ("timers", timers),
        ("nested", nested),
        ("repause", repause),
        ("completion", completion),
        ("mutex", mutex),
        ("callback", callback),
        ("async", async_cb),
        ("no_pause", no_pause),
        ("sys_pause", sys_pause),
        ("delay", delay),
    ];
    for (name, f) in scenarios {
        run(name, f);
    }
}
