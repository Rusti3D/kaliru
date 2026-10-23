// Micro-controller clock synchronization (port of klippy/clocksync.py)
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// ClockSync and SecondarySync are one struct; `secondary` holds the
// SecondarySync state. python's "%d" of a float truncates (int()).
use crate::error::KResult;
use crate::logging;
use crate::msgproto::{Params, Val};
use crate::reactor::{Reactor, Timer, NEVER, NOW};
use crate::serialhdl::{params_time, SerialReader};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

pub const RTT_AGE: f64 = 0.000010 / (60.0 * 60.0);
pub const DECAY: f64 = 1.0 / 30.0;
pub const TRANSMIT_EXTRA: f64 = 0.001;

pub struct Secondary {
    pub main_sync: Rc<ClockSync>,
    pub clock_adj: Cell<(f64, f64)>,
    pub last_sync_time: Cell<f64>,
}

pub struct ClockSync {
    reactor: Rc<Reactor>,
    serial: RefCell<Option<Rc<SerialReader>>>,
    get_clock_timer: RefCell<Option<Timer>>,
    get_clock_cmd: RefCell<Vec<u8>>,
    cmd_queue: RefCell<Option<crate::chelper::serialqueue::CommandQueue>>,
    queries_pending: Cell<i64>,
    pub mcu_freq: Cell<f64>,
    last_clock: Cell<i64>,
    pub clock_est: Cell<(f64, f64, f64)>,
    // Minimum round-trip-time tracking
    min_half_rtt: Cell<f64>,
    min_rtt_time: Cell<f64>,
    // Linear regression of mcu clock and system sent_time
    time_avg: Cell<f64>,
    time_variance: Cell<f64>,
    clock_avg: Cell<f64>,
    clock_covariance: Cell<f64>,
    prediction_variance: Cell<f64>,
    last_prediction_time: Cell<f64>,
    pub secondary: Option<Secondary>,
}

fn trunc(f: f64) -> i64 {
    f as i64
}

impl ClockSync {
    pub fn new(reactor: &Rc<Reactor>) -> Rc<Self> {
        Self::build(reactor, None)
    }

    /// SecondarySync(reactor, main_sync)
    pub fn new_secondary(reactor: &Rc<Reactor>, main_sync: Rc<ClockSync>) -> Rc<Self> {
        Self::build(
            reactor,
            Some(Secondary { main_sync, clock_adj: Cell::new((0.0, 1.0)), last_sync_time: Cell::new(0.0) }),
        )
    }

    fn build(reactor: &Rc<Reactor>, secondary: Option<Secondary>) -> Rc<Self> {
        let cs = Rc::new(ClockSync {
            reactor: reactor.clone(),
            serial: RefCell::new(None),
            get_clock_timer: RefCell::new(None),
            get_clock_cmd: RefCell::new(vec![]),
            cmd_queue: RefCell::new(None),
            queries_pending: Cell::new(0),
            mcu_freq: Cell::new(1.0),
            last_clock: Cell::new(0),
            clock_est: Cell::new((0.0, 0.0, 0.0)),
            min_half_rtt: Cell::new(999999999.9),
            min_rtt_time: Cell::new(0.0),
            time_avg: Cell::new(0.0),
            time_variance: Cell::new(0.0),
            clock_avg: Cell::new(0.0),
            clock_covariance: Cell::new(0.0),
            prediction_variance: Cell::new(0.0),
            last_prediction_time: Cell::new(0.0),
            secondary,
        });
        let w = Rc::downgrade(&cs);
        let t = reactor.register_timer(
            Box::new(move |e| match w.upgrade() {
                Some(cs) => cs.get_clock_event(e),
                None => NEVER,
            }),
            NEVER,
        );
        *cs.get_clock_timer.borrow_mut() = Some(t);
        cs
    }

    pub fn disconnect(&self) {
        if let Some(t) = self.get_clock_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NEVER);
        }
    }

    pub fn connect(self: &Rc<Self>, serial: &Rc<SerialReader>) -> KResult<()> {
        *self.serial.borrow_mut() = Some(serial.clone());
        self.mcu_freq.set(serial.get_msgparser().get_constant_float("CLOCK_FREQ", crate::msgproto::Default::Sentinel)?);
        // Load initial clock and frequency
        let params = serial.send_with_response("get_uptime", "uptime")?;
        let high = params.get("high").and_then(|v| v.as_int()).unwrap_or(0);
        let clock = params.get("clock").and_then(|v| v.as_int()).unwrap_or(0);
        self.last_clock.set((high << 32) | clock);
        self.clock_avg.set(self.last_clock.get() as f64);
        self.time_avg.set(params_time(&params, "#sent_time"));
        self.clock_est.set((self.time_avg.get(), self.clock_avg.get(), self.mcu_freq.get()));
        self.prediction_variance.set((0.001 * self.mcu_freq.get()).powi(2));
        // Enable periodic get_clock timer
        for _ in 0..8 {
            self.reactor.pause(self.reactor.monotonic() + 0.050)?;
            self.last_prediction_time.set(-9999.0);
            let params = serial.send_with_response("get_clock", "clock")?;
            self.handle_clock(&params);
        }
        *self.get_clock_cmd.borrow_mut() = serial.get_msgparser().create_command("get_clock")?;
        *self.cmd_queue.borrow_mut() = Some(serial.alloc_command_queue());
        let w = Rc::downgrade(self);
        serial.register_response(
            Some(Rc::new(move |p: &Params| {
                if let Some(cs) = w.upgrade() {
                    cs.handle_clock(p);
                }
                Ok(())
            })),
            "clock",
            None,
        );
        if let Some(t) = self.get_clock_timer.borrow().as_ref() {
            self.reactor.update_timer(t, NOW);
        }
        if let Some(sec) = &self.secondary {
            // SecondarySync.connect
            sec.clock_adj.set((0.0, self.mcu_freq.get()));
            let curtime = self.reactor.monotonic();
            let main_print_time = sec.main_sync.estimated_print_time(curtime);
            let local_print_time = self.estimated_print_time(curtime);
            sec.clock_adj.set((main_print_time - local_print_time, self.mcu_freq.get()));
            self.calibrate_clock(0.0, curtime);
        }
        Ok(())
    }

    pub fn connect_file(&self, serial: &Rc<SerialReader>, pace: bool) -> KResult<()> {
        *self.serial.borrow_mut() = Some(serial.clone());
        self.mcu_freq.set(serial.get_msgparser().get_constant_float("CLOCK_FREQ", crate::msgproto::Default::Sentinel)?);
        self.clock_est.set((0.0, 0.0, self.mcu_freq.get()));
        let freq = if pace { self.mcu_freq.get() } else { 1000000000000.0 };
        serial.set_clock_est(freq, self.reactor.monotonic(), 0, 0);
        if let Some(sec) = &self.secondary {
            sec.clock_adj.set((0.0, self.mcu_freq.get()));
        }
        Ok(())
    }

    // MCU clock querying (_handle_clock is invoked from background thread)
    fn get_clock_event(&self, eventtime: f64) -> f64 {
        if let (Some(serial), Some(q)) = (self.serial.borrow().as_ref(), self.cmd_queue.borrow().as_ref()) {
            let _ = serial.raw_send(&self.get_clock_cmd.borrow(), 0, 0, q);
        }
        self.queries_pending.set(self.queries_pending.get() + 1);
        // Use an unusual time for the next event so clock messages
        // don't resonate with other periodic events.
        eventtime + 0.9839
    }

    fn handle_clock(&self, params: &Params) {
        self.queries_pending.set(0);
        // Extend clock to 64bit
        let last_clock = self.last_clock.get();
        let pclock = params.get("clock").and_then(Val::as_int).unwrap_or(0);
        let clock_delta = (pclock - last_clock) & 0xFFFFFFFF;
        let clock = last_clock + clock_delta;
        self.last_clock.set(clock);
        // Check if this is the best round-trip-time seen so far
        let sent_time = params_time(params, "#sent_time");
        if sent_time == 0. {
            return;
        }
        let receive_time = params_time(params, "#receive_time");
        let half_rtt = 0.5 * (receive_time - sent_time);
        let aged_rtt = (sent_time - self.min_rtt_time.get()) * RTT_AGE;
        if half_rtt < self.min_half_rtt.get() + aged_rtt {
            self.min_half_rtt.set(half_rtt);
            self.min_rtt_time.set(sent_time);
            logging::debug(&format!("new minimum rtt {:.3}: hrtt={:.6} freq={}", sent_time, half_rtt, trunc(self.clock_est.get().2)));
        }
        let clock = clock as f64;
        // Filter out samples that are extreme outliers
        let exp_clock = (sent_time - self.time_avg.get()) * self.clock_est.get().2 + self.clock_avg.get();
        let clock_diff2 = (clock - exp_clock).powi(2);
        if clock_diff2 > 25.0 * self.prediction_variance.get() && clock_diff2 > (0.000500 * self.mcu_freq.get()).powi(2) {
            if clock > exp_clock && sent_time < self.last_prediction_time.get() + 10.0 {
                logging::debug(&format!(
                    "Ignoring clock sample {:.3}: freq={} diff={} stddev={:.3}",
                    sent_time,
                    trunc(self.clock_est.get().2),
                    trunc(clock - exp_clock),
                    self.prediction_variance.get().sqrt()
                ));
                return;
            }
            logging::info(&format!(
                "Resetting prediction variance {:.3}: freq={} diff={} stddev={:.3}",
                sent_time,
                trunc(self.clock_est.get().2),
                trunc(clock - exp_clock),
                self.prediction_variance.get().sqrt()
            ));
            self.prediction_variance.set((0.001 * self.mcu_freq.get()).powi(2));
        } else {
            self.last_prediction_time.set(sent_time);
            self.prediction_variance.set((1.0 - DECAY) * (self.prediction_variance.get() + clock_diff2 * DECAY));
        }
        // Add clock and sent_time to linear regression
        let diff_sent_time = sent_time - self.time_avg.get();
        self.time_avg.set(self.time_avg.get() + DECAY * diff_sent_time);
        self.time_variance.set((1.0 - DECAY) * (self.time_variance.get() + diff_sent_time.powi(2) * DECAY));
        let diff_clock = clock - self.clock_avg.get();
        self.clock_avg.set(self.clock_avg.get() + DECAY * diff_clock);
        self.clock_covariance.set((1.0 - DECAY) * (self.clock_covariance.get() + diff_sent_time * diff_clock * DECAY));
        // Update prediction from linear regression
        let new_freq = self.clock_covariance.get() / self.time_variance.get();
        let pred_stddev = self.prediction_variance.get().sqrt();
        if let Some(serial) = self.serial.borrow().as_ref() {
            serial.set_clock_est(
                new_freq,
                self.time_avg.get() + TRANSMIT_EXTRA,
                trunc(self.clock_avg.get() - 3.0 * pred_stddev) as u64,
                clock as u64,
            );
        }
        self.clock_est.set((self.time_avg.get() + self.min_half_rtt.get(), self.clock_avg.get(), new_freq));
    }

    // clock frequency conversions
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        match &self.secondary {
            None => trunc(print_time * self.mcu_freq.get()),
            Some(sec) => {
                if sec.clock_adj.get().1 == 1.0 {
                    // ponytail: python also logs traceback.format_stack() lines here
                    logging::warning("Clock not yet synchronized, clock is untrustworthy");
                }
                let (adjusted_offset, adjusted_freq) = sec.clock_adj.get();
                trunc((print_time - adjusted_offset) * adjusted_freq)
            }
        }
    }

    pub fn clock_to_print_time(&self, clock: f64) -> f64 {
        match &self.secondary {
            None => clock / self.mcu_freq.get(),
            Some(sec) => {
                if sec.clock_adj.get().1 == 1.0 {
                    logging::warning("Clock not yet synchronized, print time is untrustworthy");
                }
                let (adjusted_offset, adjusted_freq) = sec.clock_adj.get();
                clock / adjusted_freq + adjusted_offset
            }
        }
    }

    // system time conversions
    pub fn get_clock(&self, eventtime: f64) -> i64 {
        let (sample_time, clock, freq) = self.clock_est.get();
        trunc(clock + (eventtime - sample_time) * freq)
    }

    pub fn estimate_clock_systime(&self, reqclock: f64) -> f64 {
        let (sample_time, clock, freq) = self.clock_est.get();
        (reqclock - clock) / freq + sample_time
    }

    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.clock_to_print_time(self.get_clock(eventtime) as f64)
    }

    // misc commands
    pub fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        let last_clock = self.last_clock.get();
        let mut clock_diff = (clock32 - last_clock) & 0xFFFFFFFF;
        clock_diff -= (clock_diff & 0x80000000) << 1;
        last_clock + clock_diff
    }

    pub fn is_active(&self) -> bool {
        self.queries_pending.get() <= 4
    }

    pub fn dump_debug(&self) -> String {
        let (sample_time, clock, freq) = self.clock_est.get();
        let base = format!(
            "clocksync state: mcu_freq={} last_clock={} clock_est=({:.3} {} {:.3}) min_half_rtt={:.6} min_rtt_time={:.3} time_avg={:.3}({:.3}) clock_avg={:.3}({:.3}) pred_variance={:.3}",
            trunc(self.mcu_freq.get()),
            self.last_clock.get(),
            sample_time,
            trunc(clock),
            freq,
            self.min_half_rtt.get(),
            self.min_rtt_time.get(),
            self.time_avg.get(),
            self.time_variance.get(),
            self.clock_avg.get(),
            self.clock_covariance.get(),
            self.prediction_variance.get()
        );
        match &self.secondary {
            None => base,
            Some(sec) => {
                let (o, f) = sec.clock_adj.get();
                format!("{} clock_adj=({:.3} {:.3})", base, o, f)
            }
        }
    }

    pub fn stats(&self, _eventtime: f64) -> String {
        let (_, _, freq) = self.clock_est.get();
        let base = format!("freq={}", trunc(freq));
        match &self.secondary {
            None => base,
            Some(sec) => format!("{} adj={}", base, trunc(sec.clock_adj.get().1)),
        }
    }

    pub fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> (f64, f64) {
        let Some(sec) = &self.secondary else {
            return (0.0, self.mcu_freq.get());
        };
        // Calculate: est_print_time = main_sync.estimatated_print_time()
        let (ser_time, ser_clock, ser_freq) = sec.main_sync.clock_est.get();
        let main_mcu_freq = sec.main_sync.mcu_freq.get();
        let est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock;
        let est_print_time = est_main_clock / main_mcu_freq;
        // Determine sync1_print_time and sync2_print_time
        let sync1_print_time = print_time.max(est_print_time);
        let sync2_print_time = (sync1_print_time + 4.0)
            .max(sec.last_sync_time.get())
            .max(print_time + 2.5 * (print_time - est_print_time));
        // Calc sync2_sys_time (inverse of main_sync.estimatated_print_time)
        let sync2_main_clock = sync2_print_time * main_mcu_freq;
        let sync2_sys_time = ser_time + (sync2_main_clock - ser_clock) / ser_freq;
        // Adjust freq so estimated print_time will match at sync2_print_time
        let sync1_clock = self.print_time_to_clock(sync1_print_time);
        let sync2_clock = self.get_clock(sync2_sys_time);
        let adjusted_freq = (sync2_clock - sync1_clock) as f64 / (sync2_print_time - sync1_print_time);
        let adjusted_offset = sync1_print_time - sync1_clock as f64 / adjusted_freq;
        // Apply new values
        sec.clock_adj.set((adjusted_offset, adjusted_freq));
        sec.last_sync_time.set(sync2_print_time);
        (adjusted_offset, adjusted_freq)
    }
}
