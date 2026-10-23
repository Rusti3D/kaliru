// Trigger sync "trsync" message dispatch (port of klippy/chelper/trdispatch.c)
//
// Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// The C code links trdispatch_mcu nodes into a list in the trdispatch and
// protects their fields with the trdispatch lock. Here the per-mcu state
// lives in the locked trdispatch state and a TrdispatchMcu is a handle (an
// id) into it; its fastreader finds the state through a Weak reference.
use super::msgblock::{
    clock_from_clock32, clock_from_time, clock_to_time, message_alloc_and_encode, msgblock_decode, ClockEstimate,
};
use super::pollreactor::PR_NEVER;
use super::serialqueue::{CommandQueue, FastReader, SerialQueue};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, Weak};

struct TdmState {
    id: usize,
    fr: Arc<FastReader>,
    sq: Arc<SerialQueue>,
    cq: CommandQueue,
    trsync_oid: u32,
    set_timeout_msgtag: u32,
    trigger_msgtag: u32,
    // Remaining fields protected by trdispatch lock
    last_status_clock: u64,
    expire_clock: u64,
    expire_ticks: u64,
    min_extend_ticks: u64,
    ce: ClockEstimate,
}

#[derive(Default)]
struct TdState {
    tdm_list: Vec<TdmState>,
    is_active: bool,
    can_trigger: bool,
    dispatch_reason: u32,
}

/// struct trdispatch
pub struct Trdispatch {
    lock: Arc<Mutex<TdState>>,
}

/// struct trdispatch_mcu (a handle into its trdispatch)
pub struct TrdispatchMcu {
    td: Weak<Mutex<TdState>>,
    id: usize,
}

static NEXT_TDM_ID: AtomicUsize = AtomicUsize::new(1);

// Send: trsync_trigger oid=%c reason=%c
fn send_trsync_trigger(tdm: &TdmState, dispatch_reason: u32) {
    let msg = [tdm.trigger_msgtag, tdm.trsync_oid, dispatch_reason];
    let qm = message_alloc_and_encode(&msg);
    tdm.sq.send_one(&tdm.cq, qm);
}

// Send: trsync_set_timeout oid=%c clock=%u
fn send_trsync_set_timeout(tdm: &TdmState) {
    let msg = [tdm.set_timeout_msgtag, tdm.trsync_oid, tdm.expire_clock as u32];
    let mut qm = message_alloc_and_encode(&msg);
    qm.req_clock = tdm.expire_clock;
    tdm.sq.send_one(&tdm.cq, qm);
}

// Handle a trsync_state message (callback from serialqueue fastreader)
fn handle_trsync_state(td: &Weak<Mutex<TdState>>, id: usize, data: &[u8]) {
    let Some(td) = td.upgrade() else { return };
    let mut st = td.lock().unwrap();
    let Some(idx) = st.tdm_list.iter().position(|m| m.id == id) else { return };

    // Parse: trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u
    let mut fields = [0u32; 5];
    let ret = msgblock_decode(&mut fields, data);
    if ret != 0 || fields[1] != st.tdm_list[idx].trsync_oid {
        return;
    }
    let (can_trigger, clock) = (fields[2], fields[4]);

    // Process message
    if !st.can_trigger {
        return;
    }

    if can_trigger == 0 {
        // mcu reports trigger or timeout - propagate to all mcus
        st.can_trigger = false;
        let reason = st.dispatch_reason;
        for m in &st.tdm_list {
            send_trsync_trigger(m, reason);
        }
        return;
    }

    // mcu is still working okay - update last_status_clock
    let tdm = &mut st.tdm_list[idx];
    tdm.ce = tdm.sq.get_clock_est();
    tdm.last_status_clock = clock_from_clock32(&tdm.ce, clock);

    // Determine minimum acknowledged time among all mcus
    let (mut min_time, mut next_min_time) = (PR_NEVER, PR_NEVER);
    let mut min_tdm = None;
    for m in &st.tdm_list {
        let status_time = clock_to_time(&m.ce, m.last_status_clock);
        if status_time < next_min_time {
            next_min_time = status_time;
            if status_time < min_time {
                next_min_time = min_time;
                min_time = status_time;
                min_tdm = Some(m.id);
            }
        }
    }
    if next_min_time == PR_NEVER {
        next_min_time = min_time;
    }

    // Send trsync_set_timeout messages to other mcus (if needed)
    for m in st.tdm_list.iter_mut() {
        let status_time = if Some(m.id) == min_tdm { next_min_time } else { min_time };
        let expire = clock_from_time(&m.ce, status_time).wrapping_add(m.expire_ticks);
        if expire.wrapping_sub(m.expire_clock) as i64 >= m.min_extend_ticks as i64 {
            m.expire_clock = expire;
            send_trsync_set_timeout(m);
        }
    }
}

impl Trdispatch {
    /// trdispatch_alloc()
    pub fn alloc() -> Trdispatch {
        Trdispatch { lock: Arc::new(Mutex::new(TdState::default())) }
    }

    /// trdispatch_start(): begin synchronization
    pub fn start(&self, dispatch_reason: u32) {
        let mut st = self.lock.lock().unwrap();
        if st.is_active || st.tdm_list.is_empty() {
            return;
        }
        st.dispatch_reason = dispatch_reason;
        st.is_active = true;
        st.can_trigger = true;
        let readers: Vec<_> = st.tdm_list.iter().map(|m| (m.sq.clone(), m.fr.clone())).collect();
        drop(st);

        // Register handle_trsync_state message parser for each mcu
        for (sq, fr) in readers {
            sq.add_fastreader(fr);
        }
    }

    /// trdispatch_stop(): cleanup after a test completes
    pub fn stop(&self) {
        let mut st = self.lock.lock().unwrap();
        if !st.is_active {
            return;
        }
        st.is_active = false;
        let readers: Vec<_> = st.tdm_list.iter().map(|m| (m.sq.clone(), m.fr.clone())).collect();
        drop(st);

        // Unregister handle_trsync_state message parsers
        for (sq, fr) in readers {
            sq.rm_fastreader(&fr);
        }
    }

    /// trdispatch_mcu_alloc()
    pub fn mcu_alloc(
        &self,
        sq: Arc<SerialQueue>,
        cq: CommandQueue,
        trsync_oid: u32,
        set_timeout_msgtag: u32,
        trigger_msgtag: u32,
        state_msgtag: u32,
    ) -> TrdispatchMcu {
        let id = NEXT_TDM_ID.fetch_add(1, Ordering::SeqCst);
        // Setup fastreader to match trsync_state messages
        let dummy = message_alloc_and_encode(&[state_msgtag, trsync_oid]);
        let weak = Arc::downgrade(&self.lock);
        let w = weak.clone();
        let fr = Arc::new(FastReader {
            prefix: dummy.msg[..dummy.len].to_vec(),
            func: Box::new(move |data| handle_trsync_state(&w, id, data)),
        });
        self.lock.lock().unwrap().tdm_list.push(TdmState {
            id,
            fr,
            sq,
            cq,
            trsync_oid,
            set_timeout_msgtag,
            trigger_msgtag,
            last_status_clock: 0,
            expire_clock: 0,
            expire_ticks: 0,
            min_extend_ticks: 0,
            ce: ClockEstimate::default(),
        });
        TrdispatchMcu { td: weak, id }
    }
}

impl TrdispatchMcu {
    /// trdispatch_mcu_free(): unlink from the trdispatch
    pub fn free(self) {
        let Some(td) = self.td.upgrade() else { return };
        let mut st = td.lock().unwrap();
        let was_active = st.is_active;
        let Some(pos) = st.tdm_list.iter().position(|m| m.id == self.id) else { return };
        let tdm = st.tdm_list.remove(pos);
        drop(st);

        // If trdispatch_start() registered the fastreader then
        // trdispatch_stop() will no longer see this node - deregister it here
        if was_active {
            tdm.sq.rm_fastreader(&tdm.fr);
        }
    }

    /// trdispatch_mcu_setup(): setup for a trigger test
    pub fn setup(&self, last_status_clock: u64, expire_clock: u64, expire_ticks: u64, min_extend_ticks: u64) {
        let Some(td) = self.td.upgrade() else { return };
        let mut st = td.lock().unwrap();
        let Some(tdm) = st.tdm_list.iter_mut().find(|m| m.id == self.id) else { return };
        tdm.last_status_clock = last_status_clock;
        tdm.expire_clock = expire_clock;
        tdm.expire_ticks = expire_ticks;
        tdm.min_extend_ticks = min_extend_ticks;
        tdm.ce = tdm.sq.get_clock_est();
    }
}
