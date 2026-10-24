// INDX induction coil driver and its auto-tuner (coil_driver.h,
// coil_driver.c++)
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::atsamd::clock::route_pclock;
use crate::atsamd::internal::*;
use crate::board::{gpio_adc_setup, timer_from_us, timer_is_before, timer_read_time};
use crate::config::CONFIG_CLOCK_FREQ;
use crate::ctr::Global;
use crate::generic::armcm_irq::armcm_irq_shutdown;
use crate::generic::cmsis::{NVIC_DisableIRQ, NVIC_EnableIRQ, NVIC_SetPriority};
use crate::sched::{sched_shutdown, R};
use crate::{decl_constant_str, decl_ctr, output, sendf, shutdown, static_string};
use core::sync::atomic::{AtomicU32, Ordering};

fn drive_tcc() -> &'static Tcc {
    unsafe { &*TCC3 }
}
const DRIVE_TCC_PCLK_ID: u32 = TCC3_GCLK_ID;
const DRIVE_TCC_PM_ID: u32 = ID_TCC3;
// Channel for drive signal output
const DRIVE_PIN: u32 = GPIO('B', 17);
const DRIVE_PIN_FUNC: char = 'F';
const DRIVE_TCC_CC: usize = 1;
const DRIVE_TCC_SYNCBUSY_CC: u32 = TCC_SYNCBUSY_CC1;
const DRIVE_TCC_IRQN: i32 = TCC3_0_IRQn;
// Channel for ADC triggering during tuning
const DRIVE_TCC_TUNE_CC: usize = 0;
const DRIVE_TCC_TUNE_SYNCBUSY_CC: u32 = TCC_SYNCBUSY_CC0;
const DRIVE_TCC_TUNE_EVSYS_CHANNEL: u32 = 2;

// Length of burst modulation frame
const FRAME_CYCLES: u32 = 512;

// Overvoltage and tuning input is via resistive divider
const OV_DIVIDER_R1: f32 = 45000.0; // Drain to tap
const OV_DIVIDER_R2: f32 = 1500.0; // Tap to gnd
// The feedback is on both pins PA06(for AC) and PB05(for ADC)
decl_constant_str!("RESERVE_PINS_INDX_FEEDBACK_SENSE", "PA06,PB05");

// Feedback system
const FEEDBACK_AC_PIN: u32 = GPIO('A', 6);
const FEEDBACK_AC_PIN_FUNC: char = 'B';
const FEEDBACK_ADC_PIN: u32 = GPIO('B', 5);
const FEEDBACK_ADC_PIN_FUNC: char = 'B';

const OV_EVSYS_CHANNEL: u32 = 1;
const OV_VDDANA: f32 = 3.3; // AC reference voltage
const OV_TRIP_VOLTAGE: f32 = 100.0; // overvoltage threshold
const TUNE_TARGET_PEAK_V: f32 = 96.0; // target drive peak voltage
//
const OV_SCALER_VALUE: u32 =
    (64.0f32 * OV_TRIP_VOLTAGE * OV_DIVIDER_R2 / ((OV_DIVIDER_R1 + OV_DIVIDER_R2) * OV_VDDANA) + 0.5) as u32 - 1;

const _: () = assert!(OV_SCALER_VALUE <= 63, "overvoltage scaler value out of range");

const TUNE_TARGET_SCALER_VALUE: u32 =
    (64.0f32 * TUNE_TARGET_PEAK_V * OV_DIVIDER_R2 / ((OV_DIVIDER_R1 + OV_DIVIDER_R2) * OV_VDDANA) + 0.5) as u32 - 1;
const _: () = assert!(TUNE_TARGET_SCALER_VALUE <= 63, "tune target scaler value out of range");
const _: () = assert!(TUNE_TARGET_SCALER_VALUE < OV_SCALER_VALUE, "Tuner target must be below overvoltage target");

// Feedback ADC for current sensing and sampling waveform during coil tuning
fn feedback_adc() -> &'static Adc {
    unsafe { &*ADC1 }
}
const FEEDBACK_ADC_0_IRQN: i32 = ADC1_0_IRQn;
const FEEDBACK_ADC_1_IRQN: i32 = ADC1_1_IRQn;
const FEEDBACK_ADC_GCLK_ID: u32 = ADC1_GCLK_ID;
const FEEDBACK_ADC_PM_ID: u32 = ID_ADC1;
const DRAIN_SENSE_ADC_MUXPOS: u32 = 7;

// Current sense system definitions
const CURRENT_SENSE_PIN: u32 = GPIO('B', 6);
const CURRENT_SENSE_PIN_FUNC: char = 'B';
const CURRENT_SENSE_ADC_MUXPOS: u32 = 8;
const CURRENT_SENSE_RATE: u32 = 32000;
decl_constant_str!("RESERVE_PINS_INDX_CURRENT_SENSE", "PB06");
// 3.3V / 4095 / (50 * 0.02 ohm)
decl_constant_str!("INDX_CURRENT_SENSE_AMP_PER_COUNT", "0.000805860806");
decl_ctr!("DECL_CONSTANT INDX_CURRENT_SENSE_RATE 32000");

// Current sense pacing
fn current_sense_trigger_tc() -> &'static TcCount16 {
    unsafe { &*TC2 }.COUNT16()
}
const CURRENT_SENSE_TRIGGER_TC_GCLK_ID: u32 = TC2_GCLK_ID;
const CURRENT_SENSE_TRIGGER_TC_PM_ID: u32 = ID_TC2;
const ADC_TRIGGER_EVSYS_CHANNEL: u32 = 0;
const ADC_TRIGGER_PERIOD: u32 = CONFIG_CLOCK_FREQ / CURRENT_SENSE_RATE;

const DRAIN_DIVIDER_GAIN: f32 = OV_DIVIDER_R2 / (OV_DIVIDER_R1 + OV_DIVIDER_R2);
const ADC_COUNT_MAX: f32 = 4095.0; // 12-bit full scale
const DRAIN_V_PER_COUNT: f32 = OV_VDDANA / ADC_COUNT_MAX / DRAIN_DIVIDER_GAIN;

// ON-time search bounds (clock ticks).
const TUNE_ON_FIRST_SEED: u32 = 180; // safe starting floor
const TUNE_ON_STEP: u32 = 2; // ramp granularity
const TUNE_ON_MAX: u32 = 1080;

// Provisional OFF used while ramping ON: long enough to contain a full positive
// resonance halfcycle even at the lowest resonant frequency.
const TUNE_OFF_PROV: u32 = 840;

// OFF cycles fired after the last measured cycle so its resonance completes
// before the timer stops.
const TUNE_TRAILING_OFF: u32 = 1;

// Quiet time between bursts, long enough for the tank to fully stop resonating.
const TUNE_SETTLE_US: u32 = 350;

// Whole-tune watchdog. A full sweep is typically well under ~2 s, so this is
// just a timeout to ensure that session doesn't hang in case of hardware faults
// etc.
const TUNE_TOTAL_TIMEOUT_US: u32 = 10000000; // 10 s

// Waveform sweep parameters for OFF time tuning. Must be long enough to
// encompass the full on+off cycle to ensure we can capture the full positive
// resonance halfcycle.
const TUNE_SAMPLE_STEP: u32 = 8;
const TUNE_N_SAMPLES: usize = ((TUNE_ON_MAX + TUNE_OFF_PROV) / TUNE_SAMPLE_STEP) as usize;

// Margin when finding the resonance cycle in the sampled data.
const TUNE_ZERO_MARGIN_V: f32 = 1.0;

// Waveform-dump pacing interval, to reduce risk of lost messages.
const TUNE_DUMP_INTERVAL_US: u32 = 3000;

// Runtime state shared between task context and the TCC3/AC/ADC ISRs.
// (Reg: the C volatile members)
struct DriverState {
    // Compare value (ticks) for the steady ON portion.
    on_ticks: Reg<u32>,
    // Compare value for the first cycle ON portion in a burst.
    on_ticks_first: Reg<u32>,
    // Period-1 of a normal cycle, = on_ticks + off - 1.
    period_minus_1: Reg<u32>,
    // Period-1 of the first cycle in a burst, = on_ticks_first + off - 1
    first_period_minus_1: Reg<u32>,
    // ON cycles for the burst being scheduled (latched at the boundary).
    on_cycles: Reg<u32>,
    // ON cycles requested by set_duty(), latched into on_cycles each burst.
    pending_on_cycles: Reg<u32>,
    // Index (0..FRAME_CYCLES-1) of the next cycle the ISR will program.
    cycle_idx: Reg<u32>,
    // Whether the preceding cycle is ON. If not, we should use the first cycle
    // length instead.
    prev_was_on: Reg<bool>,
    // Cumulative overvoltage trips, increased by the AC comparator ISR.
    ov_count: AtomicU32,
    // Whether the drive timings above are valid to fire (set explicitly via
    // set_timings or by a successful tune).
    timings_valid: bool,
}

static STATE: Global<DriverState> = Global::new(DriverState {
    on_ticks: Reg::new(0),
    on_ticks_first: Reg::new(0),
    period_minus_1: Reg::new(0),
    first_period_minus_1: Reg::new(0),
    on_cycles: Reg::new(0),
    pending_on_cycles: Reg::new(0),
    cycle_idx: Reg::new(0),
    prev_was_on: Reg::new(false),
    ov_count: AtomicU32::new(0),
    timings_valid: false,
});
fn state() -> &'static mut DriverState {
    STATE.get()
}

impl DriverState {
    // Cumulative overvoltage trips since boot
    fn overvoltage_count(&self) -> u32 {
        self.ov_count.load(Ordering::Relaxed)
    }

    fn record_overvoltage(&self) {
        // AC COMP0 overvoltage trip. The EVSYS channel already faulted the
        // timer, so we just record the event here.
        self.ov_count.fetch_add(1, Ordering::Relaxed);
    }
}

struct CurrentSenseState {
    tally: AtomicU32, // raw ADC counts
}

impl CurrentSenseState {
    fn on_sample(&self, result: u16) {
        self.tally.fetch_add(result as u32, Ordering::Relaxed);
    }

    fn total(&self) -> u32 {
        self.tally.load(Ordering::Relaxed)
    }
}

static CURRENT_SENSE: CurrentSenseState = CurrentSenseState { tally: AtomicU32::new(0) };

#[derive(Clone, Copy, PartialEq, Eq)]
enum TunePhase {
    Idle,
    OnFirst,     // Phase 1: ramp first time_on until cycle 0's peak crosses target
    OnFirstDone, // start Phase 2
    OffMeasure,  // Phase 2: equivalent-time sweep of the positive resonance
    // half cycle
    OffAnalyze, // find time_off
    OffDump,    // if requested, transfer the reconstructed waveform
    // (rate-limited)
    OffDone,    // start Phase 3
    SteadyOn,   // Phase 3: ramp time_on until cycle 1's peak crosses target
    SteadyDone, // report all three timings
    Error,
}

#[derive(Clone, Copy)]
enum TuneError {
    None = 0,
    Overvoltage = 1, // Overvoltage triggered during the tune (target removed?)
    InitialOnCeiling = 2, // ON-time tuning hit maximum time without reaching the target peak
    OffNoZero = 3, // the resonance never returned to zero within the sweep window
    SteadyCeiling = 4, // steady ON hit the ceiling without reaching the target
    Timeout = 5, // timed out
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum TuneStatus {
    Idle = 0,    // no tune has run since boot
    Running = 1, // tuning in progress
    Success = 2, // completed. on_first_ticks/off_ticks/on_ticks hold result
    Failed = 3,  // aborted. `error` holds the reason
}

enum BurstStatus {
    Busy,
    Completed,
    Ready,
}

struct TunerState {
    want_details: bool,

    phase: TunePhase,
    error: TuneError,
    // Latched across the return to idle so the host can poll the outcome.
    status: TuneStatus,
    burst_in_flight: bool,
    // Don't fire/act before this time
    next_fire_time: u32,
    // Snapshot of the overvoltage count at start; any change aborts the tune.
    ov_count_at_start: u32,
    // Whole-tune watchdog deadline; passing it aborts the tune.
    deadline: u32,
    on_first_ticks: u32,        // Phase 1 candidate, then the kept result
    waveform_sample_count: u32, // Desired number of waveform samples
    waveform_sample_idx: u32,   // OFF-sweep / dump index
    off_ticks: u32,             // result (ticks)
    on_ticks: u32,              // Phase 3 candidate, then the kept result
    #[allow(dead_code)] // (C: never read)
    found_steady: u32, // result (ticks)

    // --- shared with the TCC3/AC/ADC ISRs ---
    // A bounded test burst is in flight; the overflow ISR runs its own
    // scheduling instead of the normal frame/duty path.
    test_active: Reg<bool>,
    // Cycle boundaries (overflows) seen since the burst was armed.
    cycles_done: Reg<u32>,
    // Stop the burst once this many cycles have started.
    stop_after: Reg<u32>,
    // Which fired cycle's peak to keep: cycle 0 (soft-start) is pre-cleared by
    // the caller; for a later cycle (1 = first steady) the ISR clears the COMP1
    // latch as that cycle begins, so only its resonance can set it.
    measure_cycle: Reg<u32>,
    // Set by the ISR when the burst has stopped; cleared by the task.
    burst_done: Reg<bool>,
    // Set if ADC1 is remapped to feedback and needs to be mapped back to
    // current sensing later.
    adc_active: Reg<bool>,
    // Waiting for ADC to capture a sample?
    adc_capture_armed: Reg<bool>,
    // Captured ADC sample.
    adc_sample: Reg<u16>,
}

impl TunerState {
    // (C: tuner_state{})
    const fn new() -> TunerState {
        TunerState {
            want_details: false,
            phase: TunePhase::Idle,
            error: TuneError::None,
            status: TuneStatus::Idle,
            burst_in_flight: false,
            next_fire_time: 0,
            ov_count_at_start: 0,
            deadline: 0,
            on_first_ticks: 0,
            waveform_sample_count: 0,
            waveform_sample_idx: 0,
            off_ticks: 0,
            on_ticks: 0,
            found_steady: 0,
            test_active: Reg::new(false),
            cycles_done: Reg::new(0),
            stop_after: Reg::new(0),
            measure_cycle: Reg::new(0),
            burst_done: Reg::new(false),
            adc_active: Reg::new(false),
            adc_capture_armed: Reg::new(false),
            adc_sample: Reg::new(0),
        }
    }

    fn active(&self) -> bool {
        self.phase != TunePhase::Idle
    }
}

static TUNER: Global<TunerState> = Global::new(TunerState::new());
fn tuner() -> &'static mut TunerState {
    TUNER.get()
}

// IRQ declarations and assert correct indices.

const _: () = assert!(FEEDBACK_ADC_1_IRQN == 121);
decl_ctr!("DECL_ARMCM_IRQ FEEDBACK_ADC_1_Handler 121");
const _: () = assert!(FEEDBACK_ADC_0_IRQN == 120);
decl_ctr!("DECL_ARMCM_IRQ FEEDBACK_ADC_0_Handler 120");

const _: () = assert!(DRIVE_TCC_IRQN == 101);
decl_ctr!("DECL_ARMCM_IRQ TCC3_0_Handler 101");
const _: () = assert!(AC_IRQn == 122);
decl_ctr!("DECL_ARMCM_IRQ AC_Handler 122");

fn tcc_ctrlb_sync() {
    while drive_tcc().SYNCBUSY.reg.read() & TCC_SYNCBUSY_CTRLB != 0 {}
}

impl DriverState {
    // Start a drive burst from an idle tank state. Sets up first two cycles and
    // starts the timer. Future cycles are set up by the timer ISR later on.
    // Only called if the timer is idle, clears any latched overvoltage fault.
    fn arm_timer(&self) {
        let tcc = drive_tcc();
        let on_cycles = self.pending_on_cycles.read();
        self.on_cycles.write(on_cycles);
        if on_cycles == 0 {
            // Nothing to fire: leave the timer stopped, output low.
            return;
        }

        NVIC_DisableIRQ(DRIVE_TCC_IRQN);

        // A latched fault (STATUS.FAULT0) leaves the counter halted and WO1 low.
        // STOP then clear FAULT0. Clearing it while stopped keeps it stopped.
        if tcc.STATUS.reg.read() & TCC_STATUS_FAULT0 != 0 {
            tcc.CTRLBSET.reg.write(TCC_CTRLBSET_CMD_STOP as u8);
            tcc_ctrlb_sync();
            tcc.STATUS.reg.write(TCC_STATUS_FAULT0);
        }

        // Flush any PERBUF/CCBUF the ISR staged before the timer stopped (at a
        // frame boundary or a fault); it was never transferred, so PERBUFV/CCBUFV
        // stay set. A direct PER/CC write below would then never finish syncing (a
        // stopped counter has no UPDATE event to consume the buffer) and stall into
        // a watchdog reset. CMD_UPDATE clears them. No-op at first arm.
        tcc.CTRLBSET.reg.write(TCC_CTRLBSET_CMD_UPDATE as u8);
        tcc_ctrlb_sync();

        // Cycle 0 (inital ON ON + shortened period) goes in PER/CC. Cycle 1
        // (steady ON, or 0 if the burst is a single cycle) goes in
        // PERBUF/CCBUF, copied to PER/CC at the first overflow.
        let on1 = on_cycles > 1;
        let cc1 = if on1 { self.on_ticks.read() } else { 0 };
        tcc.PER.reg.write(TCC_PER_PER(self.first_period_minus_1.read()));
        tcc.CC[DRIVE_TCC_CC].reg.write(TCC_CC_CC(self.on_ticks_first.read()));
        while tcc.SYNCBUSY.reg.read() & (TCC_SYNCBUSY_PER | DRIVE_TCC_SYNCBUSY_CC) != 0 {}
        tcc.PERBUF.reg.write(TCC_PERBUF_PERBUF(self.period_minus_1.read()));
        tcc.CCBUF[DRIVE_TCC_CC].reg.write(TCC_CCBUF_CCBUF(cc1));

        self.cycle_idx.write(2);
        self.prev_was_on.write(on1);

        // Unset stale overflow flag so the first ISR aligns with cycle 2.
        tcc.INTFLAG.reg.write(TCC_INTFLAG_OVF);

        // Zero COUNT before retriggering. A retrigger of a STOPPED counter resumes
        // from current COUNT. After a fault COUNT is frozen mid-cycle, so cycle 0
        // would otherwise start partway through.
        tcc.COUNT.reg.write(TCC_COUNT_COUNT(0));
        while tcc.SYNCBUSY.reg.read() & TCC_SYNCBUSY_COUNT != 0 {}

        tcc.CTRLBSET.reg.write(TCC_CTRLBSET_CMD_RETRIGGER as u8);
        tcc_ctrlb_sync();

        NVIC_EnableIRQ(DRIVE_TCC_IRQN);
    }
}

pub struct CoilDriver {
    // Maximum number of fired cycles per 512-cycle frame (0 = no limit).
    pub duty_limit: u32,
}

impl CoilDriver {
    pub fn set_duty(&mut self, duty: f32) -> R {
        if duty != 0.0 && self.tune_active() {
            shutdown!("INDX heater target set while coil tuning");
        }
        // The coil timings have no default. Driving without first setting them
        // (explicitly or via a successful tune) is a fault.
        if duty != 0.0 && !state().timings_valid {
            shutdown!("INDX heater started before coil timings were set");
        }

        let mut cycles = if duty <= 0.0 {
            0
        } else if duty >= 1.0 {
            FRAME_CYCLES
        } else {
            (duty * FRAME_CYCLES as f32) as u32
        };
        if self.duty_limit != 0 && cycles > self.duty_limit {
            cycles = self.duty_limit;
        }
        state().pending_on_cycles.write(cycles);

        // Rearm the timer if it's not currently active.
        let status = &drive_tcc().STATUS.reg;
        let idle = status.read() & TCC_STATUS_STOP != 0 || status.read() & TCC_STATUS_FAULT0 != 0;
        if cycles > 0 && idle {
            state().arm_timer();
        }
        Ok(())
    }

    pub fn set_cycle_limit(&mut self, limit: u32) {
        self.duty_limit = limit;
    }

    // Cumulative number of overvoltage trips since boot.
    pub fn get_ov_count(&self) -> u32 {
        state().overvoltage_count()
    }

    pub fn shutdown_(&self) {
        // Disconnect timer from drive pin. The external gate pulldown holds the
        // MOSFET off.
        gpio_peripheral(DRIVE_PIN, '\0', 0);
        drive_tcc().CTRLA.reg.and(!TCC_CTRLA_ENABLE);
        NVIC_DisableIRQ(DRIVE_TCC_IRQN);

        // Disable the overvoltage comparator.
        unsafe { &*AC }.CTRLA.reg.write(0);
        NVIC_DisableIRQ(AC_IRQn);

        // Stop ADC start events first so no further conversions begin.
        current_sense_trigger_tc().CTRLA.reg.and(!TC_CTRLA_ENABLE);
        NVIC_DisableIRQ(FEEDBACK_ADC_1_IRQN);
        NVIC_DisableIRQ(FEEDBACK_ADC_0_IRQN);
    }

    pub fn get_total_charge(&self) -> u32 {
        CURRENT_SENSE.total()
    }
}

impl DriverState {
    // Per-cycle scheduler. Runs at each overflow and stages the next
    // cycle's CCBUF/PERBUF, which are the timings for the cycle that will begin
    // _after_ the one that just started.
    fn schedule_next_cycle(&self) {
        let tcc = drive_tcc();
        let mut idx = self.cycle_idx.read();

        // Shape the next cycle. Default = OFF/skipped (CC=0, full period). The
        // burst's first fired cycle (prev OFF) is soft-start (shorter ON + period,
        // preserving OFF/ZVS time); other fired cycles are steady ON.
        let on = idx < self.on_cycles.read();
        let initial = on && !self.prev_was_on.read();
        let mut cc = 0;
        let mut per = self.period_minus_1.read();
        if initial {
            cc = self.on_ticks_first.read();
            per = self.first_period_minus_1.read();
        } else if on {
            cc = self.on_ticks.read();
        }
        tcc.CCBUF[DRIVE_TCC_CC].reg.write(TCC_CCBUF_CCBUF(cc));
        tcc.PERBUF.reg.write(TCC_PERBUF_PERBUF(per));
        self.prev_was_on.write(on);

        // Check if this is the final cycle of the burst. If so, advance to the next
        // burst.
        idx += 1;
        if idx >= FRAME_CYCLES {
            idx = 0;
            let on_cycles = self.pending_on_cycles.read();
            self.on_cycles.write(on_cycles);
            if on_cycles == 0 {
                tcc.CTRLBSET.reg.write(TCC_CTRLBSET_CMD_STOP as u8);
            }
        }
        self.cycle_idx.write(idx);
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn TCC3_0_Handler() {
    drive_tcc().INTFLAG.reg.write(TCC_INTFLAG_OVF);
    if tuner().on_drive_overflow() {
        return;
    }
    state().schedule_next_cycle();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn AC_Handler() {
    unsafe { &*AC }.INTFLAG.reg.write(AC_INTFLAG_COMP0 as u8);
    state().record_overvoltage();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn FEEDBACK_ADC_1_Handler() {
    let result = feedback_adc().RESULT.reg.read();
    if tuner().capture_adc_sample(result) {
        return;
    }
    CURRENT_SENSE.on_sample(result);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn FEEDBACK_ADC_0_Handler() {
    armcm_irq_shutdown(sched_shutdown(static_string!("coil driver feedback ADC overrun")));
}

// Enables clock, running at core frequency instead of slower peripheral clkgen.
fn enable_pclock_max(pclk_id: u32, pm_id: i32) {
    route_pclock(pclk_id, 0);
    let (pm_port, pm_bit) = (pm_id as u32 / 32, 1u32 << (pm_id as u32 % 32));
    let r = unsafe { &*(*MCLK).APBAMASK.reg.ptr().add(pm_port as usize).cast::<Reg<u32>>() };
    r.or(pm_bit);
}

fn setup_adc1() -> R {
    let adc = feedback_adc();
    // Set up the ADC temperature pin, which ensures `adc_init` gets called. We
    // let this load the calibration data etc for us.
    gpio_adc_setup(0xfe)?;

    gpio_peripheral(CURRENT_SENSE_PIN, CURRENT_SENSE_PIN_FUNC, 0);
    gpio_peripheral(FEEDBACK_ADC_PIN, FEEDBACK_ADC_PIN_FUNC, 0);

    enable_pclock(FEEDBACK_ADC_GCLK_ID, FEEDBACK_ADC_PM_ID as i32);

    // Read calibration data so we have it after we reset the ADC
    let calib = adc.CALIB.reg.read();

    adc.CTRLA.reg.or(ADC_CTRLA_SWRST as u16);
    while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_SWRST != 0 {}

    // Reload calibration
    adc.CALIB.reg.write(calib);

    adc.REFCTRL.reg.write(ADC_REFCTRL_REFSEL_INTVCC1 as u8);
    while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_REFCTRL != 0 {}

    adc.SAMPCTRL.reg.write(ADC_SAMPCTRL_SAMPLEN(4) as u8);
    while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_SAMPCTRL != 0 {}

    // Start out with current sense as our ADC channel
    adc.INPUTCTRL.reg.write((ADC_INPUTCTRL_MUXPOS(CURRENT_SENSE_ADC_MUXPOS) | ADC_INPUTCTRL_MUXNEG(0x18)) as u16);
    while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_INPUTCTRL != 0 {}

    // ADC runs event triggered. CURRENT_SENSE_TRIGGER_TC paces current sense
    // measurements via EVSYS. During tuning, DRIVE_TCC takes over.
    adc.CTRLB.reg.write(ADC_CTRLB_RESSEL_12BIT as u16);
    while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_CTRLB != 0 {}
    adc.EVCTRL.reg.write(ADC_EVCTRL_STARTEI as u8);

    adc.INTENSET.reg.write((ADC_INTENSET_RESRDY | ADC_INTENSET_OVERRUN) as u8);

    NVIC_SetPriority(FEEDBACK_ADC_1_IRQN, 1);
    NVIC_SetPriority(FEEDBACK_ADC_0_IRQN, 1);
    NVIC_EnableIRQ(FEEDBACK_ADC_1_IRQN);
    NVIC_EnableIRQ(FEEDBACK_ADC_0_IRQN);

    // 48 MHz / 4 = 12 MHz ADC clock, spec calls for < 16MHz.
    adc.CTRLA.reg.write((ADC_CTRLA_PRESCALER(ADC_CTRLA_PRESCALER_DIV4_Val) | ADC_CTRLA_ENABLE) as u16);
    while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_ENABLE != 0 {}
    Ok(())
}

fn setup_adc_trigger_tc() {
    let tc = current_sense_trigger_tc();
    enable_pclock_max(CURRENT_SENSE_TRIGGER_TC_GCLK_ID, CURRENT_SENSE_TRIGGER_TC_PM_ID as i32);

    tc.CTRLA.reg.or(TC_CTRLA_SWRST);
    while tc.SYNCBUSY.reg.read() & TC_SYNCBUSY_SWRST != 0 {}

    tc.WAVE.reg.write(TC_WAVE_WAVEGEN_MFRQ as u8);
    tc.CC[0].reg.write((ADC_TRIGGER_PERIOD - 1) as u16);
    tc.EVCTRL.reg.write(TC_EVCTRL_OVFEO as u16);
    tc.CTRLA.reg.write(TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE);
    while tc.SYNCBUSY.reg.read() & TC_SYNCBUSY_ENABLE != 0 {}
}

fn setup_evsys() {
    let evsys = unsafe { &*EVSYS };
    // We need EVSYS for signal routing so bring it up now.
    unsafe { &*MCLK }.APBBMASK.reg.or(MCLK_APBBMASK_EVSYS);

    // Event channel for triggering ADC for tuning and current sensing
    evsys.Channel[DRIVE_TCC_TUNE_EVSYS_CHANNEL as usize]
        .CHANNEL
        .reg
        .write(EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC3_MC_0) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS);

    evsys.Channel[ADC_TRIGGER_EVSYS_CHANNEL as usize]
        .CHANNEL
        .reg
        .write(EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TC2_OVF) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS);
    evsys.USER[EVSYS_ID_USER_ADC1_START as usize].reg.write(ADC_TRIGGER_EVSYS_CHANNEL + 1);
}

fn setup_drive_tcc() {
    let tcc = drive_tcc();
    enable_pclock_max(DRIVE_TCC_PCLK_ID, DRIVE_TCC_PM_ID as i32);

    // Set up the drive timer.
    tcc.CTRLA.reg.or(TCC_CTRLA_SWRST);
    while tcc.SYNCBUSY.reg.read() & TCC_SYNCBUSY_SWRST != 0 {}

    tcc.CTRLA.reg.write(TCC_CTRLA_PRESCALER_DIV1);
    tcc.WAVE.reg.write(TCC_WAVE_WAVEGEN_NPWM);
    // On a non-recoverable fault, force WO1 (PB17) low so the MOSFET turns off.
    tcc.DRVCTRL.reg.write(TCC_DRVCTRL_NRE(1 << DRIVE_TCC_CC));
    // Overvoltage triggeres a non-recoverable fault on the drive timer via
    // EVSYS on event 0. During tuning, CC0 match outputs an event to trigger
    // ADC sampling.
    tcc.EVCTRL.reg.write(TCC_EVCTRL_TCEI0 | TCC_EVCTRL_EVACT0_FAULT | TCC_EVCTRL_MCEO0);

    tcc.CTRLA.reg.or(TCC_CTRLA_ENABLE);
    while tcc.SYNCBUSY.reg.read() & TCC_SYNCBUSY_ENABLE != 0 {}
    tcc.CTRLBSET.reg.write(TCC_CTRLBSET_CMD_STOP as u8);
    tcc_ctrlb_sync();

    tcc.INTENSET.reg.write(TCC_INTENSET_OVF);
    NVIC_SetPriority(DRIVE_TCC_IRQN, 1);
    NVIC_EnableIRQ(DRIVE_TCC_IRQN);

    gpio_peripheral(DRIVE_PIN, DRIVE_PIN_FUNC, 0);
}

fn setup_ac() {
    let ac = unsafe { &*AC };
    gpio_peripheral(FEEDBACK_AC_PIN, FEEDBACK_AC_PIN_FUNC, 0);

    enable_pclock(AC_GCLK_ID, ID_AC as i32);

    ac.CTRLA.reg.or(AC_CTRLA_SWRST as u8);
    while ac.SYNCBUSY.reg.read() & AC_SYNCBUSY_SWRST != 0 {}

    // COMP0: overvoltage detector, interrupt triggers on rising edge which
    // indicates over voltage.
    ac.SCALER[0].reg.write(AC_SCALER_VALUE(OV_SCALER_VALUE) as u8);
    ac.COMPCTRL[0].reg.write(
        AC_COMPCTRL_MUXPOS_PIN2
            | AC_COMPCTRL_MUXNEG_VSCALE
            | AC_COMPCTRL_SPEED_HIGH
            | AC_COMPCTRL_INTSEL_RISING
            | AC_COMPCTRL_HYSTEN
            | AC_COMPCTRL_HYST_HYST50
            | AC_COMPCTRL_ENABLE,
    );
    while ac.SYNCBUSY.reg.read() & AC_SYNCBUSY_COMPCTRL0 != 0 {}

    // COMP1: tuner peak detector. Latches rising edge, not used in normal
    // driving. No hysteresis as that would bias the detect peak high.
    ac.SCALER[1].reg.write(AC_SCALER_VALUE(TUNE_TARGET_SCALER_VALUE) as u8);
    ac.COMPCTRL[1].reg.write(
        AC_COMPCTRL_MUXPOS_PIN2 | AC_COMPCTRL_MUXNEG_VSCALE | AC_COMPCTRL_SPEED_HIGH | AC_COMPCTRL_INTSEL_RISING | AC_COMPCTRL_ENABLE,
    );
    while ac.SYNCBUSY.reg.read() & AC_SYNCBUSY_COMPCTRL1 != 0 {}

    ac.EVCTRL.reg.write(AC_EVCTRL_COMPEO0 as u16); // COMP0 state sent to event output
    ac.INTENSET.reg.write(AC_INTENSET_COMP0 as u8); // COMP0 rising edge triggers AC_Handler

    NVIC_SetPriority(AC_IRQn, 2);
    NVIC_EnableIRQ(AC_IRQn);

    ac.CTRLA.reg.write(AC_CTRLA_ENABLE as u8);
    while ac.SYNCBUSY.reg.read() & AC_SYNCBUSY_ENABLE != 0 {}

    // COMP0 is routed to event on DRIVE_TCC/TCC3 so next cycle can be
    // prevented.
    let evsys = unsafe { &*EVSYS };
    evsys.Channel[OV_EVSYS_CHANNEL as usize]
        .CHANNEL
        .reg
        .write(EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_AC_COMP_0) | EVSYS_CHANNEL_PATH_ASYNCHRONOUS);
    evsys.USER[EVSYS_ID_USER_TCC3_EV_0 as usize].reg.write(OV_EVSYS_CHANNEL + 1);
}

impl CoilDriver {
    // time_on_first is the (shorter) ON time used for the first fired cycle of
    // every burst
    pub fn set_timings(&mut self, time_on: f32, time_off: f32, time_on_first: f32) {
        let osc_clk_freq = CONFIG_CLOCK_FREQ as f32;

        let on_ticks = (osc_clk_freq * time_on) as u32;
        let off_ticks = (osc_clk_freq * time_off) as u32;
        let on_ticks_first = (osc_clk_freq * time_on_first) as u32;
        state().set_drive_ticks(on_ticks, on_ticks_first, off_ticks);
    }
}

pub fn coil_driver_create() -> R<CoilDriver> {
    let driver = CoilDriver { duty_limit: 0 };

    setup_evsys();
    setup_drive_tcc();
    setup_ac();
    setup_adc1()?;
    setup_adc_trigger_tc();

    Ok(driver)
}

const fn volts_to_count(v: f32) -> u32 {
    let mut c = (v / DRAIN_V_PER_COUNT + 0.5) as i32;
    if c < 0 {
        c = 0;
    }
    if c > 4095 {
        c = 4095;
    }
    c as u32
}

fn ticks_to_ns(ticks: u32) -> u32 {
    (ticks as f32 * (1.0e9f32 / CONFIG_CLOCK_FREQ as f32) + 0.5) as u32
}

// Read and clear the latched COMP1 peak flag.
fn tune_peak_tripped() -> bool {
    let ac = unsafe { &*AC };
    if ac.INTFLAG.reg.read() as u32 & AC_INTFLAG_COMP1 == 0 {
        return false;
    }
    ac.INTFLAG.reg.write(AC_INTFLAG_COMP1 as u8);
    true
}

impl DriverState {
    // Set new drive timings in ticks. If the specified timings are not valid,
    // `timings_valid` is set to false and the currently set ticks are not touched.
    fn set_drive_ticks(&mut self, on_ticks: u32, on_ticks_first: u32, off_ticks: u32) {
        if on_ticks == 0 || off_ticks == 0 || on_ticks_first == 0 || on_ticks_first > on_ticks {
            self.timings_valid = false;
            return;
        }

        NVIC_DisableIRQ(DRIVE_TCC_IRQN);
        self.on_ticks.write(on_ticks);
        self.period_minus_1.write(on_ticks.wrapping_add(off_ticks).wrapping_sub(1));
        self.on_ticks_first.write(on_ticks_first);
        self.first_period_minus_1.write(on_ticks_first.wrapping_add(off_ticks).wrapping_sub(1));
        self.timings_valid = true;
        NVIC_EnableIRQ(DRIVE_TCC_IRQN);
    }
}

impl TunerState {
    // Fire a single test burst from idle timer. Can fire either 1 or 2 cycles.
    fn fire_burst(&mut self, n_cycles: u32, measure: u32) {
        self.cycles_done.write(0);
        self.stop_after.write(n_cycles + TUNE_TRAILING_OFF);
        self.measure_cycle.write(measure);
        self.burst_done.write(false);
        self.test_active.write(true);
        state().pending_on_cycles.write(n_cycles);
        self.burst_in_flight = true;
        state().arm_timer();
    }

    // Called from drive TCC handler. Returns true if tuner handled the event.
    fn on_drive_overflow(&self) -> bool {
        if !self.test_active.read() {
            return false;
        }
        let tcc = drive_tcc();

        // The cycle now beginning. Parameters were already loaded when the burst
        // started.
        let started = self.cycles_done.read() + 1;
        self.cycles_done.write(started);

        // If we reached cycle to be measured, reset tuning peak flag so it can be
        // latched if the peak is reached.
        if started == self.measure_cycle.read() {
            unsafe { &*AC }.INTFLAG.reg.write(AC_INTFLAG_COMP1 as u8);
        }

        // Next cycle may final cycle, so put in the extra off time time.
        tcc.CCBUF[DRIVE_TCC_CC].reg.write(TCC_CCBUF_CCBUF(0));
        tcc.PERBUF.reg.write(TCC_PERBUF_PERBUF(state().period_minus_1.read()));

        if started >= self.stop_after.read() {
            tcc.CTRLBSET.reg.write(TCC_CTRLBSET_CMD_STOP as u8);
            self.test_active.write(false);
            self.burst_done.write(true);
        }
        true
    }

    // Called from ADC handler. Returns true if the tuner handled the event.
    fn capture_adc_sample(&self, result: u16) -> bool {
        if !self.adc_active.read() {
            return false;
        }
        if self.adc_capture_armed.read() {
            self.adc_sample.write(result);
            self.adc_capture_armed.write(false);
        }
        true
    }
}

fn tune_set_cc0(value: u32) {
    let tcc = drive_tcc();
    tcc.CC[DRIVE_TCC_TUNE_CC].reg.write(TCC_CC_CC(value));
    while tcc.SYNCBUSY.reg.read() & DRIVE_TCC_TUNE_SYNCBUSY_CC != 0 {}
}

impl TunerState {
    // Remap ADC1 to read feedback voltage instead of current sensing
    fn adc_arm(&mut self) {
        let adc = feedback_adc();
        adc.INPUTCTRL.reg.write((ADC_INPUTCTRL_MUXPOS(DRAIN_SENSE_ADC_MUXPOS) | ADC_INPUTCTRL_MUXNEG(0x18)) as u16);
        while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_INPUTCTRL != 0 {}
        self.adc_active.write(true);
        unsafe { &*EVSYS }.USER[EVSYS_ID_USER_ADC1_START as usize].reg.write(DRIVE_TCC_TUNE_EVSYS_CHANNEL + 1);
    }

    // Restore ADC1 to read current sensor
    fn adc_restore(&mut self) {
        let adc = feedback_adc();
        adc.INPUTCTRL.reg.write((ADC_INPUTCTRL_MUXPOS(CURRENT_SENSE_ADC_MUXPOS) | ADC_INPUTCTRL_MUXNEG(0x18)) as u16);
        while adc.SYNCBUSY.reg.read() & ADC_SYNCBUSY_INPUTCTRL != 0 {}
        self.adc_active.write(false);
        unsafe { &*EVSYS }.USER[EVSYS_ID_USER_ADC1_START as usize].reg.write(ADC_TRIGGER_EVSYS_CHANNEL + 1);
    }
}

// Reconstructed resonance waveform.
// A tuning round will (over)write every sample before it's read.
static TUNE_WAVEFORM_SAMPLES: Global<[u16; TUNE_N_SAMPLES]> = Global::new([0; TUNE_N_SAMPLES]);

impl TunerState {
    // Check status of current burst
    fn burst_poll(&mut self) -> BurstStatus {
        if self.burst_in_flight {
            if !self.burst_done.read() {
                return BurstStatus::Busy;
            }
            self.burst_in_flight = false;
            self.next_fire_time = timer_read_time().wrapping_add(timer_from_us(TUNE_SETTLE_US));
            return BurstStatus::Completed;
        }
        if timer_is_before(timer_read_time(), self.next_fire_time) {
            return BurstStatus::Busy;
        }
        BurstStatus::Ready
    }

    fn start(&mut self, want_details: bool) {
        // start_tune() has already verified the coil is idle.
        *self = TunerState::new();
        self.want_details = want_details;
        self.phase = TunePhase::OnFirst;
        self.status = TuneStatus::Running;
        self.on_first_ticks = TUNE_ON_FIRST_SEED;
        self.next_fire_time = timer_read_time();
        self.ov_count_at_start = state().overvoltage_count();
        self.deadline = timer_read_time().wrapping_add(timer_from_us(TUNE_TOTAL_TIMEOUT_US));
        // The test bursts overwrite the published drive timings, so any previously
        // valid set is now gone; only a successful tune (or a fresh set_timings)
        // makes them valid again.
        state().timings_valid = false;
    }

    // Stop everything and enter the error phase.
    fn abort(&mut self, e: TuneError) {
        if self.adc_active.read() {
            self.adc_restore();
        }
        drive_tcc().CTRLBSET.reg.write(TCC_CTRLBSET_CMD_STOP as u8);
        self.test_active.write(false);
        self.burst_in_flight = false;
        // A failed tune has no valid result; clear the timings so a poll can't read
        // a stale/partial value (`error` carries the reason instead).
        self.on_first_ticks = 0;
        self.off_ticks = 0;
        self.error = e;
        self.phase = TunePhase::Error;
    }

    // Latch the outcome, notify the host, and return to idle.
    fn finish(&mut self, s: TuneStatus) -> R {
        self.status = s;
        // A successful tune leaves the found timings in the drive state,
        // so heating may use them
        if s == TuneStatus::Success {
            state().timings_valid = true;
        }
        sendf!("indx_coil_tune_finished status=%c error=%c", s as u32, self.error as u32)?;
        self.phase = TunePhase::Idle;
        Ok(())
    }
}

const fn counts_to_mv(counts: u32) -> u32 {
    (counts as f32 * DRAIN_V_PER_COUNT * 1000.0 + 0.5) as u32
}

impl TunerState {
    // Phase 1: from a de-energized tank, fire single-cycle bursts at increasing
    // time_on_first until the resonance peak first crosses the target.
    fn step_on_first(&mut self) {
        match self.burst_poll() {
            BurstStatus::Busy => {}
            BurstStatus::Completed => {
                if tune_peak_tripped() {
                    // reached the target peak
                    self.phase = TunePhase::OnFirstDone;
                    return;
                }
                self.on_first_ticks += TUNE_ON_STEP;
                if self.on_first_ticks > TUNE_ON_MAX {
                    self.abort(TuneError::InitialOnCeiling);
                }
            }
            BurstStatus::Ready => {
                // Single-cycle burst: cycle is the only fired cycle, so any peak is
                // unambiguously its own. Steady ON time is irrelevant here.
                state().set_drive_ticks(self.on_first_ticks, self.on_first_ticks, TUNE_OFF_PROV);
                tune_peak_tripped(); // clear target peak latch
                self.fire_burst(1, 0);
            }
        }
    }

    // Phase 2: with fixed initial cycle ON time, run consecutive bursts and do
    // equivalent-time sampling of the resonance waveform. To work around timing
    // issues around firing the sampler, we grab the entire waveform before the
    // actual OFF event as well. This way, when we go to find the actual OFF time we
    // can measure between the zero points instead of relying on the start of the
    // waveform being at exactly the OFF phase start.
    fn step_off_measure(&mut self) {
        match self.burst_poll() {
            BurstStatus::Busy => {}
            BurstStatus::Completed => {
                TUNE_WAVEFORM_SAMPLES.get()[self.waveform_sample_idx as usize] = self.adc_sample.read();
                self.waveform_sample_idx += 1;
                if self.waveform_sample_idx >= self.waveform_sample_count {
                    self.adc_restore();
                    self.phase = TunePhase::OffAnalyze;
                }
            }
            BurstStatus::Ready => {
                // Excite with the found time_on_first and step the CC0 match further
                // into cycle 0's OFF/resonance each burst.
                state().set_drive_ticks(self.on_first_ticks, self.on_first_ticks, TUNE_OFF_PROV);
                tune_set_cc0(self.waveform_sample_idx * TUNE_SAMPLE_STEP);
                self.adc_capture_armed.write(true);
                self.fire_burst(1, 0);
            }
        }
    }

    // Analyze captured waveform to find ideal off time.
    // The sampled waveform right now contains (most of) the ON phase, and the
    // entire OFF phase. We need to determine the total duration of the OFF phase.
    fn analyze_off(&mut self) -> R {
        let zero_threshold = volts_to_count(TUNE_ZERO_MARGIN_V);
        let samples = TUNE_WAVEFORM_SAMPLES.get();

        // Find the start and end of the OFF phase. Note that the signal rise/fall
        // is so fast that we don't really need to deal with hysteresis when finding
        // the edges.
        let mut phase_off_start: i16 = -1;
        let mut phase_off_end: i16 = -1;
        for i in 0..self.waveform_sample_count {
            let sample = samples[i as usize] as u32;
            if phase_off_start < 0 && sample >= zero_threshold {
                // ON->OFF transition
                phase_off_start = i as i16;
            } else if phase_off_start >= 0 && phase_off_end < 0 && sample <= zero_threshold {
                // OFF->ON transition
                phase_off_end = i as i16;
                break;
            }
        }

        // If we didn't find both start and end, we can't find the OFF time.
        if phase_off_start == -1 || phase_off_end == -1 {
            self.abort(TuneError::OffNoZero);
            return Ok(());
        }

        self.off_ticks = ((phase_off_end as i32 - phase_off_start as i32 + 1) as u32).wrapping_mul(TUNE_SAMPLE_STEP);

        // If details were requested, enter dump phase.
        self.next_fire_time =
            timer_read_time().wrapping_add(if self.want_details { 0 } else { timer_from_us(TUNE_SETTLE_US) });
        self.phase = if self.want_details { TunePhase::OffDump } else { TunePhase::OffDone };
        output!("C %u", self.want_details)?;
        self.waveform_sample_idx = 0;
        Ok(())
    }

    // Phase 3: find the steady-state ON time, that is the ON time for any phase
    // after the first one. Ramp the steady state ON time from the time found for
    // the initial ON time until the target is crossed. The starting point was set
    // by the `off_done` phase handler.
    fn step_steady_on(&mut self) {
        match self.burst_poll() {
            BurstStatus::Busy => {}
            BurstStatus::Completed => {
                if tune_peak_tripped() {
                    // target peak reached
                    self.phase = TunePhase::SteadyDone;
                    return;
                }
                self.on_ticks += TUNE_ON_STEP;
                if self.on_ticks > TUNE_ON_MAX {
                    self.abort(TuneError::SteadyCeiling);
                }
            }
            BurstStatus::Ready => {
                state().set_drive_ticks(self.on_ticks, self.on_first_ticks, self.off_ticks);
                tune_peak_tripped(); // clear target peak latch
                self.fire_burst(2, 1);
            }
        }
    }

    fn step(&mut self) -> R {
        // Guards shared for all tuning phases.
        if self.phase != TunePhase::Idle && self.phase != TunePhase::Error {
            if state().overvoltage_count() != self.ov_count_at_start {
                self.abort(TuneError::Overvoltage);
            } else if !timer_is_before(timer_read_time(), self.deadline) {
                self.abort(TuneError::Timeout);
            }
        }

        match self.phase {
            TunePhase::OnFirst => self.step_on_first(),

            TunePhase::OnFirstDone => {
                self.adc_arm(); // Remap ADC to feedback input for off sweep phase
                self.waveform_sample_idx = 0;
                self.waveform_sample_count = (self.on_first_ticks + TUNE_OFF_PROV) / TUNE_SAMPLE_STEP;
                self.phase = TunePhase::OffMeasure;
            }

            TunePhase::OffMeasure => self.step_off_measure(),

            TunePhase::OffAnalyze => self.analyze_off()?,

            TunePhase::OffDump => {
                // Stream the reconstructed waveform one sample at a time, so
                // it can be overlaid on the scope trace to validate the measurement.
                // Since this is diagnostic only, dropped samples are acceptable.
                if timer_is_before(timer_read_time(), self.next_fire_time) {
                    return Ok(());
                }
                if self.waveform_sample_idx < self.waveform_sample_count {
                    sendf!(
                        "indx_coil_tune_wave index=%u ns=%u mv=%u",
                        self.waveform_sample_idx,
                        ticks_to_ns(self.waveform_sample_idx * TUNE_SAMPLE_STEP),
                        counts_to_mv(TUNE_WAVEFORM_SAMPLES.get()[self.waveform_sample_idx as usize] as u32)
                    )?;
                    self.waveform_sample_idx += 1;
                    self.next_fire_time = timer_read_time().wrapping_add(timer_from_us(TUNE_DUMP_INTERVAL_US));
                } else {
                    self.next_fire_time = timer_read_time();
                    self.phase = TunePhase::OffDone;
                }
            }

            TunePhase::OffDone => {
                // Phase 3: steady state ON time is always >= the initial ON time, so
                // start from there. Next fire time was set by the previous phase.
                self.on_ticks = self.on_first_ticks;
                self.phase = TunePhase::SteadyOn;
            }

            TunePhase::SteadyOn => self.step_steady_on(),

            TunePhase::SteadyDone => self.finish(TuneStatus::Success)?,

            TunePhase::Error => self.finish(TuneStatus::Failed)?,

            TunePhase::Idle => {}
        }
        Ok(())
    }
}

impl CoilDriver {
    // Start driver tuning procedure, to find timing parameters.
    pub fn start_tune(&mut self, want_details: bool) -> R {
        if tuner().active() {
            return Ok(());
        }
        if state().pending_on_cycles.read() != 0 {
            shutdown!("INDX coil tune requested while heater active");
        }
        tuner().start(want_details);
        Ok(())
    }

    // True while a tuning session is in progress.
    pub fn tune_active(&self) -> bool {
        tuner().active()
    }

    // Advance the tuning state machine one step.
    pub fn tune_step(&mut self) -> R {
        tuner().step()
    }

    // Send the latched tuning outcome (status/error/found timings) to the host.
    pub fn report_tune_status(&self) -> R {
        let t = tuner();
        sendf!(
            "indx_coil_tune_result status=%c error=%c on_first=%u off=%u on=%u",
            t.status as u32,
            t.error as u32,
            ticks_to_ns(t.on_first_ticks),
            ticks_to_ns(t.off_ticks),
            ticks_to_ns(t.on_ticks)
        )
    }

    // Send the drive timings currently in effect to the host.
    pub fn report_params(&self) -> R {
        let s = state();
        let on = s.on_ticks.read();
        let off = s.period_minus_1.read().wrapping_add(1).wrapping_sub(on);
        sendf!(
            "indx_coil_driver_params time_on=%u time_off=%u time_on_first=%u",
            ticks_to_ns(on),
            ticks_to_ns(off),
            ticks_to_ns(s.on_ticks_first.read())
        )
    }
}
