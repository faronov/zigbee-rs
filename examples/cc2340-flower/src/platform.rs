//! Flower-board adapters for the shared sleepy-sensor lifecycle.
//!
//! Maps the board's raw LED/button/reset resources onto lifecycle
//! semantics, following the reference firmware's indications:
//!
//! - joining: green blink per network search;
//! - joined (fresh join or resumed boot): three green blinks;
//! - report, identify: green;
//! - factory reset (5-s hold) and leave: red blinks;
//! - fatal fault: red.
//!
//! [`StatusSink::set`] is synchronous, so multi-blink patterns are queued
//! there and played by [`FlowerWake`] at the start of the next idle wait,
//! when the radio is already asleep.

use core::cell::{Cell, RefCell};

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use flower_cc2340r5::{
    pins::{self, GreenLed, RedLed, UserButton},
    power,
    reset::SystemReset,
    standby::{self, StandbyRefusal},
};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, SensorStatus, SleepDepth, StatusSink, Supervisor, WaitRequest,
    WakeController, WakeReason,
};
use zigbee_mac::cc2340::{Cc2340Mac, RadioError};

/// A press edge counts only if SW1 is still down this long after it.
const PRESS_CONFIRM_MS: u64 = 30;
/// Level-sampling period while a press is held (long-press detection only).
const HELD_SAMPLE_MS: u32 = 25;
/// Standby counters are logged every this many retention waits.
const STANDBY_LOG_EVERY: u32 = 30;
const PATTERN_ON_MS: u64 = 150;
const PATTERN_OFF_MS: u64 = 150;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitError {
    Radio(RadioError),
}

impl From<RadioError> for WaitError {
    fn from(error: RadioError) -> Self {
        Self::Radio(error)
    }
}

// ── Indicators ──────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pattern {
    /// Three green blinks.
    Joined,
    /// Three red blinks.
    Left,
}

impl Pattern {
    const fn blinks(self) -> u8 {
        3
    }

    const fn red(self) -> bool {
        matches!(self, Pattern::Left)
    }
}

/// The two active-high status LEDs plus one queued pattern.
pub struct Indicators {
    green: GreenLed,
    red: RedLed,
    pending: Option<Pattern>,
}

impl Indicators {
    pub fn new(mut green: GreenLed, mut red: RedLed) -> Self {
        green.off();
        red.off();
        Self {
            green,
            red,
            pending: None,
        }
    }

    /// Fatal fault: red on, green off.
    pub fn fault(&mut self) {
        self.show(false, true);
    }

    fn show(&mut self, green: bool, red: bool) {
        self.green.set(green);
        self.red.set(red);
    }
}

/// Play `pattern` now. No borrow is held across an await.
pub async fn play(leds: &RefCell<Indicators>, pattern: Pattern) {
    for _ in 0..pattern.blinks() {
        leds.borrow_mut().show(!pattern.red(), pattern.red());
        Timer::after(Duration::from_millis(PATTERN_ON_MS)).await;
        leds.borrow_mut().show(false, false);
        Timer::after(Duration::from_millis(PATTERN_OFF_MS)).await;
    }
}

async fn play_pending(leds: &RefCell<Indicators>) {
    let pending = leds.borrow_mut().pending.take();
    if let Some(pattern) = pending {
        play(leds, pattern).await;
    }
}

/// Lifecycle status rendered on the shared [`Indicators`].
pub struct StatusLeds<'a> {
    leds: &'a RefCell<Indicators>,
    joined: bool,
}

impl<'a> StatusLeds<'a> {
    pub const fn new(leds: &'a RefCell<Indicators>) -> Self {
        Self {
            leds,
            joined: false,
        }
    }
}

impl StatusSink for StatusLeds<'_> {
    fn set(&mut self, status: SensorStatus) {
        let mut leds = self.leds.borrow_mut();
        let (green, red) = match status {
            SensorStatus::Off => (false, false),
            SensorStatus::Joining { on } => (on, false),
            // `Joined { active }` is a level (the post-join interview
            // window); a battery device shows the transition as a short
            // pattern instead of holding the LED on for minutes.
            SensorStatus::Joined { active } => {
                if active && !self.joined {
                    leds.pending = Some(Pattern::Joined);
                }
                (false, false)
            }
            SensorStatus::Identifying { on } | SensorStatus::Reporting { on } => (on, false),
            SensorStatus::Resetting { on } => (false, on),
            SensorStatus::Ota => (true, false),
            SensorStatus::Fault => (false, true),
        };
        self.joined = matches!(
            status,
            SensorStatus::Joined { .. }
                | SensorStatus::Identifying { .. }
                | SensorStatus::Reporting { .. }
        );
        leds.show(green, red);
    }
}

// ── Wake ────────────────────────────────────────────────────────────

/// SW1 (DIO20) invokes the product's user action. It is a falling-edge,
/// standby-wake-enabled interrupt; between events the executor sleeps in
/// WFE (fast polls) or device standby (slow polls).
pub struct FlowerWake<'a> {
    button: UserButton,
    leds: &'a RefCell<Indicators>,
    retention_waits: u32,
    lf_good_logged: bool,
    last_refusal: Option<StandbyRefusal>,
}

impl<'a> FlowerWake<'a> {
    pub fn new(mut button: UserButton, leds: &'a RefCell<Indicators>) -> Self {
        // Edges latched before the lifecycle starts are not wake requests.
        button.enable_press_interrupt();
        let _ = pins::take_button_edges();
        Self {
            button,
            leds,
            retention_waits: 0,
            lf_good_logged: false,
            last_refusal: None,
        }
    }

    /// Wait until `deadline` or a confirmed SW1 press. With `standby`, the
    /// gaps between events are spent in device standby; a refusal other
    /// than a pending interrupt falls back to the WFE idle wait. Returns the
    /// wake reason and whether standby was entered at all.
    async fn timer_or_button(&mut self, deadline: Instant, standby: bool) -> (WakeReason, bool) {
        // A press held across the previous action is not a new request.
        if self.button.is_pressed() {
            return (WakeReason::Button, false);
        }
        let mut try_standby = standby;
        let mut entered = false;
        loop {
            let edges = if try_standby {
                let edges = pins::take_button_edges();
                if edges != 0 {
                    edges
                } else if Instant::now() >= deadline {
                    return (WakeReason::Timer, entered);
                } else {
                    match standby::standby_until(deadline.as_ticks()) {
                        Ok(()) => entered = true,
                        Err(StandbyRefusal::WakePending) => {}
                        Err(refusal) => {
                            self.last_refusal = Some(refusal);
                            try_standby = false;
                        }
                    }
                    continue;
                }
            } else {
                match select(pins::wait_for_button_edge(), Timer::at(deadline)).await {
                    Either::First(edges) => edges,
                    Either::Second(()) => return (WakeReason::Timer, entered),
                }
            };
            if edges & self.button.edge_mask() != 0 {
                Timer::after(Duration::from_millis(PRESS_CONFIRM_MS)).await;
                if self.button.is_pressed() {
                    return (WakeReason::Button, entered);
                }
            }
            if Instant::now() >= deadline {
                return (WakeReason::Timer, entered);
            }
        }
    }

    /// Play a queued status pattern, then wait for the rest of the request.
    async fn wait_with_pattern(
        &mut self,
        request: &WaitRequest,
        standby: bool,
    ) -> (WakeReason, bool) {
        let deadline = Instant::now() + Duration::from_millis(u64::from(request.timeout_ms));
        play_pending(self.leds).await;
        self.timer_or_button(deadline, standby).await
    }

    fn log_standby(&mut self) {
        self.retention_waits = self.retention_waits.wrapping_add(1);
        let lf_status = power::lf_clock_status();
        let lf_good = power::lf_status_good(lf_status);
        if lf_good != self.lf_good_logged || self.retention_waits % STANDBY_LOG_EVERY == 1 {
            self.lf_good_logged = lf_good;
            log::info!(
                "[FLOWER STANDBY] waits={} lf_good={} lfclkstat={:#010x} {:?} last_refusal={:?}",
                self.retention_waits,
                lf_good,
                lf_status,
                standby::stats(),
                self.last_refusal
            );
        }
    }
}

impl WakeController<Cc2340Mac> for FlowerWake<'_> {
    type Mark = Instant;
    type Error = WaitError;

    fn mark(&self) -> Self::Mark {
        Instant::now()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark + Duration::from_millis(u64::from(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later
            .saturating_duration_since(earlier)
            .as_millis()
            .min(u64::from(u32::MAX)) as u32
    }

    async fn wait(
        &mut self,
        mac: &mut Cc2340Mac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {
                mac.prepare_active_wait()?;
                let (reason, _) = self.wait_with_pattern(&request, false).await;
                Ok(reason)
            }
            SleepDepth::Idle => {
                // Radio powered down, CPU in WFE, HF clock tree up.
                mac.radio_sleep()?;
                let (reason, _) = self.wait_with_pattern(&request, false).await;
                mac.radio_wake()?;
                Ok(reason)
            }
            SleepDepth::Retention => {
                // Device standby between events, only with the radio
                // actually asleep (standby powers the LRFD domain off).
                mac.radio_sleep()?;
                let allow_standby = mac.radio_is_asleep();
                let (reason, entered) = self.wait_with_pattern(&request, allow_standby).await;
                if entered && !mac.radio_note_standby() {
                    log::error!("[FLOWER] standby entered without a sleeping radio");
                }
                self.log_standby();
                mac.radio_wake()?;
                Ok(reason)
            }
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        let started = Instant::now();
        loop {
            if !self.button.is_pressed() {
                return false;
            }
            let elapsed_ms = Self::elapsed_ms(Instant::now(), started);
            if elapsed_ms >= duration_ms {
                return true;
            }
            Timer::after(Duration::from_millis(u64::from(
                (duration_ms - elapsed_ms).clamp(1, HELD_SAMPLE_MS),
            )))
            .await;
        }
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

// ── Supervision ─────────────────────────────────────────────────────

/// Reset-only supervision: no CC2340 watchdog is qualified yet. The reset
/// handle is borrowed so the composition root can also reboot after a
/// network leave.
pub struct ResetOnlySupervisor<'a> {
    reset: &'a SystemReset,
}

impl<'a> ResetOnlySupervisor<'a> {
    pub const fn new(reset: &'a SystemReset) -> Self {
        Self { reset }
    }
}

impl Supervisor for ResetOnlySupervisor<'_> {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        self.reset.reset()
    }
}

// ── Diagnostics ─────────────────────────────────────────────────────

struct RttLogger;

impl log::Log for RttLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            rtt_target::rprintln!(
                "[{}][{}] {}",
                record.level(),
                record.target(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

static LOGGER: RttLogger = RttLogger;

/// Lifecycle diagnostics rendered through RTT. Also latches a network
/// leave, after which the composition root reboots so the next boot is
/// factory-new and re-detects the fitted sensors.
pub struct RttDiagnostics<'a> {
    left_network: &'a Cell<bool>,
}

impl Diagnostics for RttDiagnostics<'_> {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::Left | DiagnosticEvent::LeaveRequested => {
                self.left_network.set(true);
                log::warn!("[FLOWER][sensor] {event:?}");
            }
            DiagnosticEvent::SecurityFailure(_)
            | DiagnosticEvent::ProfileFailure(_)
            | DiagnosticEvent::WakeFailed => {
                log::error!("[FLOWER][sensor] {event:?}");
            }
            DiagnosticEvent::ZigbeeInitializationFailed
            | DiagnosticEvent::CommissioningFailed { .. }
            | DiagnosticEvent::SecureRejoinInitializationFailed
            | DiagnosticEvent::SecureRejoinFailed { .. }
            | DiagnosticEvent::FactoryResetInitializationFailed
            | DiagnosticEvent::FactoryResetFailed { .. }
            | DiagnosticEvent::SecureRejoinPending { .. }
            | DiagnosticEvent::SecureRejoinLimitReached { .. }
            | DiagnosticEvent::MeasurementFailed { .. }
            | DiagnosticEvent::BatteryReadFailed
            | DiagnosticEvent::ReportingRejected { .. }
            | DiagnosticEvent::UnexpectedOtaEvent => {
                log::warn!("[FLOWER][sensor] {event:?}");
            }
            _ => log::info!("[FLOWER][sensor] {event:?}"),
        }
    }
}

/// Initialize the RTT transport.
pub fn init_diagnostics(left_network: &Cell<bool>) -> RttDiagnostics<'_> {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 1024,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 16,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);

    cortex_m::interrupt::free(|_| unsafe {
        // Startup is single-threaded and the time driver has not started.
        if log::set_logger_racy(&LOGGER).is_ok() {
            log::set_max_level_racy(log::LevelFilter::Info);
        }
    });

    RttDiagnostics { left_network }
}

pub fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
