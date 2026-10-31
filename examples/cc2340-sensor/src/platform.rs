//! LP-EM-CC2340R5 application adapters for the shared sleepy-sensor lifecycle.
//!
//! This composition maps the board's raw, typed LED/button/reset resources
//! onto the CC2340 sensor product's selected lifecycle behavior. The board
//! crate intentionally contains none of these application semantics.

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use lp_em_cc2340r5::{
    pins::{self, Button1, Button2, Led1, Led2},
    power,
    reset::SystemReset,
    standby::{self, StandbyRefusal},
};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, SensorStatus, SleepDepth, StatusSink, Supervisor, WaitRequest,
    WakeController, WakeReason,
};
use zigbee_mac::MacError;
use zigbee_mac::cc2340::{Cc2340SoftMac, RadioError};

/// A press edge counts only if the button is still down this long after it
/// (contact bounce and release glitches must not trigger BTN1's
/// join/leave action).
const PRESS_CONFIRM_MS: u64 = 30;
/// Level-sampling period while a press is being held (long-press detection
/// only; idle waits are purely interrupt driven).
const HELD_SAMPLE_MS: u32 = 25;
/// Standby counters are logged every this many retention waits.
const STANDBY_LOG_EVERY: u32 = 30;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaitError {
    Radio(RadioError),
    Mac(MacError),
}

impl From<RadioError> for WaitError {
    fn from(error: RadioError) -> Self {
        Self::Radio(error)
    }
}

impl From<MacError> for WaitError {
    fn from(error: MacError) -> Self {
        Self::Mac(error)
    }
}

/// BTN1 invokes the product's join/leave/reset action. BTN2 records an
/// adapter diagnostic without triggering a network action.
///
/// Both buttons are falling-edge interrupts (`GPIO_COMB`); between events the
/// executor sleeps in WFE and only the SYSTIM alarm or a press wakes it.
pub struct LaunchPadWake {
    action: Button1,
    diagnostics: Button2,
    retention_waits: u32,
    lf_good_logged: bool,
    last_refusal: Option<StandbyRefusal>,
}

impl LaunchPadWake {
    pub fn new(mut action: Button1, mut diagnostics: Button2) -> Self {
        // Edges latched before the lifecycle starts are not wake requests.
        action.enable_press_interrupt();
        diagnostics.enable_press_interrupt();
        let _ = pins::take_button_edges();
        Self {
            action,
            diagnostics,
            retention_waits: 0,
            lf_good_logged: false,
            last_refusal: None,
        }
    }

    /// Wait `timeout_ms` or until a confirmed BTN1 press. BTN2 presses are
    /// logged and do not end the wait.
    ///
    /// With `standby` the gaps between events are spent in device standby
    /// ([`standby::standby_until`]; RTC or button-IOC wake). A refusal other
    /// than a pending interrupt (LF clock not yet qualified, deadline too
    /// close) falls back to the WFE idle wait for the rest of this wait.
    /// Returns the wake reason and whether standby was entered at all.
    async fn timer_or_button(
        &mut self,
        timeout_ms: u32,
        sleep_depth: SleepDepth,
        standby: bool,
    ) -> (WakeReason, bool) {
        let deadline = Instant::now() + Duration::from_millis(u64::from(timeout_ms));
        // A press held across the previous action is not a new request; the
        // level check keeps the old "pressed at wait start" semantics only
        // for a press that is genuinely still down.
        if self.action.is_pressed() {
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
                        // Interrupts that woke us were serviced when the
                        // critical section ended; re-check edges/deadline.
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
            if edges & self.diagnostics.edge_mask() != 0 {
                log::info!(
                    "[CC2340] BTN2 diagnostics: {sleep_depth:?} wait, timeout={timeout_ms}ms, standby={:?}, lf_good={}",
                    standby::stats(),
                    power::lf_clock_good()
                );
            }
            if edges & self.action.edge_mask() != 0 {
                Timer::after(Duration::from_millis(PRESS_CONFIRM_MS)).await;
                if self.action.is_pressed() {
                    return (WakeReason::Button, entered);
                }
            }
            if Instant::now() >= deadline {
                return (WakeReason::Timer, entered);
            }
        }
    }

    /// Log standby counters every [`STANDBY_LOG_EVERY`] retention waits and
    /// whenever the LF clock qualification state changes.
    fn log_standby(&mut self) {
        self.retention_waits = self.retention_waits.wrapping_add(1);
        let lf_status = power::lf_clock_status();
        let lf_good = power::lf_status_good(lf_status);
        if lf_good != self.lf_good_logged || self.retention_waits % STANDBY_LOG_EVERY == 1 {
            self.lf_good_logged = lf_good;
            log::info!(
                "[CC2340 STANDBY] waits={} lf_good={} lfclkstat={:#010x} {:?} last_refusal={:?}",
                self.retention_waits,
                lf_good,
                lf_status,
                standby::stats(),
                self.last_refusal
            );
        }
    }
}

impl WakeController<Cc2340SoftMac> for LaunchPadWake {
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
        mac: &mut Cc2340SoftMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {
                mac.phy_mut().prepare_active_wait()?;
                let (reason, _) = self
                    .timer_or_button(request.timeout_ms, request.sleep_depth, false)
                    .await;
                Ok(reason)
            }
            SleepDepth::Idle => {
                // Radio cores/refsys off and LRFD clocks gated; CPU in WFE
                // with no periodic tick; HFXT stopped (CPU/SYSTIM on HFOSC).
                // This is not standby: the HF clock tree stays up.
                mac.sleep()?;
                let (reason, _) = self
                    .timer_or_button(request.timeout_ms, request.sleep_depth, false)
                    .await;
                // Eager restore, so a restore failure is reported here rather
                // than as a failed poll.
                mac.wake()?;
                Ok(reason)
            }
            SleepDepth::Retention => {
                // Idle radio power-down, then device standby between events
                // (`RadioSleepDisposition::StandbyCapable`). Standby is only
                // attempted with the radio actually asleep and no received
                // frame held in its RX FIFO: it powers the LRFD domain off.
                mac.sleep()?;
                let allow_standby = mac.phy().is_asleep() && !mac.phy().holds_received_frame();
                let (reason, entered) = self
                    .timer_or_button(request.timeout_ms, request.sleep_depth, allow_standby)
                    .await;
                if entered && !mac.phy_mut().note_standby() {
                    log::error!("[CC2340] standby entered without a sleeping radio");
                }
                self.log_standby();
                // Full radio re-init after standby (or the plain clock
                // restore otherwise), eagerly so failures surface here.
                mac.wake()?;
                Ok(reason)
            }
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        let started = Instant::now();
        loop {
            if !self.action.is_pressed() {
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

/// LED1 communicates network/activity state; LED2 communicates identify,
/// report, reset, OTA, and fault state.
pub struct StatusLeds {
    network: Led1,
    identify: Led2,
}

impl StatusLeds {
    pub const fn new(network: Led1, identify: Led2) -> Self {
        Self { network, identify }
    }
}

impl StatusSink for StatusLeds {
    fn set(&mut self, status: SensorStatus) {
        let (network_on, identify_on) = match status {
            SensorStatus::Off => (false, false),
            SensorStatus::Joining { on } | SensorStatus::Joined { active: on } => (on, false),
            SensorStatus::Identifying { on } | SensorStatus::Reporting { on } => (false, on),
            SensorStatus::Resetting { on } => (on, on),
            SensorStatus::Ota | SensorStatus::Fault => (true, true),
        };
        self.network.set(network_on);
        self.identify.set(identify_on);
    }
}

/// Reset-only supervision for this product. No watchdog configuration is
/// claimed until a CC2340 watchdog implementation is qualified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogDisposition {
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SupervisorDiagnostics {
    pub heartbeat_count: u32,
    pub watchdog: WatchdogDisposition,
}

pub struct ResetOnlySupervisor {
    reset: SystemReset,
    heartbeat_count: u32,
}

impl ResetOnlySupervisor {
    pub const fn new(reset: SystemReset) -> Self {
        Self {
            reset,
            heartbeat_count: 0,
        }
    }

    pub const fn watchdog_disposition(&self) -> WatchdogDisposition {
        WatchdogDisposition::Unavailable
    }

    pub fn heartbeat(&mut self) {
        self.heartbeat_count = self.heartbeat_count.wrapping_add(1);
    }

    pub const fn diagnostics(&self) -> SupervisorDiagnostics {
        SupervisorDiagnostics {
            heartbeat_count: self.heartbeat_count,
            watchdog: WatchdogDisposition::Unavailable,
        }
    }
}

impl Supervisor for ResetOnlySupervisor {
    fn heartbeat(&mut self) {
        ResetOnlySupervisor::heartbeat(self);
    }

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        self.reset.reset()
    }
}

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticsDisposition {
    RttLog,
}

pub const fn disposition() -> DiagnosticsDisposition {
    DiagnosticsDisposition::RttLog
}

/// Typed shared-lifecycle diagnostics rendered through RTT.
#[derive(Debug, Default, Clone, Copy)]
pub struct RttDiagnostics;

impl RttDiagnostics {
    pub const fn new() -> Self {
        Self
    }
}

impl Diagnostics for RttDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::SecurityFailure(_)
            | DiagnosticEvent::ProfileFailure(_)
            | DiagnosticEvent::WakeFailed => {
                log::error!("[CC2340][sensor] {event:?}");
            }
            DiagnosticEvent::ZigbeeInitializationFailed
            | DiagnosticEvent::CommissioningFailed { .. }
            | DiagnosticEvent::SecureRejoinInitializationFailed
            | DiagnosticEvent::SecureRejoinFailed { .. }
            | DiagnosticEvent::FactoryResetInitializationFailed
            | DiagnosticEvent::FactoryResetFailed { .. }
            | DiagnosticEvent::SecureRejoinPending { .. }
            | DiagnosticEvent::SecureRejoinLimitReached { .. }
            | DiagnosticEvent::EnvironmentReadFailed
            | DiagnosticEvent::BatteryReadFailed
            | DiagnosticEvent::ReportingRejected { .. }
            | DiagnosticEvent::UnexpectedOtaEvent => {
                log::warn!("[CC2340][sensor] {event:?}");
            }
            _ => log::info!("[CC2340][sensor] {event:?}"),
        }
    }
}

/// Initialize the RTT transport used by this firmware composition.
pub fn init_diagnostics() -> RttDiagnostics {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 2048,
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
        // Startup is single-threaded and the board time driver has not
        // started.
        if log::set_logger_racy(&LOGGER).is_ok() {
            log::set_max_level_racy(log::LevelFilter::Info);
        }
    });

    RttDiagnostics::new()
}

pub fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
