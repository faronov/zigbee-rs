//! LP-EM-CC2340R5 application adapters for the shared sleepy-sensor lifecycle.
//!
//! This composition maps the board's raw, typed LED/button/reset resources
//! onto the CC2340 sensor product's selected lifecycle behavior. The board
//! crate intentionally contains none of these application semantics.

use embassy_time::{Duration, Instant, Timer};
use lp_em_cc2340r5::{
    pins::{Button1, Button2, Led1, Led2},
    reset::SystemReset,
};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, SensorStatus, SleepDepth, StatusSink, Supervisor, WaitRequest,
    WakeController, WakeReason,
};
use zigbee_mac::cc2340::{Cc2340Mac, RadioError};

const BUTTON_SAMPLE_MS: u32 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveWaitError {
    Radio(RadioError),
    UnsupportedSleepDepth,
}

impl From<RadioError> for ActiveWaitError {
    fn from(error: RadioError) -> Self {
        Self::Radio(error)
    }
}

/// BTN1 invokes the product's join/leave/reset action. BTN2 records an
/// adapter diagnostic without triggering a network action.
pub struct ActiveWakeController {
    action: Button1,
    diagnostics: Button2,
    diagnostics_was_pressed: bool,
}

impl ActiveWakeController {
    pub const fn new(action: Button1, diagnostics: Button2) -> Self {
        Self {
            action,
            diagnostics,
            diagnostics_was_pressed: false,
        }
    }

    async fn sample_delay(remaining_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(
            remaining_ms.clamp(1, BUTTON_SAMPLE_MS),
        )))
        .await;
    }
}

impl WakeController<Cc2340Mac> for ActiveWakeController {
    type Mark = Instant;
    type Error = ActiveWaitError;

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
        if request.sleep_depth != SleepDepth::Active {
            return Err(ActiveWaitError::UnsupportedSleepDepth);
        }

        mac.prepare_active_wait()?;
        let started = Instant::now();

        loop {
            if self.action.is_pressed() {
                return Ok(WakeReason::Button);
            }

            let diagnostics_pressed = self.diagnostics.is_pressed();
            if diagnostics_pressed && !self.diagnostics_was_pressed {
                log::info!(
                    "[CC2340] BTN2 diagnostics: active-only radio wait, timeout={}ms",
                    request.timeout_ms
                );
            }
            self.diagnostics_was_pressed = diagnostics_pressed;

            let elapsed_ms = Self::elapsed_ms(Instant::now(), started);
            if elapsed_ms >= request.timeout_ms {
                return Ok(WakeReason::Timer);
            }
            Self::sample_delay(request.timeout_ms - elapsed_ms).await;
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
            Self::sample_delay(duration_ms - elapsed_ms).await;
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
                size: 512,
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
        // Startup is single-threaded and the board SysTick has not started.
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
