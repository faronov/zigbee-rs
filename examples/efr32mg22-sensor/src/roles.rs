//! Board start-up and capability adapters shared by the mains role bins
//! (`ed.rs`, `router.rs`).
//!
//! The original sleepy sensor (`main.rs` + `platform.rs`) keeps its own
//! sequence so its bytes stay identical to the hardware-validated build.

#[cfg(any(
    all(feature = "ed", feature = "router"),
    all(feature = "ed", feature = "sed"),
    all(feature = "router", feature = "sed"),
))]
compile_error!(
    "enable exactly one role feature (sed, ed or router); use --no-default-features for ed/router"
);

use efr32mg22_devkit::{Button0, Led0, resources::BoardResources};
use efr32mg22_sensor_product as product;
use embassy_time::Instant;
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StackEventSummary, StatusSink, Supervisor,
};

/// Resources a role composition needs after clocks and time are running.
pub struct Started {
    pub radio: efr32s2_hal::radio::RadioToken,
    pub flash: efr32s2_hal::flash::Efr32s2Flash,
    pub led: Led0,
    pub button: Button0,
}

/// HFXO, VCOM log, RTCC time driver; EM2 is never entered by these roles.
pub fn start(model: &str) -> Started {
    let BoardResources {
        clocks,
        led0,
        button0,
        vcom,
        radio,
        rtcc,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());

    let clocks = efr32mg22_devkit::init_clocks(clocks);
    crate::vcom_log::init(vcom.into_vcom());
    if let Err(error) = clocks {
        log::error!("[MG22] HFXO start failed: {error:?}");
        halt()
    }
    let rtcc = rtcc.start().unwrap_or_else(|error| {
        log::error!("[MG22] RTCC start failed: {error:?}");
        halt()
    });
    efr32s2_hal::time_driver::init(rtcc);

    log::info!(
        "[MG22] {} on {} / {} ({})",
        model,
        efr32mg22_devkit::BOARD_RADIO,
        efr32mg22_devkit::BOARD_MAIN,
        efr32mg22_devkit::MCU_PART
    );
    Started {
        radio,
        flash,
        led: led0.into_led(),
        button: button0.into_button(),
    }
}

/// Reject an all-zero/all-ones DEVINFO EUI-64 before it reaches the stack.
pub fn check_ieee(ieee: [u8; 8]) {
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[MG22] no valid IEEE EUI-64");
        halt()
    }
    log::info!("[MG22] EUI-64 {:02X?}", ieee);
}

/// Open the shared security journal and log its disposition.
pub fn open_security_store(
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> product::storage::SecurityStore {
    let (store, migration) = product::storage::security_store(flash).unwrap_or_else(|error| {
        log::error!("[MG22] security journal open failed: {error:?}");
        halt()
    });
    match migration {
        product::storage::MigrationDisposition::ExistingJournal => {
            log::info!("[MG22] valid security journal preserved")
        }
        product::storage::MigrationDisposition::FreshReset => {
            log::warn!("[MG22] blank/corrupt persistence reset; fresh join required")
        }
    }
    store
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => log::warn!("[MG22] cleared network state: {what} changed"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[MG22] {what} guard failed: {error:?}");
            halt()
        }
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: immediately on a BTN0 press, otherwise after the product delay.
pub struct CommissioningTrigger {
    button: Button0,
    was_pressed: bool,
    awaiting_since: Option<Instant>,
}

impl CommissioningTrigger {
    pub fn new(button: Button0) -> Self {
        Self {
            was_pressed: button.is_pressed(),
            button,
            awaiting_since: None,
        }
    }

    /// Call once per run-loop step; `true` means request commissioning.
    pub fn poll(&mut self, awaiting: bool) -> bool {
        let pressed = self.button.is_pressed();
        let press_edge = pressed && !self.was_pressed;
        self.was_pressed = pressed;
        if !awaiting {
            self.awaiting_since = None;
            return false;
        }
        let since = *self.awaiting_since.get_or_insert_with(Instant::now);
        if press_edge || since.elapsed().as_millis() >= product::RECOMMISSION_AFTER_LEAVE_MS {
            self.awaiting_since = None;
            return true;
        }
        false
    }
}

/// LED0 on while online, off otherwise; lifecycle changes on VCOM.
pub struct LedStatus {
    led: Led0,
    last: Option<RouterStatus>,
}

impl LedStatus {
    pub fn new(led: Led0) -> Self {
        Self { led, last: None }
    }
}

impl StatusSink for LedStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last == Some(status) {
            return;
        }
        self.last = Some(status);
        match status {
            RouterStatus::Online { .. } | RouterStatus::Fault { .. } => self.led.on(),
            _ => self.led.off(),
        }
        log::info!("[MG22] status {status:?}");
    }
}

pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[MG22] commissioning attempt {attempt}")
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[MG22] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[MG22] retry {attempt} in {delay_ms} ms")
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[MG22] online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildrenRestored { count } => {
                log::info!("[MG22] {count} children restored")
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildTableDiscarded { error } => {
                log::warn!("[MG22] child table discarded: {error:?}")
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildTableSaved => log::info!("[MG22] child table saved"),
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildRemovalCompleted {
                short_address,
                delivered,
                ..
            } => log::info!("[MG22] child 0x{short_address:04X} removed (delivered={delivered})"),
            #[cfg(feature = "router")]
            DiagnosticEvent::ApsTablesRestored { count } => {
                log::info!("[MG22] {count} APS table entries restored")
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ApsTablesDiscarded { error } => {
                log::warn!("[MG22] APS tables discarded: {error:?}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::PermitJoinChanged { open }) => {
                log::info!("[MG22] permit join {open}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                address,
                short_address,
                ..
            }) => log::info!("[MG22] device announce 0x{short_address:04X} {address:02X?}"),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[MG22] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[MG22] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[MG22] factory-new after Leave; press BTN0 or wait {} s",
                product::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[MG22] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[MG22] secure rejoin ok: addr=0x{short_address:04X}")
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => {
                log::warn!("[MG22] secure rejoin failed ({failures}): {error:?}")
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[MG22] secure rejoin limit reached at {failures}")
            }
            DiagnosticEvent::FactoryReset => log::info!("[MG22] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[MG22] fatal: {error:?}"),
            _ => {}
        }
    }
}

/// No watchdog claim yet; fatal errors reset the MCU after the log drains.
pub struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        crate::vcom_log::flush();
        cortex_m::peripheral::SCB::sys_reset()
    }
}

pub type Parts = RouterParts<LedStatus, ResetSupervisor, LogDiagnostics>;

pub fn parts(led: Led0) -> Parts {
    RouterParts::new(LedStatus::new(led), ResetSupervisor, LogDiagnostics)
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

/// Stop with LED0 lit; the VCOM log keeps the reason.
pub fn halt() -> ! {
    crate::vcom_log::flush();
    efr32mg22_devkit::emergency_led_on();
    loop {
        cortex_m::asm::wfi();
    }
}
