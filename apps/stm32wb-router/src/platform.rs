//! Capability adapters of the STM32WB router compositions: lifecycle status
//! and diagnostics on the log, the reset supervisor, and the `step()` error
//! policy.

use embassy_time::{Duration, Timer};
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StackEventSummary, StatusSink, Supervisor,
};
use zigbee_runtime::binding_persistence::BindingPersistenceError;

/// Reject an all-zero/all-ones UID64 before it reaches the stack.
pub fn check_ieee(ieee: [u8; 8]) {
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[WB] no valid IEEE EUI-64");
        halt()
    }
    log::info!("[WB] EUI-64 {:02X?}", ieee);
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => log::warn!("[WB] cleared network state: {what} changed"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[WB] {what} guard failed: {error:?}");
            halt()
        }
    }
}

/// Lifecycle changes on the log, handed to the composition's indicator
/// (and display) tasks through `publish`.
pub struct LogStatus {
    last: Option<RouterStatus>,
    publish: fn(RouterStatus),
}

impl LogStatus {
    pub const fn new(publish: fn(RouterStatus)) -> Self {
        Self {
            last: None,
            publish,
        }
    }
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            (self.publish)(status);
            log::info!("[WB] status {status:?}");
        }
    }
}

/// Lifecycle diagnostics on the log. `commission_button` names the button
/// that requests commissioning after a Leave.
pub struct LogDiagnostics {
    commission_button: &'static str,
}

impl LogDiagnostics {
    pub const fn new(commission_button: &'static str) -> Self {
        Self { commission_button }
    }
}

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[WB] commissioning attempt {attempt}")
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[WB] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[WB] retry {attempt} in {delay_ms} ms")
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[WB] online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            DiagnosticEvent::ChildrenRestored { count } => {
                log::info!("[WB] {count} children restored")
            }
            DiagnosticEvent::ChildTableDiscarded { error } => {
                log::warn!("[WB] child table discarded: {error:?}")
            }
            DiagnosticEvent::ChildTableSaved => log::info!("[WB] child table saved"),
            DiagnosticEvent::ChildRemovalCompleted {
                short_address,
                delivered,
                ..
            } => log::info!("[WB] child 0x{short_address:04X} removed (delivered={delivered})"),
            DiagnosticEvent::ApsTablesRestored { count } => {
                log::info!("[WB] {count} APS table entries restored")
            }
            DiagnosticEvent::ApsTablesDiscarded { error } => {
                log::warn!("[WB] APS tables discarded: {error:?}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::PermitJoinChanged { open }) => {
                log::info!("[WB] permit join {open}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                address,
                short_address,
                ..
            }) => log::info!("[WB] device announce 0x{short_address:04X} {address:02X?}"),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[WB] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[WB] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[WB] factory-new after Leave; press {} or wait {} s",
                self.commission_button,
                stm32wb_router_product::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[WB] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[WB] secure rejoin ok: addr=0x{short_address:04X}")
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => {
                log::warn!("[WB] secure rejoin failed ({failures}): {error:?}")
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[WB] secure rejoin limit reached at {failures}")
            }
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableRestored { count } => {
                log::info!("[WB] {count} GP proxy table entries restored")
            }
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableDiscarded { error } => {
                log::warn!("[WB] GP proxy table discarded: {error:?}")
            }
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableSaved => log::info!("[WB] GP proxy table saved"),
            #[cfg(feature = "green-power")]
            DiagnosticEvent::GpProxyTableCleared => log::info!("[WB] GP proxy table cleared"),
            DiagnosticEvent::FactoryReset => log::info!("[WB] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[WB] fatal: {error:?}"),
            _ => {}
        }
    }
}

/// No watchdog claim yet; fatal errors reset the chip (CPU2 restarts with
/// it) after the log drains.
pub struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        crate::log::emergency(format_args!("[WB] reset"));
        cortex_m::peripheral::SCB::sys_reset()
    }
}

pub type Parts = RouterParts<LogStatus, ResetSupervisor, LogDiagnostics>;

/// `publish` receives every lifecycle change; `commission_button` names
/// the button that requests commissioning in the log.
pub fn parts(publish: fn(RouterStatus), commission_button: &'static str) -> Parts {
    RouterParts::new(
        LogStatus::new(publish),
        ResetSupervisor,
        LogDiagnostics::new(commission_button),
    )
}

/// Spacing between retries of a retry-safe `step()` failure.
const STEP_RETRY_BACKOFF: Duration = Duration::from_millis(250);
/// Consecutive retry-safe failures tolerated before resetting (~10 s).
const STEP_RETRY_LIMIT: u8 = 40;

/// Classifies `step()` errors. Delivery failures of a prepared binding ACK or
/// response (typically "no route yet" right after a resume) are retry-safe:
/// the runtime keeps the prepared frames and never re-applies the binding,
/// so the next `step()` retries them. Everything else resets.
#[derive(Default)]
pub struct StepErrors {
    consecutive: u8,
}

impl StepErrors {
    pub const fn new() -> Self {
        Self { consecutive: 0 }
    }

    pub fn ok(&mut self) {
        self.consecutive = 0;
    }

    /// Reset on a fatal error; back off before the next `step()` otherwise.
    pub async fn handle(
        &mut self,
        parts: &mut Parts,
        archetype: NodeArchetype,
        error: RouterAppError,
    ) {
        self.record(parts, archetype, error);
        Timer::after(STEP_RETRY_BACKOFF).await;
    }

    // Not inlined into the caller's async state machine: `RouterAppError`'s
    // (inline) `Debug` stays in this crate, once.
    #[inline(never)]
    fn record(&mut self, parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) {
        let retryable = matches!(
            error,
            RouterAppError::Binding(
                BindingPersistenceError::Acknowledgement(_) | BindingPersistenceError::Response(_)
            )
        );
        if !retryable || self.consecutive >= STEP_RETRY_LIMIT {
            fatal(parts, archetype, error)
        }
        self.consecutive += 1;
        log::warn!(
            "[WB] retrying after {error:?} ({}/{STEP_RETRY_LIMIT})",
            self.consecutive
        );
    }
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

/// Halt on a composition the app rejected (a product policy error).
pub fn invalid_composition(error: RouterAppError) -> ! {
    log::error!("[WB] invalid composition: {error:?}");
    halt()
}

/// Stop; the log keeps the reason.
pub fn halt() -> ! {
    crate::log::emergency(format_args!("[WB] halted"));
    loop {
        cortex_m::asm::wfi();
    }
}
