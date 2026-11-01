//! Board start-up and capability adapters shared by the mains role bins
//! (`ed.rs`, `router.rs`).
//!
//! The original sleepy sensor (`main.rs`) keeps its own sequence (Embassy
//! `NrfRadioPhy`, defmt over RTT) so its image stays identical to the
//! hardware-validated build.

#[cfg(any(
    all(feature = "ed", feature = "router"),
    all(feature = "ed", feature = "sed"),
    all(feature = "router", feature = "sed"),
))]
compile_error!(
    "enable exactly one role feature (sed, ed or router); use --no-default-features for ed/router"
);

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::interrupt::{InterruptExt, Priority};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::{bind_interrupts, interrupt, peripherals, rng, uarte};
use embassy_time::{Duration, Instant, Timer};
use nrf52833_sensor_product as product;
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StackEventSummary, StatusSink, Supervisor,
};
use zigbee_mac::nrf::{NrfEcbToken, NrfIsrRadioPhy, NrfRadioIrq};
use zigbee_runtime::binding_persistence::BindingPersistenceError;

bind_interrupts!(pub struct Irqs {
    RADIO => NrfRadioIrq;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    UARTE0 => uarte::InterruptHandler<peripherals::UARTE0>;
});

// Ensure all RAM banks are powered on before .bss/.data initialisation;
// identical to the sleepy sensor's `__pre_init` (see `main.rs`).
core::arch::global_asm!(
    ".section .text.__pre_init",
    ".global __pre_init",
    ".thumb_func",
    "__pre_init:",
    "ldr r0, =0x40000904", // POWER.RAM[0].POWERSET
    "mvn r1, #0",          // r1 = 0xFFFFFFFF
    "str r1, [r0, #0x00]", // RAM[0].POWERSET
    "str r1, [r0, #0x10]", // RAM[1].POWERSET
    "str r1, [r0, #0x20]", // RAM[2].POWERSET
    "str r1, [r0, #0x30]", // RAM[3].POWERSET
    "str r1, [r0, #0x40]", // RAM[4].POWERSET
    "str r1, [r0, #0x50]", // RAM[5].POWERSET
    "str r1, [r0, #0x60]", // RAM[6].POWERSET
    "str r1, [r0, #0x70]", // RAM[7].POWERSET
    "str r1, [r0, #0x80]", // RAM[8].POWERSET
    "bx lr",
);

pub type Phy = NrfIsrRadioPhy<'static, peripherals::RNG>;

/// Resources a role composition needs after clocks and time are running.
pub struct Started {
    pub phy: Phy,
    pub nvmc: Nvmc<'static>,
    pub led: Output<'static>,
    pub button: Input<'static>,
}

/// HFXO, DC/DC, VCOM log and the interrupt-driven radio with hardware AES.
///
/// RADIO runs alone at priority 0 so the Imm-Ack deadline never waits for
/// another handler; time, GPIOTE, RNG and UART run below it.
pub fn start(spawner: Spawner, model: &str) -> Started {
    let mut config = embassy_nrf::config::Config::default();
    // HFCLK from the DK's 32 MHz crystal — required by the 802.15.4 radio.
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.dcdc = embassy_nrf::config::DcdcConfig { reg1: true };
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(config);

    let vcom = nrf52833_dk::vcom_tx(p.UARTE0, Irqs, p.P0_06);
    interrupt::UARTE0.set_priority(Priority::P3);
    crate::vcom_log::init();
    spawner.must_spawn(crate::vcom_log::drain(vcom));
    log::info!("[NRF] {model} on nRF52833 DK (PCA10100)");

    let rng = rng::Rng::new(p.RNG, Irqs);
    interrupt::RNG.set_priority(Priority::P3);
    let mut phy = NrfIsrRadioPhy::new(
        p.RADIO, p.TIMER0, p.PPI_CH20, p.PPI_CH26, p.PPI_CH27, Irqs, rng,
    );
    let Some(aes) = NrfEcbToken::take() else {
        log::error!("[NRF] ECB AES already owned");
        halt()
    };
    if let Err(error) = phy.install_aes_engine(aes) {
        log::error!("[NRF] ECB AES startup KAT failed: {error:?}");
        halt()
    }

    Started {
        phy,
        nvmc: Nvmc::new(p.NVMC),
        led: nrf52833_dk::led(p.P0_13),
        button: nrf52833_dk::button(p.P0_11),
    }
}

/// Reject an all-zero/all-ones FICR EUI-64 before it reaches the stack.
pub fn check_ieee(ieee: [u8; 8]) {
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[NRF] no valid IEEE EUI-64");
        halt()
    }
    log::info!("[NRF] EUI-64 {:02X?}", ieee);
}

/// Open the shared security journal.
pub fn open_security_store(nvmc: Nvmc<'static>) -> product::storage::SecurityStore<'static> {
    product::storage::security_store(nvmc)
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => log::warn!("[NRF] cleared network state: {what} changed"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[NRF] {what} guard failed: {error:?}");
            halt()
        }
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: immediately on a Button 1 press, otherwise after the product
/// delay.
pub struct CommissioningTrigger {
    button: Input<'static>,
    was_pressed: bool,
    awaiting_since: Option<Instant>,
}

impl CommissioningTrigger {
    pub fn new(button: Input<'static>) -> Self {
        Self {
            was_pressed: button.is_low(),
            button,
            awaiting_since: None,
        }
    }

    /// Call once per run-loop step; `true` means request commissioning.
    pub fn poll(&mut self, awaiting: bool) -> bool {
        let pressed = self.button.is_low();
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

/// LED1 on while online, off otherwise; lifecycle changes on VCOM.
pub struct LedStatus {
    led: Output<'static>,
    last: Option<RouterStatus>,
}

impl LedStatus {
    pub fn new(led: Output<'static>) -> Self {
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
            // Active low.
            RouterStatus::Online { .. } | RouterStatus::Fault { .. } => self.led.set_low(),
            _ => self.led.set_high(),
        }
        log::info!("[NRF] status {status:?}");
    }
}

pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[NRF] commissioning attempt {attempt}")
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[NRF] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[NRF] retry {attempt} in {delay_ms} ms")
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[NRF] online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildrenRestored { count } => {
                log::info!("[NRF] {count} children restored")
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildTableDiscarded { error } => {
                log::warn!("[NRF] child table discarded: {error:?}")
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildTableSaved => log::info!("[NRF] child table saved"),
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildRemovalCompleted {
                short_address,
                delivered,
                ..
            } => log::info!("[NRF] child 0x{short_address:04X} removed (delivered={delivered})"),
            #[cfg(feature = "router")]
            DiagnosticEvent::ApsTablesRestored { count } => {
                log::info!("[NRF] {count} APS table entries restored")
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ApsTablesDiscarded { error } => {
                log::warn!("[NRF] APS tables discarded: {error:?}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::PermitJoinChanged { open }) => {
                log::info!("[NRF] permit join {open}")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                address,
                short_address,
                ..
            }) => log::info!("[NRF] device announce 0x{short_address:04X} {address:02X?}"),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[NRF] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[NRF] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[NRF] factory-new after Leave; press Button 1 or wait {} s",
                product::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[NRF] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[NRF] secure rejoin ok: addr=0x{short_address:04X}")
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => {
                log::warn!("[NRF] secure rejoin failed ({failures}): {error:?}")
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[NRF] secure rejoin limit reached at {failures}")
            }
            DiagnosticEvent::FactoryReset => log::info!("[NRF] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[NRF] fatal: {error:?}"),
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
        crate::vcom_log::emergency(format_args!("[NRF] reset"));
        cortex_m::peripheral::SCB::sys_reset()
    }
}

pub type Parts = RouterParts<LedStatus, ResetSupervisor, LogDiagnostics>;

pub fn parts(led: Output<'static>) -> Parts {
    RouterParts::new(LedStatus::new(led), ResetSupervisor, LogDiagnostics)
}

/// Spacing between retries of a retry-safe `step()` failure.
const STEP_RETRY_BACKOFF: Duration = Duration::from_millis(250);
/// Consecutive retry-safe failures tolerated before resetting (~10 s).
const STEP_RETRY_LIMIT: u8 = 40;

/// Classifies `step()` errors. Delivery failures of a prepared binding ACK or
/// response (typically "no route yet" right after a resume) are retry-safe:
/// the runtime keeps the prepared frames and never re-applies the binding,
/// so the next `step()` retries them. Everything else resets.
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

    pub async fn handle(
        &mut self,
        parts: &mut Parts,
        archetype: NodeArchetype,
        error: RouterAppError,
    ) {
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
            "[NRF] retrying after {error:?} ({}/{STEP_RETRY_LIMIT})",
            self.consecutive
        );
        Timer::after(STEP_RETRY_BACKOFF).await;
    }
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

/// Stop with LED1 lit; the VCOM log keeps the reason.
pub fn halt() -> ! {
    crate::vcom_log::emergency(format_args!("[NRF] halted"));
    nrf52833_dk::emergency_led_on();
    loop {
        cortex_m::asm::wfi();
    }
}
