//! Start-up and capability adapters of the router composition (`main.rs`).

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StackEventSummary, StatusSink, Supervisor,
};
use stm32wb_hal::embassy_stm32::gpio::Input;
use stm32wb_hal::embassy_stm32::{self, bind_interrupts, dma, ipcc, peripherals};
use stm32wb5mm_dk_router_product as product;
use zigbee_runtime::binding_persistence::BindingPersistenceError;

bind_interrupts!(pub struct Irqs {
    IPCC_C1_RX => ipcc::ReceiveInterruptHandler;
    IPCC_C1_TX => ipcc::TransmitInterruptHandler;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    #[cfg(feature = "mic")]
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

/// Resources a composition needs once clocks, time and the log run.
pub struct Started {
    pub ipcc: embassy_stm32::Peri<'static, peripherals::IPCC>,
    pub rng: embassy_stm32::Peri<'static, peripherals::RNG>,
    pub button_b1: Input<'static>,
    pub button_b2: Input<'static>,
    #[cfg(any(feature = "rgb-led", feature = "lcd"))]
    pub spi1: stm32wb5mm_dk::spi1::Spi1Pins,
    #[cfg(feature = "touch")]
    pub touch: stm32wb5mm_dk::touch::TouchPins,
    #[cfg(feature = "sensors")]
    pub sensor_bus: stm32wb5mm_dk::sensors::SensorBus,
    #[cfg(feature = "mic")]
    pub mic: stm32wb5mm_dk::mic::MicPins,
}

/// HSE 32 MHz / PLL 64 MHz (the clock tree CPU2's radio requires), LSE for
/// the radio wake-up timer (plus the SAI1 clock with `mic`), and the VCOM
/// log.
pub fn start(spawner: Spawner, model: &str) -> Started {
    let mut config = embassy_stm32::Config::default();
    config.rcc = stm32wb5mm_dk::rcc_config();
    let p = embassy_stm32::init(config);

    let vcom = stm32wb5mm_dk::vcom_tx(p.USART1, p.PB6, p.DMA1_CH1, Irqs);
    crate::vcom_log::init();
    spawner.spawn(crate::vcom_log::drain(vcom).unwrap_or_else(|_| halt()));
    log::info!("[WB] {model} on STM32WB5MM-DK");

    Started {
        ipcc: p.IPCC,
        rng: p.RNG,
        button_b1: stm32wb5mm_dk::button_b1(p.PC12),
        button_b2: stm32wb5mm_dk::button_b2(p.PC13),
        #[cfg(any(feature = "rgb-led", feature = "lcd"))]
        spi1: stm32wb5mm_dk::spi1::Spi1Pins {
            spi1: p.SPI1,
            sck: p.PA1,
            mosi: p.PA7,
            lcd_cs: p.PH0,
            lcd_dc: p.PC9,
            lcd_rst: p.PC8,
            led_select: p.PH1,
        },
        #[cfg(feature = "touch")]
        touch: stm32wb5mm_dk::touch::TouchPins {
            tsc: p.TSC,
            key_sample: p.PD10,
            key: p.PD11,
            shield_sample: p.PC6,
            shield: p.PC7,
        },
        #[cfg(feature = "sensors")]
        sensor_bus: stm32wb5mm_dk::sensors::i2c3(p.I2C3, p.PB13, p.PB11),
        #[cfg(feature = "mic")]
        mic: stm32wb5mm_dk::mic::MicPins {
            sai1: p.SAI1,
            clock: p.PA8,
            data: p.PA9,
            dma: p.DMA1_CH2,
        },
    }
}

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

/// Lifecycle changes on VCOM, published to the indicator and display
/// tasks through [`crate::shared::STATUS`].
pub struct LogStatus {
    last: Option<RouterStatus>,
}

impl LogStatus {
    pub const fn new() -> Self {
        Self { last: None }
    }
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            crate::shared::publish_status(status);
            log::info!("[WB] status {status:?}");
        }
    }
}

pub struct LogDiagnostics;

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
                "[WB] factory-new after Leave; press B1 or wait {} s",
                product::RECOMMISSION_AFTER_LEAVE_MS / 1000
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
        crate::vcom_log::emergency(format_args!("[WB] reset"));
        cortex_m::peripheral::SCB::sys_reset()
    }
}

pub type Parts = RouterParts<LogStatus, ResetSupervisor, LogDiagnostics>;

pub fn parts() -> Parts {
    RouterParts::new(LogStatus::new(), ResetSupervisor, LogDiagnostics)
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
            "[WB] retrying after {error:?} ({}/{STEP_RETRY_LIMIT})",
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

/// Stop; the VCOM log keeps the reason.
pub fn halt() -> ! {
    crate::vcom_log::emergency(format_args!("[WB] halted"));
    loop {
        cortex_m::asm::wfi();
    }
}
