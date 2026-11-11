//! Start-up and capability adapters shared by the mains role bins
//! (`ed.rs`, `router.rs`).
//!
//! The original sleepy sensor (`main.rs` + `platform.rs`) keeps its own
//! sequence so its bytes stay identical to the validated build.

#[cfg(any(
    all(feature = "ed", feature = "router"),
    all(feature = "ed", feature = "sed"),
    all(feature = "router", feature = "sed"),
))]
compile_error!(
    "enable exactly one role feature (sed, ed or router); use --no-default-features for ed/router"
);

#[cfg(all(feature = "status-led", not(any(feature = "ed", feature = "router"))))]
compile_error!("`status-led` is for the mains roles (ed, router)");

#[cfg(all(feature = "light-sleep", any(feature = "ed", feature = "router")))]
compile_error!("`light-sleep` is a sleepy-sensor feature; the mains roles keep RX on");

use embassy_time::Instant;
use esp32_zigbee_devkit_product as product;
use esp_hal::gpio::{Input, InputConfig, Pull};
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StackEventSummary, StatusSink, Supervisor,
};

/// `log` → USB Serial/JTAG console.
struct EspLogger;

impl log::Log for EspLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }
    fn log(&self, record: &log::Record) {
        esp_println::println!("[{}] {}", record.level(), record.args());
    }
    fn flush(&self) {}
}

static LOGGER: EspLogger = EspLogger;

/// Resources a role composition needs after the clock is running.
pub struct Started {
    pub radio: esp_hal::peripherals::IEEE802154<'static>,
    pub aes: esp_hal::peripherals::AES<'static>,
    pub button: Input<'static>,
    /// Peripherals of the status sink (see [`parts`]).
    pub status_pins: StatusPins,
}

/// Heap, console log and SYSTIMER time driver. The mains roles never enter
/// light sleep.
pub fn start(model: &str) -> Started {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // zigbee-mac's esp-radio driver allocates its RX queue.
    esp_alloc::heap_allocator!(size: 32768);
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);
    product::time_driver::init(peripherals.SYSTIMER);
    log::info!(
        "[ESP32-C6] {} firmware {}",
        model,
        crate::FIRMWARE_VERSION_STR
    );
    Started {
        radio: peripherals.IEEE802154,
        aes: peripherals.AES,
        // BOOT button (GPIO9, active low).
        button: Input::new(
            peripherals.GPIO9,
            InputConfig::default().with_pull(Pull::Up),
        ),
        #[cfg(feature = "status-led")]
        status_pins: (peripherals.RMT, peripherals.GPIO8),
        #[cfg(not(feature = "status-led"))]
        status_pins: StatusPins,
    }
}

/// Bring up the radio PHY with the hardware AES engine.
pub fn radio_phy(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
) -> zigbee_mac::esp::EspRadioPhy<'static> {
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(radio);
    let mut phy =
        zigbee_mac::esp::EspRadioPhy::new(ieee802154, esp_radio::ieee802154::Config::default());
    if let Err(error) = phy.install_aes_engine(aes) {
        log::error!("[ESP32-C6] hardware AES KAT failed: {:?}", error);
        halt()
    }
    phy
}

/// Reject an all-zero/all-ones eFuse EUI-64 before it reaches the stack.
pub fn check_ieee(ieee: [u8; 8]) {
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[ESP32-C6] no valid IEEE EUI-64");
        halt()
    }
    log::info!("[ESP32-C6] EUI-64 {:02X?}", ieee);
}

/// Open the shared security journal (running the one-time legacy migration)
/// and log its disposition.
pub fn open_security_store() -> product::storage::SecurityStore {
    let (store, migration) = product::storage::open_security_store().unwrap_or_else(|error| {
        log::error!("[ESP32-C6] persistence migration failed: {:?}", error);
        halt()
    });
    log::info!("[ESP32-C6] persistence migration: {:?}", migration);
    store
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => log::warn!("[ESP32-C6] cleared network state: {} changed", what),
        Ok(false) => {}
        Err(error) => {
            log::error!("[ESP32-C6] {} guard failed: {:?}", what, error);
            halt()
        }
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: immediately on a BOOT press, otherwise after the product delay.
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

/// Lifecycle changes on the console. The DevKitC-1 LED is a WS2812 on
/// GPIO8 (no plain GPIO LED): it is shown only with `status-led` (see
/// [`Status`]).
pub struct LogStatus {
    last: Option<RouterStatus>,
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last == Some(status) {
            return;
        }
        self.last = Some(status);
        log::info!("[ESP32-C6] status {:?}", status);
    }
}

/// The status sink of this image: the console, plus the WS2812 on GPIO8
/// with `status-led` (mains roles only; sleepy images never drive it).
#[cfg(feature = "status-led")]
pub type Status = product::status_led::RgbStatusLed<LogStatus>;
#[cfg(not(feature = "status-led"))]
pub type Status = LogStatus;

/// What [`Status`] owns: the LED's RMT and data pin, or nothing.
#[cfg(feature = "status-led")]
pub type StatusPins = (
    esp_hal::peripherals::RMT<'static>,
    esp_hal::peripherals::GPIO8<'static>,
);
#[cfg(not(feature = "status-led"))]
pub struct StatusPins;

#[cfg(feature = "status-led")]
fn status((rmt, pin): StatusPins) -> Status {
    let led = product::status_led::StatusLed::new(rmt, pin)
        .map_err(|error| log::warn!("[ESP32-C6] status LED unavailable: {:?}", error))
        .ok();
    product::status_led::RgbStatusLed::new(
        LogStatus { last: None },
        led,
        &product::status_led::STATUS_PALETTE,
    )
}

#[cfg(not(feature = "status-led"))]
fn status(_: StatusPins) -> Status {
    LogStatus { last: None }
}

pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[ESP32-C6] commissioning attempt {}", attempt)
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[ESP32-C6] commissioning/resume failed: {:?}", error)
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[ESP32-C6] retry {} in {} ms", attempt, delay_ms)
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[ESP32-C6] online: addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address,
                channel,
                pan_id
            ),
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildrenRestored { count } => {
                log::info!("[ESP32-C6] {} children restored", count)
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildTableDiscarded { error } => {
                log::warn!("[ESP32-C6] child table discarded: {:?}", error)
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildTableSaved => log::info!("[ESP32-C6] child table saved"),
            #[cfg(feature = "router")]
            DiagnosticEvent::ChildRemovalCompleted {
                short_address,
                delivered,
                ..
            } => log::info!(
                "[ESP32-C6] child 0x{:04X} removed (delivered={})",
                short_address,
                delivered
            ),
            #[cfg(feature = "router")]
            DiagnosticEvent::ApsTablesRestored { count } => {
                log::info!("[ESP32-C6] {} APS table entries restored", count)
            }
            #[cfg(feature = "router")]
            DiagnosticEvent::ApsTablesDiscarded { error } => {
                log::warn!("[ESP32-C6] APS tables discarded: {:?}", error)
            }
            DiagnosticEvent::StackEvent(StackEventSummary::PermitJoinChanged { open }) => {
                log::info!("[ESP32-C6] permit join {}", open)
            }
            DiagnosticEvent::StackEvent(StackEventSummary::DeviceAnnounced {
                address,
                short_address,
                ..
            }) => log::info!(
                "[ESP32-C6] device announce 0x{:04X} {:02X?}",
                short_address,
                address
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[ESP32-C6] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[ESP32-C6] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[ESP32-C6] factory-new after Leave; press BOOT or wait {} s",
                product::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[ESP32-C6] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[ESP32-C6] secure rejoin ok: addr=0x{:04X}", short_address)
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => log::warn!(
                "[ESP32-C6] secure rejoin failed ({}): {:?}",
                failures,
                error
            ),
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[ESP32-C6] secure rejoin limit reached at {}", failures)
            }
            DiagnosticEvent::FactoryReset => log::info!("[ESP32-C6] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[ESP32-C6] fatal: {:?}", error),
            _ => {}
        }
    }
}

/// No watchdog claim yet; fatal errors reset the chip.
pub struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        esp_hal::system::software_reset()
    }
}

pub type Parts = RouterParts<Status, ResetSupervisor, LogDiagnostics>;

pub fn parts(status_pins: StatusPins) -> Parts {
    RouterParts::new(status(status_pins), ResetSupervisor, LogDiagnostics)
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

/// Stop; the console keeps the reason.
pub fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
