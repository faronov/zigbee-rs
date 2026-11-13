//! Start-up and capability adapters shared by the mains role bins
//! (`bin/ed.rs`, `bin/router.rs`).
//!
//! The sensor SED (`main.rs`) keeps its own sequence so its bytes stay
//! comparable to the existing build. The XT-ZB1 has no fitted LED or
//! button: status and lifecycle events go to the UART log (compiled out by
//! the default `production` feature; build with
//! `--no-default-features --features diagnostic-logging,<role>` to see
//! them), and a factory-new node re-arms commissioning on a timer.

#[cfg(all(feature = "ed", feature = "router-core"))]
compile_error!("enable exactly one mains role feature (ed or router)");
#[cfg(all(feature = "production", feature = "diagnostic-logging"))]
compile_error!("select either production or diagnostic-logging, not both");

use core::cell::RefCell;
use core::fmt::Write;

use bl702_xt_zb1_product as product;
use embassy_time::Instant;
use product::storage::MainFlash;
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StatusSink, Supervisor,
};
use zigbee_mac::bl702::radio_phy::Bl702RadioPhy;
use zigbee_mac::{MacError, MacPib};

use crate::{hal, time_driver};

struct UartWriter;

impl Write for UartWriter {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        for byte in text.bytes() {
            hal::uart_write(byte).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

struct Logger;

impl log::Log for Logger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let _ = writeln!(UartWriter, "[{}] {}\r", record.level(), record.args());
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

/// Board resources a role composition needs after start-up.
pub struct Started {
    pub ieee: [u8; 8],
    pub radio: Bl702RadioPhy,
    pub pib: MacPib,
    pub flash: &'static RefCell<MainFlash>,
}

/// Clocks, UART log, alarm interrupt; must run before the executor.
pub fn boot(model: &str) -> hal::ApplicationResources {
    // First: everything after this may be the deepest stack user.
    product::stack_guard::arm();
    let Ok(resources) = hal::init() else {
        hal::halt()
    };
    // Single hart, interrupts still disabled: the racy logger setup is safe.
    unsafe {
        let _ = log::set_logger_racy(&LOGGER);
        log::set_max_level_racy(log::LevelFilter::Info);
    }
    let _ = write!(UartWriter, "BL702 {} {}\r\n", model, product::SW_BUILD);
    if !time_driver::start() {
        log::warn!("alarm irq failed: polling executor");
    }
    resources
}

/// Radio with per-die calibration, TX power and the hardware AES engine;
/// the product IEEE address; the shared main flash. Not inlined, so the
/// calibration temporaries stay out of the task frame.
#[inline(never)]
pub fn start(resources: hal::ApplicationResources) -> Started {
    static FLASH: static_cell::StaticCell<RefCell<MainFlash>> = static_cell::StaticCell::new();

    let hal::ApplicationResources { flash, aes, .. } = resources;
    let Ok(flash) = product::storage::main_flash(flash) else {
        fault("flash")
    };
    let ieee = product::identity::ieee_address(hal::chip_id());
    let Ok(mut radio) = (unsafe { Bl702RadioPhy::initialize(hal::delay_us) }) else {
        fault("RF initialization")
    };
    if zigbee_mac::RadioPhy::set_tx_power(&mut radio, product::TX_POWER_DBM).is_err() {
        fault("TX power")
    }
    // CCM*/AES-MMO stay on SEC_ENG; the known-answer tests fail closed.
    if radio.install_aes_engine(aes).is_err() {
        fault("hardware AES self-test")
    }
    Started {
        ieee,
        radio,
        pib: MacPib::new(ieee, ieee[0], ieee[1]),
        flash: FLASH.init(RefCell::new(flash)),
    }
}

/// Halt on a MAC construction error.
pub fn mac<M>(result: Result<M, MacError>) -> M {
    match result {
        Ok(mac) => mac,
        Err(error) => {
            log::error!("fault: MAC construction {:?}", error);
            hal::halt()
        }
    }
}

/// Halt on an identity or role guard store error.
pub fn guard<E>(result: Result<bool, E>) {
    if result.is_err() {
        fault("identity/role guard store")
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: the board has no button, so after the product delay.
#[derive(Default)]
pub struct CommissioningTrigger {
    awaiting_since: Option<Instant>,
}

impl CommissioningTrigger {
    /// Call once per run-loop step; `true` means request commissioning.
    pub fn poll(&mut self, awaiting: bool) -> bool {
        if !awaiting {
            self.awaiting_since = None;
            return false;
        }
        let since = *self.awaiting_since.get_or_insert_with(Instant::now);
        if since.elapsed().as_millis() >= product::RECOMMISSION_AFTER_LEAVE_MS {
            self.awaiting_since = None;
            return true;
        }
        false
    }
}

/// Status changes on the log.
#[derive(Default)]
pub struct LogStatus {
    last: Option<RouterStatus>,
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            log::info!("status {:?}", status);
        }
    }
}

/// Every lifecycle event on the log.
pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    #[inline(never)]
    fn record(&mut self, event: DiagnosticEvent) {
        log::info!("lifecycle {:?}", event);
    }
}

/// No watchdog is qualified on this board; fatal errors re-enter the boot
/// ROM and the bootloader.
pub struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        bl702_hal::pm::software_reset()
    }
}

pub type Parts = RouterParts<LogStatus, ResetSupervisor, LogDiagnostics>;

pub fn parts() -> Parts {
    RouterParts::new(LogStatus::default(), ResetSupervisor, LogDiagnostics)
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

fn fault(reason: &str) -> ! {
    let _ = write!(UartWriter, "fault: {}\r\n", reason);
    hal::halt()
}
