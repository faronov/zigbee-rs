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

use efr32mg1_hal::peripherals::{Crypto, Peripherals};
use efr32mg1_tradfri::resources::BoardResources;
use efr32mg1_tradfri::{Button, Led};
use efr32mg1_tradfri_product as product;
use embassy_time::Instant;
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StatusSink, Supervisor,
};

/// Resources a role composition needs after clocks and time are running.
pub struct Started {
    pub crypto: Crypto,
    pub flash: efr32mg1_hal::flash::Efr32mg1Flash,
    pub led: Led,
    pub button: Button,
}

/// RTT, PA0 LED, PB13 button (polled, no wake IRQ), HFXO and the RTCC time
/// driver. EM2 is never entered by these roles. The external flash, sensor
/// I2C and supply ADC tokens are dropped unused: the mains roles own no
/// sensor and no OTA path.
pub fn start(model: &str) -> Started {
    let board = BoardResources::take().unwrap_or_else(|| halt());
    let chip = Peripherals::take().unwrap_or_else(|| halt());

    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 256,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);

    let led = board.pa0.into_led();
    led.off();
    let button = board.button.into_button();
    if efr32mg1_tradfri::init_clocks().is_err() {
        rtt_target::rprintln!("[EFR32] HFXO start failed");
        halt()
    }
    crate::time_driver::init();

    rtt_target::rprintln!("[EFR32] {} on TRADFRI EFR32MG1P", model);
    Started {
        crypto: chip.crypto,
        flash: chip.flash,
        led,
        button,
    }
}

/// Reject an all-zero/all-ones EUI-64 before it reaches the stack.
pub fn check_ieee(ieee: [u8; 8]) {
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        rtt_target::rprintln!("[EFR32] no valid IEEE EUI-64");
        halt()
    }
    rtt_target::rprintln!("[EFR32] EUI-64 {:02X?}", ieee);
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => rtt_target::rprintln!("[EFR32] cleared network state: {} changed", what),
        Ok(false) => {}
        Err(error) => {
            rtt_target::rprintln!("[EFR32] {} guard failed: {:?}", what, error);
            halt()
        }
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: immediately on a button press, otherwise after the product delay.
pub struct CommissioningTrigger {
    button: Button,
    was_pressed: bool,
    awaiting_since: Option<Instant>,
}

impl CommissioningTrigger {
    pub fn new(button: Button) -> Self {
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

/// PA0 LED on while online, off otherwise; lifecycle changes on RTT.
pub struct LedStatus {
    led: Led,
    last: Option<RouterStatus>,
}

impl LedStatus {
    pub fn new(led: Led) -> Self {
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
        rtt_target::rprintln!("[EFR32] status {:?}", status);
    }
}

/// Every lifecycle event on RTT (compact `Debug` rendering).
pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    #[inline(never)]
    fn record(&mut self, event: DiagnosticEvent) {
        rtt_target::rprintln!("[EFR32][lifecycle] {:?}", event);
    }
}

/// No watchdog is fitted; fatal errors reset the MCU.
pub struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

pub type Parts = RouterParts<LedStatus, ResetSupervisor, LogDiagnostics>;

pub fn parts(led: Led) -> Parts {
    RouterParts::new(LedStatus::new(led), ResetSupervisor, LogDiagnostics)
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

/// Stop with the PA0 LED lit; RTT keeps the reason.
pub fn halt() -> ! {
    use efr32mg1_hal::gpio::{Mode, Pin, Port};
    let pin = Pin::new(Port::A, 0);
    pin.configure(Mode::PushPull, false);
    pin.set_high();
    loop {
        cortex_m::asm::wfi();
    }
}
