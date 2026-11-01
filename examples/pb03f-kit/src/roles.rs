//! Board start-up and capability adapters shared by the mains role bins
//! (`bin/ed.rs`, `bin/router.rs`).
//!
//! The polling end device (`bin/zigbee`) keeps its own sequence so its bytes
//! stay comparable to the existing build. Lifecycle events go to the CH340
//! UART console, the online state to the green LED.

#[cfg(all(feature = "ed", feature = "router"))]
compile_error!("enable exactly one mains role feature (ed or router)");

use embassy_time::Instant;
use pb03f_kit::{InternalFlash, Led, Resources, UserButton, console_log};
use pb03f_kit_product as product;
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StatusSink, Supervisor,
};
use zigbee_mac::MacError;
use zigbee_mac::phy6222::Phy6222RadioPhy;

/// Resources a role composition needs after the board is up.
pub struct Started {
    pub ieee: zigbee_types::IeeeAddress,
    pub radio: Phy6222RadioPhy,
    pub flash: InternalFlash,
    pub led: Led,
    pub button: UserButton,
}

/// Time driver, console, radio interrupt, board resources and the factory
/// EUI-64. The supply monitor and the other LEDs are dropped unused: the
/// mains roles own no battery.
pub fn start(model: &str) -> Started {
    pb03f_kit::time::init();
    let Ok(Resources {
        flash,
        mut leds,
        user_button,
        console,
        ..
    }) = Resources::take()
    else {
        halt()
    };
    console_log::init(console.into_uart(), log::LevelFilter::Info);
    log::info!("{} {} {}", product::MANUFACTURER, model, product::SW_BUILD);
    leds.all_off();
    let led = leds.green;
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pb03f_kit::vectors::Interrupt::LlIrq);
    }
    let Some(radio) = Phy6222RadioPhy::take_with_config(product::radio::radio_config()) else {
        fault(led, "radio already taken")
    };
    let Ok(ieee) = product::identity::validate_ieee_address(radio.extended_address()) else {
        fault(led, "factory IEEE address missing")
    };
    Started {
        ieee,
        radio,
        flash,
        led,
        button: user_button,
    }
}

/// Halt on a MAC construction error.
pub fn mac<M>(result: Result<M, MacError>, led: &mut Led) -> M {
    match result {
        Ok(mac) => mac,
        Err(error) => {
            log::error!("fault: MAC construction {:?}", error);
            led.set_on(true);
            halt()
        }
    }
}

/// Halt on an identity or role guard store error.
pub fn guard<E>(result: Result<bool, E>, led: &mut Led) {
    if result.is_err() {
        log::error!("fault: identity/role guard store");
        led.set_on(true);
        halt()
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: immediately on a button press, otherwise after the product delay.
pub struct CommissioningTrigger {
    button: UserButton,
    was_pressed: bool,
    awaiting_since: Option<Instant>,
}

impl CommissioningTrigger {
    pub fn new(button: UserButton) -> Self {
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

/// Green LED on while online or faulted, off otherwise; status changes on
/// the console.
pub struct LedStatus {
    led: Led,
    last: Option<RouterStatus>,
}

impl StatusSink for LedStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last == Some(status) {
            return;
        }
        self.last = Some(status);
        self.led.set_on(matches!(
            status,
            RouterStatus::Online { .. } | RouterStatus::Fault { .. }
        ));
        log::info!("status {:?}", status);
    }
}

/// Every lifecycle event on the console (compact `Debug` rendering).
pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    #[inline(never)]
    fn record(&mut self, event: DiagnosticEvent) {
        log::info!("lifecycle {:?}", event);
    }
}

/// No watchdog is qualified on this part; fatal errors reset the MCU.
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
    RouterParts::new(
        LedStatus { led, last: None },
        ResetSupervisor,
        LogDiagnostics,
    )
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

fn fault(mut led: Led, reason: &str) -> ! {
    log::error!("fault: {}", reason);
    led.set_on(true);
    halt()
}

/// Stop with the radio interrupt still serviced.
pub fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
