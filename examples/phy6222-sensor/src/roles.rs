//! Board start-up and capability adapters shared by the mains role bins
//! (`ed.rs`, `router.rs`).
//!
//! The original sleepy sensor (`main.rs` + `platform.rs`) keeps its own
//! sequence so its bytes stay comparable to the existing build. The EVK has
//! no console in production builds, so the adapters report through the
//! status LED only.

#[cfg(any(
    all(feature = "ed", feature = "router"),
    all(feature = "ed", feature = "sed"),
    all(feature = "router", feature = "sed"),
))]
compile_error!(
    "enable exactly one role feature (sed, ed or router); use --no-default-features for ed/router"
);

use embassy_time::Instant;
use phy62x2_evk::{InternalFlash, Resources, StatusLed, UserButton};
use phy62x2_evk_product as product;
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
    pub led: StatusLed,
    pub button: UserButton,
}

/// Time driver, radio interrupt, board resources and the factory EUI-64.
/// The supply monitor and the sensor I2C bus are dropped unused: the mains
/// roles own no sensor and no battery.
pub fn start() -> Started {
    phy62x2_evk::time::init();
    unsafe {
        cortex_m::peripheral::NVIC::unmask(phy62x2_evk::vectors::Interrupt::LlIrq);
    }
    let Ok(Resources {
        flash,
        status_led,
        user_button,
        ..
    }) = Resources::take()
    else {
        halt()
    };
    let mut led = status_led;
    led.set_on(false);
    let Some(radio) = Phy6222RadioPhy::take() else {
        fault(led)
    };
    let Ok(ieee) = product::identity::validate_ieee_address(radio.extended_address()) else {
        fault(led)
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
pub fn mac<M>(result: Result<M, MacError>, led: &mut StatusLed) -> M {
    match result {
        Ok(mac) => mac,
        Err(_) => {
            led.set_on(true);
            halt()
        }
    }
}

/// Halt on an identity or role guard store error.
pub fn guard<E>(result: Result<bool, E>, led: &mut StatusLed) {
    if result.is_err() {
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

/// Status LED on while online or faulted, off otherwise.
pub struct LedStatus {
    led: StatusLed,
}

impl StatusSink for LedStatus {
    fn set(&mut self, status: RouterStatus) {
        self.led.set_on(matches!(
            status,
            RouterStatus::Online { .. } | RouterStatus::Fault { .. }
        ));
    }
}

/// No console on the EVK production build.
pub struct NoDiagnostics;

impl Diagnostics for NoDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
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

pub type Parts = RouterParts<LedStatus, ResetSupervisor, NoDiagnostics>;

pub fn parts(led: StatusLed) -> Parts {
    RouterParts::new(LedStatus { led }, ResetSupervisor, NoDiagnostics)
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

fn fault(mut led: StatusLed) -> ! {
    led.set_on(true);
    halt()
}

/// Stop with the radio interrupt still serviced.
pub fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
