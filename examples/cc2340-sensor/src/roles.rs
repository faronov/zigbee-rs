//! Board start-up and capability adapters shared by the mains role bins
//! (`ed.rs`, `router.rs`).
//!
//! The original sleepy sensor (`main.rs` + `platform.rs`) keeps its own
//! sequence so its bytes stay identical to the existing build.

#[cfg(any(
    all(feature = "ed", feature = "router"),
    all(feature = "ed", feature = "sed"),
    all(feature = "router", feature = "sed"),
))]
compile_error!(
    "enable exactly one role feature (sed, ed or router); use --no-default-features for ed/router"
);

use cc2340_sensor_product as product;
use embassy_time::Instant;
use lp_em_cc2340r5 as board;
use lp_em_cc2340r5::pins::{Button1, Led1};
use lp_em_cc2340r5::reset::SystemReset;
use router_app::{
    DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts, RouterStatus,
    StatusSink, Supervisor,
};
use zigbee_mac::MacError;
use zigbee_mac::cc2340::{Cc2340RadioPhy, RadioError};

/// Resources a role composition needs after the board is up.
pub struct Started {
    pub ieee: zigbee_types::IeeeAddress,
    pub flash: board::flash::InternalFlash,
    pub led: Led1,
    pub button: Button1,
    pub reset: SystemReset,
}

/// RTT, the board (DC/DC, LF clock, SYSTIM time driver), LED1, BTN1
/// (polled, no wake IRQ) and the factory EUI-64. Standby is never entered by
/// these roles. LED2, BTN2 and BATMON are dropped unused: the mains roles
/// own no sensor.
pub fn start(model: &str) -> Started {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 512,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);

    let Some(resources) = board::take() else {
        halt()
    };
    let board::Resources {
        mut led1,
        button1,
        flash,
        reset,
        ..
    } = resources;
    led1.off();
    rtt_target::rprintln!("[CC2340] {} on {}", model, board::BOARD_NAME);

    let ieee = match board::identity::factory_ieee_address() {
        Ok(address) if address != [0; 8] && address != [0xFF; 8] => address,
        _ => {
            rtt_target::rprintln!("[CC2340] no valid factory EUI-64");
            halt()
        }
    };
    rtt_target::rprintln!("[CC2340] EUI-64 {:02X?}", ieee);
    Started {
        ieee,
        flash,
        led: led1,
        button: button1,
        reset,
    }
}

/// Bring the radio up; the fallback (no TI SDK) build stops here with a
/// distinct diagnostic, like the sensor.
pub fn radio(ieee: zigbee_types::IeeeAddress) -> Cc2340RadioPhy {
    let mut phy = Cc2340RadioPhy::new(ieee).unwrap_or_else(|error| {
        rtt_target::rprintln!("[CC2340] rejected factory identity: {:?}", error);
        halt()
    });
    if let Err(error) = phy.initialize_radio() {
        match error {
            RadioError::FirmwareUnavailable => {
                rtt_target::rprintln!("[CC2340] radio firmware unavailable (fallback build)")
            }
            RadioError::RadioConfigUnavailable => {
                rtt_target::rprintln!("[CC2340] radio configuration unavailable (partial SDK)")
            }
            _ => rtt_target::rprintln!("[CC2340] radio initialization failed: {:?}", error),
        }
        halt()
    }
    phy
}

/// Halt on a MAC construction error.
pub fn mac<M>(result: Result<M, MacError>) -> M {
    result.unwrap_or_else(|error| {
        rtt_target::rprintln!("[CC2340] MAC construction failed: {:?}", error);
        halt()
    })
}

/// Log the outcome of the identity and role guards; halt on a store error.
pub fn guard_outcome<E: core::fmt::Debug>(what: &str, result: Result<bool, E>) {
    match result {
        Ok(true) => rtt_target::rprintln!("[CC2340] cleared network state: {} changed", what),
        Ok(false) => {}
        Err(error) => {
            rtt_target::rprintln!("[CC2340] {} guard failed: {:?}", what, error);
            halt()
        }
    }
}

/// Decides when a factory-new node (after Leave) starts commissioning
/// again: immediately on a BTN1 press, otherwise after the product delay.
pub struct CommissioningTrigger {
    button: Button1,
    was_pressed: bool,
    awaiting_since: Option<Instant>,
}

impl CommissioningTrigger {
    pub fn new(button: Button1) -> Self {
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

/// LED1 on while online, off otherwise; lifecycle changes on RTT.
pub struct LedStatus {
    led: Led1,
    last: Option<RouterStatus>,
}

impl StatusSink for LedStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last == Some(status) {
            return;
        }
        self.last = Some(status);
        self.led.set(matches!(
            status,
            RouterStatus::Online { .. } | RouterStatus::Fault { .. }
        ));
        rtt_target::rprintln!("[CC2340] status {:?}", status);
    }
}

/// Every lifecycle event on RTT (compact `Debug` rendering).
pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    #[inline(never)]
    fn record(&mut self, event: DiagnosticEvent) {
        rtt_target::rprintln!("[CC2340][lifecycle] {:?}", event);
    }
}

/// No watchdog is qualified on this part; fatal errors reset the MCU.
pub struct ResetSupervisor(SystemReset);

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        self.0.reset()
    }
}

pub type Parts = RouterParts<LedStatus, ResetSupervisor, LogDiagnostics>;

pub fn parts(led: Led1, reset: SystemReset) -> Parts {
    RouterParts::new(
        LedStatus { led, last: None },
        ResetSupervisor(reset),
        LogDiagnostics,
    )
}

/// Record, show and reset on an unrecoverable lifecycle error.
pub fn fatal(parts: &mut Parts, archetype: NodeArchetype, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault { archetype });
    parts.supervisor.reset()
}

/// Stop; RTT keeps the reason.
pub fn halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}
