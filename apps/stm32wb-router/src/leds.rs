//! Router status on three discrete red/green/blue LEDs (Nucleo-WB55, USB
//! dongle).
//!
//! The router loop publishes lifecycle changes ([`publish_status`], the
//! `publish` hook of [`crate::platform::parts`]) and the permit-join state
//! ([`publish_permit_join`]); [`run`] maps them with the shared
//! [`router_app::StatusIndicator`] and the product palette
//! ([`STATUS_PALETTE`]) to an LED combination
//! ([`DiscreteLeds::from_rgb`]) every [`TICK_MS`]. Identify is read from
//! the status, so the LEDs blink while the Identify cluster runs.

use core::cell::Cell;

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::{Instant, Timer};
use router_app::{RouterStatus, StatusIndicator};
use stm32wb_hal::embassy_stm32::gpio::Output;
use stm32wb_router_product::indication::{DiscreteLeds, STATUS_PALETTE};

/// Indicator update period.
pub const TICK_MS: u64 = 50;

static STATUS: Mutex<CriticalSectionRawMutex, Cell<Option<RouterStatus>>> =
    Mutex::new(Cell::new(None));
static PERMIT_JOIN: Mutex<CriticalSectionRawMutex, Cell<bool>> = Mutex::new(Cell::new(false));

/// Record a lifecycle change for [`run`].
pub fn publish_status(status: RouterStatus) {
    STATUS.lock(|cell| cell.set(Some(status)));
}

/// Record whether this router is open for joining.
pub fn publish_permit_join(open: bool) {
    PERMIT_JOIN.lock(|cell| cell.set(open));
}

/// The three status LEDs; each pin is already an output driven to "off".
pub struct StatusLeds {
    pub red: Output<'static>,
    pub green: Output<'static>,
    pub blue: Output<'static>,
    /// Whether a high level lights the LEDs.
    pub active_high: bool,
}

impl StatusLeds {
    fn show(&mut self, leds: DiscreteLeds) {
        let active_high = self.active_high;
        for (pin, on) in [
            (&mut self.red, leds.red),
            (&mut self.green, leds.green),
            (&mut self.blue, leds.blue),
        ] {
            if on == active_high {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
    }
}

/// Drive `leds` from the published state forever; run it in its own task.
pub async fn run(mut leds: StatusLeds) -> ! {
    let now_ms = || Instant::now().as_millis() as u32;
    let mut indicator = StatusIndicator::new(&STATUS_PALETTE, now_ms());
    let mut shown = None;
    loop {
        let now = now_ms();
        if let Some(status) = STATUS.lock(Cell::get) {
            indicator.set_status(status, now);
        }
        indicator.set_permit_join(PERMIT_JOIN.lock(Cell::get));
        let combination = DiscreteLeds::from_rgb(indicator.update(now));
        if shown != Some(combination) {
            leds.show(combination);
            shown = Some(combination);
        }
        Timer::after_millis(TICK_MS).await;
    }
}
