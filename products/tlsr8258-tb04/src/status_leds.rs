//! TB-04 LED adapters shared by the always-on roles (router and End
//! Device) of the shared `router-app` lifecycle.
//!
//! Red/green show the online state; blue shows Identify. The supervisor
//! owns no watchdog: TB-04 production firmware has none.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use router_app::{RouterStatus, StatusSink, Supervisor};
use tlsr8258_tb04::leds::{Led, StatusLeds};

const BLUE_OFF: u8 = 0;
const BLUE_IDENTIFY: u8 = 1;

// Status and supervision deliberately own disjoint pins. These atomics carry
// only semantic LED state between their two adapters; no GPIO owner is
// duplicated and radio IRQ behavior is unaffected.
static BLUE_MODE: AtomicU8 = AtomicU8::new(BLUE_OFF);
static ONLINE_EPOCH: AtomicU32 = AtomicU32::new(0);

/// Red/green status adapter for the fitted TB-04 LEDs.
pub struct RouterLedStatus {
    red: Led,
    green: Led,
    online: bool,
}

/// Blue Identify LED service plus the product's no-watchdog supervisor.
///
/// `heartbeat` is called at least once per bounded router step, so the blue
/// LED keeps the original one-second Identify phase without adding another
/// timer or another application future.
pub struct RouterLedSupervisor {
    blue: Led,
    observed_epoch: u32,
    tick_anchor: u32,
    online_elapsed_secs: u32,
}

/// Split an already initialized board LED bundle into disjoint app adapters.
pub fn led_adapters(leds: StatusLeds) -> (RouterLedStatus, RouterLedSupervisor) {
    let StatusLeds { red, green, blue } = leds;
    BLUE_MODE.store(BLUE_OFF, Ordering::Relaxed);
    ONLINE_EPOCH.store(0, Ordering::Relaxed);
    (
        RouterLedStatus {
            red,
            green,
            online: false,
        },
        RouterLedSupervisor {
            blue,
            observed_epoch: 0,
            tick_anchor: tlsr8258_hal::timer::now_ticks(),
            online_elapsed_secs: 0,
        },
    )
}

impl RouterLedStatus {
    fn searching(&mut self) {
        self.online = false;
        BLUE_MODE.store(BLUE_OFF, Ordering::Relaxed);
        self.green.write(false);
        self.red.write(true);
    }
}

impl StatusSink for RouterLedStatus {
    fn set(&mut self, status: RouterStatus) {
        match status {
            RouterStatus::Online { identifying, .. } => {
                if !self.online {
                    let epoch = ONLINE_EPOCH.load(Ordering::Relaxed);
                    ONLINE_EPOCH.store(epoch.wrapping_add(1), Ordering::Relaxed);
                }
                self.online = true;
                BLUE_MODE.store(
                    if identifying { BLUE_IDENTIFY } else { BLUE_OFF },
                    Ordering::Relaxed,
                );
                self.red.write(false);
                self.green.write(true);
            }
            // A secured rejoin ran with the joined LED still asserted in the
            // previous firmware. Keep that indication until success or until
            // reset/recommission explicitly takes the router offline.
            RouterStatus::Rejoining { .. } if self.online => {}
            RouterStatus::Starting { .. }
            | RouterStatus::Commissioning { .. }
            | RouterStatus::Rejoining { .. }
            | RouterStatus::Recommissioning { .. }
            | RouterStatus::Resetting { .. }
            | RouterStatus::Fault { .. } => self.searching(),
        }
    }
}

impl Supervisor for RouterLedSupervisor {
    fn heartbeat(&mut self) {
        let now = tlsr8258_hal::timer::now_ticks();
        let epoch = ONLINE_EPOCH.load(Ordering::Relaxed);
        if epoch != self.observed_epoch {
            self.observed_epoch = epoch;
            self.tick_anchor = now;
            self.online_elapsed_secs = 0;
        } else {
            let one_second = tlsr8258_hal::timer::ms(1_000);
            let elapsed = now.wrapping_sub(self.tick_anchor);
            if elapsed >= one_second {
                let elapsed_secs = elapsed / one_second;
                self.tick_anchor = self
                    .tick_anchor
                    .wrapping_add(elapsed_secs.saturating_mul(one_second));
                self.online_elapsed_secs = self.online_elapsed_secs.wrapping_add(elapsed_secs);
            }
        }
        let blue_on = BLUE_MODE.load(Ordering::Relaxed) == BLUE_IDENTIFY
            && (self.online_elapsed_secs & 1) == 0;
        self.blue.write(blue_on);
    }

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        self.blue.write(false);
        loop {
            tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
        }
    }
}
