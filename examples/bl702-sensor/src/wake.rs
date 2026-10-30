//! BL702 wake controller: radio-quiesced CPU idle between parent polls.
//!
//! `SleepDepth::Idle` disables M154 receive, waits for the M154 state machine
//! to report idle, and then awaits an Embassy timer. With no other work ready
//! the executor executes WFI until the TIMER channel 0 comparator interrupt
//! (see `time_driver`) wakes it. Clocks, the RF analog block, PLL and per-die
//! calibration stay powered, so the radio needs no restoration: the next MAC
//! RX/TX starts from the same idle state it uses between ordinary operations
//! (lazy restoration per the `WakeController` contract).
//!
//! `SleepDepth::Retention` (PDS/HBN) is rejected before any state changes;
//! `bl702_hal::pm` keeps refusing it until restore is hardware-proven.

use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{SleepDepth, WaitRequest, WakeController, WakeReason};
use zigbee_mac::bl702::radio_phy::Bl702SoftMac;

/// Emit one diagnostic summary every this many idle waits.
const IDLE_LOG_INTERVAL: u32 = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    RetentionUnsupported,
}

#[derive(Debug, Default)]
pub struct Bl702Wake {
    idle_waits: u32,
    radio_vetoes: u32,
    idle_ms: u64,
}

impl Bl702Wake {
    pub const fn new() -> Self {
        Self {
            idle_waits: 0,
            radio_vetoes: 0,
            idle_ms: 0,
        }
    }

    fn record_idle(&mut self, slept_ms: u64) {
        self.idle_waits = self.idle_waits.wrapping_add(1);
        self.idle_ms = self.idle_ms.saturating_add(slept_ms);
        if self.idle_waits == 1 || self.idle_waits.is_multiple_of(IDLE_LOG_INTERVAL) {
            log::info!(
                "idle sleep: waits={}, idle_ms={}, radio_vetoes={}, alarm_irqs={}, wfi={}",
                self.idle_waits,
                self.idle_ms,
                self.radio_vetoes,
                crate::time_driver::alarm_interrupts(),
                crate::time_driver::idle_sleep_available()
            );
        }
    }
}

impl WakeController<Bl702SoftMac> for Bl702Wake {
    type Mark = Instant;
    type Error = WakeError;

    fn mark(&self) -> Self::Mark {
        Instant::now()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark + Duration::from_millis(u64::from(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later
            .saturating_duration_since(earlier)
            .as_millis()
            .min(u64::from(u32::MAX)) as u32
    }

    async fn wait(
        &mut self,
        mac: &mut Bl702SoftMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        let timeout = Duration::from_millis(u64::from(request.timeout_ms));
        match request.sleep_depth {
            SleepDepth::Active => {
                Timer::after(timeout).await;
                Ok(WakeReason::Timer)
            }
            SleepDepth::Idle => {
                if !mac.phy_mut().quiesce() {
                    // The MAC remains usable; let the application service the
                    // radio instead of sleeping (contract-permitted veto).
                    self.radio_vetoes = self.radio_vetoes.wrapping_add(1);
                    log::warn!("idle sleep vetoed: M154 radio not idle");
                    return Ok(WakeReason::Activity);
                }
                let started = Instant::now();
                Timer::after(timeout).await;
                self.record_idle(started.elapsed().as_millis());
                Ok(WakeReason::Timer)
            }
            // Reject before timer registration or radio state changes.
            SleepDepth::Retention => Err(WakeError::RetentionUnsupported),
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        // No fitted user button is owned by this product.
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}
