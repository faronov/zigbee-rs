//! Sleep transactions behind [`sensor_sed_app::WakeController`].
//!
//! - `Active`: busy wait on Timer0.
//! - `Idle`: atomic full-SRAM timer SUSPEND. The MAC/radio is quiesced and
//!   restored inside the PM transaction, then Timer0 is rebased by the
//!   system-timer time spent asleep before IRQs resume.
//! - `Retention` (feature `retention`): LOW32K sleep. The transaction never
//!   returns on success; the image re-enters through `_rust_retention_entry`
//!   and [`crate::retention::resume`]. Without the feature it is rejected.

use sensor_sed_app::{SleepDepth, WaitRequest, WakeController, WakeReason};
use tlsr8258_hal::pm::SuspendTransactionPhase;
use zigbee_mac::MacError;
use zigbee_mac::telink::TelinkMac;

/// Timer0 active waits plus the TLSR8258 low-power transactions.
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkWake;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelinkWakeError {
    RetentionUnsupported,
    #[cfg(feature = "retention")]
    RetentionContextInvalid,
    PowerManagement(tlsr8258_hal::pm::PmError),
    LowPowerNotEntered,
    TimerWakeMissing,
    /// The radio could not be restored from the retained PIB after wake.
    RadioResume(MacError),
}

impl From<tlsr8258_hal::pm::PmError> for TelinkWakeError {
    fn from(error: tlsr8258_hal::pm::PmError) -> Self {
        Self::PowerManagement(error)
    }
}

/// MAC quiesce/restore and monotonic-time rebase shared by both PM
/// transactions. The lifecycle has completed its service iteration before
/// `wait`, so the MAC is idle at `Prepare`.
struct SleepClock {
    timer0_before: u32,
    system_before: u32,
    resumed: Result<(), MacError>,
}

impl SleepClock {
    const fn new() -> Self {
        Self {
            timer0_before: 0,
            system_before: 0,
            resumed: Ok(()),
        }
    }

    fn transition(&mut self, mac: &mut TelinkMac, phase: SuspendTransactionPhase) {
        match phase {
            SuspendTransactionPhase::Prepare => {
                self.timer0_before = tlsr8258_hal::timer::now_ticks();
                self.system_before = tlsr8258_hal::pm::system_timer_ticks();
                mac.prepare_for_sleep();
            }
            SuspendTransactionPhase::Restore => {
                // Radio init resets Timer0: restore the MAC first, then rebase
                // monotonic time. IRQs resume only after this returns.
                self.resumed = mac.resume_after_sleep();
                let elapsed =
                    tlsr8258_hal::pm::system_timer_ticks().wrapping_sub(self.system_before);
                tlsr8258_hal::timer::rebase_after_suspend(self.timer0_before, elapsed);
            }
        }
    }
}

fn suspend(mac: &mut TelinkMac, timeout_ms: u32) -> Result<(), TelinkWakeError> {
    let mut clock = SleepClock::new();
    let status = tlsr8258_hal::pm::cpu_suspend_timer_rc_transaction(
        tlsr8258_hal::pm::system_timer_ms(timeout_ms),
        |phase| clock.transition(mac, phase),
    );
    clock.resumed.map_err(TelinkWakeError::RadioResume)?;
    let status = status?;
    if !status.entered_low_power() {
        return Err(TelinkWakeError::LowPowerNotEntered);
    }
    if !status.woke_by_timer() {
        return Err(TelinkWakeError::TimerWakeMissing);
    }
    Ok(())
}

#[cfg(feature = "retention")]
fn retention_sleep(mac: &mut TelinkMac, timeout_ms: u32) -> Result<(), TelinkWakeError> {
    let record = crate::retention::record_for(mac)?;
    let mut clock = SleepClock::new();
    let result = tlsr8258_hal::pm::cpu_sleep_timer_rc_retention_transaction(
        tlsr8258_hal::pm::system_timer_ms(timeout_ms),
        record,
        |phase| clock.transition(mac, phase),
    );
    clock.resumed.map_err(TelinkWakeError::RadioResume)?;
    match result {
        Ok(never) => match never {},
        Err(error) => Err(TelinkWakeError::PowerManagement(error)),
    }
}

impl WakeController<TelinkMac> for TelinkWake {
    type Mark = u32;
    type Error = TelinkWakeError;

    fn mark(&self) -> Self::Mark {
        tlsr8258_hal::timer::now_ticks()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark.wrapping_add(tlsr8258_hal::timer::ms(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later.wrapping_sub(earlier) / tlsr8258_hal::timer::TICKS_PER_MS
    }

    async fn wait(
        &mut self,
        mac: &mut TelinkMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {
                tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(request.timeout_ms));
            }
            SleepDepth::Idle => suspend(mac, request.timeout_ms)?,
            #[cfg(feature = "retention")]
            SleepDepth::Retention => retention_sleep(mac, request.timeout_ms)?,
            #[cfg(not(feature = "retention"))]
            SleepDepth::Retention => return Err(TelinkWakeError::RetentionUnsupported),
        }
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(duration_ms));
    }
}
