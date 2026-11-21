//! Product policy for confirming or rolling back a freshly installed OTA
//! image.
//!
//! [`crate::ota`] implements the flash protocol: an activated image boots as
//! `ESP_OTA_IMG_NEW`, the first boot records an attempt, a second boot without
//! confirmation reselects the previous slot. This module decides *when* the
//! running image has proven itself. The bundled espflash ESP-IDF bootloader
//! is built without `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, so rollback is
//! entirely application-driven.
//!
//! # Evidence
//!
//! Only diagnostics derived from a completed over-the-air exchange confirm the
//! image immediately: a fresh join, a secured rejoin, or an inbound,
//! decrypted ZCL frame (default response, report, reporting configuration or
//! an unhandled command).
//!
//! A silent resume ([`DiagnosticEvent::JoinedOrResumed`]) restores the stored
//! parent relationship without any radio exchange, so it is weak evidence. It
//! confirms only if the image then runs for the whole
//! [`CONFIRMATION_WINDOW_MS`] without a failed rejoin, commissioning or wake.
//! A broken radio is detected by the stack as lost polls and surfaces as a
//! failed secure rejoin inside the window.
//!
//! Failure is acted on immediately:
//!
//! * [`DiagnosticEvent::SecureRejoinLimitReached`] is recorded *before* the
//!   application wipes network state and commissions as factory-new. Rolling
//!   back at that point keeps the previous image's commissioned state.
//! * A security-store failure means the new image cannot use the durable
//!   journal the previous image wrote.
//!
//! The window is measured with the monotonic Embassy clock. The application
//! services [`poll`] from the supervisor heartbeat, so the deadline is
//! observed with at most one slow-poll period of delay.

use sensor_sed_app::DiagnosticEvent;

/// How long an image that only resumed silently must run cleanly before it is
/// confirmed, and how long an image without any network evidence may run
/// before it is rolled back. Generous enough to cover several secure-rejoin
/// attempts at the product's join retry interval.
pub const CONFIRMATION_WINDOW_MS: u64 = 15 * 60 * 1_000;

/// What the caller must do with the running, unconfirmed image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// Keep waiting for evidence.
    Wait,
    /// The image has proven network operation: call [`crate::ota::confirm_boot`].
    Confirm,
    /// The image failed: call [`crate::ota::roll_back`] and reset.
    RollBack,
}

/// Classification of one diagnostic event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Evidence {
    /// A completed over-the-air exchange with the network.
    Proven,
    /// Network state restored without a radio exchange.
    Resumed,
    /// The network connection is failing; weak evidence is withdrawn.
    Degraded,
    /// The image cannot keep the previous image's network state.
    Fatal,
    Neutral,
}

fn classify(event: &DiagnosticEvent) -> Evidence {
    match event {
        DiagnosticEvent::Joined { .. }
        | DiagnosticEvent::SecureRejoinSucceeded { .. }
        | DiagnosticEvent::DefaultResponse { .. }
        | DiagnosticEvent::AttributeReport { .. }
        | DiagnosticEvent::ReportingConfigured { .. }
        | DiagnosticEvent::ReportingRejected { .. }
        | DiagnosticEvent::InterviewConfigurationComplete { .. }
        | DiagnosticEvent::UnhandledCommand { .. }
        | DiagnosticEvent::Identify { .. } => Evidence::Proven,
        DiagnosticEvent::JoinedOrResumed { .. } => Evidence::Resumed,
        DiagnosticEvent::ZigbeeInitializationFailed
        | DiagnosticEvent::CommissioningFailed { .. }
        | DiagnosticEvent::CommissioningComplete { success: false }
        | DiagnosticEvent::SecureRejoinInitializationFailed
        | DiagnosticEvent::SecureRejoinFailed { .. }
        | DiagnosticEvent::SecureRejoinPending { .. }
        | DiagnosticEvent::Left
        | DiagnosticEvent::WakeFailed => Evidence::Degraded,
        DiagnosticEvent::SecureRejoinLimitReached { .. } | DiagnosticEvent::SecurityFailure(_) => {
            Evidence::Fatal
        }
        _ => Evidence::Neutral,
    }
}

/// Pending-verification state for one boot of an unconfirmed image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootVerification {
    deadline_ms: u64,
    resumed: bool,
}

impl BootVerification {
    /// Start the confirmation window at `now_ms`.
    pub const fn new(now_ms: u64) -> Self {
        Self {
            deadline_ms: now_ms.saturating_add(CONFIRMATION_WINDOW_MS),
            resumed: false,
        }
    }

    /// Account for one diagnostic event.
    pub fn observe(&mut self, event: &DiagnosticEvent) -> Verdict {
        match classify(event) {
            Evidence::Proven => Verdict::Confirm,
            Evidence::Resumed => {
                self.resumed = true;
                Verdict::Wait
            }
            Evidence::Degraded => {
                self.resumed = false;
                Verdict::Wait
            }
            Evidence::Fatal => Verdict::RollBack,
            Evidence::Neutral => Verdict::Wait,
        }
    }

    /// Check the confirmation deadline.
    pub fn poll(&self, now_ms: u64) -> Verdict {
        if now_ms < self.deadline_ms {
            Verdict::Wait
        } else if self.resumed {
            Verdict::Confirm
        } else {
            Verdict::RollBack
        }
    }
}

#[cfg(target_os = "none")]
mod hardware {
    use core::cell::Cell;

    use critical_section::Mutex;
    use sensor_sed_app::DiagnosticEvent;

    use super::{BootVerification, Verdict};
    use crate::ota::{self, BootCheck, BootCheckError, EspOtaFlash};

    static PENDING: Mutex<Cell<Option<BootVerification>>> = Mutex::new(Cell::new(None));

    fn now_ms() -> u64 {
        embassy_time::Instant::now().as_millis()
    }

    /// Run the once-per-boot `otadata` check and arm the confirmation window
    /// for an unconfirmed image. Call before the Zigbee stack starts.
    /// [`BootCheck::RolledBack`] requires an immediate reset.
    ///
    /// An error leaves rollback disarmed for this boot: without running-image
    /// evidence or a valid partition table no slot can be selected safely,
    /// and OTA initialization reports the same fault.
    pub fn begin() -> Result<BootCheck, BootCheckError> {
        let check = ota::check_boot(&mut EspOtaFlash::new())?;
        if let BootCheck::PendingVerify { .. } = check {
            let verification = BootVerification::new(now_ms());
            critical_section::with(|cs| PENDING.borrow(cs).set(Some(verification)));
        }
        Ok(check)
    }

    /// Feed one diagnostic event to the pending verification, if any.
    pub fn observe(event: &DiagnosticEvent) {
        let verdict = critical_section::with(|cs| {
            let cell = PENDING.borrow(cs);
            let mut pending = cell.get()?;
            let verdict = pending.observe(event);
            cell.set(Some(pending));
            Some(verdict)
        });
        if let Some(verdict) = verdict {
            act(verdict);
        }
    }

    /// Enforce the confirmation deadline. Call from the supervisor heartbeat.
    pub fn poll() {
        let verdict = critical_section::with(|cs| PENDING.borrow(cs).get())
            .map(|pending| pending.poll(now_ms()));
        if let Some(verdict) = verdict {
            act(verdict);
        }
    }

    fn act(verdict: Verdict) {
        match verdict {
            Verdict::Wait => {}
            Verdict::Confirm => match ota::confirm_boot(&mut EspOtaFlash::new()) {
                Ok(_) => {
                    critical_section::with(|cs| PENDING.borrow(cs).set(None));
                    log::info!("[ESP OTA] running image confirmed");
                }
                // Stay pending: the next evidence retries the confirmation.
                Err(error) => log::error!("[ESP OTA] confirmation failed: {:?}", error),
            },
            Verdict::RollBack => {
                match ota::roll_back(&mut EspOtaFlash::new()) {
                    Ok(slot) => log::warn!("[ESP OTA] rolling back to slot {:?}", slot),
                    // The boot-attempt mark is already programmed, so the
                    // next boot's check rolls back if this write failed.
                    Err(error) => log::error!("[ESP OTA] rollback failed: {:?}", error),
                }
                esp_hal::system::software_reset();
            }
        }
    }
}

#[cfg(target_os = "none")]
pub use hardware::{begin, observe, poll};

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::security_store::SecurityStoreError;

    const JOINED_OR_RESUMED: DiagnosticEvent = DiagnosticEvent::JoinedOrResumed {
        short_address: 0x1234,
        channel: 15,
        pan_id: 0x1A62,
    };

    #[test]
    fn over_the_air_evidence_confirms_immediately() {
        for event in [
            DiagnosticEvent::Joined {
                short_address: 1,
                channel: 11,
                pan_id: 2,
            },
            DiagnosticEvent::SecureRejoinSucceeded { short_address: 1 },
            DiagnosticEvent::DefaultResponse {
                src_addr: 0,
                cluster_id: 0x0402,
                command_id: 0x0A,
                status: 0,
            },
            DiagnosticEvent::InterviewConfigurationComplete {
                configured: 3,
                expected: 3,
            },
        ] {
            let mut verification = BootVerification::new(0);
            assert_eq!(verification.observe(&event), Verdict::Confirm, "{event:?}");
        }
    }

    #[test]
    fn silent_resume_confirms_only_after_a_clean_window() {
        let mut verification = BootVerification::new(1_000);
        assert_eq!(verification.observe(&JOINED_OR_RESUMED), Verdict::Wait);
        assert_eq!(
            verification.observe(&DiagnosticEvent::ReportSent),
            Verdict::Wait
        );
        assert_eq!(
            verification.poll(1_000 + CONFIRMATION_WINDOW_MS - 1),
            Verdict::Wait
        );
        assert_eq!(
            verification.poll(1_000 + CONFIRMATION_WINDOW_MS),
            Verdict::Confirm
        );
    }

    #[test]
    fn rejoin_failures_withdraw_resume_evidence() {
        let mut verification = BootVerification::new(0);
        verification.observe(&JOINED_OR_RESUMED);
        assert_eq!(
            verification.observe(&DiagnosticEvent::SecureRejoinPending { failures: 1 }),
            Verdict::Wait
        );
        assert_eq!(verification.poll(CONFIRMATION_WINDOW_MS), Verdict::RollBack);

        // A later successful resume restores it; a secured rejoin confirms.
        verification.observe(&JOINED_OR_RESUMED);
        assert_eq!(verification.poll(CONFIRMATION_WINDOW_MS), Verdict::Confirm);
    }

    #[test]
    fn no_network_by_the_deadline_rolls_back() {
        let verification = BootVerification::new(5);
        assert_eq!(verification.poll(4 + CONFIRMATION_WINDOW_MS), Verdict::Wait);
        assert_eq!(
            verification.poll(5 + CONFIRMATION_WINDOW_MS),
            Verdict::RollBack
        );
        assert_eq!(
            BootVerification::new(u64::MAX).poll(u64::MAX),
            Verdict::RollBack
        );
    }

    #[test]
    fn state_destroying_failures_roll_back_before_the_factory_reset() {
        for event in [
            DiagnosticEvent::SecureRejoinLimitReached { failures: 3 },
            DiagnosticEvent::SecurityFailure(SecurityStoreError::Hardware),
        ] {
            let mut verification = BootVerification::new(0);
            verification.observe(&JOINED_OR_RESUMED);
            assert_eq!(verification.observe(&event), Verdict::RollBack, "{event:?}");
        }
    }
}
