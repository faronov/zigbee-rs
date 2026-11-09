//! Reusable Zigbee OTA client lifecycle for products that stage images for
//! a bootloader through a [`FirmwareWriter`] (feature `ota`).
//!
//! [`ImageOtaClient`] queries the OTA server on a [`QuerySchedule`] while
//! joined, answers Image Notify, accepts an offer only when the product's
//! channel can install images, downloads through the
//! [`OtaSession`]/[`OtaManager`](zigbee_runtime::ota::OtaManager) of a
//! [`WithOta`] profile, activates the verified image after `SensorApp` has
//! checkpointed the security state and then resets into the bootloader.
//! [`OtaLifecycle::confirm_image`] confirms an installed image on trial
//! through the writer (the trial log of a zigbee-rs loader).
//!
//! The product owns the identity, the writer and its flash view, the reset
//! function and whether the installed loader can take images; this module
//! owns only the platform-independent sequencing.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::FirmwareWriter;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::ota_transport::{OtaEventOutcome as RuntimeOtaEventOutcome, OtaSession};
use zigbee_runtime::profile::{ApplicationProfile, WithOta};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::ota::{
    OtaActivationOutcome, OtaEventOutcome, OtaLifecycle, OtaServiceOutcome, is_ota_event,
};

/// Product-selected client timing and server.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaClientPolicy {
    /// Endpoint of the OTA Upgrade client cluster.
    pub endpoint: u8,
    /// `(short address, endpoint)` of the OTA server.
    pub server: (u16, u8),
    /// Joined seconds before the first Query Next Image.
    pub first_query_delay_secs: u32,
    /// Joined seconds between later queries.
    pub query_interval_secs: u32,
}

impl OtaClientPolicy {
    /// The coordinator's endpoint 1 (Zigbee2MQTT, ZHA), first query five
    /// minutes after joining, then daily.
    pub const fn coordinator(endpoint: u8) -> Self {
        Self {
            endpoint,
            server: (0x0000, 1),
            first_query_delay_secs: 300,
            query_interval_secs: 24 * 60 * 60,
        }
    }
}

/// Client-initiated Query Next Image schedule, in joined seconds (the
/// lifecycle only services OTA while joined).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuerySchedule {
    remaining_secs: u32,
    interval_secs: u32,
}

impl QuerySchedule {
    pub const fn new(policy: &OtaClientPolicy) -> Self {
        Self {
            remaining_secs: policy.first_query_delay_secs,
            interval_secs: policy.query_interval_secs,
        }
    }

    /// Advance by `elapsed_secs`; `true` when a query is due, which restarts
    /// the interval.
    pub fn advance(&mut self, elapsed_secs: u16) -> bool {
        self.remaining_secs = self.remaining_secs.saturating_sub(u32::from(elapsed_secs));
        if self.remaining_secs == 0 {
            self.remaining_secs = self.interval_secs.max(1);
            true
        } else {
            false
        }
    }

    pub const fn remaining_secs(&self) -> u32 {
        self.remaining_secs
    }
}

/// OTA lifecycle for [`SensorApp`](crate::SensorApp) over any
/// [`WithOta`] profile: periodic queries, Image Notify, the channel policy,
/// fast polling while downloading (through [`OtaLifecycle::is_active`]),
/// activation and reset, and confirmation of an image on trial.
pub struct ImageOtaClient {
    session: OtaSession,
    policy: OtaClientPolicy,
    schedule: QuerySchedule,
    /// Whether something installs staged images (for a zigbee-rs loader:
    /// it trusts one of the application's keys). Otherwise offers are
    /// declined and no query is sent.
    accepts_images: bool,
    /// Resets into the bootloader once the upgrade request is written.
    reset: fn() -> !,
}

impl ImageOtaClient {
    pub const fn new(policy: OtaClientPolicy, accepts_images: bool, reset: fn() -> !) -> Self {
        Self {
            session: OtaSession::new(),
            schedule: QuerySchedule::new(&policy),
            policy,
            accepts_images,
            reset,
        }
    }

    pub const fn accepts_images(&self) -> bool {
        self.accepts_images
    }

    pub const fn schedule(&self) -> &QuerySchedule {
        &self.schedule
    }

    fn log_status(status: Option<&StackEvent>) {
        match status {
            Some(StackEvent::OtaImageAvailable { version, size }) => {
                log::info!("[ota] offer version=0x{version:08X} size={size}");
            }
            Some(StackEvent::OtaProgress { percent }) => log::info!("[ota] {percent}%"),
            Some(StackEvent::OtaDelayedActivation { delay_secs }) => {
                log::info!("[ota] activation delayed {delay_secs}s");
            }
            Some(StackEvent::OtaComplete) => log::info!("[ota] verified"),
            Some(StackEvent::OtaFailed) => log::warn!("[ota] failed"),
            _ => {}
        }
    }
}

impl<M, S, P, W> OtaLifecycle<M, S, WithOta<P, W>> for ImageOtaClient
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    W: FirmwareWriter,
{
    const ENABLED: bool = true;

    fn is_active(&self, profile: &WithOta<P, W>) -> bool {
        OtaSession::is_active(profile.ota())
    }

    fn next_deadline_ms(&self, _profile: &WithOta<P, W>) -> Option<u32> {
        // The query schedule and the manager count whole seconds of the
        // lifecycle's service calls; while a session is active SensorApp
        // already uses the fast-poll cadence.
        None
    }

    async fn handle_event(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, W>>,
        event: &StackEvent,
    ) -> OtaEventOutcome {
        let (device, profile) = node.device_and_profile_mut();
        let manager = profile.ota_mut();
        let outcome = self
            .session
            .handle_event(device, manager, self.policy.endpoint, event)
            .await;
        match outcome {
            RuntimeOtaEventOutcome::NotOta if !is_ota_event(event) => {
                return OtaEventOutcome::NotHandled;
            }
            // A status event the transport itself raised earlier.
            RuntimeOtaEventOutcome::NotOta => Self::log_status(Some(event)),
            RuntimeOtaEventOutcome::Ignored => {}
            RuntimeOtaEventOutcome::Consumed(status) => Self::log_status(status.as_ref()),
        }

        if self.session.offer_pending(manager) {
            if self.accepts_images {
                let status = self.session.accept_offer(device, manager).await;
                Self::log_status(status.as_ref());
            } else {
                log::warn!("[ota] offer declined: no loader trusts this build's keys");
                self.session.decline_offer(manager);
            }
        }

        OtaEventOutcome::Handled {
            activation_pending: self.session.activation_pending()
                || matches!(event, StackEvent::OtaComplete),
        }
    }

    async fn service(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, W>>,
        elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        let (device, profile) = node.device_and_profile_mut();
        let manager = profile.ota_mut();
        let mut queried = false;
        if self.schedule.advance(elapsed_secs) && self.accepts_images {
            log::info!("[ota] query next image");
            self.session
                .query_next_image(device, manager, self.policy.server)
                .await;
            queried = OtaSession::is_active(manager);
        }
        let status = self.session.service(device, manager, elapsed_secs).await;
        Self::log_status(status.as_ref());

        OtaServiceOutcome {
            progressed: queried || status.is_some(),
            activation_pending: self.session.activation_pending(),
        }
    }

    fn activate(&mut self, node: &mut ZigbeeNode<'_, M, S, WithOta<P, W>>) -> OtaActivationOutcome {
        // SensorApp has just checkpointed the security state. The writer
        // writes the upgrade request; the bootloader installs on reset.
        let manager = node.profile_mut().ota_mut();
        match self.session.activate(manager) {
            Ok(()) => {
                log::info!("[ota] activated, resetting into the bootloader");
                (self.reset)()
            }
            Err(error) => {
                log::warn!("[ota] activation failed: {error:?}");
                OtaActivationOutcome::Failed
            }
        }
    }

    fn abort(&mut self, node: &mut ZigbeeNode<'_, M, S, WithOta<P, W>>) {
        // The shared keep-awake bound gave up on this session: return the
        // manager to idle (a suspended download stays resumable) and drop
        // any half-finished activation request.
        node.profile_mut().ota_mut().end_session();
        self.session = OtaSession::new();
    }

    fn confirm_image(&mut self, node: &mut ZigbeeNode<'_, M, S, WithOta<P, W>>) -> bool {
        // One write into the primary slot's trial log while the installed
        // image is on trial; nothing for a confirmed or factory image.
        match node.profile_mut().ota_mut().writer_mut().confirm_image() {
            Ok(()) => {
                log::info!("[ota] image confirmed");
                true
            }
            Err(error) => {
                log::warn!("[ota] confirm failed: {error:?}");
                false
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    #[test]
    fn queries_after_the_first_delay_then_on_the_interval() {
        let policy = OtaClientPolicy::coordinator(1);
        let mut schedule = QuerySchedule::new(&policy);
        let mut due_at = Vec::new();
        let mut now = 0u32;
        for step in [30u16, 1, 250, 19, 7, 3_600, 30].iter().cycle().take(200) {
            now += u32::from(*step);
            if schedule.advance(*step) {
                due_at.push(now);
            }
        }
        assert_eq!(due_at[0], 300, "first query once 300 s have elapsed");
        for pair in due_at.windows(2) {
            let gap = pair[1] - pair[0];
            assert!(
                (policy.query_interval_secs..policy.query_interval_secs + 3_600).contains(&gap)
            );
        }
        let mut schedule = QuerySchedule::new(&policy);
        assert!(schedule.advance(u16::MAX));
        assert!(!schedule.advance(1));
    }

    #[test]
    fn zero_interval_does_not_query_every_call() {
        let policy = OtaClientPolicy {
            first_query_delay_secs: 0,
            query_interval_secs: 0,
            ..OtaClientPolicy::coordinator(1)
        };
        let mut schedule = QuerySchedule::new(&policy);
        assert!(schedule.advance(0));
        assert!(!schedule.advance(0));
        assert!(schedule.advance(1));
    }
}
