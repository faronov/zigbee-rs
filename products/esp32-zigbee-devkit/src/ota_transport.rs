//! Application-facing lifecycle for the shared ESP32 OTA backend.
//!
//! ZCL decoding, request construction, retry timing and the OTA state machine
//! live in [`zigbee_runtime::ota_transport::OtaSession`]. This wrapper pairs
//! that session with the product's mandatory
//! [`WithOta`](zigbee_runtime::profile::WithOta) profile and the shared
//! sleepy-sensor application.

use sensor_sed_app::{
    OtaActivationOutcome as AppOtaActivationOutcome, OtaEventOutcome as AppOtaEventOutcome,
    OtaLifecycle, OtaServiceOutcome, is_ota_event,
};
use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::FirmwareWriter;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::ota_transport::{OtaEventOutcome as RuntimeOtaEventOutcome, OtaSession};
use zigbee_runtime::profile::{ApplicationProfile, WithOta};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::ENDPOINT;
use crate::policy::OTA_KEEP_AWAKE_MS;

/// Logging and policy wrapper around the shared [`OtaSession`] transport.
pub struct OtaTransport {
    session: OtaSession,
}

impl OtaTransport {
    /// Create an idle transport.
    pub const fn new() -> Self {
        Self {
            session: OtaSession::new(),
        }
    }

    fn log_status(&mut self, status: Option<&StackEvent>) {
        match status {
            Some(StackEvent::OtaImageAvailable { version, size }) => {
                log::info!("[ESP OTA] image 0x{:08X} ({} bytes)", version, size);
            }
            Some(StackEvent::OtaProgress { percent }) => {
                log::info!("[ESP OTA] {}%", percent);
            }
            Some(StackEvent::OtaDelayedActivation { delay_secs }) => {
                log::info!("[ESP OTA] activation in {}s", delay_secs);
            }
            Some(StackEvent::OtaFailed) => {
                log::warn!("[ESP OTA] transfer failed");
            }
            Some(StackEvent::OtaComplete) => {
                log::info!("[ESP OTA] image verified, awaiting checkpoint");
            }
            _ => {}
        }
    }
}

impl<M, S, P, F> OtaLifecycle<M, S, WithOta<P, F>> for OtaTransport
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    F: FirmwareWriter,
{
    const ENABLED: bool = true;

    fn is_active(&self, profile: &WithOta<P, F>) -> bool {
        OtaSession::is_active(profile.ota())
    }

    fn next_deadline_ms(&self, _profile: &WithOta<P, F>) -> Option<u32> {
        // OtaManager advances in whole seconds from `service`; while it is
        // active the shared SensorApp already selects the fast poll cadence.
        None
    }

    async fn handle_event(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, F>>,
        event: &StackEvent,
    ) -> AppOtaEventOutcome {
        let outcome = {
            let (device, profile) = node.device_and_profile_mut();
            self.session
                .handle_event(device, profile.ota_mut(), ENDPOINT, event)
                .await
        };

        match outcome {
            RuntimeOtaEventOutcome::NotOta if !is_ota_event(event) => {
                AppOtaEventOutcome::NotHandled
            }
            RuntimeOtaEventOutcome::NotOta => {
                // Status events normally originate inside OtaSession and are
                // logged through `Consumed`, but accepting one here keeps the
                // product lifecycle total over every OTA StackEvent.
                self.log_status(Some(event));
                AppOtaEventOutcome::Handled {
                    keep_awake_ms: Some(OTA_KEEP_AWAKE_MS),
                    activation_pending: self.session.activation_pending()
                        || matches!(event, StackEvent::OtaComplete),
                }
            }
            RuntimeOtaEventOutcome::Ignored => AppOtaEventOutcome::Handled {
                keep_awake_ms: Some(OTA_KEEP_AWAKE_MS),
                activation_pending: self.session.activation_pending(),
            },
            RuntimeOtaEventOutcome::Consumed(status) => {
                self.log_status(status.as_ref());
                AppOtaEventOutcome::Handled {
                    keep_awake_ms: Some(OTA_KEEP_AWAKE_MS),
                    activation_pending: self.session.activation_pending(),
                }
            }
        }
    }

    async fn service(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, F>>,
        elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        let status = {
            let (device, profile) = node.device_and_profile_mut();
            self.session
                .service(device, profile.ota_mut(), elapsed_secs)
                .await
        };
        self.log_status(status.as_ref());

        OtaServiceOutcome {
            keep_awake_ms: status.as_ref().map(|_| OTA_KEEP_AWAKE_MS),
            activation_pending: self.session.activation_pending(),
        }
    }

    fn activate(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, WithOta<P, F>>,
    ) -> AppOtaActivationOutcome {
        // SensorApp checkpoints its SecurityStateStore immediately before this
        // method. A successful ESP writer activation resets and does not
        // return; a returned error leaves the current boot slot authoritative.
        match self.session.activate(node.profile_mut().ota_mut()) {
            Ok(()) => AppOtaActivationOutcome::Activated,
            Err(error) => {
                log::error!("[ESP OTA] activation failed: {:?}", error);
                AppOtaActivationOutcome::Failed
            }
        }
    }
}

impl Default for OtaTransport {
    fn default() -> Self {
        Self::new()
    }
}
