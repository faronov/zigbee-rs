//! Application-facing transport for the shared ESP32 OTA backend.
//!
//! ZCL decoding, request construction, retry timing and the OTA state machine
//! live in [`zigbee_runtime::ota_transport::OtaSession`]. This wrapper keeps
//! the ESP32 application loops small and gives both chips identical policy.

use zigbee_mac::MacDriver;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};
use zigbee_runtime::ota_transport::{OtaEventOutcome, OtaSession};
use zigbee_runtime::profile::OtaBackend;

use crate::ENDPOINT;

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

    /// Whether an OTA download, verification or activation wait is active.
    pub fn is_active<W: FirmwareWriter>(backend: &OtaBackend<W>) -> bool {
        OtaSession::is_active(backend.manager())
    }

    /// Feed a decoded stack event to the OTA client.
    pub async fn handle_event<M, W>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        backend: &mut OtaBackend<W>,
        event: &StackEvent,
    ) -> bool
    where
        M: MacDriver,
        W: FirmwareWriter,
    {
        let outcome = self
            .session
            .handle_event(device, backend.manager_mut(), ENDPOINT, event)
            .await;
        match outcome {
            OtaEventOutcome::NotOta => false,
            OtaEventOutcome::Ignored => true,
            OtaEventOutcome::Consumed(status) => {
                self.log_status(status);
                true
            }
        }
    }

    /// Advance timers and send any request queued by the OTA engine.
    pub async fn service<M, W>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        backend: &mut OtaBackend<W>,
        elapsed_seconds: u16,
    ) where
        M: MacDriver,
        W: FirmwareWriter,
    {
        let status = self
            .session
            .service(device, backend.manager_mut(), elapsed_seconds)
            .await;
        self.log_status(status);
    }

    /// Whether the application should checkpoint security state and activate.
    pub fn activation_pending(&self) -> bool {
        self.session.activation_pending()
    }

    /// Mark the staged slot active. A real ESP backend resets and never
    /// returns after this succeeds.
    pub fn activate<W: FirmwareWriter>(
        &mut self,
        backend: &mut OtaBackend<W>,
    ) -> Result<(), FirmwareError> {
        self.session.activate(backend.manager_mut())
    }

    fn log_status(&mut self, status: Option<StackEvent>) {
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

impl Default for OtaTransport {
    fn default() -> Self {
        Self::new()
    }
}
