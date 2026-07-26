//! OTA transport glue: routes cluster 0x0019 traffic between the Zigbee
//! runtime and the flash writer selected by the product profile.
//!
//! The product profile (`esp32_zigbee_devkit_product::profile::SensorProfile`,
//! a [`zigbee_runtime::profile::OptionalOta`]) owns the OTA cluster instance
//! and whether a firmware backend exists at all — a checked, possibly
//! incompatible partition table disables it there, never here.
//!
//! The session bookkeeping itself — remembering which server is driving the
//! upgrade so a second server cannot interleave, sending/retrying the
//! manager's queued request, and tracking when a verified image needs
//! activation — is shared with the EFR32MG1 example through
//! [`zigbee_runtime::ota_transport::OtaSession`]. What is left here is
//! purely ESP32-specific: console diagnostics, since this platform has no
//! shared logging transport wired up.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};
use zigbee_runtime::ota_transport::{OtaEventOutcome, OtaSession};
use zigbee_runtime::profile::OtaBackend;
use zigbee_runtime::ZigbeeDevice;

/// Endpoint hosting the OTA client cluster.
const OTA_ENDPOINT: u8 = 1;

/// Console-diagnostics wrapper around the shared [`OtaSession`] transport.
#[derive(Default)]
pub struct OtaTransport {
    session: OtaSession,
}

impl OtaTransport {
    pub const fn new() -> Self {
        Self {
            session: OtaSession::new(),
        }
    }

    /// Whether a verified image is waiting to be activated.
    pub fn activation_pending(&self) -> bool {
        self.session.activation_pending()
    }

    /// Whether a transfer is in flight (drives fast polling).
    pub fn is_active<F: FirmwareWriter>(backend: &OtaBackend<F>) -> bool {
        OtaSession::is_active(backend.manager())
    }

    /// Handle a stack event. Returns `true` if it was OTA Upgrade cluster
    /// traffic that this transport consumed or deliberately ignored (the
    /// caller should not keep matching on it either way).
    pub async fn handle_event<M: MacDriver, F: FirmwareWriter>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        backend: &mut OtaBackend<F>,
        event: &StackEvent,
    ) -> bool {
        let outcome = self
            .session
            .handle_event(device, backend.manager_mut(), OTA_ENDPOINT, event)
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

    /// Advance timers and flush any queued request.
    pub async fn service<M: MacDriver, F: FirmwareWriter>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        backend: &mut OtaBackend<F>,
        elapsed_secs: u16,
    ) {
        let status = self
            .session
            .service(device, backend.manager_mut(), elapsed_secs)
            .await;
        self.log_status(status);
    }

    /// Activate the verified image. Does not return on success: the chip
    /// resets into the staged slot. Call only after persisting state.
    pub fn activate<F: FirmwareWriter>(
        &mut self,
        backend: &mut OtaBackend<F>,
    ) -> Result<(), FirmwareError> {
        self.session.activate(backend.manager_mut())
    }

    fn log_status(&mut self, status: Option<StackEvent>) {
        match status {
            Some(StackEvent::OtaImageAvailable { version, size }) => {
                esp_println::println!("[ESP32-C6] OTA image 0x{:08X} ({} bytes)", version, size);
            }
            Some(StackEvent::OtaProgress { percent }) => {
                esp_println::println!("[ESP32-C6] OTA {}%", percent);
            }
            Some(StackEvent::OtaDelayedActivation { delay_secs }) => {
                esp_println::println!("[ESP32-C6] OTA activation in {}s", delay_secs);
            }
            Some(StackEvent::OtaFailed) => {
                esp_println::println!("[ESP32-C6] OTA failed");
            }
            Some(StackEvent::OtaComplete) => {
                esp_println::println!("[ESP32-C6] OTA image verified — awaiting checkpoint");
            }
            _ => {}
        }
    }
}
