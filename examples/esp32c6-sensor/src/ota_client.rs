//! OTA client wiring: routes cluster 0x0019 traffic between the Zigbee runtime
//! and the flash writer of `esp32-zigbee-devkit`.
//!
//! The runtime owns the protocol (`OtaManager` + the ZCL OTA cluster) and the
//! board crate owns the flash; what is left — and what lives here — is the
//! transport glue:
//!
//! * remembering which server is driving the upgrade, so a second server
//!   cannot interleave blocks into the same transfer,
//! * sending the request the manager queued and putting it back if the parent
//!   was not reachable (OTA requests carry their own file offset, so a resend
//!   is idempotent),
//! * deferring activation until the application has checkpointed its state, so
//!   the reset into the new image cannot lose the network keys.

use alloc::boxed::Box;

use esp32_zigbee_devkit::ota::{EspFirmwareWriter, EspOtaFlash};
use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::FirmwareError;
use zigbee_runtime::ota::{OtaConfig, OtaManager};
use zigbee_runtime::ZigbeeDevice;
use zigbee_types::ShortAddress;
use zigbee_zcl::clusters::ota::{OtaCluster, OtaState, CMD_IMAGE_NOTIFY};
use zigbee_zcl::ClusterId;

/// Endpoint hosting the OTA client cluster.
const OTA_ENDPOINT: u8 = 1;

/// OTA manager specialised for this board.
pub type EspOtaManager = OtaManager<EspFirmwareWriter<EspOtaFlash>>;

enum OtaBackend {
    Enabled(Box<EspOtaManager>),
    Disabled(Box<OtaCluster>),
}

/// OTA client state machine plus its transport bookkeeping.
pub struct OtaClient {
    backend: OtaBackend,
    /// (short address, endpoint) of the server driving the current transfer.
    server: Option<(u16, u8)>,
    /// Set when the transfer failed and the manager has to be reset once its
    /// last frame has been flushed.
    cleanup_pending: bool,
    /// Set when a verified image is waiting for the application's checkpoint.
    activation_pending: bool,
}

impl OtaClient {
    /// Build the client, opening the flash and choosing the staging slot.
    ///
    /// A missing or incompatible partition table disables OTA without
    /// preventing the sensor from joining and reporting. This also avoids
    /// bricking a remotely upgraded image if its layout expectations ever
    /// differ from the table installed on the device.
    pub fn new(config: OtaConfig) -> Self {
        let disabled_cluster = OtaCluster::new(
            config.manufacturer_code,
            config.image_type,
            config.current_version,
        );
        let backend =
            match EspFirmwareWriter::new(EspOtaFlash::new(), esp_hal::system::software_reset) {
                Ok(writer) => {
                    esp_println::println!(
                        "[ESP32-C6] OTA ready: running slot {}, staging slot {}, version {}",
                        writer.running_slot(),
                        writer.target_slot(),
                        config.current_version
                    );
                    OtaBackend::Enabled(Box::new(OtaManager::new(writer, config)))
                }
                Err(error) => {
                    esp_println::println!(
                        "[ESP32-C6] OTA disabled: incompatible flash layout ({:?})",
                        error
                    );
                    OtaBackend::Disabled(Box::new(disabled_cluster))
                }
            };
        Self {
            backend,
            server: None,
            cleanup_pending: false,
            activation_pending: false,
        }
    }

    /// Whether the flash layout supports OTA and the endpoint should advertise
    /// the OTA Upgrade client cluster.
    pub fn is_enabled(&self) -> bool {
        matches!(self.backend, OtaBackend::Enabled(_))
    }

    /// The ZCL cluster, to be passed to the runtime as a `ClusterRef`.
    pub fn cluster_mut(&mut self) -> &mut OtaCluster {
        match &mut self.backend {
            OtaBackend::Enabled(manager) => manager.cluster_mut(),
            OtaBackend::Disabled(cluster) => cluster,
        }
    }

    /// Whether a transfer is in flight (drives fast polling).
    pub fn is_active(&self) -> bool {
        match &self.backend {
            OtaBackend::Enabled(manager) => matches!(
                manager.state(),
                OtaState::QuerySent
                    | OtaState::Downloading { .. }
                    | OtaState::Verifying
                    | OtaState::WaitingActivate
            ),
            OtaBackend::Disabled(_) => false,
        }
    }

    /// Whether a verified image is waiting to be activated.
    pub fn activation_pending(&self) -> bool {
        self.activation_pending
    }

    /// Handle a stack event. Returns `true` if it was an OTA command that this
    /// client consumed.
    pub async fn handle_event<M: MacDriver>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        event: &StackEvent,
    ) -> bool {
        let StackEvent::CommandReceived {
            src_addr,
            source_endpoint,
            endpoint,
            cluster_id,
            command_id,
            payload,
            ..
        } = event
        else {
            return false;
        };
        if *cluster_id != ClusterId::OTA_UPGRADE.0 {
            return false;
        }
        if !self.is_enabled() {
            return true;
        }
        if *endpoint != OTA_ENDPOINT {
            return true;
        }

        let sender = (*src_addr, *source_endpoint);
        match self.server {
            // Another server must not interleave into a running transfer.
            Some(server) if server != sender => return true,
            // Only an ImageNotify may start a transfer with a new server.
            None if *command_id != CMD_IMAGE_NOTIFY.0 => return true,
            None => self.server = Some(sender),
            Some(_) => {}
        }

        let status = match &mut self.backend {
            OtaBackend::Enabled(manager) => {
                manager.handle_incoming(*command_id, payload.as_slice(), None)
            }
            OtaBackend::Disabled(_) => return true,
        };
        self.handle_status(status);
        self.send_pending(device).await;
        if matches!(
            &self.backend,
            OtaBackend::Enabled(manager) if manager.state() == OtaState::Idle
        ) {
            self.server = None;
        }
        true
    }

    /// Advance timers and flush any queued request.
    pub async fn service<M: MacDriver>(&mut self, device: &mut ZigbeeDevice<M>, elapsed_secs: u16) {
        let status = match &mut self.backend {
            OtaBackend::Enabled(manager) => manager.tick(elapsed_secs),
            OtaBackend::Disabled(_) => return,
        };
        self.handle_status(status);
        self.send_pending(device).await;
    }

    /// Activate the verified image. Does not return: the chip resets into the
    /// staged slot. Call only after persisting state.
    pub fn activate(&mut self) -> Result<(), FirmwareError> {
        self.activation_pending = false;
        let result = match &mut self.backend {
            OtaBackend::Enabled(manager) => manager.activate(),
            OtaBackend::Disabled(_) => Err(FirmwareError::ActivateFailed),
        };
        if result.is_err() {
            self.cleanup_pending = true;
        }
        result
    }

    fn handle_status(&mut self, status: Option<StackEvent>) {
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
                self.cleanup_pending = true;
            }
            Some(StackEvent::OtaComplete) => {
                esp_println::println!("[ESP32-C6] OTA image verified — awaiting checkpoint");
                self.activation_pending = true;
            }
            _ => {}
        }
    }

    async fn send_pending<M: MacDriver>(&mut self, device: &mut ZigbeeDevice<M>) {
        let pending = match &mut self.backend {
            OtaBackend::Enabled(manager) => manager.take_pending_frame(),
            OtaBackend::Disabled(_) => return,
        };
        if let Some(frame) = pending {
            let Some((server, server_endpoint)) = self.server else {
                self.abort_transfer();
                self.cleanup_pending = false;
                return;
            };
            if device
                .send_zcl_frame(
                    ShortAddress(server),
                    server_endpoint,
                    frame.endpoint,
                    frame.cluster_id,
                    frame.zcl_data.as_slice(),
                )
                .await
                .is_err()
            {
                // Requeue for the next service tick; the request carries its
                // own offset, so resending it cannot corrupt the download. The
                // manager's response timeout bounds the retries.
                let requeued = match &mut self.backend {
                    OtaBackend::Enabled(manager) => manager.requeue_pending_frame(frame),
                    OtaBackend::Disabled(_) => false,
                };
                if !requeued {
                    self.abort_transfer();
                    self.server = None;
                    self.cleanup_pending = false;
                }
                return;
            }
        }
        if self.cleanup_pending {
            self.abort_transfer();
            self.server = None;
            self.cleanup_pending = false;
            self.activation_pending = false;
        }
    }

    fn abort_transfer(&mut self) {
        if let OtaBackend::Enabled(manager) = &mut self.backend {
            manager.abort();
        }
    }
}
