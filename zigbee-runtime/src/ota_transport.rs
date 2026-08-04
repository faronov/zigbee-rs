//! OTA session transport — the network bookkeeping every platform needs.
//!
//! [`OtaManager`](crate::ota::OtaManager) owns the OTA cluster state machine:
//! which query/block/upgrade-end command to send next, how to parse the OTA
//! image header, and when a download is complete. It has no notion of the
//! network: it only queues a [`PendingOtaFrame`](crate::ota::PendingOtaFrame)
//! and reports [`StackEvent`]s for the caller to act on.
//!
//! [`OtaSession`] is the transport glue that turns that into a running
//! upgrade:
//! * remembering which server (short address + endpoint) is driving the
//!   current transfer, so a second server cannot interleave into it;
//! * accepting only an Image Notify to start a transfer with a new server;
//! * sending the manager's queued request via APS and putting it back for
//!   the next call if the parent was unreachable (OTA requests carry their
//!   own file offset, so a resend is idempotent);
//! * tracking whether a verified image is waiting for the application to
//!   checkpoint state before calling [`OtaSession::activate`].
//!
//! This is deliberately *only* the part that was byte-for-byte identical
//! between the EFR32MG1 and ESP32-C6 example applications. What stays out
//! of this type, and remains in each composition root:
//! * image parsing/validation and query/block/upgrade sequencing — owned by
//!   [`OtaManager`];
//! * physical storage, version policy, and boot selection — owned by each
//!   platform's [`FirmwareWriter`](crate::firmware_writer::FirmwareWriter);
//! * *when* to react to a status — fast-poll window extensions, console
//!   logging, and the checkpoint-before-activate ordering differ between a
//!   profile where OTA is mandatory ([`crate::profile::WithOta`]) and one
//!   where a checked partition/bootloader layout can leave it absent
//!   ([`crate::profile::OptionalOta`]), so callers pass `Option<&mut
//!   OtaManager<F>>` and keep that policy themselves.

use crate::ZigbeeDevice;
use crate::event_loop::StackEvent;
use crate::firmware_writer::{FirmwareError, FirmwareWriter};
use crate::ota::OtaManager;
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::ota::{CMD_IMAGE_NOTIFY, OtaState};
use zigbee_zcl::frame::ZclFrameType;

/// Result of handing a [`StackEvent`] to [`OtaSession::handle_event`].
#[derive(Debug)]
pub enum OtaEventOutcome {
    /// The event was not OTA Upgrade cluster traffic for this session.
    NotOta,
    /// The event *was* OTA Upgrade cluster traffic, but this session
    /// declined to act on it: the endpoint didn't match, another server is
    /// already mid-transfer, only an Image Notify may start a transfer with
    /// an unknown server, or there is no firmware backend at all (a checked
    /// layout left it absent). The caller should still treat the traffic as
    /// handled — do not fall through to other command handling — but there
    /// is no status to react to.
    ///
    /// Kept distinct from [`Self::Consumed`] because the two example
    /// platforms react differently to declined traffic (one extends its
    /// fast-poll window on any recognised OTA traffic, the other only once
    /// a request actually reaches the manager); that policy stays in each
    /// composition root.
    Ignored,
    /// The event reached the OTA manager. Carries the resulting status, if
    /// any, so the caller can react to it — for example to extend a
    /// fast-poll window or log progress.
    Consumed(Option<StackEvent>),
}

/// Network session bookkeeping for one OTA client endpoint.
///
/// Generic over the firmware writer `F` so the same type drives either a
/// mandatory [`OtaManager`] (always passed as `Some`) or an optional one
/// behind [`crate::profile::OtaBackend`] (a checked layout can leave it
/// `None` for the lifetime of the device).
#[derive(Debug, Default)]
pub struct OtaSession {
    /// (short address, endpoint) of the server driving the current transfer.
    server: Option<(u16, u8)>,
    /// Set when the transfer failed and the manager has to be reset once its
    /// last frame has been flushed.
    cleanup_pending: bool,
    /// Set when a verified image is waiting for the application's checkpoint.
    activation_pending: bool,
}

impl OtaSession {
    /// Create a new, idle session.
    pub const fn new() -> Self {
        Self {
            server: None,
            cleanup_pending: false,
            activation_pending: false,
        }
    }

    /// Whether a verified image is waiting for the application to persist
    /// state before calling [`Self::activate`].
    pub const fn activation_pending(&self) -> bool {
        self.activation_pending
    }

    /// Whether a transfer is in flight (drives fast polling).
    pub fn is_active<F: FirmwareWriter>(manager: Option<&OtaManager<F>>) -> bool {
        manager.is_some_and(|manager| {
            matches!(
                manager.state(),
                OtaState::QuerySent
                    | OtaState::Downloading { .. }
                    | OtaState::Verifying
                    | OtaState::WaitingActivate
            )
        })
    }

    /// Handle a stack event that may be OTA Upgrade cluster traffic.
    ///
    /// `endpoint` is the local endpoint hosting the OTA client cluster.
    /// Returns [`OtaEventOutcome::NotOta`] if `event` was not addressed to
    /// the OTA Upgrade cluster at all, so the caller can keep matching on it
    /// for its own purposes.
    pub async fn handle_event<M: MacDriver, F: FirmwareWriter>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        manager: Option<&mut OtaManager<F>>,
        endpoint: u8,
        event: &StackEvent,
    ) -> OtaEventOutcome {
        let StackEvent::CommandReceived {
            src_addr,
            source_endpoint,
            endpoint: dst_endpoint,
            cluster_id,
            frame_type,
            command_id,
            payload,
            ..
        } = event
        else {
            return OtaEventOutcome::NotOta;
        };
        if *cluster_id != ClusterId::OTA_UPGRADE.0 {
            return OtaEventOutcome::NotOta;
        }
        if *frame_type != ZclFrameType::ClusterSpecific {
            return OtaEventOutcome::Ignored;
        }
        // No firmware backend on this device (checked layout disabled it) —
        // the traffic is OTA, but there is nothing to drive it with.
        let Some(manager) = manager else {
            return OtaEventOutcome::Ignored;
        };
        if *dst_endpoint != endpoint {
            return OtaEventOutcome::Ignored;
        }

        let sender = (*src_addr, *source_endpoint);
        match self.server {
            // Another server must not interleave into a running transfer.
            Some(server) if server != sender => return OtaEventOutcome::Ignored,
            // Only an Image Notify may start a transfer with a new server.
            None if *command_id != CMD_IMAGE_NOTIFY.0 => return OtaEventOutcome::Ignored,
            None => self.server = Some(sender),
            Some(_) => {}
        }

        let status = manager.handle_incoming(*command_id, payload.as_slice(), None);
        self.note_status(&status);
        self.send_pending(device, manager).await;
        if manager.state() == OtaState::Idle {
            self.server = None;
        }
        OtaEventOutcome::Consumed(status)
    }

    /// Advance timers and flush any queued request. Returns the resulting
    /// status, if any, so the caller can react to it the same way as for
    /// [`Self::handle_event`] (a delayed activation counting down, or the
    /// upgrade completing, can both happen between incoming frames).
    pub async fn service<M: MacDriver, F: FirmwareWriter>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        manager: Option<&mut OtaManager<F>>,
        elapsed_secs: u16,
    ) -> Option<StackEvent> {
        let manager = manager?;
        let status = manager.tick(elapsed_secs);
        self.note_status(&status);
        self.send_pending(device, manager).await;
        status
    }

    /// Activate a verified image after the application has persisted any
    /// state that must survive the reset into the staged image.
    pub fn activate<F: FirmwareWriter>(
        &mut self,
        manager: Option<&mut OtaManager<F>>,
    ) -> Result<(), FirmwareError> {
        self.activation_pending = false;
        let Some(manager) = manager else {
            return Err(FirmwareError::ActivateFailed);
        };
        let result = manager.activate();
        if result.is_err() {
            self.cleanup_pending = true;
        }
        result
    }

    /// Track the bookkeeping-relevant status: an aborted transfer needs
    /// cleanup once its last frame is flushed, and a verified image needs
    /// the application to checkpoint before activating.
    fn note_status(&mut self, status: &Option<StackEvent>) {
        match status {
            Some(StackEvent::OtaFailed) => self.cleanup_pending = true,
            Some(StackEvent::OtaComplete) => self.activation_pending = true,
            _ => {}
        }
    }

    async fn send_pending<M: MacDriver, F: FirmwareWriter>(
        &mut self,
        device: &mut ZigbeeDevice<M>,
        manager: &mut OtaManager<F>,
    ) {
        let pending = manager.take_pending_frame();
        if let Some(frame) = pending {
            let Some((server, server_endpoint)) = self.server else {
                manager.abort();
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
                // Requeue for the next call; the request carries its own
                // offset, so resending it cannot corrupt the download. The
                // manager's response timeout bounds the retries.
                if !manager.requeue_pending_frame(frame) {
                    manager.abort();
                    self.server = None;
                    self.cleanup_pending = false;
                }
                return;
            }
        }
        if self.cleanup_pending {
            manager.abort();
            self.server = None;
            self.cleanup_pending = false;
            self.activation_pending = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::firmware_writer::MockFirmwareWriter;
    use crate::ota::OtaConfig;
    use crate::security_store::{
        PersistentSecurityState, RamSecurityStateStore, SecurityStateStore,
    };
    use core::future::Future;
    use core::task::{Context, Poll, Waker};
    use std::sync::Arc;
    use std::task::Wake;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::DeviceType;
    use zigbee_zcl::clusters::ota::{
        CMD_IMAGE_BLOCK_RESPONSE, CMD_IMAGE_NOTIFY, CMD_QUERY_NEXT_IMAGE_RESPONSE,
        CMD_UPGRADE_END_RESPONSE,
    };

    const ENDPOINT: u8 = 1;
    const IEEE_ADDRESS: [u8; 8] = [0x02, 0x55, 0x4E, 0x33, 0x39, 0x36, 0x34, 0x46];
    const SERVER_A: (u16, u8) = (0x1111, 1);
    const SERVER_B: (u16, u8) = (0x2222, 1);

    struct NoopWake;
    impl Wake for NoopWake {
        fn wake(self: Arc<Self>) {}
    }

    fn block_on<F: Future>(future: F) -> F::Output {
        let waker = Waker::from(Arc::new(NoopWake));
        let mut context = Context::from_waker(&waker);
        let mut future = core::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    /// A joined end device with a MockMac, ready to send unicast ZCL frames
    /// to its parent — set up the same way as the runtime's own
    /// commissioning tests, by restoring a fabricated commissioned state
    /// instead of driving a real join handshake.
    fn joined_device() -> ZigbeeDevice<MockMac> {
        let mut device = ZigbeeDevice::builder(MockMac::new(IEEE_ADDRESS))
            .device_type(DeviceType::EndDevice)
            .build();
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = IEEE_ADDRESS;
        state.channel = 15;
        state.depth = 1;
        state.parent_address = 0x0000;
        state.network_key = [2; 16];
        state.global_counter_limit = 0x400;
        state.tclk_present = true;
        state.trust_center_address = [3; 8];
        state.trust_center_link_key = [4; 16];
        state.tclk_counter_limit = 0x400;

        let mut store = RamSecurityStateStore::new();
        store.store(&state).unwrap();
        block_on(device.start_or_resume_with_security_store(&mut store)).unwrap();
        assert!(device.is_joined(), "test harness device must be joined");
        device
    }

    fn manager(endpoint: u8) -> OtaManager<MockFirmwareWriter> {
        OtaManager::new(
            MockFirmwareWriter::new(4096),
            OtaConfig {
                manufacturer_code: 0x1234,
                image_type: 0x0001,
                current_version: 1,
                endpoint,
                block_size: 48,
                auto_accept: true,
                hardware_version: None,
            },
        )
    }

    fn command_event(
        server: (u16, u8),
        endpoint: u8,
        cluster_id: u16,
        command_id: u8,
        payload: &[u8],
    ) -> StackEvent {
        let mut buf = heapless::Vec::new();
        for &b in payload {
            let _ = buf.push(b);
        }
        StackEvent::CommandReceived {
            src_addr: server.0,
            source_endpoint: server.1,
            endpoint,
            cluster_id,
            frame_type: ZclFrameType::ClusterSpecific,
            command_id,
            seq_number: 0,
            payload: buf,
        }
    }

    fn image_notify(server: (u16, u8), endpoint: u8) -> StackEvent {
        // Payload type 0 (jitter only) always matches this device.
        command_event(
            server,
            endpoint,
            ClusterId::OTA_UPGRADE.0,
            CMD_IMAGE_NOTIFY.0,
            &[],
        )
    }

    /// Build a minimal valid OTA file: 56B header + 6B sub-element header,
    /// no firmware payload — enough to exercise header parsing and a single
    /// zero-length block without needing multi-block bookkeeping.
    fn build_ota_file(mfg: u16, img_type: u16, version: u32) -> heapless::Vec<u8, 64> {
        let mut file = heapless::Vec::<u8, 64>::new();
        let push_bytes = |bytes: &[u8], file: &mut heapless::Vec<u8, 64>| {
            for &b in bytes {
                let _ = file.push(b);
            }
        };
        push_bytes(&0x0BEEF11Eu32.to_le_bytes(), &mut file); // magic
        push_bytes(&0x0100u16.to_le_bytes(), &mut file); // header version
        push_bytes(&56u16.to_le_bytes(), &mut file); // header length
        push_bytes(&0u16.to_le_bytes(), &mut file); // field control
        push_bytes(&mfg.to_le_bytes(), &mut file);
        push_bytes(&img_type.to_le_bytes(), &mut file);
        push_bytes(&version.to_le_bytes(), &mut file);
        push_bytes(&0x0002u16.to_le_bytes(), &mut file); // stack version
        for _ in 0..32 {
            let _ = file.push(0);
        } // header string
        push_bytes(&56u32.to_le_bytes(), &mut file); // total image size == header only
        push_bytes(&0x0000u16.to_le_bytes(), &mut file); // sub-element tag: UpgradeImage
        push_bytes(&0u32.to_le_bytes(), &mut file); // sub-element length: 0 bytes of firmware
        file
    }

    #[test]
    fn non_ota_events_are_not_consumed() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);

        let joined = StackEvent::Joined {
            short_address: 0x5678,
            channel: 15,
            pan_id: 0x1234,
        };
        assert!(matches!(
            block_on(session.handle_event(&mut device, Some(&mut mgr), ENDPOINT, &joined)),
            OtaEventOutcome::NotOta
        ));

        let other_cluster = command_event(SERVER_A, ENDPOINT, ClusterId::BASIC.0, 0x00, &[]);
        assert!(matches!(
            block_on(session.handle_event(&mut device, Some(&mut mgr), ENDPOINT, &other_cluster)),
            OtaEventOutcome::NotOta
        ));
        assert_eq!(
            mgr.state(),
            OtaState::Idle,
            "unrelated traffic must not touch the manager"
        );
    }

    #[test]
    fn ota_foundation_command_is_not_treated_as_image_notify() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);
        let sent_before = device.mac_mut().tx_history().len();
        let mut event = command_event(
            SERVER_A,
            ENDPOINT,
            ClusterId::OTA_UPGRADE.0,
            CMD_IMAGE_NOTIFY.0,
            &[0x02, 0x00],
        );
        let StackEvent::CommandReceived { frame_type, .. } = &mut event else {
            unreachable!();
        };
        *frame_type = ZclFrameType::Global;

        let outcome = block_on(session.handle_event(&mut device, Some(&mut mgr), ENDPOINT, &event));

        assert!(matches!(outcome, OtaEventOutcome::Ignored));
        assert_eq!(mgr.state(), OtaState::Idle);
        assert_eq!(device.mac_mut().tx_history().len(), sent_before);
    }

    #[test]
    fn disabled_backend_consumes_ota_traffic_without_state() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let sent_before = device.mac_mut().tx_history().len();

        assert!(!OtaSession::is_active::<MockFirmwareWriter>(None));

        let event = image_notify(SERVER_A, ENDPOINT);
        let outcome: OtaEventOutcome =
            block_on(session.handle_event::<MockMac, MockFirmwareWriter>(
                &mut device,
                None,
                ENDPOINT,
                &event,
            ));
        assert!(matches!(outcome, OtaEventOutcome::Ignored));
        assert!(!session.activation_pending());
        assert_eq!(
            device.mac_mut().tx_history().len(),
            sent_before,
            "a missing backend must not send anything"
        );
    }

    #[test]
    fn traffic_for_a_different_endpoint_is_ignored() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);
        let sent_before = device.mac_mut().tx_history().len();

        let event = image_notify(SERVER_A, ENDPOINT + 1);
        let outcome = block_on(session.handle_event(&mut device, Some(&mut mgr), ENDPOINT, &event));
        assert!(matches!(outcome, OtaEventOutcome::Ignored));
        assert_eq!(mgr.state(), OtaState::Idle);
        assert_eq!(device.mac_mut().tx_history().len(), sent_before);
    }

    #[test]
    fn image_notify_starts_a_query_and_records_the_server() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);
        let sent_before = device.mac_mut().tx_history().len();

        let event = image_notify(SERVER_A, ENDPOINT);
        let outcome = block_on(session.handle_event(&mut device, Some(&mut mgr), ENDPOINT, &event));
        assert!(matches!(outcome, OtaEventOutcome::Consumed(None)));
        assert_eq!(mgr.state(), OtaState::QuerySent);
        assert!(OtaSession::is_active(Some(&mgr)));
        assert_eq!(
            device.mac_mut().tx_history().len(),
            sent_before + 1,
            "the query request must have been sent over the air"
        );
    }

    #[test]
    fn a_second_server_cannot_interleave_into_a_running_transfer() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);

        block_on(session.handle_event(
            &mut device,
            Some(&mut mgr),
            ENDPOINT,
            &image_notify(SERVER_A, ENDPOINT),
        ));
        let sent_before = device.mac_mut().tx_history().len();

        let outcome = block_on(session.handle_event(
            &mut device,
            Some(&mut mgr),
            ENDPOINT,
            &image_notify(SERVER_B, ENDPOINT),
        ));
        assert!(matches!(outcome, OtaEventOutcome::Ignored));
        assert_eq!(
            device.mac_mut().tx_history().len(),
            sent_before,
            "traffic from a second server must not be forwarded to the manager"
        );
        assert_eq!(mgr.state(), OtaState::QuerySent);
    }

    /// Build a Query Next Image Response advertising `total_size` bytes at
    /// `version`, ready to hand to [`command_event`].
    fn query_response_bytes(version: u32, total_size: u32) -> [u8; 13] {
        let mut resp = [0u8; 13];
        resp[0] = 0x00; // success
        resp[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
        resp[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
        resp[5..9].copy_from_slice(&version.to_le_bytes());
        resp[9..13].copy_from_slice(&total_size.to_le_bytes());
        resp
    }

    /// Build one Image Block Response carrying `chunk` at `offset`.
    ///
    /// `chunk` must be small enough that the 14-byte response header plus
    /// the chunk fits in the 64-byte [`StackEvent::CommandReceived`] payload
    /// — exactly the limit real hardware runs into, which is why an OTA
    /// image header (56B) plus its sub-element header (6B) is always
    /// delivered across at least two block responses in practice.
    fn block_response_event(
        server: (u16, u8),
        endpoint: u8,
        version: u32,
        offset: u32,
        chunk: &[u8],
    ) -> StackEvent {
        assert!(
            14 + chunk.len() <= 64,
            "chunk too large for a StackEvent payload"
        );
        let mut payload = [0u8; 64];
        payload[0] = 0x00; // success
        payload[1..3].copy_from_slice(&0x1234u16.to_le_bytes());
        payload[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
        payload[5..9].copy_from_slice(&version.to_le_bytes());
        payload[9..13].copy_from_slice(&offset.to_le_bytes());
        payload[13] = chunk.len() as u8;
        payload[14..14 + chunk.len()].copy_from_slice(chunk);
        command_event(
            server,
            endpoint,
            ClusterId::OTA_UPGRADE.0,
            CMD_IMAGE_BLOCK_RESPONSE.0,
            &payload[..14 + chunk.len()],
        )
    }

    /// Feed `ota_file` to `mgr` through `session` as a real server would:
    /// one image notify, a query response, then as many block responses as
    /// the 64-byte transport cap requires (mirroring block_size=48 in the
    /// product profiles). Returns once the transfer reaches `WaitingActivate`
    /// or fails.
    fn drive_download(
        session: &mut OtaSession,
        device: &mut ZigbeeDevice<MockMac>,
        mgr: &mut OtaManager<MockFirmwareWriter>,
        server: (u16, u8),
        ota_file: &[u8],
        version: u32,
    ) -> Option<StackEvent> {
        let total = ota_file.len() as u32;
        block_on(session.handle_event(
            device,
            Some(mgr),
            ENDPOINT,
            &image_notify(server, ENDPOINT),
        ));
        let outcome = block_on(session.handle_event(
            device,
            Some(mgr),
            ENDPOINT,
            &command_event(
                server,
                ENDPOINT,
                ClusterId::OTA_UPGRADE.0,
                CMD_QUERY_NEXT_IMAGE_RESPONSE.0,
                &query_response_bytes(version, total),
            ),
        ));
        assert!(
            matches!(
                outcome,
                OtaEventOutcome::Consumed(Some(StackEvent::OtaImageAvailable { .. }))
            ),
            "query response must start the download: {outcome:?}"
        );

        let mut offset = 0u32;
        const CHUNK: usize = 48;
        let mut last_status = None;
        while offset < total {
            let end = ((offset as usize) + CHUNK).min(ota_file.len());
            let chunk = &ota_file[offset as usize..end];
            let event = block_response_event(server, ENDPOINT, version, offset, chunk);
            let outcome = block_on(session.handle_event(device, Some(mgr), ENDPOINT, &event));
            let OtaEventOutcome::Consumed(status) = outcome else {
                panic!("block response must be consumed as OTA traffic");
            };
            last_status = status;
            offset = end as u32;
            // Drive the tick: the manager needs one to turn the completed
            // write into either the next block request or, once the last
            // block lands, the verify + Upgrade End Request step.
            if let Some(status) = block_on(session.service(device, Some(mgr), 0)) {
                last_status = Some(status);
            }
        }
        last_status
    }

    #[test]
    fn full_download_completes_and_defers_activation() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);

        let ota_file = build_ota_file(0x1234, 0x0001, 2);
        let last_status =
            drive_download(&mut session, &mut device, &mut mgr, SERVER_A, &ota_file, 2);
        assert!(
            matches!(last_status, Some(StackEvent::OtaProgress { .. })),
            "last status before the end request must be progress: {last_status:?}"
        );

        let mut end_resp = [0u8; 16];
        end_resp[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
        end_resp[2..4].copy_from_slice(&0x0001u16.to_le_bytes());
        end_resp[4..8].copy_from_slice(&2u32.to_le_bytes());
        end_resp[8..12].copy_from_slice(&1000u32.to_le_bytes()); // current time
        end_resp[12..16].copy_from_slice(&0u32.to_le_bytes()); // upgrade now
        let outcome = block_on(session.handle_event(
            &mut device,
            Some(&mut mgr),
            ENDPOINT,
            &command_event(
                SERVER_A,
                ENDPOINT,
                ClusterId::OTA_UPGRADE.0,
                CMD_UPGRADE_END_RESPONSE.0,
                &end_resp,
            ),
        ));
        assert!(matches!(
            outcome,
            OtaEventOutcome::Consumed(Some(StackEvent::OtaComplete))
        ));
        assert!(
            session.activation_pending(),
            "activation must wait for the application to checkpoint state first"
        );

        // The application checkpoints here, then activates.
        assert!(session.activate(Some(&mut mgr)).is_ok());
        assert!(!session.activation_pending());
        assert!(mgr.writer().is_activated());
    }

    #[test]
    fn a_failed_transfer_is_cleaned_up_and_the_session_resets() {
        let mut session = OtaSession::new();
        let mut device = joined_device();
        let mut mgr = manager(ENDPOINT);

        // A file for a different manufacturer triggers a header validation
        // failure once enough of the header has been buffered.
        let ota_file = build_ota_file(0x9999, 0x0001, 2);
        let last_status =
            drive_download(&mut session, &mut device, &mut mgr, SERVER_A, &ota_file, 2);
        assert!(
            matches!(last_status, Some(StackEvent::OtaFailed)),
            "manufacturer mismatch must fail the transfer: {last_status:?}"
        );
        assert_eq!(
            mgr.state(),
            OtaState::Idle,
            "the session must flush the failure end request and abort back to idle"
        );

        // The session must accept a brand new server after the reset.
        let outcome = block_on(session.handle_event(
            &mut device,
            Some(&mut mgr),
            ENDPOINT,
            &image_notify(SERVER_B, ENDPOINT),
        ));
        assert!(matches!(outcome, OtaEventOutcome::Consumed(None)));
        assert_eq!(mgr.state(), OtaState::QuerySent);
    }
}
