//! OTA Manager — runtime integration for OTA firmware upgrades.
//!
//! Combines the ZCL OTA cluster (state machine + command parsing) with
//! a FirmwareWriter (platform flash abstraction) to handle the complete
//! OTA upgrade flow.
//!
//! Key responsibilities:
//! - Gates the download on application acceptance when `auto_accept` is off:
//!   the offered image identity (manufacturer, image type, file version,
//!   size) is reported with [`StackEvent::OtaImageAvailable`] and only that
//!   exact image may be downloaded after [`OtaManager::accept_ota`].
//! - Erases the flash slot before the first block of an accepted download.
//! - Streams the OTA file through a bounded parser: validates the header
//!   against the offered image (manufacturer, image type, file version newer
//!   than the running image, total size, hardware-version range), walks the
//!   sub-elements with overflow-checked arithmetic, and writes only the
//!   payload of the mandatory Upgrade Image (tag `0x0000`) sub-element.
//! - Verifies that exactly the declared payload length was written.
//!
//! # Image integrity
//!
//! The ZCL OTA file format's optional Image Integrity Code (tag `0x0003`,
//! AES-MMO over the file) and ECDSA signature sub-elements are *not* verified
//! here: sub-elements after the Upgrade Image are skipped, and
//! [`FirmwareWriter::verify`] receives no hash. Authenticity therefore relies
//! on the platform bootloader (for example Gecko Bootloader / MCUboot image
//! signature checks). The runtime never activates an image whose header,
//! layout, or length disagrees with the server's offer.
//!
//! # Downgrades
//!
//! The ZCL cluster only accepts offers newer than `current_version`, and the
//! downloaded header must carry exactly the offered version, so a server
//! cannot force a downgrade through this manager.
//!
//! # Lost Upgrade End Response
//!
//! A verified image waiting for the server's Upgrade End Response is never
//! erased by the response timeout. The Upgrade End Request is retransmitted a
//! bounded number of times; if the server stays silent the session returns
//! to idle with [`StackEvent::OtaFailed`] while the verified image stays
//! staged but inactive (the next accepted download erases the slot).
//!
//! # Lost Image Block Responses
//!
//! An unanswered Image Block Request is retransmitted with a short, capped
//! backoff (2 s growing to 5 s) for as long as the response timeout allows.
//! OTA servers abandon a transfer that makes no progress for a short time
//! (zigpy: 30 s), so the client must keep asking well inside that window.
//!
//! # Resume after an interrupted download
//!
//! When the server goes silent (response timeout) or answers a block request
//! with `ABORT` (a server-side stall timeout or cancellation), a download
//! that already staged bytes is *suspended* rather than discarded: the
//! session returns to idle with [`StackEvent::OtaFailed`] but the parser
//! state and the firmware writer keep the staged prefix. If a later session
//! offers exactly the same image (manufacturer, image type, file version and
//! size), the download continues at the saved file offset without erasing
//! the slot; the writer keeps receiving sequential writes, so no programmed
//! byte is rewritten. Any other image discards the staged data and starts at
//! offset 0 with a fresh erase. Corruption (header/layout mismatch, a writer
//! error, verification failure) and an explicit [`OtaManager::abort`] always
//! discard. The suspended state lives in RAM only; a reset restarts the
//! download from offset 0.
//!
//! Enabled with the `ota` feature flag.

use crate::event_loop::StackEvent;
use crate::firmware_writer::FirmwareWriter;
use zigbee_zcl::clusters::ota::{
    self, ImageBlockRequest, OtaAction, OtaCluster, OtaState, QueryNextImageRequest,
    UpgradeEndRequest,
};
use zigbee_zcl::clusters::ota_image::OtaImageHeader;
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, CommandId};

/// Silence after which an outstanding OTA request is given up.
const OTA_RESPONSE_TIMEOUT_SECS: u32 = 120;
/// Delay before the first retransmission of an unanswered block request.
const OTA_BLOCK_RETRY_INTERVAL_SECS: u32 = 2;
/// Backoff ceiling for block-request retransmissions. Retries continue at
/// this interval until [`OTA_RESPONSE_TIMEOUT_SECS`]; it must stay well
/// below server stall timeouts (zigpy aborts after 30 s without progress).
const OTA_BLOCK_RETRY_MAX_INTERVAL_SECS: u32 = 5;
/// ZCL status ABORT (0x95) in an Image Block Response: the server ended the
/// transfer (ZCL r8 §11.13.8.2).
const OTA_STATUS_ABORT: u8 = 0x95;
/// Upgrade End Request retransmissions while a verified image waits for the
/// server's Upgrade End Response.
const OTA_END_REQUEST_MAX_RETRIES: u8 = 3;
/// OTA header bytes buffered before the first sub-element.
const OTA_HEADER_BUFFER: usize = 128;
/// Upgrade Image sub-element tag (ZCL OTA file format).
const OTA_TAG_UPGRADE_IMAGE: u16 = 0x0000;
/// Sub-element header: tag (2) + length (4).
const OTA_SUB_ELEMENT_HEADER_LEN: u32 = 6;

/// Identity of an OTA image offered by the server's Query Next Image Response.
///
/// With `auto_accept = false` the application receives this offer through
/// [`StackEvent::OtaImageAvailable`] / [`OtaManager::offered_image`] and
/// accepts exactly this image with [`OtaManager::accept_ota`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaImageOffer {
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub file_version: u32,
    pub image_size: u32,
}

/// OTA configuration.
#[derive(Debug, Clone)]
pub struct OtaConfig {
    /// Manufacturer code for this device.
    pub manufacturer_code: u16,
    /// Image type for this device.
    pub image_type: u16,
    /// Current firmware version.
    pub current_version: u32,
    /// Endpoint where OTA cluster lives.
    pub endpoint: u8,
    /// Block size for image requests (default: 48).
    pub block_size: u8,
    /// Auto-accept OTA images (if false, app must call accept_ota()).
    pub auto_accept: bool,
    /// Hardware version (included in QueryNextImageRequest if set).
    pub hardware_version: Option<u16>,
}

impl Default for OtaConfig {
    fn default() -> Self {
        Self {
            manufacturer_code: 0x0000,
            image_type: 0x0000,
            current_version: 0x00000001,
            endpoint: 1,
            block_size: ota::DEFAULT_BLOCK_SIZE,
            auto_accept: true,
            hardware_version: None,
        }
    }
}

/// Pending OTA ZCL frame to be sent.
pub struct PendingOtaFrame {
    /// Serialized ZCL frame bytes.
    pub zcl_data: heapless::Vec<u8, 128>,
    /// Source/destination endpoint.
    pub endpoint: u8,
    /// Cluster ID (always 0x0019).
    pub cluster_id: u16,
}

struct BlockRetry {
    request: ImageBlockRequest,
    zcl_seq: u8,
    elapsed_secs: u32,
    retries_sent: u8,
}

/// OTA Manager — coordinates OTA cluster + firmware writer.
///
/// Handles the OTA file format: parses the OTA image header from the
/// first received blocks, validates manufacturer/image_type, then strips
/// the header and sub-element overhead — writing only raw firmware bytes
/// to the flash slot.
pub struct OtaManager<F: FirmwareWriter> {
    /// OTA ZCL cluster (state machine + attributes).
    cluster: OtaCluster,
    /// Platform firmware writer.
    writer: F,
    /// OTA configuration.
    config: OtaConfig,
    /// Pending outgoing frame queued for the transport.
    pending_frame: Option<PendingOtaFrame>,
    /// ZCL sequence counter (borrowed from device).
    zcl_seq: u8,
    /// Download context — tracks header parsing and payload offset.
    download_ctx: OtaDownloadCtx,
    /// Offer reported to the application and awaiting `accept_ota()`.
    offered_image: Option<OtaImageOffer>,
    /// Offer the application accepted; only this exact image may download.
    accepted_image: Option<OtaImageOffer>,
    /// Upgrade End Request retransmissions sent for the verified image.
    end_request_retries: u8,
    /// Time spent waiting for the next OTA server response.
    response_wait_secs: u32,
    /// Logical retry state for the currently outstanding block request.
    block_retry: Option<BlockRetry>,
    /// Whether the verified image is waiting for application-controlled activation.
    activation_pending: bool,
    /// An interrupted download whose staged prefix is kept for resumption
    /// (`download_ctx` and the writer stay untouched while set).
    suspended: bool,
}

/// Position of the streaming OTA file parser.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OtaParsePhase {
    /// Buffering the OTA file header.
    Header,
    /// Skipping to / buffering the next sub-element header.
    SubElement,
    /// Writing the Upgrade Image payload.
    Payload,
    /// Past the Upgrade Image; remaining sub-elements are skipped.
    Trailer,
}

/// Tracks OTA file parsing and firmware write offset during download.
struct OtaDownloadCtx {
    phase: OtaParsePhase,
    /// Buffer for the file header, then for one 6-byte sub-element header.
    header_buf: heapless::Vec<u8, OTA_HEADER_BUFFER>,
    /// OTA file offset of the next unconsumed byte.
    file_offset: u32,
    /// OTA file offset of the next sub-element header.
    next_element: u32,
    /// OTA file offset of the Upgrade Image payload.
    payload_start: u32,
    /// Upgrade Image payload length declared by its sub-element header.
    firmware_size: u32,
    /// Firmware bytes actually written to flash.
    firmware_written: u32,
    /// Whether erase_slot() has been called.
    slot_erased: bool,
    /// The offer this download must match.
    offer: Option<OtaImageOffer>,
}

impl OtaDownloadCtx {
    fn new() -> Self {
        Self {
            phase: OtaParsePhase::Header,
            header_buf: heapless::Vec::new(),
            file_offset: 0,
            next_element: 0,
            payload_start: 0,
            firmware_size: 0,
            firmware_written: 0,
            slot_erased: false,
            offer: None,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    /// Whether the complete Upgrade Image payload was written.
    fn payload_complete(&self) -> bool {
        matches!(self.phase, OtaParsePhase::Payload | OtaParsePhase::Trailer)
            && self.firmware_size > 0
            && self.firmware_written == self.firmware_size
    }
}

impl<F: FirmwareWriter> OtaManager<F> {
    /// Create a new OTA manager.
    pub fn new(writer: F, config: OtaConfig) -> Self {
        let mut cluster = OtaCluster::new(
            config.manufacturer_code,
            config.image_type,
            config.current_version,
        );
        cluster.set_block_size(config.block_size);
        if let Some(hw) = config.hardware_version {
            cluster.set_hardware_version(hw);
        }

        Self {
            cluster,
            writer,
            config,
            pending_frame: None,
            zcl_seq: 0,
            download_ctx: OtaDownloadCtx::new(),
            offered_image: None,
            accepted_image: None,
            end_request_retries: 0,
            response_wait_secs: 0,
            block_retry: None,
            activation_pending: false,
            suspended: false,
        }
    }

    fn next_seq(&mut self) -> u8 {
        let s = self.zcl_seq;
        self.zcl_seq = self.zcl_seq.wrapping_add(1);
        s
    }

    /// Get the current OTA state.
    pub fn state(&self) -> OtaState {
        self.cluster.state()
    }

    /// Get download progress (0-100%).
    pub fn progress(&self) -> u8 {
        self.cluster.progress_percent()
    }

    /// Get the OTA cluster (for attribute reads).
    pub fn cluster(&self) -> &OtaCluster {
        &self.cluster
    }

    /// Get mutable access to the OTA cluster for runtime attribute dispatch.
    pub fn cluster_mut(&mut self) -> &mut OtaCluster {
        &mut self.cluster
    }

    pub const fn endpoint(&self) -> u8 {
        self.config.endpoint
    }

    /// Borrow the platform firmware writer (staging slot state, diagnostics).
    pub fn writer(&self) -> &F {
        &self.writer
    }

    /// Initiate an OTA image query.
    pub fn start_query(&mut self) -> Option<StackEvent> {
        self.response_wait_secs = 0;
        self.block_retry = None;
        self.activation_pending = false;
        let action = self.cluster.start_query();
        self.process_action(action)
    }

    /// Process an incoming OTA server→client command.
    ///
    /// `server_ieee` is the IEEE address of the sender (for UpgradeServerID).
    pub fn handle_incoming(
        &mut self,
        cmd_id: u8,
        payload: &[u8],
        server_ieee: Option<u64>,
    ) -> Option<StackEvent> {
        self.handle_incoming_with_sequence(cmd_id, payload, None, server_ieee)
    }

    /// Process an incoming OTA command with its ZCL transaction sequence.
    pub fn handle_incoming_with_sequence(
        &mut self,
        cmd_id: u8,
        payload: &[u8],
        zcl_seq: Option<u8>,
        server_ieee: Option<u64>,
    ) -> Option<StackEvent> {
        if cmd_id == ota::CMD_IMAGE_BLOCK_RESPONSE.0
            && payload.first().is_some_and(|status| *status != 0x00)
            && zcl_seq.is_some()
            && self.block_retry.as_ref().map(|retry| retry.zcl_seq) != zcl_seq
        {
            return None;
        }

        // A server ABORT ends the transfer on the server side (for example a
        // stall timeout). It says nothing about the staged data, so keep it
        // for a later session offering the same image.
        if cmd_id == ota::CMD_IMAGE_BLOCK_RESPONSE.0
            && payload.first() == Some(&OTA_STATUS_ABORT)
            && self.can_suspend()
        {
            log::warn!(
                "[OTA] Server aborted the download at offset {}; keeping staged data",
                self.download_ctx.file_offset
            );
            self.suspend();
            return Some(StackEvent::OtaFailed);
        }

        let outcome = self
            .cluster
            .process_server_command_with_outcome(cmd_id, payload);
        if outcome.accepted {
            self.response_wait_secs = 0;
            // Only a matching response proves that the outstanding request
            // reached the server. Stale blocks must not erase its retry copy.
            self.pending_frame = None;
            self.block_retry = None;
            if let Some(ieee) = server_ieee {
                self.cluster.set_upgrade_server_id(ieee);
            }
        }
        self.process_action(outcome.action)
    }

    /// Tick the OTA engine (called from runtime tick).
    pub fn tick(&mut self, elapsed_secs: u16) -> Option<StackEvent> {
        if matches!(
            self.cluster.state(),
            OtaState::QuerySent
                | OtaState::Downloading { .. }
                | OtaState::Verifying
                | OtaState::WaitingActivate
        ) {
            self.response_wait_secs = self.response_wait_secs.saturating_add(elapsed_secs as u32);
            if self.response_wait_secs >= OTA_RESPONSE_TIMEOUT_SECS {
                if self.cluster.state() == OtaState::WaitingActivate {
                    return self.end_response_timeout();
                }
                if self.can_suspend() {
                    log::warn!(
                        "[OTA] Server silent; suspending download at offset {}",
                        self.download_ctx.file_offset
                    );
                    self.suspend();
                } else {
                    self.end_session();
                }
                return Some(StackEvent::OtaFailed);
            }
        } else {
            self.response_wait_secs = 0;
        }

        // Handle WaitForData countdown
        let action = self.cluster.tick(elapsed_secs);
        if !matches!(&action, OtaAction::None) {
            return self.process_action(action);
        }

        let retry_request = if self.pending_frame.is_none()
            && matches!(self.cluster.state(), OtaState::Downloading { .. })
        {
            self.block_retry.as_mut().and_then(|retry| {
                retry.elapsed_secs = retry.elapsed_secs.saturating_add(elapsed_secs as u32);
                // No retry cap: the response timeout bounds the retries, and
                // going quiet would let the server's stall timer expire.
                let interval = (OTA_BLOCK_RETRY_INTERVAL_SECS + u32::from(retry.retries_sent))
                    .min(OTA_BLOCK_RETRY_MAX_INTERVAL_SECS);
                if retry.elapsed_secs >= interval {
                    retry.elapsed_secs = 0;
                    retry.retries_sent = retry.retries_sent.saturating_add(1);
                    Some((retry.request.clone(), retry.zcl_seq))
                } else {
                    None
                }
            })
        } else {
            None
        };
        if let Some((request, zcl_seq)) = retry_request {
            self.build_and_queue_block_request(&request, zcl_seq);
        }
        None
    }

    /// Take the pending outgoing frame (consumed by runtime to send via APS).
    pub fn take_pending_frame(&mut self) -> Option<PendingOtaFrame> {
        self.pending_frame.take()
    }

    /// Requeue a frame after a transient transport failure.
    ///
    /// OTA requests are idempotent because they carry the requested file
    /// offset. The response timeout remains the upper bound for retries.
    pub fn requeue_pending_frame(&mut self, frame: PendingOtaFrame) -> bool {
        if self.pending_frame.is_some() {
            return false;
        }
        self.pending_frame = Some(frame);
        true
    }

    /// Activate a verified image after the application has persisted any state
    /// that must survive the bootloader reset.
    pub fn activate(&mut self) -> Result<(), crate::firmware_writer::FirmwareError> {
        if !self.activation_pending {
            return Err(crate::firmware_writer::FirmwareError::ActivateFailed);
        }
        self.activation_pending = false;
        self.writer.activate()
    }

    /// Abort the current OTA.
    ///
    /// Discards any staged image (including a suspended download), pending
    /// offer, and acceptance.
    pub fn abort(&mut self) {
        let _ = self.writer.abort();
        self.suspended = false;
        self.reset_session();
    }

    /// Return to idle after a session ended, keeping a suspended download
    /// for resumption and discarding everything else (see [`Self::abort`]).
    pub fn end_session(&mut self) {
        if self.suspended {
            self.reset_session();
        } else {
            self.abort();
        }
    }

    /// Whether an interrupted download is kept for resumption.
    pub fn is_suspended(&self) -> bool {
        self.suspended
    }

    /// The image and file offset of a suspended download, if any.
    pub fn suspended_download(&self) -> Option<(OtaImageOffer, u32)> {
        if !self.suspended {
            return None;
        }
        self.download_ctx
            .offer
            .map(|offer| (offer, self.download_ctx.file_offset))
    }

    /// Whether the running download staged data that a later session for
    /// the same image can continue.
    fn can_suspend(&self) -> bool {
        let ctx = &self.download_ctx;
        matches!(
            self.cluster.state(),
            OtaState::Downloading { .. } | OtaState::WaitForData { .. }
        ) && ctx.slot_erased
            && ctx.offer.is_some()
            && ctx.file_offset > 0
    }

    /// Return to idle keeping the parser state and staged writer data.
    fn suspend(&mut self) {
        self.suspended = true;
        self.reset_session();
    }

    /// Drop a suspended download before staging a different image.
    fn discard_suspended(&mut self) {
        let _ = self.writer.abort();
        self.suspended = false;
        self.download_ctx.reset();
    }

    /// Return to idle without touching the firmware slot.
    fn reset_session(&mut self) {
        self.cluster.abort();
        self.pending_frame = None;
        if !self.suspended {
            self.download_ctx.reset();
        }
        self.offered_image = None;
        self.accepted_image = None;
        self.end_request_retries = 0;
        self.response_wait_secs = 0;
        self.block_retry = None;
        self.activation_pending = false;
    }

    /// The server did not answer the Upgrade End Request for a verified image.
    ///
    /// Retransmit the request a bounded number of times. The verified image is
    /// never erased by this timeout: after the last retry the session returns
    /// to idle and reports [`StackEvent::OtaFailed`], leaving the staged (but
    /// not activated) image in the slot until the next accepted download.
    fn end_response_timeout(&mut self) -> Option<StackEvent> {
        self.response_wait_secs = 0;
        if self.end_request_retries < OTA_END_REQUEST_MAX_RETRIES {
            self.end_request_retries += 1;
            if let OtaAction::SendEndRequest(req) = self.cluster.mark_verified() {
                self.build_and_queue_end_request(&req);
            }
            return None;
        }
        log::warn!("[OTA] No Upgrade End Response; keeping verified image inactive");
        self.reset_session();
        Some(StackEvent::OtaFailed)
    }

    /// Image offered by the server and awaiting [`accept_ota`](Self::accept_ota).
    pub fn offered_image(&self) -> Option<OtaImageOffer> {
        self.offered_image
    }

    /// Image the application accepted for download, if any.
    pub fn accepted_image(&self) -> Option<OtaImageOffer> {
        self.accepted_image
    }

    /// Accept the pending OTA image (for `auto_accept = false` mode).
    ///
    /// Call this after receiving [`StackEvent::OtaImageAvailable`]. The offer
    /// is recorded as accepted and the server is queried again; the download
    /// starts only if the server offers exactly the accepted image
    /// (manufacturer, image type, file version, and size). A different offer
    /// raises a new [`StackEvent::OtaImageAvailable`]. Returns `None` when no
    /// offer is pending.
    pub fn accept_ota(&mut self) -> Option<StackEvent> {
        let offer = self.offered_image.take()?;
        self.accepted_image = Some(offer);
        self.response_wait_secs = 0;
        self.block_retry = None;
        self.activation_pending = false;
        let action = self.cluster.start_query();
        self.process_action(action)
    }

    /// Set the OTA server's IEEE address (UpgradeServerID attribute).
    pub fn set_upgrade_server_id(&mut self, ieee: u64) {
        self.cluster.set_upgrade_server_id(ieee);
    }

    /// Process an OtaAction into a StackEvent and/or queue an outgoing frame.
    fn process_action(&mut self, action: OtaAction) -> Option<StackEvent> {
        match action {
            OtaAction::SendQuery(req) => {
                // Reset download context for new OTA session; a suspended
                // download is kept until the server's offer is known.
                if !self.suspended {
                    self.download_ctx.reset();
                }
                self.block_retry = None;
                self.build_and_queue_request(ota::CMD_QUERY_NEXT_IMAGE_REQUEST, &req);
                None
            }
            OtaAction::SendBlockRequest(mut req) => {
                let mut announce = req.file_offset == 0;
                if req.file_offset == 0 && (!self.download_ctx.slot_erased || self.suspended) {
                    let offer = OtaImageOffer {
                        manufacturer_code: req.manufacturer_code,
                        image_type: req.image_type,
                        file_version: req.file_version,
                        image_size: self.cluster.state().download_total(),
                    };
                    if !self.config.auto_accept {
                        if self.accepted_image != Some(offer) {
                            // Pause until the application accepts this exact
                            // image; the cluster returns to idle meanwhile.
                            self.accepted_image = None;
                            self.offered_image = Some(offer);
                            self.cluster.abort();
                            return Some(StackEvent::OtaImageAvailable {
                                version: offer.file_version,
                                size: offer.image_size,
                            });
                        }
                        // Already reported and accepted: start silently.
                        announce = false;
                    }

                    if self.suspended {
                        if self.download_ctx.offer == Some(offer) {
                            let saved = self.download_ctx.file_offset;
                            match self.cluster.resume_download(saved) {
                                OtaAction::SendBlockRequest(resumed) => {
                                    log::info!("[OTA] Resuming download at offset {}", saved);
                                    self.suspended = false;
                                    req = resumed;
                                }
                                _ => self.discard_suspended(),
                            }
                        } else {
                            log::info!("[OTA] Different image offered; discarding staged data");
                            self.discard_suspended();
                        }
                    }

                    if !self.download_ctx.slot_erased {
                        match self.writer.erase_slot() {
                            Ok(()) => {
                                log::info!("[OTA] Flash slot erased, ready for download");
                                self.download_ctx.slot_erased = true;
                                self.download_ctx.offer = Some(offer);
                            }
                            Err(e) => {
                                log::warn!("[OTA] Erase slot failed: {:?}", e);
                                let fail_action = self.cluster.mark_failed();
                                return self.process_action(fail_action);
                            }
                        }
                    }
                } else if !self.config.auto_accept {
                    announce = false;
                }
                let zcl_seq = self.next_seq();
                self.block_retry = Some(BlockRetry {
                    request: req.clone(),
                    zcl_seq,
                    elapsed_secs: 0,
                    retries_sent: 0,
                });
                self.build_and_queue_block_request(&req, zcl_seq);
                // Emit OtaImageAvailable on first block request (start of an
                // auto-accepted download).
                if announce {
                    let total = match self.cluster.state() {
                        OtaState::Downloading { total_size, .. } => total_size,
                        _ => 0,
                    };
                    let version = self.cluster.target_version();
                    return Some(StackEvent::OtaImageAvailable {
                        version,
                        size: total,
                    });
                }
                None
            }
            OtaAction::WriteBlock { offset, data } => match self.write_ota_block(offset, &data) {
                Ok(()) => {
                    let progress = self.cluster.progress_percent();
                    if self.cluster.is_download_complete() {
                        self.cluster.mark_download_complete();
                        let verified = if self.download_ctx.payload_complete() {
                            self.writer.verify(self.download_ctx.firmware_size, None)
                        } else {
                            log::warn!("[OTA] Image ended without a complete Upgrade Image");
                            Err(crate::firmware_writer::FirmwareError::VerifyFailed)
                        };
                        match verified {
                            Ok(()) => {
                                let action = self.cluster.mark_verified();
                                let status = self.process_action(action);
                                debug_assert!(status.is_none());
                                if status.is_some() {
                                    return status;
                                }
                            }
                            Err(e) => {
                                log::warn!("[OTA] Verify failed: {:?}", e);
                                let action = self.cluster.mark_failed();
                                return self.process_action(action);
                            }
                        }
                    } else {
                        // Queue the next stop-and-wait request now. The
                        // transport that delivered this block can send it
                        // before the application enters another poll cycle.
                        let action = self.cluster.next_block_request();
                        let followup = self.process_action(action);
                        debug_assert!(followup.is_none());
                        if followup.is_some() {
                            return followup;
                        }
                    }
                    Some(StackEvent::OtaProgress { percent: progress })
                }
                Err(e) => {
                    log::warn!("[OTA] Write failed at offset {}: {:?}", offset, e);
                    let fail_action = self.cluster.mark_failed();
                    self.process_action(fail_action);
                    Some(StackEvent::OtaFailed)
                }
            },
            OtaAction::SendEndRequest(req) => {
                self.block_retry = None;
                // The accepted image's download has finished either way.
                self.accepted_image = None;
                self.end_request_retries = 0;
                let failed = req.status != 0;
                self.build_and_queue_end_request(&req);
                failed.then_some(StackEvent::OtaFailed)
            }
            OtaAction::ActivateImage => {
                self.block_retry = None;
                self.accepted_image = None;
                self.activation_pending = true;
                Some(StackEvent::OtaComplete)
            }
            OtaAction::Wait(secs) => Some(StackEvent::OtaDelayedActivation { delay_secs: secs }),
            OtaAction::None => None,
        }
    }

    /// Stream one OTA file block through the bounded image parser.
    ///
    /// The OTA file format is `[header][sub-element]*`, each sub-element
    /// being `tag(2) + length(4) + data(length)`. Only the payload of the
    /// Upgrade Image sub-element (tag `0x0000`) is written to flash. All file
    /// offsets are overflow-checked and bounded by the offered image size, so
    /// a hostile image cannot wrap the cursor or loop forever.
    fn write_ota_block(
        &mut self,
        ota_offset: u32,
        data: &[u8],
    ) -> Result<(), crate::firmware_writer::FirmwareError> {
        use crate::firmware_writer::FirmwareError;

        let Some(offer) = self.download_ctx.offer else {
            return Err(FirmwareError::VerifyFailed);
        };
        if ota_offset != self.download_ctx.file_offset {
            log::warn!("[OTA] Non-contiguous block at offset {}", ota_offset);
            return Err(FirmwareError::VerifyFailed);
        }
        let mut rest = data;
        while !rest.is_empty() {
            let pos = self.download_ctx.file_offset;
            let consumed = match self.download_ctx.phase {
                OtaParsePhase::Header => self.consume_header(rest)?,
                OtaParsePhase::SubElement => self.consume_sub_element(rest, &offer)?,
                OtaParsePhase::Payload => {
                    let ctx = &self.download_ctx;
                    // `payload_start + firmware_size <= image_size` was
                    // checked when the sub-element header was accepted.
                    let payload_end = ctx.payload_start + ctx.firmware_size;
                    if pos >= payload_end {
                        self.download_ctx.phase = OtaParsePhase::Trailer;
                        0
                    } else {
                        let n = rest.len().min((payload_end - pos) as usize);
                        let flash_offset = pos - ctx.payload_start;
                        self.writer.write_block(flash_offset, &rest[..n])?;
                        self.download_ctx.firmware_written = flash_offset + n as u32;
                        if pos + n as u32 == payload_end {
                            self.download_ctx.phase = OtaParsePhase::Trailer;
                        }
                        n
                    }
                }
                // Sub-elements after the Upgrade Image (signature, integrity
                // code, vendor data) are not verified — see module docs.
                OtaParsePhase::Trailer => rest.len(),
            };
            rest = &rest[consumed..];
            // `consumed <= data.len() <= 64` and the cluster bounds the block
            // end by the offered image size, so this cannot overflow.
            self.download_ctx.file_offset = pos + consumed as u32;
        }
        Ok(())
    }

    /// Buffer and validate the OTA file header. Returns bytes consumed.
    fn consume_header(
        &mut self,
        data: &[u8],
    ) -> Result<usize, crate::firmware_writer::FirmwareError> {
        use crate::firmware_writer::FirmwareError;
        use zigbee_zcl::clusters::ota_image::OTA_HEADER_MIN_SIZE;

        let buf = &mut self.download_ctx.header_buf;
        // The header length lives at bytes 6..8; buffer at least that much,
        // then exactly the declared header.
        let target = if buf.len() < 8 {
            8
        } else {
            let header_len = u16::from_le_bytes([buf[6], buf[7]]) as usize;
            if !(OTA_HEADER_MIN_SIZE..=OTA_HEADER_BUFFER).contains(&header_len) {
                log::warn!("[OTA] Unsupported header length {}", header_len);
                return Err(FirmwareError::VerifyFailed);
            }
            header_len
        };
        let n = data.len().min(target - buf.len());
        // Cannot fail: `target <= OTA_HEADER_BUFFER`.
        let _ = buf.extend_from_slice(&data[..n]);
        if buf.len() < 8 || buf.len() < target || target == 8 {
            return Ok(n);
        }

        let header = match OtaImageHeader::parse(buf) {
            Ok((header, _)) => header,
            Err(e) => {
                log::warn!("[OTA] Header parse failed: {:?}", e);
                return Err(FirmwareError::VerifyFailed);
            }
        };
        // Identity, version and size against the Query Next Image Response
        // that started this download (which already required a newer
        // version), and the hardware-version range.
        if !self.cluster.validate_image_header(&header)
            || u32::from(header.header_length) > header.total_image_size
        {
            log::warn!(
                "[OTA] Header rejected: mfg=0x{:04X} type=0x{:04X} version=0x{:08X} size={}",
                header.manufacturer_code,
                header.image_type,
                header.file_version,
                header.total_image_size
            );
            return Err(FirmwareError::VerifyFailed);
        }

        log::info!(
            "[OTA] Header parsed: version=0x{:08X} header={}B total={}B",
            header.file_version,
            header.header_length,
            header.total_image_size,
        );
        let ctx = &mut self.download_ctx;
        ctx.header_buf.clear();
        ctx.next_element = u32::from(header.header_length);
        ctx.phase = OtaParsePhase::SubElement;
        Ok(n)
    }

    /// Skip to and parse the next sub-element header. Returns bytes consumed.
    fn consume_sub_element(
        &mut self,
        data: &[u8],
        offer: &OtaImageOffer,
    ) -> Result<usize, crate::firmware_writer::FirmwareError> {
        use crate::firmware_writer::FirmwareError;

        let slot_size = self.writer.slot_size();
        let ctx = &mut self.download_ctx;
        let pos = ctx.file_offset;
        if pos < ctx.next_element {
            // Skip the body of a sub-element we do not consume.
            return Ok(data.len().min((ctx.next_element - pos) as usize));
        }
        let want = OTA_SUB_ELEMENT_HEADER_LEN as usize - ctx.header_buf.len();
        let n = data.len().min(want);
        let _ = ctx.header_buf.extend_from_slice(&data[..n]);
        if ctx.header_buf.len() < OTA_SUB_ELEMENT_HEADER_LEN as usize {
            return Ok(n);
        }
        let b = &ctx.header_buf;
        let tag = u16::from_le_bytes([b[0], b[1]]);
        let length = u32::from_le_bytes([b[2], b[3], b[4], b[5]]);
        ctx.header_buf.clear();

        let element_end = ctx
            .next_element
            .checked_add(OTA_SUB_ELEMENT_HEADER_LEN)
            .and_then(|start| start.checked_add(length))
            .filter(|end| *end <= offer.image_size);
        let Some(element_end) = element_end else {
            log::warn!("[OTA] Sub-element 0x{:04X} overruns the image", tag);
            return Err(FirmwareError::VerifyFailed);
        };

        if tag == OTA_TAG_UPGRADE_IMAGE {
            if length == 0 {
                log::warn!("[OTA] Empty Upgrade Image sub-element");
                return Err(FirmwareError::VerifyFailed);
            }
            if length > slot_size {
                log::warn!(
                    "[OTA] Firmware too large: {}B > slot {}B",
                    length,
                    slot_size
                );
                return Err(FirmwareError::ImageTooLarge);
            }
            ctx.payload_start = ctx.next_element + OTA_SUB_ELEMENT_HEADER_LEN;
            ctx.firmware_size = length;
            ctx.phase = OtaParsePhase::Payload;
        } else {
            ctx.next_element = element_end;
        }
        Ok(n)
    }

    fn build_and_queue_request(&mut self, cmd_id: CommandId, req: &QueryNextImageRequest) {
        let seq = self.next_seq();
        let mut frame =
            ZclFrame::new_cluster_specific(seq, cmd_id, ClusterDirection::ClientToServer, false);
        let mut buf = [0u8; 16];
        let len = req.serialize(&mut buf);
        for &b in &buf[..len] {
            let _ = frame.payload.push(b);
        }
        self.queue_frame(frame);
    }

    fn build_and_queue_block_request(&mut self, req: &ImageBlockRequest, zcl_seq: u8) {
        let mut frame = ZclFrame::new_cluster_specific(
            zcl_seq,
            ota::CMD_IMAGE_BLOCK_REQUEST,
            ClusterDirection::ClientToServer,
            false,
        );
        let mut buf = [0u8; 16];
        let len = req.serialize(&mut buf);
        for &b in &buf[..len] {
            let _ = frame.payload.push(b);
        }
        self.queue_frame(frame);
    }

    fn build_and_queue_end_request(&mut self, req: &UpgradeEndRequest) {
        let seq = self.next_seq();
        let mut frame = ZclFrame::new_cluster_specific(
            seq,
            ota::CMD_UPGRADE_END_REQUEST,
            ClusterDirection::ClientToServer,
            false,
        );
        let mut buf = [0u8; 12];
        let len = req.serialize(&mut buf);
        for &b in &buf[..len] {
            let _ = frame.payload.push(b);
        }
        self.queue_frame(frame);
    }

    fn queue_frame(&mut self, frame: ZclFrame) {
        let mut zcl_buf = [0u8; 128];
        if let Ok(len) = frame.serialize(&mut zcl_buf) {
            let mut data = heapless::Vec::new();
            for &b in &zcl_buf[..len] {
                let _ = data.push(b);
            }
            self.pending_frame = Some(PendingOtaFrame {
                zcl_data: data,
                endpoint: self.config.endpoint,
                cluster_id: zigbee_zcl::ClusterId::OTA_UPGRADE.0,
            });
        }
    }
}
