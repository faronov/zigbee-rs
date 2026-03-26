//! OTA Manager — runtime integration for OTA firmware upgrades.
//!
//! Combines the ZCL OTA cluster (state machine + command parsing) with
//! a FirmwareWriter (platform flash abstraction) to handle the complete
//! OTA upgrade flow.
//!
//! Key responsibilities:
//! - Erases flash slot before download starts
//! - Parses OTA image header from first blocks, validates manufacturer/image_type
//! - Strips OTA header + sub-element header, writes only firmware payload to flash
//! - Tracks actual firmware bytes written for correct verification
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
    /// Pending outgoing frame (queued for sending in tick()).
    pending_frame: Option<PendingOtaFrame>,
    /// Whether we need to request the next block after a write.
    need_next_block: bool,
    /// ZCL sequence counter (borrowed from device).
    zcl_seq: u8,
    /// Download context — tracks header parsing and payload offset.
    download_ctx: OtaDownloadCtx,
}

/// Tracks OTA file header parsing and firmware write offset during download.
struct OtaDownloadCtx {
    /// Whether the OTA image header has been parsed from initial blocks.
    header_parsed: bool,
    /// Buffer for accumulating header bytes from early blocks.
    header_buf: heapless::Vec<u8, 72>,
    /// Bytes to skip at start of OTA file (header_length + 6 for sub-element header).
    skip_bytes: u32,
    /// Actual firmware payload size (total_image_size - skip_bytes).
    firmware_size: u32,
    /// Firmware bytes actually written to flash.
    firmware_written: u32,
    /// Whether erase_slot() has been called.
    slot_erased: bool,
}

impl OtaDownloadCtx {
    fn new() -> Self {
        Self {
            header_parsed: false,
            header_buf: heapless::Vec::new(),
            skip_bytes: 0,
            firmware_size: 0,
            firmware_written: 0,
            slot_erased: false,
        }
    }

    fn reset(&mut self) {
        self.header_parsed = false;
        self.header_buf.clear();
        self.skip_bytes = 0;
        self.firmware_size = 0;
        self.firmware_written = 0;
        self.slot_erased = false;
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

        Self {
            cluster,
            writer,
            config,
            pending_frame: None,
            need_next_block: false,
            zcl_seq: 0,
            download_ctx: OtaDownloadCtx::new(),
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

    /// Initiate an OTA image query.
    pub fn start_query(&mut self) -> Option<StackEvent> {
        let action = self.cluster.start_query();
        self.process_action(action)
    }

    /// Process an incoming OTA server→client command.
    pub fn handle_incoming(&mut self, cmd_id: u8, payload: &[u8]) -> Option<StackEvent> {
        let action = self.cluster.process_server_command(cmd_id, payload);
        self.process_action(action)
    }

    /// Tick the OTA engine (called from runtime tick).
    pub fn tick(&mut self, elapsed_secs: u16) -> Option<StackEvent> {
        // Handle pending next-block request after a write
        if self.need_next_block {
            self.need_next_block = false;

            // Check if download is complete
            if self.cluster.is_download_complete() {
                // Save the firmware size BEFORE state transition
                let fw_size = self.download_ctx.firmware_size;
                self.cluster.mark_download_complete();
                // Verify using actual firmware bytes written, not OTA file size
                let verify_size = if fw_size > 0 {
                    fw_size
                } else {
                    self.download_ctx.firmware_written
                };
                match self.writer.verify(verify_size, None) {
                    Ok(()) => {
                        let action = self.cluster.mark_verified();
                        return self.process_action(action);
                    }
                    Err(e) => {
                        log::warn!("[OTA] Verify failed: {:?}", e);
                        let _ = self.writer.abort();
                        let action = self.cluster.mark_failed();
                        return self.process_action(action);
                    }
                }
            }

            // Request next block
            let action = self.cluster.next_block_request();
            return self.process_action(action);
        }

        // Handle WaitForData countdown
        let action = self.cluster.tick(elapsed_secs);
        self.process_action(action)
    }

    /// Take the pending outgoing frame (consumed by runtime to send via APS).
    pub fn take_pending_frame(&mut self) -> Option<PendingOtaFrame> {
        self.pending_frame.take()
    }

    /// Abort the current OTA.
    pub fn abort(&mut self) {
        self.cluster.abort();
        let _ = self.writer.abort();
        self.pending_frame = None;
        self.need_next_block = false;
        self.download_ctx.reset();
    }

    /// Process an OtaAction into a StackEvent and/or queue an outgoing frame.
    fn process_action(&mut self, action: OtaAction) -> Option<StackEvent> {
        match action {
            OtaAction::SendQuery(req) => {
                // Reset download context for new OTA session
                self.download_ctx.reset();
                self.build_and_queue_request(ota::CMD_QUERY_NEXT_IMAGE_REQUEST, &req);
                None
            }
            OtaAction::SendBlockRequest(req) => {
                // Erase slot before first block if not done yet
                if !self.download_ctx.slot_erased {
                    match self.writer.erase_slot() {
                        Ok(()) => {
                            log::info!("[OTA] Flash slot erased, ready for download");
                            self.download_ctx.slot_erased = true;
                        }
                        Err(e) => {
                            log::warn!("[OTA] Erase slot failed: {:?}", e);
                            let fail_action = self.cluster.mark_failed();
                            return self.process_action(fail_action);
                        }
                    }
                }
                self.build_and_queue_block_request(&req);
                // Emit OtaImageAvailable on first block request (start of download)
                if req.file_offset == 0 {
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
                    self.need_next_block = true;
                    let progress = self.cluster.progress_percent();
                    Some(StackEvent::OtaProgress { percent: progress })
                }
                Err(e) => {
                    log::warn!("[OTA] Write failed at offset {}: {:?}", offset, e);
                    let _ = self.writer.abort();
                    let fail_action = self.cluster.mark_failed();
                    self.process_action(fail_action);
                    Some(StackEvent::OtaFailed)
                }
            },
            OtaAction::SendEndRequest(req) => {
                self.build_and_queue_end_request(&req);
                None
            }
            OtaAction::ActivateImage => match self.writer.activate() {
                Ok(()) => Some(StackEvent::OtaComplete),
                Err(e) => {
                    log::warn!("[OTA] Activate failed: {:?}", e);
                    Some(StackEvent::OtaFailed)
                }
            },
            OtaAction::Wait(secs) => Some(StackEvent::OtaDelayedActivation { delay_secs: secs }),
            OtaAction::None => None,
        }
    }

    /// Write an OTA block, handling header parsing and payload stripping.
    ///
    /// The OTA file format is: [OTA header] [sub-element header] [firmware payload]
    /// We parse the header from the first block(s), validate manufacturer/image_type,
    /// then write only the firmware payload bytes to flash.
    fn write_ota_block(
        &mut self,
        ota_offset: u32,
        data: &[u8],
    ) -> Result<(), crate::firmware_writer::FirmwareError> {
        use crate::firmware_writer::FirmwareError;

        if !self.download_ctx.header_parsed {
            // Accumulate bytes for header parsing
            for &b in data {
                let _ = self.download_ctx.header_buf.push(b);
            }

            // Try to parse OTA header once we have minimum header bytes (56)
            if self.download_ctx.header_buf.len() >= 56 {
                match OtaImageHeader::parse(&self.download_ctx.header_buf) {
                    Ok((header, header_len)) => {
                        // Validate manufacturer and image type
                        if header.manufacturer_code != self.config.manufacturer_code {
                            log::warn!(
                                "[OTA] Manufacturer mismatch: got 0x{:04X}, expected 0x{:04X}",
                                header.manufacturer_code,
                                self.config.manufacturer_code
                            );
                            return Err(FirmwareError::VerifyFailed);
                        }
                        if header.image_type != self.config.image_type {
                            log::warn!(
                                "[OTA] Image type mismatch: got 0x{:04X}, expected 0x{:04X}",
                                header.image_type,
                                self.config.image_type
                            );
                            return Err(FirmwareError::VerifyFailed);
                        }

                        // Sub-element header is 6 bytes (tag u16 + length u32)
                        let skip = header_len as u32 + 6;
                        let fw_size = header.total_image_size.saturating_sub(skip);

                        log::info!(
                            "[OTA] Header parsed: version=0x{:08X} header={}B skip={}B firmware={}B",
                            header.file_version,
                            header_len,
                            skip,
                            fw_size,
                        );

                        // Check firmware fits in flash slot
                        if fw_size > self.writer.slot_size() {
                            log::warn!(
                                "[OTA] Firmware too large: {}B > slot {}B",
                                fw_size,
                                self.writer.slot_size()
                            );
                            return Err(FirmwareError::ImageTooLarge);
                        }

                        self.download_ctx.skip_bytes = skip;
                        self.download_ctx.firmware_size = fw_size;
                        self.download_ctx.header_parsed = true;

                        // Write any payload bytes that are past the header in the buffer
                        let buf_len = self.download_ctx.header_buf.len() as u32;
                        if buf_len > skip {
                            let payload_start = skip as usize;
                            let buf_ref = &self.download_ctx.header_buf;
                            // Copy payload bytes to a temp buffer to avoid borrow conflict
                            let mut tmp = [0u8; 72];
                            let plen = buf_ref.len() - payload_start;
                            tmp[..plen].copy_from_slice(&buf_ref[payload_start..]);
                            self.writer.write_block(0, &tmp[..plen])?;
                            self.download_ctx.firmware_written = plen as u32;
                        }
                    }
                    Err(e) => {
                        log::warn!("[OTA] Header parse failed: {:?}", e);
                        return Err(FirmwareError::VerifyFailed);
                    }
                }
            }
            // Still accumulating header bytes — nothing to write yet
            return Ok(());
        }

        // Header already parsed — write firmware payload bytes
        let skip = self.download_ctx.skip_bytes;
        let block_end = ota_offset + data.len() as u32;

        if block_end <= skip {
            // Entire block is still in header/sub-element area — skip
            return Ok(());
        }

        let (data_start, flash_offset) = if ota_offset < skip {
            // Block partially overlaps header — write only the payload portion
            let data_skip = (skip - ota_offset) as usize;
            (&data[data_skip..], 0u32)
        } else {
            // Block is entirely in payload area
            (data, ota_offset - skip)
        };

        // Sanity check: don't write past firmware size
        let max_write = self.download_ctx.firmware_size.saturating_sub(flash_offset) as usize;
        let write_data = if data_start.len() > max_write {
            &data_start[..max_write]
        } else {
            data_start
        };

        if !write_data.is_empty() {
            self.writer.write_block(flash_offset, write_data)?;
            self.download_ctx.firmware_written = flash_offset + write_data.len() as u32;
        }

        Ok(())
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

    fn build_and_queue_block_request(&mut self, req: &ImageBlockRequest) {
        let seq = self.next_seq();
        let mut frame = ZclFrame::new_cluster_specific(
            seq,
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
                cluster_id: 0x0019,
            });
        }
    }
}
