//! OTA Upgrade cluster (0x0019) — client-side implementation.
//!
//! Implements the Zigbee OTA client state machine:
//! - Idle → query server for new image
//! - Downloading → block-by-block image transfer
//! - Verifying → check image integrity
//! - WaitingActivate → ready for reboot
//!
//! The runtime's OtaManager drives this state machine and uses
//! a FirmwareWriter to persist the downloaded image to flash.

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// ── Attribute IDs ───────────────────────────────────────────────

pub const ATTR_UPGRADE_SERVER_ID: AttributeId = AttributeId(0x0000);
pub const ATTR_FILE_OFFSET: AttributeId = AttributeId(0x0001);
pub const ATTR_CURRENT_FILE_VERSION: AttributeId = AttributeId(0x0002);
pub const ATTR_CURRENT_STACK_VERSION: AttributeId = AttributeId(0x0003);
pub const ATTR_DOWNLOADED_FILE_VERSION: AttributeId = AttributeId(0x0004);
pub const ATTR_DOWNLOADED_STACK_VERSION: AttributeId = AttributeId(0x0005);
pub const ATTR_IMAGE_UPGRADE_STATUS: AttributeId = AttributeId(0x0006);
pub const ATTR_MANUFACTURER_ID: AttributeId = AttributeId(0x0007);
pub const ATTR_IMAGE_TYPE_ID: AttributeId = AttributeId(0x0008);
pub const ATTR_MIN_BLOCK_PERIOD: AttributeId = AttributeId(0x0009);

// ── Command IDs ─────────────────────────────────────────────────

// Client → Server
pub const CMD_QUERY_NEXT_IMAGE_REQUEST: CommandId = CommandId(0x01);
pub const CMD_IMAGE_BLOCK_REQUEST: CommandId = CommandId(0x03);
pub const CMD_IMAGE_PAGE_REQUEST: CommandId = CommandId(0x04);
pub const CMD_UPGRADE_END_REQUEST: CommandId = CommandId(0x06);

// Server → Client
pub const CMD_IMAGE_NOTIFY: CommandId = CommandId(0x00);
pub const CMD_QUERY_NEXT_IMAGE_RESPONSE: CommandId = CommandId(0x02);
pub const CMD_IMAGE_BLOCK_RESPONSE: CommandId = CommandId(0x05);
pub const CMD_UPGRADE_END_RESPONSE: CommandId = CommandId(0x07);

// ── Image Upgrade Status values ─────────────────────────────────

pub const STATUS_NORMAL: u8 = 0x00;
pub const STATUS_DOWNLOAD_IN_PROGRESS: u8 = 0x01;
pub const STATUS_DOWNLOAD_COMPLETE: u8 = 0x02;
pub const STATUS_WAITING_TO_UPGRADE: u8 = 0x03;
pub const STATUS_COUNT_DOWN: u8 = 0x04;
pub const STATUS_WAIT_FOR_MORE: u8 = 0x05;

// ── Default block size ──────────────────────────────────────────

/// Safe block size that fits in a single MAC frame without APS fragmentation.
pub const DEFAULT_BLOCK_SIZE: u8 = 48;

// ── OTA State Machine ───────────────────────────────────────────

/// OTA client state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaState {
    /// No OTA in progress.
    Idle,
    /// Query Next Image Request sent, waiting for response.
    QuerySent,
    /// Downloading image block-by-block.
    Downloading {
        /// Current file offset.
        offset: u32,
        /// Total image size.
        total_size: u32,
    },
    /// Download complete, verifying image.
    Verifying,
    /// Image verified, waiting for activation.
    WaitingActivate,
    /// Waiting for server-specified delay before retrying.
    WaitForData {
        /// Seconds to wait.
        delay_secs: u32,
        /// Timer countdown.
        elapsed: u32,
        /// Saved download offset to resume from.
        download_offset: u32,
        /// Saved download total size.
        download_total: u32,
    },
    /// OTA completed successfully.
    Done,
    /// OTA failed.
    Failed,
}

impl OtaState {
    /// Get the total download size (only valid during Downloading).
    pub fn download_total(&self) -> u32 {
        match self {
            OtaState::Downloading { total_size, .. } => *total_size,
            _ => 0,
        }
    }
}

// ── Actions returned by the OTA engine ──────────────────────────

/// Actions the runtime should perform after processing an OTA command.
#[derive(Debug)]
pub enum OtaAction {
    /// Send a Query Next Image Request.
    SendQuery(QueryNextImageRequest),
    /// Send an Image Block Request.
    SendBlockRequest(ImageBlockRequest),
    /// Write a block of data to the firmware slot.
    WriteBlock {
        offset: u32,
        data: heapless::Vec<u8, 64>,
    },
    /// Send an Upgrade End Request (success or failure).
    SendEndRequest(UpgradeEndRequest),
    /// Activate the new firmware image and reboot.
    ActivateImage,
    /// Wait N seconds before the next action.
    Wait(u32),
    /// Nothing to do.
    None,
}

// ── Command structures ──────────────────────────────────────────

/// Query Next Image Request (client → server).
#[derive(Debug, Clone)]
pub struct QueryNextImageRequest {
    pub field_control: u8,
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub current_file_version: u32,
    pub hardware_version: Option<u16>,
}

impl QueryNextImageRequest {
    /// Serialize into a buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.field_control;
        buf[1..3].copy_from_slice(&self.manufacturer_code.to_le_bytes());
        buf[3..5].copy_from_slice(&self.image_type.to_le_bytes());
        buf[5..9].copy_from_slice(&self.current_file_version.to_le_bytes());
        let mut len = 9;
        if let Some(hw) = self.hardware_version {
            buf[len..len + 2].copy_from_slice(&hw.to_le_bytes());
            len += 2;
        }
        len
    }
}

/// Query Next Image Response (server → client).
#[derive(Debug, Clone)]
pub struct QueryNextImageResponse {
    pub status: u8,
    pub manufacturer_code: Option<u16>,
    pub image_type: Option<u16>,
    pub file_version: Option<u32>,
    pub image_size: Option<u32>,
}

impl QueryNextImageResponse {
    /// Parse from payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let status = data[0];
        if status != 0x00 {
            // No image available
            return Some(Self {
                status,
                manufacturer_code: None,
                image_type: None,
                file_version: None,
                image_size: None,
            });
        }
        // Success response: status(1) + mfg(2) + type(2) + version(4) + size(4) = 13
        if data.len() < 13 {
            return None;
        }
        Some(Self {
            status,
            manufacturer_code: Some(u16::from_le_bytes([data[1], data[2]])),
            image_type: Some(u16::from_le_bytes([data[3], data[4]])),
            file_version: Some(u32::from_le_bytes([data[5], data[6], data[7], data[8]])),
            image_size: Some(u32::from_le_bytes([data[9], data[10], data[11], data[12]])),
        })
    }
}

/// Image Block Request (client → server).
#[derive(Debug, Clone)]
pub struct ImageBlockRequest {
    pub field_control: u8,
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub file_version: u32,
    pub file_offset: u32,
    pub max_data_size: u8,
}

impl ImageBlockRequest {
    /// Serialize into a buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.field_control;
        buf[1..3].copy_from_slice(&self.manufacturer_code.to_le_bytes());
        buf[3..5].copy_from_slice(&self.image_type.to_le_bytes());
        buf[5..9].copy_from_slice(&self.file_version.to_le_bytes());
        buf[9..13].copy_from_slice(&self.file_offset.to_le_bytes());
        buf[13] = self.max_data_size;
        14
    }
}

/// Image Block Response (server → client) — success variant.
#[derive(Debug, Clone)]
pub struct ImageBlockResponse {
    pub status: u8,
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub file_version: u32,
    pub file_offset: u32,
    pub data_size: u8,
    pub data: heapless::Vec<u8, 64>,
}

/// Image Block Response — WaitForData variant.
#[derive(Debug, Clone)]
pub struct ImageBlockWaitForData {
    pub current_time: u32,
    pub request_time: u32,
    pub minimum_block_period: u16,
}

/// Parsed Image Block Response (either success or wait).
#[derive(Debug, Clone)]
pub enum ParsedBlockResponse {
    Success(ImageBlockResponse),
    WaitForData(ImageBlockWaitForData),
    Error(u8),
}

impl ParsedBlockResponse {
    /// Parse from payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let status = data[0];
        match status {
            0x00 => {
                // Success
                if data.len() < 14 {
                    return None;
                }
                let mfr = u16::from_le_bytes([data[1], data[2]]);
                let img_type = u16::from_le_bytes([data[3], data[4]]);
                let version = u32::from_le_bytes([data[5], data[6], data[7], data[8]]);
                let offset = u32::from_le_bytes([data[9], data[10], data[11], data[12]]);
                let data_size = data[13];
                // Validate that the payload actually contains data_size bytes
                if data.len() < 14 + data_size as usize {
                    log::warn!(
                        "[OTA] Block truncated: expected {} bytes, got {}",
                        data_size,
                        data.len() - 14
                    );
                    return None;
                }
                let mut block_data = heapless::Vec::new();
                for &b in &data[14..14 + data_size as usize] {
                    let _ = block_data.push(b);
                }
                Some(Self::Success(ImageBlockResponse {
                    status,
                    manufacturer_code: mfr,
                    image_type: img_type,
                    file_version: version,
                    file_offset: offset,
                    data_size,
                    data: block_data,
                }))
            }
            0x97 => {
                // WAIT_FOR_DATA
                if data.len() < 11 {
                    return None;
                }
                Some(Self::WaitForData(ImageBlockWaitForData {
                    current_time: u32::from_le_bytes([data[1], data[2], data[3], data[4]]),
                    request_time: u32::from_le_bytes([data[5], data[6], data[7], data[8]]),
                    minimum_block_period: u16::from_le_bytes([data[9], data[10]]),
                }))
            }
            _ => Some(Self::Error(status)),
        }
    }
}

/// Upgrade End Request (client → server).
#[derive(Debug, Clone)]
pub struct UpgradeEndRequest {
    pub status: u8,
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub file_version: u32,
}

impl UpgradeEndRequest {
    /// Serialize into a buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.status;
        buf[1..3].copy_from_slice(&self.manufacturer_code.to_le_bytes());
        buf[3..5].copy_from_slice(&self.image_type.to_le_bytes());
        buf[5..9].copy_from_slice(&self.file_version.to_le_bytes());
        9
    }
}

/// Upgrade End Response (server → client).
#[derive(Debug, Clone)]
pub struct UpgradeEndResponse {
    pub manufacturer_code: u16,
    pub image_type: u16,
    pub file_version: u32,
    pub current_time: u32,
    pub upgrade_time: u32,
}

impl UpgradeEndResponse {
    /// Parse from payload bytes.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 16 {
            return None;
        }
        Some(Self {
            manufacturer_code: u16::from_le_bytes([data[0], data[1]]),
            image_type: u16::from_le_bytes([data[2], data[3]]),
            file_version: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            current_time: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            upgrade_time: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
        })
    }
}

// ── OTA Cluster ─────────────────────────────────────────────────

/// OTA Upgrade cluster (client-side).
///
/// Manages OTA attributes and provides command parsing/building.
/// The actual download state machine is driven by the runtime's OtaManager.
pub struct OtaCluster {
    store: AttributeStore<12>,
    state: OtaState,
    manufacturer_code: u16,
    image_type: u16,
    current_version: u32,
    /// Target version being downloaded (set by query response).
    target_version: u32,
    /// Total image size being downloaded.
    target_size: u32,
    /// Block size to request.
    block_size: u8,
}

impl OtaCluster {
    pub fn new(manufacturer_code: u16, image_type: u16, current_version: u32) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_UPGRADE_SERVER_ID,
                data_type: ZclDataType::IeeeAddr,
                access: AttributeAccess::ReadOnly,
                name: "UpgradeServerID",
            },
            ZclValue::IeeeAddr(0xFFFFFFFFFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_FILE_OFFSET,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "FileOffset",
            },
            ZclValue::U32(0xFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_FILE_VERSION,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "CurrentFileVersion",
            },
            ZclValue::U32(current_version),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_CURRENT_STACK_VERSION,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "CurrentZigbeeStackVersion",
            },
            ZclValue::U16(0x0002), // Zigbee PRO
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOWNLOADED_FILE_VERSION,
                data_type: ZclDataType::U32,
                access: AttributeAccess::ReadOnly,
                name: "DownloadedFileVersion",
            },
            ZclValue::U32(0xFFFFFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_DOWNLOADED_STACK_VERSION,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "DownloadedZigbeeStackVersion",
            },
            ZclValue::U16(0xFFFF),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_IMAGE_UPGRADE_STATUS,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "ImageUpgradeStatus",
            },
            ZclValue::Enum8(STATUS_NORMAL),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MANUFACTURER_ID,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ManufacturerID",
            },
            ZclValue::U16(manufacturer_code),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_IMAGE_TYPE_ID,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "ImageTypeID",
            },
            ZclValue::U16(image_type),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_BLOCK_PERIOD,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MinimumBlockPeriod",
            },
            ZclValue::U16(0),
        );
        Self {
            store,
            state: OtaState::Idle,
            manufacturer_code,
            image_type,
            current_version,
            target_version: 0,
            target_size: 0,
            block_size: DEFAULT_BLOCK_SIZE,
        }
    }

    /// Get the current OTA state.
    pub fn state(&self) -> OtaState {
        self.state
    }

    /// Get the target firmware version being downloaded.
    pub fn target_version(&self) -> u32 {
        self.target_version
    }

    /// Set the block size for image block requests.
    pub fn set_block_size(&mut self, size: u8) {
        self.block_size = size.min(64);
    }

    /// Build a Query Next Image Request to initiate an OTA check.
    pub fn start_query(&mut self) -> OtaAction {
        self.state = OtaState::QuerySent;
        let _ = self
            .store
            .set(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        OtaAction::SendQuery(QueryNextImageRequest {
            field_control: 0x00,
            manufacturer_code: self.manufacturer_code,
            image_type: self.image_type,
            current_file_version: self.current_version,
            hardware_version: None,
        })
    }

    /// Process an incoming server→client OTA command.
    ///
    /// Returns the action(s) the runtime should perform.
    pub fn process_server_command(&mut self, cmd_id: u8, payload: &[u8]) -> OtaAction {
        match cmd_id {
            0x00 => self.handle_image_notify(payload),
            0x02 => self.handle_query_response(payload),
            0x05 => self.handle_block_response(payload),
            0x07 => self.handle_end_response(payload),
            _ => {
                log::warn!("[OTA] Unknown server command: 0x{:02X}", cmd_id);
                OtaAction::None
            }
        }
    }

    /// Tick the OTA engine (called periodically).
    /// Handles WaitForData countdown and resumes download.
    pub fn tick(&mut self, elapsed_secs: u16) -> OtaAction {
        match self.state {
            OtaState::WaitForData {
                delay_secs,
                elapsed,
                download_offset,
                download_total,
            } => {
                let new_elapsed = elapsed + elapsed_secs as u32;
                if new_elapsed >= delay_secs {
                    // Timer expired — restore Downloading state and retry
                    self.state = OtaState::Downloading {
                        offset: download_offset,
                        total_size: download_total,
                    };
                    self.build_block_request(download_offset, download_total)
                } else {
                    self.state = OtaState::WaitForData {
                        delay_secs,
                        elapsed: new_elapsed,
                        download_offset,
                        download_total,
                    };
                    OtaAction::None
                }
            }
            _ => OtaAction::None,
        }
    }

    /// Download progress as percentage (0-100).
    pub fn progress_percent(&self) -> u8 {
        match self.state {
            OtaState::Downloading { offset, total_size } if total_size > 0 => {
                ((offset as u64 * 100) / total_size as u64) as u8
            }
            OtaState::Verifying | OtaState::WaitingActivate | OtaState::Done => 100,
            _ => 0,
        }
    }

    /// Abort the current OTA operation.
    pub fn abort(&mut self) {
        self.state = OtaState::Idle;
        let _ = self
            .store
            .set(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        let _ = self.store.set(ATTR_FILE_OFFSET, ZclValue::U32(0xFFFFFFFF));
    }

    /// Mark download as complete and transition to Verifying.
    pub fn mark_download_complete(&mut self) {
        self.state = OtaState::Verifying;
        let _ = self.store.set(
            ATTR_IMAGE_UPGRADE_STATUS,
            ZclValue::Enum8(STATUS_DOWNLOAD_COMPLETE),
        );
        let _ = self.store.set(
            ATTR_DOWNLOADED_FILE_VERSION,
            ZclValue::U32(self.target_version),
        );
    }

    /// Mark verification passed, move to WaitingActivate.
    pub fn mark_verified(&mut self) -> OtaAction {
        self.state = OtaState::WaitingActivate;
        let _ = self.store.set(
            ATTR_IMAGE_UPGRADE_STATUS,
            ZclValue::Enum8(STATUS_WAITING_TO_UPGRADE),
        );
        OtaAction::SendEndRequest(UpgradeEndRequest {
            status: 0x00, // Success
            manufacturer_code: self.manufacturer_code,
            image_type: self.image_type,
            file_version: self.target_version,
        })
    }

    /// Mark OTA as failed.
    pub fn mark_failed(&mut self) -> OtaAction {
        let version = self.target_version;
        self.state = OtaState::Failed;
        let _ = self
            .store
            .set(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        OtaAction::SendEndRequest(UpgradeEndRequest {
            status: 0x96, // INVALID_IMAGE per ZCL spec §11.13.9.5
            manufacturer_code: self.manufacturer_code,
            image_type: self.image_type,
            file_version: version,
        })
    }

    // ── Private command handlers ─────────────────────────────

    fn handle_image_notify(&mut self, _payload: &[u8]) -> OtaAction {
        log::info!("[OTA] Image Notify received — starting query");
        self.start_query()
    }

    fn handle_query_response(&mut self, payload: &[u8]) -> OtaAction {
        // State guard: only accept query response when we're waiting for one
        if self.state != OtaState::QuerySent {
            log::warn!(
                "[OTA] Query Response in wrong state {:?}, ignoring",
                self.state
            );
            return OtaAction::None;
        }

        let resp = match QueryNextImageResponse::parse(payload) {
            Some(r) => r,
            None => {
                log::warn!("[OTA] Failed to parse Query Response");
                self.state = OtaState::Idle;
                return OtaAction::None;
            }
        };

        if resp.status != 0x00 {
            log::info!(
                "[OTA] No new image available (status=0x{:02X})",
                resp.status
            );
            self.state = OtaState::Idle;
            return OtaAction::None;
        }

        let version = resp.file_version.unwrap_or(0);
        let size = resp.image_size.unwrap_or(0);

        log::info!(
            "[OTA] New image available: version=0x{:08X} size={}",
            version,
            size
        );

        self.target_version = version;
        self.target_size = size;
        self.state = OtaState::Downloading {
            offset: 0,
            total_size: size,
        };

        let _ = self.store.set(
            ATTR_IMAGE_UPGRADE_STATUS,
            ZclValue::Enum8(STATUS_DOWNLOAD_IN_PROGRESS),
        );
        let _ = self.store.set(ATTR_FILE_OFFSET, ZclValue::U32(0));

        self.build_block_request(0, size)
    }

    fn handle_block_response(&mut self, payload: &[u8]) -> OtaAction {
        // State guard: only accept block responses during download or wait
        match self.state {
            OtaState::Downloading { .. } | OtaState::WaitForData { .. } => {}
            _ => {
                log::warn!(
                    "[OTA] Block Response in wrong state {:?}, ignoring",
                    self.state
                );
                return OtaAction::None;
            }
        }

        let parsed = match ParsedBlockResponse::parse(payload) {
            Some(p) => p,
            None => {
                log::warn!("[OTA] Failed to parse Block Response");
                return OtaAction::None;
            }
        };

        match parsed {
            ParsedBlockResponse::Success(block) => {
                let new_offset = block.file_offset + block.data_size as u32;

                // Update state
                let total = self.target_size;
                self.state = OtaState::Downloading {
                    offset: new_offset,
                    total_size: total,
                };
                let _ = self.store.set(ATTR_FILE_OFFSET, ZclValue::U32(new_offset));

                log::debug!(
                    "[OTA] Block: offset={} size={} progress={}%",
                    block.file_offset,
                    block.data_size,
                    self.progress_percent()
                );

                // Return write action — runtime will write then request next block
                OtaAction::WriteBlock {
                    offset: block.file_offset,
                    data: block.data,
                }
            }
            ParsedBlockResponse::WaitForData(wait) => {
                let delay = wait.minimum_block_period.max(1) as u32;
                log::debug!("[OTA] Server says wait {} seconds", delay);
                // Save current download position so we can resume
                let (offset, total) = match self.state {
                    OtaState::Downloading { offset, total_size } => (offset, total_size),
                    _ => (0, 0),
                };
                self.state = OtaState::WaitForData {
                    delay_secs: delay,
                    elapsed: 0,
                    download_offset: offset,
                    download_total: total,
                };
                OtaAction::Wait(delay)
            }
            ParsedBlockResponse::Error(status) => {
                log::warn!("[OTA] Block response error: 0x{:02X}", status);
                // Transition to failed and send Upgrade End Request so server stops waiting
                self.mark_failed()
            }
        }
    }

    fn handle_end_response(&mut self, payload: &[u8]) -> OtaAction {
        let resp = match UpgradeEndResponse::parse(payload) {
            Some(r) => r,
            None => {
                log::warn!("[OTA] Failed to parse End Response");
                return OtaAction::None;
            }
        };

        // upgrade_time == 0 means upgrade immediately
        // upgrade_time == 0xFFFFFFFF means wait for another command
        if resp.upgrade_time == 0 || resp.upgrade_time == resp.current_time {
            log::info!("[OTA] Server says upgrade NOW");
            self.state = OtaState::Done;
            OtaAction::ActivateImage
        } else if resp.upgrade_time == 0xFFFFFFFF {
            log::info!("[OTA] Server says wait for signal");
            OtaAction::None
        } else {
            let delay = resp.upgrade_time.saturating_sub(resp.current_time);
            log::info!("[OTA] Server says upgrade in {} seconds", delay);
            OtaAction::Wait(delay)
        }
    }

    fn build_block_request(&self, offset: u32, _total_size: u32) -> OtaAction {
        OtaAction::SendBlockRequest(ImageBlockRequest {
            field_control: 0x00,
            manufacturer_code: self.manufacturer_code,
            image_type: self.image_type,
            file_version: self.target_version,
            file_offset: offset,
            max_data_size: self.block_size,
        })
    }

    /// Build the next block request for the current download offset.
    pub fn next_block_request(&self) -> OtaAction {
        match self.state {
            OtaState::Downloading { offset, total_size } => {
                if offset >= total_size {
                    OtaAction::None
                } else {
                    self.build_block_request(offset, total_size)
                }
            }
            _ => OtaAction::None,
        }
    }

    /// Check if download is complete (all bytes received).
    pub fn is_download_complete(&self) -> bool {
        match self.state {
            OtaState::Downloading { offset, total_size } => offset >= total_size,
            _ => false,
        }
    }
}

impl Cluster for OtaCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::OTA_UPGRADE
    }

    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // OTA commands are handled by process_server_command() through the runtime,
        // not through the generic Cluster dispatch.
        Err(ZclStatus::UnsupClusterCommand)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }
}
