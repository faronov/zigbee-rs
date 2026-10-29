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
    /// Server deferred activation until a later Upgrade End Response.
    WaitingUpgradeCommand,
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
    /// Image is verified and waiting for the server-selected upgrade time.
    WaitingUpgrade { delay_secs: u32, elapsed: u32 },
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

/// Result of processing one OTA server command.
#[derive(Debug)]
pub struct OtaCommandOutcome {
    /// Action for the runtime to perform.
    pub action: OtaAction,
    /// Whether the command matched the current OTA transaction.
    pub accepted: bool,
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
    /// Returns 0 (nothing written) if `buf` is too small.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 9 + 2 * self.hardware_version.is_some() as usize {
            return 0;
        }
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
    /// Returns 0 (nothing written) if `buf` is too small.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 14 {
            return 0;
        }
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
    ///
    /// Returns `None` for truncated payloads and for success blocks carrying
    /// more than 64 data bytes (the maximum `max_data_size` this client
    /// requests), so data is never silently truncated.
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
                // A block larger than the 64-byte buffer cannot be stored
                // without dropping bytes; reject it rather than truncate.
                let block_data =
                    heapless::Vec::from_slice(&data[14..14 + data_size as usize]).ok()?;
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
    /// Returns 0 (nothing written) if `buf` is too small.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 9 {
            return 0;
        }
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
    /// Hardware version (included in QueryNextImageRequest if set).
    hardware_version: Option<u16>,
    /// Caller-provided random value for QueryJitter selection.
    notify_random: Option<u8>,
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
            hardware_version: None,
            notify_random: None,
        }
    }

    /// Update a server-maintained (read-only) attribute.
    ///
    /// These attributes are read-only over the air, so the ZCL write path
    /// (`set`) would reject them; internal updates use `set_raw`. Failure
    /// means the attribute is not registered, which is a programming error.
    fn set_attr(&mut self, id: AttributeId, value: ZclValue) {
        let result = self.store.set_raw(id, value);
        debug_assert!(result.is_ok(), "OTA attribute 0x{:04X} missing", id.0);
        if result.is_err() {
            log::warn!("[OTA] attribute 0x{:04X} update failed", id.0);
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

    /// Set the hardware version for this device (sent in QueryNextImageRequest).
    pub fn set_hardware_version(&mut self, hw_version: u16) {
        self.hardware_version = Some(hw_version);
    }

    /// Set the UpgradeServerID attribute (IEEE address of the OTA server).
    pub fn set_upgrade_server_id(&mut self, ieee: u64) {
        self.set_attr(ATTR_UPGRADE_SERVER_ID, ZclValue::IeeeAddr(ieee));
    }

    /// Build a Query Next Image Request to initiate an OTA check.
    pub fn start_query(&mut self) -> OtaAction {
        self.state = OtaState::QuerySent;
        self.set_attr(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        let hw = self.hardware_version;
        OtaAction::SendQuery(QueryNextImageRequest {
            field_control: if hw.is_some() { 0x01 } else { 0x00 },
            manufacturer_code: self.manufacturer_code,
            image_type: self.image_type,
            current_file_version: self.current_version,
            hardware_version: hw,
        })
    }

    /// Process an incoming server→client OTA command.
    ///
    /// Returns the action(s) the runtime should perform.
    pub fn process_server_command(&mut self, cmd_id: u8, payload: &[u8]) -> OtaAction {
        self.process_server_command_with_outcome(cmd_id, payload)
            .action
    }

    /// Process an OTA command and report whether it matched the active transaction.
    pub fn process_server_command_with_outcome(
        &mut self,
        cmd_id: u8,
        payload: &[u8],
    ) -> OtaCommandOutcome {
        let previous_state = self.state;
        let action = match cmd_id {
            0x00 => self.handle_image_notify(payload),
            0x02 => self.handle_query_response(payload),
            0x05 => self.handle_block_response(payload),
            0x07 => self.handle_end_response(payload),
            _ => {
                log::warn!("[OTA] Unknown server command: 0x{:02X}", cmd_id);
                OtaAction::None
            }
        };
        let accepted = !matches!(action, OtaAction::None) || self.state != previous_state;
        OtaCommandOutcome { action, accepted }
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
            OtaState::WaitingUpgrade {
                delay_secs,
                elapsed,
            } => {
                let new_elapsed = elapsed.saturating_add(elapsed_secs as u32);
                if new_elapsed >= delay_secs {
                    self.state = OtaState::Done;
                    OtaAction::ActivateImage
                } else {
                    self.state = OtaState::WaitingUpgrade {
                        delay_secs,
                        elapsed: new_elapsed,
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
            OtaState::Verifying
            | OtaState::WaitingActivate
            | OtaState::WaitingUpgradeCommand
            | OtaState::WaitingUpgrade { .. }
            | OtaState::Done => 100,
            _ => 0,
        }
    }

    /// Abort the current OTA operation.
    pub fn abort(&mut self) {
        self.state = OtaState::Idle;
        self.set_attr(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        self.set_attr(ATTR_FILE_OFFSET, ZclValue::U32(0xFFFFFFFF));
    }

    /// Mark download as complete and transition to Verifying.
    pub fn mark_download_complete(&mut self) {
        self.state = OtaState::Verifying;
        self.set_attr(
            ATTR_IMAGE_UPGRADE_STATUS,
            ZclValue::Enum8(STATUS_DOWNLOAD_COMPLETE),
        );
        self.set_attr(
            ATTR_DOWNLOADED_FILE_VERSION,
            ZclValue::U32(self.target_version),
        );
    }

    /// Mark verification passed, move to WaitingActivate.
    pub fn mark_verified(&mut self) -> OtaAction {
        self.state = OtaState::WaitingActivate;
        self.set_attr(
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
        self.fail_with(ZclStatus::InvalidImage as u8)
    }

    /// Fail the transfer, reporting `status` in the Upgrade End Request
    /// (INVALID_IMAGE / ABORT, ZCL r8 §11.13.9).
    fn fail_with(&mut self, status: u8) -> OtaAction {
        self.state = OtaState::Failed;
        self.set_attr(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        OtaAction::SendEndRequest(UpgradeEndRequest {
            status,
            manufacturer_code: self.manufacturer_code,
            image_type: self.image_type,
            file_version: self.target_version,
        })
    }

    /// Cross-check a downloaded image header against the Query Next Image
    /// Response that started the download (ZCL r8 §11.4.2, §11.13.6):
    /// manufacturer code, image type, file version and total image size must
    /// match, and when both the header and this device carry a hardware
    /// version it must be within the header's min/max range.
    pub fn validate_image_header(&self, header: &super::ota_image::OtaImageHeader) -> bool {
        let hw_ok = match (
            self.hardware_version,
            header.min_hardware_version,
            header.max_hardware_version,
        ) {
            (Some(hw), Some(min), Some(max)) => (min..=max).contains(&hw),
            _ => true,
        };
        header.manufacturer_code == self.manufacturer_code
            && header.image_type == self.image_type
            && header.file_version == self.target_version
            && header.total_image_size == self.target_size
            && hw_ok
    }

    /// Supply a random value in `1..=100` used to apply the QueryJitter of
    /// the next Image Notify (ZCL r8 §11.13.3.4). Without one, a notify
    /// is treated as if the device was selected.
    pub fn set_notify_random(&mut self, random: u8) {
        self.notify_random = Some(random);
    }

    // ── Private command handlers ─────────────────────────────

    fn handle_image_notify(&mut self, payload: &[u8]) -> OtaAction {
        // A notify is only acted on when no transfer is in progress; a
        // previously failed transfer may be restarted.
        if !matches!(self.state, OtaState::Idle | OtaState::Failed) {
            log::debug!(
                "[OTA] Image Notify received in state {:?}, ignoring",
                self.state
            );
            return OtaAction::None;
        }

        // ImageNotify (ZCL r8 §11.13.3): payload type, QueryJitter, then
        //   type 1 = +mfg, 2 = +image_type, 3 = +file version.
        // Fields that are present and don't match our device are ignored.
        // An empty notify is tolerated as "type 0, jitter 100" (legacy
        // servers); any present-but-invalid field rejects the command.
        let payload_type = payload.first().copied().unwrap_or(0);
        let jitter = if payload.is_empty() {
            100
        } else {
            payload.get(1).copied().unwrap_or(0)
        };
        const NOTIFY_LEN: [usize; 4] = [0, 4, 6, 10];
        let field = |i: usize| u16::from_le_bytes([payload[i], payload[i + 1]]);
        if payload_type > 3
            || payload.len() < NOTIFY_LEN[payload_type as usize]
            || !(1..=100).contains(&jitter)
        {
            log::debug!("[OTA] Malformed Image Notify");
            return OtaAction::None;
        }
        if payload_type >= 1 && field(2) != 0xFFFF && field(2) != self.manufacturer_code {
            log::debug!("[OTA] ImageNotify mfg not for us");
            return OtaAction::None;
        }
        if payload_type >= 2 && field(4) != 0xFFFF && field(4) != self.image_type {
            log::debug!("[OTA] ImageNotify type not for us");
            return OtaAction::None;
        }
        if payload_type >= 3 {
            let ver = u32::from_le_bytes([payload[6], payload[7], payload[8], payload[9]]);
            if ver != 0xFFFFFFFF && ver == self.current_version {
                log::debug!("[OTA] ImageNotify version matches current, ignoring");
                return OtaAction::None;
            }
        }
        // QueryJitter: only devices drawing a value <= jitter respond.
        if let Some(random) = self.notify_random.take()
            && random > jitter
        {
            log::debug!("[OTA] ImageNotify jitter: not selected");
            return OtaAction::None;
        }

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
        if resp.manufacturer_code != Some(self.manufacturer_code)
            || resp.image_type != Some(self.image_type)
            || version <= self.current_version
            || size == 0
        {
            log::warn!("[OTA] Ignoring invalid Query Response image metadata");
            self.state = OtaState::Idle;
            return OtaAction::None;
        }

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

        self.set_attr(
            ATTR_IMAGE_UPGRADE_STATUS,
            ZclValue::Enum8(STATUS_DOWNLOAD_IN_PROGRESS),
        );
        self.set_attr(ATTR_FILE_OFFSET, ZclValue::U32(0));

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

        let Some(parsed) = ParsedBlockResponse::parse(payload) else {
            // A success block claiming more data than requested cannot be
            // stored without a hole; abort instead of stalling (§11.13.8).
            if payload.first() == Some(&0) && payload.get(13).is_some_and(|&n| n > self.block_size)
            {
                log::warn!("[OTA] Block larger than MaxDataSize, aborting");
                return self.fail_with(ZclStatus::Abort as u8);
            }
            log::warn!("[OTA] Failed to parse Block Response");
            return OtaAction::None;
        };

        match parsed {
            ParsedBlockResponse::Success(block) => {
                let expected_offset = match self.state {
                    OtaState::Downloading { offset, .. }
                    | OtaState::WaitForData {
                        download_offset: offset,
                        ..
                    } => offset,
                    _ => unreachable!(),
                };
                if block.manufacturer_code != self.manufacturer_code
                    || block.image_type != self.image_type
                    || block.file_version != self.target_version
                    || block.file_offset != expected_offset
                {
                    log::warn!("[OTA] Ignoring block for a different image or offset");
                    return OtaAction::None;
                }
                let Some(new_offset) = block.file_offset.checked_add(block.data_size as u32) else {
                    log::warn!("[OTA] Rejecting image block with overflowing offset");
                    return self.mark_failed();
                };
                if block.data_size == 0 || new_offset > self.target_size {
                    log::warn!("[OTA] Rejecting empty or oversized image block");
                    return self.mark_failed();
                }
                // Never advance the offset beyond the bytes actually stored,
                // and never accept more than the requested MaxDataSize.
                if block.data_size > self.block_size || block.data.len() != block.data_size as usize
                {
                    log::warn!("[OTA] Block exceeds requested MaxDataSize, aborting");
                    return self.fail_with(ZclStatus::Abort as u8);
                }

                // Update state
                let total = self.target_size;
                self.state = OtaState::Downloading {
                    offset: new_offset,
                    total_size: total,
                };
                self.set_attr(ATTR_FILE_OFFSET, ZclValue::U32(new_offset));

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
                let delay = wait.request_time.saturating_sub(wait.current_time);
                self.set_attr(
                    ATTR_MIN_BLOCK_PERIOD,
                    ZclValue::U16(wait.minimum_block_period),
                );
                log::debug!("[OTA] Server says wait {} seconds", delay);
                // Save current download position so we can resume
                let (offset, total) = match self.state {
                    OtaState::Downloading { offset, total_size } => (offset, total_size),
                    OtaState::WaitForData {
                        download_offset,
                        download_total,
                        ..
                    } => (download_offset, download_total),
                    _ => unreachable!(),
                };
                if delay == 0 {
                    self.state = OtaState::Downloading {
                        offset,
                        total_size: total,
                    };
                    self.build_block_request(offset, total)
                } else {
                    self.state = OtaState::WaitForData {
                        delay_secs: delay,
                        elapsed: 0,
                        download_offset: offset,
                        download_total: total,
                    };
                    OtaAction::Wait(delay)
                }
            }
            ParsedBlockResponse::Error(status) => {
                log::warn!("[OTA] Block response error: 0x{:02X}", status);
                // Transition to failed and send Upgrade End Request so server stops waiting
                self.mark_failed()
            }
        }
    }

    fn handle_end_response(&mut self, payload: &[u8]) -> OtaAction {
        if !matches!(
            self.state,
            OtaState::WaitingActivate | OtaState::WaitingUpgradeCommand
        ) {
            log::warn!("[OTA] Ignoring End Response outside WaitingActivate");
            return OtaAction::None;
        }
        let resp = match UpgradeEndResponse::parse(payload) {
            Some(r) => r,
            None => {
                log::warn!("[OTA] Failed to parse End Response");
                return OtaAction::None;
            }
        };
        if resp.manufacturer_code != self.manufacturer_code
            || resp.image_type != self.image_type
            || resp.file_version != self.target_version
        {
            log::warn!("[OTA] Ignoring End Response for a different image");
            return OtaAction::None;
        }

        // upgrade_time == 0 means upgrade immediately
        // upgrade_time == 0xFFFFFFFF means wait for another command
        if resp.upgrade_time == 0xFFFFFFFF {
            log::info!("[OTA] Server says wait for signal");
            self.state = OtaState::WaitingUpgradeCommand;
            OtaAction::None
        } else if resp.upgrade_time == 0 || resp.upgrade_time <= resp.current_time {
            log::info!("[OTA] Server says upgrade NOW");
            self.state = OtaState::Done;
            OtaAction::ActivateImage
        } else {
            let delay = resp.upgrade_time.saturating_sub(resp.current_time);
            log::info!("[OTA] Server says upgrade in {} seconds", delay);
            self.state = OtaState::WaitingUpgrade {
                delay_secs: delay,
                elapsed: 0,
            };
            self.set_attr(
                ATTR_IMAGE_UPGRADE_STATUS,
                ZclValue::Enum8(STATUS_COUNT_DOWN),
            );
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

    /// Continue a freshly started download at `offset` instead of 0.
    ///
    /// Valid only directly after a Query Next Image Response started a
    /// download (`Downloading { offset: 0, .. }`) and for
    /// `0 < offset < total_size`; the caller must already hold the first
    /// `offset` bytes of this exact image. The OTA server answers Image Block
    /// Requests at any offset (ZCL r8 §11.13.6.2), so this only changes where
    /// the client asks. Returns [`OtaAction::None`] (state unchanged) when
    /// the download cannot be resumed at `offset`.
    pub fn resume_download(&mut self, offset: u32) -> OtaAction {
        match self.state {
            OtaState::Downloading {
                offset: 0,
                total_size,
            } if offset > 0 && offset < total_size => {
                self.state = OtaState::Downloading { offset, total_size };
                self.set_attr(ATTR_FILE_OFFSET, ZclValue::U32(offset));
                self.build_block_request(offset, total_size)
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
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        // Route server→client OTA commands through the state machine.
        // The result is an OtaAction which the runtime must pick up via
        // process_server_command() — here we just validate the command is known.
        match cmd_id.0 {
            0x00 | 0x02 | 0x05 | 0x07 => {
                // The runtime processes the command after preserving its APS source.
                let _ = payload;
                Ok(heapless::Vec::new())
            }
            _ => Err(ZclStatus::UnsupClusterCommand),
        }
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        // Server→client commands this cluster receives
        let mut cmds = heapless::Vec::new();
        let _ = cmds.push(CMD_IMAGE_NOTIFY.0);
        let _ = cmds.push(CMD_QUERY_NEXT_IMAGE_RESPONSE.0);
        let _ = cmds.push(CMD_IMAGE_BLOCK_RESPONSE.0);
        let _ = cmds.push(CMD_UPGRADE_END_RESPONSE.0);
        cmds
    }

    fn generated_commands(&self) -> heapless::Vec<u8, 32> {
        // Client→server commands this cluster generates
        let mut cmds = heapless::Vec::new();
        let _ = cmds.push(CMD_QUERY_NEXT_IMAGE_REQUEST.0);
        let _ = cmds.push(CMD_IMAGE_BLOCK_REQUEST.0);
        let _ = cmds.push(CMD_UPGRADE_END_REQUEST.0);
        cmds
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    /// Resets only the in-memory OTA *transport* conversation state back to
    /// idle — it does not touch image identity (`manufacturer_code`,
    /// `image_type`, `current_version`), the product-configured
    /// `hardware_version`, or any staged/verified image bytes, which live
    /// in the runtime's `FirmwareWriter`/staging storage outside this
    /// cluster. A Zigbee OTA client can always safely restart from a fresh
    /// `QueryNextImageRequest`, so abandoning an in-flight download's
    /// offset bookkeeping here is safe and never erases or writes flash.
    fn reset_to_factory_defaults(&mut self) {
        self.state = OtaState::Idle;
        self.target_version = 0;
        self.target_size = 0;
        self.block_size = DEFAULT_BLOCK_SIZE;
        self.set_attr(ATTR_IMAGE_UPGRADE_STATUS, ZclValue::Enum8(STATUS_NORMAL));
        self.set_attr(ATTR_FILE_OFFSET, ZclValue::U32(0xFFFFFFFF));
        self.set_attr(ATTR_DOWNLOADED_FILE_VERSION, ZclValue::U32(0xFFFFFFFF));
        self.set_attr(ATTR_DOWNLOADED_STACK_VERSION, ZclValue::U16(0xFFFF));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clusters::ota_image::OtaImageHeader;

    const MFR: u16 = 0x1234;
    const TYPE: u16 = 0x0001;

    fn downloading(size: u32) -> OtaCluster {
        let mut c = OtaCluster::new(MFR, TYPE, 1);
        c.start_query();
        let mut rsp = [0u8; 13];
        rsp[1..3].copy_from_slice(&MFR.to_le_bytes());
        rsp[3..5].copy_from_slice(&TYPE.to_le_bytes());
        rsp[5..9].copy_from_slice(&2u32.to_le_bytes());
        rsp[9..13].copy_from_slice(&size.to_le_bytes());
        assert!(matches!(
            c.process_server_command(0x02, &rsp),
            OtaAction::SendBlockRequest(_)
        ));
        c
    }

    fn block(offset: u32, data: &[u8]) -> heapless::Vec<u8, 128> {
        let mut p = heapless::Vec::new();
        p.push(0).unwrap();
        p.extend_from_slice(&MFR.to_le_bytes()).unwrap();
        p.extend_from_slice(&TYPE.to_le_bytes()).unwrap();
        p.extend_from_slice(&2u32.to_le_bytes()).unwrap();
        p.extend_from_slice(&offset.to_le_bytes()).unwrap();
        p.push(data.len() as u8).unwrap();
        p.extend_from_slice(data).unwrap();
        p
    }

    fn attr(c: &OtaCluster, id: AttributeId) -> ZclValue {
        c.attributes().get(id).unwrap().clone()
    }

    #[test]
    fn read_only_attributes_are_updated_internally() {
        let mut c = downloading(1000);
        assert_eq!(
            attr(&c, ATTR_IMAGE_UPGRADE_STATUS),
            ZclValue::Enum8(STATUS_DOWNLOAD_IN_PROGRESS)
        );
        assert_eq!(attr(&c, ATTR_FILE_OFFSET), ZclValue::U32(0));
        assert!(matches!(
            c.process_server_command(0x05, &block(0, &[1; 10])),
            OtaAction::WriteBlock { offset: 0, .. }
        ));
        assert_eq!(attr(&c, ATTR_FILE_OFFSET), ZclValue::U32(10));
        c.set_upgrade_server_id(0x0011_2233_4455_6677);
        assert_eq!(
            attr(&c, ATTR_UPGRADE_SERVER_ID),
            ZclValue::IeeeAddr(0x0011_2233_4455_6677)
        );
        // WAIT_FOR_DATA updates MinimumBlockPeriod.
        let mut w = [0u8; 11];
        w[0] = 0x97;
        w[5] = 5;
        w[9] = 250;
        c.process_server_command(0x05, &w);
        assert_eq!(attr(&c, ATTR_MIN_BLOCK_PERIOD), ZclValue::U16(250));
    }

    #[test]
    fn resume_download_requests_the_saved_offset_once() {
        let mut c = downloading(1000);
        match c.resume_download(480) {
            OtaAction::SendBlockRequest(r) => assert_eq!(r.file_offset, 480),
            other => panic!("unexpected {other:?}"),
        }
        assert_eq!(
            c.state(),
            OtaState::Downloading {
                offset: 480,
                total_size: 1000
            }
        );
        assert_eq!(attr(&c, ATTR_FILE_OFFSET), ZclValue::U32(480));
        // The next block must continue at the resumed offset.
        assert!(matches!(
            c.process_server_command(0x05, &block(480, &[1; 10])),
            OtaAction::WriteBlock { offset: 480, .. }
        ));
        // Only a fresh download can be moved, and only inside the image.
        assert!(matches!(c.resume_download(100), OtaAction::None));
        let mut c = downloading(1000);
        assert!(matches!(c.resume_download(0), OtaAction::None));
        assert!(matches!(c.resume_download(1000), OtaAction::None));
        let mut idle = OtaCluster::new(MFR, TYPE, 1);
        assert!(matches!(idle.resume_download(10), OtaAction::None));
        assert_eq!(idle.state(), OtaState::Idle);
    }

    #[test]
    fn oversized_block_aborts_instead_of_leaving_a_hole() {
        // Larger than the 64-byte buffer.
        let mut c = downloading(1000);
        match c.process_server_command(0x05, &block(0, &[7; 80])) {
            OtaAction::SendEndRequest(r) => assert_eq!(r.status, ZclStatus::Abort as u8),
            other => panic!("unexpected {other:?}"),
        }
        assert_eq!(c.state(), OtaState::Failed);
        // Fits the buffer but exceeds the requested MaxDataSize.
        let mut c = downloading(1000);
        c.set_block_size(32);
        match c.process_server_command(0x05, &block(0, &[7; 40])) {
            OtaAction::SendEndRequest(r) => assert_eq!(r.status, ZclStatus::Abort as u8),
            other => panic!("unexpected {other:?}"),
        }
        assert_eq!(attr(&c, ATTR_FILE_OFFSET), ZclValue::U32(0));
        assert!(ParsedBlockResponse::parse(&block(0, &[0; 65])).is_none());
    }

    #[test]
    fn image_header_is_cross_checked_against_query_response() {
        let mut c = downloading(1000);
        c.set_hardware_version(3);
        let mut raw = [0u8; 70];
        raw[0..4].copy_from_slice(&0x0BEE_F11Eu32.to_le_bytes());
        raw[4..6].copy_from_slice(&0x0100u16.to_le_bytes());
        raw[6..8].copy_from_slice(&68u16.to_le_bytes());
        raw[8..10].copy_from_slice(&0x0006u16.to_le_bytes()); // dest + hw
        raw[10..12].copy_from_slice(&MFR.to_le_bytes());
        raw[12..14].copy_from_slice(&TYPE.to_le_bytes());
        raw[14..18].copy_from_slice(&2u32.to_le_bytes());
        raw[52..56].copy_from_slice(&1000u32.to_le_bytes());
        raw[56..64].copy_from_slice(&0x0807_0605_0403_0201u64.to_le_bytes());
        raw[64..66].copy_from_slice(&2u16.to_le_bytes());
        raw[66..68].copy_from_slice(&4u16.to_le_bytes());
        let (hdr, len) = OtaImageHeader::parse(&raw).unwrap();
        assert_eq!(len, 68);
        assert_eq!(hdr.upgrade_file_destination, Some(0x0807_0605_0403_0201));
        assert_eq!(hdr.min_hardware_version, Some(2));
        assert_eq!(hdr.max_hardware_version, Some(4));
        assert!(c.validate_image_header(&hdr));
        let mut bad = hdr.clone();
        bad.total_image_size = 999;
        assert!(!c.validate_image_header(&bad));
        let mut bad = hdr.clone();
        bad.file_version = 3;
        assert!(!c.validate_image_header(&bad));
        let mut bad = hdr;
        bad.min_hardware_version = Some(4);
        assert!(!c.validate_image_header(&bad));
    }

    #[test]
    fn image_notify_honours_jitter_state_and_length() {
        let mut c = OtaCluster::new(MFR, TYPE, 1);
        // Jitter 0 or > 100 and truncated payloads are malformed.
        assert!(matches!(c.process_server_command(0, &[0]), OtaAction::None));
        assert!(matches!(
            c.process_server_command(0, &[0, 0]),
            OtaAction::None
        ));
        assert!(matches!(
            c.process_server_command(0, &[0, 101]),
            OtaAction::None
        ));
        assert!(matches!(
            c.process_server_command(0, &[1, 50, 0x34]),
            OtaAction::None
        ));
        assert!(matches!(
            c.process_server_command(0, &[4, 50]),
            OtaAction::None
        ));
        // Random value above the jitter: not selected (value is consumed).
        c.set_notify_random(60);
        assert!(matches!(
            c.process_server_command(0, &[0, 50]),
            OtaAction::None
        ));
        c.set_notify_random(50);
        assert!(matches!(
            c.process_server_command(0, &[0, 50]),
            OtaAction::SendQuery(_)
        ));
        // Ignored while a transfer is in progress.
        assert!(matches!(
            c.process_server_command(0, &[0, 100]),
            OtaAction::None
        ));
        assert_eq!(c.state(), OtaState::QuerySent);
    }

    #[test]
    fn request_serializers_never_panic_on_small_buffers() {
        let c = downloading(1000);
        let OtaAction::SendBlockRequest(req) = c.next_block_request() else {
            panic!()
        };
        let mut buf = [0u8; 32];
        let full = req.serialize(&mut buf);
        for cap in 0..full {
            assert_eq!(req.serialize(&mut buf[..cap]), 0);
        }
        let end = UpgradeEndRequest {
            status: 0,
            manufacturer_code: MFR,
            image_type: TYPE,
            file_version: 2,
        };
        let full = end.serialize(&mut buf);
        for cap in 0..full {
            assert_eq!(end.serialize(&mut buf[..cap]), 0);
        }
    }
}
