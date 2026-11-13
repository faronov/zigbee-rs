//! Transport-neutral model of the ST 802.15.4 MAC SAP (CPU2 coprocessor).
//!
//! The STM32WB wireless coprocessor runs ST's full-function 802.15.4 MAC
//! (`stm32wb5x_Mac_802_15_4_fw.bin`). CPU1 talks to it over IPCC with MLME and
//! MCPS requests, and receives confirms and indications on a notification
//! channel. This module describes those primitives as plain Rust values so
//! that [`super::Stm32wbMac`] (the mapping onto [`crate::MacDriver`]) has no
//! dependency on a particular HAL, mailbox layout or async runtime, and can be
//! exercised on the host.
//!
//! A platform crate implements [`WbMacTransport`] over the real mailbox
//! (for example with `embassy-stm32-wpan`). The transport is responsible for
//! the shared-memory lifetime rules: any buffer it hands to CPU2 (an MSDU or
//! a PIB value) must stay valid until CPU2 has consumed it.

use zigbee_types::{IeeeAddress, PanId, ShortAddress};

use crate::primitives::MacFrame;
use crate::{MacAddress, PlatformServices};

/// ST MAC status codes (`802_15_4_mac_errors.h`, IEEE 802.15.4-2011 table).
pub mod status {
    pub const SUCCESS: u8 = 0x00;
    pub const PAN_AT_CAPACITY: u8 = 0x01;
    pub const PAN_ACCESS_DENIED: u8 = 0x02;
    pub const INVALID_REQUEST: u8 = 0xC0;
    pub const COUNTER_ERROR: u8 = 0xDB;
    pub const BEACON_LOSS: u8 = 0xE0;
    pub const CHANNEL_ACCESS_FAILURE: u8 = 0xE1;
    pub const DENIED: u8 = 0xE2;
    pub const SECURITY_ERROR: u8 = 0xE4;
    pub const FRAME_TOO_LONG: u8 = 0xE5;
    pub const INVALID_HANDLE: u8 = 0xE7;
    pub const INVALID_PARAMETER: u8 = 0xE8;
    pub const NO_ACK: u8 = 0xE9;
    pub const NO_BEACON: u8 = 0xEA;
    pub const NO_DATA: u8 = 0xEB;
    pub const NO_SHORT_ADDRESS: u8 = 0xEC;
    pub const TRANSACTION_EXPIRED: u8 = 0xF0;
    pub const TRANSACTION_OVERFLOW: u8 = 0xF1;
    pub const TX_ACTIVE: u8 = 0xF2;
    pub const UNSUPPORTED_ATTRIBUTE: u8 = 0xF4;
    pub const INVALID_ADDRESS: u8 = 0xF5;
    pub const TRACKING_OFF: u8 = 0xF8;
    pub const READ_ONLY: u8 = 0xFB;
    pub const SCAN_IN_PROGRESS: u8 = 0xFC;
    /// Failure of the command channel itself (not an IEEE status).
    pub const FAILURE: u8 = 0xFF;
}

/// ST MAC PIB attribute identifiers used by this backend.
pub mod pib_id {
    pub const PHY_CURRENT_CHANNEL: u8 = 0x00;
    pub const PHY_TRANSMIT_POWER: u8 = 0x02;
    pub const ASSOCIATION_PERMIT: u8 = 0x41;
    pub const AUTO_REQUEST: u8 = 0x42;
    pub const BEACON_PAYLOAD: u8 = 0x45;
    pub const BEACON_PAYLOAD_LENGTH: u8 = 0x46;
    pub const COORD_EXTENDED_ADDRESS: u8 = 0x4A;
    pub const COORD_SHORT_ADDRESS: u8 = 0x4B;
    pub const MAX_CSMA_BACKOFFS: u8 = 0x4E;
    pub const MIN_BE: u8 = 0x4F;
    pub const PAN_ID: u8 = 0x50;
    pub const PROMISCUOUS_MODE: u8 = 0x51;
    pub const RX_ON_WHEN_IDLE: u8 = 0x52;
    pub const SHORT_ADDRESS: u8 = 0x53;
    pub const TRANSACTION_PERSISTENCE_TIME: u8 = 0x55;
    pub const ASSOCIATED_PAN_COORD: u8 = 0x56;
    pub const MAX_BE: u8 = 0x57;
    pub const MAX_FRAME_RETRIES: u8 = 0x59;
    pub const RESPONSE_WAIT_TIME: u8 = 0x5A;
    pub const EXTENDED_ADDRESS: u8 = 0x6F;
}

/// Addressing mode on the SAP (`g_*_ADDR_MODE_c`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SapAddressMode {
    None = 0x00,
    Short = 0x02,
    Extended = 0x03,
}

/// Scan type on the SAP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SapScanType {
    Ed = 0x00,
    Active = 0x01,
    Passive = 0x02,
    Orphan = 0x03,
}

/// A request from CPU1 to the coprocessor MAC.
///
/// Borrowed slices must be copied (or pinned) by the transport for as long
/// as CPU2 may read them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request<'a> {
    Reset {
        set_default_pib: bool,
    },
    /// MLME-SET with the attribute value in its little-endian wire form.
    Set {
        attribute: u8,
        value: &'a [u8],
    },
    Scan {
        scan_type: SapScanType,
        /// Bit `n` selects channel `n` (page 0).
        channels: u32,
        duration: u8,
    },
    Associate {
        channel: u8,
        coordinator: MacAddress,
        capability: u8,
    },
    AssociateResponse {
        device: IeeeAddress,
        short_address: ShortAddress,
        /// IEEE 802.15.4 association status (0 success, 1 at capacity,
        /// 2 access denied).
        status: u8,
    },
    OrphanResponse {
        orphan: IeeeAddress,
        short_address: ShortAddress,
        associated_member: bool,
    },
    Disassociate {
        device: MacAddress,
        reason: u8,
        tx_indirect: bool,
    },
    Start {
        pan_id: PanId,
        channel: u8,
        beacon_order: u8,
        superframe_order: u8,
        pan_coordinator: bool,
        battery_life_ext: bool,
    },
    Poll {
        coordinator: MacAddress,
    },
    Data {
        src_addr_mode: SapAddressMode,
        destination: MacAddress,
        handle: u8,
        ack: bool,
        indirect: bool,
        msdu: &'a [u8],
    },
    Purge {
        handle: u8,
    },
}

/// Beacon received during an active or passive scan
/// (MLME-BEACON-NOTIFY.indication with `macAutoRequest == FALSE`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BeaconNotify {
    pub channel: u8,
    pub coordinator: MacAddress,
    /// Raw superframe specification (little endian).
    pub superframe_spec: u16,
    pub lqi: u8,
    pub security_use: bool,
    /// Beacon payload (the Zigbee beacon payload for a Zigbee network).
    pub payload: heapless::Vec<u8, 52>,
}

/// An MCPS-DATA.indication: MSDU and addressing, MAC header already removed.
#[derive(Debug)]
pub struct DataIndication {
    pub source: MacAddress,
    pub destination: MacAddress,
    pub lqi: u8,
    pub rssi: i8,
    pub security_use: bool,
    pub msdu: MacFrame,
}

/// A confirm or indication delivered by the coprocessor MAC.
#[derive(Debug)]
pub enum Event {
    ResetConfirm {
        status: u8,
    },
    SetConfirm {
        status: u8,
        attribute: u8,
    },
    ScanConfirm {
        status: u8,
        scan_type: u8,
        /// Bitmap of channels that were not scanned.
        unscanned_channels: u32,
        /// ED results, one per scanned channel in ascending channel order.
        energy: heapless::Vec<u8, 16>,
    },
    AssociateConfirm {
        status: u8,
        short_address: ShortAddress,
    },
    DisassociateConfirm {
        status: u8,
    },
    StartConfirm {
        status: u8,
    },
    PollConfirm {
        status: u8,
    },
    DataConfirm {
        handle: u8,
        status: u8,
        timestamp: u32,
    },
    PurgeConfirm {
        handle: u8,
        status: u8,
    },
    BeaconNotify(BeaconNotify),
    AssociateIndication {
        device: IeeeAddress,
        capability: u8,
        security_use: bool,
    },
    DisassociateIndication {
        device: IeeeAddress,
        reason: u8,
    },
    CommStatus {
        pan_id: PanId,
        source: Option<MacAddress>,
        destination: Option<MacAddress>,
        status: u8,
    },
    OrphanIndication {
        orphan: IeeeAddress,
    },
    /// MLME-POLL.indication: a Data Request command received from
    /// `requester` (ST extension used by Zigbee parents).
    PollIndication {
        requester: MacAddress,
    },
    DataIndication(DataIndication),
    SyncLoss {
        reason: u8,
    },
    /// Any other event (GTS, DPS, ranging, ...); counted and dropped.
    Other {
        id: u16,
    },
}

/// Mailbox transport to the coprocessor MAC, plus the platform services the
/// portable stack requires (clock, delay, entropy, AES provider).
#[allow(async_fn_in_trait)]
pub trait WbMacTransport: PlatformServices {
    /// Issue one request. `Ok` means CPU2 accepted the command (synchronous
    /// command-channel status `0x00`); the matching confirm, if any, arrives
    /// later through [`Self::event`]. `Err` carries the ST status code.
    async fn request(&mut self, request: &Request<'_>) -> Result<(), u8>;

    /// Next coprocessor event, waiting at most `timeout_us`
    /// (`0` = only an event that is already pending).
    async fn event(&mut self, timeout_us: u32) -> Option<Event>;
}
