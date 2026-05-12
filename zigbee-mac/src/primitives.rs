//! MLME and MCPS primitive request/confirm/indication types.
//!
//! These structs map to IEEE 802.15.4 MAC service primitives as required
//! by a Zigbee PRO R22 stack. Each primitive follows the pattern:
//!   - Request:    parameters the upper layer sends DOWN to MAC
//!   - Confirm:    result MAC sends UP after completing the request
//!   - Indication: unsolicited event MAC sends UP (e.g. received frame)

use zigbee_types::{ChannelMask, IeeeAddress, MacAddress, PanId, ShortAddress};

// ── Scan ────────────────────────────────────────────────────────

/// Type of MAC scan
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ScanType {
    /// Energy Detection — measure noise on each channel
    Ed = 0x00,
    /// Active — send beacon requests, collect responses
    Active = 0x01,
    /// Passive — listen for beacons without transmitting
    Passive = 0x02,
    /// Orphan — search for coordinator after losing sync
    Orphan = 0x03,
}

/// MLME-SCAN.request parameters
#[derive(Debug, Clone)]
pub struct MlmeScanRequest {
    pub scan_type: ScanType,
    /// Bitmask of channels to scan (bits 11-26 for 2.4 GHz)
    pub channel_mask: ChannelMask,
    /// Scan duration exponent: scan time = aBaseSuperframeDuration * (2^n + 1)
    /// Range 0-14. Typical: 3 (~138ms/ch) for fast, 5 (~530ms/ch) for thorough
    pub scan_duration: u8,
}

/// Descriptor for a discovered PAN (from beacon)
#[derive(Debug, Clone)]
pub struct PanDescriptor {
    /// Channel on which the beacon was received
    pub channel: u8,
    /// Coordinator address (short or extended)
    pub coord_address: MacAddress,
    /// Superframe specification from the beacon
    pub superframe_spec: SuperframeSpec,
    /// Link Quality Indicator (0-255)
    pub lqi: u8,
    /// Whether the beacon had security enabled
    pub security_use: bool,
    /// Zigbee-specific beacon payload
    pub zigbee_beacon: ZigbeeBeaconPayload,
}

/// IEEE 802.15.4 Superframe Specification (decoded from 16-bit field)
#[derive(Debug, Clone, Copy, Default)]
pub struct SuperframeSpec {
    pub beacon_order: u8,
    pub superframe_order: u8,
    pub final_cap_slot: u8,
    pub battery_life_ext: bool,
    pub pan_coordinator: bool,
    pub association_permit: bool,
}

impl SuperframeSpec {
    pub fn from_raw(raw: u16) -> Self {
        Self {
            beacon_order: (raw & 0x000F) as u8,
            superframe_order: ((raw >> 4) & 0x000F) as u8,
            final_cap_slot: ((raw >> 8) & 0x000F) as u8,
            battery_life_ext: (raw >> 12) & 1 != 0,
            pan_coordinator: (raw >> 14) & 1 != 0,
            association_permit: (raw >> 15) & 1 != 0,
        }
    }
}

/// Zigbee beacon payload (appended after IEEE 802.15.4 beacon)
#[derive(Debug, Clone)]
pub struct ZigbeeBeaconPayload {
    /// Must be 0x00 for Zigbee
    pub protocol_id: u8,
    /// Stack profile (1 = ZigBee, 2 = ZigBee PRO)
    pub stack_profile: u8,
    /// Protocol version
    pub protocol_version: u8,
    /// Router capacity available
    pub router_capacity: bool,
    /// Device depth in network tree
    pub device_depth: u8,
    /// End device capacity available
    pub end_device_capacity: bool,
    /// Extended PAN ID (64-bit)
    pub extended_pan_id: IeeeAddress,
    /// TX offset (24-bit, for beacon scheduling)
    pub tx_offset: [u8; 3],
    /// Network update ID
    pub update_id: u8,
}

/// Energy Detection result for a single channel
#[derive(Debug, Clone, Copy)]
pub struct EdValue {
    pub channel: u8,
    /// Energy level (0-255, higher = more noise)
    pub energy: u8,
}

/// MLME-SCAN.confirm — result of a scan operation
#[derive(Debug)]
pub struct MlmeScanConfirm {
    pub scan_type: ScanType,
    /// Discovered PANs (Active/Passive scan) — max 27 entries
    pub pan_descriptors: PanDescriptorList,
    /// Energy measurements (ED scan) — one per scanned channel
    pub energy_list: EdList,
}

/// Fixed-capacity list of PAN descriptors (no heap allocation)
pub const MAX_PAN_DESCRIPTORS: usize = 16;
pub type PanDescriptorList = heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>;

/// Fixed-capacity list of ED values
pub const MAX_ED_VALUES: usize = 4;
pub type EdList = heapless::Vec<EdValue, MAX_ED_VALUES>;

// ── Association ─────────────────────────────────────────────────

/// Device capability info (sent in Association Request)
#[derive(Debug, Clone, Copy, Default)]
pub struct CapabilityInfo {
    /// Device is an FFD (Full Function Device)
    pub device_type_ffd: bool,
    /// Device is mains-powered
    pub mains_powered: bool,
    /// RX is on when idle (not sleepy)
    pub rx_on_when_idle: bool,
    /// Device can do MAC-level security
    pub security_capable: bool,
    /// Device wants a short address from coordinator
    pub allocate_address: bool,
}

impl CapabilityInfo {
    pub fn to_byte(self) -> u8 {
        let mut b: u8 = 0;
        if self.device_type_ffd {
            b |= 1 << 1;
        }
        if self.mains_powered {
            b |= 1 << 2;
        }
        if self.rx_on_when_idle {
            b |= 1 << 3;
        }
        if self.security_capable {
            b |= 1 << 6;
        }
        if self.allocate_address {
            b |= 1 << 7;
        }
        b
    }

    pub fn from_byte(b: u8) -> Self {
        Self {
            device_type_ffd: b & (1 << 1) != 0,
            mains_powered: b & (1 << 2) != 0,
            rx_on_when_idle: b & (1 << 3) != 0,
            security_capable: b & (1 << 6) != 0,
            allocate_address: b & (1 << 7) != 0,
        }
    }
}

/// MLME-ASSOCIATE.request parameters
#[derive(Debug, Clone)]
pub struct MlmeAssociateRequest {
    /// Channel to associate on
    pub channel: u8,
    /// Coordinator address
    pub coord_address: MacAddress,
    /// Our capability information
    pub capability_info: CapabilityInfo,
}

/// Association status returned by coordinator
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AssociationStatus {
    Success = 0x00,
    PanAtCapacity = 0x01,
    PanAccessDenied = 0x02,
}

/// MLME-ASSOCIATE.confirm — result of association attempt
#[derive(Debug, Clone)]
pub struct MlmeAssociateConfirm {
    /// Short address assigned by coordinator (0xFFFF/0xFFFE on failure)
    pub short_address: ShortAddress,
    pub status: AssociationStatus,
}

/// MLME-ASSOCIATE.indication — coordinator received association request
#[derive(Debug, Clone)]
pub struct MlmeAssociateIndication {
    /// Extended address of the requesting device
    pub device_address: IeeeAddress,
    pub capability_info: CapabilityInfo,
}

/// MLME-ASSOCIATE.response — coordinator's reply to association request
#[derive(Debug, Clone)]
pub struct MlmeAssociateResponse {
    /// Extended address of the requesting device
    pub device_address: IeeeAddress,
    /// Short address to assign (or 0xFFFE to deny)
    pub short_address: ShortAddress,
    pub status: AssociationStatus,
}

// ── Disassociation ──────────────────────────────────────────────

/// Disassociation reason codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DisassociateReason {
    /// Coordinator wishes device to leave
    CoordinatorLeave = 0x01,
    /// Device wishes to leave
    DeviceLeave = 0x02,
}

/// MLME-DISASSOCIATE.request
#[derive(Debug, Clone)]
pub struct MlmeDisassociateRequest {
    pub device_address: MacAddress,
    pub reason: DisassociateReason,
    /// If true, send via indirect transmission
    pub tx_indirect: bool,
}

// ── Start ───────────────────────────────────────────────────────

/// MLME-START.request — start or configure a PAN
#[derive(Debug, Clone)]
pub struct MlmeStartRequest {
    pub pan_id: PanId,
    pub channel: u8,
    pub beacon_order: u8,
    pub superframe_order: u8,
    /// True if this device is the PAN coordinator
    pub pan_coordinator: bool,
    /// Whether to accept battery life extension
    pub battery_life_ext: bool,
}

// ── Data service ────────────────────────────────────────────────

/// Transmit options for MCPS-DATA
#[derive(Debug, Clone, Copy, Default)]
pub struct TxOptions {
    /// Request MAC-level acknowledgement
    pub ack_tx: bool,
    /// Use indirect transmission (coordinator → sleepy device)
    pub indirect: bool,
    /// Apply MAC-level security
    pub security_enabled: bool,
}

/// Maximum MAC payload size (127 - MHR overhead ≈ 102 bytes typical)
pub const MAX_MAC_PAYLOAD: usize = 127;

/// MCPS-DATA.request — transmit a MAC frame
#[derive(Debug)]
pub struct McpsDataRequest<'a> {
    pub src_addr_mode: AddressMode,
    pub dst_address: MacAddress,
    pub payload: &'a [u8],
    pub msdu_handle: u8,
    pub tx_options: TxOptions,
}

/// Address mode (how source/destination are encoded)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AddressMode {
    /// No address
    None = 0x00,
    /// 16-bit short address
    Short = 0x02,
    /// 64-bit extended address
    Extended = 0x03,
}

/// MCPS-DATA.confirm — transmit result
#[derive(Debug, Clone)]
pub struct McpsDataConfirm {
    pub msdu_handle: u8,
    pub timestamp: Option<u32>,
}

/// MCPS-DATA.indication — received frame
#[derive(Debug)]
pub struct McpsDataIndication {
    pub src_address: MacAddress,
    pub dst_address: MacAddress,
    pub lqi: u8,
    pub payload: MacFrame,
    pub security_use: bool,
}

/// Received MAC frame data (fixed buffer, no heap)
#[derive(Debug)]
pub struct MacFrame {
    buf: [u8; MAX_MAC_PAYLOAD],
    len: usize,
}

impl MacFrame {
    pub fn new() -> Self {
        Self {
            buf: [0u8; MAX_MAC_PAYLOAD],
            len: 0,
        }
    }

    pub fn from_slice(data: &[u8]) -> Option<Self> {
        if data.len() > MAX_MAC_PAYLOAD {
            return None;
        }
        let mut frame = Self::new();
        frame.buf[..data.len()].copy_from_slice(data);
        frame.len = data.len();
        Some(frame)
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for MacFrame {
    fn default() -> Self {
        Self::new()
    }
}
