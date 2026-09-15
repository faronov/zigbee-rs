//! APS Information Base (AIB).
//!
//! The AIB stores APS-layer configuration and state attributes
//! as defined in Zigbee PRO R22 spec Table 2-28.

use zigbee_types::IeeeAddress;

/// APS Information Base — all APS layer attributes.
///
/// The AIB is analogous to the MAC PIB and NWK NIB.
#[derive(Debug)]
pub struct Aib {
    // ── Addressing / identity ───────────────────────────
    /// Whether this device is the designated Trust Center coordinator.
    /// If true, this device distributes network keys and manages security.
    pub aps_designated_coordinator: bool,

    /// Channel mask used during network formation/join.
    /// Bitmask of IEEE 802.15.4 channels (bits 11-26 for 2.4 GHz).
    pub aps_channel_mask: u32,

    /// Extended PAN ID to use when forming or joining a network.
    /// Set to all zeros to accept any network.
    pub aps_use_extended_pan_id: IeeeAddress,

    /// Whether the device should attempt unsecured join
    /// (without pre-configured link key). Default: true for Zigbee 3.0.
    pub aps_use_insecure_join: bool,

    // ── Timing / network quality ────────────────────────
    /// Minimum inter-frame delay (ms) between consecutive APS data frames.
    /// Prevents flooding the NWK layer. Zigbee spec default: 10.
    pub aps_interframe_delay: u8,

    /// Last measured channel energy (0x00-0xFF).
    /// Updated by the network manager after an energy detect scan.
    pub aps_last_channel_energy: u8,

    /// Last measured channel failure rate (0x00-0xFF).
    /// Percentage of transmission failures on the current channel.
    pub aps_last_channel_failure_rate: u8,

    /// Channel timer — time (in hours) since the last channel change.
    /// Used by the network manager to decide channel switching.
    pub aps_channel_timer: u32,

    // ── Fragmentation ───────────────────────────────────
    /// Maximum number of octets in a single APS transmission
    /// (before fragmentation). Set based on NWK payload capacity.
    pub aps_max_window_size: u8,

    /// Maximum number of retries for fragmented transmissions.
    pub aps_max_frame_retries: u8,

    // ── Duplicate rejection ─────────────────────────────
    /// APS duplicate rejection table timeout (ms).
    /// How long to remember received APS counters for dedup.
    pub aps_duplicate_rejection_timeout: u16,

    // ── Security ────────────────────────────────────────
    /// Trust Center address (IEEE). All-zeros if not set.
    pub aps_trust_center_address: IeeeAddress,

    /// Whether APS security is enabled.
    pub aps_security_enabled: bool,

    // ── Counters ────────────────────────────────────────
    /// APS frame counter for outgoing secured frames (per-key).
    pub aps_outgoing_frame_counter: u32,
}

impl Aib {
    /// Create an AIB with Zigbee PRO R22 default values.
    pub fn new() -> Self {
        Self {
            aps_designated_coordinator: false,
            aps_channel_mask: 0x07FF_F800, // All 2.4 GHz channels (11-26)
            aps_use_extended_pan_id: [0u8; 8],
            aps_use_insecure_join: true,
            aps_interframe_delay: 10,
            aps_last_channel_energy: 0,
            aps_last_channel_failure_rate: 0,
            aps_channel_timer: 0,
            aps_max_window_size: 8,
            aps_max_frame_retries: 3,
            aps_duplicate_rejection_timeout: 3000,
            aps_trust_center_address: [0u8; 8],
            aps_security_enabled: true,
            aps_outgoing_frame_counter: 0,
        }
    }

    /// Increment outgoing APS frame counter. Returns the pre-increment value.
    pub fn next_frame_counter(&mut self) -> u32 {
        let fc = self.aps_outgoing_frame_counter;
        self.aps_outgoing_frame_counter = self.aps_outgoing_frame_counter.wrapping_add(1);
        fc
    }
}

impl Default for Aib {
    fn default() -> Self {
        Self::new()
    }
}

// ── AIB attribute identifiers (for APSME-GET / APSME-SET) ───────

/// AIB attribute identifiers (Zigbee spec Table 2-28).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AibAttribute {
    ApsDesignatedCoordinator = 0xC2,
    ApsChannelMaskList = 0xC3,
    ApsUseExtendedPanId = 0xC4,
    ApsGroupTable = 0xC5,
    ApsUseInsecureJoin = 0xC8,
    ApsInterframeDelay = 0xC9,
    ApsLastChannelEnergy = 0xCA,
    ApsLastChannelFailureRate = 0xCB,
    ApsChannelTimer = 0xCC,
    ApsMaxWindowSize = 0xCD,
    ApsTrustCenterAddress = 0xAB,
    ApsSecurityEnabled = 0xCF,
}

impl AibAttribute {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0xC2 => Some(Self::ApsDesignatedCoordinator),
            0xC3 => Some(Self::ApsChannelMaskList),
            0xC4 => Some(Self::ApsUseExtendedPanId),
            0xC5 => Some(Self::ApsGroupTable),
            0xC8 => Some(Self::ApsUseInsecureJoin),
            0xC9 => Some(Self::ApsInterframeDelay),
            0xCA => Some(Self::ApsLastChannelEnergy),
            0xCB => Some(Self::ApsLastChannelFailureRate),
            0xCC => Some(Self::ApsChannelTimer),
            0xCD => Some(Self::ApsMaxWindowSize),
            0xAB => Some(Self::ApsTrustCenterAddress),
            0xCF => Some(Self::ApsSecurityEnabled),
            _ => None,
        }
    }
}
