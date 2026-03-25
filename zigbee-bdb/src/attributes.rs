//! BDB attributes (BDB v3.0.1 spec Table 5).
//!
//! These attributes control the behavior of the BDB commissioning
//! procedures and are persisted across resets via NV storage.

use zigbee_types::{ChannelMask, IeeeAddress};

use crate::state_machine::CommissioningMode;

// ── BDB channel defaults ────────────────────────────────────

/// BDB primary channel set: channels 11, 15, 20, 25.
///
/// These are scanned first during network steering / formation.
pub const BDB_PRIMARY_CHANNEL_SET: ChannelMask =
    ChannelMask((1 << 11) | (1 << 15) | (1 << 20) | (1 << 25)); // 0x0210_8800

/// BDB secondary channel set: all 2.4 GHz channels except the primary set.
pub const BDB_SECONDARY_CHANNEL_SET: ChannelMask =
    ChannelMask(ChannelMask::ALL_2_4GHZ.0 & !BDB_PRIMARY_CHANNEL_SET.0); // 0x05EF_7000

/// BDB minimum commissioning time for Finding & Binding (seconds).
pub const BDB_MIN_COMMISSIONING_TIME: u16 = 180;

// ── Node join link key type ─────────────────────────────────

/// How the joining node's link key was obtained (BDB spec Table 5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum NodeJoinLinkKeyType {
    /// Default global Trust Center link key ("ZigBeeAlliance09")
    #[default]
    DefaultGlobalTrustCenterLinkKey = 0x00,
    /// IC-derived Trust Center link key (install code)
    IcDerivedTrustCenterLinkKey = 0x01,
    /// Application-specific Trust Center link key (pre-configured)
    AppTrustCenterLinkKey = 0x02,
    /// Touchlink preconfigured link key
    TouchlinkPreconfiguredLinkKey = 0x03,
}

// ── BDB commissioning status ────────────────────────────────

/// Status of the last commissioning attempt (BDB spec Table 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BdbCommissioningStatus {
    #[default]
    Success = 0x00,
    InProgress = 0x01,
    NoNetwork = 0x02,
    TlTargetFailure = 0x03,
    TlNotAddressAssignment = 0x04,
    TlNoScanResponse = 0x05,
    NotPermitted = 0x06,
    SteeringFormationFailure = 0x07,
    NoIdentifyQueryResponse = 0x08,
    BindingTableFull = 0x09,
    NoScanResponse = 0x0A,
}

// ── BDB attributes ──────────────────────────────────────────

/// All BDB attributes as defined in BDB v3.0.1 spec Table 5.
#[derive(Debug, Clone)]
pub struct BdbAttributes {
    /// Group ID used for group bindings during Finding & Binding.
    /// 0xFFFF means no group binding.
    pub commissioning_group_id: u16,

    /// Bitmask of commissioning modes the application requests.
    /// See [`CommissioningMode`] for bit definitions.
    pub commissioning_mode: CommissioningMode,

    /// Status of the most recent commissioning attempt.
    pub commissioning_status: BdbCommissioningStatus,

    /// EUI-64 of the most recent device that joined through this node.
    pub joining_node_eui64: IeeeAddress,

    /// New Trust Center link key for the most recent joining device.
    pub joining_node_new_tc_link_key: [u8; 16],

    /// Bitmask indicating which commissioning modes this device supports
    /// (based on hardware and device type).
    pub node_commissioning_capability: CommissioningMode,

    /// Whether this node is currently part of a Zigbee network.
    pub node_is_on_a_network: bool,

    /// How this node's link key was obtained when it joined.
    pub node_join_link_key_type: NodeJoinLinkKeyType,

    /// Primary channel set — scanned first during steering/formation.
    pub primary_channel_set: ChannelMask,

    /// Secondary channel set — scanned if primary yields no results.
    pub secondary_channel_set: ChannelMask,

    /// Timeout (seconds) for a joining node to complete TC link key exchange.
    pub trust_center_node_join_timeout: u16,

    /// Whether the Trust Center requires the joining device to complete
    /// a Trust Center link key exchange before being fully admitted.
    pub trust_center_require_key_exchange: bool,

    /// Number of steering attempts remaining.
    pub steering_attempts_remaining: u8,
}

impl Default for BdbAttributes {
    fn default() -> Self {
        Self {
            commissioning_group_id: 0xFFFF,
            commissioning_mode: CommissioningMode::STEERING,
            commissioning_status: BdbCommissioningStatus::Success,
            joining_node_eui64: [0u8; 8],
            joining_node_new_tc_link_key: [0u8; 16],
            node_commissioning_capability: CommissioningMode::STEERING,
            node_is_on_a_network: false,
            node_join_link_key_type: NodeJoinLinkKeyType::default(),
            primary_channel_set: BDB_PRIMARY_CHANNEL_SET,
            secondary_channel_set: BDB_SECONDARY_CHANNEL_SET,
            trust_center_node_join_timeout: 10,
            trust_center_require_key_exchange: true,
            steering_attempts_remaining: 5,
        }
    }
}
