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

/// Common low-interference Zigbee channels, including channel 15.
///
/// Steering gives channel 15 its own first scan, then scans the remaining
/// channels in this set before falling back to the rest of the 2.4 GHz band.
pub const BDB_POPULAR_CHANNEL_SET: ChannelMask = ChannelMask(
    (1 << 11) | (1 << 14) | (1 << 15) | (1 << 19) | (1 << 20) | (1 << 24) | (1 << 25) | (1 << 26),
);

/// All 2.4 GHz channels not included in [`BDB_POPULAR_CHANNEL_SET`].
pub const BDB_POPULAR_CHANNEL_FALLBACK_SET: ChannelMask =
    ChannelMask(ChannelMask::ALL_2_4GHZ.0 & !BDB_POPULAR_CHANNEL_SET.0);

/// BDB minimum commissioning time for Finding & Binding (seconds).
pub const BDB_MIN_COMMISSIONING_TIME: u16 = 180;

// ── Node join link key type ─────────────────────────────────

/// How the joining node's network key was protected (BDB v3.0.1 Table 6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum NodeJoinLinkKeyType {
    /// Centralized network using the default global Trust Center link key.
    #[default]
    DefaultGlobalTrustCenterLinkKey = 0x00,
    /// Distributed network using the distributed-security global link key.
    DistributedSecurityGlobalLinkKey = 0x01,
    /// Centralized network using an install-code-derived preconfigured link key.
    InstallCodeDerivedPreconfiguredLinkKey = 0x02,
    /// Distributed network joined with the touchlink preconfigured link key.
    TouchlinkPreconfiguredLinkKey = 0x03,
}

/// Method used to replace the initial Trust Center link key after joining.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum TcLinkKeyExchangeMethod {
    /// APS Request-Key / Transport-Key / Verify-Key / Confirm-Key exchange.
    #[default]
    ApsRequestKey = 0x00,
    /// Certificate-Based Key Establishment.
    CertificateBasedKeyExchange = 0x01,
}

// ── BDB commissioning status ────────────────────────────────

/// Status of the last commissioning attempt (BDB spec Table 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BdbCommissioningStatus {
    #[default]
    Success = 0x00,
    InProgress = 0x01,
    NotAaCapable = 0x02,
    NoNetwork = 0x03,
    TargetFailure = 0x04,
    FormationFailure = 0x05,
    NoIdentifyQueryResponse = 0x06,
    BindingTableFull = 0x07,
    NoScanResponse = 0x08,
    NotPermitted = 0x09,
    TcLinkKeyExchangeFailure = 0x0A,
    NotOnANetwork = 0x0B,
    OnANetwork = 0x0C,
}

// ── BDB attributes ──────────────────────────────────────────

/// BDB attributes and implementation-owned commissioning state.
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

    /// Whether the Trust Center admits only nodes provisioned with install codes.
    pub join_uses_install_code_key: bool,

    /// Bitmask indicating which commissioning modes this device supports
    /// (based on hardware and device type).
    pub node_commissioning_capability: CommissioningMode,

    /// Whether this node is currently part of a Zigbee network.
    pub node_is_on_a_network: bool,

    /// How this node's link key was obtained when it joined.
    pub node_join_link_key_type: NodeJoinLinkKeyType,

    /// Primary channel set — scanned first during steering/formation.
    pub primary_channel_set: ChannelMask,

    /// IEEE 802.15.4 scan-duration exponent used by BDB discovery and formation.
    pub scan_duration: u8,

    /// Secondary channel set — scanned if primary yields no results.
    pub secondary_channel_set: ChannelMask,

    /// Attempts made in the current Trust Center link-key exchange stage.
    pub tc_link_key_exchange_attempts: u8,

    /// Maximum attempts for each Trust Center link-key exchange stage.
    pub tc_link_key_exchange_attempts_max: u8,

    /// Trust Center link-key exchange mechanism selected by the application.
    pub tc_link_key_exchange_method: TcLinkKeyExchangeMethod,

    /// Timeout (seconds) for a joining node to complete TC link key exchange.
    pub trust_center_node_join_timeout: u8,

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
            join_uses_install_code_key: false,
            node_commissioning_capability: CommissioningMode::STEERING,
            node_is_on_a_network: false,
            node_join_link_key_type: NodeJoinLinkKeyType::default(),
            primary_channel_set: BDB_PRIMARY_CHANNEL_SET,
            scan_duration: 3,
            secondary_channel_set: BDB_SECONDARY_CHANNEL_SET,
            tc_link_key_exchange_attempts: 0,
            tc_link_key_exchange_attempts_max: 3,
            tc_link_key_exchange_method: TcLinkKeyExchangeMethod::ApsRequestKey,
            trust_center_node_join_timeout: 15,
            trust_center_require_key_exchange: true,
            steering_attempts_remaining: 5,
        }
    }
}
