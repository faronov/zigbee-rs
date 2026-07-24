//! Zigbee PRO R22 Base Device Behavior (BDB) commissioning layer (v3.0.1).
//!
//! BDB defines standardised commissioning methods for Zigbee 3.0 devices:
//!
//! | Method              | Module              | BDB spec |
//! |---------------------|---------------------|----------|
//! | Network Steering    | [`steering`]        | §8.3     |
//! | Network Formation   | [`formation`]       | §8.4     |
//! | Finding & Binding   | [`finding_binding`] | §8.5     |
//! | Touchlink           | [`touchlink`]       | §8.7     |
//!
//! # Architecture
//! ```text
//! ┌──────────────────────────────────────┐
//! │  Application                         │
//! └──────────────┬───────────────────────┘
//!                │ BDB commissioning API
//! ┌──────────────┴───────────────────────┐
//! │  BDB Layer (this crate)              │
//! │  ├── state_machine: top-level FSM    │
//! │  ├── steering: join existing network │
//! │  ├── formation: create network       │
//! │  ├── finding_binding: EZ-Mode F&B    │
//! │  ├── touchlink: proximity comm.      │
//! │  └── attributes: BDB attributes      │
//! └──────────────┬───────────────────────┘
//!                │ ZDP services / NLME-*
//! ┌──────────────┴───────────────────────┐
//! │  ZDO Layer (zigbee-zdo)              │
//! └──────────────────────────────────────┘
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

pub mod attributes;
pub mod finding_binding;
pub mod formation;
pub mod security_persistence;
pub mod state_machine;
pub mod steering;
pub mod tclk_exchange;
pub mod touchlink;

use zigbee_mac::MacDriver;
use zigbee_zdo::ZdoLayer;

pub use attributes::BdbAttributes;
pub use security_persistence::{
    CounterReservation, FRAME_COUNTER_RESERVATION_SIZE, NetworkSecurityState, SecurityPersistence,
    SecurityPersistenceError, TrustCenterLinkKeyState,
};
pub use state_machine::{BdbState, CommissioningMode};
pub use tclk_exchange::{TclkExchange, TclkProgress, TclkStage};

// ── BDB status codes ────────────────────────────────────────

/// BDB commissioning status (BDB spec Table 4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BdbStatus {
    /// Commissioning completed successfully
    Success = 0x00,
    /// Commissioning is currently in progress
    InProgress = 0x01,
    /// The node is not on a network (for operations that require it)
    NotOnNetwork = 0x02,
    /// The operation is not supported by this device type
    NotPermitted = 0x03,
    /// No scan response — no beacons received during steering
    NoScanResponse = 0x04,
    /// Network formation failed
    FormationFailure = 0x05,
    /// Network steering failed after all retries
    SteeringFailure = 0x06,
    /// No Identify Query response during Finding & Binding
    NoIdentifyResponse = 0x07,
    /// Binding table full or cluster matching failed
    BindingTableFull = 0x08,
    /// Touchlink commissioning failed or not supported
    TouchlinkFailure = 0x09,
    /// Target device is not in identifying mode
    TargetFailure = 0x0A,
    /// Operation timed out
    Timeout = 0x0B,
    /// Trust Center link-key exchange failed
    TrustCenterLinkKeyExchangeFailure = 0x0C,
    /// Security state could not be durably persisted.
    PersistenceFailure = 0x0D,
}

/// Last significant stage reached by off-network steering.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SteeringStage {
    #[default]
    Idle = 0,
    Scanning = 1,
    Joining = 2,
    WaitingForTransportKey = 3,
    TransportKeyReceived = 4,
    Announcing = 5,
    VerifyingLinkKey = 6,
    Complete = 7,
    NoNetworks = 8,
    NoJoinCandidate = 9,
    JoinFailed = 10,
    TransportKeyMissing = 11,
    RequestingTrustCenterLinkKey = 12,
    WaitingForTrustCenterLinkKey = 13,
    WaitingForConfirmKey = 14,
    TrustCenterLinkKeyExchangeFailed = 15,
    QueryingTrustCenterNodeDescriptor = 16,
    PersistenceFailed = 17,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum KeyFrameResult {
    #[default]
    None = 0,
    NwkParseFailed = 1,
    SecuredNoActiveKey = 2,
    SecurityHeaderParseFailed = 3,
    ActiveKeyDecryptFailed = 4,
    UnsecuredAps = 5,
    ApsProcessedNoKey = 6,
    KeyInstalled = 7,
}

/// Compact steering telemetry that remains valid after a failed join attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SteeringDiagnostics {
    pub stage: SteeringStage,
    pub scan_requests: u16,
    pub networks_discovered: u16,
    pub permit_closed_rejects: u16,
    pub join_attempts: u16,
    pub join_successes: u16,
    pub last_join_status: u8,
    pub channel: u8,
    pub pan_id: u16,
    pub parent_address: u16,
    pub parent_lqi: u8,
    pub parent_depth: u8,
    pub assigned_address: u16,
    pub passive_rx_frames: u16,
    pub poll_attempts: u16,
    pub poll_data_frames: u16,
    pub poll_errors: u16,
    pub last_frame_len: u8,
    pub last_frame_prefix: [u8; 16],
    pub nwk_header_len: u8,
    pub nwk_security: bool,
    pub key_frame_result: KeyFrameResult,
    pub transport_key_received: bool,
    pub verify_key_attempts: u16,
    pub verify_key_successes: u16,
    pub verify_key_error: u8,
    pub request_key_attempts: u16,
    pub request_key_send_successes: u16,
    pub request_key_send_failures: u16,
    pub request_key_error: u8,
    pub node_desc_requests: u16,
    pub node_desc_send_failures: u16,
    pub node_desc_responses: u16,
    pub node_desc_timeouts: u16,
    pub node_desc_parse_failures: u16,
    pub last_node_desc_status: u8,
    pub trust_center_server_mask: u16,
    pub trust_center_stack_revision: u8,
    pub tclk_installations: u16,
    pub confirm_key_frames: u32,
    pub confirm_key_successes: u32,
    pub confirm_key_rejections: u32,
    pub last_confirm_key_status: u8,
}

// ── BDB layer ───────────────────────────────────────────────

/// The BDB commissioning layer — wraps the ZDO layer and drives
/// the Zigbee 3.0 commissioning state machine.
///
/// Generic over `M: MacDriver` — the hardware-specific MAC.
///
/// # Usage
/// ```rust,ignore
/// let bdb = BdbLayer::new(zdo_layer);
/// bdb.initialize()?;
/// bdb.commission().await?;
/// while bdb.tclk_exchange_active() {
///     // Process normal incoming stack traffic, then:
///     bdb.advance_tclk_exchange(None).await;
/// }
/// ```
pub struct BdbLayer<M: MacDriver> {
    zdo: ZdoLayer<M>,
    attributes: BdbAttributes,
    state: BdbState,
    steering_diagnostics: SteeringDiagnostics,
    /// Pending Find & Bind target request: (endpoint, identify_time_secs)
    pub fb_target_request: Option<(u8, u16)>,
    /// Collected F&B identify query responses: (nwk_addr, endpoint).
    pub fb_identify_responses: heapless::Vec<(u16, u8), 8>,
    /// F&B initiator window — seconds remaining to collect responses.
    /// When > 0, the initiator is waiting for Identify Query Responses.
    pub fb_window_remaining: u16,
    /// Endpoint being used for F&B initiator procedure.
    fb_initiator_endpoint: u8,
    /// In-flight event-driven unique Trust Center link-key exchange.
    ///
    /// Armed after network-up + `Device_annce`; advanced one bounded step per
    /// tick/poll via [`BdbLayer::advance_tclk_exchange`]. `None` when no
    /// post-network commissioning security handshake is pending.
    tclk_exchange: Option<TclkExchange>,
}

impl<M: MacDriver> BdbLayer<M> {
    /// Create a new BDB layer wrapping the given ZDO layer.
    #[inline(never)]
    pub fn new(zdo: ZdoLayer<M>) -> Self {
        Self {
            zdo,
            attributes: BdbAttributes::default(),
            state: BdbState::Idle,
            steering_diagnostics: SteeringDiagnostics::default(),
            fb_target_request: None,
            fb_identify_responses: heapless::Vec::new(),
            fb_window_remaining: 0,
            fb_initiator_endpoint: 0,
            tclk_exchange: None,
        }
    }

    /// Construct a BDB layer directly into caller-provided storage.
    ///
    /// # Safety
    /// `slot` must point to valid, properly aligned, uninitialized storage for `Self`.
    #[inline(never)]
    pub unsafe fn write_into(slot: *mut Self, mac: M, device_type: zigbee_nwk::DeviceType) {
        unsafe {
            ZdoLayer::write_into(core::ptr::addr_of_mut!((*slot).zdo), mac, device_type);
            core::ptr::addr_of_mut!((*slot).attributes).write(BdbAttributes::default());
            core::ptr::addr_of_mut!((*slot).state).write(BdbState::Idle);
            core::ptr::addr_of_mut!((*slot).steering_diagnostics)
                .write(SteeringDiagnostics::default());
            core::ptr::addr_of_mut!((*slot).fb_target_request).write(None);
            core::ptr::addr_of_mut!((*slot).fb_identify_responses).write(heapless::Vec::new());
            core::ptr::addr_of_mut!((*slot).fb_window_remaining).write(0);
            core::ptr::addr_of_mut!((*slot).fb_initiator_endpoint).write(0);
            core::ptr::addr_of_mut!((*slot).tclk_exchange).write(None);
        }
    }

    // ── Layer access ────────────────────────────────────────

    pub fn zdo(&self) -> &ZdoLayer<M> {
        &self.zdo
    }

    pub fn zdo_mut(&mut self) -> &mut ZdoLayer<M> {
        &mut self.zdo
    }

    pub fn attributes(&self) -> &BdbAttributes {
        &self.attributes
    }

    pub fn attributes_mut(&mut self) -> &mut BdbAttributes {
        &mut self.attributes
    }

    pub fn state(&self) -> &BdbState {
        &self.state
    }

    pub fn steering_diagnostics(&self) -> SteeringDiagnostics {
        self.steering_diagnostics
    }

    /// Whether the device is currently on a Zigbee network.
    pub fn is_on_network(&self) -> bool {
        self.attributes.node_is_on_a_network
    }

    /// Reset BDB attributes to factory defaults.
    ///
    /// Called as part of factory reset — restores all BDB attributes
    /// to their default values and clears internal F&B state.
    pub fn reset_attributes(&mut self) {
        self.attributes = BdbAttributes::default();
        self.fb_target_request = None;
        self.fb_identify_responses.clear();
        self.fb_window_remaining = 0;
        self.fb_initiator_endpoint = 0;
        self.tclk_exchange = None;
        self.state = BdbState::Idle;
    }

    /// Whether an event-driven unique Trust Center link-key exchange is still
    /// running. The runtime uses this to decide when to advance commissioning
    /// security from its tick/poll loop.
    pub fn tclk_exchange_active(&self) -> bool {
        self.tclk_exchange.is_some()
    }

    /// The current stage of the in-flight unique-TCLK exchange, if any.
    pub fn tclk_exchange_stage(&self) -> Option<TclkStage> {
        self.tclk_exchange.as_ref().map(|exchange| exchange.stage)
    }

    /// Arm a unique-TCLK exchange directly, bypassing scan/join, for tests.
    #[cfg(test)]
    pub(crate) fn arm_tclk_exchange_for_test(
        &mut self,
        tc_addr: zigbee_types::ShortAddress,
        tc_ieee: zigbee_types::IeeeAddress,
    ) {
        let now = self.zdo.aps().nwk().mac().monotonic_micros();
        self.attributes.node_is_on_a_network = true;
        self.tclk_exchange = Some(TclkExchange::new(tc_addr, tc_ieee, now));
    }
}
