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
pub mod state_machine;
pub mod steering;
pub mod touchlink;

use zigbee_mac::MacDriver;
use zigbee_zdo::ZdoLayer;

pub use attributes::BdbAttributes;
pub use state_machine::{BdbState, CommissioningMode};

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
/// bdb.initialize().await?;
/// bdb.commission().await?;
/// ```
pub struct BdbLayer<M: MacDriver> {
    zdo: ZdoLayer<M>,
    attributes: BdbAttributes,
    state: BdbState,
    /// Pending Find & Bind target request: (endpoint, identify_time_secs)
    pub fb_target_request: Option<(u8, u16)>,
    /// Collected F&B identify query responses: (nwk_addr, endpoint).
    pub fb_identify_responses: heapless::Vec<(u16, u8), 8>,
    /// F&B initiator window — seconds remaining to collect responses.
    /// When > 0, the initiator is waiting for Identify Query Responses.
    pub fb_window_remaining: u16,
    /// Endpoint being used for F&B initiator procedure.
    fb_initiator_endpoint: u8,
}

impl<M: MacDriver> BdbLayer<M> {
    /// Create a new BDB layer wrapping the given ZDO layer.
    pub fn new(zdo: ZdoLayer<M>) -> Self {
        Self {
            zdo,
            attributes: BdbAttributes::default(),
            state: BdbState::Idle,
            fb_target_request: None,
            fb_identify_responses: heapless::Vec::new(),
            fb_window_remaining: 0,
            fb_initiator_endpoint: 0,
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
        self.state = BdbState::Idle;
    }
}
