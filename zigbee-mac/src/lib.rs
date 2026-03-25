//! IEEE 802.15.4 MAC abstraction layer for zigbee-rs.
//!
//! This crate defines the `MacDriver` trait — the single abstraction boundary
//! between platform-specific 802.15.4 radio hardware and the platform-independent
//! Zigbee stack (NWK, APS, ZCL, BDB).
//!
//! Each hardware platform implements `MacDriver` once (~500 lines). The entire
//! upper stack is built against this trait and never touches hardware directly.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────┐
//! │  Zigbee Stack (NWK / APS / ZCL / BDB)   │  platform-independent
//! └────────────────┬────────────────────────┘
//!                  │ MacDriver trait
//! ┌────────────────┴────────────────────────┐
//! │  MAC backends: esp / nrf / stm32wb / …  │  platform-specific
//! └─────────────────────────────────────────┘
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod pib;
pub mod primitives;

#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod esp;

#[cfg(any(feature = "nrf52840", feature = "nrf52833"))]
pub mod nrf;

#[cfg(feature = "stm32wb55")]
pub mod stm32wb;

#[cfg(feature = "efr32mg24")]
pub mod efr32;

#[cfg(feature = "cc2652")]
pub mod cc26xx;

#[cfg(feature = "bl702")]
pub mod bl702;

#[cfg(feature = "serial")]
pub mod serial;

#[cfg(any(test, feature = "mock"))]
pub mod mock;

// Re-exports for convenience
pub use pib::{PibAttribute, PibValue};
pub use primitives::*;

use zigbee_types::TxPower;

// ── Error types ─────────────────────────────────────────────────

/// MAC layer error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacError {
    /// No beacon received during scan
    NoBeacon,
    /// Invalid parameters supplied to primitive
    InvalidParameter,
    /// Radio hardware error
    RadioError,
    /// Channel access failure (CSMA-CA failed)
    ChannelAccessFailure,
    /// No acknowledgement received
    NoAck,
    /// Frame too long for PHY
    FrameTooLong,
    /// Operation not supported by this backend
    Unsupported,
    /// Security processing failed
    SecurityError,
    /// Frame pending in indirect queue
    TransactionOverflow,
    /// Transaction expired before delivery
    TransactionExpired,
    /// Scan already in progress
    ScanInProgress,
    /// Superframe tracking lost
    TrackingOff,
    /// Association denied by coordinator
    AssociationDenied,
    /// PAN at capacity
    PanAtCapacity,
    /// Unknown error
    Other,
}

// ── The MAC Driver trait ────────────────────────────────────────

/// Abstract async MAC driver — the sole interface between hardware and stack.
///
/// Implementors provide platform-specific 802.15.4 radio access. The trait
/// covers the minimal complete set of MLME/MCPS primitives needed for
/// Zigbee PRO R22 operation as End Device, Router, or Coordinator.
///
/// All methods are async to accommodate interrupt-driven radios with
/// Embassy/async executors. Implementations MUST be safe to call from
/// a single-threaded async executor (no `Send`/`Sync` requirement).
pub trait MacDriver {
    // ── MLME: Scan ──────────────────────────────────────────

    /// MLME-SCAN.request — perform ED, Active, Passive, or Orphan scan.
    ///
    /// Scans the channels specified in `req.channel_mask` for the given
    /// `req.scan_duration`. Returns discovered PAN descriptors (active/passive)
    /// or energy measurements (ED scan).
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError>;

    // ── MLME: Association ───────────────────────────────────

    /// MLME-ASSOCIATE.request — request association with a coordinator.
    ///
    /// Sends an Association Request command to `req.coord_address` on
    /// `req.channel`. Returns the assigned short address on success.
    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError>;

    /// MLME-ASSOCIATE.response — respond to an association indication.
    ///
    /// Only needed for Coordinator/Router roles. Sends the association
    /// response (with assigned short address or denial) back to the
    /// requesting device.
    async fn mlme_associate_response(&mut self, rsp: MlmeAssociateResponse)
    -> Result<(), MacError>;

    /// MLME-DISASSOCIATE.request — leave the PAN.
    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError>;

    // ── MLME: Start / Reset ─────────────────────────────────

    /// MLME-RESET.request — reset MAC to default state.
    ///
    /// If `set_default_pib` is true, all PIB attributes are reset to defaults.
    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError>;

    /// MLME-START.request — start a PAN (coordinator) or begin transmitting
    /// beacons (router). End devices do not use this.
    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError>;

    // ── MLME: PIB access ────────────────────────────────────

    /// MLME-GET.request — read a MAC PIB attribute.
    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError>;

    /// MLME-SET.request — write a MAC PIB attribute.
    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError>;

    // ── MLME: Orphan / Sync / Poll ──────────────────────────

    /// MLME-POLL.request — request data from coordinator (for sleepy devices).
    ///
    /// Sends a Data Request command to the coordinator and waits for
    /// any pending indirect frame.
    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError>;

    // ── MCPS: Data service ──────────────────────────────────

    /// MCPS-DATA.request — transmit a MAC frame.
    ///
    /// Sends `req.payload` to `req.dst_address` with the specified options
    /// (ack request, security, etc.). Returns TX confirmation.
    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError>;

    /// MCPS-DATA.indication — receive the next incoming MAC frame.
    ///
    /// Blocks until a frame is received from the radio. The caller is
    /// responsible for filtering by frame type / addressing.
    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError>;

    // ── Capability queries ──────────────────────────────────

    /// Returns the hardware capabilities of this MAC backend.
    fn capabilities(&self) -> MacCapabilities;
}

// ── Capability descriptor ───────────────────────────────────────

/// What this MAC backend can do — lets the stack adapt behavior.
#[derive(Debug, Clone, Copy)]
pub struct MacCapabilities {
    /// Can act as PAN coordinator (start network, assign addresses)
    pub coordinator: bool,
    /// Can act as router (relay frames)
    pub router: bool,
    /// Supports MAC-level security (encryption in hardware)
    pub hardware_security: bool,
    /// Maximum frame payload size (typically 127 - overhead)
    pub max_payload: u16,
    /// Supported TX power range
    pub tx_power_min: TxPower,
    pub tx_power_max: TxPower,
}
