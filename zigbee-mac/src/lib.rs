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
//! │  MAC backends: esp / nrf / bl702 / …     │  platform-specific
//! └─────────────────────────────────────────┘
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod frames;
pub mod phy;
pub mod pib;
pub mod platform;
pub mod primitives;
pub mod soft_mac;

#[cfg(any(test, feature = "esp32c6", feature = "esp32h2"))]
mod esp_aes;

#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod esp;

#[cfg(any(feature = "nrf52840", feature = "nrf52833"))]
pub mod nrf;

#[cfg(feature = "bl702")]
pub mod bl702;

#[cfg(feature = "cc2340")]
pub mod cc2340;

#[cfg(feature = "tlsr8258")]
pub mod telink;

#[cfg(feature = "phy6222")]
pub mod phy6222;

#[cfg(feature = "efr32")]
pub mod efr32;

#[cfg(feature = "efr32mg21")]
pub mod efr32s2;

#[cfg(any(test, feature = "mock"))]
pub mod mock;

// Re-exports for convenience
pub use phy::{
    MAX_PHY_FRAME_LEN, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame, RadioPhy,
};
pub use pib::{MacPib, PibAttribute, PibError, PibValue};
pub use platform::{ForwardAesProvider, PlatformServices, WrappingTickExtender};
pub use primitives::*;
pub use soft_mac::{AckResult, SoftMacCore};

use zigbee_types::{MacAddress, TxPower};

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
    /// No data frame received within timeout
    NoData,
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
pub trait MacDriver: PlatformServices {
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
    /// Only needed for Coordinator/Router roles. Retains the response as a
    /// bounded indirect transaction, sets ACK Frame Pending for the joining
    /// EUI-64, and transmits only after that device sends a matching extended
    /// address Data Request. `Ok(())` confirms queueing, not over-air delivery.
    async fn mlme_associate_response(&mut self, rsp: MlmeAssociateResponse)
    -> Result<(), MacError>;

    /// Transmit an on-demand beacon after a Beacon Request.
    ///
    /// Backends that cannot act as a parent retain an unsupported default.
    async fn mlme_beacon_response(&mut self, _rsp: MlmeBeaconResponse) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    /// MLME-DISASSOCIATE.request — leave the PAN.
    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError>;

    // ── MLME: Start / Reset ─────────────────────────────────

    /// MLME-RESET.request — reset MAC to default state.
    ///
    /// If `set_default_pib` is true, all PIB attributes are reset to defaults.
    fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError>;

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

    /// Poll the coordinator, discarding any response completed after the
    /// supplied timeout.
    async fn mlme_poll_timeout(&mut self, timeout_us: u32) -> Result<Option<MacFrame>, MacError> {
        let started = self.monotonic_micros();
        let result = self.mlme_poll().await?;
        if self.monotonic_micros().wrapping_sub(started) >= timeout_us {
            Ok(None)
        } else {
            Ok(result)
        }
    }

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

    /// Receive the next incoming MAC frame before `timeout_us` expires.
    ///
    /// Unlike [`Self::mcps_data_indication`], this primitive must not block
    /// beyond the supplied timeout and returns [`MacError::NoData`] when the
    /// deadline expires.
    async fn mcps_data_indication_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError>;

    /// Configure whether ACKs to Data Requests from `child` carry the Frame
    /// Pending bit.
    ///
    /// The caller owns the indirect transaction queue and must arm this
    /// before the child polls, then clear it after its last queued transaction
    /// is dequeued. Implementations must bound the remembered child set.
    fn set_indirect_data_pending(
        &mut self,
        _child: MacAddress,
        _pending: bool,
    ) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    /// Transmit one indirect transaction already dequeued by the upper layer.
    ///
    /// This is called only after a matching Data Request event. It does not
    /// transfer queue ownership to the backend.
    async fn mcps_indirect_data(
        &mut self,
        _req: McpsDataRequest<'_>,
    ) -> Result<McpsDataConfirm, MacError> {
        Err(MacError::Unsupported)
    }

    // ── MAC management/command events ─────────────────────

    /// Receive the next MAC command event independently of MCPS data.
    ///
    /// Backends that do not expose command frames retain their existing
    /// behavior through this default no-event implementation.
    async fn mac_command_event(&mut self) -> Result<MacCommandEvent, MacError> {
        Err(MacError::NoData)
    }

    /// Receive a MAC command event before `timeout_us` expires.
    ///
    /// Implementations must retain any normal data frame observed while
    /// waiting so it remains available through `mcps_data_indication`.
    async fn mac_command_event_timeout(
        &mut self,
        _timeout_us: u32,
    ) -> Result<MacCommandEvent, MacError> {
        self.mac_command_event().await
    }

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
    /// Whether the backend performs autonomous MAC-level 802.15.4 security
    /// in hardware (frame encryption/authentication offloaded to the radio).
    ///
    /// This is **not** a "the silicon has an AES block" flag: a backend whose
    /// portable Rust stack still performs Zigbee CCM* itself — even when it
    /// sources the block cipher from a hardware accelerator via
    /// `ForwardAesProvider` — reports `false`.
    pub hardware_security: bool,
    /// Maximum frame payload size (typically 127 - overhead)
    pub max_payload: u16,
    /// Supported TX power range
    pub tx_power_min: TxPower,
    pub tx_power_max: TxPower,
}

// ── Parent-side capability boundary ─────────────────────────────

/// Capability marker for a MAC backend that genuinely implements the
/// parent-side 802.15.4 operations required to accept and serve children.
///
/// The base [`MacDriver`] trait keeps `mlme_beacon_response`,
/// `set_indirect_data_pending`, `mcps_indirect_data`, `mac_command_event` and
/// `mac_command_event_timeout` as `Unsupported`/`NoData` defaults so that an
/// **end-device** backend needs to implement only [`MacDriver`]. Implementing
/// `ParentMacDriver` is an explicit, *truthful* assertion that a backend
/// overrides those parent primitives with real behavior:
///
/// - on-demand beacon response to a Beacon Request,
/// - association indication → response with ACK Frame Pending,
/// - child Data Request events with indirect (frame-pending) delivery.
///
/// A backend that only satisfies the [`MacDriver`] defaults (returning
/// `Unsupported`/`NoData`) **must not** implement this trait, so a router role
/// cannot be constructed on top of a MAC that cannot parent. This keeps a
/// platform's capability claim honest and lets the runtime bound router
/// construction on a genuine parent MAC rather than the Cargo `router` feature
/// alone.
///
/// # Sealed, audited capability assertion — not compiler proof
///
/// `ParentMacDriver` is **sealed**: its supertrait [`sealed::SealedParent`] can
/// only be named inside this crate, so the trait can be implemented **only by
/// in-tree MAC backends** whose parent primitives have actually been reviewed.
/// This is deliberately an *audited assertion*, not a mechanical proof — the
/// type system cannot verify that a backend truly answers Beacon Requests,
/// associates children and delivers indirect data, so sealing prevents an
/// arbitrary downstream crate from falsely claiming the capability to unlock
/// router construction. Adding a new parent backend therefore requires an
/// in-crate `impl sealed::SealedParent` plus `impl ParentMacDriver`, which is
/// the point at which the parent behaviour is reviewed.
///
/// The trait carries no additional methods in this phase: it is a compile-time
/// capability boundary layered over the existing [`MacDriver`] surface. Moving
/// the parent primitive *declarations* out of [`MacDriver`] and into this
/// trait is deferred to a later slice (it ripples through every backend and the
/// feature-gated `receive`/`tick` parent interleaving).
pub trait ParentMacDriver: MacDriver + sealed::SealedParent {}

pub(crate) mod sealed {
    /// Seals [`ParentMacDriver`](super::ParentMacDriver) to in-tree backends.
    ///
    /// Only backends defined in this crate can name and implement this trait,
    /// so the parent-capability assertion cannot be forged by a downstream
    /// crate. See the `ParentMacDriver` docs for why this is an audited
    /// assertion rather than a compiler-checked proof.
    pub trait SealedParent {}
}
