//! MLME and MCPS primitive request/confirm/indication types.
//!
//! These structs map to IEEE 802.15.4 MAC service primitives as required
//! by a Zigbee PRO R22 stack. Each primitive follows the pattern:
//!   - Request:    parameters the upper layer sends DOWN to MAC
//!   - Confirm:    result MAC sends UP after completing the request
//!   - Indication: unsolicited event MAC sends UP (e.g. received frame)

use crate::{MacError, pib::PibPayload};
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
    /// IEEE 802.15.4 Link Quality Indicator, normalized to `0..=255` (larger
    /// is better).
    ///
    /// NWK link cost (R22 §3.6.3.1) and rejoin parent selection are derived
    /// from this value, so backends must normalize a raw hardware reading
    /// before it reaches here; see [`crate::lqi`].
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
pub const MAX_ED_VALUES: usize = 16;
pub type EdList = heapless::Vec<EdValue, MAX_ED_VALUES>;

// ── Association ─────────────────────────────────────────────────

/// Device capability info (sent in Association Request)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
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
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeAssociateIndication {
    /// Extended address of the requesting device
    pub device_address: IeeeAddress,
    /// Address of the coordinator/router targeted by the request.
    pub coordinator_address: MacAddress,
    pub capability_info: CapabilityInfo,
    pub lqi: u8,
    /// Whether MAC security was enabled on the received command.
    pub security_use: bool,
}

/// A coordinator/router received a Beacon Request command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeBeaconRequestIndication {
    /// Broadcast destination carried by the request.
    pub destination_address: MacAddress,
    pub lqi: u8,
    /// Whether MAC security was enabled on the received command.
    pub security_use: bool,
}

/// A coordinator/router received a Data Request command.
///
/// `source_address` is deliberately a full [`MacAddress`]: associated
/// children normally poll with a short address, while a device waiting for
/// its Association Response polls with its extended address.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeDataRequestIndication {
    pub source_address: MacAddress,
    pub destination_address: MacAddress,
    pub lqi: u8,
    /// Whether MAC security was enabled on the received command.
    pub security_use: bool,
}

/// Completion of a deferred MLME-ASSOCIATE.response transaction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeAssociateResponseDelivery {
    pub device_address: IeeeAddress,
    pub short_address: ShortAddress,
    pub status: AssociationStatus,
    /// `Ok(())` means the Association Response was acknowledged over the air.
    pub result: Result<(), MacError>,
}

/// A coordinator/router received an Orphan Notification command.
///
/// The command carries no payload beyond the orphan's extended address, which
/// is the only identity a parent may use to decide whether the device is its
/// child (R22 §3.6.1.4.3.2).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeOrphanIndication {
    /// Extended address of the orphaned device.
    pub orphan_address: IeeeAddress,
    /// Broadcast destination carried by the notification.
    pub destination_address: MacAddress,
    pub lqi: u8,
    /// Whether MAC security was enabled on the received command.
    pub security_use: bool,
}

/// MLME-ORPHAN.response — a parent's answer to an orphan notification.
///
/// `associated_member == false` terminates the procedure without transmitting
/// anything (R22 §3.6.1.4.3.2: "the procedure shall be terminated without
/// indication to the higher layer"); it exists so a backend can distinguish an
/// explicit "not my child" decision from a dropped event.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeOrphanResponse {
    /// Extended address of the orphaned device.
    pub orphan_address: IeeeAddress,
    /// Network address this parent already holds for the orphan; sent in the
    /// Coordinator Realignment `Short Address` field.
    pub short_address: ShortAddress,
    /// Whether the orphan is a child of this device.
    pub associated_member: bool,
}

/// MAC management/command event delivered independently of MCPS data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MacCommandEvent {
    BeaconRequest(MlmeBeaconRequestIndication),
    AssociationRequest(MlmeAssociateIndication),
    AssociationResponseDelivery(MlmeAssociateResponseDelivery),
    DataRequest(MlmeDataRequestIndication),
    OrphanNotification(MlmeOrphanIndication),
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

// ── Beacon response / indirect delivery ────────────────────────

/// Maximum number of pending short or extended addresses encodable in one
/// IEEE 802.15.4 beacon Pending Address Specification field.
pub const MAX_BEACON_PENDING_ADDRESSES: usize = 7;
pub type BeaconPendingShortList = heapless::Vec<ShortAddress, MAX_BEACON_PENDING_ADDRESSES>;
pub type BeaconPendingExtendedList = heapless::Vec<IeeeAddress, MAX_BEACON_PENDING_ADDRESSES>;

/// Parameters for an on-demand beacon response in Zigbee non-beacon mode.
///
/// Beacon/superframe orders are fixed to 15, the GTS specification is empty,
/// and the bounded pending-address lists are encoded in the MAC beacon.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MlmeBeaconResponse {
    pub pan_coordinator: bool,
    pub association_permit: bool,
    pub pending_short_addresses: BeaconPendingShortList,
    pub pending_extended_addresses: BeaconPendingExtendedList,
    pub beacon_payload: PibPayload,
}

impl MlmeBeaconResponse {
    pub fn new(beacon_payload: PibPayload) -> Self {
        Self {
            pan_coordinator: false,
            association_permit: false,
            pending_short_addresses: BeaconPendingShortList::new(),
            pending_extended_addresses: BeaconPendingExtendedList::new(),
            beacon_payload,
        }
    }
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

/// Validate an MLME-START.request that a **parent-capable** backend is willing
/// to honour as a Zigbee non-beacon *router* start.
///
/// Zigbee PRO R22 operates exclusively in non-beacon mode, so the only start a
/// router accepts has `beacon_order == superframe_order == 15` and
/// `pan_coordinator == false`. A PAN-coordinator start is a *separate*
/// capability ([`MacCapabilities::coordinator`](crate::MacCapabilities)) that no
/// in-tree backend implements yet, so it is rejected here rather than silently
/// treated as a router start.
///
/// Shared by every backend that accepts MLME-START so the accepted parameter
/// shape is identical across platforms and host-testable in one place.
///
/// # Errors
///
/// - [`MacError::Unsupported`] — a PAN-coordinator start, or beacon mode.
/// - [`MacError::InvalidParameter`] — channel outside the 2.4 GHz O-QPSK page-0
///   range 11..=26, or the broadcast PAN ID `0xFFFF`.
pub fn validate_router_start(req: &MlmeStartRequest) -> Result<(), MacError> {
    if req.pan_coordinator {
        return Err(MacError::Unsupported);
    }
    if req.beacon_order != 15 || req.superframe_order != 15 {
        return Err(MacError::Unsupported);
    }
    if !(11..=26).contains(&req.channel) {
        return Err(MacError::InvalidParameter);
    }
    if req.pan_id.0 == 0xFFFF {
        return Err(MacError::InvalidParameter);
    }
    Ok(())
}

/// The MLME-START.request outcome for a backend that does **not** implement the
/// sealed [`ParentMacDriver`](crate::ParentMacDriver) parent primitives.
///
/// Starting a PAN — as coordinator or as router — is an assertion that the
/// device will answer Beacon Requests, admit children and deliver indirect
/// transactions. A backend that retains the `Unsupported`/`NoData`
/// [`MacDriver`](crate::MacDriver) defaults for those primitives can do none of
/// that, so it must fail explicitly instead of returning `Ok(())` after merely
/// retuning its channel and PAN ID: a silent success makes the NWK layer
/// believe a router started, which is exactly the dishonest capability claim
/// the `ParentMacDriver` seal exists to prevent.
///
/// Malformed requests are still reported as [`MacError::InvalidParameter`] so a
/// caller can distinguish a programming error from a missing capability.
///
/// This function never returns `Ok`. When a backend gains real parent
/// primitives (and therefore `ParentMacDriver`), replace the call with
/// [`validate_router_start`] plus the platform's own radio configuration.
pub fn start_requires_parent_capability(req: &MlmeStartRequest) -> Result<(), MacError> {
    validate_router_start(req)?;
    Err(MacError::Unsupported)
}

// ── Data service ────────────────────────────────────────────────

/// Transmit options for MCPS-DATA
#[derive(Debug, Clone, Copy, Default)]
pub struct TxOptions {
    /// Request MAC-level acknowledgement
    pub ack_tx: bool,
    /// Advertise that another indirect transaction remains queued.
    pub frame_pending: bool,
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

#[cfg(test)]
mod start_validation_tests {
    use super::*;

    fn router_start() -> MlmeStartRequest {
        MlmeStartRequest {
            pan_id: PanId(0x1A62),
            channel: 15,
            beacon_order: 15,
            superframe_order: 15,
            pan_coordinator: false,
            battery_life_ext: false,
        }
    }

    #[test]
    fn well_formed_router_start_is_accepted() {
        assert_eq!(validate_router_start(&router_start()), Ok(()));
    }

    #[test]
    fn pan_coordinator_start_is_unsupported() {
        let mut req = router_start();
        req.pan_coordinator = true;
        assert_eq!(validate_router_start(&req), Err(MacError::Unsupported));
    }

    #[test]
    fn beacon_mode_start_is_unsupported() {
        let mut req = router_start();
        req.beacon_order = 8;
        assert_eq!(validate_router_start(&req), Err(MacError::Unsupported));

        let mut req = router_start();
        req.superframe_order = 8;
        assert_eq!(validate_router_start(&req), Err(MacError::Unsupported));
    }

    #[test]
    fn channel_outside_page_zero_is_invalid() {
        for channel in [0u8, 10, 27, 255] {
            let mut req = router_start();
            req.channel = channel;
            assert_eq!(
                validate_router_start(&req),
                Err(MacError::InvalidParameter),
                "channel {channel} must be rejected"
            );
        }
        for channel in 11..=26u8 {
            let mut req = router_start();
            req.channel = channel;
            assert_eq!(validate_router_start(&req), Ok(()));
        }
    }

    #[test]
    fn broadcast_pan_id_is_invalid() {
        let mut req = router_start();
        req.pan_id = PanId(0xFFFF);
        assert_eq!(validate_router_start(&req), Err(MacError::InvalidParameter));
    }

    // ── Non-parent backends ──────────────────────────────────────

    #[test]
    fn non_parent_backend_never_starts_a_pan() {
        // Even a perfectly well-formed router start must fail: the backend
        // cannot answer Beacon Requests or serve children.
        assert_eq!(
            start_requires_parent_capability(&router_start()),
            Err(MacError::Unsupported)
        );
    }

    #[test]
    fn non_parent_backend_rejects_coordinator_start() {
        let mut req = router_start();
        req.pan_coordinator = true;
        assert_eq!(
            start_requires_parent_capability(&req),
            Err(MacError::Unsupported)
        );
    }

    #[test]
    fn non_parent_backend_reports_malformed_requests_distinctly() {
        // A missing capability and a programming error must stay
        // distinguishable at the call site.
        let mut req = router_start();
        req.channel = 99;
        assert_eq!(
            start_requires_parent_capability(&req),
            Err(MacError::InvalidParameter)
        );
    }
}

#[cfg(test)]
mod parent_capability_tests {
    use crate::{MacCapabilities, TxPower};

    /// Every in-tree non-parent backend routes its descriptor through
    /// `MacCapabilities::non_parent`. Exercising the exact argument sets used
    /// by nRF, ESP32, EFR32MG1, EFR32MG21, BL702, CC2340 and PHY62x2 pins the
    /// invariant for all of them: a backend that keeps the `MacDriver`
    /// parent-primitive defaults can never advertise a routing or
    /// coordinating capability.
    #[test]
    fn non_parent_backends_never_claim_router_or_coordinator() {
        let backends = [
            ("nrf", 102u16, TxPower(-20), TxPower(8)),
            ("esp", 102, TxPower(-24), TxPower(21)),
            ("efr32", 102, TxPower(-20), TxPower(19)),
            ("efr32s2", 102, TxPower(-20), TxPower(19)),
            ("bl702", 102, TxPower(-21), TxPower(14)),
            ("cc2340", 116, TxPower(-20), TxPower(8)),
            ("phy6222", 102, TxPower(0), TxPower(10)),
        ];

        for (name, payload, min, max) in backends {
            let capabilities = MacCapabilities::non_parent(payload, min, max);
            assert!(
                !capabilities.router,
                "{name} must not advertise router capability without ParentMacDriver"
            );
            assert!(
                !capabilities.coordinator,
                "{name} must not advertise coordinator capability without ParentMacDriver"
            );
            assert!(
                !capabilities.hardware_security,
                "{name} performs Zigbee CCM* in the Rust stack"
            );
            assert_eq!(capabilities.max_payload, payload);
            assert_eq!(capabilities.tx_power_min.0, min.0);
            assert_eq!(capabilities.tx_power_max.0, max.0);
        }
    }

    /// The runtime arms parent servicing from `capabilities().router` and
    /// `zigbee-bdb` gates network formation on `.coordinator`, so a genuine
    /// parent backend must still be able to claim both. This guards against
    /// "fix" attempts that force every backend to `false`.
    #[cfg(feature = "mock")]
    #[test]
    fn a_sealed_parent_backend_can_still_claim_parent_capability() {
        use crate::{ParentMacDriver, mock::MockMac};

        // Only type-checks because `MockMac` implements the sealed trait.
        fn requires_parent_mac<M: ParentMacDriver>(mac: &M) -> crate::MacCapabilities {
            mac.capabilities()
        }

        let mac = MockMac::new([0x11; 8]);
        let capabilities = requires_parent_mac(&mac);
        assert!(capabilities.router);
        assert!(capabilities.coordinator);
    }
}
