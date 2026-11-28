//! NETWORK group (`0x01xx`, endpoint ZIGBEE), spec 7.2.

use crate::codec::{List, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::types::{Ieee, Key};

wire_struct! {
    /// `NWK_INIT` request (host checkpoint floors, 0 = none).
    pub struct NwkInit {
        /// NWK frame-counter floor.
        pub nwk_fc_floor: u32,
        /// APS frame-counter floor.
        pub aps_fc_floor: u32,
    }
}

wire_struct! {
    /// `NWK_INIT` response body.
    pub struct NwkInitRsp {
        /// Network state.
        pub network_state: u8,
    }
}

wire_struct! {
    /// `NWK_FORM` request.
    pub struct NwkForm {
        /// Channel mask (bit n = channel n).
        pub channel_mask: u32,
        /// PAN ID (`FFFF` = random).
        pub pan_id: u16,
        /// Extended PAN ID (0 = random).
        pub ext_pan_id: [u8; 8],
        /// Network key (0 = random).
        pub network_key: Key,
        /// Key sequence.
        pub key_seq: u8,
        /// NWK update ID.
        pub nwk_update_id: u8,
        /// TX power (dBm).
        pub tx_power: i8,
        /// Network manager.
        pub nwk_manager: u16,
    }
}

wire_struct! {
    /// `NWK_LEAVE_RESET` request.
    pub struct NwkLeaveReset {
        /// [`leave_reset_flags`].
        pub flags: u8,
    }
}

/// `flags` of `NWK_LEAVE_RESET`.
pub mod leave_reset_flags {
    /// Keep the IEEE override.
    pub const KEEP_IEEE: u8 = 0x01;
    /// Keep the host-endpoint registrations.
    pub const KEEP_ENDPOINTS: u8 = 0x02;
    /// Broadcast a NWK Leave (rejoin 0) before the wipe: an explicit "leave
    /// network". Without it the wipe is local and nothing is transmitted,
    /// as a wipe before a restore or a new formation must be: the routers
    /// of the network would drop a coordinator they heard leave.
    pub const ANNOUNCE_LEAVE: u8 = 0x04;
}

wire_struct! {
    /// `NWK_GET_INFO` response body.
    pub struct NwkGetInfoRsp {
        /// State.
        pub state: u8,
        /// PAN ID.
        pub pan_id: u16,
        /// Extended PAN ID (on-air order).
        pub ext_pan_id: [u8; 8],
        /// Channel.
        pub channel: u8,
        /// Channel mask.
        pub channel_mask: u32,
        /// Short address.
        pub short: u16,
        /// IEEE.
        pub ieee: Ieee,
        /// NWK update ID.
        pub nwk_update_id: u8,
        /// Network manager.
        pub nwk_manager: u16,
        /// Key sequence.
        pub key_seq: u8,
        /// Security level.
        pub security_level: u8,
        /// TX power.
        pub tx_power: i8,
        /// Stack profile.
        pub stack_profile: u8,
    }
}

wire_struct! {
    /// `NWK_PERMIT_JOIN` request.
    pub struct NwkPermitJoin {
        /// Duration in seconds.
        pub duration_s: u8,
        /// `0xFFFC` all routers + NCP, `0x0000` NCP only, else a router.
        pub target: u16,
        /// Bit0 TC significance.
        pub flags: u8,
    }
}

wire_struct! {
    /// `NWK_ENERGY_SCAN` request.
    pub struct NwkEnergyScan {
        /// Channel mask.
        pub channel_mask: u32,
        /// Scan duration exponent.
        pub duration_exp: u8,
        /// Repetitions.
        pub count: u8,
    }
}

wire_struct! {
    /// `NWK_CHANGE_CHANNEL` request.
    pub struct NwkChangeChannel {
        /// New channel.
        pub channel: u8,
        /// Update ID (0 = current + 1).
        pub nwk_update_id: u8,
    }
}

wire_struct! {
    /// `NWK_SET_TX_POWER` request.
    pub struct NwkSetTxPower {
        /// Requested dBm.
        pub dbm: i8,
    }
}

wire_struct! {
    /// `NWK_SET_TX_POWER` response body.
    pub struct NwkSetTxPowerRsp {
        /// Applied dBm.
        pub applied_dbm: i8,
    }
}

wire_struct! {
    /// `NWK_CONCENTRATOR` request.
    pub struct NwkConcentrator {
        /// 0 off, 1 low-RAM, 2 high-RAM.
        pub mode: u8,
        /// Minimum MTORR interval.
        pub min_s: u16,
        /// Maximum MTORR interval.
        pub max_s: u16,
        /// Radius.
        pub radius: u8,
    }
}

wire_struct! {
    /// `NWK_ACTIVE_SCAN` request.
    pub struct NwkActiveScan {
        /// Channel mask.
        pub channel_mask: u32,
        /// Duration exponent.
        pub duration_exp: u8,
    }
}

wire_struct! {
    /// One beacon in `NWK_ACTIVE_SCAN` results.
    pub struct ScanResult {
        /// Channel.
        pub channel: u8,
        /// PAN ID.
        pub pan: u16,
        /// Extended PAN ID.
        pub epid: [u8; 8],
        /// Source short address.
        pub src: u16,
        /// LQI.
        pub lqi: u8,
        /// RSSI.
        pub rssi: i8,
        /// Permit-join flag.
        pub permit: u8,
        /// Stack profile.
        pub stack_profile: u8,
        /// Update ID.
        pub update_id: u8,
    }
}

wire_struct! {
    /// `NWK_ACTIVE_SCAN` response body.
    pub struct NwkActiveScanRsp<'a> {
        /// Beacons.
        pub results: List<'a, ScanResult>,
    }
}

wire_struct! {
    /// `NWK_ADD_ENDPOINT` request.
    pub struct NwkAddEndpoint<'a> {
        /// Zigbee endpoint.
        pub ep: u8,
        /// Profile.
        pub profile: u16,
        /// Device ID.
        pub device_id: u16,
        /// Device version.
        pub version: u8,
        /// Input clusters.
        pub in_clusters: List<'a, u16>,
        /// Output clusters.
        pub out_clusters: List<'a, u16>,
    }
}

wire_struct! {
    /// `NWK_SET_MANUFACTURER_CODE` request.
    pub struct NwkSetManufacturerCode {
        /// Manufacturer code.
        pub code: u16,
    }
}

wire_struct! {
    /// `NWK_GROUP_MEMBERSHIP` request.
    pub struct NwkGroupMembership {
        /// 1 add, 2 remove, 3 clear.
        pub op: u8,
        /// Group.
        pub group: u16,
        /// Zigbee endpoint.
        pub ep: u8,
    }
}

wire_struct! {
    /// `NWK_REMOVE_DEVICE` request.
    pub struct NwkRemoveDevice {
        /// Short address.
        pub nwk: u16,
        /// IEEE.
        pub ieee: Ieee,
        /// Bit0 rejoin, bit1 remove children, bit2 TC remove via parent.
        pub flags: u8,
    }
}

wire_struct! {
    /// `EVT_NETWORK_STATE`.
    pub struct EvtNetworkState {
        /// 0 down, 1 forming, 2 up, 3 leaving.
        pub state: u8,
        /// Reason.
        pub reason: u8,
        /// Zigbee status.
        pub zb_status: u8,
        /// TID of the request that caused it (0 if none).
        pub req_tid: u16,
    }
}

wire_struct! {
    /// `EVT_DEVICE_JOINED`.
    pub struct EvtDeviceJoined {
        /// Short address.
        pub nwk: u16,
        /// IEEE.
        pub ieee: Ieee,
        /// Parent short address.
        pub parent_nwk: u16,
        /// 0 association, 1 secured rejoin, 2 unsecured/TC rejoin, 3 direct.
        pub join_type: u8,
        /// Capability.
        pub capability: u8,
        /// 0 admitted, 1 denied.
        pub decision: u8,
    }
}

wire_struct! {
    /// `EVT_DEVICE_AUTHORIZED`.
    pub struct EvtDeviceAuthorized {
        /// Short address.
        pub nwk: u16,
        /// IEEE.
        pub ieee: Ieee,
        /// 0 key delivered, 1 TCLK verified, 2 failed.
        pub result: u8,
        /// Link-key type.
        pub link_key_type: u8,
    }
}

wire_struct! {
    /// `EVT_DEVICE_LEFT`.
    pub struct EvtDeviceLeft {
        /// Short address.
        pub nwk: u16,
        /// IEEE.
        pub ieee: Ieee,
        /// Bit0 rejoin, bit1 removed by TC, bit2 reported by parent.
        pub flags: u8,
        /// Reason.
        pub reason: u8,
    }
}

wire_struct! {
    /// `EVT_ADDRESS_CHANGED`.
    pub struct EvtAddressChanged {
        /// IEEE.
        pub ieee: Ieee,
        /// Old short address.
        pub old_nwk: u16,
        /// New short address.
        pub new_nwk: u16,
        /// 0 announce, 1 conflict.
        pub cause: u8,
    }
}

wire_struct! {
    /// `EVT_ROUTE_RECORD` (relays in Route Record order: entry 0 nearest
    /// the originator).
    pub struct EvtRouteRecord<'a> {
        /// Originator short address.
        pub src_nwk: u16,
        /// Originator IEEE (0 if unknown).
        pub src_ieee: Ieee,
        /// Relays.
        pub relays: List<'a, u16>,
    }
}

wire_struct! {
    /// `EVT_NWK_STATUS`.
    pub struct EvtNwkStatus {
        /// Destination.
        pub dst: u16,
        /// NWK status code.
        pub nwk_status: u8,
        /// Reporter.
        pub reporter: u16,
    }
}

wire_struct! {
    /// `EVT_PAN_ID_CONFLICT`.
    pub struct EvtPanIdConflict {
        /// Conflicting PAN.
        pub conflicting_pan: u16,
        /// Count.
        pub count: u8,
    }
}

wire_struct! {
    /// `EVT_PERMIT_JOIN_STATE`.
    pub struct EvtPermitJoinState {
        /// Remaining seconds.
        pub remaining_s: u8,
    }
}

wire_struct! {
    /// `EVT_CHANNEL_CHANGED`.
    pub struct EvtChannelChanged {
        /// Channel.
        pub channel: u8,
        /// Update ID.
        pub nwk_update_id: u8,
        /// Request TID.
        pub req_tid: u16,
    }
}

wire_struct! {
    /// One `{u8 channel, i8 max_rssi_dbm, u8 energy}` result.
    pub struct EnergyResult {
        /// Channel.
        pub channel: u8,
        /// Maximum RSSI.
        pub max_rssi_dbm: i8,
        /// Energy.
        pub energy: u8,
    }
}

wire_struct! {
    /// `EVT_SCAN_PROGRESS` (best effort, after each channel of a host scan).
    pub struct EvtScanProgress {
        /// TID of the `NWK_ENERGY_SCAN` / `NWK_ACTIVE_SCAN`.
        pub req_tid: u16,
        /// 0 energy, 1 active.
        pub scan: u8,
        /// The channel just finished.
        pub channel: u8,
        /// Channels done, across passes.
        pub channels_done: u8,
        /// Channels × passes.
        pub channels_total: u8,
        /// Beacons recorded so far (active scan; 0 for an energy scan).
        pub networks_found: u8,
    }
}

wire_struct! {
    /// `EVT_ENERGY_SCAN_RESULT`.
    pub struct EvtEnergyScanResult<'a> {
        /// Request TID.
        pub req_tid: u16,
        /// Status.
        pub status: u8,
        /// Results.
        pub results: List<'a, EnergyResult>,
    }
}

/// Frame Pending of the MAC ACK that answered a child's poll
/// (`EVT_CHILD_POLL.flags` bits 1–0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AckFramePending {
    /// `0`: the ACK carried Frame Pending = 0. The child turned its receiver
    /// off after it: nothing was waiting in the NCP for it at that instant.
    Clear,
    /// `1`: the ACK carried Frame Pending = 1. The NCP had a frame queued
    /// for the child, which stays awake to receive it.
    Set,
    /// `2` (and the reserved `3`): the NCP does not know the decision
    /// exactly (its radio backend acknowledges in hardware without an exact
    /// per-poll record).
    Unknown,
}

impl AckFramePending {
    /// Bits 1–0 of `EVT_CHILD_POLL.flags`; the reserved value 3 reads as
    /// [`Self::Unknown`].
    pub const fn from_flags(flags: u8) -> Self {
        match flags & child_poll::ACK_FRAME_PENDING_MASK {
            0 => Self::Clear,
            1 => Self::Set,
            _ => Self::Unknown,
        }
    }

    /// The two-bit wire value.
    pub const fn bits(self) -> u8 {
        match self {
            Self::Clear => 0,
            Self::Set => 1,
            Self::Unknown => 2,
        }
    }

    /// From the MAC's report (`None`: not reported).
    pub const fn from_report(frame_pending: Option<bool>) -> Self {
        match frame_pending {
            Some(false) => Self::Clear,
            Some(true) => Self::Set,
            None => Self::Unknown,
        }
    }

    /// The Frame Pending bit, when known.
    pub const fn known(self) -> Option<bool> {
        match self {
            Self::Clear => Some(false),
            Self::Set => Some(true),
            Self::Unknown => None,
        }
    }
}

/// `EVT_CHILD_POLL` values.
pub mod child_poll {
    /// `flags` bits 1–0: [`super::AckFramePending`]. Bits 7–2 are reserved
    /// (sent as 0, ignored by receivers).
    pub const ACK_FRAME_PENDING_MASK: u8 = 0x03;
    /// `child_poll_interval_ms` default.
    pub const DEFAULT_INTERVAL_MS: u16 = 2_000;
    /// Smallest accepted `child_poll_interval_ms`.
    pub const MIN_INTERVAL_MS: u16 = 250;
    /// Largest accepted `child_poll_interval_ms`.
    pub const MAX_INTERVAL_MS: u16 = 60_000;
}

wire_struct! {
    /// `EVT_CHILD_POLL`: a child of the NCP polled it (MAC Data Request),
    /// rate-limited per child and globally (spec 7.2).
    pub struct EvtChildPoll {
        /// The child's short address.
        pub nwk: u16,
        /// The child's IEEE address; all zero when the NCP does not know it.
        pub ieee: Ieee,
        /// Bits 1–0 [`AckFramePending`]; bits 7–2 reserved.
        pub flags: u8,
        /// This child's polls since its previous `EVT_CHILD_POLL` that were
        /// not reported (per-child interval, global cap, full queue),
        /// saturating; 0 for the first report the NCP remembers.
        pub unreported: u16,
    }
}

impl EvtChildPoll {
    /// The report of one poll (`ieee` `None`: unknown, sent as all zero).
    pub const fn new(
        nwk: u16,
        ieee: Option<Ieee>,
        ack_frame_pending: AckFramePending,
        unreported: u16,
    ) -> Self {
        Self {
            nwk,
            ieee: match ieee {
                Some(ieee) => ieee,
                None => Ieee([0; 8]),
            },
            flags: ack_frame_pending.bits(),
            unreported,
        }
    }

    /// Frame Pending of the ACK to this poll.
    pub const fn ack_frame_pending(&self) -> AckFramePending {
        AckFramePending::from_flags(self.flags)
    }

    /// The child's IEEE address, `None` when the NCP did not know it.
    pub fn known_ieee(&self) -> Option<Ieee> {
        (self.ieee != Ieee([0; 8])).then_some(self.ieee)
    }
}

message_def! {
    NwkInit = ids::NWK_INIT, Request;
    NwkInitRsp = ids::NWK_INIT, Response;
    NwkForm = ids::NWK_FORM, Request;
    NwkLeaveReset = ids::NWK_LEAVE_RESET, Request;
    NwkGetInfoRsp = ids::NWK_GET_INFO, Response;
    NwkPermitJoin = ids::NWK_PERMIT_JOIN, Request;
    NwkEnergyScan = ids::NWK_ENERGY_SCAN, Request;
    NwkChangeChannel = ids::NWK_CHANGE_CHANNEL, Request;
    NwkSetTxPower = ids::NWK_SET_TX_POWER, Request;
    NwkSetTxPowerRsp = ids::NWK_SET_TX_POWER, Response;
    NwkConcentrator = ids::NWK_CONCENTRATOR, Request;
    NwkActiveScan = ids::NWK_ACTIVE_SCAN, Request;
    NwkActiveScanRsp<'a> = ids::NWK_ACTIVE_SCAN, Response;
    NwkAddEndpoint<'a> = ids::NWK_ADD_ENDPOINT, Request;
    NwkSetManufacturerCode = ids::NWK_SET_MANUFACTURER_CODE, Request;
    NwkGroupMembership = ids::NWK_GROUP_MEMBERSHIP, Request;
    NwkRemoveDevice = ids::NWK_REMOVE_DEVICE, Request;
    EvtNetworkState = ids::EVT_NETWORK_STATE, Event;
    EvtDeviceJoined = ids::EVT_DEVICE_JOINED, Event;
    EvtDeviceAuthorized = ids::EVT_DEVICE_AUTHORIZED, Event;
    EvtDeviceLeft = ids::EVT_DEVICE_LEFT, Event;
    EvtAddressChanged = ids::EVT_ADDRESS_CHANGED, Event;
    EvtRouteRecord<'a> = ids::EVT_ROUTE_RECORD, Event;
    EvtNwkStatus = ids::EVT_NWK_STATUS, Event;
    EvtPanIdConflict = ids::EVT_PAN_ID_CONFLICT, Event;
    EvtPermitJoinState = ids::EVT_PERMIT_JOIN_STATE, Event;
    EvtChannelChanged = ids::EVT_CHANNEL_CHANGED, Event;
    EvtEnergyScanResult<'a> = ids::EVT_ENERGY_SCAN_RESULT, Event;
    EvtScanProgress = ids::EVT_SCAN_PROGRESS, Event;
    EvtChildPoll = ids::EVT_CHILD_POLL, Event;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::{Batch, BatchWriter, Kind};

    #[test]
    fn child_poll_round_trips_with_its_frame_pending_report() {
        let ieee = Ieee::from_u64(0x0015_8d00_04c5_d6e7);
        let event = EvtChildPoll::new(0x4F21, Some(ieee), AckFramePending::Set, 3);
        let mut buf = [0u8; 32];
        let mut w = BatchWriter::new(&mut buf);
        w.event(9, &event).unwrap();
        let wire = w.written();
        assert_eq!(
            wire,
            [
                0x80, 0x8C, 0x01, 0x09, 0x00, 0x0D, 0x00, // header: event 0x018C, seq 9
                0x21, 0x4F, 0xE7, 0xD6, 0xC5, 0x04, 0x00, 0x8D, 0x15, 0x00, 0x01, 0x03, 0x00,
            ]
        );
        let batch = Batch::parse(wire).unwrap();
        let message = batch.iter().next().unwrap();
        assert_eq!(
            (message.header.kind, message.header.msg_id),
            (Kind::Event, ids::EVT_CHILD_POLL)
        );
        let decoded: EvtChildPoll = message.decode().unwrap();
        assert_eq!(decoded, event);
        assert_eq!(decoded.ack_frame_pending(), AckFramePending::Set);
        assert_eq!(decoded.ack_frame_pending().known(), Some(true));
        assert_eq!(decoded.known_ieee(), Some(ieee));
    }

    #[test]
    fn child_poll_unknown_values_read_as_unknown() {
        let event = EvtChildPoll::new(0x1234, None, AckFramePending::from_report(None), 0);
        assert_eq!(event.flags, 2);
        assert_eq!(event.known_ieee(), None);
        // The reserved value 3 and reserved bits 7–2 (a later minor).
        for flags in [0x03, 0xFE, 0x82] {
            let event = EvtChildPoll {
                flags,
                ..event.clone()
            };
            assert_eq!(event.ack_frame_pending(), AckFramePending::Unknown);
        }
        let cleared = EvtChildPoll {
            flags: 0xFC,
            ..event
        };
        assert_eq!(cleared.ack_frame_pending().known(), Some(false));
    }
}
