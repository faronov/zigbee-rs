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
        /// Bit0 keep IEEE override, bit1 keep host endpoints.
        pub flags: u8,
    }
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
}
