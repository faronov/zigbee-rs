//! GREEN POWER group (`0x0Axx`, endpoint ZIGBEE), spec 7.11: NCP-sink
//! configuration, commissioning, Sink/Proxy Table access and sink events,
//! and remote GP provisioning (GP Pairing Configuration, GP Pairing, the
//! Translation Table and Sink/Proxy Table requests to other devices).

pub use crate::records::{
    GpSinkRecord, GpTranslation, GpTranslationRecord, GpdId, gp_translation_payload,
};

use crate::codec::{Bytes8, List, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::records::Record;
use crate::types::{Ieee, Key, Status};

/// `GP_SINK_CONFIG.mode`.
pub mod sink_mode {
    /// The host runs its own sink.
    pub const HOST_SINK: u8 = 0;
    /// The NCP sink is authoritative.
    pub const NCP_SINK: u8 = 1;
}

/// `EVT_GP_FRAME.origin`.
pub mod frame_origin {
    /// Direct GPDF.
    pub const DIRECT: u8 = 1;
    /// GP Notification from a proxy.
    pub const NOTIFICATION: u8 = 2;
    /// GP Commissioning Notification from a proxy.
    pub const COMMISSIONING_NOTIFICATION: u8 = 3;
}

/// `EVT_GP_FRAME.result`.
pub mod frame_result {
    /// Translated.
    pub const TRANSLATED: u8 = 0;
    /// Passed through.
    pub const PASSED_THROUGH: u8 = 1;
    /// Commissioned.
    pub const COMMISSIONED: u8 = 2;
    /// Commissioning Reply sent.
    pub const COMMISSIONING_REPLY_SENT: u8 = 3;
    /// Decommissioned.
    pub const DECOMMISSIONED: u8 = 4;
    /// Commissioning step accepted.
    pub const COMMISSIONING_STEP: u8 = 5;
}

wire_struct! {
    /// `GP_SINK_CONFIG` request.
    pub struct GpSinkConfig {
        /// [`sink_mode`].
        pub mode: u8,
        /// 0 full unicast, 1 derived groupcast, 2 pre-commissioned groupcast, 3 lightweight unicast.
        pub communication_mode: u8,
        /// Group for mode 2.
        pub group: u16,
        /// `gpsCommissioningExitMode` bits.
        pub exit_mode: u8,
        /// Commissioning window.
        pub window_s: u16,
        /// `gpsSecurityLevel`.
        pub security_level: u8,
        /// Groupcast radius.
        pub groupcast_radius: u8,
    }
}

wire_struct! {
    /// `GP_SET_SHARED_KEY` request.
    pub struct GpSetSharedKey {
        /// Key type.
        pub key_type: u8,
        /// Key.
        pub key: Key,
    }
}

wire_struct! {
    /// `GP_SINK_COMMISSIONING` request.
    pub struct GpSinkCommissioning {
        /// 1 enter, 0 exit.
        pub action: u8,
        /// Bit0 involve proxies, bit1 exit on first pairing.
        pub flags: u8,
        /// Window.
        pub window_s: u16,
    }
}

wire_struct! {
    /// `GP_SINK_TABLE_READ` request.
    pub struct GpSinkTableRead {
        /// First index.
        pub start: u8,
    }
}

wire_struct! {
    /// `GP_PROXY_TABLE_READ` request.
    pub struct GpProxyTableRead {
        /// First index.
        pub start: u8,
    }
}

wire_struct! {
    /// `GP_SINK_TABLE_READ` response body (`R_GP_SINK` TLV records; keys omitted when
    /// export is locked).
    pub struct GpSinkTableReadRsp<'a> {
        /// Total entries.
        pub total: u8,
        /// Records from `start`.
        pub records: List<'a, Record<'a>>,
    }
}

wire_struct! {
    /// `GP_PROXY_TABLE_READ` response body (`R_GP_PROXY` TLV records; keys omitted when
    /// export is locked).
    pub struct GpProxyTableReadRsp<'a> {
        /// Total entries.
        pub total: u8,
        /// Records from `start`.
        pub records: List<'a, Record<'a>>,
    }
}

wire_struct! {
    /// `GP_SINK_TABLE_PUT` request.
    pub struct GpSinkTablePut<'a> {
        /// Bit0 send GP Pairing AddSink.
        pub flags: u8,
        /// `R_GP_SINK` records.
        pub records: List<'a, Record<'a>>,
    }
}

wire_struct! {
    /// `GP_SINK_TABLE_PUT` response body.
    pub struct GpSinkTablePutRsp<'a> {
        /// Per-record status.
        pub statuses: List<'a, Status>,
    }
}

wire_struct! {
    /// `GP_SINK_REMOVE` request.
    pub struct GpSinkRemove {
        /// GPD.
        pub gpd_id: GpdId,
        /// Bit0 send GP Pairing RemoveGPD.
        pub flags: u8,
    }
}

wire_struct! {
    /// `EVT_GP_FRAME` (only frames the sink accepted).
    pub struct EvtGpFrame<'a> {
        /// [`frame_origin`].
        pub origin: u8,
        /// [`frame_result`].
        pub result: u8,
        /// GPD.
        pub gpd_id: GpdId,
        /// GPD device ID.
        pub device_id: u8,
        /// GPD command ID.
        pub command_id: u8,
        /// GPD frame counter.
        pub frame_counter: u32,
        /// Security level.
        pub security_level: u8,
        /// Key type.
        pub key_type: u8,
        /// Bit0 rxAfterTx, bit1 decrypted, bit2 key in payload TC-LK wrapped.
        pub flags: u8,
        /// Proxy (`FFFF` = direct).
        pub proxy_nwk: u16,
        /// GPP-GPD link.
        pub gpp_gpd_link: u8,
        /// RSSI.
        pub rssi: i8,
        /// LQI.
        pub lqi: u8,
        /// Command payload.
        pub payload: Bytes8<'a>,
    }
}

wire_struct! {
    /// `EVT_GP_COMMISSIONING`.
    pub struct EvtGpCommissioning {
        /// 1 entered, 0 left.
        pub state: u8,
        /// Window (`FFFF` none).
        pub window_s: u16,
        /// Proxies involved.
        pub involve_proxies: u8,
        /// 1 explicit, 2 first pairing, 3 timeout.
        pub exit_reason: u8,
    }
}

wire_struct! {
    /// `EVT_GP_COMMISSIONED`.
    pub struct EvtGpCommissioned<'a> {
        /// GPD.
        pub gpd_id: GpdId,
        /// GPD device ID.
        pub device_id: u8,
        /// Bit0 bidirectional, bit1 key present.
        pub flags: u8,
        /// `R_GP_SINK` record.
        pub record: Record<'a>,
    }
}

wire_struct! {
    /// `EVT_GP_COMMISSIONING_FAILED`.
    pub struct EvtGpCommissioningFailed {
        /// GPD.
        pub gpd_id: GpdId,
        /// `SinkDrop` reason (1 Invalid … 13 QueueFull).
        pub reason: u8,
    }
}

wire_struct! {
    /// `EVT_GP_DECOMMISSIONED`.
    pub struct EvtGpDecommissioned {
        /// GPD.
        pub gpd_id: GpdId,
    }
}

/// `GP_PAIRING_CONFIGURATION.flags`.
pub mod pairing_config_flags {
    /// GPD MAC sequence number capability.
    pub const MAC_SEQ_CAPABLE: u8 = 0x01;
    /// GPD RxOnCapability.
    pub const RX_ON_CAPABLE: u8 = 0x02;
    /// GPD fixed location.
    pub const FIXED_LOCATION: u8 = 0x04;
    /// `SecurityUse`: security level, key type, key and frame counter are
    /// sent.
    pub const SECURITY: u8 = 0x08;
    /// `assigned_alias` is sent.
    pub const ASSIGNED_ALIAS: u8 = 0x10;
}

/// `GP_PAIRING_CONFIGURATION.paired`: `NumberOfPairedEndpoints`.
pub mod paired_endpoints {
    /// No paired endpoint (0x00): the sink only reports the GPD's commands.
    pub const NONE: u8 = 0x00;
    /// The `endpoints` list.
    pub const LISTED: u8 = 0x01;
    /// The sink derives the endpoints (0xFE).
    pub const SINK_DERIVES: u8 = 0xFE;
    /// All matching sink endpoints (0xFF).
    pub const ALL: u8 = 0xFF;
}

/// `GP_SEND_PAIRING.flags`.
pub mod send_pairing_flags {
    /// AddSink.
    pub const ADD_SINK: u8 = 0x01;
    /// RemoveGPD.
    pub const REMOVE_GPD: u8 = 0x02;
    /// GPD fixed.
    pub const GPD_FIXED: u8 = 0x04;
    /// GPD MAC sequence number capability.
    pub const MAC_SEQ_CAPABLE: u8 = 0x08;
    /// `key` is sent.
    pub const KEY: u8 = 0x10;
    /// `assigned_alias` is sent.
    pub const ASSIGNED_ALIAS: u8 = 0x20;
    /// `frame_counter` is sent.
    pub const FRAME_COUNTER: u8 = 0x40;
    /// `groupcast_radius` is sent.
    pub const GROUPCAST_RADIUS: u8 = 0x80;
}

/// `EVT_GP_REMOTE_*_TABLE.flags`.
pub mod remote_table_flags {
    /// Entries the NCP could not represent (an LPED GPD ID, an additional
    /// information block, a malformed entry) were omitted; `records` holds
    /// the entries before the first of them.
    pub const OMITTED: u8 = 0x01;
}

wire_struct! {
    /// A commissioned group and its alias (`FFFF`: derived).
    pub struct GpGroup {
        /// Group.
        pub group: u16,
        /// Alias.
        pub alias: u16,
    }
}

wire_struct! {
    /// `GP_PAIRING_CONFIGURATION` request: GP Pairing Configuration to a
    /// sink.
    pub struct GpPairingConfiguration<'a> {
        /// NWK unicast address or a broadcast address (`FFFC`-`FFFF`).
        pub dst: u16,
        /// GP `Actions`: bits 0-2 the action (0 none, 1 extend, 2 replace,
        /// 3 remove pairing, 4 remove GPD), bit 3 the sink sends GP Pairing.
        pub actions: u8,
        /// GPD.
        pub gpd_id: GpdId,
        /// GPD DeviceID.
        pub device_id: u8,
        /// 0 full unicast, 1 derived groupcast, 2 pre-commissioned
        /// groupcast, 3 lightweight unicast.
        pub communication_mode: u8,
        /// [`pairing_config_flags`].
        pub flags: u8,
        /// Assigned alias (with [`pairing_config_flags::ASSIGNED_ALIAS`]).
        pub assigned_alias: u16,
        /// Groupcast radius.
        pub groupcast_radius: u8,
        /// Security level (with [`pairing_config_flags::SECURITY`]).
        pub security_level: u8,
        /// Key type (with [`pairing_config_flags::SECURITY`]).
        pub key_type: u8,
        /// GPD key (with [`pairing_config_flags::SECURITY`]).
        pub key: Key,
        /// GPD frame counter (with security or the MAC sequence number
        /// capability).
        pub frame_counter: u32,
        /// Pre-commissioned groups (communication mode 2).
        pub groups: List<'a, GpGroup>,
        /// [`paired_endpoints`].
        pub paired: u8,
        /// Paired endpoints ([`paired_endpoints::LISTED`]).
        pub endpoints: List<'a, u8>,
        /// GP `ApplicationInformation` bits 0-3 (manufacturer ID, model ID,
        /// GPD commands, clusters); 0: none sent.
        pub app_info: u8,
        /// Manufacturer ID.
        pub manufacturer_id: u16,
        /// Model ID.
        pub model_id: u16,
        /// GPD command IDs.
        pub gpd_commands: List<'a, u8>,
        /// GPD server clusters.
        pub server_clusters: List<'a, u16>,
        /// GPD client clusters.
        pub client_clusters: List<'a, u16>,
    }
}

wire_struct! {
    /// `GP_SEND_PAIRING` request: GP Pairing to the proxies (fields that
    /// `flags` and the communication mode do not select are ignored).
    pub struct GpSendPairing {
        /// Usually `FFFD`; a NWK address for one proxy.
        pub dst: u16,
        /// [`send_pairing_flags`].
        pub flags: u8,
        /// GPD.
        pub gpd_id: GpdId,
        /// 0 full unicast, 1 derived groupcast, 2 pre-commissioned
        /// groupcast, 3 lightweight unicast.
        pub communication_mode: u8,
        /// Security level.
        pub security_level: u8,
        /// Key type.
        pub key_type: u8,
        /// Sink IEEE (unicast modes).
        pub sink_ieee: Ieee,
        /// Sink NWK (unicast modes).
        pub sink_nwk: u16,
        /// Sink group (groupcast modes).
        pub sink_group: u16,
        /// GPD DeviceID (AddSink).
        pub device_id: u8,
        /// GPD frame counter.
        pub frame_counter: u32,
        /// GPD key.
        pub key: Key,
        /// Assigned alias.
        pub assigned_alias: u16,
        /// Groupcast radius.
        pub groupcast_radius: u8,
    }
}

wire_struct! {
    /// `GP_TRANSLATION_UPDATE` request: GP Translation Table Update to a
    /// sink.
    pub struct GpTranslationUpdate<'a> {
        /// NWK unicast address or a broadcast address.
        pub dst: u16,
        /// 0 add, 1 replace, 2 remove.
        pub action: u8,
        /// GPD.
        pub gpd_id: GpdId,
        /// At most 7 translations.
        pub translations: List<'a, GpTranslation<'a>>,
    }
}

wire_struct! {
    /// `GP_TRANSLATION_REQUEST` request (unicast).
    pub struct GpTranslationRequest {
        /// Sink.
        pub dst: u16,
        /// First entry.
        pub start_index: u8,
    }
}

wire_struct! {
    /// `GP_REMOTE_SINK_TABLE_REQUEST` request (unicast).
    pub struct GpRemoteSinkTableRequest {
        /// Sink.
        pub dst: u16,
        /// First entry; `FF`: the entry of `gpd_id`.
        pub index: u8,
        /// GPD (with `index` `FF`).
        pub gpd_id: GpdId,
    }
}

wire_struct! {
    /// `GP_REMOTE_PROXY_TABLE_REQUEST` request (unicast).
    pub struct GpRemoteProxyTableRequest {
        /// Proxy.
        pub dst: u16,
        /// First entry; `FF`: the entry of `gpd_id`.
        pub index: u8,
        /// GPD (with `index` `FF`).
        pub gpd_id: GpdId,
    }
}

wire_struct! {
    /// Response body of every remote GP request: the ZCL transaction
    /// sequence number the NCP sent, which the matching `EVT_GP_REMOTE_*`
    /// carries.
    pub struct GpRemoteRsp {
        /// ZCL TSN.
        pub zcl_tsn: u8,
    }
}

wire_struct! {
    /// `EVT_GP_REMOTE_STATUS`: the ZCL Default Response to a remote GP
    /// command.
    pub struct EvtGpRemoteStatus {
        /// Responding node.
        pub src_nwk: u16,
        /// TSN of the request.
        pub zcl_tsn: u8,
        /// GP command the status answers.
        pub command_id: u8,
        /// ZCL status.
        pub zcl_status: u8,
    }
}

wire_struct! {
    /// `EVT_GP_REMOTE_SINK_TABLE`: a GP Sink Table Response.
    pub struct EvtGpRemoteSinkTable<'a> {
        /// Sink.
        pub src_nwk: u16,
        /// TSN of the request.
        pub zcl_tsn: u8,
        /// ZCL status (SUCCESS, NOT_FOUND).
        pub status: u8,
        /// Entries in the sink's table.
        pub total: u8,
        /// First index (`FF` for a request by GPD ID).
        pub start_index: u8,
        /// [`remote_table_flags`].
        pub flags: u8,
        /// `R_GP_SINK` records (keys as received).
        pub records: List<'a, Record<'a>>,
    }
}

wire_struct! {
    /// `EVT_GP_REMOTE_PROXY_TABLE`: a GP Proxy Table Response.
    pub struct EvtGpRemoteProxyTable<'a> {
        /// Proxy.
        pub src_nwk: u16,
        /// TSN of the request.
        pub zcl_tsn: u8,
        /// ZCL status (SUCCESS, NOT_FOUND).
        pub status: u8,
        /// Entries in the proxy's table.
        pub total: u8,
        /// First index (`FF` for a request by GPD ID).
        pub start_index: u8,
        /// [`remote_table_flags`].
        pub flags: u8,
        /// `R_GP_PROXY` records, one entry each (`format_version` 1).
        pub records: List<'a, Record<'a>>,
    }
}

wire_struct! {
    /// `EVT_GP_REMOTE_TRANSLATION_TABLE`: a GP Translation Table Response.
    pub struct EvtGpRemoteTranslationTable<'a> {
        /// Sink.
        pub src_nwk: u16,
        /// TSN of the request.
        pub zcl_tsn: u8,
        /// ZCL status (SUCCESS, NOT_FOUND, INSUFFICIENT_SPACE).
        pub status: u8,
        /// Entries in the sink's table.
        pub total: u8,
        /// First index.
        pub start_index: u8,
        /// [`remote_table_flags`].
        pub flags: u8,
        /// `R_GP_TRANSLATION` records.
        pub records: List<'a, Record<'a>>,
    }
}

message_def! {
    GpPairingConfiguration<'a> = ids::GP_PAIRING_CONFIGURATION, Request;
    GpSendPairing = ids::GP_SEND_PAIRING, Request;
    GpTranslationUpdate<'a> = ids::GP_TRANSLATION_UPDATE, Request;
    GpTranslationRequest = ids::GP_TRANSLATION_REQUEST, Request;
    GpRemoteSinkTableRequest = ids::GP_REMOTE_SINK_TABLE_REQUEST, Request;
    GpRemoteProxyTableRequest = ids::GP_REMOTE_PROXY_TABLE_REQUEST, Request;
    EvtGpRemoteStatus = ids::EVT_GP_REMOTE_STATUS, Event;
    EvtGpRemoteSinkTable<'a> = ids::EVT_GP_REMOTE_SINK_TABLE, Event;
    EvtGpRemoteProxyTable<'a> = ids::EVT_GP_REMOTE_PROXY_TABLE, Event;
    EvtGpRemoteTranslationTable<'a> = ids::EVT_GP_REMOTE_TRANSLATION_TABLE, Event;
}

message_def! {
    GpSinkConfig = ids::GP_SINK_CONFIG, Request;
    GpSetSharedKey = ids::GP_SET_SHARED_KEY, Request;
    GpSinkCommissioning = ids::GP_SINK_COMMISSIONING, Request;
    GpSinkTableRead = ids::GP_SINK_TABLE_READ, Request;
    GpProxyTableRead = ids::GP_PROXY_TABLE_READ, Request;
    GpSinkTableReadRsp<'a> = ids::GP_SINK_TABLE_READ, Response;
    GpProxyTableReadRsp<'a> = ids::GP_PROXY_TABLE_READ, Response;
    GpSinkTablePut<'a> = ids::GP_SINK_TABLE_PUT, Request;
    GpSinkTablePutRsp<'a> = ids::GP_SINK_TABLE_PUT, Response;
    GpSinkRemove = ids::GP_SINK_REMOVE, Request;
    EvtGpFrame<'a> = ids::EVT_GP_FRAME, Event;
    EvtGpCommissioning = ids::EVT_GP_COMMISSIONING, Event;
    EvtGpCommissioned<'a> = ids::EVT_GP_COMMISSIONED, Event;
    EvtGpCommissioningFailed = ids::EVT_GP_COMMISSIONING_FAILED, Event;
    EvtGpDecommissioned = ids::EVT_GP_DECOMMISSIONED, Event;
}
