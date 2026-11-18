//! GREEN POWER group (`0x0Axx`, endpoint ZIGBEE), spec 7.11: NCP-sink
//! configuration, commissioning, Sink/Proxy Table access and sink events.

pub use crate::records::{GpSinkRecord, GpdId};

use crate::codec::{Bytes8, List, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::records::Record;
use crate::types::{Key, Status};

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
