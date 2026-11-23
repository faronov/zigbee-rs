//! SYSTEM group (`0x00xx`, endpoint SYSTEM), spec 7.1.
//!
//! The layouts of `SYS_NEGOTIATE` and `EVT_NCP_READY` are frozen across all
//! majors (6.6).

use crate::codec::{Bytes8, List, Rest, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::types::{Ieee, Status};

/// NCP capability bits (`ncp_caps`, 6.6).
pub mod caps {
    /// APS request may carry relays.
    pub const HOST_SOURCE_ROUTES: u64 = 1 << 0;
    /// MTORR origination.
    pub const CONCENTRATOR: u64 = 1 << 1;
    /// APS fragmentation.
    pub const APS_FRAGMENTATION: u64 = 1 << 2;
    /// Install codes.
    pub const INSTALL_CODES: u64 = 1 << 3;
    /// Network/link keys readable.
    pub const KEY_EXPORT: u64 = 1 << 4;
    /// Repeatable IEEE override.
    pub const IEEE_REWRITABLE: u64 = 1 << 5;
    /// Host-backed tables (section 8).
    pub const HOST_BACKED_TABLES: u64 = 1 << 6;
    /// NCP firmware update.
    pub const NCP_FW_UPDATE: u64 = 1 << 7;
    /// GP proxy.
    pub const GREEN_POWER_PROXY: u64 = 1 << 8;
    /// Energy scan.
    pub const ENERGY_SCAN: u64 = 1 << 9;
    /// Network key rotation.
    pub const NETWORK_KEY_ROTATION: u64 = 1 << 10;
    /// Packet capture (reserved).
    pub const PACKET_CAPTURE: u64 = 1 << 12;
    /// Authenticated link (reserved, section 13).
    pub const LINK_AUTH: u64 = 1 << 13;
    /// EZSP hashed TCLK.
    pub const EZSP_HASHED_TCLK: u64 = 1 << 14;
    /// Backup/restore.
    pub const BACKUP_RESTORE: u64 = 1 << 15;
    /// Manufacturer code.
    pub const MANUFACTURER_CODE: u64 = 1 << 16;
    /// Group membership.
    pub const GROUP_MEMBERSHIP: u64 = 1 << 17;
    /// TX power.
    pub const TX_POWER: u64 = 1 << 18;
    /// Combo GP sink.
    pub const GREEN_POWER_SINK: u64 = 1 << 20;
    /// PTA coexistence.
    pub const PTA_COEX: u64 = 1 << 21;
    /// RTS/CTS active on this link.
    pub const UART_FLOW_CONTROL: u64 = 1 << 22;
    /// `NWK_ACTIVE_SCAN`.
    pub const ACTIVE_SCAN: u64 = 1 << 23;
    /// The APS group table is readable (`TBL_READ` / `TBL_STATS` of table
    /// `APS_GROUP` `0x0D`) and backed up (`BKP_EXPORT_BEGIN` flags bit3,
    /// `APS_GROUP` records restored).
    pub const APS_GROUP_TABLE: u64 = 1 << 24;
}

/// Host capability bits (`host_caps`, 8.8).
pub mod host_caps {
    /// The host serves the TABLES protocol.
    pub const HOST_TABLES: u64 = 1 << 0;
    /// The host keeps the FC checkpoint log.
    pub const FC_CHECKPOINTS: u64 = 1 << 1;
    /// The host uses the retry cache (6.5).
    pub const RETRY_CACHE: u64 = 1 << 2;
    /// The host mirrors NCP-sink GPDs (7.11).
    pub const GP_SHADOW: u64 = 1 << 3;
}

/// Capacity IDs (6.6).
pub mod capacity {
    /// `max_aps_inflight`
    pub const MAX_APS_INFLIGHT: u8 = 1;
    /// `max_children`
    pub const MAX_CHILDREN: u8 = 2;
    /// `link_key_cache`
    pub const LINK_KEY_CACHE: u8 = 3;
    /// `address_map_cache`
    pub const ADDRESS_MAP_CACHE: u8 = 4;
    /// `source_route_cache`
    pub const SOURCE_ROUTE_CACHE: u8 = 5;
    /// `max_asdu`
    pub const MAX_ASDU: u8 = 6;
    /// `fc_stride`
    pub const FC_STRIDE: u8 = 7;
    /// `max_host_endpoints`
    pub const MAX_HOST_ENDPOINTS: u8 = 8;
    /// `group_table`
    pub const GROUP_TABLE: u8 = 9;
    /// `binding_table`
    pub const BINDING_TABLE: u8 = 10;
    /// `neighbor_table`
    pub const NEIGHBOR_TABLE: u8 = 11;
    /// `route_table`
    pub const ROUTE_TABLE: u8 = 12;
    /// `max_restore_records_per_put`
    pub const MAX_RESTORE_RECORDS_PER_PUT: u8 = 13;
    /// `max_source_route_relays`
    pub const MAX_SOURCE_ROUTE_RELAYS: u8 = 14;
    /// `gp_sink_table`
    pub const GP_SINK_TABLE: u8 = 15;
    /// `gp_proxy_table`
    pub const GP_PROXY_TABLE: u8 = 16;
    /// `max_ncp_requests`
    pub const MAX_NCP_REQUESTS: u8 = 17;
    /// `max_tc_devices`
    pub const MAX_TC_DEVICES: u8 = 18;
    /// `host_response_bytes`: response bytes a host may keep outstanding
    /// per endpoint without `BUSY` for queue space (6.2).
    pub const HOST_RESPONSE_BYTES: u8 = 19;
}

/// Config IDs for `SYS_SET_CONFIG` / `SYS_GET_CONFIG` (7.1).
pub mod config {
    /// `host_absent_queue_len`
    pub const HOST_ABSENT_QUEUE_LEN: u16 = 0x0001;
    /// `tbl_lookup_timeout_ms`
    pub const TBL_LOOKUP_TIMEOUT_MS: u16 = 0x0002;
    /// `tbl_miss_queue_len`
    pub const TBL_MISS_QUEUE_LEN: u16 = 0x0003;
    /// `zdo_forward_handled`
    pub const ZDO_FORWARD_HANDLED: u16 = 0x0004;
    /// `aps_ack_timeout_ms`
    pub const APS_ACK_TIMEOUT_MS: u16 = 0x0005;
    /// `indirect_timeout_ms`
    pub const INDIRECT_TIMEOUT_MS: u16 = 0x0006;
    /// `log_level`
    pub const LOG_LEVEL: u16 = 0x0008;
    /// `coex_enable`
    pub const COEX_ENABLE: u16 = 0x0009;
    /// `coex_tx_priority_mask`
    pub const COEX_TX_PRIORITY_MASK: u16 = 0x000A;
}

/// `SYS_RESET` confirm word for factory reset ("FRES").
pub const FACTORY_RESET_CONFIRM: u32 = 0x4652_4553;

wire_struct! {
    /// `SYS_NEGOTIATE` request.
    pub struct SysNegotiateReq<'a> {
        /// Lowest major supported.
        pub major_min: u8,
        /// Highest major supported.
        pub major_max: u8,
        /// [`host_caps`] bits.
        pub host_caps: u64,
        /// Host name (≤ 32 B UTF-8).
        pub name: Bytes8<'a>,
    }
}

wire_struct! {
    /// One `{u8 id, u32 value}` capacity entry.
    pub struct Capacity {
        /// [`capacity`] ID.
        pub id: u8,
        /// Value.
        pub value: u32,
    }
}

wire_struct! {
    /// `SYS_NEGOTIATE` response body.
    pub struct SysNegotiateRsp<'a> {
        /// Selected major.
        pub major: u8,
        /// NCP minor.
        pub minor: u8,
        /// [`caps`] bits.
        pub ncp_caps: u64,
        /// Capacities.
        pub capacities: List<'a, Capacity>,
        /// Firmware version string (≤ 32 B).
        pub fw: Bytes8<'a>,
        /// Build ID.
        pub build_id: [u8; 8],
        /// Chip (1 ESP32-H2, 2 ESP32-C6, …).
        pub chip: u8,
    }
}

impl SysNegotiateRsp<'_> {
    /// Look up a capacity value.
    pub fn capacity(&self, id: u8) -> Option<u32> {
        self.capacities.iter().find(|c| c.id == id).map(|c| c.value)
    }
}

wire_struct! {
    /// `SYS_GET_INFO` response body (request has no payload).
    pub struct SysGetInfoRsp<'a> {
        /// Factory IEEE.
        pub ieee_factory: Ieee,
        /// IEEE in use.
        pub ieee_active: Ieee,
        /// 0 factory, 1 override.
        pub ieee_source: u8,
        /// Chip revision.
        pub chip_rev: u16,
        /// Flash size in bytes.
        pub flash_size: u32,
        /// 1 USJ, 2 UART.
        pub transport: u8,
        /// UART baud (0 for USJ).
        pub uart_baud: u32,
        /// 0 none, 1 RTS/CTS.
        pub uart_flow: u8,
        /// Board name.
        pub board: Bytes8<'a>,
        /// Product name.
        pub product: Bytes8<'a>,
    }
}

wire_struct! {
    /// `SYS_RESET` request.
    pub struct SysReset {
        /// 0 reboot, 1 bootloader, 2 factory reset.
        pub mode: u8,
        /// [`FACTORY_RESET_CONFIRM`] for mode 2.
        pub confirm: u32,
    }
}

wire_struct! {
    /// `SYS_GET_STATUS` response body.
    pub struct SysGetStatusRsp {
        /// Network state.
        pub network_state: u8,
        /// Uptime in seconds.
        pub uptime_s: u32,
        /// Free APS slots.
        pub free_aps_slots: u16,
        /// Host-absent queue depth.
        pub host_absent_queue: u16,
        /// Dirty writebacks.
        pub dirty_writebacks: u16,
        /// Restore in progress.
        pub restore_in_progress: u8,
        /// Bit0 key export locked, bit1 paging, bit2 PTA, bit3 GP commissioning, bit4 permit join.
        pub flags: u8,
    }
}

wire_struct! {
    /// `{u16 id, u8 len, value[]}` config value.
    pub struct ConfigValue<'a> {
        /// [`config`] ID.
        pub id: u16,
        /// Value bytes.
        pub value: Bytes8<'a>,
    }
}

wire_struct! {
    /// `SYS_SET_CONFIG` request.
    pub struct SysSetConfig<'a> {
        /// Values to set.
        pub values: List<'a, ConfigValue<'a>>,
    }
}

wire_struct! {
    /// `{u16 id, u8 status}`.
    pub struct ConfigStatus {
        /// Config ID.
        pub id: u16,
        /// Per-item status.
        pub status: Status,
    }
}

wire_struct! {
    /// `SYS_SET_CONFIG` response body.
    pub struct SysSetConfigRsp<'a> {
        /// Per-item results.
        pub results: List<'a, ConfigStatus>,
    }
}

wire_struct! {
    /// `SYS_GET_CONFIG` request.
    pub struct SysGetConfig<'a> {
        /// IDs to read.
        pub ids: List<'a, u16>,
    }
}

wire_struct! {
    /// `{u16 id, u8 status, u8 len, value[]}`.
    pub struct ConfigReadItem<'a> {
        /// Config ID.
        pub id: u16,
        /// Per-item status.
        pub status: Status,
        /// Value bytes.
        pub value: Bytes8<'a>,
    }
}

wire_struct! {
    /// `SYS_GET_CONFIG` response body.
    pub struct SysGetConfigRsp<'a> {
        /// Items.
        pub items: List<'a, ConfigReadItem<'a>>,
    }
}

wire_struct! {
    /// `SYS_ECHO` request and response body (≤ 64 B).
    pub struct SysEcho<'a> {
        /// Echoed data.
        pub data: Rest<'a>,
    }
}

wire_struct! {
    /// `SYS_SET_IEEE` request (all `FF` = revert to factory).
    pub struct SysSetIeee {
        /// New IEEE.
        pub ieee: Ieee,
    }
}

wire_struct! {
    /// `SYS_SET_UART` request.
    pub struct SysSetUart {
        /// New baud rate.
        pub baud: u32,
        /// 0 none, 1 RTS/CTS.
        pub flow: u8,
        /// Delay before both sides switch.
        pub apply_delay_ms: u16,
    }
}

wire_struct! {
    /// `EVT_NCP_READY` (layout frozen across majors).
    pub struct EvtNcpReady {
        /// NCP boot cookie.
        pub ncp_cookie: u32,
        /// Reset reason (0 unknown, 1 power-on, …).
        pub reset_reason: u8,
        /// 0 none, 1 formed-down, 2 up.
        pub network_state: u8,
        /// NCP table epoch.
        pub table_epoch: u32,
        /// Durable NWK frame-counter limit.
        pub nwk_fc_limit: u32,
        /// Durable APS frame-counter limit.
        pub aps_fc_limit: u32,
        /// Dirty writebacks waiting.
        pub dirty_writebacks: u16,
        /// Indications dropped while the host was absent.
        pub dropped_indications: u32,
    }
}

wire_struct! {
    /// `EVT_INDICATIONS_DROPPED`.
    pub struct EvtIndicationsDropped {
        /// Link endpoint.
        pub endpoint: u8,
        /// 1 host absent, 2 no credit, 3 queue full.
        pub reason: u8,
        /// Dropped count.
        pub count: u32,
        /// First drop time.
        pub first_ms: u32,
        /// Last drop time.
        pub last_ms: u32,
    }
}

wire_struct! {
    /// `EVT_FATAL`.
    pub struct EvtFatal<'a> {
        /// Kind.
        pub kind: u8,
        /// Code.
        pub code: u32,
        /// Text.
        pub text: Bytes8<'a>,
    }
}

wire_struct! {
    /// `EVT_PROTOCOL_ERROR`.
    pub struct EvtProtocolError {
        /// Link endpoint.
        pub endpoint: u8,
        /// Offending message ID.
        pub msg_id: u16,
        /// Reason.
        pub reason: u8,
    }
}

message_def! {
    SysNegotiateReq<'a> = ids::SYS_NEGOTIATE, Request;
    SysNegotiateRsp<'a> = ids::SYS_NEGOTIATE, Response;
    SysGetInfoRsp<'a> = ids::SYS_GET_INFO, Response;
    SysReset = ids::SYS_RESET, Request;
    SysGetStatusRsp = ids::SYS_GET_STATUS, Response;
    SysSetConfig<'a> = ids::SYS_SET_CONFIG, Request;
    SysSetConfigRsp<'a> = ids::SYS_SET_CONFIG, Response;
    SysGetConfig<'a> = ids::SYS_GET_CONFIG, Request;
    SysGetConfigRsp<'a> = ids::SYS_GET_CONFIG, Response;
    SysEcho<'a> = ids::SYS_ECHO, Request;
    SysSetIeee = ids::SYS_SET_IEEE, Request;
    SysSetUart = ids::SYS_SET_UART, Request;
    EvtNcpReady = ids::EVT_NCP_READY, Event;
    EvtIndicationsDropped = ids::EVT_INDICATIONS_DROPPED, Event;
    EvtFatal<'a> = ids::EVT_FATAL, Event;
    EvtProtocolError = ids::EVT_PROTOCOL_ERROR, Event;
}
