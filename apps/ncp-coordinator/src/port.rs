//! The coordinator as the NCP dispatcher sees it.
//!
//! [`CoordinatorPort`] keeps the ZRHP dispatcher ([`crate::Ncp`])
//! independent of the concrete stack composition: the dispatcher answers
//! what it can from snapshots and turns every operation that awaits the
//! stack (formation, resume, transmissions) into an [`Op`] that the
//! platform loop executes with [`crate::Ncp::drive`]. The production
//! implementation over `router_app::TrustCenterCoordinatorApp` is
//! [`crate::tc::TcPort`]; host tests use a model.

use heapless::Vec;

/// Largest unfragmented ASDU the NCP accepts from the host (`max_asdu`).
pub const MAX_ASDU: usize = 82;
/// Largest ASDU forwarded to the host in `EVT_APS_DATA_INDICATION`.
pub const MAX_INDICATION_ASDU: usize = 128;

/// `EVT_NCP_READY.network_state` / `NWK_INIT` response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NetworkState {
    /// No network in NV.
    None = 0,
    /// A network is in NV but not running (waiting for `NWK_INIT`).
    FormedDown = 1,
    /// Running.
    Up = 2,
}

/// `NWK_GET_INFO` fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetworkInfo {
    pub state: NetworkState,
    pub pan_id: u16,
    pub ext_pan_id: [u8; 8],
    pub channel: u8,
    pub channel_mask: u32,
    pub short: u16,
    pub ieee: [u8; 8],
    pub nwk_update_id: u8,
    pub nwk_manager: u16,
    pub key_seq: u8,
    pub tx_power: i8,
}

/// Outgoing frame counters (`SEC_GET_FRAME_COUNTERS`, `EVT_FC_CHECKPOINT`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FrameCounters {
    pub nwk_current: u32,
    /// Durable NWK limit.
    pub nwk_limit: u32,
    pub aps_current: u32,
    /// Durable global APS limit.
    pub aps_limit: u32,
    /// Global APS reservation commits (checkpoint generation).
    pub aps_commits: u32,
}

/// Destination of an outgoing APS frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Destination {
    Group(u16),
    Nwk(u16),
    Ieee([u8; 8]),
    Broadcast(u16),
}

/// A host `APS_DATA_REQUEST` accepted into the APS queue.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataRequest {
    pub req_tid: u16,
    pub dst: Destination,
    pub dst_ep: u8,
    pub src_ep: u8,
    pub profile: u16,
    pub cluster: u16,
    /// ZRHP `tx_options` bits (spec 7.3).
    pub tx_options: u8,
    pub radius: u8,
    pub asdu: Vec<u8, MAX_ASDU>,
}

/// Outcome of a transmission (`EVT_APS_DATA_CONFIRM`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataConfirm {
    /// Raw Zigbee status (0 = success).
    pub zb_status: u8,
    pub aps_counter: u8,
    pub dst_nwk: u16,
    /// bit0 APS ACK received.
    pub flags: u8,
}

/// A received APS data frame for the host.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Indication {
    /// `0x01` group, `0x02` unicast, `0x0F` broadcast.
    pub dst_mode: u8,
    pub dst: u16,
    pub dst_ep: u8,
    pub src_nwk: u16,
    pub src_ieee: Option<[u8; 8]>,
    pub src_ep: u8,
    pub profile: u16,
    pub cluster: u16,
    pub aps_secured: bool,
    pub nwk_secured: bool,
    /// Link key of an APS-secured frame ([`key_source`]).
    pub key_source: u8,
    /// ZDO request already answered by the NCP.
    pub handled_by_ncp: bool,
    pub lqi: u8,
    /// Last hop RSSI in dBm; -128 when the radio does not measure it.
    pub rssi: i8,
    pub last_hop_nwk: u16,
    pub aps_counter: u8,
    pub rx_time_ms: u32,
    pub asdu: Vec<u8, MAX_INDICATION_ASDU>,
}

/// A device event for the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceEvent {
    /// `EVT_DEVICE_JOINED`: a Trust Center join decision.
    Joined {
        nwk: u16,
        ieee: [u8; 8],
        /// `0xFFFF` when the parent's short address is unknown.
        parent_nwk: u16,
        /// [`joined`]`::JOIN_*`.
        join_type: u8,
        /// MAC capability of a child of the NCP; 0 when the device joined
        /// through another parent (its `Device_annce` carries it).
        capability: u8,
        /// [`joined`]`::ADMITTED` or [`joined`]`::DENIED`.
        decision: u8,
    },
    /// `EVT_DEVICE_LEFT`.
    Left {
        nwk: u16,
        /// All zero when unknown.
        ieee: [u8; 8],
        /// [`left`]`::FLAG_*`.
        flags: u8,
        /// [`left`]`::REASON_*`.
        reason: u8,
    },
}

/// Formation parameters from `NWK_FORM` (zero/`FFFF` = NCP chooses).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FormParams {
    pub channel_mask: u32,
    pub pan_id: u16,
    pub ext_pan_id: [u8; 8],
    pub network_key: [u8; 16],
    pub key_seq: u8,
    pub nwk_update_id: u8,
    pub tx_power: i8,
    pub nwk_manager: u16,
}

impl FormParams {
    /// Whether every identity field is left to the NCP (the only form the
    /// runtime supports today; see [`OpError::Unsupported`]).
    pub fn ncp_chosen_identity(&self) -> bool {
        self.pan_id == 0xFFFF
            && self.ext_pan_id == [0; 8]
            && self.network_key == [0; 16]
            && self.key_seq == 0
            && self.nwk_update_id == 0
            && self.nwk_manager == 0
    }
}

/// An operation the platform loop runs with [`crate::Ncp::drive`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Op {
    Init {
        tid: u16,
        nwk_fc_floor: u32,
        aps_fc_floor: u32,
    },
    Form {
        tid: u16,
        params: FormParams,
    },
    PermitJoin {
        duration_s: u8,
        target: u16,
    },
    Data(DataRequest),
    /// `flags`: `NWK_LEAVE_RESET` flags (bit1 keep host endpoints).
    LeaveReset {
        flags: u8,
    },
}

/// Why an operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpError {
    /// Not allowed now (`INVALID_STATE`).
    InvalidState,
    /// No network in NV (`NOT_FORMED`).
    NotFormed,
    /// Persistence failed (`STORAGE_ERROR`).
    Storage,
    /// Not implemented by this NCP build (`UNSUPPORTED_COMMAND`).
    Unsupported,
    /// Zigbee-level failure with its raw status.
    Zigbee(u8),
}

/// The coordinator stack behind the NCP.
#[allow(async_fn_in_trait)]
pub trait CoordinatorPort {
    fn ieee(&self) -> [u8; 8];
    fn state(&self) -> NetworkState;
    fn info(&self) -> NetworkInfo;
    fn frame_counters(&self) -> FrameCounters;
    /// Active (`which = 0`) or alternate network key and its sequence.
    fn network_key(&self, which: u8) -> Option<([u8; 16], u8)>;
    fn permit_join_remaining_s(&self) -> u8;
    /// Next received frame for the host, if any.
    fn take_indication(&mut self) -> Option<Indication>;
    /// Next device event, if any.
    fn take_device_event(&mut self) -> Option<DeviceEvent>;

    /// Resume the network from NV, then raise the outgoing counters to the
    /// host floors before the first transmission.
    async fn init(&mut self, nwk_fc_floor: u32, aps_fc_floor: u32) -> Result<(), OpError>;
    /// Form a new network.
    async fn form(&mut self, params: FormParams) -> Result<(), OpError>;
    async fn permit_join(&mut self, duration_s: u8, target: u16) -> Result<(), OpError>;
    async fn send(&mut self, request: &DataRequest) -> Result<DataConfirm, OpError>;
    /// Leave and wipe the network state (`NWK_LEAVE_RESET`).
    async fn leave_reset(&mut self) -> Result<(), OpError>;

    /// The Green Power combo (spec 7.11), `None` when this stack runs none
    /// (the GREEN POWER commands are then `UNSUPPORTED_COMMAND`).
    fn gp(&mut self) -> Option<&mut dyn crate::gp::GpPort> {
        None
    }
    /// The final confirm of a [`Self::send`] whose confirm was
    /// [`DataConfirm::deferred`] (APS ACK received, or not), if one is ready.
    fn take_data_confirm(&mut self) -> Option<DeferredConfirm> {
        None
    }
    /// `max_host_endpoints`; 0 when [`Self::add_endpoint`] is unsupported.
    fn max_host_endpoints(&self) -> u8 {
        0
    }
    /// Register (or replace) a host endpoint (`NWK_ADD_ENDPOINT`): the NCP's
    /// ZDO server answers `Active_EP`, `Simple_Desc` and `Match_Desc` with it.
    fn add_endpoint(&mut self, endpoint: &HostEndpoint) -> Result<(), EndpointError> {
        let _ = endpoint;
        Err(EndpointError::Unsupported)
    }
    /// Forget every host endpoint (`NWK_LEAVE_RESET` without flags bit1).
    fn clear_endpoints(&mut self) {}
}

/// `EVT_DEVICE_JOINED` values.
pub mod joined {
    /// MAC association (Update-Device "standard device unsecured join").
    pub const JOIN_ASSOCIATION: u8 = 0;
    /// NWK rejoin secured with the network key.
    pub const JOIN_SECURED_REJOIN: u8 = 1;
    /// Unsecured (Trust Center) rejoin.
    pub const JOIN_UNSECURED_REJOIN: u8 = 2;
    /// Admitted by the Trust Center.
    pub const ADMITTED: u8 = 0;
    /// Rejected or removed by Trust Center policy.
    pub const DENIED: u8 = 1;
}

/// `EVT_DEVICE_LEFT` values.
pub mod left {
    /// The device announced a Leave with the rejoin bit set.
    pub const FLAG_REJOIN: u8 = 0x01;
    /// The Trust Center removed it.
    pub const FLAG_REMOVED_BY_TC: u8 = 0x02;
    /// Its parent router reported the departure (Update-Device).
    pub const FLAG_REPORTED_BY_PARENT: u8 = 0x04;
    /// It announced its departure with a NWK Leave heard by the NCP.
    pub const REASON_LEAVE: u8 = 1;
    /// Its parent router reported it with Update-Device "device left".
    pub const REASON_PARENT_REPORT: u8 = 2;
    /// Trust Center removal: a rejected rejoin, an expired join timeout or
    /// a failed key exchange.
    pub const REASON_TC_REMOVAL: u8 = 3;
    /// A child of the NCP aged out: an end device missed its End Device
    /// Timeout, or a router child stopped sending Link Status.
    pub const REASON_CHILD_TIMEOUT: u8 = 4;
}

/// `EVT_APS_DATA_INDICATION.key_source` values.
pub mod key_source {
    pub const NONE: u8 = 0;
    pub const GLOBAL_TRUST_CENTER: u8 = 1;
    pub const UNIQUE_TRUST_CENTER: u8 = 2;
    pub const APPLICATION: u8 = 3;
}

impl DataConfirm {
    /// `flags` bit0: the APS ACK was received.
    pub const FLAG_ACK_RECEIVED: u8 = 0x01;
    /// Port-internal `flags` marker, never sent to the host: the frame went
    /// out with `ack_request` and its final confirm follows from
    /// [`CoordinatorPort::take_data_confirm`].
    pub const DEFERRED: u8 = 0x80;

    /// Whether the final confirm follows from
    /// [`CoordinatorPort::take_data_confirm`].
    pub const fn deferred(&self) -> bool {
        self.flags & Self::DEFERRED != 0
    }
}

/// The final `EVT_APS_DATA_CONFIRM` of an acknowledged send.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeferredConfirm {
    pub req_tid: u16,
    /// `zb_status` 0 with [`DataConfirm::FLAG_ACK_RECEIVED`] once the APS ACK
    /// arrived; `0xA7` (APS `NO_ACK`) when the retries were exhausted, ACK
    /// tracking was lost, or the network was reset.
    pub confirm: DataConfirm,
    /// From the transmission to the outcome.
    pub elapsed_ms: u16,
}

/// Most host endpoints the NCP registers (`max_host_endpoints`).
pub const MAX_HOST_ENDPOINTS: usize = 8;
/// Most input (or output) clusters of one host endpoint.
pub const MAX_ENDPOINT_CLUSTERS: usize = 16;

/// A host endpoint (`NWK_ADD_ENDPOINT`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HostEndpoint {
    /// 1..=240.
    pub endpoint: u8,
    pub profile: u16,
    pub device_id: u16,
    pub version: u8,
    pub in_clusters: Vec<u16, MAX_ENDPOINT_CLUSTERS>,
    pub out_clusters: Vec<u16, MAX_ENDPOINT_CLUSTERS>,
}

/// Why `NWK_ADD_ENDPOINT` failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EndpointError {
    /// Endpoint outside 1..=240, or too many clusters (`INVALID_PARAMETER`).
    InvalidParameter,
    /// `max_host_endpoints` reached, or the ZDO registry is full
    /// (`NO_SPACE`).
    NoSpace,
    /// Not supported by this port (`UNSUPPORTED_COMMAND`).
    Unsupported,
}
