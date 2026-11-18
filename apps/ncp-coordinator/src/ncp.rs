//! The ZRHP v1 NCP: link pump and command dispatch.
//!
//! [`Ncp::service`] is called from the platform loop between bounded stack
//! steps. It never blocks: it moves bytes between the host transport and
//! the link, dispatches complete requests, answers what it can from stack
//! snapshots, queues stack operations ([`Op`]) for [`Ncp::drive`], and
//! emits events (device, data, frame-counter checkpoints, logs) while the
//! host has credit for them.
//!
//! Implemented command groups (everything else is answered
//! `UNSUPPORTED_COMMAND`, never silently dropped):
//!
//! | Group | Commands |
//! |---|---|
//! | SYSTEM | `SYS_NEGOTIATE`, `SYS_GET_INFO`, `SYS_RESET`, `SYS_GET_STATUS`, `SYS_SET_CONFIG`/`SYS_GET_CONFIG` (`log_level`), `SYS_ECHO`, `SYS_SET_UART` |
//! | DIAG | `DIAG_GET_COUNTERS`/`DIAG_RESET_COUNTERS` (link and queue groups) |
//! | NETWORK | `NWK_INIT`, `NWK_FORM` (NCP-chosen identity), `NWK_LEAVE_RESET`, `NWK_GET_INFO`, `NWK_PERMIT_JOIN` (NCP or all routers), `NWK_ADD_ENDPOINT`; `EVT_DEVICE_JOINED`, `EVT_DEVICE_LEFT` |
//! | DATA | `APS_DATA_REQUEST` → `EVT_APS_DATA_CONFIRM` (after the APS ACK outcome with `ack_request`); `EVT_APS_DATA_INDICATION` |
//! | SECURITY | `SEC_GET_NETWORK_KEY`, `SEC_GET_FRAME_COUNTERS`, `SEC_LOCK_KEY_EXPORT`; `EVT_FC_CHECKPOINT` |
//! | LOG | `LOG_CONFIG`; `EVT_LOG_RECORD`, `EVT_LOG_DROPPED` |
//! | GREEN POWER | all of spec 7.11, in [`crate::gp`] |

use heapless::{Deque, Vec};
use zigbee_zrhp::codec::{Bytes8, Bytes16, List, Rest};
use zigbee_zrhp::frame::{ResetReason, TX_BUFFER_LEN};
use zigbee_zrhp::link::{Link, LinkConfig, LinkEvent, LinkState};
use zigbee_zrhp::msg::data::{ApsDataRequest, ApsDst, EvtApsDataConfirm, EvtApsDataIndication};
use zigbee_zrhp::msg::diag::{Counter, DiagGetCounters, DiagGetCountersRsp, DiagResetCounters};
use zigbee_zrhp::msg::log::{EvtLogDropped, EvtLogRecord, LogConfig};
use zigbee_zrhp::msg::network::{
    EvtDeviceJoined, EvtDeviceLeft, EvtNetworkState, NwkAddEndpoint, NwkForm, NwkGetInfoRsp,
    NwkInit, NwkInitRsp, NwkLeaveReset, NwkPermitJoin,
};
use zigbee_zrhp::msg::security::{
    EvtFcCheckpoint, SecGetFrameCountersRsp, SecGetNetworkKey, SecGetNetworkKeyRsp,
    SecLockKeyExport,
};
use zigbee_zrhp::msg::system::{
    Capacity, ConfigReadItem, ConfigStatus, EvtIndicationsDropped, EvtNcpReady, SysGetConfig,
    SysGetConfigRsp, SysGetInfoRsp, SysGetStatusRsp, SysNegotiateReq, SysNegotiateRsp, SysReset,
    SysSetConfig, SysSetConfigRsp, SysSetUart, capacity, caps, config,
};
use zigbee_zrhp::msg::{Batch, Kind, RawMessage, home_endpoint, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::types::{Endpoint, Ieee, Status};

use crate::log_ring::LogRing;
use crate::port::{
    CoordinatorPort, DataRequest, Destination, DeviceEvent, EndpointError, FormParams,
    HostEndpoint, MAX_ASDU, NetworkState, Op, OpError,
};
use crate::transport::{HostTransport, UartChangeError};

/// Link slots (spec 8.9: 16 × 652 B each direction).
pub const WINDOW: usize = 16;
/// Bytes of queued outgoing messages per endpoint.
pub const QUEUE_BYTES: usize = 1536;
/// `max_aps_inflight`: accepted, not yet confirmed `APS_DATA_REQUEST`s.
pub const MAX_APS_INFLIGHT: usize = 16;
/// Outgoing-counter reservation stride (`FRAME_COUNTER_RESERVATION_SIZE`).
pub const FC_STRIDE: u32 = 0x400;
/// ZRHP major version spoken.
pub const MAJOR: u8 = 1;
/// Minor version.
pub const MINOR: u8 = 0;
/// `SEC_LOCK_KEY_EXPORT` confirm word ("LOCK").
pub const LOCK_CONFIRM: u32 = 0x4B43_4F4C;
/// A frame whose bytes make no transport progress for this long is
/// discarded (no host reading); ARQ recovers it once a host is back.
pub const TX_STALL_DISCARD_US: u64 = 50_000;
/// No valid frame for this long after a `SYS_SET_UART` switch ⇒ revert.
pub const UART_REVERT_US: u64 = 3_000_000;
/// Delay between a `SYS_RESET` response and the reset itself.
pub const RESET_DELAY_US: u64 = 50_000;

/// Static facts the product supplies.
#[derive(Debug, Clone, Copy)]
pub struct NcpIdentity {
    pub fw_version: &'static str,
    pub build_id: [u8; 8],
    /// `SYS_NEGOTIATE.chip`.
    pub chip: u8,
    pub chip_rev: u16,
    /// Fitted flash size (0 = unknown).
    pub flash_size: u32,
    pub board: &'static str,
    pub product: &'static str,
    /// `EVT_NCP_READY.reset_reason`.
    pub reset_reason: u8,
    /// `KEY_EXPORT` built in (spec 7.5, on by default on ESP32).
    pub key_export: bool,
    /// `max_children` of this composition.
    pub max_children: u32,
    /// TC device table size (`max_tc_devices`).
    pub max_tc_devices: u32,
    /// `false` when the partition table does not fit the fitted flash: the
    /// NCP answers `SYS_*` but refuses to start the stack (spec 11.5) and
    /// reports `EVT_FATAL` after every handshake.
    pub stack_allowed: bool,
}

/// What the platform must do after [`Ncp::service`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformRequest {
    /// Reboot (0) or reboot into the ROM bootloader (1).
    Reset { mode: u8 },
}

#[derive(Debug, Clone, Copy)]
struct UartSwitch {
    apply_at_us: u64,
    baud: u32,
    flow: bool,
    previous: (u32, bool),
    applied_rx_frames: Option<u32>,
    applied_at_us: u64,
}

/// Queue-group counters (DIAG group `0x07xx`).
#[derive(Debug, Clone, Copy, Default)]
struct QueueCounters {
    aps_high_water: u32,
    busy_responses: u32,
    indications_dropped: u32,
    log_dropped: u32,
    bad_batches: u32,
}

/// The NCP.
pub struct Ncp<const LOG_N: usize> {
    link: Link<WINDOW, WINDOW>,
    queues: EndpointQueues<QUEUE_BYTES>,
    tx: [u8; TX_BUFFER_LEN],
    tx_len: usize,
    tx_off: usize,
    tx_progress_us: u64,
    need_delimiter: bool,
    identity: NcpIdentity,
    negotiated: bool,
    event_seq: [u16; Endpoint::COUNT],
    ops: Deque<(Op, u64), { MAX_APS_INFLIGHT + 4 }>,
    data_inflight: usize,
    key_export_locked: bool,
    pending_reset: Option<(u8, u64)>,
    uart_switch: Option<UartSwitch>,
    counters: QueueCounters,
    pending_diag: Deque<(u16, u8), 4>,
    reported_nwk_limit: u32,
    reported_aps_limit: u32,
    nwk_checkpoints: u32,
    indication_drops_unreported: u32,
    boot_us: u64,
}

impl<const LOG_N: usize> Ncp<LOG_N> {
    /// `cookie`: a random non-zero boot cookie from the hardware RNG.
    pub fn new(identity: NcpIdentity, cookie: u32, now_us: u64) -> Self {
        let mut link = Link::new(LinkConfig::ncp(), cookie.max(1));
        let reason = match identity.reset_reason {
            3 => ResetReason::Watchdog,
            5 => ResetReason::FirmwareUpdate,
            _ => ResetReason::PowerOn,
        };
        link.open(now_us, reason);
        Self {
            link,
            queues: EndpointQueues::new(),
            tx: [0; TX_BUFFER_LEN],
            tx_len: 0,
            tx_off: 0,
            tx_progress_us: now_us,
            need_delimiter: false,
            identity,
            negotiated: false,
            event_seq: [0; Endpoint::COUNT],
            ops: Deque::new(),
            data_inflight: 0,
            key_export_locked: false,
            pending_reset: None,
            uart_switch: None,
            counters: QueueCounters::default(),
            pending_diag: Deque::new(),
            reported_nwk_limit: 0,
            reported_aps_limit: 0,
            nwk_checkpoints: 0,
            indication_drops_unreported: 0,
            boot_us: now_us,
        }
    }

    pub fn link(&self) -> &Link<WINDOW, WINDOW> {
        &self.link
    }

    /// Session established and negotiated.
    pub fn host_ready(&self) -> bool {
        self.negotiated && self.link.state() == LinkState::Established
    }

    pub fn key_export_locked(&self) -> bool {
        self.key_export_locked
    }

    /// Earliest time `service` must run again (link timers, deferred work).
    pub fn next_deadline(&self) -> Option<u64> {
        let mut next = self.link.next_deadline();
        let mut fold = |t: u64| next = Some(next.map_or(t, |n: u64| n.min(t)));
        if let Some((_, at)) = self.pending_reset {
            fold(at);
        }
        if let Some(switch) = self.uart_switch {
            fold(if switch.applied_rx_frames.is_some() {
                switch.applied_at_us + UART_REVERT_US
            } else {
                switch.apply_at_us
            });
        }
        if self.tx_len != 0 {
            fold(self.tx_progress_us + TX_STALL_DISCARD_US);
        }
        next
    }

    /// One non-blocking pass. Returns a request the platform must carry out.
    pub fn service<T: HostTransport, C: CoordinatorPort>(
        &mut self,
        now_us: u64,
        transport: &mut T,
        port: &mut C,
        log: &LogRing<LOG_N>,
    ) -> Option<PlatformRequest> {
        self.receive(now_us, transport, port, log);
        self.service_uart_switch(now_us, transport);
        if self.host_ready() {
            self.emit_stack_events(now_us, port);
        }
        self.emit_logs(log);
        self.transmit(now_us, transport);
        match self.pending_reset {
            Some((mode, at)) if now_us >= at && self.tx_len == 0 => {
                self.pending_reset = None;
                Some(PlatformRequest::Reset { mode })
            }
            _ => None,
        }
    }

    /// Run the next queued stack operation (if any) and queue its outcome.
    /// `now_us` is read again after the operation for confirm timing.
    pub async fn drive<C: CoordinatorPort>(
        &mut self,
        port: &mut C,
        mut now_us: impl FnMut() -> u64,
    ) -> bool {
        let Some((op, accepted_us)) = self.ops.pop_front() else {
            return false;
        };
        match op {
            Op::Init {
                tid,
                nwk_fc_floor,
                aps_fc_floor,
            } => {
                let result = port.init(nwk_fc_floor, aps_fc_floor).await;
                self.network_outcome(tid, result, port);
            }
            Op::Form { tid, params } => {
                let result = port.form(params).await;
                self.network_outcome(tid, result, port);
            }
            Op::PermitJoin { duration_s, target } => {
                let status = op_status(port.permit_join(duration_s, target).await);
                self.respond_status(ids::NWK_PERMIT_JOIN, accepted_tid(accepted_us), status);
            }
            Op::LeaveReset { flags } => {
                let result = port.leave_reset().await;
                if result.is_ok() && flags & 0x02 == 0 {
                    port.clear_endpoints();
                }
                let status = op_status(result);
                self.respond_status(ids::NWK_LEAVE_RESET, accepted_tid(accepted_us), status);
                if let Some((2, _)) = self.pending_reset {
                    // Factory reset: reboot once the wipe completed.
                    self.pending_reset = Some((0, now_us() + RESET_DELAY_US));
                }
            }
            Op::Data(request) => {
                let result = port.send(&request).await;
                self.data_inflight = self.data_inflight.saturating_sub(1);
                if matches!(&result, Ok(confirm) if confirm.deferred()) {
                    // Awaiting the APS ACK: `emit_stack_events` sends the
                    // confirm from `take_data_confirm`.
                    return true;
                }
                let elapsed_ms = now_us().saturating_sub(accepted_us) / 1000;
                let (status, confirm) = match result {
                    Ok(confirm) if confirm.zb_status == 0 => (0, confirm),
                    Ok(confirm) => (1, confirm),
                    Err(error) => (
                        1,
                        crate::port::DataConfirm {
                            zb_status: match error {
                                OpError::Zigbee(status) => status,
                                _ => 0x01,
                            },
                            aps_counter: 0,
                            dst_nwk: 0xFFFF,
                            flags: 0,
                        },
                    ),
                };
                let event = EvtApsDataConfirm {
                    req_tid: request.req_tid,
                    status,
                    zb_status: confirm.zb_status,
                    aps_counter: confirm.aps_counter,
                    dst_nwk: confirm.dst_nwk,
                    flags: confirm.flags,
                    elapsed_ms: elapsed_ms.min(u64::from(u16::MAX)) as u16,
                };
                self.event(Endpoint::Zigbee, &event);
            }
        }
        true
    }

    // ── Receive and dispatch ────────────────────────────────────────

    fn receive<T: HostTransport, C: CoordinatorPort>(
        &mut self,
        now_us: u64,
        transport: &mut T,
        port: &mut C,
        log: &LogRing<LOG_N>,
    ) {
        let faults = transport.take_line_faults();
        for index in 0..faults.errors {
            self.link.on_line_error(now_us, index < faults.overruns);
        }
        let mut buf = [0u8; 256];
        loop {
            let count = transport.read(&mut buf);
            if count == 0 {
                break;
            }
            let mut offset = 0;
            while offset < count {
                offset += self.link.push_rx(&buf[offset..count], now_us);
                self.drain_events(now_us, transport, port, log);
            }
        }
        self.drain_events(now_us, transport, port, log);
    }

    fn drain_events<T: HostTransport, C: CoordinatorPort>(
        &mut self,
        now_us: u64,
        transport: &mut T,
        port: &mut C,
        log: &LogRing<LOG_N>,
    ) {
        let cookie = self.link.cookie();
        // Events borrow the link; dispatch only touches the other fields.
        while let Some(event) = self.link.poll_event() {
            match event {
                LinkEvent::Established(_) => {
                    self.negotiated = false;
                    self.event_seq = [0; Endpoint::COUNT];
                    self.queues.clear();
                    let ready = ncp_ready(
                        cookie,
                        &self.identity,
                        self.indication_drops_unreported,
                        port,
                    );
                    push_event(&mut self.queues, &mut self.event_seq, &ready);
                    if !self.identity.stack_allowed {
                        let fatal = zigbee_zrhp::msg::system::EvtFatal {
                            kind: 1,
                            code: self.identity.flash_size,
                            text: Bytes8(b"layout mismatch"),
                        };
                        push_event(&mut self.queues, &mut self.event_seq, &fatal);
                    }
                }
                LinkEvent::PeerReset { .. } | LinkEvent::LinkDown | LinkEvent::LinkFailed => {
                    self.negotiated = false;
                }
                LinkEvent::Data { endpoint, payload } => match Batch::parse(payload) {
                    Ok(batch) => {
                        let mut state = DispatchState {
                            negotiated: &mut self.negotiated,
                            ops: &mut self.ops,
                            data_inflight: &mut self.data_inflight,
                            key_export_locked: &mut self.key_export_locked,
                            pending_reset: &mut self.pending_reset,
                            uart_switch: &mut self.uart_switch,
                            pending_diag: &mut self.pending_diag,
                            counters: &mut self.counters,
                            identity: &self.identity,
                            boot_us: self.boot_us,
                        };
                        for message in batch.iter() {
                            Self::dispatch(
                                &mut self.queues,
                                &mut state,
                                now_us,
                                endpoint,
                                &message,
                                transport,
                                port,
                                log,
                            );
                        }
                    }
                    Err(_) => {
                        self.counters.bad_batches = self.counters.bad_batches.saturating_add(1);
                        let error = zigbee_zrhp::msg::system::EvtProtocolError {
                            endpoint: endpoint as u8,
                            msg_id: Batch::first_msg_id(payload).unwrap_or(0),
                            reason: 1,
                        };
                        push_event(&mut self.queues, &mut self.event_seq, &error);
                    }
                },
                LinkEvent::TxFailed { .. } => {
                    // Responses and events are not retried: the host times
                    // out or resyncs (6.5).
                }
            }
        }
        // DIAG_GET_COUNTERS needs the link, free again here.
        while let Some((tid, mask)) = self.pending_diag.pop_front() {
            let counters = self.counters_response(mask);
            let response = DiagGetCountersRsp {
                counters: List::new(&counters),
            };
            let _ = self.queues.response(tid, Status::Success, Some(&response));
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn dispatch<T: HostTransport, C: CoordinatorPort>(
        queues: &mut EndpointQueues<QUEUE_BYTES>,
        state: &mut DispatchState<'_>,
        now_us: u64,
        endpoint: Endpoint,
        message: &RawMessage<'_>,
        transport: &mut T,
        port: &mut C,
        log: &LogRing<LOG_N>,
    ) {
        if message.header.kind != Kind::Request {
            // No NCP-originated requests exist yet; stray responses/events
            // from the host are ignored.
            return;
        }
        let id = message.header.msg_id;
        let tid = message.header.tid;
        let Some(home) = home_endpoint(id) else {
            let _ = queues
                .queue(endpoint)
                .push_with(|w| w.response_status(id, tid, Status::UnsupportedCommand));
            return;
        };
        let status_only = |queues: &mut EndpointQueues<QUEUE_BYTES>, status: Status| {
            let _ = queues
                .queue(endpoint)
                .push_with(|w| w.response_status(id, tid, status));
        };
        if home != endpoint {
            status_only(queues, Status::WrongEndpoint);
            return;
        }
        if !*state.negotiated && id != ids::SYS_NEGOTIATE {
            status_only(queues, Status::InvalidState);
            return;
        }
        if !state.identity.stack_allowed && matches!(id, ids::NWK_INIT | ids::NWK_FORM) {
            status_only(queues, Status::InvalidState);
            return;
        }
        macro_rules! decode {
            ($ty:ty) => {
                match message.decode::<$ty>() {
                    Ok(value) => value,
                    Err(_) => {
                        status_only(queues, Status::InvalidParameter);
                        return;
                    }
                }
            };
        }
        match id {
            ids::SYS_NEGOTIATE => {
                let request = decode!(SysNegotiateReq);
                if request.major_min > MAJOR || request.major_max < MAJOR {
                    status_only(queues, Status::VersionMismatch);
                    return;
                }
                let info = transport.info();
                let mut ncp_caps = 0u64;
                if state.identity.key_export {
                    ncp_caps |= caps::KEY_EXPORT;
                }
                if info.kind == 2 && info.flow_control {
                    ncp_caps |= caps::UART_FLOW_CONTROL;
                }
                // Green Power combo (spec 7.11).
                let (gp_caps, gp_sink_table, gp_proxy_table) = crate::gp::negotiate_facts(port);
                ncp_caps |= gp_caps;
                let capacities = [
                    Capacity {
                        id: capacity::MAX_APS_INFLIGHT,
                        value: MAX_APS_INFLIGHT as u32,
                    },
                    Capacity {
                        id: capacity::MAX_CHILDREN,
                        value: state.identity.max_children,
                    },
                    Capacity {
                        id: capacity::MAX_ASDU,
                        value: MAX_ASDU as u32,
                    },
                    Capacity {
                        id: capacity::FC_STRIDE,
                        value: FC_STRIDE,
                    },
                    Capacity {
                        id: capacity::MAX_HOST_ENDPOINTS,
                        value: u32::from(port.max_host_endpoints()),
                    },
                    Capacity {
                        id: capacity::MAX_NCP_REQUESTS,
                        value: 0,
                    },
                    Capacity {
                        id: capacity::MAX_TC_DEVICES,
                        value: state.identity.max_tc_devices,
                    },
                    // Table sizes of the linked stack (`ncp-capacity`).
                    Capacity {
                        id: capacity::LINK_KEY_CACHE,
                        value: zigbee_aps::security::MAX_KEY_TABLE_ENTRIES as u32,
                    },
                    Capacity {
                        id: capacity::ADDRESS_MAP_CACHE,
                        value: zigbee_nwk::address_map::MAX_ADDRESS_MAP_ENTRIES as u32,
                    },
                    Capacity {
                        id: capacity::SOURCE_ROUTE_CACHE,
                        value: zigbee_nwk::routing::MAX_SOURCE_ROUTES as u32,
                    },
                    Capacity {
                        id: capacity::NEIGHBOR_TABLE,
                        value: zigbee_nwk::neighbor::MAX_NEIGHBORS as u32,
                    },
                    Capacity {
                        id: capacity::GP_SINK_TABLE,
                        value: gp_sink_table,
                    },
                    Capacity {
                        id: capacity::GP_PROXY_TABLE,
                        value: gp_proxy_table,
                    },
                ];
                let response = SysNegotiateRsp {
                    major: MAJOR,
                    minor: MINOR,
                    ncp_caps,
                    capacities: List::new(&capacities),
                    fw: Bytes8(state.identity.fw_version.as_bytes()),
                    build_id: state.identity.build_id,
                    chip: state.identity.chip,
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
                *state.negotiated = true;
            }
            ids::SYS_GET_INFO => {
                let info = transport.info();
                let ieee = Ieee(port.ieee());
                let response = SysGetInfoRsp {
                    ieee_factory: ieee,
                    ieee_active: ieee,
                    ieee_source: 0,
                    chip_rev: state.identity.chip_rev,
                    flash_size: state.identity.flash_size,
                    transport: info.kind,
                    uart_baud: info.baud,
                    uart_flow: u8::from(info.flow_control),
                    board: Bytes8(state.identity.board.as_bytes()),
                    product: Bytes8(state.identity.product.as_bytes()),
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
            }
            ids::SYS_RESET => {
                let request = decode!(SysReset);
                match request.mode {
                    0 | 1 => {
                        status_only(queues, Status::Success);
                        *state.pending_reset = Some((request.mode, now_us + RESET_DELAY_US));
                    }
                    2 if request.confirm == zigbee_zrhp::msg::system::FACTORY_RESET_CONFIRM => {
                        if state
                            .ops
                            .push_back((Op::LeaveReset { flags: 0 }, u64::from(tid)))
                            .is_err()
                        {
                            status_only(queues, Status::Busy);
                            return;
                        }
                        status_only(queues, Status::Success);
                        // Rebooted by `drive` once the wipe completed.
                        *state.pending_reset = Some((2, u64::MAX));
                    }
                    _ => status_only(queues, Status::InvalidParameter),
                }
            }
            ids::SYS_GET_STATUS => {
                let mut flags = 0u8;
                if *state.key_export_locked {
                    flags |= 0x01;
                }
                if port.permit_join_remaining_s() != 0 {
                    flags |= 0x10;
                }
                let response = SysGetStatusRsp {
                    network_state: port.state() as u8,
                    uptime_s: (now_us.saturating_sub(state.boot_us) / 1_000_000) as u32,
                    free_aps_slots: (MAX_APS_INFLIGHT
                        - (*state.data_inflight).min(MAX_APS_INFLIGHT))
                        as u16,
                    host_absent_queue: 0,
                    dirty_writebacks: 0,
                    restore_in_progress: 0,
                    flags,
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
            }
            ids::SYS_SET_CONFIG => {
                let request = decode!(SysSetConfig);
                let mut results: Vec<ConfigStatus, 16> = Vec::new();
                for item in request.values.iter() {
                    let status = match item.id {
                        config::LOG_LEVEL if item.value.0.len() == 1 && item.value.0[0] <= 5 => {
                            log.set_level(item.value.0[0]);
                            Status::Success
                        }
                        config::LOG_LEVEL => Status::InvalidParameter,
                        _ => Status::UnsupportedCommand,
                    };
                    if results
                        .push(ConfigStatus {
                            id: item.id,
                            status,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
                let response = SysSetConfigRsp {
                    results: List::new(&results),
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
            }
            ids::SYS_GET_CONFIG => {
                let request = decode!(SysGetConfig);
                let level = [log.level()];
                let mut items: Vec<ConfigReadItem<'_>, 16> = Vec::new();
                for config_id in request.ids.iter() {
                    let item = if config_id == config::LOG_LEVEL {
                        ConfigReadItem {
                            id: config_id,
                            status: Status::Success,
                            value: Bytes8(&level),
                        }
                    } else {
                        ConfigReadItem {
                            id: config_id,
                            status: Status::UnsupportedCommand,
                            value: Bytes8(&[]),
                        }
                    };
                    if items.push(item).is_err() {
                        break;
                    }
                }
                let response = SysGetConfigRsp {
                    items: List::new(&items),
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
            }
            ids::SYS_ECHO => {
                let data = message.payload;
                if data.len() > 64 {
                    status_only(queues, Status::InvalidParameter);
                    return;
                }
                let mut payload = [0u8; 65];
                payload[0] = Status::Success.to_u8();
                payload[1..=data.len()].copy_from_slice(data);
                let _ =
                    queues
                        .queue(endpoint)
                        .raw(Kind::Response, id, tid, &payload[..=data.len()]);
            }
            ids::SYS_SET_UART => {
                let request = decode!(SysSetUart);
                let flow = request.flow != 0;
                let status = match transport.check_uart(request.baud, flow) {
                    Ok(()) => {
                        let info = transport.info();
                        *state.uart_switch = Some(UartSwitch {
                            apply_at_us: now_us + u64::from(request.apply_delay_ms) * 1000,
                            baud: request.baud,
                            flow,
                            previous: (info.baud, info.flow_control),
                            applied_rx_frames: None,
                            applied_at_us: 0,
                        });
                        Status::Success
                    }
                    Err(UartChangeError::InvalidRate) => Status::InvalidParameter,
                    Err(UartChangeError::Busy) => Status::Busy,
                    Err(UartChangeError::Unsupported) => Status::UnsupportedCommand,
                };
                status_only(queues, status);
            }
            ids::DIAG_GET_COUNTERS => {
                let request = decode!(DiagGetCounters);
                // Answered once the link no longer lends this frame.
                if state
                    .pending_diag
                    .push_back((tid, request.group_mask))
                    .is_err()
                {
                    status_only(queues, Status::Busy);
                }
            }
            ids::DIAG_RESET_COUNTERS => {
                let request = decode!(DiagResetCounters);
                crate::gp::reset_counters(request.group_mask);
                *state.counters = QueueCounters::default();
                status_only(queues, Status::Success);
            }
            ids::NWK_INIT => {
                let request = decode!(NwkInit);
                let network_state = port.state();
                if network_state == NetworkState::None {
                    let _ = queues.response(
                        tid,
                        Status::NotFormed,
                        Some(&NwkInitRsp { network_state: 0 }),
                    );
                    return;
                }
                let op = Op::Init {
                    tid,
                    nwk_fc_floor: request.nwk_fc_floor,
                    aps_fc_floor: request.aps_fc_floor,
                };
                if state.ops.push_back((op, now_us)).is_err() {
                    status_only(queues, Status::Busy);
                    return;
                }
                let _ = queues.response(
                    tid,
                    Status::Success,
                    Some(&NwkInitRsp {
                        network_state: network_state as u8,
                    }),
                );
            }
            ids::NWK_FORM => {
                let request = decode!(NwkForm);
                if port.state() != NetworkState::None {
                    status_only(queues, Status::InvalidState);
                    return;
                }
                let params = FormParams {
                    channel_mask: request.channel_mask,
                    pan_id: request.pan_id,
                    ext_pan_id: request.ext_pan_id,
                    network_key: request.network_key,
                    key_seq: request.key_seq,
                    nwk_update_id: request.nwk_update_id,
                    tx_power: request.tx_power,
                    nwk_manager: request.nwk_manager,
                };
                if !params.ncp_chosen_identity() {
                    // Host-supplied PAN ID/EPID/key need BDB formation
                    // parameters the runtime does not take yet.
                    status_only(queues, Status::UnsupportedCommand);
                    return;
                }
                if state
                    .ops
                    .push_back((Op::Form { tid, params }, now_us))
                    .is_err()
                {
                    status_only(queues, Status::Busy);
                    return;
                }
                status_only(queues, Status::Success);
            }
            ids::NWK_LEAVE_RESET => {
                // Older hosts send no flags: clear the host endpoints.
                let flags = message.decode::<NwkLeaveReset>().map_or(0, |r| r.flags);
                if state
                    .ops
                    .push_back((Op::LeaveReset { flags }, u64::from(tid)))
                    .is_err()
                {
                    status_only(queues, Status::Busy);
                }
                // Responded by `drive` after the wipe.
            }
            ids::NWK_ADD_ENDPOINT => {
                let request = decode!(NwkAddEndpoint);
                let status = match host_endpoint(&request) {
                    Some(endpoint) => match port.add_endpoint(&endpoint) {
                        Ok(()) => Status::Success,
                        Err(EndpointError::InvalidParameter) => Status::InvalidParameter,
                        Err(EndpointError::NoSpace) => Status::NoSpace,
                        Err(EndpointError::Unsupported) => Status::UnsupportedCommand,
                    },
                    None => Status::InvalidParameter,
                };
                status_only(queues, status);
            }
            ids::NWK_GET_INFO => {
                let info = port.info();
                let response = NwkGetInfoRsp {
                    state: info.state as u8,
                    pan_id: info.pan_id,
                    ext_pan_id: info.ext_pan_id,
                    channel: info.channel,
                    channel_mask: info.channel_mask,
                    short: info.short,
                    ieee: Ieee(info.ieee),
                    nwk_update_id: info.nwk_update_id,
                    nwk_manager: info.nwk_manager,
                    key_seq: info.key_seq,
                    security_level: 5,
                    tx_power: info.tx_power,
                    stack_profile: 2,
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
            }
            ids::NWK_PERMIT_JOIN => {
                let request = decode!(NwkPermitJoin);
                if port.state() != NetworkState::Up {
                    status_only(queues, Status::InvalidState);
                    return;
                }
                if !matches!(request.target, 0x0000 | 0xFFFC) {
                    // A single remote router needs Mgmt_Permit_Joining_req
                    // addressed to it; not wired yet.
                    status_only(queues, Status::UnsupportedCommand);
                    return;
                }
                let op = Op::PermitJoin {
                    duration_s: request.duration_s,
                    target: request.target,
                };
                if state.ops.push_back((op, u64::from(tid))).is_err() {
                    status_only(queues, Status::Busy);
                }
                // Responded by `drive`.
            }
            ids::APS_DATA_REQUEST => {
                let request = decode!(ApsDataRequest);
                // Green Power sink ownership (spec 7.11): CONFLICT in
                // NCP_SINK mode, local proxy loopback in HOST_SINK mode.
                if let Some(status) = crate::gp::screen_data_request(port, &request) {
                    status_only(queues, status);
                    return;
                }
                let status = Self::accept_data(state, port, tid, now_us, &request);
                if status == Status::Busy {
                    state.counters.busy_responses = state.counters.busy_responses.saturating_add(1);
                }
                status_only(queues, status);
            }
            ids::SEC_GET_NETWORK_KEY => {
                let request = decode!(SecGetNetworkKey);
                if !state.identity.key_export || *state.key_export_locked {
                    status_only(queues, Status::AccessDenied);
                    return;
                }
                match port.network_key(request.which) {
                    Some((key, seq)) => {
                        let response = SecGetNetworkKeyRsp {
                            key,
                            seq,
                            nwk_fc_limit: port.frame_counters().nwk_limit,
                        };
                        let _ = queues.response(tid, Status::Success, Some(&response));
                    }
                    None if port.state() == NetworkState::None => {
                        status_only(queues, Status::NotFormed)
                    }
                    None => status_only(queues, Status::NotFound),
                }
            }
            ids::SEC_GET_FRAME_COUNTERS => {
                let counters = port.frame_counters();
                let response = SecGetFrameCountersRsp {
                    nwk_current: counters.nwk_current,
                    nwk_limit: counters.nwk_limit,
                    aps_current: counters.aps_current,
                    aps_limit: counters.aps_limit,
                    stride: FC_STRIDE,
                };
                let _ = queues.response(tid, Status::Success, Some(&response));
            }
            ids::SEC_LOCK_KEY_EXPORT => {
                let request = decode!(SecLockKeyExport);
                if request.confirm != LOCK_CONFIRM {
                    status_only(queues, Status::InvalidParameter);
                    return;
                }
                *state.key_export_locked = true;
                status_only(queues, Status::Success);
            }
            ids::LOG_CONFIG => {
                let request = decode!(LogConfig);
                if request.level > 5 {
                    status_only(queues, Status::InvalidParameter);
                    return;
                }
                log.set_level(request.level);
                status_only(queues, Status::Success);
            }
            // GREEN POWER (`0x0Axx`, spec 7.11) and unknown IDs.
            _ => crate::gp::dispatch_gp(
                queues,
                endpoint,
                message,
                state.identity.key_export && !*state.key_export_locked,
                port,
            ),
        }
    }

    fn accept_data<C: CoordinatorPort>(
        state: &mut DispatchState<'_>,
        port: &C,
        tid: u16,
        now_us: u64,
        request: &ApsDataRequest<'_>,
    ) -> Status {
        if port.state() != NetworkState::Up {
            return Status::InvalidState;
        }
        if request.relays.is_some() || request.alias.is_some() {
            // HOST_SOURCE_ROUTES and alias transmission are not advertised.
            return Status::UnsupportedCommand;
        }
        if request.tx_options & 0x08 != 0 && request.asdu.len() > MAX_ASDU {
            return Status::UnsupportedCommand;
        }
        let Ok(asdu) = Vec::from_slice(request.asdu) else {
            return Status::TooLarge;
        };
        let dst = match request.dst {
            ApsDst::Group(group) => Destination::Group(group),
            ApsDst::Nwk(nwk) => Destination::Nwk(nwk),
            ApsDst::Ieee(ieee) => Destination::Ieee(ieee.0),
            ApsDst::Broadcast(address) if address >= 0xFFF8 => Destination::Broadcast(address),
            ApsDst::Broadcast(_) => return Status::InvalidParameter,
        };
        // No ZDO loopback (spec 7.4, decision 10).
        if request.profile == 0
            && match dst {
                Destination::Nwk(nwk) => nwk == port.info().short,
                Destination::Ieee(ieee) => ieee == port.ieee(),
                _ => false,
            }
        {
            return Status::InvalidParameter;
        }
        if *state.data_inflight >= MAX_APS_INFLIGHT {
            return Status::Busy;
        }
        let data = DataRequest {
            req_tid: tid,
            dst,
            dst_ep: request.dst_ep,
            src_ep: request.src_ep,
            profile: request.profile,
            cluster: request.cluster,
            tx_options: request.tx_options,
            radius: request.radius,
            asdu,
        };
        if state.ops.push_back((Op::Data(data), now_us)).is_err() {
            return Status::Busy;
        }
        *state.data_inflight += 1;
        state.counters.aps_high_water = state
            .counters
            .aps_high_water
            .max(*state.data_inflight as u32);
        Status::Success
    }

    // ── Outgoing events ─────────────────────────────────────────────

    fn event<'a, T: zigbee_zrhp::codec::Wire<'a> + zigbee_zrhp::msg::MessageDef>(
        &mut self,
        _endpoint: Endpoint,
        body: &T,
    ) -> bool {
        push_event(&mut self.queues, &mut self.event_seq, body)
    }

    fn respond_status(&mut self, id: u16, tid: u16, status: Status) {
        if let Some(endpoint) = home_endpoint(id) {
            let _ = self
                .queues
                .queue(endpoint)
                .push_with(|w| w.response_status(id, tid, status));
        }
    }

    fn network_outcome<C: CoordinatorPort>(
        &mut self,
        tid: u16,
        result: Result<(), OpError>,
        port: &C,
    ) {
        let (state, zb_status) = match result {
            Ok(()) => (port.state(), 0),
            Err(OpError::Zigbee(status)) => (port.state(), status),
            Err(_) => (port.state(), 0x01),
        };
        let event = EvtNetworkState {
            state: if state == NetworkState::Up { 2 } else { 0 },
            reason: u8::from(result.is_err()),
            zb_status,
            req_tid: tid,
        };
        self.event(Endpoint::Zigbee, &event);
    }

    fn emit_stack_events<C: CoordinatorPort>(&mut self, now_us: u64, port: &mut C) {
        // Frame-counter checkpoints after every durable reservation.
        let counters = port.frame_counters();
        let key_seq = port.info().key_seq;
        if counters.nwk_limit != self.reported_nwk_limit
            && self.queues.queue(Endpoint::Zigbee).free() >= 32
        {
            self.nwk_checkpoints = self.nwk_checkpoints.wrapping_add(1);
            let event = EvtFcCheckpoint {
                domain: 1,
                key_seq,
                durable_limit: counters.nwk_limit,
                generation: self.nwk_checkpoints,
            };
            if self.event(Endpoint::Zigbee, &event) {
                self.reported_nwk_limit = counters.nwk_limit;
            }
        }
        if counters.aps_limit != self.reported_aps_limit
            && self.queues.queue(Endpoint::Zigbee).free() >= 32
        {
            let event = EvtFcCheckpoint {
                domain: 2,
                key_seq,
                durable_limit: counters.aps_limit,
                generation: counters.aps_commits,
            };
            if self.event(Endpoint::Zigbee, &event) {
                self.reported_aps_limit = counters.aps_limit;
            }
        }
        while self.queues.queue(Endpoint::Zigbee).free() >= 32 {
            let Some(event) = port.take_device_event() else {
                break;
            };
            match event {
                DeviceEvent::Joined {
                    nwk,
                    ieee,
                    parent_nwk,
                    join_type,
                    capability,
                    decision,
                } => {
                    self.event(
                        Endpoint::Zigbee,
                        &EvtDeviceJoined {
                            nwk,
                            ieee: Ieee(ieee),
                            parent_nwk,
                            join_type,
                            capability,
                            decision,
                        },
                    );
                }
                DeviceEvent::Left {
                    nwk,
                    ieee,
                    flags,
                    reason,
                } => {
                    self.event(
                        Endpoint::Zigbee,
                        &EvtDeviceLeft {
                            nwk,
                            ieee: Ieee(ieee),
                            flags,
                            reason,
                        },
                    );
                }
            }
        }
        // Confirms that waited for their APS ACK.
        while self.queues.queue(Endpoint::Zigbee).free() >= 32 {
            let Some(deferred) = port.take_data_confirm() else {
                break;
            };
            let confirm = deferred.confirm;
            let event = EvtApsDataConfirm {
                req_tid: deferred.req_tid,
                status: u8::from(confirm.zb_status != 0),
                zb_status: confirm.zb_status,
                aps_counter: confirm.aps_counter,
                dst_nwk: confirm.dst_nwk,
                flags: confirm.flags,
                elapsed_ms: deferred.elapsed_ms,
            };
            self.event(Endpoint::Zigbee, &event);
        }
        // Indications only while the queue can take a full-size one; the
        // stack's bounded queue absorbs (and counts) the rest.
        while self.queues.queue(Endpoint::Zigbee).free()
            >= 7 + 32 + crate::port::MAX_INDICATION_ASDU
        {
            let Some(indication) = port.take_indication() else {
                break;
            };
            let mut flags = 0u8;
            if indication.aps_secured {
                flags |= 0x01;
            }
            if indication.nwk_secured {
                flags |= 0x02;
            }
            if indication.src_ieee.is_some() {
                flags |= 0x04;
            }
            match indication.dst_mode {
                0x0F => flags |= 0x08,
                0x01 => flags |= 0x10,
                _ => {}
            }
            if indication.handled_by_ncp {
                flags |= 0x40;
            }
            let event = EvtApsDataIndication {
                dst_mode: indication.dst_mode,
                dst: indication.dst,
                dst_ep: indication.dst_ep,
                src_nwk: indication.src_nwk,
                src_ieee: Ieee(indication.src_ieee.unwrap_or([0; 8])),
                src_ep: indication.src_ep,
                profile: indication.profile,
                cluster: indication.cluster,
                flags,
                lqi: indication.lqi,
                rssi: indication.rssi,
                last_hop_nwk: indication.last_hop_nwk,
                aps_counter: indication.aps_counter,
                key_source: indication.key_source,
                rx_time_ms: indication.rx_time_ms,
                asdu: Bytes16(&indication.asdu),
            };
            self.event(Endpoint::Zigbee, &event);
        }
        // Green Power reports and events (spec 7.11).
        crate::gp::emit_gp_events(
            &mut self.queues,
            &mut self.event_seq,
            self.identity.key_export && !self.key_export_locked,
            port,
        );
        let _ = now_us;
        if self.indication_drops_unreported != 0 && self.queues.queue(Endpoint::System).free() >= 32
        {
            let event = EvtIndicationsDropped {
                endpoint: Endpoint::Zigbee as u8,
                reason: 3,
                count: self.indication_drops_unreported,
                first_ms: 0,
                last_ms: (now_us / 1000) as u32,
            };
            if self.event(Endpoint::System, &event) {
                self.indication_drops_unreported = 0;
            }
        }
    }

    /// Record indications the stack dropped (its queue was full).
    pub fn note_indications_dropped(&mut self, count: u32) {
        self.indication_drops_unreported = self.indication_drops_unreported.saturating_add(count);
        self.counters.indications_dropped = self.counters.indications_dropped.saturating_add(count);
    }

    fn emit_logs(&mut self, log: &LogRing<LOG_N>) {
        if !self.host_ready() || !self.link.can_send(Endpoint::Log) {
            if !log.is_empty() && !self.host_ready() {
                // No session: the ring keeps the newest records.
                return;
            }
            if !self.link.can_send(Endpoint::Log) {
                log.drop_all();
            }
            if let Some((count, _)) = log.take_dropped() {
                self.counters.log_dropped = self.counters.log_dropped.saturating_add(count);
            }
            return;
        }
        if let Some((count, first_seq)) = log.take_dropped() {
            self.counters.log_dropped = self.counters.log_dropped.saturating_add(count);
            self.event(Endpoint::Log, &EvtLogDropped { count, first_seq });
        }
        while self.queues.queue(Endpoint::Log).free() >= 7 + 8 + crate::log_ring::MAX_LOG_TEXT {
            let Some(record) = log.pop() else {
                break;
            };
            let event = EvtLogRecord {
                time_ms: record.time_ms,
                level: record.level,
                module: 0,
                seq: record.seq,
                text: Rest(record.text.as_bytes()),
            };
            self.event(Endpoint::Log, &event);
        }
    }

    // ── UART rate switch (SYS_SET_UART) ─────────────────────────────

    fn service_uart_switch<T: HostTransport>(&mut self, now_us: u64, transport: &mut T) {
        let Some(mut switch) = self.uart_switch else {
            return;
        };
        let rx_frames = self.link.counters().rx_frames;
        match switch.applied_rx_frames {
            None if now_us >= switch.apply_at_us && self.tx_len == 0 => {
                if transport.apply_uart(switch.baud, switch.flow).is_ok() {
                    switch.applied_rx_frames = Some(rx_frames);
                    switch.applied_at_us = now_us;
                    self.uart_switch = Some(switch);
                } else {
                    self.uart_switch = None;
                }
            }
            Some(at_switch) if rx_frames != at_switch => self.uart_switch = None,
            Some(_) if now_us >= switch.applied_at_us + UART_REVERT_US => {
                let (baud, flow) = switch.previous;
                let _ = transport.apply_uart(baud, flow);
                self.uart_switch = None;
            }
            _ => {}
        }
    }

    // ── Transmit ────────────────────────────────────────────────────

    fn transmit<T: HostTransport>(&mut self, now_us: u64, transport: &mut T) {
        loop {
            if self.tx_len != 0 {
                if self.need_delimiter && transport.write(&[0]) == 1 {
                    self.need_delimiter = false;
                }
                let written = if self.need_delimiter {
                    0
                } else {
                    transport.write(&self.tx[self.tx_off..self.tx_len])
                };
                if written != 0 {
                    self.tx_off += written;
                    self.tx_progress_us = now_us;
                }
                if self.tx_off < self.tx_len {
                    if now_us.saturating_sub(self.tx_progress_us) >= TX_STALL_DISCARD_US {
                        // No host is reading: drop the frame; ARQ (or the
                        // next RESET_REQ) replaces it once one is.
                        if self.tx_off != 0 {
                            self.need_delimiter = true;
                        }
                        self.tx_len = 0;
                        self.tx_off = 0;
                        self.link.note_discarded_no_host();
                        continue;
                    }
                    return;
                }
                self.tx_len = 0;
                self.tx_off = 0;
            }
            match self
                .link
                .poll_transmit(now_us, &mut self.queues, &mut self.tx)
            {
                Some(length) => {
                    self.tx_len = length;
                    self.tx_off = 0;
                    self.tx_progress_us = now_us;
                }
                None => {
                    transport.end_burst();
                    return;
                }
            }
        }
    }

    /// Answer `DIAG_GET_COUNTERS` (link group from the link, queue group
    /// from the NCP). Called from `dispatch` through `service` because the
    /// link is borrowed while frames are delivered.
    fn counters_response(&self, group_mask: u8) -> Vec<Counter, 48> {
        let mut out = Vec::new();
        if group_mask & 0x01 != 0 {
            for (id, value) in self.link.counters().entries() {
                let _ = out.push(Counter { id, value });
            }
        }
        if group_mask & 0x40 != 0 {
            for (id, value) in [
                (0x0701, self.counters.aps_high_water),
                (0x0702, self.counters.busy_responses),
                (0x0703, self.counters.indications_dropped),
                (0x0704, self.counters.log_dropped),
                (0x0705, self.counters.bad_batches),
            ] {
                let _ = out.push(Counter { id, value });
            }
        }
        crate::gp::append_counters(group_mask, &mut out);
        out
    }
}

/// Mutable dispatcher state, split from the link so frames can be
/// dispatched while the link lends their payload.
struct DispatchState<'s> {
    negotiated: &'s mut bool,
    pending_diag: &'s mut Deque<(u16, u8), 4>,
    ops: &'s mut Deque<(Op, u64), { MAX_APS_INFLIGHT + 4 }>,
    data_inflight: &'s mut usize,
    key_export_locked: &'s mut bool,
    pending_reset: &'s mut Option<(u8, u64)>,
    uart_switch: &'s mut Option<UartSwitch>,
    counters: &'s mut QueueCounters,
    identity: &'s NcpIdentity,
    boot_us: u64,
}

fn next_seq(seqs: &mut [u16; Endpoint::COUNT], endpoint: Endpoint) -> u16 {
    let seq = &mut seqs[endpoint as usize];
    *seq = seq.wrapping_add(1);
    if *seq == 0 {
        *seq = 1;
    }
    *seq
}

/// Ops answered after completion carry the request TID in the time slot.
fn accepted_tid(slot: u64) -> u16 {
    slot as u16
}

/// `NWK_ADD_ENDPOINT` as a [`HostEndpoint`]; `None` when a cluster list is
/// longer than [`crate::port::MAX_ENDPOINT_CLUSTERS`].
fn host_endpoint(request: &NwkAddEndpoint<'_>) -> Option<HostEndpoint> {
    let mut endpoint = HostEndpoint {
        endpoint: request.ep,
        profile: request.profile,
        device_id: request.device_id,
        version: request.version,
        in_clusters: Vec::new(),
        out_clusters: Vec::new(),
    };
    for cluster in request.in_clusters.iter() {
        endpoint.in_clusters.push(cluster).ok()?;
    }
    for cluster in request.out_clusters.iter() {
        endpoint.out_clusters.push(cluster).ok()?;
    }
    Some(endpoint)
}

fn op_status(result: Result<(), OpError>) -> Status {
    match result {
        Ok(()) => Status::Success,
        Err(OpError::InvalidState) => Status::InvalidState,
        Err(OpError::NotFormed) => Status::NotFormed,
        Err(OpError::Storage) => Status::StorageError,
        Err(OpError::Unsupported) => Status::UnsupportedCommand,
        Err(OpError::Zigbee(_)) => Status::Failure,
    }
}

fn ncp_ready<C: CoordinatorPort>(
    cookie: u32,
    identity: &NcpIdentity,
    dropped_indications: u32,
    port: &C,
) -> EvtNcpReady {
    let counters = port.frame_counters();
    EvtNcpReady {
        ncp_cookie: cookie,
        reset_reason: identity.reset_reason,
        network_state: port.state() as u8,
        table_epoch: 0,
        nwk_fc_limit: counters.nwk_limit,
        aps_fc_limit: counters.aps_limit,
        dirty_writebacks: 0,
        dropped_indications,
    }
}

/// Queue an event on its home endpoint with that endpoint's next sequence.
pub(crate) fn push_event<'a, T: zigbee_zrhp::codec::Wire<'a> + zigbee_zrhp::msg::MessageDef>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    seqs: &mut [u16; Endpoint::COUNT],
    body: &T,
) -> bool {
    let Some(endpoint) = home_endpoint(T::ID) else {
        return false;
    };
    let seq = next_seq(seqs, endpoint);
    queues.event(seq, body).is_ok()
}

impl<const LOG_N: usize> Ncp<LOG_N> {
    /// [`Self::new`] directly into caller-provided (normally static)
    /// storage, field by field: neither the NCP (about 33 KiB) nor its link
    /// engine (about 22 KiB) is built on the stack and moved.
    pub fn new_into(
        dst: &mut core::mem::MaybeUninit<Self>,
        identity: NcpIdentity,
        cookie: u32,
        now_us: u64,
    ) -> &mut Self {
        // Every field is written below, with the value `new` gives it; a
        // new field must be added to both.
        let _exhaustive = |ncp: &Self| {
            let Self {
                link: _,
                queues: _,
                tx: _,
                tx_len: _,
                tx_off: _,
                tx_progress_us: _,
                need_delimiter: _,
                identity: _,
                negotiated: _,
                event_seq: _,
                ops: _,
                data_inflight: _,
                key_export_locked: _,
                pending_reset: _,
                uart_switch: _,
                counters: _,
                pending_diag: _,
                reported_nwk_limit: _,
                reported_aps_limit: _,
                nwk_checkpoints: _,
                indication_drops_unreported: _,
                boot_us: _,
            } = ncp;
        };
        let reason = match identity.reset_reason {
            3 => ResetReason::Watchdog,
            5 => ResetReason::FirmwareUpdate,
            _ => ResetReason::PowerOn,
        };
        let slot = dst.as_mut_ptr();
        // SAFETY: `slot` is valid, aligned, uninitialized storage for `Self`
        // and every field is written exactly once before the reference is
        // created.
        unsafe {
            use core::ptr::addr_of_mut;
            addr_of_mut!((*slot).link).write(Link::new(LinkConfig::ncp(), cookie.max(1)));
            addr_of_mut!((*slot).queues).write(EndpointQueues::new());
            // An all-zero byte array is a valid value.
            addr_of_mut!((*slot).tx).write_bytes(0, 1);
            addr_of_mut!((*slot).tx_len).write(0);
            addr_of_mut!((*slot).tx_off).write(0);
            addr_of_mut!((*slot).tx_progress_us).write(now_us);
            addr_of_mut!((*slot).need_delimiter).write(false);
            addr_of_mut!((*slot).identity).write(identity);
            addr_of_mut!((*slot).negotiated).write(false);
            addr_of_mut!((*slot).event_seq).write([0; Endpoint::COUNT]);
            addr_of_mut!((*slot).ops).write(Deque::new());
            addr_of_mut!((*slot).data_inflight).write(0);
            addr_of_mut!((*slot).key_export_locked).write(false);
            addr_of_mut!((*slot).pending_reset).write(None);
            addr_of_mut!((*slot).uart_switch).write(None);
            addr_of_mut!((*slot).counters).write(QueueCounters::default());
            addr_of_mut!((*slot).pending_diag).write(Deque::new());
            addr_of_mut!((*slot).reported_nwk_limit).write(0);
            addr_of_mut!((*slot).reported_aps_limit).write(0);
            addr_of_mut!((*slot).nwk_checkpoints).write(0);
            addr_of_mut!((*slot).indication_drops_unreported).write(0);
            addr_of_mut!((*slot).boot_us).write(now_us);
            let ncp = &mut *slot;
            ncp.link.open(now_us, reason);
            ncp
        }
    }
}
