//! Synchronous, `MacDriver`-independent local ZCL command dispatch.
//!
//! `ZigbeeDevice::process_incoming` is generic over the `M: MacDriver` radio
//! backend and is an `async fn`, so every byte of code inlined into it is
//! monomorphised into a large per-backend async state machine. The local
//! application-endpoint ZCL work — ZCL frame parse, foundation command
//! dispatch (Read/Write/Configure Reporting/Discover/…), cluster-specific
//! command handling, reporting configuration, the Groups → APS bridge and the
//! Finding & Binding Identify Query collection — is entirely *synchronous*: it
//! only mutates cluster/runtime-local state and enqueues responses into the
//! shared pending-response queue that the async layer already drains in
//! `flush_pending_responses`.
//!
//! This module hoists that synchronous engine out of the async, generic
//! receive path into a plain function that borrows only runtime-local,
//! `M`-independent state ([`LocalZclCtx`]). The two side effects that *do*
//! touch `M`-generic layer state — APS group-table updates and the F&B
//! Identify response list, both owned by the APS/BDB layers reachable only
//! through `ZigbeeDevice<M, _>` — are returned as a small
//! [`LocalZclOutcome`] action set for the async caller to apply. Nothing here
//! awaits, allocates, or is generic over `M`, so the ZCL engine is compiled
//! once instead of once per radio backend.

use zigbee_types::ShortAddress;
use zigbee_zcl::clusters::Cluster;
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::foundation::reporting::ReportingEngine;
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, ClusterId, CommandId, ZclStatus};

use crate::event_loop;
use crate::remote_reporting::{RecordOutcome, RemoteReportingState};
use crate::{
    ClusterRef, EndpointConfig, EndpointIdentifyCluster, PENDING_ZCL_DATA_CAP, PendingZclResponse,
};

/// Capacity of a cluster-specific command response payload.
///
/// Fixed by the [`Cluster::handle_command`] return type
/// (`heapless::Vec<u8, 64>`); `response_payload` below reuses this const, and
/// its `Some(resp)` assignment forces the two to stay equal (a wider
/// `handle_command` return would fail to type-check here, not silently grow).
const CLUSTER_RESPONSE_PAYLOAD_CAP: usize = 64;

/// Maximum ZCL header length of a cluster-specific response frame. Such frames
/// are built with [`ZclFrame::new_cluster_specific`], which never sets a
/// manufacturer code, so the header is frame-control + sequence + command = 3
/// bytes.
const CLUSTER_RESPONSE_HEADER_LEN: usize = 3;

// Proof that a cluster-specific response can never overflow a queued
// `PendingZclResponse`, so the drop-on-overflow branch of the shared
// `queue_frame` is unreachable for that path. Because the frame provably fits,
// no overflow ever occurs and the response is always queued whole. A future
// capacity bump on either side turns this into a compile error instead of
// silently changing behavior for larger cluster payloads. Locked at runtime by
// `cluster_specific_max_response_is_queued_whole`.
const _: () =
    assert!(CLUSTER_RESPONSE_HEADER_LEN + CLUSTER_RESPONSE_PAYLOAD_CAP <= PENDING_ZCL_DATA_CAP);

/// An APS group-table mutation requested by a Groups cluster command.
///
/// The Groups cluster's own state is synced synchronously inside the
/// dispatcher (it lives in the `ClusterRef` slice), but the mirrored APS group
/// table is owned by the `M`-generic APS layer, so the concrete request is
/// bubbled up for the async caller to apply through `apsme_*`.
pub(crate) enum GroupTableAction {
    Add { group: u16, endpoint: u8 },
    Remove { group: u16, endpoint: u8 },
    RemoveAll { endpoint: u8 },
}

/// Result of one synchronous local ZCL dispatch.
///
/// `event` is what `process_incoming` should return (`None` only when the ZCL
/// frame failed to parse). `group_action` and `fb_identify_target` are the
/// two `M`-generic side effects the async caller must apply against the
/// APS/BDB layers after the borrow of runtime-local state is released.
pub(crate) struct LocalZclOutcome {
    pub(crate) event: Option<event_loop::StackEvent>,
    pub(crate) group_action: Option<GroupTableAction>,
    pub(crate) fb_identify_target: Option<(u16, u8)>,
}

/// Borrowed, `M`-independent runtime state needed to dispatch a local ZCL
/// command and enqueue its response.
///
/// Every field is a plain runtime-local resource (endpoint config, standard
/// clusters, application clusters, the reporting engine, the pending-response
/// queue and a serialization scratch buffer) — none of them mention the radio
/// backend, so the dispatcher built on top compiles once regardless of `M`.
pub(crate) struct LocalZclCtx<'a, 'c, const N: usize> {
    endpoints: &'a [EndpointConfig],
    basic_cluster: &'a mut BasicCluster,
    identify_clusters: &'a mut [EndpointIdentifyCluster],
    reporting: &'a mut ReportingEngine,
    /// Interview record of clusters a remote client configured in full.
    ///
    /// Held next to, and kept strictly distinct from, `reporting`: the engine
    /// above also contains the product's own default configurations.
    remote_reporting: &'a mut RemoteReportingState,
    pending_responses: &'a mut heapless::Vec<PendingZclResponse, N>,
    clusters: &'a mut [ClusterRef<'c>],
    zcl_scratch: &'a mut [u8; 253],
    group_action: Option<GroupTableAction>,
    fb_identify_target: Option<(u16, u8)>,
}

impl<'a, 'c, const N: usize> LocalZclCtx<'a, 'c, N> {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        endpoints: &'a [EndpointConfig],
        basic_cluster: &'a mut BasicCluster,
        identify_clusters: &'a mut [EndpointIdentifyCluster],
        reporting: &'a mut ReportingEngine,
        remote_reporting: &'a mut RemoteReportingState,
        pending_responses: &'a mut heapless::Vec<PendingZclResponse, N>,
        clusters: &'a mut [ClusterRef<'c>],
        zcl_scratch: &'a mut [u8; 253],
    ) -> Self {
        Self {
            endpoints,
            basic_cluster,
            identify_clusters,
            reporting,
            remote_reporting,
            pending_responses,
            clusters,
            zcl_scratch,
            group_action: None,
            fb_identify_target: None,
        }
    }

    /// Run the synchronous local ZCL engine and package its outcome.
    ///
    /// `#[inline(never)]`: keeping this out of `process_incoming`'s async
    /// state machine is the whole point of the split. Inlined back, the ZCL
    /// engine bloats the ~40 KB receive future and is optimised as one
    /// oversized function; as a standalone, non-generic function it is compiled
    /// and optimised once regardless of the `MacDriver` backend, and the async
    /// receive future shrinks substantially (measured −15 KB on TLSR8258).
    #[inline(never)]
    pub(crate) fn dispatch(
        mut self,
        dst_ep: u8,
        src_endpoint: u8,
        cluster_id: u16,
        src_addr: u16,
        payload: &[u8],
    ) -> LocalZclOutcome {
        let event = self.dispatch_inner(dst_ep, src_endpoint, cluster_id, src_addr, payload);
        LocalZclOutcome {
            event,
            group_action: self.group_action,
            fb_identify_target: self.fb_identify_target,
        }
    }

    fn endpoint_has_server_cluster(&self, endpoint: u8, cluster_id: ClusterId) -> bool {
        self.endpoints.iter().any(|configured| {
            configured.endpoint == endpoint && configured.server_clusters.contains(&cluster_id)
        })
    }

    fn endpoint_has_client_cluster(&self, endpoint: u8, cluster_id: ClusterId) -> bool {
        self.endpoints.iter().any(|configured| {
            configured.endpoint == endpoint && configured.client_clusters.contains(&cluster_id)
        })
    }

    /// Resolve a server cluster on `endpoint` to a shared trait object.
    ///
    /// `#[inline(never)]` and deliberately non-generic: the endpoint/cluster
    /// scan is identical for every foundation handler, so compiling it once and
    /// letting the generic [`Self::with_cluster`] shell contribute only the
    /// per-callsite closure keeps the lookup out of each monomorphisation.
    #[inline(never)]
    fn resolve_cluster(&self, endpoint: u8, cluster_id: ClusterId) -> Option<&dyn Cluster> {
        if !self.endpoint_has_server_cluster(endpoint, cluster_id) {
            return None;
        }
        match cluster_id {
            ClusterId::BASIC => Some(&*self.basic_cluster),
            ClusterId::IDENTIFY => {
                let entry = self
                    .identify_clusters
                    .iter()
                    .find(|entry| entry.endpoint == endpoint)?;
                Some(&entry.cluster)
            }
            _ => {
                let cluster = self.clusters.iter().find(|cluster| {
                    cluster.endpoint == endpoint && cluster.cluster.cluster_id() == cluster_id
                })?;
                Some(&*cluster.cluster)
            }
        }
    }

    /// Mutable counterpart of [`Self::resolve_cluster`], shared by the write /
    /// cluster-specific handlers for the same monomorphisation reason.
    #[inline(never)]
    fn resolve_cluster_mut(
        &mut self,
        endpoint: u8,
        cluster_id: ClusterId,
    ) -> Option<&mut dyn Cluster> {
        if !self.endpoint_has_server_cluster(endpoint, cluster_id) {
            return None;
        }
        match cluster_id {
            ClusterId::BASIC => Some(&mut *self.basic_cluster),
            ClusterId::IDENTIFY => {
                let entry = self
                    .identify_clusters
                    .iter_mut()
                    .find(|entry| entry.endpoint == endpoint)?;
                Some(&mut entry.cluster)
            }
            _ => {
                let cluster = self.clusters.iter_mut().find(|cluster| {
                    cluster.endpoint == endpoint && cluster.cluster.cluster_id() == cluster_id
                })?;
                Some(&mut *cluster.cluster)
            }
        }
    }

    fn resolve_cluster_for_direction(
        &self,
        endpoint: u8,
        cluster_id: ClusterId,
        direction: ClusterDirection,
    ) -> Option<&dyn Cluster> {
        match direction {
            ClusterDirection::ClientToServer => self.resolve_cluster(endpoint, cluster_id),
            ClusterDirection::ServerToClient => {
                if !self.endpoint_has_client_cluster(endpoint, cluster_id) {
                    return None;
                }
                let cluster = self.clusters.iter().find(|cluster| {
                    cluster.endpoint == endpoint && cluster.cluster.cluster_id() == cluster_id
                })?;
                Some(&*cluster.cluster)
            }
        }
    }

    fn with_cluster<T>(
        &self,
        endpoint: u8,
        cluster_id: ClusterId,
        access: impl FnOnce(&dyn Cluster) -> T,
    ) -> Option<T> {
        self.resolve_cluster(endpoint, cluster_id).map(access)
    }

    fn with_cluster_mut<T>(
        &mut self,
        endpoint: u8,
        cluster_id: ClusterId,
        access: impl FnOnce(&mut dyn Cluster) -> T,
    ) -> Option<T> {
        self.resolve_cluster_mut(endpoint, cluster_id).map(access)
    }

    fn with_cluster_for_direction<T>(
        &self,
        endpoint: u8,
        cluster_id: ClusterId,
        direction: ClusterDirection,
        access: impl FnOnce(&dyn Cluster) -> T,
    ) -> Option<T> {
        self.resolve_cluster_for_direction(endpoint, cluster_id, direction)
            .map(access)
    }

    fn dispatch_inner(
        &mut self,
        dst_ep: u8,
        src_endpoint: u8,
        cluster_id: u16,
        src_addr: u16,
        payload: &[u8],
    ) -> Option<event_loop::StackEvent> {
        // Application endpoint — parse ZCL frame
        rt_trace!(
            "[RT] zcl ep={} cluster=0x{:04X} from=0x{:04X} len={}",
            dst_ep,
            cluster_id,
            src_addr,
            payload.len()
        );
        log::info!(
            "[Runtime] ZCL frame: ep={} cluster=0x{:04X} from 0x{:04X} len={}",
            dst_ep,
            cluster_id,
            src_addr,
            payload.len()
        );
        let zcl_frame = match ZclFrame::parse(payload) {
            Ok(f) => f,
            Err(_) => {
                log::warn!("[Runtime] Failed to parse ZCL frame on ep {}", dst_ep);
                return None;
            }
        };

        let cmd_id = zcl_frame.header.command_id.0;
        rt_trace!(
            "[RT] zcl_cmd ep={} cluster=0x{:04X} cmd=0x{:02X} seq={} dir={:?} payload={}",
            dst_ep,
            cluster_id,
            cmd_id,
            zcl_frame.header.seq_number,
            zcl_frame.header.direction(),
            zcl_frame.payload.len(),
        );

        // Check if this is a Report Attributes (0x0A) — incoming report from remote
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x0A
        {
            return Some(event_loop::StackEvent::AttributeReport {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                attr_id: if payload.len() >= 5 {
                    u16::from_le_bytes([payload[3], payload[4]])
                } else {
                    0
                },
            });
        }

        // Check if this is a Default Response (0x0B) — received from remote
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x0B
        {
            let (resp_cmd, resp_status) = if zcl_frame.payload.len() >= 2 {
                (zcl_frame.payload[0], zcl_frame.payload[1])
            } else {
                (0, 0)
            };
            log::debug!(
                "[Runtime] Default Response for cmd 0x{:02X} status=0x{:02X} from 0x{:04X}",
                resp_cmd,
                resp_status,
                src_addr,
            );
            return Some(event_loop::StackEvent::DefaultResponse {
                src_addr,
                endpoint: dst_ep,
                cluster_id,
                command_id: resp_cmd,
                status: resp_status,
            });
        }

        // Check if this is Configure Reporting (0x06) — a remote ZCL client
        // (typically the coordinator, during its interview) configuring the
        // reports this device should send.
        //
        // Two independent results come out of this branch. The per-record
        // status list drives the standards-mandated Configure Reporting
        // Response (0x07) and is unchanged. Separately, the cluster is added
        // to the runtime's remote-reporting record *only* when the whole
        // command was well formed and every record succeeded — see
        // [`crate::remote_reporting`]. Anything less (empty or malformed
        // payload, unsupported attribute, unreportable attribute,
        // invalid/disabled data type, reporting-table capacity failure) is
        // reported to the client but never counts towards interview
        // completion.
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x06
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            use zigbee_zcl::foundation::reporting::{
                ConfigureReportingResponse, ConfigureReportingStatusRecord, ReportDirection,
                ReportingConfig,
            };
            let payload = zcl_frame.payload.as_slice();
            let mut response = ConfigureReportingResponse {
                records: heapless::Vec::new(),
            };
            let mut i = 0usize;
            let mut records = 0usize;
            let mut parse_ok = true;
            let mut all_records_succeeded = true;
            // Outbound (device→client) reporting progress may only be advanced
            // by a command made *entirely* of Send-direction records. A
            // Receive-direction record configures how *this* device consumes a
            // client's reports, never what it sends, so a receive-only or
            // mixed command must not count towards interview completion even if
            // every record is individually accepted.
            let mut all_records_send = true;
            rt_trace!(
                "[RT] zcl_cfg_reporting ep={} cluster=0x{:04X} len={}",
                dst_ep,
                cluster_id,
                payload.len(),
            );

            while i < payload.len() {
                let direction = match payload[i] {
                    0x00 => ReportDirection::Send,
                    0x01 => ReportDirection::Receive,
                    _other => {
                        rt_trace!("[RT] zcl_cfg bad_dir=0x{:02X}", _other);
                        parse_ok = false;
                        break;
                    }
                };
                if direction == ReportDirection::Receive {
                    all_records_send = false;
                }
                i += 1;
                if i + 2 > payload.len() {
                    parse_ok = false;
                    break;
                }
                let attribute_id =
                    zigbee_zcl::AttributeId(u16::from_le_bytes([payload[i], payload[i + 1]]));
                i += 2;

                let cfg = if direction == ReportDirection::Send {
                    if i + 5 > payload.len() {
                        parse_ok = false;
                        break;
                    }
                    let Some(data_type) = zigbee_zcl::data_types::ZclDataType::from_u8(payload[i])
                    else {
                        rt_trace!("[RT] zcl_cfg bad_type=0x{:02X}", payload[i]);
                        parse_ok = false;
                        break;
                    };
                    i += 1;
                    let min_interval = u16::from_le_bytes([payload[i], payload[i + 1]]);
                    i += 2;
                    let max_interval = u16::from_le_bytes([payload[i], payload[i + 1]]);
                    i += 2;
                    let data_type_enabled = zigbee_zcl::data_types::is_data_type_enabled(data_type);
                    let reportable_change = if zigbee_zcl::data_types::is_analog_type(data_type) {
                        if data_type_enabled {
                            let Some((val, consumed)) =
                                zigbee_zcl::data_types::ZclValue::deserialize(
                                    data_type,
                                    &payload[i..],
                                )
                            else {
                                parse_ok = false;
                                break;
                            };
                            i += consumed;
                            Some(val)
                        } else {
                            let Some(value_size) =
                                zigbee_zcl::data_types::data_type_size(data_type)
                            else {
                                parse_ok = false;
                                break;
                            };
                            if i + value_size > payload.len() {
                                parse_ok = false;
                                break;
                            }
                            i += value_size;
                            None
                        }
                    } else {
                        None
                    };
                    ReportingConfig {
                        direction,
                        attribute_id,
                        data_type,
                        min_interval,
                        max_interval,
                        reportable_change,
                    }
                } else {
                    if i + 2 > payload.len() {
                        parse_ok = false;
                        break;
                    }
                    let timeout = u16::from_le_bytes([payload[i], payload[i + 1]]);
                    i += 2;
                    ReportingConfig {
                        direction,
                        attribute_id,
                        data_type: zigbee_zcl::data_types::ZclDataType::NoData,
                        min_interval: 0,
                        max_interval: timeout,
                        reportable_change: None,
                    }
                };

                let attr_definition = self
                    .with_cluster(dst_ep, ClusterId(cluster_id), |cluster| {
                        cluster
                            .attributes()
                            .find(cfg.attribute_id)
                            .map(|definition| (definition.access, definition.data_type))
                    })
                    .flatten();
                let status = if !zigbee_zcl::data_types::is_data_type_enabled(cfg.data_type) {
                    ZclStatus::InvalidDataType
                } else if let Some((access, attribute_type)) = attr_definition {
                    if cfg.direction == ReportDirection::Send && cfg.data_type != attribute_type {
                        ZclStatus::InvalidDataType
                    } else if cfg.direction == ReportDirection::Send && !access.is_reportable() {
                        ZclStatus::UnreportableAttribute
                    } else {
                        match self
                            .reporting
                            .configure_for_cluster(dst_ep, cluster_id, cfg.clone())
                        {
                            Ok(()) => ZclStatus::Success,
                            Err(s) => s,
                        }
                    }
                } else {
                    ZclStatus::UnsupportedAttribute
                };
                if status != ZclStatus::Success {
                    all_records_succeeded = false;
                }
                let _ = response.records.push(ConfigureReportingStatusRecord {
                    status,
                    direction: cfg.direction,
                    attribute_id: cfg.attribute_id,
                });
                records += 1;
                rt_trace!(
                    "[RT] zcl_cfg attr=0x{:04X} dir={} status=0x{:02X}",
                    cfg.attribute_id.0,
                    cfg.direction as u8,
                    status as u8,
                );
            }

            if parse_ok && records > 0 {
                // Queue Configure Reporting Response (0x07)
                queue_reporting_response(
                    self.pending_responses,
                    ShortAddress(src_addr),
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    &response,
                );
                log::info!(
                    "[Runtime] Configure Reporting: ep={} cluster=0x{:04X} ({} attrs)",
                    dst_ep,
                    cluster_id,
                    records
                );
            } else {
                rt_trace!(
                    "[RT] zcl_cfg_reporting parse_fail ep={} cluster=0x{:04X} len={}",
                    dst_ep,
                    cluster_id,
                    payload.len(),
                );
            }

            // Interview accounting: only a well-formed, non-empty command made
            // entirely of Send-direction records whose statuses were all
            // `Success` advances outbound reporting progress. A partially
            // rejected command (the client did not get what it asked for) or a
            // receive-only/mixed command (which does not configure what this
            // device *sends*) is answered but never counts.
            if parse_ok && records > 0 && all_records_succeeded && all_records_send {
                match self.remote_reporting.record(dst_ep, cluster_id) {
                    RecordOutcome::Added | RecordOutcome::AlreadyRecorded => {}
                    RecordOutcome::Full => {
                        // Never a silent success: the cluster genuinely is not
                        // tracked, so say so rather than let an application
                        // wait on a count that can no longer grow.
                        log::warn!(
                            "[Runtime] Remote reporting record full; ep={} cluster=0x{:04X} not tracked",
                            dst_ep,
                            cluster_id
                        );
                    }
                }
                let configured_clusters = self.remote_reporting.cluster_count(dst_ep);
                rt_trace!(
                    "[RT] zcl_cfg_reporting remote_ok ep={} cluster=0x{:04X} clusters={}",
                    dst_ep,
                    cluster_id,
                    configured_clusters,
                );
                return Some(event_loop::StackEvent::ReportingConfigured {
                    src_addr,
                    source_endpoint: src_endpoint,
                    endpoint: dst_ep,
                    cluster_id,
                    configured_clusters,
                });
            }

            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // Check if this is Read Reporting Config (0x08)
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x08
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            use zigbee_zcl::foundation::reporting::{
                ReadReportingConfigRequest, ReadReportingConfigResponse,
                ReadReportingConfigResponseRecord,
            };
            if let Some(req) = ReadReportingConfigRequest::parse(zcl_frame.payload.as_slice()) {
                let mut response = ReadReportingConfigResponse {
                    records: heapless::Vec::new(),
                };
                for rec in &req.records {
                    if let Some(cfg) = self.reporting.get_config(
                        dst_ep,
                        cluster_id,
                        rec.direction,
                        rec.attribute_id,
                    ) {
                        if rec.direction == zigbee_zcl::foundation::reporting::ReportDirection::Send
                        {
                            let _ = response.records.push(ReadReportingConfigResponseRecord {
                                status: ZclStatus::Success,
                                direction: rec.direction,
                                attribute_id: rec.attribute_id,
                                config: Some(cfg.clone()),
                                timeout: None,
                            });
                        } else {
                            // Receive direction: return timeout only
                            let _ = response.records.push(ReadReportingConfigResponseRecord {
                                status: ZclStatus::Success,
                                direction: rec.direction,
                                attribute_id: rec.attribute_id,
                                config: None,
                                timeout: Some(cfg.max_interval),
                            });
                        }
                    } else {
                        let _ = response.records.push(ReadReportingConfigResponseRecord {
                            status: ZclStatus::UnsupportedAttribute,
                            direction: rec.direction,
                            attribute_id: rec.attribute_id,
                            config: None,
                            timeout: None,
                        });
                    }
                }
                queue_read_reporting_response(
                    self.pending_responses,
                    ShortAddress(src_addr),
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    &response,
                );
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Read Attributes (0x00) ──────────────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x00
        {
            let request_direction = zcl_frame.header.direction();
            if let Some(req) = zigbee_zcl::foundation::read_attributes::ReadAttributesRequest::parse(
                zcl_frame.payload.as_slice(),
            ) {
                rt_trace!(
                    "[RT] zcl_read ep={} cluster=0x{:04X} attrs={} from=0x{:04X}",
                    dst_ep,
                    cluster_id,
                    req.attributes.len(),
                    src_addr,
                );
                log::info!(
                    "[ZCL] ReadAttr ep={} cluster=0x{:04X} attrs={} from 0x{:04X}",
                    dst_ep,
                    cluster_id,
                    req.attributes.len(),
                    src_addr,
                );
                // Find the cluster's attribute store
                if let Some(response) = self.with_cluster_for_direction(
                    dst_ep,
                    ClusterId(cluster_id),
                    request_direction,
                    |cluster| {
                        zigbee_zcl::foundation::read_attributes::process_read_dyn(
                            cluster.attributes(),
                            &req,
                        )
                    },
                ) {
                    let payload_buf = &mut *self.zcl_scratch;
                    let payload_len = response.serialize(payload_buf).min(payload_buf.len());
                    rt_trace!(
                        "[RT] zcl_read_rsp cluster=0x{:04X} len={} records={}",
                        cluster_id,
                        payload_len,
                        response.records.len(),
                    );
                    log::info!(
                        "[ZCL] ReadAttr response: {} bytes, {} records queued",
                        payload_len,
                        response.records.len(),
                    );
                    queue_global_response_for_direction_inner(
                        self.pending_responses,
                        src_addr,
                        src_endpoint,
                        dst_ep,
                        cluster_id,
                        zcl_frame.header.seq_number,
                        0x01, // Read Attributes Response
                        match request_direction {
                            ClusterDirection::ClientToServer => ClusterDirection::ServerToClient,
                            ClusterDirection::ServerToClient => ClusterDirection::ClientToServer,
                        },
                        &payload_buf[..payload_len],
                    );
                } else {
                    rt_trace!(
                        "[RT] zcl_read no_cluster ep={} cluster=0x{:04X} have={}",
                        dst_ep,
                        cluster_id,
                        self.clusters.len(),
                    );
                    log::warn!(
                        "[ZCL] ReadAttr: no cluster found for ep={} cluster=0x{:04X} (have {} clusters)",
                        dst_ep,
                        cluster_id,
                        self.clusters.len(),
                    );
                }
            } else {
                rt_trace!(
                    "[RT] zcl_read parse_fail ep={} cluster=0x{:04X} len={}",
                    dst_ep,
                    cluster_id,
                    zcl_frame.payload.len(),
                );
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Write Attributes (0x02) ─────────────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x02
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            match zigbee_zcl::foundation::write_attributes::WriteAttributesRequest::parse_checked(
                zcl_frame.payload.as_slice(),
            ) {
                Ok(outcome) => {
                    if let Some(applied) =
                        self.with_cluster_mut(dst_ep, ClusterId(cluster_id), |cluster| {
                            zigbee_zcl::foundation::write_attributes::process_write_dyn(
                                cluster.attributes_mut(),
                                &outcome.request,
                            )
                        })
                    {
                        // Splice the disabled-data-type failures back among the
                        // applied statuses in exact request order.
                        let response = outcome.merge_in_request_order(&applied);
                        let payload_buf = &mut *self.zcl_scratch;
                        let payload_len = response.serialize(payload_buf);
                        queue_global_response_inner(
                            self.pending_responses,
                            src_addr,
                            src_endpoint,
                            dst_ep,
                            cluster_id,
                            zcl_frame.header.seq_number,
                            0x04,
                            &payload_buf[..payload_len],
                        );
                    }
                }
                Err(
                    zigbee_zcl::foundation::write_attributes::WriteAttributesParseError::Malformed,
                ) => {}
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Write Attributes Undivided (0x03) ────────────────────
        // All-or-nothing: if any attribute fails, none are written.
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x03
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            match zigbee_zcl::foundation::write_attributes::WriteAttributesRequest::parse_checked(
                zcl_frame.payload.as_slice(),
            ) {
                Ok(outcome) => {
                    let has_invalid_data_types = !outcome.invalid_data_types.is_empty();
                    if let Some(applied) =
                        self.with_cluster_mut(dst_ep, ClusterId(cluster_id), |cluster| {
                            if has_invalid_data_types {
                                zigbee_zcl::foundation::write_attributes::validate_write_undivided_dyn(
                                    cluster.attributes_mut(),
                                    &outcome.request,
                                )
                            } else {
                                zigbee_zcl::foundation::write_attributes::process_write_undivided_dyn(
                                    cluster.attributes_mut(),
                                    &outcome.request,
                                )
                            }
                        })
                    {
                        // A disabled data type makes the whole undivided write
                        // fail atomically (validate-only above), so merge its
                        // failure with the per-record statuses in request order.
                        let response = outcome.merge_in_request_order(&applied);
                        let payload_buf = &mut *self.zcl_scratch;
                        let payload_len = response.serialize(payload_buf);
                        queue_global_response_inner(
                            self.pending_responses,
                            src_addr,
                            src_endpoint,
                            dst_ep,
                            cluster_id,
                            zcl_frame.header.seq_number,
                            0x04,
                            &payload_buf[..payload_len],
                        );
                    }
                }
                Err(
                    zigbee_zcl::foundation::write_attributes::WriteAttributesParseError::Malformed,
                ) => {}
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Write Attributes No Response (0x05) ─────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x05
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            if let Ok(outcome) =
                zigbee_zcl::foundation::write_attributes::WriteAttributesRequest::parse_checked(
                    zcl_frame.payload.as_slice(),
                )
                && self
                    .with_cluster_mut(dst_ep, ClusterId(cluster_id), |cluster| {
                        zigbee_zcl::foundation::write_attributes::process_write_no_response_dyn(
                            cluster.attributes_mut(),
                            &outcome.request,
                        )
                    })
                    .is_some()
            {
                // No response sent for 0x05
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Discover Attributes (0x0C) ──────────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x0C
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverAttributesRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(response) =
                self.with_cluster(dst_ep, ClusterId(cluster_id), |cluster| {
                    zigbee_zcl::foundation::discover::process_discover_dyn(
                        cluster.attributes(),
                        &req,
                    )
                })
            {
                let payload_buf = &mut *self.zcl_scratch;
                let payload_len = response.serialize(payload_buf);
                queue_global_response_inner(
                    self.pending_responses,
                    src_addr,
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x0D, // Discover Attributes Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Discover Commands Received (0x11) / Generated (0x13) ─
        // Identical apart from the command list source and the response
        // command id, so they share one handler to avoid emitting the
        // parse → collect → serialize → queue scaffolding (and the
        // `with_cluster` closure) twice.
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && (cmd_id == 0x11 || cmd_id == 0x13)
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverCommandsRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(all) = self.with_cluster(dst_ep, ClusterId(cluster_id), |cluster| {
                if cmd_id == 0x11 {
                    cluster.received_commands()
                } else {
                    cluster.generated_commands()
                }
            }) {
                let response = zigbee_zcl::foundation::discover::process_discover_commands(
                    &all,
                    req.start_command_id,
                    req.max_results,
                );
                let payload_buf = &mut *self.zcl_scratch;
                let payload_len = response.serialize(payload_buf);
                // 0x11 → 0x12 Received Response, 0x13 → 0x14 Generated Response.
                let response_cmd = if cmd_id == 0x11 { 0x12 } else { 0x14 };
                queue_global_response_inner(
                    self.pending_responses,
                    src_addr,
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    response_cmd,
                    &payload_buf[..payload_len],
                );
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Discover Attributes Extended (0x15) ─────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::Global
            && cmd_id == 0x15
            && zcl_frame.header.direction() == ClusterDirection::ClientToServer
        {
            if let Some(req) = zigbee_zcl::foundation::discover::DiscoverAttributesRequest::parse(
                zcl_frame.payload.as_slice(),
            ) && let Some(response) =
                self.with_cluster(dst_ep, ClusterId(cluster_id), |cluster| {
                    zigbee_zcl::foundation::discover::process_discover_extended_dyn(
                        cluster.attributes(),
                        &req,
                    )
                })
            {
                let payload_buf = &mut *self.zcl_scratch;
                let payload_len = response.serialize(payload_buf);
                queue_global_response_inner(
                    self.pending_responses,
                    src_addr,
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    0x16, // Discover Attributes Extended Response
                    &payload_buf[..payload_len],
                );
            }
            return Some(command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // ── Cluster-specific command dispatch ────────────────────
        if zcl_frame.header.frame_type() == zigbee_zcl::frame::ZclFrameType::ClusterSpecific {
            // Intercept Identify Query Response (cluster 0x0003, cmd 0x00, server→client)
            // for F&B initiator target collection
            if cluster_id == ClusterId::IDENTIFY.0
                && cmd_id == zigbee_zcl::clusters::identify::CMD_IDENTIFY_QUERY_RESPONSE.0
                && zcl_frame.header.direction() == ClusterDirection::ServerToClient
            {
                self.fb_identify_target = Some((src_addr, src_endpoint));
                log::debug!(
                    "[Runtime] F&B: Identify Query Response from 0x{:04X} ep {}",
                    src_addr,
                    src_endpoint,
                );
            }

            if zcl_frame.header.direction() == ClusterDirection::ServerToClient {
                return Some(cluster_command_received_event(
                    src_addr,
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    cmd_id,
                    zcl_frame.header.seq_number,
                    zcl_frame.payload.as_slice(),
                ));
            }

            let mut cmd_status = ZclStatus::Success;
            let mut response_payload: Option<heapless::Vec<u8, CLUSTER_RESPONSE_PAYLOAD_CAP>> =
                None;
            let mut cluster_found = false;

            if let Some(result) = self.with_cluster_mut(dst_ep, ClusterId(cluster_id), |cluster| {
                cluster.handle_command(CommandId(cmd_id), zcl_frame.payload.as_slice())
            }) {
                cluster_found = true;
                match result {
                    Ok(resp) => {
                        response_payload = if resp.is_empty() { None } else { Some(resp) };
                    }
                    Err(status) => {
                        cmd_status = status;
                    }
                }

                // Groups cluster → APS group table bridge
                if cluster_id == ClusterId::GROUPS.0 {
                    // Parse group action from command ID and sync to APS table.
                    // Can't use GroupsCluster::take_action() through trait object,
                    // so we infer the action from the ZCL command directly.
                    match cmd_id {
                        command
                            if command == zigbee_zcl::clusters::groups::CMD_ADD_GROUP.0
                                && zcl_frame.payload.len() >= 2 =>
                        {
                            // Add Group — group_id is first 2 bytes of payload
                            let gid =
                                u16::from_le_bytes([zcl_frame.payload[0], zcl_frame.payload[1]]);
                            self.group_action = Some(GroupTableAction::Add {
                                group: gid,
                                endpoint: dst_ep,
                            });
                        }
                        command
                            if command == zigbee_zcl::clusters::groups::CMD_REMOVE_GROUP.0
                                && zcl_frame.payload.len() >= 2 =>
                        {
                            // Remove Group — group_id is first 2 bytes
                            let gid =
                                u16::from_le_bytes([zcl_frame.payload[0], zcl_frame.payload[1]]);
                            self.group_action = Some(GroupTableAction::Remove {
                                group: gid,
                                endpoint: dst_ep,
                            });
                        }
                        command
                            if command == zigbee_zcl::clusters::groups::CMD_REMOVE_ALL_GROUPS.0 =>
                        {
                            // Remove All Groups
                            self.group_action =
                                Some(GroupTableAction::RemoveAll { endpoint: dst_ep });
                        }
                        command
                            if command
                                == zigbee_zcl::clusters::groups::CMD_ADD_GROUP_IF_IDENTIFYING.0
                                && zcl_frame.payload.len() >= 2 =>
                        {
                            // Add Group If Identifying — only add if Identify cluster
                            // on this endpoint has IdentifyTime > 0
                            let gid =
                                u16::from_le_bytes([zcl_frame.payload[0], zcl_frame.payload[1]]);
                            let is_identifying = self
                                .with_cluster(dst_ep, ClusterId::IDENTIFY, |cluster| {
                                    cluster
                                        .attributes()
                                        .get(zigbee_zcl::AttributeId(0x0000))
                                        .map(|value| {
                                            matches!(
                                                value,
                                                zigbee_zcl::data_types::ZclValue::U16(time)
                                                    if *time > 0
                                            )
                                        })
                                        .unwrap_or(false)
                                })
                                .unwrap_or(false);
                            if is_identifying {
                                // Add to APS group table
                                self.group_action = Some(GroupTableAction::Add {
                                    group: gid,
                                    endpoint: dst_ep,
                                });
                                // Also add to GroupsCluster internal list via CMD_ADD_GROUP
                                // (cluster's handle_command for 0x05 is a no-op; use 0x00 to sync)
                                let add_payload = gid.to_le_bytes();
                                let _ =
                                    self.with_cluster_mut(dst_ep, ClusterId::GROUPS, |cluster| {
                                        cluster.handle_command(
                                            zigbee_zcl::clusters::groups::CMD_ADD_GROUP,
                                            &add_payload,
                                        )
                                    });
                            }
                        }
                        _ => {}
                    }
                }
            }

            // Send cluster-specific response if the cluster produced one
            if let Some(resp) = response_payload {
                // Determine the response command ID.
                // For most clusters, the response uses the same cmd_id.
                // Exceptions per ZCL spec:
                // - Identify Query (0x01) → IdentifyQueryResponse (0x00)
                let response_cmd_id = if cluster_id == ClusterId::IDENTIFY.0
                    && cmd_id == zigbee_zcl::clusters::identify::CMD_IDENTIFY_QUERY.0
                {
                    zigbee_zcl::clusters::identify::CMD_IDENTIFY_QUERY_RESPONSE.0
                } else {
                    cmd_id
                };
                let mut frame = ZclFrame::new_cluster_specific(
                    zcl_frame.header.seq_number,
                    CommandId(response_cmd_id),
                    ClusterDirection::ServerToClient,
                    true,
                );
                for &b in resp.as_slice() {
                    let _ = frame.payload.push(b);
                }
                queue_frame(
                    self.pending_responses,
                    ShortAddress(src_addr),
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    &frame,
                    // Provably never overflows (see the module-level `const _`
                    // bound), so it is always queued whole.
                );
            } else if cluster_found && !zcl_frame.header.disable_default_response() {
                // Only send Default Response for clusters we handle in ClusterRef.
                // Unmatched clusters (e.g. OTA 0x0019) are app-handled — don't
                // send spurious Default Responses that confuse the coordinator.
                queue_default_response(
                    self.pending_responses,
                    ShortAddress(src_addr),
                    src_endpoint,
                    dst_ep,
                    cluster_id,
                    zcl_frame.header.seq_number,
                    cmd_id,
                    cmd_status,
                    zcl_frame.header.direction(),
                );
            }

            // Basic cluster factory reset → distinct event
            if cluster_id == ClusterId::BASIC.0
                && cmd_id == zigbee_zcl::clusters::basic::CMD_RESET_TO_FACTORY_DEFAULTS.0
                && cluster_found
                && cmd_status == ZclStatus::Success
                && zcl_frame.header.direction() == ClusterDirection::ClientToServer
            {
                return Some(event_loop::StackEvent::FactoryResetRequested);
            }

            return Some(cluster_command_received_event(
                src_addr,
                src_endpoint,
                dst_ep,
                cluster_id,
                cmd_id,
                zcl_frame.header.seq_number,
                zcl_frame.payload.as_slice(),
            ));
        }

        // Other global commands — send Default Response for unsupported, then pass through
        if !zcl_frame.header.disable_default_response() {
            // Send UNSUP_GENERAL_COMMAND for unhandled foundation commands
            queue_default_response(
                self.pending_responses,
                ShortAddress(src_addr),
                src_endpoint,
                dst_ep,
                cluster_id,
                zcl_frame.header.seq_number,
                cmd_id,
                ZclStatus::UnsupGeneralCommand,
                zcl_frame.header.direction(),
            );
        }
        Some(command_received_event(
            src_addr,
            src_endpoint,
            dst_ep,
            cluster_id,
            cmd_id,
            zcl_frame.header.seq_number,
            zcl_frame.payload.as_slice(),
        ))
    }
}

/// Build the `CommandReceived` pass-through event for a foundation command.
///
fn command_received_event(
    src_addr: u16,
    src_endpoint: u8,
    dst_ep: u8,
    cluster_id: u16,
    cmd_id: u8,
    seq_number: u8,
    payload: &[u8],
) -> event_loop::StackEvent {
    command_received_event_with_type(
        src_addr,
        src_endpoint,
        dst_ep,
        cluster_id,
        cmd_id,
        seq_number,
        payload,
        zigbee_zcl::frame::ZclFrameType::Global,
    )
}

/// Build the same pass-through event for a cluster-specific command.
fn cluster_command_received_event(
    src_addr: u16,
    src_endpoint: u8,
    dst_ep: u8,
    cluster_id: u16,
    cmd_id: u8,
    seq_number: u8,
    payload: &[u8],
) -> event_loop::StackEvent {
    command_received_event_with_type(
        src_addr,
        src_endpoint,
        dst_ep,
        cluster_id,
        cmd_id,
        seq_number,
        payload,
        zigbee_zcl::frame::ZclFrameType::ClusterSpecific,
    )
}

/// Keep the bounded payload copy and event construction out of each dispatch arm.
#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn command_received_event_with_type(
    src_addr: u16,
    src_endpoint: u8,
    dst_ep: u8,
    cluster_id: u16,
    cmd_id: u8,
    seq_number: u8,
    payload: &[u8],
    frame_type: zigbee_zcl::frame::ZclFrameType,
) -> event_loop::StackEvent {
    event_loop::StackEvent::CommandReceived {
        src_addr,
        source_endpoint: src_endpoint,
        endpoint: dst_ep,
        cluster_id,
        frame_type,
        command_id: cmd_id,
        seq_number,
        payload: heapless::Vec::from_slice(payload).unwrap_or_default(),
    }
}

/// Serialize a built ZCL response frame and enqueue it for the next tick.
///
/// The foundation queue helpers below differ only in how they build the
/// outgoing [`ZclFrame`]; the serialize → bounded copy → `pending_responses`
/// push tail is identical, so it lives here exactly once (generic only over the
/// queue depth `N`).
///
/// If the serialized frame does not fit the [`PENDING_ZCL_DATA_CAP`]-byte
/// `PendingZclResponse` buffer, the whole frame is dropped (`return`). A
/// truncated frame would be a malformed ZCL response, so it is dropped rather
/// than sent corrupt. This matches every reachable historical foundation
/// helper:
///
/// * The global-response path (e.g. Read Attributes) is caller-sized and can
///   genuinely overflow; historically it serialized into a scratch buffer and
///   returned without queueing when the copy into the `PendingZclResponse`
///   buffer overflowed.
/// * The Read Reporting Configuration response path historically serialized
///   into a 128-byte buffer, so a maximum-size response (3 header bytes + a
///   128-byte payload = 131 bytes) failed serialization and queued nothing —
///   i.e. it dropped (regression: `read_reporting_max_response_is_dropped`).
/// * The default / configure-reporting / cluster-specific paths build frames
///   that are provably small enough that the drop branch never runs (see the
///   module-level `const _` assertion for the cluster-specific bound).
#[inline(never)]
fn queue_frame<const N: usize>(
    pending_responses: &mut heapless::Vec<PendingZclResponse, N>,
    dst_addr: ShortAddress,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    frame: &ZclFrame,
) {
    let mut zcl_buf = [0u8; 256];
    let Ok(len) = frame.serialize(&mut zcl_buf) else {
        rt_trace!("[RT] zcl_queue serialize_fail cluster=0x{:04X}", cluster_id);
        return;
    };
    let mut data = heapless::Vec::new();
    for &b in &zcl_buf[..len] {
        if data.push(b).is_err() {
            rt_trace!(
                "[RT] zcl_queue frame_dropped cluster=0x{:04X} len={} cap={}",
                cluster_id,
                len,
                data.capacity(),
            );
            return;
        }
    }
    if pending_responses
        .push(PendingZclResponse {
            dst_addr,
            dst_endpoint,
            src_endpoint,
            cluster_id,
            zcl_data: data,
        })
        .is_err()
    {
        log::warn!("[ZCL] Response queue full");
    }
}

/// Queue a ZCL global command response for sending in the next tick.
///
/// Used for Read Attributes (0x00→0x01), Write Attributes (0x02→0x04),
/// Discover (0x0C→0x0D) and friends.
#[allow(clippy::too_many_arguments)]
pub(crate) fn queue_global_response_inner<const N: usize>(
    pending_responses: &mut heapless::Vec<PendingZclResponse, N>,
    dst_addr: u16,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    seq: u8,
    response_cmd: u8,
    payload: &[u8],
) {
    queue_global_response_for_direction_inner(
        pending_responses,
        dst_addr,
        dst_endpoint,
        src_endpoint,
        cluster_id,
        seq,
        response_cmd,
        ClusterDirection::ServerToClient,
        payload,
    );
}

#[allow(clippy::too_many_arguments)]
fn queue_global_response_for_direction_inner<const N: usize>(
    pending_responses: &mut heapless::Vec<PendingZclResponse, N>,
    dst_addr: u16,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    seq: u8,
    response_cmd: u8,
    direction: ClusterDirection,
    payload: &[u8],
) {
    let mut frame = ZclFrame::new_global(seq, CommandId(response_cmd), direction, true);
    for &b in payload {
        if frame.payload.push(b).is_err() {
            rt_trace!(
                "[RT] zcl_queue payload_truncated cluster=0x{:04X} cap={}",
                cluster_id,
                frame.payload.capacity(),
            );
            break;
        }
    }
    queue_frame(
        pending_responses,
        ShortAddress(dst_addr),
        dst_endpoint,
        src_endpoint,
        cluster_id,
        &frame,
    );
}

/// Queue a ZCL Default Response to be sent in next tick().
#[allow(clippy::too_many_arguments)]
pub(crate) fn queue_default_response<const N: usize>(
    pending_responses: &mut heapless::Vec<PendingZclResponse, N>,
    dst_addr: ShortAddress,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    seq: u8,
    triggering_cmd: u8,
    status: ZclStatus,
    triggering_direction: ClusterDirection,
) {
    let response_direction = match triggering_direction {
        ClusterDirection::ClientToServer => ClusterDirection::ServerToClient,
        ClusterDirection::ServerToClient => ClusterDirection::ClientToServer,
    };
    let mut frame = ZclFrame::new_global(
        seq,
        CommandId(0x0B), // Default Response
        response_direction,
        true,
    );
    let _ = frame.payload.push(triggering_cmd);
    let _ = frame.payload.push(status as u8);
    queue_frame(
        pending_responses,
        dst_addr,
        dst_endpoint,
        src_endpoint,
        cluster_id,
        &frame,
    );
}

/// Queue a Configure Reporting Response (0x07).
pub(crate) fn queue_reporting_response<const N: usize>(
    pending_responses: &mut heapless::Vec<PendingZclResponse, N>,
    dst_addr: ShortAddress,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    seq: u8,
    response: &zigbee_zcl::foundation::reporting::ConfigureReportingResponse,
) {
    let mut frame =
        ZclFrame::new_global(seq, CommandId(0x07), ClusterDirection::ServerToClient, true);
    let mut payload_buf = [0u8; 64];
    let payload_len = response.serialize(&mut payload_buf);
    for &b in &payload_buf[..payload_len] {
        let _ = frame.payload.push(b);
    }
    queue_frame(
        pending_responses,
        dst_addr,
        dst_endpoint,
        src_endpoint,
        cluster_id,
        &frame,
    );
}

/// Queue a Read Reporting Configuration Response (0x09).
pub(crate) fn queue_read_reporting_response<const N: usize>(
    pending_responses: &mut heapless::Vec<PendingZclResponse, N>,
    dst_addr: ShortAddress,
    dst_endpoint: u8,
    src_endpoint: u8,
    cluster_id: u16,
    seq: u8,
    response: &zigbee_zcl::foundation::reporting::ReadReportingConfigResponse,
) {
    let mut frame =
        ZclFrame::new_global(seq, CommandId(0x09), ClusterDirection::ServerToClient, true);
    let mut payload_buf = [0u8; 128];
    let payload_len = response.serialize(&mut payload_buf);
    for &b in &payload_buf[..payload_len] {
        let _ = frame.payload.push(b);
    }
    queue_frame(
        pending_responses,
        dst_addr,
        dst_endpoint,
        src_endpoint,
        cluster_id,
        &frame,
        // A maximum Read Reporting Configuration Response is 3 header bytes + a
        // 128-byte payload = 131 bytes, exceeding PENDING_ZCL_DATA_CAP. Restore
        // the exact prior behavior: the historical helper serialized into a
        // 128-byte buffer, so such a frame failed serialization and queued
        // nothing — i.e. the oversized frame is dropped, not truncated
        // (regression: `read_reporting_max_response_is_dropped`).
    );
}

#[cfg(test)]
mod tests {
    use super::{GroupTableAction, LocalZclCtx};
    use crate::remote_reporting::RemoteReportingState;
    use crate::{ClusterRef, EndpointConfig, EndpointIdentifyCluster, PendingZclResponse};
    use zigbee_zcl::clusters::Cluster;
    use zigbee_zcl::clusters::basic::{BasicCluster, PowerSource};
    use zigbee_zcl::clusters::groups::GroupsCluster;
    use zigbee_zcl::clusters::identify::IdentifyCluster;
    use zigbee_zcl::clusters::ota::OtaCluster;
    use zigbee_zcl::clusters::temperature::TemperatureCluster;
    use zigbee_zcl::foundation::reporting::ReportingEngine;
    use zigbee_zcl::frame::ZclFrame;
    use zigbee_zcl::{ClusterDirection, ClusterId, CommandId, DeviceId, ZclStatus};

    const EP: u8 = 1;
    const EP2: u8 = 2;
    const SRC_ADDR: u16 = 0x1234;
    const SRC_EP: u8 = 3;
    const SEQ: u8 = 0x42;

    /// Owns the runtime-local state a `LocalZclCtx` borrows. Deliberately built
    /// with **no** `MacDriver`/`ZigbeeDevice` in sight — the whole point of the
    /// split is that local ZCL dispatch needs none.
    struct Fixture {
        endpoints: heapless::Vec<EndpointConfig, { crate::MAX_ENDPOINTS }>,
        basic: BasicCluster,
        identify: heapless::Vec<EndpointIdentifyCluster, { crate::MAX_ENDPOINTS }>,
        reporting: ReportingEngine,
        remote_reporting: RemoteReportingState,
        pending: heapless::Vec<PendingZclResponse, 4>,
        scratch: [u8; 253],
    }

    impl Fixture {
        fn new(server_clusters: &[ClusterId]) -> Self {
            Self::with_clusters(server_clusters, &[])
        }

        /// Same as [`Self::new`], but the server clusters are configured on a
        /// second local endpoint as well, so per-endpoint behavior can be
        /// exercised.
        fn with_second_endpoint(server_clusters: &[ClusterId]) -> Self {
            let mut fx = Self::new(server_clusters);
            let mut servers = heapless::Vec::new();
            for c in server_clusters {
                servers.push(*c).unwrap();
            }
            fx.endpoints
                .push(EndpointConfig {
                    endpoint: EP2,
                    profile_id: 0x0104,
                    device_id: DeviceId::TEMPERATURE_SENSOR,
                    device_version: 1,
                    server_clusters: servers,
                    client_clusters: heapless::Vec::new(),
                })
                .unwrap();
            let _ = fx.identify.push(EndpointIdentifyCluster {
                endpoint: EP2,
                cluster: IdentifyCluster::new(),
            });
            fx
        }

        fn with_clusters(server_clusters: &[ClusterId], client_clusters: &[ClusterId]) -> Self {
            let mut servers = heapless::Vec::new();
            for c in server_clusters {
                servers.push(*c).unwrap();
            }
            let mut clients = heapless::Vec::new();
            for c in client_clusters {
                clients.push(*c).unwrap();
            }
            let mut endpoints = heapless::Vec::new();
            endpoints
                .push(EndpointConfig {
                    endpoint: EP,
                    profile_id: 0x0104,
                    device_id: DeviceId::TEMPERATURE_SENSOR,
                    device_version: 1,
                    server_clusters: servers,
                    client_clusters: clients,
                })
                .unwrap();
            let mut identify = heapless::Vec::new();
            let _ = identify.push(EndpointIdentifyCluster {
                endpoint: EP,
                cluster: IdentifyCluster::new(),
            });
            Self {
                endpoints,
                basic: BasicCluster::new("TestCo", "M1", "20240101", "1.0", PowerSource::Battery),
                identify,
                reporting: ReportingEngine::new(),
                remote_reporting: RemoteReportingState::new(),
                pending: heapless::Vec::new(),
                scratch: [0u8; 253],
            }
        }

        /// Dispatch a request against the fixture's local state, with the given
        /// application clusters wired into the `ClusterRef` slice.
        fn dispatch(
            &mut self,
            clusters: &mut [ClusterRef<'_>],
            cluster_id: u16,
            frame: &[u8],
        ) -> super::LocalZclOutcome {
            self.dispatch_to_endpoint(EP, clusters, cluster_id, frame)
        }

        /// Dispatch against an explicit local endpoint, so endpoint
        /// separation of the remote-reporting record can be exercised.
        fn dispatch_to_endpoint(
            &mut self,
            endpoint: u8,
            clusters: &mut [ClusterRef<'_>],
            cluster_id: u16,
            frame: &[u8],
        ) -> super::LocalZclOutcome {
            LocalZclCtx::new(
                &self.endpoints,
                &mut self.basic,
                &mut self.identify,
                &mut self.reporting,
                &mut self.remote_reporting,
                &mut self.pending,
                clusters,
                &mut self.scratch,
            )
            .dispatch(endpoint, SRC_EP, cluster_id, SRC_ADDR, frame)
        }
    }

    fn global_request(cmd: u8, ddr: bool, payload: &[u8]) -> heapless::Vec<u8, 64> {
        let mut frame =
            ZclFrame::new_global(SEQ, CommandId(cmd), ClusterDirection::ClientToServer, ddr);
        for &b in payload {
            frame.payload.push(b).unwrap();
        }
        let mut buf = [0u8; 64];
        let len = frame.serialize(&mut buf).unwrap();
        heapless::Vec::from_slice(&buf[..len]).unwrap()
    }

    fn cluster_request(cmd: u8, ddr: bool, payload: &[u8]) -> heapless::Vec<u8, 64> {
        let mut frame = ZclFrame::new_cluster_specific(
            SEQ,
            CommandId(cmd),
            ClusterDirection::ClientToServer,
            ddr,
        );
        for &b in payload {
            frame.payload.push(b).unwrap();
        }
        let mut buf = [0u8; 64];
        let len = frame.serialize(&mut buf).unwrap();
        heapless::Vec::from_slice(&buf[..len]).unwrap()
    }

    /// Build the exact ZCL bytes a queued response *should* contain, using the
    /// same header builder the stack uses, so equality is a real byte-for-byte
    /// oracle over command id, direction, sequence and payload.
    fn expected_global_response(cmd: u8, payload: &[u8]) -> heapless::Vec<u8, 128> {
        let mut frame =
            ZclFrame::new_global(SEQ, CommandId(cmd), ClusterDirection::ServerToClient, true);
        for &b in payload {
            frame.payload.push(b).unwrap();
        }
        let mut buf = [0u8; 128];
        let len = frame.serialize(&mut buf).unwrap();
        heapless::Vec::from_slice(&buf[..len]).unwrap()
    }

    /// Minimal application cluster whose `handle_command` returns the maximum
    /// (`CLUSTER_RESPONSE_PAYLOAD_CAP`) response payload — the largest a cluster
    /// can produce. Used to prove a cluster-specific response is queued whole
    /// (never dropped, never truncated) through the shared `queue_frame`.
    struct MaxRespCluster {
        store: zigbee_zcl::attribute::AttributeStore<1>,
    }

    impl MaxRespCluster {
        const CID: ClusterId = ClusterId(0xFC00);
        fn new() -> Self {
            Self {
                store: zigbee_zcl::attribute::AttributeStore::new(),
            }
        }
    }

    impl Cluster for MaxRespCluster {
        fn cluster_id(&self) -> ClusterId {
            Self::CID
        }
        fn handle_command(
            &mut self,
            _cmd: CommandId,
            _payload: &[u8],
        ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
            let mut v = heapless::Vec::new();
            for i in 0..super::CLUSTER_RESPONSE_PAYLOAD_CAP as u8 {
                v.push(i).unwrap();
            }
            Ok(v)
        }
        fn attributes(&self) -> &dyn zigbee_zcl::clusters::AttributeStoreAccess {
            &self.store
        }
        fn attributes_mut(&mut self) -> &mut dyn zigbee_zcl::clusters::AttributeStoreMutAccess {
            &mut self.store
        }
    }

    /// Minimal application cluster with a single **write-only** attribute —
    /// i.e. one that exists but can never be reported. Used to exercise the
    /// `UnreportableAttribute` branch of Configure Reporting; every attribute
    /// of the standard measurement clusters is reportable.
    struct WriteOnlyCluster {
        store: zigbee_zcl::attribute::AttributeStore<1>,
    }

    impl WriteOnlyCluster {
        const CID: ClusterId = ClusterId(0xFC01);
        const ATTR: zigbee_zcl::AttributeId = zigbee_zcl::AttributeId(0x0000);
        fn new() -> Self {
            let mut store = zigbee_zcl::attribute::AttributeStore::new();
            let _ = store.register(
                zigbee_zcl::attribute::AttributeDefinition {
                    id: Self::ATTR,
                    data_type: zigbee_zcl::data_types::ZclDataType::I16,
                    access: zigbee_zcl::attribute::AttributeAccess::WriteOnly,
                    name: "WriteOnly",
                },
                zigbee_zcl::data_types::ZclValue::I16(0),
            );
            Self { store }
        }
    }

    impl Cluster for WriteOnlyCluster {
        fn cluster_id(&self) -> ClusterId {
            Self::CID
        }
        fn handle_command(
            &mut self,
            _cmd: CommandId,
            _payload: &[u8],
        ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
            Err(ZclStatus::UnsupClusterCommand)
        }
        fn attributes(&self) -> &dyn zigbee_zcl::clusters::AttributeStoreAccess {
            &self.store
        }
        fn attributes_mut(&mut self) -> &mut dyn zigbee_zcl::clusters::AttributeStoreMutAccess {
            &mut self.store
        }
    }

    /// Queue-policy lock (non-blocking finding): a maximal cluster-specific
    /// response must be queued *whole*. The module-level `const _` assertion
    /// proves a cluster-specific frame can never exceed the queue buffer, so the
    /// shared `queue_frame`'s drop branch is unreachable and the response is
    /// always queued whole. This test exercises the largest possible payload to
    /// keep that proof honest.
    #[test]
    fn cluster_specific_max_response_is_queued_whole() {
        let mut fx = Fixture::new(&[MaxRespCluster::CID]);
        let mut cluster = MaxRespCluster::new();
        let mut clusters = [ClusterRef {
            endpoint: EP,
            cluster: &mut cluster,
        }];
        // Any client→server cluster-specific command; the test cluster ignores
        // the id and always returns the maximal payload.
        let req = cluster_request(0x00, true, &[]);
        let outcome = fx.dispatch(&mut clusters, MaxRespCluster::CID.0, &req);

        assert!(outcome.event.is_some());
        // Not dropped: exactly one response is queued.
        assert_eq!(fx.pending.len(), 1);

        // The maximal payload arrives whole behind the 3-byte cluster-specific
        // header — neither truncated to the buffer nor dropped.
        let mut expected = ZclFrame::new_cluster_specific(
            SEQ,
            CommandId(0x00),
            ClusterDirection::ServerToClient,
            true,
        );
        for i in 0..super::CLUSTER_RESPONSE_PAYLOAD_CAP as u8 {
            expected.payload.push(i).unwrap();
        }
        let mut buf = [0u8; 128];
        let len = expected.serialize(&mut buf).unwrap();
        assert_eq!(
            len,
            super::CLUSTER_RESPONSE_HEADER_LEN + super::CLUSTER_RESPONSE_PAYLOAD_CAP
        );
        assert_eq!(fx.pending[0].zcl_data.as_slice(), &buf[..len]);
    }

    /// Regression lock for the Read Reporting Configuration response overflow
    /// policy (non-blocking finding): a maximum-size response serializes to a
    /// frame larger than `PENDING_ZCL_DATA_CAP`. The historical helper
    /// serialized into a 128-byte buffer, so such a frame failed serialization
    /// and queued *nothing* — it is dropped, not truncated. This pins the
    /// shared `queue_frame`'s drop-on-overflow policy for the read-reporting
    /// path so it can never silently revert to enqueueing a truncated frame.
    #[test]
    fn read_reporting_max_response_is_dropped() {
        use zigbee_zcl::data_types::{ZclDataType, ZclValue};
        use zigbee_zcl::foundation::reporting::{
            ReadReportingConfigResponse, ReadReportingConfigResponseRecord, ReportDirection,
            ReportingConfig,
        };

        // Build the largest possible response: every record is a Send record
        // carrying a config with an 8-byte reportable change, so the serialized
        // payload fills the 128-byte builder buffer and the framed response
        // (3 header + 128 payload = 131 bytes) exceeds PENDING_ZCL_DATA_CAP.
        let mut response = ReadReportingConfigResponse {
            records: heapless::Vec::new(),
        };
        for n in 0..response.records.capacity() {
            response
                .records
                .push(ReadReportingConfigResponseRecord {
                    status: ZclStatus::Success,
                    direction: ReportDirection::Send,
                    attribute_id: zigbee_zcl::AttributeId(n as u16),
                    config: Some(ReportingConfig {
                        direction: ReportDirection::Send,
                        attribute_id: zigbee_zcl::AttributeId(n as u16),
                        data_type: ZclDataType::U64,
                        min_interval: 0x1111,
                        max_interval: 0x2222,
                        reportable_change: Some(ZclValue::U64(0x0102_0304_0506_0708)),
                    }),
                    timeout: None,
                })
                .unwrap();
        }

        // Independently reconstruct the full serialized 0x09 frame the helper
        // builds, and confirm it really does overflow the queue buffer, so the
        // drop policy is genuinely exercised (not a fits-whole no-op).
        let mut frame =
            ZclFrame::new_global(SEQ, CommandId(0x09), ClusterDirection::ServerToClient, true);
        let mut payload_buf = [0u8; 128];
        let payload_len = response.serialize(&mut payload_buf);
        for &b in &payload_buf[..payload_len] {
            frame.payload.push(b).unwrap();
        }
        let mut full = [0u8; 256];
        let full_len = frame.serialize(&mut full).unwrap();
        assert!(full_len > crate::PENDING_ZCL_DATA_CAP);

        let mut pending: heapless::Vec<PendingZclResponse, 4> = heapless::Vec::new();
        super::queue_read_reporting_response(
            &mut pending,
            zigbee_types::ShortAddress(SRC_ADDR),
            SRC_EP,
            EP,
            ClusterId::TEMPERATURE.0,
            SEQ,
            &response,
        );

        // Dropped: an oversized framed response queues nothing rather than
        // enqueueing a truncated (malformed) frame.
        assert!(pending.is_empty());
    }

    #[test]
    fn read_attributes_queues_byte_identical_read_response() {
        let mut fx = Fixture::new(&[ClusterId::BASIC]);
        // Read Basic ManufacturerName (0x0004).
        let req = global_request(0x00, true, &[0x04, 0x00]);
        let outcome = fx.dispatch(&mut [], ClusterId::BASIC.0, &req);

        assert!(outcome.event.is_some());
        assert!(outcome.group_action.is_none());
        assert!(outcome.fb_identify_target.is_none());
        assert_eq!(fx.pending.len(), 1);

        // Read Attributes Response record: attr(04 00) status(00) type(0x42
        // char string) len(06) "TestCo".
        let expected = expected_global_response(
            0x01,
            &[
                0x04, 0x00, 0x00, 0x42, 0x06, b'T', b'e', b's', b't', b'C', b'o',
            ],
        );
        assert_eq!(fx.pending[0].zcl_data.as_slice(), expected.as_slice());
        assert_eq!(fx.pending[0].dst_addr.0, SRC_ADDR);
        assert_eq!(fx.pending[0].dst_endpoint, SRC_EP);
        assert_eq!(fx.pending[0].src_endpoint, EP);
    }

    #[test]
    fn ota_client_read_attributes_frame_1643_queues_response() {
        let mut fx = Fixture::with_clusters(&[], &[ClusterId::OTA_UPGRADE]);
        let mut ota = OtaCluster::new(0x1234, 0x0001, 0x0102_0304);
        let mut clusters = [ClusterRef {
            endpoint: EP,
            cluster: &mut ota,
        }];

        // Captured ZHA interview frame 1643: global, server→client, TSN 7,
        // Read Attributes, CurrentFileVersion (0x0002).
        let request = [0x18, 0x07, 0x00, 0x02, 0x00];
        let outcome = fx.dispatch(&mut clusters, ClusterId::OTA_UPGRADE.0, &request);

        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived {
                frame_type: zigbee_zcl::frame::ZclFrameType::Global,
                ..
            })
        ));
        assert_eq!(fx.pending.len(), 1);
        assert_eq!(
            fx.pending[0].zcl_data.as_slice(),
            &[
                0x10, // global, client→server, disable default response
                0x07, // matching TSN
                0x01, // Read Attributes Response
                0x02, 0x00, // CurrentFileVersion
                0x00, // SUCCESS
                0x23, // uint32
                0x04, 0x03, 0x02, 0x01,
            ]
        );
        assert_eq!(fx.pending[0].dst_addr.0, SRC_ADDR);
        assert_eq!(fx.pending[0].dst_endpoint, SRC_EP);
        assert_eq!(fx.pending[0].src_endpoint, EP);
    }

    #[test]
    fn dispatch_is_deterministic_across_runs() {
        let req = global_request(0x00, true, &[0x04, 0x00]);
        let mut a = Fixture::new(&[ClusterId::BASIC]);
        let mut b = Fixture::new(&[ClusterId::BASIC]);
        a.dispatch(&mut [], ClusterId::BASIC.0, &req);
        b.dispatch(&mut [], ClusterId::BASIC.0, &req);
        assert_eq!(a.pending[0].zcl_data, b.pending[0].zcl_data);
    }

    #[test]
    fn write_attributes_queues_write_response() {
        // Temperature MeasuredValue (0x0000) is read-only, so the write fails
        // with a status record but still produces a 0x04 Write Attributes
        // Response — exercising parse_checked + process_write_dyn + queue.
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let mut clusters = [ClusterRef {
            endpoint: EP,
            cluster: &mut temp,
        }];
        // attr 0x0000, type I16 (0x29), value 0x0064.
        let req = global_request(0x02, true, &[0x00, 0x00, 0x29, 0x64, 0x00]);
        let outcome = fx.dispatch(&mut clusters, ClusterId::TEMPERATURE.0, &req);

        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x04);
        assert_eq!(resp.header.direction(), ClusterDirection::ServerToClient);
        // Read-only attribute → non-success status in the record.
        assert_ne!(resp.payload[0], ZclStatus::Success as u8);
    }

    #[test]
    fn configure_reporting_configures_engine_and_queues_response() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let mut clusters = [ClusterRef {
            endpoint: EP,
            cluster: &mut temp,
        }];
        // Configure Reporting (0x06): dir=Send(0), attr 0x0000, type I16(0x29),
        // min 10, max 60, reportable change I16 = 5.
        let req = global_request(
            0x06,
            true,
            &[0x00, 0x00, 0x00, 0x29, 0x0A, 0x00, 0x3C, 0x00, 0x05, 0x00],
        );
        let outcome = fx.dispatch(&mut clusters, ClusterId::TEMPERATURE.0, &req);

        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07); // Configure Reporting Response
        // A Success record for a reportable attribute must have registered a
        // config in the reporting engine.
        assert!(
            fx.reporting
                .get_config(
                    EP,
                    ClusterId::TEMPERATURE.0,
                    zigbee_zcl::foundation::reporting::ReportDirection::Send,
                    zigbee_zcl::AttributeId(0x0000),
                )
                .is_some()
        );
    }

    // ── Remote (client-configured) reporting interview state ──
    //
    // These lock the rule in `crate::remote_reporting`: a cluster is recorded
    // only after a non-empty, well-formed Configure Reporting command made
    // entirely of Send-direction records in which *every* record succeeded.
    // Each test also checks that the standards-mandated Configure Reporting
    // Response (0x07) behavior is unchanged, so the interview accounting can
    // never be "fixed" by weakening the response.

    /// One Send record for a reportable attribute: `[dir, attr_lo, attr_hi,
    /// type, min_lo, min_hi, max_lo, max_hi, change_lo, change_hi]`.
    fn configure_reporting_record(attr_id: u16, data_type: u8) -> [u8; 10] {
        let attr = attr_id.to_le_bytes();
        [
            0x00, attr[0], attr[1], data_type, 0x0A, 0x00, 0x3C, 0x00, 0x05, 0x00,
        ]
    }

    fn temperature_clusters(cluster: &mut TemperatureCluster) -> [ClusterRef<'_>; 1] {
        [ClusterRef {
            endpoint: EP,
            cluster,
        }]
    }

    /// Every record succeeded → exactly one distinct cluster is recorded and
    /// the dedicated event carries the running count.
    #[test]
    fn successful_remote_configure_reporting_records_one_cluster() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x29));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::ReportingConfigured {
                src_addr: SRC_ADDR,
                source_endpoint: SRC_EP,
                endpoint: EP,
                cluster_id: 0x0402,
                configured_clusters: 1,
            })
        ));
        assert_eq!(fx.remote_reporting.cluster_count(EP), 1);
        assert!(fx.remote_reporting.contains(EP, ClusterId::TEMPERATURE.0));

        // Standards behavior unchanged: an all-Success command still answers
        // with a 0x07 response.
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
    }

    /// Re-configuring the same cluster (ZHA re-runs the interview after a
    /// rejoin, and coordinators retry) must not inflate the count.
    #[test]
    fn repeated_remote_configure_reporting_does_not_double_count() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x29));
        for _ in 0..3 {
            fx.dispatch(
                &mut temperature_clusters(&mut temp),
                ClusterId::TEMPERATURE.0,
                &req,
            );
        }
        assert_eq!(fx.remote_reporting.cluster_count(EP), 1);

        // A second attribute on the same cluster is still one cluster.
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x29));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::ReportingConfigured {
                configured_clusters: 1,
                ..
            })
        ));
        assert_eq!(fx.remote_reporting.total_cluster_count(), 1);
    }

    /// Locally configured defaults populate the reporting engine but are not
    /// a remote interview — the historical bug this state exists to fix.
    #[test]
    fn local_default_reporting_does_not_affect_the_remote_count() {
        use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        fx.reporting
            .configure_for_cluster(
                EP,
                ClusterId::TEMPERATURE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: zigbee_zcl::AttributeId(0x0000),
                    data_type: zigbee_zcl::data_types::ZclDataType::I16,
                    min_interval: 10,
                    max_interval: 60,
                    reportable_change: None,
                },
            )
            .unwrap();

        assert_eq!(fx.reporting.configured_cluster_count(EP), 1);
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(fx.remote_reporting.is_empty());
    }

    /// An unsupported attribute is answered per-record but never counted.
    #[test]
    fn unsupported_attribute_is_not_counted() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x00FF, 0x29));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::UnsupportedAttribute as u8);
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived {
                command_id: 0x06,
                ..
            })
        ));
    }

    /// A known but unreportable (write-only) attribute is likewise not an
    /// interview step.
    #[test]
    fn unreportable_attribute_is_not_counted() {
        let mut fx = Fixture::new(&[WriteOnlyCluster::CID]);
        let mut cluster = WriteOnlyCluster::new();
        let req = global_request(
            0x06,
            true,
            &configure_reporting_record(WriteOnlyCluster::ATTR.0, 0x29),
        );
        let outcome = fx.dispatch(
            &mut [ClusterRef {
                endpoint: EP,
                cluster: &mut cluster,
            }],
            WriteOnlyCluster::CID.0,
            &req,
        );

        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::UnreportableAttribute as u8);
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    /// A known, enabled ZCL type still has to match the attribute's declared
    /// type. Temperature MeasuredValue is `I16`; presenting it as `U16` is an
    /// `InvalidDataType` rejection, not interview progress.
    #[test]
    fn mismatched_attribute_data_type_is_not_counted() {
        use zigbee_zcl::foundation::reporting::ReportDirection;

        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x21));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::InvalidDataType as u8);
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(
            fx.reporting
                .get_config(
                    EP,
                    ClusterId::TEMPERATURE.0,
                    ReportDirection::Send,
                    zigbee_zcl::AttributeId(0x0000),
                )
                .is_none()
        );
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    /// A data type this build does not support yields `InvalidDataType`;
    /// the record is answered but the cluster is not counted. Only runs where
    /// a type is genuinely disabled (`cargo test -p zigbee-runtime
    /// --no-default-features`) — with both float features on, no standard
    /// type is disabled.
    #[cfg(not(feature = "float32"))]
    #[test]
    fn disabled_data_type_is_not_counted() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        // Float32 (0x39) with a 4-byte reportable change.
        let req = global_request(
            0x06,
            true,
            &[
                0x00, 0x00, 0x00, 0x39, 0x0A, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
        );
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::InvalidDataType as u8);
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    /// An unknown data-type byte fails the parse: no response is queued (the
    /// command is malformed) and nothing is counted.
    #[test]
    fn unknown_data_type_is_not_counted() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0xFE));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        assert!(fx.pending.is_empty());
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    /// A truncated record and an empty payload are both malformed/empty
    /// commands, not completed interview steps.
    #[test]
    fn malformed_and_empty_commands_are_not_counted() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);

        // Truncated: direction + attribute + type, then nothing.
        let truncated = global_request(0x06, true, &[0x00, 0x00, 0x00, 0x29, 0x0A]);
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &truncated,
        );
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);

        // Empty payload: zero records.
        let empty = global_request(0x06, true, &[]);
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &empty,
        );
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(fx.pending.is_empty());
    }

    /// A command that configures one attribute successfully and is rejected
    /// for another did not give the client what it asked for. The successful
    /// record still lands in the reporting engine (per-record semantics), but
    /// the cluster is *not* recorded as remotely configured.
    #[test]
    fn partially_rejected_command_configures_engine_but_is_not_counted() {
        use zigbee_zcl::foundation::reporting::ReportDirection;

        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let mut payload = heapless::Vec::<u8, 32>::new();
        payload
            .extend_from_slice(&configure_reporting_record(0x0000, 0x29))
            .unwrap();
        payload
            .extend_from_slice(&configure_reporting_record(0x00FF, 0x29))
            .unwrap();
        let req = global_request(0x06, true, &payload);
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        // Per-record response: Success for 0x0000, failure for 0x00FF.
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::Success as u8);
        assert_eq!(resp.payload[4], ZclStatus::UnsupportedAttribute as u8);
        assert!(
            fx.reporting
                .get_config(
                    EP,
                    ClusterId::TEMPERATURE.0,
                    ReportDirection::Send,
                    zigbee_zcl::AttributeId(0x0000),
                )
                .is_some()
        );
        // …but the interview step is not complete.
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    /// A reporting-table capacity failure is a rejection, not a completed
    /// configuration.
    #[test]
    fn reporting_capacity_failure_is_not_counted() {
        use zigbee_zcl::foundation::reporting::{
            MAX_REPORT_CONFIGS, ReportDirection, ReportingConfig,
        };

        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        // Fill the engine with unrelated configurations.
        for n in 0..MAX_REPORT_CONFIGS {
            fx.reporting
                .configure_for_cluster(
                    EP,
                    0x1000 + n as u16,
                    ReportingConfig {
                        direction: ReportDirection::Send,
                        attribute_id: zigbee_zcl::AttributeId(n as u16),
                        data_type: zigbee_zcl::data_types::ZclDataType::I16,
                        min_interval: 10,
                        max_interval: 60,
                        reportable_change: None,
                    },
                )
                .unwrap();
        }

        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x29));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.payload[0], ZclStatus::InsufficientSpace as u8);
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    /// The same cluster on two endpoints is two distinct interview steps.
    #[test]
    fn remote_reporting_separates_endpoints() {
        let mut fx = Fixture::with_second_endpoint(&[ClusterId::TEMPERATURE]);
        let mut temp1 = TemperatureCluster::new(-4000, 12500);
        let mut temp2 = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x29));

        fx.dispatch_to_endpoint(
            EP,
            &mut [ClusterRef {
                endpoint: EP,
                cluster: &mut temp1,
            }],
            ClusterId::TEMPERATURE.0,
            &req,
        );
        let outcome = fx.dispatch_to_endpoint(
            EP2,
            &mut [ClusterRef {
                endpoint: EP2,
                cluster: &mut temp2,
            }],
            ClusterId::TEMPERATURE.0,
            &req,
        );

        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::ReportingConfigured {
                endpoint: EP2,
                configured_clusters: 1,
                ..
            })
        ));
        assert_eq!(fx.remote_reporting.cluster_count(EP), 1);
        assert_eq!(fx.remote_reporting.cluster_count(EP2), 1);
        assert_eq!(fx.remote_reporting.total_cluster_count(), 2);
    }

    /// A new commissioning/rejoin lifecycle starts from an empty record.
    #[test]
    fn resetting_clears_the_remote_interview_record() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &configure_reporting_record(0x0000, 0x29));
        fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );
        assert_eq!(fx.remote_reporting.cluster_count(EP), 1);

        fx.remote_reporting.clear();
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(!fx.remote_reporting.contains(EP, ClusterId::TEMPERATURE.0));

        // …and a later command re-populates it.
        fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );
        assert_eq!(fx.remote_reporting.cluster_count(EP), 1);
    }

    /// A single Receive-direction record: `[dir=Receive, attr_lo, attr_hi,
    /// timeout_lo, timeout_hi]`.
    fn receive_reporting_record(attr_id: u16, timeout: u16) -> [u8; 5] {
        let attr = attr_id.to_le_bytes();
        let to = timeout.to_le_bytes();
        [0x01, attr[0], attr[1], to[0], to[1]]
    }

    /// A receive-only Configure Reporting command configures how this device
    /// *consumes* a client's reports, not what it *sends*. Even though the
    /// record is individually accepted (the attribute exists and reporting is
    /// registered, so the 0x07 status is Success), it must not advance
    /// outbound interview progress. This is the historical bug: a Receive
    /// record on a valid attribute returns Success and would otherwise be
    /// counted as a completed send-reporting step.
    #[test]
    fn receive_only_configure_reporting_is_not_counted() {
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let req = global_request(0x06, true, &receive_reporting_record(0x0000, 0x003C));
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        // Standards behavior unchanged: the per-record status is Success and a
        // 0x07 response is still queued …
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::Success as u8);
        // … but outbound reporting progress is untouched, and this is not the
        // dedicated ReportingConfigured event.
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived {
                command_id: 0x06,
                ..
            })
        ));
    }

    /// A command mixing a successful Send record and a Receive record is not
    /// a pure send-reporting configuration, so it too must not advance
    /// outbound interview progress even though every record is accepted.
    #[test]
    fn mixed_send_and_receive_configure_reporting_is_not_counted() {
        use zigbee_zcl::foundation::reporting::ReportDirection;

        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let mut payload = heapless::Vec::<u8, 32>::new();
        payload
            .extend_from_slice(&configure_reporting_record(0x0000, 0x29))
            .unwrap();
        payload
            .extend_from_slice(&receive_reporting_record(0x0000, 0x003C))
            .unwrap();
        let req = global_request(0x06, true, &payload);
        let outcome = fx.dispatch(
            &mut temperature_clusters(&mut temp),
            ClusterId::TEMPERATURE.0,
            &req,
        );

        // Both records are individually accepted …
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x07);
        assert_eq!(resp.payload[0], ZclStatus::Success as u8);
        // … and the Send record still lands in the engine (per-record
        // semantics) …
        assert!(
            fx.reporting
                .get_config(
                    EP,
                    ClusterId::TEMPERATURE.0,
                    ReportDirection::Send,
                    zigbee_zcl::AttributeId(0x0000),
                )
                .is_some()
        );
        // … but the mixed command does not count as a completed send-reporting
        // interview step.
        assert_eq!(fx.remote_reporting.cluster_count(EP), 0);
        assert!(matches!(
            outcome.event,
            Some(crate::event_loop::StackEvent::CommandReceived { .. })
        ));
    }

    #[test]
    fn unsupported_global_command_queues_default_response() {
        let mut fx = Fixture::new(&[ClusterId::BASIC]);
        // Unknown global command 0xFF, default response NOT disabled.
        let req = global_request(0xFF, false, &[]);
        let outcome = fx.dispatch(&mut [], ClusterId::BASIC.0, &req);

        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let expected =
            expected_global_response(0x0B, &[0xFF, ZclStatus::UnsupGeneralCommand as u8]);
        assert_eq!(fx.pending[0].zcl_data.as_slice(), expected.as_slice());
    }

    #[test]
    fn handled_cluster_command_without_response_queues_default_response() {
        // Identify (0x00) sets IdentifyTime and returns no payload → a Success
        // Default Response is queued because default responses are enabled.
        let mut fx = Fixture::new(&[ClusterId::IDENTIFY]);
        let req = cluster_request(0x00, false, &[0x05, 0x00]);
        let outcome = fx.dispatch(&mut [], ClusterId::IDENTIFY.0, &req);

        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x0B);
        assert_eq!(resp.payload[0], 0x00); // triggering command
        assert_eq!(resp.payload[1], ZclStatus::Success as u8);
    }

    #[test]
    fn identify_query_queues_cluster_specific_response_with_remapped_command() {
        let mut fx = Fixture::new(&[ClusterId::IDENTIFY]);
        // Make the endpoint identifying so the query yields a response payload.
        fx.identify[0]
            .cluster
            .handle_command(CommandId(0x00), &[0x0A, 0x00])
            .unwrap();
        // Identify Query (0x01).
        let req = cluster_request(0x01, true, &[]);
        let outcome = fx.dispatch(&mut [], ClusterId::IDENTIFY.0, &req);

        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        // Identify Query (0x01) response command is remapped to 0x00.
        assert_eq!(resp.header.command_id.0, 0x00);
        assert_eq!(resp.header.direction(), ClusterDirection::ServerToClient);
        assert_eq!(resp.payload.as_slice(), &[0x0A, 0x00]); // IdentifyTime echo
    }

    #[test]
    fn add_group_returns_group_table_action_and_syncs_cluster() {
        let mut fx = Fixture::new(&[ClusterId::GROUPS]);
        let mut groups = GroupsCluster::new();
        let mut clusters = [ClusterRef {
            endpoint: EP,
            cluster: &mut groups,
        }];
        // Add Group 0x0007 with empty name.
        let req = cluster_request(0x00, true, &[0x07, 0x00, 0x00]);
        let outcome = fx.dispatch(&mut clusters, ClusterId::GROUPS.0, &req);

        // The M-generic APS group-table mutation must bubble up as an action.
        match outcome.group_action {
            Some(GroupTableAction::Add { group, endpoint }) => {
                assert_eq!(group, 0x0007);
                assert_eq!(endpoint, EP);
            }
            other => panic!("expected Add group action, got {:?}", other.is_some()),
        }
        // Add Group Response (cmd 0x00) is queued with success + group id.
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x00);
        assert_eq!(resp.payload[0], ZclStatus::Success as u8);
        assert_eq!(&resp.payload[1..3], &[0x07, 0x00]);
    }

    #[test]
    fn discover_commands_received_and_generated_use_distinct_response_commands() {
        // The 0x11/0x13 handlers were merged into one; this locks in that each
        // input command still maps to its own response command id.
        // Discover Commands Received (0x11) → Received Response (0x12).
        let mut fx = Fixture::new(&[ClusterId::IDENTIFY]);
        let req = global_request(0x11, true, &[0x00, 0xFF]);
        let outcome = fx.dispatch(&mut [], ClusterId::IDENTIFY.0, &req);
        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x12);
        assert_eq!(resp.header.direction(), ClusterDirection::ServerToClient);

        // Discover Commands Generated (0x13) → Generated Response (0x14).
        let mut fx2 = Fixture::new(&[ClusterId::IDENTIFY]);
        let req2 = global_request(0x13, true, &[0x00, 0xFF]);
        fx2.dispatch(&mut [], ClusterId::IDENTIFY.0, &req2);
        assert_eq!(fx2.pending.len(), 1);
        let resp2 = ZclFrame::parse(fx2.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp2.header.command_id.0, 0x14);
        assert_eq!(resp2.header.direction(), ClusterDirection::ServerToClient);
    }

    #[test]
    fn write_no_response_applies_write_without_queuing_a_response() {
        // Write Attributes No Response (0x05) to IdentifyTime (RW U16) must apply
        // the value through the shared write engine and queue *no* response.
        let mut fx = Fixture::new(&[ClusterId::IDENTIFY]);
        // attr 0x0000, type U16 (0x21), value 0x000A.
        let req = global_request(0x05, true, &[0x00, 0x00, 0x21, 0x0A, 0x00]);
        let outcome = fx.dispatch(&mut [], ClusterId::IDENTIFY.0, &req);

        assert!(outcome.event.is_some());
        // 0x05 never emits a response.
        assert_eq!(fx.pending.len(), 0);
        // The write was applied to the cluster's attribute store.
        assert_eq!(
            fx.identify[0]
                .cluster
                .attributes()
                .get(zigbee_zcl::AttributeId(0x0000)),
            Some(&zigbee_zcl::data_types::ZclValue::U16(0x000A))
        );
    }

    #[test]
    fn write_undivided_read_only_attribute_applies_nothing() {
        // Undivided (0x03) all-or-nothing: a read-only target must leave the
        // store untouched and still queue a 0x04 response with a failure record.
        let mut fx = Fixture::new(&[ClusterId::TEMPERATURE]);
        let mut temp = TemperatureCluster::new(-4000, 12500);
        let mut clusters = [ClusterRef {
            endpoint: EP,
            cluster: &mut temp,
        }];
        // MeasuredValue (0x0000) is read-only; type I16 (0x29), value 0x0064.
        let req = global_request(0x03, true, &[0x00, 0x00, 0x29, 0x64, 0x00]);
        let outcome = fx.dispatch(&mut clusters, ClusterId::TEMPERATURE.0, &req);

        assert!(outcome.event.is_some());
        assert_eq!(fx.pending.len(), 1);
        let resp = ZclFrame::parse(fx.pending[0].zcl_data.as_slice()).unwrap();
        assert_eq!(resp.header.command_id.0, 0x04);
        // Read-only attribute → non-success status, nothing applied.
        assert_ne!(resp.payload[0], ZclStatus::Success as u8);
    }
}
