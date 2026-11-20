//! Remote GP provisioning of the GREEN POWER group (spec 7.11,
//! `0x0A08`–`0x0A0D`, `EVT_GP_REMOTE_*`): the host configures other GP
//! sinks and proxies through the NCP.
//!
//! Each request becomes one Green Power cluster command sent from endpoint
//! 242 to endpoint 242 (profile 0xA1E0) with the NWK key, APS ACK when
//! unicast, through the same APS queue as `APS_DATA_REQUEST`: the response
//! carries the ZCL TSN, `EVT_APS_DATA_CONFIRM` (with the request TID)
//! reports the delivery, and the target's answer arrives as an
//! `EVT_GP_REMOTE_*` event with the same TSN:
//!
//! | Request | GP command | Answer |
//! |---|---|---|
//! | `GP_PAIRING_CONFIGURATION` | Pairing Configuration (c→s 0x09) | `EVT_GP_REMOTE_STATUS` (unicast) |
//! | `GP_SEND_PAIRING` | Pairing (s→c 0x01), also to the local proxy | none |
//! | `GP_TRANSLATION_UPDATE` | Translation Table Update (c→s 0x07) | `EVT_GP_REMOTE_STATUS` (unicast) |
//! | `GP_TRANSLATION_REQUEST` | Translation Table Request (c→s 0x08) | `EVT_GP_REMOTE_TRANSLATION_TABLE` |
//! | `GP_REMOTE_SINK_TABLE_REQUEST` | Sink Table Request (c→s 0x0A) | `EVT_GP_REMOTE_SINK_TABLE` |
//! | `GP_REMOTE_PROXY_TABLE_REQUEST` | Proxy Table Request (s→c 0x0B) | `EVT_GP_REMOTE_PROXY_TABLE` |
//!
//! The NCP sink keeps its authority (decision N1/step 5): in `NCP_SINK`
//! mode a `GP_SEND_PAIRING` that removes one of its own GPDs, or that names
//! the NCP itself as the unicast sink, is refused with `CONFLICT`.

use core::sync::atomic::{AtomicU8, Ordering};

use heapless::Vec;
use zigbee_gp::cluster::{Pairing, PairingSink, ProxyTableRequest, client_cmd, server_cmd};
use zigbee_gp::config::{
    ApplicationInformation, PairingConfiguration, PairingSecurity, ProxyTableResponse,
    SinkTableResponse, TranslationRecord, TranslationTableRequest, TranslationTableResponse,
    TranslationTableUpdate, TtPayload,
};
use zigbee_gp::sink::SinkTableRequest;
use zigbee_gp::types::{CommunicationMode, KeyType, SecurityLevel};
use zigbee_gp::{GP_CLUSTER_ID, GP_ENDPOINT, GP_PROFILE_ID};
use zigbee_runtime::green_power::ncp::GpRemoteFrame;
use zigbee_zrhp::codec::{CodecError, Reader, Wire, Writer};
use zigbee_zrhp::msg::gp::{
    EvtGpRemoteStatus, GpPairingConfiguration, GpRemoteProxyTableRequest, GpRemoteSinkTableRequest,
    GpSendPairing, GpTranslationRequest, GpTranslationUpdate, gp_translation_payload,
    paired_endpoints, pairing_config_flags, remote_table_flags, send_pairing_flags,
};
use zigbee_zrhp::msg::{Kind, MessageDef, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::records::{GpProxyRecord, GpTranslation, GpTranslationRecord, Record};
use zigbee_zrhp::types::{Endpoint, Status};

use crate::gp::{GP_PROXY_FORMAT_VERSION, gp_gpd_id, proxy_entry_bytes, sink_record, zrhp_gpd_id};
use crate::ncp::QUEUE_BYTES;
use crate::port::{CoordinatorPort, DataRequest, Destination, MAX_ASDU};

/// Queue room an `EVT_GP_REMOTE_*` event may need: the message header, the
/// event fields and the records of a full response frame.
pub const MAX_REMOTE_EVENT: usize =
    7 + 8 + 2 * zigbee_runtime::green_power::ncp::MAX_REMOTE_PAYLOAD + 64;
/// Most pre-commissioned groups, paired endpoints, GPD commands or
/// clusters one request carries (the ASDU limits them further).
const MAX_LIST: usize = 16;

/// ZCL TSNs of remote GP requests (independent of the runtime's own).
static TSN: AtomicU8 = AtomicU8::new(0);

fn next_tsn() -> u8 {
    TSN.fetch_add(1, Ordering::Relaxed).wrapping_add(1)
}

/// Whether `id` is a remote GP request.
pub(crate) fn is_remote_request(id: u16) -> bool {
    matches!(
        id,
        ids::GP_PAIRING_CONFIGURATION
            | ids::GP_SEND_PAIRING
            | ids::GP_TRANSLATION_UPDATE
            | ids::GP_TRANSLATION_REQUEST
            | ids::GP_REMOTE_SINK_TABLE_REQUEST
            | ids::GP_REMOTE_PROXY_TABLE_REQUEST
    )
}

/// A remote GP request ready for the APS queue.
pub(crate) struct RemoteRequest {
    /// The APS frame.
    pub data: DataRequest,
    /// Its ZCL TSN.
    pub tsn: u8,
    /// Unicast destination whose answer goes to the host.
    pub expect: Option<u16>,
    /// GP Pairing to hand to the local proxy once accepted.
    pub loopback: bool,
}

const ZCL_CLUSTER_SPECIFIC: u8 = 0x01;
const ZCL_SERVER_TO_CLIENT: u8 = 0x08;
const ZCL_DISABLE_DEFAULT_RESPONSE: u8 = 0x10;

/// Validated destination: unicast NWK (not this node) or a broadcast.
fn destination(dst: u16, own: u16, unicast_only: bool) -> Result<Destination, Status> {
    match dst {
        0xFFFC | 0xFFFD | 0xFFFF if !unicast_only => Ok(Destination::Broadcast(dst)),
        _ if dst >= 0xFFF8 || dst == own => Err(Status::InvalidParameter),
        _ => Ok(Destination::Nwk(dst)),
    }
}

/// Build the APS frame of a remote GP request (`tid`: the request TID that
/// `EVT_APS_DATA_CONFIRM` reports).
pub(crate) fn remote_request<C: CoordinatorPort>(
    message: &RawMessage<'_>,
    tid: u16,
    port: &mut C,
) -> Result<RemoteRequest, Status> {
    let own = port.info().short;
    let own_ieee = port.ieee();
    let decode_error = |_| Status::InvalidParameter;
    let mut payload = [0u8; MAX_ASDU - 3];
    let gp_error = |e: zigbee_gp::Error| match e {
        zigbee_gp::Error::BufferFull => Status::TooLarge,
        _ => Status::InvalidParameter,
    };
    let (dst, server_to_client, command, len, unicast_only, ddr) = match message.header.msg_id {
        ids::GP_PAIRING_CONFIGURATION => {
            let r: GpPairingConfiguration = message.decode().map_err(decode_error)?;
            let n = pairing_configuration(&r, &mut payload)?;
            (
                r.dst,
                false,
                server_cmd::PAIRING_CONFIGURATION,
                n,
                false,
                false,
            )
        }
        ids::GP_SEND_PAIRING => {
            let r: GpSendPairing = message.decode().map_err(decode_error)?;
            let pairing = send_pairing(&r)?;
            let gp = port.gp().ok_or(Status::UnsupportedCommand)?;
            if !gp.host_sink() {
                let own_entry = gp
                    .sink_entries()
                    .iter()
                    .any(|e| e.gpd_id.matches(&pairing.gpd_id));
                let names_ncp = matches!(
                    pairing.sink,
                    PairingSink::Unicast { ieee, nwk } if ieee == own_ieee || nwk == own
                );
                if names_ncp || (own_entry && pairing.remove_gpd) {
                    zigbee_runtime::green_power::ncp::note_conflict();
                    return Err(Status::Conflict);
                }
            }
            let n = pairing.encode(&mut payload).map_err(gp_error)?;
            (r.dst, true, client_cmd::PAIRING, n, false, true)
        }
        ids::GP_TRANSLATION_UPDATE => {
            let r: GpTranslationUpdate = message.decode().map_err(decode_error)?;
            let mut records: Vec<TranslationRecord<'_>, 7> = Vec::new();
            for t in r.translations.iter() {
                records
                    .push(translation_record(&t))
                    .map_err(|_| Status::InvalidParameter)?;
            }
            let n = TranslationTableUpdate::encode(
                &gp_gpd_id(&r.gpd_id),
                r.action,
                &records,
                &mut payload,
            )
            .map_err(gp_error)?;
            (
                r.dst,
                false,
                server_cmd::TRANSLATION_TABLE_UPDATE,
                n,
                false,
                false,
            )
        }
        ids::GP_TRANSLATION_REQUEST => {
            let r: GpTranslationRequest = message.decode().map_err(decode_error)?;
            let n = TranslationTableRequest {
                start_index: r.start_index,
            }
            .encode(&mut payload)
            .map_err(gp_error)?;
            (
                r.dst,
                false,
                server_cmd::TRANSLATION_TABLE_REQUEST,
                n,
                true,
                true,
            )
        }
        ids::GP_REMOTE_SINK_TABLE_REQUEST => {
            let r: GpRemoteSinkTableRequest = message.decode().map_err(decode_error)?;
            let request = match r.index {
                0xFF => SinkTableRequest::ByGpdId(gp_gpd_id(&r.gpd_id)),
                index => SinkTableRequest::ByIndex(index),
            };
            let n = request.encode(&mut payload).map_err(gp_error)?;
            (r.dst, false, server_cmd::SINK_TABLE_REQUEST, n, true, true)
        }
        ids::GP_REMOTE_PROXY_TABLE_REQUEST => {
            let r: GpRemoteProxyTableRequest = message.decode().map_err(decode_error)?;
            let request = match r.index {
                0xFF => ProxyTableRequest::ByGpdId(gp_gpd_id(&r.gpd_id)),
                index => ProxyTableRequest::ByIndex(index),
            };
            let n = request.encode(&mut payload).map_err(gp_error)?;
            (r.dst, true, client_cmd::PROXY_TABLE_REQUEST, n, true, true)
        }
        _ => return Err(Status::UnsupportedCommand),
    };
    let dst = destination(dst, own, unicast_only)?;
    let unicast = matches!(dst, Destination::Nwk(_));
    let tsn = next_tsn();
    let mut fc = ZCL_CLUSTER_SPECIFIC;
    if server_to_client {
        fc |= ZCL_SERVER_TO_CLIENT;
    }
    if ddr || !unicast {
        fc |= ZCL_DISABLE_DEFAULT_RESPONSE;
    }
    let mut asdu = Vec::new();
    asdu.extend_from_slice(&[fc, tsn, command])
        .and_then(|()| asdu.extend_from_slice(&payload[..len]))
        .map_err(|_| Status::TooLarge)?;
    Ok(RemoteRequest {
        data: DataRequest {
            req_tid: tid,
            dst,
            dst_ep: GP_ENDPOINT,
            src_ep: GP_ENDPOINT,
            profile: GP_PROFILE_ID,
            cluster: GP_CLUSTER_ID,
            tx_options: u8::from(unicast),
            radius: 0,
            asdu,
        },
        tsn,
        expect: match dst {
            Destination::Nwk(nwk) if message.header.msg_id != ids::GP_SEND_PAIRING => Some(nwk),
            _ => None,
        },
        loopback: message.header.msg_id == ids::GP_SEND_PAIRING && !unicast,
    })
}

/// GP Pairing Configuration payload of a `GP_PAIRING_CONFIGURATION`.
fn pairing_configuration(r: &GpPairingConfiguration<'_>, out: &mut [u8]) -> Result<usize, Status> {
    use pairing_config_flags::*;
    // Application Description (0b101) and reserved actions are not
    // provisioned through ZRHP.
    if r.actions & 0x07 > 4 || r.actions & 0xF0 != 0 || r.communication_mode > 3 {
        return Err(Status::InvalidParameter);
    }
    let mode = CommunicationMode::from_bits(r.communication_mode);
    let security = if r.flags & SECURITY != 0 {
        let level = SecurityLevel::from_bits(r.security_level);
        if r.security_level > 3 || level == SecurityLevel::Reserved {
            return Err(Status::InvalidParameter);
        }
        Some(PairingSecurity {
            level,
            key_type: KeyType::from_bits(r.key_type).map_err(|_| Status::InvalidParameter)?,
            key: r.key,
        })
    } else {
        None
    };
    let mac_seq_cap = r.flags & MAC_SEQ_CAPABLE != 0;
    let mut groups: Vec<u8, { 4 * MAX_LIST }> = Vec::new();
    for g in r.groups.iter() {
        groups
            .extend_from_slice(&g.group.to_le_bytes())
            .and_then(|()| groups.extend_from_slice(&g.alias.to_le_bytes()))
            .map_err(|_| Status::TooLarge)?;
    }
    if (mode == CommunicationMode::CommissionedGroupcast) == groups.is_empty() {
        return Err(Status::InvalidParameter);
    }
    let mut endpoints: Vec<u8, MAX_LIST> = Vec::new();
    for ep in r.endpoints.iter() {
        endpoints.push(ep).map_err(|_| Status::TooLarge)?;
    }
    let number_of_paired_endpoints = match r.paired {
        paired_endpoints::NONE | paired_endpoints::SINK_DERIVES | paired_endpoints::ALL
            if endpoints.is_empty() =>
        {
            r.paired
        }
        paired_endpoints::LISTED if !endpoints.is_empty() => endpoints.len() as u8,
        _ => return Err(Status::InvalidParameter),
    };
    if r.app_info & !0x0F != 0 {
        return Err(Status::InvalidParameter);
    }
    let mut commands: Vec<u8, MAX_LIST> = Vec::new();
    for c in r.gpd_commands.iter() {
        commands.push(c).map_err(|_| Status::TooLarge)?;
    }
    let mut server: Vec<u8, { 2 * MAX_LIST }> = Vec::new();
    for c in r.server_clusters.iter() {
        server
            .extend_from_slice(&c.to_le_bytes())
            .map_err(|_| Status::TooLarge)?;
    }
    let mut client: Vec<u8, { 2 * MAX_LIST }> = Vec::new();
    for c in r.client_clusters.iter() {
        client
            .extend_from_slice(&c.to_le_bytes())
            .map_err(|_| Status::TooLarge)?;
    }
    let application_information = (r.app_info != 0).then_some(ApplicationInformation {
        bits: r.app_info,
        manufacturer_id: (r.app_info & 0x01 != 0).then_some(r.manufacturer_id),
        model_id: (r.app_info & 0x02 != 0).then_some(r.model_id),
        gpd_commands: &commands,
        server_clusters: &server,
        client_clusters: &client,
        switch_information: &[],
        application_description: None,
    });
    let pc = PairingConfiguration {
        actions: r.actions,
        gpd_id: gp_gpd_id(&r.gpd_id),
        communication_mode: mode,
        mac_seq_cap,
        rx_on_cap: r.flags & RX_ON_CAPABLE != 0,
        fixed_location: r.flags & FIXED_LOCATION != 0,
        device_id: r.device_id,
        groups: &groups,
        assigned_alias: (r.flags & ASSIGNED_ALIAS != 0).then_some(r.assigned_alias),
        groupcast_radius: r.groupcast_radius,
        security,
        frame_counter: (security.is_some() || mac_seq_cap).then_some(r.frame_counter),
        number_of_paired_endpoints,
        paired_endpoints: &endpoints,
        application_information,
    };
    pc.encode(out).map_err(|e| match e {
        zigbee_gp::Error::BufferFull => Status::TooLarge,
        _ => Status::InvalidParameter,
    })
}

/// GP Pairing of a `GP_SEND_PAIRING`.
fn send_pairing(r: &GpSendPairing) -> Result<Pairing, Status> {
    use send_pairing_flags::*;
    if r.communication_mode > 3 || r.security_level > 3 {
        return Err(Status::InvalidParameter);
    }
    let mode = CommunicationMode::from_bits(r.communication_mode);
    let add_sink = r.flags & ADD_SINK != 0;
    let remove_gpd = r.flags & REMOVE_GPD != 0;
    let sink = if remove_gpd {
        PairingSink::None
    } else if mode.is_unicast() {
        PairingSink::Unicast {
            ieee: r.sink_ieee.0,
            nwk: r.sink_nwk,
        }
    } else {
        PairingSink::Group(r.sink_group)
    };
    let level = SecurityLevel::from_bits(r.security_level);
    if level == SecurityLevel::Reserved || gp_gpd_id(&r.gpd_id).is_reserved() {
        return Err(Status::InvalidParameter);
    }
    Ok(Pairing {
        gpd_id: gp_gpd_id(&r.gpd_id),
        add_sink,
        remove_gpd,
        communication_mode: mode,
        gpd_fixed: r.flags & GPD_FIXED != 0,
        mac_seq_cap: r.flags & MAC_SEQ_CAPABLE != 0,
        security_level: level,
        key_type: KeyType::from_bits(r.key_type).map_err(|_| Status::InvalidParameter)?,
        sink,
        device_id: add_sink.then_some(r.device_id),
        frame_counter: (r.flags & FRAME_COUNTER != 0).then_some(r.frame_counter),
        key: (r.flags & KEY != 0).then_some(r.key),
        assigned_alias: (r.flags & ASSIGNED_ALIAS != 0).then_some(r.assigned_alias),
        groupcast_radius: (r.flags & GROUPCAST_RADIUS != 0).then_some(r.groupcast_radius),
    })
}

/// The GP translation of a ZRHP translation (index 0xFF, as GP 1.1 sends).
fn translation_record<'a>(t: &GpTranslation<'a>) -> TranslationRecord<'a> {
    TranslationRecord {
        index: 0xFF,
        gpd_command: t.gpd_command,
        endpoint: t.endpoint,
        profile: t.profile,
        cluster: t.cluster,
        zcl_command: t.zcl_command,
        payload: match t.payload_len {
            gp_translation_payload::DERIVED => TtPayload::Derived,
            gp_translation_payload::COPY_GPD => TtPayload::CopyGpd,
            _ => TtPayload::Fixed(t.payload),
        },
        additional_info: None,
    }
}

/// The ZRHP translation of a GP translation.
fn zrhp_translation<'a>(t: &TranslationRecord<'a>) -> GpTranslation<'a> {
    let (payload_len, payload) = match t.payload {
        TtPayload::Derived => (gp_translation_payload::DERIVED, &[][..]),
        TtPayload::CopyGpd => (gp_translation_payload::COPY_GPD, &[][..]),
        TtPayload::Fixed(p) => (p.len() as u8, p),
    };
    GpTranslation {
        gpd_command: t.gpd_command,
        endpoint: t.endpoint,
        profile: t.profile,
        cluster: t.cluster,
        zcl_command: t.zcl_command,
        payload_len,
        payload,
    }
}

// ── Events ──────────────────────────────────────────────────────────

/// An `EVT_GP_REMOTE_*_TABLE` body (`ID`: the event) written straight
/// from the GP response.
struct TableBody<'f, const ID: u16> {
    frame: &'f GpRemoteFrame,
    keys: bool,
}

impl<const ID: u16> MessageDef for TableBody<'_, ID> {
    const ID: u16 = ID;
    const KIND: Kind = Kind::Event;
}

impl<'a, const ID: u16> Wire<'a> for TableBody<'_, ID> {
    fn read(_: &mut Reader<'a>) -> Result<Self, CodecError> {
        Err(CodecError::Invalid)
    }

    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        let frame = self.frame;
        let p = &frame.payload[..];
        let byte = |i: usize, default: u8| p.get(i).copied().unwrap_or(default);
        w.u16(frame.src_nwk)?;
        w.u8(frame.tsn)?;
        // Status, total and start index. GP Translation Table Response has
        // an options byte after the status.
        let at = usize::from(ID == ids::EVT_GP_REMOTE_TRANSLATION_TABLE);
        w.bytes(&[byte(0, 0xFF), byte(1 + at, 0), byte(2 + at, 0)])?;
        let flags_at = w.reserve(1)?;
        let count_at = w.reserve(1)?;
        let mut n = 0u8;
        let mut omitted = frame.truncated;
        match ID {
            ids::EVT_GP_REMOTE_SINK_TABLE => match SinkTableResponse::decode(p) {
                Ok(response) => {
                    for entry in response.entries() {
                        let Ok(entry) = entry else {
                            omitted = true;
                            break;
                        };
                        w.write(&Record::GpSink(sink_record(&entry, self.keys)))?;
                        n += 1;
                    }
                }
                Err(_) => omitted = true,
            },
            ids::EVT_GP_REMOTE_PROXY_TABLE => match ProxyTableResponse::decode(p) {
                Ok(response) => {
                    let mut encoded = [0u8; 1 + zigbee_gp::proxy::MAX_ENTRY_ENCODED_LEN];
                    for entry in response.entries() {
                        let Ok((_, entry)) = entry else {
                            omitted = true;
                            break;
                        };
                        let Ok(len) = proxy_entry_bytes(&entry, self.keys, &mut encoded) else {
                            omitted = true;
                            break;
                        };
                        w.write(&Record::GpProxy(GpProxyRecord {
                            format_version: GP_PROXY_FORMAT_VERSION,
                            entries: &encoded[..len],
                        }))?;
                        n += 1;
                    }
                }
                Err(_) => omitted = true,
            },
            _ => match TranslationTableResponse::decode(p) {
                Ok(response) if !response.additional_info => {
                    for (gpd, record) in response.entries() {
                        w.write(&Record::GpTranslation(GpTranslationRecord {
                            gpd_id: zrhp_gpd_id(&gpd),
                            translation: zrhp_translation(&record),
                        }))?;
                        n += 1;
                    }
                }
                // Additional information blocks, LPED or malformed entries.
                _ => omitted |= byte(4, 0) != 0,
            },
        }
        let flags = if omitted {
            remote_table_flags::OMITTED
        } else {
            0
        };
        w.patch(flags_at, &[flags]);
        w.patch(count_at, &[n]);
        Ok(())
    }
}

/// Queue the `EVT_GP_REMOTE_*` event of a remote GP response.
pub(crate) fn emit_remote(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    seqs: &mut [u16; Endpoint::COUNT],
    frame: &GpRemoteFrame,
    keys: bool,
) {
    let global = frame.frame_control & 0x03 == 0;
    let to_client = frame.frame_control & ZCL_SERVER_TO_CLIENT != 0;
    match (global, to_client, frame.command_id) {
        (true, _, 0x0B) => {
            let event = EvtGpRemoteStatus {
                src_nwk: frame.src_nwk,
                zcl_tsn: frame.tsn,
                command_id: frame.payload.first().copied().unwrap_or(0xFF),
                zcl_status: frame.payload.get(1).copied().unwrap_or(0xFF),
            };
            crate::ncp::push_event(queues, seqs, &event);
        }
        (false, true, client_cmd::SINK_TABLE_RESPONSE) => {
            let body = TableBody::<{ ids::EVT_GP_REMOTE_SINK_TABLE }> { frame, keys };
            crate::ncp::push_event(queues, seqs, &body);
        }
        (false, true, client_cmd::TRANSLATION_TABLE_RESPONSE) => {
            let body = TableBody::<{ ids::EVT_GP_REMOTE_TRANSLATION_TABLE }> { frame, keys };
            crate::ncp::push_event(queues, seqs, &body);
        }
        (false, false, server_cmd::PROXY_TABLE_RESPONSE) => {
            let body = TableBody::<{ ids::EVT_GP_REMOTE_PROXY_TABLE }> { frame, keys };
            crate::ncp::push_event(queues, seqs, &body);
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests;
