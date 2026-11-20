//! Remote GP provisioning steps (ZRHP 7.11 "Remote GP provisioning"): pair a
//! GPD with a sink through GP Pairing Configuration, retarget its commands
//! with GP Translation Table Update and read the sink's tables back.
//!
//! ```text
//! --gp-sink 0xNWK          the sink (unicast)
//! --gp-src 0xSRCID         the GPD (ApplicationID 0)
//! --gp-pair KEY:FC         Pairing Configuration Extend: derived groupcast,
//!                          level 3 individual key KEY (32 hex digits, as on
//!                          air), frame counter FC, sink-derived endpoints
//! --gp-translate C:EP:CL:Z GP Translation Table Update add: GPD command C to
//!                          endpoint EP, cluster CL, ZCL command Z (hex,
//!                          profile 0x0104, no payload)
//! --gp-untranslate C:EP:CL:Z  the same translation, removed
//! --gp-remove              Pairing Configuration Remove GPD
//! --gp-tables              Sink Table Request (by GPD) and Translation
//!                          Table Request
//! --gp-watch SECS          print every event for SECS afterwards
//! ```
//!
//! Every step waits for the request's response and the matching
//! `EVT_GP_REMOTE_*` (the ZCL Default Response or table response), so a
//! refused command fails the run.

use std::time::{Duration, Instant};

use zigbee_zrhp::codec::List;
use zigbee_zrhp::msg::gp::{
    EvtGpRemoteSinkTable, EvtGpRemoteStatus, EvtGpRemoteTranslationTable, GpPairingConfiguration,
    GpRemoteRsp, GpRemoteSinkTableRequest, GpTranslationRequest, GpTranslationUpdate,
    paired_endpoints, pairing_config_flags,
};
use zigbee_zrhp::msg::{Kind, ids};
use zigbee_zrhp::records::{GpTranslation, GpdId};

use crate::{ByteStream, Host, Received, body, report_event};

/// How long a remote request may take (APS ACK, a sink's reply).
const REMOTE_TIMEOUT: Duration = Duration::from_secs(15);

fn hex_u32(text: &str) -> Result<u32, String> {
    u32::from_str_radix(text.trim_start_matches("0x"), 16).map_err(|_| format!("{text}: hex"))
}

fn translation(spec: &str) -> Result<(u8, u8, u16, u8), String> {
    let parts: Vec<&str> = spec.split(':').collect();
    let [c, ep, cl, z] = parts[..] else {
        return Err(format!("{spec}: GPDCMD:EP:CLUSTER:ZCLCMD"));
    };
    Ok((
        hex_u32(c)? as u8,
        hex_u32(ep)? as u8,
        hex_u32(cl)? as u16,
        hex_u32(z)? as u8,
    ))
}

/// The arguments of the GP steps, from the command line.
pub struct GpArgs {
    pub sink: u16,
    pub src: u32,
    pub pair: Option<([u8; 16], u32)>,
    pub translate: Option<(u8, u8, u16, u8)>,
    pub untranslate: Option<(u8, u8, u16, u8)>,
    pub remove: bool,
    pub tables: bool,
    pub watch_s: u64,
}

impl GpArgs {
    /// `None` without `--gp-sink`.
    pub fn parse(args: &[String]) -> Result<Option<Self>, String> {
        let value = |flag: &str| {
            args.iter()
                .position(|a| a == flag)
                .and_then(|i| args.get(i + 1))
                .cloned()
        };
        let Some(sink) = value("--gp-sink") else {
            return Ok(None);
        };
        let src = hex_u32(&value("--gp-src").ok_or("--gp-src 0xSRCID required")?)?;
        let pair = match value("--gp-pair") {
            None => None,
            Some(spec) => {
                let (key_hex, fc) = spec.split_once(':').ok_or("--gp-pair KEY:FC")?;
                let bytes = crate::unhex(key_hex);
                let key: [u8; 16] = bytes
                    .try_into()
                    .map_err(|_| "--gp-pair: a 16-byte key (32 hex digits)")?;
                Some((key, fc.parse().map_err(|_| "--gp-pair: decimal FC")?))
            }
        };
        Ok(Some(Self {
            sink: hex_u32(&sink)? as u16,
            src,
            pair,
            translate: value("--gp-translate")
                .map(|s| translation(&s))
                .transpose()?,
            untranslate: value("--gp-untranslate")
                .map(|s| translation(&s))
                .transpose()?,
            remove: args.iter().any(|a| a == "--gp-remove"),
            tables: args.iter().any(|a| a == "--gp-tables"),
            watch_s: value("--gp-watch")
                .map_or(Ok(0), |v| v.parse().map_err(|_| "--gp-watch SECS"))?,
        }))
    }
}

/// Send a remote request, then wait for its response and the event that
/// carries the same ZCL TSN.
fn remote<S: ByteStream>(
    host: &mut Host<S>,
    what: &str,
    tid: u16,
    event_id: u16,
) -> Result<Received, String> {
    let started = Instant::now();
    let rsp = host.response_within(tid, REMOTE_TIMEOUT)?;
    let tsn = body::<GpRemoteRsp>(&rsp)
        .map_err(|e| format!("{what}: {e}"))?
        .zcl_tsn;
    let event = host.wait_for(&format!("{what} answer (tsn {tsn})"), REMOTE_TIMEOUT, |m| {
        m.kind == Kind::Event
            && (m.msg_id == event_id || m.msg_id == ids::EVT_GP_REMOTE_STATUS)
            && m.payload.get(2) == Some(&tsn)
    })?;
    println!("{what}: tsn {tsn}, answered after {:?}", started.elapsed());
    Ok(event)
}

/// The Default Response of a configuration command; `Err` unless SUCCESS.
fn expect_success(what: &str, event: &Received) -> Result<(), String> {
    let status: EvtGpRemoteStatus = event.raw().decode().map_err(|e| format!("{e:?}"))?;
    println!(
        "  EVT_GP_REMOTE_STATUS from 0x{:04X}: command 0x{:02X} status 0x{:02X}",
        status.src_nwk, status.command_id, status.zcl_status
    );
    if status.zcl_status != 0 {
        return Err(format!("{what}: ZCL status 0x{:02X}", status.zcl_status));
    }
    Ok(())
}

fn translation_update<S: ByteStream>(
    host: &mut Host<S>,
    args: &GpArgs,
    action: u8,
    (gpd_command, endpoint, cluster, zcl_command): (u8, u8, u16, u8),
) -> Result<(), String> {
    let what = if action == 2 {
        "GP_TRANSLATION_UPDATE remove"
    } else {
        "GP_TRANSLATION_UPDATE add"
    };
    let translations = [GpTranslation {
        gpd_command,
        endpoint,
        profile: 0x0104,
        cluster,
        zcl_command,
        payload_len: 0,
        payload: &[],
    }];
    let tid = host.request(&GpTranslationUpdate {
        dst: args.sink,
        action,
        gpd_id: GpdId::SrcId(args.src),
        translations: List::new(&translations),
    });
    println!(
        "-> {what} 0x{:08X}: cmd 0x{gpd_command:02X} -> ep {endpoint} cluster 0x{cluster:04X} cmd 0x{zcl_command:02X}",
        args.src
    );
    let event = remote(host, what, tid, ids::EVT_GP_REMOTE_STATUS)?;
    expect_success(what, &event)
}

fn pairing_configuration<S: ByteStream>(
    host: &mut Host<S>,
    args: &GpArgs,
    actions: u8,
    security: Option<([u8; 16], u32)>,
) -> Result<(), String> {
    let (key, frame_counter) = security.unwrap_or(([0; 16], 0));
    let what = if actions == 4 {
        "GP_PAIRING_CONFIGURATION remove GPD"
    } else {
        "GP_PAIRING_CONFIGURATION extend"
    };
    let tid = host.request(&GpPairingConfiguration {
        dst: args.sink,
        actions,
        gpd_id: GpdId::SrcId(args.src),
        device_id: 0x02,
        communication_mode: 1,
        flags: if security.is_some() {
            pairing_config_flags::SECURITY
                | pairing_config_flags::MAC_SEQ_CAPABLE
                | pairing_config_flags::FIXED_LOCATION
        } else {
            pairing_config_flags::FIXED_LOCATION
        },
        assigned_alias: 0,
        groupcast_radius: 0,
        security_level: 3,
        key_type: 4,
        key,
        frame_counter,
        groups: List::new(&[]),
        paired: paired_endpoints::SINK_DERIVES,
        endpoints: List::new(&[]),
        app_info: 0,
        manufacturer_id: 0,
        model_id: 0,
        gpd_commands: List::new(&[]),
        server_clusters: List::new(&[]),
        client_clusters: List::new(&[]),
    });
    println!(
        "-> {what} to 0x{:04X}: 0x{:08X} derived groupcast, security {}, frame counter {frame_counter}",
        args.sink,
        args.src,
        security.is_some()
    );
    let event = remote(host, what, tid, ids::EVT_GP_REMOTE_STATUS)?;
    expect_success(what, &event)
}

fn tables<S: ByteStream>(host: &mut Host<S>, args: &GpArgs) -> Result<(), String> {
    let tid = host.request(&GpRemoteSinkTableRequest {
        dst: args.sink,
        index: 0xFF,
        gpd_id: GpdId::SrcId(args.src),
    });
    let event = remote(
        host,
        "GP_REMOTE_SINK_TABLE_REQUEST",
        tid,
        ids::EVT_GP_REMOTE_SINK_TABLE,
    )?;
    if event.msg_id == ids::EVT_GP_REMOTE_STATUS {
        expect_success("GP_REMOTE_SINK_TABLE_REQUEST", &event)?;
    } else {
        let table: EvtGpRemoteSinkTable = event.raw().decode().map_err(|e| format!("{e:?}"))?;
        println!(
            "  EVT_GP_REMOTE_SINK_TABLE from 0x{:04X}: status 0x{:02X} total {} start 0x{:02X} flags 0x{:02X}",
            table.src_nwk, table.status, table.total, table.start_index, table.flags
        );
        for record in table.records.iter() {
            match record {
                zigbee_zrhp::records::Record::GpSink(mut sink) => {
                    // Never print key material.
                    let keyed = sink.key.take().is_some();
                    println!("    {sink:?} (key present {keyed})");
                }
                other => println!("    {other:?}"),
            }
        }
    }
    let tid = host.request(&GpTranslationRequest {
        dst: args.sink,
        start_index: 0,
    });
    let event = remote(
        host,
        "GP_TRANSLATION_REQUEST",
        tid,
        ids::EVT_GP_REMOTE_TRANSLATION_TABLE,
    )?;
    if event.msg_id == ids::EVT_GP_REMOTE_STATUS {
        return expect_success("GP_TRANSLATION_REQUEST", &event);
    }
    let table: EvtGpRemoteTranslationTable = event.raw().decode().map_err(|e| format!("{e:?}"))?;
    println!(
        "  EVT_GP_REMOTE_TRANSLATION_TABLE from 0x{:04X}: status 0x{:02X} total {} start {} flags 0x{:02X}",
        table.src_nwk, table.status, table.total, table.start_index, table.flags
    );
    for record in table.records.iter() {
        println!("    {record:?}");
    }
    Ok(())
}

/// Run the selected GP steps in a fixed order: pair, translate, untranslate,
/// tables, remove, watch.
pub fn run<S: ByteStream>(host: &mut Host<S>, args: &GpArgs) -> Result<(), String> {
    if let Some((key, fc)) = args.pair {
        pairing_configuration(host, args, 1, Some((key, fc)))?;
    }
    if let Some(t) = args.translate {
        translation_update(host, args, 0, t)?;
    }
    if let Some(t) = args.untranslate {
        translation_update(host, args, 2, t)?;
    }
    if args.tables {
        tables(host, args)?;
    }
    if args.remove {
        pairing_configuration(host, args, 4, None)?;
    }
    let deadline = Instant::now() + Duration::from_secs(args.watch_s);
    while Instant::now() < deadline {
        host.round(Duration::from_millis(100))?;
        let events: Vec<Received> = host
            .inbox
            .drain(..)
            .filter(|m| m.kind == Kind::Event)
            .collect();
        for message in &events {
            if report_event(message).is_none() && message.msg_id >> 8 == 0x0A {
                println!("GP event 0x{:04X} {:02X?}", message.msg_id, message.payload);
            }
        }
    }
    Ok(())
}
