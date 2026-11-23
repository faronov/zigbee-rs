//! Hardware checks of the `ncp-host-gaps` commands: DIAG tables, group
//! membership, channel change, install codes and `SYS_RESET` under load.

use std::time::{Duration, Instant};

use zigbee_zrhp::msg::diag::{
    DiagGetChildren, DiagGetChildrenRsp, DiagGetNeighbors, DiagGetNeighborsRsp, DiagGetRoutes,
    DiagGetRoutesRsp,
};
use zigbee_zrhp::msg::diag::{DiagGetCounters, DiagGetCountersRsp};
use zigbee_zrhp::msg::ids;
use zigbee_zrhp::msg::network::{
    EvtChannelChanged, EvtEnergyScanResult, NwkActiveScan, NwkActiveScanRsp, NwkChangeChannel,
    NwkEnergyScan, NwkGetInfoRsp, NwkGroupMembership, NwkSetTxPower, NwkSetTxPowerRsp,
};
use zigbee_zrhp::msg::security::{SecAddInstallCode, SecAddInstallCodeRsp};
use zigbee_zrhp::msg::system::{EvtNcpReady, SysEcho, SysReset};
use zigbee_zrhp::types::Ieee;

use super::{ByteStream, Host, body, data_request};
use zigbee_zrhp::codec::{Bytes8, Rest};

/// Every page of `DIAG_GET_NEIGHBORS`, `DIAG_GET_ROUTES`, `DIAG_GET_CHILDREN`.
pub fn diag<S: ByteStream>(host: &mut Host<S>) -> Result<(), String> {
    let mut start = 0u8;
    loop {
        let tid = host.request(&DiagGetNeighbors { start });
        let rsp = host.response(tid)?;
        let page: DiagGetNeighborsRsp = body(&rsp)?;
        let n = page.entries.iter().count();
        for e in page.entries.iter() {
            println!(
                "DIAG neighbor: ieee {:02X?} nwk 0x{:04X} type {} rx_on {} rel {} lqi {} cost {} age {}",
                e.ieee.0,
                e.nwk,
                e.device_type,
                e.rx_on,
                e.relationship,
                e.lqi,
                e.outgoing_cost,
                e.age
            );
        }
        println!(
            "DIAG_GET_NEIGHBORS start {start}: total {} n {n}",
            page.total
        );
        start = start.saturating_add(n as u8);
        if n == 0 || start >= page.total {
            break;
        }
    }
    let tid = host.request(&DiagGetRoutes { start: 0 });
    let rsp = host.response(tid)?;
    let routes: DiagGetRoutesRsp = body(&rsp)?;
    for e in routes.entries.iter() {
        println!(
            "DIAG route: dst 0x{:04X} next 0x{:04X} status {} flags 0x{:02X}",
            e.dst, e.next_hop, e.status, e.flags
        );
    }
    println!("DIAG_GET_ROUTES: total {}", routes.total);
    let tid = host.request(&DiagGetChildren { start: 0 });
    let rsp = host.response(tid)?;
    let children: DiagGetChildrenRsp = body(&rsp)?;
    for e in children.entries.iter() {
        println!(
            "DIAG child: ieee {:02X?} nwk 0x{:04X} cap 0x{:02X} timeout_exp {} last_poll_ms {}",
            e.ieee.0, e.nwk, e.capability, e.timeout_exp, e.last_poll_ms
        );
    }
    println!("DIAG_GET_CHILDREN: total {}", children.total);
    Ok(())
}

/// `NWK_GROUP_MEMBERSHIP{op, group, ep}`.
pub fn group<S: ByteStream>(host: &mut Host<S>, op: u8, group: u16, ep: u8) -> Result<(), String> {
    let tid = host.request(&NwkGroupMembership { op, group, ep });
    let rsp = host.response(tid)?;
    println!(
        "NWK_GROUP_MEMBERSHIP op {op} group 0x{group:04X} ep {ep}: {:?}",
        rsp.status()
    );
    if rsp.status().is_success() {
        Ok(())
    } else {
        Err("group membership refused".into())
    }
}

/// `NWK_CHANGE_CHANNEL`, then wait for `EVT_CHANNEL_CHANGED`.
pub fn change_channel<S: ByteStream>(host: &mut Host<S>, channel: u8) -> Result<(), String> {
    let started = Instant::now();
    let tid = host.request(&NwkChangeChannel {
        channel,
        nwk_update_id: 0,
    });
    let rsp = host.response(tid)?;
    println!(
        "NWK_CHANGE_CHANNEL {channel}: {:?} after {:?}",
        rsp.status(),
        started.elapsed()
    );
    if !rsp.status().is_success() {
        return Err("channel change refused".into());
    }
    let event = host.event(ids::EVT_CHANNEL_CHANGED, Duration::from_secs(20))?;
    let event: EvtChannelChanged = event.raw().decode().map_err(|e| format!("{e:?}"))?;
    println!(
        "EVT_CHANNEL_CHANGED: channel {} update_id {} req_tid {} after {:?}",
        event.channel,
        event.nwk_update_id,
        event.req_tid,
        started.elapsed()
    );
    if event.channel != channel {
        return Err("radio stayed on the old channel".into());
    }
    Ok(())
}

/// `SEC_ADD_INSTALL_CODE` for `ieee` with the R22 example install code
/// (`83FED340 7A939723 A5C639B2 6916D505`, CRC `C3B5`), then a bad CRC.
pub fn install_code<S: ByteStream>(host: &mut Host<S>, ieee: [u8; 8]) -> Result<(), String> {
    let mut code = [
        0x83u8, 0xFE, 0xD3, 0x40, 0x7A, 0x93, 0x97, 0x23, 0xA5, 0xC6, 0x39, 0xB2, 0x69, 0x16, 0xD5,
        0x05, 0xC3, 0xB5,
    ]
    .to_vec();
    let tid = host.request(&SecAddInstallCode {
        ieee: Ieee(ieee),
        code: Bytes8(&code),
    });
    let rsp = host.response(tid)?;
    let key: SecAddInstallCodeRsp = body(&rsp)?;
    println!("SEC_ADD_INSTALL_CODE: derived key {:02X?}", key.derived_key);
    code[0] ^= 1;
    let tid = host.request(&SecAddInstallCode {
        ieee: Ieee(ieee),
        code: Bytes8(&code),
    });
    let rsp = host.response(tid)?;
    println!("SEC_ADD_INSTALL_CODE (bad CRC): {:?}", rsp.status());
    Ok(())
}

/// Queue ZCL reads to a sleepy `nwk` and echoes, then `SYS_RESET` at once;
/// time until the NCP announces itself again.
pub fn reset_under_load<S: ByteStream>(host: &mut Host<S>, nwk: u16, ep: u8) -> Result<(), String> {
    for seq in 0..4u8 {
        let zcl = [0x00, 0xA0 + seq, 0x00, 0x04, 0x00];
        data_request(host, nwk, ep, 1, 0x0104, 0x0000, &zcl);
    }
    for i in 0..8u8 {
        host.request(&SysEcho {
            data: Rest(&[i; 64]),
        });
    }
    let started = Instant::now();
    let tid = host.request(&SysReset {
        mode: 0,
        confirm: 0,
    });
    let rsp = host.response(tid)?;
    println!(
        "SYS_RESET: {:?} after {:?}",
        rsp.status(),
        started.elapsed()
    );
    let ready = host.event(ids::EVT_NCP_READY, Duration::from_secs(40))?;
    let ready: EvtNcpReady = ready.raw().decode().map_err(|e| format!("{e:?}"))?;
    println!(
        "EVT_NCP_READY after {:?}: cookie 0x{:08X} reset_reason {} network_state {}",
        started.elapsed(),
        ready.ncp_cookie,
        ready.reset_reason,
        ready.network_state
    );
    Ok(())
}

/// `TBL_INVALIDATE` of a DEVICE record (removes a provisioned key).
pub fn invalidate<S: ByteStream>(host: &mut Host<S>, ieee: [u8; 8]) -> Result<(), String> {
    use zigbee_zrhp::msg::tables::{TableKey, TblInvalidate};
    let tid = host.request(&TblInvalidate {
        table: zigbee_zrhp::records::record_type::DEVICE,
        key: TableKey::Ieee(Ieee(ieee)),
    });
    let rsp = host.response(tid)?;
    println!("TBL_INVALIDATE {ieee:02X?}: {:?}", rsp.status());
    Ok(())
}

/// Print the events of `secs` seconds (permit-join state, indications).
fn watch<S: ByteStream>(host: &mut Host<S>, secs: u64) -> Result<(), String> {
    let deadline = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < deadline {
        host.round(Duration::from_millis(100))?;
        let events: Vec<_> = host
            .inbox
            .drain(..)
            .filter(|m| m.kind == zigbee_zrhp::msg::Kind::Event)
            .collect();
        for message in &events {
            super::report_event(message);
        }
    }
    Ok(())
}

/// `NWK_PERMIT_JOIN{secs, NCP only}`, then watch the window to its expiry.
pub fn permit<S: ByteStream>(host: &mut Host<S>, secs: u8) -> Result<(), String> {
    use zigbee_zrhp::msg::network::NwkPermitJoin;
    let started = Instant::now();
    let tid = host.request(&NwkPermitJoin {
        duration_s: secs,
        target: 0x0000,
        flags: 0,
    });
    let rsp = host.response(tid)?;
    println!(
        "NWK_PERMIT_JOIN {secs} s: {:?} after {:?}",
        rsp.status(),
        started.elapsed()
    );
    watch(host, u64::from(secs) + 4)
}

/// ZCL Identify to `nwk`/`ep`: `spec` is IdentifyTime seconds (then an
/// Identify Query while it runs), or `effect:N` for Trigger Effect N.
pub fn identify<S: ByteStream>(
    host: &mut Host<S>,
    nwk: u16,
    ep: u8,
    spec: &str,
) -> Result<(), String> {
    // Cluster-specific, client to server, default response enabled.
    let (command, payload, secs): (u8, Vec<u8>, u64) = match spec.strip_prefix("effect:") {
        Some(effect) => {
            let effect: u8 = effect.parse().map_err(|_| "--identify effect:N")?;
            (0x40, vec![effect, 0x00], 20)
        }
        None => {
            let secs: u16 = spec.parse().map_err(|_| "--identify SECS")?;
            (0x00, secs.to_le_bytes().to_vec(), u64::from(secs) + 10)
        }
    };
    let mut zcl = vec![0x01, 0x31, command];
    zcl.extend_from_slice(&payload);
    data_request(host, nwk, ep, 1, 0x0104, 0x0003, &zcl);
    println!("-> Identify cmd 0x{command:02X} {payload:02X?} to 0x{nwk:04X} ep {ep}");
    watch(host, 15)?;
    if command == 0x00 {
        data_request(host, nwk, ep, 1, 0x0104, 0x0003, &[0x01, 0x32, 0x01]);
        println!("-> Identify Query");
    }
    watch(host, secs.saturating_sub(15))
}

/// `NWK_ENERGY_SCAN` of `channel_mask`, then its `EVT_ENERGY_SCAN_RESULT`.
pub fn energy_scan<S: ByteStream>(
    host: &mut Host<S>,
    channel_mask: u32,
    duration_exp: u8,
    count: u8,
) -> Result<(), String> {
    let started = Instant::now();
    let tid = host.request(&NwkEnergyScan {
        channel_mask,
        duration_exp,
        count,
    });
    let rsp = host.response(tid)?;
    println!(
        "NWK_ENERGY_SCAN 0x{channel_mask:08X} exp {duration_exp} x{count}: {:?} after {:?}",
        rsp.status(),
        started.elapsed()
    );
    if !rsp.status().is_success() {
        return Err("energy scan refused".into());
    }
    let event = host.event(ids::EVT_ENERGY_SCAN_RESULT, Duration::from_secs(70))?;
    let event: EvtEnergyScanResult = event.raw().decode().map_err(|e| format!("{e:?}"))?;
    println!(
        "EVT_ENERGY_SCAN_RESULT: req_tid {} status {} after {:?}",
        event.req_tid,
        event.status,
        started.elapsed()
    );
    for r in event.results.iter() {
        println!(
            "  channel {:2}: energy {:3} max_rssi {:4} dBm",
            r.channel, r.energy, r.max_rssi_dbm
        );
    }
    if event.req_tid != tid || event.status != 0 {
        return Err("energy scan failed".into());
    }
    Ok(())
}

/// `NWK_SET_TX_POWER{dbm}`, then `NWK_GET_INFO.tx_power`.
pub fn tx_power<S: ByteStream>(host: &mut Host<S>, dbm: i8) -> Result<(), String> {
    let tid = host.request(&NwkSetTxPower { dbm });
    let rsp = host.response(tid)?;
    let applied: NwkSetTxPowerRsp = body(&rsp)?;
    println!(
        "NWK_SET_TX_POWER {dbm}: applied {} dBm",
        applied.applied_dbm
    );
    let tid = host.request_raw(ids::NWK_GET_INFO, &[]);
    let rsp = host.response(tid)?;
    let info: NwkGetInfoRsp = body(&rsp)?;
    println!("NWK_GET_INFO: tx_power {} dBm", info.tx_power);
    if info.tx_power != applied.applied_dbm {
        return Err("NWK_GET_INFO does not report the applied power".into());
    }
    Ok(())
}

/// `DIAG_GET_COUNTERS{group_mask}`.
pub fn counters<S: ByteStream>(host: &mut Host<S>, group_mask: u8) -> Result<(), String> {
    let tid = host.request(&DiagGetCounters { group_mask });
    let rsp = host.response(tid)?;
    let counters: DiagGetCountersRsp = body(&rsp)?;
    println!("DIAG_GET_COUNTERS 0x{group_mask:02X}:");
    for c in counters.counters.iter() {
        println!("  0x{:04X} = {}", c.id, c.value);
    }
    Ok(())
}

/// `NWK_ACTIVE_SCAN` of `channel_mask` (synchronous response).
pub fn active_scan<S: ByteStream>(
    host: &mut Host<S>,
    channel_mask: u32,
    duration_exp: u8,
) -> Result<(), String> {
    let started = Instant::now();
    let tid = host.request(&NwkActiveScan {
        channel_mask,
        duration_exp,
    });
    let rsp = host.response(tid)?;
    println!(
        "NWK_ACTIVE_SCAN 0x{channel_mask:08X} exp {duration_exp}: {:?} after {:?}",
        rsp.status(),
        started.elapsed()
    );
    let beacons: NwkActiveScanRsp = body(&rsp)?;
    for b in beacons.results.iter() {
        println!(
            "  channel {:2} pan 0x{:04X} epid {:02X?} src 0x{:04X} lqi {:3} rssi {:4} permit {} profile {} update {}",
            b.channel, b.pan, b.epid, b.src, b.lqi, b.rssi, b.permit, b.stack_profile, b.update_id
        );
    }
    Ok(())
}

/// `SYS_SET_CONFIG{coex_tx_priority_mask, coex_enable}` (spec 7.1, 11.5).
pub fn coex<S: ByteStream>(host: &mut Host<S>, enable: bool, mask: u8) -> Result<(), String> {
    use zigbee_zrhp::codec::List;
    use zigbee_zrhp::msg::system::{ConfigValue, SysSetConfig, SysSetConfigRsp, config};
    let mask_value = [mask];
    let enable_value = [u8::from(enable)];
    let values = [
        ConfigValue {
            id: config::COEX_TX_PRIORITY_MASK,
            value: Bytes8(&mask_value),
        },
        ConfigValue {
            id: config::COEX_ENABLE,
            value: Bytes8(&enable_value),
        },
    ];
    let tid = host.request(&SysSetConfig {
        values: List::new(&values),
    });
    let rsp = host.response(tid)?;
    let results: SysSetConfigRsp = body(&rsp)?;
    let statuses: Vec<_> = results.results.iter().map(|r| r.status).collect();
    println!("SYS_SET_CONFIG coex_enable {enable} mask 0x{mask:02X}: {statuses:?}");
    Ok(())
}
