//! Hardware checks of the `ncp-host-gaps` commands: DIAG tables, group
//! membership, channel change, install codes and `SYS_RESET` under load.

use std::time::{Duration, Instant};

use zigbee_zrhp::msg::diag::{
    DiagGetChildren, DiagGetChildrenRsp, DiagGetNeighbors, DiagGetNeighborsRsp, DiagGetRoutes,
    DiagGetRoutesRsp,
};
use zigbee_zrhp::msg::ids;
use zigbee_zrhp::msg::network::{EvtChannelChanged, NwkChangeChannel, NwkGroupMembership};
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
