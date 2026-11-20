//! `DIAG_GET_NEIGHBORS` / `DIAG_GET_ROUTES` / `DIAG_GET_CHILDREN` (spec
//! 7.8): the coordinator's own neighbour, route and child tables, paged by
//! `start`, for the hosts' Mgmt_Lqi / Mgmt_Rtg of the coordinator (zigpy
//! topology scan, Zigbee2MQTT network map).

use zigbee_zrhp::codec::Wire;
use zigbee_zrhp::msg::diag::{DiagGetChildren, DiagGetNeighbors, DiagGetRoutes};
use zigbee_zrhp::msg::{Kind, RawMessage, ids};
use zigbee_zrhp::queue::EndpointQueues;
use zigbee_zrhp::types::{Endpoint, Status};

use crate::ncp::QUEUE_BYTES;
use crate::port::{CoordinatorPort, DiagTable};

/// Entries per response page (18-byte neighbour/child entries: 288 B).
pub const PAGE: usize = 16;

pub fn is_diag_table_request(id: u16) -> bool {
    matches!(
        id,
        ids::DIAG_GET_NEIGHBORS | ids::DIAG_GET_ROUTES | ids::DIAG_GET_CHILDREN
    )
}

/// Answer one table page. Own frame: entries are written straight into the
/// queue, no page buffer on the stack.
#[inline(never)]
pub fn dispatch_diag_table<C: CoordinatorPort>(
    queues: &mut EndpointQueues<QUEUE_BYTES>,
    endpoint: Endpoint,
    message: &RawMessage<'_>,
    port: &C,
) {
    let id = message.header.msg_id;
    let tid = message.header.tid;
    let (table, start) = match id {
        ids::DIAG_GET_NEIGHBORS => (
            DiagTable::Neighbors,
            message.decode::<DiagGetNeighbors>().map(|r| r.start),
        ),
        ids::DIAG_GET_ROUTES => (
            DiagTable::Routes,
            message.decode::<DiagGetRoutes>().map(|r| r.start),
        ),
        _ => (
            DiagTable::Children,
            message.decode::<DiagGetChildren>().map(|r| r.start),
        ),
    };
    let Ok(start) = start else {
        let _ = queues
            .queue(endpoint)
            .push_with(|w| w.response_status(id, tid, Status::InvalidParameter));
        return;
    };
    let start = usize::from(start);
    let total = port.diag_count(table).min(usize::from(u8::MAX));
    let count = total.saturating_sub(start).min(PAGE);
    let _ = queues.queue(endpoint).push_with(|w| {
        w.push_with(Kind::Response, false, id, tid, |w| {
            w.u8(Status::Success.to_u8())?;
            w.u8(total as u8)?;
            w.u8(count as u8)?;
            for index in start..start + count {
                match table {
                    DiagTable::Neighbors => port
                        .diag_neighbor(index)
                        .unwrap_or(EMPTY_NEIGHBOR)
                        .write(w)?,
                    DiagTable::Routes => port.diag_route(index).unwrap_or(EMPTY_ROUTE).write(w)?,
                    DiagTable::Children => {
                        port.diag_child(index).unwrap_or(EMPTY_CHILD).write(w)?
                    }
                }
            }
            Ok(())
        })
    });
}

const EMPTY_NEIGHBOR: zigbee_zrhp::msg::diag::Neighbor = zigbee_zrhp::msg::diag::Neighbor {
    ieee: zigbee_zrhp::types::Ieee([0; 8]),
    nwk: 0xFFFF,
    device_type: 3,
    rx_on: 2,
    relationship: 3,
    lqi: 0,
    outgoing_cost: 0,
    age: 0,
};

const EMPTY_ROUTE: zigbee_zrhp::msg::diag::Route = zigbee_zrhp::msg::diag::Route {
    dst: 0xFFFF,
    next_hop: 0xFFFF,
    status: 3,
    flags: 0,
};

const EMPTY_CHILD: zigbee_zrhp::msg::diag::Child = zigbee_zrhp::msg::diag::Child {
    ieee: zigbee_zrhp::types::Ieee([0; 8]),
    nwk: 0xFFFF,
    capability: 0,
    timeout_exp: 0,
    last_poll_ms: u32::MAX,
};
