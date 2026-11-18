//! DIAG group (`0x07xx`, endpoint SYSTEM), spec 7.8.

use crate::codec::{List, wire_struct};
use crate::msg::ids;
use crate::msg::message_def;
use crate::types::Ieee;

/// `group_mask` bits.
pub mod group {
    /// Link counters (`0x01xx`).
    pub const LINK: u8 = 0x01;
    /// MAC (`0x02xx`).
    pub const MAC: u8 = 0x02;
    /// NWK (`0x03xx`).
    pub const NWK: u8 = 0x04;
    /// APS (`0x04xx`).
    pub const APS: u8 = 0x08;
    /// Security / TC (`0x05xx`).
    pub const SECURITY: u8 = 0x10;
    /// Tables (`0x06xx`).
    pub const TABLES: u8 = 0x20;
    /// Queues (`0x07xx`).
    pub const QUEUES: u8 = 0x40;
}

wire_struct! {
    /// `DIAG_GET_COUNTERS` request.
    pub struct DiagGetCounters {
        /// [`group`] mask.
        pub group_mask: u8,
    }
}

wire_struct! {
    /// `{u16 id, u32 value}`.
    pub struct Counter {
        /// Counter ID.
        pub id: u16,
        /// Value.
        pub value: u32,
    }
}

wire_struct! {
    /// `DIAG_GET_COUNTERS` response body.
    pub struct DiagGetCountersRsp<'a> {
        /// Counters.
        pub counters: List<'a, Counter>,
    }
}

wire_struct! {
    /// `DIAG_RESET_COUNTERS` request.
    pub struct DiagResetCounters {
        /// [`group`] mask.
        pub group_mask: u8,
    }
}

wire_struct! {
    /// `DIAG_GET_NEIGHBORS` request.
    pub struct DiagGetNeighbors {
        /// First index.
        pub start: u8,
    }
}

wire_struct! {
    /// `DIAG_GET_ROUTES` request.
    pub struct DiagGetRoutes {
        /// First index.
        pub start: u8,
    }
}

wire_struct! {
    /// `DIAG_GET_CHILDREN` request.
    pub struct DiagGetChildren {
        /// First index.
        pub start: u8,
    }
}

wire_struct! {
    /// `DIAG_GET_SOURCE_ROUTES` request.
    pub struct DiagGetSourceRoutes {
        /// First index.
        pub start: u8,
    }
}

wire_struct! {
    /// Neighbor entry.
    pub struct Neighbor {
        /// IEEE.
        pub ieee: Ieee,
        /// Short address.
        pub nwk: u16,
        /// Device type.
        pub device_type: u8,
        /// RX on when idle.
        pub rx_on: u8,
        /// Relationship.
        pub relationship: u8,
        /// LQI.
        pub lqi: u8,
        /// Outgoing cost.
        pub outgoing_cost: u8,
        /// Age.
        pub age: u8,
    }
}

wire_struct! {
    /// `DIAG_GET_NEIGHBORS` response body.
    pub struct DiagGetNeighborsRsp<'a> {
        /// Total entries.
        pub total: u8,
        /// Entries from `start`.
        pub entries: List<'a, Neighbor>,
    }
}

wire_struct! {
    /// Route entry.
    pub struct Route {
        /// Destination.
        pub dst: u16,
        /// Next hop.
        pub next_hop: u16,
        /// Status.
        pub status: u8,
        /// Flags.
        pub flags: u8,
    }
}

wire_struct! {
    /// `DIAG_GET_ROUTES` response body.
    pub struct DiagGetRoutesRsp<'a> {
        /// Total entries.
        pub total: u8,
        /// Entries.
        pub entries: List<'a, Route>,
    }
}

wire_struct! {
    /// Child entry.
    pub struct Child {
        /// IEEE.
        pub ieee: Ieee,
        /// Short address.
        pub nwk: u16,
        /// Capability.
        pub capability: u8,
        /// Timeout index.
        pub timeout_exp: u8,
        /// Time since last poll.
        pub last_poll_ms: u32,
    }
}

wire_struct! {
    /// `DIAG_GET_CHILDREN` response body.
    pub struct DiagGetChildrenRsp<'a> {
        /// Total entries.
        pub total: u8,
        /// Entries.
        pub entries: List<'a, Child>,
    }
}

wire_struct! {
    /// `{u16 dst, u8 k, k×u16}` cached source route.
    pub struct SourceRoute<'a> {
        /// Destination.
        pub dst: u16,
        /// Relays.
        pub relays: List<'a, u16>,
    }
}

wire_struct! {
    /// `DIAG_GET_SOURCE_ROUTES` response body.
    pub struct DiagGetSourceRoutesRsp<'a> {
        /// Total entries.
        pub total: u8,
        /// Entries.
        pub entries: List<'a, SourceRoute<'a>>,
    }
}

message_def! {
    DiagGetCounters = ids::DIAG_GET_COUNTERS, Request;
    DiagGetCountersRsp<'a> = ids::DIAG_GET_COUNTERS, Response;
    DiagResetCounters = ids::DIAG_RESET_COUNTERS, Request;
    DiagGetNeighbors = ids::DIAG_GET_NEIGHBORS, Request;
    DiagGetRoutes = ids::DIAG_GET_ROUTES, Request;
    DiagGetChildren = ids::DIAG_GET_CHILDREN, Request;
    DiagGetSourceRoutes = ids::DIAG_GET_SOURCE_ROUTES, Request;
    DiagGetNeighborsRsp<'a> = ids::DIAG_GET_NEIGHBORS, Response;
    DiagGetRoutesRsp<'a> = ids::DIAG_GET_ROUTES, Response;
    DiagGetChildrenRsp<'a> = ids::DIAG_GET_CHILDREN, Response;
    DiagGetSourceRoutesRsp<'a> = ids::DIAG_GET_SOURCE_ROUTES, Response;
}
