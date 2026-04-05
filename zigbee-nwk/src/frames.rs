//! NWK frame construction and parsing.
//!
//! Implements NWK frame header encoding/decoding per Zigbee PRO R22 spec
//! Chapter 3.3. Handles both data frames and command frames.

use zigbee_types::{IeeeAddress, ShortAddress};

/// NWK frame types (2-bit field in Frame Control)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NwkFrameType {
    Data = 0b00,
    Command = 0b01,
    InterPan = 0b11,
}

/// NWK command frame identifiers (Table 3-42)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NwkCommandId {
    RouteRequest = 0x01,
    RouteReply = 0x02,
    NetworkStatus = 0x03,
    Leave = 0x04,
    RouteRecord = 0x05,
    RejoinRequest = 0x06,
    RejoinResponse = 0x07,
    LinkStatus = 0x08,
    NetworkReport = 0x09,
    NetworkUpdate = 0x0A,
    EdTimeoutRequest = 0x0B,
    EdTimeoutResponse = 0x0C,
    LinkPowerDelta = 0x0D,
}

impl NwkCommandId {
    /// Parse a command ID byte. Returns None for unknown IDs.
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x01 => Some(Self::RouteRequest),
            0x02 => Some(Self::RouteReply),
            0x03 => Some(Self::NetworkStatus),
            0x04 => Some(Self::Leave),
            0x05 => Some(Self::RouteRecord),
            0x06 => Some(Self::RejoinRequest),
            0x07 => Some(Self::RejoinResponse),
            0x08 => Some(Self::LinkStatus),
            0x09 => Some(Self::NetworkReport),
            0x0A => Some(Self::NetworkUpdate),
            0x0B => Some(Self::EdTimeoutRequest),
            0x0C => Some(Self::EdTimeoutResponse),
            0x0D => Some(Self::LinkPowerDelta),
            _ => None,
        }
    }
}

/// NWK frame control field (16 bits)
#[derive(Debug, Clone, Copy, Default)]
pub struct NwkFrameControl {
    /// Frame type (2 bits)
    pub frame_type: u8,
    /// Protocol version (4 bits) — always 0x02 for Zigbee PRO
    pub protocol_version: u8,
    /// Discover route (2 bits): 0=suppress, 1=enable
    pub discover_route: u8,
    /// Multicast flag
    pub multicast: bool,
    /// Security enabled
    pub security: bool,
    /// Source route present
    pub source_route: bool,
    /// Destination IEEE address present
    pub dst_ieee_present: bool,
    /// Source IEEE address present
    pub src_ieee_present: bool,
    /// End device initiator
    pub end_device_initiator: bool,
}

impl NwkFrameControl {
    pub fn parse(raw: u16) -> Self {
        Self {
            frame_type: (raw & 0x03) as u8,
            protocol_version: ((raw >> 2) & 0x0F) as u8,
            discover_route: ((raw >> 6) & 0x03) as u8,
            multicast: (raw >> 8) & 1 != 0,
            security: (raw >> 9) & 1 != 0,
            source_route: (raw >> 10) & 1 != 0,
            dst_ieee_present: (raw >> 11) & 1 != 0,
            src_ieee_present: (raw >> 12) & 1 != 0,
            end_device_initiator: (raw >> 13) & 1 != 0,
        }
    }

    pub fn serialize(&self) -> u16 {
        let mut fc: u16 = 0;
        fc |= (self.frame_type as u16) & 0x03;
        fc |= ((self.protocol_version as u16) & 0x0F) << 2;
        fc |= ((self.discover_route as u16) & 0x03) << 6;
        if self.multicast {
            fc |= 1 << 8;
        }
        if self.security {
            fc |= 1 << 9;
        }
        if self.source_route {
            fc |= 1 << 10;
        }
        if self.dst_ieee_present {
            fc |= 1 << 11;
        }
        if self.src_ieee_present {
            fc |= 1 << 12;
        }
        if self.end_device_initiator {
            fc |= 1 << 13;
        }
        fc
    }
}

/// NWK frame header
#[derive(Debug, Clone)]
pub struct NwkHeader {
    pub frame_control: NwkFrameControl,
    pub dst_addr: ShortAddress,
    pub src_addr: ShortAddress,
    pub radius: u8,
    pub seq_number: u8,
    /// Optional destination IEEE address (when dst_ieee_present)
    pub dst_ieee: Option<IeeeAddress>,
    /// Optional source IEEE address (when src_ieee_present)
    pub src_ieee: Option<IeeeAddress>,
    /// Multicast control (when multicast flag set)
    pub multicast_control: Option<u8>,
    /// Source route subframe (when source_route flag set)
    pub source_route: Option<SourceRoute>,
}

/// Source route subframe
#[derive(Debug, Clone)]
pub struct SourceRoute {
    pub relay_count: u8,
    pub relay_index: u8,
    pub relay_list: heapless::Vec<ShortAddress, 16>,
}

impl NwkHeader {
    /// Parse a NWK header from raw bytes. Returns (header, bytes_consumed).
    pub fn parse(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < 8 {
            return None;
        }

        let fc_raw = u16::from_le_bytes([data[0], data[1]]);
        let frame_control = NwkFrameControl::parse(fc_raw);
        let dst_addr = ShortAddress(u16::from_le_bytes([data[2], data[3]]));
        let src_addr = ShortAddress(u16::from_le_bytes([data[4], data[5]]));
        let radius = data[6];
        let seq_number = data[7];

        let mut offset = 8;

        // Optional destination IEEE
        let dst_ieee = if frame_control.dst_ieee_present && data.len() >= offset + 8 {
            let mut addr = [0u8; 8];
            addr.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            Some(addr)
        } else {
            None
        };

        // Optional source IEEE
        let src_ieee = if frame_control.src_ieee_present && data.len() >= offset + 8 {
            let mut addr = [0u8; 8];
            addr.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            Some(addr)
        } else {
            None
        };

        // Multicast control
        let multicast_control = if frame_control.multicast && data.len() > offset {
            let mc = data[offset];
            offset += 1;
            Some(mc)
        } else {
            None
        };

        // Source route subframe
        let source_route = if frame_control.source_route && data.len() >= offset + 2 {
            let relay_count = data[offset];
            let relay_index = data[offset + 1];
            offset += 2;
            let mut relay_list = heapless::Vec::new();
            for i in 0..relay_count {
                if data.len() >= offset + 2 {
                    let relay = ShortAddress(u16::from_le_bytes([data[offset], data[offset + 1]]));
                    if relay_list.push(relay).is_err() {
                        log::warn!(
                            "NWK source route: relay list full (capacity={}, have={})",
                            relay_list.capacity(),
                            i,
                        );
                        break;
                    }
                    offset += 2;
                }
            }
            Some(SourceRoute {
                relay_count,
                relay_index,
                relay_list,
            })
        } else {
            None
        };

        Some((
            Self {
                frame_control,
                dst_addr,
                src_addr,
                radius,
                seq_number,
                dst_ieee,
                src_ieee,
                multicast_control,
                source_route,
            },
            offset,
        ))
    }

    /// Serialize the NWK header into a buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let fc = self.frame_control.serialize();
        buf[0] = (fc & 0xFF) as u8;
        buf[1] = ((fc >> 8) & 0xFF) as u8;
        buf[2] = (self.dst_addr.0 & 0xFF) as u8;
        buf[3] = ((self.dst_addr.0 >> 8) & 0xFF) as u8;
        buf[4] = (self.src_addr.0 & 0xFF) as u8;
        buf[5] = ((self.src_addr.0 >> 8) & 0xFF) as u8;
        buf[6] = self.radius;
        buf[7] = self.seq_number;

        let mut offset = 8;

        if let Some(ref ieee) = self.dst_ieee {
            buf[offset..offset + 8].copy_from_slice(ieee);
            offset += 8;
        }
        if let Some(ref ieee) = self.src_ieee {
            buf[offset..offset + 8].copy_from_slice(ieee);
            offset += 8;
        }
        if let Some(mc) = self.multicast_control {
            buf[offset] = mc;
            offset += 1;
        }
        if let Some(ref sr) = self.source_route {
            buf[offset] = sr.relay_count;
            buf[offset + 1] = sr.relay_index;
            offset += 2;
            for relay in &sr.relay_list {
                buf[offset] = (relay.0 & 0xFF) as u8;
                buf[offset + 1] = ((relay.0 >> 8) & 0xFF) as u8;
                offset += 2;
            }
        }

        offset
    }
}

/// NWK command payloads
/// Leave command (NWK command ID 0x04)
#[derive(Debug, Clone, Copy)]
pub struct LeaveCommand {
    /// Request = device wants to leave; Indication = device told to leave
    pub remove_children: bool,
    pub rejoin: bool,
}

impl LeaveCommand {
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        Some(Self {
            remove_children: data[0] & (1 << 6) != 0,
            rejoin: data[0] & (1 << 5) != 0,
        })
    }

    pub fn serialize(&self) -> u8 {
        let mut cmd_opts: u8 = 0;
        if self.remove_children {
            cmd_opts |= 1 << 6;
        }
        if self.rejoin {
            cmd_opts |= 1 << 5;
        }
        cmd_opts
    }
}

/// Rejoin Request command (NWK command ID 0x06)
#[derive(Debug, Clone, Copy)]
pub struct RejoinRequest {
    pub capability_info: u8,
}

/// Rejoin Response command (NWK command ID 0x07)
#[derive(Debug, Clone, Copy)]
pub struct RejoinResponse {
    pub short_address: ShortAddress,
    pub rejoin_status: u8,
}

impl RejoinResponse {
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 3 {
            return None;
        }
        Some(Self {
            short_address: ShortAddress(u16::from_le_bytes([data[0], data[1]])),
            rejoin_status: data[2],
        })
    }
}

/// Route Request command (NWK command ID 0x01)
#[derive(Debug, Clone)]
pub struct RouteRequest {
    pub command_options: u8,
    pub route_request_id: u8,
    pub dst_addr: ShortAddress,
    pub path_cost: u8,
    pub dst_ieee: Option<IeeeAddress>,
}

impl RouteRequest {
    /// Parse a Route Request from the payload (after command ID byte).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 5 {
            return None;
        }
        let command_options = data[0];
        let route_request_id = data[1];
        let dst_addr = ShortAddress(u16::from_le_bytes([data[2], data[3]]));
        let path_cost = data[4];
        let dst_ieee = if command_options & (1 << 5) != 0 && data.len() >= 13 {
            let mut addr = [0u8; 8];
            addr.copy_from_slice(&data[5..13]);
            Some(addr)
        } else {
            None
        };
        Some(Self {
            command_options,
            route_request_id,
            dst_addr,
            path_cost,
            dst_ieee,
        })
    }

    /// Serialize to buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.command_options;
        buf[1] = self.route_request_id;
        buf[2] = (self.dst_addr.0 & 0xFF) as u8;
        buf[3] = ((self.dst_addr.0 >> 8) & 0xFF) as u8;
        buf[4] = self.path_cost;
        let mut offset = 5;
        if let Some(ref ieee) = self.dst_ieee {
            buf[offset..offset + 8].copy_from_slice(ieee);
            offset += 8;
        }
        offset
    }
}

/// Route Reply command (NWK command ID 0x02)
#[derive(Debug, Clone)]
pub struct RouteReply {
    pub command_options: u8,
    pub route_request_id: u8,
    pub originator: ShortAddress,
    pub responder: ShortAddress,
    pub path_cost: u8,
    pub originator_ieee: Option<IeeeAddress>,
    pub responder_ieee: Option<IeeeAddress>,
}

impl RouteReply {
    /// Parse a Route Reply from the payload (after command ID byte).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 7 {
            return None;
        }
        let command_options = data[0];
        let route_request_id = data[1];
        let originator = ShortAddress(u16::from_le_bytes([data[2], data[3]]));
        let responder = ShortAddress(u16::from_le_bytes([data[4], data[5]]));
        let path_cost = data[6];
        let mut offset = 7;
        let originator_ieee = if command_options & (1 << 4) != 0 && data.len() >= offset + 8 {
            let mut addr = [0u8; 8];
            addr.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            Some(addr)
        } else {
            None
        };
        let responder_ieee = if command_options & (1 << 5) != 0 && data.len() >= offset + 8 {
            let mut addr = [0u8; 8];
            addr.copy_from_slice(&data[offset..offset + 8]);
            Some(addr)
        } else {
            None
        };
        Some(Self {
            command_options,
            route_request_id,
            originator,
            responder,
            path_cost,
            originator_ieee,
            responder_ieee,
        })
    }

    /// Serialize to buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.command_options;
        buf[1] = self.route_request_id;
        buf[2] = (self.originator.0 & 0xFF) as u8;
        buf[3] = ((self.originator.0 >> 8) & 0xFF) as u8;
        buf[4] = (self.responder.0 & 0xFF) as u8;
        buf[5] = ((self.responder.0 >> 8) & 0xFF) as u8;
        buf[6] = self.path_cost;
        let mut offset = 7;
        if let Some(ref ieee) = self.originator_ieee {
            buf[offset..offset + 8].copy_from_slice(ieee);
            offset += 8;
        }
        if let Some(ref ieee) = self.responder_ieee {
            buf[offset..offset + 8].copy_from_slice(ieee);
            offset += 8;
        }
        offset
    }
}

/// Network Status command (NWK command ID 0x03)
///
/// Sent when a routing error occurs (e.g., relay failure, no route available).
/// See Zigbee spec Table 3-44.
#[derive(Debug, Clone, Copy)]
pub struct NetworkStatusCommand {
    /// Status code (Table 3-45):
    /// 0x00 = No route available
    /// 0x01 = Tree link failure
    /// 0x02 = Non-tree link failure
    /// 0x03 = Low battery level
    /// 0x04 = No routing capacity
    /// 0x05 = No indirect capacity
    /// 0x06 = Indirect transaction expiry
    /// 0x07 = Target device unavailable
    /// 0x08 = Target address unallocated
    /// 0x09 = Parent link failure
    /// 0x0B = Address conflict
    /// 0x0C = Route discovery failed
    /// 0x0D = Route validation error
    pub status_code: u8,
    /// Destination address that triggered the error
    pub destination: ShortAddress,
}

impl NetworkStatusCommand {
    pub const NO_ROUTE_AVAILABLE: u8 = 0x00;
    pub const TREE_LINK_FAILURE: u8 = 0x01;
    pub const NON_TREE_LINK_FAILURE: u8 = 0x02;

    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 3 {
            return None;
        }
        Some(Self {
            status_code: data[0],
            destination: ShortAddress(u16::from_le_bytes([data[1], data[2]])),
        })
    }

    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.status_code;
        buf[1] = (self.destination.0 & 0xFF) as u8;
        buf[2] = ((self.destination.0 >> 8) & 0xFF) as u8;
        3
    }
}

/// Link status entry (one neighbor's link quality)
#[derive(Debug, Clone, Copy)]
pub struct LinkStatusEntry {
    pub address: ShortAddress,
    pub incoming_cost: u8,
    pub outgoing_cost: u8,
}

/// Link Status command (NWK command ID 0x08)
#[derive(Debug, Clone)]
pub struct LinkStatusCommand {
    pub entries: heapless::Vec<LinkStatusEntry, 16>,
}

impl LinkStatusCommand {
    /// Parse Link Status from payload (after command ID byte).
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        let count = (data[0] & 0x1F) as usize;
        let mut entries = heapless::Vec::new();
        let mut offset = 1;
        for _ in 0..count {
            if data.len() < offset + 3 {
                break;
            }
            let address = ShortAddress(u16::from_le_bytes([data[offset], data[offset + 1]]));
            let cost_byte = data[offset + 2];
            let incoming_cost = cost_byte & 0x07;
            let outgoing_cost = (cost_byte >> 3) & 0x07;
            let _ = entries.push(LinkStatusEntry {
                address,
                incoming_cost,
                outgoing_cost,
            });
            offset += 3;
        }
        Some(Self { entries })
    }

    /// Serialize to buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.entries.len() as u8;
        let mut offset = 1;
        for entry in &self.entries {
            buf[offset] = (entry.address.0 & 0xFF) as u8;
            buf[offset + 1] = ((entry.address.0 >> 8) & 0xFF) as u8;
            buf[offset + 2] = (entry.incoming_cost & 0x07) | ((entry.outgoing_cost & 0x07) << 3);
            offset += 3;
        }
        offset
    }
}
