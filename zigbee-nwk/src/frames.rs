//! NWK frame construction and parsing.
//!
//! Implements NWK frame header encoding/decoding per Zigbee PRO R22 spec
//! Chapter 3.3. Handles both data frames and command frames.

use zigbee_types::{IeeeAddress, PanId, ShortAddress};

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
        let source_route = if frame_control.source_route {
            if data.len() < offset + 2 {
                return None;
            }
            let relay_count = data[offset];
            let relay_index = data[offset + 1];
            offset += 2;
            let relay_count_usize = relay_count as usize;
            if relay_count == 0
                || relay_index >= relay_count
                || relay_count_usize > 16
                || data.len() < offset + relay_count_usize * 2
            {
                return None;
            }
            let mut relay_list = heapless::Vec::new();
            for _ in 0..relay_count {
                let relay = ShortAddress(u16::from_le_bytes([data[offset], data[offset + 1]]));
                relay_list.push(relay).ok()?;
                offset += 2;
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
    pub remove_children: bool,
    /// `true` asks the receiver to leave; `false` indicates the sender left.
    pub request: bool,
    pub rejoin: bool,
}

impl LeaveCommand {
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.is_empty() {
            return None;
        }
        Some(Self {
            remove_children: data[0] & (1 << 7) != 0,
            request: data[0] & (1 << 6) != 0,
            rejoin: data[0] & (1 << 5) != 0,
        })
    }

    pub fn serialize(&self) -> u8 {
        let mut cmd_opts: u8 = 0;
        if self.remove_children {
            cmd_opts |= 1 << 7;
        }
        if self.request {
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
/// See Zigbee R22 Table 3-51.
#[derive(Debug, Clone, Copy)]
pub struct NetworkStatusCommand {
    /// Status code (Table 3-51):
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
    /// 0x0A = Validate route
    /// 0x0B = Source route failure
    /// 0x0C = Many-to-one route failure
    /// 0x0D = Address conflict
    pub status_code: u8,
    /// Destination address that triggered the error
    pub destination: ShortAddress,
}

impl NetworkStatusCommand {
    pub const NO_ROUTE_AVAILABLE: u8 = 0x00;
    pub const TREE_LINK_FAILURE: u8 = 0x01;
    pub const NON_TREE_LINK_FAILURE: u8 = 0x02;
    pub const SOURCE_ROUTE_FAILURE: u8 = 0x0B;
    pub const MANY_TO_ONE_ROUTE_FAILURE: u8 = 0x0C;
    /// R22 Table 3-51 — the destination address field names an address that
    /// two devices in this network are using (§3.6.1.9.3).
    pub const ADDRESS_CONFLICT: u8 = 0x0D;
    /// R22 Table 3-51 — PAN identifier update, reported to the next higher
    /// layer when a Network Update command changes the short PAN ID
    /// (§3.6.1.13.2, §3.6.1.13.3).
    pub const PAN_ID_UPDATE: u8 = 0x0F;
    /// R22 Table 3-51 — network address update, reported to the next higher
    /// layer when address conflict resolution assigns a new short address
    /// (§3.6.1.9.3).
    pub const NETWORK_ADDRESS_UPDATE: u8 = 0x10;

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

/// Link status entry (one router neighbor's link costs, R22 Figure 3-23).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkStatusEntry {
    pub address: ShortAddress,
    /// This device's own estimate of the link cost *from* the neighbor
    /// (1..=7). R22 §3.4.8.3.2.
    pub incoming_cost: u8,
    /// The neighbor table's outgoing cost for this neighbor — the cost the
    /// neighbor last reported for the reverse direction. `0` means no Link
    /// Status naming this device has been received from it (R22 §3.6.1.5).
    pub outgoing_cost: u8,
}

/// Entry count carried by the 5-bit entry-count sub-field (R22 Figure 3-22).
pub const MAX_LINK_STATUS_ENTRIES: usize = 31;

/// Link status entries this stack places in a single command frame.
///
/// One entry is three octets, so 16 entries occupy 49 payload octets. With the
/// NWK header (8 octets plus the mandatory source IEEE address), the security
/// auxiliary header and the MIC, that stays inside a 127-octet MAC frame with
/// room to spare. A neighbor table with more router neighbors than this is
/// fragmented across several frames (R22 §3.4.8.3.2).
pub const LINK_STATUS_ENTRIES_PER_FRAME: usize = 16;

/// Command options bit 5 — first frame of the sender's link status.
pub const LINK_STATUS_FIRST_FRAME: u8 = 0x20;
/// Command options bit 6 — last frame of the sender's link status.
pub const LINK_STATUS_LAST_FRAME: u8 = 0x40;
/// Command options bits 0-4 — entry count.
pub const LINK_STATUS_COUNT_MASK: u8 = 0x1F;

/// Link Status command (NWK command ID 0x08) built for transmission.
///
/// R22 §3.4.8.3.1: the command options octet carries the entry count in bits
/// 0-4, the first-frame flag in bit 5 and the last-frame flag in bit 6. A
/// sender whose whole link status fits in one frame sets **both** flags.
#[derive(Debug, Clone)]
pub struct LinkStatusCommand {
    /// First frame of this sender's link status list.
    pub first_frame: bool,
    /// Last frame of this sender's link status list.
    pub last_frame: bool,
    pub entries: heapless::Vec<LinkStatusEntry, LINK_STATUS_ENTRIES_PER_FRAME>,
}

impl LinkStatusCommand {
    /// Serialized size of this command's payload (after the command ID byte).
    pub fn wire_size(&self) -> usize {
        1 + 3 * self.entries.len()
    }

    /// Serialize to buffer. Returns bytes written, or `None` if `buf` is too
    /// small for the whole list — a truncated list would advertise link costs
    /// for entries that are not in the frame.
    pub fn serialize(&self, buf: &mut [u8]) -> Option<usize> {
        let size = self.wire_size();
        if buf.len() < size {
            return None;
        }
        let mut options = (self.entries.len() as u8) & LINK_STATUS_COUNT_MASK;
        if self.first_frame {
            options |= LINK_STATUS_FIRST_FRAME;
        }
        if self.last_frame {
            options |= LINK_STATUS_LAST_FRAME;
        }
        buf[0] = options;
        let mut offset = 1;
        for entry in &self.entries {
            buf[offset] = (entry.address.0 & 0xFF) as u8;
            buf[offset + 1] = ((entry.address.0 >> 8) & 0xFF) as u8;
            // R22 Figure 3-23: incoming cost in bits 0-2, bit 3 reserved,
            // outgoing cost in bits 4-6, bit 7 reserved.
            buf[offset + 2] = (entry.incoming_cost & 0x07) | ((entry.outgoing_cost & 0x07) << 4);
            offset += 3;
        }
        Some(size)
    }
}

/// A received Link Status command, borrowed from the authenticated payload.
///
/// Receive processing (R22 §3.6.3.4.2) only needs the covered address range
/// and this device's own entry, so the list is never copied: a sender may
/// legitimately carry up to [`MAX_LINK_STATUS_ENTRIES`] entries, which is more
/// than this stack ever transmits itself.
#[derive(Debug, Clone, Copy)]
pub struct LinkStatusReport<'a> {
    first_frame: bool,
    last_frame: bool,
    entries: &'a [u8],
}

impl<'a> LinkStatusReport<'a> {
    /// Parse a Link Status payload (after the command ID byte).
    ///
    /// Returns `None` when the frame carries fewer entry octets than its own
    /// entry count claims: a short list would otherwise silently change the
    /// covered address range and, with it, whether a missing entry means
    /// "cost unknown" or "this frame says nothing about me".
    pub fn parse(data: &'a [u8]) -> Option<Self> {
        let options = *data.first()?;
        let count = (options & LINK_STATUS_COUNT_MASK) as usize;
        let end = 1 + 3 * count;
        if data.len() < end {
            return None;
        }
        Some(Self {
            first_frame: options & LINK_STATUS_FIRST_FRAME != 0,
            last_frame: options & LINK_STATUS_LAST_FRAME != 0,
            entries: &data[1..end],
        })
    }

    pub const fn first_frame(&self) -> bool {
        self.first_frame
    }

    pub const fn last_frame(&self) -> bool {
        self.last_frame
    }

    pub const fn len(&self) -> usize {
        self.entries.len() / 3
    }

    pub const fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Iterate the entries in the order they appear on the wire (R22 requires
    /// ascending network address order).
    pub fn iter(&self) -> impl Iterator<Item = LinkStatusEntry> + 'a {
        self.entries
            .as_chunks::<3>()
            .0
            .iter()
            .map(|chunk| LinkStatusEntry {
                address: ShortAddress(u16::from_le_bytes([chunk[0], chunk[1]])),
                incoming_cost: chunk[2] & 0x07,
                outgoing_cost: (chunk[2] >> 4) & 0x07,
            })
    }

    /// Whether `address` falls inside the range this frame describes.
    ///
    /// R22 §3.6.3.4.2 derives the range from the first and last addresses in
    /// the list together with the first-frame and last-frame flags: a first
    /// frame covers everything below its first entry and a last frame
    /// everything above its last entry, so a single frame (both flags) covers
    /// the whole address space and an intermediate fragment covers only the
    /// span it lists.
    pub fn covers(&self, address: ShortAddress) -> bool {
        let mut entries = self.iter();
        let Some(first) = entries.next() else {
            // An empty list only says something when it is the sender's whole
            // link status: "no router neighbors at all".
            return self.first_frame && self.last_frame;
        };
        let last = entries.last().unwrap_or(first);
        let lower = if self.first_frame {
            0x0000
        } else {
            first.address.0
        };
        let upper = if self.last_frame {
            0xFFFF
        } else {
            last.address.0
        };
        (lower..=upper).contains(&address.0)
    }

    /// The incoming cost this sender reported for `address`, if listed.
    pub fn incoming_cost_for(&self, address: ShortAddress) -> Option<u8> {
        self.iter()
            .find(|entry| entry.address == address)
            .map(|entry| entry.incoming_cost)
    }
}

// ── Network Report / Network Update (R22 §3.4.9, §3.4.10) ───────

/// Command options bits 0-4 — report/update information record count.
pub const NWK_REPORT_COUNT_MASK: u8 = 0x1F;
/// Command options bits 5-7 — report/update command identifier.
pub const NWK_REPORT_ID_SHIFT: u8 = 5;
/// Report command identifier 0x00 — PAN identifier conflict (R22 Figure 3-26).
pub const NWK_REPORT_PAN_ID_CONFLICT: u8 = 0x00;
/// Update command identifier 0x00 — PAN identifier update (R22 Figure 3-30).
pub const NWK_UPDATE_PAN_ID_UPDATE: u8 = 0x00;
/// PAN identifiers this stack reports in one PAN ID conflict report.
pub const MAX_PAN_ID_CONFLICT_REPORT: usize = 8;

/// Network Report command (NWK command ID 0x09) of type PAN identifier
/// conflict.
///
/// Payload (R22 Figure 3-24): command options, the 64-bit EPID of the
/// reporter's network, then `count` 16-bit PAN identifiers observed in the
/// reporter's neighborhood (R22 Figure 3-27).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PanIdConflictReport {
    pub epid: IeeeAddress,
    pub pan_ids: heapless::Vec<PanId, MAX_PAN_ID_CONFLICT_REPORT>,
}

impl PanIdConflictReport {
    pub fn wire_size(&self) -> usize {
        9 + 2 * self.pan_ids.len()
    }

    pub fn serialize(&self, buf: &mut [u8]) -> Option<usize> {
        let size = self.wire_size();
        if buf.len() < size {
            return None;
        }
        buf[0] = ((self.pan_ids.len() as u8) & NWK_REPORT_COUNT_MASK)
            | (NWK_REPORT_PAN_ID_CONFLICT << NWK_REPORT_ID_SHIFT);
        buf[1..9].copy_from_slice(&self.epid);
        let mut offset = 9;
        for pan_id in &self.pan_ids {
            buf[offset..offset + 2].copy_from_slice(&pan_id.0.to_le_bytes());
            offset += 2;
        }
        Some(size)
    }

    /// Parse a Network Report payload (after the command ID byte).
    ///
    /// Returns `None` for a truncated frame, and for a report command
    /// identifier this stack does not implement — R22 defines only PAN
    /// identifier conflict (0x00); 0x01..=0x07 are reserved.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 9 {
            return None;
        }
        let count = (data[0] & NWK_REPORT_COUNT_MASK) as usize;
        if data[0] >> NWK_REPORT_ID_SHIFT != NWK_REPORT_PAN_ID_CONFLICT {
            return None;
        }
        let end = 9 + 2 * count;
        if data.len() < end {
            return None;
        }
        let mut epid = [0u8; 8];
        epid.copy_from_slice(&data[1..9]);
        let mut pan_ids = heapless::Vec::new();
        for chunk in data[9..end].as_chunks::<2>().0 {
            // A report from a dense neighborhood may name more PAN IDs than
            // this build stores. The excess is dropped rather than rejecting
            // the report: every PAN ID that *is* kept still constrains the
            // replacement PAN ID choice.
            let _ = pan_ids.push(PanId(u16::from_le_bytes([chunk[0], chunk[1]])));
        }
        Some(Self { epid, pan_ids })
    }
}

/// Network Update command (NWK command ID 0x0A) of type PAN identifier update.
///
/// Payload (R22 Figure 3-28): command options, the 64-bit EPID of the network
/// being updated, the sender's `nwkUpdateId`, then the single new 16-bit PAN
/// identifier (R22 Figure 3-31).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PanIdUpdate {
    pub epid: IeeeAddress,
    pub update_id: u8,
    pub new_pan_id: PanId,
}

impl PanIdUpdate {
    pub const WIRE_SIZE: usize = 12;

    pub fn serialize(&self, buf: &mut [u8]) -> Option<usize> {
        if buf.len() < Self::WIRE_SIZE {
            return None;
        }
        // R22 §3.4.10.3.4: a PAN identifier update carries exactly one record.
        buf[0] = 1 | (NWK_UPDATE_PAN_ID_UPDATE << NWK_REPORT_ID_SHIFT);
        buf[1..9].copy_from_slice(&self.epid);
        buf[9] = self.update_id;
        buf[10..12].copy_from_slice(&self.new_pan_id.0.to_le_bytes());
        Some(Self::WIRE_SIZE)
    }

    /// Parse a Network Update payload (after the command ID byte).
    ///
    /// Returns `None` for a truncated frame, an update command identifier
    /// other than PAN identifier update, or a record count other than the
    /// single record R22 §3.4.10.3.4 mandates for that type.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < Self::WIRE_SIZE {
            return None;
        }
        if data[0] >> NWK_REPORT_ID_SHIFT != NWK_UPDATE_PAN_ID_UPDATE
            || data[0] & NWK_REPORT_COUNT_MASK != 1
        {
            return None;
        }
        let mut epid = [0u8; 8];
        epid.copy_from_slice(&data[1..9]);
        Some(Self {
            epid,
            update_id: data[9],
            new_pan_id: PanId(u16::from_le_bytes([data[10], data[11]])),
        })
    }
}

// ── End Device Timeout ──────────────────────────────────────────

/// Highest defined Requested Timeout Enumeration value (R22 Table 3-59).
pub const ED_TIMEOUT_ENUM_MAX: u8 = 14;
/// Timeout enumeration a device falls back to when no negotiation succeeded.
///
/// This is also the value a R22 parent applies to a child that never sent an
/// End Device Timeout Request, so it is the only safe recurring fallback.
pub const ED_TIMEOUT_ENUM_DEFAULT: u8 = 8;
/// Timeout enumeration requested on a fresh join or secured rejoin.
///
/// Enumeration 14 (16384 minutes ≈ 11 days) keeps a battery sensor in the
/// parent's child table across long sleep windows. A parent that refuses it
/// answers `INCORRECT_VALUE`, which walks the request back down towards
/// [`ED_TIMEOUT_ENUM_DEFAULT`].
pub const ED_TIMEOUT_ENUM_REQUESTED: u8 = ED_TIMEOUT_ENUM_MAX;

/// End Device Timeout Response status: the requested timeout was accepted.
pub const ED_TIMEOUT_STATUS_SUCCESS: u8 = 0x00;
/// End Device Timeout Response status: the requested timeout was refused.
pub const ED_TIMEOUT_STATUS_INCORRECT_VALUE: u8 = 0x01;

/// `nwkParentInformation` bit 0 — the parent supports MAC Data Poll keepalive.
pub const PARENT_INFO_MAC_DATA_POLL_KEEPALIVE: u8 = 0x01;
/// `nwkParentInformation` bit 1 — the parent supports End Device Timeout
/// Request keepalive.
pub const PARENT_INFO_ED_TIMEOUT_REQUEST_KEEPALIVE: u8 = 0x02;
/// Defined `nwkParentInformation` bits; every other bit is reserved.
pub const PARENT_INFO_MASK: u8 =
    PARENT_INFO_MAC_DATA_POLL_KEEPALIVE | PARENT_INFO_ED_TIMEOUT_REQUEST_KEEPALIVE;

/// Convert a Requested Timeout Enumeration into seconds.
///
/// R22 Table 3-59 is arithmetic rather than a table lookup: enumeration 0 is
/// 10 seconds and every other enumeration `n` is `2^(n-1) * 2` minutes, i.e.
/// `120 << (n - 1)` seconds. Values above [`ED_TIMEOUT_ENUM_MAX`] are not
/// defined and are rejected instead of being clamped, so a corrupt persisted
/// or received enumeration can never silently become a valid deadline.
pub const fn ed_timeout_enum_to_seconds(timeout_enum: u8) -> Option<u32> {
    match timeout_enum {
        0 => Some(10),
        1..=ED_TIMEOUT_ENUM_MAX => Some(120u32 << (timeout_enum - 1)),
        _ => None,
    }
}

/// End Device Timeout Request (NWK command 0x0B).
///
/// Sent by an end device to its parent after joining/rejoining to request a
/// specific keepalive timeout. The parent uses this to decide how long to
/// keep the child in its neighbour table.
///
/// Payload (R22 §3.4.11):
/// - byte 0: Requested Timeout Enumeration, 0..=14
///   (0=10s, 1=2m, 2=4m, … 8=256m default, … 14=16384m ≈ 11 days)
/// - byte 1: End Device Configuration — **reserved in R22 and always 0**.
///   Earlier drafts carried keepalive-capability bits here; a R22 parent
///   rejects a non-zero value, so this crate never sets one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdTimeoutRequest {
    /// Requested Timeout Enumeration (0..=14), see table above.
    pub requested_timeout: u8,
    /// End Device Configuration — reserved in R22, always 0.
    pub ed_config: u8,
}

impl EdTimeoutRequest {
    /// Create a request for `requested_timeout`, rejecting undefined
    /// enumerations.
    pub const fn new(requested_timeout: u8) -> Option<Self> {
        if requested_timeout > ED_TIMEOUT_ENUM_MAX {
            return None;
        }
        Some(Self {
            requested_timeout,
            // Reserved in R22 — see the type documentation.
            ed_config: 0,
        })
    }

    /// Create with the maximum timeout (enumeration 14 ≈ 11 days).
    pub const fn max_timeout() -> Self {
        Self {
            requested_timeout: ED_TIMEOUT_ENUM_REQUESTED,
            ed_config: 0,
        }
    }

    /// The requested timeout in seconds, or `None` if the enumeration is
    /// undefined.
    pub const fn timeout_seconds(&self) -> Option<u32> {
        ed_timeout_enum_to_seconds(self.requested_timeout)
    }

    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.requested_timeout;
        buf[1] = self.ed_config;
        2
    }

    /// Parse a request payload, rejecting undefined enumerations and any
    /// non-zero reserved End Device Configuration byte. Trailing bytes from a
    /// future revision are ignored.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 2 || data[0] > ED_TIMEOUT_ENUM_MAX || data[1] != 0 {
            return None;
        }
        Some(Self {
            requested_timeout: data[0],
            ed_config: data[1],
        })
    }
}

/// End Device Timeout Response (NWK command 0x0C).
///
/// Payload (R22 §3.4.12):
/// - byte 0: status (0x00 success, 0x01 incorrect value)
/// - byte 1: `nwkParentInformation` — bit0 MAC Data Poll keepalive,
///   bit1 End Device Timeout Request keepalive, other bits reserved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdTimeoutResponse {
    /// 0x00 = success, 0x01 = incorrect value.
    pub status: u8,
    /// Parent information: bit0 = MAC Data Poll Keepalive supported,
    /// bit1 = End Device Timeout Request keepalive supported.
    pub parent_info: u8,
}

impl EdTimeoutResponse {
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.status;
        buf[1] = self.parent_info;
        2
    }

    /// Parse a response payload. Exactly two bytes are meaningful; trailing
    /// bytes added by a future revision are ignored rather than rejected.
    pub fn parse(data: &[u8]) -> Option<Self> {
        if data.len() < 2 {
            return None;
        }
        Some(Self {
            status: data[0],
            parent_info: data[1],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::NwkHeader;
    use super::{
        ED_TIMEOUT_ENUM_MAX, EdTimeoutRequest, EdTimeoutResponse, LINK_STATUS_FIRST_FRAME,
        LINK_STATUS_LAST_FRAME, LinkStatusCommand, LinkStatusEntry, LinkStatusReport,
        NWK_REPORT_ID_SHIFT, PanIdConflictReport, PanIdUpdate, ed_timeout_enum_to_seconds,
    };
    use zigbee_types::{PanId, ShortAddress};

    #[test]
    fn rejects_malformed_source_route_subframes() {
        let malformed: &[&[u8]] = &[
            // Missing relay count and index.
            &[0x08, 0x04, 1, 0, 2, 0, 5, 1],
            // Missing relay index.
            &[0x08, 0x04, 1, 0, 2, 0, 5, 1, 1],
            // Empty relay list.
            &[0x08, 0x04, 1, 0, 2, 0, 5, 1, 0, 0],
            // Relay index outside the list.
            &[0x08, 0x04, 1, 0, 2, 0, 5, 1, 1, 1, 3, 0],
            // Truncated relay list.
            &[0x08, 0x04, 1, 0, 2, 0, 5, 1, 2, 1, 3, 0],
        ];

        for frame in malformed {
            assert!(NwkHeader::parse(frame).is_none(), "{frame:02X?}");
        }
    }

    #[test]
    fn ed_timeout_request_reserves_the_configuration_byte() {
        let mut buf = [0xAAu8; 2];
        assert_eq!(EdTimeoutRequest::max_timeout().serialize(&mut buf), 2);
        assert_eq!(buf, [ED_TIMEOUT_ENUM_MAX, 0x00]);

        let mut buf = [0xAAu8; 2];
        assert_eq!(EdTimeoutRequest::new(8).unwrap().serialize(&mut buf), 2);
        assert_eq!(buf, [8, 0x00]);
    }

    #[test]
    fn ed_timeout_request_rejects_undefined_enumerations() {
        assert_eq!(EdTimeoutRequest::new(ED_TIMEOUT_ENUM_MAX + 1), None);
        assert_eq!(EdTimeoutRequest::new(0xFF), None);
        assert!(EdTimeoutRequest::new(0).is_some());
        assert!(EdTimeoutRequest::new(ED_TIMEOUT_ENUM_MAX).is_some());
    }

    #[test]
    fn ed_timeout_request_parse_enforces_the_wire_contract() {
        assert_eq!(EdTimeoutRequest::parse(&[14]), None);
        // Reserved End Device Configuration byte must be zero in R22.
        assert_eq!(EdTimeoutRequest::parse(&[14, 0x02]), None);
        assert_eq!(EdTimeoutRequest::parse(&[15, 0x00]), None);
        assert_eq!(
            EdTimeoutRequest::parse(&[14, 0x00, 0x99]),
            Some(EdTimeoutRequest::max_timeout()),
        );
    }

    #[test]
    fn ed_timeout_enumeration_seconds_are_arithmetic() {
        assert_eq!(ed_timeout_enum_to_seconds(0), Some(10));
        assert_eq!(ed_timeout_enum_to_seconds(1), Some(120));
        assert_eq!(ed_timeout_enum_to_seconds(2), Some(240));
        assert_eq!(ed_timeout_enum_to_seconds(8), Some(256 * 60));
        assert_eq!(ed_timeout_enum_to_seconds(14), Some(16384 * 60));
        assert_eq!(ed_timeout_enum_to_seconds(15), None);
        assert_eq!(ed_timeout_enum_to_seconds(0xFF), None);
    }

    #[test]
    fn ed_timeout_response_round_trips_and_ignores_trailing_bytes() {
        let response = EdTimeoutResponse {
            status: 0x00,
            parent_info: 0x03,
        };
        let mut buf = [0xAAu8; 2];
        assert_eq!(response.serialize(&mut buf), 2);
        assert_eq!(buf, [0x00, 0x03]);
        assert_eq!(EdTimeoutResponse::parse(&buf), Some(response));
        assert_eq!(
            EdTimeoutResponse::parse(&[0x00, 0x03, 0x77]),
            Some(response)
        );
        assert_eq!(EdTimeoutResponse::parse(&[0x00]), None);
    }

    // ── R22 §3.4.8 Link Status ──────────────────────────────────

    #[test]
    fn link_status_options_and_cost_octet_match_r22_bit_layout() {
        // R22 Figure 3-22: entry count bits 0-4, first frame bit 5, last frame
        // bit 6. R22 Figure 3-23: incoming cost bits 0-2, bit 3 reserved,
        // outgoing cost bits 4-6, bit 7 reserved.
        let mut command = LinkStatusCommand {
            first_frame: true,
            last_frame: true,
            entries: heapless::Vec::new(),
        };
        command
            .entries
            .push(LinkStatusEntry {
                address: ShortAddress(0x1234),
                incoming_cost: 3,
                outgoing_cost: 5,
            })
            .unwrap();

        let mut buf = [0xAAu8; 8];
        assert_eq!(command.serialize(&mut buf), Some(4));
        assert_eq!(buf[..4], [0x61, 0x34, 0x12, 0x53]);

        let report = LinkStatusReport::parse(&buf[..4]).expect("the frame parses");
        assert!(report.first_frame() && report.last_frame());
        assert_eq!(report.len(), 1);
        let entry = report.iter().next().unwrap();
        assert_eq!(entry.address, ShortAddress(0x1234));
        assert_eq!(entry.incoming_cost, 3);
        assert_eq!(entry.outgoing_cost, 5);
    }

    #[test]
    fn link_status_reserved_bits_are_not_decoded_as_cost() {
        // Bits 3 and 7 are reserved: a sender that sets them must not change
        // the costs a receiver reads.
        let report =
            LinkStatusReport::parse(&[0x61, 0x34, 0x12, 0x53 | 0x88]).expect("the frame parses");
        let entry = report.iter().next().unwrap();
        assert_eq!(entry.incoming_cost, 3);
        assert_eq!(entry.outgoing_cost, 5);
    }

    #[test]
    fn link_status_rejects_a_list_shorter_than_its_entry_count() {
        // Two entries claimed, one supplied: the covered range and the
        // "listed or not" answer would both be wrong.
        assert!(LinkStatusReport::parse(&[0x62, 0x34, 0x12, 0x11]).is_none());
        assert!(LinkStatusReport::parse(&[]).is_none());
    }

    #[test]
    fn link_status_covered_range_follows_the_first_and_last_flags() {
        let entries = [
            0x02u8 | LINK_STATUS_FIRST_FRAME,
            0x00,
            0x10,
            0x11,
            0x00,
            0x20,
            0x11,
        ];
        let first = LinkStatusReport::parse(&entries).expect("the frame parses");
        // A first frame covers everything below its first entry ...
        assert!(first.covers(ShortAddress(0x0001)));
        assert!(first.covers(ShortAddress(0x2000)));
        // ... but nothing above its last entry, because a later frame does.
        assert!(!first.covers(ShortAddress(0x2001)));

        let last = [
            0x02u8 | LINK_STATUS_LAST_FRAME,
            0x00,
            0x20,
            0x11,
            0x00,
            0x30,
            0x11,
        ];
        let last = LinkStatusReport::parse(&last).expect("the frame parses");
        assert!(!last.covers(ShortAddress(0x1FFF)));
        assert!(last.covers(ShortAddress(0x3000)));
        assert!(last.covers(ShortAddress(0xF000)));

        let middle = [0x02u8, 0x00, 0x20, 0x11, 0x00, 0x30, 0x11];
        let middle = LinkStatusReport::parse(&middle).expect("the frame parses");
        assert!(!middle.covers(ShortAddress(0x1000)));
        assert!(middle.covers(ShortAddress(0x2500)));
        assert!(!middle.covers(ShortAddress(0x4000)));
    }

    #[test]
    fn an_empty_single_frame_link_status_covers_every_address() {
        // "I have no router neighbours at all" is a statement about the whole
        // address space; an empty *fragment* says nothing.
        let whole = LinkStatusReport::parse(&[LINK_STATUS_FIRST_FRAME | LINK_STATUS_LAST_FRAME])
            .expect("the frame parses");
        assert!(whole.covers(ShortAddress(0x1234)));
        assert_eq!(whole.incoming_cost_for(ShortAddress(0x1234)), None);

        let fragment = LinkStatusReport::parse(&[0x00]).expect("the frame parses");
        assert!(!fragment.covers(ShortAddress(0x1234)));
    }

    // ── R22 §3.4.9 / §3.4.10 Network Report and Network Update ──

    #[test]
    fn pan_id_conflict_report_round_trips_with_the_r22_field_order() {
        let mut report = PanIdConflictReport {
            epid: [0x11; 8],
            pan_ids: heapless::Vec::new(),
        };
        report.pan_ids.push(PanId(0x1234)).unwrap();
        report.pan_ids.push(PanId(0xABCD)).unwrap();

        let mut buf = [0xAAu8; 16];
        assert_eq!(report.serialize(&mut buf), Some(13));
        // Command options: count 2 in bits 0-4, report identifier 0 (PAN ID
        // conflict) in bits 5-7.
        assert_eq!(buf[0], 0x02);
        assert_eq!(buf[1..9], [0x11; 8]);
        assert_eq!(buf[9..13], [0x34, 0x12, 0xCD, 0xAB]);
        assert_eq!(PanIdConflictReport::parse(&buf[..13]), Some(report));
    }

    #[test]
    fn network_report_rejects_reserved_types_and_truncation() {
        let mut buf = [0u8; 13];
        buf[0] = 0x02 | (1 << NWK_REPORT_ID_SHIFT); // reserved report type 1
        assert!(PanIdConflictReport::parse(&buf).is_none());

        let mut buf = [0u8; 12];
        buf[0] = 0x02; // two PAN IDs claimed, only one supplied
        assert!(PanIdConflictReport::parse(&buf).is_none());
        assert!(PanIdConflictReport::parse(&[0x00; 8]).is_none());
    }

    #[test]
    fn pan_id_update_round_trips_with_the_r22_field_order() {
        let update = PanIdUpdate {
            epid: [0x22; 8],
            update_id: 7,
            new_pan_id: PanId(0xBEEF),
        };
        let mut buf = [0xAAu8; PanIdUpdate::WIRE_SIZE];
        assert_eq!(update.serialize(&mut buf), Some(12));
        // One record, update identifier 0 (PAN identifier update).
        assert_eq!(buf[0], 0x01);
        assert_eq!(buf[1..9], [0x22; 8]);
        assert_eq!(buf[9], 7);
        assert_eq!(buf[10..12], [0xEF, 0xBE]);
        assert_eq!(PanIdUpdate::parse(&buf), Some(update));
    }

    #[test]
    fn network_update_rejects_reserved_types_and_wrong_record_counts() {
        let mut buf = [0u8; PanIdUpdate::WIRE_SIZE];
        buf[0] = 0x01 | (1 << NWK_REPORT_ID_SHIFT); // reserved update type 1
        assert!(PanIdUpdate::parse(&buf).is_none());

        let mut buf = [0u8; PanIdUpdate::WIRE_SIZE];
        buf[0] = 0x02; // R22 §3.4.10.3.4 allows exactly one record
        assert!(PanIdUpdate::parse(&buf).is_none());

        assert!(PanIdUpdate::parse(&[0x01; 11]).is_none());
    }
}
