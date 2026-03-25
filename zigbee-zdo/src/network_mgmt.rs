//! Network management services (ZDP clusters 0x0031–0x0038).
//!
//! Includes neighbor-table (LQI), routing-table, binding-table queries,
//! leave, permit-joining, and NWK update requests/responses.

use heapless::Vec;
use zigbee_types::{IeeeAddress, ShortAddress};

use crate::ZdoError;

// ── Mgmt_Lqi (0x0031 / 0x8031) ─────────────────────────────────

/// Mgmt_Lqi_req: `start_index(1)`
#[derive(Debug, Clone, Copy)]
pub struct MgmtLqiReq {
    pub start_index: u8,
}

impl MgmtLqiReq {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.start_index;
        Ok(1)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.is_empty() {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            start_index: data[0],
        })
    }
}

/// Single neighbor-table record inside [`MgmtLqiRsp`] (22 bytes).
///
/// ```text
/// ext_pan_id(8) | ext_addr(8) | nwk_addr(2) |
/// device_type(2b) | rx_on_idle(2b) | relationship(3b) | reserved(1b) |
/// permit_joining(2b) | reserved(6b) |
/// depth(1) | lqi(1)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NeighborTableRecord {
    pub extended_pan_id: [u8; 8],
    pub extended_addr: IeeeAddress,
    pub network_addr: ShortAddress,
    /// 2-bit device type (0=Coord, 1=Router, 2=EndDevice, 3=Unknown).
    pub device_type: u8,
    /// 2-bit rx-on-when-idle (0=off, 1=on, 2=unknown).
    pub rx_on_when_idle: u8,
    /// 3-bit relationship (0=parent, 1=child, 2=sibling, 3=none, 4=prev_child).
    pub relationship: u8,
    /// 2-bit permit-joining status (0=not accepting, 1=accepting, 2=unknown).
    pub permit_joining: u8,
    pub depth: u8,
    pub lqi: u8,
}

impl NeighborTableRecord {
    pub const WIRE_SIZE: usize = 22;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::WIRE_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..8].copy_from_slice(&self.extended_pan_id);
        buf[8..16].copy_from_slice(&self.extended_addr);
        buf[16..18].copy_from_slice(&self.network_addr.0.to_le_bytes());
        buf[18] = (self.device_type & 0x03)
            | ((self.rx_on_when_idle & 0x03) << 2)
            | ((self.relationship & 0x07) << 4);
        buf[19] = self.permit_joining & 0x03;
        buf[20] = self.depth;
        buf[21] = self.lqi;
        Ok(Self::WIRE_SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::WIRE_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(&data[0..8]);
        let mut extended_addr = [0u8; 8];
        extended_addr.copy_from_slice(&data[8..16]);
        Ok(Self {
            extended_pan_id,
            extended_addr,
            network_addr: ShortAddress(u16::from_le_bytes([data[16], data[17]])),
            device_type: data[18] & 0x03,
            rx_on_when_idle: (data[18] >> 2) & 0x03,
            relationship: (data[18] >> 4) & 0x07,
            permit_joining: data[19] & 0x03,
            depth: data[20],
            lqi: data[21],
        })
    }
}

/// Mgmt_Lqi_rsp.
///
/// ```text
/// status(1) | neighbor_table_entries(1) | start_index(1) |
/// list_count(1) | neighbor_table_list(22·N)
/// ```
#[derive(Debug, Clone)]
pub struct MgmtLqiRsp {
    pub status: crate::ZdpStatus,
    pub neighbor_table_entries: u8,
    pub start_index: u8,
    pub neighbor_table_list: Vec<NeighborTableRecord, 16>,
}

impl MgmtLqiRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = 4 + self.neighbor_table_list.len() * NeighborTableRecord::WIRE_SIZE;
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1] = self.neighbor_table_entries;
        buf[2] = self.start_index;
        buf[3] = self.neighbor_table_list.len() as u8;
        let mut off = 4;
        for rec in self.neighbor_table_list.iter() {
            let n = rec.serialize(&mut buf[off..])?;
            off += n;
        }
        Ok(off)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 4 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let neighbor_table_entries = data[1];
        let start_index = data[2];
        let list_count = data[3] as usize;
        let mut off = 4;
        let mut neighbor_table_list = Vec::new();
        for _ in 0..list_count {
            if off + NeighborTableRecord::WIRE_SIZE > data.len() {
                break;
            }
            let rec = NeighborTableRecord::parse(&data[off..])?;
            let _ = neighbor_table_list.push(rec);
            off += NeighborTableRecord::WIRE_SIZE;
        }
        Ok(Self {
            status,
            neighbor_table_entries,
            start_index,
            neighbor_table_list,
        })
    }
}

// ── Mgmt_Rtg (0x0032 / 0x8032) ─────────────────────────────────

/// Mgmt_Rtg_req: `start_index(1)`
pub type MgmtRtgReq = MgmtLqiReq;

/// Single routing-table record (5 bytes).
///
/// ```text
/// dst_addr(2) | status(3b)|mem_constrained(1b)|many2one(1b)|rr_req(1b)|reserved(2b) |
/// next_hop(2)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoutingTableRecord {
    pub dst_addr: ShortAddress,
    /// 3-bit route status.
    pub status: u8,
    pub memory_constrained: bool,
    pub many_to_one: bool,
    pub route_record_required: bool,
    pub next_hop: ShortAddress,
}

impl RoutingTableRecord {
    pub const WIRE_SIZE: usize = 5;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::WIRE_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..2].copy_from_slice(&self.dst_addr.0.to_le_bytes());
        buf[2] = (self.status & 0x07)
            | if self.memory_constrained { 0x08 } else { 0 }
            | if self.many_to_one { 0x10 } else { 0 }
            | if self.route_record_required { 0x20 } else { 0 };
        buf[3..5].copy_from_slice(&self.next_hop.0.to_le_bytes());
        Ok(Self::WIRE_SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::WIRE_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            dst_addr: ShortAddress(u16::from_le_bytes([data[0], data[1]])),
            status: data[2] & 0x07,
            memory_constrained: data[2] & 0x08 != 0,
            many_to_one: data[2] & 0x10 != 0,
            route_record_required: data[2] & 0x20 != 0,
            next_hop: ShortAddress(u16::from_le_bytes([data[3], data[4]])),
        })
    }
}

/// Mgmt_Rtg_rsp.
#[derive(Debug, Clone)]
pub struct MgmtRtgRsp {
    pub status: crate::ZdpStatus,
    pub routing_table_entries: u8,
    pub start_index: u8,
    pub routing_table_list: Vec<RoutingTableRecord, 16>,
}

impl MgmtRtgRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = 4 + self.routing_table_list.len() * RoutingTableRecord::WIRE_SIZE;
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1] = self.routing_table_entries;
        buf[2] = self.start_index;
        buf[3] = self.routing_table_list.len() as u8;
        let mut off = 4;
        for rec in self.routing_table_list.iter() {
            let n = rec.serialize(&mut buf[off..])?;
            off += n;
        }
        Ok(off)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 4 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let routing_table_entries = data[1];
        let start_index = data[2];
        let list_count = data[3] as usize;
        let mut off = 4;
        let mut routing_table_list = Vec::new();
        for _ in 0..list_count {
            if off + RoutingTableRecord::WIRE_SIZE > data.len() {
                break;
            }
            let rec = RoutingTableRecord::parse(&data[off..])?;
            let _ = routing_table_list.push(rec);
            off += RoutingTableRecord::WIRE_SIZE;
        }
        Ok(Self {
            status,
            routing_table_entries,
            start_index,
            routing_table_list,
        })
    }
}

// ── Mgmt_Bind (0x0033 / 0x8033) ────────────────────────────────

/// Mgmt_Bind_req: `start_index(1)`
pub type MgmtBindReq = MgmtLqiReq;

/// Single binding-table record used in [`MgmtBindRsp`].
///
/// Same layout as a Bind_req payload (see [`crate::binding_mgmt::BindReq`]).
#[derive(Debug, Clone)]
pub struct BindingTableRecord {
    pub src_addr: IeeeAddress,
    pub src_endpoint: u8,
    pub cluster_id: u16,
    pub dst_addr_mode: u8,
    pub dst: crate::binding_mgmt::BindTarget,
}

impl BindingTableRecord {
    pub fn wire_size(&self) -> usize {
        match self.dst {
            crate::binding_mgmt::BindTarget::Group(_) => 14,
            crate::binding_mgmt::BindTarget::Unicast { .. } => 21,
        }
    }

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let needed = self.wire_size();
        if buf.len() < needed {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..8].copy_from_slice(&self.src_addr);
        buf[8] = self.src_endpoint;
        buf[9..11].copy_from_slice(&self.cluster_id.to_le_bytes());
        match self.dst {
            crate::binding_mgmt::BindTarget::Group(group) => {
                buf[11] = 0x01;
                buf[12..14].copy_from_slice(&group.to_le_bytes());
                Ok(14)
            }
            crate::binding_mgmt::BindTarget::Unicast {
                dst_addr,
                dst_endpoint,
            } => {
                buf[11] = 0x03;
                buf[12..20].copy_from_slice(&dst_addr);
                buf[20] = dst_endpoint;
                Ok(21)
            }
        }
    }

    pub fn parse(data: &[u8]) -> Result<(Self, usize), ZdoError> {
        if data.len() < 12 {
            return Err(ZdoError::InvalidLength);
        }
        let mut src_addr = [0u8; 8];
        src_addr.copy_from_slice(&data[0..8]);
        let src_endpoint = data[8];
        let cluster_id = u16::from_le_bytes([data[9], data[10]]);
        let dst_addr_mode = data[11];
        let (dst, consumed) = match dst_addr_mode {
            0x01 => {
                if data.len() < 14 {
                    return Err(ZdoError::InvalidLength);
                }
                let g = u16::from_le_bytes([data[12], data[13]]);
                (crate::binding_mgmt::BindTarget::Group(g), 14)
            }
            0x03 => {
                if data.len() < 21 {
                    return Err(ZdoError::InvalidLength);
                }
                let mut dst_addr = [0u8; 8];
                dst_addr.copy_from_slice(&data[12..20]);
                (
                    crate::binding_mgmt::BindTarget::Unicast {
                        dst_addr,
                        dst_endpoint: data[20],
                    },
                    21,
                )
            }
            _ => return Err(ZdoError::InvalidData),
        };
        Ok((
            Self {
                src_addr,
                src_endpoint,
                cluster_id,
                dst_addr_mode,
                dst,
            },
            consumed,
        ))
    }
}

/// Mgmt_Bind_rsp.
#[derive(Debug, Clone)]
pub struct MgmtBindRsp {
    pub status: crate::ZdpStatus,
    pub binding_table_entries: u8,
    pub start_index: u8,
    pub binding_table_list: Vec<BindingTableRecord, 16>,
}

impl MgmtBindRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < 4 {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1] = self.binding_table_entries;
        buf[2] = self.start_index;
        buf[3] = self.binding_table_list.len() as u8;
        let mut off = 4;
        for rec in self.binding_table_list.iter() {
            let n = rec.serialize(&mut buf[off..])?;
            off += n;
        }
        Ok(off)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 4 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let binding_table_entries = data[1];
        let start_index = data[2];
        let list_count = data[3] as usize;
        let mut off = 4;
        let mut binding_table_list = Vec::new();
        for _ in 0..list_count {
            if off >= data.len() {
                break;
            }
            let (rec, consumed) = BindingTableRecord::parse(&data[off..])?;
            let _ = binding_table_list.push(rec);
            off += consumed;
        }
        Ok(Self {
            status,
            binding_table_entries,
            start_index,
            binding_table_list,
        })
    }
}

// ── Mgmt_Leave (0x0034 / 0x8034) ───────────────────────────────

/// Mgmt_Leave_req: `device_address(8) | options(1)`
///
/// Options byte: bit 6 = Remove Children, bit 7 = Rejoin.
#[derive(Debug, Clone, Copy)]
pub struct MgmtLeaveReq {
    pub device_address: IeeeAddress,
    pub remove_children: bool,
    pub rejoin: bool,
}

impl MgmtLeaveReq {
    pub const SIZE: usize = 9;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..8].copy_from_slice(&self.device_address);
        buf[8] = if self.remove_children { 0x40 } else { 0 } | if self.rejoin { 0x80 } else { 0 };
        Ok(Self::SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let mut device_address = [0u8; 8];
        device_address.copy_from_slice(&data[0..8]);
        Ok(Self {
            device_address,
            remove_children: data[8] & 0x40 != 0,
            rejoin: data[8] & 0x80 != 0,
        })
    }
}

/// Mgmt_Leave_rsp: `status(1)`
#[derive(Debug, Clone, Copy)]
pub struct MgmtLeaveRsp {
    pub status: crate::ZdpStatus,
}

impl MgmtLeaveRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.is_empty() {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        Ok(1)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.is_empty() {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            status: crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?,
        })
    }
}

// ── Mgmt_Permit_Joining (0x0036 / 0x8036) ──────────────────────

/// Mgmt_Permit_Joining_req: `permit_duration(1) | tc_significance(1)`
#[derive(Debug, Clone, Copy)]
pub struct MgmtPermitJoiningReq {
    /// Duration in seconds (0x00 = disable, 0xFF = indefinite).
    pub permit_duration: u8,
    /// 1 = also update Trust Center; 0 = local only.
    pub tc_significance: u8,
}

impl MgmtPermitJoiningReq {
    pub const SIZE: usize = 2;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.permit_duration;
        buf[1] = self.tc_significance;
        Ok(Self::SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::SIZE {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            permit_duration: data[0],
            tc_significance: data[1],
        })
    }
}

/// Mgmt_Permit_Joining_rsp: `status(1)`
pub type MgmtPermitJoiningRsp = MgmtLeaveRsp;

// ── Mgmt_NWK_Update (0x0038 / 0x8038) ──────────────────────────

/// Decoded Mgmt_NWK_Update_req payload.
///
/// The format depends on the `scan_duration` field:
/// * 0x00..=0x05 — energy detection scan, `scan_count` follows.
/// * 0xFE — channel change, `nwk_update_id` follows.
/// * 0xFF — NWK manager address change, `nwk_update_id` + `nwk_manager_addr` follow.
#[derive(Debug, Clone)]
pub enum MgmtNwkUpdateReq {
    /// Energy-detection scan request.
    EdScan {
        scan_channels: u32,
        scan_duration: u8,
        scan_count: u8,
    },
    /// Channel change request.
    ChannelChange {
        scan_channels: u32,
        nwk_update_id: u8,
    },
    /// NWK manager address change.
    ManagerChange {
        scan_channels: u32,
        nwk_update_id: u8,
        nwk_manager_addr: ShortAddress,
    },
}

impl MgmtNwkUpdateReq {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        match self {
            Self::EdScan {
                scan_channels,
                scan_duration,
                scan_count,
            } => {
                if buf.len() < 6 {
                    return Err(ZdoError::BufferTooSmall);
                }
                buf[0..4].copy_from_slice(&scan_channels.to_le_bytes());
                buf[4] = *scan_duration;
                buf[5] = *scan_count;
                Ok(6)
            }
            Self::ChannelChange {
                scan_channels,
                nwk_update_id,
            } => {
                if buf.len() < 6 {
                    return Err(ZdoError::BufferTooSmall);
                }
                buf[0..4].copy_from_slice(&scan_channels.to_le_bytes());
                buf[4] = 0xFE;
                buf[5] = *nwk_update_id;
                Ok(6)
            }
            Self::ManagerChange {
                scan_channels,
                nwk_update_id,
                nwk_manager_addr,
            } => {
                if buf.len() < 8 {
                    return Err(ZdoError::BufferTooSmall);
                }
                buf[0..4].copy_from_slice(&scan_channels.to_le_bytes());
                buf[4] = 0xFF;
                buf[5] = *nwk_update_id;
                buf[6..8].copy_from_slice(&nwk_manager_addr.0.to_le_bytes());
                Ok(8)
            }
        }
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 5 {
            return Err(ZdoError::InvalidLength);
        }
        let scan_channels = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let scan_duration = data[4];
        match scan_duration {
            0x00..=0x05 => {
                if data.len() < 6 {
                    return Err(ZdoError::InvalidLength);
                }
                Ok(Self::EdScan {
                    scan_channels,
                    scan_duration,
                    scan_count: data[5],
                })
            }
            0xFE => {
                if data.len() < 6 {
                    return Err(ZdoError::InvalidLength);
                }
                Ok(Self::ChannelChange {
                    scan_channels,
                    nwk_update_id: data[5],
                })
            }
            0xFF => {
                if data.len() < 8 {
                    return Err(ZdoError::InvalidLength);
                }
                Ok(Self::ManagerChange {
                    scan_channels,
                    nwk_update_id: data[5],
                    nwk_manager_addr: ShortAddress(u16::from_le_bytes([data[6], data[7]])),
                })
            }
            _ => Err(ZdoError::InvalidData),
        }
    }
}

/// Mgmt_NWK_Update_notify (response).
///
/// ```text
/// status(1) | scanned_channels(4) | total_tx(2) | tx_failures(2) |
/// list_count(1) | energy_values(N)
/// ```
#[derive(Debug, Clone)]
pub struct MgmtNwkUpdateRsp {
    pub status: crate::ZdpStatus,
    pub scanned_channels: u32,
    pub total_transmissions: u16,
    pub transmission_failures: u16,
    pub energy_values: Vec<u8, 16>,
}

impl MgmtNwkUpdateRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = 10 + self.energy_values.len();
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..5].copy_from_slice(&self.scanned_channels.to_le_bytes());
        buf[5..7].copy_from_slice(&self.total_transmissions.to_le_bytes());
        buf[7..9].copy_from_slice(&self.transmission_failures.to_le_bytes());
        buf[9] = self.energy_values.len() as u8;
        buf[10..total].copy_from_slice(&self.energy_values);
        Ok(total)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 10 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let scanned_channels = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
        let total_transmissions = u16::from_le_bytes([data[5], data[6]]);
        let transmission_failures = u16::from_le_bytes([data[7], data[8]]);
        let count = data[9] as usize;
        let mut energy_values = Vec::new();
        for i in 0..count {
            if 10 + i >= data.len() {
                break;
            }
            let _ = energy_values.push(data[10 + i]);
        }
        Ok(Self {
            status,
            scanned_channels,
            total_transmissions,
            transmission_failures,
            energy_values,
        })
    }
}
