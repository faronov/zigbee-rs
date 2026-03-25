//! Device and service discovery (ZDP clusters 0x0000 – 0x0099).
//!
//! Each ZDP command has a request struct and a response struct with
//! `serialize` / `parse` methods.  The transaction-sequence number (TSN)
//! is **not** included in these structs — it is prepended/stripped by the
//! ZDP dispatcher in [`crate::handler`].

use heapless::Vec;
use zigbee_types::{IeeeAddress, ShortAddress};

use crate::ZdoError;
use crate::descriptors::{NodeDescriptor, PowerDescriptor, SimpleDescriptor};

// ── NWK_addr (0x0000 / 0x8000) ─────────────────────────────────

/// Request type field for address discovery.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RequestType {
    /// Single device response.
    Single = 0x00,
    /// Extended response (include associated device list).
    Extended = 0x01,
}

/// NWK_addr_req — resolve an IEEE address to a NWK address.
///
/// ```text
/// IEEE_addr(8) | request_type(1) | start_index(1)
/// ```
#[derive(Debug, Clone)]
pub struct NwkAddrReq {
    pub ieee_addr: IeeeAddress,
    pub request_type: RequestType,
    pub start_index: u8,
}

impl NwkAddrReq {
    pub const MIN_SIZE: usize = 10;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::MIN_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..8].copy_from_slice(&self.ieee_addr);
        buf[8] = self.request_type as u8;
        buf[9] = self.start_index;
        Ok(Self::MIN_SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::MIN_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let mut ieee_addr = [0u8; 8];
        ieee_addr.copy_from_slice(&data[0..8]);
        let request_type = match data[8] {
            0 => RequestType::Single,
            1 => RequestType::Extended,
            _ => return Err(ZdoError::InvalidData),
        };
        Ok(Self {
            ieee_addr,
            request_type,
            start_index: data[9],
        })
    }
}

/// NWK_addr_rsp — response to [`NwkAddrReq`].
///
/// ```text
/// status(1) | IEEE_addr(8) | NWK_addr(2) | [num_assoc(1) | start_idx(1) | assoc_list(2·N)]
/// ```
#[derive(Debug, Clone)]
pub struct NwkAddrRsp {
    pub status: crate::ZdpStatus,
    pub ieee_addr: IeeeAddress,
    pub nwk_addr: ShortAddress,
    pub num_assoc_dev: u8,
    pub start_index: u8,
    pub assoc_dev_list: Vec<ShortAddress, 32>,
}

impl NwkAddrRsp {
    pub const MIN_SIZE: usize = 11;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = Self::MIN_SIZE
            + if self.num_assoc_dev > 0 {
                2 + self.assoc_dev_list.len() * 2
            } else {
                0
            };
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..9].copy_from_slice(&self.ieee_addr);
        buf[9..11].copy_from_slice(&self.nwk_addr.0.to_le_bytes());
        let mut off = 11;
        if self.num_assoc_dev > 0 {
            buf[off] = self.num_assoc_dev;
            off += 1;
            buf[off] = self.start_index;
            off += 1;
            for a in self.assoc_dev_list.iter() {
                buf[off..off + 2].copy_from_slice(&a.0.to_le_bytes());
                off += 2;
            }
        }
        Ok(off)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::MIN_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let mut ieee_addr = [0u8; 8];
        ieee_addr.copy_from_slice(&data[1..9]);
        let nwk_addr = ShortAddress(u16::from_le_bytes([data[9], data[10]]));
        let mut num_assoc_dev = 0u8;
        let mut start_index = 0u8;
        let mut assoc_dev_list = Vec::new();
        if data.len() > Self::MIN_SIZE {
            num_assoc_dev = data[11];
            start_index = data[12];
            let mut off = 13;
            for _ in 0..num_assoc_dev {
                if off + 2 > data.len() {
                    break;
                }
                let a = ShortAddress(u16::from_le_bytes([data[off], data[off + 1]]));
                let _ = assoc_dev_list.push(a);
                off += 2;
            }
        }
        Ok(Self {
            status,
            ieee_addr,
            nwk_addr,
            num_assoc_dev,
            start_index,
            assoc_dev_list,
        })
    }
}

// ── IEEE_addr (0x0001 / 0x8001) ─────────────────────────────────

/// IEEE_addr_req — resolve a NWK address to an IEEE address.
///
/// ```text
/// NWK_addr_of_interest(2) | request_type(1) | start_index(1)
/// ```
#[derive(Debug, Clone)]
pub struct IeeeAddrReq {
    pub nwk_addr_of_interest: ShortAddress,
    pub request_type: RequestType,
    pub start_index: u8,
}

impl IeeeAddrReq {
    pub const MIN_SIZE: usize = 4;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::MIN_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..2].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        buf[2] = self.request_type as u8;
        buf[3] = self.start_index;
        Ok(Self::MIN_SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::MIN_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[0], data[1]]));
        let request_type = match data[2] {
            0 => RequestType::Single,
            1 => RequestType::Extended,
            _ => return Err(ZdoError::InvalidData),
        };
        Ok(Self {
            nwk_addr_of_interest,
            request_type,
            start_index: data[3],
        })
    }
}

/// IEEE_addr_rsp — same structure as [`NwkAddrRsp`].
pub type IeeeAddrRsp = NwkAddrRsp;

// ── Node_Desc (0x0002 / 0x8002) ────────────────────────────────

/// Node_Desc_req: `NWK_addr_of_interest(2)`
#[derive(Debug, Clone)]
pub struct NodeDescReq {
    pub nwk_addr_of_interest: ShortAddress,
}

impl NodeDescReq {
    pub const SIZE: usize = 2;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..2].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        Ok(Self::SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::SIZE {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            nwk_addr_of_interest: ShortAddress(u16::from_le_bytes([data[0], data[1]])),
        })
    }
}

/// Node_Desc_rsp: `status(1) | NWK_addr(2) | node_descriptor(13)`
#[derive(Debug, Clone)]
pub struct NodeDescRsp {
    pub status: crate::ZdpStatus,
    pub nwk_addr_of_interest: ShortAddress,
    pub node_descriptor: Option<NodeDescriptor>,
}

impl NodeDescRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let min = 3;
        if buf.len() < min {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..3].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        if let Some(ref nd) = self.node_descriptor {
            let n = nd.serialize(&mut buf[3..])?;
            Ok(3 + n)
        } else {
            Ok(3)
        }
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 3 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[1], data[2]]));
        let node_descriptor =
            if status == crate::ZdpStatus::Success && data.len() >= 3 + NodeDescriptor::WIRE_SIZE {
                Some(NodeDescriptor::parse(&data[3..])?)
            } else {
                None
            };
        Ok(Self {
            status,
            nwk_addr_of_interest,
            node_descriptor,
        })
    }
}

// ── Power_Desc (0x0003 / 0x8003) ───────────────────────────────

/// Power_Desc_req: `NWK_addr_of_interest(2)`
pub type PowerDescReq = NodeDescReq;

/// Power_Desc_rsp: `status(1) | NWK_addr(2) | power_descriptor(2)`
#[derive(Debug, Clone)]
pub struct PowerDescRsp {
    pub status: crate::ZdpStatus,
    pub nwk_addr_of_interest: ShortAddress,
    pub power_descriptor: Option<PowerDescriptor>,
}

impl PowerDescRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < 3 {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..3].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        if let Some(ref pd) = self.power_descriptor {
            let n = pd.serialize(&mut buf[3..])?;
            Ok(3 + n)
        } else {
            Ok(3)
        }
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 3 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[1], data[2]]));
        let power_descriptor = if status == crate::ZdpStatus::Success
            && data.len() >= 3 + PowerDescriptor::WIRE_SIZE
        {
            Some(PowerDescriptor::parse(&data[3..])?)
        } else {
            None
        };
        Ok(Self {
            status,
            nwk_addr_of_interest,
            power_descriptor,
        })
    }
}

// ── Simple_Desc (0x0004 / 0x8004) ──────────────────────────────

/// Simple_Desc_req: `NWK_addr_of_interest(2) | endpoint(1)`
#[derive(Debug, Clone)]
pub struct SimpleDescReq {
    pub nwk_addr_of_interest: ShortAddress,
    pub endpoint: u8,
}

impl SimpleDescReq {
    pub const SIZE: usize = 3;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..2].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        buf[2] = self.endpoint;
        Ok(Self::SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::SIZE {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            nwk_addr_of_interest: ShortAddress(u16::from_le_bytes([data[0], data[1]])),
            endpoint: data[2],
        })
    }
}

/// Simple_Desc_rsp: `status(1) | NWK_addr(2) | length(1) | simple_descriptor(var)`
#[derive(Debug, Clone)]
pub struct SimpleDescRsp {
    pub status: crate::ZdpStatus,
    pub nwk_addr_of_interest: ShortAddress,
    pub simple_descriptor: Option<SimpleDescriptor>,
}

impl SimpleDescRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < 4 {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..3].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        if let Some(ref sd) = self.simple_descriptor {
            let desc_len = sd.wire_size();
            buf[3] = desc_len as u8;
            let n = sd.serialize(&mut buf[4..])?;
            Ok(4 + n)
        } else {
            buf[3] = 0;
            Ok(4)
        }
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 4 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[1], data[2]]));
        let desc_len = data[3] as usize;
        let simple_descriptor = if desc_len > 0 && data.len() >= 4 + desc_len {
            Some(SimpleDescriptor::parse(&data[4..4 + desc_len])?)
        } else {
            None
        };
        Ok(Self {
            status,
            nwk_addr_of_interest,
            simple_descriptor,
        })
    }
}

// ── Active_EP (0x0005 / 0x8005) ────────────────────────────────

/// Active_EP_req: `NWK_addr_of_interest(2)`
pub type ActiveEpReq = NodeDescReq;

/// Active_EP_rsp: `status(1) | NWK_addr(2) | EP_count(1) | EP_list(N)`
#[derive(Debug, Clone)]
pub struct ActiveEpRsp {
    pub status: crate::ZdpStatus,
    pub nwk_addr_of_interest: ShortAddress,
    pub active_ep_list: Vec<u8, 32>,
}

impl ActiveEpRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = 4 + self.active_ep_list.len();
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..3].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        buf[3] = self.active_ep_list.len() as u8;
        buf[4..total].copy_from_slice(&self.active_ep_list);
        Ok(total)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 4 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[1], data[2]]));
        let count = data[3] as usize;
        if data.len() < 4 + count {
            return Err(ZdoError::InvalidLength);
        }
        let mut active_ep_list = Vec::new();
        for &ep in &data[4..4 + count] {
            let _ = active_ep_list.push(ep);
        }
        Ok(Self {
            status,
            nwk_addr_of_interest,
            active_ep_list,
        })
    }
}

// ── Match_Desc (0x0006 / 0x8006) ───────────────────────────────

/// Match_Desc_req.
///
/// ```text
/// NWK_addr(2) | profile_id(2) | num_in(1) | in_clusters(2·N) |
/// num_out(1) | out_clusters(2·M)
/// ```
#[derive(Debug, Clone)]
pub struct MatchDescReq {
    pub nwk_addr_of_interest: ShortAddress,
    pub profile_id: u16,
    pub input_clusters: Vec<u16, 16>,
    pub output_clusters: Vec<u16, 16>,
}

impl MatchDescReq {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = 5 + self.input_clusters.len() * 2 + 1 + self.output_clusters.len() * 2;
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        let mut off = 0;
        buf[off..off + 2].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        off += 2;
        buf[off..off + 2].copy_from_slice(&self.profile_id.to_le_bytes());
        off += 2;
        buf[off] = self.input_clusters.len() as u8;
        off += 1;
        for &c in self.input_clusters.iter() {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        buf[off] = self.output_clusters.len() as u8;
        off += 1;
        for &c in self.output_clusters.iter() {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        Ok(off)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 5 {
            return Err(ZdoError::InvalidLength);
        }
        let mut off = 0;
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[off], data[off + 1]]));
        off += 2;
        let profile_id = u16::from_le_bytes([data[off], data[off + 1]]);
        off += 2;

        let in_count = data[off] as usize;
        off += 1;
        if data.len() < off + in_count * 2 + 1 {
            return Err(ZdoError::InvalidLength);
        }
        let mut input_clusters = Vec::new();
        for _ in 0..in_count {
            let c = u16::from_le_bytes([data[off], data[off + 1]]);
            let _ = input_clusters.push(c);
            off += 2;
        }

        let out_count = data[off] as usize;
        off += 1;
        if data.len() < off + out_count * 2 {
            return Err(ZdoError::InvalidLength);
        }
        let mut output_clusters = Vec::new();
        for _ in 0..out_count {
            let c = u16::from_le_bytes([data[off], data[off + 1]]);
            let _ = output_clusters.push(c);
            off += 2;
        }

        Ok(Self {
            nwk_addr_of_interest,
            profile_id,
            input_clusters,
            output_clusters,
        })
    }
}

/// Match_Desc_rsp: `status(1) | NWK_addr(2) | match_len(1) | match_list(N)`
#[derive(Debug, Clone)]
pub struct MatchDescRsp {
    pub status: crate::ZdpStatus,
    pub nwk_addr_of_interest: ShortAddress,
    pub match_list: Vec<u8, 32>,
}

impl MatchDescRsp {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let total = 4 + self.match_list.len();
        if buf.len() < total {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = self.status as u8;
        buf[1..3].copy_from_slice(&self.nwk_addr_of_interest.0.to_le_bytes());
        buf[3] = self.match_list.len() as u8;
        buf[4..total].copy_from_slice(&self.match_list);
        Ok(total)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < 4 {
            return Err(ZdoError::InvalidLength);
        }
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        let nwk_addr_of_interest = ShortAddress(u16::from_le_bytes([data[1], data[2]]));
        let count = data[3] as usize;
        if data.len() < 4 + count {
            return Err(ZdoError::InvalidLength);
        }
        let mut match_list = Vec::new();
        for &ep in &data[4..4 + count] {
            let _ = match_list.push(ep);
        }
        Ok(Self {
            status,
            nwk_addr_of_interest,
            match_list,
        })
    }
}
