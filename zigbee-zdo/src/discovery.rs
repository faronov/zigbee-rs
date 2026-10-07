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

/// Largest `NumAssocDev` one unfragmented address response can carry:
/// `(ZDP_MAX_RX_PAYLOAD - TSN(1) - 13) / 2 = 43`, reached by an unsecured
/// frame. A NWK-secured response holds at most 34 and a concentrator's own
/// response 25, so every frame this stack can receive or send fits.
pub const NWK_ADDR_RSP_MAX_ASSOC_DEV: usize =
    (crate::ZDP_MAX_RX_PAYLOAD - 1 - NwkAddrRsp::MIN_SIZE - 2) / 2;

const _: () = assert!(NWK_ADDR_RSP_MAX_ASSOC_DEV == 43);

/// NWK_addr_rsp — response to [`NwkAddrReq`] (R22 Table 2-92).
///
/// ```text
/// status(1) | IEEE_addr(8) | NWK_addr(2) | [num_assoc(1) | [start_idx(1) | assoc_list(2·N)]]
/// ```
///
/// Accepted structural forms (checked identically for every status):
/// * 11 octets — no association extension;
/// * 12 octets — `NumAssocDev = 0`, no `StartIndex`;
/// * ≥ 13 octets — `NumAssocDev = N`, `StartIndex` and at least `2·N` list
///   octets; trailing octets after the declared list are ignored.
///
/// A successful parse always holds exactly `NumAssocDev` addresses. This is a
/// structural check only: R22 §2.4.3.1.1 omits the association extension from
/// Single and non-SUCCESS responses, and matching the form to the request type
/// and status is left to the caller.
#[derive(Debug, Clone)]
pub struct NwkAddrRsp {
    pub status: crate::ZdpStatus,
    pub ieee_addr: IeeeAddress,
    pub nwk_addr: ShortAddress,
    pub num_assoc_dev: u8,
    pub start_index: u8,
    pub assoc_dev_list: Vec<ShortAddress, NWK_ADDR_RSP_MAX_ASSOC_DEV>,
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
        let (num_assoc_dev, start_index, list) = match &data[Self::MIN_SIZE..] {
            [] | [0] => (0, 0, &[][..]),
            [_] => return Err(ZdoError::InvalidLength),
            [num, start, list @ ..] => (*num, *start, list),
        };
        let count = usize::from(num_assoc_dev);
        let list = list.get(..2 * count).ok_or(ZdoError::InvalidLength)?;
        if count > NWK_ADDR_RSP_MAX_ASSOC_DEV {
            return Err(ZdoError::InvalidData);
        }
        let mut assoc_dev_list = Vec::new();
        for a in list.chunks_exact(2) {
            assoc_dev_list
                .push(ShortAddress(u16::from_le_bytes([a[0], a[1]])))
                .map_err(|_| ZdoError::InvalidData)?;
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

#[cfg(test)]
mod addr_rsp_tests {
    //! PROTO-01: `NWK_addr_rsp` / `IEEE_addr_rsp` wire-length semantics
    //! (R22 05-3474-22 Table 2-92 / Figure 2-65, §2.4.3.1.1).
    //!
    //! Accepted structural forms (the parser does not check them against the
    //! request type or status; R22 omits the extension from Single and
    //! non-SUCCESS responses):
    //! * 11 octets — no association extension;
    //! * 12 octets — `NumAssocDev = 0` (no `StartIndex`, no list), the R22
    //!   Extended SUCCESS form with no associated devices;
    //! * `13 + 2·N` octets — `NumAssocDev = N ≥ 1` followed by `StartIndex`
    //!   and `N` 16-bit addresses (trailing octets are ignored).
    extern crate std;

    use super::*;
    use crate::ZdpStatus;
    use std::format;
    use std::panic::{AssertUnwindSafe, catch_unwind};
    use std::string::String;
    use std::vec::Vec as StdVec;

    const IEEE: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const NWK: u16 = 0x1234;

    fn base() -> StdVec<u8> {
        let mut v = StdVec::new();
        v.push(ZdpStatus::Success as u8);
        v.extend_from_slice(&IEEE);
        v.extend_from_slice(&NWK.to_le_bytes());
        v
    }

    fn with_entries(num: u8, start: u8, entries: usize) -> StdVec<u8> {
        let mut v = base();
        v.push(num);
        v.push(start);
        for i in 0..entries {
            v.extend_from_slice(&(0x2000u16 + i as u16).to_le_bytes());
        }
        v
    }

    type Outcome = Result<Result<NwkAddrRsp, ZdoError>, String>;

    fn parse_catching(data: &[u8]) -> Outcome {
        catch_unwind(AssertUnwindSafe(|| NwkAddrRsp::parse(data))).map_err(|p| {
            p.downcast_ref::<String>()
                .cloned()
                .or_else(|| p.downcast_ref::<&str>().map(|s| String::from(*s)))
                .unwrap_or_default()
        })
    }

    /// Lengths 0..=14, every structurally distinct `NumAssocDev` value.
    #[test]
    fn nwk_addr_rsp_length_table_0_to_14() {
        let mut failures = StdVec::new();
        for len in 0..=14usize {
            let nums: &[u8] = if len > 11 {
                &[0, 1, 2, 0x21, 0xFF]
            } else {
                &[0]
            };
            for &num in nums {
                let mut data = base();
                data.resize(len.max(11), 0xAB);
                data.truncate(len);
                if len > 11 {
                    data[11] = num;
                }
                let outcome = parse_catching(&data);
                let ok = match (len, num, &outcome) {
                    (0..=10, _, Ok(Err(ZdoError::InvalidLength))) => true,
                    (11, _, Ok(Ok(r))) => r.num_assoc_dev == 0 && r.assoc_dev_list.is_empty(),
                    // NumAssocDev = 0 means "no associated devices": no list may
                    // be fabricated, whatever follows.
                    (12.., 0, Ok(Ok(r))) => r.num_assoc_dev == 0 && r.assoc_dev_list.is_empty(),
                    // NumAssocDev ≥ 1 needs StartIndex + 2·N octets; a frame of
                    // at most 14 octets can carry neither N = 1 (needs 15) nor more.
                    (12.., 1.., Ok(Err(_))) => true,
                    _ => false,
                };
                if !ok {
                    failures.push(format!("len={len:2} num=0x{num:02X} -> {outcome:?}"));
                }
            }
        }
        assert!(
            failures.is_empty(),
            "NWK_addr_rsp length table violations:\n{}",
            failures.join("\n")
        );
    }

    /// The 12-octet Extended/no-devices form is an accepted structural form.
    #[test]
    fn nwk_addr_rsp_parses_the_extended_response_without_associated_devices() {
        let mut data = base();
        data.push(0);
        let rsp = NwkAddrRsp::parse(&data)
            .expect("12-octet NumAssocDev=0 is an accepted structural form");
        assert_eq!(rsp.status, ZdpStatus::Success);
        assert_eq!(rsp.ieee_addr, IEEE);
        assert_eq!(rsp.nwk_addr, ShortAddress(NWK));
        assert_eq!(rsp.num_assoc_dev, 0);
        assert!(rsp.assoc_dev_list.is_empty());
    }

    /// Positive controls: exactly base length, and complete extension frames.
    #[test]
    fn nwk_addr_rsp_parses_complete_frames() {
        let rsp = NwkAddrRsp::parse(&base()).unwrap();
        assert_eq!((rsp.num_assoc_dev, rsp.assoc_dev_list.len()), (0, 0));

        for n in [1u8, 2, 5, 32] {
            let data = with_entries(n, 7, n as usize);
            assert_eq!(data.len(), 13 + 2 * n as usize);
            let rsp = NwkAddrRsp::parse(&data).unwrap();
            assert_eq!(rsp.num_assoc_dev, n);
            assert_eq!(rsp.start_index, 7);
            assert_eq!(rsp.assoc_dev_list.len(), n as usize);
            assert_eq!(rsp.assoc_dev_list[0], ShortAddress(0x2000));
        }
    }

    /// A complete extension header (NumAssocDev + StartIndex) with a missing,
    /// partial or short list is truncated and must not be silently accepted.
    #[test]
    fn nwk_addr_rsp_rejects_a_list_shorter_than_num_assoc_dev() {
        let cases: &[(u8, usize, usize)] = &[
            (1, 0, 0), // 13 octets: header only
            (1, 0, 1), // 14 octets: half an address
            (2, 1, 0), // 15 octets: one of two addresses
            (3, 2, 1), // 18 octets: two and a half of three
            (0xFF, 4, 0),
        ];
        let mut failures = StdVec::new();
        for &(num, entries, extra) in cases {
            let mut data = with_entries(num, 0, entries);
            data.extend(core::iter::repeat_n(0xCD, extra));
            let outcome = parse_catching(&data);
            if !matches!(outcome, Ok(Err(_))) {
                failures.push(format!(
                    "num={num} entries={entries} (+{extra} octet) len={} -> {outcome:?}",
                    data.len()
                ));
            }
        }
        assert!(
            failures.is_empty(),
            "truncated association lists were accepted:\n{}",
            failures.join("\n")
        );
    }

    /// A count the fixed-capacity list cannot hold must never be reported
    /// as a successfully parsed (but silently shortened) list.
    #[test]
    fn nwk_addr_rsp_never_silently_drops_entries_beyond_capacity() {
        let mut failures = StdVec::new();
        for n in [33u8, 57, 0xFF] {
            let data = with_entries(n, 0, n as usize);
            match parse_catching(&data) {
                Ok(Err(_)) => {}
                Ok(Ok(r)) if r.assoc_dev_list.len() == n as usize => {}
                other => failures.push(format!(
                    "num={n} len={} -> list.len()={:?}",
                    data.len(),
                    other.map(|r| r.map(|r| r.assoc_dev_list.len()))
                )),
            }
        }
        assert!(
            failures.is_empty(),
            "entries beyond capacity were silently dropped:\n{}",
            failures.join("\n")
        );
    }

    /// Capacity is the largest list one unfragmented frame can carry (43,
    /// unsecured); at capacity every entry is kept, above it the frame is
    /// rejected explicitly instead of being shortened.
    #[test]
    fn nwk_addr_rsp_capacity_is_the_largest_single_frame_list() {
        let secured_max = (crate::ZDP_MAX_PAYLOAD - 1 - NwkAddrRsp::MIN_SIZE - 2) / 2;
        assert_eq!(secured_max, 34);
        assert_eq!(NWK_ADDR_RSP_MAX_ASSOC_DEV, 43);
        // TSN + largest list fills the unsecured ASDU; one more entry would not.
        assert_eq!(
            1 + NwkAddrRsp::MIN_SIZE + 2 + 2 * NWK_ADDR_RSP_MAX_ASSOC_DEV,
            crate::ZDP_MAX_RX_PAYLOAD
        );
        for n in [secured_max, NWK_ADDR_RSP_MAX_ASSOC_DEV] {
            let data = with_entries(n as u8, 3, n);
            let rsp = NwkAddrRsp::parse(&data).unwrap();
            assert_eq!(usize::from(rsp.num_assoc_dev), n);
            assert_eq!(rsp.assoc_dev_list.len(), n);
            assert_eq!(
                rsp.assoc_dev_list[n - 1],
                ShortAddress(0x2000 + n as u16 - 1)
            );
        }
        let over = NWK_ADDR_RSP_MAX_ASSOC_DEV + 1;
        assert!(matches!(
            NwkAddrRsp::parse(&with_entries(over as u8, 0, over)),
            Err(ZdoError::InvalidData)
        ));
        // A truncated over-capacity frame reports the truncation first.
        assert!(matches!(
            NwkAddrRsp::parse(&with_entries(over as u8, 0, over - 1)),
            Err(ZdoError::InvalidLength)
        ));
    }

    /// The parser intentionally checks structure identically for every
    /// status: it accepts the same structural forms for non-SUCCESS statuses
    /// even though R22 omits the extension there, leaving status semantics to
    /// the caller. Trailing octets after the declared list (or after a zero
    /// count) are ignored.
    #[test]
    fn nwk_addr_rsp_structure_is_status_independent_and_ignores_trailing_octets() {
        let mut data = with_entries(2, 9, 2);
        data.extend_from_slice(&[0xEE, 0xEE, 0xEE]);
        let rsp = NwkAddrRsp::parse(&data).unwrap();
        assert_eq!(rsp.assoc_dev_list.len(), 2);
        assert_eq!(rsp.start_index, 9);

        let mut zero = with_entries(0, 5, 0);
        zero.extend_from_slice(&[0x01, 0x02]);
        let rsp = NwkAddrRsp::parse(&zero).unwrap();
        assert_eq!((rsp.num_assoc_dev, rsp.start_index), (0, 5));
        assert!(rsp.assoc_dev_list.is_empty());

        for status in [ZdpStatus::DeviceNotFound, ZdpStatus::InvRequestType] {
            let mut ok = with_entries(1, 0, 1);
            ok[0] = status as u8;
            assert_eq!(NwkAddrRsp::parse(&ok).unwrap().status, status);
            let mut bad = with_entries(1, 0, 0);
            bad[0] = status as u8;
            assert!(matches!(
                NwkAddrRsp::parse(&bad),
                Err(ZdoError::InvalidLength)
            ));
        }
    }

    /// Arbitrary short byte strings never panic: exhaustive over every
    /// length 0..=16 × every NumAssocDev value × boundary fillers, plus a
    /// seeded random sweep up to 48 octets.
    #[test]
    fn nwk_addr_rsp_parse_never_panics_on_arbitrary_short_input() {
        let mut panics: std::collections::BTreeMap<usize, (u32, String)> = Default::default();
        let mut record = |data: &[u8]| {
            if let Err(msg) = parse_catching(data) {
                let e = panics.entry(data.len()).or_insert((0, msg));
                e.0 += 1;
            }
        };
        for len in 0..=16usize {
            for fill in [0x00u8, 0x01, 0x7F, 0xFF] {
                for num in 0..=255u8 {
                    let mut data = std::vec![fill; len];
                    if len > 11 {
                        data[11] = num;
                    }
                    record(&data);
                }
            }
        }
        let mut seed = 0x9E37_79B9_7F4A_7C15u64;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for _ in 0..20_000 {
            let len = (next() % 49) as usize;
            let data: StdVec<u8> = (0..len).map(|_| next() as u8).collect();
            record(&data);
        }
        assert!(
            panics.is_empty(),
            "NwkAddrRsp::parse panicked (len -> (count, first message)): {panics:#?}"
        );
    }
}
