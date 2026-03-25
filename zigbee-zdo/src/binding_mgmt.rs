//! Bind / Unbind management (ZDP clusters 0x0021–0x0022 / 0x8021–0x8022).
//!
//! The bind request carries a full binding-table entry definition; the
//! response is a single status byte.

use zigbee_types::IeeeAddress;

use crate::ZdoError;

// ── Bind target ─────────────────────────────────────────────────

/// Destination address mode inside a Bind/Unbind request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BindDstMode {
    /// 16-bit group address (no endpoint).
    Group = 0x01,
    /// 64-bit IEEE address + endpoint.
    Extended = 0x03,
}

/// Bind destination — either a group address or an IEEE address + endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BindTarget {
    Group(u16),
    Unicast {
        dst_addr: IeeeAddress,
        dst_endpoint: u8,
    },
}

// ── Bind_req / Unbind_req (0x0021, 0x0022) ─────────────────────

/// Bind_req / Unbind_req payload (identical layout).
///
/// ```text
/// src_addr(8) | src_endpoint(1) | cluster_id(2) | dst_addr_mode(1) |
/// dst_addr(2 or 8) | [dst_endpoint(1)]
/// ```
#[derive(Debug, Clone)]
pub struct BindReq {
    pub src_addr: IeeeAddress,
    pub src_endpoint: u8,
    pub cluster_id: u16,
    pub dst: BindTarget,
}

impl BindReq {
    /// Minimum size (group destination): 8+1+2+1+2 = 14 bytes.
    pub const MIN_SIZE: usize = 14;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let needed = match self.dst {
            BindTarget::Group(_) => 14,
            BindTarget::Unicast { .. } => 21,
        };
        if buf.len() < needed {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0..8].copy_from_slice(&self.src_addr);
        buf[8] = self.src_endpoint;
        buf[9..11].copy_from_slice(&self.cluster_id.to_le_bytes());
        match self.dst {
            BindTarget::Group(group) => {
                buf[11] = BindDstMode::Group as u8;
                buf[12..14].copy_from_slice(&group.to_le_bytes());
                Ok(14)
            }
            BindTarget::Unicast {
                dst_addr,
                dst_endpoint,
            } => {
                buf[11] = BindDstMode::Extended as u8;
                buf[12..20].copy_from_slice(&dst_addr);
                buf[20] = dst_endpoint;
                Ok(21)
            }
        }
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::MIN_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let mut src_addr = [0u8; 8];
        src_addr.copy_from_slice(&data[0..8]);
        let src_endpoint = data[8];
        let cluster_id = u16::from_le_bytes([data[9], data[10]]);
        let dst_mode = data[11];
        let dst = match dst_mode {
            0x01 => {
                // Group
                if data.len() < 14 {
                    return Err(ZdoError::InvalidLength);
                }
                BindTarget::Group(u16::from_le_bytes([data[12], data[13]]))
            }
            0x03 => {
                // Extended (IEEE + endpoint)
                if data.len() < 21 {
                    return Err(ZdoError::InvalidLength);
                }
                let mut dst_addr = [0u8; 8];
                dst_addr.copy_from_slice(&data[12..20]);
                BindTarget::Unicast {
                    dst_addr,
                    dst_endpoint: data[20],
                }
            }
            _ => return Err(ZdoError::InvalidData),
        };
        Ok(Self {
            src_addr,
            src_endpoint,
            cluster_id,
            dst,
        })
    }
}

/// Unbind_req is identical in layout to Bind_req.
pub type UnbindReq = BindReq;

// ── Bind_rsp / Unbind_rsp (0x8021, 0x8022) ─────────────────────

/// Bind_rsp / Unbind_rsp — single status byte.
#[derive(Debug, Clone, Copy)]
pub struct BindRsp {
    pub status: crate::ZdpStatus,
}

impl BindRsp {
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
        let status = crate::ZdpStatus::from_u8(data[0]).ok_or(ZdoError::InvalidData)?;
        Ok(Self { status })
    }
}

/// Unbind_rsp is identical to Bind_rsp.
pub type UnbindRsp = BindRsp;
