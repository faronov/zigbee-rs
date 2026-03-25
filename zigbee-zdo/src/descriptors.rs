//! Zigbee node, power, simple, complex, and user descriptors.
//!
//! Each descriptor type has `serialize` / `parse` methods that operate on
//! raw byte slices using little-endian encoding per the Zigbee PRO R22 spec.

use heapless::Vec;

use crate::ZdoError;

/// Maximum input/output clusters in a [`SimpleDescriptor`].
pub const MAX_CLUSTERS: usize = 16;

// ── Logical type (3 bits, Node Descriptor) ──────────────────────

/// Zigbee logical device type (3-bit field in the Node Descriptor).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LogicalType {
    Coordinator = 0x00,
    Router = 0x01,
    EndDevice = 0x02,
}

impl LogicalType {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val & 0x07 {
            0x00 => Some(Self::Coordinator),
            0x01 => Some(Self::Router),
            0x02 => Some(Self::EndDevice),
            _ => None,
        }
    }
}

// ── Node Descriptor (13 bytes on the wire) ──────────────────────

/// Node Descriptor (Zigbee spec Table 2-33).
///
/// ```text
/// Byte 0:  logical_type(3) | complex_avail(1) | user_avail(1) | aps_flags(3)
/// Byte 1:  frequency_band(5) | reserved(3)
/// Byte 2:  mac_capabilities
/// Bytes 3-4:  manufacturer_code (LE)
/// Byte 5:  max_buffer_size
/// Bytes 6-7:  max_incoming_transfer (LE)
/// Bytes 8-9:  server_mask (LE)
/// Bytes 10-11: max_outgoing_transfer (LE)
/// Byte 12: descriptor_capabilities
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodeDescriptor {
    pub logical_type: LogicalType,
    pub complex_desc_available: bool,
    pub user_desc_available: bool,
    /// Deprecated APS flags (bits 5-7 of byte 0). Usually 0.
    pub aps_flags: u8,
    /// 5-bit bitmask: bit 3 = 2.4 GHz.
    pub frequency_band: u8,
    /// MAC capability flags byte.
    pub mac_capabilities: u8,
    pub manufacturer_code: u16,
    pub max_buffer_size: u8,
    pub max_incoming_transfer: u16,
    /// Server mask (Primary Trust Center, Backup Trust Center, …).
    pub server_mask: u16,
    pub max_outgoing_transfer: u16,
    pub descriptor_capabilities: u8,
}

impl Default for NodeDescriptor {
    fn default() -> Self {
        Self {
            logical_type: LogicalType::EndDevice,
            complex_desc_available: false,
            user_desc_available: false,
            aps_flags: 0,
            frequency_band: 0x08, // 2.4 GHz
            mac_capabilities: 0,
            manufacturer_code: 0,
            max_buffer_size: 127,
            max_incoming_transfer: 127,
            server_mask: 0,
            max_outgoing_transfer: 127,
            descriptor_capabilities: 0,
        }
    }
}

impl NodeDescriptor {
    /// On-the-wire size in bytes.
    pub const WIRE_SIZE: usize = 13;

    /// Serialize into `buf`, returning the number of bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::WIRE_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = (self.logical_type as u8 & 0x07)
            | if self.complex_desc_available { 0x08 } else { 0 }
            | if self.user_desc_available { 0x10 } else { 0 }
            | ((self.aps_flags & 0x07) << 5);
        buf[1] = self.frequency_band & 0x1F;
        buf[2] = self.mac_capabilities;
        buf[3..5].copy_from_slice(&self.manufacturer_code.to_le_bytes());
        buf[5] = self.max_buffer_size;
        buf[6..8].copy_from_slice(&self.max_incoming_transfer.to_le_bytes());
        buf[8..10].copy_from_slice(&self.server_mask.to_le_bytes());
        buf[10..12].copy_from_slice(&self.max_outgoing_transfer.to_le_bytes());
        buf[12] = self.descriptor_capabilities;
        Ok(Self::WIRE_SIZE)
    }

    /// Parse from a byte slice.
    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::WIRE_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let logical_type = LogicalType::from_u8(data[0] & 0x07).ok_or(ZdoError::InvalidData)?;
        Ok(Self {
            logical_type,
            complex_desc_available: data[0] & 0x08 != 0,
            user_desc_available: data[0] & 0x10 != 0,
            aps_flags: (data[0] >> 5) & 0x07,
            frequency_band: data[1] & 0x1F,
            mac_capabilities: data[2],
            manufacturer_code: u16::from_le_bytes([data[3], data[4]]),
            max_buffer_size: data[5],
            max_incoming_transfer: u16::from_le_bytes([data[6], data[7]]),
            server_mask: u16::from_le_bytes([data[8], data[9]]),
            max_outgoing_transfer: u16::from_le_bytes([data[10], data[11]]),
            descriptor_capabilities: data[12],
        })
    }
}

// ── Power Descriptor (2 bytes on the wire) ──────────────────────

/// Power Descriptor (Zigbee spec Table 2-35).
///
/// ```text
/// Byte 0:  current_power_mode(4) | available_power_sources(4)
/// Byte 1:  current_power_source(4) | current_power_level(4)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PowerDescriptor {
    /// Current power mode (4 bits).
    pub current_power_mode: u8,
    /// Available power sources (4-bit bitmask).
    pub available_power_sources: u8,
    /// Current power source (4-bit bitmask).
    pub current_power_source: u8,
    /// Current power source level (4 bits).
    pub current_power_level: u8,
}

impl Default for PowerDescriptor {
    fn default() -> Self {
        Self {
            current_power_mode: 0,
            available_power_sources: 0x01, // constant (mains)
            current_power_source: 0x01,
            current_power_level: 0x0C, // 100%
        }
    }
}

impl PowerDescriptor {
    pub const WIRE_SIZE: usize = 2;

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        if buf.len() < Self::WIRE_SIZE {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = (self.current_power_mode & 0x0F) | ((self.available_power_sources & 0x0F) << 4);
        buf[1] = (self.current_power_source & 0x0F) | ((self.current_power_level & 0x0F) << 4);
        Ok(Self::WIRE_SIZE)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::WIRE_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        Ok(Self {
            current_power_mode: data[0] & 0x0F,
            available_power_sources: (data[0] >> 4) & 0x0F,
            current_power_source: data[1] & 0x0F,
            current_power_level: (data[1] >> 4) & 0x0F,
        })
    }
}

// ── Simple Descriptor (variable length) ─────────────────────────

/// Simple Descriptor (Zigbee spec Table 2-36).
///
/// ```text
/// Byte 0:       endpoint
/// Bytes 1-2:    profile_id (LE)
/// Bytes 3-4:    device_id (LE)
/// Byte 5:       device_version(4) | reserved(4)
/// Byte 6:       input_cluster_count (N)
/// Bytes 7..:    input_cluster_list (2·N bytes, LE each)
/// Next byte:    output_cluster_count (M)
/// Following:    output_cluster_list (2·M bytes, LE each)
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimpleDescriptor {
    pub endpoint: u8,
    pub profile_id: u16,
    pub device_id: u16,
    pub device_version: u8,
    pub input_clusters: Vec<u16, MAX_CLUSTERS>,
    pub output_clusters: Vec<u16, MAX_CLUSTERS>,
}

impl SimpleDescriptor {
    /// Minimum size with zero clusters: endpoint(1) + profile(2) + device(2) +
    /// version(1) + in_count(1) + out_count(1) = 8.
    pub const MIN_WIRE_SIZE: usize = 8;

    /// Calculate the serialized size.
    pub fn wire_size(&self) -> usize {
        Self::MIN_WIRE_SIZE + self.input_clusters.len() * 2 + self.output_clusters.len() * 2
    }

    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let needed = self.wire_size();
        if buf.len() < needed {
            return Err(ZdoError::BufferTooSmall);
        }
        let mut off = 0;
        buf[off] = self.endpoint;
        off += 1;
        buf[off..off + 2].copy_from_slice(&self.profile_id.to_le_bytes());
        off += 2;
        buf[off..off + 2].copy_from_slice(&self.device_id.to_le_bytes());
        off += 2;
        buf[off] = self.device_version & 0x0F;
        off += 1;
        // Input clusters
        buf[off] = self.input_clusters.len() as u8;
        off += 1;
        for &c in self.input_clusters.iter() {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        // Output clusters
        buf[off] = self.output_clusters.len() as u8;
        off += 1;
        for &c in self.output_clusters.iter() {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        Ok(off)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.len() < Self::MIN_WIRE_SIZE {
            return Err(ZdoError::InvalidLength);
        }
        let mut off = 0;
        let endpoint = data[off];
        off += 1;
        let profile_id = u16::from_le_bytes([data[off], data[off + 1]]);
        off += 2;
        let device_id = u16::from_le_bytes([data[off], data[off + 1]]);
        off += 2;
        let device_version = data[off] & 0x0F;
        off += 1;

        // Input clusters
        let in_count = data[off] as usize;
        off += 1;
        if data.len() < off + in_count * 2 + 1 {
            return Err(ZdoError::InvalidLength);
        }
        let mut input_clusters = Vec::new();
        for _ in 0..in_count {
            let c = u16::from_le_bytes([data[off], data[off + 1]]);
            input_clusters.push(c).map_err(|_| ZdoError::InvalidData)?;
            off += 2;
        }

        // Output clusters
        let out_count = data[off] as usize;
        off += 1;
        if data.len() < off + out_count * 2 {
            return Err(ZdoError::InvalidLength);
        }
        let mut output_clusters = Vec::new();
        for _ in 0..out_count {
            let c = u16::from_le_bytes([data[off], data[off + 1]]);
            output_clusters.push(c).map_err(|_| ZdoError::InvalidData)?;
            off += 2;
        }

        Ok(Self {
            endpoint,
            profile_id,
            device_id,
            device_version,
            input_clusters,
            output_clusters,
        })
    }
}

// ── Complex Descriptor (simplified) ─────────────────────────────

/// Simplified Complex Descriptor.
///
/// The full Complex Descriptor is a list of compressed XML tags; we store
/// the raw bytes and expose a length.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComplexDescriptor {
    pub data: Vec<u8, 64>,
}

impl Default for ComplexDescriptor {
    fn default() -> Self {
        Self { data: Vec::new() }
    }
}

impl ComplexDescriptor {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let len = self.data.len();
        if buf.len() < len {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[..len].copy_from_slice(&self.data);
        Ok(len)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        let mut v = Vec::new();
        for &b in data {
            v.push(b).map_err(|_| ZdoError::InvalidData)?;
        }
        Ok(Self { data: v })
    }
}

// ── User Descriptor (≤ 16 ASCII characters) ─────────────────────

/// User Descriptor — up to 16 characters of user-settable text.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UserDescriptor {
    pub data: Vec<u8, 16>,
}

impl Default for UserDescriptor {
    fn default() -> Self {
        Self { data: Vec::new() }
    }
}

impl UserDescriptor {
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, ZdoError> {
        let len = self.data.len();
        if buf.len() < 1 + len {
            return Err(ZdoError::BufferTooSmall);
        }
        buf[0] = len as u8;
        buf[1..1 + len].copy_from_slice(&self.data);
        Ok(1 + len)
    }

    pub fn parse(data: &[u8]) -> Result<Self, ZdoError> {
        if data.is_empty() {
            return Err(ZdoError::InvalidLength);
        }
        let len = data[0] as usize;
        if data.len() < 1 + len {
            return Err(ZdoError::InvalidLength);
        }
        let mut v = Vec::new();
        for &b in &data[1..1 + len] {
            v.push(b).map_err(|_| ZdoError::InvalidData)?;
        }
        Ok(Self { data: v })
    }
}
