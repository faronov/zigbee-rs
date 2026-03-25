//! Serial protocol types, framing, and codec for USB-attached 802.15.4 dongles.
//!
//! Defines a simple custom serial protocol for a thin MCU bridge firmware
//! (e.g. nRF52840 dongle running a minimal 802.15.4 serial bridge). The
//! protocol maps 1:1 to the [`MacDriver`](crate::MacDriver) trait primitives.
//!
//! # Frame format
//!
//! ```text
//! START(0xF1) | CMD(1) | SEQ(1) | LEN_HI(1) | LEN_LO(1) | PAYLOAD(N) | CRC16_HI(1) | CRC16_LO(1)
//! ```
//!
//! CRC16-CCITT (poly 0x1021, init 0xFFFF) is computed over CMD + SEQ + LEN + PAYLOAD.

#![allow(dead_code)]

pub mod driver;
pub mod ezsp;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use zigbee_types::{MacAddress, PanId, ShortAddress};

// ── Constants ───────────────────────────────────────────────────

pub const FRAME_START: u8 = 0xF1;
pub const MAX_FRAME_SIZE: usize = 256;
pub const FRAME_HEADER_SIZE: usize = 5; // START + CMD + SEQ + LEN_HI + LEN_LO
pub const FRAME_CRC_SIZE: usize = 2;
pub const FRAME_OVERHEAD: usize = FRAME_HEADER_SIZE + FRAME_CRC_SIZE;
pub const MAX_PAYLOAD_SIZE: usize = MAX_FRAME_SIZE - FRAME_OVERHEAD;

// ── Command IDs (host → dongle) ─────────────────────────────────

pub const CMD_RESET_REQ: u8 = 0x01;
pub const CMD_SCAN_REQ: u8 = 0x02;
pub const CMD_ASSOCIATE_REQ: u8 = 0x03;
pub const CMD_ASSOCIATE_RSP: u8 = 0x04;
pub const CMD_DISASSOCIATE_REQ: u8 = 0x05;
pub const CMD_START_REQ: u8 = 0x06;
pub const CMD_GET_REQ: u8 = 0x07;
pub const CMD_SET_REQ: u8 = 0x08;
pub const CMD_POLL_REQ: u8 = 0x09;
pub const CMD_DATA_REQ: u8 = 0x0A;

// ── Command IDs (dongle → host) ─────────────────────────────────

pub const CMD_RESET_CONF: u8 = 0x81;
pub const CMD_SCAN_CONF: u8 = 0x82;
pub const CMD_ASSOCIATE_CONF: u8 = 0x83;
pub const CMD_ASSOCIATE_IND: u8 = 0x84;
pub const CMD_DATA_CONF: u8 = 0x8A;
pub const CMD_DATA_IND: u8 = 0x8B;
pub const CMD_GET_CONF: u8 = 0x87;
pub const CMD_SET_CONF: u8 = 0x88;
pub const CMD_POLL_CONF: u8 = 0x89;
pub const CMD_STATUS: u8 = 0xFF;

// ── Error type ──────────────────────────────────────────────────

/// Errors specific to the serial transport
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialError {
    /// CRC mismatch on received frame
    CrcError,
    /// Frame exceeds maximum size
    FrameTooLong,
    /// Received frame has invalid structure
    MalformedFrame,
    /// Unexpected command ID in response
    UnexpectedCommand,
    /// Timed out waiting for response
    Timeout,
    /// Underlying I/O error
    IoError,
    /// Payload could not be deserialized
    PayloadError,
    /// Buffer too small
    BufferFull,
}

// ── SerialPort trait ────────────────────────────────────────────

/// Abstract async serial port — implemented by platform-specific USB/UART drivers.
pub trait SerialPort {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, SerialError>;
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, SerialError>;
}

// ── CRC16-CCITT ─────────────────────────────────────────────────

/// Compute CRC16-CCITT (poly=0x1021, init=0xFFFF) over the given data.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ── Serial Frame ────────────────────────────────────────────────

/// A parsed serial protocol frame.
#[derive(Debug, Clone)]
pub struct SerialFrame {
    pub cmd: u8,
    pub seq: u8,
    pub payload: heapless::Vec<u8, MAX_PAYLOAD_SIZE>,
}

impl SerialFrame {
    /// Create a new frame with the given command, sequence number, and payload.
    pub fn new(cmd: u8, seq: u8, payload: &[u8]) -> Result<Self, SerialError> {
        let mut v = heapless::Vec::new();
        v.extend_from_slice(payload)
            .map_err(|_| SerialError::FrameTooLong)?;
        Ok(Self {
            cmd,
            seq,
            payload: v,
        })
    }

    /// Serialize this frame into a byte buffer. Returns the number of bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> Result<usize, SerialError> {
        let total = FRAME_OVERHEAD + self.payload.len();
        if total > buf.len() || total > MAX_FRAME_SIZE {
            return Err(SerialError::FrameTooLong);
        }

        let len = self.payload.len() as u16;
        buf[0] = FRAME_START;
        buf[1] = self.cmd;
        buf[2] = self.seq;
        buf[3] = (len >> 8) as u8;
        buf[4] = len as u8;
        buf[5..5 + self.payload.len()].copy_from_slice(&self.payload);

        // CRC over CMD + SEQ + LEN + PAYLOAD
        let crc = crc16_ccitt(&buf[1..5 + self.payload.len()]);
        let crc_offset = 5 + self.payload.len();
        buf[crc_offset] = (crc >> 8) as u8;
        buf[crc_offset + 1] = crc as u8;

        Ok(total)
    }

    /// Parse a frame from a byte buffer. Returns the frame and number of bytes consumed.
    pub fn parse(buf: &[u8]) -> Result<(Self, usize), SerialError> {
        if buf.len() < FRAME_OVERHEAD {
            return Err(SerialError::MalformedFrame);
        }
        if buf[0] != FRAME_START {
            return Err(SerialError::MalformedFrame);
        }

        let cmd = buf[1];
        let seq = buf[2];
        let len = ((buf[3] as u16) << 8) | (buf[4] as u16);
        let payload_len = len as usize;

        let total = FRAME_OVERHEAD + payload_len;
        if total > buf.len() || total > MAX_FRAME_SIZE {
            return Err(SerialError::MalformedFrame);
        }

        // Verify CRC
        let crc_offset = 5 + payload_len;
        let expected_crc = ((buf[crc_offset] as u16) << 8) | (buf[crc_offset + 1] as u16);
        let computed_crc = crc16_ccitt(&buf[1..crc_offset]);
        if expected_crc != computed_crc {
            return Err(SerialError::CrcError);
        }

        let mut payload = heapless::Vec::new();
        payload
            .extend_from_slice(&buf[5..5 + payload_len])
            .map_err(|_| SerialError::FrameTooLong)?;

        Ok((Self { cmd, seq, payload }, total))
    }
}

// ── SerialCodec ─────────────────────────────────────────────────

/// Streaming frame codec that accumulates bytes and extracts complete frames.
pub struct SerialCodec {
    buf: [u8; MAX_FRAME_SIZE],
    pos: usize,
}

impl Default for SerialCodec {
    fn default() -> Self {
        Self::new()
    }
}

impl SerialCodec {
    pub fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME_SIZE],
            pos: 0,
        }
    }

    /// Feed bytes into the codec. Returns Ok(Some(frame)) when a complete
    /// frame has been assembled, Ok(None) if more data is needed.
    pub fn feed(&mut self, data: &[u8]) -> Result<Option<SerialFrame>, SerialError> {
        for &byte in data {
            // If buffer is empty, wait for START byte
            if self.pos == 0 && byte != FRAME_START {
                continue;
            }

            if self.pos >= MAX_FRAME_SIZE {
                // Overflow — reset and look for next START
                self.pos = 0;
                if byte == FRAME_START {
                    self.buf[0] = byte;
                    self.pos = 1;
                }
                continue;
            }

            self.buf[self.pos] = byte;
            self.pos += 1;

            // Need at least header to know payload length
            if self.pos >= FRAME_HEADER_SIZE {
                let payload_len = ((self.buf[3] as usize) << 8) | (self.buf[4] as usize);
                let total = FRAME_OVERHEAD + payload_len;

                if total > MAX_FRAME_SIZE {
                    // Invalid length — reset
                    self.pos = 0;
                    continue;
                }

                if self.pos >= total {
                    let result = SerialFrame::parse(&self.buf[..total]);
                    self.pos = 0;
                    return result.map(|(frame, _)| Some(frame));
                }
            }
        }
        Ok(None)
    }

    /// Reset the codec state.
    pub fn reset(&mut self) {
        self.pos = 0;
    }
}

// ── Payload serialization helpers ───────────────────────────────

/// Helpers for building request payloads and parsing response payloads.
pub struct PayloadBuilder;

impl PayloadBuilder {
    // ── Request payloads (host → dongle) ────────────────────

    /// Build RESET_REQ payload: [set_default_pib: u8]
    pub fn reset_req(set_default_pib: bool, out: &mut [u8]) -> usize {
        out[0] = set_default_pib as u8;
        1
    }

    /// Build SCAN_REQ payload: [scan_type: u8, channel_mask: u32 BE, duration: u8]
    pub fn scan_req(req: &MlmeScanRequest, out: &mut [u8]) -> usize {
        out[0] = req.scan_type as u8;
        let mask = req.channel_mask.0;
        out[1] = (mask >> 24) as u8;
        out[2] = (mask >> 16) as u8;
        out[3] = (mask >> 8) as u8;
        out[4] = mask as u8;
        out[5] = req.scan_duration;
        6
    }

    /// Build ASSOCIATE_REQ payload:
    /// [channel: u8, addr_mode: u8, pan_id: u16 LE, addr(2 or 8), capability: u8]
    pub fn associate_req(req: &MlmeAssociateRequest, out: &mut [u8]) -> usize {
        let mut pos = 0;
        out[pos] = req.channel;
        pos += 1;
        pos += Self::write_mac_address(&req.coord_address, &mut out[pos..]);
        out[pos] = req.capability_info.to_byte();
        pos += 1;
        pos
    }

    /// Build ASSOCIATE_RSP payload:
    /// [device_addr: 8, short_addr: u16 LE, status: u8]
    pub fn associate_rsp(rsp: &MlmeAssociateResponse, out: &mut [u8]) -> usize {
        let mut pos = 0;
        out[pos..pos + 8].copy_from_slice(&rsp.device_address);
        pos += 8;
        out[pos] = rsp.short_address.0 as u8;
        out[pos + 1] = (rsp.short_address.0 >> 8) as u8;
        pos += 2;
        out[pos] = rsp.status as u8;
        pos += 1;
        pos
    }

    /// Build DISASSOCIATE_REQ payload:
    /// [addr_mode + pan + addr, reason: u8, tx_indirect: u8]
    pub fn disassociate_req(req: &MlmeDisassociateRequest, out: &mut [u8]) -> usize {
        let mut pos = 0;
        pos += Self::write_mac_address(&req.device_address, &mut out[pos..]);
        out[pos] = req.reason as u8;
        pos += 1;
        out[pos] = req.tx_indirect as u8;
        pos += 1;
        pos
    }

    /// Build START_REQ payload:
    /// [pan_id: u16 LE, channel: u8, beacon_order: u8, superframe_order: u8,
    ///  pan_coordinator: u8, battery_life_ext: u8]
    pub fn start_req(req: &MlmeStartRequest, out: &mut [u8]) -> usize {
        out[0] = req.pan_id.0 as u8;
        out[1] = (req.pan_id.0 >> 8) as u8;
        out[2] = req.channel;
        out[3] = req.beacon_order;
        out[4] = req.superframe_order;
        out[5] = req.pan_coordinator as u8;
        out[6] = req.battery_life_ext as u8;
        7
    }

    /// Build GET_REQ payload: [attribute: u8]
    pub fn get_req(attr: PibAttribute, out: &mut [u8]) -> usize {
        out[0] = attr as u8;
        1
    }

    /// Build SET_REQ payload: [attribute: u8, value...]
    pub fn set_req(attr: PibAttribute, value: &PibValue, out: &mut [u8]) -> usize {
        let mut pos = 0;
        out[pos] = attr as u8;
        pos += 1;
        pos += Self::write_pib_value(value, &mut out[pos..]);
        pos
    }

    /// Build DATA_REQ payload:
    /// [src_addr_mode: u8, dst_addr, handle: u8, tx_options: u8, payload_len: u16 LE, payload...]
    pub fn data_req(req: &McpsDataRequest<'_>, out: &mut [u8]) -> usize {
        let mut pos = 0;
        out[pos] = req.src_addr_mode as u8;
        pos += 1;
        pos += Self::write_mac_address(&req.dst_address, &mut out[pos..]);
        out[pos] = req.msdu_handle;
        pos += 1;
        let tx_opts = (req.tx_options.ack_tx as u8)
            | ((req.tx_options.indirect as u8) << 1)
            | ((req.tx_options.security_enabled as u8) << 2);
        out[pos] = tx_opts;
        pos += 1;
        let plen = req.payload.len() as u16;
        out[pos] = plen as u8;
        out[pos + 1] = (plen >> 8) as u8;
        pos += 2;
        out[pos..pos + req.payload.len()].copy_from_slice(req.payload);
        pos += req.payload.len();
        pos
    }

    // ── Address helpers ─────────────────────────────────────

    /// Write a MacAddress into a buffer. Returns bytes written.
    /// Format: [addr_mode: u8, pan_id: u16 LE, addr: 2 or 8 bytes]
    fn write_mac_address(addr: &MacAddress, out: &mut [u8]) -> usize {
        match addr {
            MacAddress::Short(pan, short) => {
                out[0] = AddressMode::Short as u8;
                out[1] = pan.0 as u8;
                out[2] = (pan.0 >> 8) as u8;
                out[3] = short.0 as u8;
                out[4] = (short.0 >> 8) as u8;
                5
            }
            MacAddress::Extended(pan, ext) => {
                out[0] = AddressMode::Extended as u8;
                out[1] = pan.0 as u8;
                out[2] = (pan.0 >> 8) as u8;
                out[3..11].copy_from_slice(ext);
                11
            }
        }
    }

    /// Read a MacAddress from a buffer. Returns (address, bytes consumed).
    fn read_mac_address(buf: &[u8]) -> Result<(MacAddress, usize), SerialError> {
        if buf.is_empty() {
            return Err(SerialError::PayloadError);
        }
        match buf[0] {
            0x02 => {
                // Short
                if buf.len() < 5 {
                    return Err(SerialError::PayloadError);
                }
                let pan = PanId(u16::from_le_bytes([buf[1], buf[2]]));
                let short = ShortAddress(u16::from_le_bytes([buf[3], buf[4]]));
                Ok((MacAddress::Short(pan, short), 5))
            }
            0x03 => {
                // Extended
                if buf.len() < 11 {
                    return Err(SerialError::PayloadError);
                }
                let pan = PanId(u16::from_le_bytes([buf[1], buf[2]]));
                let mut ext = [0u8; 8];
                ext.copy_from_slice(&buf[3..11]);
                Ok((MacAddress::Extended(pan, ext), 11))
            }
            _ => Err(SerialError::PayloadError),
        }
    }

    // ── PIB value helpers ───────────────────────────────────

    fn write_pib_value(value: &PibValue, out: &mut [u8]) -> usize {
        match value {
            PibValue::Bool(v) => {
                out[0] = 0x00; // type tag
                out[1] = *v as u8;
                2
            }
            PibValue::U8(v) => {
                out[0] = 0x01;
                out[1] = *v;
                2
            }
            PibValue::U16(v) => {
                out[0] = 0x02;
                out[1] = *v as u8;
                out[2] = (*v >> 8) as u8;
                3
            }
            PibValue::U32(v) => {
                out[0] = 0x03;
                out[1] = *v as u8;
                out[2] = (*v >> 8) as u8;
                out[3] = (*v >> 16) as u8;
                out[4] = (*v >> 24) as u8;
                5
            }
            PibValue::I8(v) => {
                out[0] = 0x04;
                out[1] = *v as u8;
                2
            }
            PibValue::ShortAddress(v) => {
                out[0] = 0x05;
                out[1] = v.0 as u8;
                out[2] = (v.0 >> 8) as u8;
                3
            }
            PibValue::PanId(v) => {
                out[0] = 0x06;
                out[1] = v.0 as u8;
                out[2] = (v.0 >> 8) as u8;
                3
            }
            PibValue::ExtendedAddress(v) => {
                out[0] = 0x07;
                out[1..9].copy_from_slice(v);
                9
            }
            PibValue::Payload(p) => {
                let data = p.as_slice();
                out[0] = 0x08;
                out[1] = data.len() as u8;
                out[2..2 + data.len()].copy_from_slice(data);
                2 + data.len()
            }
        }
    }

    fn read_pib_value(buf: &[u8]) -> Result<(PibValue, usize), SerialError> {
        if buf.is_empty() {
            return Err(SerialError::PayloadError);
        }
        match buf[0] {
            0x00 => {
                if buf.len() < 2 {
                    return Err(SerialError::PayloadError);
                }
                Ok((PibValue::Bool(buf[1] != 0), 2))
            }
            0x01 => {
                if buf.len() < 2 {
                    return Err(SerialError::PayloadError);
                }
                Ok((PibValue::U8(buf[1]), 2))
            }
            0x02 => {
                if buf.len() < 3 {
                    return Err(SerialError::PayloadError);
                }
                Ok((PibValue::U16(u16::from_le_bytes([buf[1], buf[2]])), 3))
            }
            0x03 => {
                if buf.len() < 5 {
                    return Err(SerialError::PayloadError);
                }
                Ok((
                    PibValue::U32(u32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]])),
                    5,
                ))
            }
            0x04 => {
                if buf.len() < 2 {
                    return Err(SerialError::PayloadError);
                }
                Ok((PibValue::I8(buf[1] as i8), 2))
            }
            0x05 => {
                if buf.len() < 3 {
                    return Err(SerialError::PayloadError);
                }
                Ok((
                    PibValue::ShortAddress(ShortAddress(u16::from_le_bytes([buf[1], buf[2]]))),
                    3,
                ))
            }
            0x06 => {
                if buf.len() < 3 {
                    return Err(SerialError::PayloadError);
                }
                Ok((
                    PibValue::PanId(PanId(u16::from_le_bytes([buf[1], buf[2]]))),
                    3,
                ))
            }
            0x07 => {
                if buf.len() < 9 {
                    return Err(SerialError::PayloadError);
                }
                let mut addr = [0u8; 8];
                addr.copy_from_slice(&buf[1..9]);
                Ok((PibValue::ExtendedAddress(addr), 9))
            }
            0x08 => {
                if buf.len() < 2 {
                    return Err(SerialError::PayloadError);
                }
                let len = buf[1] as usize;
                if buf.len() < 2 + len {
                    return Err(SerialError::PayloadError);
                }
                let payload =
                    PibPayload::from_slice(&buf[2..2 + len]).ok_or(SerialError::PayloadError)?;
                Ok((PibValue::Payload(payload), 2 + len))
            }
            _ => Err(SerialError::PayloadError),
        }
    }
}

// ── Response payload parsers ────────────────────────────────────

pub struct PayloadParser;

impl PayloadParser {
    /// Parse RESET_CONF payload: [status: u8]
    pub fn reset_conf(payload: &[u8]) -> Result<u8, SerialError> {
        if payload.is_empty() {
            return Err(SerialError::PayloadError);
        }
        Ok(payload[0])
    }

    /// Parse SCAN_CONF payload:
    /// [status: u8, scan_type: u8, num_results: u8, results...]
    /// For Active/Passive: each result is a PanDescriptor
    /// For ED: each result is [channel: u8, energy: u8]
    pub fn scan_conf(payload: &[u8]) -> Result<MlmeScanConfirm, SerialError> {
        if payload.len() < 3 {
            return Err(SerialError::PayloadError);
        }
        let _status = payload[0];
        let scan_type = match payload[1] {
            0x00 => ScanType::Ed,
            0x01 => ScanType::Active,
            0x02 => ScanType::Passive,
            0x03 => ScanType::Orphan,
            _ => return Err(SerialError::PayloadError),
        };
        let num = payload[2] as usize;
        let mut pos = 3;

        match scan_type {
            ScanType::Ed => {
                let mut energy_list = heapless::Vec::new();
                for _ in 0..num {
                    if pos + 2 > payload.len() {
                        return Err(SerialError::PayloadError);
                    }
                    let _ = energy_list.push(EdValue {
                        channel: payload[pos],
                        energy: payload[pos + 1],
                    });
                    pos += 2;
                }
                Ok(MlmeScanConfirm {
                    scan_type,
                    pan_descriptors: heapless::Vec::new(),
                    energy_list,
                })
            }
            ScanType::Active | ScanType::Passive | ScanType::Orphan => {
                let mut pan_descriptors = heapless::Vec::new();
                for _ in 0..num {
                    let (pd, consumed) = Self::read_pan_descriptor(&payload[pos..])?;
                    let _ = pan_descriptors.push(pd);
                    pos += consumed;
                }
                Ok(MlmeScanConfirm {
                    scan_type,
                    pan_descriptors,
                    energy_list: heapless::Vec::new(),
                })
            }
        }
    }

    /// Parse a PAN descriptor from a byte slice.
    /// Format: [channel: u8, coord_addr(5 or 11), superframe: u16 LE, lqi: u8,
    ///          security: u8, zigbee_beacon(15)]
    fn read_pan_descriptor(buf: &[u8]) -> Result<(PanDescriptor, usize), SerialError> {
        if buf.is_empty() {
            return Err(SerialError::PayloadError);
        }
        let mut pos = 0;
        let channel = buf[pos];
        pos += 1;
        let (coord_address, addr_len) = PayloadBuilder::read_mac_address(&buf[pos..])?;
        pos += addr_len;
        if pos + 4 > buf.len() {
            return Err(SerialError::PayloadError);
        }
        let superframe_raw = u16::from_le_bytes([buf[pos], buf[pos + 1]]);
        pos += 2;
        let lqi = buf[pos];
        pos += 1;
        let security_use = buf[pos] != 0;
        pos += 1;

        // Zigbee beacon payload: protocol_id(1) + nwk_info(2) + epid(8) + tx_offset(3) + update_id(1) = 15
        if pos + 15 > buf.len() {
            return Err(SerialError::PayloadError);
        }
        let protocol_id = buf[pos];
        pos += 1;
        let nwk_byte0 = buf[pos];
        let nwk_byte1 = buf[pos + 1];
        pos += 2;
        let stack_profile = nwk_byte0 & 0x0F;
        let protocol_version = (nwk_byte0 >> 4) & 0x0F;
        let router_capacity = nwk_byte1 & 0x04 != 0;
        let device_depth = (nwk_byte1 >> 3) & 0x0F;
        let end_device_capacity = nwk_byte1 & 0x80 != 0;

        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(&buf[pos..pos + 8]);
        pos += 8;
        let mut tx_offset = [0u8; 3];
        tx_offset.copy_from_slice(&buf[pos..pos + 3]);
        pos += 3;
        let update_id = buf[pos];
        pos += 1;

        Ok((
            PanDescriptor {
                channel,
                coord_address,
                superframe_spec: SuperframeSpec::from_raw(superframe_raw),
                lqi,
                security_use,
                zigbee_beacon: ZigbeeBeaconPayload {
                    protocol_id,
                    stack_profile,
                    protocol_version,
                    router_capacity,
                    device_depth,
                    end_device_capacity,
                    extended_pan_id,
                    tx_offset,
                    update_id,
                },
            },
            pos,
        ))
    }

    /// Parse ASSOCIATE_CONF payload: [status: u8, short_addr: u16 LE]
    pub fn associate_conf(payload: &[u8]) -> Result<MlmeAssociateConfirm, SerialError> {
        if payload.len() < 3 {
            return Err(SerialError::PayloadError);
        }
        let status = match payload[0] {
            0x00 => AssociationStatus::Success,
            0x01 => AssociationStatus::PanAtCapacity,
            _ => AssociationStatus::PanAccessDenied,
        };
        let short_address = ShortAddress(u16::from_le_bytes([payload[1], payload[2]]));
        Ok(MlmeAssociateConfirm {
            short_address,
            status,
        })
    }

    /// Parse ASSOCIATE_IND payload: [device_addr: 8, capability: u8]
    pub fn associate_ind(payload: &[u8]) -> Result<MlmeAssociateIndication, SerialError> {
        if payload.len() < 9 {
            return Err(SerialError::PayloadError);
        }
        let mut device_address = [0u8; 8];
        device_address.copy_from_slice(&payload[0..8]);
        let capability_info = CapabilityInfo::from_byte(payload[8]);
        Ok(MlmeAssociateIndication {
            device_address,
            capability_info,
        })
    }

    /// Parse GET_CONF payload: [status: u8, attribute: u8, value...]
    pub fn get_conf(payload: &[u8]) -> Result<(u8, PibValue), SerialError> {
        if payload.len() < 3 {
            return Err(SerialError::PayloadError);
        }
        let status = payload[0];
        let _attr = payload[1];
        let (value, _) = PayloadBuilder::read_pib_value(&payload[2..])?;
        Ok((status, value))
    }

    /// Parse SET_CONF payload: [status: u8]
    pub fn set_conf(payload: &[u8]) -> Result<u8, SerialError> {
        if payload.is_empty() {
            return Err(SerialError::PayloadError);
        }
        Ok(payload[0])
    }

    /// Parse POLL_CONF payload: [status: u8, has_data: u8, data_len: u16 LE, data...]
    pub fn poll_conf(payload: &[u8]) -> Result<Option<MacFrame>, SerialError> {
        if payload.len() < 2 {
            return Err(SerialError::PayloadError);
        }
        let _status = payload[0];
        let has_data = payload[1] != 0;
        if !has_data {
            return Ok(None);
        }
        if payload.len() < 4 {
            return Err(SerialError::PayloadError);
        }
        let data_len = u16::from_le_bytes([payload[2], payload[3]]) as usize;
        if payload.len() < 4 + data_len {
            return Err(SerialError::PayloadError);
        }
        let frame =
            MacFrame::from_slice(&payload[4..4 + data_len]).ok_or(SerialError::PayloadError)?;
        Ok(Some(frame))
    }

    /// Parse DATA_CONF payload: [status: u8, handle: u8, timestamp: u32 LE (optional)]
    pub fn data_conf(payload: &[u8]) -> Result<McpsDataConfirm, SerialError> {
        if payload.len() < 2 {
            return Err(SerialError::PayloadError);
        }
        let _status = payload[0];
        let msdu_handle = payload[1];
        let timestamp = if payload.len() >= 6 {
            Some(u32::from_le_bytes([
                payload[2], payload[3], payload[4], payload[5],
            ]))
        } else {
            None
        };
        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp,
        })
    }

    /// Parse DATA_IND payload:
    /// [src_addr(5 or 11), dst_addr(5 or 11), lqi: u8, security: u8, data_len: u16 LE, data...]
    pub fn data_ind(payload: &[u8]) -> Result<McpsDataIndication, SerialError> {
        let mut pos = 0;
        let (src_address, src_len) = PayloadBuilder::read_mac_address(&payload[pos..])?;
        pos += src_len;
        let (dst_address, dst_len) = PayloadBuilder::read_mac_address(&payload[pos..])?;
        pos += dst_len;
        if pos + 4 > payload.len() {
            return Err(SerialError::PayloadError);
        }
        let lqi = payload[pos];
        pos += 1;
        let security_use = payload[pos] != 0;
        pos += 1;
        let data_len = u16::from_le_bytes([payload[pos], payload[pos + 1]]) as usize;
        pos += 2;
        if pos + data_len > payload.len() {
            return Err(SerialError::PayloadError);
        }
        let frame =
            MacFrame::from_slice(&payload[pos..pos + data_len]).ok_or(SerialError::PayloadError)?;
        Ok(McpsDataIndication {
            src_address,
            dst_address,
            lqi,
            payload: frame,
            security_use,
        })
    }

    /// Parse STATUS payload: [status_code: u8, detail_len: u8, detail...]
    pub fn status(payload: &[u8]) -> Result<u8, SerialError> {
        if payload.is_empty() {
            return Err(SerialError::PayloadError);
        }
        Ok(payload[0])
    }
}

// ── Tests ───────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_known_vector() {
        // "123456789" → 0x29B1 for CRC-CCITT (0xFFFF init)
        let data = b"123456789";
        assert_eq!(crc16_ccitt(data), 0x29B1);
    }

    #[test]
    fn frame_roundtrip() {
        let frame = SerialFrame::new(CMD_RESET_REQ, 0x01, &[0x01]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();
        let (parsed, consumed) = SerialFrame::parse(&buf[..len]).unwrap();
        assert_eq!(consumed, len);
        assert_eq!(parsed.cmd, CMD_RESET_REQ);
        assert_eq!(parsed.seq, 0x01);
        assert_eq!(parsed.payload.as_slice(), &[0x01]);
    }

    #[test]
    fn codec_feed_incremental() {
        let frame =
            SerialFrame::new(CMD_SCAN_REQ, 0x02, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x03]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();

        let mut codec = SerialCodec::new();
        // Feed byte by byte
        for i in 0..len - 1 {
            assert!(codec.feed(&buf[i..i + 1]).unwrap().is_none());
        }
        let result = codec.feed(&buf[len - 1..len]).unwrap();
        assert!(result.is_some());
        let parsed = result.unwrap();
        assert_eq!(parsed.cmd, CMD_SCAN_REQ);
        assert_eq!(parsed.seq, 0x02);
    }

    #[test]
    fn crc_error_detected() {
        let frame = SerialFrame::new(CMD_RESET_REQ, 0x01, &[0x01]).unwrap();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = frame.serialize(&mut buf).unwrap();
        // Corrupt CRC
        buf[len - 1] ^= 0xFF;
        assert_eq!(
            SerialFrame::parse(&buf[..len]).unwrap_err(),
            SerialError::CrcError
        );
    }
}
