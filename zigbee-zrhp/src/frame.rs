//! Link frame layout (spec 5.2, 5.3): the 8-byte header, CRC-32C trailer,
//! frame types and the `ACK`/`NAK`, `RESET_*` and `REJECT` bodies.
//!
//! Decoded frame:
//!
//! ```text
//! ctrl | epoch | endpoint | seq | ack | ep_seq | len (u16 LE) | payload[len] | crc32c (u32 LE)
//! ```
//!
//! [`parse`] validates a decoded frame in the normative order (length, CRC,
//! version, type). [`encode_wire`] builds the COBS-encoded wire form directly
//! from a header and payload slices, without an intermediate buffer.

use crate::cobs::{CobsError, Encoder};
use crate::crc::Crc32c;

/// Link protocol version carried in `ctrl` bits 7–6.
pub const LINK_VERSION: u8 = 1;
/// Header length.
pub const HEADER_LEN: usize = 8;
/// CRC trailer length.
pub const CRC_LEN: usize = 4;
/// Maximum payload (`rx_mtu`, fixed at 640 in v1).
pub const MAX_PAYLOAD: usize = 640;
/// Maximum decoded frame: 8 + 640 + 4.
pub const MAX_FRAME: usize = HEADER_LEN + MAX_PAYLOAD + CRC_LEN;
/// Receive accumulation bound for encoded bytes (spec 5.1: frames with more
/// than 656 encoded bytes are discarded).
pub const MAX_ENCODED_FRAME: usize = 656;
/// Largest wire frame: minimal COBS of 652 bytes (655) plus the delimiter.
pub const MAX_WIRE_FRAME: usize = crate::cobs::max_encoded_len(MAX_FRAME) + 1;
/// A transmit buffer that can always hold one frame plus an optional
/// leading `0x00` resynchronisation byte.
pub const TX_BUFFER_LEN: usize = MAX_WIRE_FRAME + 1;

/// `ctrl` bit 5.
pub const CTRL_POLL: u8 = 0x20;
/// `ctrl` bit 4.
pub const CTRL_RETX: u8 = 0x10;

/// Frame type (`ctrl` bits 3–0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    /// Batched messages; reliable.
    Data = 0,
    /// Acknowledgement ([`AckBody`]).
    Ack = 1,
    /// Negative acknowledgement ([`AckBody`]).
    Nak = 2,
    /// Session reset request ([`ResetBody`]).
    ResetReq = 3,
    /// Session reset acknowledgement ([`ResetBody`]).
    ResetAck = 4,
    /// Refusal of one DATA frame ([`RejectBody`]).
    Reject = 5,
}

impl FrameType {
    /// Decode a type nibble; `None` for reserved types 6–15.
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Data,
            1 => Self::Ack,
            2 => Self::Nak,
            3 => Self::ResetReq,
            4 => Self::ResetAck,
            5 => Self::Reject,
            _ => return None,
        })
    }
}

/// Decoded frame header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Frame type.
    pub ftype: FrameType,
    /// `POLL` flag.
    pub poll: bool,
    /// `RETX` flag.
    pub retx: bool,
    /// Session epoch (0 in RESET frames).
    pub epoch: u8,
    /// Link endpoint (0 in non-DATA frames).
    pub endpoint: u8,
    /// DATA sequence number (0 otherwise).
    pub seq: u8,
    /// Cumulative ack (next expected DATA seq).
    pub ack: u8,
    /// Per-endpoint sequence of a DATA frame (0 otherwise).
    pub ep_seq: u8,
}

impl Header {
    /// A header of type `ftype` with every other field zero.
    pub const fn new(ftype: FrameType) -> Self {
        Self {
            ftype,
            poll: false,
            retx: false,
            epoch: 0,
            endpoint: 0,
            seq: 0,
            ack: 0,
            ep_seq: 0,
        }
    }

    /// The `ctrl` byte.
    pub const fn ctrl(&self) -> u8 {
        (LINK_VERSION << 6)
            | if self.poll { CTRL_POLL } else { 0 }
            | if self.retx { CTRL_RETX } else { 0 }
            | self.ftype as u8
    }

    /// Serialise the header for a payload of `len` bytes.
    pub const fn to_bytes(&self, len: u16) -> [u8; HEADER_LEN] {
        let l = len.to_le_bytes();
        [
            self.ctrl(),
            self.epoch,
            self.endpoint,
            self.seq,
            self.ack,
            self.ep_seq,
            l[0],
            l[1],
        ]
    }
}

/// Frame validation errors, in the order the spec checks them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Shorter than header + CRC, or `len` does not match (counts as `nbad`).
    Length,
    /// CRC mismatch (counts as `nbad`).
    Crc,
    /// Valid CRC, unknown link version (answered with REJECT `UNSUPPORTED`).
    Version(u8),
    /// Valid CRC, reserved frame type (answered with REJECT `UNSUPPORTED`).
    Type(u8),
}

impl FrameError {
    /// `true` for errors that are counted as noise (`nbad`), `false` for
    /// errors after a valid CRC (peer bug / version mismatch).
    pub const fn is_noise(&self) -> bool {
        matches!(self, Self::Length | Self::Crc)
    }
}

/// A parsed frame borrowing the decoded buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    /// Header.
    pub header: Header,
    /// Payload (`len` bytes).
    pub payload: &'a [u8],
}

/// Raw header fields of a frame that passed the CRC but failed the
/// version/type check, needed to build a REJECT.
pub fn raw_header(decoded: &[u8]) -> Option<[u8; HEADER_LEN]> {
    decoded.get(..HEADER_LEN)?.try_into().ok()
}

/// Validate and parse a decoded (COBS-removed) frame.
pub fn parse(decoded: &[u8]) -> Result<Frame<'_>, FrameError> {
    if decoded.len() < HEADER_LEN + CRC_LEN {
        return Err(FrameError::Length);
    }
    let len = u16::from_le_bytes([decoded[6], decoded[7]]) as usize;
    if decoded.len() != HEADER_LEN + len + CRC_LEN {
        return Err(FrameError::Length);
    }
    let body_end = HEADER_LEN + len;
    let mut crc = Crc32c::new();
    crc.update(&decoded[..body_end]);
    let rx_crc = u32::from_le_bytes([
        decoded[body_end],
        decoded[body_end + 1],
        decoded[body_end + 2],
        decoded[body_end + 3],
    ]);
    if crc.finish() != rx_crc {
        return Err(FrameError::Crc);
    }
    let ctrl = decoded[0];
    if ctrl >> 6 != LINK_VERSION {
        return Err(FrameError::Version(ctrl >> 6));
    }
    let ftype = FrameType::from_u8(ctrl & 0x0F).ok_or(FrameError::Type(ctrl & 0x0F))?;
    Ok(Frame {
        header: Header {
            ftype,
            poll: ctrl & CTRL_POLL != 0,
            retx: ctrl & CTRL_RETX != 0,
            epoch: decoded[1],
            endpoint: decoded[2],
            seq: decoded[3],
            ack: decoded[4],
            ep_seq: decoded[5],
        },
        payload: &decoded[HEADER_LEN..body_end],
    })
}

/// Errors building a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// Payload longer than [`MAX_PAYLOAD`].
    PayloadTooLong,
    /// Output buffer too small.
    BufferTooSmall,
}

impl From<CobsError> for EncodeError {
    fn from(_: CobsError) -> Self {
        Self::BufferTooSmall
    }
}

/// Build a decoded frame (header + payload + CRC) into `out`; the payload is
/// the concatenation of `parts`. Returns the frame length.
pub fn encode_decoded(
    header: &Header,
    parts: &[&[u8]],
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let len: usize = parts.iter().map(|p| p.len()).sum();
    if len > MAX_PAYLOAD {
        return Err(EncodeError::PayloadTooLong);
    }
    let total = HEADER_LEN + len + CRC_LEN;
    if out.len() < total {
        return Err(EncodeError::BufferTooSmall);
    }
    out[..HEADER_LEN].copy_from_slice(&header.to_bytes(len as u16));
    let mut pos = HEADER_LEN;
    for p in parts {
        out[pos..pos + p.len()].copy_from_slice(p);
        pos += p.len();
    }
    let crc = crate::crc::crc32c(&out[..pos]);
    out[pos..pos + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
    Ok(total)
}

/// Build the wire form (COBS + `0x00` delimiter) of a frame directly into
/// `out`; the payload is the concatenation of `parts`. When `lead_zero` is
/// set a `0x00` resynchronisation byte is emitted first (spec 5.1). Returns
/// the number of bytes written.
pub fn encode_wire(
    header: &Header,
    parts: &[&[u8]],
    lead_zero: bool,
    out: &mut [u8],
) -> Result<usize, EncodeError> {
    let len: usize = parts.iter().map(|p| p.len()).sum();
    if len > MAX_PAYLOAD {
        return Err(EncodeError::PayloadTooLong);
    }
    let start = usize::from(lead_zero);
    if out.len() <= start {
        return Err(EncodeError::BufferTooSmall);
    }
    if lead_zero {
        out[0] = 0;
    }
    let hdr = header.to_bytes(len as u16);
    let mut crc = Crc32c::new();
    crc.update(&hdr);
    let mut enc = Encoder::new(&mut out[start..])?;
    enc.push_slice(&hdr)?;
    for p in parts {
        crc.update(p);
        enc.push_slice(p)?;
    }
    enc.push_slice(&crc.finish().to_le_bytes())?;
    let n = start + enc.finish();
    *out.get_mut(n).ok_or(EncodeError::BufferTooSmall)? = 0;
    Ok(n + 1)
}

/// One `{endpoint, value}` credit entry (absolute credit edge in ACK/NAK,
/// initial credit in RESET).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CreditEntry {
    /// Link endpoint.
    pub endpoint: u8,
    /// Credit edge (ACK/NAK) or initial credit (RESET).
    pub value: u8,
}

/// Iterator over a raw credit list.
#[derive(Debug, Clone, Default)]
pub struct Credits<'a> {
    raw: &'a [u8],
}

impl<'a> Credits<'a> {
    /// Wrap raw `{endpoint, value}` pairs (an odd trailing byte is ignored).
    pub const fn new(raw: &'a [u8]) -> Self {
        Self { raw }
    }

    /// The raw `2·n` bytes.
    pub const fn raw(&self) -> &'a [u8] {
        self.raw
    }

    /// Number of entries.
    pub const fn len(&self) -> usize {
        self.raw.len() / 2
    }

    /// `true` when the list is empty.
    pub const fn is_empty(&self) -> bool {
        self.raw.is_empty()
    }
}

impl Iterator for Credits<'_> {
    type Item = CreditEntry;
    fn next(&mut self) -> Option<CreditEntry> {
        let (head, rest) = self.raw.split_first_chunk::<2>()?;
        self.raw = rest;
        Some(CreditEntry {
            endpoint: head[0],
            value: head[1],
        })
    }
}

/// Body errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyError {
    /// Body shorter than its fixed part or credit list.
    Truncated,
}

/// `ACK` / `NAK` body.
#[derive(Debug, Clone)]
pub struct AckBody<'a> {
    /// Bit *i* set ⇒ DATA `ack + 1 + i` received.
    pub sack: u32,
    /// Frames discarded for errors since the last valid frame (saturating).
    pub nbad: u8,
    /// Absolute credit edges.
    pub credits: Credits<'a>,
}

impl<'a> AckBody<'a> {
    /// Parse; trailing bytes after the credit list are ignored (6.6).
    pub fn parse(payload: &'a [u8]) -> Result<Self, BodyError> {
        if payload.len() < 6 {
            return Err(BodyError::Truncated);
        }
        let n = payload[5] as usize;
        let raw = payload.get(6..6 + 2 * n).ok_or(BodyError::Truncated)?;
        Ok(Self {
            sack: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            nbad: payload[4],
            credits: Credits { raw },
        })
    }

    /// Serialise into `out`; returns the length.
    pub fn write(
        sack: u32,
        nbad: u8,
        credits: &[CreditEntry],
        out: &mut [u8],
    ) -> Result<usize, EncodeError> {
        let len = 6 + 2 * credits.len();
        if credits.len() > u8::MAX as usize || out.len() < len {
            return Err(EncodeError::BufferTooSmall);
        }
        out[..4].copy_from_slice(&sack.to_le_bytes());
        out[4] = nbad;
        out[5] = credits.len() as u8;
        for (i, c) in credits.iter().enumerate() {
            out[6 + 2 * i] = c.endpoint;
            out[7 + 2 * i] = c.value;
        }
        Ok(len)
    }
}

/// Link role in `ResetBody.role`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Role {
    /// Host side.
    Host = 0,
    /// NCP side.
    Ncp = 1,
}

/// `ResetBody.reason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ResetReason {
    /// Power-on.
    PowerOn = 0,
    /// Peer timeout (link down).
    PeerTimeout = 1,
    /// Protocol error.
    ProtocolError = 2,
    /// Retransmission retries exhausted.
    RetryExhausted = 3,
    /// Host opened the port.
    HostOpen = 4,
    /// Peer requested (for example after `SYS_SET_UART`).
    PeerRequested = 5,
    /// Watchdog reset.
    Watchdog = 6,
    /// Firmware update.
    FirmwareUpdate = 7,
}

impl ResetReason {
    /// Decode; unknown values map to `None`.
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::PowerOn,
            1 => Self::PeerTimeout,
            2 => Self::ProtocolError,
            3 => Self::RetryExhausted,
            4 => Self::HostOpen,
            5 => Self::PeerRequested,
            6 => Self::Watchdog,
            7 => Self::FirmwareUpdate,
            _ => return None,
        })
    }
}

/// `RESET_REQ` / `RESET_ACK` body (layout frozen across majors).
#[derive(Debug, Clone)]
pub struct ResetBody<'a> {
    /// Sender's boot/open cookie (non-zero).
    pub cookie: u32,
    /// Acknowledged (RESET_ACK) or last known (RESET_REQ) peer cookie; 0 = none.
    pub peer_cookie: u32,
    /// Raw role byte (0 host, 1 NCP).
    pub role: u8,
    /// Highest link version supported.
    pub link_version: u8,
    /// Out-of-order frames the sender can buffer (4..=32).
    pub rx_window: u8,
    /// Raw reason byte ([`ResetReason`]).
    pub reason: u8,
    /// Largest payload accepted (640 in v1).
    pub rx_mtu: u16,
    /// Sender's delayed-ack timer (µs).
    pub ack_delay_us: u16,
    /// Proposed keepalive interval (ms).
    pub keepalive_ms: u16,
    /// Proposed link timeout (ms).
    pub link_timeout_ms: u16,
    /// Initial credits the peer may use per endpoint (from `ep_seq` 0).
    pub credits: Credits<'a>,
}

impl<'a> ResetBody<'a> {
    /// Fixed part length.
    pub const FIXED_LEN: usize = 21;

    /// Parse; trailing bytes after the credit list are ignored.
    pub fn parse(p: &'a [u8]) -> Result<Self, BodyError> {
        if p.len() < Self::FIXED_LEN {
            return Err(BodyError::Truncated);
        }
        let n = p[20] as usize;
        let raw = p.get(21..21 + 2 * n).ok_or(BodyError::Truncated)?;
        let u16_at = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        Ok(Self {
            cookie: u32::from_le_bytes([p[0], p[1], p[2], p[3]]),
            peer_cookie: u32::from_le_bytes([p[4], p[5], p[6], p[7]]),
            role: p[8],
            link_version: p[9],
            rx_window: p[10],
            reason: p[11],
            rx_mtu: u16_at(12),
            ack_delay_us: u16_at(14),
            keepalive_ms: u16_at(16),
            link_timeout_ms: u16_at(18),
            credits: Credits { raw },
        })
    }

    /// Serialise the fixed fields plus `credits` into `out`.
    pub fn write(&self, credits: &[CreditEntry], out: &mut [u8]) -> Result<usize, EncodeError> {
        let len = Self::FIXED_LEN + 2 * credits.len();
        if credits.len() > u8::MAX as usize || out.len() < len {
            return Err(EncodeError::BufferTooSmall);
        }
        out[0..4].copy_from_slice(&self.cookie.to_le_bytes());
        out[4..8].copy_from_slice(&self.peer_cookie.to_le_bytes());
        out[8] = self.role;
        out[9] = self.link_version;
        out[10] = self.rx_window;
        out[11] = self.reason;
        out[12..14].copy_from_slice(&self.rx_mtu.to_le_bytes());
        out[14..16].copy_from_slice(&self.ack_delay_us.to_le_bytes());
        out[16..18].copy_from_slice(&self.keepalive_ms.to_le_bytes());
        out[18..20].copy_from_slice(&self.link_timeout_ms.to_le_bytes());
        out[20] = credits.len() as u8;
        for (i, c) in credits.iter().enumerate() {
            out[21 + 2 * i] = c.endpoint;
            out[22 + 2 * i] = c.value;
        }
        Ok(len)
    }
}

/// `REJECT` reasons (spec 5.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RejectReason {
    /// Frame exceeded the advertised credit.
    NoCredit = 1,
    /// Payload longer than `rx_mtu`.
    TooLong = 2,
    /// Endpoint not supported.
    UnknownEndpoint = 3,
    /// Unknown link version or frame type.
    Unsupported = 4,
    /// DATA seq outside the receive window.
    WindowViolation = 5,
    /// DATA with a stale epoch.
    StaleEpoch = 6,
    /// DATA while no session is established.
    NotInSession = 7,
}

impl RejectReason {
    /// Decode; unknown values map to `None`.
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::NoCredit,
            2 => Self::TooLong,
            3 => Self::UnknownEndpoint,
            4 => Self::Unsupported,
            5 => Self::WindowViolation,
            6 => Self::StaleEpoch,
            7 => Self::NotInSession,
            _ => return None,
        })
    }
}

/// `REJECT` body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RejectBody {
    /// Rejected DATA seq.
    pub seq: u8,
    /// Its endpoint.
    pub endpoint: u8,
    /// Its `ep_seq`.
    pub ep_seq: u8,
    /// Raw reason byte ([`RejectReason`]).
    pub reason: u8,
}

impl RejectBody {
    /// Body length.
    pub const LEN: usize = 4;

    /// Parse; trailing bytes are ignored.
    pub fn parse(p: &[u8]) -> Result<Self, BodyError> {
        if p.len() < Self::LEN {
            return Err(BodyError::Truncated);
        }
        Ok(Self {
            seq: p[0],
            endpoint: p[1],
            ep_seq: p[2],
            reason: p[3],
        })
    }

    /// Serialised body.
    pub const fn to_bytes(&self) -> [u8; Self::LEN] {
        [self.seq, self.endpoint, self.ep_seq, self.reason]
    }
}

/// Session epoch from both cookies (spec 5.5):
/// `1 + crc32c(le32(host_cookie) || le32(ncp_cookie)) mod 255`.
pub fn session_epoch(host_cookie: u32, ncp_cookie: u32) -> u8 {
    let mut crc = Crc32c::new();
    crc.update(&host_cookie.to_le_bytes());
    crc.update(&ncp_cookie.to_le_bytes());
    1 + (crc.finish() % 255) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch_vector() {
        assert_eq!(session_epoch(0x1A2B_3C4D, 0xC0FF_EE01), 33);
    }

    #[test]
    fn max_wire() {
        assert_eq!(MAX_WIRE_FRAME, 656);
    }
}
