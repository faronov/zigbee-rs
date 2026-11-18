//! Message layer (spec 6): message header, batches, kinds, IDs and the
//! typed command/event codecs of section 7–9.
//!
//! # Receiving
//!
//! ```
//! use zigbee_zrhp::msg::{self, Batch, Kind};
//! use zigbee_zrhp::msg::system::SysNegotiateReq;
//!
//! # let payload = [0x00,0x01,0x00,0x01,0x00,0x0c,0x00,0x01,0x01,0,0,0,0,0,0,0,0,0x01,b'x'];
//! let batch = Batch::parse(&payload).expect("lengths tile the payload");
//! for m in batch.iter() {
//!     match (m.header.kind, m.header.msg_id) {
//!         (Kind::Request, msg::ids::SYS_NEGOTIATE) => {
//!             let req: SysNegotiateReq = m.decode().unwrap();
//!             assert_eq!(req.name.as_str(), Some("x"));
//!         }
//!         _ => {}
//!     }
//! }
//! ```
//!
//! # Sending
//!
//! [`BatchWriter`] packs whole messages into a DATA payload (or into a
//! [`crate::queue::TxQueue`]).

use crate::codec::{CodecError, Reader, Wire, Writer};
use crate::types::{Endpoint, Status};

pub mod backup;
pub mod data;
pub mod diag;
pub mod firmware;
pub mod gp;
pub mod ids;
pub mod log;
pub mod network;
pub mod security;
pub mod system;
pub mod tables;

/// Message header length.
pub const HEADER_LEN: usize = 7;
/// Largest message payload: 640 − 7.
pub const MAX_MESSAGE_PAYLOAD: usize = crate::frame::MAX_PAYLOAD - HEADER_LEN;

/// `flags` bit 5.
pub const FLAG_RETRY: u8 = 0x20;

/// Message kind (`flags` bits 7–6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Kind {
    /// Request (either direction).
    Request = 0,
    /// Response (first payload byte is the status).
    Response = 1,
    /// Unsolicited event.
    Event = 2,
}

/// Decoded message header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MsgHeader {
    /// Kind.
    pub kind: Kind,
    /// `RETRY` flag (6.5).
    pub retry: bool,
    /// Command or event ID.
    pub msg_id: u16,
    /// Transaction ID (request/response) or event sequence (event).
    pub tid: u16,
    /// Payload length.
    pub len: u16,
}

impl MsgHeader {
    /// Serialise.
    pub const fn to_bytes(&self) -> [u8; HEADER_LEN] {
        let flags = ((self.kind as u8) << 6) | if self.retry { FLAG_RETRY } else { 0 };
        let id = self.msg_id.to_le_bytes();
        let tid = self.tid.to_le_bytes();
        let len = self.len.to_le_bytes();
        [flags, id[0], id[1], tid[0], tid[1], len[0], len[1]]
    }

    /// Parse; reserved kind 3 and non-zero reserved flag bits are errors.
    pub fn parse(b: &[u8]) -> Result<Self, BatchError> {
        if b.len() < HEADER_LEN {
            return Err(BatchError::Truncated);
        }
        let flags = b[0];
        let kind = match flags >> 6 {
            0 => Kind::Request,
            1 => Kind::Response,
            2 => Kind::Event,
            _ => return Err(BatchError::ReservedKind),
        };
        if flags & 0x1F != 0 {
            return Err(BatchError::ReservedFlags);
        }
        Ok(Self {
            kind,
            retry: flags & FLAG_RETRY != 0,
            msg_id: u16::from_le_bytes([b[1], b[2]]),
            tid: u16::from_le_bytes([b[3], b[4]]),
            len: u16::from_le_bytes([b[5], b[6]]),
        })
    }
}

/// Why a DATA payload was rejected as a batch (counted as `msg_bad_batch`
/// and reported with `EVT_PROTOCOL_ERROR`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatchError {
    /// A header or payload runs past the end of the frame.
    Truncated,
    /// Kind 3.
    ReservedKind,
    /// Reserved flag bits set.
    ReservedFlags,
    /// Empty DATA payload (a DATA frame carries at least one message).
    Empty,
}

/// One message borrowed from a DATA payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawMessage<'a> {
    /// Header.
    pub header: MsgHeader,
    /// Payload (`header.len` bytes).
    pub payload: &'a [u8],
}

impl<'a> RawMessage<'a> {
    /// Decode a request or event body. Trailing bytes are ignored (6.6).
    pub fn decode<T: Wire<'a>>(&self) -> Result<T, CodecError> {
        T::read(&mut Reader::new(self.payload))
    }

    /// Decode a response: the status, then the body if any bytes follow it.
    pub fn decode_response<T: Wire<'a>>(&self) -> Result<Response<T>, CodecError> {
        Response::read(&mut Reader::new(self.payload))
    }

    /// The status byte of a response.
    pub fn status(&self) -> Option<Status> {
        self.payload.first().map(|&b| Status::from_u8(b))
    }
}

/// A validated DATA payload: message lengths tile it exactly (6.1).
#[derive(Debug, Clone, Copy)]
pub struct Batch<'a> {
    payload: &'a [u8],
}

impl<'a> Batch<'a> {
    /// Validate the whole payload first; only a valid batch is dispatched.
    pub fn parse(payload: &'a [u8]) -> Result<Self, BatchError> {
        if payload.is_empty() {
            return Err(BatchError::Empty);
        }
        let mut pos = 0;
        while pos < payload.len() {
            let h = MsgHeader::parse(&payload[pos..])?;
            pos += HEADER_LEN + h.len as usize;
            if pos > payload.len() {
                return Err(BatchError::Truncated);
            }
        }
        Ok(Self { payload })
    }

    /// The first message's header, for `EVT_PROTOCOL_ERROR` reporting of a
    /// bad batch (best effort).
    pub fn first_msg_id(payload: &[u8]) -> Option<u16> {
        payload.get(1..3).map(|b| u16::from_le_bytes([b[0], b[1]]))
    }

    /// Iterate the messages in order.
    pub fn iter(&self) -> BatchIter<'a> {
        BatchIter { rest: self.payload }
    }
}

/// Iterator over a [`Batch`].
#[derive(Debug, Clone)]
pub struct BatchIter<'a> {
    rest: &'a [u8],
}

impl<'a> Iterator for BatchIter<'a> {
    type Item = RawMessage<'a>;
    fn next(&mut self) -> Option<RawMessage<'a>> {
        let header = MsgHeader::parse(self.rest).ok()?;
        let end = HEADER_LEN + header.len as usize;
        let payload = self.rest.get(HEADER_LEN..end)?;
        self.rest = &self.rest[end..];
        Some(RawMessage { header, payload })
    }
}

/// A response: status plus an optional body (absent when the status is the
/// only byte, as for most errors).
#[derive(Debug, Clone, PartialEq)]
pub struct Response<T> {
    /// Status.
    pub status: Status,
    /// Fields after the status.
    pub body: Option<T>,
}

impl<'a, T: Wire<'a>> Wire<'a> for Response<T> {
    fn read(r: &mut Reader<'a>) -> Result<Self, CodecError> {
        let status = Status::from_u8(r.u8()?);
        let body = if r.is_empty() {
            None
        } else {
            Some(T::read(r)?)
        };
        Ok(Self { status, body })
    }
    fn write(&self, w: &mut Writer<'_>) -> Result<(), CodecError> {
        w.u8(self.status.to_u8())?;
        if let Some(b) = &self.body {
            b.write(w)?;
        }
        Ok(())
    }
}

/// Static description of a typed message body.
pub trait MessageDef {
    /// `msg_id`.
    const ID: u16;
    /// Kind of message this body belongs to.
    const KIND: Kind;
}

/// Bind body types to their message ID and kind.
macro_rules! message_def {
    ($($t:ident $(<$lt:lifetime>)? = $id:path, $kind:ident;)*) => {
        $(
            impl$(<$lt>)? $crate::msg::MessageDef for $t$(<$lt>)? {
                const ID: u16 = $id;
                const KIND: $crate::msg::Kind = $crate::msg::Kind::$kind;
            }
        )*
    };
}
pub(crate) use message_def;

/// Home endpoint of a `msg_id` group (6.7); `None` for unassigned groups
/// and the experimental range (which lives on endpoints ≥ 32).
pub const fn home_endpoint(msg_id: u16) -> Option<Endpoint> {
    Some(match msg_id >> 8 {
        0x00 | 0x07 => Endpoint::System,
        0x01..=0x04 | 0x0A => Endpoint::Zigbee,
        0x05 => Endpoint::Tables,
        0x06 | 0x08 => Endpoint::Bulk,
        0x09 => Endpoint::Log,
        _ => return None,
    })
}

/// `true` when the low byte marks an event ID (`0x80..=0xFF`).
pub const fn is_event_id(msg_id: u16) -> bool {
    msg_id & 0x80 != 0
}

/// Packs whole messages into a buffer (a DATA payload or a queue slot).
///
/// ```
/// use zigbee_zrhp::msg::{BatchWriter, system::SysSetUart};
/// let mut buf = [0u8; 64];
/// let mut w = BatchWriter::new(&mut buf);
/// w.request(5, &SysSetUart { baud: 921_600, flow: 0, apply_delay_ms: 50 }).unwrap();
/// assert_eq!(w.written(), &[0x00, 0x09, 0x00, 0x05, 0x00, 0x07, 0x00,
///     0x00, 0x10, 0x0e, 0x00, 0x00, 0x32, 0x00]);
/// ```
#[derive(Debug)]
pub struct BatchWriter<'b> {
    w: Writer<'b>,
}

impl<'b> BatchWriter<'b> {
    /// Write into `buf`.
    pub fn new(buf: &'b mut [u8]) -> Self {
        Self {
            w: Writer::new(buf),
        }
    }

    /// Bytes written.
    pub const fn len(&self) -> usize {
        self.w.len()
    }

    /// `true` when nothing was written.
    pub const fn is_empty(&self) -> bool {
        self.w.is_empty()
    }

    /// Written bytes.
    pub fn written(&self) -> &[u8] {
        self.w.written()
    }

    /// Append one message whose payload is produced by `f`. On error the
    /// writer is left at the previous message boundary.
    pub fn push_with(
        &mut self,
        kind: Kind,
        retry: bool,
        msg_id: u16,
        tid: u16,
        f: impl FnOnce(&mut Writer<'_>) -> Result<(), CodecError>,
    ) -> Result<(), CodecError> {
        let start = self.w.len();
        let res = (|| {
            let at = self.w.reserve(HEADER_LEN)?;
            f(&mut self.w)?;
            let len = self.w.len() - at - HEADER_LEN;
            if len > MAX_MESSAGE_PAYLOAD {
                return Err(CodecError::Invalid);
            }
            let h = MsgHeader {
                kind,
                retry,
                msg_id,
                tid,
                len: len as u16,
            };
            self.w.patch(at, &h.to_bytes());
            Ok(())
        })();
        if res.is_err() {
            self.w.truncate(start);
        }
        res
    }

    /// Append a raw message.
    pub fn push_raw(
        &mut self,
        kind: Kind,
        retry: bool,
        msg_id: u16,
        tid: u16,
        payload: &[u8],
    ) -> Result<(), CodecError> {
        self.push_with(kind, retry, msg_id, tid, |w| w.bytes(payload))
    }

    /// Append a typed request.
    pub fn request<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        debug_assert!(matches!(T::KIND, Kind::Request));
        self.push_with(Kind::Request, false, T::ID, tid, |w| body.write(w))
    }

    /// Append a typed request with the `RETRY` flag (6.5).
    pub fn request_retry<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        self.push_with(Kind::Request, true, T::ID, tid, |w| body.write(w))
    }

    /// Append a request without a payload (`SYS_GET_INFO`, …).
    pub fn request_empty(&mut self, msg_id: u16, tid: u16) -> Result<(), CodecError> {
        self.push_with(Kind::Request, false, msg_id, tid, |_| Ok(()))
    }

    /// Append a typed response (`status` + body).
    pub fn response<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        status: Status,
        body: Option<&T>,
    ) -> Result<(), CodecError> {
        self.push_with(Kind::Response, false, T::ID, tid, |w| {
            w.u8(status.to_u8())?;
            match body {
                Some(b) => b.write(w),
                None => Ok(()),
            }
        })
    }

    /// Append a status-only response.
    pub fn response_status(
        &mut self,
        msg_id: u16,
        tid: u16,
        status: Status,
    ) -> Result<(), CodecError> {
        self.push_with(Kind::Response, false, msg_id, tid, |w| w.u8(status.to_u8()))
    }

    /// Append a typed event with event sequence `seq`.
    pub fn event<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        seq: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        debug_assert!(matches!(T::KIND, Kind::Event));
        self.push_with(Kind::Event, false, T::ID, seq, |w| body.write(w))
    }
}

/// 16-bit transaction ID allocator that skips 0 (6.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TidAllocator {
    next: u16,
}

impl TidAllocator {
    /// Start at 1.
    pub const fn new() -> Self {
        Self { next: 1 }
    }

    /// Next TID (never 0).
    pub fn next_tid(&mut self) -> u16 {
        let t = self.next;
        self.next = self.next.wrapping_add(1);
        if self.next == 0 {
            self.next = 1;
        }
        t
    }
}

impl Default for TidAllocator {
    fn default() -> Self {
        Self::new()
    }
}

/// Encode one complete message (header + typed body) into `buf`; returns
/// its length. Convenience for tests and single-message frames.
pub fn encode_message<'a, T: Wire<'a>>(
    kind: Kind,
    msg_id: u16,
    tid: u16,
    body: &T,
    buf: &mut [u8],
) -> Result<usize, CodecError> {
    let mut bw = BatchWriter::new(buf);
    bw.push_with(kind, false, msg_id, tid, |w| body.write(w))?;
    Ok(bw.len())
}
