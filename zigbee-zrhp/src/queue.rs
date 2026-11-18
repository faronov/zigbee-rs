//! Transmit-side message queues implementing [`TxSource`].
//!
//! The link layer never owns message queues: when a window slot and credit
//! are available it asks a [`TxSource`] to fill the slot's payload buffer
//! with whole messages of the endpoint it scheduled (opportunistic batching,
//! spec 5.10). [`EndpointQueues`] is a ready-made, fixed-size source.

use crate::codec::CodecError;
use crate::codec::Wire;
use crate::msg::{BatchWriter, HEADER_LEN, Kind, MessageDef};
use crate::types::{Endpoint, Status};

/// A supplier of outgoing messages, polled by
/// [`Link::poll_transmit`](crate::link::Link::poll_transmit).
pub trait TxSource {
    /// `true` when `endpoint` has at least one queued message.
    fn ready(&self, endpoint: Endpoint) -> bool;

    /// Move as many whole messages of `endpoint` as fit into `buf` (whose
    /// length is the peer MTU) and return the number of bytes written.
    /// Messages must never be split. Returning 0 sends nothing.
    fn fill(&mut self, endpoint: Endpoint, buf: &mut [u8]) -> usize;
}

/// A source with nothing to send.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoSource;

impl TxSource for NoSource {
    fn ready(&self, _: Endpoint) -> bool {
        false
    }
    fn fill(&mut self, _: Endpoint, _: &mut [u8]) -> usize {
        0
    }
}

/// FIFO of encoded messages for one endpoint, `N` bytes.
#[derive(Debug)]
pub struct TxQueue<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> TxQueue<N> {
    /// Empty queue.
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
        }
    }

    /// Queued bytes.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// `true` when nothing is queued.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Free bytes.
    pub const fn free(&self) -> usize {
        N - self.len
    }

    /// Drop everything.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Append messages written by `f`; on error nothing is queued.
    pub fn push_with(
        &mut self,
        f: impl FnOnce(&mut BatchWriter<'_>) -> Result<(), CodecError>,
    ) -> Result<(), CodecError> {
        let mut w = BatchWriter::new(&mut self.buf[self.len..]);
        f(&mut w)?;
        self.len += w.len();
        Ok(())
    }

    /// Queue a typed request.
    pub fn request<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        self.push_with(|w| w.request(tid, body))
    }

    /// Queue a typed response.
    pub fn response<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        status: Status,
        body: Option<&T>,
    ) -> Result<(), CodecError> {
        self.push_with(|w| w.response(tid, status, body))
    }

    /// Queue a typed event.
    pub fn event<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        seq: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        self.push_with(|w| w.event(seq, body))
    }

    /// Queue a raw message.
    pub fn raw(
        &mut self,
        kind: Kind,
        msg_id: u16,
        tid: u16,
        payload: &[u8],
    ) -> Result<(), CodecError> {
        self.push_with(|w| w.push_raw(kind, false, msg_id, tid, payload))
    }

    /// Move the longest prefix of whole messages that fits `out`.
    pub fn fill(&mut self, out: &mut [u8]) -> usize {
        let mut take = 0;
        while take + HEADER_LEN <= self.len {
            let mlen = u16::from_le_bytes([self.buf[take + 5], self.buf[take + 6]]) as usize;
            let end = take + HEADER_LEN + mlen;
            if end > out.len() || end > self.len {
                break;
            }
            take = end;
        }
        out[..take].copy_from_slice(&self.buf[..take]);
        self.buf.copy_within(take..self.len, 0);
        self.len -= take;
        take
    }
}

impl<const N: usize> Default for TxQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// One [`TxQueue`] of `N` bytes per DATA endpoint.
#[derive(Debug)]
pub struct EndpointQueues<const N: usize> {
    queues: [TxQueue<N>; 5],
}

impl<const N: usize> EndpointQueues<N> {
    /// Empty queues.
    pub const fn new() -> Self {
        Self {
            queues: [const { TxQueue::new() }; 5],
        }
    }

    /// The queue of a DATA endpoint (`Link` panics in debug builds and maps
    /// to SYSTEM otherwise).
    pub fn queue(&mut self, endpoint: Endpoint) -> &mut TxQueue<N> {
        debug_assert!(endpoint != Endpoint::Link);
        &mut self.queues[(endpoint as usize).saturating_sub(1).min(4)]
    }

    /// Queue a typed request on its home endpoint.
    pub fn request<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        let ep = crate::msg::home_endpoint(T::ID).ok_or(CodecError::Invalid)?;
        self.queue(ep).request(tid, body)
    }

    /// Queue a typed response on its home endpoint.
    pub fn response<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        tid: u16,
        status: Status,
        body: Option<&T>,
    ) -> Result<(), CodecError> {
        let ep = crate::msg::home_endpoint(T::ID).ok_or(CodecError::Invalid)?;
        self.queue(ep).response(tid, status, body)
    }

    /// Queue a typed event on its home endpoint.
    pub fn event<'a, T: Wire<'a> + MessageDef>(
        &mut self,
        seq: u16,
        body: &T,
    ) -> Result<(), CodecError> {
        let ep = crate::msg::home_endpoint(T::ID).ok_or(CodecError::Invalid)?;
        self.queue(ep).event(seq, body)
    }

    /// Drop every queued message.
    pub fn clear(&mut self) {
        for q in &mut self.queues {
            q.clear();
        }
    }
}

impl<const N: usize> Default for EndpointQueues<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> TxSource for EndpointQueues<N> {
    fn ready(&self, endpoint: Endpoint) -> bool {
        endpoint != Endpoint::Link && !self.queues[endpoint as usize - 1].is_empty()
    }
    fn fill(&mut self, endpoint: Endpoint, buf: &mut [u8]) -> usize {
        if endpoint == Endpoint::Link {
            return 0;
        }
        self.queues[endpoint as usize - 1].fill(buf)
    }
}
