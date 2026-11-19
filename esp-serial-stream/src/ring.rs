//! Bounded single-producer, single-consumer byte ring.
//!
//! The receive interrupt (producer) and the main loop (consumer) share one
//! ring without a lock. Indices are free-running `usize` counters; the
//! capacity must be a power of two so the wrap is a mask. Only the producer
//! writes `head` and only the consumer writes `tail`, so on a single-core
//! RISC-V target `Acquire`/`Release` ordering is all the synchronization
//! the two contexts need.
//!
//! [`ByteRing::split`] hands out the two endpoints exactly once, so the
//! single-producer/single-consumer contract is enforced by ownership.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// A byte ring of `N` bytes (`N` a power of two, at least 2).
pub struct ByteRing<const N: usize> {
    buf: UnsafeCell<[u8; N]>,
    head: AtomicUsize,
    tail: AtomicUsize,
    split: AtomicBool,
}

// SAFETY: the buffer is only accessed through the unique `Producer` (bytes
// in `[tail+len, tail+N)`, i.e. the free part) and the unique `Consumer`
// (bytes in `[tail, head)`); `split` guarantees at most one of each.
unsafe impl<const N: usize> Sync for ByteRing<N> {}

impl<const N: usize> Default for ByteRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ByteRing<N> {
    const VALID: () = assert!(N >= 2 && N.is_power_of_two(), "ring size must be 2^k");

    pub const fn new() -> Self {
        #[allow(clippy::let_unit_value)]
        let () = Self::VALID;
        Self {
            buf: UnsafeCell::new([0; N]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            split: AtomicBool::new(false),
        }
    }

    /// Capacity in bytes.
    pub const fn capacity(&self) -> usize {
        N
    }

    /// The producer and consumer endpoints; `None` after the first call.
    pub fn split(&self) -> Option<(Producer<'_, N>, Consumer<'_, N>)> {
        if self.split.swap(true, Ordering::AcqRel) {
            return None;
        }
        Some((Producer { ring: self }, Consumer { ring: self }))
    }

    fn len(&self) -> usize {
        self.head
            .load(Ordering::Acquire)
            .wrapping_sub(self.tail.load(Ordering::Acquire))
    }
}

/// Writing endpoint (one context only, normally the receive interrupt).
pub struct Producer<'a, const N: usize> {
    ring: &'a ByteRing<N>,
}

impl<const N: usize> Producer<'_, N> {
    /// Free space in bytes.
    pub fn free(&self) -> usize {
        N - self.ring.len()
    }

    /// Append one byte; `false` (byte not stored) when the ring is full.
    #[inline]
    pub fn push(&mut self, byte: u8) -> bool {
        let head = self.ring.head.load(Ordering::Relaxed);
        let tail = self.ring.tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= N {
            return false;
        }
        // SAFETY: slot `head` is free (checked above) and only this unique
        // producer writes free slots.
        unsafe { (*self.ring.buf.get())[head & (N - 1)] = byte };
        self.ring
            .head
            .store(head.wrapping_add(1), Ordering::Release);
        true
    }

    /// Append as much of `data` as fits; returns the number of bytes stored.
    pub fn push_slice(&mut self, data: &[u8]) -> usize {
        let head = self.ring.head.load(Ordering::Relaxed);
        let tail = self.ring.tail.load(Ordering::Acquire);
        let free = N - head.wrapping_sub(tail);
        let count = free.min(data.len());
        for (offset, byte) in data[..count].iter().enumerate() {
            // SAFETY: as in `push`, every written slot is in the free part.
            unsafe { (*self.ring.buf.get())[head.wrapping_add(offset) & (N - 1)] = *byte };
        }
        self.ring
            .head
            .store(head.wrapping_add(count), Ordering::Release);
        count
    }
}

/// Reading endpoint (one context only, normally the main loop).
pub struct Consumer<'a, const N: usize> {
    ring: &'a ByteRing<N>,
}

impl<const N: usize> Consumer<'_, N> {
    /// Buffered bytes.
    pub fn len(&self) -> usize {
        self.ring.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Free space in bytes (as the producer sees it now).
    pub fn free(&self) -> usize {
        N - self.len()
    }

    /// Move up to `out.len()` bytes out of the ring; returns the count.
    pub fn pop_slice(&mut self, out: &mut [u8]) -> usize {
        let tail = self.ring.tail.load(Ordering::Relaxed);
        let head = self.ring.head.load(Ordering::Acquire);
        let count = head.wrapping_sub(tail).min(out.len());
        for (offset, slot) in out[..count].iter_mut().enumerate() {
            // SAFETY: slots in `[tail, head)` were published by the producer
            // with `Release` and are not written again until `tail` moves.
            *slot = unsafe { (*self.ring.buf.get())[tail.wrapping_add(offset) & (N - 1)] };
        }
        self.ring
            .tail
            .store(tail.wrapping_add(count), Ordering::Release);
        count
    }

    /// Drop every buffered byte.
    pub fn clear(&mut self) {
        let head = self.ring.head.load(Ordering::Acquire);
        self.ring.tail.store(head, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_is_unique() {
        let ring = ByteRing::<8>::new();
        assert!(ring.split().is_some());
        assert!(ring.split().is_none());
    }

    #[test]
    fn fills_to_capacity_and_refuses_overflow() {
        let ring = ByteRing::<8>::new();
        let (mut tx, mut rx) = ring.split().unwrap();
        assert_eq!(tx.push_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]), 8);
        assert!(!tx.push(11));
        assert_eq!(tx.free(), 0);
        let mut out = [0u8; 16];
        assert_eq!(rx.pop_slice(&mut out), 8);
        assert_eq!(&out[..8], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(rx.is_empty());
    }

    #[test]
    fn wraps_and_preserves_order_across_many_cycles() {
        let ring = ByteRing::<16>::new();
        let (mut tx, mut rx) = ring.split().unwrap();
        let mut next_in = 0u8;
        let mut next_out = 0u8;
        let mut out = [0u8; 7];
        for round in 0..1000usize {
            let chunk = (round % 11) + 1;
            for _ in 0..chunk {
                if tx.push(next_in) {
                    next_in = next_in.wrapping_add(1);
                }
            }
            let take = (round % 5) + 1;
            let got = rx.pop_slice(&mut out[..take]);
            for byte in &out[..got] {
                assert_eq!(*byte, next_out);
                next_out = next_out.wrapping_add(1);
            }
        }
        let mut rest = [0u8; 16];
        let got = rx.pop_slice(&mut rest);
        for byte in &rest[..got] {
            assert_eq!(*byte, next_out);
            next_out = next_out.wrapping_add(1);
        }
        assert_eq!(next_in, next_out);
    }

    #[test]
    fn clear_discards_buffered_bytes() {
        let ring = ByteRing::<4>::new();
        let (mut tx, mut rx) = ring.split().unwrap();
        tx.push_slice(&[1, 2, 3]);
        rx.clear();
        assert!(rx.is_empty());
        assert_eq!(tx.free(), 4);
    }
}
