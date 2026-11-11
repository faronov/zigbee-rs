//! GPDF duplicate filtering (GP spec A.3.6.1.2).
//!
//! A GPDF is a duplicate if the same GPD sent a frame with the same MAC
//! sequence number (level 0) or security frame counter (levels 2/3) within
//! `gpDuplicateTimeout`. The filter is a fixed-size ring with wrapping
//! millisecond timestamps.

use crate::types::GpdId;

/// Default gpDuplicateTimeout in milliseconds.
pub const DUPLICATE_TIMEOUT_MS: u32 = 2_000;

#[derive(Debug, Clone, Copy)]
struct Entry {
    gpd: GpdId,
    counter: u32,
    seen_ms: u32,
}

/// Fixed-capacity duplicate filter.
#[derive(Debug)]
pub struct DuplicateFilter<const N: usize> {
    entries: [Option<Entry>; N],
    next: usize,
    timeout_ms: u32,
}

impl<const N: usize> Default for DuplicateFilter<N> {
    fn default() -> Self {
        Self::new(DUPLICATE_TIMEOUT_MS)
    }
}

impl<const N: usize> DuplicateFilter<N> {
    /// Create a filter with the given timeout.
    pub const fn new(timeout_ms: u32) -> Self {
        Self {
            entries: [None; N],
            next: 0,
            timeout_ms,
        }
    }

    fn expired(&self, e: &Entry, now_ms: u32) -> bool {
        now_ms.wrapping_sub(e.seen_ms) >= self.timeout_ms
    }

    /// Returns `true` if the frame is a duplicate; otherwise records it.
    ///
    /// `counter` is the security frame counter for protected frames or the
    /// MAC sequence number for unprotected frames.
    pub fn check_and_record(&mut self, gpd: &GpdId, counter: u32, now_ms: u32) -> bool {
        let mut free = None;
        for i in 0..N {
            match self.entries[i] {
                Some(e) if self.expired(&e, now_ms) => {
                    self.entries[i] = None;
                    free.get_or_insert(i);
                }
                Some(e) if e.gpd == *gpd && e.counter == counter => return true,
                Some(_) => {}
                None => {
                    free.get_or_insert(i);
                }
            }
        }
        if N == 0 {
            return false;
        }
        let slot = free.unwrap_or_else(|| {
            let s = self.next;
            self.next = (self.next + 1) % N;
            s
        });
        self.entries[slot] = Some(Entry {
            gpd: *gpd,
            counter,
            seen_ms: now_ms,
        });
        false
    }

    /// Forget all entries for a GPD (e.g. after decommissioning).
    pub fn forget(&mut self, gpd: &GpdId) {
        for e in self.entries.iter_mut() {
            if e.is_some_and(|e| e.gpd == *gpd) {
                *e = None;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_duplicates_within_timeout() {
        let mut f = DuplicateFilter::<4>::default();
        let g = GpdId::SrcId(1);
        assert!(!f.check_and_record(&g, 5, 0));
        assert!(f.check_and_record(&g, 5, 1999));
        assert!(!f.check_and_record(&g, 6, 10));
        assert!(!f.check_and_record(&GpdId::SrcId(2), 5, 10));
        assert!(!f.check_and_record(&g, 5, 2000));
    }

    #[test]
    fn wraps_time_and_evicts() {
        let mut f = DuplicateFilter::<2>::default();
        let g = GpdId::SrcId(1);
        assert!(!f.check_and_record(&g, 1, u32::MAX - 10));
        assert!(f.check_and_record(&g, 1, 100));
        assert!(!f.check_and_record(&g, 2, 100));
        assert!(!f.check_and_record(&g, 3, 100));
        // Oldest slot was overwritten; table holds at most N entries.
        assert_eq!(f.entries.iter().filter(|e| e.is_some()).count(), 2);
    }
}
