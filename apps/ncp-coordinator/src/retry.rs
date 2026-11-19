//! Result cache for retried requests (spec 6.5).
//!
//! After a `PEER_RESYNC` (same cookies) the host may resend a request whose
//! outcome it does not know with the same TID and the `RETRY` flag. The
//! NCP keeps `(tid, msg_id, status, ≤ 16 bytes of result)` of the last
//! [`RETRY_CACHE_ENTRIES`] completed host requests of the current cookie
//! pair and answers such a retry from the cache instead of executing it
//! again. A `PEER_RESTARTED` (new host cookie) clears it.
//!
//! Responses with more than 16 bytes of result are read-only queries
//! (`SYS_GET_INFO`, `TBL_READ`, ...); they are not cached and a retry simply
//! runs them again, which is harmless.

use heapless::Deque;

/// Entries kept (spec 6.5).
pub const RETRY_CACHE_ENTRIES: usize = 64;
/// Most result bytes after `status` a cached response carries.
pub const MAX_CACHED_RESULT: usize = 16;

#[derive(Debug, Clone, Copy)]
struct Entry {
    tid: u16,
    msg_id: u16,
    len: u8,
    /// `status` then the result.
    payload: [u8; 1 + MAX_CACHED_RESULT],
}

/// See the module documentation.
#[derive(Debug)]
pub struct RetryCache {
    entries: Deque<Entry, RETRY_CACHE_ENTRIES>,
    hits: u32,
}

impl Default for RetryCache {
    fn default() -> Self {
        Self::new()
    }
}

impl RetryCache {
    pub const fn new() -> Self {
        Self {
            entries: Deque::new(),
            hits: 0,
        }
    }

    /// Forget everything (host restarted).
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Remember a completed response; `payload` is `status` + result.
    /// Longer results are not cached (see the module documentation).
    pub fn remember(&mut self, tid: u16, msg_id: u16, payload: &[u8]) {
        if payload.is_empty() || payload.len() > 1 + MAX_CACHED_RESULT {
            return;
        }
        if let Some(position) = self
            .entries
            .iter()
            .position(|entry| entry.tid == tid && entry.msg_id == msg_id)
        {
            // A reused TID: the newer response replaces the older one.
            let mut kept: Deque<Entry, RETRY_CACHE_ENTRIES> = Deque::new();
            for (index, entry) in self.entries.iter().enumerate() {
                if index != position {
                    let _ = kept.push_back(*entry);
                }
            }
            self.entries = kept;
        }
        if self.entries.is_full() {
            self.entries.pop_front();
        }
        let mut entry = Entry {
            tid,
            msg_id,
            len: payload.len() as u8,
            payload: [0; 1 + MAX_CACHED_RESULT],
        };
        entry.payload[..payload.len()].copy_from_slice(payload);
        let _ = self.entries.push_back(entry);
    }

    /// The cached `status` + result of `(tid, msg_id)`, if any.
    pub fn lookup(&mut self, tid: u16, msg_id: u16) -> Option<&[u8]> {
        let entry = self
            .entries
            .iter()
            .find(|entry| entry.tid == tid && entry.msg_id == msg_id)?;
        self.hits = self.hits.saturating_add(1);
        Some(&entry.payload[..usize::from(entry.len)])
    }

    /// Retries answered from the cache (DIAG queue group).
    pub const fn hits(&self) -> u32 {
        self.hits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retries_hit_the_latest_result_and_long_results_are_skipped() {
        let mut cache = RetryCache::new();
        cache.remember(7, 0x0201, &[0x00]);
        assert_eq!(cache.lookup(7, 0x0201), Some(&[0x00][..]));
        assert_eq!(cache.lookup(7, 0x0202), None, "keyed by msg_id too");
        cache.remember(7, 0x0201, &[0x03]);
        assert_eq!(cache.lookup(7, 0x0201), Some(&[0x03][..]));
        cache.remember(8, 0x0002, &[0; 40]);
        assert_eq!(cache.lookup(8, 0x0002), None);
        for tid in 100..100 + RETRY_CACHE_ENTRIES as u16 {
            cache.remember(tid, 0x0802, &[0, 1, 2, 3, 4]);
        }
        assert_eq!(cache.lookup(7, 0x0201), None, "oldest evicted");
        assert!(cache.lookup(100, 0x0802).is_some());
        cache.clear();
        assert_eq!(cache.lookup(100, 0x0802), None);
    }
}
