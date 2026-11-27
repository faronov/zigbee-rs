//! Bounded log ring for the ZRHP LOG endpoint (spec 7.10, decision 12).
//!
//! Records are UTF-8 text of at most [`MAX_LOG_TEXT`] bytes, truncated at a
//! character boundary, so every `EVT_LOG_RECORD` is valid UTF-8. When the
//! ring is full the oldest record is dropped and counted, except the first
//! [`BOOT_HOLD`] records since reset (the boot log: radio crystal tuning,
//! EUI-64 source, NV state), which stay until the host has read them; the
//! dispatcher reports drops with `EVT_LOG_DROPPED`. Logs never take credit
//! or window slots from other endpoints: they are only sent while the LOG
//! endpoint has credit (a host may grant it none, or grant it later: the
//! ring waits for it).
//!
//! The ring is filled from thread context through [`LogRing::push`]
//! (a `log::Log` implementation on the platform side); interrupt handlers
//! must not log.

use core::cell::RefCell;
use core::fmt::Write;

use critical_section::Mutex;
use heapless::{Deque, String};

pub const MAX_LOG_TEXT: usize = 200;

/// Records kept from reset until they are read (at most half the ring).
pub const BOOT_HOLD: usize = 4;

/// ZRHP log levels: 1 error … 5 trace (0 = off).
pub const fn level_code(level: log::Level) -> u8 {
    match level {
        log::Level::Error => 1,
        log::Level::Warn => 2,
        log::Level::Info => 3,
        log::Level::Debug => 4,
        log::Level::Trace => 5,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LogRecord {
    pub time_ms: u32,
    pub level: u8,
    pub seq: u16,
    pub text: String<MAX_LOG_TEXT>,
}

struct Inner<const N: usize> {
    records: Deque<LogRecord, N>,
    next_seq: u16,
    level: u8,
    dropped: u32,
    first_dropped_seq: u16,
    /// Records at the front kept from reset until read.
    held: usize,
}

/// `N` records of up to 200 bytes each.
pub struct LogRing<const N: usize> {
    inner: Mutex<RefCell<Inner<N>>>,
}

impl<const N: usize> Default for LogRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Truncating UTF-8 writer.
struct Bounded<'a>(&'a mut String<MAX_LOG_TEXT>);

impl Write for Bounded<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for ch in s.chars() {
            if self.0.push(ch).is_err() {
                return Err(core::fmt::Error);
            }
        }
        Ok(())
    }
}

impl<const N: usize> LogRing<N> {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(Inner {
                records: Deque::new(),
                next_seq: 0,
                level: 3,
                dropped: 0,
                first_dropped_seq: 0,
                held: if BOOT_HOLD < N / 2 { BOOT_HOLD } else { N / 2 },
            })),
        }
    }

    /// Records at or below this ZRHP level are kept (0 = off).
    pub fn set_level(&self, level: u8) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).level = level.min(5));
    }

    pub fn level(&self) -> u8 {
        critical_section::with(|cs| self.inner.borrow_ref(cs).level)
    }

    /// Append a record (formatted text, truncated to 200 bytes).
    pub fn push(&self, time_ms: u32, level: u8, args: core::fmt::Arguments<'_>) {
        if level == 0 || level > self.level() {
            return;
        }
        let mut text = String::new();
        let _ = Bounded(&mut text).write_fmt(args);
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow_ref_mut(cs);
            let seq = inner.next_seq;
            inner.next_seq = seq.wrapping_add(1);
            if inner.records.is_full()
                && let Some(old) = inner.evict()
            {
                if inner.dropped == 0 {
                    inner.first_dropped_seq = old.seq;
                }
                inner.dropped = inner.dropped.saturating_add(1);
            }
            let _ = inner.records.push_back(LogRecord {
                time_ms,
                level,
                seq,
                text,
            });
        });
    }

    /// Oldest record, if any.
    pub fn pop(&self) -> Option<LogRecord> {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow_ref_mut(cs);
            let record = inner.records.pop_front()?;
            inner.held = inner.held.saturating_sub(1);
            Some(record)
        })
    }

    /// Put a record back at the front (it could not be queued).
    pub fn unpop(&self, record: LogRecord) {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow_ref_mut(cs);
            if inner.records.push_front(record).is_err() {
                inner.dropped = inner.dropped.saturating_add(1);
            }
        });
    }

    pub fn is_empty(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).records.is_empty())
    }

    /// Dropped records since the last call, with the first dropped `seq`.
    pub fn take_dropped(&self) -> Option<(u32, u16)> {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow_ref_mut(cs);
            (inner.dropped != 0).then(|| {
                let report = (inner.dropped, inner.first_dropped_seq);
                inner.dropped = 0;
                report
            })
        })
    }
}

impl<const N: usize> Inner<N> {
    /// Remove the oldest record not held from reset: it is moved to the
    /// front past the held ones (at most [`BOOT_HOLD`] swaps), which keep
    /// their order.
    fn evict(&mut self) -> Option<LogRecord> {
        let at = self.held.min(self.records.len().saturating_sub(1));
        for index in (1..=at).rev() {
            self.records.swap(index, index - 1);
        }
        self.records.pop_front()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn truncates_at_a_char_boundary_and_counts_overflow() {
        let ring = LogRing::<2>::new();
        let long = "é".repeat(150); // 300 bytes
        ring.push(1, 3, format_args!("{long}"));
        let record = ring.pop().unwrap();
        assert!(record.text.len() <= MAX_LOG_TEXT);
        assert!(core::str::from_utf8(record.text.as_bytes()).is_ok());
        assert_eq!(record.text.len(), 200);

        // A 2-record ring holds one boot record (the first was read).
        for n in 0..5 {
            ring.push(n, 2, format_args!("r{n}"));
        }
        assert_eq!(ring.take_dropped(), Some((3, 1)));
        assert_eq!(ring.pop().unwrap().text.as_str(), "r3");
        assert_eq!(ring.take_dropped(), None);
    }

    #[test]
    fn the_boot_records_wait_for_the_host() {
        let ring = LogRing::<8>::new();
        for n in 0..20 {
            ring.push(n, 3, format_args!("r{n}"));
        }
        // The first four stay in order; the newest four follow.
        assert_eq!(ring.take_dropped(), Some((12, 4)));
        let read: std::vec::Vec<_> = core::iter::from_fn(|| ring.pop())
            .map(|record| record.text.as_str().to_owned())
            .collect();
        assert_eq!(read, ["r0", "r1", "r2", "r3", "r16", "r17", "r18", "r19"]);

        // Once read, the ring keeps only the newest records.
        for n in 20..30 {
            ring.push(n, 3, format_args!("r{n}"));
        }
        assert_eq!(ring.take_dropped(), Some((2, 20)));
        assert_eq!(ring.pop().unwrap().text.as_str(), "r22");
    }

    #[test]
    fn level_filter_and_off() {
        let ring = LogRing::<4>::new();
        ring.set_level(2);
        ring.push(0, 3, format_args!("info"));
        ring.push(0, 1, format_args!("error"));
        assert_eq!(ring.pop().unwrap().text.as_str(), "error");
        ring.set_level(0);
        ring.push(0, 1, format_args!("off"));
        assert!(ring.is_empty());
    }
}
