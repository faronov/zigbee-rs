//! Boot status records: the trial (confirm) state of an installed image
//! and, with `swap-move`, the progress of a slot exchange.
//!
//! # Record encoding
//!
//! A record is a 16-bit code followed by its bitwise complement
//! (`[lo, hi, !lo, !hi]`), programmed as one write unit of
//! `max(4, F::WRITE_SIZE)` bytes (the rest of the unit `0xFF`). Records are
//! appended in order; the first fully erased unit ends the log.
//!
//! Power loss while programming clears only a subset of the bits being
//! cleared, and an interrupted erase only sets bits. Either way a damaged
//! record fails its complement check and can never turn into a *different*
//! valid record. A damaged ("torn") unit is skipped and the next record is
//! appended after it; after the install record it counts as a boot attempt,
//! so a trial always ends.
//!
//! # Overwrite mode
//!
//! The log sits at the start of the primary slot's trailer sector (the last
//! sector, never part of an image). The loader erases that sector when it
//! starts an install and appends [`Record::Done`] once the copy validates,
//! before it drops the request. Without that record the primary image is
//! *permanent*: factory images and images installed by older loaders are
//! never put on trial.
//!
//! # Trial
//!
//! After [`Record::Done`] the application appends [`Record::Confirmed`]
//! once it has proved itself, and the loader appends [`Record::Attempt`]
//! before every boot of the unconfirmed image, up to the configured number
//! of trial boots.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

use crate::boot::{BootError, Layout};

/// Bytes kept free at the end of the overwrite-mode trailer sector, where
/// MCUboot-format tools put their own trailer (magic, flags).
const TRAILER_RESERVE: u32 = 128;
/// Largest supported flash write unit.
pub(crate) const MAX_UNIT: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Record {
    /// Exchange step `index` finished (swap-move).
    #[cfg(feature = "swap-move")]
    Step(u16),
    /// The install (or revert) finished; in overwrite mode the image is on
    /// trial from here.
    Done,
    /// The install's request magic was dropped (swap-move).
    #[cfg(feature = "swap-move")]
    Cleared,
    /// The application confirmed the image.
    Confirmed,
    /// The loader started the unconfirmed image once.
    Attempt,
}

const KIND_SHIFT: u16 = 13;
const VALUE_MASK: u16 = (1 << KIND_SHIFT) - 1;
/// Largest step index a record holds.
#[cfg(feature = "swap-move")]
pub(crate) const MAX_STEP: u16 = VALUE_MASK;

impl Record {
    // Kinds 0 and 7 are unused so that no code is 0x0000 or 0xFFFF.
    const fn code(self) -> u16 {
        match self {
            #[cfg(feature = "swap-move")]
            Self::Step(index) => (2 << KIND_SHIFT) | (index & VALUE_MASK),
            Self::Done => 3 << KIND_SHIFT,
            #[cfg(feature = "swap-move")]
            Self::Cleared => 4 << KIND_SHIFT,
            Self::Confirmed => 5 << KIND_SHIFT,
            Self::Attempt => 6 << KIND_SHIFT,
        }
    }

    fn decode(code: u16) -> Option<Self> {
        let value = code & VALUE_MASK;
        match (code >> KIND_SHIFT, value) {
            #[cfg(feature = "swap-move")]
            (2, index) => Some(Self::Step(index)),
            (3, 0) => Some(Self::Done),
            #[cfg(feature = "swap-move")]
            (4, 0) => Some(Self::Cleared),
            (5, 0) => Some(Self::Confirmed),
            (6, 0) => Some(Self::Attempt),
            _ => None,
        }
    }

    fn encode(self) -> [u8; 4] {
        let [lo, hi] = self.code().to_le_bytes();
        [lo, hi, !lo, !hi]
    }
}

/// Write unit of a record on flash `F`.
pub(crate) const fn unit<F: NorFlash>() -> u32 {
    assert!(F::WRITE_SIZE <= MAX_UNIT && MAX_UNIT.is_multiple_of(F::WRITE_SIZE));
    if F::WRITE_SIZE < 4 {
        4
    } else {
        F::WRITE_SIZE as u32
    }
}

/// What a log says.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct LogState {
    /// Highest finished exchange step plus one (swap-move).
    #[cfg(feature = "swap-move")]
    pub steps: u16,
    pub done: bool,
    #[cfg(feature = "swap-move")]
    pub cleared: bool,
    pub confirmed: bool,
    /// Attempts (and torn units) after [`Record::Done`].
    pub attempts: u16,
    /// Address of the first erased unit, or the end of the log when full.
    pub next: u32,
}

/// An append-only record area `[start, end)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Log {
    pub start: u32,
    pub end: u32,
}

impl Log {
    /// The overwrite-mode trial log in the primary's trailer sector.
    pub(crate) const fn trailer(layout: &Layout) -> Self {
        let start = layout.trailer_sector(layout.primary);
        Self {
            start,
            end: start + layout.sector_size - TRAILER_RESERVE,
        }
    }

    pub(crate) fn read<F: ReadNorFlash + NorFlash>(
        &self,
        flash: &mut F,
    ) -> Result<LogState, F::Error> {
        let unit = unit::<F>();
        let mut state = LogState {
            next: self.end,
            ..LogState::default()
        };
        let mut address = self.start;
        while address + unit <= self.end {
            let mut bytes = [0u8; 4];
            flash.read(address, &mut bytes)?;
            if bytes == [0xFF; 4] {
                state.next = address;
                break;
            }
            let record = if bytes[2] == !bytes[0] && bytes[3] == !bytes[1] {
                Record::decode(u16::from_le_bytes([bytes[0], bytes[1]]))
            } else {
                None
            };
            match record {
                #[cfg(feature = "swap-move")]
                Some(Record::Step(index)) if !state.done => {
                    state.steps = state.steps.max(index + 1);
                }
                Some(Record::Done) => state.done = true,
                #[cfg(feature = "swap-move")]
                Some(Record::Cleared) if state.done => state.cleared = true,
                Some(Record::Confirmed) if state.done => state.confirmed = true,
                // Torn units and attempts after the install count as boots.
                Some(Record::Attempt) | None if state.done => {
                    state.attempts = state.attempts.saturating_add(1);
                }
                _ => {}
            }
            address += unit;
        }
        Ok(state)
    }

    /// Append `record` at `state.next` and account for it.
    pub(crate) fn append<F: NorFlash>(
        &self,
        flash: &mut F,
        state: &mut LogState,
        record: Record,
    ) -> Result<(), BootError> {
        let unit = unit::<F>();
        if state.next + unit > self.end {
            return Err(BootError::StatusFull);
        }
        let mut bytes = [0xFFu8; MAX_UNIT];
        bytes[..4].copy_from_slice(&record.encode());
        flash
            .write(state.next, &bytes[..unit as usize])
            .map_err(|_| BootError::Flash)?;
        state.next += unit;
        match record {
            #[cfg(feature = "swap-move")]
            Record::Step(index) => state.steps = state.steps.max(index + 1),
            Record::Done => state.done = true,
            #[cfg(feature = "swap-move")]
            Record::Cleared => state.cleared = true,
            Record::Confirmed => state.confirmed = true,
            Record::Attempt => state.attempts = state.attempts.saturating_add(1),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_avoid_erased_and_zero_and_round_trip() {
        #[cfg_attr(not(feature = "swap-move"), allow(unused_mut))]
        let mut records = std::vec![Record::Done, Record::Confirmed, Record::Attempt];
        #[cfg(feature = "swap-move")]
        records.extend([Record::Step(0), Record::Step(MAX_STEP), Record::Cleared]);
        for record in records {
            let code = record.code();
            assert!(code != 0 && code != 0xFFFF);
            assert_eq!(Record::decode(code), Some(record));
        }
    }

    /// No partial programming of a record (a subset of its zero bits) and
    /// no partial erase of it (a superset of its one bits) yields another
    /// valid record.
    #[test]
    fn damage_never_forges_a_different_record() {
        #[cfg_attr(not(feature = "swap-move"), allow(unused_mut))]
        let mut records = std::vec![Record::Done, Record::Confirmed, Record::Attempt];
        #[cfg(feature = "swap-move")]
        records.extend([Record::Step(5), Record::Cleared]);
        for record in records {
            let good = u32::from_le_bytes(record.encode());
            for seed in 0..20_000u32 {
                let noise = seed.wrapping_mul(0x9E37_79B9).rotate_left(seed % 32);
                // Torn program: some zero bits stay one.
                let torn = good | (noise & !good);
                // Torn erase: some zero bits become one.
                for damaged in [torn, good | noise] {
                    let bytes = damaged.to_le_bytes();
                    let valid = bytes[2] == !bytes[0] && bytes[3] == !bytes[1];
                    if valid {
                        assert_eq!(
                            Record::decode(u16::from_le_bytes([bytes[0], bytes[1]])),
                            Some(record)
                        );
                    }
                }
            }
        }
    }
}
