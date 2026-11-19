//! Background pre-erase of a two-sector journal's spare sector
//! (`journal-pre-erase`).
//!
//! A ping-pong journal erases its spare logical sector at the moment it
//! rolls over, inside the store that needs the room. On a flash whose
//! erase masks interrupts (ESP32 ROM driver) that makes the radio deaf for
//! the whole erase — 8 × ~30 ms for a 32 KiB Trust Center sector — in the
//! middle of a Zigbee exchange. With this helper the journal erases the
//! spare sector ahead of time, one erase unit per call, while the owner
//! knows the radio is quiet, and the rollover only erases what is left.
//!
//! Safety: the journal pre-erases only the sector that does *not* hold its
//! current, validated record, so that sector holds only older generations.
//! What was erased is remembered in RAM only and only until the journal
//! writes to that sector (its rollover) or loses track of its current
//! record; after a reboot the rollover erases the whole sector again, so a
//! power loss during a pre-erase can never leave a half-erased sector that
//! is later programmed without a full erase.
//!
//! Wear: the erase is the one the rollover would do, done earlier; only a
//! reboot between a pre-erase and the rollover repeats it (at most one extra
//! erase of the spare sector per boot).

use embedded_storage::nor_flash::NorFlash;

/// Pre-erase progress of one journal's spare sector.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct SpareErase {
    /// Logical sector (0/1) the progress refers to.
    sector: Option<usize>,
    /// Bytes from its start erased since boot and not written since.
    erased: u32,
}

impl SpareErase {
    pub(crate) const fn new() -> Self {
        Self {
            sector: None,
            erased: 0,
        }
    }

    /// Forget every pre-erase (the journal wrote the sector, or no longer
    /// knows which record is current).
    pub(crate) fn forget(&mut self) {
        *self = Self::new();
    }

    /// Whether all `size` bytes of `sector` are erased since boot.
    pub(crate) fn is_complete(&self, sector: usize, size: u32) -> bool {
        self.sector == Some(sector) && self.erased >= size
    }

    /// Erase the next erase unit of the spare `sector` (`start..start +
    /// size`). Returns whether it erased (`false`: already fully erased).
    pub(crate) fn step<S: NorFlash>(
        &mut self,
        storage: &mut S,
        sector: usize,
        start: u32,
        size: u32,
    ) -> Result<bool, S::Error> {
        if self.sector != Some(sector) {
            *self = Self {
                sector: Some(sector),
                erased: 0,
            };
        }
        if self.erased >= size {
            return Ok(false);
        }
        let unit = (S::ERASE_SIZE as u32).clamp(1, size - self.erased);
        let from = start + self.erased;
        storage.erase(from, from + unit)?;
        self.erased += unit;
        Ok(true)
    }

    /// Erase `sector` for a rollover, skipping the part pre-erased since
    /// boot. The progress is spent either way: the caller writes next.
    pub(crate) fn erase_for_rollover<S: NorFlash>(
        &mut self,
        storage: &mut S,
        sector: usize,
        start: u32,
        size: u32,
    ) -> Result<(), S::Error> {
        let done = if self.sector == Some(sector) {
            self.erased.min(size)
        } else {
            0
        };
        self.forget();
        if done < size {
            storage.erase(start + done, start + size)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    /// 2 × 8 KiB, 4 KiB erase units; counts erases per unit.
    struct Flash {
        bytes: [u8; 16 * 1024],
        erases: [u32; 4],
    }

    impl ErrorType for Flash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for Flash {
        const READ_SIZE: usize = 1;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let at = offset as usize;
            bytes.copy_from_slice(&self.bytes[at..at + bytes.len()]);
            Ok(())
        }
        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for Flash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = 4096;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            for unit in from / 4096..to / 4096 {
                self.erases[unit as usize] += 1;
                self.bytes[unit as usize * 4096..][..4096].fill(0xFF);
            }
            Ok(())
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let at = offset as usize;
            self.bytes[at..at + bytes.len()].copy_from_slice(bytes);
            Ok(())
        }
    }

    #[test]
    fn a_pre_erased_sector_is_not_erased_again_at_rollover() {
        let mut flash = Flash {
            bytes: [0; 16 * 1024],
            erases: [0; 4],
        };
        let mut spare = SpareErase::new();
        assert_eq!(spare.step(&mut flash, 1, 8192, 8192), Ok(true));
        assert_eq!(spare.step(&mut flash, 1, 8192, 8192), Ok(true));
        assert_eq!(spare.step(&mut flash, 1, 8192, 8192), Ok(false));
        assert_eq!(flash.erases, [0, 0, 1, 1]);
        spare.erase_for_rollover(&mut flash, 1, 8192, 8192).unwrap();
        assert_eq!(flash.erases, [0, 0, 1, 1], "no second erase");
        // Spent: the next rollover into it erases again.
        spare.erase_for_rollover(&mut flash, 1, 8192, 8192).unwrap();
        assert_eq!(flash.erases, [0, 0, 2, 2]);
    }

    #[test]
    fn a_partial_pre_erase_only_skips_its_prefix() {
        let mut flash = Flash {
            bytes: [0; 16 * 1024],
            erases: [0; 4],
        };
        let mut spare = SpareErase::new();
        spare.step(&mut flash, 0, 0, 8192).unwrap();
        spare.erase_for_rollover(&mut flash, 0, 0, 8192).unwrap();
        assert_eq!(flash.erases, [1, 1, 0, 0]);
        assert!(flash.bytes[..8192].iter().all(|byte| *byte == 0xFF));
    }

    #[test]
    fn progress_on_the_other_sector_or_a_forgotten_one_is_not_trusted() {
        let mut flash = Flash {
            bytes: [0; 16 * 1024],
            erases: [0; 4],
        };
        let mut spare = SpareErase::new();
        spare.step(&mut flash, 1, 8192, 8192).unwrap();
        spare.erase_for_rollover(&mut flash, 0, 0, 8192).unwrap();
        assert_eq!(flash.erases, [1, 1, 1, 0]);
        spare.step(&mut flash, 1, 8192, 8192).unwrap();
        spare.forget();
        spare.erase_for_rollover(&mut flash, 1, 8192, 8192).unwrap();
        assert_eq!(flash.erases, [1, 1, 3, 1]);
    }
}
