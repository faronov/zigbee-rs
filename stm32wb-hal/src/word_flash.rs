//! One logical 32-bit word per physical 64-bit flash double word.
//!
//! STM32WB main flash is programmed in 64-bit double words, each carrying
//! its own ECC. A double word may be programmed **once** after an erase:
//! a second program, even one that only clears more bits, fails with
//! `PROGERR`. The zigbee-runtime journals (security state, child table, APS
//! tables) are written with a 4-byte granularity: a record prefix first, its
//! 4-byte commit marker later, and replay entries one after another inside a
//! 128-byte slot. Two of those writes can land in the same double word.
//!
//! [`WordPerDoubleWord`] therefore presents a NOR flash with
//! `WRITE_SIZE = 4` and maps logical word `n` onto physical double word `n`:
//! the word is stored in the lower half and the upper half stays `0xFF`.
//! Every double word is programmed at most once between erases, at the cost
//! of half the physical capacity. One logical erase sector (2 KiB) is one
//! physical page (4 KiB).
//!
//! A double word left half-programmed by a power loss has an uncorrectable
//! ECC error. Reads report that per double word; the adapter returns the
//! logical word as `0x00000000`, which no journal accepts as erased (so it
//! never tries to program it again) and which fails every record CRC.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

/// Physical flash page (erase unit) in bytes.
pub const PHYSICAL_PAGE_SIZE: usize = 4096;
/// Physical program unit in bytes.
pub const DOUBLE_WORD: usize = 8;
/// Logical program unit presented to the journals.
pub const LOGICAL_WORD: usize = 4;
/// Logical erase unit: one physical page holds this many logical bytes.
pub const LOGICAL_SECTOR_SIZE: usize = PHYSICAL_PAGE_SIZE / 2;

/// Result of reading one physical double word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DoubleWordRead {
    Data([u8; DOUBLE_WORD]),
    /// Uncorrectable ECC error (torn program or erase).
    EccError,
}

/// Physical access to a page-aligned flash window, in window-relative
/// offsets. Implemented by the chip flash driver and by host test doubles.
pub trait DoubleWordFlash {
    type Error: core::fmt::Debug;

    /// Window size in bytes, a multiple of [`PHYSICAL_PAGE_SIZE`].
    fn physical_capacity(&self) -> usize;
    /// Read the double word at `offset` (8-byte aligned).
    fn read_double_word(&mut self, offset: u32) -> Result<DoubleWordRead, Self::Error>;
    /// Program the erased double word at `offset` (8-byte aligned).
    fn program_double_word(
        &mut self,
        offset: u32,
        value: [u8; DOUBLE_WORD],
    ) -> Result<(), Self::Error>;
    /// Erase the page at `offset` (page aligned).
    fn erase_page(&mut self, offset: u32) -> Result<(), Self::Error>;
}

/// Error of the logical view.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordFlashError<E> {
    OutOfBounds,
    NotAligned,
    /// The target word already holds different data.
    NotErased,
    Backend(E),
}

impl<E: core::fmt::Debug> NorFlashError for WordFlashError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::NotErased | Self::Backend(_) => NorFlashErrorKind::Other,
        }
    }
}

/// NOR view with 4-byte words over a double-word-programmed flash window.
pub struct WordPerDoubleWord<F> {
    flash: F,
}

const ERASED_WORD: [u8; LOGICAL_WORD] = [0xFF; LOGICAL_WORD];
const ECC_WORD: [u8; LOGICAL_WORD] = [0x00; LOGICAL_WORD];

impl<F: DoubleWordFlash> WordPerDoubleWord<F> {
    pub fn new(flash: F) -> Self {
        Self { flash }
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    fn logical_capacity(&self) -> usize {
        self.flash.physical_capacity() / 2
    }

    fn check_range(&self, offset: u32, length: usize) -> Result<(), WordFlashError<F::Error>> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= self.logical_capacity())
            .map(|_| ())
            .ok_or(WordFlashError::OutOfBounds)
    }

    fn physical(word_offset: u32) -> u32 {
        word_offset * 2
    }

    fn read_word(
        &mut self,
        word_offset: u32,
    ) -> Result<[u8; LOGICAL_WORD], WordFlashError<F::Error>> {
        match self
            .flash
            .read_double_word(Self::physical(word_offset))
            .map_err(WordFlashError::Backend)?
        {
            DoubleWordRead::Data(data) => Ok([data[0], data[1], data[2], data[3]]),
            DoubleWordRead::EccError => Ok(ECC_WORD),
        }
    }
}

impl<F: DoubleWordFlash> ErrorType for WordPerDoubleWord<F> {
    type Error = WordFlashError<F::Error>;
}

impl<F: DoubleWordFlash> ReadNorFlash for WordPerDoubleWord<F> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.check_range(offset, bytes.len())?;
        let mut position = 0usize;
        while position < bytes.len() {
            let logical = offset as usize + position;
            let word_offset = (logical & !(LOGICAL_WORD - 1)) as u32;
            let word = self.read_word(word_offset)?;
            let start = logical - word_offset as usize;
            let take = (LOGICAL_WORD - start).min(bytes.len() - position);
            bytes[position..position + take].copy_from_slice(&word[start..start + take]);
            position += take;
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.logical_capacity()
    }
}

impl<F: DoubleWordFlash> NorFlash for WordPerDoubleWord<F> {
    const WRITE_SIZE: usize = LOGICAL_WORD;
    const ERASE_SIZE: usize = LOGICAL_SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from > to {
            return Err(WordFlashError::OutOfBounds);
        }
        self.check_range(from, (to - from) as usize)?;
        if !(from as usize).is_multiple_of(LOGICAL_SECTOR_SIZE)
            || !(to as usize).is_multiple_of(LOGICAL_SECTOR_SIZE)
        {
            return Err(WordFlashError::NotAligned);
        }
        let mut sector = from;
        while sector < to {
            self.flash
                .erase_page(Self::physical(sector))
                .map_err(WordFlashError::Backend)?;
            sector += LOGICAL_SECTOR_SIZE as u32;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.check_range(offset, bytes.len())?;
        if !(offset as usize).is_multiple_of(LOGICAL_WORD)
            || !bytes.len().is_multiple_of(LOGICAL_WORD)
        {
            return Err(WordFlashError::NotAligned);
        }
        for (index, chunk) in bytes.chunks_exact(LOGICAL_WORD).enumerate() {
            let word_offset = offset + (index * LOGICAL_WORD) as u32;
            let word = [chunk[0], chunk[1], chunk[2], chunk[3]];
            if word == ERASED_WORD {
                // Writing all-ones is a NOR no-op. Leaving the double word
                // untouched keeps an erased one programmable.
                continue;
            }
            let physical = Self::physical(word_offset);
            match self
                .flash
                .read_double_word(physical)
                .map_err(WordFlashError::Backend)?
            {
                DoubleWordRead::Data(current) if current == [0xFF; DOUBLE_WORD] => {
                    let mut value = [0xFF; DOUBLE_WORD];
                    value[..LOGICAL_WORD].copy_from_slice(&word);
                    self.flash
                        .program_double_word(physical, value)
                        .map_err(WordFlashError::Backend)?;
                }
                // Rewriting identical data is a NOR no-op as well.
                DoubleWordRead::Data(current) if current[..LOGICAL_WORD] == word => {}
                _ => return Err(WordFlashError::NotErased),
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec;
    use std::vec::Vec;

    use super::*;

    struct SimFlash {
        bytes: Vec<u8>,
        programmed: Vec<bool>,
        ecc: Vec<bool>,
        programs: usize,
        erases: usize,
    }

    impl SimFlash {
        fn new(pages: usize) -> Self {
            let size = pages * PHYSICAL_PAGE_SIZE;
            Self {
                bytes: vec![0xFF; size],
                programmed: vec![false; size / DOUBLE_WORD],
                ecc: vec![false; size / DOUBLE_WORD],
                programs: 0,
                erases: 0,
            }
        }
    }

    #[derive(Debug, PartialEq, Eq)]
    enum SimError {
        DoubleProgram,
        Unaligned,
    }

    impl DoubleWordFlash for SimFlash {
        type Error = SimError;

        fn physical_capacity(&self) -> usize {
            self.bytes.len()
        }

        fn read_double_word(&mut self, offset: u32) -> Result<DoubleWordRead, SimError> {
            if !offset.is_multiple_of(DOUBLE_WORD as u32) {
                return Err(SimError::Unaligned);
            }
            let index = offset as usize / DOUBLE_WORD;
            if self.ecc[index] {
                return Ok(DoubleWordRead::EccError);
            }
            let start = offset as usize;
            let mut data = [0; DOUBLE_WORD];
            data.copy_from_slice(&self.bytes[start..start + DOUBLE_WORD]);
            Ok(DoubleWordRead::Data(data))
        }

        fn program_double_word(&mut self, offset: u32, value: [u8; 8]) -> Result<(), SimError> {
            if !offset.is_multiple_of(DOUBLE_WORD as u32) {
                return Err(SimError::Unaligned);
            }
            let index = offset as usize / DOUBLE_WORD;
            if self.programmed[index] {
                return Err(SimError::DoubleProgram);
            }
            self.programmed[index] = true;
            self.programs += 1;
            let start = offset as usize;
            self.bytes[start..start + DOUBLE_WORD].copy_from_slice(&value);
            Ok(())
        }

        fn erase_page(&mut self, offset: u32) -> Result<(), SimError> {
            if !offset.is_multiple_of(PHYSICAL_PAGE_SIZE as u32) {
                return Err(SimError::Unaligned);
            }
            let start = offset as usize;
            self.bytes[start..start + PHYSICAL_PAGE_SIZE].fill(0xFF);
            let first = start / DOUBLE_WORD;
            let count = PHYSICAL_PAGE_SIZE / DOUBLE_WORD;
            self.programmed[first..first + count].fill(false);
            self.ecc[first..first + count].fill(false);
            self.erases += 1;
            Ok(())
        }
    }

    #[test]
    fn geometry_halves_capacity() {
        let flash = WordPerDoubleWord::new(SimFlash::new(4));
        assert_eq!(flash.capacity(), 2 * PHYSICAL_PAGE_SIZE);
        assert_eq!(<WordPerDoubleWord<SimFlash> as NorFlash>::WRITE_SIZE, 4);
        assert_eq!(<WordPerDoubleWord<SimFlash> as NorFlash>::ERASE_SIZE, 2048);
    }

    #[test]
    fn adjacent_word_writes_never_program_a_double_word_twice() {
        let mut flash = WordPerDoubleWord::new(SimFlash::new(1));
        // Record prefix, then its commit marker in the very next word: on a
        // raw double-word flash these would share one double word.
        flash.write(0, b"ZBSS").unwrap();
        flash.write(4, b"CMIT").unwrap();
        let mut back = [0; 8];
        flash.read(0, &mut back).unwrap();
        assert_eq!(&back, b"ZBSSCMIT");
        let raw = flash.into_inner();
        assert_eq!(&raw.bytes[0..8], b"ZBSS\xFF\xFF\xFF\xFF");
        assert_eq!(&raw.bytes[8..16], b"CMIT\xFF\xFF\xFF\xFF");
        assert_eq!(raw.programs, 2);
    }

    #[test]
    fn unaligned_reads_span_words() {
        let mut flash = WordPerDoubleWord::new(SimFlash::new(1));
        flash.write(8, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        let mut back = [0; 5];
        flash.read(10, &mut back).unwrap();
        assert_eq!(back, [3, 4, 5, 6, 7]);
        let mut erased = [0; 3];
        flash.read(17, &mut erased).unwrap();
        assert_eq!(erased, [0xFF; 3]);
    }

    #[test]
    fn rewriting_identical_or_erased_data_is_a_no_op() {
        let mut flash = WordPerDoubleWord::new(SimFlash::new(1));
        flash.write(0, &[0xFF; 8]).unwrap();
        flash.write(0, &[9, 9, 9, 9]).unwrap();
        flash.write(0, &[9, 9, 9, 9]).unwrap();
        assert_eq!(
            flash.write(0, &[8, 8, 8, 8]),
            Err(WordFlashError::NotErased)
        );
        assert_eq!(flash.into_inner().programs, 1);
    }

    #[test]
    fn misaligned_writes_and_erases_are_rejected() {
        let mut flash = WordPerDoubleWord::new(SimFlash::new(2));
        assert_eq!(flash.write(2, &[0; 4]), Err(WordFlashError::NotAligned));
        assert_eq!(flash.write(0, &[0; 3]), Err(WordFlashError::NotAligned));
        assert_eq!(flash.erase(0, 1024), Err(WordFlashError::NotAligned));
        assert_eq!(flash.erase(0, 8192), Err(WordFlashError::OutOfBounds));
        assert_eq!(flash.write(4096, &[0; 4]), Err(WordFlashError::OutOfBounds));
    }

    #[test]
    fn logical_sector_erase_maps_to_one_physical_page() {
        let mut flash = WordPerDoubleWord::new(SimFlash::new(2));
        flash.write(0, &[1; 4]).unwrap();
        flash.write(2048, &[2; 4]).unwrap();
        flash.erase(2048, 4096).unwrap();
        let mut first = [0; 4];
        let mut second = [0; 4];
        flash.read(0, &mut first).unwrap();
        flash.read(2048, &mut second).unwrap();
        assert_eq!(first, [1; 4]);
        assert_eq!(second, [0xFF; 4]);
        // The erased word is programmable again.
        flash.write(2048, &[3; 4]).unwrap();
        assert_eq!(flash.into_inner().erases, 1);
    }

    #[test]
    fn torn_double_word_reads_as_zero_and_is_not_reprogrammed() {
        let mut sim = SimFlash::new(1);
        sim.ecc[1] = true;
        sim.programmed[1] = true;
        let mut flash = WordPerDoubleWord::new(sim);
        let mut back = [0xAA; 4];
        flash.read(4, &mut back).unwrap();
        assert_eq!(back, [0; 4]);
        assert_eq!(flash.write(4, b"CMIT"), Err(WordFlashError::NotErased));
    }
}
