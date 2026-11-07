//! Host-only NOR flash simulator with power-cut injection.
//!
//! [`SimFlash`] models a NOR device: erase sets whole `ERASE`-byte sectors
//! to `0xFF`, programming can only clear bits and must be aligned to
//! `WRITE` bytes. Every erase and program counts as one operation; with
//! [`SimFlash::cut_at`] the chosen operation is interrupted the way a power
//! loss would leave it ([`Cut`]) and every later operation fails until
//! [`SimFlash::power_cycle`].
//!
//! Tests replay a scenario once without a cut to count its operations, then
//! once per operation index and [`Cut`] mode, reboot, and check the
//! invariants (a valid image still boots, the request is resolved,
//! protected regions are untouched).

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

/// What the interrupted operation leaves behind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cut {
    /// Nothing: power failed before the operation started.
    Before,
    /// The first half of the range, rounded up to whole erase sectors or
    /// write units, was erased or programmed (so a one-sector erase
    /// completes, but power fails before it is acknowledged).
    Half,
    /// Every byte in the range is torn: an interrupted erase sets an
    /// arbitrary subset of bits, an interrupted program clears an arbitrary
    /// subset of the bits it was clearing.
    Torn,
}

impl Cut {
    pub const ALL: [Cut; 3] = [Cut::Before, Cut::Half, Cut::Torn];
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimError {
    /// Power was cut during or before this operation.
    PowerLoss,
    /// Offset or length not aligned to the read, write or erase unit.
    Unaligned,
    OutOfBounds,
    /// Programming a write unit that was not erased (only with
    /// [`SimFlash::strict`]).
    NotErased,
}

impl NorFlashError for SimError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Unaligned => NorFlashErrorKind::NotAligned,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::PowerLoss | Self::NotErased => NorFlashErrorKind::Other,
        }
    }
}

/// Simulated NOR flash starting at address 0.
#[derive(Debug, Clone)]
pub struct SimFlash<const ERASE: usize, const WRITE: usize> {
    data: Vec<u8>,
    /// Index of the operation to interrupt, and how.
    cut: Option<(usize, Cut)>,
    powered: bool,
    operations: usize,
    /// Refuse to program a write unit that is not fully erased (flash with
    /// ECC or a program-once-per-erase rule).
    strict: bool,
    /// Erase count per sector.
    wear: Vec<u32>,
    seed: u32,
}

impl<const ERASE: usize, const WRITE: usize> SimFlash<ERASE, WRITE> {
    /// An erased device of `size` bytes (a multiple of `ERASE`).
    pub fn new(size: usize) -> Self {
        assert!(size.is_multiple_of(ERASE) && ERASE.is_multiple_of(WRITE));
        Self {
            data: vec![0xFF; size],
            cut: None,
            powered: true,
            operations: 0,
            strict: false,
            wear: vec![0; size / ERASE],
            seed: 0x2545_F491,
        }
    }

    /// Require every programmed write unit to be erased first.
    pub fn strict(mut self) -> Self {
        self.strict = true;
        self
    }

    /// Place `bytes` at `address` without counting an operation.
    pub fn load(&mut self, address: u32, bytes: &[u8]) {
        let start = address as usize;
        self.data[start..start + bytes.len()].copy_from_slice(bytes);
    }

    pub fn bytes(&self) -> &[u8] {
        &self.data
    }

    pub fn range(&self, address: u32, len: usize) -> &[u8] {
        &self.data[address as usize..address as usize + len]
    }

    /// Interrupt operation number `index` (0-based, counted from now) as
    /// `cut` describes; it and every later operation fail.
    pub fn cut_at(&mut self, index: usize, cut: Cut) {
        self.cut = Some((self.operations + index, cut));
    }

    /// Restore power and clear any pending cut.
    pub fn power_cycle(&mut self) {
        self.cut = None;
        self.powered = true;
    }

    /// Whether power has been cut since the last [`SimFlash::power_cycle`].
    pub fn lost_power(&self) -> bool {
        !self.powered
    }

    /// Erase and program operations so far.
    pub fn operations(&self) -> usize {
        self.operations
    }

    /// Highest erase count of any sector.
    pub fn max_wear(&self) -> u32 {
        self.wear.iter().copied().max().unwrap_or(0)
    }

    fn next_random(&mut self) -> u8 {
        // xorshift32: deterministic torn patterns.
        let mut x = self.seed;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.seed = x;
        x as u8
    }

    /// Count an operation; `Some(cut)` if it is the interrupted one.
    fn begin(&mut self) -> Result<Option<Cut>, SimError> {
        if !self.powered {
            return Err(SimError::PowerLoss);
        }
        let index = self.operations;
        self.operations += 1;
        match self.cut {
            Some((at, cut)) if at == index => {
                self.powered = false;
                Ok(Some(cut))
            }
            _ => Ok(None),
        }
    }

    fn check(&self, offset: u32, len: usize, unit: usize) -> Result<(), SimError> {
        let start = offset as usize;
        if !start.is_multiple_of(unit) || !len.is_multiple_of(unit) {
            return Err(SimError::Unaligned);
        }
        if start
            .checked_add(len)
            .is_none_or(|end| end > self.data.len())
        {
            return Err(SimError::OutOfBounds);
        }
        Ok(())
    }
}

impl<const ERASE: usize, const WRITE: usize> ErrorType for SimFlash<ERASE, WRITE> {
    type Error = SimError;
}

impl<const ERASE: usize, const WRITE: usize> ReadNorFlash for SimFlash<ERASE, WRITE> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), SimError> {
        self.check(offset, bytes.len(), 1)?;
        let start = offset as usize;
        bytes.copy_from_slice(&self.data[start..start + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.data.len()
    }
}

impl<const ERASE: usize, const WRITE: usize> NorFlash for SimFlash<ERASE, WRITE> {
    const WRITE_SIZE: usize = WRITE;
    const ERASE_SIZE: usize = ERASE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), SimError> {
        if to < from {
            return Err(SimError::OutOfBounds);
        }
        self.check(from, (to - from) as usize, ERASE)?;
        let cut = self.begin()?;
        let (start, end) = (from as usize, to as usize);
        let applied = match cut {
            None => end,
            Some(Cut::Before) => start,
            Some(Cut::Half) => start + ((end - start) / 2).next_multiple_of(ERASE),
            Some(Cut::Torn) => {
                for index in start..end {
                    self.data[index] |= self.next_random();
                }
                start
            }
        };
        self.data[start..applied].fill(0xFF);
        for sector in start / ERASE..applied.div_ceil(ERASE) {
            self.wear[sector] += 1;
        }
        if cut.is_some() {
            Err(SimError::PowerLoss)
        } else {
            Ok(())
        }
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), SimError> {
        self.check(offset, bytes.len(), WRITE)?;
        let start = offset as usize;
        if self.strict {
            for (unit, new) in self.data[start..start + bytes.len()]
                .chunks(WRITE)
                .zip(bytes.chunks(WRITE))
            {
                let programs = new.iter().any(|byte| *byte != 0xFF);
                if programs && unit.iter().any(|byte| *byte != 0xFF) {
                    return Err(SimError::NotErased);
                }
            }
        }
        let cut = self.begin()?;
        let applied = match cut {
            None => bytes.len(),
            Some(Cut::Before) => 0,
            Some(Cut::Half) => (bytes.len() / 2).next_multiple_of(WRITE),
            Some(Cut::Torn) => {
                for (index, byte) in bytes.iter().enumerate() {
                    let noise = self.next_random();
                    self.data[start + index] &= *byte | noise;
                }
                0
            }
        };
        for (cell, byte) in self.data[start..start + applied].iter_mut().zip(bytes) {
            *cell &= *byte;
        }
        if cut.is_some() {
            Err(SimError::PowerLoss)
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Flash = SimFlash<0x100, 4>;

    #[test]
    fn nor_semantics() {
        let mut flash = Flash::new(0x400);
        flash.write(0, &[0x0F, 0xF0, 0xFF, 0x00]).unwrap();
        flash.write(0, &[0xF0, 0xFF, 0xFF, 0xFF]).unwrap();
        assert_eq!(flash.range(0, 4), [0x00, 0xF0, 0xFF, 0x00]);
        assert_eq!(flash.write(1, &[0; 4]), Err(SimError::Unaligned));
        assert_eq!(flash.erase(0, 0x80), Err(SimError::Unaligned));
        assert_eq!(flash.erase(0, 0x500), Err(SimError::OutOfBounds));
        flash.erase(0, 0x100).unwrap();
        assert!(flash.range(0, 0x100).iter().all(|byte| *byte == 0xFF));
        assert_eq!(flash.operations(), 3);
        assert_eq!(flash.max_wear(), 1);
    }

    #[test]
    fn strict_refuses_reprogramming() {
        let mut flash = Flash::new(0x400).strict();
        flash.write(0, &[0x0F, 0xFF, 0xFF, 0xFF]).unwrap();
        assert_eq!(flash.write(0, &[0x00; 4]), Err(SimError::NotErased));
        // Writing all-erased data is not programming.
        flash.write(0, &[0xFF; 4]).unwrap();
    }

    #[test]
    fn cuts() {
        let mut flash = Flash::new(0x400);
        flash.load(0, &[0; 0x400]);
        flash.cut_at(0, Cut::Half);
        assert_eq!(flash.erase(0, 0x200), Err(SimError::PowerLoss));
        assert!(flash.range(0, 0x100).iter().all(|byte| *byte == 0xFF));
        assert!(flash.range(0x100, 0x100).iter().all(|byte| *byte == 0));
        assert!(flash.lost_power());
        assert_eq!(flash.write(0x300, &[0; 4]), Err(SimError::PowerLoss));
        flash.power_cycle();

        flash.erase(0x300, 0x400).unwrap();
        flash.cut_at(1, Cut::Before);
        flash.write(0x300, &[0xAA; 4]).unwrap();
        assert_eq!(flash.write(0x304, &[0x00; 8]), Err(SimError::PowerLoss));
        assert_eq!(flash.range(0x300, 4), [0xAA; 4]);
        assert!(flash.range(0x304, 8).iter().all(|byte| *byte == 0xFF));
        flash.power_cycle();

        flash.erase(0x200, 0x300).unwrap();
        flash.cut_at(0, Cut::Torn);
        assert_eq!(flash.write(0x200, &[0x00; 0x100]), Err(SimError::PowerLoss));
        let torn = flash.range(0x200, 0x100);
        assert!(torn.iter().any(|byte| *byte != 0xFF) && torn.iter().any(|byte| *byte != 0));
    }
}
