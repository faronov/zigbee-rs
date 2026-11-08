//! Architecture checks on the entry of a validated image.
//!
//! The signature proves who built the image, not that it was linked for
//! this slot. These checks catch a correctly signed image built for another
//! layout before control is transferred to it.

/// Why a RISC-V (fixed-entry) image was not started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryError {
    /// The header does not place the entry at the slot's fixed header
    /// offset the application is linked for.
    Offset,
    /// The entry is outside the image or misaligned.
    Range,
}

/// Check an entry that is the first instruction after the image header (a
/// riscv-rt `_start`): it must be exactly `header_size` into the primary
/// slot at `primary`, 4-byte aligned, and inside the image payload ending
/// at `image_end`.
pub fn check_fixed_entry(
    primary: u32,
    header_size: u32,
    entry: u32,
    image_end: u32,
) -> Result<(), EntryError> {
    if entry != primary + header_size {
        return Err(EntryError::Offset);
    }
    if !entry.is_multiple_of(4) || entry >= image_end {
        return Err(EntryError::Range);
    }
    Ok(())
}

/// Why a Cortex-M image was not started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VectorError {
    StackPointer,
    ResetVector,
}

/// Sanity-check a Cortex-M vector table: the initial stack pointer must be
/// 8-byte aligned within `ram` (an empty full-descending stack may point at
/// its end), and the reset handler a Thumb address inside the image.
pub fn check_vector_table(
    initial_sp: u32,
    reset: u32,
    vector_table: u32,
    image_end: u32,
    ram: (u32, u32),
) -> Result<(), VectorError> {
    if !initial_sp.is_multiple_of(8) || initial_sp <= ram.0 || initial_sp > ram.1 {
        return Err(VectorError::StackPointer);
    }
    if reset & 1 == 0 || reset & !1 < vector_table || reset & !1 >= image_end {
        return Err(VectorError::ResetVector);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_entry_checks() {
        let check = |primary, entry, end| check_fixed_entry(primary, 0x100, entry, end);
        assert_eq!(check(0x10000, 0x10100, 0x40000), Ok(()));
        assert_eq!(check(0x10000, 0x10200, 0x40000), Err(EntryError::Offset));
        assert_eq!(check(0x10000, 0x10100, 0x10100), Err(EntryError::Range));
        assert_eq!(check(0x10002, 0x10102, 0x40000), Err(EntryError::Range));
        assert_eq!(check_fixed_entry(0x10000, 0x200, 0x10200, 0x40000), Ok(()));
    }

    const RAM: (u32, u32) = (0x2000_0000, 0x2000_9000);

    #[test]
    fn vector_table_checks() {
        let vt = 0x4100;
        let end = 0x4100 + 0x1000;
        assert_eq!(
            check_vector_table(0x2000_9000, 0x4201, vt, end, RAM),
            Ok(())
        );
        assert_eq!(
            check_vector_table(0x2000_1000, 0x4101 + 0xEFE, vt, end, RAM),
            Ok(())
        );
        for sp in [0x2000_0000, 0x2000_9008, 0x2000_8FFC, 0xFFFF_FFFF, 0] {
            assert_eq!(
                check_vector_table(sp, 0x4201, vt, end, RAM),
                Err(VectorError::StackPointer),
                "{sp:#x}"
            );
        }
        for reset in [0x4200, 0x40FF, 0x5101, 0x0000_0101, 0xFFFF_FFFF] {
            assert_eq!(
                check_vector_table(0x2000_9000, reset, vt, end, RAM),
                Err(VectorError::ResetVector),
                "{reset:#x}"
            );
        }
    }
}
