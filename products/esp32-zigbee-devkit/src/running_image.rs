//! Actual running-image evidence for C6/H2/C5; never interpret otadata as evidence.
//!
//! ESP32-C5 (same sources, `esp32c5`): fixed 64 KiB pages (mmu_ll_get_page_size
//! has no register), SOC_MMU_ENTRY_NUM=512, SOC_MMU_ACCESS_SPIRAM=BIT(9),
//! SOC_MMU_VALID=BIT(10), SOC_MMU_SENSITIVE=BIT(11), value mask 0x1ff,
//! SOC_MMU_MAX_PADDR_PAGE_NUM=512; the SPI0 MMU_ITEM_CONTENT/INDEX registers
//! are at the same 0x6000_237c/0x6000_2380.
//!
//! Sources: ESP-IDF v5.5.1, for BOTH esp32c6 and esp32h2:
//! - components/hal/<chip>/include/hal/mmu_ll.h:
//!   mmu_ll_get_page_size, mmu_ll_get_entry_id, mmu_ll_check_entry_valid,
//!   mmu_ll_entry_id_to_paddr_base
//! - components/soc/<chip>/include/soc/ext_mem_defs.h:
//!   SOC_MMU_ENTRY_NUM=256, SOC_MMU_VALID=BIT(9), SOC_MMU_SENSITIVE=BIT(10),
//!   SOC_MMU_VALID_VAL_MASK=0x1ff, SOC_MMU_MAX_PADDR_PAGE_NUM=256,
//!   SOC_IRAM0_CACHE_ADDRESS_LOW=0x42000000
//! - components/soc/<chip>/register/soc/spi_mem_reg.h:
//!   SPI0 MMU_ITEM_CONTENT +0x37c, MMU_ITEM_INDEX +0x380,
//!   MMU_POWER_CTRL +0x384, PAGE_SIZE bits 3:4.
//!
//! Unlike esp-bootloader-esp-idf 0.4's booted_partition(), this does NOT assume
//! entry zero, a 64 KiB page, or a valid entry. It translates the address of a
//! function in this application's linked code. Identical images in both slots
//! are unambiguous with MMU evidence (descriptor comparisons alone are not).

#[cfg(any(target_os = "none", test))]
use crate::layout::{FLASH_SIZE, OTA_SLOT_COUNT, OTA_SLOT_SIZE, ota_slot_offset};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunningImageError {
    /// Backend has no reliable current-image evidence.
    Unavailable,
    /// Evidence was in RAM/ROM or outside the configured flash mapping window.
    NotFlashMapped,
    /// MMU entry is invalid, reserved, or outside the physical flash.
    InvalidMapping,
    /// Raw flash OTA does not support encrypted flash.
    EncryptedFlash,
    /// Mapped address is not in either checked application partition.
    OutsideOtaSlots,
    /// Evidence changed during this boot/session. Do not touch flash.
    Changed,
}

#[cfg(any(test, all(target_os = "none", not(feature = "esp32c5"))))]
fn page_shift(page_code: u8) -> Result<u32, RunningImageError> {
    match page_code {
        0..=3 => Ok(16 - page_code as u32),
        _ => Err(RunningImageError::InvalidMapping),
    }
}

#[cfg(any(test, all(target_os = "none", not(feature = "esp32c5"))))]
fn entry_index(vaddr: u32, page_code: u8) -> Result<u32, RunningImageError> {
    let shift = page_shift(page_code)?;
    let linear = vaddr
        .checked_sub(0x4200_0000)
        .filter(|offset| *offset < (256 << shift))
        .ok_or(RunningImageError::NotFlashMapped)?;
    Ok(linear >> shift)
}

#[cfg(any(test, all(target_os = "none", not(feature = "esp32c5"))))]
fn mapped_slot(vaddr: u32, page_code: u8, raw_entry: u32) -> Result<u8, RunningImageError> {
    entry_index(vaddr, page_code)?;
    let shift = page_shift(page_code)?;
    if raw_entry & (1 << 9) == 0 || raw_entry & !0x7ff != 0 {
        return Err(RunningImageError::InvalidMapping);
    }
    if raw_entry & (1 << 10) != 0 {
        return Err(RunningImageError::EncryptedFlash);
    }
    let physical_page = raw_entry & 0x1ff;
    // Follow mmu_ll_check_valid_paddr_region's documented physical-page bound,
    // even though the entry value mask has nine bits.
    if physical_page >= 256 {
        return Err(RunningImageError::InvalidMapping);
    }
    let physical = (physical_page << shift) | (vaddr & ((1 << shift) - 1));
    if physical >= FLASH_SIZE {
        return Err(RunningImageError::InvalidMapping);
    }
    for slot in 0..OTA_SLOT_COUNT {
        let base = ota_slot_offset(slot);
        if (base..base + OTA_SLOT_SIZE).contains(&physical) {
            return Ok(slot);
        }
    }
    Err(RunningImageError::OutsideOtaSlots)
}

/// ESP32-C5 entry: flash-mapped vaddr, 64 KiB page, 512 entries.
#[cfg(any(test, all(target_os = "none", feature = "esp32c5")))]
const C5_PAGE_SHIFT: u32 = 16;

#[cfg(any(test, all(target_os = "none", feature = "esp32c5")))]
fn c5_entry_index(vaddr: u32) -> Result<u32, RunningImageError> {
    let linear = vaddr
        .checked_sub(0x4200_0000)
        .filter(|offset| *offset < (512 << C5_PAGE_SHIFT))
        .ok_or(RunningImageError::NotFlashMapped)?;
    Ok(linear >> C5_PAGE_SHIFT)
}

#[cfg(any(test, all(target_os = "none", feature = "esp32c5")))]
fn c5_mapped_slot(vaddr: u32, raw_entry: u32) -> Result<u8, RunningImageError> {
    c5_entry_index(vaddr)?;
    // VALID = BIT(10); no bits above SENSITIVE = BIT(11).
    if raw_entry & (1 << 10) == 0 || raw_entry & !0xfff != 0 {
        return Err(RunningImageError::InvalidMapping);
    }
    if raw_entry & (1 << 11) != 0 {
        return Err(RunningImageError::EncryptedFlash);
    }
    // ACCESS_SPIRAM = BIT(9): a PSRAM page is not application flash.
    if raw_entry & (1 << 9) != 0 {
        return Err(RunningImageError::InvalidMapping);
    }
    let physical_page = raw_entry & 0x1ff;
    let physical = (physical_page << C5_PAGE_SHIFT) | (vaddr & ((1 << C5_PAGE_SHIFT) - 1));
    if physical >= FLASH_SIZE {
        return Err(RunningImageError::InvalidMapping);
    }
    for slot in 0..OTA_SLOT_COUNT {
        let base = ota_slot_offset(slot);
        if (base..base + OTA_SLOT_SIZE).contains(&physical) {
            return Ok(slot);
        }
    }
    Err(RunningImageError::OutsideOtaSlots)
}

/// Uses the normal `.text` placement of this function as the current-image
/// anchor. RAM-only builds fail closed. Reading the table never changes a
/// mapping; it temporarily changes and restores only the index selector.
#[cfg(all(target_os = "none", not(feature = "esp32c5")))]
#[inline(never)]
pub(super) fn detect() -> Result<u8, RunningImageError> {
    if esp_hal::efuse::Efuse::flash_encryption() {
        return Err(RunningImageError::EncryptedFlash);
    }
    let vaddr = detect as *const () as usize as u32;
    critical_section::with(|_| {
        // SAFETY: these are C6/H2 SPI0's documented MMU register addresses.
        // The index/content access is serialized against interrupts on these
        // single-core chips. No cache disable, remap or flash operation occurs.
        // SPI0 base 0x60002000 is also used by esp-bootloader-esp-idf 0.4.0
        // partitions.rs and the C6/H2 PACs. All reads/writes are volatile.
        unsafe {
            let power = (0x6000_2384 as *const u32).read_volatile();
            let page_code = ((power >> 3) & 3) as u8;
            let index = entry_index(vaddr, page_code)?;
            let selector = 0x6000_2380 as *mut u32;
            let saved = selector.read_volatile();
            selector.write_volatile(index);
            let raw = (0x6000_237c as *const u32).read_volatile();
            selector.write_volatile(saved);
            mapped_slot(vaddr, page_code, raw)
        }
    })
}

/// ESP32-C5 variant of [`detect`]: no page-size register (fixed 64 KiB) and
/// the C5 entry format; esp-hal 1.1 `efuse::flash_encryption()`.
#[cfg(all(target_os = "none", feature = "esp32c5"))]
#[inline(never)]
pub(super) fn detect() -> Result<u8, RunningImageError> {
    if esp_hal::efuse::flash_encryption() {
        return Err(RunningImageError::EncryptedFlash);
    }
    let vaddr = detect as *const () as usize as u32;
    critical_section::with(|_| {
        // SAFETY: ESP-IDF v5.5.1 esp32c5 `spi_mem_reg.h`: SPI0 (0x6000_2000)
        // MMU_ITEM_CONTENT +0x37c, MMU_ITEM_INDEX +0x380. Single core; the
        // index selector is restored. No cache disable, remap or flash op.
        unsafe {
            let index = c5_entry_index(vaddr)?;
            let selector = 0x6000_2380 as *mut u32;
            let saved = selector.read_volatile();
            selector.write_volatile(index);
            let raw = (0x6000_237c as *const u32).read_volatile();
            selector.write_volatile(saved);
            c5_mapped_slot(vaddr, raw)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c5_translates_flash_pages_and_refuses_psram_encrypted_and_invalid() {
        for slot in 0..OTA_SLOT_COUNT {
            let offset = 0x3_0122;
            let vaddr = 0x4200_0000 + offset;
            let physical = ota_slot_offset(slot) + offset;
            let raw = (1 << 10) | (physical >> C5_PAGE_SHIFT);
            assert_eq!(c5_entry_index(vaddr), Ok(offset >> C5_PAGE_SHIFT));
            assert_eq!(c5_mapped_slot(vaddr, raw), Ok(slot));
            // The same page through PSRAM is not application flash.
            assert_eq!(
                c5_mapped_slot(vaddr, raw | (1 << 9)),
                Err(RunningImageError::InvalidMapping)
            );
            assert_eq!(
                c5_mapped_slot(vaddr, raw | (1 << 11)),
                Err(RunningImageError::EncryptedFlash)
            );
            // The C6/H2 valid bit (9) alone is not valid on the C5.
            assert_eq!(
                c5_mapped_slot(vaddr, (1 << 9) | (physical >> C5_PAGE_SHIFT)),
                Err(RunningImageError::InvalidMapping)
            );
        }
        // 512 x 64 KiB virtual window: 0x4400_0000 is outside.
        assert_eq!(
            c5_entry_index(0x4400_0000),
            Err(RunningImageError::NotFlashMapped)
        );
        assert_eq!(c5_entry_index(0x43FF_FFFF), Ok(511));
        assert_eq!(
            c5_mapped_slot(0x4080_0000, 0x401),
            Err(RunningImageError::NotFlashMapped)
        );
        assert_eq!(
            c5_mapped_slot(0x4200_0122, 0x1401),
            Err(RunningImageError::InvalidMapping)
        );
        assert_eq!(
            c5_mapped_slot(0x4200_0000, 0x400),
            Err(RunningImageError::OutsideOtaSlots)
        );
    }

    #[test]
    fn translates_actual_code_page_with_documented_page_and_flash_bounds() {
        for code in 0..=3 {
            let shift = page_shift(code).unwrap();
            for slot in 0..OTA_SLOT_COUNT {
                // Not entry zero: code may live many pages after the descriptor.
                let offset = 0x3_0122;
                let vaddr = 0x4200_0000 + offset;
                let physical = ota_slot_offset(slot) + offset;
                let raw = (1 << 9) | (physical >> shift);
                assert_eq!(entry_index(vaddr, code), Ok(offset >> shift));
                let expected = if physical >> shift >= 256 {
                    // 8 KiB pages cannot address ota_1 under IDF's physical
                    // page bound. Do not silently mask away the upper bit.
                    Err(RunningImageError::InvalidMapping)
                } else {
                    Ok(slot)
                };
                assert_eq!(mapped_slot(vaddr, code, raw), expected);
            }
        }
    }

    #[test]
    fn refuses_invalid_encrypted_out_of_range_and_non_application_evidence() {
        assert_eq!(
            mapped_slot(0x4200_0122, 0, 0),
            Err(RunningImageError::InvalidMapping)
        );
        assert_eq!(
            mapped_slot(0x4080_0000, 0, 0x201),
            Err(RunningImageError::NotFlashMapped)
        );
        assert_eq!(
            mapped_slot(0x4000_0000, 0, 0x201),
            Err(RunningImageError::NotFlashMapped)
        );
        assert_eq!(
            mapped_slot(0x4300_0000, 0, 0x201),
            Err(RunningImageError::NotFlashMapped)
        );
        assert_eq!(
            mapped_slot(0x4200_0000, 3, 0x201),
            Err(RunningImageError::OutsideOtaSlots)
        );
        assert_eq!(
            mapped_slot(0x4200_0122, 0, 0x601),
            Err(RunningImageError::EncryptedFlash)
        );
        assert_eq!(
            mapped_slot(0x4200_0122, 0, 0x3ff),
            Err(RunningImageError::InvalidMapping)
        );
        assert_eq!(
            mapped_slot(0x4200_0122, 0, 0xa01),
            Err(RunningImageError::InvalidMapping)
        );
        assert_eq!(
            mapped_slot(0x4200_0122, 0, 0x200),
            Err(RunningImageError::OutsideOtaSlots)
        );
        assert_eq!(
            mapped_slot(0x4200_0122, 4, 0x201),
            Err(RunningImageError::InvalidMapping)
        );
    }
}
