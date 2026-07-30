//! BL702 XIP serial-flash access through the mask-ROM driver table.
//!
//! The HAL exposes the raw physical flash only. Product crates remain
//! responsible for application, persistence, OTA, and protected-region
//! boundaries.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::peripherals::Flash;

#[cfg(target_arch = "riscv32")]
const ROM_API_TABLE: u32 = 0x2101_8800;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_XIP_ERASE: usize = 78;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_XIP_WRITE: usize = 79;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_XIP_READ: usize = 80;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_LOAD_FLASH_CONFIG: usize = 157;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_SF_CTRL_AES_ENABLE: usize = 144;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_SF_CTRL_AES_DISABLE: usize = 145;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_SF_CTRL_IS_AES_ENABLE: usize = 146;
#[cfg(any(target_arch = "riscv32", test))]
const ROM_L1C_CACHE_FLUSH: usize = 121;
#[cfg(target_arch = "riscv32")]
const L1C_CONFIG: u32 = 0x4000_9000;
#[cfg(target_arch = "riscv32")]
const L1C_WAY_DISABLE_SHIFT: u32 = 8;
const MAX_PHYSICAL_CAPACITY: usize = 16 * 1024 * 1024;
const XIP_WINDOWS: [(usize, usize); 4] = [
    (0x2300_0000, 0x2400_0000),
    (0x3300_0000, 0x3400_0000),
    (0x4300_0000, 0x4400_0000),
    (0x5300_0000, 0x5400_0000),
];
const SECTOR_SIZE: usize = 4096;

#[repr(C, align(4))]
struct FlashConfig {
    bytes: [u8; 84],
}

const _: () = assert!(core::mem::size_of::<FlashConfig>() == 84);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    OutOfBounds,
    NotAligned,
    InvalidCapacity,
    InvalidBootConfiguration,
    SourceInXip,
    RomError(i32),
    UnsupportedTarget,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// Raw physical serial flash accessed by BL702 ROM routines.
pub struct XipFlash {
    _token: Flash,
    config: FlashConfig,
    capacity: usize,
    io_mode: u32,
}

impl XipFlash {
    /// Load and CRC-check the flash configuration from the ROM boot header.
    ///
    /// `capacity` describes the physically fitted flash and must be supplied
    /// by the board/product composition layer.
    pub fn new(token: Flash, capacity: usize) -> Result<Self, FlashError> {
        if capacity == 0 || capacity > MAX_PHYSICAL_CAPACITY {
            return Err(FlashError::InvalidCapacity);
        }
        let mut config = FlashConfig { bytes: [0; 84] };
        rom_load_config(&mut config)?;
        let io_mode = u32::from(config.bytes[0] & 0x0f);
        if io_mode > 4 {
            return Err(FlashError::InvalidBootConfiguration);
        }
        Ok(Self {
            _token: token,
            config,
            capacity,
            io_mode,
        })
    }

    fn validate_range(&self, offset: u32, length: usize) -> Result<(), FlashError> {
        validate_range(self.capacity, offset, length)
    }
}

impl ErrorType for XipFlash {
    type Error = FlashError;
}

impl ReadNorFlash for XipFlash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.validate_range(offset, bytes.len())?;
        if bytes.is_empty() {
            return Ok(());
        }
        rom_read(
            &mut self.config,
            self.io_mode,
            offset,
            bytes.as_mut_ptr(),
            bytes.len(),
        )
    }

    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl NorFlash for XipFlash {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        validate_erase_range(self.capacity, from, to)?;
        if from == to {
            return Ok(());
        }
        rom_erase(&mut self.config, self.io_mode, from, to - 1)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !(offset as usize).is_multiple_of(Self::WRITE_SIZE)
            || !bytes.len().is_multiple_of(Self::WRITE_SIZE)
        {
            return Err(FlashError::NotAligned);
        }
        self.validate_range(offset, bytes.len())?;
        if bytes.is_empty() {
            return Ok(());
        }
        if slice_overlaps_xip(bytes.as_ptr(), bytes.len()) {
            return Err(FlashError::SourceInXip);
        }
        rom_write(
            &mut self.config,
            self.io_mode,
            offset,
            bytes.as_ptr(),
            bytes.len(),
        )
    }
}

fn slice_overlaps_xip(pointer: *const u8, length: usize) -> bool {
    let start = pointer as usize;
    let end = start.saturating_add(length);
    XIP_WINDOWS
        .iter()
        .any(|&(window_start, window_end)| start < window_end && end > window_start)
}

fn validate_range(capacity: usize, offset: u32, length: usize) -> Result<(), FlashError> {
    (offset as usize)
        .checked_add(length)
        .filter(|end| *end <= capacity)
        .map(|_| ())
        .ok_or(FlashError::OutOfBounds)
}

fn validate_erase_range(capacity: usize, from: u32, to: u32) -> Result<(), FlashError> {
    if from > to || to as usize > capacity {
        return Err(FlashError::OutOfBounds);
    }
    if !(from as usize).is_multiple_of(SECTOR_SIZE) || !(to as usize).is_multiple_of(SECTOR_SIZE) {
        return Err(FlashError::NotAligned);
    }
    Ok(())
}

#[cfg(target_arch = "riscv32")]
#[inline(never)]
#[unsafe(link_section = ".data.ram_code")]
fn rom_load_config(config: &mut FlashConfig) -> Result<(), FlashError> {
    type LoadConfig = unsafe extern "C" fn(u32, *mut FlashConfig) -> i32;
    type CacheFlush = unsafe extern "C" fn(u8) -> i32;

    // Resolve ROM entries and cache configuration before changing the cache
    // or interrupt state.
    let load_config: LoadConfig = unsafe { core::mem::transmute(rom_entry(ROM_LOAD_FLASH_CONFIG)) };
    let cache_flush: CacheFlush = unsafe { core::mem::transmute(rom_entry(ROM_L1C_CACHE_FLUSH)) };
    let way_disable = ((crate::mmio::read32(L1C_CONFIG) >> L1C_WAY_DISABLE_SHIFT) & 0x0f) as u8;

    let previous_mstatus: usize;
    // SAFETY: The complete cache-remap sequence executes from RAM and mirrors
    // the SDK flash initialization's interrupt exclusion.
    unsafe {
        core::arch::asm!("csrrci {}, mstatus, 8", out(reg) previous_mstatus);
    }
    let flush_before = unsafe { cache_flush(way_disable) };
    let load_result = if flush_before == 0 {
        // SAFETY: The ROM ABI and table index match the BL702 SDK.
        unsafe { load_config(0, config) }
    } else {
        flush_before
    };
    // SF_Cfg_Get_Flash_Cfg_Need_Lock temporarily remaps the XIP image offset
    // and reads through cache, so stale lines must be invalidated afterward.
    let flush_after = unsafe { cache_flush(way_disable) };
    if previous_mstatus & (1 << 3) != 0 {
        // SAFETY: Restore MIE only if it was enabled on entry.
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }

    if load_result != 0 {
        Err(FlashError::RomError(load_result))
    } else {
        rom_result(flush_after)
    }
}

#[cfg(not(target_arch = "riscv32"))]
fn rom_load_config(_config: &mut FlashConfig) -> Result<(), FlashError> {
    Err(FlashError::UnsupportedTarget)
}

#[cfg(target_arch = "riscv32")]
#[inline(never)]
#[unsafe(link_section = ".data.ram_code")]
fn rom_read(
    config: &mut FlashConfig,
    io_mode: u32,
    address: u32,
    data: *mut u8,
    length: usize,
) -> Result<(), FlashError> {
    type Function = unsafe extern "C" fn(*mut FlashConfig, u32, u32, *mut u8, u32) -> i32;
    type AesState = unsafe extern "C" fn() -> u8;
    type AesControl = unsafe extern "C" fn();

    // Resolve every ROM entry before XIP is disturbed.
    let function: Function = unsafe { core::mem::transmute(rom_entry(ROM_XIP_READ)) };
    let aes_state: AesState = unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_IS_AES_ENABLE)) };
    let aes_disable: AesControl =
        unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_AES_DISABLE)) };
    let aes_enable: AesControl = unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_AES_ENABLE)) };

    let previous_mstatus: usize;
    // SAFETY: This RAM-resident routine serializes the XIP controller state.
    unsafe {
        core::arch::asm!("csrrci {}, mstatus, 8", out(reg) previous_mstatus);
    }
    let aes_was_enabled = unsafe { aes_state() } != 0;
    if aes_was_enabled {
        unsafe { aes_disable() };
    }
    // SAFETY: The verified ROM routine saves and restores XIP state.
    let result = unsafe { function(config, io_mode, address, data, length as u32) };
    if aes_was_enabled {
        unsafe { aes_enable() };
    }
    if previous_mstatus & (1 << 3) != 0 {
        // SAFETY: Restore MIE only if it was enabled on entry.
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }
    rom_result(result)
}

#[cfg(not(target_arch = "riscv32"))]
fn rom_read(
    _config: &mut FlashConfig,
    _io_mode: u32,
    _address: u32,
    _data: *mut u8,
    _length: usize,
) -> Result<(), FlashError> {
    Err(FlashError::UnsupportedTarget)
}

#[cfg(target_arch = "riscv32")]
#[inline(never)]
#[unsafe(link_section = ".data.ram_code")]
fn rom_write(
    config: &mut FlashConfig,
    io_mode: u32,
    address: u32,
    data: *const u8,
    length: usize,
) -> Result<(), FlashError> {
    type Function = unsafe extern "C" fn(*mut FlashConfig, u32, u32, *mut u8, u32) -> i32;
    type AesState = unsafe extern "C" fn() -> u8;
    type AesControl = unsafe extern "C" fn();

    let function: Function = unsafe { core::mem::transmute(rom_entry(ROM_XIP_WRITE)) };
    let aes_state: AesState = unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_IS_AES_ENABLE)) };
    let aes_disable: AesControl =
        unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_AES_DISABLE)) };
    let aes_enable: AesControl = unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_AES_ENABLE)) };

    let previous_mstatus: usize;
    // SAFETY: This RAM-resident routine serializes the XIP controller state.
    unsafe {
        core::arch::asm!("csrrci {}, mstatus, 8", out(reg) previous_mstatus);
    }
    let aes_was_enabled = unsafe { aes_state() } != 0;
    if aes_was_enabled {
        unsafe { aes_disable() };
    }
    // SAFETY: The source was checked against every BL702 flash XIP alias.
    let result = unsafe { function(config, io_mode, address, data.cast_mut(), length as u32) };
    if aes_was_enabled {
        unsafe { aes_enable() };
    }
    if previous_mstatus & (1 << 3) != 0 {
        // SAFETY: Restore MIE only if it was enabled on entry.
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }
    rom_result(result)
}

#[cfg(not(target_arch = "riscv32"))]
fn rom_write(
    _config: &mut FlashConfig,
    _io_mode: u32,
    _address: u32,
    _data: *const u8,
    _length: usize,
) -> Result<(), FlashError> {
    Err(FlashError::UnsupportedTarget)
}

#[cfg(target_arch = "riscv32")]
#[inline(never)]
#[unsafe(link_section = ".data.ram_code")]
fn rom_erase(
    config: &mut FlashConfig,
    io_mode: u32,
    start: u32,
    inclusive_end: u32,
) -> Result<(), FlashError> {
    type Function = unsafe extern "C" fn(*mut FlashConfig, u32, u32, u32) -> i32;
    type AesState = unsafe extern "C" fn() -> u8;
    type AesControl = unsafe extern "C" fn();

    let function: Function = unsafe { core::mem::transmute(rom_entry(ROM_XIP_ERASE)) };
    let aes_state: AesState = unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_IS_AES_ENABLE)) };
    let aes_disable: AesControl =
        unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_AES_DISABLE)) };
    let aes_enable: AesControl = unsafe { core::mem::transmute(rom_entry(ROM_SF_CTRL_AES_ENABLE)) };

    let previous_mstatus: usize;
    // SAFETY: This RAM-resident routine serializes the XIP controller state.
    unsafe {
        core::arch::asm!("csrrci {}, mstatus, 8", out(reg) previous_mstatus);
    }
    let aes_was_enabled = unsafe { aes_state() } != 0;
    if aes_was_enabled {
        unsafe { aes_disable() };
    }
    // SAFETY: The verified ROM routine saves and restores XIP state.
    let result = unsafe { function(config, io_mode, start, inclusive_end) };
    if aes_was_enabled {
        unsafe { aes_enable() };
    }
    if previous_mstatus & (1 << 3) != 0 {
        // SAFETY: Restore MIE only if it was enabled on entry.
        unsafe { core::arch::asm!("csrsi mstatus, 8") };
    }
    rom_result(result)
}

#[cfg(not(target_arch = "riscv32"))]
fn rom_erase(
    _config: &mut FlashConfig,
    _io_mode: u32,
    _start: u32,
    _inclusive_end: u32,
) -> Result<(), FlashError> {
    Err(FlashError::UnsupportedTarget)
}

#[cfg(target_arch = "riscv32")]
unsafe fn rom_entry(index: usize) -> usize {
    // SAFETY: The BL702 mask ROM exposes a word-addressed function table at
    // this fixed address. Individual callers use verified SDK indices.
    unsafe { core::ptr::read_volatile((ROM_API_TABLE + (index as u32 * 4)) as *const u32) as usize }
}

#[cfg(target_arch = "riscv32")]
fn rom_result(result: i32) -> Result<(), FlashError> {
    if result == 0 {
        Ok(())
    } else {
        Err(FlashError::RomError(result))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flash_configuration_abi_is_exact() {
        assert_eq!(core::mem::size_of::<FlashConfig>(), 84);
        assert_eq!(core::mem::align_of::<FlashConfig>(), 4);
    }

    #[test]
    fn rom_table_indices_match_bl702_sdk_enum() {
        assert_eq!(ROM_XIP_ERASE, 78);
        assert_eq!(ROM_XIP_WRITE, 79);
        assert_eq!(ROM_XIP_READ, 80);
        assert_eq!(ROM_SF_CTRL_AES_ENABLE, 144);
        assert_eq!(ROM_SF_CTRL_AES_DISABLE, 145);
        assert_eq!(ROM_SF_CTRL_IS_AES_ENABLE, 146);
        assert_eq!(ROM_L1C_CACHE_FLUSH, 121);
        assert_eq!(ROM_LOAD_FLASH_CONFIG, 157);
    }

    #[test]
    fn xip_source_detection_is_bounded() {
        for &(start, end) in &XIP_WINDOWS {
            assert!(slice_overlaps_xip(start as *const u8, 4));
            assert!(slice_overlaps_xip((end - 1) as *const u8, 4));
        }
        assert!(!slice_overlaps_xip(0x4201_4000 as *const u8, 4));
    }

    #[test]
    fn physical_ranges_include_boundaries_and_reject_overflow() {
        assert_eq!(validate_range(1024, 0, 1024), Ok(()));
        assert_eq!(validate_range(1024, 1024, 0), Ok(()));
        assert_eq!(validate_range(1024, 1023, 2), Err(FlashError::OutOfBounds));
        assert_eq!(
            validate_range(1024, u32::MAX, usize::MAX),
            Err(FlashError::OutOfBounds)
        );
    }

    #[test]
    fn embedded_storage_erase_range_accepts_empty_aligned_ranges() {
        assert_eq!(XipFlash::WRITE_SIZE, 1);
        assert_eq!(validate_erase_range(8192, 0, 0), Ok(()));
        assert_eq!(validate_erase_range(8192, 8192, 8192), Ok(()));
        assert_eq!(
            validate_erase_range(8192, 4096, 0),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_erase_range(8192, 0, 8193),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            validate_erase_range(8192, 1, 4096),
            Err(FlashError::NotAligned)
        );
    }
}
