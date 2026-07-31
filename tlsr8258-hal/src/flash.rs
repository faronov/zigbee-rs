//! RAM-resident TLSR8258 SPI-flash read, page-program, and sector-erase
//! operations. The application must initialize clocks before use.

#![cfg(target_arch = "tc32")]

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use super::mmio::REG_IRQ_EN;

const REG_MSPI_DATA: u32 = 0x80000C;
const REG_MSPI_CTRL: u32 = 0x80000D;
const FALLBACK_IEEE: [u8; 8] = [0x9F, 0x5D, 0xC3, 0x0C, 0x00, 0x4B, 0x12, 0x00];

const FLASH_WRITE_ENABLE: u8 = 0x06;
const FLASH_READ_STATUS: u8 = 0x05;
const FLASH_PAGE_PROGRAM: u8 = 0x02;
const FLASH_SECTOR_ERASE: u8 = 0x20;

pub const PAGE_SIZE: usize = 256;
pub const SECTOR_SIZE: u32 = 4096;

pub const IEEE_SOURCE_FALLBACK: u8 = 0;
pub const IEEE_SOURCE_FACTORY: u8 = 1;
pub const IEEE_SOURCE_FLASH_UID: u8 = 2;

/// Physical SPI-flash geometry used to locate Telink's factory MAC and
/// calibration sectors.
///
/// These addresses are the `FLASH_ADDR_OF_MAC_ADDR_*` and
/// `FLASH_ADDR_OF_F_CFG_INFO_*` constants from the official Zigbee SDK's
/// `proj/drivers/drv_nv.h`. They are not interchangeable: using the legacy
/// 512 KiB address on a 1 MiB smart plug reads ordinary application data
/// instead of its factory EUI-64.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashGeometry {
    KiB512,
    MiB1,
    MiB2,
    MiB4,
}

impl FlashGeometry {
    pub const fn capacity(self) -> usize {
        match self {
            Self::KiB512 => 512 * 1024,
            Self::MiB1 => 1024 * 1024,
            Self::MiB2 => 2 * 1024 * 1024,
            Self::MiB4 => 4 * 1024 * 1024,
        }
    }

    pub const fn from_capacity(capacity: usize) -> Option<Self> {
        match capacity {
            0x0008_0000 => Some(Self::KiB512),
            0x0010_0000 => Some(Self::MiB1),
            0x0020_0000 => Some(Self::MiB2),
            0x0040_0000 => Some(Self::MiB4),
            _ => None,
        }
    }

    pub const fn factory_ieee_address(self) -> u32 {
        match self {
            Self::KiB512 => 0x0007_6000,
            Self::MiB1 => 0x000F_F000,
            Self::MiB2 => 0x001F_F000,
            Self::MiB4 => 0x003F_F000,
        }
    }

    pub const fn factory_config_address(self) -> u32 {
        match self {
            Self::KiB512 => 0x0007_7000,
            Self::MiB1 => 0x000F_E000,
            Self::MiB2 => 0x001F_E000,
            Self::MiB4 => 0x003F_E000,
        }
    }

    pub const fn adc_calibration_address(self) -> u32 {
        self.factory_config_address() + 0xC0
    }

    /// Decode the standard JEDEC capacity byte used by the flash parts
    /// supported by Telink's factory layout table.
    pub const fn from_jedec_capacity_code(code: u8) -> Option<Self> {
        match code {
            0x13 => Some(Self::KiB512),
            0x14 => Some(Self::MiB1),
            0x15 => Some(Self::MiB2),
            0x16 => Some(Self::MiB4),
            _ => None,
        }
    }
}

const _: () = {
    assert!(FlashGeometry::KiB512.factory_ieee_address() == 0x76000);
    assert!(FlashGeometry::KiB512.factory_config_address() == 0x77000);
    assert!(FlashGeometry::KiB512.adc_calibration_address() == 0x770C0);
    assert!(FlashGeometry::MiB1.factory_ieee_address() == 0xFF000);
    assert!(FlashGeometry::MiB1.factory_config_address() == 0xFE000);
    assert!(FlashGeometry::MiB1.adc_calibration_address() == 0xFE0C0);
    assert!(matches!(
        FlashGeometry::from_capacity(0x0008_0000),
        Some(FlashGeometry::KiB512)
    ));
    assert!(matches!(
        FlashGeometry::from_capacity(0x0010_0000),
        Some(FlashGeometry::MiB1)
    ));
    assert!(matches!(
        FlashGeometry::from_jedec_capacity_code(0x13),
        Some(FlashGeometry::KiB512)
    ));
    assert!(matches!(
        FlashGeometry::from_jedec_capacity_code(0x14),
        Some(FlashGeometry::MiB1)
    ));
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    Timeout,
    AddressOverflow,
    UnalignedSector,
    BufferNotInRam,
    /// The JEDEC capacity code is not one of the geometries for which the
    /// Telink SDK defines factory-data locations.
    UnsupportedGeometry,
    /// The detected JEDEC capacity does not match the product-selected
    /// geometry. Writes must fail rather than target another layout's NV
    /// or factory sectors.
    GeometryMismatch,
    /// No ADC-backed voltage guard is installed, or the
    /// registered guard reported [`VoltageReading::Unavailable`] — the
    /// flash-supply voltage could not be checked at all.
    VoltageGuardUnavailable,
    /// The voltage guard took a real reading, but it is below/unstable
    /// relative to the Zbit safety thresholds — a genuinely low or noisy
    /// supply, distinct from [`FlashError::VoltageGuardUnavailable`].
    VoltageUnsafe,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::AddressOverflow => NorFlashErrorKind::OutOfBounds,
            Self::UnalignedSector => NorFlashErrorKind::NotAligned,
            Self::Timeout
            | Self::BufferNotInRam
            | Self::UnsupportedGeometry
            | Self::GeometryMismatch
            | Self::VoltageGuardUnavailable
            | Self::VoltageUnsafe => NorFlashErrorKind::Other,
        }
    }
}

/// Full-chip TLSR8258 NOR flash controller.
///
/// Board crates provide the guaranteed flash capacity and expose bounded
/// partitions to applications.
pub struct Tlsr8258Flash {
    capacity: usize,
}

impl Tlsr8258Flash {
    pub const fn new(capacity: usize) -> Self {
        Self { capacity }
    }

    pub const fn for_geometry(geometry: FlashGeometry) -> Self {
        Self::new(geometry.capacity())
    }

    fn validate_range(&self, address: u32, length: usize) -> Result<(), FlashError> {
        let start = usize::try_from(address).map_err(|_| FlashError::AddressOverflow)?;
        start
            .checked_add(length)
            .filter(|end| *end <= self.capacity)
            .map(|_| ())
            .ok_or(FlashError::AddressOverflow)
    }
}

impl ErrorType for Tlsr8258Flash {
    type Error = FlashError;
}

impl ReadNorFlash for Tlsr8258Flash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.validate_range(offset, bytes.len())?;
        if read_bytes(offset, bytes) {
            Ok(())
        } else {
            Err(FlashError::Timeout)
        }
    }

    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl NorFlash for Tlsr8258Flash {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = SECTOR_SIZE as usize;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::AddressOverflow);
        }
        if from & (SECTOR_SIZE - 1) != 0 || to & (SECTOR_SIZE - 1) != 0 {
            return Err(FlashError::UnalignedSector);
        }
        self.validate_range(from, (to - from) as usize)?;
        let mut address = from;
        while address < to {
            erase_sector(address)?;
            address += SECTOR_SIZE;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.validate_range(offset, bytes.len())?;
        program(offset, bytes)
    }
}

#[inline(always)]
fn irq_disable() -> u8 {
    use core::sync::atomic::{Ordering, compiler_fence};

    let previous = unsafe { core::ptr::read_volatile(REG_IRQ_EN as *const u8) };
    unsafe { core::ptr::write_volatile(REG_IRQ_EN as *mut u8, 0) };
    compiler_fence(Ordering::SeqCst);
    previous
}

#[inline(always)]
fn irq_restore(previous: u8) {
    use core::sync::atomic::{Ordering, compiler_fence};

    compiler_fence(Ordering::SeqCst);
    unsafe { core::ptr::write_volatile(REG_IRQ_EN as *mut u8, previous) };
}

#[inline(always)]
fn delay_nops(count: u32) {
    for _ in 0..count {
        unsafe { core::arch::asm!("nop") };
    }
}

#[inline(always)]
fn mspi_wait() -> bool {
    for _ in 0..100_000u32 {
        if unsafe { core::ptr::read_volatile(REG_MSPI_CTRL as *const u8) } & 0x10 == 0 {
            return true;
        }
        unsafe { core::arch::asm!("nop") };
    }
    false
}

#[inline(always)]
fn mspi_high() {
    unsafe { core::ptr::write_volatile(REG_MSPI_CTRL as *mut u8, 0x01) };
}

#[inline(always)]
fn mspi_low() {
    unsafe { core::ptr::write_volatile(REG_MSPI_CTRL as *mut u8, 0x00) };
}

#[inline(always)]
fn mspi_write(byte: u8) {
    unsafe { core::ptr::write_volatile(REG_MSPI_DATA as *mut u8, byte) };
}

#[inline(always)]
fn mspi_get() -> u8 {
    unsafe { core::ptr::read_volatile(REG_MSPI_DATA as *const u8) }
}

#[inline(always)]
fn send_command(command: u8) -> bool {
    mspi_high();
    // Vendor flash_send_cmd() guarantees at least 1 us of CS-high time.
    delay_nops(64);
    mspi_low();
    mspi_write(command);
    mspi_wait()
}

#[inline(always)]
fn send_address(address: u32) -> bool {
    mspi_write((address >> 16) as u8);
    if !mspi_wait() {
        return false;
    }
    mspi_write((address >> 8) as u8);
    if !mspi_wait() {
        return false;
    }
    mspi_write(address as u8);
    mspi_wait()
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn read_command_inner(
    command: u8,
    address: u32,
    address_enabled: bool,
    dummy_count: u8,
    output: &mut [u8],
) -> bool {
    if !send_command(command) {
        mspi_high();
        return false;
    }
    if address_enabled && !send_address(address) {
        mspi_high();
        return false;
    }
    let mut dummy = 0;
    while dummy < dummy_count {
        mspi_write(0);
        if !mspi_wait() {
            mspi_high();
            return false;
        }
        dummy += 1;
    }
    mspi_write(0);
    if !mspi_wait() {
        mspi_high();
        return false;
    }
    unsafe { core::ptr::write_volatile(REG_MSPI_CTRL as *mut u8, 0x0A) };
    if !mspi_wait() {
        mspi_high();
        return false;
    }
    let mut index = 0;
    while index < output.len() {
        output[index] = mspi_get();
        if !mspi_wait() {
            mspi_high();
            return false;
        }
        index += 1;
    }
    mspi_high();
    true
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn read_command(
    command: u8,
    address: u32,
    address_enabled: bool,
    dummy_count: u8,
    output: &mut [u8],
) -> bool {
    let previous_irq = irq_disable();
    let result = read_command_inner(command, address, address_enabled, dummy_count, output);
    irq_restore(previous_irq);
    result
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
pub fn read_bytes(address: u32, output: &mut [u8]) -> bool {
    read_command(0x03, address, true, 0, output)
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
pub fn jedec_id(output: &mut [u8; 3]) -> bool {
    read_command(0x9F, 0, false, 0, output)
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn read_uid(output: &mut [u8; 16]) -> bool {
    read_command(0x4B, 0, true, 1, output)
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn wait_flash_idle() -> bool {
    // Telink waits 100 us before the first RDSR1 poll. This deliberately
    // overshoots at both supported 24/48 MHz clocks.
    delay_nops(10_000);
    if !send_command(FLASH_READ_STATUS) {
        mspi_high();
        return false;
    }

    for _ in 0..10_000_000u32 {
        mspi_write(0);
        if !mspi_wait() {
            mspi_high();
            return false;
        }
        if mspi_get() & 0x01 == 0 {
            mspi_high();
            return true;
        }
    }
    mspi_high();
    false
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn write_command_inner(command: u8, address: u32, data: &[u8]) -> bool {
    if !send_command(FLASH_WRITE_ENABLE) {
        mspi_high();
        return false;
    }
    if !send_command(command) || !send_address(address) {
        mspi_high();
        return false;
    }

    let mut index = 0;
    while index < data.len() {
        mspi_write(data[index]);
        if !mspi_wait() {
            mspi_high();
            return false;
        }
        index += 1;
    }

    mspi_high();
    wait_flash_idle()
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn write_command(command: u8, address: u32, data: &[u8]) -> bool {
    let previous_irq = irq_disable();
    let result = write_command_inner(command, address, data);
    irq_restore(previous_irq);
    result
}

fn ensure_ram_buffer(data: &[u8]) -> Result<(), FlashError> {
    if data.is_empty() {
        return Ok(());
    }
    if !super::mmio::sram_contains(data.as_ptr() as usize, data.len()) {
        return Err(FlashError::BufferNotInRam);
    }
    Ok(())
}

/// Zbit flash (`ZB25WD40B`/`ZB25WD80B`, JEDEC MID `0x13325E`/`0x14325E`)
/// requires an ADC voltage/fluctuation guard before program/erase
/// operations — see `platform/chip_8258/flash.c`'s
/// `flash_mspi_write_ram()`, which refuses to send the address phase
/// unless `adc_get_result_with_fluct()` reads above `FLASH_ZBIT_SAFE_VOL`
/// (2200 mV) with a fluctuation below `FLASH_ZBIT_SAFE_VOLFLUCT` (500 mV).
const FLASH_ZBIT_SAFE_VOL_MV: u16 = 2200;
const FLASH_ZBIT_SAFE_VOLFLUCT_MV: u16 = 500;

/// Outcome of a single voltage-guard reading attempt.
///
/// This is deliberately not a plain `Option<(u16, u16)>`: a failed/absent
/// ADC reading (misconfigured pin, ADC not powered, DMA buffer error) and a
/// *successful* reading that happens to show a genuinely low or unstable
/// voltage are different situations for the caller to diagnose, so they get
/// distinct [`FlashError`] variants below rather than collapsing to one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoltageReading {
    /// A real ADC reading was obtained: `(voltage_mv, fluctuation_mv)`.
    Measured(u16, u16),
    /// No reading could be obtained right now (e.g. `adc::AdcError`
    /// surfaced from the callback) — distinct from a confirmed low/
    /// unstable measured voltage.
    Unavailable,
}

type VoltageGuardFn = fn() -> VoltageReading;

/// Storage for the registered [`VoltageGuardFn`], as the bit pattern of the
/// function pointer (`0` = none registered).
///
/// A `static mut` setter here would be unsound (safe code could call it
/// from two contexts and race the plain read/write); this crate has no
/// threads, but `AtomicUsize` gives a genuinely sound safe API for free
/// (single-instruction load/store, no critical section needed) rather than
/// pushing an `unsafe` requirement onto every call site of
/// `crate::adc::Adc::install_flash_voltage_guard`.
static VOLTAGE_GUARD: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Register the owned ADC-backed voltage guard.
///
/// Kept crate-private so safe external code cannot install a constant or
/// otherwise unowned callback that bypasses
/// [`crate::adc::Adc::install_flash_voltage_guard`].
pub(crate) fn set_voltage_guard(guard: VoltageGuardFn) {
    VOLTAGE_GUARD.store(guard as usize, core::sync::atomic::Ordering::SeqCst);
}

fn voltage_guard() -> Option<VoltageGuardFn> {
    let raw = VOLTAGE_GUARD.load(core::sync::atomic::Ordering::SeqCst);
    if raw == 0 {
        return None;
    }
    // SAFETY: the only non-zero values ever stored are `guard as usize`
    // for the real ADC-backed callback installed by this crate — `usize` is
    // this target's pointer-sized integer, so the round trip through the same
    // integer representation is valid.
    Some(unsafe { core::mem::transmute::<usize, VoltageGuardFn>(raw) })
}

fn ensure_safe_flash() -> Result<(), FlashError> {
    let mut id = [0u8; 3];
    if !jedec_id(&mut id) {
        return Err(FlashError::Timeout);
    }
    if id != [0x5E, 0x32, 0x13] && id != [0x5E, 0x32, 0x14] {
        return Ok(());
    }
    let Some(guard) = voltage_guard() else {
        return Err(FlashError::VoltageGuardUnavailable);
    };
    match guard() {
        VoltageReading::Unavailable => Err(FlashError::VoltageGuardUnavailable),
        VoltageReading::Measured(voltage_mv, fluctuation_mv)
            if voltage_mv > FLASH_ZBIT_SAFE_VOL_MV
                && fluctuation_mv < FLASH_ZBIT_SAFE_VOLFLUCT_MV =>
        {
            Ok(())
        }
        VoltageReading::Measured(_, _) => Err(FlashError::VoltageUnsafe),
    }
}

/// Program bytes without crossing a hardware page-program boundary.
///
/// `data` must reside in TLSR8258 SRAM because flash is unavailable while
/// each page is being programmed.
pub fn program(mut address: u32, mut data: &[u8]) -> Result<(), FlashError> {
    if data.is_empty() {
        return Ok(());
    }
    ensure_ram_buffer(data)?;
    address
        .checked_add(data.len() as u32)
        .filter(|end| *end <= 0x0100_0000)
        .ok_or(FlashError::AddressOverflow)?;

    while !data.is_empty() {
        let page_remaining = PAGE_SIZE - (address as usize & (PAGE_SIZE - 1));
        let count = data.len().min(page_remaining);
        // Re-check immediately before every physical page-program command.
        // A multi-page write must not rely on one stale supply reading
        // taken before the first page.
        ensure_safe_flash()?;
        if !write_command(FLASH_PAGE_PROGRAM, address, &data[..count]) {
            return Err(FlashError::Timeout);
        }
        address += count as u32;
        data = &data[count..];
    }
    Ok(())
}

/// Erase one 4 KiB sector.
pub fn erase_sector(address: u32) -> Result<(), FlashError> {
    if address & (SECTOR_SIZE - 1) != 0 {
        return Err(FlashError::UnalignedSector);
    }
    if address
        .checked_add(SECTOR_SIZE)
        .filter(|end| *end <= 0x0100_0000)
        .is_none()
    {
        return Err(FlashError::AddressOverflow);
    }
    ensure_safe_flash()?;
    if write_command(FLASH_SECTOR_ERASE, address, &[]) {
        Ok(())
    } else {
        Err(FlashError::Timeout)
    }
}

/// Read the JEDEC ID and map its capacity byte to a supported Telink
/// factory-data geometry.
pub fn detect_geometry() -> Result<FlashGeometry, FlashError> {
    let mut id = [0u8; 3];
    if !jedec_id(&mut id) {
        return Err(FlashError::Timeout);
    }
    FlashGeometry::from_jedec_capacity_code(id[2]).ok_or(FlashError::UnsupportedGeometry)
}

/// Fail closed if the fitted flash does not match the product-selected
/// geometry.
pub fn verify_geometry(expected: FlashGeometry) -> Result<(), FlashError> {
    if detect_geometry()? == expected {
        Ok(())
    } else {
        Err(FlashError::GeometryMismatch)
    }
}

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
fn factory_ieee_unchecked(geometry: FlashGeometry, address: &mut [u8; 8]) -> u8 {
    *address = [0xFFu8; 8];
    let read_ok = read_bytes(geometry.factory_ieee_address(), address);
    let all_ff = address[0] == 0xFF
        && address[1] == 0xFF
        && address[2] == 0xFF
        && address[3] == 0xFF
        && address[4] == 0xFF
        && address[5] == 0xFF
        && address[6] == 0xFF
        && address[7] == 0xFF;
    let all_zero = address[0] == 0
        && address[1] == 0
        && address[2] == 0
        && address[3] == 0
        && address[4] == 0
        && address[5] == 0
        && address[6] == 0
        && address[7] == 0;
    let valid = read_ok && !all_ff && !all_zero;
    if valid {
        return IEEE_SOURCE_FACTORY;
    }

    let mut uid = [0u8; 16];
    if read_uid(&mut uid) && uid_is_valid(&uid) {
        address[0] = uid[6];
        address[1] = uid[5];
        address[2] = uid[4];
        address[3] = uid[3];
        address[4] = uid[2];
        address[5] = uid[1];
        address[6] = uid[0];
        address[7] = 0x02;
        return IEEE_SOURCE_FLASH_UID;
    }

    *address = FALLBACK_IEEE;
    IEEE_SOURCE_FALLBACK
}

/// Verify the fitted flash geometry, then read its factory EUI-64.
///
/// This is the required entry point for non-512-KiB products. Verification
/// happens before the factory sector is read, so a product built for one
/// geometry cannot accidentally accept ordinary application bytes at
/// another geometry's factory-EUI address as a valid device identity.
pub fn factory_ieee_for(geometry: FlashGeometry, address: &mut [u8; 8]) -> Result<u8, FlashError> {
    verify_geometry(geometry)?;
    Ok(factory_ieee_unchecked(geometry, address))
}

/// Read the factory EUI-64 using the legacy 512 KiB Telink layout.
///
/// Existing TB-04 firmware uses 512 KiB flash, so this compatibility
/// wrapper preserves its behavior. Products with any other geometry must
/// call [`factory_ieee_for`] explicitly; silently probing the 512 KiB
/// address on a 1 MiB part can produce a plausible but incorrect identity.
pub fn factory_ieee(address: &mut [u8; 8]) -> u8 {
    factory_ieee_unchecked(FlashGeometry::KiB512, address)
}

#[inline(always)]
fn uid_is_valid(uid: &[u8; 16]) -> bool {
    let mut all_ff = true;
    let mut all_zero = true;
    let mut no_uid_pattern = true;
    let mut index = 0;
    while index < uid.len() {
        all_ff &= uid[index] == 0xFF;
        all_zero &= uid[index] == 0;
        no_uid_pattern &= uid[index] == if index & 1 == 0 { 0x51 } else { 0x01 };
        index += 1;
    }
    !all_ff && !all_zero && !no_uid_pattern
}
