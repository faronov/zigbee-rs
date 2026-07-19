//! RAM-resident TLSR8258 SPI-flash read, page-program, and sector-erase
//! operations. The application must initialize clocks before use.

#![cfg(target_arch = "tc32")]

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

const REG_MSPI_DATA: u32 = 0x80000C;
const REG_MSPI_CTRL: u32 = 0x80000D;
const REG_IRQ_EN: u32 = 0x800643;
const FACTORY_IEEE_ADDR: u32 = 0x0007_6000;
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    Timeout,
    AddressOverflow,
    UnalignedSector,
    BufferNotInRam,
    /// No voltage guard is registered (see [`set_voltage_guard`]), or the
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
    let previous = unsafe { core::ptr::read_volatile(REG_IRQ_EN as *const u8) };
    unsafe { core::ptr::write_volatile(REG_IRQ_EN as *mut u8, 0) };
    previous
}

#[inline(always)]
fn irq_restore(previous: u8) {
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

/// Outcome of a single voltage-guard reading attempt, returned by a
/// [`VoltageGuardFn`].
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

/// Voltage-guard callback signature. Wire this to a real ADC reading (e.g.
/// `adc::sample_with_fluctuation_mv`) via [`set_voltage_guard`].
///
/// The board-specific piece this crate cannot supply on its own is *which*
/// GPIO pin the ADC should sample to approximate flash-supply voltage —
/// see `adc.rs`'s module docs. Until the application calls
/// [`set_voltage_guard`] with a real reading wired to its own board, Zbit
/// program/erase calls are refused outright (the previous, more
/// conservative behavior of this module).
pub type VoltageGuardFn = fn() -> VoltageReading;

/// Storage for the registered [`VoltageGuardFn`], as the bit pattern of the
/// function pointer (`0` = none registered).
///
/// A `static mut` setter here would be unsound (safe code could call it
/// from two contexts and race the plain read/write); this crate has no
/// threads, but `AtomicUsize` gives a genuinely sound safe API for free
/// (single-instruction load/store, no critical section needed) rather than
/// pushing an `unsafe` requirement onto every call site of
/// [`set_voltage_guard`].
static VOLTAGE_GUARD: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Register the ADC-backed voltage guard used before Zbit flash
/// program/erase operations. Safe to call at any time (e.g. re-registered
/// after re-initializing the ADC); the most recent registration wins.
pub fn set_voltage_guard(guard: VoltageGuardFn) {
    VOLTAGE_GUARD.store(guard as usize, core::sync::atomic::Ordering::SeqCst);
}

fn voltage_guard() -> Option<VoltageGuardFn> {
    let raw = VOLTAGE_GUARD.load(core::sync::atomic::Ordering::SeqCst);
    if raw == 0 {
        return None;
    }
    // SAFETY: the only non-zero values ever stored are `guard as usize`
    // for a real `VoltageGuardFn` passed into `set_voltage_guard` — `usize`
    // is defined as this target's pointer-sized integer, so the round trip
    // through the same integer representation is valid.
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
    ensure_safe_flash()?;
    address
        .checked_add(data.len() as u32)
        .filter(|end| *end <= 0x0100_0000)
        .ok_or(FlashError::AddressOverflow)?;

    while !data.is_empty() {
        let page_remaining = PAGE_SIZE - (address as usize & (PAGE_SIZE - 1));
        let count = data.len().min(page_remaining);
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

#[inline(never)]
#[unsafe(link_section = ".ram_code")]
pub fn factory_ieee(address: &mut [u8; 8]) -> u8 {
    *address = [0xFFu8; 8];
    let read_ok = read_bytes(FACTORY_IEEE_ADDR, address);
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
