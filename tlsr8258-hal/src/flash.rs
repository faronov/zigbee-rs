//! RAM-resident TLSR8258 SPI-flash read, page-program, and sector-erase
//! operations. The application must initialize clocks before use.

#![cfg(target_arch = "tc32")]

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
    VoltageGuardRequired,
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
    let start = data.as_ptr() as usize;
    let end = start
        .checked_add(data.len())
        .ok_or(FlashError::AddressOverflow)?;
    if start < 0x0084_0000 || end > 0x0085_0000 {
        return Err(FlashError::BufferNotInRam);
    }
    Ok(())
}

fn ensure_safe_flash() -> Result<(), FlashError> {
    let mut id = [0u8; 3];
    if !jedec_id(&mut id) {
        return Err(FlashError::Timeout);
    }
    // The official SDK requires an ADC voltage/fluctuation guard for Zbit
    // ZB25WD40/80 parts. Refuse destructive operations until that guard is
    // available rather than writing them unsafely.
    if id == [0x5E, 0x32, 0x13] || id == [0x5E, 0x32, 0x14] {
        return Err(FlashError::VoltageGuardRequired);
    }
    Ok(())
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
