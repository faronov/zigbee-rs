//! Private volatile MMIO helpers.

#[inline(always)]
pub(crate) fn read32(address: u32) -> u32 {
    // SAFETY: Callers provide aligned BL702 MMIO or memory-mapped flash
    // addresses and retain ownership of the corresponding peripheral token.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline(always)]
pub(crate) fn write32(address: u32, value: u32) {
    // SAFETY: Callers provide aligned BL702 MMIO addresses and retain
    // ownership of the corresponding peripheral token.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[inline(always)]
pub(crate) fn rmw(address: u32, mask: u32, value: u32) {
    write32(address, (read32(address) & !mask) | (value & mask));
}
