//! Volatile register access and the memory map (SDK `inc/hw_memmap.h`).

#![allow(dead_code)]

pub const FLASH_BASE: u32 = 0x4003_0000;
pub const VIMS_BASE: u32 = 0x4003_4000;
pub const UART0_BASE: u32 = 0x4000_1000;
pub const GPIO_BASE: u32 = 0x4002_2000;
pub const CRYPTO_BASE: u32 = 0x4002_4000;
pub const TRNG_BASE: u32 = 0x4002_8000;
pub const RFC_PWR_BASE: u32 = 0x4004_0000;
pub const RFC_DBELL_BASE: u32 = 0x4004_1000;
pub const IOC_BASE: u32 = 0x4008_1000;
pub const PRCM_BASE: u32 = 0x4008_2000;
pub const ADI3_BASE: u32 = 0x4008_6200;
pub const AON_PMCTL_BASE: u32 = 0x4009_0000;
pub const AON_RTC_BASE: u32 = 0x4009_2000;
pub const AON_IOC_BASE: u32 = 0x4009_4000;
pub const AUX_SYSIF_BASE: u32 = 0x400C_6000;
pub const AUX_DDI0_OSC_BASE: u32 = 0x400C_A000;
pub const AUX_ADI4_BASE: u32 = 0x400C_B000;
pub const FCFG1_BASE: u32 = 0x5000_1000;
/// CCFG alias of the last 88 bytes of flash (independent of flash size).
pub const CCFG_BASE: u32 = 0x5000_3000;
/// RF core RAM (CPE patches).
pub const RFC_RAM_BASE: u32 = 0x2100_0000;

/// Driverlib ROM API table (`driverlib/rom.h`).
pub const ROM_API_TABLE: u32 = 0x1000_0180;
/// Boot ROM hardware API table (`driverlib/rom.h` `HARD_API_T`).
pub const ROM_HAPI_TABLE: u32 = 0x1000_0048;

#[inline(always)]
pub fn read(addr: u32) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(addr as *const u32)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = addr;
        0
    }
}

#[inline(always)]
pub fn write(addr: u32, value: u32) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile(addr as *mut u32, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (addr, value);
}

#[inline(always)]
pub fn read16(addr: u32) -> u16 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(addr as *const u16)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = addr;
        0
    }
}

#[inline(always)]
pub fn write16(addr: u32, value: u16) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile(addr as *mut u16, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (addr, value);
}

#[inline(always)]
pub fn read8(addr: u32) -> u8 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(addr as *const u8)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = addr;
        0
    }
}

#[inline(always)]
pub fn write8(addr: u32, value: u8) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile(addr as *mut u8, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (addr, value);
}

#[inline(always)]
pub fn modify(addr: u32, clear: u32, set: u32) {
    write(addr, (read(addr) & !clear) | set);
}

/// Entry `index` of the driverlib ROM sub-table `table` (`ROM_API_TABLE[table]`).
#[inline(always)]
pub fn rom_fn(table: u32, index: u32) -> u32 {
    read(read(ROM_API_TABLE + table * 4) + index * 4)
}

/// Entry `index` of the boot ROM HAPI table.
#[inline(always)]
pub fn hapi_fn(index: u32) -> u32 {
    read(ROM_HAPI_TABLE + index * 4)
}

/// Read AON_RTC:SYNC so that every earlier AON write has reached the AON
/// domain (driverlib `SysCtrlAonSync`).
#[inline(always)]
pub fn aon_sync() {
    let _ = read(AON_RTC_BASE + 0x2C);
}
