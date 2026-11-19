//! Device trim and clock start-up.
//!
//! [`trim_device`] is driverlib `SetupTrimDevice()` (`driverlib/setup.c`,
//! SDK 8.33.00.16, identical for both families except the cap-array
//! workaround, which [`crate::ccfg::Ccfg::check_recoverable`] keeps
//! disabled). The trim and clock helpers it calls are the boot ROM's
//! driverlib table entries (`ROM_API_SETUP_ROM_TABLE`, `ROM_API_OSC_TABLE`),
//! the same code TI's start-up reaches through `rom.h`; the HF clock source
//! switch uses the ROM HAPI `HFSourceSafeSwitch` because it must not run from
//! flash.

use crate::mmio::{self, AUX_DDI0_OSC_BASE, CCFG_BASE};
#[cfg(all(target_arch = "arm", target_os = "none"))]
use crate::mmio::{
    ADI3_BASE, AON_IOC_BASE, AON_PMCTL_BASE, AON_RTC_BASE, AUX_ADI4_BASE, AUX_SYSIF_BASE,
    FCFG1_BASE, FLASH_BASE, PRCM_BASE, VIMS_BASE,
};

// ROM_API_TABLE sub-tables (`driverlib/rom.h`).
const ROM_OSC_TABLE: u32 = 24;
const ROM_SETUP_TABLE: u32 = 28;
// ROM_API_SETUP_ROM_TABLE entries.
const SETUP_CFG1: u32 = 0;
const SETUP_CFG2: u32 = 1;
const SETUP_CFG3: u32 = 2;
const SETUP_SUBSECINC: u32 = 17;
const SETUP_CACHE_MODE: u32 = 18;
// ROM_API_OSC_TABLE entries.
const OSC_CLOCK_SOURCE_SET: u32 = 1;
const OSC_CLOCK_SOURCE_GET: u32 = 0;
// HAPI entries.
const HAPI_HF_SOURCE_SAFE_SWITCH: u32 = 14;

const OSC_SRC_CLK_HF: u32 = 0x1;
const OSC_XOSC_HF: u32 = 0x1;

const CCFG_MODE_CONF_1: u32 = CCFG_BASE + 0x1FAC;
const CCFG_SIZE_AND_DIS_FLAGS: u32 = CCFG_BASE + 0x1FB0;
const CCFG_MODE_CONF: u32 = CCFG_BASE + 0x1FB4;

const ADI_O_SET: u32 = 0x10;
const ADI_O_CLR: u32 = 0x20;
const ADI_O_MASK4B: u32 = 0x40;
const ADI_O_MASK8B: u32 = 0x60;
const DDI_O_MASK16B: u32 = 0x400;
const DDI_0_OSC_CTL0: u32 = 0x00;
const DDI_0_OSC_STAT0: u32 = 0x3C;
const CLK_DCDC_SRC_SEL_M: u32 = 0x0100_0000;

/// 32.768 kHz RTC increment (`SUBSECINC_32768_HZ`).
pub const SUBSECINC_32768_HZ: u32 = 0x0080_0000;
/// RTC increment while SCLK_LF still runs from RCOSC_HF/1536 (31.25 kHz).
pub const SUBSECINC_31250_HZ: u32 = 0x0086_37BD;

/// Start-up failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetupError {
    /// The silicon does not match the compiled chip feature.
    Chip(crate::chip::ChipMismatch),
    /// CCFG enables the XOSC cap-array workaround, which is not implemented.
    UnsupportedCcfg,
    /// XOSC_HF never became ready.
    XoscHf,
    /// The LF clock never switched to the CCFG-selected source.
    LfClock,
}

unsafe fn call0(addr: u32) {
    let f: extern "C" fn() = unsafe { core::mem::transmute(addr as usize) };
    f()
}

unsafe fn call1(addr: u32, a: u32) {
    let f: extern "C" fn(u32) = unsafe { core::mem::transmute(addr as usize) };
    f(a)
}

unsafe fn call2(addr: u32, a: u32, b: u32) {
    let f: extern "C" fn(u32, u32) = unsafe { core::mem::transmute(addr as usize) };
    f(a, b)
}

unsafe fn call1_ret(addr: u32, a: u32) -> u32 {
    let f: extern "C" fn(u32) -> u32 = unsafe { core::mem::transmute(addr as usize) };
    f(a)
}

/// `SetupTrimDevice()`: factory trims, regulator/DC-DC and XOSC set-up from
/// FCFG1 and CCFG, cache mode, AON RTC start. Call once, first thing in
/// `main`, before any clock or peripheral is touched.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn trim_device() -> Result<(), SetupError> {
    crate::chip::Identity::read()
        .check_compiled()
        .map_err(SetupError::Chip)?;
    let flags = mmio::read(CCFG_SIZE_AND_DIS_FLAGS);
    if matches!(crate::chip::FAMILY, crate::chip::Family::X2) && flags & (1 << 4) == 0 {
        // OSC_CapArrayAdjustWorkaround_Boot() would be required.
        return Err(SetupError::UnsupportedCcfg);
    }

    let mut fcfg1_revision = mmio::read(FCFG1_BASE + 0x31C);
    if fcfg1_revision == 0xFFFF_FFFF {
        fcfg1_revision = 0;
    }

    // Enable standby in the flash bank.
    mmio::modify(FLASH_BASE + 0x24, 1 << 1, 0);
    unsafe { call0(mmio::rom_fn(ROM_SETUP_TABLE, SETUP_CACHE_MODE)) };

    let latched = mmio::read(AON_IOC_BASE + 0x0C) & 1 != 0;
    let io_pad_sleep_dis = mmio::read(AON_PMCTL_BASE + 0x2C) & 1 != 0;
    if !latched {
        // Wake from power-down: no trim (TrimAfter...PowerDown is empty).
    } else if !io_pad_sleep_dis {
        trim_after_shutdown(fcfg1_revision);
    } else {
        // Cold reset: TrimAfterColdReset() is empty.
        trim_after_shutdown(fcfg1_revision);
    }

    // VIMS domain powered only with the CPU domain.
    mmio::write(PRCM_BASE + 0x18C, 0);
    // Flash pump sleep exit wait time.
    mmio::modify(FLASH_BASE + 0x2048, 0x0FFF_0000, 0x139 << 16);

    // BOOT_DET 1 -> 3 at the end of the flash boot.
    let resetctl = mmio::read(AON_PMCTL_BASE + 0x28);
    if (resetctl & (0x3 << 12)) >> 12 == 1 {
        let base = resetctl & !(0x0200_0000 | 0x0100_0000 | 0x0002_0000 | 0x0001_0000 | 0x10);
        mmio::write(AON_PMCTL_BASE + 0x28, base | 0x0002_0000);
        mmio::write(AON_PMCTL_BASE + 0x28, base);
    }

    // Reset and start the AON RTC with the CH0..CH2 combined event.
    mmio::modify(AON_RTC_BASE, 0, 1 << 7);
    cortex_m::peripheral::NVIC::unpend(crate::vectors::Interrupt::AON_RTC_COMB);
    mmio::modify(AON_RTC_BASE, 0x7 << 16, 0x7 << 16);
    mmio::modify(AON_RTC_BASE, 0, 1);
    if (mmio::read(CCFG_MODE_CONF) >> 22) & 0x3 == 2 {
        // SCLK_LF still runs from RCOSC_HF until the crystal is up.
        unsafe {
            call1(
                mmio::rom_fn(ROM_SETUP_TABLE, SETUP_SUBSECINC),
                SUBSECINC_31250_HZ,
            )
        };
    }
    mmio::aon_sync();

    // No VIMS mode change may be in progress when leaving.
    crate::prcm::wait(|| mmio::read(VIMS_BASE) & (1 << 3) == 0);
    Ok(())
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
fn trim_after_shutdown(fcfg1_revision: u32) {
    // Alternative DC/DC setting from CCFG (DITHER_EN, IPEAK) as one 4-bit
    // masked write; driverlib truncates the value to a byte.
    if mmio::read(CCFG_SIZE_AND_DIS_FLAGS) & (1 << 1) == 0 {
        let value = (0xF0 | (mmio::read(CCFG_MODE_CONF_1) >> 16)) as u8;
        mmio::write8(ADI3_BASE + ADI_O_MASK4B + 0x0B * 2, value);
    }

    // Force the DC/DC onto RCOSC until SCLK_HF has switched to XOSC.
    mmio::write(
        AUX_DDI0_OSC_BASE + DDI_O_MASK16B + (DDI_0_OSC_CTL0 << 1) + 4,
        CLK_DCDC_SRC_SEL_M | (CLK_DCDC_SRC_SEL_M >> 16),
    );
    let _ = mmio::read16(AUX_DDI0_OSC_BASE + DDI_0_OSC_CTL0);

    let mode_conf = mmio::read(CCFG_MODE_CONF);
    unsafe { call1(mmio::rom_fn(ROM_SETUP_TABLE, SETUP_CFG1), mode_conf) };

    // VDDR boost: VDDR_EXT_LOAD = 0 and VDDS_BOD_LEVEL = 1.
    if mode_conf & (1 << 25) == 0 && mode_conf & (1 << 24) != 0 {
        let reg = ADI3_BASE + 0x09;
        mmio::write8(reg, mmio::read8(reg) | 0x02);
    }

    unsafe {
        call2(
            mmio::rom_fn(ROM_SETUP_TABLE, SETUP_CFG2),
            fcfg1_revision,
            mode_conf,
        )
    };

    // LPM bias trims from FCFG1:DAC_BIAS_CNF.
    let trim = mmio::read(FCFG1_BASE + 0x40C);
    let iout = (trim & 0x0003_F000) >> 12;
    mmio::write8(AUX_ADI4_BASE + 0x0E, (iout & 0x3F) as u8);
    if trim & 0x100 != 0 {
        mmio::write8(ADI3_BASE + ADI_O_SET + 0x0C, 0x40);
    } else {
        mmio::write8(ADI3_BASE + ADI_O_CLR + 0x0C, 0x40);
    }
    let width = (trim & 0x0E00) >> 9;
    mmio::write16(
        AUX_ADI4_BASE + ADI_O_MASK8B + 0x05 * 2,
        (0x38 << 8) | (width << 3) as u16,
    );

    unsafe { call1(mmio::rom_fn(ROM_SETUP_TABLE, SETUP_CFG3), mode_conf) };

    aux_opmode_change(2); // power-down active (PDA)
    // Disable the eFuse clock.
    mmio::modify(FLASH_BASE + 0x24, 0, 1 << 5);
}

/// `AUXSYSIFOpModeChange()`: step the AUX operational mode one level at a
/// time (order PDLP < PDA < A < LP is encoded by the driverlib tables).
#[cfg(all(target_arch = "arm", target_os = "none"))]
fn aux_opmode_change(target: u32) {
    const MODE_TO_ORDER: [u32; 4] = [1, 2, 0, 3];
    const ORDER_TO_MODE: [u32; 4] = [2, 0, 1, 3];
    loop {
        let current = mmio::read(AUX_SYSIF_BASE) & 0x3;
        crate::prcm::wait(|| mmio::read(AUX_SYSIF_BASE + 4) & 0x3 == current);
        if current == target {
            return;
        }
        let order = MODE_TO_ORDER[current as usize];
        let next = if order < MODE_TO_ORDER[target as usize] {
            ORDER_TO_MODE[order as usize + 1]
        } else {
            ORDER_TO_MODE[order as usize - 1]
        };
        mmio::write(AUX_SYSIF_BASE, next);
    }
}

/// Switch SCLK_HF to the 48 MHz crystal (required by the RF core). Blocks
/// until the crystal is ready; bounded.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn start_xosc_hf() -> Result<(), SetupError> {
    let get = mmio::rom_fn(ROM_OSC_TABLE, OSC_CLOCK_SOURCE_GET);
    if unsafe { call1_ret(get, OSC_SRC_CLK_HF) } == OSC_XOSC_HF {
        return Ok(());
    }
    unsafe {
        call2(
            mmio::rom_fn(ROM_OSC_TABLE, OSC_CLOCK_SOURCE_SET),
            OSC_SRC_CLK_HF,
            OSC_XOSC_HF,
        )
    };
    if !crate::prcm::wait(|| mmio::read16(AUX_DDI0_OSC_BASE + DDI_0_OSC_STAT0) & 1 != 0) {
        return Err(SetupError::XoscHf);
    }
    // OSCHfSourceSwitch(): the switch itself must execute from ROM.
    cortex_m::interrupt::free(|_| unsafe { call0(mmio::hapi_fn(HAPI_HF_SOURCE_SAFE_SWITCH)) });
    // DC/DC clock back to "auto" now that SCLK_HF runs from XOSC.
    mmio::write(
        AUX_DDI0_OSC_BASE + DDI_O_MASK16B + (DDI_0_OSC_CTL0 << 1) + 4,
        CLK_DCDC_SRC_SEL_M,
    );
    if unsafe { call1_ret(get, OSC_SRC_CLK_HF) } != OSC_XOSC_HF {
        return Err(SetupError::XoscHf);
    }
    Ok(())
}

/// `STAT0.SCLK_LF_SRC`: 0 RCOSC_HF/1536, 1 XOSC_HF/1536, 2 RCOSC_LF, 3 XOSC_LF.
pub fn lf_source() -> u32 {
    (mmio::read(AUX_DDI0_OSC_BASE + DDI_0_OSC_STAT0) >> 29) & 0x3
}

/// Wait until SCLK_LF runs from the CCFG-selected 32 kHz source (crystal
/// or RCOSC_LF), then give the RTC its 32.768 kHz increment. `spins`
/// bounds the wait (the crystal needs up to a few hundred ms).
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn wait_lf_clock(spins: u32) -> Result<u32, SetupError> {
    let wanted = match (mmio::read(CCFG_MODE_CONF) >> 22) & 0x3 {
        2 => 3, // XOSC_LF
        3 => 2, // RCOSC_LF
        _ => return Err(SetupError::LfClock),
    };
    let mut n = 0;
    while lf_source() != wanted {
        n += 1;
        if n > spins {
            return Err(SetupError::LfClock);
        }
        core::hint::spin_loop();
    }
    if wanted == 3 {
        unsafe {
            call1(
                mmio::rom_fn(ROM_SETUP_TABLE, SETUP_SUBSECINC),
                SUBSECINC_32768_HZ,
            )
        };
        mmio::aon_sync();
    }
    Ok(wanted)
}

/// Software system reset (driverlib `SysCtrlSystemReset` via the CPU).
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}
