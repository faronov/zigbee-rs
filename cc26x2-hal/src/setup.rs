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
const OSC_SRC_CLK_LF: u32 = 0x4;
const OSC_XOSC_HF: u32 = 0x1;
const OSC_RCOSC_LF: u32 = 0x2;
// `STAT0.SCLK_LF_SRC` values.
const LF_SRC_RCOSC_LF: u32 = 2;
const LF_SRC_XOSC_LF: u32 = 3;

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
    /// SCLK_LF reached neither the crystal nor RCOSC_LF (or the CCFG
    /// selects another LF option).
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

/// How long [`start_lf_clock`] waits for the 32.768 kHz crystal before it
/// falls back to RCOSC_LF, counted on the AON RTC from `trim_device()`
/// (where the boot ROM's `SetupAfterColdResetWakeupFromShutDownCfg3`
/// requested XOSC_LF from the CCFG).
///
/// TI documents no XOSC_LF start-up time for the CC13x2/CC26x2 and its
/// Power driver waits without bound (the `LFSRCDONE` oscillator interrupt,
/// standby disallowed meanwhile); tuning-fork crystals within the
/// datasheet's ESR/load limits typically qualify within a few hundred ms.
/// One second covers that with margin, keeps the boot of a board without a
/// working crystal short, and a crystal that is merely slower only costs LF
/// accuracy (RCOSC_LF is measured and compensated, see [`LfClock`]).
pub const XOSC_LF_TIMEOUT_MS: u32 = 1000;

/// SCLK_LF source after [`start_lf_clock`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LfSource {
    /// The 32.768 kHz crystal (CCFG XOSC_LF).
    Crystal,
    /// RCOSC_LF selected by the CCFG.
    RcOsc,
    /// RCOSC_LF because XOSC_LF did not qualify within the timeout.
    RcOscFallback,
}

/// Result of [`start_lf_clock`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LfClock {
    pub source: LfSource,
    /// Milliseconds since `trim_device()` until SCLK_LF switched.
    pub waited_ms: u32,
    /// RCOSC_LF only: its frequency offset from 32.768 kHz in ppm, measured
    /// against the CPU clock (XOSC_HF) and compensated in the RTC
    /// increment. `None` for the crystal or an implausible measurement
    /// (then the nominal increment is kept).
    pub rcosc_ppm: Option<i32>,
}

/// CPU clock (SCLK_HF from XOSC_HF) used for the RCOSC_LF measurement.
const CPU_HZ: u32 = 48_000_000;
/// SCLK_LF periods of the RCOSC_LF measurement (125 ms).
const LF_MEASURE_CYCLES: u32 = 4096;

/// RTC increment and ppm offset for an RCOSC_LF that ran `lf_cycles`
/// periods in `cpu_cycles` of a `cpu_hz` clock. `None` outside
/// 30..36 kHz (RCOSC_LF is trimmed to about 32.8 kHz).
pub const fn rcosc_compensation(
    lf_cycles: u32,
    cpu_cycles: u32,
    cpu_hz: u32,
) -> Option<(u32, i32)> {
    if lf_cycles == 0 || cpu_cycles == 0 {
        return None;
    }
    // f = lf_cycles * cpu_hz / cpu_cycles, in mHz.
    let f_mhz = (lf_cycles as u64 * cpu_hz as u64 * 1000) / cpu_cycles as u64;
    if f_mhz < 30_000_000 || f_mhz > 36_000_000 {
        return None;
    }
    // SUBSECINC = 2^38 / f (2^23 at 32.768 kHz).
    let inc = ((1u64 << 38) * cpu_cycles as u64) / (lf_cycles as u64 * cpu_hz as u64);
    let ppm = (f_mhz as i64 - 32_768_000) * 1_000_000 / 32_768_000;
    Some((inc as u32, ppm as i32))
}

/// Start SCLK_LF and give the RTC the matching increment.
///
/// - CCFG XOSC_LF: wait until the crystal qualifies; if it has not within
///   `xosc_timeout_ms` of `trim_device()` (AON RTC time), select RCOSC_LF
///   (`OSCClockSourceSet(OSC_SRC_CLK_LF, OSC_RCOSC_LF)`, what the ROM does
///   for a CCFG RCOSC_LF) and continue on it. A crystal that qualifies
///   later is not switched back to.
/// - CCFG RCOSC_LF: wait for it (it switches within milliseconds).
///
/// On RCOSC_LF the oscillator is measured once against XOSC_HF and the RTC
/// increment is compensated, so the Embassy time base stays within the
/// measurement error at the boot temperature; RCOSC_LF still drifts with
/// temperature afterwards (about 50 ppm/°C per the datasheet) and is not
/// recalibrated. Requires XOSC_HF ([`start_xosc_hf`]).
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn start_lf_clock(xosc_timeout_ms: u32) -> Result<LfClock, SetupError> {
    let crystal = match (mmio::read(CCFG_MODE_CONF) >> 22) & 0x3 {
        2 => true,
        3 => false,
        _ => return Err(SetupError::LfClock),
    };
    let wanted = if crystal {
        LF_SRC_XOSC_LF
    } else {
        LF_SRC_RCOSC_LF
    };
    let deadline = u64::from(xosc_timeout_ms) * 32_768 / 1000;
    let mut source = if crystal {
        LfSource::Crystal
    } else {
        LfSource::RcOsc
    };
    // The RTC runs from trim_device() on (31.25 kHz increment while
    // SCLK_LF is RCOSC_HF-derived), so it is the time base of the wait.
    while lf_source() != wanted {
        if crate::aon_rtc::now_ticks() >= deadline {
            if !crystal {
                return Err(SetupError::LfClock);
            }
            source = LfSource::RcOscFallback;
            break;
        }
        core::hint::spin_loop();
    }
    if source == LfSource::RcOscFallback {
        unsafe {
            call2(
                mmio::rom_fn(ROM_OSC_TABLE, OSC_CLOCK_SOURCE_SET),
                OSC_SRC_CLK_LF,
                OSC_RCOSC_LF,
            )
        };
        mmio::aon_sync();
        let limit = crate::aon_rtc::now_ticks() + deadline.max(32_768);
        while lf_source() != LF_SRC_RCOSC_LF {
            if crate::aon_rtc::now_ticks() >= limit {
                return Err(SetupError::LfClock);
            }
            core::hint::spin_loop();
        }
    }
    let waited_ms = (crate::aon_rtc::now_ticks() * 1000 / 32_768) as u32;
    let mut increment = SUBSECINC_32768_HZ;
    let mut rcosc_ppm = None;
    if source != LfSource::Crystal
        && let Some((inc, ppm)) = measure_lf()
    {
        increment = inc;
        rcosc_ppm = Some(ppm);
    }
    unsafe { call1(mmio::rom_fn(ROM_SETUP_TABLE, SETUP_SUBSECINC), increment) };
    mmio::aon_sync();
    Ok(LfClock {
        source,
        waited_ms,
        rcosc_ppm,
    })
}

/// Count CPU cycles (DWT CYCCNT, XOSC_HF) over [`LF_MEASURE_CYCLES`]
/// SCLK_LF periods, edge to edge (each period advances the RTC `SUBSEC`).
#[cfg(all(target_arch = "arm", target_os = "none"))]
fn measure_lf() -> Option<(u32, i32)> {
    const DEMCR: u32 = 0xE000_EDFC;
    const DWT_CTRL: u32 = 0xE000_1000;
    const DWT_CYCCNT: u32 = 0xE000_1004;
    const SUBSEC: u32 = crate::mmio::AON_RTC_BASE + 0x0C;
    mmio::modify(DEMCR, 0, 1 << 24);
    mmio::modify(DWT_CTRL, 0, 1);
    let edge = |last: u32| -> Option<u32> {
        // Bounded: one SCLK_LF period is ~1500 CPU cycles.
        for _ in 0..100_000u32 {
            let now = mmio::read(SUBSEC);
            if now != last {
                return Some(now);
            }
        }
        None
    };
    let mut value = edge(mmio::read(SUBSEC))?;
    let start = mmio::read(DWT_CYCCNT);
    for _ in 0..LF_MEASURE_CYCLES {
        value = edge(value)?;
    }
    let cycles = mmio::read(DWT_CYCCNT).wrapping_sub(start);
    rcosc_compensation(LF_MEASURE_CYCLES, cycles, CPU_HZ)
}

/// Software system reset (driverlib `SysCtrlSystemReset` via the CPU).
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rcosc_compensation_matches_the_ti_increments() {
        // Exactly 32.768 kHz: 4096 periods = 125 ms = 6 000 000 cycles.
        assert_eq!(
            rcosc_compensation(4096, 6_000_000, 48_000_000),
            Some((SUBSECINC_32768_HZ, 0))
        );
        // 31.25 kHz gives TI's SUBSECINC_31250_HZ (2^38 / 31250, truncated).
        let cycles = (4096u64 * 48_000_000 / 31_250) as u32;
        let (inc, ppm) = rcosc_compensation(4096, cycles, 48_000_000).unwrap();
        assert!(inc.abs_diff(SUBSECINC_31250_HZ) <= 1, "{inc:#X}");
        assert_eq!(ppm, -46_325);
        // RCOSC_LF at its typical 32.8 kHz runs ~977 ppm fast: smaller
        // increment.
        let cycles = (4096u64 * 48_000_000 / 32_800) as u32;
        let (inc, ppm) = rcosc_compensation(4096, cycles, 48_000_000).unwrap();
        assert!(inc < SUBSECINC_32768_HZ);
        assert!((970..=980).contains(&ppm), "{ppm}");
    }

    #[test]
    fn implausible_measurements_are_rejected() {
        assert_eq!(rcosc_compensation(4096, 0, 48_000_000), None);
        assert_eq!(rcosc_compensation(0, 6_000_000, 48_000_000), None);
        // A stuck SUBSEC (edge wait gave up) or a far-off clock.
        assert_eq!(rcosc_compensation(4096, 100, 48_000_000), None);
        assert_eq!(rcosc_compensation(4096, 60_000_000, 48_000_000), None);
    }
}
