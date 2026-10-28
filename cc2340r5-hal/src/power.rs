//! Chip-level power configuration for the CC2340R52.
//!
//! Applied once from [`crate::take_core`], before any other driver runs:
//!
//! * **DC/DC regulator for VDDR** when the board fits the DC/DC inductor
//!   ([`VddrRegulator::DcDc`]; the board crate decides). TI's generated
//!   `Board_init()`
//!   (`source/ti/drivers/.meta/power/PowerCC23X0.Board_init.c.xdt`) calls
//!   `PMCTLSetVoltageRegulator(PMCTL_VOLTAGE_REGULATOR_DCDC)` for it after
//!   `Power_init()`. The ROM leaves the GLDO selected. Measured on
//!   LP-EM-CC2340R5 (radio gated, CPU sleeping): 2.73 mA with GLDO -> 1.82 mA with DC/DC, and
//!   the +5 dBm poll peak dropped from ~19 mA to ~10.6 mA.
//! * **Flash LDO off in idle** (`CLKCTL.IDLECFG.MODE = LDO_OFF`). The flash is
//!   powered down while the CPU sleeps and restarted (a few µs) on wake.
//!   Measured: 1.82 mA -> 1.50 mA. The hardware requires that no DMA accesses
//!   flash in this mode; this firmware uses no DMA (the LRFD runs from its own
//!   RAM and the TX/RX FIFOs).
//! * **LFOSC (internal 32 kHz RC) as LF clock.** After reset
//!   `CKMD.LFCLKSEL` is `FAKE` (LF ticks derived from the HF clock), which
//!   stops in standby, so the RTC could never wake the device. TI's CCFG
//!   default is `LF XOSC`, but on the LP-EM-CC2340R5 the 32.768 kHz crystal
//!   does not run: with `PowerLPF3_selectLFXT()` mirrored, `LFCLKSTAT` stayed
//!   `0x01DE_8000` (GOOD=0, tick/inc source FAKE) with `RIS.LFCLKLOSS` and
//!   `RIS.LFCLKOOR` set, and the LaunchPad quick-start guide (SWRU588A) routes
//!   DIO3/DIO4 (`LFXT_P`/`LFXT_N`) to the BoosterPack header. [`select_lfosc`]
//!   therefore mirrors `PowerLPF3_selectLFOSC()` (`PowerCC23X0.c`), which
//!   qualified within milliseconds (`LFCLKSTAT` 0x805E_5555: GOOD, tick source
//!   LFCLK, increment source AVG ≈ 30.3 µs). Standby is refused until
//!   [`lf_clock_good`] (TI sets `PowerLPF3_DISALLOW_STANDBY` until the
//!   `LFCLKGOOD` interrupt).
//!
//!   Deviation from TI: `PowerCC23X0_standbyPolicy()`
//!   (`kernel/freertos/dpl/PowerCC23X0_freertos.c`) additionally waits for
//!   `LFCLKSTAT.FLTSETTLED` and `RIS.LFTICK` with LFOSC, busy-looping
//!   meanwhile. TI keeps HFXT running whenever the CPU is awake; this firmware
//!   only runs HFXT while the radio is on (a few ms per poll), and
//!   FLTSETTLED was never observed to set. The LFINC filter still tracks
//!   (`LFINCSRC = AVG`, `LFINCCTL.INT` updating) and the measured standby
//!   time base was 60.545 s against 60.54 s wall over one minute, so
//!   [`lf_clock_good`] gates on GOOD plus a real (non-FAKE) LF tick and
//!   increment source instead.

/// `PMCTL.VDDRCTL` (`hw_pmctl.h`: `PMCTL_BASE` 0x4000_0000 + 0x14).
pub const PMCTL_VDDRCTL: u32 = 0x4000_0014;
/// `PMCTL_VDDRCTL_SELECT_DCDC`.
pub const VDDRCTL_SELECT_DCDC: u32 = 0x0000_0001;
/// `CLKCTL.IDLECFG` (`hw_clkctl.h`: `CLKCTL_BASE` 0x4002_0000 + 0x48).
pub const CLKCTL_IDLECFG: u32 = 0x4002_0048;
/// `CLKCTL_IDLECFG_MODE_LDO_OFF`.
pub const IDLECFG_MODE_LDO_OFF: u32 = 0x0000_0001;

/// `CKMD.TRIM1` (`hw_ckmd.h`: `CKMD_BASE` 0x4000_1000 + 0x114).
pub const CKMD_TRIM1: u32 = 0x4000_1114;
/// `CKMD_TRIM1_NABIAS_LFOSC`: 150 nA LFOSC bias to minimise RTN.
pub const TRIM1_NABIAS_LFOSC: u32 = 0x0000_8000;
/// `CKMD.LFOSCCTL` (+0x8C).
pub const CKMD_LFOSCCTL: u32 = 0x4000_108C;
/// `CKMD_LFOSCCTL_EN`.
pub const LFOSCCTL_EN: u32 = 0x0000_0001;
/// `CKMD.LFINCCTL` (+0x98).
pub const CKMD_LFINCCTL: u32 = 0x4000_1098;
/// `CKMD_LFINCCTL_PREVENTSTBY_M`: LFINC filter settling blocks standby.
pub const LFINCCTL_PREVENTSTBY: u32 = 0x8000_0000;
/// `CKMD.LFCLKSEL` (+0xC0).
pub const CKMD_LFCLKSEL: u32 = 0x4000_10C0;
/// `CKMD_LFCLKSEL_MAIN_LFOSC`.
pub const LFCLKSEL_MAIN_LFOSC: u32 = 0x0000_0001;
/// `CKMD.LFCLKSTAT` (+0xE0).
pub const CKMD_LFCLKSTAT: u32 = 0x4000_10E0;
/// `CKMD_LFCLKSTAT_GOOD_M`: the selected LF clock is qualified.
pub const LFCLKSTAT_GOOD: u32 = 0x8000_0000;
/// `CKMD_LFCLKSTAT_LFTICKSRC_M`: set while LF ticks are FAKE (HF-derived).
pub const LFCLKSTAT_LFTICKSRC_FAKE: u32 = 0x0100_0000;
/// `CKMD_LFCLKSTAT_LFINCSRC_M` (bits 23:22) and its `FAKE` value.
pub const LFCLKSTAT_LFINCSRC_M: u32 = 0x00C0_0000;
/// `CKMD_LFCLKSTAT_LFINCSRC_FAKE`.
pub const LFCLKSTAT_LFINCSRC_FAKE: u32 = 0x00C0_0000;

/// VDDR regulator fitted on the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VddrRegulator {
    /// Keep the ROM default global LDO (no DC/DC inductor fitted).
    GlobalLdo,
    /// Switch to the DC/DC converter (`PMCTLSetVoltageRegulator(DCDC)`);
    /// only valid when the board populates the DC/DC inductor.
    DcDc,
}

/// Select the VDDR regulator, switch the flash LDO off during idle and
/// select LFOSC.
#[cfg(target_os = "none")]
pub(crate) fn init(regulator: VddrRegulator) {
    // SAFETY: fixed, always-present CC2340R52 PMCTL/CLKCTL registers; called
    // once from `take_core()` before any concurrent user exists.
    unsafe {
        if regulator == VddrRegulator::DcDc {
            let vddrctl = PMCTL_VDDRCTL as *mut u32;
            // PMCTLSetVoltageRegulator(): preserve the other VDDRCTL fields.
            vddrctl.write_volatile(vddrctl.read_volatile() | VDDRCTL_SELECT_DCDC);
        }
        (CLKCTL_IDLECFG as *mut u32).write_volatile(IDLECFG_MODE_LDO_OFF);
    }
    select_lfosc();
}

/// `PowerLPF3_selectLFOSC()`: LFOSC bias, LFCLKSEL = LFOSC, start LFOSC,
/// let the LFINC filter not block standby. Qualification completes in the
/// background (see [`lf_clock_good`]).
#[cfg(target_os = "none")]
fn select_lfosc() {
    // SAFETY: fixed CKMD registers; called once from `init()`.
    unsafe {
        let trim1 = CKMD_TRIM1 as *mut u32;
        trim1.write_volatile(trim1.read_volatile() | TRIM1_NABIAS_LFOSC);
        (CKMD_LFCLKSEL as *mut u32).write_volatile(LFCLKSEL_MAIN_LFOSC);
        (CKMD_LFOSCCTL as *mut u32).write_volatile(LFOSCCTL_EN);
        let lfincctl = CKMD_LFINCCTL as *mut u32;
        lfincctl.write_volatile(lfincctl.read_volatile() & !LFINCCTL_PREVENTSTBY);
    }
}

/// Whether an `LFCLKSTAT` value shows a qualified, real LF clock: GOOD, and
/// neither the tick nor the increment source is the HF-derived FAKE one.
pub const fn lf_status_good(lfclkstat: u32) -> bool {
    lfclkstat & LFCLKSTAT_GOOD != 0
        && lfclkstat & LFCLKSTAT_LFTICKSRC_FAKE == 0
        && lfclkstat & LFCLKSTAT_LFINCSRC_M != LFCLKSTAT_LFINCSRC_FAKE
}

/// Whether the selected LF clock (LFOSC) is running and qualified, i.e. the
/// RTC keeps time in standby and can wake the device. See the module docs
/// for why FLTSETTLED is not required.
#[cfg(target_os = "none")]
pub fn lf_clock_good() -> bool {
    lf_status_good(lf_clock_status())
}

/// Raw `CKMD.LFCLKSTAT`, for diagnostics.
#[cfg(target_os = "none")]
pub fn lf_clock_status() -> u32 {
    // SAFETY: read-only status register.
    unsafe { (CKMD_LFCLKSTAT as *const u32).read_volatile() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_addresses_match_the_cc23x0r5_headers() {
        assert_eq!(PMCTL_VDDRCTL, 0x4000_0000 + 0x14);
        assert_eq!(VDDRCTL_SELECT_DCDC, 1);
        assert_eq!(CLKCTL_IDLECFG, 0x4002_0000 + 0x48);
        assert_eq!(IDLECFG_MODE_LDO_OFF, 1);
        assert_eq!(CKMD_LFOSCCTL, 0x4000_1000 + 0x8C);
        assert_eq!(CKMD_LFINCCTL, 0x4000_1000 + 0x98);
        assert_eq!(CKMD_LFCLKSEL, 0x4000_1000 + 0xC0);
        assert_eq!(CKMD_LFCLKSTAT, 0x4000_1000 + 0xE0);
        assert_eq!(CKMD_TRIM1, 0x4000_1000 + 0x114);
    }

    #[test]
    fn lf_status_matches_the_values_read_on_hardware() {
        // LFXT selected, crystal absent: FAKE tick and increment sources.
        assert!(!lf_status_good(0x01DE_8000));
        // LFOSC qualified: GOOD, tick source LFCLK, increment source AVG.
        assert!(lf_status_good(0x805E_5555));
        // GOOD alone is not enough while ticks are still FAKE.
        assert!(!lf_status_good(LFCLKSTAT_GOOD | LFCLKSTAT_LFTICKSRC_FAKE));
        assert!(!lf_status_good(LFCLKSTAT_GOOD | LFCLKSTAT_LFINCSRC_FAKE));
    }
}
