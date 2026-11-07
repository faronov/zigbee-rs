//! Power-down flags and the PMU power register words they select
//! (ESP-IDF v5.5.1 H2 `PMU_SLEEP_POWER_CONFIG_DEFAULT`, `pmu_sleep_power_init`).

/// `PMU_SLEEP_PD_TOP`: peripheral/TOP domain.
pub(crate) const PD_TOP: u32 = 1 << 0;
/// `PMU_SLEEP_PD_VDDSDIO`: flash supply.
pub(crate) const PD_VDDSDIO: u32 = 1 << 1;
/// `PMU_SLEEP_PD_MODEM`: radio MAC/baseband domain.
pub(crate) const PD_MODEM: u32 = 1 << 2;
/// `PMU_SLEEP_PD_CPU`: CPU core, interrupt controller and cache controller.
pub(crate) const PD_CPU: u32 = 1 << 3;
/// `PMU_SLEEP_PD_XTAL`: 32 MHz crystal.
pub(crate) const PD_XTAL: u32 = 1 << 4;
/// `PMU_SLEEP_PD_RC_FAST`: 8 MHz RC oscillator.
pub(crate) const PD_RC_FAST: u32 = 1 << 5;
/// `PMU_SLEEP_PD_XTAL32K`: 32 kHz crystal.
pub(crate) const PD_XTAL32K: u32 = 1 << 6;

/// Every power-down flag deep sleep sets.
pub(crate) const PD_DEEP: u32 =
    PD_TOP | PD_VDDSDIO | PD_MODEM | PD_CPU | PD_XTAL | PD_RC_FAST | PD_XTAL32K;

/// Oscillators a light sleep may stop in addition to its mode's domains.
///
/// Stopping XTAL also stops SYSTIMER; the caller's calibrated RTC accounting
/// must cover that.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PowerDown {
    /// Stop the 32 MHz main crystal (`PMU_SLEEP_PD_XTAL`).
    pub xtal: bool,
    /// Stop the RC_FAST oscillator (`PMU_SLEEP_PD_RC_FAST`).
    pub rc_fast: bool,
}

impl PowerDown {
    /// Keep every oscillator running.
    pub const NONE: Self = Self {
        xtal: false,
        rc_fast: false,
    };
    /// ESP-IDF's default light-sleep oscillator set: XTAL and RC_FAST off.
    pub const OSCILLATORS: Self = Self {
        xtal: true,
        rc_fast: true,
    };

    /// `PMU_SLEEP_PD_*` bits (pd_xtal = bit 4, pd_rc_fast = bit 5).
    pub const fn flags(self) -> u32 {
        (if self.xtal { PD_XTAL } else { 0 }) | (if self.rc_fast { PD_RC_FAST } else { 0 })
    }
}

/// RTC slow clock source (`LP_CLKRST.lp_clk_conf.slow_clk_sel`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SlowClock {
    RcSlow,
    Xtal32k,
    Rc32k,
    OscSlow,
}

impl SlowClock {
    pub(crate) fn from_sel(sel: u8) -> Self {
        if sel == 1 {
            Self::Xtal32k
        } else if sel == 2 {
            Self::Rc32k
        } else if sel == 3 {
            Self::OscSlow
        } else {
            Self::RcSlow
        }
    }
}

// HP_SLEEP dig power (`pmu_hp_dig_power_reg_t`).
const HP_VDD_SPI_PD_EN: u32 = 1 << 21;
const HP_MODEM_PD_EN: u32 = 1 << 27;
const HP_CPU_PD_EN: u32 = 1 << 29;
const HP_TOP_PD_EN: u32 = 1 << 31;
// HP_SLEEP clock power (`pmu_hp_clk_power_reg_t`).
const HP_I2C_ISO_EN: u32 = 1 << 26;
const HP_I2C_RETENTION: u32 = 1 << 27;
// LP dig power (`pmu_lp_dig_power_reg_t`).
const LP_MEM_DSLP: u32 = 1 << 30;
// LP clock power (`pmu_lp_clk_power_reg_t`).
const LP_XPD_XTAL32K: u32 = 1 << 28;
const LP_XPD_RC32K: u32 = 1 << 29;
const LP_XPD_FOSC: u32 = 1 << 30;

/// Whole-register values `pmu_sleep_power_init` writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PowerWords {
    /// `hp_sleep_dig_power`.
    pub hp_dig: u32,
    /// `hp_sleep_hp_ck_power` (BBPLL and its I2C off; analog I2C isolated
    /// and retained).
    pub hp_clk: u32,
    /// `hp_sleep_xtal.hp_sleep_xpd_xtal`.
    pub hp_xpd_xtal: bool,
    /// `hp_sleep_lp_dig_power` (LP_ACTIVE).
    pub lp_active_dig: u32,
    /// `hp_sleep_lp_ck_power` (LP_ACTIVE).
    pub lp_active_clk: u32,
    /// `lp_sleep_lp_dig_power`.
    pub lp_sleep_dig: u32,
    /// `lp_sleep_lp_ck_power`.
    pub lp_sleep_clk: u32,
    /// `lp_sleep_xtal.lp_sleep_xpd_xtal`.
    pub lp_xpd_xtal: bool,
}

/// `PMU_SLEEP_POWER_CONFIG_DEFAULT` with `PMU_SLEEP_PD_XTAL32K`/`RC32K` set
/// unless that oscillator is the RTC slow clock (as IDF forces it on). The
/// VBAT brownout source is a deep-sleep-on-VBAT option only, so it stays 0.
pub(crate) fn power_words(pd: u32, slow: SlowClock) -> PowerWords {
    let xtal32k = slow == SlowClock::Xtal32k;
    let rc32k = slow == SlowClock::Rc32k;
    let bit = |set: bool, mask: u32| if set { mask } else { 0 };
    PowerWords {
        hp_dig: bit(pd & PD_VDDSDIO != 0, HP_VDD_SPI_PD_EN)
            | bit(pd & PD_MODEM != 0, HP_MODEM_PD_EN)
            | bit(pd & PD_CPU != 0, HP_CPU_PD_EN)
            | bit(pd & PD_TOP != 0, HP_TOP_PD_EN),
        hp_clk: HP_I2C_ISO_EN | HP_I2C_RETENTION,
        hp_xpd_xtal: pd & PD_XTAL == 0,
        lp_active_dig: 0,
        lp_active_clk: bit(xtal32k, LP_XPD_XTAL32K) | bit(rc32k, LP_XPD_RC32K) | LP_XPD_FOSC,
        lp_sleep_dig: LP_MEM_DSLP,
        lp_sleep_clk: bit(xtal32k && pd & PD_XTAL32K == 0, LP_XPD_XTAL32K)
            | bit(rc32k, LP_XPD_RC32K)
            | bit(pd & PD_RC_FAST == 0, LP_XPD_FOSC),
        lp_xpd_xtal: pd & PD_XTAL == 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flags_use_the_idf_bit_positions() {
        assert_eq!(PowerDown::OSCILLATORS.flags(), (1 << 4) | (1 << 5));
        assert_eq!(PowerDown::NONE.flags(), 0);
        assert_eq!(PD_DEEP, 0x7F);
    }

    #[test]
    fn retained_oscillator_sleep_keeps_every_domain_powered() {
        let w = power_words(PD_XTAL | PD_RC_FAST, SlowClock::RcSlow);
        assert_eq!(w.hp_dig, 0);
        assert_eq!(w.hp_clk, (1 << 26) | (1 << 27));
        assert!(!w.hp_xpd_xtal && !w.lp_xpd_xtal);
        assert_eq!(w.lp_active_dig, 0);
        assert_eq!(w.lp_active_clk, 1 << 30);
        assert_eq!(w.lp_sleep_dig, 1 << 30);
        assert_eq!(w.lp_sleep_clk, 0);
    }

    #[test]
    fn oscillators_kept_stay_powered_in_lp_sleep() {
        let w = power_words(0, SlowClock::RcSlow);
        assert!(w.hp_xpd_xtal && w.lp_xpd_xtal);
        assert_eq!(w.lp_sleep_clk, 1 << 30);
    }

    #[test]
    fn domain_flags_map_to_the_hp_dig_power_bits() {
        assert_eq!(power_words(PD_TOP, SlowClock::RcSlow).hp_dig, 1 << 31);
        assert_eq!(power_words(PD_CPU, SlowClock::RcSlow).hp_dig, 1 << 29);
        assert_eq!(power_words(PD_MODEM, SlowClock::RcSlow).hp_dig, 1 << 27);
        assert_eq!(power_words(PD_VDDSDIO, SlowClock::RcSlow).hp_dig, 1 << 21);
        let top = power_words(PD_TOP | PD_CPU | PD_MODEM, SlowClock::RcSlow).hp_dig;
        assert_eq!(top, (1 << 31) | (1 << 29) | (1 << 27));
    }

    #[test]
    fn a_32k_slow_clock_stays_powered_unless_deep_sleep_stops_it() {
        let w = power_words(PD_XTAL | PD_RC_FAST, SlowClock::Xtal32k);
        assert_eq!(w.lp_active_clk, (1 << 28) | (1 << 30));
        assert_eq!(w.lp_sleep_clk, 1 << 28);
        assert_eq!(power_words(PD_DEEP, SlowClock::Xtal32k).lp_sleep_clk, 0);
        let rc = power_words(PD_DEEP, SlowClock::Rc32k);
        assert_eq!(rc.lp_active_clk, (1 << 29) | (1 << 30));
        assert_eq!(rc.lp_sleep_clk, 1 << 29);
    }

    #[test]
    fn slow_clock_selector_decodes_every_value() {
        assert_eq!(SlowClock::from_sel(0), SlowClock::RcSlow);
        assert_eq!(SlowClock::from_sel(1), SlowClock::Xtal32k);
        assert_eq!(SlowClock::from_sel(2), SlowClock::Rc32k);
        assert_eq!(SlowClock::from_sel(3), SlowClock::OscSlow);
    }
}
