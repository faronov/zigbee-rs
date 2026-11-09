//! PMU sleep regulator/bias settings (ESP-IDF v5.5.1 H2 `pmu_param.c`,
//! `pmu_sleep_config_default`).

use crate::power::{PD_RC_FAST, PD_XTAL};

/// HP/LP sleep regulator and bias settings (`pmu_sleep_analog_config_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SleepAnalog {
    pub hp_xpd_trx: bool,
    pub hp_pd_cur: bool,
    pub hp_bias_sleep: bool,
    pub hp_xpd: bool,
    pub hp_dbias: u8,
    pub hp_drv_b: u32,
    pub lp_pd_cur: bool,
    pub lp_bias_sleep: bool,
    pub lp_slp_xpd: bool,
    pub lp_slp_dbias: u8,
    pub lp_xpd: bool,
    pub lp_dbias: u8,
    pub lp_drv_b: u8,
}

/// Raw H2 eFuse inputs of the PMU voltage selection (`pmu_param.c`).
///
/// The product reads these once (`ACTIVE_HP_DBIAS`, `ACTIVE_LP_DBIAS`,
/// `DSLP_DBIAS` and the block version); the sleep path never touches eFuse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DbiasFuses {
    pub blk_major: u8,
    pub blk_minor: u8,
    pub active_hp: u8,
    pub active_lp: u8,
    pub deep_sleep: u8,
}

impl DbiasFuses {
    const HP_CALI_DBIAS_DEFAULT: u8 = 17;
    const LP_CALI_DBIAS_DEFAULT: u8 = 18;
    const LP_DBIAS_SLEEP_0V7_DEFAULT: u8 = 6;

    fn calibrated(&self) -> bool {
        u32::from(self.blk_major) * 100 + u32::from(self.blk_minor) >= 3
    }

    fn active(&self, fuse: u8, default: u8) -> u8 {
        if self.calibrated() && fuse != 0 {
            (fuse + 2).min(31)
        } else {
            default
        }
    }

    /// `get_act_hp_dbias`.
    pub fn act_hp(&self) -> u8 {
        self.active(self.active_hp, Self::HP_CALI_DBIAS_DEFAULT)
    }

    /// `get_act_lp_dbias`.
    pub fn act_lp(&self) -> u8 {
        self.active(self.active_lp, Self::LP_CALI_DBIAS_DEFAULT)
    }

    /// `get_slp_lp_dbias`.
    pub fn slp_lp(&self) -> u8 {
        if self.calibrated() && self.deep_sleep != 0 {
            self.deep_sleep
        } else {
            Self::LP_DBIAS_SLEEP_0V7_DEFAULT
        }
    }
}

/// ESP-IDF v5.5.1 H2 `pmu_sleep_config_default` light-sleep analog branch.
/// It depends only on the XTAL/RC_FAST power-down flags (not on TOP).
pub(crate) fn h2_light_sleep_analog(power_down: u32, fuses: &DbiasFuses) -> SleepAnalog {
    // PMU_SLEEP_ANALOG_LSLP_CONFIG_DEFAULT
    let mut analog = SleepAnalog {
        hp_xpd_trx: false,
        hp_pd_cur: true,
        hp_bias_sleep: true,
        hp_xpd: true,
        hp_dbias: 1, // PMU_HP_DBIAS_LIGHTSLEEP_0V6_DEFAULT
        hp_drv_b: 0x00FF_FFF8,
        lp_pd_cur: true,
        lp_bias_sleep: true,
        lp_slp_xpd: false,
        lp_slp_dbias: 0,
        lp_xpd: true,
        lp_dbias: fuses.slp_lp(),
        lp_drv_b: 0,
    };
    if power_down & PD_XTAL == 0 {
        analog.hp_xpd_trx = true;
        analog.hp_dbias = fuses.act_hp();
        analog.hp_pd_cur = false;
        analog.hp_bias_sleep = false;
        analog.lp_pd_cur = false;
        analog.lp_bias_sleep = false;
        analog.lp_dbias = fuses.act_lp();
    } else if power_down & PD_RC_FAST == 0 {
        analog.hp_dbias = fuses.act_hp();
        analog.lp_dbias = fuses.act_lp();
    }
    analog
}

/// ESP-IDF v5.5.1 H2 deep-sleep analog branch:
/// `PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT` (unnamed fields are zero) with the
/// LP sleep dbias from `get_slp_lp_dbias()`.
pub(crate) fn h2_deep_sleep_analog(fuses: &DbiasFuses) -> SleepAnalog {
    SleepAnalog {
        hp_xpd_trx: false,    // PMU_XPD_TRX_SLEEP_DEFAULT
        hp_pd_cur: false,     // PMU_PD_CUR_SLEEP_ON
        hp_bias_sleep: false, // PMU_BIASSLP_SLEEP_ON
        hp_xpd: false,        // PMU_HP_XPD_DEEPSLEEP
        hp_dbias: 0,
        hp_drv_b: 0,
        lp_pd_cur: true,     // PMU_PD_CUR_SLEEP_DEFAULT
        lp_bias_sleep: true, // PMU_BIASSLP_SLEEP_DEFAULT
        lp_slp_xpd: false,   // PMU_LP_SLP_XPD_SLEEP_DEFAULT
        lp_slp_dbias: 0,     // PMU_LP_SLP_DBIAS_SLEEP_DEFAULT
        lp_xpd: true,        // PMU_LP_XPD_SLEEP_DEFAULT
        lp_dbias: fuses.slp_lp(),
        lp_drv_b: 7, // PMU_LP_DRVB_DEEPSLEEP
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const UNFUSED: DbiasFuses = DbiasFuses {
        blk_major: 0,
        blk_minor: 2,
        active_hp: 20,
        active_lp: 21,
        deep_sleep: 9,
    };

    #[test]
    fn dbias_follows_the_efuse_block_version_and_defaults() {
        assert_eq!(
            (UNFUSED.act_hp(), UNFUSED.act_lp(), UNFUSED.slp_lp()),
            (17, 18, 6)
        );
        let fused = DbiasFuses {
            blk_minor: 3,
            ..UNFUSED
        };
        assert_eq!(
            (fused.act_hp(), fused.act_lp(), fused.slp_lp()),
            (22, 23, 9)
        );
        let major = DbiasFuses {
            blk_major: 1,
            blk_minor: 0,
            ..UNFUSED
        };
        assert_eq!(major.act_hp(), 22);
        let blank = DbiasFuses {
            active_hp: 0,
            active_lp: 30,
            deep_sleep: 0,
            ..fused
        };
        assert_eq!(
            (blank.act_hp(), blank.act_lp(), blank.slp_lp()),
            (17, 31, 6)
        );
    }

    #[test]
    fn deep_sleep_analog_matches_the_idf_dslp_default() {
        let fused = DbiasFuses {
            blk_minor: 3,
            ..UNFUSED
        };
        let deep = h2_deep_sleep_analog(&fused);
        // HP regulator off, HP current/bias kept on, as IDF's DSLP default.
        assert!(!deep.hp_xpd_trx && !deep.hp_pd_cur && !deep.hp_bias_sleep && !deep.hp_xpd);
        assert_eq!((deep.hp_dbias, deep.hp_drv_b), (0, 0));
        // LP regulator on at the fused 0.7 V sleep dbias, driver 7.
        assert!(deep.lp_pd_cur && deep.lp_bias_sleep && !deep.lp_slp_xpd && deep.lp_xpd);
        assert_eq!((deep.lp_slp_dbias, deep.lp_dbias, deep.lp_drv_b), (0, 9, 7));
        assert_eq!(h2_deep_sleep_analog(&UNFUSED).lp_dbias, 6);
    }

    #[test]
    fn light_sleep_analog_matches_the_idf_branches() {
        let fused = DbiasFuses {
            blk_minor: 3,
            ..UNFUSED
        };
        // XTAL and RC_FAST off: 0.6 V HP retention, biases off, TRX off.
        let low = h2_light_sleep_analog(PD_XTAL | PD_RC_FAST, &fused);
        assert!(!low.hp_xpd_trx && low.hp_pd_cur && low.hp_bias_sleep && low.hp_xpd);
        assert_eq!((low.hp_dbias, low.hp_drv_b), (1, 0x00FF_FFF8));
        assert!(low.lp_pd_cur && low.lp_bias_sleep && !low.lp_slp_xpd && low.lp_xpd);
        assert_eq!((low.lp_slp_dbias, low.lp_dbias, low.lp_drv_b), (0, 9, 0));
        // RC_FAST kept: active regulator voltages, biases still off.
        let fosc = h2_light_sleep_analog(PD_XTAL, &fused);
        assert_eq!((fosc.hp_dbias, fosc.lp_dbias), (22, 23));
        assert!(fosc.hp_pd_cur && !fosc.hp_xpd_trx);
        // XTAL kept: TRX, current and bias stay on at active voltages.
        for flags in [0, PD_RC_FAST] {
            let on = h2_light_sleep_analog(flags, &fused);
            assert!(on.hp_xpd_trx && !on.hp_pd_cur && !on.hp_bias_sleep);
            assert!(!on.lp_pd_cur && !on.lp_bias_sleep);
            assert_eq!((on.hp_dbias, on.lp_dbias), (22, 23));
        }
    }

    #[test]
    fn top_and_cpu_flags_do_not_change_the_analog_branch() {
        use crate::power::{PD_CPU, PD_MODEM, PD_TOP};
        let fused = DbiasFuses {
            blk_minor: 3,
            ..UNFUSED
        };
        let base = PD_XTAL | PD_RC_FAST;
        assert_eq!(
            h2_light_sleep_analog(base | PD_TOP | PD_CPU | PD_MODEM, &fused),
            h2_light_sleep_analog(base, &fused)
        );
    }
}
