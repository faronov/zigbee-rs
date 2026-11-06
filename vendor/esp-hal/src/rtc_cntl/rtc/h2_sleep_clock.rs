//! H2 clock decoding for retained sleep; ESP-IDF v5.5.1 rtc_clk.c/clk_tree_ll.h.
//!
//! `PCR_SOC_CLK_SEL`: 0 XTAL (32 MHz), 1 PLL (BBPLL, 96 MHz), 2 RC_FAST
//! (8 MHz), 3 FLASH_PLL (64 MHz). The pinned PAC documents 3 as reserved, but
//! IDF's `soc_cpu_clk_src_t` and `rtc_clk_cpu_freq_get_config` define it, and
//! the IDF v5.5.1 second-stage bootloader leaves the CPU on it.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ClockConfig {
    pub source: u8,
    pub cpu_div: u8,
    pub ahb_div: u8,
    pub cpu_mhz: u32,
}

impl ClockConfig {
    pub(crate) fn decode(source: u8, cpu_div: u8, ahb_div: u8) -> Option<Self> {
        let root_mhz = match source {
            0 => 32,
            1 => 96,
            2 => 8,
            // FLASH_PLL: fixed 64 MHz, derived from the 96 MHz BBPLL
            // (`clk_ll_flash_pll_get_freq_mhz`).
            3 => 64,
            _ => return None,
        };
        let cpu_divisor = u32::from(cpu_div) + 1;
        let ahb_divisor = u32::from(ahb_div) + 1;
        if root_mhz % cpu_divisor != 0
            || root_mhz > 32 * ahb_divisor
            || ahb_divisor % cpu_divisor != 0
        {
            return None;
        }
        Some(Self {
            source,
            cpu_div,
            ahb_div,
            cpu_mhz: root_mhz / cpu_divisor,
        })
    }

    /// Whether the source is derived from BBPLL, so a wake must re-enable
    /// and recalibrate BBPLL (96 MHz) before selecting it again. IDF
    /// `rtc_clk_cpu_freq_set_config` does so for both PLL and FLASH_PLL when
    /// the CPU was switched to XTAL for sleep.
    pub(crate) fn needs_bbpll(&self) -> bool {
        matches!(self.source, 1 | 3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pll_and_divided_cpu_and_bus_are_preserved() {
        for (cpu, ahb, mhz) in [(0, 2, 96), (1, 3, 48), (2, 5, 32), (5, 11, 16)] {
            let plan = ClockConfig::decode(1, cpu, ahb).unwrap();
            assert_eq!(
                (plan.source, plan.cpu_div, plan.ahb_div, plan.cpu_mhz),
                (1, cpu, ahb, mhz)
            );
        }
    }

    #[test]
    fn flash_pll_cpu_and_bus_dividers_are_preserved() {
        // IDF rtc_clk_cpu_freq_to_flash_pll: div 1 -> AHB div 2, else AHB = CPU.
        for (cpu, ahb, mhz) in [(0, 1, 64), (1, 1, 32), (3, 3, 16), (1, 3, 32)] {
            let plan = ClockConfig::decode(3, cpu, ahb).unwrap();
            assert_eq!(
                (plan.source, plan.cpu_div, plan.ahb_div, plan.cpu_mhz),
                (3, cpu, ahb, mhz)
            );
            assert!(plan.needs_bbpll());
        }
    }

    #[test]
    fn only_pll_derived_sources_need_bbpll_on_wake() {
        assert!(ClockConfig::decode(1, 0, 2).unwrap().needs_bbpll());
        assert!(!ClockConfig::decode(0, 0, 0).unwrap().needs_bbpll());
        assert!(!ClockConfig::decode(2, 0, 0).unwrap().needs_bbpll());
    }

    #[test]
    fn xtal_and_rc_fast_keep_their_dividers() {
        assert_eq!(ClockConfig::decode(0, 1, 3).unwrap().cpu_mhz, 16);
        assert_eq!(ClockConfig::decode(2, 1, 3).unwrap().cpu_mhz, 4);
    }

    #[test]
    fn unknown_sources_and_invalid_bus_ratios_fail_before_switching() {
        for (source, cpu, ahb) in [
            // FLASH_PLL with a 64 MHz bus (AHB must stay <= 32 MHz).
            (3, 0, 0),
            // A CPU divider that does not divide 64 MHz.
            (3, 2, 5),
            (4, 0, 2),
            (1, 0, 1),
            (1, 1, 2),
            (0, 2, 2),
            (0, 255, 255),
        ] {
            assert_eq!(ClockConfig::decode(source, cpu, ahb), None);
        }
    }
}
