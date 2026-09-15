//! H2 clock decoding for retained sleep; ESP-IDF v5.5.1 rtc_clk.c/clk_tree_ll.h.

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
            // FLASH_PLL is supported by IDF, but not by the pinned H2 PAC/TRM.
            // Fail closed rather than extending this backport to that source.
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
    fn xtal_and_rc_fast_keep_their_dividers() {
        assert_eq!(ClockConfig::decode(0, 1, 3).unwrap().cpu_mhz, 16);
        assert_eq!(ClockConfig::decode(2, 1, 3).unwrap().cpu_mhz, 4);
    }

    #[test]
    fn unknown_sources_and_invalid_bus_ratios_fail_before_switching() {
        for (source, cpu, ahb) in [
            (3, 0, 1),
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
