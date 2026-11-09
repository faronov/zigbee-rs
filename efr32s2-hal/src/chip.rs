//! Per-chip constants for the EFR32 Series 2 devices served by this crate.
//!
//! Every value here was checked against the GSDK 4.5.0 device headers
//! (`platform/Device/SiliconLabs/EFR32MG2{1,2}/Include`). Only addresses
//! that actually differ between the two dies are selected by feature; the
//! shared ones live in the modules that use them.

/// Which Series 2 die the crate was built for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    /// EFR32MG21 (xG21, Series 2 config 1): 20 dBm PA, 64 KiB SRAM, no CMU
    /// clock gates.
    Mg21,
    /// EFR32MG22 (xG22, Series 2 config 2): 6 dBm PA, 32 KiB SRAM, CMU
    /// `CLKEN0/1` bus-clock gates.
    Mg22,
}

#[cfg(feature = "efr32mg21")]
pub const CHIP: Chip = Chip::Mg21;
#[cfg(feature = "efr32mg22")]
pub const CHIP: Chip = Chip::Mg22;

/// Smallest main flash of the supported MG21/MG22 parts (512 KiB). Larger
/// parts (MG21 F1024) report their size through
/// [`crate::flash::device_flash_capacity`].
pub const FLASH_SIZE: u32 = 512 * 1024;

/// Usable SRAM at `0x2000_0000`.
#[cfg(feature = "efr32mg21")]
pub const RAM_SIZE: u32 = 64 * 1024;
#[cfg(feature = "efr32mg22")]
pub const RAM_SIZE: u32 = 32 * 1024;

/// Secure-alias peripheral bases that differ between MG21 and MG22.
pub mod base {
    #[cfg(feature = "efr32mg21")]
    pub const USART0: u32 = 0x4005_8000;
    #[cfg(feature = "efr32mg21")]
    pub const USART1: u32 = 0x4005_C000;
    #[cfg(feature = "efr32mg22")]
    pub const USART0: u32 = 0x4005_C000;
    #[cfg(feature = "efr32mg22")]
    pub const USART1: u32 = 0x4006_0000;

    pub const CMU: u32 = 0x4000_8000;
    pub const GPIO: u32 = 0x4003_C000;
    pub const RTCC: u32 = 0x4800_0000;
    pub const I2C0: u32 = 0x4A01_0000;
    pub const EMU: u32 = 0x4000_4000;
}

/// GPIO peripheral-route register offsets (relative to `GPIO` base).
pub mod route {
    #[cfg(feature = "efr32mg21")]
    pub const FRC_ROUTEEN: u32 = 0x484;
    #[cfg(feature = "efr32mg21")]
    pub const I2C0_ROUTEEN: u32 = 0x498;
    #[cfg(feature = "efr32mg21")]
    pub const USART0_ROUTEEN: u32 = 0x5AC;
    #[cfg(feature = "efr32mg21")]
    pub const USART1_ROUTEEN: u32 = 0x5CC;

    #[cfg(feature = "efr32mg22")]
    pub const FRC_ROUTEEN: u32 = 0x47C;
    #[cfg(feature = "efr32mg22")]
    pub const I2C0_ROUTEEN: u32 = 0x490;
    #[cfg(feature = "efr32mg22")]
    pub const USART0_ROUTEEN: u32 = 0x61C;
    #[cfg(feature = "efr32mg22")]
    pub const USART1_ROUTEEN: u32 = 0x63C;
}

/// CMU bus-clock gates.
///
/// xG22 gates every peripheral bus clock in `CMU_CLKEN0/1` (reset value 0);
/// xG21 has no gates, so [`enable`](crate::clock::enable_bus_clock) is a
/// no-op there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusClock {
    pub(crate) register: u8,
    pub(crate) bit: u8,
}

impl BusClock {
    const fn clken0(bit: u8) -> Self {
        Self { register: 0, bit }
    }
    const fn clken1(bit: u8) -> Self {
        Self { register: 1, bit }
    }

    pub const TIMER0: Self = Self::clken0(4);
    pub const USART0: Self = Self::clken0(8);
    pub const USART1: Self = Self::clken0(9);
    pub const WDOG0: Self = Self::clken0(13);
    pub const I2C0: Self = Self::clken0(14);
    pub const SYSCFG: Self = Self::clken0(16);
    pub const HFXO0: Self = Self::clken0(19);
    pub const GPIO: Self = Self::clken0(26);
    pub const LFRCO: Self = Self::clken0(21);
    pub const LFXO: Self = Self::clken0(22);
    pub const RTCC: Self = Self::clken0(30);
    pub const DCDC: Self = Self::clken0(31);
    pub const SMU: Self = Self::clken1(15);
    pub const MSC: Self = Self::clken1(17);
}

/// `CMU_CLKEN1` bits 0..=12: AGC, MODEM, RFCRC, FRC, PROTIMER, RAC, SYNTH,
/// RDSCRATCHPAD, RDMAILBOX0/1, PRORTC, BUFC, IFADCDEBUG (xG22).
pub const CLKEN1_RADIO_MASK: u32 = 0x1FFF;

/// Bus clocks the HAL enables at [`crate::peripherals::Peripherals::take`].
pub const BASE_BUS_CLOCKS: [BusClock; 4] = [
    BusClock::GPIO,
    BusClock::HFXO0,
    BusClock::SYSCFG,
    BusClock::MSC,
];

#[cfg(all(test, feature = "efr32mg22"))]
mod tests {
    use super::*;

    #[test]
    fn xg22_golden_clken_matches_gsdk_masks() {
        // CMU->CLKEN0 captured from GSDK RAIL firmware on BRD4183A:
        // GPIO | HFXO0 | SYSCFG = 0x0409_0000.
        let clken0: u32 = [BusClock::GPIO, BusClock::HFXO0, BusClock::SYSCFG]
            .iter()
            .map(|c| 1u32 << c.bit)
            .sum();
        assert_eq!(clken0, 0x0409_0000);
        // CLKEN1 0x0002_9FFF = radio | SMU | MSC.
        let clken1 = CLKEN1_RADIO_MASK | (1 << BusClock::SMU.bit) | (1 << BusClock::MSC.bit);
        assert_eq!(clken1, 0x0002_9FFF);
    }
}
