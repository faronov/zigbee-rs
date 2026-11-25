//! Per-chip constants for the EFR32 Series 2 devices served by this crate.
//!
//! Every value here was checked against the GSDK 4.5.0 device headers
//! (`platform/Device/SiliconLabs/EFR32MG2{1,2,4}/Include`). Only addresses
//! that actually differ between the dies are selected by feature; the
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
    /// EFR32MG24 (xG24, Series 2 config 4): 10 or 20 dBm PA, 256 KiB SRAM,
    /// flash at `0x0800_0000`, `CLKEN0/1` gates at other bit positions than
    /// xG22, SYSRTC instead of RTCC, one USART plus two EUSARTs. Only in
    /// xG24 builds, so the xG21/xG22 `Debug` output and images stay
    /// unchanged.
    #[cfg(feature = "efr32mg24")]
    Mg24,
}

#[cfg(feature = "efr32mg21")]
pub const CHIP: Chip = Chip::Mg21;
#[cfg(feature = "efr32mg22")]
pub const CHIP: Chip = Chip::Mg22;
#[cfg(feature = "efr32mg24")]
pub const CHIP: Chip = Chip::Mg24;

/// Smallest main flash of the supported MG21/MG22 parts (512 KiB). Larger
/// parts (MG21 F1024) report their size through
/// [`crate::flash::device_flash_capacity`].
#[cfg(not(feature = "efr32mg24"))]
pub const FLASH_SIZE: u32 = 512 * 1024;
/// Smallest main flash of the EFR32MG24 family (F768); the F1536 parts
/// report their size through [`crate::flash::device_flash_capacity`].
#[cfg(feature = "efr32mg24")]
pub const FLASH_SIZE: u32 = 768 * 1024;

/// Bus address of main-flash offset 0 (`FLASH_BASE`): `0x0` on xG21/xG22,
/// `0x0800_0000` on xG24. [`crate::flash`] offsets are relative to it.
#[cfg(not(feature = "efr32mg24"))]
pub const FLASH_BASE: u32 = 0;
#[cfg(feature = "efr32mg24")]
pub const FLASH_BASE: u32 = 0x0800_0000;

/// Usable SRAM at `0x2000_0000`.
#[cfg(feature = "efr32mg21")]
pub const RAM_SIZE: u32 = 64 * 1024;
#[cfg(feature = "efr32mg22")]
pub const RAM_SIZE: u32 = 32 * 1024;
#[cfg(feature = "efr32mg24")]
pub const RAM_SIZE: u32 = 256 * 1024;

/// Secure-alias peripheral bases that differ between the dies.
pub mod base {
    #[cfg(feature = "efr32mg21")]
    pub const USART0: u32 = 0x4005_8000;
    #[cfg(feature = "efr32mg21")]
    pub const USART1: u32 = 0x4005_C000;
    #[cfg(feature = "efr32mg22")]
    pub const USART0: u32 = 0x4005_C000;
    #[cfg(feature = "efr32mg22")]
    pub const USART1: u32 = 0x4006_0000;

    /// xG24 has a single USART; `EUSART0/1` are a different block.
    #[cfg(feature = "efr32mg24")]
    pub const USART0: u32 = 0x4005_C000;

    pub const CMU: u32 = 0x4000_8000;
    pub const GPIO: u32 = 0x4003_C000;
    #[cfg(not(feature = "efr32mg24"))]
    pub const RTCC: u32 = 0x4800_0000;
    /// xG24 replaces the RTCC with SYSRTC0.
    #[cfg(feature = "efr32mg24")]
    pub const SYSRTC0: u32 = 0x400A_8000;
    #[cfg(not(feature = "efr32mg24"))]
    pub const I2C0: u32 = 0x4A01_0000;
    #[cfg(feature = "efr32mg24")]
    pub const I2C0: u32 = 0x4B00_0000;
    pub const EMU: u32 = 0x4000_4000;
    #[cfg(not(feature = "efr32mg24"))]
    pub const HFXO0: u32 = 0x4000_C000;
    #[cfg(feature = "efr32mg24")]
    pub const HFXO0: u32 = 0x4A00_4000;
    #[cfg(not(feature = "efr32mg24"))]
    pub const WDOG0: u32 = 0x4A01_8000;
    #[cfg(feature = "efr32mg24")]
    pub const WDOG0: u32 = 0x4B00_4000;
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

    #[cfg(feature = "efr32mg24")]
    pub const FRC_ROUTEEN: u32 = 0x4D4;
    #[cfg(feature = "efr32mg24")]
    pub const I2C0_ROUTEEN: u32 = 0x4E8;
    #[cfg(feature = "efr32mg24")]
    pub const USART0_ROUTEEN: u32 = 0x720;
}

/// CMU bus-clock gates.
///
/// xG22 and xG24 gate every peripheral bus clock in `CMU_CLKEN0/1` (reset
/// value 0) at die-specific bit positions; xG21 has no gates, so
/// [`enable`](crate::clock::enable_bus_clock) is a no-op there.
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

    #[cfg(not(feature = "efr32mg24"))]
    pub const TIMER0: Self = Self::clken0(4);
    #[cfg(not(feature = "efr32mg24"))]
    pub const USART0: Self = Self::clken0(8);
    #[cfg(not(feature = "efr32mg24"))]
    pub const USART1: Self = Self::clken0(9);
    #[cfg(not(feature = "efr32mg24"))]
    pub const IADC0: Self = Self::clken0(10);
    #[cfg(not(feature = "efr32mg24"))]
    pub const WDOG0: Self = Self::clken0(13);
    #[cfg(not(feature = "efr32mg24"))]
    pub const I2C0: Self = Self::clken0(14);
    #[cfg(not(feature = "efr32mg24"))]
    pub const SYSCFG: Self = Self::clken0(16);
    #[cfg(not(feature = "efr32mg24"))]
    pub const HFXO0: Self = Self::clken0(19);
    #[cfg(not(feature = "efr32mg24"))]
    pub const GPIO: Self = Self::clken0(26);
    #[cfg(not(feature = "efr32mg24"))]
    pub const LFRCO: Self = Self::clken0(21);
    #[cfg(not(feature = "efr32mg24"))]
    pub const LFXO: Self = Self::clken0(22);
    #[cfg(not(feature = "efr32mg24"))]
    pub const RTCC: Self = Self::clken0(30);
    #[cfg(not(feature = "efr32mg24"))]
    pub const DCDC: Self = Self::clken0(31);
    #[cfg(not(feature = "efr32mg24"))]
    pub const CRYPTOACC: Self = Self::clken1(13);
    #[cfg(not(feature = "efr32mg24"))]
    pub const SMU: Self = Self::clken1(15);
    #[cfg(not(feature = "efr32mg24"))]
    pub const MSC: Self = Self::clken1(17);

    // xG24 (`efr32mg24_cmu.h`, `_CMU_CLKEN0/1_*_SHIFT`).
    #[cfg(feature = "efr32mg24")]
    pub const TIMER0: Self = Self::clken0(4);
    #[cfg(feature = "efr32mg24")]
    pub const USART0: Self = Self::clken0(9);
    #[cfg(feature = "efr32mg24")]
    pub const IADC0: Self = Self::clken0(10);
    #[cfg(feature = "efr32mg24")]
    pub const WDOG0: Self = Self::clken0(13);
    #[cfg(feature = "efr32mg24")]
    pub const I2C0: Self = Self::clken0(14);
    #[cfg(feature = "efr32mg24")]
    pub const SYSCFG: Self = Self::clken0(16);
    #[cfg(feature = "efr32mg24")]
    pub const HFXO0: Self = Self::clken0(20);
    #[cfg(feature = "efr32mg24")]
    pub const LFRCO: Self = Self::clken0(22);
    #[cfg(feature = "efr32mg24")]
    pub const LFXO: Self = Self::clken0(23);
    #[cfg(feature = "efr32mg24")]
    pub const GPIO: Self = Self::clken0(26);
    /// SYSRTC0, the xG24 replacement of the RTCC (named `RTCC` here so the
    /// time base keeps one name across dies).
    #[cfg(feature = "efr32mg24")]
    pub const RTCC: Self = Self::clken0(30);
    #[cfg(feature = "efr32mg24")]
    pub const DCDC: Self = Self::clken0(31);
    #[cfg(feature = "efr32mg24")]
    pub const SEMAILBOXHOST: Self = Self::clken1(10);
    #[cfg(feature = "efr32mg24")]
    pub const SMU: Self = Self::clken1(14);
    #[cfg(feature = "efr32mg24")]
    pub const MSC: Self = Self::clken1(16);
    #[cfg(feature = "efr32mg24")]
    pub const EUSART0: Self = Self::clken1(22);
    #[cfg(feature = "efr32mg24")]
    pub const EUSART1: Self = Self::clken1(23);
}

/// `CMU_CLKEN1` bits 0..=12: AGC, MODEM, RFCRC, FRC, PROTIMER, RAC, SYNTH,
/// RDSCRATCHPAD, RDMAILBOX0/1, PRORTC, BUFC, IFADCDEBUG (xG22).
#[cfg(not(feature = "efr32mg24"))]
pub const CLKEN1_RADIO_MASK: u32 = 0x1FFF;
/// xG24 `CMU_CLKEN1` radio bits: AGC, MODEM, RFCRC, FRC, PROTIMER, RAC,
/// SYNTH, RFSCRATCHPAD (0..=7), RFMAILBOX (9), BUFC (11), RFECA0/1 (25, 26)
/// and ECAIFADC (28). Taken from the header bit names; which of them RAIL
/// actually opens has not been captured on an xG24 yet.
#[cfg(feature = "efr32mg24")]
pub const CLKEN1_RADIO_MASK: u32 = 0xFF | (1 << 9) | (1 << 11) | (1 << 25) | (1 << 26) | (1 << 28);

/// Bus clocks the HAL enables at [`crate::peripherals::Peripherals::take`].
pub const BASE_BUS_CLOCKS: [BusClock; 4] = [
    BusClock::GPIO,
    BusClock::HFXO0,
    BusClock::SYSCFG,
    BusClock::MSC,
];

#[cfg(all(test, feature = "efr32mg24"))]
mod mg24_tests {
    use super::*;

    #[test]
    fn xg24_header_values() {
        // offsetof()/`_SHIFT` values from GSDK 4.5.0 efr32mg24a410f1536im48.h,
        // efr32mg24_cmu.h and efr32mg24_gpio.h.
        assert_eq!(FLASH_BASE, 0x0800_0000);
        assert_eq!(RAM_SIZE, 0x0004_0000);
        assert_eq!(base::USART0, 0x4005_C000);
        assert_eq!(base::HFXO0, 0x4A00_4000);
        assert_eq!(base::SYSRTC0, 0x400A_8000);
        assert_eq!(route::USART0_ROUTEEN, 0x720);
        let clken0: u32 = [BusClock::GPIO, BusClock::HFXO0, BusClock::SYSCFG]
            .iter()
            .map(|c| 1u32 << c.bit)
            .sum();
        assert_eq!(clken0, (1 << 26) | (1 << 20) | (1 << 16));
        assert_eq!((BusClock::USART0.register, BusClock::USART0.bit), (0, 9));
        assert_eq!((BusClock::MSC.register, BusClock::MSC.bit), (1, 16));
        assert_eq!(CLKEN1_RADIO_MASK & (1 << 10), 0, "SE mailbox is not radio");
    }
}

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

/// xG21 has no CMU bus-clock gates (`clock::enable_bus_clock` is a no-op
/// there); the Secure Engine mailbox keeps the xG24 name so `se` builds on
/// both dies (`se-trng`; appended: keeps the line numbers above).
#[cfg(feature = "efr32mg21")]
impl BusClock {
    pub const SEMAILBOXHOST: Self = Self::clken1(10);
}
