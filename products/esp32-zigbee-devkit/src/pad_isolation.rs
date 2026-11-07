//! Isolation of unused ESP32-H2 GPIO pads, for light-sleep current.
//!
//! ESP-IDF v5.5.1 isolates every pad in sleep
//! (`esp_sleep_config_gpio_isolate`: sleep direction disabled, sleep pulls
//! off) and enables the automatic switch to that sleep configuration
//! (`esp_sleep_enable_gpio_switch`: `SLP_SEL` per pad, except the console
//! UART). The H2 PMU applies the per-pad sleep configuration in HP_SLEEP
//! because the HAL sets `dig_pad_slp_sel` for light sleep without TOP
//! power-down.
//!
//! This module does the same for the pads a product declares unused, and
//! additionally clears their active input buffer and pulls: a pad that no
//! driver owns has nothing to keep, and the floor lab measured the active
//! configuration as the leak (the Waveshare ESP32-H2-Zero GPIO8 reset
//! state, input buffer on with an external pull-up, costs about 128 uA in
//! light sleep). The product selects the mask; which pads are wired is a
//! board fact recorded next to that selection.
//!
//! Differences from ESP-IDF, both fail-safe: the mask is an explicit
//! selection instead of "every pad", and flash (MSPI) and USB Serial/JTAG
//! pads are refused rather than isolated, so neither the flash-resident
//! code nor the USB console/flashing path depends on sleep pad behaviour.
//!
//! With the TOP domain powered down in light sleep the PMU leaves
//! `dig_pad_slp_sel` clear (ESP-IDF `PMU_SLEEP_DIGITAL_LSLP_CONFIG_DEFAULT`)
//! and IO_MUX/GPIO lose power, so neither the sleep nor the active
//! configuration reaches the pads. `apply` therefore also latches every
//! isolated pad with the always-on per-pad hold (`LP_AON_GPIO_HOLD0`,
//! ESP-IDF `gpio_ll_hold_en`, bit n = GPIOn per `GPIO_HOLD_MASK`); ESP-IDF
//! notes the LP_AON hold as the only way to keep a pad state across TOP
//! power-down. The hold is set after the isolated configuration is written,
//! so the latched state is the isolated one, and it is harmless without TOP
//! power-down because the latched and sleep configurations are equal.
//!
//! The hold survives every reset except power-on. An image that later
//! drives one of these pads must release its hold bit first; `esp-hal` does
//! not do that when it configures a pin.
//!
//! A pad isolated here and later claimed by an `esp-hal` driver is
//! reconfigured by that driver (`init_gpio` clears `FUN_IE` and `SLP_SEL`),
//! so a stale mask cannot silently isolate an owned pin; it is still a
//! configuration error the caller excludes at compile time.

/// Number of GPIO pads (`SOC_GPIO_PIN_COUNT`, ESP-IDF `soc/esp32h2/soc_caps.h`).
pub const PAD_COUNT: u32 = 28;
/// In-package SPI flash pads GPIO15..=20 (`MSPI_IOMUX_PIN_NUM_*`, ESP-IDF
/// `soc/esp32h2/include/soc/spi_pins.h`).
pub const FLASH_PADS: u32 = 0b11_1111 << 15;
/// USB Serial/JTAG D- (GPIO26) and D+ (GPIO27).
pub const USB_PADS: u32 = (1 << 26) | (1 << 27);

/// Why a pad mask was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadIsolationError {
    /// The bit names no GPIO pad.
    NoSuchPad(u8),
    /// An in-package flash pad.
    FlashPad(u8),
    /// A USB Serial/JTAG pad.
    UsbPad(u8),
}

/// A validated set of unused pads (bit `n` = GPIO`n`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UnusedPads(u32);

impl UnusedPads {
    /// Validate `mask`; usable in a `const` so a bad build configuration
    /// fails to compile. The lowest offending pad is reported.
    pub const fn new(mask: u32) -> Result<Self, PadIsolationError> {
        let mut pad = 0;
        while pad < 32 {
            let bit = 1u32 << pad;
            if mask & bit != 0 {
                if pad >= PAD_COUNT {
                    return Err(PadIsolationError::NoSuchPad(pad as u8));
                }
                if FLASH_PADS & bit != 0 {
                    return Err(PadIsolationError::FlashPad(pad as u8));
                }
                if USB_PADS & bit != 0 {
                    return Err(PadIsolationError::UsbPad(pad as u8));
                }
            }
            pad += 1;
        }
        Ok(Self(mask))
    }

    /// `new` for constants: a refused mask is a compile-time error.
    pub const fn checked(mask: u32) -> Self {
        match Self::new(mask) {
            Ok(pads) => pads,
            Err(PadIsolationError::NoSuchPad(_)) => {
                panic!("pad mask names a pad that does not exist")
            }
            Err(PadIsolationError::FlashPad(_)) => {
                panic!("pad mask includes a flash pad (GPIO15..=20)")
            }
            Err(PadIsolationError::UsbPad(_)) => panic!("pad mask includes a USB pad (GPIO26/27)"),
        }
    }

    pub const fn mask(self) -> u32 {
        self.0
    }

    pub const fn contains(self, pad: u8) -> bool {
        pad < 32 && self.0 & (1 << pad) != 0
    }

    /// Output driver off; active and sleep input buffer and pulls off; the
    /// pad muxed to GPIO (ESP-IDF's deep-sleep isolation notes the bottom
    /// current rises otherwise) and switched to its (isolated) sleep
    /// configuration in sleep; then held (`LP_AON_GPIO_HOLD0`) so the
    /// isolated state also survives TOP power-down.
    ///
    /// # Safety
    ///
    /// No driver may own or use a pad in the set, now or later. Call it once
    /// all pins the product uses are configured; it is not interrupt-safe
    /// against concurrent IO_MUX/GPIO configuration.
    #[cfg(all(target_os = "none", feature = "esp32h2"))]
    pub unsafe fn apply(self) {
        use esp_hal::peripherals::{GPIO, IO_MUX};

        /// `PIN_FUNC_GPIO` on ESP32-H2.
        const FUNC_GPIO: u8 = 1;
        GPIO::regs()
            .enable_w1tc()
            .write(|w| unsafe { w.bits(self.0) });
        let mut pad = 0;
        while pad < PAD_COUNT as usize {
            if self.0 & (1 << pad) != 0 {
                IO_MUX::regs().gpio(pad).modify(|_, w| unsafe {
                    w.mcu_sel()
                        .bits(FUNC_GPIO)
                        .fun_ie()
                        .clear_bit()
                        .fun_wpu()
                        .clear_bit()
                        .fun_wpd()
                        .clear_bit()
                        .mcu_oe()
                        .clear_bit()
                        .mcu_ie()
                        .clear_bit()
                        .mcu_wpu()
                        .clear_bit()
                        .mcu_wpd()
                        .clear_bit()
                        .slp_sel()
                        .set_bit()
                });
            }
            pad += 1;
        }
        // Latch only after the isolated configuration is in place; bits of
        // pads outside the set are left alone.
        esp_hal::peripherals::LP_AON::regs()
            .gpio_hold0()
            .modify(|r, w| unsafe { w.gpio_hold0().bits(r.gpio_hold0().bits() | self.0) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_and_ordinary_pads_are_accepted() {
        assert_eq!(UnusedPads::new(0).map(UnusedPads::mask), Ok(0));
        let pads = UnusedPads::new(0x0240_7DFF).unwrap();
        assert!(pads.contains(8));
        assert!(!pads.contains(9));
        assert!(!pads.contains(23));
        assert!(!pads.contains(40));
    }

    #[test]
    fn flash_usb_and_missing_pads_are_refused() {
        for pad in 15..=20 {
            assert_eq!(
                UnusedPads::new(1 << pad),
                Err(PadIsolationError::FlashPad(pad))
            );
        }
        assert_eq!(UnusedPads::new(1 << 21).map(UnusedPads::mask), Ok(1 << 21));
        assert_eq!(UnusedPads::new(1 << 26), Err(PadIsolationError::UsbPad(26)));
        assert_eq!(UnusedPads::new(1 << 27), Err(PadIsolationError::UsbPad(27)));
        for pad in 28..32 {
            assert_eq!(
                UnusedPads::new(1 << pad),
                Err(PadIsolationError::NoSuchPad(pad))
            );
        }
    }

    #[test]
    fn the_lowest_offending_pad_is_reported() {
        assert_eq!(
            UnusedPads::new((1 << 27) | (1 << 16) | 1),
            Err(PadIsolationError::FlashPad(16))
        );
    }
}
