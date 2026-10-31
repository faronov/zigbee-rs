//! Exclusive BRD4181A board resources.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32s2_hal::{
    clock::ClockControl,
    gpio::{Disabled, Level, Pin, Pull},
    peripherals::Peripherals,
    radio::RadioToken,
    rtcc::RtccToken,
    usart::Usart,
};

use crate::{
    BUTTON0_PIN, BUTTON0_PORT, Button0, Efr32s2Flash, LED0_PIN, LED0_PORT, Led0, VCOM_BAUD,
    VCOM_ENABLE_PIN, VCOM_ENABLE_PORT, VCOM_TX_PIN, VCOM_TX_PORT, Vcom, configure_button_interrupt,
};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete singleton resource set for BRD4181A on BRD4001A.
pub struct BoardResources {
    pub clocks: ClockControl,
    pub led0: Led0Token,
    pub button0: Button0Token,
    pub vcom: VcomToken,
    /// 802.15.4 radio (needs the HFXO from [`crate::init_clocks`]).
    pub radio: RadioToken,
    /// LFRCO-clocked RTCC: EM2 wake and time base.
    pub rtcc: RtccToken,
    /// Raw, exclusive internal-flash controller. Product code bounds it to
    /// its selected partitions before using it for persistence.
    pub flash: Efr32s2Flash,
}

impl BoardResources {
    /// Acquire all supported board resources once.
    pub fn take() -> Option<Self> {
        if TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }

        let Some(mut peripherals) = Peripherals::take() else {
            TAKEN.store(false, Ordering::Release);
            return None;
        };

        // SAFETY: BoardResources is a singleton and these are distinct fitted
        // pins of the BRD4181A configuration (PB0, PD2, PD4, PA5).
        let (led0, button0, vcom_enable, vcom_tx) = unsafe {
            (
                peripherals.gpio.claim_pin(LED0_PORT, LED0_PIN).ok()?,
                peripherals.gpio.claim_pin(BUTTON0_PORT, BUTTON0_PIN).ok()?,
                peripherals
                    .gpio
                    .claim_pin(VCOM_ENABLE_PORT, VCOM_ENABLE_PIN)
                    .ok()?,
                peripherals.gpio.claim_pin(VCOM_TX_PORT, VCOM_TX_PIN).ok()?,
            )
        };

        Some(Self {
            clocks: peripherals.clocks,
            led0: Led0Token(led0),
            button0: Button0Token(button0),
            vcom: VcomToken {
                usart: peripherals.usart0,
                enable: vcom_enable,
                tx: vcom_tx,
            },
            radio: peripherals.radio,
            rtcc: peripherals.rtcc,
            flash: peripherals.flash,
        })
    }
}

/// Exclusive PB0 ownership.
pub struct Led0Token(Pin<Disabled>);

impl Led0Token {
    /// Configure PB0 as an initially-off, active-high push-pull LED.
    pub fn into_led(self) -> Led0 {
        Led0::from_pin(self.0.into_push_pull(Level::Low))
    }
}

/// Exclusive PD2 ownership.
pub struct Button0Token(Pin<Disabled>);

impl Button0Token {
    /// Configure PD2 as a plain input with falling-edge interrupt.
    ///
    /// BRD4001A provides the active-low button's external bias. No internal
    /// pull is enabled, matching Silicon Labs' BRD4181A board configuration.
    pub fn into_button(self) -> Button0 {
        let mut pin = self.0.into_input(Pull::None);
        configure_button_interrupt(&mut pin);
        Button0::from_pin(pin)
    }
}

/// Exclusive USART0 + PD4/PA5 ownership for the WSTK VCOM.
pub struct VcomToken {
    usart: Usart,
    enable: Pin<Disabled>,
    tx: Pin<Disabled>,
}

impl VcomToken {
    /// Enable the WSTK VCOM and start a 115200 8N1 TX-only UART.
    /// Requires the HFXO ([`crate::init_clocks`]) for the baud rate.
    pub fn into_vcom(self) -> Vcom {
        let enable = self.enable.into_push_pull(Level::High);
        let uart = self
            .usart
            .into_uart_tx(self.tx.into_push_pull(Level::High), VCOM_BAUD);
        Vcom::new(uart, enable)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_resources_are_singleton() {
        assert!(BoardResources::take().is_some());
        assert!(BoardResources::take().is_none());
    }
}
