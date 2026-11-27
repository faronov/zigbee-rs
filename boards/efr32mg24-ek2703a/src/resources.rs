//! Exclusive EK2703A (BRD2703A) resources.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32s2_hal::{
    clock::ClockControl,
    gpio::{Disabled, Input, Level, Pin, Pull, PushPull},
    peripherals::Peripherals,
    radio::RadioToken,
    rtcc::RtccToken,
    usart::{UartTx, Usart},
    watchdog::WdogToken,
};

use crate::{
    BUTTON0_PIN, BUTTON0_PORT, BUTTON1_PIN, BUTTON1_PORT, Efr32s2Flash, LED0_PIN, LED0_PORT,
    LED1_PIN, LED1_PORT, VCOM_RX_PIN, VCOM_RX_PORT, VCOM_TX_PIN, VCOM_TX_PORT,
};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete singleton resource set.
pub struct BoardResources {
    pub clocks: ClockControl,
    /// USART0 to the VCOM (the on-board J-Link OB's USB CDC port).
    pub host_uart: HostUartToken,
    /// 802.15.4 radio (needs the HFXO).
    pub radio: RadioToken,
    /// LFRCO-clocked SYSRTC0 (the HAL's `rtcc` module): time base.
    pub rtcc: RtccToken,
    pub wdog: WdogToken,
    /// Raw, exclusive internal-flash controller; the product bounds it to
    /// its partitions.
    pub flash: Efr32s2Flash,
    pub leds: Leds,
    pub buttons: Buttons,
}

impl BoardResources {
    /// Acquire all board resources once.
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
        let gpio = &mut peripherals.gpio;
        // SAFETY: singleton; every pin below is distinct and wired only to
        // its kit function (GSDK brd2703a).
        let (tx, rx, led0, led1, btn0, btn1) = unsafe {
            (
                gpio.claim_pin(VCOM_TX_PORT, VCOM_TX_PIN).ok()?,
                gpio.claim_pin(VCOM_RX_PORT, VCOM_RX_PIN).ok()?,
                gpio.claim_pin(LED0_PORT, LED0_PIN).ok()?,
                gpio.claim_pin(LED1_PORT, LED1_PIN).ok()?,
                gpio.claim_pin(BUTTON0_PORT, BUTTON0_PIN).ok()?,
                gpio.claim_pin(BUTTON1_PORT, BUTTON1_PIN).ok()?,
            )
        };
        Some(Self {
            clocks: peripherals.clocks,
            host_uart: HostUartToken {
                usart: peripherals.usart0,
                ldma: peripherals.ldma,
                tx,
                rx,
            },
            radio: peripherals.radio,
            rtcc: peripherals.rtcc,
            wdog: peripherals.wdog,
            flash: peripherals.flash,
            leds: Leds { led0, led1 },
            buttons: Buttons { btn0, btn1 },
        })
    }
}

/// Exclusive USART0 + PA5/PA6 (RTS PA8 / CTS PA9 left unclaimed).
pub struct HostUartToken {
    usart: Usart,
    /// The host link's receiver DMA.
    ldma: efr32s2_hal::ldma::Ldma,
    tx: Pin<Disabled>,
    rx: Pin<Disabled>,
}

impl HostUartToken {
    /// 8N1 link to the VCOM without flow control (LDMA receiver,
    /// interrupt-driven transmitter). Requires
    /// the HFXO.
    #[cfg(feature = "host-uart")]
    pub fn open(
        self,
        baud: u32,
    ) -> Result<efr32s2_hal::usart::host::HostUart, efr32s2_hal::usart::host::HostUartError> {
        efr32s2_hal::usart::host::HostUart::open(
            self.usart,
            self.ldma,
            self.tx.into_push_pull(Level::High),
            self.rx.into_input(Pull::Up),
            baud,
        )
    }

    /// Blocking transmit-only UART (logs) on the VCOM. Requires the HFXO.
    pub fn into_uart_tx(self, baud: u32) -> UartTx {
        let _unused = (self.rx, self.ldma);
        self.usart
            .into_uart_tx(self.tx.into_push_pull(Level::High), baud)
    }
}

/// LED0 (PA4) and LED1 (PA7), active high, still unconfigured.
pub struct Leds {
    led0: Pin<Disabled>,
    led1: Pin<Disabled>,
}

impl Leds {
    /// Both LEDs as push-pull outputs, off.
    pub fn into_outputs(self) -> (Pin<PushPull>, Pin<PushPull>) {
        (
            self.led0.into_push_pull(Level::Low),
            self.led1.into_push_pull(Level::Low),
        )
    }
}

/// BTN0 (PB2) and BTN1 (PB3), active low with the board pull-ups.
pub struct Buttons {
    btn0: Pin<Disabled>,
    btn1: Pin<Disabled>,
}

impl Buttons {
    /// Both buttons as plain inputs, as GSDK's `simple_button`
    /// (`gpioModeInput`, board pull-ups).
    pub fn into_inputs(self) -> (Pin<Input>, Pin<Input>) {
        (
            self.btn0.into_input(Pull::None),
            self.btn1.into_input(Pull::None),
        )
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
