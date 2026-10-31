//! BSP for BRD4183A (EFR32MG22C224F512IM40) on the BRD4001A Wireless
//! Starter Kit.
//!
//! Wiring is from GSDK 4.5.0 `hardware/board/config/brd4183a_brd4001a`:
//! LED0 PB1 (active high), BTN0 PB0 (active low, WSTK pull-up), VCOM
//! USART TX PA5 with the WSTK VCOM enable on PA4, HFXO 38.4 MHz with
//! CTUNE 120. The WSTK Si7021 is not reachable from this radio board
//! (lab gate `s2-i2c-scan`), so no sensor bus is exposed.
//!
//! The crate exposes fitted resources only; it owns no Zigbee runtime,
//! memory layout or persistence policy.

#![no_std]

pub mod resources;

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    gpio::{self, Input, InterruptEdge, Pin, Port, PushPull},
    usart::UartTx,
};

pub const BOARD_RADIO: &str = "BRD4183A";
pub const BOARD_MAIN: &str = "BRD4001A";
pub const MCU_PART: &str = "EFR32MG22C224F512IM40";
pub const HCLK_HZ: u32 = 38_400_000;
pub const HFXO_CTUNE: u16 = 120;

pub const LED0_PORT: Port = Port::B;
pub const LED0_PIN: u8 = 1;
pub const BUTTON0_PORT: Port = Port::B;
pub const BUTTON0_PIN: u8 = 0;
/// PB0 uses regular EXTI line 0, serviced by `GPIO_EVEN`. Port B keeps
/// asynchronous edge interrupts in EM2.
pub const BUTTON0_INTERRUPT_LINE: u8 = 0;
pub const VCOM_ENABLE_PORT: Port = Port::A;
pub const VCOM_ENABLE_PIN: u8 = 4;
pub const VCOM_TX_PORT: Port = Port::A;
pub const VCOM_TX_PIN: u8 = 5;
pub const VCOM_BAUD: u32 = 115_200;

/// Select BRD4183A's 38.4 MHz crystal before starting the radio.
pub fn init_clocks(mut clocks: ClockControl) -> Result<SystemClocks, ClockError> {
    clocks.configure_hfxo(HfxoConfig {
        frequency_hz: HCLK_HZ,
        ctune: HFXO_CTUNE,
    })
}

/// BRD4183A LED0 (PB1), active high.
pub struct Led0 {
    pin: Pin<PushPull>,
}

impl Led0 {
    pub(crate) fn from_pin(pin: Pin<PushPull>) -> Self {
        Self { pin }
    }

    #[inline]
    pub fn on(&mut self) {
        self.pin.set_high();
    }

    #[inline]
    pub fn off(&mut self) {
        self.pin.set_low();
    }

    #[inline]
    pub fn is_on(&self) -> bool {
        self.pin.is_set_high()
    }

    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

/// BRD4183A BTN0 (PB0), active low with the WSTK's external pull-up.
pub struct Button0 {
    pin: Pin<Input>,
}

impl Button0 {
    pub(crate) fn from_pin(pin: Pin<Input>) -> Self {
        Self { pin }
    }

    #[inline]
    pub fn is_pressed(&self) -> bool {
        !self.pin.is_high()
    }

    pub fn take_interrupt(&mut self) -> bool {
        if !self.pin.interrupt_pending() {
            return false;
        }
        self.pin.clear_interrupt();
        true
    }
}

/// Acknowledge and clear BTN0's `GPIO_EVEN` source from its ISR.
pub fn service_button0_interrupt() -> bool {
    if !gpio::interrupt_line_pending(BUTTON0_INTERRUPT_LINE) {
        return false;
    }
    // SAFETY: BRD4183A's singleton Button0 token exclusively owns EXTI0.
    unsafe { gpio::clear_interrupt_line(BUTTON0_INTERRUPT_LINE) };
    true
}

/// WSTK virtual COM port: USART1 TX on PA5, enabled through PA4.
pub struct Vcom {
    uart: UartTx,
    enable: Pin<PushPull>,
}

impl Vcom {
    pub(crate) fn new(uart: UartTx, enable: Pin<PushPull>) -> Self {
        Self { uart, enable }
    }

    #[inline]
    pub fn uart(&mut self) -> &mut UartTx {
        &mut self.uart
    }

    /// Drive the WSTK VCOM enable (PA4). Holding it low during EM2 made no
    /// measurable difference on the AEM (11.0 vs 10.8 µA).
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled {
            self.enable.set_high();
        } else {
            self.enable.set_low();
        }
    }
}

/// Force LED0 on for a terminal fault where normal ownership cannot be used.
pub fn emergency_led_on() {
    // SAFETY: terminal-only; the caller never returns to code that could use
    // the normally owned LED token.
    if let Ok(pin) = unsafe { Pin::steal(LED0_PORT, LED0_PIN) } {
        let mut pin = pin.into_push_pull(gpio::Level::Low);
        pin.set_high();
    }
}

pub(crate) fn configure_button_interrupt(pin: &mut Pin<Input>) {
    pin.configure_interrupt(InterruptEdge::Falling)
        .expect("BRD4183A BTN0 has a regular EXTI line");
}
