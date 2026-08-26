//! BSP for BRD4181A on the BRD4001A Wireless Starter Kit.
//!
//! This crate intentionally supports one concrete configuration:
//! EFR32MG21A020F512IM32, LED0 on PB0, and BTN0 on PD2.
//! It exposes fitted board resources, including the raw exclusive internal
//! flash token, but deliberately owns no Zigbee runtime or flash-partition
//! policy.

#![no_std]

pub mod resources;

pub use efr32mg21_hal::flash::Efr32mg21Flash;
use efr32mg21_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    gpio::{self, Input, InterruptEdge, Pin, Port, PushPull},
};

pub const BOARD_RADIO: &str = "BRD4181A";
pub const BOARD_MAIN: &str = "BRD4001A";
pub const MCU_PART: &str = "EFR32MG21A020F512IM32";
pub const HCLK_HZ: u32 = 38_400_000;
pub const HFXO_CTUNE: u16 = 133;

pub const LED0_PORT: Port = Port::B;
pub const LED0_PIN: u8 = 0;
pub const BUTTON0_PORT: Port = Port::D;
pub const BUTTON0_PIN: u8 = 2;
pub const BUTTON0_INTERRUPT_LINE: u8 = 2;

/// Select BRD4181A's 38.4 MHz crystal before starting SysTick or radio code.
pub fn init_clocks(mut clocks: ClockControl) -> Result<SystemClocks, ClockError> {
    clocks.configure_hfxo(HfxoConfig {
        frequency_hz: HCLK_HZ,
        ctune: HFXO_CTUNE,
    })
}

/// BRD4181A LED0 (PB0), active high.
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

/// BRD4181A BTN0 (PD2), active low and externally biased by the WSTK.
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

/// Acknowledge and clear BTN0's GPIO_EVEN source from its ISR.
pub fn service_button0_interrupt() -> bool {
    if !gpio::interrupt_line_pending(BUTTON0_INTERRUPT_LINE) {
        return false;
    }
    // SAFETY: BRD4181A's singleton Button0 token exclusively owns EXTI2.
    unsafe { gpio::clear_interrupt_line(BUTTON0_INTERRUPT_LINE) };
    true
}

/// Force LED0 on for a terminal fault where normal ownership cannot be used.
pub fn emergency_led_on() {
    // SAFETY: this function is terminal-only; the caller does not return to
    // code that could use the normally owned LED token.
    if let Ok(pin) = unsafe { Pin::steal(LED0_PORT, LED0_PIN) } {
        let mut pin = pin.into_push_pull(gpio::Level::Low);
        pin.set_high();
    }
}

pub(crate) fn configure_button_interrupt(pin: &mut Pin<Input>) {
    // PD2 is supported by regular EXTI line 2 on GPIO_EVEN.
    pin.configure_interrupt(InterruptEdge::Falling)
        .expect("BRD4181A BTN0 has a regular EXTI line");
}
