//! BSP for BRD4181A on the BRD4001A Wireless Starter Kit.
//!
//! This crate intentionally supports one concrete configuration:
//! EFR32MG21A010F1024IM32 (GSDK `brd4181a.slcc`), LED0 on PB0, BTN0 on PD2, and the WSTK VCOM on
//! USART0 TX PA5 with its enable on PD4 (GSDK 4.5.0
//! `hardware/board/config/brd4181a_brd4001a`). It exposes fitted board
//! resources, including the raw exclusive internal flash token, but
//! deliberately owns no Zigbee runtime or flash-partition policy.

#![no_std]

pub mod resources;

pub use efr32s2_hal::flash::Efr32s2Flash;
use efr32s2_hal::{
    clock::{ClockControl, ClockError, HfxoConfig, SystemClocks},
    gpio::{self, Input, InterruptEdge, Pin, Port, PushPull},
    usart::UartTx,
};

pub const BOARD_RADIO: &str = "BRD4181A";
pub const BOARD_MAIN: &str = "BRD4001A";
pub const MCU_PART: &str = "EFR32MG21A010F1024IM32";
pub const HCLK_HZ: u32 = 38_400_000;
pub const HFXO_CTUNE: u16 = 133;

pub const LED0_PORT: Port = Port::B;
pub const LED0_PIN: u8 = 0;
pub const BUTTON0_PORT: Port = Port::D;
pub const BUTTON0_PIN: u8 = 2;
pub const BUTTON0_INTERRUPT_LINE: u8 = 2;
pub const VCOM_ENABLE_PORT: Port = Port::D;
pub const VCOM_ENABLE_PIN: u8 = 4;
pub const VCOM_TX_PORT: Port = Port::A;
pub const VCOM_TX_PIN: u8 = 5;
pub const VCOM_BAUD: u32 = 115_200;

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

/// WSTK virtual COM port: USART0 TX on PA5, enabled through PD4.
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

    /// Drive the WSTK VCOM enable (PD4).
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
