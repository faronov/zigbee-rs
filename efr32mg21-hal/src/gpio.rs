//! Typed EFR32MG21 Series-2 GPIO primitives.
//!
//! The register layout is the Series-2 layout at `0x4003_C000`; it is not
//! compatible with the Series-1 GPIO block at `0x4000_A000`.

use core::marker::PhantomData;

const GPIO_BASE: u32 = 0x4003_C000;
const PORT_STRIDE: u32 = 0x30;
const MODEL_OFFSET: u32 = 0x04;
const MODEH_OFFSET: u32 = 0x0C;
const DOUT_OFFSET: u32 = 0x10;
const DIN_OFFSET: u32 = 0x14;

const LOCK_OFFSET: u32 = 0x300;
const EXTIPSELL_OFFSET: u32 = 0x400;
const EXTIPINSELL_OFFSET: u32 = 0x408;
const EXTIRISE_OFFSET: u32 = 0x410;
const EXTIFALL_OFFSET: u32 = 0x414;
const IF_OFFSET: u32 = 0x420;
const IEN_OFFSET: u32 = 0x424;

const SET_ALIAS_OFFSET: u32 = 0x1000;
const CLEAR_ALIAS_OFFSET: u32 = 0x2000;
const GPIO_LOCK_UNLOCK: u32 = 0xA534;

const MODE_INPUT: u32 = 1;
const MODE_INPUT_PULL: u32 = 2;
const MODE_PUSH_PULL: u32 = 4;

/// GPIO port indices implemented by EFR32MG21.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Port {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Low,
    High,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    /// Plain input. The board must provide any required external bias.
    None,
    Up,
    Down,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptEdge {
    Rising,
    Falling,
    Both,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioError {
    InvalidPin,
    InterruptLineUnavailable,
}

pub struct Disabled;
pub struct Input;
pub struct PushPull;

/// Ownership token for the GPIO block.
pub struct Gpio {
    _private: (),
}

impl Gpio {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Claim one physical pin.
    ///
    /// # Safety
    ///
    /// The caller must ensure that no other live [`Pin`] names the same
    /// `(port, number)`. Board resource singletons are the intended caller.
    pub unsafe fn claim_pin(&mut self, port: Port, number: u8) -> Result<Pin<Disabled>, GpioError> {
        unsafe { Pin::steal(port, number) }
    }
}

/// Uniquely owned GPIO pin in a type-level mode.
pub struct Pin<MODE> {
    port: Port,
    number: u8,
    _mode: PhantomData<MODE>,
}

impl<MODE> Pin<MODE> {
    pub const fn port(&self) -> Port {
        self.port
    }

    pub const fn number(&self) -> u8 {
        self.number
    }

    const fn port_base(&self) -> u32 {
        GPIO_BASE + (self.port as u32) * PORT_STRIDE
    }

    const fn mode_register(&self) -> (u32, u32) {
        if self.number < 8 {
            (self.port_base() + MODEL_OFFSET, (self.number as u32) * 4)
        } else {
            (
                self.port_base() + MODEH_OFFSET,
                ((self.number - 8) as u32) * 4,
            )
        }
    }

    const fn interrupt_mask(&self) -> u32 {
        1 << self.number
    }

    fn set_latch(&self, level: Level) {
        let alias = match level {
            Level::Low => CLEAR_ALIAS_OFFSET,
            Level::High => SET_ALIAS_OFFSET,
        };
        unsafe {
            write(self.port_base() + alias + DOUT_OFFSET, 1 << self.number);
        }
    }

    fn configure_mode(&self, mode: u32) {
        let (address, shift) = self.mode_register();
        unsafe {
            write(GPIO_BASE + LOCK_OFFSET, GPIO_LOCK_UNLOCK);
            modify(address, 0xF << shift, mode << shift);
        }
    }
}

impl Pin<Disabled> {
    /// Construct a pin without consulting the GPIO ownership token.
    ///
    /// # Safety
    ///
    /// The caller must guarantee unique ownership. This exists for board
    /// singleton construction and terminal fault indication only.
    pub unsafe fn steal(port: Port, number: u8) -> Result<Self, GpioError> {
        if number >= 16 {
            return Err(GpioError::InvalidPin);
        }
        Ok(Self {
            port,
            number,
            _mode: PhantomData,
        })
    }

    pub fn into_push_pull(self, initial: Level) -> Pin<PushPull> {
        self.set_latch(initial);
        self.configure_mode(MODE_PUSH_PULL);
        Pin {
            port: self.port,
            number: self.number,
            _mode: PhantomData,
        }
    }

    pub fn into_input(self, pull: Pull) -> Pin<Input> {
        let (mode, latch) = match pull {
            Pull::None => (MODE_INPUT, Level::Low),
            Pull::Up => (MODE_INPUT_PULL, Level::High),
            Pull::Down => (MODE_INPUT_PULL, Level::Low),
        };
        self.set_latch(latch);
        self.configure_mode(mode);
        Pin {
            port: self.port,
            number: self.number,
            _mode: PhantomData,
        }
    }
}

impl Pin<PushPull> {
    #[inline]
    pub fn set_high(&mut self) {
        self.set_latch(Level::High);
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.set_latch(Level::Low);
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        unsafe { read(self.port_base() + DOUT_OFFSET) & (1 << self.number) != 0 }
    }

    #[inline]
    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

impl Pin<Input> {
    #[inline]
    pub fn is_high(&self) -> bool {
        unsafe { read(self.port_base() + DIN_OFFSET) & (1 << self.number) != 0 }
    }

    /// Route this pin to its same-numbered Series-2 EXTI channel.
    ///
    /// EFR32MG21 exposes eight regular EXTI channels. Channel `n` selects one
    /// of pins `n`, `n + 4`, `n + 8`, or `n + 12`; therefore this helper is
    /// deliberately limited to pins 0 through 7.
    pub fn configure_interrupt(&mut self, edge: InterruptEdge) -> Result<(), GpioError> {
        if self.number >= 8 {
            return Err(GpioError::InterruptLineUnavailable);
        }

        let line = self.number;
        let mask = self.interrupt_mask();
        let shift = (line as u32) * 4;
        unsafe {
            write(GPIO_BASE + LOCK_OFFSET, GPIO_LOCK_UNLOCK);
            write(GPIO_BASE + CLEAR_ALIAS_OFFSET + IEN_OFFSET, mask);
            modify(
                GPIO_BASE + EXTIPSELL_OFFSET,
                0x3 << shift,
                (self.port as u32) << shift,
            );
            modify(
                GPIO_BASE + EXTIPINSELL_OFFSET,
                0x3 << shift,
                ((self.number % 4) as u32) << shift,
            );
            modify(
                GPIO_BASE + EXTIRISE_OFFSET,
                mask,
                if matches!(edge, InterruptEdge::Rising | InterruptEdge::Both) {
                    mask
                } else {
                    0
                },
            );
            modify(
                GPIO_BASE + EXTIFALL_OFFSET,
                mask,
                if matches!(edge, InterruptEdge::Falling | InterruptEdge::Both) {
                    mask
                } else {
                    0
                },
            );
            clear_interrupt_line(line);
            write(GPIO_BASE + SET_ALIAS_OFFSET + IEN_OFFSET, mask);
        }
        Ok(())
    }

    #[inline]
    pub fn interrupt_pending(&self) -> bool {
        interrupt_line_pending(self.number)
    }

    #[inline]
    pub fn clear_interrupt(&mut self) {
        unsafe { clear_interrupt_line(self.number) }
    }

    pub fn disable_interrupt(&mut self) {
        let mask = self.interrupt_mask();
        unsafe {
            write(GPIO_BASE + CLEAR_ALIAS_OFFSET + IEN_OFFSET, mask);
            clear_interrupt_line(self.number);
        }
    }
}

/// Read one EXTI flag without constructing a pin alias.
#[inline]
pub fn interrupt_line_pending(line: u8) -> bool {
    line < 16 && unsafe { read(GPIO_BASE + IF_OFFSET) & (1 << line) != 0 }
}

/// Clear one EXTI flag without constructing a pin alias.
///
/// # Safety
///
/// The caller must own the interrupt line or be its interrupt handler.
#[inline]
pub unsafe fn clear_interrupt_line(line: u8) {
    if line < 16 {
        unsafe {
            write(GPIO_BASE + CLEAR_ALIAS_OFFSET + IF_OFFSET, 1 << line);
        }
    }
}

#[inline]
unsafe fn read(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[inline]
unsafe fn modify(address: u32, mask: u32, value: u32) {
    let current = unsafe { read(address) };
    unsafe { write(address, (current & !mask) | (value & mask)) };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn series_two_pin_registers_match_brd4181a_pins() {
        let led = Pin::<Disabled> {
            port: Port::B,
            number: 0,
            _mode: PhantomData,
        };
        assert_eq!(led.mode_register(), (0x4003_C034, 0));
        assert_eq!(led.port_base() + DOUT_OFFSET, 0x4003_C040);
        assert_eq!(
            led.port_base() + SET_ALIAS_OFFSET + DOUT_OFFSET,
            0x4003_D040
        );

        let button = Pin::<Disabled> {
            port: Port::D,
            number: 2,
            _mode: PhantomData,
        };
        assert_eq!(button.mode_register(), (0x4003_C094, 8));
        assert_eq!(button.port_base() + DIN_OFFSET, 0x4003_C0A4);
    }

    #[test]
    fn interrupt_registers_are_series_two_aliases() {
        assert_eq!(GPIO_BASE + EXTIPSELL_OFFSET, 0x4003_C400);
        assert_eq!(GPIO_BASE + EXTIPINSELL_OFFSET, 0x4003_C408);
        assert_eq!(GPIO_BASE + CLEAR_ALIAS_OFFSET + IF_OFFSET, 0x4003_E420);
        assert_eq!(GPIO_BASE + SET_ALIAS_OFFSET + IEN_OFFSET, 0x4003_D424);
    }
}
