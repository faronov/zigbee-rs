//! EFR32MG1 Series 1 GPIO primitives.

use core::convert::Infallible;

use crate::clock;

const GPIO_BASE: u32 = 0x4000_A000;
const PORT_STRIDE: u32 = 0x30;
const MODEL_OFFSET: u32 = 0x04;
const MODEH_OFFSET: u32 = 0x08;
const DOUT_OFFSET: u32 = 0x0C;
const DIN_OFFSET: u32 = 0x1C;
const EXTIPSELL_OFFSET: u32 = 0x400;
const EXTIPSELH_OFFSET: u32 = 0x404;
const EXTIPINSELL_OFFSET: u32 = 0x408;
const EXTIPINSELH_OFFSET: u32 = 0x40C;
const EXTIRISE_OFFSET: u32 = 0x410;
const EXTIFALL_OFFSET: u32 = 0x414;
const IF_OFFSET: u32 = 0x41C;
const IFC_OFFSET: u32 = 0x424;
const IEN_OFFSET: u32 = 0x428;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptEdge {
    Rising,
    Falling,
    Both,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Port {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    F = 5,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    Disabled = 0,
    Input = 1,
    InputPull = 2,
    InputPullFilter = 3,
    PushPull = 4,
    WiredAnd = 8,
    WiredAndFilter = 9,
    WiredAndPullUp = 10,
    WiredAndPullUpFilter = 11,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pin {
    port: Port,
    number: u8,
}

impl Pin {
    pub const fn new(port: Port, number: u8) -> Self {
        assert!(number < 16);
        Self { port, number }
    }

    pub const fn port(self) -> Port {
        self.port
    }

    pub const fn number(self) -> u8 {
        self.number
    }

    pub fn configure(self, mode: Mode, initial_high: bool) {
        clock::enable_gpio_clock();
        if initial_high {
            self.set_high();
        } else {
            self.set_low();
        }

        let (mode_address, shift) = self.mode_register();
        unsafe {
            let current = read(mode_address);
            write(
                mode_address,
                (current & !(0xF << shift)) | ((mode as u32) << shift),
            );
        }
    }

    #[inline]
    pub fn set_high(self) {
        unsafe {
            let address = self.port_base() + DOUT_OFFSET;
            write(address, read(address) | (1 << self.number));
        }
    }

    #[inline]
    pub fn set_low(self) {
        unsafe {
            let address = self.port_base() + DOUT_OFFSET;
            write(address, read(address) & !(1 << self.number));
        }
    }

    #[inline]
    pub fn is_high(self) -> bool {
        unsafe { read(self.port_base() + DIN_OFFSET) & (1 << self.number) != 0 }
    }

    #[inline]
    pub fn output_is_high(self) -> bool {
        unsafe { read(self.port_base() + DOUT_OFFSET) & (1 << self.number) != 0 }
    }

    /// Route this pin to its same-numbered external interrupt line.
    ///
    /// EFR32xG1 groups lines by pin number: line 13 can select any port's
    /// pin 13, but not an arbitrary pin from another four-pin group.
    pub fn configure_interrupt(self, edge: InterruptEdge) {
        clock::enable_gpio_clock();
        let mask = self.interrupt_mask();
        let (port_select, port_shift) = interrupt_port_select(self.number);
        let (pin_select, pin_shift) = interrupt_pin_select(self.number);

        unsafe {
            modify(GPIO_BASE + IEN_OFFSET, mask, 0);
            modify(
                GPIO_BASE + port_select,
                0xF << port_shift,
                (self.port as u32) << port_shift,
            );
            modify(
                GPIO_BASE + pin_select,
                0x3 << pin_shift,
                ((self.number & 0x3) as u32) << pin_shift,
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
            write(GPIO_BASE + IFC_OFFSET, mask);
            modify(GPIO_BASE + IEN_OFFSET, mask, mask);
        }
    }

    #[inline]
    pub fn interrupt_pending(self) -> bool {
        unsafe { read(GPIO_BASE + IF_OFFSET) & self.interrupt_mask() != 0 }
    }

    #[inline]
    pub fn clear_interrupt(self) {
        unsafe { write(GPIO_BASE + IFC_OFFSET, self.interrupt_mask()) }
    }

    #[inline]
    pub fn disable_interrupt(self) {
        let mask = self.interrupt_mask();
        unsafe {
            modify(GPIO_BASE + IEN_OFFSET, mask, 0);
            write(GPIO_BASE + IFC_OFFSET, mask);
        }
    }

    const fn port_base(self) -> u32 {
        GPIO_BASE + (self.port as u32) * PORT_STRIDE
    }

    const fn interrupt_mask(self) -> u32 {
        1 << self.number
    }

    const fn mode_register(self) -> (u32, u32) {
        if self.number < 8 {
            (self.port_base() + MODEL_OFFSET, (self.number as u32) * 4)
        } else {
            (
                self.port_base() + MODEH_OFFSET,
                ((self.number - 8) as u32) * 4,
            )
        }
    }
}

impl embedded_hal::digital::ErrorType for Pin {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(*self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(*self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Pin {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_is_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!self.output_is_high())
    }
}

impl embedded_hal::digital::InputPin for Pin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(*self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Pin::is_high(*self))
    }
}

const fn interrupt_port_select(line: u8) -> (u32, u32) {
    if line < 8 {
        (EXTIPSELL_OFFSET, (line as u32) * 4)
    } else {
        (EXTIPSELH_OFFSET, ((line - 8) as u32) * 4)
    }
}

const fn interrupt_pin_select(line: u8) -> (u32, u32) {
    if line < 8 {
        (EXTIPINSELL_OFFSET, (line as u32) * 4)
    } else {
        (EXTIPINSELH_OFFSET, ((line - 8) as u32) * 4)
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
    unsafe { write(address, (read(address) & !mask) | (value & mask)) }
}

#[cfg(test)]
mod tests {
    use super::{
        EXTIPINSELH_OFFSET, EXTIPINSELL_OFFSET, EXTIPSELH_OFFSET, EXTIPSELL_OFFSET,
        interrupt_pin_select, interrupt_port_select,
    };

    #[test]
    fn interrupt_line_13_uses_high_route_fields() {
        assert_eq!(interrupt_port_select(13), (EXTIPSELH_OFFSET, 20));
        assert_eq!(interrupt_pin_select(13), (EXTIPINSELH_OFFSET, 20));
    }

    #[test]
    fn interrupt_line_3_uses_low_route_fields() {
        assert_eq!(interrupt_port_select(3), (EXTIPSELL_OFFSET, 12));
        assert_eq!(interrupt_pin_select(3), (EXTIPINSELL_OFFSET, 12));
    }
}
