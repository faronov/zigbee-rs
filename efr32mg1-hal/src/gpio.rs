//! EFR32MG1 Series 1 GPIO primitives.

use crate::clock;

const GPIO_BASE: u32 = 0x4000_A000;
const PORT_STRIDE: u32 = 0x30;
const MODEL_OFFSET: u32 = 0x04;
const MODEH_OFFSET: u32 = 0x08;
const DOUT_OFFSET: u32 = 0x0C;
const DIN_OFFSET: u32 = 0x1C;

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

    const fn port_base(self) -> u32 {
        GPIO_BASE + (self.port as u32) * PORT_STRIDE
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

#[inline]
unsafe fn read(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}
