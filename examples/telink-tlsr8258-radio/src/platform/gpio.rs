//! Minimal multi-port GPIO, transcribed from the sensor example's `gpio`
//! module. Used only for LED heartbeat/status, never for radio timing.

const REG_BASE: u32 = 0x800000;

#[derive(Clone, Copy)]
pub struct Pin {
    pub port_base: u32,
    pub bit: u8,
}

#[allow(dead_code)]
pub const PA: u32 = REG_BASE + 0x580;
pub const PB: u32 = REG_BASE + 0x588;
pub const PC: u32 = REG_BASE + 0x590;
#[allow(dead_code)]
pub const PD: u32 = REG_BASE + 0x598;

impl Pin {
    pub const fn new(port_base: u32, bit: u8) -> Self {
        Self { port_base, bit }
    }

    #[allow(dead_code)] // only used by the arm-gated set_output/write methods
    fn mask(self) -> u8 {
        1u8 << self.bit
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_output(self) {
        use super::mmio::{r8, w8};
        let mask = self.mask();
        unsafe {
            let v = r8(self.port_base + 6);
            w8(self.port_base + 6, v | mask); // GPIO function enable
            let v = r8(self.port_base + 2);
            w8(self.port_base + 2, v & !mask); // OEN active-low: output
            let v = r8(self.port_base + 1);
            w8(self.port_base + 1, v & !mask); // input disable
        }
    }

    #[cfg(target_arch = "tc32")]
    pub fn write(self, high: bool) {
        use super::mmio::{r8, w8};
        let mask = self.mask();
        unsafe {
            let v = r8(self.port_base + 3);
            w8(self.port_base + 3, if high { v | mask } else { v & !mask });
        }
    }
}

pub mod board {
    //! TB-04-Kit RGB LED pinout (same board as telink-tlsr8258-sensor).
    use super::Pin;
    pub const LED_RED: Pin = Pin::new(super::PC, 1);
    pub const LED_GREEN: Pin = Pin::new(super::PB, 5);
    pub const LED_BLUE: Pin = Pin::new(super::PC, 4);
}
