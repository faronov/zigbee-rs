//! Blocking TLSR8258 SPI master implementing `embedded_hal::spi::SpiBus`.
//!
//! Chip select is intentionally not part of this bus driver. Applications
//! should own a separate GPIO CS (or use an `embedded-hal-bus` device).

use embedded_hal::spi::{ErrorKind, Mode, Phase, Polarity};
#[cfg(target_arch = "tc32")]
use embedded_hal::spi::{ErrorType, SpiBus};

use crate::gpio::{Pin, Port};

const REG_SPI_DATA: u32 = crate::mmio::REG_BASE + 0x08;
const REG_SPI_CTRL: u32 = crate::mmio::REG_BASE + 0x09;
const REG_SPI_SPEED: u32 = crate::mmio::REG_BASE + 0x0A;
const REG_SPI_MODE: u32 = crate::mmio::REG_BASE + 0x0B;
const REG_PIN_I2C_SPI_OUT_EN: u32 = crate::mmio::REG_BASE + 0x5B6;
const REG_PIN_I2C_SPI_EN: u32 = crate::mmio::REG_BASE + 0x5B7;

const CTRL_MASTER_ENABLE: u8 = 1 << 1;
const CTRL_DATA_OUT_DISABLE: u8 = 1 << 2;
const CTRL_READ: u8 = 1 << 3;
const CTRL_SHARE_MODE: u8 = 1 << 5;
const CTRL_BUSY: u8 = 1 << 6;
const SPEED_ENABLE: u8 = 1 << 7;
const DUMMY_BYTE: u8 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    MostSignificantFirst,
    LeastSignificantFirst,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinGroup {
    A2A3A4,
    B7B6D7,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Config {
    pub reference_hz: u32,
    pub bus_hz: u32,
    pub mode: Mode,
    pub bit_order: BitOrder,
    pub mosi: Pin,
    pub miso: Pin,
    pub clock: Pin,
    pub timeout_iterations: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    InvalidClock,
    InvalidPins,
    InvalidTimeout,
    UnsupportedMode,
    BusTimeout,
}

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct SpiMaster {
    config: Config,
    group: PinGroup,
    divider: u8,
    actual_bus_hz: u32,
}

impl SpiMaster {
    #[cfg(target_arch = "tc32")]
    pub fn new(
        _peripheral: crate::peripherals::SerialController,
        config: Config,
    ) -> Result<Self, SpiError> {
        let (divider, actual_bus_hz) = clock_divider(config.reference_hz, config.bus_hz)?;
        let group =
            pin_group(&config.mosi, &config.miso, &config.clock).ok_or(SpiError::InvalidPins)?;
        if config.timeout_iterations == 0 {
            return Err(SpiError::InvalidTimeout);
        }
        if !matches!(config.bit_order, BitOrder::MostSignificantFirst) {
            // TLSR8258 SPI has no documented LSB-first control bit.
            return Err(SpiError::UnsupportedMode);
        }

        let controller = Self {
            config,
            group,
            divider,
            actual_bus_hz,
        };
        controller.configure_pins()?;
        controller.configure_peripheral();
        Ok(controller)
    }

    pub const fn actual_bus_hz(&self) -> u32 {
        self.actual_bus_hz
    }

    pub const fn pin_group(&self) -> PinGroup {
        self.group
    }

    #[cfg(target_arch = "tc32")]
    pub fn disable(&mut self) -> Result<(), SpiError> {
        self.flush_bus()?;
        unsafe {
            let speed = crate::mmio::r8(REG_SPI_SPEED);
            crate::mmio::w8(REG_SPI_SPEED, speed & !SPEED_ENABLE);
        }
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    fn configure_pins(&self) -> Result<(), SpiError> {
        for pin in [&self.config.mosi, &self.config.miso, &self.config.clock] {
            crate::gpio::set_function(pin, crate::gpio::PinFunction::Spi)
                .map_err(|_| SpiError::InvalidPins)?;
        }
        crate::gpio::set_input_enable(&self.config.miso, true)
            .map_err(|_| SpiError::InvalidPins)?;

        // Group-level selectors transcribed from spi_master_gpio_set().
        // 0x5b7 selects the one input path and excludes the overlapping I2C
        // input. 0x5b6 selects the matching output path.
        unsafe {
            let output = crate::mmio::r8(REG_PIN_I2C_SPI_OUT_EN);
            let input = crate::mmio::r8(REG_PIN_I2C_SPI_EN);
            let (output, input) = group_selectors(self.group, output, input);
            crate::mmio::w8(REG_PIN_I2C_SPI_OUT_EN, output);
            crate::mmio::w8(REG_PIN_I2C_SPI_EN, input);
        }
        Ok(())
    }

    #[cfg(target_arch = "tc32")]
    fn configure_peripheral(&self) {
        // Clock-gate and reset the shared SPI block via the generic
        // reg_clk_en0/reg_rst0 facade (see `crate::reset::Peripheral::Spi`)
        // instead of hand-rolling the same read-modify-write this module
        // used to perform locally.
        crate::reset::enable_clock(crate::reset::Peripheral::Spi)
            .expect("SPI has a documented reg_clk_en0 bit");
        crate::reset::pulse_reset(crate::reset::Peripheral::Spi);
        unsafe {
            crate::mmio::w8(REG_SPI_SPEED, SPEED_ENABLE | self.divider);
            let mode = crate::mmio::r8(REG_SPI_MODE);
            crate::mmio::w8(REG_SPI_MODE, (mode & !0x03) | encode_mode(self.config.mode));
            let control = crate::mmio::r8(REG_SPI_CTRL);
            crate::mmio::w8(REG_SPI_CTRL, control | CTRL_MASTER_ENABLE);
        }
    }

    #[cfg(target_arch = "tc32")]
    fn wait_not_busy(&self) -> Result<(), SpiError> {
        for _ in 0..self.config.timeout_iterations {
            if unsafe { crate::mmio::r8(REG_SPI_CTRL) } & CTRL_BUSY == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SpiError::BusTimeout)
    }

    #[cfg(target_arch = "tc32")]
    fn write_setup(&self) {
        unsafe {
            let control = crate::mmio::r8(REG_SPI_CTRL);
            crate::mmio::w8(
                REG_SPI_CTRL,
                (control | CTRL_MASTER_ENABLE)
                    & !(CTRL_DATA_OUT_DISABLE | CTRL_READ | CTRL_SHARE_MODE),
            );
        }
    }

    #[cfg(target_arch = "tc32")]
    fn read_setup(&self) {
        unsafe {
            let control = crate::mmio::r8(REG_SPI_CTRL);
            crate::mmio::w8(
                REG_SPI_CTRL,
                (control | CTRL_MASTER_ENABLE | CTRL_DATA_OUT_DISABLE | CTRL_READ)
                    & !CTRL_SHARE_MODE,
            );
        }
    }

    #[cfg(target_arch = "tc32")]
    fn transfer_byte(&self, byte: u8) -> Result<u8, SpiError> {
        unsafe { crate::mmio::w8(REG_SPI_DATA, byte) };
        self.wait_not_busy()?;
        Ok(unsafe { crate::mmio::r8(REG_SPI_DATA) })
    }

    #[cfg(target_arch = "tc32")]
    fn flush_bus(&self) -> Result<(), SpiError> {
        self.wait_not_busy()
    }
}

#[cfg(target_arch = "tc32")]
impl ErrorType for SpiMaster {
    type Error = SpiError;
}

#[cfg(target_arch = "tc32")]
impl SpiBus<u8> for SpiMaster {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        self.read_setup();

        // Reading DATA primes the receive pipeline in 8258 read mode.
        let _ = unsafe { crate::mmio::r8(REG_SPI_DATA) };
        self.wait_not_busy()?;
        let last = words.len() - 1;
        for (index, word) in words.iter_mut().enumerate() {
            if index == last {
                // Match the vendor sequence: clear RD before consuming the
                // final byte so the read does not launch an extra clock byte.
                unsafe {
                    let control = crate::mmio::r8(REG_SPI_CTRL);
                    crate::mmio::w8(REG_SPI_CTRL, control & !CTRL_READ);
                }
            }
            *word = unsafe { crate::mmio::r8(REG_SPI_DATA) };
            self.wait_not_busy()?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.write_setup();
        for &word in words {
            let _ = self.transfer_byte(word)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.write_setup();
        let count = read.len().max(write.len());
        for index in 0..count {
            let received = self.transfer_byte(write.get(index).copied().unwrap_or(DUMMY_BYTE))?;
            if let Some(slot) = read.get_mut(index) {
                *slot = received;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.write_setup();
        for word in words {
            *word = self.transfer_byte(*word)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_bus()
    }
}

fn pin_group(mosi: &Pin, miso: &Pin, clock: &Pin) -> Option<PinGroup> {
    match (
        mosi.port_and_bit(),
        miso.port_and_bit(),
        clock.port_and_bit(),
    ) {
        ((Port::A, 2), (Port::A, 3), (Port::A, 4)) => Some(PinGroup::A2A3A4),
        ((Port::B, 7), (Port::B, 6), (Port::D, 7)) => Some(PinGroup::B7B6D7),
        _ => None,
    }
}

const fn group_selectors(group: PinGroup, output: u8, input: u8) -> (u8, u8) {
    match group {
        // PAGROUP bits 4..5 plus PDGROUP bit 7.
        PinGroup::A2A3A4 => (output | 0xB0, (input | 0x03) & !0x30),
        // PBGROUP bit 6 plus PDGROUP bit 7.
        PinGroup::B7B6D7 => (output | 0xC0, (input | 0x0C) & !0xC0),
    }
}

fn clock_divider(reference_hz: u32, bus_hz: u32) -> Result<(u8, u32), SpiError> {
    if reference_hz == 0 || bus_hz == 0 || bus_hz > reference_hz / 2 {
        return Err(SpiError::InvalidClock);
    }
    let denominator = bus_hz.checked_mul(2).ok_or(SpiError::InvalidClock)?;
    let factor = reference_hz
        .checked_add(denominator - 1)
        .ok_or(SpiError::InvalidClock)?
        / denominator;
    if factor == 0 || factor > 128 {
        return Err(SpiError::InvalidClock);
    }
    let actual = reference_hz / (factor * 2);
    Ok(((factor - 1) as u8, actual))
}

const fn encode_mode(mode: Mode) -> u8 {
    match (mode.polarity, mode.phase) {
        (Polarity::IdleLow, Phase::CaptureOnFirstTransition) => 0,
        (Polarity::IdleHigh, Phase::CaptureOnFirstTransition) => 1,
        (Polarity::IdleLow, Phase::CaptureOnSecondTransition) => 2,
        (Polarity::IdleHigh, Phase::CaptureOnSecondTransition) => 3,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};

    #[test]
    fn register_map_and_control_bits_match_8258_header() {
        assert_eq!(REG_SPI_DATA, 0x800008);
        assert_eq!(REG_SPI_CTRL, 0x800009);
        assert_eq!(REG_SPI_SPEED, 0x80000A);
        assert_eq!(REG_SPI_MODE, 0x80000B);
        assert_eq!(REG_PIN_I2C_SPI_OUT_EN, 0x8005B6);
        assert_eq!(REG_PIN_I2C_SPI_EN, 0x8005B7);
        assert_eq!(CTRL_MASTER_ENABLE, 0x02);
        assert_eq!(CTRL_DATA_OUT_DISABLE, 0x04);
        assert_eq!(CTRL_READ, 0x08);
        assert_eq!(CTRL_BUSY, 0x40);
        assert_eq!(SPEED_ENABLE, 0x80);
    }

    #[test]
    fn divider_matches_tlsr8258_formula_without_overspeed() {
        assert_eq!(clock_divider(24_000_000, 4_000_000), Ok((2, 4_000_000)));
        assert_eq!(clock_divider(24_000_000, 1_000_000), Ok((11, 1_000_000)));
        assert_eq!(clock_divider(24_000_000, 3_500_000), Ok((3, 3_000_000)));
    }

    #[test]
    fn invalid_spi_clocks_are_rejected() {
        assert_eq!(clock_divider(0, 1_000_000), Err(SpiError::InvalidClock));
        assert_eq!(clock_divider(24_000_000, 0), Err(SpiError::InvalidClock));
        assert_eq!(
            clock_divider(24_000_000, 12_000_001),
            Err(SpiError::InvalidClock)
        );
        assert_eq!(
            clock_divider(24_000_000, 90_000),
            Err(SpiError::InvalidClock)
        );
    }

    #[test]
    fn pin_groups_validate_signal_order() {
        assert_eq!(
            pin_group(
                &Pin::new(Port::A, 2),
                &Pin::new(Port::A, 3),
                &Pin::new(Port::A, 4)
            ),
            Some(PinGroup::A2A3A4)
        );
        assert_eq!(
            pin_group(
                &Pin::new(Port::B, 7),
                &Pin::new(Port::B, 6),
                &Pin::new(Port::D, 7)
            ),
            Some(PinGroup::B7B6D7)
        );
        assert_eq!(
            pin_group(
                &Pin::new(Port::B, 6),
                &Pin::new(Port::B, 7),
                &Pin::new(Port::D, 7)
            ),
            None
        );
    }

    #[test]
    fn group_selectors_match_vendor_output_and_input_masks() {
        assert_eq!(group_selectors(PinGroup::A2A3A4, 0, 0xFF), (0xB0, 0xCF));
        assert_eq!(group_selectors(PinGroup::B7B6D7, 0, 0xFF), (0xC0, 0x3F));
    }

    #[test]
    fn all_standard_modes_use_documented_register_encoding() {
        assert_eq!(encode_mode(MODE_0), 0);
        assert_eq!(encode_mode(MODE_2), 1);
        assert_eq!(encode_mode(MODE_1), 2);
        assert_eq!(encode_mode(MODE_3), 3);
    }
}
