//! Transport-independent TI TMP102 digital temperature sensor driver
//! (blocking, embedded-hal 1.0).
//!
//! The driver is built for battery devices: keep the sensor in shutdown
//! (`SD = 1`) and trigger single conversions with the one-shot bit. A
//! conversion is split into [`Tmp102::start_one_shot`] and
//! [`Tmp102::read_temperature_centi`] so the caller can wait
//! [`CONVERSION_TIME_MS`] with its own (for example async) timer instead of
//! busy-waiting. After a one-shot conversion the TMP102 returns to shutdown
//! by itself.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// I2C address with `ADD0` tied to GND.
pub const ADDRESS_GND: SevenBitAddress = 0x48;
/// I2C address with `ADD0` tied to V+.
pub const ADDRESS_VCC: SevenBitAddress = 0x49;
/// I2C address with `ADD0` tied to SDA.
pub const ADDRESS_SDA: SevenBitAddress = 0x4A;
/// I2C address with `ADD0` tied to SCL.
pub const ADDRESS_SCL: SevenBitAddress = 0x4B;

/// Datasheet maximum conversion time (26 ms typical, 35 ms maximum).
pub const CONVERSION_TIME_MS: u32 = 35;

const REG_TEMPERATURE: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;

/// Configuration register first byte: shutdown mode.
const CONFIG_SD: u8 = 1 << 0;
/// Configuration register first byte: one-shot conversion trigger.
const CONFIG_OS: u8 = 1 << 7;
/// Configuration register second byte: extended (13-bit) mode.
const CONFIG_EM: u8 = 1 << 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    /// The temperature register read back as all ones, which is what an
    /// absent or stuck device returns on a bus with pull-ups.
    InvalidReading,
}

/// Decodes the two temperature-register bytes into hundredths of a degree
/// Celsius. Bit 0 of the second byte flags 13-bit extended mode; in
/// 12-bit mode it is always zero.
pub fn decode_centi_celsius(bytes: [u8; 2]) -> Option<i16> {
    if bytes == [0xFF, 0xFF] {
        return None;
    }
    let word = i16::from_be_bytes(bytes);
    let counts = if bytes[1] & 0x01 != 0 {
        word >> 3
    } else {
        word >> 4
    };
    // One count is 0.0625 °C = 6.25 centi-degrees.
    Some(((i32::from(counts) * 25) / 4) as i16)
}

/// Blocking TMP102 driver.
pub struct Tmp102<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
}

impl<I2C> Tmp102<I2C> {
    pub const fn new(i2c: I2C, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }

    pub const fn address(&self) -> SevenBitAddress {
        self.address
    }

    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Tmp102<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Checks that a device acknowledges the address by setting the
    /// pointer register to the configuration register. This is the probe
    /// commonly used for autodetection; it does not change the configuration.
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[REG_CONFIG])
            .map_err(Error::I2c)
    }

    fn read_config(&mut self) -> Result<[u8; 2], Error<I2C::Error>> {
        let mut config = [0u8; 2];
        self.i2c
            .write_read(self.address, &[REG_CONFIG], &mut config)
            .map_err(Error::I2c)?;
        Ok(config)
    }

    fn write_config(&mut self, config: [u8; 2]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[REG_CONFIG, config[0], config[1]])
            .map_err(Error::I2c)
    }

    /// Puts the sensor into shutdown (`SD = 1`), preserving the rest of the
    /// configuration.
    pub fn shutdown(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_config()?;
        config[0] |= CONFIG_SD;
        self.write_config(config)
    }

    /// Starts one conversion and keeps the sensor in shutdown afterwards
    /// (`OS = 1`, `SD = 1`). Read the result after [`CONVERSION_TIME_MS`].
    pub fn start_one_shot(&mut self) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_config()?;
        config[0] |= CONFIG_OS | CONFIG_SD;
        self.write_config(config)
    }

    /// Whether extended (13-bit) mode is enabled.
    pub fn extended_mode(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read_config()?[1] & CONFIG_EM != 0)
    }

    /// Reads the last completed conversion in hundredths of a degree
    /// Celsius.
    pub fn read_temperature_centi(&mut self) -> Result<i16, Error<I2C::Error>> {
        let mut bytes = [0u8; 2];
        self.i2c
            .write_read(self.address, &[REG_TEMPERATURE], &mut bytes)
            .map_err(Error::I2c)?;
        decode_centi_celsius(bytes).ok_or(Error::InvalidReading)
    }

    /// Triggers a one-shot conversion, waits the worst-case conversion time
    /// and returns the result in hundredths of a degree Celsius.
    pub fn measure<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<i16, Error<I2C::Error>> {
        self.start_one_shot()?;
        delay.delay_ms(CONVERSION_TIME_MS);
        self.read_temperature_centi()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};

    #[test]
    fn decodes_datasheet_12_bit_examples() {
        // TMP102 datasheet table 8-2 (12-bit).
        assert_eq!(decode_centi_celsius([0x7F, 0xF0]), Some(12_793)); // 127.9375
        assert_eq!(decode_centi_celsius([0x64, 0x00]), Some(10_000)); // 100
        assert_eq!(decode_centi_celsius([0x19, 0x00]), Some(2_500)); // 25
        assert_eq!(decode_centi_celsius([0x00, 0x40]), Some(25)); // 0.25
        assert_eq!(decode_centi_celsius([0x00, 0x00]), Some(0));
        assert_eq!(decode_centi_celsius([0xFF, 0xC0]), Some(-25)); // -0.25
        assert_eq!(decode_centi_celsius([0xE7, 0x00]), Some(-2_500)); // -25
        assert_eq!(decode_centi_celsius([0xC9, 0x00]), Some(-5_500)); // -55
    }

    #[test]
    fn decodes_datasheet_13_bit_examples() {
        // Extended mode: 13-bit value left-aligned, bit 0 set.
        assert_eq!(decode_centi_celsius([0x4B, 0x01]), Some(15_000)); // 150
        assert_eq!(decode_centi_celsius([0x0C, 0x81]), Some(2_500)); // 25
        assert_eq!(decode_centi_celsius([0xF3, 0x81]), Some(-2_500)); // -25
    }

    #[test]
    fn all_ones_is_invalid() {
        assert_eq!(decode_centi_celsius([0xFF, 0xFF]), None);
    }

    /// Register-level fake: tracks the pointer register and the 16-bit
    /// configuration, and records every write.
    struct FakeTmp102 {
        present: bool,
        pointer: u8,
        config: [u8; 2],
        temperature: [u8; 2],
        writes: heapless::Vec<heapless::Vec<u8, 3>, 8>,
    }

    impl FakeTmp102 {
        fn new() -> Self {
            Self {
                present: true,
                pointer: 0,
                // Power-on default: continuous, 4 Hz, 12-bit.
                config: [0x60, 0xA0],
                temperature: [0x19, 0x00],
                writes: heapless::Vec::new(),
            }
        }
    }

    impl ErrorType for FakeTmp102 {
        type Error = ErrorKind;
    }

    impl I2c for FakeTmp102 {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            if !self.present || address != ADDRESS_GND {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        self.writes
                            .push(heapless::Vec::from_slice(bytes).unwrap())
                            .unwrap();
                        self.pointer = bytes[0];
                        if self.pointer == REG_CONFIG && bytes.len() == 3 {
                            // OS reads back as 0 while converting.
                            self.config = [bytes[1] & !CONFIG_OS, bytes[2]];
                        }
                    }
                    Operation::Read(buffer) => {
                        let register = match self.pointer {
                            REG_CONFIG => self.config,
                            REG_TEMPERATURE => self.temperature,
                            _ => [0, 0],
                        };
                        buffer.copy_from_slice(&register);
                    }
                }
            }
            Ok(())
        }
    }

    struct NoDelay;

    impl embedded_hal::delay::DelayNs for NoDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn probe_only_sets_the_pointer() {
        let mut sensor = Tmp102::new(FakeTmp102::new(), ADDRESS_GND);
        sensor.probe().unwrap();
        let fake = sensor.release();
        assert_eq!(fake.writes.len(), 1);
        assert_eq!(fake.writes[0].as_slice(), [REG_CONFIG]);
        assert_eq!(fake.config, [0x60, 0xA0]);
    }

    #[test]
    fn probe_reports_missing_device() {
        let mut fake = FakeTmp102::new();
        fake.present = false;
        let mut sensor = Tmp102::new(fake, ADDRESS_GND);
        assert!(matches!(sensor.probe(), Err(Error::I2c(_))));
    }

    #[test]
    fn one_shot_sets_os_and_sd_and_preserves_second_byte() {
        let mut sensor = Tmp102::new(FakeTmp102::new(), ADDRESS_GND);
        let centi = sensor.measure(&mut NoDelay).unwrap();
        assert_eq!(centi, 2_500);
        let fake = sensor.release();
        assert_eq!(
            fake.writes[1].as_slice(),
            [REG_CONFIG, 0x60 | CONFIG_OS | CONFIG_SD, 0xA0]
        );
        assert_eq!(fake.writes.last().unwrap().as_slice(), [REG_TEMPERATURE]);
        // The device is left in shutdown after the conversion.
        assert_eq!(fake.config[0] & CONFIG_SD, CONFIG_SD);
    }

    #[test]
    fn shutdown_sets_only_sd() {
        let mut sensor = Tmp102::new(FakeTmp102::new(), ADDRESS_GND);
        sensor.shutdown().unwrap();
        let fake = sensor.release();
        assert_eq!(fake.config, [0x61, 0xA0]);
    }

    #[test]
    fn floating_bus_reading_is_rejected() {
        let mut fake = FakeTmp102::new();
        fake.temperature = [0xFF, 0xFF];
        let mut sensor = Tmp102::new(fake, ADDRESS_GND);
        assert_eq!(sensor.read_temperature_centi(), Err(Error::InvalidReading));
    }
}
