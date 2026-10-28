//! Transport-independent TI OPT3001 ambient light sensor driver (blocking,
//! embedded-hal 1.0).
//!
//! The OPT3001 powers up in shutdown. For battery devices, trigger
//! single-shot conversions with [`Opt3001::start_single_shot`]; the sensor
//! returns to shutdown by itself once the conversion completes. Wait
//! [`ConversionTime::max_delay_ms`] with the caller's own timer, then call
//! [`Opt3001::read_lux_centi`].

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// I2C address with `ADDR` tied to GND.
pub const ADDRESS_GND: SevenBitAddress = 0x44;
/// I2C address with `ADDR` tied to VDD.
pub const ADDRESS_VDD: SevenBitAddress = 0x45;
/// I2C address with `ADDR` tied to SDA.
pub const ADDRESS_SDA: SevenBitAddress = 0x46;
/// I2C address with `ADDR` tied to SCL.
pub const ADDRESS_SCL: SevenBitAddress = 0x47;

/// Value of the manufacturer ID register ("TI").
pub const MANUFACTURER_ID: u16 = 0x5449;
/// Value of the device ID register.
pub const DEVICE_ID: u16 = 0x3001;

const REG_RESULT: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_MANUFACTURER_ID: u8 = 0x7E;
const REG_DEVICE_ID: u8 = 0x7F;

/// Automatic full-scale range selection (RN = 0b1100).
const CONFIG_RANGE_AUTO: u16 = 0b1100 << 12;
/// Conversion time field (CT).
const CONFIG_CT_800MS: u16 = 1 << 11;
/// Mode of conversion field (M[1:0]): single-shot.
const CONFIG_MODE_SINGLE_SHOT: u16 = 0b01 << 9;
/// Mode of conversion field mask.
const CONFIG_MODE_MASK: u16 = 0b11 << 9;
/// Conversion-ready flag (CRF), read only.
const CONFIG_CRF: u16 = 1 << 7;
/// Latch field (L): the power-on default, kept for compatibility.
const CONFIG_LATCH: u16 = 1 << 4;

/// Conversion (integration) time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConversionTime {
    Ms100,
    Ms800,
}

impl ConversionTime {
    /// Datasheet maximum conversion time for this setting.
    pub const fn max_delay_ms(self) -> u32 {
        match self {
            ConversionTime::Ms100 => 110,
            ConversionTime::Ms800 => 880,
        }
    }

    const fn config_bits(self) -> u16 {
        match self {
            ConversionTime::Ms100 => 0,
            ConversionTime::Ms800 => CONFIG_CT_800MS,
        }
    }
}

/// Configuration word that starts one auto-ranged conversion.
pub const fn single_shot_config(conversion_time: ConversionTime) -> u16 {
    CONFIG_RANGE_AUTO | conversion_time.config_bits() | CONFIG_MODE_SINGLE_SHOT | CONFIG_LATCH
}

/// Decodes the result register into centi-lux: `lux = 0.01 * 2^E * R`,
/// so centi-lux is exactly `R << E`.
pub const fn decode_lux_centi(raw: u16) -> u32 {
    let exponent = (raw >> 12) as u32;
    let mantissa = (raw & 0x0FFF) as u32;
    // E is at most 11 on a real device; clamp so a corrupt value cannot
    // overflow.
    let exponent = if exponent > 11 { 11 } else { exponent };
    mantissa << exponent
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    /// The device acknowledged but did not identify as an OPT3001.
    WrongDevice,
}

/// Blocking OPT3001 driver.
pub struct Opt3001<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
}

impl<I2C> Opt3001<I2C> {
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

impl<I2C> Opt3001<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
        let mut bytes = [0u8; 2];
        self.i2c
            .write_read(self.address, &[register], &mut bytes)
            .map_err(Error::I2c)?;
        Ok(u16::from_be_bytes(bytes))
    }

    fn write_config(&mut self, config: u16) -> Result<(), Error<I2C::Error>> {
        let [high, low] = config.to_be_bytes();
        self.i2c
            .write(self.address, &[REG_CONFIG, high, low])
            .map_err(Error::I2c)
    }

    /// Verifies the manufacturer and device ID registers. The reference
    /// firmware checks only the manufacturer ID; checking the device ID too
    /// rejects other TI parts sharing the address.
    pub fn probe(&mut self) -> Result<(), Error<I2C::Error>> {
        if self.read_register(REG_MANUFACTURER_ID)? != MANUFACTURER_ID
            || self.read_register(REG_DEVICE_ID)? != DEVICE_ID
        {
            return Err(Error::WrongDevice);
        }
        Ok(())
    }

    /// Starts one auto-ranged conversion. The sensor shuts down by itself
    /// once the result is available.
    pub fn start_single_shot(
        &mut self,
        conversion_time: ConversionTime,
    ) -> Result<(), Error<I2C::Error>> {
        self.write_config(single_shot_config(conversion_time))
    }

    /// Whether the last conversion has completed (CRF).
    pub fn conversion_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read_register(REG_CONFIG)? & CONFIG_CRF != 0)
    }

    /// Forces shutdown mode (M = 0b00), e.g. after an aborted conversion.
    pub fn shutdown(&mut self) -> Result<(), Error<I2C::Error>> {
        let config = self.read_register(REG_CONFIG)?;
        self.write_config(config & !CONFIG_MODE_MASK)
    }

    /// Reads the result register as centi-lux.
    pub fn read_lux_centi(&mut self) -> Result<u32, Error<I2C::Error>> {
        Ok(decode_lux_centi(self.read_register(REG_RESULT)?))
    }

    /// Runs one single-shot conversion and returns centi-lux.
    pub fn measure<D: embedded_hal::delay::DelayNs>(
        &mut self,
        conversion_time: ConversionTime,
        delay: &mut D,
    ) -> Result<u32, Error<I2C::Error>> {
        self.start_single_shot(conversion_time)?;
        delay.delay_ms(conversion_time.max_delay_ms());
        self.read_lux_centi()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{ErrorKind, ErrorType, NoAcknowledgeSource, Operation};

    #[test]
    fn single_shot_config_words() {
        assert_eq!(single_shot_config(ConversionTime::Ms100), 0xC210);
        assert_eq!(single_shot_config(ConversionTime::Ms800), 0xCA10);
    }

    #[test]
    fn decodes_datasheet_result_examples() {
        // OPT3001 datasheet table 9 examples.
        assert_eq!(decode_lux_centi(0x0001), 1); // 0.01 lx
        assert_eq!(decode_lux_centi(0x3456), 0x456 << 3); // 88.80 lx
        assert_eq!(decode_lux_centi(0x7800), 0x800 << 7); // 2621.44 lx
        assert_eq!(decode_lux_centi(0xB7FF), 0x7FF << 11); // 41932.80 lx
        assert_eq!(decode_lux_centi(0xBFFF), 8_386_560); // 83865.60 lx
    }

    #[test]
    fn corrupt_exponent_is_clamped() {
        assert_eq!(decode_lux_centi(0xFFFF), 0xFFF << 11);
    }

    struct FakeOpt3001 {
        manufacturer: u16,
        pointer: u8,
        config: u16,
        result: u16,
        writes: heapless::Vec<heapless::Vec<u8, 3>, 8>,
    }

    impl FakeOpt3001 {
        fn new() -> Self {
            Self {
                manufacturer: MANUFACTURER_ID,
                pointer: 0,
                config: 0xC810,
                result: 0x3456,
                writes: heapless::Vec::new(),
            }
        }
    }

    impl ErrorType for FakeOpt3001 {
        type Error = ErrorKind;
    }

    impl I2c for FakeOpt3001 {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            if address != ADDRESS_GND {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        self.writes
                            .push(heapless::Vec::from_slice(bytes).unwrap())
                            .unwrap();
                        self.pointer = bytes[0];
                        if bytes.len() == 3 && self.pointer == REG_CONFIG {
                            self.config = u16::from_be_bytes([bytes[1], bytes[2]]);
                        }
                    }
                    Operation::Read(buffer) => {
                        let value = match self.pointer {
                            REG_RESULT => self.result,
                            REG_CONFIG => self.config,
                            REG_MANUFACTURER_ID => self.manufacturer,
                            REG_DEVICE_ID => DEVICE_ID,
                            _ => 0,
                        };
                        buffer.copy_from_slice(&value.to_be_bytes());
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
    fn probe_checks_identity() {
        let mut sensor = Opt3001::new(FakeOpt3001::new(), ADDRESS_GND);
        sensor.probe().unwrap();

        let mut fake = FakeOpt3001::new();
        fake.manufacturer = 0x1234;
        let mut sensor = Opt3001::new(fake, ADDRESS_GND);
        assert_eq!(sensor.probe(), Err(Error::WrongDevice));

        let mut sensor = Opt3001::new(FakeOpt3001::new(), ADDRESS_VDD);
        assert!(matches!(sensor.probe(), Err(Error::I2c(_))));
    }

    #[test]
    fn measure_writes_single_shot_then_reads_result() {
        let mut sensor = Opt3001::new(FakeOpt3001::new(), ADDRESS_GND);
        let lux = sensor.measure(ConversionTime::Ms100, &mut NoDelay).unwrap();
        assert_eq!(lux, 8_880);
        let fake = sensor.release();
        assert_eq!(fake.writes[0].as_slice(), [REG_CONFIG, 0xC2, 0x10]);
        assert_eq!(fake.writes[1].as_slice(), [REG_RESULT]);
    }

    #[test]
    fn shutdown_clears_only_the_mode() {
        let mut fake = FakeOpt3001::new();
        fake.config = 0xCE90;
        let mut sensor = Opt3001::new(fake, ADDRESS_GND);
        sensor.shutdown().unwrap();
        assert_eq!(sensor.release().config, 0xC890);
    }
}
