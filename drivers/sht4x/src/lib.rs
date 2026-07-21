//! Transport-independent SHT4x protocol, with both a blocking (embedded-hal
//! 1.0) and an async (embedded-hal-async 1.0) API.
//!
//! The blocking API lives at the crate root as [`Sht4x`]; the async API
//! lives in [`asynch`] under the same type name (the `asynch` module name
//! avoids clashing with the `async` keyword). Both share the same command
//! bytes, CRC routine and raw-to-engineering-unit conversion, implemented
//! exactly once at the crate root.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// The only I2C address used by SHT4x parts (address selection is done via
/// part number/marking, not an address pin).
pub const ADDRESS: SevenBitAddress = 0x44;

const CMD_SOFT_RESET: u8 = 0x94;
const CMD_READ_SERIAL: u8 = 0x89;
const CMD_MEASURE_HIGH_PRECISION: u8 = 0xFD;
const CMD_MEASURE_MEDIUM_PRECISION: u8 = 0xF6;
const CMD_MEASURE_LOW_PRECISION: u8 = 0xE0;

/// Measurement repeatability, trading conversion time for noise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Precision {
    High,
    Medium,
    Low,
}

impl Precision {
    const fn command(self) -> u8 {
        match self {
            Precision::High => CMD_MEASURE_HIGH_PRECISION,
            Precision::Medium => CMD_MEASURE_MEDIUM_PRECISION,
            Precision::Low => CMD_MEASURE_LOW_PRECISION,
        }
    }

    /// Datasheet worst-case conversion time in milliseconds.
    const fn delay_ms(self) -> u32 {
        match self {
            Precision::High => 9,
            Precision::Medium => 5,
            Precision::Low => 2,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcField {
    Temperature,
    Humidity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    Crc {
        field: CrcField,
        expected: u8,
        received: u8,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in hundredths of a degree Celsius (centi-degrees).
    pub temperature_centi_celsius: i32,
    /// Relative humidity in hundredths of a percent (centi-percent),
    /// clamped to the sensor's `0..=100%` physical range.
    pub humidity_centi_percent: u16,
}

/// Sensirion CRC-8, polynomial 0x31, initial value 0xFF (shared by all
/// Sensirion I2C sensors).
pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0xFFu8;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn validate_crc<E>(field: CrcField, bytes: &[u8], received: u8) -> Result<(), Error<E>> {
    let expected = crc8(bytes);
    if expected == received {
        Ok(())
    } else {
        Err(Error::Crc {
            field,
            expected,
            received,
        })
    }
}

/// Datasheet equations (1)/(2): `T = -45 + 175 * raw / 65535`,
/// `RH = -6 + 125 * raw / 65535`, humidity clamped to `0..=100%`.
pub fn decode_measurement(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let temperature = -4_500i32 + (17_500i64 * i64::from(raw_temperature) / 65_535) as i32;
    let humidity_raw = -600i32 + (12_500i64 * i64::from(raw_humidity) / 65_535) as i32;
    Measurement {
        temperature_centi_celsius: temperature,
        humidity_centi_percent: humidity_raw.clamp(0, 10_000) as u16,
    }
}

fn parse_response<E>(response: &[u8; 6]) -> Result<Measurement, Error<E>> {
    validate_crc(CrcField::Temperature, &response[..2], response[2])?;
    validate_crc(CrcField::Humidity, &response[3..5], response[5])?;
    Ok(decode_measurement(
        u16::from_be_bytes([response[0], response[1]]),
        u16::from_be_bytes([response[3], response[4]]),
    ))
}

/// Blocking SHT4x driver.
pub struct Sht4x<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
}

impl<I2C> Sht4x<I2C> {
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

impl<I2C> Sht4x<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Send soft reset. The caller must wait at least 1 ms before further
    /// commands.
    pub fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[CMD_SOFT_RESET])
            .map_err(Error::I2c)
    }

    /// Reads the 32-bit serial number, CRC-checked per 16-bit half-word.
    pub fn read_serial_number(&mut self) -> Result<u32, Error<I2C::Error>> {
        let mut response = [0u8; 6];
        self.i2c
            .write_read(self.address, &[CMD_READ_SERIAL], &mut response)
            .map_err(Error::I2c)?;
        validate_crc(CrcField::Temperature, &response[..2], response[2])?;
        validate_crc(CrcField::Humidity, &response[3..5], response[5])?;
        let high = u16::from_be_bytes([response[0], response[1]]);
        let low = u16::from_be_bytes([response[3], response[4]]);
        Ok((u32::from(high) << 16) | u32::from(low))
    }

    /// Trigger a measurement at the given precision. The caller must wait
    /// [`Precision::delay_ms`] before [`Self::read_measurement`].
    pub fn start_measurement(&mut self, precision: Precision) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[precision.command()])
            .map_err(Error::I2c)
    }

    /// Read and CRC-check a previously triggered measurement.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut response = [0u8; 6];
        self.i2c
            .read(self.address, &mut response)
            .map_err(Error::I2c)?;
        parse_response(&response)
    }

    /// Trigger a measurement, wait the required conversion time and read
    /// back the result.
    pub fn measure<D: embedded_hal::delay::DelayNs>(
        &mut self,
        precision: Precision,
        delay: &mut D,
    ) -> Result<Measurement, Error<I2C::Error>> {
        self.start_measurement(precision)?;
        delay.delay_ms(precision.delay_ms());
        self.read_measurement()
    }
}

/// Async (embedded-hal-async 1.0) counterpart of the blocking API. See the
/// crate-root [`super::Sht4x`]; command bytes, CRC and conversion logic are
/// shared, not duplicated.
pub mod asynch {
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};

    use super::{
        CMD_READ_SERIAL, CMD_SOFT_RESET, CrcField, Error, Measurement, Precision, parse_response,
        validate_crc,
    };

    pub struct Sht4x<I2C> {
        i2c: I2C,
        address: SevenBitAddress,
    }

    impl<I2C> Sht4x<I2C> {
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

    impl<I2C> Sht4x<I2C>
    where
        I2C: I2c<SevenBitAddress>,
    {
        pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[CMD_SOFT_RESET])
                .await
                .map_err(Error::I2c)
        }

        pub async fn read_serial_number(&mut self) -> Result<u32, Error<I2C::Error>> {
            let mut response = [0u8; 6];
            self.i2c
                .write_read(self.address, &[CMD_READ_SERIAL], &mut response)
                .await
                .map_err(Error::I2c)?;
            validate_crc(CrcField::Temperature, &response[..2], response[2])?;
            validate_crc(CrcField::Humidity, &response[3..5], response[5])?;
            let high = u16::from_be_bytes([response[0], response[1]]);
            let low = u16::from_be_bytes([response[3], response[4]]);
            Ok((u32::from(high) << 16) | u32::from(low))
        }

        pub async fn start_measurement(
            &mut self,
            precision: Precision,
        ) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[precision.command()])
                .await
                .map_err(Error::I2c)
        }

        pub async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut response = [0u8; 6];
            self.i2c
                .read(self.address, &mut response)
                .await
                .map_err(Error::I2c)?;
            parse_response(&response)
        }

        pub async fn measure<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            precision: Precision,
            delay: &mut D,
        ) -> Result<Measurement, Error<I2C::Error>> {
            self.start_measurement(precision).await?;
            delay.delay_ms(precision.delay_ms()).await;
            self.read_measurement().await
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_matches_sensirion_example() {
        assert_eq!(crc8(&[0xBE, 0xEF]), 0x92);
    }

    #[test]
    fn conversion_endpoints_match_datasheet() {
        let low = decode_measurement(0, 0);
        assert_eq!(low.temperature_centi_celsius, -4_500);
        assert_eq!(low.humidity_centi_percent, 0); // clamped from -6%

        let high = decode_measurement(u16::MAX, u16::MAX);
        assert_eq!(high.temperature_centi_celsius, 13_000);
        assert_eq!(high.humidity_centi_percent, 10_000); // clamped from 119%
    }

    #[test]
    fn humidity_clamp_boundary_is_exact() {
        // RH crosses 0% at raw = 6/125*65535 = 3146.4, so raw=3147 should be
        // just above zero and raw=3146 should clamp to zero.
        assert_eq!(decode_measurement(0, 3_146).humidity_centi_percent, 0);
        assert!(decode_measurement(0, 3_200).humidity_centi_percent > 0);
    }

    #[test]
    fn read_measurement_reports_crc_error() {
        let mut response = [0x66, 0x66, 0, 0x66, 0x66, 0];
        response[2] = crc8(&response[..2]);
        response[5] = crc8(&response[3..5]);
        response[5] ^= 0xFF; // corrupt humidity CRC

        let err: Result<Measurement, Error<core::convert::Infallible>> = parse_response(&response);
        assert_eq!(
            err,
            Err(Error::Crc {
                field: CrcField::Humidity,
                expected: crc8(&response[3..5]),
                received: response[5],
            })
        );
    }

    #[test]
    fn precision_commands_and_delays_match_datasheet() {
        assert_eq!(Precision::High.command(), 0xFD);
        assert_eq!(Precision::Medium.command(), 0xF6);
        assert_eq!(Precision::Low.command(), 0xE0);
        assert!(Precision::High.delay_ms() >= Precision::Medium.delay_ms());
        assert!(Precision::Medium.delay_ms() >= Precision::Low.delay_ms());
    }

    /// Fake I2C bus that records the sequence of operations and serves a
    /// canned response, to verify `measure` issues "start" before "read".
    struct FakeI2c {
        calls: heapless::Vec<&'static str, 4>,
        response: [u8; 6],
    }

    impl embedded_hal::i2c::ErrorType for FakeI2c {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal::i2c::I2c for FakeI2c {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    embedded_hal::i2c::Operation::Write(bytes) => {
                        if bytes == &[CMD_MEASURE_HIGH_PRECISION] {
                            self.calls.push("start").unwrap();
                        }
                    }
                    embedded_hal::i2c::Operation::Read(buf) => {
                        self.calls.push("read").unwrap();
                        buf.copy_from_slice(&self.response);
                    }
                }
            }
            Ok(())
        }
    }

    struct FakeDelay;

    impl embedded_hal::delay::DelayNs for FakeDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    fn canned_response() -> [u8; 6] {
        let mut response = [0x50, 0x00, 0, 0x80, 0x00, 0];
        response[2] = crc8(&response[..2]);
        response[5] = crc8(&response[3..5]);
        response
    }

    #[test]
    fn measure_issues_start_then_read_and_decodes_result() {
        let i2c = FakeI2c {
            calls: heapless::Vec::new(),
            response: canned_response(),
        };
        let expected = decode_measurement(0x5000, 0x8000);

        let mut sensor = Sht4x::new(i2c, ADDRESS);
        let measurement = sensor.measure(Precision::High, &mut FakeDelay).unwrap();

        assert_eq!(measurement, expected);
        assert_eq!(sensor.release().calls.as_slice(), ["start", "read"]);
    }
}
