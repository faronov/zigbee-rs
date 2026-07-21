//! Transport-independent Sensirion SCD4x (CO2/temperature/humidity)
//! protocol, with both a blocking (embedded-hal 1.0) and an async
//! (embedded-hal-async 1.0) API.
//!
//! The blocking API lives at the crate root as [`Scd4x`]; the async API
//! lives in [`asynch`] under the same type name (the `asynch` module name
//! avoids clashing with the `async` keyword). Both share the same 16-bit
//! command words, CRC routine and raw-to-engineering-unit conversion,
//! implemented exactly once at the crate root.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// The only I2C address used by SCD4x parts.
pub const ADDRESS: SevenBitAddress = 0x62;

const CMD_START_PERIODIC_MEASUREMENT: u16 = 0x21B1;
const CMD_READ_MEASUREMENT: u16 = 0xEC05;
const CMD_STOP_PERIODIC_MEASUREMENT: u16 = 0x3F86;
const CMD_GET_DATA_READY_STATUS: u16 = 0xE4B8;

/// `get_data_ready_status` reports "not ready" when the low 11 bits of the
/// status word are all zero.
const DATA_READY_MASK: u16 = 0x07FF;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcField {
    Co2,
    Temperature,
    Humidity,
    Status,
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
    /// CO2 concentration in parts per million.
    pub co2_ppm: u16,
    /// Temperature in hundredths of a degree Celsius (centi-degrees).
    pub temperature_centi_celsius: i32,
    /// Relative humidity in hundredths of a percent (centi-percent).
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

/// Rounds `n / d` to the nearest integer, away from zero on ties (`d` must
/// be positive).
const fn round_div(n: i32, d: i32) -> i32 {
    if n >= 0 {
        (n + d / 2) / d
    } else {
        -((-n + d / 2) / d)
    }
}

/// Sensirion's official fixed-point conversion (equivalent to the
/// datasheet's `-45 + 175 * raw / 2^16` and `100 * raw / 2^16`, but computed
/// exactly as the reference driver does via a `>> 13` shift instead of
/// floating point).
pub fn decode_measurement(raw_co2: u16, raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let temp_milli_celsius = ((21_875i32 * i32::from(raw_temperature)) >> 13) - 45_000;
    let humidity_milli_percent = (12_500i32 * i32::from(raw_humidity)) >> 13;
    Measurement {
        co2_ppm: raw_co2,
        temperature_centi_celsius: round_div(temp_milli_celsius, 10),
        humidity_centi_percent: round_div(humidity_milli_percent, 10).clamp(0, 10_000) as u16,
    }
}

fn parse_response<E>(response: &[u8; 9]) -> Result<Measurement, Error<E>> {
    validate_crc(CrcField::Co2, &response[..2], response[2])?;
    validate_crc(CrcField::Temperature, &response[3..5], response[5])?;
    validate_crc(CrcField::Humidity, &response[6..8], response[8])?;
    Ok(decode_measurement(
        u16::from_be_bytes([response[0], response[1]]),
        u16::from_be_bytes([response[3], response[4]]),
        u16::from_be_bytes([response[6], response[7]]),
    ))
}

fn is_data_ready(status: u16) -> bool {
    status & DATA_READY_MASK != 0
}

/// Blocking SCD4x driver.
pub struct Scd4x<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
}

impl<I2C> Scd4x<I2C> {
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

impl<I2C> Scd4x<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Starts continuous periodic measurement (5 s interval). The sensor
    /// signals a new sample roughly every 5 s via
    /// [`Self::data_ready`]/[`Self::read_measurement`].
    pub fn start_periodic_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &CMD_START_PERIODIC_MEASUREMENT.to_be_bytes())
            .map_err(Error::I2c)
    }

    /// Stops periodic measurement. The caller must wait at least 500 ms
    /// before sending further commands.
    pub fn stop_periodic_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &CMD_STOP_PERIODIC_MEASUREMENT.to_be_bytes())
            .map_err(Error::I2c)
    }

    /// Returns whether a new sample is available to read.
    pub fn data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
        let mut response = [0u8; 3];
        self.i2c
            .write_read(
                self.address,
                &CMD_GET_DATA_READY_STATUS.to_be_bytes(),
                &mut response,
            )
            .map_err(Error::I2c)?;
        validate_crc(CrcField::Status, &response[..2], response[2])?;
        Ok(is_data_ready(u16::from_be_bytes([
            response[0],
            response[1],
        ])))
    }

    /// Reads and CRC-checks the most recent measurement. Only valid once
    /// [`Self::data_ready`] reports `true`.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut response = [0u8; 9];
        self.i2c
            .write_read(
                self.address,
                &CMD_READ_MEASUREMENT.to_be_bytes(),
                &mut response,
            )
            .map_err(Error::I2c)?;
        parse_response(&response)
    }
}

/// Async (embedded-hal-async 1.0) counterpart of the blocking API. See the
/// crate-root [`super::Scd4x`]; command words, CRC and conversion logic are
/// shared, not duplicated.
pub mod asynch {
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};

    use super::{
        CMD_GET_DATA_READY_STATUS, CMD_READ_MEASUREMENT, CMD_START_PERIODIC_MEASUREMENT,
        CMD_STOP_PERIODIC_MEASUREMENT, CrcField, Error, Measurement, is_data_ready, parse_response,
        validate_crc,
    };

    pub struct Scd4x<I2C> {
        i2c: I2C,
        address: SevenBitAddress,
    }

    impl<I2C> Scd4x<I2C> {
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

    impl<I2C> Scd4x<I2C>
    where
        I2C: I2c<SevenBitAddress>,
    {
        pub async fn start_periodic_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &CMD_START_PERIODIC_MEASUREMENT.to_be_bytes())
                .await
                .map_err(Error::I2c)
        }

        pub async fn stop_periodic_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &CMD_STOP_PERIODIC_MEASUREMENT.to_be_bytes())
                .await
                .map_err(Error::I2c)
        }

        pub async fn data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
            let mut response = [0u8; 3];
            self.i2c
                .write_read(
                    self.address,
                    &CMD_GET_DATA_READY_STATUS.to_be_bytes(),
                    &mut response,
                )
                .await
                .map_err(Error::I2c)?;
            validate_crc(CrcField::Status, &response[..2], response[2])?;
            Ok(is_data_ready(u16::from_be_bytes([
                response[0],
                response[1],
            ])))
        }

        pub async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut response = [0u8; 9];
            self.i2c
                .write_read(
                    self.address,
                    &CMD_READ_MEASUREMENT.to_be_bytes(),
                    &mut response,
                )
                .await
                .map_err(Error::I2c)?;
            parse_response(&response)
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
    fn conversion_matches_sensirion_reference_driver() {
        // Reference values from Sensirion's embedded-i2c-scd4x driver
        // (`scd4x_read_measurement`), which uses the exact same `>>13`
        // fixed-point formula rather than the datasheet's float equations.
        let m = decode_measurement(800, 0, 0);
        assert_eq!(m.co2_ppm, 800);
        assert_eq!(m.temperature_centi_celsius, -4_500); // -45.00 degC
        assert_eq!(m.humidity_centi_percent, 0);

        let m = decode_measurement(1200, 0x8000, 0x8000);
        assert_eq!(m.temperature_centi_celsius, 4_250); // 42.50 degC
        assert_eq!(m.humidity_centi_percent, 5_000); // 50.00 %RH
    }

    #[test]
    fn humidity_is_clamped_to_valid_range() {
        let m = decode_measurement(0, 0, u16::MAX);
        assert!(m.humidity_centi_percent <= 10_000);
    }

    #[test]
    fn data_ready_mask_ignores_reserved_bits() {
        assert!(!is_data_ready(0x0000));
        assert!(!is_data_ready(0x0800)); // only reserved bit 11 set
        assert!(is_data_ready(0x0001));
        assert!(is_data_ready(0x0801)); // reserved bit + ready bit
    }

    #[test]
    fn read_measurement_reports_crc_error_on_corrupt_co2() {
        let mut response = canned_response();
        response[2] ^= 0xFF;
        let err: Result<Measurement, Error<core::convert::Infallible>> = parse_response(&response);
        assert_eq!(
            err,
            Err(Error::Crc {
                field: CrcField::Co2,
                expected: crc8(&canned_response()[..2]),
                received: canned_response()[2] ^ 0xFF,
            })
        );
    }

    fn canned_response() -> [u8; 9] {
        let mut response = [0x03, 0x20, 0, 0x66, 0x66, 0, 0x66, 0x66, 0];
        response[2] = crc8(&response[..2]);
        response[5] = crc8(&response[3..5]);
        response[8] = crc8(&response[6..8]);
        response
    }

    /// Fake I2C bus that records the sequence of commands sent and serves
    /// canned responses, to verify that `start_periodic_measurement` is a
    /// write-only command and `read_measurement` performs a write-then-read
    /// transaction for the correct command word.
    struct FakeI2c {
        calls: heapless::Vec<u16, 4>,
        response: [u8; 9],
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
                        self.calls
                            .push(u16::from_be_bytes([bytes[0], bytes[1]]))
                            .unwrap();
                    }
                    embedded_hal::i2c::Operation::Read(buf) => {
                        buf.copy_from_slice(&self.response[..buf.len()]);
                    }
                }
            }
            Ok(())
        }
    }

    #[test]
    fn read_measurement_sends_correct_command_word() {
        let i2c = FakeI2c {
            calls: heapless::Vec::new(),
            response: canned_response(),
        };
        let mut sensor = Scd4x::new(i2c, ADDRESS);
        let measurement = sensor.read_measurement().unwrap();

        assert_eq!(measurement.co2_ppm, 0x0320);
        assert_eq!(sensor.release().calls.as_slice(), [CMD_READ_MEASUREMENT]);
    }

    #[test]
    fn start_and_stop_send_correct_command_words() {
        let i2c = FakeI2c {
            calls: heapless::Vec::new(),
            response: canned_response(),
        };
        let mut sensor = Scd4x::new(i2c, ADDRESS);
        sensor.start_periodic_measurement().unwrap();
        sensor.stop_periodic_measurement().unwrap();

        assert_eq!(
            sensor.release().calls.as_slice(),
            [
                CMD_START_PERIODIC_MEASUREMENT,
                CMD_STOP_PERIODIC_MEASUREMENT
            ]
        );
    }
}
