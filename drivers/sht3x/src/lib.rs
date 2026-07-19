//! Transport-independent SHT3x protocol over blocking embedded-hal I2C.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

pub const PRIMARY_ADDRESS: SevenBitAddress = 0x44;
pub const SECONDARY_ADDRESS: SevenBitAddress = 0x45;

const CMD_SOFT_RESET: [u8; 2] = [0x30, 0xA2];
const CMD_READ_STATUS: [u8; 2] = [0xF3, 0x2D];
const CMD_HIGH_REPEATABILITY_SINGLE_SHOT: [u8; 2] = [0x24, 0x00];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcField {
    Status,
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
pub struct Status {
    pub raw: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    pub temperature_centi_celsius: i16,
    pub humidity_centi_percent: u16,
}

pub struct Sht3x<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
}

impl<I2C> Sht3x<I2C> {
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

impl<I2C> Sht3x<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Send soft reset. The caller must wait at least 2 ms before status read.
    pub fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &CMD_SOFT_RESET)
            .map_err(Error::I2c)
    }

    /// Read and CRC-check the two-byte status word.
    pub fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
        let mut response = [0u8; 3];
        self.i2c
            .write_read(self.address, &CMD_READ_STATUS, &mut response)
            .map_err(Error::I2c)?;
        validate_crc(CrcField::Status, &response[..2], response[2])?;
        Ok(Status {
            raw: u16::from_be_bytes([response[0], response[1]]),
        })
    }

    /// Trigger high-repeatability, clock-stretching-disabled single shot.
    ///
    /// The caller must wait at least 15 ms; 20 ms is recommended.
    pub fn start_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &CMD_HIGH_REPEATABILITY_SINGLE_SHOT)
            .map_err(Error::I2c)
    }

    /// Read and CRC-check a previously triggered single-shot measurement.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut response = [0u8; 6];
        self.i2c
            .read(self.address, &mut response)
            .map_err(Error::I2c)?;
        validate_crc(CrcField::Temperature, &response[..2], response[2])?;
        validate_crc(CrcField::Humidity, &response[3..5], response[5])?;

        Ok(decode_measurement(
            u16::from_be_bytes([response[0], response[1]]),
            u16::from_be_bytes([response[3], response[4]]),
        ))
    }
}

pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0xFF;
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

pub fn decode_measurement(raw_temperature: u16, raw_humidity: u16) -> Measurement {
    let temperature = -4_500i32 + (17_500u32 * u32::from(raw_temperature) / 65_535) as i32;
    let humidity = (10_000u32 * u32::from(raw_humidity) / 65_535).min(10_000);
    Measurement {
        temperature_centi_celsius: temperature as i16,
        humidity_centi_percent: humidity as u16,
    }
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

#[cfg(test)]
mod tests {
    use super::{crc8, decode_measurement};

    #[test]
    fn crc_matches_sensirion_example() {
        assert_eq!(crc8(&[0xBE, 0xEF]), 0x92);
    }

    #[test]
    fn conversion_endpoints_match_datasheet() {
        let low = decode_measurement(0, 0);
        assert_eq!(low.temperature_centi_celsius, -4_500);
        assert_eq!(low.humidity_centi_percent, 0);

        let high = decode_measurement(u16::MAX, u16::MAX);
        assert_eq!(high.temperature_centi_celsius, 13_000);
        assert_eq!(high.humidity_centi_percent, 10_000);
    }
}
