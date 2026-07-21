//! Transport-independent SHT3x protocol, with both a blocking (embedded-hal
//! 1.0) and an async (embedded-hal-async 1.0) API.
//!
//! The blocking API lives at the crate root as [`Sht3x`], unchanged from the
//! original blocking-only driver. The async API lives in [`asynch`] under the
//! same type name, following the common embedded-rust convention of using an
//! `asynch` module name (`async` is a reserved keyword). Both share the same
//! command bytes, CRC routine and raw-to-engineering-unit conversion so the
//! protocol logic is implemented exactly once.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

pub const PRIMARY_ADDRESS: SevenBitAddress = 0x44;
pub const SECONDARY_ADDRESS: SevenBitAddress = 0x45;

const CMD_SOFT_RESET: [u8; 2] = [0x30, 0xA2];
const CMD_READ_STATUS: [u8; 2] = [0xF3, 0x2D];
const CMD_HIGH_REPEATABILITY_SINGLE_SHOT: [u8; 2] = [0x24, 0x00];

/// Conversion time for a high-repeatability single shot measurement
/// (datasheet: 15 ms typical, 20 ms recommended worst case).
const MEASUREMENT_DELAY_MS: u32 = 20;

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

    /// Trigger a single-shot measurement, wait the required conversion time
    /// and read back the result. Equivalent to calling
    /// [`Self::start_measurement`], waiting, then [`Self::read_measurement`].
    pub fn measure<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Error<I2C::Error>> {
        self.start_measurement()?;
        delay.delay_ms(MEASUREMENT_DELAY_MS);
        self.read_measurement()
    }
}

/// Async (embedded-hal-async 1.0) counterpart of the blocking API.
///
/// The module name `asynch` avoids clashing with the `async` keyword; this
/// mirrors the crate-root [`Sht3x`] type name and method set, but every
/// bus operation is an `async fn` and delays use
/// [`embedded_hal_async::delay::DelayNs`].
pub mod asynch {
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};

    use super::{
        CMD_HIGH_REPEATABILITY_SINGLE_SHOT, CMD_READ_STATUS, CMD_SOFT_RESET, CrcField, Error,
        MEASUREMENT_DELAY_MS, Measurement, Status, decode_measurement, validate_crc,
    };

    /// Async SHT3x driver. See the crate-root [`super::Sht3x`] for the
    /// blocking equivalent; both share command bytes, CRC and conversion
    /// logic defined once at the crate root.
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
        pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &CMD_SOFT_RESET)
                .await
                .map_err(Error::I2c)
        }

        /// Read and CRC-check the two-byte status word.
        pub async fn read_status(&mut self) -> Result<Status, Error<I2C::Error>> {
            let mut response = [0u8; 3];
            self.i2c
                .write_read(self.address, &CMD_READ_STATUS, &mut response)
                .await
                .map_err(Error::I2c)?;
            validate_crc(CrcField::Status, &response[..2], response[2])?;
            Ok(Status {
                raw: u16::from_be_bytes([response[0], response[1]]),
            })
        }

        /// Trigger high-repeatability, clock-stretching-disabled single shot.
        ///
        /// The caller must wait at least 15 ms; 20 ms is recommended.
        pub async fn start_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &CMD_HIGH_REPEATABILITY_SINGLE_SHOT)
                .await
                .map_err(Error::I2c)
        }

        /// Read and CRC-check a previously triggered single-shot measurement.
        pub async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut response = [0u8; 6];
            self.i2c
                .read(self.address, &mut response)
                .await
                .map_err(Error::I2c)?;
            validate_crc(CrcField::Temperature, &response[..2], response[2])?;
            validate_crc(CrcField::Humidity, &response[3..5], response[5])?;

            Ok(decode_measurement(
                u16::from_be_bytes([response[0], response[1]]),
                u16::from_be_bytes([response[3], response[4]]),
            ))
        }

        /// Trigger a single-shot measurement, wait the required conversion
        /// time and read back the result.
        pub async fn measure<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<I2C::Error>> {
            self.start_measurement().await?;
            delay.delay_ms(MEASUREMENT_DELAY_MS).await;
            self.read_measurement().await
        }
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
    use super::{
        CMD_HIGH_REPEATABILITY_SINGLE_SHOT, PRIMARY_ADDRESS, Sht3x, crc8, decode_measurement,
    };
    use embedded_hal::i2c::{ErrorType, I2c, Operation};

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

    /// Minimal fake I2C bus that records the sequence of operations and
    /// serves a canned single-shot measurement response, so that `measure`
    /// can be verified to issue "start" before "read" rather than assuming
    /// it from the source alone.
    struct FakeI2c {
        calls: heapless::Vec<&'static str, 4>,
        response: [u8; 6],
    }

    impl ErrorType for FakeI2c {
        type Error = core::convert::Infallible;
    }

    impl I2c for FakeI2c {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        if *bytes == CMD_HIGH_REPEATABILITY_SINGLE_SHOT {
                            self.calls.push("start").unwrap();
                        }
                    }
                    Operation::Read(buf) => {
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

    /// Canned response for raw temperature 0x6666, raw humidity 0x6666
    /// (arbitrary but non-trivial), with correct CRC bytes.
    fn canned_response() -> [u8; 6] {
        let mut response = [0x66, 0x66, 0, 0x66, 0x66, 0];
        response[2] = crc8(&response[..2]);
        response[5] = crc8(&response[3..5]);
        response
    }

    #[test]
    fn blocking_measure_issues_start_then_read_and_decodes_result() {
        let i2c = FakeI2c {
            calls: heapless::Vec::new(),
            response: canned_response(),
        };
        let expected = decode_measurement(0x6666, 0x6666);

        let mut sensor = Sht3x::new(i2c, PRIMARY_ADDRESS);
        let measurement = sensor.measure(&mut FakeDelay).unwrap();

        assert_eq!(measurement, expected);
        assert_eq!(sensor.release().calls.as_slice(), ["start", "read"]);
    }

    #[test]
    fn read_measurement_reports_crc_error_on_corrupt_temperature() {
        let mut response = canned_response();
        response[2] ^= 0xFF; // corrupt the temperature CRC byte
        let i2c = FakeI2c {
            calls: heapless::Vec::new(),
            response,
        };

        let mut sensor = Sht3x::new(i2c, PRIMARY_ADDRESS);
        let err = sensor.read_measurement().unwrap_err();
        assert_eq!(
            err,
            super::Error::Crc {
                field: super::CrcField::Temperature,
                expected: crc8(&canned_response()[..2]),
                received: canned_response()[2] ^ 0xFF,
            }
        );
    }

    mod asynch {
        use super::super::asynch::Sht3x;
        use super::{FakeDelay, FakeI2c, canned_response};
        use crate::{CMD_HIGH_REPEATABILITY_SINGLE_SHOT, PRIMARY_ADDRESS, decode_measurement};
        use embedded_hal_async::i2c::{I2c, Operation};

        // `embedded_hal_async::i2c::ErrorType` is a re-export of the same
        // `embedded_hal::i2c::ErrorType` trait already implemented for
        // `FakeI2c` in the parent module, so it does not need a second impl
        // here; only the async `I2c` trait itself is distinct.
        impl I2c for FakeI2c {
            async fn transaction(
                &mut self,
                _address: u8,
                operations: &mut [Operation<'_>],
            ) -> Result<(), Self::Error> {
                for op in operations {
                    match op {
                        Operation::Write(bytes) => {
                            if *bytes == CMD_HIGH_REPEATABILITY_SINGLE_SHOT {
                                self.calls.push("start").unwrap();
                            }
                        }
                        Operation::Read(buf) => {
                            self.calls.push("read").unwrap();
                            buf.copy_from_slice(&self.response);
                        }
                    }
                }
                Ok(())
            }
        }

        impl embedded_hal_async::delay::DelayNs for FakeDelay {
            async fn delay_ns(&mut self, _ns: u32) {}
        }

        /// Polls a future to completion assuming it never actually yields
        /// (true for the fake I2C/delay above), so a no-op waker suffices.
        fn block_on<F: core::future::Future>(fut: F) -> F::Output {
            let waker = core::task::Waker::noop();
            let mut cx = core::task::Context::from_waker(waker);
            let mut fut = core::pin::pin!(fut);
            loop {
                if let core::task::Poll::Ready(value) = fut.as_mut().poll(&mut cx) {
                    return value;
                }
            }
        }

        #[test]
        fn async_measure_issues_start_then_read_and_decodes_result() {
            let i2c = FakeI2c {
                calls: heapless::Vec::new(),
                response: canned_response(),
            };
            let expected = decode_measurement(0x6666, 0x6666);

            let mut sensor = Sht3x::new(i2c, PRIMARY_ADDRESS);
            let measurement = block_on(sensor.measure(&mut FakeDelay)).unwrap();

            assert_eq!(measurement, expected);
            assert_eq!(sensor.release().calls.as_slice(), ["start", "read"]);
        }
    }
}
