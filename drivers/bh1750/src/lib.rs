//! Transport-independent ROHM BH1750 ambient light sensor protocol, with
//! both a blocking (embedded-hal 1.0) and an async (embedded-hal-async 1.0)
//! API.
//!
//! The blocking API lives at the crate root as [`Bh1750`]; the async API
//! lives in [`asynch`] under the same type name (the `asynch` module name
//! avoids clashing with the `async` keyword). Both share the same opcode
//! bytes and raw-to-lux conversion, implemented exactly once at the crate
//! root.
//!
//! BH1750 has no register addressing: every command is a single opcode
//! byte, and measurement results are a plain 2-byte big-endian count.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// I2C address when the `ADDR` pin is tied low.
pub const ADDRESS_LOW: SevenBitAddress = 0x23;
/// I2C address when the `ADDR` pin is tied high.
pub const ADDRESS_HIGH: SevenBitAddress = 0x5C;

const OPCODE_POWER_DOWN: u8 = 0x00;
const OPCODE_POWER_ON: u8 = 0x01;
const OPCODE_RESET: u8 = 0x07;

/// Measurement mode, selecting resolution and whether the sensor
/// auto-powers-down after a single conversion.
///
/// Continuous modes keep converting until [`Bh1750::power_down`] is called.
/// One-time modes convert once and then automatically enter power-down, so
/// the sensor must be powered on again (implicitly done by re-sending the
/// opcode; see datasheet section 5) before triggering another one-time
/// measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    ContinuousHighRes,
    ContinuousHighRes2,
    ContinuousLowRes,
    OneTimeHighRes,
    OneTimeHighRes2,
    OneTimeLowRes,
}

impl Mode {
    const fn opcode(self) -> u8 {
        match self {
            Mode::ContinuousHighRes => 0x10,
            Mode::ContinuousHighRes2 => 0x11,
            Mode::ContinuousLowRes => 0x13,
            Mode::OneTimeHighRes => 0x20,
            Mode::OneTimeHighRes2 => 0x21,
            Mode::OneTimeLowRes => 0x23,
        }
    }

    /// Whether this mode uses the doubled (0.5 lx step) resolution, which
    /// halves the raw-to-lux scale factor.
    const fn is_high_res2(self) -> bool {
        matches!(self, Mode::ContinuousHighRes2 | Mode::OneTimeHighRes2)
    }

    /// Datasheet worst-case conversion time in milliseconds (default
    /// measurement-time register value of 69).
    pub const fn max_delay_ms(self) -> u32 {
        match self {
            Mode::ContinuousHighRes
            | Mode::ContinuousHighRes2
            | Mode::OneTimeHighRes
            | Mode::OneTimeHighRes2 => 180,
            Mode::ContinuousLowRes | Mode::OneTimeLowRes => 24,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
}

/// Converts a raw 16-bit count to illuminance in centi-lux (hundredths of a
/// lux), using the datasheet formula `lux = count / 1.2` (halved again for
/// the doubled-resolution modes: `lux = count / 2.4`), at the default
/// measurement-time register value of 69.
pub fn decode_lux_centi(raw: u16, mode: Mode) -> u32 {
    // `count * 1000 / 12` == `count / 1.2 * 100` (centi-lux) without floats.
    let scaled = u32::from(raw) * 1_000 / 12;
    if mode.is_high_res2() {
        scaled / 2
    } else {
        scaled
    }
}

/// Blocking BH1750 driver.
pub struct Bh1750<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
}

impl<I2C> Bh1750<I2C> {
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

impl<I2C> Bh1750<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Enters power-down mode (also aborts any continuous measurement).
    pub fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[OPCODE_POWER_DOWN])
            .map_err(Error::I2c)
    }

    /// Wakes the sensor. Required before [`Self::reset`] and before the
    /// first measurement after a power-down.
    pub fn power_on(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[OPCODE_POWER_ON])
            .map_err(Error::I2c)
    }

    /// Clears the illuminance data register. Must be called only while
    /// powered on (see [`Self::power_on`]).
    pub fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[OPCODE_RESET])
            .map_err(Error::I2c)
    }

    /// Sends the mode opcode, starting a conversion.
    pub fn start(&mut self, mode: Mode) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[mode.opcode()])
            .map_err(Error::I2c)
    }

    /// Reads the raw 16-bit count of a completed conversion.
    pub fn read_raw(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut response = [0u8; 2];
        self.i2c
            .read(self.address, &mut response)
            .map_err(Error::I2c)?;
        Ok(u16::from_be_bytes(response))
    }

    /// Reads back a completed conversion and converts it to centi-lux for
    /// the given mode.
    pub fn read_lux_centi(&mut self, mode: Mode) -> Result<u32, Error<I2C::Error>> {
        Ok(decode_lux_centi(self.read_raw()?, mode))
    }

    /// Starts a conversion, waits the worst-case conversion time and reads
    /// back the result in centi-lux.
    pub fn measure<D: embedded_hal::delay::DelayNs>(
        &mut self,
        mode: Mode,
        delay: &mut D,
    ) -> Result<u32, Error<I2C::Error>> {
        self.start(mode)?;
        delay.delay_ms(mode.max_delay_ms());
        self.read_lux_centi(mode)
    }
}

/// Async (embedded-hal-async 1.0) counterpart of the blocking API. See the
/// crate-root [`super::Bh1750`]; opcodes and conversion logic are shared,
/// not duplicated.
pub mod asynch {
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};

    use super::{Error, Mode, OPCODE_POWER_DOWN, OPCODE_POWER_ON, OPCODE_RESET, decode_lux_centi};

    pub struct Bh1750<I2C> {
        i2c: I2C,
        address: SevenBitAddress,
    }

    impl<I2C> Bh1750<I2C> {
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

    impl<I2C> Bh1750<I2C>
    where
        I2C: I2c<SevenBitAddress>,
    {
        pub async fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[OPCODE_POWER_DOWN])
                .await
                .map_err(Error::I2c)
        }

        pub async fn power_on(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[OPCODE_POWER_ON])
                .await
                .map_err(Error::I2c)
        }

        pub async fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[OPCODE_RESET])
                .await
                .map_err(Error::I2c)
        }

        pub async fn start(&mut self, mode: Mode) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(self.address, &[mode.opcode()])
                .await
                .map_err(Error::I2c)
        }

        pub async fn read_raw(&mut self) -> Result<u16, Error<I2C::Error>> {
            let mut response = [0u8; 2];
            self.i2c
                .read(self.address, &mut response)
                .await
                .map_err(Error::I2c)?;
            Ok(u16::from_be_bytes(response))
        }

        pub async fn read_lux_centi(&mut self, mode: Mode) -> Result<u32, Error<I2C::Error>> {
            Ok(decode_lux_centi(self.read_raw().await?, mode))
        }

        pub async fn measure<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            mode: Mode,
            delay: &mut D,
        ) -> Result<u32, Error<I2C::Error>> {
            self.start(mode).await?;
            delay.delay_ms(mode.max_delay_ms()).await;
            self.read_lux_centi(mode).await
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn opcodes_match_datasheet() {
        assert_eq!(Mode::ContinuousHighRes.opcode(), 0x10);
        assert_eq!(Mode::ContinuousHighRes2.opcode(), 0x11);
        assert_eq!(Mode::ContinuousLowRes.opcode(), 0x13);
        assert_eq!(Mode::OneTimeHighRes.opcode(), 0x20);
        assert_eq!(Mode::OneTimeHighRes2.opcode(), 0x21);
        assert_eq!(Mode::OneTimeLowRes.opcode(), 0x23);
    }

    #[test]
    fn high_res_conversion_matches_datasheet_formula() {
        // Datasheet worked example: count 0x83 (~131) at default MTreg=69
        // corresponds to roughly 109 lx (131 / 1.2 = 109.17).
        assert_eq!(decode_lux_centi(131, Mode::ContinuousHighRes), 10_916);
        // Zero count is zero lux.
        assert_eq!(decode_lux_centi(0, Mode::ContinuousHighRes), 0);
        // 1.2 raw counts per lux, so 12 counts == 10.00 lx exactly.
        assert_eq!(decode_lux_centi(12, Mode::ContinuousHighRes), 1_000);
    }

    #[test]
    fn high_res2_mode_halves_the_scale() {
        // H-Res2 has double the resolution (0.5 lx/count), so the same raw
        // count yields half the lux of H-Res mode.
        let high_res = decode_lux_centi(1_200, Mode::ContinuousHighRes);
        let high_res2 = decode_lux_centi(1_200, Mode::ContinuousHighRes2);
        assert_eq!(high_res, 100_000); // 1000.00 lx
        assert_eq!(high_res2, 50_000); // 500.00 lx
    }

    #[test]
    fn max_delay_reflects_resolution() {
        assert_eq!(Mode::ContinuousHighRes.max_delay_ms(), 180);
        assert_eq!(Mode::ContinuousLowRes.max_delay_ms(), 24);
    }

    /// Fake I2C bus that records the opcodes written and serves a canned
    /// 2-byte response, to verify `measure` issues the mode opcode before
    /// reading back the result.
    struct FakeI2c {
        writes: heapless::Vec<u8, 4>,
        response: [u8; 2],
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
                        self.writes.push(bytes[0]).unwrap();
                    }
                    embedded_hal::i2c::Operation::Read(buf) => {
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

    #[test]
    fn measure_sends_opcode_before_reading_result() {
        let i2c = FakeI2c {
            writes: heapless::Vec::new(),
            response: 1_200u16.to_be_bytes(),
        };
        let mut sensor = Bh1750::new(i2c, ADDRESS_LOW);
        let lux = sensor
            .measure(Mode::ContinuousHighRes, &mut FakeDelay)
            .unwrap();

        assert_eq!(lux, decode_lux_centi(1_200, Mode::ContinuousHighRes));
        assert_eq!(
            sensor.release().writes.as_slice(),
            [Mode::ContinuousHighRes.opcode()]
        );
    }
}
