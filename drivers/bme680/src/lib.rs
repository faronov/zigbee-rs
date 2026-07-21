//! Transport-independent Bosch BME680 protocol, with both a blocking
//! (embedded-hal 1.0) and an async (embedded-hal-async 1.0) API.
//!
//! The blocking API lives at the crate root as [`Bme680`]; the async API
//! lives in [`asynch`] under the same type name (the `asynch` module name
//! avoids clashing with the `async` keyword). Both share the same register
//! map, calibration parsing and Bosch integer compensation formulas,
//! implemented exactly once at the crate root.
//!
//! # Unsupported: gas resistance
//!
//! This driver only implements forced-mode temperature, humidity and
//! pressure measurement. BME680 gas resistance measurement requires
//! configuring a heater profile (target temperature/duration, heater
//! resistance lookup using `res_heat_range`/`res_heat_val`/`range_sw_err`
//! calibration and a device-specific resistance-to-Ohms conversion) which is
//! not implemented here. [`Measurement`] therefore intentionally has no gas
//! resistance field rather than reporting a fabricated value; gas heater
//! registers are never written by this driver.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// Primary I2C address (`SDO` pulled low).
pub const PRIMARY_ADDRESS: SevenBitAddress = 0x76;
/// Secondary I2C address (`SDO` pulled high).
pub const SECONDARY_ADDRESS: SevenBitAddress = 0x77;

/// Expected `chip_id` register value.
pub const CHIP_ID: u8 = 0x61;

const REG_CHIP_ID: u8 = 0xD0;
const REG_SOFT_RESET: u8 = 0xE0;
const REG_CTRL_HUM: u8 = 0x72;
const REG_CTRL_MEAS: u8 = 0x74;
const REG_FIELD0: u8 = 0x1D;
const REG_COEFF1: u8 = 0x8A;
const REG_COEFF2: u8 = 0xE1;

const CMD_SOFT_RESET: u8 = 0xB6;
/// Datasheet-specified startup time after a soft reset.
const STARTUP_DELAY_MS: u32 = 2;

const LEN_COEFF1: usize = 23;
const LEN_COEFF2: usize = 14;
const LEN_COEFF_ALL: usize = LEN_COEFF1 + LEN_COEFF2;
/// Only the status/index/pressure/temperature/humidity bytes are read; the
/// gas ADC bytes that follow in the field register block are unused since
/// gas resistance is not supported.
const LEN_FIELD: usize = 10;

/// Oversampling setting for one measured quantity (identical register
/// encoding to BME280/BMP280).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Oversampling {
    Skip,
    X1,
    X2,
    X4,
    X8,
    X16,
}

impl Oversampling {
    const fn register_bits(self) -> u8 {
        match self {
            Oversampling::Skip => 0b000,
            Oversampling::X1 => 0b001,
            Oversampling::X2 => 0b010,
            Oversampling::X4 => 0b011,
            Oversampling::X8 => 0b100,
            Oversampling::X16 => 0b101,
        }
    }

    /// Number of internal measurement cycles used in the conversion-time
    /// formula (`os_to_meas_cycles` in Bosch's reference driver).
    const fn measurement_cycles(self) -> u32 {
        match self {
            Oversampling::Skip => 0,
            Oversampling::X1 => 1,
            Oversampling::X2 => 2,
            Oversampling::X4 => 4,
            Oversampling::X8 => 8,
            Oversampling::X16 => 16,
        }
    }
}

/// Power/measurement mode (`mode` field of `ctrl_meas`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    Sleep,
    Forced,
}

impl PowerMode {
    const fn register_bits(self) -> u8 {
        match self {
            PowerMode::Sleep => 0b00,
            PowerMode::Forced => 0b01,
        }
    }
}

/// Oversampling configuration for temperature, pressure and humidity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SamplingConfig {
    pub temperature: Oversampling,
    pub pressure: Oversampling,
    pub humidity: Oversampling,
}

impl Default for SamplingConfig {
    fn default() -> Self {
        Self {
            temperature: Oversampling::X2,
            pressure: Oversampling::X4,
            humidity: Oversampling::X1,
        }
    }
}

impl SamplingConfig {
    fn ctrl_meas(self, mode: PowerMode) -> u8 {
        (self.temperature.register_bits() << 5)
            | (self.pressure.register_bits() << 2)
            | mode.register_bits()
    }

    fn ctrl_hum(self) -> u8 {
        self.humidity.register_bits()
    }

    /// Worst-case forced-mode conversion time in milliseconds, following
    /// Bosch's `bme68x_get_meas_dur` formula for non-parallel (forced) mode
    /// with the gas heater disabled (heater switching/gas duration terms
    /// are still budgeted for, matching the reference driver's default
    /// non-parallel timing even though this driver never enables the
    /// heater).
    fn measurement_delay_ms(self) -> u32 {
        let cycles = self.temperature.measurement_cycles()
            + self.pressure.measurement_cycles()
            + self.humidity.measurement_cycles();
        let delay_us = cycles * 1_963 + 477 * 4 + 477 * 5 + 1_000;
        delay_us.div_ceil(1_000)
    }
}

/// Parsed factory calibration coefficients (Bosch `par_*` trim values) used
/// for temperature, pressure and humidity compensation. Gas-heater-related
/// coefficients are intentionally omitted; see the crate-level
/// documentation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Calibration {
    par_t1: u16,
    par_t2: i16,
    par_t3: i8,
    par_p1: u16,
    par_p2: i16,
    par_p3: i8,
    par_p4: i16,
    par_p5: i16,
    par_p6: i8,
    par_p7: i8,
    par_p8: i16,
    par_p9: i16,
    par_p10: u8,
    par_h1: u16,
    par_h2: u16,
    par_h3: i8,
    par_h4: i8,
    par_h5: i8,
    par_h6: u8,
    par_h7: i8,
}

fn concat(msb: u8, lsb: u8) -> u16 {
    (u16::from(msb) << 8) | u16::from(lsb)
}

fn parse_calibration(data: &[u8; LEN_COEFF_ALL]) -> Calibration {
    Calibration {
        par_t2: concat(data[1], data[0]) as i16,
        par_t3: data[2] as i8,
        par_p1: concat(data[5], data[4]),
        par_p2: concat(data[7], data[6]) as i16,
        par_p3: data[8] as i8,
        par_p4: concat(data[11], data[10]) as i16,
        par_p5: concat(data[13], data[12]) as i16,
        par_p7: data[14] as i8,
        par_p6: data[15] as i8,
        par_p8: concat(data[19], data[18]) as i16,
        par_p9: concat(data[21], data[20]) as i16,
        par_p10: data[22],
        par_h2: (u16::from(data[23]) << 4) | (u16::from(data[24]) >> 4),
        par_h1: (u16::from(data[25]) << 4) | (u16::from(data[24]) & 0x0F),
        par_h3: data[26] as i8,
        par_h4: data[27] as i8,
        par_h5: data[28] as i8,
        par_h6: data[29],
        par_h7: data[30] as i8,
        par_t1: concat(data[32], data[31]),
    }
}

/// Uncompensated (raw ADC) sensor readings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RawData {
    pressure: u32,
    temperature: u32,
    humidity: u32,
}

fn parse_raw_data(data: &[u8; LEN_FIELD]) -> RawData {
    let pressure =
        (u32::from(data[2]) << 12) | (u32::from(data[3]) << 4) | (u32::from(data[4]) >> 4);
    let temperature =
        (u32::from(data[5]) << 12) | (u32::from(data[6]) << 4) | (u32::from(data[7]) >> 4);
    let humidity = (u32::from(data[8]) << 8) | u32::from(data[9]);
    RawData {
        pressure,
        temperature,
        humidity,
    }
}

/// A fully compensated measurement, in fixed-point engineering units. There
/// is deliberately no gas resistance field; see the crate-level
/// documentation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in hundredths of a degree Celsius (centi-degrees).
    pub temperature_centi_celsius: i32,
    /// Pressure in Pascal.
    pub pressure_pa: u32,
    /// Relative humidity in hundredths of a percent (centi-percent).
    pub humidity_centi_percent: u16,
}

/// Bosch's official integer temperature compensation formula (BME680).
/// Returns `(t_fine, temperature_centi_celsius)`.
///
/// All arithmetic uses wrapping operations to mirror the reference driver's
/// reliance on 32-bit two's-complement wraparound; this keeps the driver
/// panic-free for the full range of raw ADC values a sensor can report,
/// rather than only for "typical" ones.
fn compensate_temperature(adc_t: u32, calib: &Calibration) -> (i32, i32) {
    let adc_t = adc_t as i32;
    let var1 = (adc_t >> 3).wrapping_sub(i32::from(calib.par_t1) << 1);
    let var2 = (var1.wrapping_mul(i32::from(calib.par_t2))) >> 11;
    let var3 = ((var1 >> 1).wrapping_mul(var1 >> 1)) >> 12;
    let var3 = (var3.wrapping_mul(i32::from(calib.par_t3) << 4)) >> 14;
    let t_fine = var2.wrapping_add(var3);
    let temperature = (t_fine.wrapping_mul(5).wrapping_add(128)) >> 8;
    (t_fine, temperature)
}

/// Bosch's official integer pressure compensation formula (BME680). Returns
/// pressure directly in Pascal. See [`compensate_temperature`] for why
/// wrapping arithmetic is used throughout.
fn compensate_pressure(adc_p: u32, t_fine: i32, calib: &Calibration) -> u32 {
    let adc_p = adc_p as i32;
    let mut var1 = (t_fine >> 1).wrapping_sub(64_000);
    let mut var2 =
        ((((var1 >> 2).wrapping_mul(var1 >> 2)) >> 11).wrapping_mul(i32::from(calib.par_p6))) >> 2;
    var2 = var2.wrapping_add((var1.wrapping_mul(i32::from(calib.par_p5))) << 1);
    var2 = (var2 >> 2).wrapping_add(i32::from(calib.par_p4) << 16);
    var1 = ((((var1 >> 2).wrapping_mul(var1 >> 2)) >> 13)
        .wrapping_mul(i32::from(calib.par_p3) << 5)
        >> 3)
        .wrapping_add((i32::from(calib.par_p2).wrapping_mul(var1)) >> 1);
    var1 >>= 18;
    var1 = ((32_768 + var1).wrapping_mul(i32::from(calib.par_p1))) >> 15;

    let mut pressure_comp = 1_048_576i32.wrapping_sub(adc_p);
    pressure_comp = (pressure_comp.wrapping_sub(var2 >> 12)).wrapping_mul(3_125);
    const OVERFLOW_CHECK: i32 = 0x4000_0000;
    pressure_comp = if pressure_comp >= OVERFLOW_CHECK {
        (pressure_comp / var1) << 1
    } else {
        (pressure_comp << 1) / var1
    };

    var1 = (i32::from(calib.par_p9)
        .wrapping_mul(((pressure_comp >> 3).wrapping_mul(pressure_comp >> 3)) >> 13))
        >> 12;
    var2 = ((pressure_comp >> 2).wrapping_mul(i32::from(calib.par_p8))) >> 13;
    let var3 = (((pressure_comp >> 8).wrapping_mul(pressure_comp >> 8))
        .wrapping_mul(pressure_comp >> 8))
    .wrapping_mul(i32::from(calib.par_p10))
        >> 17;
    pressure_comp = pressure_comp.wrapping_add(
        (var1
            .wrapping_add(var2)
            .wrapping_add(var3)
            .wrapping_add(i32::from(calib.par_p7) << 7))
            >> 4,
    );

    pressure_comp as u32
}

/// Bosch's official integer humidity compensation formula (BME680). Returns
/// humidity scaled by 1000 (milli-percent), clamped to `0..=100000`. See
/// [`compensate_temperature`] for why wrapping arithmetic is used
/// throughout.
fn compensate_humidity(adc_h: u32, t_fine: i32, calib: &Calibration) -> i32 {
    let temp_scaled = (t_fine.wrapping_mul(5).wrapping_add(128)) >> 8;
    let var1 = (adc_h as i32)
        .wrapping_sub(i32::from(calib.par_h1) * 16)
        .wrapping_sub((temp_scaled.wrapping_mul(i32::from(calib.par_h3)) / 100) >> 1);
    let var2 = (i32::from(calib.par_h2).wrapping_mul(
        (temp_scaled.wrapping_mul(i32::from(calib.par_h4)) / 100)
            .wrapping_add(
                ((temp_scaled
                    .wrapping_mul(temp_scaled.wrapping_mul(i32::from(calib.par_h5)) / 100))
                    >> 6)
                    / 100,
            )
            .wrapping_add(1 << 14),
    )) >> 10;
    let var3 = var1.wrapping_mul(var2);
    let mut var4 = i32::from(calib.par_h6) << 7;
    var4 = (var4.wrapping_add(temp_scaled.wrapping_mul(i32::from(calib.par_h7)) / 100)) >> 4;
    let var5 = ((var3 >> 14).wrapping_mul(var3 >> 14)) >> 10;
    let var6 = (var4.wrapping_mul(var5)) >> 1;
    let humidity = ((var3.wrapping_add(var6) >> 10).wrapping_mul(1_000)) >> 12;
    humidity.clamp(0, 100_000)
}

fn make_measurement(raw: RawData, calib: &Calibration) -> Measurement {
    let (t_fine, temperature_centi_celsius) = compensate_temperature(raw.temperature, calib);
    let pressure_pa = compensate_pressure(raw.pressure, t_fine, calib);
    let humidity_milli_percent = compensate_humidity(raw.humidity, t_fine, calib);
    Measurement {
        temperature_centi_celsius,
        pressure_pa,
        humidity_centi_percent: (humidity_milli_percent / 10) as u16,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    /// Chip-id register did not match [`CHIP_ID`].
    UnknownChip(u8),
}

/// Blocking BME680 driver. Only forced-mode temperature/humidity/pressure
/// measurement is implemented; see the crate-level documentation for why
/// gas resistance is unsupported.
pub struct Bme680<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
    calib: Calibration,
    sampling: SamplingConfig,
}

impl<I2C> Bme680<I2C> {
    /// Creates a driver instance. Call [`Self::init`] before taking
    /// measurements.
    pub const fn new(i2c: I2C, address: SevenBitAddress) -> Self {
        Self {
            i2c,
            address,
            calib: Calibration {
                par_t1: 0,
                par_t2: 0,
                par_t3: 0,
                par_p1: 0,
                par_p2: 0,
                par_p3: 0,
                par_p4: 0,
                par_p5: 0,
                par_p6: 0,
                par_p7: 0,
                par_p8: 0,
                par_p9: 0,
                par_p10: 0,
                par_h1: 0,
                par_h2: 0,
                par_h3: 0,
                par_h4: 0,
                par_h5: 0,
                par_h6: 0,
                par_h7: 0,
            },
            sampling: SamplingConfig {
                temperature: Oversampling::Skip,
                pressure: Oversampling::Skip,
                humidity: Oversampling::Skip,
            },
        }
    }

    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Bme680<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Reads and validates the chip id, soft-resets the sensor and reads
    /// calibration data. The caller must supply a delay implementation for
    /// the datasheet-mandated post-reset startup wait.
    pub fn init<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        let mut id = [0u8; 1];
        self.i2c
            .write_read(self.address, &[REG_CHIP_ID], &mut id)
            .map_err(Error::I2c)?;
        if id[0] != CHIP_ID {
            return Err(Error::UnknownChip(id[0]));
        }

        self.i2c
            .write(self.address, &[REG_SOFT_RESET, CMD_SOFT_RESET])
            .map_err(Error::I2c)?;
        delay.delay_ms(STARTUP_DELAY_MS);

        let mut coeff1 = [0u8; LEN_COEFF1];
        self.i2c
            .write_read(self.address, &[REG_COEFF1], &mut coeff1)
            .map_err(Error::I2c)?;
        let mut coeff = [0u8; LEN_COEFF_ALL];
        coeff[..LEN_COEFF1].copy_from_slice(&coeff1);
        self.i2c
            .write_read(self.address, &[REG_COEFF2], &mut coeff[LEN_COEFF1..])
            .map_err(Error::I2c)?;
        self.calib = parse_calibration(&coeff);

        Ok(())
    }

    /// Writes the oversampling configuration used by [`Self::trigger_forced`].
    pub fn set_sampling(&mut self, sampling: SamplingConfig) -> Result<(), Error<I2C::Error>> {
        self.sampling = sampling;
        self.i2c
            .write(self.address, &[REG_CTRL_HUM, sampling.ctrl_hum()])
            .map_err(Error::I2c)?;
        self.i2c
            .write(
                self.address,
                &[REG_CTRL_MEAS, sampling.ctrl_meas(PowerMode::Sleep)],
            )
            .map_err(Error::I2c)
    }

    /// Triggers a single forced-mode conversion using the previously
    /// configured oversampling. The gas heater is never enabled (`ctrl_gas`
    /// is left at its power-on-reset value of "heater off"), so only
    /// temperature/pressure/humidity are converted.
    pub fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(
                self.address,
                &[REG_CTRL_MEAS, self.sampling.ctrl_meas(PowerMode::Forced)],
            )
            .map_err(Error::I2c)
    }

    /// Worst-case conversion time for the current sampling configuration.
    pub fn measurement_delay_ms(&self) -> u32 {
        self.sampling.measurement_delay_ms()
    }

    /// Reads and compensates the most recent conversion result.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut data = [0u8; LEN_FIELD];
        self.i2c
            .write_read(self.address, &[REG_FIELD0], &mut data)
            .map_err(Error::I2c)?;
        let raw = parse_raw_data(&data);
        Ok(make_measurement(raw, &self.calib))
    }

    /// Triggers a forced-mode conversion, waits the required time and reads
    /// back the compensated result.
    pub fn measure_forced<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Measurement, Error<I2C::Error>> {
        self.trigger_forced()?;
        delay.delay_ms(self.measurement_delay_ms());
        self.read_measurement()
    }
}

/// Async (embedded-hal-async 1.0) counterpart of the blocking API. See the
/// crate-root [`super::Bme680`] documentation; register map, calibration
/// parsing and compensation formulas are shared, not duplicated.
pub mod asynch {
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};

    use super::{
        CHIP_ID, CMD_SOFT_RESET, Calibration, Error, LEN_COEFF_ALL, LEN_COEFF1, LEN_FIELD,
        Measurement, PowerMode, REG_CHIP_ID, REG_COEFF1, REG_COEFF2, REG_CTRL_HUM, REG_CTRL_MEAS,
        REG_FIELD0, REG_SOFT_RESET, STARTUP_DELAY_MS, SamplingConfig, make_measurement,
        parse_calibration, parse_raw_data,
    };

    pub struct Bme680<I2C> {
        i2c: I2C,
        address: SevenBitAddress,
        calib: Calibration,
        sampling: SamplingConfig,
    }

    impl<I2C> Bme680<I2C> {
        pub const fn new(i2c: I2C, address: SevenBitAddress) -> Self {
            Self {
                i2c,
                address,
                calib: Calibration {
                    par_t1: 0,
                    par_t2: 0,
                    par_t3: 0,
                    par_p1: 0,
                    par_p2: 0,
                    par_p3: 0,
                    par_p4: 0,
                    par_p5: 0,
                    par_p6: 0,
                    par_p7: 0,
                    par_p8: 0,
                    par_p9: 0,
                    par_p10: 0,
                    par_h1: 0,
                    par_h2: 0,
                    par_h3: 0,
                    par_h4: 0,
                    par_h5: 0,
                    par_h6: 0,
                    par_h7: 0,
                },
                sampling: SamplingConfig {
                    temperature: super::Oversampling::Skip,
                    pressure: super::Oversampling::Skip,
                    humidity: super::Oversampling::Skip,
                },
            }
        }

        pub fn release(self) -> I2C {
            self.i2c
        }
    }

    impl<I2C> Bme680<I2C>
    where
        I2C: I2c<SevenBitAddress>,
    {
        pub async fn init<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<(), Error<I2C::Error>> {
            let mut id = [0u8; 1];
            self.i2c
                .write_read(self.address, &[REG_CHIP_ID], &mut id)
                .await
                .map_err(Error::I2c)?;
            if id[0] != CHIP_ID {
                return Err(Error::UnknownChip(id[0]));
            }

            self.i2c
                .write(self.address, &[REG_SOFT_RESET, CMD_SOFT_RESET])
                .await
                .map_err(Error::I2c)?;
            delay.delay_ms(STARTUP_DELAY_MS).await;

            let mut coeff1 = [0u8; LEN_COEFF1];
            self.i2c
                .write_read(self.address, &[REG_COEFF1], &mut coeff1)
                .await
                .map_err(Error::I2c)?;
            let mut coeff = [0u8; LEN_COEFF_ALL];
            coeff[..LEN_COEFF1].copy_from_slice(&coeff1);
            self.i2c
                .write_read(self.address, &[REG_COEFF2], &mut coeff[LEN_COEFF1..])
                .await
                .map_err(Error::I2c)?;
            self.calib = parse_calibration(&coeff);

            Ok(())
        }

        pub async fn set_sampling(
            &mut self,
            sampling: SamplingConfig,
        ) -> Result<(), Error<I2C::Error>> {
            self.sampling = sampling;
            self.i2c
                .write(self.address, &[REG_CTRL_HUM, sampling.ctrl_hum()])
                .await
                .map_err(Error::I2c)?;
            self.i2c
                .write(
                    self.address,
                    &[REG_CTRL_MEAS, sampling.ctrl_meas(PowerMode::Sleep)],
                )
                .await
                .map_err(Error::I2c)
        }

        pub async fn trigger_forced(&mut self) -> Result<(), Error<I2C::Error>> {
            self.i2c
                .write(
                    self.address,
                    &[REG_CTRL_MEAS, self.sampling.ctrl_meas(PowerMode::Forced)],
                )
                .await
                .map_err(Error::I2c)
        }

        pub fn measurement_delay_ms(&self) -> u32 {
            self.sampling.measurement_delay_ms()
        }

        pub async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut data = [0u8; LEN_FIELD];
            self.i2c
                .write_read(self.address, &[REG_FIELD0], &mut data)
                .await
                .map_err(Error::I2c)?;
            let raw = parse_raw_data(&data);
            Ok(make_measurement(raw, &self.calib))
        }

        pub async fn measure_forced<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Measurement, Error<I2C::Error>> {
            self.trigger_forced().await?;
            delay.delay_ms(self.measurement_delay_ms()).await;
            self.read_measurement().await
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Calibration values and worked-example ADC readings taken from the
    /// Bosch BME68x SensorAPI reference driver's self-test fixture, known
    /// to compensate to a plausible indoor-air reading (~23 degC /
    /// ~900 hPa / ~40 %RH).
    fn sample_calibration() -> Calibration {
        Calibration {
            par_t1: 26559,
            par_t2: 26234,
            par_t3: 3,
            par_p1: 33864,
            par_p2: -10405,
            par_p3: 88,
            par_p4: 6763,
            par_p5: -111,
            par_p6: 30,
            par_p7: 30,
            par_p8: -6485,
            par_p9: -2647,
            par_p10: 30,
            par_h1: 738,
            par_h2: 1008,
            par_h3: 0,
            par_h4: 45,
            par_h5: 20,
            par_h6: 120,
            par_h7: -100,
        }
    }

    #[test]
    fn temperature_pressure_and_humidity_match_reference_python_model() {
        // Golden values cross-checked against an independent Python
        // re-implementation of the same published Bosch integer formulas
        // (int32 arithmetic, arithmetic right shifts, C-style truncating
        // division), using the calibration constants from
        // `sample_calibration` and plausible ADC readings for an indoor
        // environment (~24 degC / ~987 hPa / ~41 %RH).
        let calib = sample_calibration();
        let (t_fine, temp) = compensate_temperature(501_700, &calib);
        assert_eq!(t_fine, 122_911);
        assert_eq!(temp, 2_401); // 24.01 degC

        let pressure = compensate_pressure(400_000, t_fine, &calib);
        assert_eq!(pressure, 98_654); // Pa

        assert_eq!(compensate_humidity(0, t_fine, &calib), 0);
        assert_eq!(compensate_humidity(20_000, t_fine, &calib), 41_023); // 41.023 %RH (milli-percent)
    }

    #[test]
    fn humidity_is_clamped_to_valid_range() {
        let calib = sample_calibration();
        let (t_fine, _) = compensate_temperature(501_700, &calib);
        let humidity_low = compensate_humidity(0, t_fine, &calib);
        assert_eq!(humidity_low, 0);
        let humidity_high = compensate_humidity(u32::from(u16::MAX), t_fine, &calib);
        assert_eq!(humidity_high, 100_000);
    }

    #[test]
    fn chip_id_constant_matches_datasheet() {
        assert_eq!(CHIP_ID, 0x61);
    }

    #[test]
    fn ctrl_meas_encodes_oversampling_and_mode() {
        let sampling = SamplingConfig {
            temperature: Oversampling::X2,
            pressure: Oversampling::X16,
            humidity: Oversampling::X1,
        };
        assert_eq!(sampling.ctrl_meas(PowerMode::Forced), 0x55);
        assert_eq!(sampling.ctrl_hum(), 0b001);
    }

    #[test]
    fn measurement_delay_scales_with_oversampling_cycles() {
        let skip_all = SamplingConfig {
            temperature: Oversampling::Skip,
            pressure: Oversampling::Skip,
            humidity: Oversampling::Skip,
        };
        // 0 cycles: 477*4 + 477*5 + 1000 = 1908+2385+1000 = 5293us -> 6ms
        assert_eq!(skip_all.measurement_delay_ms(), 6);

        let max_all = SamplingConfig {
            temperature: Oversampling::X16,
            pressure: Oversampling::X16,
            humidity: Oversampling::X16,
        };
        // 48 cycles * 1963 + 5293 = 94224+5293 = 99517us -> 100ms
        assert_eq!(max_all.measurement_delay_ms(), 100);
    }

    #[test]
    fn raw_data_parses_20_bit_fields_and_ignores_gas_bytes() {
        let mut data = [0u8; LEN_FIELD];
        data[2] = 0xFF;
        data[3] = 0xFF;
        data[4] = 0xF0;
        data[8] = 0x80;
        data[9] = 0x00;
        let raw = parse_raw_data(&data);
        assert_eq!(raw.pressure, 0xFFFFF);
        assert_eq!(raw.temperature, 0);
        assert_eq!(raw.humidity, 0x8000);
    }

    #[test]
    fn calibration_parsing_reads_signed_and_split_fields_correctly() {
        // Craft a coefficient array where every field has a distinguishable
        // value to catch index/sign mistakes.
        let mut data = [0u8; LEN_COEFF_ALL];
        data[0] = 0x34; // T2 LSB
        data[1] = 0x12; // T2 MSB -> par_t2 = 0x1234
        data[2] = 0xFF; // T3 = -1
        data[31] = 0x78; // T1 LSB
        data[32] = 0x56; // T1 MSB -> par_t1 = 0x5678
        data[23] = 0xAB; // H2 MSB nibble source
        data[24] = 0xCD; // shared H1/H2 byte
        data[25] = 0xEF; // H1 MSB nibble source

        let calib = parse_calibration(&data);
        assert_eq!(calib.par_t2, 0x1234);
        assert_eq!(calib.par_t3, -1);
        assert_eq!(calib.par_t1, 0x5678);
        assert_eq!(calib.par_h2, (0xABu16 << 4) | (0xCDu16 >> 4));
        assert_eq!(calib.par_h1, (0xEFu16 << 4) | (0xCDu16 & 0x0F));
    }

    #[test]
    fn compensation_does_not_panic_across_full_adc_range() {
        // Sweeps representative points across the full 20-bit (T/P) and
        // 16-bit (H) ADC ranges to guard against overflow panics on inputs
        // outside the "typical" range exercised by the golden-value test.
        let calib = sample_calibration();
        for adc_t in (0..=0xFFFFFu32).step_by(40_001) {
            let (t_fine, _) = compensate_temperature(adc_t, &calib);
            for adc_p in (0..=0xFFFFFu32).step_by(50_021) {
                let _ = compensate_pressure(adc_p, t_fine, &calib);
            }
            for adc_h in (0..=0xFFFFu32).step_by(4_001) {
                let _ = compensate_humidity(adc_h, t_fine, &calib);
            }
        }
    }
}
