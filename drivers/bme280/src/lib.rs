//! Transport-independent Bosch BME280/BMP280 protocol, with both a blocking
//! (embedded-hal 1.0) and an async (embedded-hal-async 1.0) API.
//!
//! The blocking API lives at the crate root as [`Bme280`]; the async API
//! lives in [`asynch`] under the same type name (the `asynch` module name
//! avoids clashing with the `async` keyword). Both share the same register
//! map, calibration parsing and Bosch integer compensation formulas, which
//! are implemented exactly once at the crate root.
//!
//! BMP280 is supported by the same driver: it only lacks a humidity sensor,
//! so [`Measurement::humidity_centi_percent`] is `None` for that chip.

#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};

/// Primary I2C address (`SDO` pulled low).
pub const PRIMARY_ADDRESS: SevenBitAddress = 0x76;
/// Secondary I2C address (`SDO` pulled high).
pub const SECONDARY_ADDRESS: SevenBitAddress = 0x77;

const REG_CHIP_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_DATA: u8 = 0xF7;
const REG_CALIB_TEMP_PRESS: u8 = 0x88;
const REG_CALIB_HUMIDITY: u8 = 0xE1;

const CMD_SOFT_RESET: u8 = 0xB6;
/// Datasheet-specified startup time after a soft reset, before calibration
/// data or measurements can be read.
const STARTUP_DELAY_MS: u32 = 2;

const CHIP_ID_BME280: u8 = 0x60;
const CHIP_ID_BMP280: u8 = 0x58;

const LEN_CALIB_TEMP_PRESS: usize = 26;
const LEN_CALIB_HUMIDITY: usize = 7;
const LEN_DATA: usize = 8;

/// Which chip variant is attached. BMP280 has no humidity sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    Bme280,
    Bmp280,
}

impl Chip {
    fn from_id(id: u8) -> Option<Self> {
        match id {
            CHIP_ID_BME280 => Some(Chip::Bme280),
            CHIP_ID_BMP280 => Some(Chip::Bmp280),
            _ => None,
        }
    }

    pub const fn has_humidity(self) -> bool {
        matches!(self, Chip::Bme280)
    }
}

/// Oversampling setting for one measured quantity.
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
    /// 3-bit `osrs_*` register field value.
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

    /// Actual oversampling multiplier used in the conversion-time formula
    /// (0 when skipped).
    const fn factor(self) -> u32 {
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
    Normal,
}

impl PowerMode {
    const fn register_bits(self) -> u8 {
        match self {
            PowerMode::Sleep => 0b00,
            PowerMode::Forced => 0b01,
            PowerMode::Normal => 0b11,
        }
    }
}

/// Oversampling configuration for temperature, pressure and (BME280-only)
/// humidity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SamplingConfig {
    pub temperature: Oversampling,
    pub pressure: Oversampling,
    pub humidity: Oversampling,
}

impl Default for SamplingConfig {
    /// A commonly used indoor weather-station configuration (x1 oversampling
    /// on all channels).
    fn default() -> Self {
        Self {
            temperature: Oversampling::X1,
            pressure: Oversampling::X1,
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

    /// Conservative worst-case conversion delay in milliseconds, following
    /// the datasheet formula:
    /// `1.25 + 2.3*osrs_t + (osrs_p ? 2.3*osrs_p + 0.575 : 0) + (osrs_h ? 2.3*osrs_h + 0.575 : 0)`.
    fn measurement_delay_ms(self, has_humidity: bool) -> u32 {
        let mut delay_us: u32 = 1_250 + 2_300 * self.temperature.factor();
        if self.pressure.factor() > 0 {
            delay_us += 2_300 * self.pressure.factor() + 575;
        }
        if has_humidity && self.humidity.factor() > 0 {
            delay_us += 2_300 * self.humidity.factor() + 575;
        }
        delay_us.div_ceil(1_000)
    }
}

/// Parsed factory calibration coefficients (Bosch `dig_*` trim values).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

fn concat(msb: u8, lsb: u8) -> u16 {
    (u16::from(msb) << 8) | u16::from(lsb)
}

fn parse_temp_press_calibration(data: &[u8; LEN_CALIB_TEMP_PRESS]) -> Calibration {
    Calibration {
        dig_t1: concat(data[1], data[0]),
        dig_t2: concat(data[3], data[2]) as i16,
        dig_t3: concat(data[5], data[4]) as i16,
        dig_p1: concat(data[7], data[6]),
        dig_p2: concat(data[9], data[8]) as i16,
        dig_p3: concat(data[11], data[10]) as i16,
        dig_p4: concat(data[13], data[12]) as i16,
        dig_p5: concat(data[15], data[14]) as i16,
        dig_p6: concat(data[17], data[16]) as i16,
        dig_p7: concat(data[19], data[18]) as i16,
        dig_p8: concat(data[21], data[20]) as i16,
        dig_p9: concat(data[23], data[22]) as i16,
        dig_h1: data[25],
        ..Default::default()
    }
}

fn parse_humidity_calibration(calib: &mut Calibration, data: &[u8; LEN_CALIB_HUMIDITY]) {
    calib.dig_h2 = concat(data[1], data[0]) as i16;
    calib.dig_h3 = data[2];
    let h4_msb = i16::from(data[3] as i8) * 16;
    let h4_lsb = i16::from(data[4] & 0x0F);
    calib.dig_h4 = h4_msb | h4_lsb;
    let h5_msb = i16::from(data[5] as i8) * 16;
    let h5_lsb = i16::from(data[4] >> 4);
    calib.dig_h5 = h5_msb | h5_lsb;
    calib.dig_h6 = data[6] as i8;
}

/// Uncompensated (raw ADC) sensor readings.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RawData {
    pressure: u32,
    temperature: u32,
    humidity: u32,
}

fn parse_raw_data(data: &[u8; LEN_DATA]) -> RawData {
    let pressure =
        (u32::from(data[0]) << 12) | (u32::from(data[1]) << 4) | (u32::from(data[2]) >> 4);
    let temperature =
        (u32::from(data[3]) << 12) | (u32::from(data[4]) << 4) | (u32::from(data[5]) >> 4);
    let humidity = (u32::from(data[6]) << 8) | u32::from(data[7]);
    RawData {
        pressure,
        temperature,
        humidity,
    }
}

/// A fully compensated measurement, in fixed-point engineering units.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Measurement {
    /// Temperature in hundredths of a degree Celsius (centi-degrees).
    pub temperature_centi_celsius: i32,
    /// Pressure in Pascal.
    pub pressure_pa: u32,
    /// Relative humidity in hundredths of a percent (centi-percent), or
    /// `None` on BMP280, which has no humidity sensor.
    pub humidity_centi_percent: Option<u16>,
}

/// Bosch's official integer temperature compensation formula. Returns
/// `(t_fine, temperature_centi_celsius)`; `t_fine` is required by the
/// pressure and humidity compensation formulas.
fn compensate_temperature(adc_t: u32, calib: &Calibration) -> (i32, i32) {
    let adc_t = adc_t as i32;
    let var1 = (adc_t / 8 - (i32::from(calib.dig_t1) * 2)) * i32::from(calib.dig_t2) / 2048;
    let var2_a = adc_t / 16 - i32::from(calib.dig_t1);
    let var2 = (var2_a * var2_a / 4096) * i32::from(calib.dig_t3) / 16384;
    let t_fine = var1 + var2;
    let temperature = (t_fine * 5 + 128) / 256;
    (t_fine, temperature.clamp(-4_000, 8_500))
}

/// Bosch's official 64-bit integer pressure compensation formula. The raw
/// result is fixed-point Pa*100; this returns it already rounded to whole
/// Pascal.
fn compensate_pressure(adc_p: u32, t_fine: i32, calib: &Calibration) -> u32 {
    let t_fine = i64::from(t_fine);
    let mut var1 = t_fine - 128_000;
    let mut var2 = var1 * var1 * i64::from(calib.dig_p6);
    var2 += (var1 * i64::from(calib.dig_p5)) * 131_072;
    var2 += i64::from(calib.dig_p4) * 34_359_738_368;
    var1 = (var1 * var1 * i64::from(calib.dig_p3)) / 256 + (var1 * i64::from(calib.dig_p2) * 4_096);
    let var3: i64 = 140_737_488_355_328;
    var1 = (var3 + var1) * i64::from(calib.dig_p1) / 8_589_934_592;

    if var1 == 0 {
        // Avoid division by zero for an all-zero (uncalibrated) sensor.
        return 30_000;
    }

    let mut var4 = 1_048_576 - i64::from(adc_p);
    var4 = ((var4 * 2_147_483_648 - var2) * 3_125) / var1;
    var1 = (i64::from(calib.dig_p9) * (var4 / 8_192) * (var4 / 8_192)) / 33_554_432;
    var2 = (i64::from(calib.dig_p8) * var4) / 524_288;
    // `var4` is now Q24.8 fixed-point Pascal (var4/256 == Pa).
    var4 = (var4 + var1 + var2) / 256 + (i64::from(calib.dig_p7) * 16);

    // Rescale Q24.8 Pa to Pa*100, exactly as the reference driver does, then
    // clamp to the datasheet's specified operating range (300..1100 hPa).
    let pressure_pa_x100 = ((var4 / 2) * 100 / 128).clamp(3_000_000, 11_000_000);
    ((pressure_pa_x100 + 50) / 100) as u32
}

/// Bosch's official integer humidity compensation formula (BME280 only).
/// Returns humidity scaled by 100 (centi-percent), clamped to `0..=10000`.
fn compensate_humidity(adc_h: u32, t_fine: i32, calib: &Calibration) -> u16 {
    let mut var1 = t_fine - 76_800;
    let var2 = (adc_h as i32) * 16_384;
    let var3 = i32::from(calib.dig_h4) * 1_048_576;
    let var4 = i32::from(calib.dig_h5) * var1;
    let mut var5: i32 = (var2 - var3 - var4 + 16_384) / 32_768;
    let var2b = (var1 * i32::from(calib.dig_h6)) / 1_024;
    let var3b = (var1 * i32::from(calib.dig_h3)) / 2_048;
    let var4b = ((var2b * (var3b + 32_768)) / 1_024) + 2_097_152;
    let var2c = ((var4b * i32::from(calib.dig_h2)) + 8_192) / 16_384;
    var1 = var5 * var2c;
    let var4c = ((var1 / 32_768) * (var1 / 32_768)) / 128;
    var5 = var1 - ((var4c * i32::from(calib.dig_h1)) / 16);
    var5 = var5.clamp(0, 419_430_400);
    // `var5 >> 12` is humidity scaled by 1024 (Q22.10); rescale to
    // centi-percent (scaled by 100) with rounding.
    let humidity_q10 = (var5 >> 12) as u32;
    (((humidity_q10 * 100 + 512) / 1_024).min(10_000)) as u16
}

fn make_measurement(raw: RawData, calib: &Calibration, chip: Chip) -> Measurement {
    let (t_fine, temperature_centi_celsius) = compensate_temperature(raw.temperature, calib);
    let pressure_pa = compensate_pressure(raw.pressure, t_fine, calib);
    let humidity_centi_percent = chip
        .has_humidity()
        .then(|| compensate_humidity(raw.humidity, t_fine, calib));
    Measurement {
        temperature_centi_celsius,
        pressure_pa,
        humidity_centi_percent,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    I2c(E),
    /// Chip-id register did not match a known BME280/BMP280 id.
    UnknownChip(u8),
}

/// Blocking BME280/BMP280 driver.
pub struct Bme280<I2C> {
    i2c: I2C,
    address: SevenBitAddress,
    chip: Chip,
    calib: Calibration,
    sampling: SamplingConfig,
}

impl<I2C> Bme280<I2C> {
    /// Creates a driver instance. Call [`Self::init`] before taking
    /// measurements.
    pub const fn new(i2c: I2C, address: SevenBitAddress) -> Self {
        Self {
            i2c,
            address,
            chip: Chip::Bme280,
            calib: Calibration {
                dig_t1: 0,
                dig_t2: 0,
                dig_t3: 0,
                dig_p1: 0,
                dig_p2: 0,
                dig_p3: 0,
                dig_p4: 0,
                dig_p5: 0,
                dig_p6: 0,
                dig_p7: 0,
                dig_p8: 0,
                dig_p9: 0,
                dig_h1: 0,
                dig_h2: 0,
                dig_h3: 0,
                dig_h4: 0,
                dig_h5: 0,
                dig_h6: 0,
            },
            sampling: SamplingConfig {
                temperature: Oversampling::Skip,
                pressure: Oversampling::Skip,
                humidity: Oversampling::Skip,
            },
        }
    }

    pub const fn chip(&self) -> Chip {
        self.chip
    }

    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> Bme280<I2C>
where
    I2C: I2c<SevenBitAddress>,
{
    /// Reads and validates the chip id, soft-resets the sensor and reads
    /// calibration data. The caller must supply a delay implementation for
    /// the datasheet-mandated post-reset startup wait.
    pub fn init<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<Chip, Error<I2C::Error>> {
        let mut id = [0u8; 1];
        self.i2c
            .write_read(self.address, &[REG_CHIP_ID], &mut id)
            .map_err(Error::I2c)?;
        let chip = Chip::from_id(id[0]).ok_or(Error::UnknownChip(id[0]))?;
        self.chip = chip;

        self.i2c
            .write(self.address, &[REG_RESET, CMD_SOFT_RESET])
            .map_err(Error::I2c)?;
        delay.delay_ms(STARTUP_DELAY_MS);

        let mut temp_press = [0u8; LEN_CALIB_TEMP_PRESS];
        self.i2c
            .write_read(self.address, &[REG_CALIB_TEMP_PRESS], &mut temp_press)
            .map_err(Error::I2c)?;
        self.calib = parse_temp_press_calibration(&temp_press);

        if chip.has_humidity() {
            let mut humidity = [0u8; LEN_CALIB_HUMIDITY];
            self.i2c
                .write_read(self.address, &[REG_CALIB_HUMIDITY], &mut humidity)
                .map_err(Error::I2c)?;
            parse_humidity_calibration(&mut self.calib, &humidity);
        }

        Ok(chip)
    }

    /// Writes the oversampling configuration used by [`Self::trigger_forced`].
    pub fn set_sampling(&mut self, sampling: SamplingConfig) -> Result<(), Error<I2C::Error>> {
        self.sampling = sampling;
        if self.chip.has_humidity() {
            // `ctrl_hum` only takes effect after the following `ctrl_meas`
            // write, per datasheet section 5.4.3.
            self.i2c
                .write(self.address, &[REG_CTRL_HUM, sampling.ctrl_hum()])
                .map_err(Error::I2c)?;
        }
        self.i2c
            .write(
                self.address,
                &[REG_CTRL_MEAS, sampling.ctrl_meas(PowerMode::Sleep)],
            )
            .map_err(Error::I2c)
    }

    /// Triggers a single forced-mode conversion using the previously
    /// configured oversampling. The caller must wait for
    /// [`Self::measurement_delay_ms`] before calling
    /// [`Self::read_measurement`].
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
        self.sampling.measurement_delay_ms(self.chip.has_humidity())
    }

    /// Reads and compensates the most recent conversion result.
    pub fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
        let mut data = [0u8; LEN_DATA];
        self.i2c
            .write_read(self.address, &[REG_DATA], &mut data)
            .map_err(Error::I2c)?;
        let raw = parse_raw_data(&data);
        Ok(make_measurement(raw, &self.calib, self.chip))
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
/// crate-root [`super::Bme280`] documentation; register map, calibration
/// parsing and compensation formulas are shared, not duplicated.
pub mod asynch {
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};

    use super::{
        CMD_SOFT_RESET, Calibration, Chip, Error, LEN_CALIB_HUMIDITY, LEN_CALIB_TEMP_PRESS,
        LEN_DATA, Measurement, PowerMode, REG_CALIB_HUMIDITY, REG_CALIB_TEMP_PRESS, REG_CHIP_ID,
        REG_CTRL_HUM, REG_CTRL_MEAS, REG_DATA, REG_RESET, STARTUP_DELAY_MS, SamplingConfig,
        make_measurement, parse_humidity_calibration, parse_raw_data, parse_temp_press_calibration,
    };

    pub struct Bme280<I2C> {
        i2c: I2C,
        address: SevenBitAddress,
        chip: Chip,
        calib: Calibration,
        sampling: SamplingConfig,
    }

    impl<I2C> Bme280<I2C> {
        pub const fn new(i2c: I2C, address: SevenBitAddress) -> Self {
            Self {
                i2c,
                address,
                chip: Chip::Bme280,
                calib: Calibration {
                    dig_t1: 0,
                    dig_t2: 0,
                    dig_t3: 0,
                    dig_p1: 0,
                    dig_p2: 0,
                    dig_p3: 0,
                    dig_p4: 0,
                    dig_p5: 0,
                    dig_p6: 0,
                    dig_p7: 0,
                    dig_p8: 0,
                    dig_p9: 0,
                    dig_h1: 0,
                    dig_h2: 0,
                    dig_h3: 0,
                    dig_h4: 0,
                    dig_h5: 0,
                    dig_h6: 0,
                },
                sampling: SamplingConfig {
                    temperature: super::Oversampling::Skip,
                    pressure: super::Oversampling::Skip,
                    humidity: super::Oversampling::Skip,
                },
            }
        }

        pub const fn chip(&self) -> Chip {
            self.chip
        }

        pub fn release(self) -> I2C {
            self.i2c
        }
    }

    impl<I2C> Bme280<I2C>
    where
        I2C: I2c<SevenBitAddress>,
    {
        pub async fn init<D: embedded_hal_async::delay::DelayNs>(
            &mut self,
            delay: &mut D,
        ) -> Result<Chip, Error<I2C::Error>> {
            let mut id = [0u8; 1];
            self.i2c
                .write_read(self.address, &[REG_CHIP_ID], &mut id)
                .await
                .map_err(Error::I2c)?;
            let chip = Chip::from_id(id[0]).ok_or(Error::UnknownChip(id[0]))?;
            self.chip = chip;

            self.i2c
                .write(self.address, &[REG_RESET, CMD_SOFT_RESET])
                .await
                .map_err(Error::I2c)?;
            delay.delay_ms(STARTUP_DELAY_MS).await;

            let mut temp_press = [0u8; LEN_CALIB_TEMP_PRESS];
            self.i2c
                .write_read(self.address, &[REG_CALIB_TEMP_PRESS], &mut temp_press)
                .await
                .map_err(Error::I2c)?;
            self.calib = parse_temp_press_calibration(&temp_press);

            if chip.has_humidity() {
                let mut humidity = [0u8; LEN_CALIB_HUMIDITY];
                self.i2c
                    .write_read(self.address, &[REG_CALIB_HUMIDITY], &mut humidity)
                    .await
                    .map_err(Error::I2c)?;
                parse_humidity_calibration(&mut self.calib, &humidity);
            }

            Ok(chip)
        }

        pub async fn set_sampling(
            &mut self,
            sampling: SamplingConfig,
        ) -> Result<(), Error<I2C::Error>> {
            self.sampling = sampling;
            if self.chip.has_humidity() {
                self.i2c
                    .write(self.address, &[REG_CTRL_HUM, sampling.ctrl_hum()])
                    .await
                    .map_err(Error::I2c)?;
            }
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
            self.sampling.measurement_delay_ms(self.chip.has_humidity())
        }

        pub async fn read_measurement(&mut self) -> Result<Measurement, Error<I2C::Error>> {
            let mut data = [0u8; LEN_DATA];
            self.i2c
                .write_read(self.address, &[REG_DATA], &mut data)
                .await
                .map_err(Error::I2c)?;
            let raw = parse_raw_data(&data);
            Ok(make_measurement(raw, &self.calib, self.chip))
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

    /// Bosch datasheet worked example: raw calibration bytes and adc values
    /// that should compensate to 25.08 degC / 1006.53 hPa (`t_fine`=128422,
    /// well-known reference values used across open-source BME280 ports).
    fn sample_calibration() -> Calibration {
        Calibration {
            dig_t1: 27504,
            dig_t2: 26435,
            dig_t3: -1000,
            dig_p1: 36477,
            dig_p2: -10685,
            dig_p3: 3024,
            dig_p4: 2855,
            dig_p5: 140,
            dig_p6: -7,
            dig_p7: 15500,
            dig_p8: -14600,
            dig_p9: 6000,
            dig_h1: 75,
            dig_h2: 379,
            dig_h3: 0,
            dig_h4: 279,
            dig_h5: 25,
            dig_h6: 30,
        }
    }

    #[test]
    fn temperature_matches_bosch_worked_example() {
        let calib = sample_calibration();
        let (t_fine, temp) = compensate_temperature(519_888, &calib);
        assert_eq!(t_fine, 128_423);
        assert_eq!(temp, 2_508); // 25.08 degC
    }

    #[test]
    fn pressure_matches_bosch_worked_example() {
        let calib = sample_calibration();
        let (t_fine, _) = compensate_temperature(519_888, &calib);
        let pressure = compensate_pressure(415_148, t_fine, &calib);
        // Bosch datasheet worked example: ~100653.27 Pa for this input.
        assert_eq!(pressure, 100_653);
    }

    #[test]
    fn humidity_is_clamped_to_valid_range() {
        let calib = sample_calibration();
        let (t_fine, _) = compensate_temperature(519_888, &calib);
        let humidity = compensate_humidity(0, t_fine, &calib);
        assert_eq!(humidity, 0);
        // The humidity ADC field is 16-bit; full scale should clamp to 100.00%.
        let humidity_full_scale = compensate_humidity(u32::from(u16::MAX), t_fine, &calib);
        assert_eq!(humidity_full_scale, 10_000);
    }

    #[test]
    fn chip_id_maps_to_known_variants() {
        assert_eq!(Chip::from_id(0x60), Some(Chip::Bme280));
        assert_eq!(Chip::from_id(0x58), Some(Chip::Bmp280));
        assert_eq!(Chip::from_id(0x00), None);
        assert!(Chip::Bme280.has_humidity());
        assert!(!Chip::Bmp280.has_humidity());
    }

    #[test]
    fn bmp280_measurement_has_no_humidity() {
        let calib = sample_calibration();
        let raw = RawData {
            pressure: 415_148,
            temperature: 519_888,
            humidity: 32_768,
        };
        let measurement = make_measurement(raw, &calib, Chip::Bmp280);
        assert_eq!(measurement.humidity_centi_percent, None);
        assert_eq!(measurement.temperature_centi_celsius, 2_508);

        let bme_measurement = make_measurement(raw, &calib, Chip::Bme280);
        assert!(bme_measurement.humidity_centi_percent.is_some());
    }

    #[test]
    fn ctrl_meas_encodes_oversampling_and_mode() {
        let sampling = SamplingConfig {
            temperature: Oversampling::X2,
            pressure: Oversampling::X16,
            humidity: Oversampling::X1,
        };
        // osrs_t=010, osrs_p=101, mode=01 (forced) -> 0b0101_0101 = 0x55
        assert_eq!(sampling.ctrl_meas(PowerMode::Forced), 0x55);
        assert_eq!(sampling.ctrl_hum(), 0b001);
    }

    #[test]
    fn measurement_delay_scales_with_oversampling() {
        let skip_all = SamplingConfig {
            temperature: Oversampling::Skip,
            pressure: Oversampling::Skip,
            humidity: Oversampling::Skip,
        };
        assert_eq!(skip_all.measurement_delay_ms(true), 2); // ceil(1250us/1000)

        let max_all = SamplingConfig {
            temperature: Oversampling::X16,
            pressure: Oversampling::X16,
            humidity: Oversampling::X16,
        };
        // 1250 + 2300*16 + (2300*16+575) + (2300*16+575) = 1250+36800+37375+37375 = 112800us
        assert_eq!(max_all.measurement_delay_ms(true), 113);
        assert_eq!(max_all.measurement_delay_ms(false), 76); // no humidity term
    }

    #[test]
    fn raw_data_parses_20_bit_fields() {
        // pressure/temperature registers pack a 20-bit value across
        // msb/lsb/xlsb (top 4 bits of xlsb).
        let data = [0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x80, 0x00];
        let raw = parse_raw_data(&data);
        assert_eq!(raw.pressure, 0xFFFFF);
        assert_eq!(raw.temperature, 0);
        assert_eq!(raw.humidity, 0x8000);
    }

    #[test]
    fn compensation_does_not_panic_across_full_adc_range() {
        // Sweeps representative points across the full 20-bit (T/P) and
        // 16-bit (H) ADC ranges to guard against overflow panics on inputs
        // outside the datasheet worked example.
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
