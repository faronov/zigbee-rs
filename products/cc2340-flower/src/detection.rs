//! Autodetection and persistence of the optional I2C sensors.
//!
//! The flower board can be populated with any subset of TMP102 (0x48),
//! BH1750 (0x23) and OPT3001 (0x44). The Simple Descriptor advertised to the
//! coordinator is derived from the detected set (see
//! [`crate::profile::flower_profile`]), so the set must stay stable while
//! the device is commissioned: a sensor that fails to answer one boot must
//! not make a cluster disappear from an interviewed device.
//!
//! [`resolve_at_boot`] therefore probes the bus only while the device is
//! factory-new (never joined, or after leave/factory reset) and persists the
//! result in a dedicated 2-KiB flash sector. A commissioned boot reuses the
//! persisted set without touching the bus.

use embedded_hal::i2c::I2c;
use embedded_storage::nor_flash::NorFlash;

/// The set of fitted optional sensors.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DetectedSensors {
    pub tmp102: bool,
    pub bh1750: bool,
    pub opt3001: bool,
}

/// Temperature cluster source (the cluster itself is always present).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TemperatureSource {
    Tmp102,
    /// CC2340 BATMON die temperature: coarse, but keeps 0x0402 meaningful
    /// on boards without a TMP102.
    DieTemperature,
}

/// Illuminance cluster source; absent means no Illuminance cluster.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LightSensor {
    Opt3001,
    Bh1750,
}

const BIT_TMP102: u8 = 1 << 0;
const BIT_BH1750: u8 = 1 << 1;
const BIT_OPT3001: u8 = 1 << 2;
const KNOWN_BITS: u8 = BIT_TMP102 | BIT_BH1750 | BIT_OPT3001;

impl DetectedSensors {
    pub const NONE: Self = Self {
        tmp102: false,
        bh1750: false,
        opt3001: false,
    };

    pub const fn bits(self) -> u8 {
        (self.tmp102 as u8 * BIT_TMP102)
            | (self.bh1750 as u8 * BIT_BH1750)
            | (self.opt3001 as u8 * BIT_OPT3001)
    }

    /// Decode a persisted set, rejecting bits this firmware does not know.
    pub const fn from_bits(bits: u8) -> Option<Self> {
        if bits & !KNOWN_BITS != 0 {
            return None;
        }
        Some(Self {
            tmp102: bits & BIT_TMP102 != 0,
            bh1750: bits & BIT_BH1750 != 0,
            opt3001: bits & BIT_OPT3001 != 0,
        })
    }

    pub const fn temperature_source(self) -> TemperatureSource {
        if self.tmp102 {
            TemperatureSource::Tmp102
        } else {
            TemperatureSource::DieTemperature
        }
    }

    /// The light sensor used for the Illuminance cluster. If both are
    /// fitted, the OPT3001 wins: it has the wider range, a positive device
    /// ID check, and automatic full-scale ranging.
    pub const fn light_sensor(self) -> Option<LightSensor> {
        if self.opt3001 {
            Some(LightSensor::Opt3001)
        } else if self.bh1750 {
            Some(LightSensor::Bh1750)
        } else {
            None
        }
    }

    pub const fn has_light_sensor(self) -> bool {
        self.light_sensor().is_some()
    }
}

/// Probe every supported sensor and leave each detected one in its lowest
/// power state. A sensor counts as present only if it acknowledges both the
/// probe and the power-down command.
pub fn detect<I2C: I2c>(bus: &mut I2C) -> DetectedSensors {
    let mut tmp102 = zigbee_tmp102::Tmp102::new(&mut *bus, zigbee_tmp102::ADDRESS_GND);
    let tmp102_present = tmp102.probe().is_ok() && tmp102.shutdown().is_ok();

    // The BH1750 has no ID register; the reference also detects it by an
    // acknowledged command. Power-down is the harmless one.
    let mut bh1750 = zigbee_bh1750::Bh1750::new(&mut *bus, zigbee_bh1750::ADDRESS_LOW);
    let bh1750_present = bh1750.power_down().is_ok();

    let mut opt3001 = zigbee_opt3001::Opt3001::new(&mut *bus, zigbee_opt3001::ADDRESS_GND);
    let opt3001_present = opt3001.probe().is_ok() && opt3001.shutdown().is_ok();

    DetectedSensors {
        tmp102: tmp102_present,
        bh1750: bh1750_present,
        opt3001: opt3001_present,
    }
}

// ── Persistent record ───────────────────────────────────────────────

const RECORD_MAGIC: u32 = 0x464C_5752; // "FLWR"
const RECORD_VERSION: u8 = 1;
/// Multiple of every CC2340 flash write granule.
pub const RECORD_LEN: usize = 16;

/// `magic(4) version(1) bits(1) !bits(1) 0xFF(1) crc32(4) 0xFF(4)`;
/// the CRC covers bytes 0..8.
pub fn encode_record(sensors: DetectedSensors) -> [u8; RECORD_LEN] {
    let mut record = [0xFF; RECORD_LEN];
    record[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
    record[4] = RECORD_VERSION;
    record[5] = sensors.bits();
    record[6] = !sensors.bits();
    record[7] = 0xFF;
    let crc = crc32(&record[0..8]);
    record[8..12].copy_from_slice(&crc.to_le_bytes());
    record
}

pub fn decode_record(record: &[u8; RECORD_LEN]) -> Option<DetectedSensors> {
    let magic = u32::from_le_bytes([record[0], record[1], record[2], record[3]]);
    let crc = u32::from_le_bytes([record[8], record[9], record[10], record[11]]);
    if magic != RECORD_MAGIC
        || record[4] != RECORD_VERSION
        || record[5] != !record[6]
        || crc != crc32(&record[0..8])
    {
        return None;
    }
    DetectedSensors::from_bits(record[5])
}

/// IEEE 802.3 CRC-32 (reflected, poly 0xEDB88320), bitwise: the record is
/// tiny and read once per boot.
const fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    let mut index = 0;
    while index < bytes.len() {
        crc ^= bytes[index] as u32;
        let mut bit = 0;
        while bit < 8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
            bit += 1;
        }
        index += 1;
    }
    !crc
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionStoreError<E> {
    Flash(E),
    /// The written record did not read back intact.
    Verify,
}

/// The detected-sensor record in its own erase sector (normally a
/// [`crate::storage::DetectionPartition`]).
pub struct DetectionStore<F> {
    flash: F,
}

impl<F: NorFlash> DetectionStore<F> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    /// The persisted set, or `None` if the sector is erased, torn or from
    /// another format. Read errors are treated the same: the caller then
    /// re-detects.
    pub fn load(&mut self) -> Option<DetectedSensors> {
        let mut record = [0u8; RECORD_LEN];
        self.flash.read(0, &mut record).ok()?;
        decode_record(&record)
    }

    /// Replace the record. An interrupted erase or write leaves an erased or
    /// CRC-invalid sector, which [`Self::load`] rejects; the next boot is
    /// then necessarily factory-new and simply detects again.
    pub fn save(&mut self, sensors: DetectedSensors) -> Result<(), DetectionStoreError<F::Error>> {
        let erase_end = F::ERASE_SIZE.max(RECORD_LEN) as u32;
        self.flash
            .erase(0, erase_end)
            .map_err(DetectionStoreError::Flash)?;
        self.flash
            .write(0, &encode_record(sensors))
            .map_err(DetectionStoreError::Flash)?;
        if self.load() == Some(sensors) {
            Ok(())
        } else {
            Err(DetectionStoreError::Verify)
        }
    }
}

/// How the boot-time sensor set was obtained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionOrigin {
    /// Commissioned boot: the persisted set was reused without probing.
    Persisted,
    /// Factory-new boot: probed; the record already matched.
    DetectedUnchanged,
    /// Factory-new boot: probed and the new set was persisted.
    DetectedSaved,
    /// Probed, but persisting failed. The set is still used for this boot.
    DetectedNotSaved,
}

/// Resolve the sensor set that defines this boot's endpoint.
///
/// - Commissioned with a valid record: reuse it, do not probe.
/// - Otherwise (factory-new, or a commissioned device whose record is
///   missing/corrupt): probe with `probe` and persist the result if it
///   differs from the stored one, to avoid needless erase cycles.
pub fn resolve_at_boot<F: NorFlash>(
    store: &mut DetectionStore<F>,
    commissioned: bool,
    probe: impl FnOnce() -> DetectedSensors,
) -> (DetectedSensors, DetectionOrigin) {
    let stored = store.load();
    if commissioned && let Some(sensors) = stored {
        return (sensors, DetectionOrigin::Persisted);
    }
    let detected = probe();
    if stored == Some(detected) {
        return (detected, DetectionOrigin::DetectedUnchanged);
    }
    match store.save(detected) {
        Ok(()) => (detected, DetectionOrigin::DetectedSaved),
        Err(_) => (detected, DetectionOrigin::DetectedNotSaved),
    }
}

#[cfg(test)]
pub(crate) mod fake_bus {
    extern crate std;

    use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};
    use std::vec::Vec;

    /// I2C bus emulating the subset of TMP102/BH1750/OPT3001 behavior the
    /// product uses. Absent addresses NACK.
    #[derive(Default)]
    pub struct FakeBus {
        pub tmp102: bool,
        pub bh1750: bool,
        pub opt3001: bool,
        /// Raw TMP102 temperature register.
        pub tmp102_raw: [u8; 2],
        pub tmp102_config: [u8; 2],
        pub bh1750_raw: u16,
        pub opt3001_result: u16,
        pub opt3001_config: u16,
        pub fail_reads: bool,
        pub log: Vec<(u8, Vec<u8>)>,
        pointer: u8,
    }

    impl FakeBus {
        pub fn with(tmp102: bool, bh1750: bool, opt3001: bool) -> Self {
            Self {
                tmp102,
                bh1750,
                opt3001,
                tmp102_config: [0x60, 0xA0],
                ..Self::default()
            }
        }

        fn present(&self, address: u8) -> bool {
            match address {
                0x48 => self.tmp102,
                0x23 => self.bh1750,
                0x44 => self.opt3001,
                _ => false,
            }
        }

        pub fn writes_to(&self, address: u8) -> Vec<Vec<u8>> {
            self.log
                .iter()
                .filter(|(a, _)| *a == address)
                .map(|(_, bytes)| bytes.clone())
                .collect()
        }
    }

    impl ErrorType for FakeBus {
        type Error = ErrorKind;
    }

    impl I2c for FakeBus {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            if !self.present(address) {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
            }
            for operation in operations {
                match operation {
                    Operation::Write(bytes) => {
                        self.log.push((address, bytes.to_vec()));
                        match address {
                            0x48 => {
                                self.pointer = bytes[0];
                                if bytes.len() == 3 && bytes[0] == 0x01 {
                                    self.tmp102_config = [bytes[1], bytes[2]];
                                }
                            }
                            0x44 => {
                                self.pointer = bytes[0];
                                if bytes.len() == 3 && bytes[0] == 0x01 {
                                    self.opt3001_config = u16::from_be_bytes([bytes[1], bytes[2]]);
                                }
                            }
                            _ => {}
                        }
                    }
                    Operation::Read(buffer) => {
                        if self.fail_reads {
                            return Err(ErrorKind::Bus);
                        }
                        let value: [u8; 2] = match (address, self.pointer) {
                            (0x48, 0x00) => self.tmp102_raw,
                            (0x48, 0x01) => self.tmp102_config,
                            (0x23, _) => self.bh1750_raw.to_be_bytes(),
                            (0x44, 0x00) => self.opt3001_result.to_be_bytes(),
                            (0x44, 0x01) => self.opt3001_config.to_be_bytes(),
                            (0x44, 0x7E) => 0x5449u16.to_be_bytes(),
                            (0x44, 0x7F) => 0x3001u16.to_be_bytes(),
                            _ => [0, 0],
                        };
                        buffer.copy_from_slice(&value[..buffer.len()]);
                    }
                }
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake_bus::FakeBus;
    use super::*;
    use crate::storage::mock::MockMainFlash;
    use crate::storage::{DETECTION_PARTITION_START, DetectionPartition};

    const COMBINATIONS: [DetectedSensors; 8] = {
        let mut all = [DetectedSensors::NONE; 8];
        let mut bits = 0;
        while bits < 8 {
            all[bits as usize] = match DetectedSensors::from_bits(bits) {
                Some(sensors) => sensors,
                None => panic!(),
            };
            bits += 1;
        }
        all
    };

    fn store(flash: &mut MockMainFlash) -> DetectionStore<DetectionPartition<&mut MockMainFlash>> {
        DetectionStore::new(DetectionPartition::new(flash))
    }

    #[test]
    fn bits_round_trip_and_reject_unknown_bits() {
        for sensors in COMBINATIONS {
            assert_eq!(DetectedSensors::from_bits(sensors.bits()), Some(sensors));
        }
        assert_eq!(DetectedSensors::from_bits(0x08), None);
        assert_eq!(DetectedSensors::from_bits(0xFF), None);
    }

    #[test]
    fn source_selection() {
        let none = DetectedSensors::NONE;
        assert_eq!(none.temperature_source(), TemperatureSource::DieTemperature);
        assert_eq!(none.light_sensor(), None);
        let all = DetectedSensors {
            tmp102: true,
            bh1750: true,
            opt3001: true,
        };
        assert_eq!(all.temperature_source(), TemperatureSource::Tmp102);
        assert_eq!(all.light_sensor(), Some(LightSensor::Opt3001));
        let bh = DetectedSensors {
            bh1750: true,
            ..DetectedSensors::NONE
        };
        assert_eq!(bh.light_sensor(), Some(LightSensor::Bh1750));
        assert!(bh.has_light_sensor());
    }

    #[test]
    fn detect_reports_exactly_the_fitted_sensors_and_powers_them_down() {
        for expected in COMBINATIONS {
            let mut bus = FakeBus::with(expected.tmp102, expected.bh1750, expected.opt3001);
            assert_eq!(detect(&mut bus), expected);
            if expected.tmp102 {
                // Shutdown bit set in the config written back.
                assert_eq!(bus.tmp102_config[0] & 0x01, 0x01);
            }
            if expected.bh1750 {
                assert_eq!(bus.writes_to(0x23), [[0x00]]);
            }
            if expected.opt3001 {
                assert_eq!(bus.opt3001_config & 0x0600, 0);
            }
        }
    }

    #[test]
    fn opt3001_with_a_wrong_id_is_not_detected() {
        // Address 0x44 acknowledges but the ID registers read zero.
        struct WrongIdBus(FakeBus);
        impl embedded_hal::i2c::ErrorType for WrongIdBus {
            type Error = embedded_hal::i2c::ErrorKind;
        }
        impl I2c for WrongIdBus {
            fn transaction(
                &mut self,
                address: u8,
                operations: &mut [embedded_hal::i2c::Operation<'_>],
            ) -> Result<(), Self::Error> {
                self.0.transaction(address, operations)?;
                if address == 0x44 {
                    for operation in operations {
                        if let embedded_hal::i2c::Operation::Read(buffer) = operation {
                            buffer.fill(0);
                        }
                    }
                }
                Ok(())
            }
        }
        let mut bus = WrongIdBus(FakeBus::with(false, false, true));
        assert_eq!(detect(&mut bus), DetectedSensors::NONE);
    }

    #[test]
    fn record_round_trips_every_combination_through_flash() {
        for sensors in COMBINATIONS {
            let mut flash = MockMainFlash::new();
            store(&mut flash).save(sensors).unwrap();
            assert_eq!(store(&mut flash).load(), Some(sensors));
            // Only the detection sector was touched.
            let start = DETECTION_PARTITION_START as usize;
            assert!(flash.bytes[..start].iter().all(|b| *b == 0xFF));
            assert!(flash.bytes[start + RECORD_LEN..].iter().all(|b| *b == 0xFF));
        }
    }

    #[test]
    fn erased_flash_has_no_record() {
        let mut flash = MockMainFlash::new();
        assert_eq!(store(&mut flash).load(), None);
    }

    #[test]
    fn corrupt_records_are_rejected() {
        let good = encode_record(DetectedSensors {
            tmp102: true,
            ..DetectedSensors::NONE
        });
        assert!(decode_record(&good).is_some());
        for (index, flip) in [(0, 0x01), (4, 0x02), (5, 0x02), (6, 0x01), (9, 0x80)] {
            let mut bad = good;
            bad[index] ^= flip;
            assert_eq!(decode_record(&bad), None, "byte {index}");
        }
        // Torn write: second half still erased.
        let mut torn = good;
        torn[8..].fill(0xFF);
        assert_eq!(decode_record(&torn), None);
        // A valid CRC over unknown sensor bits is rejected too.
        let mut unknown = [0xFF; RECORD_LEN];
        unknown[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
        unknown[4] = RECORD_VERSION;
        unknown[5] = 0x08;
        unknown[6] = !0x08;
        let crc = crc32(&unknown[0..8]);
        unknown[8..12].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_record(&unknown), None);
    }

    #[test]
    fn crc32_matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn commissioned_boot_reuses_the_record_without_probing() {
        let persisted = DetectedSensors {
            bh1750: true,
            ..DetectedSensors::NONE
        };
        let mut flash = MockMainFlash::new();
        store(&mut flash).save(persisted).unwrap();
        let erases = flash.erased_physical_sectors;
        let (sensors, origin) = resolve_at_boot(&mut store(&mut flash), true, || {
            panic!("a commissioned boot must not probe the bus")
        });
        assert_eq!((sensors, origin), (persisted, DetectionOrigin::Persisted));
        assert_eq!(flash.erased_physical_sectors, erases);
    }

    #[test]
    fn factory_new_boot_redetects_and_persists_only_changes() {
        let mut flash = MockMainFlash::new();
        let first = DetectedSensors {
            tmp102: true,
            opt3001: true,
            ..DetectedSensors::NONE
        };
        assert_eq!(
            resolve_at_boot(&mut store(&mut flash), false, || first),
            (first, DetectionOrigin::DetectedSaved)
        );
        let erases = flash.erased_physical_sectors;
        assert_eq!(
            resolve_at_boot(&mut store(&mut flash), false, || first),
            (first, DetectionOrigin::DetectedUnchanged)
        );
        assert_eq!(flash.erased_physical_sectors, erases);

        // Sensor removed while factory-new (after leave): new set wins.
        let second = DetectedSensors {
            tmp102: true,
            ..DetectedSensors::NONE
        };
        assert_eq!(
            resolve_at_boot(&mut store(&mut flash), false, || second),
            (second, DetectionOrigin::DetectedSaved)
        );
        // A following commissioned boot sees the latest set.
        assert_eq!(
            resolve_at_boot(&mut store(&mut flash), true, || unreachable!()),
            (second, DetectionOrigin::Persisted)
        );
    }

    #[test]
    fn commissioned_boot_without_a_valid_record_detects_and_saves() {
        let mut flash = MockMainFlash::new();
        let detected = DetectedSensors {
            bh1750: true,
            ..DetectedSensors::NONE
        };
        assert_eq!(
            resolve_at_boot(&mut store(&mut flash), true, || detected),
            (detected, DetectionOrigin::DetectedSaved)
        );
    }

    #[test]
    fn interrupted_save_reads_as_absent() {
        let mut flash = MockMainFlash::new();
        store(&mut flash)
            .save(DetectedSensors {
                tmp102: true,
                ..DetectedSensors::NONE
            })
            .unwrap();
        // Power lost after the erase of the next save.
        let start = DETECTION_PARTITION_START as usize;
        flash.bytes[start..start + 2048].fill(0xFF);
        assert_eq!(store(&mut flash).load(), None);
    }
}
