//! Illuminance Measurement cluster (0x0400).

use crate::attribute::{AttributeAccess, AttributeDefinition, AttributeStore};
use crate::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use crate::data_types::{ZclDataType, ZclValue};
use crate::{AttributeId, ClusterId, CommandId, ZclStatus};

// Attribute IDs — MeasuredValue = 10000 × log10(lux) + 1.
pub const ATTR_MEASURED_VALUE: AttributeId = AttributeId(0x0000);
pub const ATTR_MIN_MEASURED_VALUE: AttributeId = AttributeId(0x0001);
pub const ATTR_MAX_MEASURED_VALUE: AttributeId = AttributeId(0x0002);
pub const ATTR_TOLERANCE: AttributeId = AttributeId(0x0003);
pub const ATTR_LIGHT_SENSOR_TYPE: AttributeId = AttributeId(0x0004);

/// `MeasuredValue` meaning "too low to be measured" (ZCL §4.2.2.2.1).
pub const MEASURED_VALUE_TOO_LOW: u16 = 0x0000;
/// Largest valid `MeasuredValue`; 0xFFFF means "invalid".
pub const MEASURED_VALUE_MAX: u16 = 0xFFFE;

/// Encodes an illuminance given in centi-lux (hundredths of a lux) as the
/// ZCL `MeasuredValue`, `10000 x log10(lux) + 1`, rounded to the nearest
/// step.
///
/// Uses an integer binary-logarithm (16 fractional bits, repeated
/// squaring) so it needs neither floating point nor `libm`. The result is
/// within one step of the exact value across the whole `u32` input range.
/// Zero lux encodes as [`MEASURED_VALUE_TOO_LOW`]; anything above zero but
/// below 1 lx clamps to 1 (the smallest representable illuminance), and
/// values beyond the encodable range clamp to [`MEASURED_VALUE_MAX`].
pub fn measured_value_from_centi_lux(centi_lux: u32) -> u16 {
    if centi_lux == 0 {
        return MEASURED_VALUE_TOO_LOW;
    }
    if centi_lux < 100 {
        return 1;
    }

    // log2(centi_lux) as Q16: integer part from the MSB position, fraction
    // from 16 rounds of squaring the normalised mantissa (Q1.31 in [1, 2)).
    let integer = 31 - centi_lux.leading_zeros();
    let mut mantissa = (u64::from(centi_lux) << 31) >> integer;
    let mut fraction = 0u32;
    for bit in (0..16).rev() {
        mantissa = (mantissa * mantissa) >> 31;
        if mantissa >= 1 << 32 {
            fraction |= 1 << bit;
            mantissa >>= 1;
        }
    }
    let log2_q16 = u64::from((integer << 16) | fraction);

    // 10000 x log10(centi_lux) = log2 x 10000 x log10(2); 10000 x log10(2)
    // is 3010.299957, carried here as 3_010_299_957 / 1e6.
    const DENOMINATOR: u64 = 65_536 * 1_000_000;
    let scaled = (log2_q16 * 3_010_299_957 + DENOMINATOR / 2) / DENOMINATOR;
    // lux = centi_lux / 100 subtracts 20000; the encoding adds 1.
    let value = scaled.saturating_sub(19_999);
    value.clamp(1, u64::from(MEASURED_VALUE_MAX)) as u16
}

/// Illuminance Measurement cluster.
pub struct IlluminanceCluster {
    store: AttributeStore<8>,
}

impl IlluminanceCluster {
    pub fn new(min: u16, max: u16) -> Self {
        let mut store = AttributeStore::new();
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::Reportable,
                name: "MeasuredValue",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MIN_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MinMeasuredValue",
            },
            ZclValue::U16(min),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_MAX_MEASURED_VALUE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "MaxMeasuredValue",
            },
            ZclValue::U16(max),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_TOLERANCE,
                data_type: ZclDataType::U16,
                access: AttributeAccess::ReadOnly,
                name: "Tolerance",
            },
            ZclValue::U16(0),
        );
        let _ = store.register(
            AttributeDefinition {
                id: ATTR_LIGHT_SENSOR_TYPE,
                data_type: ZclDataType::Enum8,
                access: AttributeAccess::ReadOnly,
                name: "LightSensorType",
            },
            ZclValue::Enum8(0xFF), // Unknown
        );
        Self { store }
    }

    /// Update the measured illuminance (in 10000 × log10(lux) + 1 units).
    /// See [`measured_value_from_centi_lux`] for the encoding.
    pub fn set_illuminance(&mut self, value: u16) {
        let _ = self
            .store
            .set_raw(ATTR_MEASURED_VALUE, ZclValue::U16(value));
    }
}

impl Cluster for IlluminanceCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::ILLUMINANCE
    }
    fn handle_command(
        &mut self,
        _cmd_id: CommandId,
        _payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        Err(ZclStatus::UnsupClusterCommand)
    }
    fn attributes(&self) -> &dyn AttributeStoreAccess {
        &self.store
    }
    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        &mut self.store
    }

    /// No writable attributes: `MeasuredValue` is a live sensor reading fed
    /// by the driver, and the remaining attributes are fixed physical-range
    /// configuration supplied at construction.
    fn reset_to_factory_defaults(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centi_lux_encoding_matches_exact_log10() {
        // Expected values: round(10000 * log10(centi_lux / 100) + 1),
        // computed with double-precision log10.
        const CASES: &[(u32, u16)] = &[
            (100, 1),
            (101, 44),
            (150, 1_762),
            (250, 3_980),
            (1_000, 10_001),
            (10_000, 20_001),
            (12_345, 20_916),
            (50_000, 26_991),
            (100_000, 30_001),
            (1_000_000, 40_001),
            (7_777_777, 48_910),
            (36_000_000, 55_564),
        ];
        for &(centi_lux, expected) in CASES {
            let encoded = measured_value_from_centi_lux(centi_lux);
            assert!(
                encoded.abs_diff(expected) <= 1,
                "{centi_lux} cLx encoded as {encoded}, expected {expected}"
            );
        }
    }

    #[test]
    fn decades_encode_exactly() {
        assert_eq!(measured_value_from_centi_lux(100), 1);
        assert_eq!(measured_value_from_centi_lux(1_000), 10_001);
        assert_eq!(measured_value_from_centi_lux(10_000), 20_001);
        assert_eq!(measured_value_from_centi_lux(100_000), 30_001);
        assert_eq!(measured_value_from_centi_lux(1_000_000), 40_001);
        assert_eq!(measured_value_from_centi_lux(10_000_000), 50_001);
    }

    #[test]
    fn out_of_range_inputs_clamp() {
        assert_eq!(measured_value_from_centi_lux(0), MEASURED_VALUE_TOO_LOW);
        assert_eq!(measured_value_from_centi_lux(1), 1);
        assert_eq!(measured_value_from_centi_lux(99), 1);
        assert_eq!(measured_value_from_centi_lux(400_000_000), 65_534);
        assert_eq!(measured_value_from_centi_lux(u32::MAX), MEASURED_VALUE_MAX);
    }

    #[test]
    fn encoding_is_monotonic() {
        let mut previous = 0;
        let mut centi_lux = 1u32;
        while centi_lux < u32::MAX / 2 {
            let encoded = measured_value_from_centi_lux(centi_lux);
            assert!(encoded >= previous, "not monotonic at {centi_lux}");
            previous = encoded;
            centi_lux = centi_lux + centi_lux / 7 + 1;
        }
    }
}
