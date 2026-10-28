//! Boot-time resolution of the sensor set that defines the endpoint.
//!
//! This runs before the Zigbee device is built, because the Simple
//! Descriptor is fixed at build time. It reads the security journal through
//! a temporary borrowed view to learn whether this boot resumes a
//! commissioned network, then applies [`resolve_at_boot`].

use embedded_hal::i2c::I2c;
use embedded_storage::nor_flash::{NorFlash, NorFlashError};
use zigbee_runtime::security_store::{
    PersistentSecurityState, SecurityStateStore, SecurityStoreError,
};

use crate::detection::{DetectedSensors, DetectionOrigin, DetectionStore, detect, resolve_at_boot};
use crate::storage::{DetectionPartition, security_journal};

/// Whether the persisted security state will resume a network for this
/// device identity. Mirrors
/// `ZigbeeDevice::reset_security_state_if_identity_changed`: a state
/// recorded for another EUI-64 is about to be cleared, so the boot is
/// effectively factory-new. An unreadable journal also counts as not
/// commissioned (the lifecycle then reports the security failure).
pub fn resumes_network(
    state: Result<Option<PersistentSecurityState>, SecurityStoreError>,
    ieee: [u8; 8],
) -> bool {
    matches!(
        state,
        Ok(Some(state))
            if state.commissioned
                && (state.ieee_address == [0; 8] || state.ieee_address == ieee)
    )
}

/// Resolve this boot's sensor set from `flash` (the whole main flash; only
/// the security journal and detection partitions are accessed) and, when
/// the device is factory-new, by probing `bus`.
pub fn resolve_sensors<F, I2C>(
    flash: &mut F,
    ieee: [u8; 8],
    bus: &mut I2C,
) -> (DetectedSensors, DetectionOrigin)
where
    F: NorFlash,
    F::Error: NorFlashError,
    I2C: I2c,
{
    let commissioned = resumes_network(security_journal(&mut *flash).load(), ieee);
    let mut store = DetectionStore::new(DetectionPartition::new(&mut *flash));
    resolve_at_boot(&mut store, commissioned, || detect(bus))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detection::fake_bus::FakeBus;
    use crate::storage::mock::MockMainFlash;

    const IEEE: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

    /// A minimal record that passes the journal's validation: joined with
    /// the legacy default Trust Center link key.
    fn commissioned_state(ieee: [u8; 8]) -> PersistentSecurityState {
        PersistentSecurityState {
            commissioned: true,
            ieee_address: ieee,
            channel: 15,
            pan_id: 0x1A62,
            short_address: 0x4C21,
            global_counter_limit: 0x1000,
            legacy_default_tclk: true,
            tclk_counter_limit: 0x1000,
            ..PersistentSecurityState::default()
        }
    }

    fn commission(flash: &mut MockMainFlash, ieee: [u8; 8]) {
        security_journal(flash)
            .store(&commissioned_state(ieee))
            .unwrap();
    }

    #[test]
    fn factory_new_boot_probes_and_persists() {
        let mut flash = MockMainFlash::new();
        let mut bus = FakeBus::with(true, false, true);
        let (sensors, origin) = resolve_sensors(&mut flash, IEEE, &mut bus);
        assert_eq!(
            sensors,
            DetectedSensors {
                tmp102: true,
                bh1750: false,
                opt3001: true,
            }
        );
        assert_eq!(origin, DetectionOrigin::DetectedSaved);
        assert!(!bus.log.is_empty());
    }

    #[test]
    fn commissioned_boot_keeps_the_descriptor_even_if_a_sensor_vanishes() {
        let mut flash = MockMainFlash::new();
        let mut bus = FakeBus::with(false, true, false);
        resolve_sensors(&mut flash, IEEE, &mut bus);
        commission(&mut flash, IEEE);

        // The BH1750 stops answering; a commissioned boot does not probe.
        let mut silent = FakeBus::with(false, false, false);
        let (sensors, origin) = resolve_sensors(&mut flash, IEEE, &mut silent);
        assert_eq!(origin, DetectionOrigin::Persisted);
        assert!(sensors.bh1750);
        assert!(silent.log.is_empty());
    }

    #[test]
    fn state_of_another_identity_or_after_leave_redetects() {
        let mut flash = MockMainFlash::new();
        resolve_sensors(&mut flash, IEEE, &mut FakeBus::with(false, true, false));
        commission(&mut flash, [9; 8]);
        let (sensors, origin) =
            resolve_sensors(&mut flash, IEEE, &mut FakeBus::with(true, false, false));
        assert_eq!(origin, DetectionOrigin::DetectedSaved);
        assert!(sensors.tmp102 && !sensors.bh1750);

        // Leave: the runtime persists a not-commissioned state.
        security_journal(&mut flash)
            .store(&PersistentSecurityState {
                commissioned: false,
                ieee_address: IEEE,
                global_counter_limit: 0x2000,
                ..PersistentSecurityState::default()
            })
            .unwrap();
        let (sensors, origin) = resolve_sensors(&mut flash, IEEE, &mut FakeBus::default());
        assert_eq!(origin, DetectionOrigin::DetectedSaved);
        assert_eq!(sensors, DetectedSensors::NONE);
    }

    #[test]
    fn resumes_network_rules() {
        assert!(resumes_network(Ok(Some(commissioned_state(IEEE))), IEEE));
        assert!(!resumes_network(Ok(Some(commissioned_state([9; 8]))), IEEE));
        let unbound = PersistentSecurityState {
            ieee_address: [0; 8],
            ..commissioned_state(IEEE)
        };
        assert!(resumes_network(Ok(Some(unbound)), IEEE));
        let left = PersistentSecurityState {
            commissioned: false,
            ..commissioned_state(IEEE)
        };
        assert!(!resumes_network(Ok(Some(left)), IEEE));
        assert!(!resumes_network(Ok(None), IEEE));
        assert!(!resumes_network(Err(SecurityStoreError::Corrupt), IEEE));
    }
}
