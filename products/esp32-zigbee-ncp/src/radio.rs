//! Radio facts of the ESP32-H2/C6 802.15.4 block the NCP reports (spec
//! 7.2 `NWK_SET_TX_POWER`, `EVT_ENERGY_SCAN_RESULT`; 7.8 MAC counters).
//! No RF front-end module is fitted, so radio values are antenna values.

/// ESP-IDF v5.5.1 `hal/<chip>/ieee802154_ll.h`: `IEEE802154_TXPOWER_VALUE_MIN`
/// / `_MAX` and `IEEE802154_TXPOWER_INDEX_MIN`. The driver
/// (`esp_ieee802154_pib.c` `ieee802154_txpower_convert`, esp-radio
/// `pib.rs`) programs index `(dbm - MIN) / 3 + INDEX_MIN`, i.e. 3 dB steps
/// from `MIN`, and index 15 (the maximum) at or above `MAX`.
#[cfg(not(feature = "esp32c6"))]
const TX_POWER_RANGE: (i8, i8) = (-24, 20);
#[cfg(feature = "esp32c6")]
const TX_POWER_RANGE: (i8, i8) = (-15, 20);

/// The output for a requested `dbm`: clamped to the range and rounded down
/// to a 3 dB step above the minimum; the maximum itself is exact.
pub const fn tx_power_step(dbm: i8) -> i8 {
    let (min, max) = TX_POWER_RANGE;
    if dbm >= max {
        max
    } else if dbm <= min {
        min
    } else {
        min + (dbm - min) / 3 * 3
    }
}

/// `max_rssi_dbm` of an ED value: zigbee-mac's ESP PHY maps the radio's
/// RSS (dBm) linearly from -100..=+5 onto 0..=255
/// (`zigbee-mac/src/esp/radio_phy.rs` `energy_detect`); this is its
/// inverse, rounded to the nearest dB.
pub const fn energy_to_dbm(energy: u8) -> i8 {
    ((energy as i16 * 105 + 127) / 255 - 100) as i8
}

/// MAC counters (`DIAG_GET_COUNTERS` group bit 1, ids `0x0201..`), in id
/// order: transmissions started (every CSMA-CA attempt), of those with an
/// ACK request, of those without ACK, failed starts (radio busy or no
/// TX-done; retried as a busy channel), deferred starts, child Data
/// Requests ACKed by the radio, of those with Frame Pending, stalled
/// automatic ACKs recovered.
pub type MacCounters = [u32; 8];

#[cfg(test)]
mod tests {
    use super::*;

    /// zigbee-mac's forward mapping (ESP `energy_detect`).
    fn energy(dbm: i8) -> u8 {
        let level = (i16::from(dbm) + 100).clamp(0, 105);
        (level * 255 / 105) as u8
    }

    #[test]
    fn energy_round_trips_through_the_esp_scale() {
        for dbm in -100..=5i8 {
            assert_eq!(energy_to_dbm(energy(dbm)), dbm, "{dbm} dBm");
        }
        assert_eq!(energy_to_dbm(0), -100);
        assert_eq!(energy_to_dbm(255), 5);
    }

    #[test]
    fn tx_power_is_clamped_and_stepped() {
        let (min, max) = TX_POWER_RANGE;
        assert_eq!(tx_power_step(i8::MAX), max);
        assert_eq!(tx_power_step(max), max);
        assert_eq!(tx_power_step(i8::MIN), min);
        assert_eq!(tx_power_step(min + 2), min);
        assert_eq!(tx_power_step(min + 3), min + 3);
        for dbm in min..=max {
            let step = tx_power_step(dbm);
            assert!(step <= dbm && dbm - step < 3, "{dbm} -> {step}");
            assert_eq!(tx_power_step(step), step);
        }
    }

    #[cfg(not(feature = "esp32c6"))]
    #[test]
    fn h2_steps_match_the_driver_index() {
        // index = (dbm + 24) / 3: 10 dBm (the default) is index 11, -24 + 33.
        assert_eq!(tx_power_step(10), 9);
        assert_eq!(tx_power_step(0), 0);
        assert_eq!(tx_power_step(19), 18);
    }
}
