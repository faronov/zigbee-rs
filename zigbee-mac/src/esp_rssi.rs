//! Received-frame RSSI of the ESP32-C6/H2/C5 802.15.4 radio, corrected the
//! way ESP-IDF corrects it.
//!
//! The radio overwrites the first FCS octet of every received frame with a
//! signed RSSI byte. On the ESP32-H2 that byte is not the received power:
//! ESP-IDF v5.5.1 adds `IEEE802154_RSSI_COMPENSATION_VALUE` to it before any
//! upper layer sees it (`esp_ieee802154_dev.c`
//! `ieee802154_rx_frame_info_update`: `rssi + IEEE802154_RSSI_COMPENSATION_VALUE`,
//! used by OpenThread, the Zigbee SDK and `esp_ieee802154_get_recent_rssi`).
//! The value per chip (`hal/<chip>/include/hal/ieee802154_ll.h`):
//!
//! | chip | compensation |
//! |---|---|
//! | ESP32-H2 | `bt_bb_get_rssi_comp()` from the PHY library `libbtbb` |
//! | ESP32-C6, ESP32-C5 | `0` |
//!
//! `bt_bb_get_rssi_comp()` returns `(BLE_RSSI_COMP + 1) >> 1`. In the
//! `libbtbb` that esp-radio 0.17 links (esp-wifi-sys 0.8.1, the same
//! "Aug 1 2025" build as ESP-IDF v5.5.1) `BLE_RSSI_COMP` starts at 19, i.e.
//! 10 dB, and `bt_bb_v2_init_cmplx` (run by esp-radio's
//! `esp_ieee802154_enable`, after the PHY's `rf_init`) clears it when
//! `libphy`'s `get_chip_version` found version 5 (modem register
//! `0x600A_0BFC` reads `0x0240_5170`; `0x0230_8112` gives 3, anything else
//! 2). Other H2 parts, such as the v0.1 on the Thread BR board, under-report
//! every frame by 10 dB unless the driver adds it. The value is read from
//! the library at run time, so a chip the library does not compensate gets
//! 0.
//!
//! The energy-detect result is not compensated: ESP-IDF reports
//! `ieee802154_ll_get_ed_rss()` unchanged too.

/// Largest compensation accepted from the PHY library, in dB. The library
/// computes it from a 16-bit table entry; anything above this is not a
/// value Espressif ships and is rejected instead of shifting every link
/// cost.
pub(crate) const MAX_RSSI_COMPENSATION_DB: u32 = 20;

/// The compensation to apply for a value returned by the PHY library
/// (`bt_bb_get_rssi_comp()`), `None` when it is out of range.
pub(crate) const fn compensation_db(library_value: u32) -> Option<i8> {
    if library_value <= MAX_RSSI_COMPENSATION_DB {
        Some(library_value as i8)
    } else {
        None
    }
}

/// RSSI of a received frame in dBm: the radio's raw byte (written over the
/// first FCS octet, signed) plus the chip's compensation, saturating.
pub(crate) const fn frame_rssi_dbm(raw: u8, compensation_db: i8) -> i8 {
    (raw as i8).saturating_add(compensation_db)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn library_values_in_range_are_used_as_is() {
        // libbtbb "Aug 1 2025": BLE_RSSI_COMP = 19 -> (19 + 1) >> 1 = 10.
        assert_eq!(compensation_db(10), Some(10));
        // Chips whose PHY version is above 4: the library clears it.
        assert_eq!(compensation_db(0), Some(0));
        assert_eq!(compensation_db(MAX_RSSI_COMPENSATION_DB), Some(20));
    }

    #[test]
    fn out_of_range_library_values_are_rejected() {
        assert_eq!(compensation_db(MAX_RSSI_COMPENSATION_DB + 1), None);
        assert_eq!(compensation_db(0x8000), None);
        assert_eq!(compensation_db(u32::MAX), None);
    }

    #[test]
    fn raw_byte_is_signed_and_compensated() {
        // Thread BR board H2 v0.1, hwlab 2026-11-27: a beacon sent at
        // 0 dBm by the nRF52840 radio lab read 0xC7 (-57 dBm) raw.
        assert_eq!(frame_rssi_dbm(0xC7, 10), -47);
        assert_eq!(frame_rssi_dbm(0xC7, 0), -57);
        assert_eq!(frame_rssi_dbm(0x00, 10), 10);
        assert_eq!(frame_rssi_dbm(0x80, 0), -128);
    }

    #[test]
    fn compensation_saturates_instead_of_wrapping() {
        assert_eq!(frame_rssi_dbm(0x7F, 10), i8::MAX);
        assert_eq!(frame_rssi_dbm(0x7A, 10), i8::MAX);
        assert_eq!(frame_rssi_dbm(0x80, 10), -118);
    }

    #[test]
    fn compensated_rssi_moves_the_stack_lqi_accordingly() {
        // -57 dBm raw -> LQI 137 (what the active scan reported before),
        // -47 dBm compensated -> LQI 168.
        assert_eq!(crate::lqi::from_rssi_dbm(frame_rssi_dbm(0xC7, 0)), 137);
        assert_eq!(crate::lqi::from_rssi_dbm(frame_rssi_dbm(0xC7, 10)), 168);
    }
}
