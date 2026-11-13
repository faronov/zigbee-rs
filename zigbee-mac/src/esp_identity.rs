//! ESP32-C6/H2 factory EUI-64, derived exactly like ESP-IDF's
//! `esp_read_mac(mac, ESP_MAC_IEEE802154)` (`esp_hw_support/mac_addr.c`),
//! which `esp-zigbee-sdk` uses as the device's IEEE address:
//!
//! ```text
//! base MAC   60:55:F9:F6:96:88          (eFuse BLK1 MAC, bits 47..0)
//! MAC_EXT          xx:yy                (eFuse BLK1 bits 63..56, 55..48)
//! EUI-64     60:55:F9:xx:yy:F6:96:88    (OUI first, no U/L bit change)
//! ```
//!
//! ESP-IDF inserts `MAC_EXT` verbatim. Espressif documents it as `ff:fe` by
//! default, but parts with an unprogrammed field read `00:00` (the C6 lab
//! board does), and ESP-IDF then reports `60:55:f9:00:00:f6:96:88`. This
//! module copies that behaviour instead of inventing a substitute, so a
//! zigbee-rs image and an esp-zigbee-sdk image present the same identity.
//!
//! Those are display (most-significant-octet-first) bytes. The stack's
//! [`IeeeAddress`] holds the over-the-air order instead: IEEE 802.15.4
//! transmits extended addresses least-significant octet first, so the array
//! is the display order reversed and Zigbee coordinators show the OUI first.

use zigbee_types::IeeeAddress;

/// Build the over-the-air EUI-64 from the eFuse `MAC_SYS_0`/`MAC_SYS_1`
/// shadow words.
///
/// `MAC_SYS_0` holds base-MAC display octets 2..5 (bits 31..0),
/// `MAC_SYS_1[15:0]` display octets 0..1 and `MAC_SYS_1[31:16]` the two
/// `MAC_EXT` bytes ESP-IDF inserts after the OUI, most-significant byte
/// first (`esp_efuse_table.csv` lists BLK1 bits 56..63 before 48..55).
pub(crate) const fn ieee_from_efuse_mac_words(mac_sys_0: u32, mac_sys_1: u32) -> IeeeAddress {
    let lo = mac_sys_0.to_be_bytes();
    let hi = mac_sys_1.to_le_bytes();
    // Display order: hi[1] hi[0] lo[0] | hi[3] hi[2] | lo[1] lo[2] lo[3];
    // the over-the-air array is that sequence reversed.
    [lo[3], lo[2], lo[1], hi[2], hi[3], lo[0], hi[0], hi[1]]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Display form of an over-the-air EUI-64, as ZHA prints it.
    fn display(ieee: IeeeAddress) -> u64 {
        u64::from_le_bytes(ieee)
    }

    #[test]
    fn esp32c6_lab_board_matches_esp_idf_with_oui_first() {
        // EFUSE_RD_MAC_SYS_0/1 read back from the C6 lab board over JTAG:
        // bytes 88 96 f6 f9 55 60 00 00 (base MAC 60:55:F9:F6:96:88,
        // MAC_EXT unprogrammed).
        let ieee = ieee_from_efuse_mac_words(0xF9F6_9688, 0x0000_6055);
        // ESP-IDF esp_read_mac(ESP_MAC_IEEE802154): 60:55:f9:00:00:f6:96:88.
        assert_eq!(display(ieee), 0x6055_F900_00F6_9688);
        // Over the air the least-significant octet goes first.
        assert_eq!(ieee, [0x88, 0x96, 0xF6, 0x00, 0x00, 0xF9, 0x55, 0x60]);
        // The pre-fix derivation showed 88:96:f6:fe:ff:f9:55:62 in ZHA:
        // byte-reversed, U/L bit flipped and a hard-coded ff:fe.
        assert_ne!(display(ieee), 0x8896_F6FE_FFF9_5562);
    }

    #[test]
    fn factory_mac_ext_ff_fe_follows_the_oui() {
        // ESP32-H2 base MAC 10:51:DB:63:B8:CD with MAC_EXT ff:fe burned:
        // BLK1 bits 63..56 = 0xFF, bits 55..48 = 0xFE.
        let ieee = ieee_from_efuse_mac_words(0xDB63_B8CD, 0xFFFE_1051);
        assert_eq!(display(ieee), 0x1051_DBFF_FE63_B8CD);
        assert_eq!(ieee, [0xCD, 0xB8, 0x63, 0xFE, 0xFF, 0xDB, 0x51, 0x10]);
    }

    #[test]
    fn mac_ext_bytes_are_taken_from_efuse_most_significant_first() {
        let ieee = ieee_from_efuse_mac_words(0x4455_6677, 0xBBAA_1122);
        assert_eq!(display(ieee), 0x1122_44BB_AA55_6677);
    }
}
