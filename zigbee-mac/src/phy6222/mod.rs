//! PHY6222/PHY6252 radio backend — pure Rust, zero vendor blobs.
//!
//! The 802.15.4 MAC itself is the shared [`crate::SoftMacCore`]; this module
//! provides only the PHY62x2-specific pieces underneath it:
//!
//! - [`Phy6222RadioPhy`]: the thin [`crate::RadioPhy`] +
//!   [`crate::PlatformServices`] adapter over the register-level [`driver`];
//! - [`Phy6222SoftMac`]: the end-device `SoftMacCore` alias used by the
//!   PHY62x2 examples;
//! - the factory EUI-64 (chip MADDR or the `OWN_PUBLIC_ADDR_LOC` record).
//!
//! All radio configuration uses direct register access in [`driver`]; the
//! register values are transcribed from the PHYPLUS PHY62xx SDK
//! (`rf_phy_driver.c`, `ll_hw_drv.c`) as redistributed by pvvx (THB2 and
//! phy62x2-SDK). No vendor libraries or ROM radio routines are linked.
//!
//! # Division of work
//! - `SoftMacCore`: scan, association, polling, unslotted CSMA-CA backoff,
//!   `macMaxFrameRetries`, address filtering of received frames.
//! - [`Phy6222RadioPhy`] ([`TxOffload::AckWait`]): one CCA + TX, and for
//!   frames requesting an ACK the LL engine's TRX operation, which turns
//!   the radio around in hardware and receives the Imm-Ack.
//! - [`driver`]'s SRAM-resident `LL_IRQ`: the Imm-Ack for received frames
//!   that request one and name this node exactly (`hardware_auto_ack`),
//!   161–192 µs after the frame (sniffer, PB-03F). The LL engine has no 802.15.4 ACK support, so
//!   the interrupt starts each ACK itself with a tuned TX settle delay. The
//!   frame-pending bit is always clear (end device only).

pub mod driver;

use crate::pib::MacPib;
use crate::{
    AckResult, MacError, PhyAddressFilter, PhyCapabilities, PhyError, PhyRxFrame, PlatformServices,
    RadioPhy, SoftMacCore, TxOffload,
};
use driver::{Phy6222Driver, RadioConfig, RadioError};
use embassy_time::{Duration, Instant, Timer};
use zigbee_types::IeeeAddress;

/// PHY62x2 radio exposed through the portable PHY contract.
pub struct Phy6222RadioPhy {
    driver: Phy6222Driver,
}

/// Shared software MAC (end-device role) over the PHY62x2 radio.
///
/// Construct with [`Phy6222RadioPhy::into_soft_mac`].
pub type Phy6222SoftMac = SoftMacCore<Phy6222RadioPhy>;

impl Phy6222RadioPhy {
    /// Acquire the single radio with the default configuration (48 MHz
    /// HCLK, QFN32 PA table).
    pub fn take() -> Option<Self> {
        Self::take_with_config(RadioConfig::default())
    }

    /// Acquire the single radio with a product-selected configuration
    /// (HCLK, PA table, FCS mode, TX power).
    ///
    /// The extended address is always the factory EUI-64; `LL_IRQ`
    /// acknowledges frames for it from the first receive on.
    pub fn take_with_config(config: RadioConfig) -> Option<Self> {
        let ieee = Self::read_factory_ieee();
        let driver = Phy6222Driver::take(RadioConfig {
            extended_address: ieee,
            ..config
        })?;
        log::info!(
            "[MAC] IEEE: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            ieee[0],
            ieee[1],
            ieee[2],
            ieee[3],
            ieee[4],
            ieee[5],
            ieee[6],
            ieee[7]
        );
        Some(Self { driver })
    }

    /// Factory EUI-64 in over-the-air (little-endian) order.
    pub fn extended_address(&self) -> IeeeAddress {
        self.driver.config().extended_address
    }

    /// Build the end-device [`Phy6222SoftMac`] on top of this radio.
    ///
    /// `macDSN`/`macBSN` start at 0 as in the legacy backend (a cold-start
    /// MLME-RESET with SetDefaultPIB reseeds them from the core's
    /// generator). From here on the PIB owns the channel and
    /// `phyTransmitPower` (0 dBm by default), so `RadioConfig::tx_power`
    /// only applies until the MAC takes over.
    #[inline(never)]
    pub fn into_soft_mac(self) -> Result<Phy6222SoftMac, MacError> {
        let pib = MacPib::new(self.extended_address(), 0, 0);
        SoftMacCore::new(self, pib)
    }

    /// Radio counters and calibration results, for bring-up diagnostics.
    pub fn radio_stats(&self) -> driver::RadioStats {
        self.driver.stats()
    }

    /// Current low-level radio configuration.
    pub fn radio_config(&self) -> &RadioConfig {
        self.driver.config()
    }

    /// Adjust low-level radio settings (e.g. FCS mode for lab A/B tests).
    pub fn update_radio_config(&mut self, update_fn: impl FnOnce(&mut RadioConfig)) {
        self.driver.update_config(update_fn);
    }

    /// One CCA (mode 1, energy above threshold) on the current channel.
    fn clear_channel(&mut self) -> Result<(), PhyError> {
        if self.driver.channel_busy().map_err(map_radio_error)? {
            Err(PhyError::ChannelBusy)
        } else {
            Ok(())
        }
    }

    /// Read the factory-programmed 6-byte BLE public address and convert it
    /// to the 8-byte IEEE 802.15.4 EUI-64 used by the MAC.
    ///
    /// Sources, in the vendor SDK's priority order:
    /// 1. the one-bit-hot chip MADDR words at `0x11000900` (SDK `flash.c`
    ///    `read_chip_mAddr`; `phy_sec_ext.c` `pplus_LoadMACFromChipMAddr`
    ///    lets it override the flash record);
    /// 2. the public-address record at flash `0x4000` (`OWN_PUBLIC_ADDR_LOC`,
    ///    SDK `patch.c:6348`; byte order from `ll.c` `LL_Init`). Ai-Thinker
    ///    PB-03F modules ship with only this record.
    ///
    /// Returns the address in IEEE 802.15.4 over-the-air (little-endian)
    /// order, like every other `IeeeAddress` in the stack. Falls back to a
    /// shared placeholder that production products must reject.
    fn read_factory_ieee() -> [u8; 8] {
        /// Chip MADDR: 6 one-bit-hot words after the 64-word chip ID.
        const CHIP_MADDR_ADDR: u32 = 0x1100_0900;
        /// `OWN_PUBLIC_ADDR_LOC` (XIP address of flash offset 0x4000).
        const OWN_PUBLIC_ADDR_LOC: u32 = 0x1100_4000;

        let mut words = [0u32; 6];
        for (i, word) in words.iter_mut().enumerate() {
            // SAFETY: XIP flash is always mapped and readable.
            *word =
                unsafe { core::ptr::read_volatile((CHIP_MADDR_ADDR + i as u32 * 4) as *const u32) };
        }
        let mut record = [0u8; 6];
        for (i, byte) in record.iter_mut().enumerate() {
            // SAFETY: as above.
            *byte =
                unsafe { core::ptr::read_volatile((OWN_PUBLIC_ADDR_LOC + i as u32) as *const u8) };
        }

        match chip_maddr_ble_address(&words).or_else(|| public_addr_record_ble_address(record)) {
            Some(mac) => ble_address_to_eui64(mac),
            None => {
                log::warn!("phy6222: no factory MAC — using fallback");
                [0x00, 0x0D, 0x6F, 0xFF, 0xFE, 0xDE, 0xAD, 0x01]
            }
        }
    }
}

impl RadioPhy for Phy6222RadioPhy {
    /// CCA + TX + the LL engine's TRX ACK window run in one driver call.
    const TX_OFFLOAD: TxOffload = TxOffload::AckWait;
    /// Hardware-proven legacy `Phy6222Mac` value: on a busy channel the
    /// parent's indirect transmission competes with other traffic.
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    /// Legacy `Phy6222Mac` behaviour: a short-source poll that returns
    /// nothing is repeated with the EUI-64 to fetch frames the parent
    /// queued by extended address (Transport-Key, Rejoin Response).
    const POLL_EXTENDED_FALLBACK: bool = true;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            // `LL_IRQ` acknowledges exact-destination frames itself.
            hardware_auto_ack: true,
            // Every frame is delivered; `SoftMacCore` filters them.
            hardware_address_filter: false,
            // Range of both package PA tables (`driver::PaTable`).
            tx_power_min: -20,
            tx_power_max: 5,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.clear_channel()?;
        self.driver
            .transmit_blocking(frame)
            .map_err(map_radio_error)
    }

    async fn try_transmit_acknowledged(
        &mut self,
        frame: &[u8],
    ) -> Result<Option<AckResult>, PhyError> {
        self.clear_channel()?;
        self.driver
            .transmit_wait_ack(frame)
            .map(|ack| ack.map(|frame_pending| AckResult { frame_pending }))
            .map_err(map_radio_error)
    }

    /// Never called: `LL_IRQ` sends every Imm-Ack (`hardware_auto_ack`).
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32) -> Result<Option<PhyRxFrame>, PhyError> {
        match embassy_time::with_timeout(
            Duration::from_micros(u64::from(timeout_us)),
            self.driver.receive(),
        )
        .await
        {
            Ok(Ok(frame)) => Ok(Some(frame)),
            Ok(Err(error)) => Err(map_radio_error(error)),
            Err(_) => Ok(None),
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        if self.driver.config().channel != channel {
            self.driver.update_config(|c| c.channel = channel);
        }
        Ok(())
    }

    /// The PA table entry closest to `dbm` without exceeding it is used
    /// (the lowest entry below the table).
    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if self.driver.config().tx_power != dbm {
            self.driver.update_config(|c| c.tx_power = dbm);
        }
        Ok(())
    }

    /// One 128 µs RSSI sample, whatever `duration_us` asks for.
    async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
        let (rssi, _) = self
            .driver
            .energy_detect_blocking()
            .map_err(map_radio_error)?;
        Ok(energy_from_rssi(rssi))
    }

    /// Feeds the `LL_IRQ` ACK filter; `None` (scan) keeps the last
    /// addresses, so frames naming this node are still acknowledged.
    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        let config = self.driver.config();
        let unchanged = match &filter {
            Some(f) => {
                !config.promiscuous
                    && config.pan_id == f.pan_id.0
                    && config.short_address == f.short_address.0
                    && config.extended_address == f.extended_address
            }
            None => config.promiscuous,
        };
        if !unchanged {
            self.driver.update_config(|c| {
                c.promiscuous = filter.is_none();
                if let Some(f) = filter {
                    c.pan_id = f.pan_id.0;
                    c.short_address = f.short_address.0;
                    c.extended_address = f.extended_address;
                }
            });
        }
        Ok(())
    }

    /// Stop the LL engine and gate the RF/RX-ADC clocks.
    fn sleep(&mut self) -> Result<(), PhyError> {
        self.driver.radio_sleep();
        Ok(())
    }

    /// Re-enable the clocks and re-run the analog/baseband/LL setup with the
    /// stored calibration results and configuration.
    fn wake(&mut self) -> Result<(), PhyError> {
        self.driver.radio_wake();
        Ok(())
    }
}

impl zigbee_crypto::ForwardAesProvider for Phy6222RadioPhy {}

impl PlatformServices for Phy6222RadioPhy {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

fn map_radio_error(error: RadioError) -> PhyError {
    match error {
        RadioError::CcaFailure => PhyError::ChannelBusy,
        RadioError::InvalidFrame => PhyError::FrameTooLong,
        RadioError::CrcError => PhyError::CrcFailed,
        RadioError::HardwareError | RadioError::NotInitialized | RadioError::RxTimeout => {
            PhyError::Hardware
        }
    }
}

/// RSSI (dBm) → IEEE 802.15.4 ED scale: -100 dBm and below is 0, one step
/// per dB.
fn energy_from_rssi(rssi: i8) -> u8 {
    (i16::from(rssi) + 100).clamp(0, 255) as u8
}

// ── Factory address ─────────────────────────────────────────────

/// Decode the SDK chip MADDR (six one-bit-hot words, most significant
/// byte first) into a little-endian BLE address (`ownPublicAddr` order).
fn chip_maddr_ble_address(words: &[u32; 6]) -> Option<[u8; 6]> {
    let mut mac = [0u8; 6];
    for (i, &word) in words.iter().enumerate() {
        mac[5 - i] = one_bit_hot_decode(word)?;
    }
    valid_ble_address(mac)
}

/// Decode the `OWN_PUBLIC_ADDR_LOC` record into a little-endian BLE address.
/// SDK `LL_Init`: `ownPublicAddr[3,2,1,0,5,4] = flash[0..6]`.
fn public_addr_record_ble_address(b: [u8; 6]) -> Option<[u8; 6]> {
    valid_ble_address([b[3], b[2], b[1], b[0], b[5], b[4]])
}

fn valid_ble_address(mac: [u8; 6]) -> Option<[u8; 6]> {
    (mac != [0xFF; 6] && mac != [0x00; 6]).then_some(mac)
}

/// EUI-48 → EUI-64 (`AA:BB:CC:DD:EE:FF` → `AA:BB:CC:FF:FE:DD:EE:FF`), both
/// in little-endian (over-the-air) byte order.
fn ble_address_to_eui64(m: [u8; 6]) -> [u8; 8] {
    [m[0], m[1], m[2], 0xFE, 0xFF, m[3], m[4], m[5]]
}

/// Decode one chip-MADDR word, as SDK `chip_id_one_bit_hot_convter()`
/// (`lib/sec/phy_sec_ext.c:197`): each 16-bit half must have exactly one
/// bit set; the bit index of the high half is the upper nibble and the bit
/// index of the low half is the lower nibble. Blank (`0xFFFFFFFF`) or
/// malformed words return `None`.
fn one_bit_hot_decode(word: u32) -> Option<u8> {
    let high = (word >> 16) as u16;
    let low = word as u16;
    if high.count_ones() != 1 || low.count_ones() != 1 {
        return None;
    }
    Some(((high.trailing_zeros() as u8) << 4) | low.trailing_zeros() as u8)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::parse_beacon;
    use zigbee_types::{MacAddress, PanId, ShortAddress};

    #[test]
    fn scan_parses_captured_zha_coordinator_beacon() {
        // nRF sniffer capture, channel 15: ZHA coordinator beacon answering
        // a PB-03F beacon request (FCS stripped by the sniffer).
        let beacon = [
            0x00, 0x80, 0x9E, 0xE9, 0xDF, 0x00, 0x00, 0xFF, 0xCF, 0x00, 0x00, 0x00, 0x22, 0x84,
            0xB1, 0x4C, 0xC0, 0x1E, 0xA6, 0xC7, 0xAA, 0x20, 0xFF, 0xFF, 0xFF, 0x00,
        ];
        let pd = parse_beacon(15, &beacon, 200).unwrap();
        assert_eq!(pd.zigbee_beacon.protocol_id, 0);
        assert_eq!(pd.zigbee_beacon.stack_profile, 2);
        assert!(pd.superframe_spec.association_permit);
        assert_eq!(pd.lqi, 200);
        assert_eq!(
            pd.coord_address,
            MacAddress::Short(PanId(0xDFE9), ShortAddress(0x0000))
        );
        // A MAC-only beacon carries no Zigbee payload and is not a network.
        assert!(parse_beacon(15, &beacon[..11], 200).is_none());
    }

    #[test]
    fn public_addr_record_matches_sdk_byte_order() {
        // Ai-Thinker PB-03F factory record at flash 0x4000 (BD_ADDR
        // 24:42:E3:5F:6D:04 per SDK `LL_Init`).
        let mac = public_addr_record_ble_address([0xE3, 0x5F, 0x6D, 0x04, 0x24, 0x42]).unwrap();
        assert_eq!(mac, [0x04, 0x6D, 0x5F, 0xE3, 0x42, 0x24]);
        // EUI-64 24:42:E3:FF:FE:5F:6D:04, little-endian on air.
        assert_eq!(
            ble_address_to_eui64(mac),
            [0x04, 0x6D, 0x5F, 0xFE, 0xFF, 0xE3, 0x42, 0x24]
        );
        assert_eq!(public_addr_record_ble_address([0xFF; 6]), None);
        assert_eq!(public_addr_record_ble_address([0x00; 6]), None);
    }

    #[test]
    fn one_bit_hot_decode_matches_sdk_convter() {
        // byte 0xA5: high half bit 10, low half bit 5.
        assert_eq!(one_bit_hot_decode((1 << (16 + 10)) | (1 << 5)), Some(0xA5));
        assert_eq!(one_bit_hot_decode(0x8000_0001), Some(0xF0));
        assert_eq!(one_bit_hot_decode(0xFFFF_FFFF), None);
        assert_eq!(one_bit_hot_decode(0x0000_0000), None);
        assert_eq!(one_bit_hot_decode(0x0003_0001), None);
    }

    #[test]
    fn chip_maddr_words_are_most_significant_first() {
        let enc = |b: u8| (1u32 << (16 + (b >> 4))) | (1 << (b & 0x0F));
        let words = [
            enc(0x24),
            enc(0x42),
            enc(0xE3),
            enc(0x5F),
            enc(0x6D),
            enc(0x04),
        ];
        assert_eq!(
            chip_maddr_ble_address(&words),
            Some([0x04, 0x6D, 0x5F, 0xE3, 0x42, 0x24])
        );
        let mut partial = words;
        partial[3] = 0xFFFF_FFFF;
        assert_eq!(chip_maddr_ble_address(&partial), None);
    }

    #[test]
    fn energy_scale_is_monotonic_and_bounded() {
        assert_eq!(energy_from_rssi(-127), 0);
        assert_eq!(energy_from_rssi(-100), 0);
        assert_eq!(energy_from_rssi(-60), 40);
        assert_eq!(energy_from_rssi(0), 100);
        assert!(energy_from_rssi(-50) > energy_from_rssi(-80));
    }

    #[test]
    fn radio_errors_map_to_phy_errors() {
        assert_eq!(
            map_radio_error(RadioError::CcaFailure),
            PhyError::ChannelBusy
        );
        assert_eq!(map_radio_error(RadioError::CrcError), PhyError::CrcFailed);
        assert_eq!(
            map_radio_error(RadioError::InvalidFrame),
            PhyError::FrameTooLong
        );
        assert_eq!(
            map_radio_error(RadioError::HardwareError),
            PhyError::Hardware
        );
    }
}
