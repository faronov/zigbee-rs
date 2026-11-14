//! Product policy for the BRD4183A EFR32MG22 Green Power Devices.
//!
//! Two images share this product: an environment sensor (GPD DeviceID 0x33,
//! Multi-Cluster Reporting of temperature and humidity) and an On/Off switch
//! (DeviceID 0x02, Off/On/Toggle). Both use security level 3 (encrypted,
//! 4-byte MIC, 32-bit frame counter) with a per-device random individual
//! key that the Commissioning command carries wrapped with the default
//! Trust Center link key `ZigBeeAlliance09`.
//!
//! The board (`efr32mg22-devkit`) supplies the physical radio, flash, LED,
//! button and VCOM; this crate owns the GP identity, the radio policy and
//! the flash partition of the GPD journal.

#![no_std]

use efr32s2_hal::flash::{Efr32s2Flash, FLASH_CAPACITY, FLASH_PAGE_SIZE, FlashPartition};
use gpd_app::{ALL_CHANNELS, BatteryCurve, GpdJournal, GpdKind, GpdProfile};
use zigbee_gp::types::{KeyType, SecurityLevel};

/// Manufacturer ID shared by the zigbee-rs products.
pub const MANUFACTURER_ID: u16 = 0x6D6B;
/// Model ID of the sensor (matched by the ZHA quirk).
pub const SENSOR_MODEL_ID: u16 = 0xF300;
/// Model ID of the switch (matched by the ZHA quirk).
pub const SWITCH_MODEL_ID: u16 = 0xF400;

/// No resident bootloader: the image starts at 0x0.
pub const APPLICATION_START: u32 = 0x0000_0000;
pub const APPLICATION_END: u32 = 0x0007_0000;
/// GPD journals: one two-sector journal per device kind, so flashing the
/// other image never touches (or reprovisions) this kind's identity.
pub const GPD_JOURNAL_START: u32 = APPLICATION_END;
pub const GPD_JOURNAL_SECTOR_SIZE: usize = FLASH_PAGE_SIZE;
/// Size of one kind's journal (two sectors).
pub const GPD_KIND_JOURNAL_SIZE: usize = 2 * GPD_JOURNAL_SECTOR_SIZE;
/// Switch journal `0x70000..0x74000`.
pub const SWITCH_JOURNAL_START: u32 = GPD_JOURNAL_START;
/// Sensor journal `0x74000..0x78000` (the location of the first hardware
/// commissioning; must not move, or the sensor loses its identity).
pub const SENSOR_JOURNAL_START: u32 = SWITCH_JOURNAL_START + GPD_KIND_JOURNAL_SIZE as u32;
/// Both journals.
pub const GPD_JOURNAL_SIZE: usize = 2 * GPD_KIND_JOURNAL_SIZE;
/// Never written by this product. `0x7C000..0x80000` is the Zigbee sensor
/// product's security journal and stays intact across a GPD reflash.
pub const UNTOUCHED_START: u32 = GPD_JOURNAL_START + GPD_JOURNAL_SIZE as u32;

const _: () = assert!(SENSOR_JOURNAL_START == 0x0007_4000);
const _: () = assert!(UNTOUCHED_START == 0x0007_8000);
const _: () = assert!(FLASH_CAPACITY == 512 * 1024);
const _: () = assert!((UNTOUCHED_START as usize) < FLASH_CAPACITY);
const _: () = assert!(GPD_JOURNAL_SECTOR_SIZE == 8 * 1024);

/// The flash window of the GPD journals.
pub type GpdPartition = FlashPartition<GPD_JOURNAL_START, GPD_JOURNAL_SIZE>;
/// The GPD journal of one device kind.
pub type GpdStore = GpdJournal<GpdPartition, GPD_JOURNAL_SECTOR_SIZE>;

/// Offset of `kind`'s journal inside [`GpdPartition`].
pub const fn journal_offset(kind: GpdKind) -> u32 {
    match kind {
        GpdKind::Switch => SWITCH_JOURNAL_START - GPD_JOURNAL_START,
        GpdKind::Sensor => SENSOR_JOURNAL_START - GPD_JOURNAL_START,
    }
}

/// Open `kind`'s GPD journal, consuming the board's unique flash handle:
/// this product writes nothing outside that kind's two sectors.
pub fn gpd_store(_flash: Efr32s2Flash, kind: GpdKind) -> GpdStore {
    // SAFETY: the unique `Efr32s2Flash` handle is consumed by value and this
    // is the only partition minted from it.
    let partition = unsafe { GpdPartition::new_unchecked() };
    let first = journal_offset(kind);
    GpdJournal::new(partition, first, first + GPD_JOURNAL_SECTOR_SIZE as u32)
}

/// The default global Trust Center link key `ZigBeeAlliance09`.
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = *b"ZigBeeAlliance09";

/// GPD command IDs advertised by the switch.
pub const SWITCH_COMMANDS: &[u8] = &[0x20, 0x21, 0x22];
/// GPD command IDs advertised by the sensor (Multi-Cluster Reporting).
pub const SENSOR_COMMANDS: &[u8] = &[0xA2];

const fn profile(
    kind: GpdKind,
    device_id: u8,
    model_id: u16,
    gpd_commands: &'static [u8],
) -> GpdProfile {
    GpdProfile {
        kind,
        device_id,
        manufacturer_id: MANUFACTURER_ID,
        model_id,
        gpd_commands,
        security_level: SecurityLevel::Encrypted,
        key_type: KeyType::IndividualKey,
        key_wrap_key: DEFAULT_TC_LINK_KEY,
        // One persisted reservation per 16 frames: 128 slots per sector,
        // a sector erase per 2048 frames.
        counter_block: 16,
        channel_mask: ALL_CHANNELS,
        // Beacon replies of a Zigbee router arrive within a few ms; leave
        // room for random jitter of several responders.
        scan_window_us: 140_000,
        commissioning_frames: 3,
        commissioning_gap_ms: 100,
        tx_repeats: 2,
        tx_repeat_gap_ms: 20,
        fixed_location: true,
    }
}

/// Temperature/humidity sensor (GP DeviceID 0x33, reports via
/// Multi-Cluster Reporting; the ZHA quirk maps the entities).
pub const SENSOR: GpdProfile = profile(GpdKind::Sensor, 0x33, SENSOR_MODEL_ID, SENSOR_COMMANDS);
/// On/Off switch (GP DeviceID 0x02 "On/Off Switch").
pub const SWITCH: GpdProfile = profile(GpdKind::Switch, 0x02, SWITCH_MODEL_ID, SWITCH_COMMANDS);

/// Bidirectional commissioning policy (`bidirectional` feature).
///
/// A proxy answers an rxAfterTx Commissioning GPDF only once the sink's
/// GP Response has come back through the network, so the reply normally
/// arrives on the second or a later attempt: six attempts 500 ms apart
/// cover a sink round trip of up to about 2.5 s. The receiver opens 1 ms
/// before gpdRxOffset and stays on 4 ms, for the proxy's timing error and
/// the MG22 receiver start-up.
#[cfg(feature = "bidirectional")]
pub const BIDIRECTIONAL: gpd_app::BidirectionalPolicy = gpd_app::BidirectionalPolicy {
    attempts: 6,
    attempt_gap_ms: 500,
    rx_open_us: 19_000,
    rx_window_us: 4_000,
};
#[cfg(feature = "bidirectional")]
const _: () = assert!(BIDIRECTIONAL.is_valid());

/// Sensor reporting interval.
pub const SENSOR_REPORT_INTERVAL_S: u64 = 60;
/// Button press longer than this recommissions.
pub const LONG_PRESS_MS: u64 = 3_000;

/// CR2032 coin cell on the BRD4183A/BRD4001A battery holder: 3.0 V fresh,
/// 1.8 V (the EFR32MG22 minimum supply) empty, linear in between. On the
/// WSTK's regulated 3.3 V the percentage clamps to 100 %.
pub const BATTERY: BatteryCurve = BatteryCurve {
    empty_mv: 1_800,
    full_mv: 3_000,
};

#[cfg(test)]
mod tests {
    use super::*;

    const MEMORY_X: &str = include_str!("../link/memory.x");

    fn region(name: &str) -> (u32, u32) {
        let line = MEMORY_X
            .lines()
            .map(str::trim)
            .find(|l| {
                l.strip_prefix(name)
                    .is_some_and(|r| r.trim_start().starts_with(':'))
            })
            .unwrap();
        let field = |key: &str| {
            let v = line
                .split(key)
                .nth(1)
                .unwrap()
                .trim_start()
                .strip_prefix('=')
                .unwrap()
                .split(',')
                .next()
                .unwrap()
                .trim();
            match v.strip_prefix("0x") {
                Some(hex) => u32::from_str_radix(hex, 16).unwrap(),
                None => v.strip_suffix('K').unwrap().parse::<u32>().unwrap() * 1024,
            }
        };
        (field("ORIGIN"), field("LENGTH"))
    }

    #[test]
    fn battery_curve_is_a_cr2032() {
        assert_eq!(BATTERY.half_percent(3_300), 200);
        assert_eq!(BATTERY.half_percent(2_400), 100);
        assert_eq!(BATTERY.half_percent(1_800), 0);
        assert_eq!(BATTERY.voltage_100mv(3_000), 30);
    }

    #[test]
    fn link_map_matches_constants() {
        assert_eq!(
            region("FLASH"),
            (APPLICATION_START, APPLICATION_END - APPLICATION_START)
        );
        assert_eq!(
            region("GPD_JOURNAL"),
            (GPD_JOURNAL_START, GPD_JOURNAL_SIZE as u32)
        );
        assert_eq!(
            region("UNTOUCHED"),
            (UNTOUCHED_START, FLASH_CAPACITY as u32 - UNTOUCHED_START)
        );
    }

    #[test]
    fn journals_are_disjoint_per_kind() {
        let span = |k| {
            let start = GPD_JOURNAL_START + journal_offset(k);
            start..start + GPD_KIND_JOURNAL_SIZE as u32
        };
        assert_eq!(span(GpdKind::Switch), 0x70000..0x74000);
        assert_eq!(span(GpdKind::Sensor), 0x74000..0x78000);
        assert!(span(GpdKind::Sensor).end <= UNTOUCHED_START);
    }

    #[test]
    fn profiles() {
        assert_eq!(DEFAULT_TC_LINK_KEY[0], 0x5A);
        assert_eq!(DEFAULT_TC_LINK_KEY[15], 0x39);
        for p in [SENSOR, SWITCH] {
            assert_eq!(p.manufacturer_id, 0x6D6B);
            assert_eq!(p.security_level, SecurityLevel::Encrypted);
            assert_eq!(p.key_type, KeyType::IndividualKey);
            assert_eq!(p.counter_block, 16);
        }
        assert_eq!((SENSOR.device_id, SENSOR.model_id), (0x33, 0xF300));
        assert_eq!((SWITCH.device_id, SWITCH.model_id), (0x02, 0xF400));
    }
}
