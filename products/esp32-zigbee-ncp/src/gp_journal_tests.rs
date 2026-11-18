//! The GP journal in the NCP's `zbnv` geometry ([`Nv::GREEN_POWER`]: two
//! 8 KiB logical sectors, the esp-storage write and erase sizes): the
//! largest Proxy Table (8) and Sink Table (16) pair fits one record pair,
//! survives a reboot, and long runs of counter changes and table changes
//! roll over between the sectors without losing the newest state.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};
use zigbee_gp::proxy::{ProxyEntry, SinkGroup, UnicastSink};
use zigbee_gp::sink::SinkEntry;
use zigbee_gp::types::{CommunicationMode, GpdId, KeyType, SecurityLevel};
use zigbee_runtime::gp_store::{GpProxyJournal, GpProxyStore, PersistentGpProxyTable};
use zigbee_runtime::green_power::{GP_PROXY_TABLE_ENTRIES, GP_SINK_TABLE_ENTRIES};

use crate::layout::Nv;

const EPID: [u8; 8] = [0xA5; 8];

#[derive(Debug)]
struct FlashError;

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

/// The GP window of `zbnv`, NOR semantics (programming only clears bits).
struct WindowFlash {
    data: Vec<u8>,
}

impl WindowFlash {
    fn new() -> Self {
        Self {
            data: vec![0xFF; Nv::GREEN_POWER.1 as usize],
        }
    }
}

impl ErrorType for WindowFlash {
    type Error = FlashError;
}

impl ReadNorFlash for WindowFlash {
    // esp-storage with `bytewise-read`.
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
        let start = offset as usize;
        let data = self
            .data
            .get(start..start + bytes.len())
            .ok_or(FlashError)?;
        bytes.copy_from_slice(data);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.data.len()
    }
}

impl NorFlash for WindowFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        let range = self
            .data
            .get_mut(from as usize..to as usize)
            .ok_or(FlashError)?;
        range.fill(0xFF);
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
        let start = offset as usize;
        let range = self
            .data
            .get_mut(start..start + bytes.len())
            .ok_or(FlashError)?;
        for (cell, byte) in range.iter_mut().zip(bytes) {
            *cell &= *byte;
        }
        Ok(())
    }
}

type Journal = GpProxyJournal<WindowFlash, { Nv::GREEN_POWER_SECTOR }>;

fn journal(flash: WindowFlash) -> Journal {
    Journal::new(flash, 0, Nv::GREEN_POWER_SECTOR as u32)
}

fn ieee(index: usize, tag: u8) -> [u8; 8] {
    [tag, index as u8, 3, 4, 5, 6, 7, 8]
}

/// The largest Proxy Table entry: IEEE GPD, alias, key, two unicast sinks
/// and two groups.
fn proxy_entry(index: usize) -> ProxyEntry {
    let mut e = ProxyEntry::new(GpdId::Ieee {
        ieee: ieee(index, 0x11),
        endpoint: 1,
    });
    e.assigned_alias = Some(0x7000 + index as u16);
    e.security_level = SecurityLevel::Encrypted;
    e.key_type = KeyType::IndividualKey;
    e.key = Some([index as u8; 16]);
    e.frame_counter = 1_000 + index as u32;
    for full in [false, true] {
        e.unicast_sinks
            .push(UnicastSink {
                ieee: ieee(index, 0x22),
                nwk: 0x0000,
                full,
            })
            .unwrap();
    }
    for group in [0x0B84, 0x0B85] {
        e.groups.push(SinkGroup { group, alias: None }).unwrap();
    }
    e.groupcast_radius = 5;
    e
}

/// The largest Sink Table entry: IEEE GPD, two groups, alias and key.
fn sink_entry(index: usize, counter: u32) -> SinkEntry {
    let mut e = SinkEntry::new(GpdId::Ieee {
        ieee: ieee(index, 0x33),
        endpoint: 2,
    });
    e.communication_mode = CommunicationMode::CommissionedGroupcast;
    for group in [0x0B84, 0x0C00 + index as u16] {
        e.groups
            .push(SinkGroup {
                group,
                alias: Some(0x1234),
            })
            .unwrap();
    }
    e.assigned_alias = Some(0x6000 + index as u16);
    e.security_level = SecurityLevel::Encrypted;
    e.key_type = KeyType::IndividualKey;
    e.key = Some([0x40 + index as u8; 16]);
    e.frame_counter = counter;
    e.device_id = 0x02;
    e.groupcast_radius = 7;
    e
}

fn snapshot(sink_counter: u32) -> PersistentGpProxyTable {
    let proxies: Vec<_> = (0..GP_PROXY_TABLE_ENTRIES).map(proxy_entry).collect();
    let sinks: Vec<_> = (0..GP_SINK_TABLE_ENTRIES)
        .map(|i| sink_entry(i, sink_counter + i as u32))
        .collect();
    PersistentGpProxyTable::capture(EPID, &proxies)
        .unwrap()
        .with_sink(&sinks)
        .unwrap()
}

fn load(flash: WindowFlash) -> (PersistentGpProxyTable, WindowFlash) {
    let mut reboot = journal(flash);
    let mut loaded = PersistentGpProxyTable::default();
    assert!(reboot.load_into(&mut loaded).unwrap());
    (loaded, reboot.into_storage())
}

#[test]
fn the_ncp_capacities_are_those_of_the_spec() {
    assert_eq!((GP_SINK_TABLE_ENTRIES, GP_PROXY_TABLE_ENTRIES), (16, 8));
}

#[test]
fn a_full_proxy_and_sink_table_pair_survives_a_reboot() {
    let mut store = journal(WindowFlash::new());
    let full = snapshot(500);
    store.store(&full).unwrap();
    let (loaded, _) = load(store.into_storage());
    assert_eq!(loaded.extended_pan_id(), EPID);
    assert_eq!(loaded.len(), GP_PROXY_TABLE_ENTRIES);
    assert_eq!(loaded.sink_len(), GP_SINK_TABLE_ENTRIES);
    let proxies: Vec<_> = loaded.entries().map(Result::unwrap).collect();
    assert_eq!(proxies[7].key, Some([7; 16]));
    assert_eq!(proxies[7].unicast_sinks.len(), 2);
    let sinks: Vec<_> = loaded.sink_entries().map(Result::unwrap).collect();
    for (i, entry) in sinks.iter().enumerate() {
        let expected = sink_entry(i, 500 + i as u32);
        assert_eq!(entry.gpd_id, expected.gpd_id);
        assert_eq!(entry.key, expected.key);
        assert_eq!(entry.frame_counter, expected.frame_counter);
        assert_eq!(entry.groups.as_slice(), expected.groups.as_slice());
    }
}

#[test]
fn counter_and_table_changes_roll_over_and_keep_the_newest_state() {
    let mut flash = WindowFlash::new();
    // Counter-only changes append compact records; every 50th change also
    // alters the table structure (a full record pair). Hundreds of writes
    // cross both 8 KiB sectors several times.
    let mut last = snapshot(0);
    for round in 1..=400u32 {
        let mut store = journal(flash);
        let mut next = snapshot(round * 64);
        if round % 50 == 0 {
            let proxies: Vec<_> = (0..GP_PROXY_TABLE_ENTRIES - 1).map(proxy_entry).collect();
            let sinks: Vec<_> = (0..GP_SINK_TABLE_ENTRIES)
                .map(|i| sink_entry(i, round * 64 + i as u32))
                .collect();
            next = PersistentGpProxyTable::capture(EPID, &proxies)
                .unwrap()
                .with_sink(&sinks)
                .unwrap();
        }
        store.store(&next).unwrap();
        flash = store.into_storage();
        last = next;
    }
    let (loaded, _) = load(flash);
    assert_eq!(loaded.len(), last.len());
    let sinks: Vec<_> = loaded.sink_entries().map(Result::unwrap).collect();
    let expected: Vec<_> = last.sink_entries().map(Result::unwrap).collect();
    for (got, want) in sinks.iter().zip(&expected) {
        assert_eq!(got.gpd_id, want.gpd_id);
        assert_eq!(got.frame_counter, want.frame_counter);
    }
    assert_eq!(sinks[0].frame_counter, 400 * 64);
}
