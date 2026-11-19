//! Translation section of the GP journal's sink record
//! (`green-power-translation-table`).
//!
//! The sink's Translation Table and the paired endpoints that GP Pairing
//! Configuration gave Sink Table entries are stored in the same `ZBGS`
//! record as the Sink Table, directly after its payload, so one commit
//! marker covers both and an interrupted write leaves the previous pair.
//! The record's byte 7 ([`SINK_RECORD_HAS_TRANSLATIONS`]) announces the
//! section; the header length still counts only the Sink Table payload, and
//! the record CRC covers the section. Firmware without the feature (and
//! journals written before it) never set the flag, so:
//!
//! * a journal written without translations loads with an empty
//!   Translation Table and default paired endpoints (migration);
//! * older firmware reading a new journal ignores the section and still
//!   restores the Sink Table and Proxy Table;
//! * a section of an unknown version is ignored, with a warning, instead of
//!   failing the whole restore.
//!
//! Section layout: `"TT"`, version 1, `u16` body length, then the body: a
//! translation count and per translation a length byte and
//! [`TranslationEntry::encode`]; a paired-endpoint count and per record the
//! Sink Table index, a kind (0 none, 1 list), an endpoint count and the
//! endpoints. Entries with default paired endpoints are not stored. With no
//! translation and only default paired endpoints there is no section.

use zigbee_gp::sink::SINK_TRANSLATION_ENTRIES;
use zigbee_gp::translation_table::{
    MAX_PAIRED_ENDPOINTS, MAX_TT_ENTRY_ENCODED_LEN, PairedEndpoints, TranslationEntry,
};

use super::{
    GP_PROXY_JOURNAL_SLOT_SIZE, GP_SINK_TABLE_ENTRIES, GpProxyJournal, GpStoreError,
    MAX_ENCODED_GP_SINK_TABLE_LEN, NorFlash, PersistentGpProxyTable, RECORD_CRC_OFFSET,
    RECORD_PAYLOAD_OFFSET, SINK_RECORD_HAS_TRANSLATIONS,
};
use crate::green_power::GpSink;

const TAIL_MAGIC: [u8; 2] = *b"TT";
const TAIL_VERSION: u8 = 1;
const TAIL_HEADER_LEN: usize = 5;
const PAIRED_NONE: u8 = 0;
const PAIRED_LIST: u8 = 1;
/// Largest translation section.
pub(super) const MAX_TAIL_LEN: usize = TAIL_HEADER_LEN
    + 1
    + SINK_TRANSLATION_ENTRIES * (1 + 1 + MAX_TT_ENTRY_ENCODED_LEN)
    + 1
    + GP_SINK_TABLE_ENTRIES * (3 + MAX_PAIRED_ENDPOINTS);

const _: () = assert!(SINK_TRANSLATION_ENTRIES <= u8::MAX as usize);
const _: () = assert!(MAX_TAIL_LEN <= u16::MAX as usize);

/// One stored item of a section body.
enum Item {
    Translation(TranslationEntry),
    Paired(usize, PairedEndpoints),
}

/// Walk a section body: translations, then paired-endpoint records.
/// `None` when it is malformed.
fn walk(body: &[u8], mut visit: impl FnMut(Item)) -> Option<()> {
    let mut at = 0usize;
    let byte = |at: &mut usize| -> Option<u8> {
        let b = *body.get(*at)?;
        *at += 1;
        Some(b)
    };
    let translations = byte(&mut at)?;
    for _ in 0..translations {
        let n = usize::from(byte(&mut at)?);
        let entry = TranslationEntry::decode(body.get(at..at + n)?).ok()?;
        at += n;
        visit(Item::Translation(entry));
    }
    let records = byte(&mut at)?;
    for _ in 0..records {
        let index = usize::from(byte(&mut at)?);
        let kind = byte(&mut at)?;
        let n = usize::from(byte(&mut at)?);
        let endpoints = body.get(at..at + n)?;
        at += n;
        let value = match kind {
            PAIRED_NONE if n == 0 => PairedEndpoints::None,
            PAIRED_LIST => PairedEndpoints::List(heapless::Vec::from_slice(endpoints).ok()?),
            _ => return None,
        };
        visit(Item::Paired(index, value));
    }
    (at == body.len()).then_some(())
}

/// The valid translation section of a committed sink record whose Sink
/// Table payload is `len` bytes, or an empty slice.
pub(super) fn stored_tail(record: &[u8; GP_PROXY_JOURNAL_SLOT_SIZE], len: usize) -> &[u8] {
    if record[7] & SINK_RECORD_HAS_TRANSLATIONS == 0 {
        return &[];
    }
    let at = RECORD_PAYLOAD_OFFSET + len;
    let header = &record[at..at + TAIL_HEADER_LEN];
    let body_len = usize::from(u16::from_le_bytes([header[3], header[4]]));
    let end = at + TAIL_HEADER_LEN + body_len;
    if header[..2] != TAIL_MAGIC
        || header[2] != TAIL_VERSION
        || TAIL_HEADER_LEN + body_len > MAX_TAIL_LEN
        || end > RECORD_CRC_OFFSET
        || walk(&record[at + TAIL_HEADER_LEN..end], |_| {}).is_none()
    {
        log::warn!("[GP] sink record translation section not understood; ignored");
        return &[];
    }
    &record[at..end]
}

impl PersistentGpProxyTable {
    /// The translation section behind the Sink Table payload (empty when
    /// there is none).
    pub(super) fn translation_tail(&self) -> &[u8] {
        let at = self.sink_len;
        let Some(header) = self.sink_payload.get(at..at + TAIL_HEADER_LEN) else {
            return &[];
        };
        if header[..2] != TAIL_MAGIC {
            return &[];
        }
        let len = TAIL_HEADER_LEN + usize::from(u16::from_le_bytes([header[3], header[4]]));
        self.sink_payload.get(at..at + len).unwrap_or(&[])
    }

    /// Set the translation section (validated by [`stored_tail`] or built by
    /// [`set_translations`](Self::set_translations)); empty clears it.
    pub(super) fn set_translation_tail(&mut self, tail: &[u8]) {
        let at = self.sink_len;
        if tail.is_empty() {
            self.sink_payload[at] = 0;
            return;
        }
        self.sink_payload[at..at + tail.len()].copy_from_slice(tail);
    }

    /// Capture the sink's translations and non-default paired endpoints
    /// after its Sink Table ([`with_sink`](Self::with_sink) first).
    pub fn set_translations(&mut self, sink: &GpSink) -> Result<(), GpStoreError> {
        let translations = sink.translations();
        let paired = sink
            .entries()
            .iter()
            .filter(|e| e.paired_endpoints != PairedEndpoints::Default)
            .count();
        if translations.is_empty() && paired == 0 {
            self.set_translation_tail(&[]);
            return Ok(());
        }
        // Encoded in place behind the Sink Table payload (no stack copy).
        let base = self.sink_len;
        let tail = self
            .sink_payload
            .get_mut(base..)
            .ok_or(GpStoreError::Full)?;
        let mut at = TAIL_HEADER_LEN;
        let mut put = |at: &mut usize, bytes: &[u8]| -> Result<(), GpStoreError> {
            tail.get_mut(*at..*at + bytes.len())
                .ok_or(GpStoreError::Full)?
                .copy_from_slice(bytes);
            *at += bytes.len();
            Ok(())
        };
        put(&mut at, &[translations.len() as u8])?;
        for entry in translations {
            let mut encoded = [0u8; 1 + MAX_TT_ENTRY_ENCODED_LEN];
            let n = entry
                .encode(&mut encoded)
                .map_err(|_| GpStoreError::Corrupt)?;
            put(&mut at, &[n as u8])?;
            put(&mut at, &encoded[..n])?;
        }
        put(&mut at, &[paired as u8])?;
        for (index, entry) in sink.entries().iter().enumerate() {
            let (kind, endpoints): (u8, &[u8]) = match &entry.paired_endpoints {
                PairedEndpoints::Default => continue,
                PairedEndpoints::None => (PAIRED_NONE, &[]),
                PairedEndpoints::List(list) => (PAIRED_LIST, list),
            };
            put(&mut at, &[index as u8, kind, endpoints.len() as u8])?;
            put(&mut at, endpoints)?;
        }
        let body = ((at - TAIL_HEADER_LEN) as u16).to_le_bytes();
        let mut header = 0;
        put(
            &mut header,
            &[TAIL_MAGIC[0], TAIL_MAGIC[1], TAIL_VERSION, body[0], body[1]],
        )?;
        Ok(())
    }

    /// Restore the stored translations and paired endpoints into `sink`,
    /// after its Sink Table entries.
    pub fn restore_translations(&self, sink: &mut GpSink) -> Result<(), GpStoreError> {
        let tail = self.translation_tail();
        if tail.is_empty() {
            return Ok(());
        }
        let mut full = false;
        let mut unknown_index = false;
        walk(&tail[TAIL_HEADER_LEN..], |item| match item {
            Item::Translation(entry) => full |= sink.restore_translation(entry).is_err(),
            Item::Paired(index, endpoints) => {
                unknown_index |= !sink.restore_paired_endpoints(index, endpoints)
            }
        })
        .ok_or(GpStoreError::Corrupt)?;
        if unknown_index {
            log::warn!("[GP] paired endpoints of a missing Sink Table entry ignored");
        }
        if full {
            return Err(GpStoreError::Full);
        }
        Ok(())
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> GpProxyJournal<S, SECTOR_SIZE> {
    /// Whether the sink record at `address` (Sink Table payload of
    /// `stored_len` bytes) holds the snapshot's translation section.
    pub(super) fn same_translation_tail(
        &mut self,
        address: u32,
        stored_len: u16,
        snapshot: &PersistentGpProxyTable,
    ) -> Result<bool, GpStoreError> {
        let expected = snapshot.translation_tail();
        let mut flags = [0u8; 1];
        self.storage
            .read(address + 7, &mut flags)
            .map_err(|_| GpStoreError::Hardware)?;
        if flags[0] & SINK_RECORD_HAS_TRANSLATIONS == 0 {
            return Ok(expected.is_empty());
        }
        if expected.is_empty() {
            return Ok(false);
        }
        let start = RECORD_PAYLOAD_OFFSET + usize::from(stored_len);
        if usize::from(stored_len) > MAX_ENCODED_GP_SINK_TABLE_LEN
            || start + expected.len() > RECORD_CRC_OFFSET
        {
            return Ok(false);
        }
        let mut chunk = [0u8; 64];
        for (index, part) in expected.chunks(chunk.len()).enumerate() {
            let window = &mut chunk[..part.len()];
            self.storage
                .read(address + (start + index * 64) as u32, window)
                .map_err(|_| GpStoreError::Hardware)?;
            if window != part {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gp_store::{GP_PROXY_JOURNAL_SECTOR_SIZE, GpProxyStore};
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use zigbee_gp::cluster::server_cmd;
    use zigbee_gp::config::{
        PairingConfiguration, PairingSecurity, TranslationRecord, TranslationTableUpdate,
        TtPayload, pairing_action, tt_action,
    };
    use zigbee_gp::sink::{ServerCommand, SinkConfig};
    use zigbee_gp::types::{CommunicationMode, GpdId, KeyType, SecurityLevel};
    use zigbee_mac::mock::MockMac;

    const SECTOR: usize = GP_PROXY_JOURNAL_SECTOR_SIZE;
    const EPID: [u8; 8] = [0x33; 8];

    #[derive(Debug)]
    struct FlashError;

    impl embedded_storage::nor_flash::NorFlashError for FlashError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    struct Flash {
        data: [u8; 2 * SECTOR],
        programs_left: Option<usize>,
    }

    impl ErrorType for Flash {
        type Error = FlashError;
    }

    impl ReadNorFlash for Flash {
        const READ_SIZE: usize = 1;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
            let at = offset as usize;
            bytes.copy_from_slice(self.data.get(at..at + bytes.len()).ok_or(FlashError)?);
            Ok(())
        }
        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for Flash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECTOR;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
            self.data[from as usize..to as usize].fill(0xFF);
            Ok(())
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
            if let Some(left) = self.programs_left.as_mut() {
                if *left == 0 {
                    return Err(FlashError);
                }
                *left -= 1;
            }
            let at = offset as usize;
            for (cell, byte) in self.data[at..at + bytes.len()].iter_mut().zip(bytes) {
                *cell &= *byte;
            }
            Ok(())
        }
    }

    fn journal(flash: Flash) -> GpProxyJournal<Flash> {
        GpProxyJournal::new(flash, 0, SECTOR as u32)
    }

    fn empty_flash() -> Flash {
        Flash {
            data: [0xFF; 2 * SECTOR],
            programs_left: None,
        }
    }

    fn sink() -> GpSink {
        let mut sink = GpSink::new(SinkConfig {
            endpoint: Some(1),
            ..SinkConfig::default()
        });
        sink.set_own_address(0x7C87, [1; 8], 0x1A62);
        sink
    }

    fn command(sink: &mut GpSink, command_id: u8, payload: &[u8]) -> u8 {
        let command = ServerCommand {
            src_nwk: 0x0000,
            unicast: true,
            tsn: 1,
            command_id,
            payload,
        };
        sink.on_server_command(&mut MockMac::new([0; 8]), &command, 0)
            .unwrap()
            .0
    }

    /// Pair SrcID `src` (level 3) with `endpoints` paired endpoints.
    fn pair(sink: &mut GpSink, src: u32, endpoints: (u8, &[u8])) {
        let pc = PairingConfiguration {
            actions: pairing_action::REPLACE,
            gpd_id: GpdId::SrcId(src),
            communication_mode: CommunicationMode::DerivedGroupcast,
            mac_seq_cap: true,
            rx_on_cap: false,
            fixed_location: true,
            device_id: 0x02,
            groups: &[],
            assigned_alias: None,
            groupcast_radius: 0,
            security: Some(PairingSecurity {
                level: SecurityLevel::Encrypted,
                key_type: KeyType::IndividualKey,
                key: [0x5A; 16],
            }),
            frame_counter: Some(10),
            number_of_paired_endpoints: endpoints.0,
            paired_endpoints: endpoints.1,
            application_information: None,
        };
        let mut b = [0u8; 64];
        let n = pc.encode(&mut b).unwrap();
        assert_eq!(command(sink, server_cmd::PAIRING_CONFIGURATION, &b[..n]), 0);
    }

    fn translate(sink: &mut GpSink, src: u32, endpoint: u8, zcl_command: u8) {
        let record = TranslationRecord {
            index: 0xFF,
            gpd_command: 0x22,
            endpoint,
            profile: 0x0104,
            cluster: 0x0006,
            zcl_command,
            payload: TtPayload::Fixed(&[]),
            additional_info: None,
        };
        let mut b = [0u8; 64];
        let n = TranslationTableUpdate::encode(
            &GpdId::SrcId(src),
            tt_action::REPLACE,
            &[record],
            &mut b,
        )
        .unwrap();
        assert_eq!(
            command(sink, server_cmd::TRANSLATION_TABLE_UPDATE, &b[..n]),
            0
        );
    }

    fn snapshot(sink: &GpSink) -> PersistentGpProxyTable {
        let mut snapshot = PersistentGpProxyTable::capture(EPID, &[])
            .unwrap()
            .with_sink(sink.entries())
            .unwrap();
        snapshot.set_translations(sink).unwrap();
        snapshot
    }

    fn load(journal: &mut GpProxyJournal<Flash>) -> PersistentGpProxyTable {
        let mut loaded = PersistentGpProxyTable::default();
        assert!(journal.load_into(&mut loaded).unwrap());
        loaded
    }

    fn restored(loaded: &PersistentGpProxyTable) -> GpSink {
        let mut rebooted = sink();
        for entry in loaded.sink_entries() {
            rebooted.restore(entry.unwrap()).unwrap();
        }
        loaded.restore_translations(&mut rebooted).unwrap();
        rebooted
    }

    /// The first sink record of a fresh journal sits in slot 0.
    fn record(journal: GpProxyJournal<Flash>) -> (Flash, [u8; GP_PROXY_JOURNAL_SLOT_SIZE]) {
        let mut flash = journal.into_storage();
        let mut record = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        flash.read(0, &mut record).unwrap();
        assert_eq!(record[..4], *b"ZBGS");
        (flash, record)
    }

    /// Without translations or paired endpoints the sink record is written
    /// exactly as before the feature (flag clear, 0xFF after the payload),
    /// and such a journal restores an empty Translation Table.
    #[test]
    fn journals_without_translations_keep_the_old_format_and_migrate() {
        let mut live = sink();
        pair(&mut live, 0x7A6B_9879, (0xFE, &[]));
        let mut store = journal(empty_flash());
        store.store(&snapshot(&live)).unwrap();
        let (flash, record) = record(store);
        let len = usize::from(u16::from_le_bytes([record[5], record[6]]));
        assert_eq!(record[7], 0);
        assert!(
            record[RECORD_PAYLOAD_OFFSET + len..RECORD_CRC_OFFSET]
                .iter()
                .all(|b| *b == 0xFF)
        );
        let mut journal = journal(flash);
        let loaded = load(&mut journal);
        assert!(loaded.translation_tail().is_empty());
        let rebooted = restored(&loaded);
        assert_eq!(rebooted.entries().len(), 1);
        assert!(rebooted.translations().is_empty());
        assert_eq!(
            rebooted.entries()[0].paired_endpoints,
            PairedEndpoints::Default
        );
    }

    /// Translations and paired endpoints survive a reboot; a reader without
    /// the feature still finds a valid Sink Table payload of the header
    /// length.
    #[test]
    fn translations_and_paired_endpoints_round_trip() {
        let mut live = sink();
        pair(&mut live, 0x7A6B_9879, (0xFE, &[]));
        pair(&mut live, 0x1111_2222, (2, &[2, 3]));
        pair(&mut live, 0x3333_4444, (0, &[]));
        translate(&mut live, 0x7A6B_9879, 2, 0x02);
        translate(&mut live, 0x1111_2222, 3, 0x00);
        let mut store = journal(empty_flash());
        store.store(&snapshot(&live)).unwrap();
        let (flash, record) = record(store);
        assert_eq!(record[7], SINK_RECORD_HAS_TRANSLATIONS);
        let len = usize::from(u16::from_le_bytes([record[5], record[6]]));
        PersistentGpProxyTable::validate_sink_payload(
            &record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len],
        )
        .unwrap();
        assert_eq!(&record[RECORD_PAYLOAD_OFFSET + len..][..3], b"TT\x01");
        let mut journal = journal(flash);
        let rebooted = restored(&load(&mut journal));
        assert_eq!(rebooted.translations(), live.translations());
        let paired: heapless::Vec<PairedEndpoints, 4> = rebooted
            .entries()
            .iter()
            .map(|e| e.paired_endpoints.clone())
            .collect();
        assert_eq!(
            paired.as_slice(),
            &[
                PairedEndpoints::Default,
                PairedEndpoints::List(heapless::Vec::from_slice(&[2, 3]).unwrap()),
                PairedEndpoints::None
            ]
        );
    }

    /// A counter-only change stays a compact counter record; a translation
    /// change is a structural change (a new record pair).
    #[test]
    fn translation_changes_are_structural_counter_changes_are_not() {
        let mut live = sink();
        pair(&mut live, 0x7A6B_9879, (0xFE, &[]));
        translate(&mut live, 0x7A6B_9879, 2, 0x02);
        let mut journal = journal(empty_flash());
        let first = snapshot(&live);
        journal.store(&first).unwrap();
        // Same translations, the counter moved: no new sink record.
        let mut moved = live.entries()[0].clone();
        moved.frame_counter = 500;
        let mut bumped = sink();
        bumped.restore(moved).unwrap();
        translate(&mut bumped, 0x7A6B_9879, 2, 0x02);
        journal.store(&snapshot(&bumped)).unwrap();
        let mut flash = journal.into_storage();
        let mut slot = [0u8; 4];
        flash.read(2 * 1024, &mut slot).unwrap();
        assert_eq!(slot, *b"ZBGC");
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let loaded = load(&mut journal);
        assert_eq!(loaded.translation_tail(), first.translation_tail());
        // Another translation: a full pair in the next slots.
        translate(&mut bumped, 0x7A6B_9879, 2, 0x00);
        journal.store(&snapshot(&bumped)).unwrap();
        let mut flash = journal.into_storage();
        let mut magic = [0u8; 4];
        flash.read(SECTOR as u32, &mut magic).unwrap();
        assert_eq!(magic, *b"ZBGS", "the sector rolled over to a new pair");
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let rebooted = restored(&load(&mut journal));
        assert_eq!(rebooted.translations()[0].zcl_command, 0x00);
    }

    /// Power fails while the new pair is written: the previous Sink Table
    /// and translations load.
    #[test]
    fn interrupted_write_keeps_the_previous_translations() {
        let mut live = sink();
        pair(&mut live, 0x7A6B_9879, (0xFE, &[]));
        translate(&mut live, 0x7A6B_9879, 2, 0x02);
        let mut journal = journal(empty_flash());
        let first = snapshot(&live);
        journal.store(&first).unwrap();
        translate(&mut live, 0x7A6B_9879, 2, 0x01);
        let mut flash = journal.into_storage();
        flash.programs_left = Some(1);
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert!(journal.store(&snapshot(&live)).is_err());
        let mut flash = journal.into_storage();
        flash.programs_left = None;
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let loaded = load(&mut journal);
        assert_eq!(loaded.translation_tail(), first.translation_tail());
        assert_eq!(restored(&loaded).translations()[0].zcl_command, 0x02);
    }

    /// A section of an unknown version or with a bad length is ignored, not
    /// trusted and not fatal.
    #[test]
    fn unknown_sections_are_ignored() {
        let mut live = sink();
        pair(&mut live, 0x7A6B_9879, (0xFE, &[]));
        translate(&mut live, 0x7A6B_9879, 2, 0x02);
        let snapshot = snapshot(&live);
        let tail = snapshot.translation_tail();
        let mut record = [0xFFu8; GP_PROXY_JOURNAL_SLOT_SIZE];
        record[7] = SINK_RECORD_HAS_TRANSLATIONS;
        let at = RECORD_PAYLOAD_OFFSET + 10;
        record[at..at + tail.len()].copy_from_slice(tail);
        assert_eq!(stored_tail(&record, 10), tail);
        record[at + 2] = 2;
        assert!(stored_tail(&record, 10).is_empty());
        record[at + 2] = TAIL_VERSION;
        record[at + 3] = 0xF0;
        assert!(stored_tail(&record, 10).is_empty());
        record[at + 3] = tail[3];
        record[at + TAIL_HEADER_LEN] = 9;
        assert!(stored_tail(&record, 10).is_empty());
        record[7] = 0;
        assert!(stored_tail(&record, 10).is_empty());
    }
}
