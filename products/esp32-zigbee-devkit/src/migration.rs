//! One-time migration of the ESP32-C6/H2 Zigbee persistence from the legacy
//! log-structured NV format to the crash-safe security-state journal.
//!
//! # Why this exists
//!
//! Before this product owned persistence, the ESP firmware stored network
//! state with [`zigbee_runtime::log_nv::LogStructuredNv`] through
//! `ZigbeeDevice::save_state`/`restore_state`, in the last 8 KiB of the flash
//! chip (`0x3FE000..0x400000`). The redesign replaced that with the
//! [`SecurityStateJournal`], placed at the *same* two physical sectors so
//! already-joined devices keep their reserved NV window.
//!
//! A device flashed with the new firmware therefore boots facing legacy
//! `LNV1`/`0xA55A` records where the journal expects `ZBSS` records. Without a
//! migration the journal scan finds nothing, the device looks factory-new, and
//! — the dangerous part — a fresh commissioning could re-issue NWK frame
//! counter values the legacy firmware had already transmitted. Security-counter
//! durability is a correctness requirement, so this module bridges the two
//! formats exactly once, crash-safely.
//!
//! # What is (and is not) preserved
//!
//! The legacy format persisted the NWK identity, the active network key, and
//! the *live* NWK outgoing frame counter — but never a unique Trust Center
//! Link Key (`save_state` writes no `ApsLinkKey`/`ApsTrustCenterAddress`
//! item). The legacy runtime could negotiate a unique key, but after a reboot
//! its restore path had only the well-known *default global* Trust Center link
//! key. Traffic under that default key uses the NWK frame-counter space
//! (`zigbee_aps::Apsde::next_default_tc_link_key_frame_counter`), which is
//! present in the legacy record.
//!
//! The migration therefore preserves the network: it writes a **commissioned**
//! record carrying the legacy NWK identity, key and key sequence, flagged
//! [`PersistentSecurityState::legacy_default_tclk`] so the runtime knows there
//! is no unique TCLK, installs no APS key, and invents no Trust Center address
//! or key. An already-joined ESP32-C6/H2 keeps its PAN, short address and
//! network key across the format switch and resumes exactly as the legacy
//! firmware did, instead of being treated as factory-new.
//!
//! Both durable counter ranges are floored strictly above anything the legacy
//! image can have transmitted: the legacy live counter, plus the same safety
//! margin the legacy restore path applied, plus a fresh reservation block.
//! `global_counter_limit` covers NWK security *and* the default-global-key APS
//! traffic that shares it; `tclk_counter_limit` seeds the space a unique key
//! would later use, so a Trust Center that delivers one after the upgrade
//! cannot restart it below a used value. `restore_security_state` then reserves
//! a further block from both floors before any secured frame is sent.
//!
//! If the legacy record cannot be represented as a commissioned network — no
//! stored IEEE address, or field values the new format rejects — the migration
//! degrades explicitly to [`MigrationOutcome::MigratedCounters`]: the counter
//! floors are still carried over and the device re-commissions. It never
//! fabricates the missing identity, and never silently discards the counters.
//!
//! # Crash safety
//!
//! The journal and the legacy log share the same two physical sectors, so any
//! journal write erases one of them. The migration always commits the new
//! record into the *scratch* (erased) sector, never the authoritative legacy
//! page, so the legacy record survives until the migrated record is durably
//! committed and verified. A power loss at any point leaves either the intact
//! legacy record (retry next boot) or a valid committed journal (preferred over
//! any legacy remnant), never a silent factory-new wipe.

use embedded_storage::nor_flash::NorFlash;
use zigbee_bdb::FRAME_COUNTER_RESERVATION_SIZE;
use zigbee_runtime::log_nv::LogStructuredNv;
use zigbee_runtime::nv_storage::{NvError, NvItemId, NvStorage};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SLOT_SIZE, SecurityStateJournal};
use zigbee_runtime::security_store::{
    PersistentSecurityState, SecurityStateStore, SecurityStoreError,
};

/// Frames the legacy firmware may have transmitted after its last NV save.
///
/// The legacy restore path added the identical margin (`FC_SAFETY_MARGIN` in
/// `zigbee_runtime`) before trusting a persisted NWK frame counter, so the
/// migrated floor must clear it too.
const LEGACY_FRAME_COUNTER_MARGIN: u32 = 1000;

/// Bytes of the sector header inspected to classify an erased vs written
/// sector. Matches the probe `LogStructuredNv` itself uses for its page state.
const HEADER_PROBE: usize = 16;

const JOURNAL_MAGIC: [u8; 4] = *b"ZBSS";
const JOURNAL_COMMIT_OFFSET: usize = 124;
const JOURNAL_COMMIT: [u8; 4] = *b"CMIT";

/// Result of a successful migration pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MigrationOutcome {
    /// A committed new-format journal already existed; nothing was migrated.
    /// This is the steady state on every reboot after the first migration and
    /// the preferred outcome after an interrupted migration.
    JournalPresent,
    /// Legacy on-network state was found and committed as a *commissioned*
    /// journal record: the device keeps its existing network, credentials and
    /// counter floors and resumes on the previous PAN.
    MigratedNetwork,
    /// Legacy state was found but could not be represented as a commissioned
    /// network: the device had already left the network, or the record lacks a
    /// persisted IEEE address / holds values the new format rejects. Only the
    /// frame-counter floors were carried over: the device re-commissions, but
    /// can never reuse a counter the legacy image sent.
    MigratedCounters,
    /// Neither a journal nor a usable legacy state exists; the device is
    /// genuinely factory-new and should commission from scratch.
    FactoryNew,
}

/// A migration failure the caller must **not** confuse with a factory-new
/// device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MigrationError {
    /// A flash read/erase/write failed while inspecting or writing state. The
    /// legacy region may still be intact — halt and retry, never wipe.
    Hardware,
    /// A legacy region is present but could not be parsed as a valid,
    /// on-network legacy state (unreadable, truncated, or missing a required
    /// field). Do not wipe it; surface for inspection.
    CorruptLegacy,
    /// Writing or committing the migrated journal record failed.
    Journal(SecurityStoreError),
}

/// Run the one-time legacy → journal migration over the two NV sectors.
///
/// `sector_a`/`sector_b` are offsets *within* `flash` (the reserved NV window),
/// identical to the values passed to [`SecurityStateJournal::new`]. The
/// function is idempotent: once a committed journal exists it returns
/// [`MigrationOutcome::JournalPresent`] without touching flash.
pub fn migrate<F: NorFlash>(
    flash: &mut F,
    sector_a: u32,
    sector_b: u32,
) -> Result<MigrationOutcome, MigrationError> {
    // 1. Prefer any already-committed journal. This makes reboots idempotent
    //    and, after an interrupted migration, prefers a valid new journal over
    //    a legacy remnant.
    if journal_has_state(flash, sector_a, sector_b)? {
        return Ok(MigrationOutcome::JournalPresent);
    }

    // 2. No journal. Tell "factory-new" (both sectors erased) apart from
    //    "legacy present" before mutating anything.
    if sector_is_erased(flash, sector_a)? && sector_is_erased(flash, sector_b)? {
        return Ok(MigrationOutcome::FactoryNew);
    }

    // A factory-new device can lose power during the journal's first commit:
    // one sector then contains a valid ZBSS prefix without the final `CMIT`
    // word while the other remains erased. There is no legacy state to
    // migrate, and the journal can recover by erasing/retrying that sector.
    // Do not misclassify this precise new-format interruption as corrupt
    // legacy state and permanently halt every subsequent boot.
    if lone_uncommitted_journal_record(flash, sector_a, sector_b)? {
        return Ok(MigrationOutcome::FactoryNew);
    }

    // 3. Read the legacy state. `LogStructuredNv::new` compacts to a single
    //    active page and erases the scratch page, but never the authoritative
    //    page, so a power loss here is safe.
    let Some((state, outcome)) = read_legacy_state(flash, sector_a, sector_b)? else {
        // The region held data but no on-network legacy state (an old scratch
        // page or app-only NV). Nothing to migrate; commission fresh.
        return Ok(MigrationOutcome::FactoryNew);
    };

    // 4. Commit the migrated record into the erased scratch sector so the
    //    authoritative legacy page is never erased before the new journal is
    //    durable.
    let (first, second) = scratch_first_order(flash, sector_a, sector_b)?;
    let mut journal = SecurityStateJournal::new(&mut *flash, first, second);
    journal.store(&state).map_err(MigrationError::Journal)?;
    Ok(outcome)
}

/// Whether a committed new-format journal record can be loaded.
fn journal_has_state<F: NorFlash>(
    flash: &mut F,
    sector_a: u32,
    sector_b: u32,
) -> Result<bool, MigrationError> {
    let mut journal = SecurityStateJournal::new(&mut *flash, sector_a, sector_b);
    match journal.load() {
        Ok(state) => Ok(state.is_some()),
        Err(_) => Err(MigrationError::Hardware),
    }
}

/// Whether the sector header reads as fully erased (`0xFF`).
fn sector_is_erased<F: NorFlash>(flash: &mut F, sector: u32) -> Result<bool, MigrationError> {
    let mut header = [0u8; HEADER_PROBE];
    flash
        .read(sector, &mut header)
        .map_err(|_| MigrationError::Hardware)?;
    Ok(header.iter().all(|byte| *byte == 0xFF))
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FirstSlotState {
    Erased,
    UncommittedJournal,
    Other,
}

/// Detect the recoverable result of power loss during the first-ever journal
/// commit, while keeping arbitrary garbage and committed-but-corrupt records
/// classified as errors.
fn lone_uncommitted_journal_record<F: NorFlash>(
    flash: &mut F,
    sector_a: u32,
    sector_b: u32,
) -> Result<bool, MigrationError> {
    let a = first_slot_state(flash, sector_a)?;
    let b = first_slot_state(flash, sector_b)?;
    Ok(matches!(
        (a, b),
        (FirstSlotState::UncommittedJournal, FirstSlotState::Erased)
            | (FirstSlotState::Erased, FirstSlotState::UncommittedJournal)
    ))
}

fn first_slot_state<F: NorFlash>(
    flash: &mut F,
    sector: u32,
) -> Result<FirstSlotState, MigrationError> {
    let mut slot = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
    flash
        .read(sector, &mut slot)
        .map_err(|_| MigrationError::Hardware)?;
    if slot.iter().all(|byte| *byte == 0xFF) {
        return Ok(FirstSlotState::Erased);
    }
    if slot[0..4] == JOURNAL_MAGIC
        && slot[JOURNAL_COMMIT_OFFSET..JOURNAL_COMMIT_OFFSET + JOURNAL_COMMIT.len()]
            != JOURNAL_COMMIT
    {
        return Ok(FirstSlotState::UncommittedJournal);
    }
    Ok(FirstSlotState::Other)
}

/// Decode the legacy `LogStructuredNv` state, if the device was on a network.
///
/// Returns `Ok(None)` for "no legacy network state" (factory-new) and
/// `Err(MigrationError::CorruptLegacy)` for a present-but-unparseable region —
/// the two must never be conflated.
#[allow(clippy::type_complexity)]
fn read_legacy_state<F: NorFlash>(
    flash: &mut F,
    sector_a: u32,
    sector_b: u32,
) -> Result<Option<(PersistentSecurityState, MigrationOutcome)>, MigrationError> {
    let mut nv = match LogStructuredNv::new(&mut *flash, sector_a, sector_b) {
        Ok(nv) => nv,
        Err(NvError::HardwareError) => return Err(MigrationError::Hardware),
        Err(_) => return Err(MigrationError::CorruptLegacy),
    };

    let on_network =
        read_item::<_, 1>(&mut nv, NvItemId::BdbNodeIsOnNetwork)?.is_some_and(|byte| byte[0] != 0);
    if on_network {
        return decode_legacy(&mut nv).map(Some);
    }

    // Off network, but a legacy image that left one still has its last counter
    // on flash. Carry the floor over so a later join cannot reuse a value under
    // the (unchanged) network key; only a region with no counter at all is
    // genuinely factory-new.
    let Some(live_counter) = read_item::<_, 4>(&mut nv, NvItemId::NwkFrameCounter)? else {
        return Ok(None);
    };
    let floor = reserved_counter_floor(u32::from_le_bytes(live_counter));
    let mut counters = PersistentSecurityState::empty();
    counters.global_counter_limit = floor;
    counters.tclk_counter_limit = floor;
    Ok(Some((counters, MigrationOutcome::MigratedCounters)))
}

/// Build the migrated [`PersistentSecurityState`] from the legacy NWK items,
/// reserving fresh counter floors above the legacy live counter.
///
/// Returns the commissioned network record when the legacy data is complete
/// enough to keep the device on its network, and the counters-only fallback
/// otherwise. Missing *required* items are corruption and are reported as
/// such; a merely unrepresentable network degrades instead of bricking a
/// deployed device.
fn decode_legacy<S: NvStorage>(
    nv: &mut S,
) -> Result<(PersistentSecurityState, MigrationOutcome), MigrationError> {
    let pan_id = u16::from_le_bytes(require(nv, NvItemId::NwkPanId)?);
    let channel = require::<_, 1>(nv, NvItemId::NwkChannel)?[0];
    let short_address = u16::from_le_bytes(require(nv, NvItemId::NwkShortAddress)?);
    let extended_pan_id = require::<_, 8>(nv, NvItemId::NwkExtendedPanId)?;
    let network_key = require::<_, 16>(nv, NvItemId::NwkKey)?;
    let key_sequence = require::<_, 1>(nv, NvItemId::NwkKeySeqNum)?[0];
    let live_counter = u32::from_le_bytes(require(nv, NvItemId::NwkFrameCounter)?);

    let ieee_address = read_item::<_, 8>(nv, NvItemId::NwkIeeeAddress)?.unwrap_or([0; 8]);
    let depth = read_item::<_, 1>(nv, NvItemId::NwkDepth)?.map_or(1, |b| b[0]);
    let parent_address =
        read_item::<_, 2>(nv, NvItemId::NwkParentAddress)?.map_or(0, u16::from_le_bytes);
    let update_id = read_item::<_, 1>(nv, NvItemId::NwkUpdateId)?.map_or(0, |b| b[0]);

    let counter_floor = reserved_counter_floor(live_counter);

    let mut state = PersistentSecurityState::empty();
    state.extended_pan_id = extended_pan_id;
    state.pan_id = pan_id;
    state.short_address = short_address;
    state.ieee_address = ieee_address;
    state.channel = channel;
    state.depth = depth;
    state.parent_address = parent_address;
    state.update_id = update_id;
    state.network_key = network_key;
    state.key_sequence = key_sequence;
    // NWK security and the legacy image's default-global-key APS traffic share
    // this counter space.
    state.global_counter_limit = counter_floor;
    // A negotiated unique TCLK was not persisted. Every APS-secured frame sent
    // with it was also carried by a NWK-secured frame, so the NWK live counter
    // is a conservative floor for that per-key counter too. If the Trust
    // Center later redelivers the same key, it cannot restart below a value
    // the legacy image may have used.
    state.tclk_counter_limit = counter_floor;
    // Commissioned, but explicitly without a unique Trust Center link key: no
    // TC address or key is invented, and the runtime keeps using the default
    // global key from the NWK counter space.
    state.commissioned = true;
    state.legacy_default_tclk = true;
    state.tclk_present = false;

    if state.validate().is_ok() {
        return Ok((state, MigrationOutcome::MigratedNetwork));
    }

    // The legacy record is readable but cannot describe a commissioned network
    // under the new format (typically no persisted IEEE address, which NWK
    // security nonces require). Keep the durable counter floors — losing them
    // would permit counter reuse — and let the device commission again.
    let mut counters = PersistentSecurityState::empty();
    counters.global_counter_limit = counter_floor;
    counters.tclk_counter_limit = counter_floor;
    Ok((counters, MigrationOutcome::MigratedCounters))
}

/// The reserved exclusive upper bound for the migrated counter spaces: the
/// legacy live counter, plus the frames possibly sent since the last save, plus
/// a fresh reservation block. Saturating so a near-`u32::MAX` legacy counter
/// cannot wrap.
fn reserved_counter_floor(live_counter: u32) -> u32 {
    live_counter
        .saturating_add(LEGACY_FRAME_COUNTER_MARGIN)
        .saturating_add(FRAME_COUNTER_RESERVATION_SIZE)
}

/// Order the two sectors so the journal's first record lands in the erased
/// scratch sector, keeping the authoritative legacy page intact until commit.
fn scratch_first_order<F: NorFlash>(
    flash: &mut F,
    sector_a: u32,
    sector_b: u32,
) -> Result<(u32, u32), MigrationError> {
    if sector_is_erased(flash, sector_a)? {
        Ok((sector_a, sector_b))
    } else if sector_is_erased(flash, sector_b)? {
        Ok((sector_b, sector_a))
    } else {
        // `LogStructuredNv::new` guarantees exactly one erased scratch page for
        // a valid legacy state; neither being erased means the region is not
        // what we decoded. Refuse rather than risk erasing legacy before a
        // commit.
        Err(MigrationError::CorruptLegacy)
    }
}

/// Read a fixed-length legacy item, returning `None` if absent.
fn read_item<S: NvStorage, const N: usize>(
    nv: &mut S,
    id: NvItemId,
) -> Result<Option<[u8; N]>, MigrationError> {
    let mut buf = [0u8; N];
    match nv.read(id, &mut buf) {
        Ok(len) if len == N => Ok(Some(buf)),
        Ok(_) => Err(MigrationError::CorruptLegacy),
        Err(NvError::NotFound) => Ok(None),
        Err(NvError::HardwareError) => Err(MigrationError::Hardware),
        Err(_) => Err(MigrationError::CorruptLegacy),
    }
}

/// Read a required fixed-length legacy item; absence is corruption.
fn require<S: NvStorage, const N: usize>(
    nv: &mut S,
    id: NvItemId,
) -> Result<[u8; N], MigrationError> {
    read_item::<S, N>(nv, id)?.ok_or(MigrationError::CorruptLegacy)
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use zigbee_runtime::security_journal::SECURITY_JOURNAL_SECTOR_SIZE;

    const SECTOR: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;
    const CAPACITY: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
    const SECTOR_A: u32 = 0;
    const SECTOR_B: u32 = SECTOR;

    // ── Test double: NOR flash matching the ESP geometry ────────────────────
    //
    // READ_SIZE = 1 (esp-storage `bytewise-read`), WRITE_SIZE = 4, ERASE_SIZE =
    // 4096. NOR semantics: programming only clears bits, erase sets `0xFF`, and
    // a raised bit or misaligned erase is a hardware error. Optional counters
    // model a power loss mid-write or a read fault.
    struct MockFlash {
        data: [u8; CAPACITY],
        programs_before_failure: Option<usize>,
        reads_before_failure: Option<usize>,
    }

    impl MockFlash {
        fn erased() -> Self {
            Self {
                data: [0xFF; CAPACITY],
                programs_before_failure: None,
                reads_before_failure: None,
            }
        }

        fn offset(address: u32) -> Result<usize, NorFlashErrorKind> {
            let offset = address as usize;
            if offset < CAPACITY {
                Ok(offset)
            } else {
                Err(NorFlashErrorKind::OutOfBounds)
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, address: u32, output: &mut [u8]) -> Result<(), Self::Error> {
            if let Some(remaining) = self.reads_before_failure.as_mut() {
                if *remaining == 0 {
                    return Err(NorFlashErrorKind::Other);
                }
                *remaining -= 1;
            }
            let start = Self::offset(address)?;
            let end = start
                .checked_add(output.len())
                .filter(|end| *end <= self.data.len())
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            output.copy_from_slice(&self.data[start..end]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE;

        fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
            if let Some(remaining) = self.programs_before_failure.as_mut() {
                if *remaining == 0 {
                    return Err(NorFlashErrorKind::Other);
                }
                *remaining -= 1;
            }
            let start = Self::offset(address)?;
            if !start.is_multiple_of(Self::WRITE_SIZE)
                || !data.len().is_multiple_of(Self::WRITE_SIZE)
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let end = start
                .checked_add(data.len())
                .filter(|end| *end <= self.data.len())
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            for (old, new) in self.data[start..end].iter_mut().zip(data) {
                if (*old & *new) != *new {
                    return Err(NorFlashErrorKind::Other);
                }
                *old &= *new;
            }
            Ok(())
        }

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            let start = Self::offset(from)?;
            let end = usize::try_from(to).map_err(|_| NorFlashErrorKind::OutOfBounds)?;
            if !start.is_multiple_of(SECURITY_JOURNAL_SECTOR_SIZE)
                || !end.is_multiple_of(SECURITY_JOURNAL_SECTOR_SIZE)
                || start >= end
                || end > self.data.len()
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            self.data[start..end].fill(0xFF);
            Ok(())
        }
    }

    /// A legacy on-network image, written through the real `LogStructuredNv` so
    /// the on-flash bytes are authentic. `fc` is the persisted live NWK counter.
    fn legacy_on_network(fc: u32) -> MockFlash {
        legacy_image(fc, Some([0xBB; 8]))
    }

    /// Like [`legacy_on_network`], but the persisted IEEE address can be
    /// omitted to model a legacy record saved before the MAC address was
    /// known.
    fn legacy_image(fc: u32, ieee: Option<[u8; 8]>) -> MockFlash {
        let mut flash = MockFlash::erased();
        {
            let mut nv =
                LogStructuredNv::new(&mut flash, SECTOR_A, SECTOR_B).expect("legacy nv init");
            nv.write(NvItemId::NwkPanId, &0x1234u16.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::NwkChannel, &[15]).unwrap();
            nv.write(NvItemId::NwkShortAddress, &0x5678u16.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::NwkExtendedPanId, &[0xAA; 8]).unwrap();
            if let Some(ieee) = ieee {
                nv.write(NvItemId::NwkIeeeAddress, &ieee).unwrap();
            }
            nv.write(NvItemId::NwkDepth, &[2]).unwrap();
            nv.write(NvItemId::NwkParentAddress, &0x0001u16.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::NwkUpdateId, &[7]).unwrap();
            nv.write(NvItemId::NwkKey, &[0xCC; 16]).unwrap();
            nv.write(NvItemId::NwkKeySeqNum, &[3]).unwrap();
            nv.write(NvItemId::NwkFrameCounter, &fc.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::BdbNodeIsOnNetwork, &[1]).unwrap();
        }
        flash
    }

    /// Load the committed journal state, consuming the flash.
    fn loaded(mut flash: MockFlash) -> Option<PersistentSecurityState> {
        let mut journal = SecurityStateJournal::new(&mut flash, SECTOR_A, SECTOR_B);
        journal.load().expect("journal load")
    }

    // `&mut MockFlash` is itself a `NorFlash` via the standard blanket impls in
    // `embedded-storage`, so the helpers above can borrow it directly.

    #[test]
    fn valid_legacy_state_keeps_the_device_on_its_network() {
        let mut flash = legacy_on_network(0x2000);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::MigratedNetwork)
        );

        let state = loaded(flash).expect("migrated state present");
        assert!(
            state.commissioned,
            "an already-joined device must not be treated as factory-new"
        );
        assert_eq!(state.pan_id, 0x1234);
        assert_eq!(state.channel, 15);
        assert_eq!(state.short_address, 0x5678);
        assert_eq!(state.extended_pan_id, [0xAA; 8]);
        assert_eq!(state.ieee_address, [0xBB; 8]);
        assert_eq!(state.depth, 2);
        assert_eq!(state.parent_address, 0x0001);
        assert_eq!(state.update_id, 7);
        assert_eq!(state.network_key, [0xCC; 16]);
        assert_eq!(state.key_sequence, 3);
        assert!(!state.rejoin_pending);

        // The transitional limitation is represented, not papered over: no
        // unique TCLK, and no invented Trust Center identity or key.
        assert!(state.legacy_default_tclk);
        assert!(!state.tclk_present);
        assert_eq!(state.trust_center_address, [0; 8]);
        assert_eq!(state.trust_center_link_key, [0; 16]);
    }

    #[test]
    fn legacy_state_without_an_ieee_address_preserves_counters_only() {
        let live = 0x2000;
        let mut flash = legacy_image(live, None);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::MigratedCounters)
        );

        let state = loaded(flash).expect("migrated state present");
        assert!(
            !state.commissioned,
            "NWK security nonces need the IEEE address; never fabricate one"
        );
        assert!(!state.legacy_default_tclk);
        assert_eq!(state.network_key, [0; 16]);
        // Losing the floors would allow counter reuse, so they survive even
        // when the network itself cannot.
        let floor = live + LEGACY_FRAME_COUNTER_MARGIN + FRAME_COUNTER_RESERVATION_SIZE;
        assert_eq!(state.global_counter_limit, floor);
        assert_eq!(state.tclk_counter_limit, floor);
    }

    #[test]
    fn an_off_network_legacy_record_still_preserves_its_counter_floor() {
        // Left the network under the legacy firmware: no network to restore,
        // but the counter it last used is still on flash and the network key a
        // later join receives is the same one.
        let live: u32 = 0x9000;
        let mut flash = MockFlash::erased();
        {
            let mut nv = LogStructuredNv::new(&mut flash, SECTOR_A, SECTOR_B).unwrap();
            nv.write(NvItemId::NwkFrameCounter, &live.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::BdbNodeIsOnNetwork, &[0]).unwrap();
        }
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::MigratedCounters)
        );
        let state = loaded(flash).expect("counter floor committed");
        assert!(!state.commissioned);
        let floor = live + LEGACY_FRAME_COUNTER_MARGIN + FRAME_COUNTER_RESERVATION_SIZE;
        assert_eq!(state.global_counter_limit, floor);
        assert_eq!(state.tclk_counter_limit, floor);
    }

    #[test]
    fn legacy_region_without_any_counter_is_factory_new() {
        // Application-only NV: nothing Zigbee-durable to carry over.
        let mut flash = MockFlash::erased();
        {
            let mut nv = LogStructuredNv::new(&mut flash, SECTOR_A, SECTOR_B).unwrap();
            nv.write(
                NvItemId::BdbPrimaryChannelSet,
                &0x0780_0000u32.to_le_bytes(),
            )
            .unwrap();
        }
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::FactoryNew)
        );
        assert_eq!(loaded(flash), None);
    }

    #[test]
    fn factory_new_flash_reports_no_state() {
        let mut flash = MockFlash::erased();
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::FactoryNew)
        );
        assert_eq!(loaded(flash), None);
    }

    #[test]
    fn on_network_flag_without_required_fields_is_corrupt_not_factory_new() {
        // On-network is set but the required NWK key/counter are missing.
        let mut flash = MockFlash::erased();
        {
            let mut nv = LogStructuredNv::new(&mut flash, SECTOR_A, SECTOR_B).unwrap();
            nv.write(NvItemId::BdbNodeIsOnNetwork, &[1]).unwrap();
            nv.write(NvItemId::NwkPanId, &0x1234u16.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::NwkChannel, &[15]).unwrap();
            nv.write(NvItemId::NwkShortAddress, &0x5678u16.to_le_bytes())
                .unwrap();
            nv.write(NvItemId::NwkExtendedPanId, &[0xAA; 8]).unwrap();
        }
        let before = flash.data;
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Err(MigrationError::CorruptLegacy)
        );
        // Corruption must never wipe the region.
        assert_eq!(flash.data, before, "corrupt legacy must be preserved");
    }

    #[test]
    fn unparseable_region_is_corrupt_not_factory_new() {
        // Both sectors hold non-erased garbage that is neither LNV nor journal.
        let mut flash = MockFlash::erased();
        flash.data[0..8].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11, 0x22, 0x33]);
        flash.data[SECURITY_JOURNAL_SECTOR_SIZE..SECURITY_JOURNAL_SECTOR_SIZE + 8]
            .copy_from_slice(&[0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB]);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Err(MigrationError::CorruptLegacy)
        );
    }

    #[test]
    fn read_fault_is_hardware_not_factory_new() {
        let mut flash = MockFlash::erased();
        flash.reads_before_failure = Some(0);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Err(MigrationError::Hardware)
        );
    }

    #[test]
    fn both_counter_ranges_are_reserved_above_the_legacy_counter() {
        let live = 0x2000;
        let mut flash = legacy_on_network(live);
        migrate(&mut flash, SECTOR_A, SECTOR_B).unwrap();
        let state = loaded(flash).unwrap();
        let floor = live + LEGACY_FRAME_COUNTER_MARGIN + FRAME_COUNTER_RESERVATION_SIZE;
        // NWK security *and* the default-global-key APS traffic that shares
        // the NWK counter space.
        assert_eq!(state.global_counter_limit, floor);
        // The space a unique TCLK would use once the Trust Center delivers one.
        assert_eq!(state.tclk_counter_limit, floor);
        assert!(state.global_counter_limit > live);
        assert!(state.tclk_counter_limit > live);
    }

    #[test]
    fn near_max_legacy_counter_saturates_without_wrapping() {
        let mut flash = legacy_on_network(u32::MAX - 1);
        migrate(&mut flash, SECTOR_A, SECTOR_B).unwrap();
        let state = loaded(flash).unwrap();
        assert_eq!(state.global_counter_limit, u32::MAX);
        assert_eq!(state.tclk_counter_limit, u32::MAX);
    }

    #[test]
    fn migration_interrupted_before_the_commit_word_retries_from_legacy() {
        // Program 0 writes the record prefix; program 1 writes the `CMIT`
        // word. Failing the commit leaves a half-written journal record that
        // must not be mistaken for state — and must not have cost the legacy
        // page.
        let mut flash = legacy_on_network(0x2000);
        flash.programs_before_failure = Some(1);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Err(MigrationError::Journal(SecurityStoreError::Hardware))
        );
        assert_eq!(
            loaded_ref(&mut flash),
            None,
            "uncommitted record is ignored"
        );

        flash.programs_before_failure = None;
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::MigratedNetwork)
        );
        let state = loaded_ref(&mut flash).expect("migrated after retry");
        assert!(state.commissioned);
        assert_eq!(state.pan_id, 0x1234);
        assert_eq!(state.network_key, [0xCC; 16]);
        assert_eq!(
            state.global_counter_limit,
            0x2000 + LEGACY_FRAME_COUNTER_MARGIN + FRAME_COUNTER_RESERVATION_SIZE
        );
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::JournalPresent)
        );
    }

    #[test]
    fn interrupted_first_journal_commit_recovers_as_factory_new() {
        let mut flash = MockFlash::erased();
        flash.programs_before_failure = Some(1);
        let state = PersistentSecurityState::empty();
        assert_eq!(
            SecurityStateJournal::new(&mut flash, SECTOR_A, SECTOR_B).store(&state),
            Err(SecurityStoreError::Hardware)
        );
        assert_eq!(loaded_ref(&mut flash), None);

        flash.programs_before_failure = None;
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::FactoryNew)
        );

        // The normal first store can now erase the incomplete sector and
        // commit successfully instead of the device halting forever.
        let mut retry = PersistentSecurityState::empty();
        retry.global_counter_limit = FRAME_COUNTER_RESERVATION_SIZE;
        SecurityStateJournal::new(&mut flash, SECTOR_A, SECTOR_B)
            .store(&retry)
            .unwrap();
        assert_eq!(loaded_ref(&mut flash), Some(retry));
    }

    #[test]
    fn interrupted_after_first_journal_word_recovers_as_factory_new() {
        let mut flash = MockFlash::erased();
        // Minimum program unit landed, but power failed before version, state,
        // CRC, or commit. This is not a legacy page and must remain
        // self-healing on the next boot.
        flash.data[..JOURNAL_MAGIC.len()].copy_from_slice(&JOURNAL_MAGIC);

        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::FactoryNew)
        );

        let mut retry = PersistentSecurityState::empty();
        retry.global_counter_limit = FRAME_COUNTER_RESERVATION_SIZE;
        SecurityStateJournal::new(&mut flash, SECTOR_A, SECTOR_B)
            .store(&retry)
            .unwrap();
        assert_eq!(loaded_ref(&mut flash), Some(retry));
    }

    #[test]
    fn committed_but_corrupt_journal_is_not_treated_as_factory_new() {
        let mut flash = MockFlash::erased();
        let mut state = PersistentSecurityState::empty();
        state.global_counter_limit = FRAME_COUNTER_RESERVATION_SIZE;
        SecurityStateJournal::new(&mut flash, SECTOR_A, SECTOR_B)
            .store(&state)
            .unwrap();
        // Preserve the commit word but invalidate the record CRC.
        flash.data[92] ^= 1;

        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Err(MigrationError::CorruptLegacy)
        );
    }

    #[test]
    fn a_committed_journal_is_preferred_over_a_surviving_legacy_page() {
        // Steady state after a real migration: the legacy page still occupies
        // the other sector until the journal rolls over. Later commits (here,
        // a counter-reservation extension) must win over it on every boot.
        let mut flash = legacy_on_network(0x2000);
        migrate(&mut flash, SECTOR_A, SECTOR_B).unwrap();
        let mut state = loaded_ref(&mut flash).unwrap();
        state.global_counter_limit += FRAME_COUNTER_RESERVATION_SIZE;
        SecurityStateJournal::new(&mut flash, SECTOR_A, SECTOR_B)
            .store(&state)
            .unwrap();

        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::JournalPresent)
        );
        assert_eq!(
            loaded_ref(&mut flash),
            Some(state),
            "the migration must never re-seed a stale counter floor"
        );
    }

    #[test]
    fn reboot_after_migration_is_idempotent() {
        let mut flash = legacy_on_network(0x2000);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::MigratedNetwork)
        );
        let first = loaded_ref(&mut flash);

        // Subsequent boots find the journal and change nothing.
        for _ in 0..3 {
            assert_eq!(
                migrate(&mut flash, SECTOR_A, SECTOR_B),
                Ok(MigrationOutcome::JournalPresent)
            );
            assert_eq!(loaded_ref(&mut flash), first);
        }
    }

    #[test]
    fn interrupted_migration_retries_and_prefers_the_new_journal() {
        let mut flash = legacy_on_network(0x2000);
        // Fail the very first journal program (commit never lands).
        flash.programs_before_failure = Some(0);
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Err(MigrationError::Journal(SecurityStoreError::Hardware))
        );
        // No committed journal yet, and the legacy record is still intact.
        assert_eq!(loaded_ref(&mut flash), None);

        // Next boot completes the migration from the surviving legacy record.
        flash.programs_before_failure = None;
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::MigratedNetwork)
        );
        let state = loaded_ref(&mut flash).expect("migrated after retry");
        assert_eq!(state.pan_id, 0x1234);
        assert_eq!(
            state.global_counter_limit,
            0x2000 + LEGACY_FRAME_COUNTER_MARGIN + FRAME_COUNTER_RESERVATION_SIZE
        );

        // And a following boot is idempotent.
        assert_eq!(
            migrate(&mut flash, SECTOR_A, SECTOR_B),
            Ok(MigrationOutcome::JournalPresent)
        );
    }

    fn loaded_ref(flash: &mut MockFlash) -> Option<PersistentSecurityState> {
        let mut journal = SecurityStateJournal::new(&mut *flash, SECTOR_A, SECTOR_B);
        journal.load().expect("journal load")
    }
}
