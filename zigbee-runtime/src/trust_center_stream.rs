//! Streamed multi-slot Trust Center device journal (`ncp-capacity`).
//!
//! The coordinator NCP keeps up to 256 Trust Center devices (ZRHP spec 8.9).
//! A full version-6 snapshot is about 19.9 KiB, so it cannot use the
//! single-slot [`TrustCenterDeviceJournal`] (one 4 KiB record). This journal
//! streams one snapshot across consecutive 4 KiB chunk slots of a logical
//! sector (2 × 32 KiB on the ESP32-H2 NCP) without a full-size RAM buffer.
//!
//! # Chunk slot layout (4 096 bytes)
//!
//! | offset      | size | field                                              |
//! |-------------|------|----------------------------------------------------|
//! | 0           | 4    | magic `ZBTS`                                       |
//! | 4           | 1    | encoding version (Trust Center record version 7; 6 is still read) |
//! | 5           | 1    | chunk index `k`                                    |
//! | 6           | 1    | chunk count `n`                                    |
//! | 7           | 1    | reserved, 0                                        |
//! | 8           | 4    | generation                                         |
//! | 12          | 4    | total encoded length of the snapshot               |
//! | 16          | 2    | payload bytes in this chunk                        |
//! | 18          | 2    | reserved, 0                                        |
//! | 20          | ≤ 4 060 | payload (erased after the payload)              |
//! | 4 080       | 4    | CRC-32 of the whole encoded snapshot               |
//! | 4 084       | 4    | CRC-32 of bytes 0..4 084 of this chunk             |
//! | 4 088       | 4    | commit marker `CMIT`, **last chunk only**          |
//! | 4 092       | 4    | erased                                             |
//!
//! # Commit discipline
//!
//! A store writes the prefix (bytes 0..4 088) of every chunk first and the
//! single commit marker of the last chunk only after all of them. A snapshot
//! is authoritative only when chunks `0..n` sit in consecutive slots of one
//! sector, agree on generation, count and length, each chunk CRC holds, the
//! payload CRC matches the whole-snapshot CRC, the last chunk carries the
//! commit marker **and** the payload decodes to a valid state. Anything else
//! (a torn store, a bit error, an unknown version) is ignored and the newest
//! older complete snapshot stays current.
//!
//! New snapshots are appended after the last programmed slot of the active
//! sector. When they no longer fit, the other sector is erased and the
//! snapshot is written there from slot 0; the previous sector stays intact
//! until the new snapshot is committed, so a power cut during the erase or
//! the write keeps the previous complete state. Generations never wrap
//! (`GenerationExhausted`).
//!
//! Every successful store is verified by reading back each chunk and
//! comparing it byte for byte with a re-encoding of the state.
//!
//! # Compatibility
//!
//! A committed single-slot `ZBTC` record (versions 1..=6) written by
//! [`TrustCenterDeviceJournal`] at a 4 KiB-aligned slot is read as a
//! complete snapshot of its generation, so a product that previously placed
//! that journal at the start of each logical sector migrates on its next
//! store (the following generation is written in this format).
//! Firmware without `ncp-capacity` does not read `ZBTS` chunks and starts
//! with an empty Trust Center table; it never reuses an APS counter (the
//! global limit lives in this journal and in the security journal's
//! reservation, spec 8.4).
//!
//! # Endurance
//!
//! A snapshot of `d` devices uses `ceil((d · 75 + ~700) / 4 060)` slots, so
//! a 32 KiB sector holds 8 one-slot snapshots (≤ 44 devices), and a full
//! 256-device snapshot (5 slots) causes one 32 KiB erase per store. Products
//! with sustained Trust Center churn must budget erase cycles accordingly;
//! delta records (spec 8.9 "appends") are not implemented.

use embedded_storage::nor_flash::NorFlash;

use crate::trust_center_store::{
    MAX_ENCODED_TRUST_CENTER_STATE_LEN, NCP_COUNT_HIGH_RECORD_VERSION, PersistentTrustCenterState,
    RECORD_MAGIC, RECORD_VERSION, SnapshotSink, SnapshotSource, TRUST_CENTER_JOURNAL_SLOT_SIZE,
    TrustCenterDeviceJournal, TrustCenterDeviceStore, TrustCenterStoreError,
};

/// Size of one chunk slot (also the single-slot journal's record size).
pub const TRUST_CENTER_STREAM_SLOT_SIZE: usize = TRUST_CENTER_JOURNAL_SLOT_SIZE;
const SLOT: usize = TRUST_CENTER_STREAM_SLOT_SIZE;
const CHUNK_MAGIC: [u8; 4] = *b"ZBTS";
const HEADER_LEN: usize = 20;
const STREAM_CRC_OFFSET: usize = SLOT - 16;
const CHUNK_CRC_OFFSET: usize = SLOT - 12;
const PREFIX_LEN: usize = SLOT - 8;
const COMMIT_OFFSET: usize = SLOT - 8;
const COMMIT: [u8; 4] = *b"CMIT";
/// Payload bytes one chunk carries.
pub const TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK: usize = STREAM_CRC_OFFSET - HEADER_LEN;
/// Chunks of the largest snapshot.
pub const TRUST_CENTER_STREAM_MAX_CHUNKS: usize =
    MAX_ENCODED_TRUST_CENTER_STATE_LEN.div_ceil(TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK);
/// Smallest logical sector that holds the largest snapshot.
pub const TRUST_CENTER_STREAM_MIN_SECTOR_SIZE: usize = TRUST_CENTER_STREAM_MAX_CHUNKS * SLOT;
/// Read-back granularity for erased-slot checks and verification.
const READBACK: usize = 128;

const _: () = assert!(TRUST_CENTER_STREAM_MAX_CHUNKS <= u8::MAX as usize);
const _: () = assert!(SLOT.is_multiple_of(READBACK));
const _: () = assert!(MAX_ENCODED_TRUST_CENTER_STATE_LEN <= u32::MAX as usize);

/// Crash-safe Trust Center journal whose snapshots span several slots.
///
/// `SECTOR_SIZE` is each of the two logical sectors (a multiple of 4 KiB and
/// at least [`TRUST_CENTER_STREAM_MIN_SECTOR_SIZE`]). The journal caches only
/// the location of the current snapshot, not the state itself.
pub struct TrustCenterStreamJournal<S, const SECTOR_SIZE: usize> {
    storage: S,
    sectors: [u32; 2],
    current: Option<Located>,
    scanned: bool,
    /// `current` was decoded (load) or written and verified (store), not
    /// only framed: the other sector then holds only older generations.
    #[cfg(feature = "journal-pre-erase")]
    verified: bool,
    /// Pre-erase of the sector not holding `current`.
    #[cfg(feature = "journal-pre-erase")]
    spare: crate::spare_erase::SpareErase,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Located {
    generation: u32,
    sector: usize,
    slot: usize,
    /// Slots the snapshot occupies (1 for a single-slot `ZBTC` record).
    slots: usize,
}

/// Snapshot geometry shared by the writer, the verifier and the reader.
#[derive(Debug, Clone, Copy)]
struct Stream {
    /// Trust Center record version of the payload encoding.
    version: u8,
    generation: u32,
    total_len: usize,
    chunks: usize,
    crc: u32,
}

impl Stream {
    fn payload_len(&self, chunk: usize) -> usize {
        let start = chunk * TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK;
        (self.total_len - start).min(TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK)
    }
}

impl<S, const SECTOR_SIZE: usize> TrustCenterStreamJournal<S, SECTOR_SIZE> {
    /// Bytes in one logical sector.
    pub const SECTOR_SIZE: usize = SECTOR_SIZE;
    /// Chunk slots in one logical sector.
    pub const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / SLOT;

    /// Create a journal over two distinct logical sectors of `storage`.
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= TRUST_CENTER_STREAM_MIN_SECTOR_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(SLOT));
        Self {
            storage,
            sectors: [first_sector, second_sector],
            current: None,
            scanned: false,
            #[cfg(feature = "journal-pre-erase")]
            verified: false,
            #[cfg(feature = "journal-pre-erase")]
            spare: crate::spare_erase::SpareErase::new(),
        }
    }

    /// Record whether `current` is validated; an unvalidated one also
    /// voids every pre-erase.
    #[cfg(feature = "journal-pre-erase")]
    fn set_verified(&mut self, verified: bool) {
        self.verified = verified;
        if !verified {
            self.spare.forget();
        }
    }

    pub fn storage(&self) -> &S {
        &self.storage
    }

    pub fn into_storage(self) -> S {
        self.storage
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> TrustCenterStreamJournal<S, SECTOR_SIZE> {
    /// Store `state` after the current snapshot, or in the other sector
    /// when it does not fit or `rollover` asks for the move.
    fn store_snapshot(
        &mut self,
        state: &PersistentTrustCenterState,
        rollover: bool,
    ) -> Result<(), TrustCenterStoreError> {
        state.validate()?;
        let current = self.current()?;
        let generation = match current {
            Some(located) => located
                .generation
                .checked_add(1)
                .ok_or(TrustCenterStoreError::GenerationExhausted)?,
            None => 0,
        };
        let stream = Self::stream_for(state, generation)?;
        // Append after the current snapshot, or roll over: erase the other
        // sector (the current one stays intact until the new commit).
        let append = match current {
            Some(_) if rollover => None,
            Some(located) => Some(self.append_slot(located.sector)?)
                .filter(|slot| slot + stream.chunks <= Self::SLOTS_PER_SECTOR)
                .map(|slot| (located.sector, slot)),
            None => None,
        };
        let (sector, slot) = append.unwrap_or(match current {
            Some(located) => (1 - located.sector, 0),
            None => (0, 0),
        });
        let result = (|| {
            if append.is_none() {
                let start = self.sectors[sector];
                #[cfg(feature = "journal-pre-erase")]
                self.spare
                    .erase_for_rollover(&mut self.storage, sector, start, SECTOR_SIZE as u32)
                    .map_err(|_| TrustCenterStoreError::Hardware)?;
                #[cfg(not(feature = "journal-pre-erase"))]
                self.storage
                    .erase(start, start + SECTOR_SIZE as u32)
                    .map_err(|_| TrustCenterStoreError::Hardware)?;
            }
            self.write_snapshot(sector, slot, stream, state)
        })();
        match result {
            Ok(()) => {
                self.current = Some(Located {
                    generation,
                    sector,
                    slot,
                    slots: stream.chunks,
                });
                // Written and read back (`write_snapshot` verifies).
                #[cfg(feature = "journal-pre-erase")]
                self.set_verified(true);
                Ok(())
            }
            Err(error) => {
                self.current = None;
                self.scanned = false;
                #[cfg(feature = "journal-pre-erase")]
                self.set_verified(false);
                Err(error)
            }
        }
    }

    fn geometry_ok(&self) -> bool {
        self.sectors[0] != self.sectors[1]
            && self.sectors[0].abs_diff(self.sectors[1]) >= SECTOR_SIZE as u32
            && S::READ_SIZE != 0
            && S::WRITE_SIZE != 0
            && S::ERASE_SIZE != 0
            && READBACK.is_multiple_of(S::READ_SIZE)
            && SLOT.is_multiple_of(S::WRITE_SIZE)
            && PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            && COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            && SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            && self
                .sectors
                .iter()
                .all(|sector| (*sector as usize).is_multiple_of(S::ERASE_SIZE))
            && self.sectors.iter().all(|sector| {
                (*sector as usize)
                    .checked_add(SECTOR_SIZE)
                    .is_some_and(|end| end <= self.storage.capacity())
            })
    }

    fn slot_address(&self, sector: usize, slot: usize) -> u32 {
        self.sectors[sector] + (slot * SLOT) as u32
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        output: &mut [u8; SLOT],
    ) -> Result<(), TrustCenterStoreError> {
        let address = self.slot_address(sector, slot);
        self.storage
            .read(address, output)
            .map_err(|_| TrustCenterStoreError::Hardware)
    }

    fn slot_erased(&mut self, sector: usize, slot: usize) -> Result<bool, TrustCenterStoreError> {
        let base = self.slot_address(sector, slot);
        let mut chunk = [0u8; READBACK];
        for offset in (0..SLOT).step_by(READBACK) {
            self.storage
                .read(base + offset as u32, &mut chunk)
                .map_err(|_| TrustCenterStoreError::Hardware)?;
            if chunk.iter().any(|byte| *byte != 0xFF) {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// First slot after the last programmed slot of `sector`. Slots past it
    /// are erased; a torn snapshot below it is never written over.
    fn append_slot(&mut self, sector: usize) -> Result<usize, TrustCenterStoreError> {
        for slot in (0..Self::SLOTS_PER_SECTOR).rev() {
            if !self.slot_erased(sector, slot)? {
                return Ok(slot + 1);
            }
        }
        Ok(0)
    }

    /// Validate the framing and CRCs of the snapshot starting at
    /// `(sector, slot)` without decoding it: every chunk's framing and CRC,
    /// the commit marker and the whole-snapshot CRC (a single-slot record:
    /// its framing and CRC). `Ok(None)` for anything that is not a complete,
    /// committed snapshot; `Err` only for a storage failure.
    fn frame_snapshot(
        &mut self,
        sector: usize,
        slot: usize,
    ) -> Result<Option<Located>, TrustCenterStoreError> {
        let mut buffer = [0u8; SLOT];
        self.read_slot(sector, slot, &mut buffer)?;
        if buffer[0..4] == RECORD_MAGIC {
            return Ok(
                TrustCenterDeviceJournal::<S>::frame_record(&buffer).map(|generation| Located {
                    generation,
                    sector,
                    slot,
                    slots: 1,
                }),
            );
        }
        let Some(stream) = chunk_header(&buffer, 0) else {
            return Ok(None);
        };
        if slot + stream.chunks > Self::SLOTS_PER_SECTOR {
            return Ok(None);
        }
        let mut crc = CRC_INIT;
        for chunk in 0..stream.chunks {
            if chunk != 0 {
                self.read_slot(sector, slot + chunk, &mut buffer)?;
            }
            if !chunk_matches(&buffer, &stream, chunk) {
                return Ok(None);
            }
            crc = crc32_update(
                crc,
                &buffer[HEADER_LEN..HEADER_LEN + stream.payload_len(chunk)],
            );
        }
        if !crc != stream.crc {
            return Ok(None);
        }
        Ok(Some(Located {
            generation: stream.generation,
            sector,
            slot,
            slots: stream.chunks,
        }))
    }

    /// Decode the framed snapshot at `located` into `state` in place,
    /// straight from flash. `Ok(false)` (with `state` unspecified) when it
    /// does not decode to a valid state or no longer matches `located`.
    fn decode_snapshot(
        &mut self,
        located: Located,
        state: &mut PersistentTrustCenterState,
    ) -> Result<bool, TrustCenterStoreError> {
        let mut buffer = [0u8; SLOT];
        self.read_slot(located.sector, located.slot, &mut buffer)?;
        if buffer[0..4] == RECORD_MAGIC {
            return Ok(located.slots == 1
                && TrustCenterDeviceJournal::<S>::decode_record_into(&buffer, state)
                    == Some(located.generation));
        }
        let Some(stream) = chunk_header(&buffer, 0) else {
            return Ok(false);
        };
        if stream.generation != located.generation || stream.chunks != located.slots {
            return Ok(false);
        }
        let mut source = ChunkSource {
            journal: self,
            sector: located.sector,
            slot: located.slot,
            stream,
            position: 0,
            loaded: None,
            buffer: &mut buffer,
            error: None,
        };
        let version = stream.version;
        let decoded = state.decode_source_version_into(version, &mut source);
        if let Some(error) = source.error {
            return Err(error);
        }
        Ok(decoded.is_some_and(|result| result.is_ok()))
    }

    /// Newest complete snapshot by framing and CRCs (no decode), with a
    /// generation below `below` when given. Only candidates newer than the
    /// best so far are validated; a slot's header alone selects candidates.
    fn newest_framed(
        &mut self,
        below: Option<u32>,
    ) -> Result<Option<Located>, TrustCenterStoreError> {
        let mut best: Option<Located> = None;
        let mut header = [0u8; READBACK];
        for sector in 0..2 {
            let mut slot = 0;
            while slot < Self::SLOTS_PER_SECTOR {
                let address = self.slot_address(sector, slot);
                self.storage
                    .read(address, &mut header)
                    .map_err(|_| TrustCenterStoreError::Hardware)?;
                let candidate = candidate_generation(&header).filter(|generation| {
                    below.is_none_or(|bound| *generation < bound)
                        && best.is_none_or(|current| *generation > current.generation)
                });
                if candidate.is_some()
                    && let Some(located) = self.frame_snapshot(sector, slot)?
                {
                    best = Some(located);
                    slot += located.slots;
                    continue;
                }
                slot += 1;
            }
        }
        Ok(best)
    }

    /// The current snapshot for a store. Without a prior
    /// [`TrustCenterDeviceStore::load_into`] scan, the newest complete
    /// snapshot by framing and CRCs: its generation is at least that of the
    /// newest decodable one, so the next snapshot is still ordered after
    /// every survivor.
    fn current(&mut self) -> Result<Option<Located>, TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        if !self.scanned {
            self.current = self.newest_framed(None)?;
            self.scanned = true;
            #[cfg(feature = "journal-pre-erase")]
            self.set_verified(false);
        }
        Ok(self.current)
    }

    fn stream_for(
        state: &PersistentTrustCenterState,
        generation: u32,
    ) -> Result<Stream, TrustCenterStoreError> {
        let total_len = state.encoded_len();
        let chunks = total_len.div_ceil(TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK);
        if total_len == 0 || chunks > TRUST_CENTER_STREAM_MAX_CHUNKS {
            return Err(TrustCenterStoreError::Full);
        }
        let mut crc = CrcSink(CRC_INIT);
        if state.encode_into(&mut crc)? != total_len {
            return Err(TrustCenterStoreError::Corrupt);
        }
        Ok(Stream {
            version: RECORD_VERSION,
            generation,
            total_len,
            chunks,
            crc: !crc.0,
        })
    }

    /// Program (`verify == false`) or byte-compare (`verify == true`) the
    /// chunks of `state` at `(sector, slot..)`.
    fn run_chunks(
        &mut self,
        sector: usize,
        slot: usize,
        stream: Stream,
        state: &PersistentTrustCenterState,
        verify: bool,
    ) -> Result<(), TrustCenterStoreError> {
        let mut writer = ChunkWriter {
            journal: self,
            sector,
            slot,
            stream,
            chunk: 0,
            fill: 0,
            verify,
            buffer: [0xFF; SLOT],
        };
        let written = state.encode_into(&mut writer)?;
        if written != stream.total_len || writer.fill != 0 || writer.chunk != stream.chunks {
            return Err(TrustCenterStoreError::Hardware);
        }
        Ok(())
    }

    fn write_snapshot(
        &mut self,
        sector: usize,
        slot: usize,
        stream: Stream,
        state: &PersistentTrustCenterState,
    ) -> Result<(), TrustCenterStoreError> {
        self.run_chunks(sector, slot, stream, state, false)?;
        // The single commit marker, after every chunk prefix. TLSR8258-style
        // flash can only program from SRAM; stage it on the stack.
        let marker = COMMIT;
        let last = self.slot_address(sector, slot + stream.chunks - 1);
        self.storage
            .write(last + COMMIT_OFFSET as u32, &marker)
            .map_err(|_| TrustCenterStoreError::Hardware)?;
        self.run_chunks(sector, slot, stream, state, true)
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> TrustCenterDeviceStore
    for TrustCenterStreamJournal<S, SECTOR_SIZE>
{
    fn load_into(
        &mut self,
        state: &mut PersistentTrustCenterState,
    ) -> Result<bool, TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        if self.scanned {
            let Some(located) = self.current else {
                return Ok(false);
            };
            if self.frame_snapshot(located.sector, located.slot)? == Some(located)
                && self.decode_snapshot(located, state)?
            {
                return Ok(true);
            }
            // The flash changed under the cache: rescan on the next call.
            self.current = None;
            self.scanned = false;
            #[cfg(feature = "journal-pre-erase")]
            self.set_verified(false);
            return Err(TrustCenterStoreError::Corrupt);
        }
        // Newest decodable snapshot: complete snapshots newest first, with
        // `state` as the decode scratch; one that does not decode falls back
        // to the next older one.
        let mut below = None;
        loop {
            let Some(located) = self.newest_framed(below)? else {
                self.current = None;
                self.scanned = true;
                #[cfg(feature = "journal-pre-erase")]
                self.set_verified(false);
                return Ok(false);
            };
            if self.decode_snapshot(located, state)? {
                self.current = Some(located);
                self.scanned = true;
                #[cfg(feature = "journal-pre-erase")]
                self.set_verified(true);
                return Ok(true);
            }
            below = Some(located.generation);
        }
    }

    fn store(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        self.store_snapshot(state, false)
    }

    #[cfg(feature = "journal-pre-erase")]
    fn compact(&mut self, state: &PersistentTrustCenterState) -> Result<(), TrustCenterStoreError> {
        self.store_snapshot(state, true)
    }

    fn clear(&mut self) -> Result<(), TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        self.current = None;
        self.scanned = false;
        #[cfg(feature = "journal-pre-erase")]
        self.set_verified(false);
        for sector in self.sectors {
            self.storage
                .erase(sector, sector + SECTOR_SIZE as u32)
                .map_err(|_| TrustCenterStoreError::Hardware)?;
        }
        self.scanned = true;
        Ok(())
    }

    /// Pre-erase the sector that does not hold the current, validated
    /// snapshot (it only holds older generations), one erase unit per call.
    #[cfg(feature = "journal-pre-erase")]
    fn pre_erase_step(&mut self) -> Result<bool, TrustCenterStoreError> {
        if !self.geometry_ok() {
            return Err(TrustCenterStoreError::Hardware);
        }
        let Some(current) = self.current.filter(|_| self.scanned && self.verified) else {
            return Ok(false);
        };
        let spare = 1 - current.sector;
        let start = self.sectors[spare];
        self.spare
            .step(&mut self.storage, spare, start, SECTOR_SIZE as u32)
            .map_err(|_| TrustCenterStoreError::Hardware)
    }

    /// Half of the current sector is used, the spare is fully pre-erased
    /// and moving would gain room (the snapshot is not already first in
    /// its sector). A join stores about ten snapshots; a sector holds eight.
    #[cfg(feature = "journal-pre-erase")]
    fn wants_compaction(&self) -> bool {
        self.current
            .filter(|_| self.scanned && self.verified)
            .is_some_and(|current| {
                current.slot > 0
                    && (current.slot + current.slots) * 2 >= Self::SLOTS_PER_SECTOR
                    && self
                        .spare
                        .is_complete(1 - current.sector, SECTOR_SIZE as u32)
            })
    }
}

/// Generation of a slot that may start a snapshot (a legacy record or chunk
/// 0), from its header only; full validation follows.
fn candidate_generation(slot: &[u8]) -> Option<u32> {
    if slot[0..4] == RECORD_MAGIC
        || (slot[0..4] == CHUNK_MAGIC && readable_version(slot[4]) && slot[5] == 0)
    {
        Some(u32::from_le_bytes([slot[8], slot[9], slot[10], slot[11]]))
    } else {
        None
    }
}

/// Snapshot geometry announced by chunk `index`'s header (no CRC check).
fn chunk_header(slot: &[u8; SLOT], index: usize) -> Option<Stream> {
    if slot[0..4] != CHUNK_MAGIC || !readable_version(slot[4]) || usize::from(slot[5]) != index {
        return None;
    }
    let chunks = usize::from(slot[6]);
    let total_len = u32::from_le_bytes([slot[12], slot[13], slot[14], slot[15]]) as usize;
    if chunks == 0
        || chunks > TRUST_CENTER_STREAM_MAX_CHUNKS
        || total_len == 0
        || total_len > MAX_ENCODED_TRUST_CENTER_STATE_LEN
        || total_len.div_ceil(TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK) != chunks
    {
        return None;
    }
    Some(Stream {
        version: slot[4],
        generation: u32::from_le_bytes([slot[8], slot[9], slot[10], slot[11]]),
        total_len,
        chunks,
        crc: u32::from_le_bytes(word(slot, STREAM_CRC_OFFSET)),
    })
}

/// Whether `slot` is chunk `index` of `stream`, intact and (for the last
/// chunk) committed.
fn chunk_matches(slot: &[u8; SLOT], stream: &Stream, index: usize) -> bool {
    let Some(header) = chunk_header(slot, index) else {
        return false;
    };
    let payload_len = stream.payload_len(index);
    let last = index + 1 == stream.chunks;
    header.version == stream.version
        && header.generation == stream.generation
        && header.total_len == stream.total_len
        && header.chunks == stream.chunks
        && header.crc == stream.crc
        && slot[7] == 0
        && usize::from(u16::from_le_bytes([slot[16], slot[17]])) == payload_len
        && slot[18..20] == [0, 0]
        && slot[HEADER_LEN + payload_len..STREAM_CRC_OFFSET]
            .iter()
            .all(|byte| *byte == 0xFF)
        && u32::from_le_bytes(word(slot, CHUNK_CRC_OFFSET))
            == crate::security_journal::crc32(&slot[..CHUNK_CRC_OFFSET])
        && (!last || slot[COMMIT_OFFSET..COMMIT_OFFSET + 4] == COMMIT)
}

/// Chunk encodings this build reads: the current one, and version 6 (before
/// the EZSP hashed link-key root), so an update keeps the Trust Center
/// table; the next store writes the current version.
fn readable_version(version: u8) -> bool {
    version == RECORD_VERSION || version == NCP_COUNT_HIGH_RECORD_VERSION
}

fn word(slot: &[u8; SLOT], offset: usize) -> [u8; 4] {
    [
        slot[offset],
        slot[offset + 1],
        slot[offset + 2],
        slot[offset + 3],
    ]
}

/// Incremental form of `security_journal::crc32` (same polynomial):
/// `!crc32_update(CRC_INIT, data) == crc32(data)`.
const CRC_INIT: u32 = 0xFFFF_FFFF;

fn crc32_update(mut crc: u32, data: &[u8]) -> u32 {
    for byte in data {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    crc
}

struct CrcSink(u32);

impl SnapshotSink for CrcSink {
    fn put(&mut self, bytes: &[u8]) -> Result<(), TrustCenterStoreError> {
        self.0 = crc32_update(self.0, bytes);
        Ok(())
    }
}

/// Builds each chunk image from the encoder's output, then programs its
/// prefix or compares the whole slot with flash.
struct ChunkWriter<'a, S, const SECTOR_SIZE: usize> {
    journal: &'a mut TrustCenterStreamJournal<S, SECTOR_SIZE>,
    sector: usize,
    slot: usize,
    stream: Stream,
    chunk: usize,
    fill: usize,
    verify: bool,
    buffer: [u8; SLOT],
}

impl<S: NorFlash, const SECTOR_SIZE: usize> ChunkWriter<'_, S, SECTOR_SIZE> {
    fn finish_chunk(&mut self) -> Result<(), TrustCenterStoreError> {
        let index = self.chunk;
        if index >= self.stream.chunks || self.fill != self.stream.payload_len(index) {
            return Err(TrustCenterStoreError::Hardware);
        }
        let buffer = &mut self.buffer;
        buffer[0..4].copy_from_slice(&CHUNK_MAGIC);
        buffer[4] = self.stream.version;
        buffer[5] = index as u8;
        buffer[6] = self.stream.chunks as u8;
        buffer[7] = 0;
        buffer[8..12].copy_from_slice(&self.stream.generation.to_le_bytes());
        buffer[12..16].copy_from_slice(&(self.stream.total_len as u32).to_le_bytes());
        buffer[16..18].copy_from_slice(&(self.fill as u16).to_le_bytes());
        buffer[18..20].fill(0);
        buffer[STREAM_CRC_OFFSET..STREAM_CRC_OFFSET + 4]
            .copy_from_slice(&self.stream.crc.to_le_bytes());
        let crc = crate::security_journal::crc32(&buffer[..CHUNK_CRC_OFFSET]);
        buffer[CHUNK_CRC_OFFSET..CHUNK_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        let address = self.journal.slot_address(self.sector, self.slot + index);
        if self.verify {
            if index + 1 == self.stream.chunks {
                buffer[COMMIT_OFFSET..COMMIT_OFFSET + 4].copy_from_slice(&COMMIT);
            }
            let mut read = [0u8; READBACK];
            for (offset, expected) in buffer.chunks_exact(READBACK).enumerate() {
                self.journal
                    .storage
                    .read(address + (offset * READBACK) as u32, &mut read)
                    .map_err(|_| TrustCenterStoreError::Hardware)?;
                if read != *expected {
                    return Err(TrustCenterStoreError::Hardware);
                }
            }
        } else {
            self.journal
                .storage
                .write(address, &buffer[..PREFIX_LEN])
                .map_err(|_| TrustCenterStoreError::Hardware)?;
        }
        self.buffer = [0xFF; SLOT];
        self.fill = 0;
        self.chunk += 1;
        Ok(())
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> SnapshotSink for ChunkWriter<'_, S, SECTOR_SIZE> {
    fn put(&mut self, mut bytes: &[u8]) -> Result<(), TrustCenterStoreError> {
        while !bytes.is_empty() {
            if self.chunk >= self.stream.chunks {
                return Err(TrustCenterStoreError::Full);
            }
            let capacity = self.stream.payload_len(self.chunk);
            let take = bytes.len().min(capacity - self.fill);
            self.buffer[HEADER_LEN + self.fill..HEADER_LEN + self.fill + take]
                .copy_from_slice(&bytes[..take]);
            self.fill += take;
            bytes = &bytes[take..];
            if self.fill == capacity {
                self.finish_chunk()?;
            }
        }
        Ok(())
    }
}

/// Reads a validated snapshot's payload chunk by chunk (one slot buffer).
struct ChunkSource<'a, S, const SECTOR_SIZE: usize> {
    journal: &'a mut TrustCenterStreamJournal<S, SECTOR_SIZE>,
    sector: usize,
    slot: usize,
    stream: Stream,
    position: usize,
    loaded: Option<usize>,
    buffer: &'a mut [u8; SLOT],
    /// A storage error is reported as `Hardware`, never as corruption.
    error: Option<TrustCenterStoreError>,
}

impl<S: NorFlash, const SECTOR_SIZE: usize> ChunkSource<'_, S, SECTOR_SIZE> {
    /// Payload byte at absolute stream offset `index`.
    fn at(&mut self, index: usize) -> Result<u8, TrustCenterStoreError> {
        if index >= self.stream.total_len {
            return Err(TrustCenterStoreError::Corrupt);
        }
        let chunk = index / TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK;
        if self.loaded != Some(chunk) {
            self.loaded = None;
            if let Err(error) = self
                .journal
                .read_slot(self.sector, self.slot + chunk, self.buffer)
            {
                self.error = Some(error);
                return Err(error);
            }
            // Re-validate: the flash must not have changed since the scan.
            if !chunk_matches(self.buffer, &self.stream, chunk) {
                return Err(TrustCenterStoreError::Corrupt);
            }
            self.loaded = Some(chunk);
        }
        Ok(self.buffer[HEADER_LEN + index % TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK])
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> SnapshotSource for ChunkSource<'_, S, SECTOR_SIZE> {
    fn remaining(&self) -> usize {
        self.stream.total_len - self.position
    }

    fn take(&mut self, out: &mut [u8]) -> Result<(), TrustCenterStoreError> {
        if out.len() > self.remaining() {
            return Err(TrustCenterStoreError::Corrupt);
        }
        for byte in out.iter_mut() {
            *byte = self.at(self.position)?;
            self.position += 1;
        }
        Ok(())
    }

    fn byte_at(&mut self, index: usize) -> Result<u8, TrustCenterStoreError> {
        let absolute = self
            .position
            .checked_add(index)
            .ok_or(TrustCenterStoreError::Corrupt)?;
        self.at(absolute)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trust_center_store::{
        MAX_APPLICATION_KEY_REQUESTS, MAX_PENDING_UNKNOWN_REMOVALS, NetworkKeyRotation,
        NetworkKeyRotationPhase, PendingApplicationLinkKey, PendingTrustCenterLinkKey,
        PendingUnknownRemoval, PersistentTrustCenterDevice, TRUST_CENTER_JOURNAL_SECTOR_SIZE,
    };
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use std::vec;
    use std::vec::Vec;
    use zigbee_bdb::attributes::NetworkKeyUpdateMethod;
    use zigbee_bdb::trust_center::{
        MAX_TRUST_CENTER_DEVICES, TrustCenterDevice, TrustCenterKeyAttributes, TrustCenterKeyOrigin,
    };
    use zigbee_types::ShortAddress;

    const SECTOR: usize = 32 * 1024;
    type Journal = TrustCenterStreamJournal<MockFlash, SECTOR>;

    #[derive(Debug)]
    struct MockError;

    impl embedded_storage::nor_flash::NorFlashError for MockError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    /// Two 32 KiB sectors. A power cut is simulated by `cut_after`: the
    /// operation that exhausts it programs/erases only its first half and
    /// fails, and every later operation fails.
    struct MockFlash {
        bytes: Vec<u8>,
        cut_after: Option<usize>,
        programs: usize,
        erases: usize,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                bytes: vec![0xFF; 2 * SECTOR],
                cut_after: None,
                programs: 0,
                erases: 0,
            }
        }

        /// Returns `true` when the operation may run completely.
        fn step(&mut self) -> Result<bool, MockError> {
            match self.cut_after.as_mut() {
                None => Ok(true),
                Some(0) => Err(MockError),
                Some(remaining) => {
                    *remaining -= 1;
                    Ok(*remaining != 0)
                }
            }
        }

        fn power_on(mut self) -> Self {
            self.cut_after = None;
            self
        }
    }

    impl ErrorType for MockFlash {
        type Error = MockError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let source = self
                .bytes
                .get(start..start + bytes.len())
                .ok_or(MockError)?;
            bytes.copy_from_slice(source);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = 4096;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            let complete = self.step()?;
            self.erases += 1;
            let (from, to) = (from as usize, to as usize);
            let end = if complete { to } else { from + (to - from) / 2 };
            self.bytes.get_mut(from..end).ok_or(MockError)?.fill(0xFF);
            if complete { Ok(()) } else { Err(MockError) }
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let complete = self.step()?;
            self.programs += 1;
            let start = offset as usize;
            let len = if complete {
                bytes.len()
            } else {
                bytes.len() / 2
            };
            let destination = self.bytes.get_mut(start..start + len).ok_or(MockError)?;
            for (dst, src) in destination.iter_mut().zip(bytes) {
                if (*dst & *src) != *src {
                    return Err(MockError);
                }
                *dst &= *src;
            }
            if complete { Ok(()) } else { Err(MockError) }
        }
    }

    fn ieee(index: usize) -> [u8; 8] {
        let [lo, hi] = (index as u16).to_le_bytes();
        [lo, hi, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x01]
    }

    fn device(index: usize, tag: u8) -> PersistentTrustCenterDevice {
        PersistentTrustCenterDevice {
            device: TrustCenterDevice {
                ieee_address: ieee(index),
                parent_address: [0x33; 8],
                short_address: ShortAddress(0x1000 + index as u16),
                link_key: [tag ^ index as u8; 16],
                key_origin: TrustCenterKeyOrigin::GeneratedUnique,
                key_attributes: TrustCenterKeyAttributes::Verified,
                join_timeout_remaining_secs: 0,
                active: true,
            },
            outgoing_frame_counter: 0x10,
            outgoing_frame_counter_limit: 0x410,
            incoming_frame_counter: u32::from(tag) << 8 | index as u32,
            incoming_frame_counter_valid: true,
            network_key_pending: false,
            confirm_key_pending: None,
            removal_pending: false,
            new_network_key_delivered: index.is_multiple_of(7),
            is_router: Some(index.is_multiple_of(3)),
            pending_link_key: index
                .is_multiple_of(11)
                .then_some(PendingTrustCenterLinkKey {
                    key: [0xC0 | tag; 16],
                    outgoing_frame_counter: 1,
                    outgoing_frame_counter_limit: 0x401,
                    transported: index.is_multiple_of(2),
                }),
        }
    }

    /// `devices` devices; with `full`, every bounded table is at capacity.
    fn state(devices: usize, tag: u8, full: bool) -> PersistentTrustCenterState {
        let mut state = PersistentTrustCenterState::new([0x11; 8]);
        state.set_rotation(Some(NetworkKeyRotation {
            target_sequence: tag,
            phase: NetworkKeyRotationPhase::Distributing,
            method: NetworkKeyUpdateMethod::Broadcast,
        }));
        for index in 0..devices {
            state.upsert(device(index, tag)).unwrap();
        }
        assert!(state.set_global_aps_limit(0x0001_0000 + u32::from(tag)));
        if full {
            for index in 0..MAX_APPLICATION_KEY_REQUESTS {
                state
                    .allow_application_key_request(ieee(index), ieee(index + 1))
                    .unwrap();
            }
            state.set_pending_application_key(Some(PendingApplicationLinkKey {
                initiator_address: ieee(0),
                responder_address: ieee(1),
                key: [0xAB; 16],
                delivered_to_initiator: true,
                delivered_to_responder: false,
            }));
            for index in 0..MAX_PENDING_UNKNOWN_REMOVALS {
                let mut device_address = [0x99; 8];
                device_address[0] = index as u8;
                state
                    .stage_unknown_removal(PendingUnknownRemoval {
                        parent_address: [0x44; 8],
                        device_address,
                    })
                    .unwrap();
            }
        }
        state.validate().unwrap();
        state
    }

    fn journal(flash: MockFlash) -> Journal {
        Journal::new(flash, 0, SECTOR as u32)
    }

    fn reopen(journal: Journal) -> Journal {
        self::journal(journal.into_storage().power_on())
    }

    impl Journal {
        fn storage_mut_for_test(&mut self) -> &mut MockFlash {
            &mut self.storage
        }
    }

    /// `journal-pre-erase`: once the current snapshot is validated, the
    /// other sector is erased ahead of time, one 4 KiB unit per step, and the
    /// rollover no longer erases (the radio-deaf 8 × 4 KiB erase leaves the
    /// store path).
    #[test]
    #[cfg(feature = "journal-pre-erase")]
    fn a_pre_erased_spare_sector_takes_the_erase_out_of_the_rollover() {
        let mut journal = journal(MockFlash::new());
        journal.store(&state(1, 0, false)).unwrap();
        let mut steps = 0;
        while journal.pre_erase_step().unwrap() {
            steps += 1;
        }
        assert_eq!(steps, SECTOR / 4096);
        assert!(!journal.pre_erase_step().unwrap(), "nothing left");
        let erases = journal.storage().erases;
        // Seven appends fill sector 0; the ninth store rolls over.
        for tag in 1..=8 {
            journal.store(&state(1, tag, false)).unwrap();
        }
        assert_eq!(journal.current.map(|located| located.sector), Some(1));
        assert_eq!(
            journal.storage().erases,
            erases,
            "rollover without an erase"
        );

        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(state(1, 8, false))));
    }

    /// Compaction is asked for once half of the current sector is used and
    /// the spare is fully pre-erased; re-storing then rolls over without an
    /// erase and leaves a free sector, and the request stops until the new
    /// spare is pre-erased again.
    #[test]
    #[cfg(feature = "journal-pre-erase")]
    fn a_half_used_sector_compacts_into_the_pre_erased_spare() {
        let slots = Journal::SLOTS_PER_SECTOR;
        let mut journal = journal(MockFlash::new());
        for tag in 0..slots as u8 / 2 {
            journal.store(&state(3, tag, false)).unwrap();
        }
        assert!(!journal.wants_compaction(), "spare not pre-erased yet");
        while journal.pre_erase_step().unwrap() {}
        assert!(journal.wants_compaction(), "half used, spare erased");
        let reopened = reopen(journal);
        assert!(!reopened.wants_compaction(), "progress is RAM-only");
        let mut journal = reopened;
        let current = state(3, slots as u8 / 2 - 1, false);
        assert_eq!(journal.load(), Ok(Some(current.clone())));
        while journal.pre_erase_step().unwrap() {}
        assert!(journal.wants_compaction());
        // A power cut during the move keeps the current snapshot.
        journal.storage_mut_for_test().cut_after = Some(1);
        assert!(journal.compact(&current).is_err());
        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(current.clone())));
        while journal.pre_erase_step().unwrap() {}
        assert!(journal.wants_compaction());

        let erases = journal.storage().erases;
        journal.compact(&current).unwrap();
        assert_eq!(journal.storage().erases, erases, "no erase in the move");
        assert_eq!(journal.current.map(|located| located.sector), Some(1));
        assert!(!journal.wants_compaction(), "first slot of a fresh sector");
        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(current)));
        // A whole free sector minus one slot, then a pre-erased spare.
        while journal.pre_erase_step().unwrap() {}
        let erases = journal.storage().erases;
        for tag in 0x40..0x40 + slots as u8 {
            journal.store(&state(3, tag, false)).unwrap();
        }
        assert_eq!(journal.storage().erases, erases, "{slots} stores, no erase");
    }

    /// A snapshot that is only framed (no load yet) does not authorize a
    /// pre-erase, a reboot forgets the progress (the rollover erases fully
    /// again), and a power cut in the middle of a pre-erase leaves the
    /// current snapshot intact.
    #[test]
    #[cfg(feature = "journal-pre-erase")]
    fn pre_erase_needs_a_validated_snapshot_and_survives_a_power_cut() {
        let mut journal = journal(MockFlash::new());
        journal.store(&state(2, 1, false)).unwrap();
        let mut journal = reopen(journal);
        assert_eq!(journal.pre_erase_step(), Ok(false), "not validated yet");
        assert_eq!(journal.load(), Ok(Some(state(2, 1, false))));

        journal.pre_erase_step().unwrap();
        journal.storage_mut_for_test().cut_after = Some(1);
        assert!(journal.pre_erase_step().is_err(), "power cut mid-erase");
        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(state(2, 1, false))));
        let erases = journal.storage().erases;
        for tag in 2..=9 {
            journal.store(&state(2, tag, false)).unwrap();
        }
        assert_eq!(
            journal.storage().erases,
            erases + 1,
            "after a reboot the rollover erases the whole sector"
        );
        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(state(2, 9, false))));
    }

    #[test]
    fn a_full_256_device_snapshot_streams_across_five_chunks_and_round_trips() {
        // 256 devices in five chunks; `ncp-capacity-compact`: 128 in three.
        let (devices, chunks) = if cfg!(feature = "ncp-capacity-compact") {
            (128, 3)
        } else {
            (256, 5)
        };
        assert_eq!(MAX_TRUST_CENTER_DEVICES, devices);
        let full = state(MAX_TRUST_CENTER_DEVICES, 1, true);
        assert_eq!(full.encoded_len(), MAX_ENCODED_TRUST_CENTER_STATE_LEN);
        assert_eq!(TRUST_CENTER_STREAM_MAX_CHUNKS, chunks);
        const { assert!(TRUST_CENTER_STREAM_MIN_SECTOR_SIZE <= SECTOR) };

        let mut journal = journal(MockFlash::new());
        assert_eq!(journal.load(), Ok(None), "erased flash holds no state");
        journal.store(&full).unwrap();
        assert_eq!(
            journal.current,
            Some(Located {
                generation: 0,
                sector: 0,
                slot: 0,
                slots: chunks
            })
        );
        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(full.clone())));
        assert_eq!(journal.load().unwrap().unwrap().devices().len(), devices);
        // One more device than the table holds is refused before flash work.
        let mut too_many = full;
        assert_eq!(
            too_many.upsert(device(MAX_TRUST_CENTER_DEVICES, 1)),
            Err(TrustCenterStoreError::Full)
        );
    }

    #[test]
    fn snapshots_append_then_roll_over_keeping_the_previous_sector_until_commit() {
        let mut journal = journal(MockFlash::new());
        // 2-chunk snapshots: 4 fit one 32 KiB sector.
        for generation in 0..12u8 {
            let expected = state(60, generation, false);
            journal.store(&expected).unwrap();
            let located = journal.current.unwrap();
            assert_eq!(located.slots, 2);
            assert_eq!(located.generation, u32::from(generation));
            assert_eq!(located.sector, usize::from(generation / 4) % 2);
            assert_eq!(located.slot, usize::from(generation % 4) * 2);
            journal = reopen(journal);
            assert_eq!(journal.load(), Ok(Some(expected)));
        }
        // Erase count: initial + one per rollover.
        assert_eq!(journal.storage().erases, 3);
    }

    #[test]
    fn a_power_cut_at_any_program_or_erase_keeps_the_previous_or_the_new_state() {
        // Initial state fills sector 0 so the store under test rolls over
        // (erase + 5 chunk prefixes + commit), then repeat for an append.
        for (base_devices, stores_before) in [(MAX_TRUST_CENTER_DEVICES, 1usize), (10, 1)] {
            let mut operation = 0;
            loop {
                let mut journal = journal(MockFlash::new());
                let mut previous = None;
                for generation in 0..stores_before {
                    let base = state(base_devices, generation as u8, false);
                    journal.store(&base).unwrap();
                    previous = Some(base);
                }
                let next = state(MAX_TRUST_CENTER_DEVICES, 0x80, true);
                let mut flash = journal.into_storage();
                flash.cut_after = Some(operation + 1);
                let mut journal = self::journal(flash);
                let result = journal.store(&next);
                let mut journal = reopen(journal);
                let loaded = journal.load().unwrap();
                if result.is_ok() {
                    assert_eq!(loaded, Some(next), "operation {operation}");
                    break;
                }
                assert!(
                    loaded == previous || loaded.as_ref() == Some(&next),
                    "operation {operation}: a torn snapshot must not become current"
                );
                // A retry after the cut appends past the torn chunks.
                let retry = state(5, 0x55, false);
                journal.store(&retry).unwrap();
                assert_eq!(reopen(journal).load(), Ok(Some(retry)));
                operation += 1;
                assert!(operation < 64, "store never completed");
            }
        }
    }

    #[test]
    fn a_missing_commit_marker_or_bad_crc_falls_back_to_the_previous_snapshot() {
        let older = state(MAX_TRUST_CENTER_DEVICES * 200 / 256, 1, false);
        let newer = state(MAX_TRUST_CENTER_DEVICES, 2, true);
        let mut journal = journal(MockFlash::new());
        journal.store(&older).unwrap();
        journal.store(&newer).unwrap();
        let located = journal.current.unwrap();
        let chunks = TRUST_CENTER_STREAM_MAX_CHUNKS;
        assert_eq!(located.slots, chunks);
        if chunks == 5 {
            assert_eq!(located.sector, 1, "rolled over");
        }
        let flash = journal.into_storage();
        fn chunk(flash: &mut MockFlash, slot: usize, index: usize) -> &mut [u8] {
            let start = (slot + index) * SLOT;
            &mut flash.bytes[start..start + SLOT]
        }
        // Absolute slot index of the newest snapshot's first chunk.
        let slot = located.sector * (SECTOR / SLOT) + located.slot;

        // Payload bit error in a middle chunk.
        let mut corrupted = MockFlash {
            bytes: flash.bytes.clone(),
            ..MockFlash::new()
        };
        chunk(&mut corrupted, slot, chunks / 2)[HEADER_LEN + 100] ^= 0x04;
        assert_eq!(journal_load(corrupted), Some(older.clone()));
        // Chunk CRC damaged.
        let mut corrupted = MockFlash {
            bytes: flash.bytes.clone(),
            ..MockFlash::new()
        };
        chunk(&mut corrupted, slot, 0)[CHUNK_CRC_OFFSET] ^= 0x01;
        assert_eq!(journal_load(corrupted), Some(older.clone()));
        // No commit marker on the last chunk.
        let mut uncommitted = MockFlash {
            bytes: flash.bytes.clone(),
            ..MockFlash::new()
        };
        chunk(&mut uncommitted, slot, chunks - 1)[COMMIT_OFFSET..COMMIT_OFFSET + 4].fill(0xFF);
        assert_eq!(journal_load(uncommitted), Some(older.clone()));
        // A missing middle chunk (erased slot).
        let mut missing = MockFlash {
            bytes: flash.bytes.clone(),
            ..MockFlash::new()
        };
        chunk(&mut missing, slot, chunks - 2).fill(0xFF);
        assert_eq!(journal_load(missing), Some(older.clone()));
        // A chunk from another generation spliced in.
        let mut spliced = MockFlash {
            bytes: flash.bytes.clone(),
            ..MockFlash::new()
        };
        let foreign = chunk(&mut spliced, slot, 1);
        foreign[8] ^= 0x01;
        let crc = crate::security_journal::crc32(&foreign[..CHUNK_CRC_OFFSET]);
        foreign[CHUNK_CRC_OFFSET..CHUNK_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(journal_load(spliced), Some(older.clone()));
        // Intact flash loads the newest.
        assert_eq!(journal_load(flash), Some(newer));
    }

    #[test]
    fn a_committed_snapshot_that_does_not_decode_falls_back_and_is_superseded() {
        let older = state(3, 1, false);
        let mut journal = journal(MockFlash::new());
        journal.store(&older).unwrap();
        // Generation 1: complete and CRC-valid, but its payload fails
        // validation (no extended PAN ID), so it never decodes.
        let invalid = PersistentTrustCenterState::new([0; 8]);
        let located = journal.current.unwrap();
        let stream = Journal::stream_for(&invalid, 1).unwrap();
        journal
            .write_snapshot(
                located.sector,
                located.slot + located.slots,
                stream,
                &invalid,
            )
            .unwrap();
        // The load decodes in place into a non-empty destination and falls
        // back to the older snapshot.
        let mut journal = reopen(journal);
        let mut loaded = state(MAX_TRUST_CENTER_DEVICES, 7, true);
        assert_eq!(journal.load_into(&mut loaded), Ok(true));
        assert_eq!(loaded, older);
        // A store without a prior load (framing-only scan) is still ordered
        // after the undecodable snapshot.
        let mut journal = reopen(journal);
        let newer = state(4, 2, false);
        journal.store(&newer).unwrap();
        assert!(journal.current.unwrap().generation > 1);
        assert_eq!(reopen(journal).load(), Ok(Some(newer)));
    }

    #[test]
    fn an_empty_journal_loads_nothing() {
        let mut journal = journal(MockFlash::new());
        let mut loaded = state(5, 3, false);
        assert_eq!(journal.load_into(&mut loaded), Ok(false));
    }

    fn journal_load(flash: MockFlash) -> Option<PersistentTrustCenterState> {
        journal(flash).load().unwrap()
    }

    #[test]
    fn generation_wraparound_is_refused_without_touching_flash() {
        let mut journal = journal(MockFlash::new());
        let last = state(3, 9, false);
        journal.storage.erase(0, SECTOR as u32).unwrap();
        let stream = Journal::stream_for(&last, u32::MAX).unwrap();
        journal.write_snapshot(0, 0, stream, &last).unwrap();
        let mut journal = reopen(journal);
        assert_eq!(journal.load(), Ok(Some(last.clone())));
        let before = journal.storage().bytes.clone();
        assert_eq!(
            journal.store(&state(3, 10, false)),
            Err(TrustCenterStoreError::GenerationExhausted)
        );
        assert_eq!(journal.storage().bytes, before);
        assert_eq!(journal.load(), Ok(Some(last)));
    }

    #[test]
    fn a_single_slot_record_migrates_and_the_single_slot_journal_refuses_large_states() {
        // The previous NCP layout: the 4 KiB single-slot journal at the
        // start of each 32 KiB half.
        let legacy_state = state(20, 3, false);
        let mut legacy = TrustCenterDeviceJournal::new(MockFlash::new(), 0, SECTOR as u32);
        legacy.store(&legacy_state).unwrap();
        legacy.store(&legacy_state).unwrap();
        assert_eq!(
            legacy.store(&state(MAX_TRUST_CENTER_DEVICES, 3, false)),
            Err(TrustCenterStoreError::Full),
            "a state larger than one slot is refused before any erase"
        );
        let mut journal = journal(legacy.into_storage());
        assert_eq!(journal.load(), Ok(Some(legacy_state)));
        assert_eq!(journal.current.unwrap().generation, 1);
        let next = state(MAX_TRUST_CENTER_DEVICES, 4, true);
        journal.store(&next).unwrap();
        let located = journal.current.unwrap();
        assert_eq!(
            (located.generation, located.slot),
            (2, 1),
            "appended after the record"
        );
        assert_eq!(reopen(journal).load(), Ok(Some(next)));
        const { assert!(TRUST_CENTER_JOURNAL_SECTOR_SIZE == SLOT) };
    }

    #[test]
    fn clear_erases_both_sectors() {
        let mut journal = journal(MockFlash::new());
        journal.store(&state(4, 1, false)).unwrap();
        journal.clear().unwrap();
        assert_eq!(journal.load(), Ok(None));
        assert_eq!(reopen(journal).load(), Ok(None));
    }

    #[test]
    fn incremental_crc_matches_the_journal_crc() {
        let data: Vec<u8> = (0..5000u32).map(|value| value as u8).collect();
        let crc = crc32_update(crc32_update(CRC_INIT, &data[..1234]), &data[1234..]);
        assert_eq!(!crc, crate::security_journal::crc32(&data));
    }

    /// Firmware before the hashed link-key root wrote version-6 chunks: an
    /// update must keep reading them (the Trust Center table), and the next
    /// store writes version 7.
    #[test]
    fn a_version_6_snapshot_is_read_and_superseded_by_version_7() {
        use crate::trust_center_store::NCP_COUNT_HIGH_RECORD_VERSION;
        let mut expected = state(3, 1, false);
        let mut encoded = [0u8; MAX_ENCODED_TRUST_CENTER_STATE_LEN];
        let len = expected.encode(&mut encoded);
        // Version 6 = version 7 without the 17-byte root trailer.
        let payload = &encoded[..len - 17];
        assert!(payload.len() <= TRUST_CENTER_STREAM_PAYLOAD_PER_CHUNK);
        let mut slot = [0xFFu8; SLOT];
        slot[0..4].copy_from_slice(&CHUNK_MAGIC);
        slot[4] = NCP_COUNT_HIGH_RECORD_VERSION;
        slot[5] = 0;
        slot[6] = 1;
        slot[7] = 0;
        slot[8..12].copy_from_slice(&7u32.to_le_bytes());
        slot[12..16].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        slot[16..18].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        slot[18..20].fill(0);
        slot[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
        let stream_crc = crate::security_journal::crc32(payload);
        slot[STREAM_CRC_OFFSET..STREAM_CRC_OFFSET + 4].copy_from_slice(&stream_crc.to_le_bytes());
        let chunk_crc = crate::security_journal::crc32(&slot[..CHUNK_CRC_OFFSET]);
        slot[CHUNK_CRC_OFFSET..CHUNK_CRC_OFFSET + 4].copy_from_slice(&chunk_crc.to_le_bytes());
        slot[COMMIT_OFFSET..COMMIT_OFFSET + 4].copy_from_slice(&COMMIT);
        let mut flash = MockFlash::new();
        flash.bytes[..SLOT].copy_from_slice(&slot);
        let mut journal = journal(flash);
        let mut loaded = PersistentTrustCenterState::new([0; 8]);
        assert_eq!(journal.load_into(&mut loaded), Ok(true));
        assert_eq!(loaded, expected);
        // The next store is version 7 with the root.
        expected.set_ezsp_hashed_link_key_root(Some([0x77; 16]));
        journal.store(&expected).unwrap();
        let current = journal.current.unwrap();
        assert_eq!(current.generation, 8);
        let mut journal = reopen(journal);
        let mut reloaded = PersistentTrustCenterState::new([0; 8]);
        assert_eq!(journal.load_into(&mut reloaded), Ok(true));
        assert_eq!(reloaded.ezsp_hashed_link_key_root(), Some([0x77; 16]));
        let first = (current.sector * (SECTOR / SLOT) + current.slot) * SLOT;
        assert_eq!(journal.into_storage().bytes[first + 4], RECORD_VERSION);
    }
}
