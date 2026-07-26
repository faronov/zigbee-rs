//! ESP-IDF `otadata` encoding, validation and boot-slot selection.
//!
//! The `otadata` partition holds two redundant 4 KiB sectors, each starting
//! with one 32-byte `esp_ota_select_entry_t`:
//!
//! ```text
//! offset  size  field
//!      0     4  ota_seq      (u32, little endian)
//!      4    20  seq_label
//!     24     4  ota_state    (u32, little endian)
//!     28     4  crc          (u32, little endian) — CRC-32 of ota_seq only
//! ```
//!
//! The second stage bootloader picks the entry with the highest valid sequence
//! number and boots slot `(ota_seq - 1) % 2`. An entry counts as valid when its
//! CRC matches and the sequence number is neither `0` nor `0xFFFF_FFFF`
//! (the erased value), which is why a freshly erased `otadata` makes the
//! bootloader fall back to `ota_0` — exactly the state the devkit is in before
//! the first OTA.
//!
//! Writing a new entry always targets the sector that does *not* hold the
//! currently active entry, so a power failure in the middle of an activation
//! leaves the old, still valid, entry untouched.

use crate::layout::{OTA_SLOT_COUNT, SECTOR_SIZE};

/// Size of one `esp_ota_select_entry_t`.
pub const ENTRY_SIZE: usize = 32;

/// Number of redundant `otadata` sectors.
pub const SECTOR_COUNT: usize = 2;

/// `ESP_OTA_IMG_NEW` — image written, awaiting first boot.
pub const STATE_NEW: u32 = 0x0000_0000;
/// `ESP_OTA_IMG_PENDING_VERIFY` — first boot done, app must confirm.
pub const STATE_PENDING_VERIFY: u32 = 0x0000_0001;
/// `ESP_OTA_IMG_VALID` — image confirmed, boot unconditionally.
pub const STATE_VALID: u32 = 0x0000_0002;
/// `ESP_OTA_IMG_INVALID` — do not boot this image.
pub const STATE_INVALID: u32 = 0x0000_0003;
/// `ESP_OTA_IMG_ABORTED` — rolled back.
pub const STATE_ABORTED: u32 = 0x0000_0004;
/// `ESP_OTA_IMG_UNDEFINED` — erased flash.
pub const STATE_UNDEFINED: u32 = 0xFFFF_FFFF;

/// Sequence number meaning "erased / no entry".
const SEQ_ERASED: u32 = 0xFFFF_FFFF;

/// CRC-32 (IEEE, reflected) of the four little-endian `ota_seq` bytes with the
/// ESP-IDF initial value `0xFFFF_FFFF`.
///
/// This is `esp_rom_crc32_le(UINT32_MAX, &entry->ota_seq, 4)`; the initial
/// value inverts to a zero register and the result is inverted again at the
/// end, which is the same convention as `zlib.crc32(data, 0xFFFFFFFF)`.
pub const fn crc32_seq(seq: u32) -> u32 {
    let bytes = seq.to_le_bytes();
    let mut crc: u32 = 0x0000_0000;
    let mut index = 0;
    while index < bytes.len() {
        crc ^= bytes[index] as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        index += 1;
    }
    crc ^ 0xFFFF_FFFF
}

/// One decoded `otadata` entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaSelectEntry {
    /// Boot sequence number. Slot is `(ota_seq - 1) % OTA_SLOT_COUNT`.
    pub seq: u32,
    /// Free-form label; ESP-IDF leaves it erased.
    pub label: [u8; 20],
    /// Rollback state, only interpreted when the bootloader was built with
    /// rollback support.
    pub state: u32,
    /// CRC-32 of `seq`.
    pub crc: u32,
}

impl OtaSelectEntry {
    /// Build an entry with a correct CRC and an erased label.
    pub const fn new(seq: u32, state: u32) -> Self {
        Self {
            seq,
            label: [0xFF; 20],
            state,
            crc: crc32_seq(seq),
        }
    }

    /// Decode an entry from its 32 raw flash bytes.
    pub fn decode(bytes: &[u8; ENTRY_SIZE]) -> Self {
        let mut label = [0u8; 20];
        label.copy_from_slice(&bytes[4..24]);
        Self {
            seq: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            label,
            state: u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]),
            crc: u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]),
        }
    }

    /// Encode the entry into its 32 raw flash bytes.
    pub fn encode(&self) -> [u8; ENTRY_SIZE] {
        let mut bytes = [0xFFu8; ENTRY_SIZE];
        bytes[0..4].copy_from_slice(&self.seq.to_le_bytes());
        bytes[4..24].copy_from_slice(&self.label);
        bytes[24..28].copy_from_slice(&self.state.to_le_bytes());
        bytes[28..32].copy_from_slice(&self.crc.to_le_bytes());
        bytes
    }

    /// Whether the bootloader would consider this entry.
    ///
    /// Erased sectors (`seq == 0xFFFF_FFFF`), zero sequence numbers (there is
    /// no slot `(0 - 1) % 2`) and CRC mismatches are all rejected, as are the
    /// two states that explicitly forbid booting the image.
    pub fn is_valid(&self) -> bool {
        self.seq != 0
            && self.seq != SEQ_ERASED
            && self.crc == crc32_seq(self.seq)
            && self.state != STATE_INVALID
            && self.state != STATE_ABORTED
    }

    /// Slot this entry selects, if it is valid.
    pub fn slot(&self) -> Option<u8> {
        self.is_valid()
            .then(|| ((self.seq - 1) % OTA_SLOT_COUNT as u32) as u8)
    }
}

/// Both redundant entries, in sector order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaData {
    /// Entry decoded from each `otadata` sector.
    pub entries: [OtaSelectEntry; SECTOR_COUNT],
}

/// A prepared activation: which sector to rewrite and with what.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Activation {
    /// `otadata` sector index (0 or 1) to erase and rewrite.
    pub sector: u8,
    /// Byte offset of that sector inside the `otadata` partition.
    pub sector_offset: u32,
    /// Entry to program.
    pub entry: OtaSelectEntry,
}

/// Reasons an activation entry cannot be produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaDataError {
    /// The requested slot does not exist.
    UnknownSlot,
    /// The sequence counter cannot be advanced without wrapping into the
    /// erased/invalid value.
    SequenceExhausted,
}

impl OtaData {
    /// Decode both sectors. `sectors[i]` are the first 32 bytes of sector `i`.
    pub fn decode(sectors: [&[u8; ENTRY_SIZE]; SECTOR_COUNT]) -> Self {
        Self {
            entries: [
                OtaSelectEntry::decode(sectors[0]),
                OtaSelectEntry::decode(sectors[1]),
            ],
        }
    }

    /// Highest valid sequence number, if any entry is valid.
    pub fn max_seq(&self) -> Option<u32> {
        self.entries
            .iter()
            .filter(|entry| entry.is_valid())
            .map(|entry| entry.seq)
            .max()
    }

    /// Index of the sector the bootloader would use, if any.
    pub fn active_sector(&self) -> Option<u8> {
        let max = self.max_seq()?;
        self.entries
            .iter()
            .position(|entry| entry.is_valid() && entry.seq == max)
            .map(|index| index as u8)
    }

    /// Slot the bootloader would boot, if `otadata` selects one.
    pub fn active_slot(&self) -> Option<u8> {
        let max = self.max_seq()?;
        Some(((max - 1) % OTA_SLOT_COUNT as u32) as u8)
    }

    /// Slot that is running right now.
    ///
    /// With no valid entry the bootloader falls back to the first application
    /// partition, so the running slot is `ota_0`.
    pub fn running_slot(&self) -> u8 {
        self.active_slot().unwrap_or(0)
    }

    /// Slot an update must be staged into.
    pub fn target_slot(&self) -> u8 {
        (self.running_slot() + 1) % OTA_SLOT_COUNT
    }

    /// Build the entry that makes the bootloader select `slot`.
    ///
    /// The entry is placed in the sector that is currently *not* active (an
    /// invalid one first, otherwise the one with the older sequence number),
    /// which keeps the currently bootable entry intact while the new one is
    /// erased and programmed.
    pub fn activation_for(&self, slot: u8) -> Result<Activation, OtaDataError> {
        if slot >= OTA_SLOT_COUNT {
            return Err(OtaDataError::UnknownSlot);
        }

        let count = OTA_SLOT_COUNT as u32;
        let mut seq = self.max_seq().unwrap_or(0) + 1;
        while (seq - 1) % count != slot as u32 {
            seq = seq.checked_add(1).ok_or(OtaDataError::SequenceExhausted)?;
        }
        if seq == SEQ_ERASED {
            return Err(OtaDataError::SequenceExhausted);
        }

        let sector = self.spare_sector();
        Ok(Activation {
            sector,
            sector_offset: sector as u32 * SECTOR_SIZE,
            entry: OtaSelectEntry::new(seq, STATE_VALID),
        })
    }

    /// The sector that may be overwritten without losing the active entry.
    fn spare_sector(&self) -> u8 {
        match (self.entries[0].is_valid(), self.entries[1].is_valid()) {
            (false, _) => 0,
            (true, false) => 1,
            // Both valid: overwrite the older one.
            (true, true) => {
                if self.entries[0].seq <= self.entries[1].seq {
                    0
                } else {
                    1
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ERASED: [u8; ENTRY_SIZE] = [0xFF; ENTRY_SIZE];

    fn entry_bytes(seq: u32, state: u32) -> [u8; ENTRY_SIZE] {
        OtaSelectEntry::new(seq, state).encode()
    }

    #[test]
    fn crc_matches_esp_idf_vectors() {
        // Captured from `bootloader_common_ota_select_crc()` on device.
        assert_eq!(crc32_seq(1), 0x4743_989A);
        assert_eq!(crc32_seq(2), 0x55F6_3774);
        assert_eq!(crc32_seq(3), 0xED4A_5011);
    }

    #[test]
    fn entry_round_trips_through_flash_bytes() {
        let entry = OtaSelectEntry::new(7, STATE_VALID);
        let bytes = entry.encode();
        assert_eq!(&bytes[0..4], &7u32.to_le_bytes());
        assert_eq!(&bytes[4..24], &[0xFFu8; 20]);
        assert_eq!(&bytes[24..28], &STATE_VALID.to_le_bytes());
        assert_eq!(&bytes[28..32], &crc32_seq(7).to_le_bytes());
        assert_eq!(OtaSelectEntry::decode(&bytes), entry);
    }

    #[test]
    fn erased_and_corrupt_entries_are_invalid() {
        assert!(!OtaSelectEntry::decode(&ERASED).is_valid());

        let mut corrupt = entry_bytes(2, STATE_VALID);
        corrupt[28] ^= 0x01;
        assert!(!OtaSelectEntry::decode(&corrupt).is_valid());

        let zero_seq = OtaSelectEntry::new(0, STATE_VALID);
        assert!(!zero_seq.is_valid());
        assert_eq!(zero_seq.slot(), None);

        let aborted = OtaSelectEntry::new(3, STATE_ABORTED);
        assert!(!aborted.is_valid());
        let invalid = OtaSelectEntry::new(3, STATE_INVALID);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn empty_otadata_runs_slot_zero_and_targets_slot_one() {
        let data = OtaData::decode([&ERASED, &ERASED]);
        assert_eq!(data.max_seq(), None);
        assert_eq!(data.active_slot(), None);
        assert_eq!(data.active_sector(), None);
        assert_eq!(data.running_slot(), 0);
        assert_eq!(data.target_slot(), 1);

        let activation = data.activation_for(1).unwrap();
        assert_eq!(activation.sector, 0);
        assert_eq!(activation.sector_offset, 0);
        assert_eq!(activation.entry.seq, 2);
        assert_eq!(activation.entry.slot(), Some(1));
        assert_eq!(activation.entry.state, STATE_VALID);
    }

    #[test]
    fn highest_sequence_number_wins_across_sectors() {
        let low = entry_bytes(4, STATE_VALID);
        let high = entry_bytes(5, STATE_VALID);

        let data = OtaData::decode([&low, &high]);
        assert_eq!(data.max_seq(), Some(5));
        assert_eq!(data.active_sector(), Some(1));
        assert_eq!(data.active_slot(), Some(0));
        assert_eq!(data.target_slot(), 1);

        let flipped = OtaData::decode([&high, &low]);
        assert_eq!(flipped.active_sector(), Some(0));
        assert_eq!(flipped.active_slot(), Some(0));
    }

    #[test]
    fn one_corrupt_sector_falls_back_to_the_other() {
        let mut corrupt = entry_bytes(9, STATE_VALID);
        corrupt[0] ^= 0xFF;
        let good = entry_bytes(4, STATE_VALID);

        let data = OtaData::decode([&corrupt, &good]);
        assert_eq!(data.max_seq(), Some(4));
        assert_eq!(data.active_slot(), Some(1));
        assert_eq!(data.target_slot(), 0);

        // The corrupt sector is the one that gets rewritten.
        let activation = data.activation_for(0).unwrap();
        assert_eq!(activation.sector, 0);
        assert_eq!(activation.entry.seq, 5);
        assert_eq!(activation.entry.slot(), Some(0));
    }

    #[test]
    fn activation_is_monotonic_and_never_touches_the_active_sector() {
        // Sector 0 holds seq 5 (slot 0), sector 1 holds seq 4 (slot 1).
        let newest = entry_bytes(5, STATE_VALID);
        let oldest = entry_bytes(4, STATE_VALID);
        let data = OtaData::decode([&newest, &oldest]);
        assert_eq!(data.active_sector(), Some(0));

        let activation = data.activation_for(data.target_slot()).unwrap();
        assert_eq!(activation.sector, 1, "must rewrite the stale sector");
        assert_eq!(activation.sector_offset, SECTOR_SIZE);
        assert!(activation.entry.seq > data.max_seq().unwrap());
        assert_eq!(activation.entry.slot(), Some(1));

        // Applying it selects the new slot.
        let applied = OtaData::decode([&newest, &activation.entry.encode()]);
        assert_eq!(applied.active_slot(), Some(1));
        assert_eq!(applied.active_sector(), Some(1));
        assert_eq!(applied.target_slot(), 0);
    }

    #[test]
    fn re_activating_the_same_slot_skips_a_sequence_number() {
        let current = entry_bytes(5, STATE_VALID); // slot 0
        let data = OtaData::decode([&current, &ERASED]);
        assert_eq!(data.active_slot(), Some(0));

        // Staging into slot 0 again needs seq 7, because seq 6 maps to slot 1.
        let activation = data.activation_for(0).unwrap();
        assert_eq!(activation.entry.seq, 7);
        assert_eq!(activation.entry.slot(), Some(0));
        assert_eq!(activation.sector, 1);
    }

    #[test]
    fn sequence_exhaustion_is_reported() {
        // seq 0xFFFFFFFE selects slot (0xFFFFFFFE - 1) % 2 == 1 and is the last
        // usable value, because 0xFFFFFFFF is the erased marker.
        let last = OtaSelectEntry::new(0xFFFF_FFFE, STATE_VALID).encode();
        let data = OtaData::decode([&last, &ERASED]);
        assert_eq!(data.active_slot(), Some(1));
        assert_eq!(data.target_slot(), 0);
        assert_eq!(data.activation_for(0), Err(OtaDataError::SequenceExhausted));
        assert_eq!(data.activation_for(2), Err(OtaDataError::UnknownSlot));

        // One step earlier the counter still has room.
        let earlier = OtaSelectEntry::new(0xFFFF_FFFD, STATE_VALID).encode();
        let data = OtaData::decode([&earlier, &ERASED]);
        assert_eq!(data.activation_for(1).unwrap().entry.seq, 0xFFFF_FFFE);
    }

    #[test]
    fn power_loss_before_the_crc_lands_keeps_the_old_entry() {
        let good = entry_bytes(5, STATE_VALID);
        // Torn write: sequence programmed, CRC still erased.
        let mut torn = entry_bytes(6, STATE_VALID);
        torn[28..32].copy_from_slice(&[0xFF; 4]);

        let data = OtaData::decode([&good, &torn]);
        assert_eq!(data.max_seq(), Some(5));
        assert_eq!(data.active_slot(), Some(0));
    }
}
