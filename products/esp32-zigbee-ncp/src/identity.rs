//! The persisted identity item: the IEEE override of `SYS_SET_IEEE` and
//! `BKP_RESTORE_COMMIT` (spec 7.1, 9.4 rule 7).
//!
//! The item lives in the `zbnv` identity window ([`crate::layout::Nv::
//! IDENTITY`], 2 × 4 KiB), journaled by `zigbee_runtime`'s
//! [`SettingsJournal`](zigbee_runtime::settings_journal::SettingsJournal):
//! every record carries a magic, a generation (sequence) and a CRC-32, and
//! its commit marker is programmed last; records are appended, and only a
//! full sector makes the journal erase the *other* sector. An erased window,
//! a torn record (power cut before the commit marker) or a record with a
//! bad CRC is skipped, so a load returns the newest complete record or
//! nothing, and nothing means "factory identity".
//!
//! Payload (format 1):
//!
//! ```text
//! 0      u8   format (1)
//! 1      u8   flags: bit0 IEEE override present
//! 2..10  [8]  IEEE override (as given to SYS_SET_IEEE; FF.. when absent)
//! 10..   ...  further settings (e.g. the UART default); kept verbatim
//! ```
//!
//! The override takes effect at the next boot: the composition root reads
//! it before the stack is composed and writes it to `MacExtendedAddress`.
//! The 8 bytes are stored and applied verbatim, in the order of
//! `MacPib::extended_address()` (the order the NCP reports its IEEE in);
//! the dispatcher passes `SYS_SET_IEEE.ieee` in that same order.
//! "Revert to factory" (`SYS_SET_IEEE FF…`) stores a record without the
//! flag; the factory EUI-64 is the chip's eFuse MAC and is never stored.

use zigbee_runtime::settings_journal::{
    MAX_SETTINGS_PAYLOAD, SettingsError, SettingsRecord, SettingsStore,
};

/// Payload format written by this firmware.
pub const FORMAT: u8 = 1;
const FLAG_IEEE: u8 = 0x01;
const IEEE_AT: usize = 2;
/// Length of the format-1 fields this firmware owns.
pub const FORMAT1_LEN: usize = IEEE_AT + 8;

/// Why the identity item could not be read or written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityError {
    /// Flash failure or an exhausted journal.
    Storage(SettingsError),
    /// The stored record has a format this firmware does not know (written
    /// by a newer image); it is neither applied nor overwritten.
    UnknownFormat(u8),
    /// The all-zero IEEE address is not an identity.
    InvalidIeee,
}

/// Decoded format-1 identity item.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IdentityRecord {
    pub ieee_override: Option<[u8; 8]>,
    /// Payload bytes after the format-1 fields (preserved on rewrite).
    rest: tail::Rest,
}

impl IdentityRecord {
    /// A record with no override and no further settings.
    pub const fn empty() -> Self {
        Self {
            ieee_override: None,
            rest: tail::Rest::EMPTY,
        }
    }

    /// Decode a stored payload.
    pub fn decode(payload: &[u8]) -> Result<Self, IdentityError> {
        let Some(&format) = payload.first() else {
            return Ok(Self::empty());
        };
        if format != FORMAT {
            return Err(IdentityError::UnknownFormat(format));
        }
        if payload.len() < FORMAT1_LEN {
            // Shorter than its own format: treat as corrupt (factory).
            return Ok(Self::empty());
        }
        let mut ieee = [0u8; 8];
        ieee.copy_from_slice(&payload[IEEE_AT..IEEE_AT + 8]);
        let ieee_override = (payload[1] & FLAG_IEEE != 0)
            .then_some(ieee)
            .filter(is_settable);
        Ok(Self {
            ieee_override,
            rest: tail::Rest::from_slice(&payload[FORMAT1_LEN..]),
        })
    }

    /// Encode as a journal record.
    pub fn encode(&self) -> SettingsRecord {
        let mut payload = [0xFFu8; MAX_SETTINGS_PAYLOAD];
        payload[0] = FORMAT;
        payload[1] = if self.ieee_override.is_some() {
            FLAG_IEEE
        } else {
            0
        };
        if let Some(ieee) = self.ieee_override {
            payload[IEEE_AT..IEEE_AT + 8].copy_from_slice(&ieee);
        }
        let rest = self.rest.as_slice();
        payload[FORMAT1_LEN..FORMAT1_LEN + rest.len()].copy_from_slice(rest);
        SettingsRecord::new(&payload[..FORMAT1_LEN + rest.len()])
            .expect("format-1 payload fits a settings record")
    }
}

/// Whether `ieee` may be stored as an override (all-`FF` means "revert",
/// all-zero is no address).
pub fn is_settable(ieee: &[u8; 8]) -> bool {
    *ieee != [0; 8] && *ieee != [0xFF; 8]
}

/// The identity item over a settings store (the `zbnv` journal on target,
/// a fake flash in the host tests).
pub struct IdentityItem<S> {
    store: S,
}

impl<S: SettingsStore> IdentityItem<S> {
    pub const fn new(store: S) -> Self {
        Self { store }
    }

    pub fn store_mut(&mut self) -> &mut S {
        &mut self.store
    }

    /// The stored record (`empty` when nothing valid is stored).
    pub fn load(&mut self) -> Result<IdentityRecord, IdentityError> {
        match self.store.load().map_err(IdentityError::Storage)? {
            Some(record) => IdentityRecord::decode(record.as_slice()),
            None => Ok(IdentityRecord::empty()),
        }
    }

    /// The override to apply at boot, if any.
    pub fn ieee_override(&mut self) -> Result<Option<[u8; 8]>, IdentityError> {
        Ok(self.load()?.ieee_override)
    }

    /// Persist (`Some`) or remove (`None`, or `Some(FF…)`) the override.
    /// Returns whether a record was written; an unchanged value writes
    /// nothing (no flash wear for repeated restores).
    pub fn set_ieee_override(&mut self, ieee: Option<[u8; 8]>) -> Result<bool, IdentityError> {
        let ieee = ieee.filter(|ieee| *ieee != [0xFF; 8]);
        if ieee == Some([0; 8]) {
            return Err(IdentityError::InvalidIeee);
        }
        let mut record = self.load()?;
        if record.ieee_override == ieee {
            return Ok(false);
        }
        record.ieee_override = ieee;
        self.store
            .store(&record.encode())
            .map_err(IdentityError::Storage)?;
        Ok(true)
    }
}

/// Fixed-capacity copy of the payload tail this firmware does not own.
mod tail {
    use super::{FORMAT1_LEN, MAX_SETTINGS_PAYLOAD};

    const CAPACITY: usize = MAX_SETTINGS_PAYLOAD - FORMAT1_LEN;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct Rest {
        len: u8,
        bytes: [u8; CAPACITY],
    }

    impl Rest {
        pub const EMPTY: Self = Self {
            len: 0,
            bytes: [0; CAPACITY],
        };

        pub fn from_slice(bytes: &[u8]) -> Self {
            let len = bytes.len().min(CAPACITY);
            let mut rest = Self::EMPTY;
            rest.bytes[..len].copy_from_slice(&bytes[..len]);
            rest.len = len as u8;
            rest
        }

        pub fn as_slice(&self) -> &[u8] {
            &self.bytes[..usize::from(self.len)]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::SECTOR_SIZE;
    use embedded_storage::nor_flash::{
        ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
    };
    use zigbee_runtime::settings_journal::{SETTINGS_SLOT_SIZE, SettingsJournal};

    const SECTOR: usize = SECTOR_SIZE as usize;

    /// The 2 × 4 KiB identity window as NOR flash: writes only clear bits,
    /// and a write can be torn after `tear_after` bytes (power cut).
    #[derive(Clone)]
    struct Window {
        data: Vec<u8>,
        tear_after: Option<usize>,
        erases: usize,
    }

    #[derive(Debug)]
    struct Fault;

    impl NorFlashError for Fault {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    impl Window {
        fn erased() -> Self {
            Self {
                data: vec![0xFF; 2 * SECTOR],
                tear_after: None,
                erases: 0,
            }
        }
    }

    impl ErrorType for Window {
        type Error = Fault;
    }

    impl ReadNorFlash for Window {
        const READ_SIZE: usize = 1;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Fault> {
            let start = offset as usize;
            bytes.copy_from_slice(&self.data[start..start + bytes.len()]);
            Ok(())
        }
        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for Window {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECTOR;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), Fault> {
            assert_eq!(from as usize % SECTOR, 0);
            self.data[from as usize..to as usize].fill(0xFF);
            self.erases += 1;
            Ok(())
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Fault> {
            assert_eq!(offset % 4, 0);
            for (index, byte) in bytes.iter().enumerate() {
                if let Some(budget) = self.tear_after.as_mut() {
                    if *budget == 0 {
                        return Err(Fault);
                    }
                    *budget -= 1;
                }
                self.data[offset as usize + index] &= *byte;
            }
            Ok(())
        }
    }

    type Item = IdentityItem<SettingsJournal<Window, SECTOR>>;

    fn item(window: Window) -> Item {
        IdentityItem::new(SettingsJournal::new(window, 0, SECTOR as u32))
    }

    /// Power cycle: a fresh journal over the same flash.
    fn reboot(item: Item) -> Item {
        let mut window = item_window(item);
        window.tear_after = None;
        self::item(window)
    }

    fn item_window(item: Item) -> Window {
        item.store.into_storage()
    }

    const A: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
    const B: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

    #[test]
    fn an_erased_window_is_the_factory_identity() {
        let mut item = item(Window::erased());
        assert_eq!(item.ieee_override(), Ok(None));
    }

    #[test]
    fn an_override_persists_across_reboots_and_can_be_repeated() {
        let mut item = item(Window::erased());
        assert_eq!(item.set_ieee_override(Some(A)), Ok(true));
        let mut item = reboot(item);
        assert_eq!(item.ieee_override(), Ok(Some(A)));
        // Same value again: nothing is written.
        assert_eq!(item.set_ieee_override(Some(A)), Ok(false));
        assert_eq!(item.set_ieee_override(Some(B)), Ok(true));
        let mut item = reboot(item);
        assert_eq!(item.ieee_override(), Ok(Some(B)));
    }

    #[test]
    fn all_ff_or_none_reverts_to_factory() {
        let mut item = item(Window::erased());
        item.set_ieee_override(Some(A)).unwrap();
        assert_eq!(item.set_ieee_override(Some([0xFF; 8])), Ok(true));
        let mut item = reboot(item);
        assert_eq!(item.ieee_override(), Ok(None));
        // Reverting again (or on a factory board) writes nothing.
        assert_eq!(item.set_ieee_override(None), Ok(false));
        let mut fresh = self::item(Window::erased());
        assert_eq!(fresh.set_ieee_override(None), Ok(false));
        assert!(item_window(fresh).data.iter().all(|byte| *byte == 0xFF));
    }

    #[test]
    fn the_all_zero_address_is_refused() {
        let mut item = item(Window::erased());
        assert_eq!(
            item.set_ieee_override(Some([0; 8])),
            Err(IdentityError::InvalidIeee)
        );
        assert_eq!(item.ieee_override(), Ok(None));
    }

    #[test]
    fn a_torn_write_keeps_the_previous_identity() {
        // Tear at every byte of the record (and its commit marker).
        for budget in 0..SETTINGS_SLOT_SIZE - 4 {
            let mut item = item(Window::erased());
            item.set_ieee_override(Some(A)).unwrap();
            let mut window = item_window(item);
            window.tear_after = Some(budget);
            let mut item = self::item(window);
            assert!(item.set_ieee_override(Some(B)).is_err(), "budget {budget}");
            let mut item = reboot(item);
            assert_eq!(item.ieee_override(), Ok(Some(A)), "budget {budget}");
            // The next write after the cut succeeds and wins.
            assert_eq!(item.set_ieee_override(Some(B)), Ok(true));
            assert_eq!(reboot(item).ieee_override(), Ok(Some(B)));
        }
    }

    #[test]
    fn a_torn_first_write_is_the_factory_identity() {
        let mut window = Window::erased();
        window.tear_after = Some(20);
        let mut item = item(window);
        assert!(item.set_ieee_override(Some(A)).is_err());
        assert_eq!(reboot(item).ieee_override(), Ok(None));
    }

    #[test]
    fn a_record_with_a_bad_crc_is_ignored() {
        let mut item = item(Window::erased());
        item.set_ieee_override(Some(A)).unwrap();
        item.set_ieee_override(Some(B)).unwrap();
        let mut window = item_window(item);
        // Corrupt one IEEE byte of the newest record (slot 1): it no longer
        // matches its CRC, so the older record is the newest valid one.
        let payload = SETTINGS_SLOT_SIZE + 12;
        window.data[payload + IEEE_AT] ^= 0x01;
        let mut item = self::item(window);
        assert_eq!(item.ieee_override(), Ok(Some(A)));
        // Corrupting that one as well leaves no valid record: factory.
        let mut window = item_window(item);
        window.data[12 + IEEE_AT] ^= 0x01;
        assert_eq!(self::item(window).ieee_override(), Ok(None));
    }

    #[test]
    fn the_journal_rotates_sectors_and_keeps_the_newest() {
        let mut item = item(Window::erased());
        let slots = SECTOR / SETTINGS_SLOT_SIZE;
        for round in 0..(3 * slots) as u32 {
            let mut ieee = A;
            ieee[..4].copy_from_slice(&round.to_le_bytes());
            item.set_ieee_override(Some(ieee)).unwrap();
            assert_eq!(item.ieee_override(), Ok(Some(ieee)));
        }
        let window = item_window(item);
        assert!(window.erases >= 2);
        let mut ieee = A;
        ieee[..4].copy_from_slice(&((3 * slots - 1) as u32).to_le_bytes());
        assert_eq!(self::item(window).ieee_override(), Ok(Some(ieee)));
    }

    #[test]
    fn further_settings_survive_an_override_change() {
        let mut item = item(Window::erased());
        // A record written by a firmware that also stores a UART default.
        let mut payload = vec![FORMAT, 0];
        payload.extend_from_slice(&[0xFF; 8]);
        payload.extend_from_slice(&[0x00, 0x08, 0x07, 0x00, 0x01]);
        item.store_mut()
            .store(&SettingsRecord::new(&payload).unwrap())
            .unwrap();
        item.set_ieee_override(Some(A)).unwrap();
        let stored = item.store_mut().load().unwrap().unwrap();
        assert_eq!(&stored.as_slice()[FORMAT1_LEN..], &payload[FORMAT1_LEN..]);
        assert_eq!(&stored.as_slice()[IEEE_AT..IEEE_AT + 8], &A);
    }

    #[test]
    fn an_unknown_format_is_neither_applied_nor_overwritten() {
        let mut item = item(Window::erased());
        item.store_mut()
            .store(&SettingsRecord::new(&[2, 1, 9, 9, 9, 9, 9, 9, 9, 9]).unwrap())
            .unwrap();
        assert_eq!(item.ieee_override(), Err(IdentityError::UnknownFormat(2)));
        assert_eq!(
            item.set_ieee_override(Some(A)),
            Err(IdentityError::UnknownFormat(2))
        );
    }
}
