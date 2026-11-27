//! The NV journals, each bound to its [`crate::layout`] window, and the
//! opening policy. The router role uses the same windows (except the Trust
//! Center stream, which only the NCP has).
//!
//! Opening policy (before the stack reads anything):
//!
//! * any 8 KiB page start of the window that holds neither erased flash nor
//!   one of our record magics (the stock EmberZNet NVM3, or anything else
//!   that wrote here since our journals did): every window is erased, even
//!   if records of ours survive in pages the other user never touched
//!   ([`Opened::Foreign`]);
//! * no valid security record (blank flash or foreign bytes that happen to
//!   leave every page start alone): every NCP window is erased — foreign
//!   bytes are never interpreted;
//! * a valid security record of another role image (e.g. the router):
//!   `select_role_tag` clears its keys while keeping the outgoing frame
//!   counter limits, and the role's other journals are erased;
//! * a valid NCP journal is preserved.
//!
//! A power loss during an erase repeats the same path on the next boot.
//!
//! Window offsets are relative to the flash base (`0x0800_0000`), which
//! is what `efr32s2_hal::flash::FlashPartition` expects.

use efr32s2_hal::flash::{Efr32s2Flash as Flash, FlashPartition};
use embedded_storage::nor_flash::NorFlash;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::gp_store::GpProxyJournal;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};
use zigbee_runtime::settings_journal::SettingsJournal;
#[cfg(feature = "ncp")]
use zigbee_runtime::trust_center_stream::{
    TRUST_CENTER_STREAM_MIN_SECTOR_SIZE, TrustCenterStreamJournal,
};

use crate::layout::{
    APS_TABLES, CHILDREN, GREEN_POWER, IDENTITY, NV_SIZE, NV_START, SECURITY, TRUST_CENTER,
};

pub type SecurityFlash = FlashPartition<{ SECURITY.start }, { SECURITY.size }>;
#[cfg(feature = "ncp")]
pub type TrustCenterFlash = FlashPartition<{ TRUST_CENTER.start }, { TRUST_CENTER.size }>;
pub type ChildFlash = FlashPartition<{ CHILDREN.start }, { CHILDREN.size }>;
pub type GreenPowerFlash = FlashPartition<{ GREEN_POWER.start }, { GREEN_POWER.size }>;
pub type ApsTableFlash = FlashPartition<{ APS_TABLES.start }, { APS_TABLES.size }>;
pub type IdentityFlash = FlashPartition<{ IDENTITY.start }, { IDENTITY.size }>;
type NvFlash = FlashPartition<NV_START, NV_SIZE>;

/// NWK/APS reservations, keys and replay floors: 2 x 16 KiB.
pub type SecurityStore = SecurityStateJournal<SecurityFlash, { SECURITY.sector }>;
/// Trust Center devices (256), streamed across 4 KiB chunk slots of
/// 2 x 32 KiB logical sectors.
#[cfg(feature = "ncp")]
pub type TrustCenterStore = TrustCenterStreamJournal<TrustCenterFlash, { TRUST_CENTER.sector }>;
/// 64 children in 1 KiB slots (`ncp-capacity`).
pub type ChildStore = ChildTableJournal<ChildFlash, { CHILDREN.sector }>;
/// GP combo journal (Proxy Table 8 + Sink Table 16).
pub type GreenPowerStore = GpProxyJournal<GreenPowerFlash, { GREEN_POWER.sector }>;
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, { APS_TABLES.sector }>;
/// IEEE override and persisted settings (reserved; not used yet).
pub type IdentityStore = SettingsJournal<IdentityFlash, { IDENTITY.sector }>;

#[cfg(feature = "ncp")]
const _: () = assert!(SecurityStore::NCP_SECTOR_FITS);
#[cfg(feature = "ncp")]
const _: () = assert!(TRUST_CENTER_STREAM_MIN_SECTOR_SIZE <= TRUST_CENTER.sector);

/// How the journals were found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Opened {
    /// A valid NCP journal set.
    Existing,
    /// Blank NV, or no valid security record: every window erased.
    Fresh,
    /// A page start held foreign data (e.g. the stock EmberZNet NVM3):
    /// every window erased, whatever records of ours it still held.
    Foreign(ForeignPage),
    /// Another role's journal: keys cleared (counter limits kept), its
    /// other journals erased.
    RoleChanged,
}

/// The first page start of the NV window that is not ours, for the boot
/// log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ForeignPage {
    /// Bus address of the page.
    pub address: u32,
    /// Its first word (little-endian).
    pub first_word: u32,
    /// Some page start carries the GSDK NVM3 page magic `0xB29A` in either
    /// half of its first word. A label only: the decision does not depend
    /// on recognising NVM3.
    pub nvm3: bool,
}

pub struct Stores {
    pub security: SecurityStore,
    #[cfg(feature = "ncp")]
    pub trust_center: TrustCenterStore,
    pub children: ChildStore,
    pub green_power: GreenPowerStore,
    pub aps_tables: ApsTableStore,
}

/// Open every journal of this role from the exclusive raw flash handle.
pub fn open(flash: Flash, role_tag: u16) -> Result<(Stores, Opened), SecurityStoreError> {
    let _owned = flash;
    // SAFETY: the windows are disjoint from each other and from the
    // application (layout.rs and the linker script assert it); the single
    // flash handle is consumed here, and every `NvFlash` minted by the
    // closure is a temporary dropped before the journals are used.
    let mut security = SecurityStore::new_with_sector_size(
        unsafe { SecurityFlash::new_unchecked() },
        0,
        SECURITY.sector as u32,
    );
    let opened = prepare(
        &mut security,
        || unsafe { NvFlash::new_unchecked() },
        role_tag,
    )?;
    let stores = unsafe {
        Stores {
            security,
            #[cfg(feature = "ncp")]
            trust_center: TrustCenterStore::new(
                TrustCenterFlash::new_unchecked(),
                0,
                TRUST_CENTER.sector as u32,
            ),
            children: ChildStore::new_with_sector_size(
                ChildFlash::new_unchecked(),
                0,
                CHILDREN.sector as u32,
            ),
            green_power: GreenPowerStore::new(
                GreenPowerFlash::new_unchecked(),
                0,
                GREEN_POWER.sector as u32,
            ),
            aps_tables: ApsTableStore::new_with_sector_size(
                ApsTableFlash::new_unchecked(),
                0,
                APS_TABLES.sector as u32,
            ),
        }
    };
    Ok((stores, opened))
}

/// The opening policy over any flash: `security` is the security window,
/// `nv` mints a view of the whole NV region (offset 0 = [`NV_START`]).
///
/// Nothing in the region is interpreted while any of its page starts holds
/// foreign data ([`foreign_page`]): after "ours → clear the stock NVM3
/// pages → stock EmberZNet → ours", records of our earlier run can survive
/// in a page the stock NVM3 never used (the top page `0x0817_E000`, outside
/// the 32 KiB stock instance), and resuming them would bring back the
/// pre-stock network with frame-counter reservations behind what the
/// network saw in between. Otherwise nothing is interpreted unless the
/// security journal holds a valid record: blank or foreign bytes fail its
/// magic/commit/CRC checks.
fn prepare<Sec, Nv>(
    security: &mut SecurityStateJournal<Sec, { SECURITY.sector }>,
    mut nv: impl FnMut() -> Nv,
    role_tag: u16,
) -> Result<Opened, SecurityStoreError>
where
    Sec: NorFlash,
    Nv: NorFlash,
{
    let mut opened = Opened::Existing;
    let foreign = foreign_page(&mut nv())?;
    if foreign.is_some() || security.load()?.is_none() {
        nv().erase(0, NV_SIZE as u32)
            .map_err(|_| SecurityStoreError::Hardware)?;
        // Re-read the erased journal.
        let _ = security.storage_mut();
        opened = foreign.map_or(Opened::Fresh, Opened::Foreign);
    }
    if security.select_role_tag(role_tag)? {
        // Everything but the identity and security windows.
        nv().erase(APS_TABLES.start - NV_START, SECURITY.start - NV_START)
            .map_err(|_| SecurityStoreError::Hardware)?;
        opened = Opened::RoleChanged;
    }
    Ok(opened)
}

/// Record magics of every journal in the NV window (word 0 of a slot).
/// Every 8 KiB page start of the window is a slot start: every logical
/// sector is a whole number of pages and every slot size divides the page
/// (`layout.rs`; security 128 B, Trust Center 4 KiB, children 512 B/1 KiB,
/// GP 1 KiB with 32 B counter records, APS 2 KiB, settings 64 B).
const OUR_MAGICS: [[u8; 4]; 10] = [
    *b"ZBSS", // security record
    *b"ZBSR", // security replay snapshot
    *b"ZBTS", // Trust Center stream chunk
    *b"ZBTC", // Trust Center record (pre-stream layout)
    *b"ZBCT", // child table
    *b"ZBGP", // GP proxy table
    *b"ZBGC", // GP frame counters
    *b"ZBGS", // GP sink table
    *b"ZBAT", // APS tables
    *b"ZBST", // settings / identity
];

/// Whether `word` can start a page of ours: erased flash, or one of
/// [`OUR_MAGICS`] programmed completely or partly. The journals program a
/// slot from its magic word on, and an interrupted program (or page erase)
/// only leaves bits of the magic's zeros at 1, so such a word keeps every
/// 1 bit of the magic.
const fn ours(word: u32) -> bool {
    if word == u32::MAX {
        return true;
    }
    let mut i = 0;
    while i < OUR_MAGICS.len() {
        let magic = u32::from_le_bytes(OUR_MAGICS[i]);
        if word & magic == magic {
            return true;
        }
        i += 1;
    }
    false
}

/// The GSDK NVM3 page magic in either half of a page's first word (label
/// only, see [`ForeignPage::nvm3`]).
const fn looks_like_nvm3(word: u32) -> bool {
    word >> 16 == 0xB29A || word & 0xFFFF == 0xB29A
}

/// The first 8 KiB page of the NV window whose first word is not
/// [`ours`]. Independent of the foreign format: nothing here relies on
/// knowing what is foreign. (The stock EmberZNet 7.4.5 NVM3 does carry the
/// GSDK page header, `0xB29A0001` first, as captured on a BRD4187; the
/// rule covers any other format too.)
fn foreign_page<Nv: NorFlash>(nv: &mut Nv) -> Result<Option<ForeignPage>, SecurityStoreError> {
    let mut first = None;
    let mut nvm3 = false;
    let mut page = 0;
    while page < NV_SIZE {
        let mut word = [0u8; 4];
        nv.read(page as u32, &mut word)
            .map_err(|_| SecurityStoreError::Hardware)?;
        let word = u32::from_le_bytes(word);
        if !ours(word) {
            nvm3 |= looks_like_nvm3(word);
            first.get_or_insert(ForeignPage {
                address: efr32s2_hal::chip::FLASH_BASE + NV_START + page as u32,
                first_word: word,
                nvm3: false,
            });
        }
        page += efr32s2_hal::flash::FLASH_PAGE_SIZE;
    }
    Ok(first.map(|page| ForeignPage { nvm3, ..page }))
}

// Every page start of the window is a slot start (see `OUR_MAGICS`).
const _: () = {
    const PAGE: usize = efr32s2_hal::flash::FLASH_PAGE_SIZE;
    use zigbee_runtime::{aps_table_store, child_store, gp_store, security_journal};
    assert!(PAGE.is_multiple_of(security_journal::SECURITY_JOURNAL_SLOT_SIZE));
    assert!(PAGE.is_multiple_of(child_store::CHILD_JOURNAL_SLOT_SIZE));
    assert!(PAGE.is_multiple_of(gp_store::GP_PROXY_JOURNAL_SLOT_SIZE));
    assert!(PAGE.is_multiple_of(gp_store::GP_COUNTER_RECORD_SIZE));
    assert!(PAGE.is_multiple_of(aps_table_store::APS_TABLE_JOURNAL_SLOT_SIZE));
    assert!(PAGE.is_multiple_of(zigbee_runtime::settings_journal::SETTINGS_SLOT_SIZE));
};
#[cfg(feature = "ncp")]
const _: () = assert!(
    efr32s2_hal::flash::FLASH_PAGE_SIZE
        .is_multiple_of(zigbee_runtime::trust_center_stream::TRUST_CENTER_STREAM_SLOT_SIZE)
);

const _: () = {
    // The role-change erase range is contiguous: APS, GP, children, TC.
    assert!(APS_TABLES.end() == GREEN_POWER.start);
    assert!(GREEN_POWER.end() == CHILDREN.start);
    assert!(CHILDREN.end() == TRUST_CENTER.start);
    assert!(TRUST_CENTER.end() == SECURITY.start);
    assert!(IDENTITY.end() == APS_TABLES.start);
};

#[cfg(test)]
mod tests {
    //! The opening policy on a RAM model of the NV region (EFR32 NOR rules:
    //! 4-byte programs that only clear bits, 8 KiB page erases).

    use super::*;
    use core::cell::RefCell;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use std::rc::Rc;
    use std::vec;
    use std::vec::Vec;
    use zigbee_runtime::security_store::{PersistentSecurityState, SecurityStateStore};

    use crate::layout::{FLASH_SIZE, TRUST_CENTER};

    /// Bus address of an NV offset.
    const fn bus(offset: u32) -> u32 {
        efr32s2_hal::chip::FLASH_BASE + offset
    }

    /// 16 bytes of a record of ours: `magic`, then `fill`.
    fn marked(magic: &[u8; 4], fill: u8) -> [u8; 16] {
        let mut record = [fill; 16];
        record[..4].copy_from_slice(magic);
        record
    }

    fn lcg(seed: &mut u32) -> u32 {
        *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *seed
    }

    const PAGE: usize = efr32s2_hal::flash::FLASH_PAGE_SIZE;

    /// A window `[base, base + size)` of the shared NV region.
    struct View {
        nv: Rc<RefCell<Vec<u8>>>,
        base: usize,
        size: usize,
    }

    impl View {
        fn range(
            &self,
            offset: u32,
            len: usize,
        ) -> Result<core::ops::Range<usize>, NorFlashErrorKind> {
            let start = offset as usize;
            let end = start
                .checked_add(len)
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            if end > self.size {
                return Err(NorFlashErrorKind::OutOfBounds);
            }
            Ok(self.base + start..self.base + end)
        }
    }

    impl ErrorType for View {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for View {
        const READ_SIZE: usize = 1;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let range = self.range(offset, bytes.len())?;
            bytes.copy_from_slice(&self.nv.borrow()[range]);
            Ok(())
        }
        fn capacity(&self) -> usize {
            self.size
        }
    }

    impl NorFlash for View {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = PAGE;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if !(from as usize).is_multiple_of(PAGE)
                || !(to as usize).is_multiple_of(PAGE)
                || from >= to
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = self.range(from, (to - from) as usize)?;
            self.nv.borrow_mut()[range].fill(0xFF);
            Ok(())
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if !(offset as usize).is_multiple_of(4) || !bytes.len().is_multiple_of(4) {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = self.range(offset, bytes.len())?;
            for (cell, byte) in self.nv.borrow_mut()[range].iter_mut().zip(bytes) {
                if *cell & *byte != *byte {
                    return Err(NorFlashErrorKind::Other);
                }
                *cell &= *byte;
            }
            Ok(())
        }
    }

    struct Region(Rc<RefCell<Vec<u8>>>);

    impl Region {
        fn filled(byte: u8) -> Self {
            Self(Rc::new(RefCell::new(vec![byte; NV_SIZE])))
        }
        fn nv(&self) -> View {
            View {
                nv: self.0.clone(),
                base: 0,
                size: NV_SIZE,
            }
        }
        fn journal(&self) -> SecurityStore<View> {
            SecurityStateJournal::new_with_sector_size(
                View {
                    nv: self.0.clone(),
                    base: (SECURITY.start - NV_START) as usize,
                    size: SECURITY.size,
                },
                0,
                SECURITY.sector as u32,
            )
        }
        fn open(&self, role_tag: u16) -> (SecurityStore<View>, Opened) {
            let mut journal = self.journal();
            let opened = prepare(&mut journal, || self.nv(), role_tag).unwrap();
            (journal, opened)
        }
        /// Bytes of the bus range `[start, end)` (main-flash offsets).
        fn bytes(&self, start: u32, end: u32) -> Vec<u8> {
            self.0.borrow()[(start - NV_START) as usize..(end - NV_START) as usize].to_vec()
        }
        fn fill(&self, start: u32, data: &[u8]) {
            let at = (start - NV_START) as usize;
            self.0.borrow_mut()[at..at + data.len()].copy_from_slice(data);
        }
    }

    type SecurityStore<F> = SecurityStateJournal<F, { SECURITY.sector }>;

    /// Bus offsets of the stock Dongle-M EmberZNet 7.4.5 NVM3 instance
    /// (`0x0817_6000..0x0817_E000`, four 8 KiB pages).
    const STOCK_NVM3: core::ops::Range<u32> = 0x17_6000..0x17_E000;

    /// A GSDK NVM3 page header as a fresh format leaves it: magic and
    /// version, erase count and complement with Berger codes, page state
    /// good, format word. Matches [`EMBERZNET_745_PAGE_HEADER`] for erase
    /// count 1.
    fn nvm3_header(erase_count: u32) -> [u32; 5] {
        let coded = |value: u32| {
            let value = value & 0x07FF_FFFF;
            value | (27 - value.count_ones()) << 27
        };
        [
            0xB29A_0001,
            coded(erase_count),
            coded(!erase_count),
            0xFFFF_FFFF,
            0xFFFF_903C,
        ]
    }

    /// The page header of each of the four stock NVM3 pages
    /// (`0x0817_6000`, `0x0817_8000`, `0x0817_A000`, `0x0817_C000`) as
    /// captured over SWD on a BRD4187 (EFR32MG24B220, 1536 KiB) about 90 s
    /// after SONOFF's EmberZNet 7.4.5 NCP (`zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`,
    /// SHA-256 `44b81c1e…`) first booted on an erased window. Nothing else
    /// above the image was written; the first page also held 1,856 bytes of
    /// objects.
    const EMBERZNET_745_PAGE_HEADER: [u32; 5] = [
        0xB29A_0001,
        0xD000_0001,
        0x0FFF_FFFE,
        0xFFFF_FFFF,
        0xFFFF_903C,
    ];

    /// The first words of the journal pages of a commissioned NCP of ours,
    /// captured on the same board (bus address offset, magic); every other
    /// page start of the window was erased.
    const CAPTURED_NCP_PAGE_STARTS: [(u32, [u8; 4]); 4] = [
        (0x15_C000, *b"ZBAT"),
        (0x16_6000, *b"ZBCT"),
        (0x17_0000, *b"ZBTS"),
        (0x17_8000, *b"ZBSS"),
    ];

    /// NVM3-shaped pages: a real page header followed by object headers and
    /// payloads, deterministic. Not a dump of a unit.
    fn nvm3_like_page(index: u32) -> Vec<u8> {
        let mut page = vec![0xFF; PAGE];
        for (slot, word) in nvm3_header(3 + index).iter().enumerate() {
            page[slot * 4..slot * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        let mut seed = 0x1234_5678u32 ^ index;
        let mut at = 20;
        while at + 64 <= PAGE * 3 / 4 {
            // Object header: key, length, type, then a payload.
            let key = 0x0001_0000 | (at as u32 & 0xFFFF);
            page[at..at + 4].copy_from_slice(&key.to_le_bytes());
            page[at + 4..at + 8].copy_from_slice(&(0x8000_0030u32).to_le_bytes());
            for byte in &mut page[at + 8..at + 56] {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                *byte = (seed >> 24) as u8;
            }
            at += 64;
        }
        page
    }

    fn assert_erased(region: &Region) {
        assert!(
            region
                .bytes(NV_START, FLASH_SIZE)
                .iter()
                .all(|b| *b == 0xFF)
        );
    }

    fn commissioned() -> PersistentSecurityState {
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [0xE2; 8];
        state.pan_id = 0x9BBF;
        state.ieee_address = [0x03, 0x4D, 0x5D, 0xFE, 0xFF, 0xE8, 0xE8, 0xB0];
        state.channel = 25;
        // The coordinator of the network it formed.
        state.short_address = 0x0000;
        state.depth = 0;
        state.parent_address = 0xFFFF;
        state.network_key = [0x5A; 16];
        state.global_counter_limit = 0x0001_2000;
        state
    }

    #[test]
    fn stock_nvm3_is_foreign_and_the_whole_region_is_erased() {
        let region = Region::filled(0xFF);
        for (index, page) in STOCK_NVM3.step_by(PAGE).enumerate() {
            region.fill(page, &nvm3_like_page(index as u32));
        }
        // NVM3 spans the end of the Trust Center window and most of the
        // security window.
        assert!(STOCK_NVM3.start < TRUST_CENTER.end() && STOCK_NVM3.end > SECURITY.start);
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(
            opened,
            Opened::Foreign(ForeignPage {
                address: bus(STOCK_NVM3.start),
                first_word: 0xB29A_0001,
                nvm3: true,
            })
        );
        assert_erased(&region);
        assert_eq!(journal.load(), Ok(None));
        // The erased journal takes a first record and keeps it.
        journal.store(&commissioned()).unwrap();
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(opened, Opened::Existing);
        assert_eq!(journal.load(), Ok(Some(commissioned())));
    }

    #[test]
    fn arbitrary_bytes_and_torn_lookalikes_are_foreign() {
        // Pseudo-random fill of the whole region, with our record magic at
        // every slot start of the security window (no commit, bad CRC).
        let region = Region::filled(0);
        let mut seed = 0xDEAD_BEEFu32;
        let mut bytes = vec![0u8; NV_SIZE];
        for byte in &mut bytes {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *byte = (seed >> 24) as u8;
        }
        region.fill(NV_START, &bytes);
        let mut slot = SECURITY.start;
        while slot < SECURITY.end() {
            region.fill(slot, b"ZBSS");
            slot += 256;
        }
        let (_, opened) = region.open(crate::ROLE_TAG);
        assert!(matches!(opened, Opened::Foreign(_)));
        assert_erased(&region);
        // The same bytes with every page start made to look like ours:
        // the lookalike security records still fail their commit/CRC
        // checks, so nothing is interpreted either.
        region.fill(NV_START, &bytes);
        let mut page = NV_START;
        while page < FLASH_SIZE {
            region.fill(page, b"ZBSS");
            page += PAGE as u32;
        }
        assert_eq!(region.open(crate::ROLE_TAG).1, Opened::Fresh);
        assert_erased(&region);
        // All-zero flash (e.g. a factory "fill 0") is foreign too.
        let region = Region::filled(0);
        assert_eq!(
            region.open(crate::ROLE_TAG).1,
            Opened::Foreign(ForeignPage {
                address: bus(NV_START),
                first_word: 0,
                nvm3: false,
            })
        );
        assert_erased(&region);
    }

    #[test]
    fn our_journal_survives_and_other_windows_are_untouched() {
        let region = Region::filled(0xFF);
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        journal.store(&commissioned()).unwrap();
        // Data of the other journals (here: a record in the TC window).
        let record = marked(b"ZBTS", 0xA5);
        region.fill(TRUST_CENTER.start, &record);
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(opened, Opened::Existing);
        assert_eq!(journal.load(), Ok(Some(commissioned())));
        assert_eq!(
            region.bytes(TRUST_CENTER.start, TRUST_CENTER.start + 16),
            record
        );
    }

    #[test]
    fn another_role_keeps_counters_and_erases_its_tables() {
        let region = Region::filled(0xFF);
        let other = if crate::ROLE_TAG == 0x5203 {
            0x5201
        } else {
            0x5203
        };
        let (mut journal, _) = region.open(other);
        journal.store(&commissioned()).unwrap();
        region.fill(APS_TABLES.start, &marked(b"ZBAT", 0xA5));
        region.fill(IDENTITY.start, &marked(b"ZBST", 0x5A));
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(opened, Opened::RoleChanged);
        let state = journal.load().unwrap().unwrap();
        assert!(!state.commissioned);
        assert_eq!(
            state.global_counter_limit,
            commissioned().global_counter_limit
        );
        assert!(
            region
                .bytes(APS_TABLES.start, SECURITY.start)
                .iter()
                .all(|b| *b == 0xFF)
        );
        // The identity window is kept.
        assert_eq!(
            region.bytes(IDENTITY.start, IDENTITY.start + 16),
            marked(b"ZBST", 0x5A)
        );
    }

    /// The captured stock NVM3: our model of the header is the real one,
    /// and the window is foreign with the NVM3 label (the boot log of an
    /// NCP of ours on that board said exactly this).
    #[test]
    fn the_captured_emberznet_nvm3_is_foreign() {
        assert_eq!(nvm3_header(1), EMBERZNET_745_PAGE_HEADER);
        let region = Region::filled(0xFF);
        // Our journal rolled over far enough for its newest records to sit
        // in the top page 0x0817_E000, above the stock NVM3.
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        let mut state = commissioned();
        for _ in 0..4 * SecurityStore::<View>::SLOTS_PER_SECTOR {
            if region.bytes(STOCK_NVM3.end, STOCK_NVM3.end + 4) == *b"ZBSS" {
                break;
            }
            state.global_counter_limit += 0x400;
            journal.store(&state).unwrap();
        }
        assert_eq!(region.bytes(STOCK_NVM3.end, STOCK_NVM3.end + 4), *b"ZBSS");
        // The stock NVM3 pages as captured, over the rest of the window.
        region.fill(
            STOCK_NVM3.start,
            &vec![0xFF; (STOCK_NVM3.end - STOCK_NVM3.start) as usize],
        );
        for page in STOCK_NVM3.step_by(PAGE) {
            let header: Vec<u8> = EMBERZNET_745_PAGE_HEADER
                .iter()
                .flat_map(|word| word.to_le_bytes())
                .collect();
            region.fill(page, &header);
        }
        region.fill(STOCK_NVM3.start + 20, &[0x5C; 1856]);
        // A valid journal of ours is still there and does not protect it.
        assert_eq!(region.journal().load(), Ok(Some(state)));
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(
            opened,
            Opened::Foreign(ForeignPage {
                address: 0x0817_6000,
                first_word: 0xB29A_0001,
                nvm3: true,
            })
        );
        assert_erased(&region);
        assert_eq!(journal.load(), Ok(None));
    }

    /// The captured page starts of a commissioned NCP of ours are ours.
    #[test]
    fn the_captured_ncp_page_starts_are_ours() {
        let region = Region::filled(0xFF);
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        journal.store(&commissioned()).unwrap();
        for (offset, magic) in CAPTURED_NCP_PAGE_STARTS {
            if offset != SECURITY.start {
                region.fill(offset, &marked(&magic, 0xA5));
            }
        }
        for (offset, magic) in CAPTURED_NCP_PAGE_STARTS {
            assert_eq!(region.bytes(offset, offset + 4), magic);
        }
        let before = region.bytes(NV_START, FLASH_SIZE);
        let (_, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(opened, Opened::Existing);
        assert_eq!(region.bytes(NV_START, FLASH_SIZE), before);
    }

    #[test]
    fn page_start_ownership_is_exact() {
        assert!(ours(u32::MAX));
        for magic in OUR_MAGICS {
            let magic = u32::from_le_bytes(magic);
            assert!(ours(magic));
            // A program (or erase) interrupted on the magic word: some of
            // its zero bits are still 1 (ASCII bytes have bit 7 clear).
            assert!(ours(magic | 0x8080_8080));
            assert!(ours(magic | 0x0000_0001));
            // A word missing one of the magic's 1 bits is not ours.
            assert!(!ours(magic & !(1 << magic.trailing_zeros())));
        }
        // NVM3 page magic either way round, zeros, other formats.
        for word in [
            0xB29A_0001,
            0x0001_B29A,
            0,
            u32::from_le_bytes(*b"LNV1"),
            u32::from_le_bytes(*b"ZBRP"),
            0x1234_5678,
        ] {
            assert!(!ours(word), "{word:#010x}");
        }
        assert!(looks_like_nvm3(0xB29A_0001) && looks_like_nvm3(0x0001_B29A));
        assert!(!looks_like_nvm3(0x1234_5678));
        // Random words pass very rarely (about 1 in 1,600).
        let mut seed = 0xC0FF_EE11;
        let passing = (0..16_384).filter(|_| ours(lcg(&mut seed))).count();
        assert!(passing < 40, "{passing}");
    }

    /// Our own journals after long use never look foreign: real security
    /// records across both sectors (rolled over), every other window's
    /// slots stamped with its magics, and a page start torn while its
    /// magic was programmed.
    #[test]
    fn our_page_starts_are_never_foreign() {
        let region = Region::filled(0xFF);
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        let mut state = commissioned();
        let used = |region: &Region, start: u32| region.bytes(start, start + 4) == *b"ZBSS";
        let both_sectors = |region: &Region| {
            used(region, SECURITY.start)
                && used(region, SECURITY.start + PAGE as u32)
                && used(
                    region,
                    SECURITY.start + SECURITY.sector as u32 + PAGE as u32,
                )
        };
        for _ in 0..4 * SecurityStore::<View>::SLOTS_PER_SECTOR {
            if both_sectors(&region) {
                break;
            }
            state.global_counter_limit += 0x400;
            journal.store(&state).unwrap();
        }
        assert!(both_sectors(&region), "the journal rolled over");
        let mut seed = 0x5EED_0001;
        let windows: [(crate::layout::Window, usize, &[&[u8; 4]]); 5] = [
            (IDENTITY, 64, &[b"ZBST"]),
            (APS_TABLES, 2048, &[b"ZBAT"]),
            (GREEN_POWER, 1024, &[b"ZBGP", b"ZBGS", b"ZBGC"]),
            (CHILDREN, 1024, &[b"ZBCT"]),
            (TRUST_CENTER, 4096, &[b"ZBTS", b"ZBTC"]),
        ];
        for (window, slot, magics) in windows {
            let mut at = window.start;
            let mut n = 0;
            while at < window.end() {
                let mut record = vec![0u8; slot];
                record
                    .iter_mut()
                    .for_each(|b| *b = (lcg(&mut seed) >> 24) as u8);
                record[..4].copy_from_slice(magics[n % magics.len()]);
                region.fill(at, &record);
                at += slot as u32;
                n += 1;
            }
        }
        // A page start torn while its magic was programmed, and an erased
        // page (the other half of a journal).
        let torn = u32::from_le_bytes(*b"ZBCT") | 0x8000_0080;
        region.fill(CHILDREN.start + PAGE as u32, &torn.to_le_bytes());
        region.fill(APS_TABLES.start + PAGE as u32, &vec![0xFF; PAGE]);
        let before = region.bytes(NV_START, FLASH_SIZE);
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(opened, Opened::Existing);
        assert_eq!(journal.load(), Ok(Some(state)));
        assert_eq!(region.bytes(NV_START, FLASH_SIZE), before, "nothing erased");
    }

    /// Whatever the foreign format: one page start that is neither erased
    /// nor ours makes the whole window foreign, even beside a valid journal
    /// of ours.
    #[test]
    fn foreign_data_at_any_page_start_erases_the_window() {
        let words = [0xB29A_0001u32, 0x0001_B29A, 0, 0x1234_5678, 0xFFFF_0000];
        let mut page = NV_START;
        let mut n = 0;
        while page < FLASH_SIZE {
            let word = words[n % words.len()];
            let region = Region::filled(0xFF);
            let (mut journal, _) = region.open(crate::ROLE_TAG);
            journal.store(&commissioned()).unwrap();
            region.fill(page, &word.to_le_bytes());
            let (mut journal, opened) = region.open(crate::ROLE_TAG);
            assert_eq!(
                opened,
                Opened::Foreign(ForeignPage {
                    address: bus(page),
                    first_word: word,
                    nvm3: looks_like_nvm3(word),
                }),
                "{page:#x}"
            );
            assert_erased(&region);
            assert_eq!(journal.load(), Ok(None));
            page += PAGE as u32;
            n += 1;
        }
    }

    #[test]
    fn return_from_stock_erases_stale_records_above_the_stock_nvm3() {
        // Our NCP ran long enough for the security journal to roll over:
        // its newest records sit in the top page 0x0817_E000..0x0818_0000.
        let region = Region::filled(0xFF);
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        let mut state = commissioned();
        let top = |region: &Region| {
            region
                .bytes(STOCK_NVM3.end, FLASH_SIZE)
                .chunks(4)
                .any(|word| word == b"ZBSS")
        };
        while !top(&region) {
            state.global_counter_limit += 0x400;
            journal.store(&state).unwrap();
        }
        // The 32 KiB `nvm3_clear` GBL erases 0x0817_6000..0x0817_E000 only,
        // then the stock EmberZNet formats its NVM3 there.
        region.fill(
            STOCK_NVM3.start,
            &vec![0xFF; (STOCK_NVM3.end - STOCK_NVM3.start) as usize],
        );
        for (index, page) in STOCK_NVM3.step_by(PAGE).enumerate() {
            region.fill(page, &nvm3_like_page(index as u32));
        }
        // The stale records alone would still load: the hazard.
        assert!(region.journal().load().unwrap().is_some());
        // Back on our NCP: the NVM3 pages make the whole window foreign.
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert!(matches!(
            opened,
            Opened::Foreign(ForeignPage { nvm3: true, .. })
        ));
        assert_erased(&region);
        assert_eq!(journal.load(), Ok(None));
    }

    /// The same return path when the stock pages are of a format nothing
    /// here recognises (another stock image or NVM3 version; the captured
    /// EmberZNet 7.4.5 pages do carry the GSDK header): still foreign,
    /// still erased.
    #[test]
    fn return_from_stock_with_unrecognised_pages_erases_stale_records() {
        let region = Region::filled(0xFF);
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        let mut state = commissioned();
        while region.bytes(STOCK_NVM3.end, STOCK_NVM3.end + 4) != *b"ZBSS" {
            state.global_counter_limit += 0x400;
            journal.store(&state).unwrap();
        }
        let mut seed = 0x7454_0745;
        let mut stock = vec![0u8; (STOCK_NVM3.end - STOCK_NVM3.start) as usize];
        stock
            .iter_mut()
            .for_each(|b| *b = (lcg(&mut seed) >> 24) as u8);
        for at in (0..stock.len()).step_by(PAGE) {
            // Some first word that is not ours.
            stock[at..at + 4].copy_from_slice(&(0x0003_0000 | at as u32).to_le_bytes());
        }
        region.fill(STOCK_NVM3.start, &stock);
        assert!(region.journal().load().unwrap().is_some(), "the hazard");
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(
            opened,
            Opened::Foreign(ForeignPage {
                address: bus(STOCK_NVM3.start),
                first_word: 0x0003_0000,
                nvm3: false,
            })
        );
        assert_erased(&region);
        assert_eq!(journal.load(), Ok(None));
    }
}
