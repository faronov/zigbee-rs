//! The NV journals, each bound to its [`crate::layout`] window, and the
//! opening policy. The router role uses the same windows (except the Trust
//! Center stream, which only the NCP has).
//!
//! Opening policy (before the stack reads anything):
//!
//! * no valid security record (blank flash, the stock EmberZNet NVM3 or any
//!   foreign bytes): every NCP window is erased — foreign bytes are never
//!   interpreted;
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
    /// Blank or foreign NV: every window erased.
    Fresh,
    /// Another role's journal: keys cleared (counter limits kept), its
    /// other journals erased.
    RoleChanged,
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
/// Nothing in the region is interpreted unless the security journal holds a
/// valid record: blank, EmberZNet NVM3 or any other foreign bytes fail its
/// magic/commit/CRC checks, so `load` finds no record and the whole region
/// is erased.
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
    if security.load()?.is_none() {
        nv().erase(0, NV_SIZE as u32)
            .map_err(|_| SecurityStoreError::Hardware)?;
        // Re-read the erased journal.
        let _ = security.storage_mut();
        opened = Opened::Fresh;
    }
    if security.select_role_tag(role_tag)? {
        // Everything but the identity and security windows.
        nv().erase(APS_TABLES.start - NV_START, SECURITY.start - NV_START)
            .map_err(|_| SecurityStoreError::Hardware)?;
        opened = Opened::RoleChanged;
    }
    Ok(opened)
}

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

    /// NVM3-shaped pages: an AN1135-style page header (magic `0xB29A`, an
    /// erase counter and its complement, page state) followed by object
    /// headers and payloads, deterministic. Not a dump of a unit: nothing
    /// in the policy depends on the exact NVM3 encoding, only on the bytes
    /// not being a valid zigbee-rs record.
    fn nvm3_like_page(index: u32) -> Vec<u8> {
        let mut page = vec![0xFF; PAGE];
        let header = [
            0xB29A_0001u32,
            0x0000_0003 + index,
            !(0x0000_0003 + index),
            0xFFFF_FFFF,
            0xFFFF_FFFE,
        ];
        for (slot, word) in header.iter().enumerate() {
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
        assert_eq!(opened, Opened::Fresh);
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
        assert_eq!(opened, Opened::Fresh);
        assert_erased(&region);
        // All-zero flash (e.g. a factory "fill 0") is foreign too.
        let region = Region::filled(0);
        assert_eq!(region.open(crate::ROLE_TAG).1, Opened::Fresh);
        assert_erased(&region);
    }

    #[test]
    fn our_journal_survives_and_other_windows_are_untouched() {
        let region = Region::filled(0xFF);
        let (mut journal, _) = region.open(crate::ROLE_TAG);
        journal.store(&commissioned()).unwrap();
        // Data of the other journals (here: a marker in the TC window).
        region.fill(TRUST_CENTER.start, &[0xA5; 16]);
        let (mut journal, opened) = region.open(crate::ROLE_TAG);
        assert_eq!(opened, Opened::Existing);
        assert_eq!(journal.load(), Ok(Some(commissioned())));
        assert_eq!(
            region.bytes(TRUST_CENTER.start, TRUST_CENTER.start + 16),
            [0xA5; 16]
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
        region.fill(APS_TABLES.start, &[0xA5; 16]);
        region.fill(IDENTITY.start, &[0x5A; 16]);
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
            [0x5A; 16]
        );
    }
}
