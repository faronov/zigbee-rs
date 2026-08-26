//! Product-owned BRD4181A partition and security-state opening policy.
//!
//! Older EFR32MG21 firmware used `ApplicationNv`/`LogStructuredNv` over the
//! same final two physical flash sectors. That format did not durably encode
//! the complete network key, TCLK, identity, and counter reservations required
//! by `SecurityStateStore`, so manufacturing a partial conversion would risk
//! restoring an unsafe key/counter pair.
//!
//! Migration is therefore deliberately a fresh reset: if no valid current
//! journal record is found, both 8 KiB sectors are erased before the store is
//! handed to the stack. An already-valid journal is preserved. A power loss
//! during the erase simply repeats this path on the next boot; no legacy
//! network state is ever treated as a current security snapshot.

pub use crate::journal::SecurityStore;
use crate::journal::{PERSISTENCE_PARTITION_SIZE, SECURITY_SECTOR_SIZE, security_journal};
use efr32mg21_hal::flash::Efr32mg21Flash;
use embedded_storage::nor_flash::NorFlash;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MigrationDisposition {
    ExistingJournal,
    FreshReset,
}

pub fn security_store(
    flash: Efr32mg21Flash,
) -> Result<(SecurityStore, MigrationDisposition), SecurityStoreError> {
    prepare_security_store(security_journal(flash))
}

fn prepare_security_store<S: NorFlash, const SECTOR_SIZE: usize>(
    mut store: SecurityStateJournal<S, SECTOR_SIZE>,
) -> Result<(SecurityStateJournal<S, SECTOR_SIZE>, MigrationDisposition), SecurityStoreError> {
    if store.load()?.is_some() {
        return Ok((store, MigrationDisposition::ExistingJournal));
    }

    // This is the explicit, intentionally destructive migration boundary from
    // legacy ApplicationNv (or blank/corrupt flash) to the security journal.
    // Erase both physical sectors; never expose old bytes to commissioning.
    store
        .storage_mut()
        .erase(0, PERSISTENCE_PARTITION_SIZE as u32)
        .map_err(|_| SecurityStoreError::Hardware)?;

    Ok((store, MigrationDisposition::FreshReset))
}

const _: () = assert!(PERSISTENCE_PARTITION_SIZE == SECURITY_SECTOR_SIZE * 2);

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use zigbee_runtime::security_store::PersistentSecurityState;

    #[derive(Clone)]
    struct MockFlash {
        data: [u8; PERSISTENCE_PARTITION_SIZE],
        erases: usize,
        fail_erase: bool,
    }

    impl MockFlash {
        fn erased() -> Self {
            Self {
                data: [0xFF; PERSISTENCE_PARTITION_SIZE],
                erases: 0,
                fail_erase: false,
            }
        }

        fn legacy() -> Self {
            let mut flash = Self::erased();
            flash.data[..256].fill(0xA5);
            flash
        }

        fn range(
            address: u32,
            length: usize,
        ) -> Result<core::ops::Range<usize>, NorFlashErrorKind> {
            let start = address as usize;
            let end = start
                .checked_add(length)
                .filter(|end| *end <= PERSISTENCE_PARTITION_SIZE)
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            Ok(start..end)
        }
    }

    impl ErrorType for MockFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, address: u32, output: &mut [u8]) -> Result<(), Self::Error> {
            let range = Self::range(address, output.len())?;
            output.copy_from_slice(&self.data[range]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECURITY_SECTOR_SIZE;

        fn write(&mut self, address: u32, input: &[u8]) -> Result<(), Self::Error> {
            if !(address as usize).is_multiple_of(Self::WRITE_SIZE)
                || !input.len().is_multiple_of(Self::WRITE_SIZE)
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = Self::range(address, input.len())?;
            for (old, new) in self.data[range].iter_mut().zip(input) {
                if (*old & *new) != *new {
                    return Err(NorFlashErrorKind::Other);
                }
                *old &= *new;
            }
            Ok(())
        }

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if self.fail_erase {
                return Err(NorFlashErrorKind::Other);
            }
            if from >= to
                || !(from as usize).is_multiple_of(Self::ERASE_SIZE)
                || !(to as usize).is_multiple_of(Self::ERASE_SIZE)
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = Self::range(from, (to - from) as usize)?;
            self.data[range].fill(0xFF);
            self.erases += 1;
            Ok(())
        }
    }

    fn journal(flash: MockFlash) -> SecurityStateJournal<MockFlash, { SECURITY_SECTOR_SIZE }> {
        SecurityStateJournal::new_with_sector_size(flash, 0, SECURITY_SECTOR_SIZE as u32)
    }

    #[test]
    fn legacy_bytes_take_the_explicit_fresh_reset_path() {
        let (mut store, disposition) =
            prepare_security_store(journal(MockFlash::legacy())).unwrap();
        assert_eq!(disposition, MigrationDisposition::FreshReset);
        assert_eq!(store.load(), Ok(None));
        assert_eq!(store.storage().erases, 1);
        assert!(store.storage().data.iter().all(|byte| *byte == 0xFF));
    }

    #[test]
    fn blank_flash_is_idempotently_prepared_for_fresh_commissioning() {
        let (mut store, disposition) =
            prepare_security_store(journal(MockFlash::erased())).unwrap();
        assert_eq!(disposition, MigrationDisposition::FreshReset);
        assert_eq!(store.load(), Ok(None));
        assert_eq!(store.storage().erases, 1);
    }

    #[test]
    fn an_eight_kib_sector_journal_rolls_over_and_is_never_erased_by_migration() {
        let mut existing = journal(MockFlash::erased());
        for generation in
            1..=(SecurityStateJournal::<MockFlash, { SECURITY_SECTOR_SIZE }>::SLOTS_PER_SECTOR + 1)
        {
            let mut state = PersistentSecurityState::empty();
            state.global_counter_limit = generation as u32 * 0x400;
            existing.store(&state).unwrap();
        }

        let prior_erases = existing.storage().erases;
        let (mut reopened, disposition) = prepare_security_store(existing).unwrap();
        assert_eq!(disposition, MigrationDisposition::ExistingJournal);
        assert_eq!(
            reopened.load().unwrap().unwrap().global_counter_limit,
            (SecurityStateJournal::<MockFlash, { SECURITY_SECTOR_SIZE }>::SLOTS_PER_SECTOR as u32
                + 1)
                * 0x400
        );
        assert_eq!(reopened.storage().erases, prior_erases);
    }

    #[test]
    fn fresh_reset_fails_closed_when_physical_erase_fails() {
        let mut flash = MockFlash::legacy();
        flash.fail_erase = true;
        let result = prepare_security_store(journal(flash));
        assert!(matches!(result, Err(SecurityStoreError::Hardware)));
    }
}
