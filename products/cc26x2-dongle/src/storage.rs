//! Security-state opening policy.
//!
//! The shipped Z-Stack image keeps its NV inside this product's application
//! and partition windows; foreign bytes are never interpreted. If no valid
//! journal record is found, both security sectors are erased before the
//! store reaches the stack. A valid journal is preserved. A power loss
//! during the erase repeats this path on the next boot.

pub use crate::journal::SecurityStore;
use crate::journal::{PERSISTENCE_PARTITION_SIZE, security_journal};
use embedded_storage::nor_flash::NorFlash;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MigrationDisposition {
    ExistingJournal,
    FreshReset,
}

pub fn security_store(
    flash: cc26x2_hal::flash::Flash,
) -> Result<(SecurityStore, MigrationDisposition), SecurityStoreError> {
    prepare_security_store(security_journal(flash))
}

fn prepare_security_store<S: NorFlash, const SECTOR_SIZE: usize>(
    mut store: SecurityStateJournal<S, SECTOR_SIZE>,
) -> Result<(SecurityStateJournal<S, SECTOR_SIZE>, MigrationDisposition), SecurityStoreError> {
    if store.load()?.is_some() {
        return Ok((store, MigrationDisposition::ExistingJournal));
    }
    store
        .storage_mut()
        .erase(0, PERSISTENCE_PARTITION_SIZE as u32)
        .map_err(|_| SecurityStoreError::Hardware)?;
    Ok((store, MigrationDisposition::FreshReset))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::journal::SECURITY_SECTOR_SIZE;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use zigbee_runtime::security_store::PersistentSecurityState;

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

        fn foreign() -> Self {
            let mut flash = Self::erased();
            flash.data[..512].fill(0x5A);
            flash
        }
    }

    impl ErrorType for MockFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let end = start + bytes.len();
            if end > self.data.len() {
                return Err(NorFlashErrorKind::OutOfBounds);
            }
            bytes.copy_from_slice(&self.data[start..end]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECURITY_SECTOR_SIZE;

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            if !start.is_multiple_of(4) || !bytes.len().is_multiple_of(4) {
                return Err(NorFlashErrorKind::NotAligned);
            }
            for (old, new) in self.data[start..start + bytes.len()].iter_mut().zip(bytes) {
                if *old & *new != *new {
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
            self.data[from as usize..to as usize].fill(0xFF);
            self.erases += 1;
            Ok(())
        }
    }

    fn journal(flash: MockFlash) -> SecurityStateJournal<MockFlash, { SECURITY_SECTOR_SIZE }> {
        SecurityStateJournal::new_with_sector_size(flash, 0, SECURITY_SECTOR_SIZE as u32)
    }

    #[test]
    fn foreign_bytes_take_the_fresh_reset_path() {
        let (mut store, disposition) =
            prepare_security_store(journal(MockFlash::foreign())).unwrap();
        assert_eq!(disposition, MigrationDisposition::FreshReset);
        assert_eq!(store.load(), Ok(None));
        assert!(store.storage().data.iter().all(|b| *b == 0xFF));
    }

    #[test]
    fn a_valid_journal_survives_rollover_and_reopen() {
        let mut existing = journal(MockFlash::erased());
        let slots =
            SecurityStateJournal::<MockFlash, { SECURITY_SECTOR_SIZE }>::SLOTS_PER_SECTOR + 1;
        for generation in 1..=slots {
            let mut state = PersistentSecurityState::empty();
            state.global_counter_limit = generation as u32 * 0x400;
            existing.store(&state).unwrap();
        }
        let erases = existing.storage().erases;
        let (mut reopened, disposition) = prepare_security_store(existing).unwrap();
        assert_eq!(disposition, MigrationDisposition::ExistingJournal);
        assert_eq!(
            reopened.load().unwrap().unwrap().global_counter_limit,
            slots as u32 * 0x400
        );
        assert_eq!(reopened.storage().erases, erases);
    }

    #[test]
    fn fresh_reset_fails_closed_when_erase_fails() {
        let mut flash = MockFlash::foreign();
        flash.fail_erase = true;
        assert!(matches!(
            prepare_security_store(journal(flash)),
            Err(SecurityStoreError::Hardware)
        ));
    }
}
