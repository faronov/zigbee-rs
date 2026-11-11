//! Pre-reflash compatibility gate for a live ZBT-1.
//!
//! Before a commissioned unit is reflashed (for example when the zigbee-rs
//! loader is chained above the Gecko Bootloader), take a full 1 MiB SWD flash
//! backup and run:
//!
//! ```text
//! ZBT1_FLASH_BACKUP=/path/to/backup.bin cargo test --test backup_compat -- --nocapture
//! ```
//!
//! The gate proves that this build's journals resume the unit's persisted
//! state instead of taking the destructive fresh-reset path: the security
//! journal must hold a commissioned router-tagged record (otherwise the new
//! image would erase it and the router would have to rejoin from scratch),
//! and the child and APS journals must decode for the same network. Without
//! the environment variable the test is a no-op, so CI stays hermetic.

use embedded_storage::nor_flash::{ErrorType, NorFlash, NorFlashErrorKind, ReadNorFlash};
use ha_connect_zbt1_product::{
    APS_TABLE_PARTITION_SIZE, APS_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE,
    CHILD_TABLE_PARTITION_START, PERSISTENCE_PARTITION_SIZE, PERSISTENCE_PARTITION_START, role,
};
use zigbee_runtime::aps_table_store::{ApsTableJournal, ApsTableStore};
use zigbee_runtime::child_store::{ChildTableJournal, ChildTableStore};
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::security_store::SecurityStateStore;

const SECTOR: usize = 8 * 1024;
const FLASH_SIZE: usize = 1024 * 1024;

/// Read-only view of one partition of the backup; any write or erase fails
/// the gate, because opening persisted state must never modify it.
struct BackupPartition {
    bytes: Vec<u8>,
}

impl ErrorType for BackupPartition {
    type Error = NorFlashErrorKind;
}

impl ReadNorFlash for BackupPartition {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, out: &mut [u8]) -> Result<(), Self::Error> {
        let start = offset as usize;
        let end = start
            .checked_add(out.len())
            .filter(|end| *end <= self.bytes.len())
            .ok_or(NorFlashErrorKind::OutOfBounds)?;
        out.copy_from_slice(&self.bytes[start..end]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.bytes.len()
    }
}

impl NorFlash for BackupPartition {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = SECTOR;

    fn write(&mut self, offset: u32, _: &[u8]) -> Result<(), Self::Error> {
        panic!("gate must not write persisted state (offset {offset:#x})")
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        panic!("gate must not erase persisted state ({from:#x}..{to:#x})")
    }
}

fn partition(image: &[u8], start: u32, size: usize) -> BackupPartition {
    BackupPartition {
        bytes: image[start as usize..start as usize + size].to_vec(),
    }
}

#[test]
fn backup_persistence_resumes_without_a_fresh_reset() {
    let Some(path) = std::env::var_os("ZBT1_FLASH_BACKUP") else {
        eprintln!("ZBT1_FLASH_BACKUP not set; backup compatibility gate skipped");
        return;
    };
    let image = std::fs::read(&path).expect("read ZBT1_FLASH_BACKUP");
    assert_eq!(
        image.len(),
        FLASH_SIZE,
        "expected a full 1 MiB flash backup"
    );

    let mut security = SecurityStateJournal::<_, SECTOR>::new_with_sector_size(
        partition(
            &image,
            PERSISTENCE_PARTITION_START,
            PERSISTENCE_PARTITION_SIZE,
        ),
        0,
        SECTOR as u32,
    );
    let state = security
        .load()
        .expect("security journal readable")
        .expect("valid security record (else the new image would fresh-reset and erase it)");
    let tag = security.product_tag().expect("tag readable");
    assert_eq!(
        tag,
        Some(role::ROUTER),
        "record must carry the router role tag"
    );
    assert!(state.commissioned, "router must be commissioned");
    eprintln!(
        "security: addr=0x{:04X} pan=0x{:04X} ch={} epan={:02X?} ieee={:02X?} \
         nwk-counter-limit={:#x} tclk-limit={:#x}",
        state.short_address,
        state.pan_id,
        state.channel,
        state.extended_pan_id,
        state.ieee_address,
        state.global_counter_limit,
        state.tclk_counter_limit,
    );

    let mut children = ChildTableJournal::<_, SECTOR>::new_with_sector_size(
        partition(
            &image,
            CHILD_TABLE_PARTITION_START,
            CHILD_TABLE_PARTITION_SIZE,
        ),
        0,
        SECTOR as u32,
    );
    match children.load().expect("child journal decodes") {
        Some(table) => {
            // Same acceptance rule as the runtime: an empty snapshot carries
            // no network binding.
            assert!(table.is_empty() || table.matches_network(&state.extended_pan_id));
            eprintln!("child table: {} children", table.len());
        }
        None => eprintln!("child table: empty"),
    }

    let mut aps = ApsTableJournal::<_, SECTOR>::new_with_sector_size(
        partition(&image, APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE),
        0,
        SECTOR as u32,
    );
    match aps.load().expect("APS journal decodes") {
        Some(tables) => {
            assert!(tables.is_empty() || tables.matches_network(&state.extended_pan_id));
            eprintln!("APS tables: empty={}", tables.is_empty());
        }
        None => eprintln!("APS tables: none stored"),
    }
}
