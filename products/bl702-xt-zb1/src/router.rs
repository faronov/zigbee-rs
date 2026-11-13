//! Router-role persistence, policy and profile.
//!
//! ```text
//! 0xFE000..0x100000 security journal (shared by every role image)
//! 0xFD000..0xFE000  secondary-slot trailer sector (bootloader-owned)
//! 0xFB000..0xFD000  router APS binding/group journal
//! 0xF9000..0xFB000  router child-table journal
//! 0xF7000..0xF9000  router Green Power Proxy Table journal (`green-power`)
//! 0x87000..0xF7000  secondary slot, unused by the router image
//! ```
//!
//! The router image carries no Zigbee OTA client, so the top of the
//! secondary slot below its trailer sector holds the two router journals.
//! The trailer sector, which the bootloader reads for a pending swap, is
//! never touched. A future router OTA path must cap staged images at
//! `GP_PROXY_TABLE_PARTITION_START - SECONDARY_SLOT` (0x70000 bytes) or
//! move these journals. The SED's OTA writer may overwrite them; the shared
//! security journal's role tag keeps a later router from resuming that
//! state.
//!
//! The one [`XipFlash`](bl702_hal::flash::XipFlash) token is shared by the
//! three journals through [`SharedFlash`].

use core::cell::RefCell;
use core::fmt::Debug;

use bl702_xt_zb1_layout::{MCUBOOT, NV_START, SECONDARY_SLOT, SECTOR_SIZE};
use embedded_storage::nor_flash::{NorFlash, NorFlashError};
use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::aps_table_store::{APS_TABLE_JOURNAL_SECTOR_SIZE, ApsTableJournal};
use zigbee_runtime::child_store::{CHILD_JOURNAL_SECTOR_SIZE, ChildTableJournal};
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;
use crate::storage::{FlashWindow, SecurityStore, SharedFlash, security_store};

/// Secondary-slot trailer sector; the router journals end below it.
pub const TRAILER_SECTOR: u32 = MCUBOOT.trailer_sector(SECONDARY_SLOT);

/// APS binding/group journal: two 4 KiB sectors below the trailer sector.
pub const APS_TABLE_PARTITION_SIZE: usize = APS_TABLE_JOURNAL_SECTOR_SIZE * 2;
pub const APS_TABLE_PARTITION_START: u32 = TRAILER_SECTOR - APS_TABLE_PARTITION_SIZE as u32;
/// Child-table journal: two 4 KiB sectors below the APS journal.
pub const CHILD_TABLE_PARTITION_SIZE: usize = CHILD_JOURNAL_SECTOR_SIZE * 2;
pub const CHILD_TABLE_PARTITION_START: u32 =
    APS_TABLE_PARTITION_START - CHILD_TABLE_PARTITION_SIZE as u32;

const _: () = assert!(TRAILER_SECTOR == 0x000F_D000);
const _: () = assert!(TRAILER_SECTOR + SECTOR_SIZE == NV_START);
const _: () = assert!(
    APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE == TRAILER_SECTOR as usize
);
const _: () = assert!(
    CHILD_TABLE_PARTITION_START as usize + CHILD_TABLE_PARTITION_SIZE
        == APS_TABLE_PARTITION_START as usize
);
const _: () = assert!(CHILD_TABLE_PARTITION_START == 0x000F_9000);

/// Green Power Proxy Table journal: two 4 KiB sectors below the child
/// journal (reserved in every router image; used with `green-power`).
pub const GP_PROXY_TABLE_PARTITION_SIZE: usize = GP_PROXY_TABLE_SECTOR_SIZE * 2;
pub const GP_PROXY_TABLE_PARTITION_START: u32 =
    CHILD_TABLE_PARTITION_START - GP_PROXY_TABLE_PARTITION_SIZE as u32;
const GP_PROXY_TABLE_SECTOR_SIZE: usize = 4096;
const _: () = assert!(GP_PROXY_TABLE_PARTITION_START == 0x000F_7000);
const _: () = assert!(GP_PROXY_TABLE_PARTITION_START > SECONDARY_SLOT);
const _: () = assert!(GP_PROXY_TABLE_PARTITION_START.is_multiple_of(SECTOR_SIZE));
const _: () = assert!(GP_PROXY_TABLE_SECTOR_SIZE.is_multiple_of(SECTOR_SIZE as usize));
const _: () = assert!(CHILD_TABLE_PARTITION_START > SECONDARY_SLOT);
const _: () = assert!(CHILD_TABLE_PARTITION_START.is_multiple_of(SECTOR_SIZE));
const _: () = assert!(CHILD_JOURNAL_SECTOR_SIZE.is_multiple_of(SECTOR_SIZE as usize));
const _: () = assert!(APS_TABLE_JOURNAL_SECTOR_SIZE.is_multiple_of(SECTOR_SIZE as usize));

/// 20 ms receive slices; 5..60 s commissioning retry; failed secured
/// rejoins retry on that backoff and never reset the network on their own.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

pub type RouterProfile = DeviceProfile<RangeExtender>;

/// HA Range Extender: Basic + Identify on a routing, child-capable node.
pub const fn router_profile() -> RouterProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

pub type ChildTableFlash<'a, F> =
    FlashWindow<SharedFlash<'a, F>, CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE>;
pub type ApsTableFlash<'a, F> =
    FlashWindow<SharedFlash<'a, F>, APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE>;
pub type ChildStore<'a, F> = ChildTableJournal<ChildTableFlash<'a, F>>;
pub type ApsTableStore<'a, F> = ApsTableJournal<ApsTableFlash<'a, F>>;
pub type RouterSecurityStore<'a, F> = SecurityStore<SharedFlash<'a, F>>;
#[cfg(feature = "green-power")]
pub type GpProxyTableFlash<'a, F> =
    FlashWindow<SharedFlash<'a, F>, GP_PROXY_TABLE_PARTITION_START, GP_PROXY_TABLE_PARTITION_SIZE>;
/// Green Power Proxy Table journal on two 4 KiB sectors.
#[cfg(feature = "green-power")]
pub type GpProxyTableStore<'a, F> =
    zigbee_runtime::gp_store::GpProxyJournal<GpProxyTableFlash<'a, F>, GP_PROXY_TABLE_SECTOR_SIZE>;

/// The router's three journals over the one shared flash.
pub struct RouterStores<'a, F> {
    pub security: RouterSecurityStore<'a, F>,
    pub children: ChildStore<'a, F>,
    pub aps: ApsTableStore<'a, F>,
    #[cfg(feature = "green-power")]
    pub gp_proxy_table: GpProxyTableStore<'a, F>,
}

/// Split the shared main flash into the security, child and APS journals.
/// The windows are disjoint from each other, from the running image and
/// from the secondary-slot trailer (const assertions above).
pub fn router_stores<F: NorFlash>(flash: &RefCell<F>) -> RouterStores<'_, F>
where
    F::Error: NorFlashError + Debug,
{
    RouterStores {
        security: security_store(SharedFlash::new(flash)),
        children: ChildStore::new(
            FlashWindow::new(SharedFlash::new(flash)),
            0,
            CHILD_JOURNAL_SECTOR_SIZE as u32,
        ),
        aps: ApsTableStore::new(
            FlashWindow::new(SharedFlash::new(flash)),
            0,
            APS_TABLE_JOURNAL_SECTOR_SIZE as u32,
        ),
        #[cfg(feature = "green-power")]
        gp_proxy_table: GpProxyTableStore::new(
            FlashWindow::new(SharedFlash::new(flash)),
            0,
            GP_PROXY_TABLE_SECTOR_SIZE as u32,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::storage::PartitionError;
    use crate::storage::mock::MockMainFlash;
    use embedded_storage::nor_flash::NorFlash;
    use zigbee_runtime::profile::ApplicationProfile;

    #[test]
    fn router_policy_is_valid_and_bounded() {
        assert!(ROUTER_POLICY.is_valid());
        assert!(ROUTER_POLICY.max_receive_slice_us <= 20_000);
    }

    #[test]
    fn router_advertises_a_range_extender() {
        let profile = router_profile();
        assert_eq!(profile.device_id(), DeviceId::RANGE_EXTENDER);
        assert_eq!(profile.expected_report_clusters(), 0);
    }

    #[test]
    fn router_journals_sit_below_the_secondary_trailer() {
        assert_eq!(CHILD_TABLE_PARTITION_START, 0x000F_9000);
        assert_eq!(APS_TABLE_PARTITION_START, 0x000F_B000);
        assert_eq!(
            APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE,
            0x000F_D000
        );
        assert_eq!(GP_PROXY_TABLE_PARTITION_START, 0x000F_7000);
        assert_eq!(
            GP_PROXY_TABLE_PARTITION_START as usize + GP_PROXY_TABLE_PARTITION_SIZE,
            CHILD_TABLE_PARTITION_START as usize
        );
        // Room a future router OTA could stage without touching them.
        assert_eq!(GP_PROXY_TABLE_PARTITION_START - SECONDARY_SLOT, 0x7_0000);
    }

    #[test]
    fn windows_confine_writes_to_their_partition() {
        let flash = RefCell::new(MockMainFlash::new());
        let stores = router_stores(&flash);
        let mut children = stores.children.into_storage();
        children.erase(0, SECTOR_SIZE).unwrap();
        children.write(0, &[0xA5; 4]).unwrap();
        assert_eq!(
            children.write(CHILD_TABLE_PARTITION_SIZE as u32, &[0; 4]),
            Err(PartitionError::OutOfBounds)
        );
        let mut aps = stores.aps.into_storage();
        aps.erase(0, SECTOR_SIZE).unwrap();
        aps.write(APS_TABLE_PARTITION_SIZE as u32 - 1, &[0x5A])
            .unwrap();
        assert_eq!(
            aps.write(APS_TABLE_PARTITION_SIZE as u32, &[0]),
            Err(PartitionError::OutOfBounds)
        );
        let bytes = &flash.borrow().bytes;
        let child = CHILD_TABLE_PARTITION_START as usize;
        let aps_last = TRAILER_SECTOR as usize - 1;
        assert_eq!(&bytes[child..child + 4], &[0xA5; 4]);
        assert_eq!(bytes[aps_last], 0x5A);
        assert!(bytes[..child].iter().all(|byte| *byte == 0xFF));
        assert!(bytes[child + 4..aps_last].iter().all(|byte| *byte == 0xFF));
        // The trailer sector and the security journal are untouched.
        assert!(
            bytes[TRAILER_SECTOR as usize..]
                .iter()
                .all(|byte| *byte == 0xFF)
        );
    }

    #[cfg(feature = "green-power")]
    #[test]
    fn gp_window_confines_writes_below_the_child_journal() {
        let flash = RefCell::new(MockMainFlash::new());
        let stores = router_stores(&flash);
        let mut gp = stores.gp_proxy_table.into_storage();
        gp.erase(0, SECTOR_SIZE).unwrap();
        gp.write(0, &[0x3C; 4]).unwrap();
        gp.write(GP_PROXY_TABLE_PARTITION_SIZE as u32 - 1, &[0xC3])
            .unwrap();
        assert_eq!(
            gp.write(GP_PROXY_TABLE_PARTITION_SIZE as u32, &[0; 4]),
            Err(PartitionError::OutOfBounds)
        );
        let bytes = &flash.borrow().bytes;
        let start = GP_PROXY_TABLE_PARTITION_START as usize;
        let last = CHILD_TABLE_PARTITION_START as usize - 1;
        assert_eq!(&bytes[start..start + 4], &[0x3C; 4]);
        assert_eq!(bytes[last], 0xC3);
        assert!(bytes[..start].iter().all(|byte| *byte == 0xFF));
        assert!(bytes[start + 4..last].iter().all(|byte| *byte == 0xFF));
        assert!(bytes[last + 1..].iter().all(|byte| *byte == 0xFF));
    }
}
