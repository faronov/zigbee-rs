//! Zigbee OTA for the loader-linked (`zrs-boot`) BRD4183A sleepy sensor:
//! identity, file version, the flash split and the staging writer for the
//! secondary slot.
//!
//! The zigbee-rs loader (`bootloaders/efr32s2-boot`, overwrite install,
//! MCUboot image format, [`boot/devkit.layout`](crate::boot_layout))
//! installs a staged image on the next reset. The application downloads
//! into the secondary slot through [`mcuboot_image::StagingFlash`],
//! validates the complete image with the keys both it and the loader trust,
//! and writes the upgrade request on activation, after `SensorApp` has
//! checkpointed the security state. The sequencing is the shared
//! [`sensor_sed_app::ota_client::ImageOtaClient`] ([`client`]).
//!
//! An installed image is on trial until it confirms itself (one write into
//! the primary slot's trial log) after its first acknowledged report on the
//! network. The overwrite loader cannot revert: an unconfirmed image keeps
//! running and the loader only reports it.
//!
//! # File version
//!
//! `0x25MMmmpp` from the release version `MM.mm.pp` (the package version,
//! or the TEST-ONLY `EFR32MG22_OTA_VERSION_OVERRIDE`), as for the BL702 and
//! CC2340 flower products. The MCUboot header carries `MM.mm.pp+0`, which
//! the loader's downgrade prevention compares.

use efr32s2_hal::flash::{Efr32s2Flash, FlashPartition};
use embedded_storage::nor_flash::{NorFlash, NorFlashError, ReadNorFlash};
use mcuboot_image::{
    KeySet, LoaderChannel, Mode, SlotPolicy, SlotWriter, StagingFlash, loader_ids, shared_keys,
};
use sensor_sed_app::ota_client::{ImageOtaClient, OtaClientPolicy};
use zigbee_runtime::ota::{OtaConfig, OtaManager};
use zigbee_runtime::profile::{ProfileError, WithOta};

use crate::boot_layout::{
    IMAGE_HEADER_SIZE, JOURNAL_START, LOADER_INFO_ADDR, MCUBOOT, PRIMARY_SLOT, SECONDARY_SLOT,
    SLOT_SIZE, TRUSTED_KEYS,
};
use crate::profile::{SensorProfile, sensor_profile};
use crate::{ENDPOINT, VERSION};

/// zigbee-rs development manufacturer code (also used by the ESP32, BL702
/// and flower products). Not a Zigbee Alliance assignment.
pub const MANUFACTURER_CODE: u16 = 0x1234;
/// BRD4183A EFR32MG22 sensor image type. Distinct from the other zigbee-rs
/// products (EFR32 TRADFRI `0x0002`, ESP32 `0x0001`/`0x0002`, BL702
/// `0x0702`, flower `0x2340`..`0x2342`).
pub const IMAGE_TYPE: u16 = 0x0022;

/// Most significant byte of every file version of this firmware.
pub const FILE_VERSION_PREFIX: u8 = 0x25;
/// Image Block Request size.
pub const OTA_BLOCK_SIZE: u8 = 48;

/// Query timing and server: the coordinator's endpoint 1 (ZHA,
/// Zigbee2MQTT), first query five minutes after joining, then daily.
pub const CLIENT_POLICY: OtaClientPolicy = OtaClientPolicy::coordinator(ENDPOINT);

/// `0x25MMmmpp`.
pub const fn file_version(major: u8, minor: u8, patch: u8) -> u32 {
    (FILE_VERSION_PREFIX as u32) << 24 | (major as u32) << 16 | (minor as u32) << 8 | patch as u32
}

/// This build's OTA file version.
pub const FILE_VERSION: u32 = file_version(VERSION.0, VERSION.1, VERSION.2);

/// Identity record for `tools/zrs_boot_ota.py`, which reads it from the
/// application ELF (symbol `ZRS_OTA_IDENTITY`) to write the Zigbee OTA
/// header and the MCUboot version of the image it signs:
/// `[manufacturer | image type << 16, file version]`.
pub const OTA_IDENTITY: [u32; 2] = [
    MANUFACTURER_CODE as u32 | (IMAGE_TYPE as u32) << 16,
    FILE_VERSION,
];

/// The OTA client configuration. Offers are never auto-accepted: the
/// client applies the channel policy first.
pub const fn ota_config() -> OtaConfig {
    OtaConfig {
        manufacturer_code: MANUFACTURER_CODE,
        image_type: IMAGE_TYPE,
        current_version: FILE_VERSION,
        endpoint: ENDPOINT,
        block_size: OTA_BLOCK_SIZE,
        auto_accept: false,
        hardware_version: None,
    }
}

/// Size of the flash window below the security journal: the loader, both
/// slots. Offsets equal physical addresses.
const BELOW_JOURNAL: usize = JOURNAL_START as usize;

// The window holds the loader records and both slots, and ends where the
// security journal begins.
const _: () = assert!(BELOW_JOURNAL as u32 == crate::PERSISTENCE_PARTITION_START);
const _: () = assert!(LOADER_INFO_ADDR < JOURNAL_START);
const _: () = assert!(PRIMARY_SLOT + SLOT_SIZE <= JOURNAL_START);
const _: () = assert!(SECONDARY_SLOT + SLOT_SIZE <= JOURNAL_START);

/// Internal flash below the security journal.
pub type LowFlash = FlashPartition<0, BELOW_JOURNAL>;

/// The OTA writer's flash: [`LowFlash`] bounded by [`StagingFlash`] to the
/// secondary slot and the running image's trial log.
pub type OtaFlash = StagingFlash<LowFlash>;

/// Split the unique internal-flash handle into the OTA writer's window
/// (below [`JOURNAL_START`]) and the handle for
/// [`crate::storage::security_store`], whose partition view is bounded to
/// the journal at `JOURNAL_START..`.
pub fn ota_flash(flash: Efr32s2Flash) -> (Efr32s2Flash, LowFlash) {
    // SAFETY: `LowFlash` ends where the security journal begins
    // (`boot_layout` asserts the journal is `link/memory.x`'s persistence
    // partition) and the returned handle is only used through the journal's
    // partition view. This function consumes the only safe handle before
    // minting the window. Flash is used from one executor; no interrupt
    // handler issues MSC operations.
    let low = unsafe { LowFlash::new_unchecked() };
    (flash, low)
}

/// Which loader starts this image, and so whether images are accepted.
pub fn channel<F: ReadNorFlash>(flash: &mut F) -> LoaderChannel {
    LoaderChannel::read(flash, LOADER_INFO_ADDR, &TRUSTED_KEYS)
}

/// The product's firmware writer over flash `F` (absolute addresses).
pub type Mg22Writer<F> = SlotWriter<StagingFlash<F>>;

/// The staging writer for the secondary slot. It refuses images not signed
/// by one of `keys`, with another header size, or older than the running
/// image, and writes the upgrade request only on activation.
pub fn writer_with_keys<F: NorFlash>(flash: F, keys: KeySet) -> Mg22Writer<F>
where
    F::Error: NorFlashError,
{
    SlotWriter::new(
        StagingFlash::new(flash, MCUBOOT),
        MCUBOOT,
        SlotPolicy {
            keys,
            header_size: IMAGE_HEADER_SIZE as u16,
            mode: Mode::Overwrite,
        },
    )
}

/// [`writer_with_keys`] for this build's keys that the installed loader
/// trusts (none — every image refused — without a loader record).
pub fn writer<F: NorFlash>(mut flash: F) -> Mg22Writer<F>
where
    F::Error: NorFlashError,
{
    let keys = shared_keys(
        loader_ids(&mut flash, LOADER_INFO_ADDR).as_ref(),
        &TRUSTED_KEYS,
    );
    writer_with_keys(flash, keys)
}

/// The sensor endpoint with its OTA client.
pub type OtaSensorProfile<W> = WithOta<SensorProfile, W>;

/// The sensor endpoint plus the OTA Upgrade client staging into the
/// secondary slot of `flash`.
pub fn sensor_ota_profile<F: NorFlash>(
    flash: F,
) -> Result<OtaSensorProfile<Mg22Writer<F>>, ProfileError>
where
    F::Error: NorFlashError,
{
    WithOta::new(
        sensor_profile(),
        OtaManager::new(writer(flash), ota_config()),
    )
}

/// The `SensorApp` OTA lifecycle: queries and accepts images only from
/// [`LoaderChannel::Own`]; `reset` restarts into the loader after
/// activation.
pub const fn client(channel: LoaderChannel, reset: fn() -> !) -> ImageOtaClient {
    ImageOtaClient::new(CLIENT_POLICY, channel.accepts_images(), reset)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec::Vec;

    use super::*;
    use crate::boot_layout::{BOOT_CONFIG, JOURNAL_END, LOADER_BLOCK_ADDR, LOADER_VERSION};
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind};
    use mcuboot_image::{
        BOOT_MAGIC, BootConfig, LoaderInfo, MagicState, Trial, TrialState, TrustedIds, TrustedKey,
        loader_block, trailer::read_magic,
    };
    use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};
    use zigbee_runtime::ota_transport::OtaSession;

    // Shared imgtool vectors: header 0x100, TEST-ONLY key.
    const TEST_KEY_XY: &[u8; 64] =
        include_bytes!("../../../mcuboot-image/tests/vectors/test-key.xy");
    const TEST_V1: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v1.bin");
    const TEST_V2: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v2.bin");
    const TI_V2: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/ti-v2.bin");
    const TEST_HEADER: u16 = 0x100;

    static TEST_KEYS: [TrustedKey; 1] = [TrustedKey::new(*TEST_KEY_XY)];

    /// 512 KiB of NOR flash with the EFR32 geometry (8 KiB pages, 4-byte
    /// writes that only clear bits).
    struct MockFlash {
        bytes: Vec<u8>,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                bytes: std::vec![0xFF; JOURNAL_END as usize],
            }
        }

        fn range(
            &self,
            from: u32,
            len: usize,
        ) -> Result<core::ops::Range<usize>, NorFlashErrorKind> {
            let start = from as usize;
            start
                .checked_add(len)
                .filter(|end| *end <= self.bytes.len())
                .map(|end| start..end)
                .ok_or(NorFlashErrorKind::OutOfBounds)
        }
    }

    impl ErrorType for MockFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let range = self.range(offset, bytes.len())?;
            bytes.copy_from_slice(&self.bytes[range]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = 0x2000;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if !from.is_multiple_of(0x2000) || !to.is_multiple_of(0x2000) || from > to {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = self.range(from, (to - from) as usize)?;
            self.bytes[range].fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if !offset.is_multiple_of(4) || !bytes.len().is_multiple_of(4) {
                return Err(NorFlashErrorKind::NotAligned);
            }
            let range = self.range(offset, bytes.len())?;
            for (old, new) in self.bytes[range].iter_mut().zip(bytes) {
                *old &= *new;
            }
            Ok(())
        }
    }

    /// The product writer's staging view and layout with the TEST-ONLY key
    /// and the shared vectors' 0x100-byte header.
    fn test_writer<F: NorFlash>(flash: F) -> Mg22Writer<F>
    where
        F::Error: NorFlashError,
    {
        SlotWriter::new(
            StagingFlash::new(flash, MCUBOOT),
            MCUBOOT,
            SlotPolicy {
                keys: KeySet::new(&TEST_KEYS),
                header_size: TEST_HEADER,
                mode: Mode::Overwrite,
            },
        )
    }

    /// `imgtool sign --pad`: fill the slot and append the request magic.
    fn padded(image: &[u8]) -> Vec<u8> {
        let mut out = std::vec![0xFF; SLOT_SIZE as usize];
        out[..image.len()].copy_from_slice(image);
        let magic = SLOT_SIZE as usize - BOOT_MAGIC.len();
        out[magic..].copy_from_slice(&BOOT_MAGIC);
        out
    }

    /// Flash with `running` in the primary slot, this build's loader block
    /// and a journal page.
    fn device(running: &[u8]) -> MockFlash {
        let mut flash = MockFlash::new();
        let primary = PRIMARY_SLOT as usize;
        flash.bytes[primary..primary + running.len()].copy_from_slice(running);
        let block = LOADER_BLOCK_ADDR as usize;
        flash.bytes[block..block + 64]
            .copy_from_slice(&loader_block(LOADER_VERSION, &TRUSTED_KEYS));
        let journal = JOURNAL_START as usize;
        flash.bytes[journal..journal + 64].fill(0x5A);
        flash
    }

    fn download<F: NorFlash>(writer: &mut Mg22Writer<F>, image: &[u8])
    where
        F::Error: NorFlashError,
    {
        writer.erase_slot().unwrap();
        for (index, block) in image.chunks(OTA_BLOCK_SIZE as usize).enumerate() {
            writer
                .write_block((index * OTA_BLOCK_SIZE as usize) as u32, block)
                .unwrap();
        }
    }

    fn request<F: ReadNorFlash>(flash: &mut F) -> MagicState {
        read_magic(flash, SECONDARY_SLOT, SLOT_SIZE).ok().unwrap()
    }

    fn outside_secondary(flash: &MockFlash) -> (Vec<u8>, Vec<u8>) {
        (
            flash.bytes[..SECONDARY_SLOT as usize].to_vec(),
            flash.bytes[JOURNAL_START as usize..].to_vec(),
        )
    }

    #[test]
    fn file_version_follows_the_release_version() {
        assert_eq!(file_version(0, 1, 0), 0x2500_0100);
        assert_eq!(file_version(2, 10, 3), 0x2502_0A03);
        assert_eq!(FILE_VERSION, file_version(VERSION.0, VERSION.1, VERSION.2));
        if !crate::VERSION_OVERRIDDEN {
            let expected = file_version(
                env!("CARGO_PKG_VERSION_MAJOR").parse().unwrap(),
                env!("CARGO_PKG_VERSION_MINOR").parse().unwrap(),
                env!("CARGO_PKG_VERSION_PATCH").parse().unwrap(),
            );
            assert_eq!(FILE_VERSION, expected);
            assert_eq!(crate::SW_BUILD, env!("CARGO_PKG_VERSION"));
        }
    }

    #[test]
    fn identity_config_and_client_policy() {
        let config = ota_config();
        assert_eq!(config.manufacturer_code, 0x1234);
        assert_eq!(config.image_type, 0x0022);
        assert_eq!(config.current_version, FILE_VERSION);
        assert_eq!(config.endpoint, ENDPOINT);
        assert!(!config.auto_accept);
        assert_eq!(OTA_IDENTITY, [0x0022_1234, FILE_VERSION]);
        assert_eq!(CLIENT_POLICY.server, (0x0000, 1));
        assert_eq!(CLIENT_POLICY.endpoint, ENDPOINT);
        fn reset() -> ! {
            panic!("reset")
        }
        assert!(client(LoaderChannel::Own, reset).accepts_images());
        assert!(!client(LoaderChannel::Foreign, reset).accepts_images());
        assert!(!client(LoaderChannel::NoLoader, reset).accepts_images());
    }

    #[test]
    fn channel_follows_the_loader_record() {
        // ECDSA P-256 only (the `ed25519` feature is not enabled here).
        let TrustedKey::P256 { xy: own } = TRUSTED_KEYS[0];
        let ours = LoaderInfo::new(0, 1, &own);
        let other = LoaderInfo::new(0, 1, &[0x42; 64]);
        let mut flash = MockFlash::new();
        assert_eq!(channel(&mut flash), LoaderChannel::NoLoader);
        let at = LOADER_INFO_ADDR as usize;
        flash.bytes[at..at + 32].copy_from_slice(&ours.encode());
        assert_eq!(channel(&mut flash), LoaderChannel::Own);
        flash.bytes[at + 20] ^= 1;
        assert_eq!(channel(&mut flash), LoaderChannel::NoLoader);
        flash.bytes[at..at + 32].copy_from_slice(&other.encode());
        assert_eq!(channel(&mut flash), LoaderChannel::Foreign);
        // The block the loader build places (LoaderKeys + LoaderInfo).
        let mut flash = device(TEST_V1);
        assert_eq!(channel(&mut flash), LoaderChannel::Own);
        let ids = TrustedIds::read(&mut flash, LOADER_INFO_ADDR)
            .unwrap()
            .unwrap();
        assert!(
            shared_keys(Some(&ids), &TRUSTED_KEYS)
                .iter()
                .eq(&TRUSTED_KEYS[..])
        );
    }

    /// The writer of a device without a loader record refuses everything.
    #[test]
    fn writer_without_a_loader_trusts_no_key() {
        let mut flash = MockFlash::new();
        let primary = PRIMARY_SLOT as usize;
        flash.bytes[primary..primary + TEST_V1.len()].copy_from_slice(TEST_V1);
        let mut writer = writer(&mut flash);
        download(&mut writer, TEST_V2);
        assert_eq!(
            writer.verify(TEST_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
    }

    /// The product writer requires this layout's header size: an image
    /// signed for another header is refused even with a trusted key.
    #[test]
    fn product_writer_requires_the_layout_header() {
        assert_eq!(IMAGE_HEADER_SIZE, 0x200);
        let mut flash = device(TEST_V1);
        let mut writer = writer_with_keys(&mut flash, KeySet::new(&TEST_KEYS));
        download(&mut writer, TEST_V2);
        assert_eq!(
            writer.verify(TEST_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
    }

    #[test]
    fn image_is_staged_and_requested_only_on_activation() {
        let mut flash = device(TEST_V1);
        let before = outside_secondary(&flash);
        {
            let mut writer = test_writer(&mut flash);
            download(&mut writer, TEST_V2);
            writer.verify(TEST_V2.len() as u32, None).unwrap();
            assert_eq!(request(writer.flash_mut()), MagicState::Erased);
            writer.activate().unwrap();
            assert_eq!(request(writer.flash_mut()), MagicState::Good);
        }
        let secondary = SECONDARY_SLOT as usize;
        assert_eq!(&flash.bytes[secondary..secondary + TEST_V2.len()], TEST_V2);
        assert!(outside_secondary(&flash) == before);
    }

    #[test]
    fn padded_image_withholds_its_magic_until_activation() {
        let mut flash = device(TEST_V1);
        let image = padded(TEST_V2);
        let mut writer = test_writer(&mut flash);
        download(&mut writer, &image);
        assert_eq!(request(writer.flash_mut()), MagicState::Erased);
        writer.verify(image.len() as u32, None).unwrap();
        assert_eq!(request(writer.flash_mut()), MagicState::Erased);
        writer.activate().unwrap();
        assert_eq!(request(writer.flash_mut()), MagicState::Good);
    }

    #[test]
    fn wrong_key_downgrade_and_truncation_are_refused() {
        let mut flash = device(TEST_V1);
        let mut writer = test_writer(&mut flash);
        download(&mut writer, TI_V2);
        assert_eq!(
            writer.verify(TI_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        let mut flash = device(TEST_V2);
        let mut writer = test_writer(&mut flash);
        download(&mut writer, TEST_V1);
        assert_eq!(
            writer.verify(TEST_V1.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        let mut flash = device(TEST_V1);
        let mut writer = test_writer(&mut flash);
        download(&mut writer, &TEST_V2[..TEST_V2.len() - 100]);
        assert_eq!(
            writer.verify(TEST_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(writer.activate(), Err(FirmwareError::ActivateFailed));
        assert_eq!(request(writer.flash_mut()), MagicState::Erased);
    }

    /// The staged image is what the loader installs: run the shared boot
    /// engine with this layout, then check the loader and journal survived.
    #[test]
    fn loader_installs_the_staged_image_and_it_confirms_itself() {
        let config = BootConfig {
            keys: &TEST_KEYS,
            trial_boots: 3,
            ..BOOT_CONFIG
        };
        let mut flash = device(TEST_V1);
        let loader_before = flash.bytes[..PRIMARY_SLOT as usize].to_vec();
        let journal_before = flash.bytes[JOURNAL_START as usize..].to_vec();
        {
            let mut writer = test_writer(&mut flash);
            download(&mut writer, TEST_V2);
            writer.verify(TEST_V2.len() as u32, None).unwrap();
            writer.activate().unwrap();
        }
        let booted = mcuboot_image::boot(&mut flash, &config, |_| {}).unwrap();
        assert_eq!(booted.update, mcuboot_image::Update::Installed);
        assert_eq!(booted.vector_table, PRIMARY_SLOT + u32::from(TEST_HEADER));
        assert_eq!(booted.trial, Trial::Testing { boot: 1 });
        let primary = PRIMARY_SLOT as usize;
        assert_eq!(&flash.bytes[primary..primary + TEST_V2.len()], TEST_V2);
        assert_eq!(request(&mut flash), MagicState::Erased);
        {
            let mut writer = test_writer(&mut flash);
            assert_eq!(
                writer.trial_state(),
                Some(TrialState::Testing { attempts: 1 })
            );
            writer.confirm_image().unwrap();
            assert_eq!(writer.trial_state(), Some(TrialState::Confirmed));
            writer.confirm_image().unwrap();
        }
        let booted = mcuboot_image::boot(&mut flash, &config, |_| {}).unwrap();
        assert_eq!(booted.trial, Trial::Confirmed);
        assert!(flash.bytes[..PRIMARY_SLOT as usize] == loader_before[..]);
        assert!(flash.bytes[JOURNAL_START as usize..] == journal_before[..]);
    }

    /// Whatever the writer is asked, the staging view refuses the loader,
    /// the running image outside its trial log and the journal.
    #[test]
    fn staging_view_protects_the_loader_image_and_journal() {
        let mut flash = device(TEST_V1);
        let before = flash.bytes.clone();
        let mut view = StagingFlash::new(&mut flash, MCUBOOT);
        assert!(view.erase(0, 0x2000).is_err());
        assert!(view.erase(PRIMARY_SLOT, PRIMARY_SLOT + 0x2000).is_err());
        assert!(view.write(LOADER_BLOCK_ADDR, &[0; 4]).is_err());
        assert!(view.write(PRIMARY_SLOT, &[0; 4]).is_err());
        assert!(view.write(JOURNAL_START, &[0; 4]).is_err());
        assert!(view.erase(JOURNAL_START, JOURNAL_START + 0x2000).is_err());
        let mut byte = [0];
        assert!(view.read(JOURNAL_START, &mut byte).is_err());
        assert!(flash.bytes == before);
    }

    #[test]
    fn ota_profile_adds_only_the_client_cluster() {
        use zigbee_mac::mock::MockMac;
        use zigbee_runtime::ZigbeeDevice;
        use zigbee_runtime::profile::ApplicationProfile;
        use zigbee_zcl::ClusterId;

        fn clusters<P: ApplicationProfile>(profile: &P) -> (Vec<ClusterId>, Vec<ClusterId>) {
            let device: ZigbeeDevice<MockMac> =
                ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
                    .endpoint(
                        profile.endpoint(),
                        profile.profile_id(),
                        profile.device_id(),
                        |endpoint| profile.configure_endpoint(endpoint),
                    )
                    .build();
            let endpoint = &device.endpoints()[0];
            (
                endpoint.server_clusters.to_vec(),
                endpoint.client_clusters.to_vec(),
            )
        }

        let mut flash = MockFlash::new();
        let with_ota = sensor_ota_profile(&mut flash).unwrap();
        assert!(!OtaSession::is_active(with_ota.ota()));
        let (servers, clients) = clusters(&with_ota);
        let (plain_servers, plain_clients) = clusters(&sensor_profile());
        assert_eq!(servers, plain_servers);
        assert!(plain_clients.is_empty());
        assert_eq!(clients, [ClusterId::OTA_UPGRADE]);
    }

    #[test]
    fn declined_offer_does_not_touch_flash() {
        let mut flash = device(TEST_V1);
        let before = flash.bytes.clone();
        let mut writer = test_writer(&mut flash);
        writer.abort().unwrap();
        assert!(flash.bytes == before);
    }
}
