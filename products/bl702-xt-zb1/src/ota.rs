//! Zigbee OTA for the XT-ZB1: identity, file version, the staging writer
//! for the secondary slot and the client lifecycle.
//!
//! The product bootloader (`bootloader/`, overwrite-only MCUboot image
//! format) installs a staged image on the next reset. The application only
//! downloads into the secondary slot through [`OtaFlash`], validates the
//! complete image with the key the loader trusts, and writes the upgrade
//! request on activation, after `SensorApp` has checkpointed the security
//! state.
//!
//! # Channel
//!
//! The loader advertises itself and its trusted key in a [`LoaderInfo`]
//! record at [`LOADER_INFO_ADDR`]. Images are accepted only when that
//! record trusts this build's [`OWN_KEY_XY`]; otherwise anything staged
//! would be rejected at boot, so OTA stays disabled ([`Channel::Foreign`],
//! [`Channel::NoLoader`]).
//!
//! # File version
//!
//! The OTA file version is `0x25MMmmpp` from the release version `MM.mm.pp`
//! (the package version, or the TEST-ONLY `BL702_OTA_VERSION_OVERRIDE`),
//! the same scheme as the CC2340 flower product. The MCUboot header carries
//! `MM.mm.pp+0`, which the loader's downgrade prevention compares.

use bl702_xt_zb1_layout::{IMAGE_HEADER_SIZE, LOADER_INFO_ADDR, MCUBOOT, OWN_KEY_XY};
use embedded_storage::nor_flash::{NorFlash, NorFlashError};
use mcuboot_image::{LoaderInfo, SlotPolicy, SlotWriter, TrustedKey};
use sensor_sed_app::{
    OtaActivationOutcome as AppOtaActivationOutcome, OtaEventOutcome as AppOtaEventOutcome,
    OtaLifecycle, OtaServiceOutcome, is_ota_event,
};
use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::firmware_writer::FirmwareWriter;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::ota::{OtaConfig, OtaManager};
use zigbee_runtime::ota_transport::{OtaEventOutcome as RuntimeOtaEventOutcome, OtaSession};
use zigbee_runtime::profile::{ProfileError, WithOta};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::profile::{SensorProfile, sensor_profile};
use crate::storage::OtaFlash;
use crate::{ENDPOINT, VERSION};

/// zigbee-rs development manufacturer code (also used by the ESP32 devkit
/// and flower products). Not a Zigbee Alliance assignment.
pub const MANUFACTURER_CODE: u16 = 0x1234;
/// XT-ZB1 sensor image type. Distinct from the other zigbee-rs products
/// (EFR32 TRADFRI `0x0002`, ESP32 `0x0001`/`0x0002`, flower
/// `0x2340`..`0x2342`).
pub const IMAGE_TYPE: u16 = 0x0702;

/// Most significant byte of every file version of this firmware.
pub const FILE_VERSION_PREFIX: u8 = 0x25;

/// First client query after joining, then one per day.
pub const FIRST_QUERY_DELAY_SECS: u32 = 300;
pub const QUERY_INTERVAL_SECS: u32 = 24 * 60 * 60;
/// Fast-poll window after OTA traffic.
pub const OTA_KEEP_AWAKE_MS: u32 = 120_000;
/// The OTA server: the coordinator's endpoint 1 (Zigbee2MQTT, ZHA).
pub const OTA_SERVER: (u16, u8) = (0x0000, 1);
/// Image Block Request size.
pub const OTA_BLOCK_SIZE: u8 = 48;

/// `0x25MMmmpp`.
pub const fn file_version(major: u8, minor: u8, patch: u8) -> u32 {
    (FILE_VERSION_PREFIX as u32) << 24 | (major as u32) << 16 | (minor as u32) << 8 | patch as u32
}

/// This build's OTA file version.
pub const FILE_VERSION: u32 = file_version(VERSION.0, VERSION.1, VERSION.2);

/// Which bootloader runs the application, and so whether images can be
/// accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// The product bootloader trusting this build's own key.
    Own,
    /// A zigbee-rs loader trusting another key: staged images would be
    /// rejected at boot.
    Foreign,
    /// No loader record (or an unreadable one): nothing is known to install
    /// images.
    NoLoader,
}

impl Channel {
    /// Classify the loader record read from [`LOADER_INFO_ADDR`].
    pub fn detect(info: Option<&LoaderInfo>) -> Self {
        match info {
            None => Self::NoLoader,
            Some(info) if info.trusts(&OWN_KEY_XY) => Self::Own,
            Some(_) => Self::Foreign,
        }
    }

    /// Read the loader record from physical flash. A read error disables
    /// OTA rather than guessing.
    pub fn read<F: NorFlash>(flash: &mut F) -> Self {
        match LoaderInfo::read(flash, LOADER_INFO_ADDR) {
            Ok(info) => Self::detect(info.as_ref()),
            Err(_) => Self::NoLoader,
        }
    }

    pub const fn accepts_images(self) -> bool {
        matches!(self, Self::Own)
    }
}

/// The OTA client configuration. Offers are never auto-accepted: the
/// lifecycle applies the channel policy first.
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

/// The product's firmware writer over main flash `F`.
pub type Bl702Writer<F> = SlotWriter<OtaFlash<F>>;

/// The staging writer for the secondary slot. It refuses images not signed
/// with `key_xy`, with another header size, or older than the running image,
/// and writes the upgrade request only on activation.
pub fn writer_with_key<F: NorFlash>(flash: F, key_xy: [u8; 64]) -> Bl702Writer<F>
where
    F::Error: NorFlashError,
{
    SlotWriter::new(
        OtaFlash::new(flash),
        MCUBOOT,
        SlotPolicy {
            key: TrustedKey::new(key_xy),
            header_size: IMAGE_HEADER_SIZE as u16,
        },
    )
}

/// [`writer_with_key`] for this build's own key.
pub fn writer<F: NorFlash>(flash: F) -> Bl702Writer<F>
where
    F::Error: NorFlashError,
{
    writer_with_key(flash, OWN_KEY_XY)
}

/// The sensor endpoint with its OTA client.
pub type OtaSensorProfile<W> = WithOta<SensorProfile, W>;

/// The sensor endpoint plus the OTA Upgrade client staging into the
/// secondary slot of `flash`.
pub fn sensor_ota_profile<F: NorFlash>(
    flash: F,
) -> Result<OtaSensorProfile<Bl702Writer<F>>, ProfileError>
where
    F::Error: NorFlashError,
{
    WithOta::new(
        sensor_profile(),
        OtaManager::new(writer(flash), ota_config()),
    )
}

/// Client-initiated Query Next Image schedule, in joined seconds (the
/// lifecycle only services OTA while joined).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuerySchedule {
    remaining_secs: u32,
}

impl QuerySchedule {
    pub const fn new() -> Self {
        Self {
            remaining_secs: FIRST_QUERY_DELAY_SECS,
        }
    }

    /// Advance by `elapsed_secs`; `true` when a query is due, which restarts
    /// the daily interval.
    pub fn advance(&mut self, elapsed_secs: u16) -> bool {
        self.remaining_secs = self.remaining_secs.saturating_sub(u32::from(elapsed_secs));
        if self.remaining_secs == 0 {
            self.remaining_secs = QUERY_INTERVAL_SECS;
            true
        } else {
            false
        }
    }

    pub const fn remaining_secs(&self) -> u32 {
        self.remaining_secs
    }
}

impl Default for QuerySchedule {
    fn default() -> Self {
        Self::new()
    }
}

/// OTA lifecycle for [`sensor_sed_app::SensorApp`]: periodic queries, Image
/// Notify, the channel policy, fast polling while downloading, and a reset
/// into the bootloader after activation.
///
/// Unlike the CC2340 flower product there is no supply gate: the XT-ZB1
/// battery path has no established chemistry or brown-out threshold yet.
pub struct Bl702OtaLifecycle {
    session: OtaSession,
    channel: Channel,
    schedule: QuerySchedule,
    /// Resets into the bootloader once the upgrade request is written.
    reset: fn() -> !,
}

impl Bl702OtaLifecycle {
    pub const fn new(channel: Channel, reset: fn() -> !) -> Self {
        Self {
            session: OtaSession::new(),
            channel,
            schedule: QuerySchedule::new(),
            reset,
        }
    }

    pub const fn channel(&self) -> Channel {
        self.channel
    }

    fn log_status(status: Option<&StackEvent>) {
        match status {
            Some(StackEvent::OtaImageAvailable { version, size }) => {
                log::info!("[ota] offer version=0x{version:08X} size={size}");
            }
            Some(StackEvent::OtaProgress { percent }) => log::info!("[ota] {percent}%"),
            Some(StackEvent::OtaDelayedActivation { delay_secs }) => {
                log::info!("[ota] activation delayed {delay_secs}s");
            }
            Some(StackEvent::OtaComplete) => log::info!("[ota] verified"),
            Some(StackEvent::OtaFailed) => log::warn!("[ota] failed"),
            _ => {}
        }
    }
}

impl<M, S, W> OtaLifecycle<M, S, OtaSensorProfile<W>> for Bl702OtaLifecycle
where
    M: MacDriver,
    S: SecurityStateStore,
    W: FirmwareWriter,
{
    const ENABLED: bool = true;

    fn is_active(&self, profile: &OtaSensorProfile<W>) -> bool {
        OtaSession::is_active(profile.ota())
    }

    fn next_deadline_ms(&self, _profile: &OtaSensorProfile<W>) -> Option<u32> {
        // The query schedule and the manager count whole seconds of the
        // lifecycle's service calls; while a session is active SensorApp
        // already uses the fast-poll cadence.
        None
    }

    async fn handle_event(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, OtaSensorProfile<W>>,
        event: &StackEvent,
    ) -> AppOtaEventOutcome {
        let (device, profile) = node.device_and_profile_mut();
        let manager = profile.ota_mut();
        let outcome = self
            .session
            .handle_event(device, manager, ENDPOINT, event)
            .await;
        match outcome {
            RuntimeOtaEventOutcome::NotOta if !is_ota_event(event) => {
                return AppOtaEventOutcome::NotHandled;
            }
            // A status event the transport itself raised earlier.
            RuntimeOtaEventOutcome::NotOta => Self::log_status(Some(event)),
            RuntimeOtaEventOutcome::Ignored => {}
            RuntimeOtaEventOutcome::Consumed(status) => Self::log_status(status.as_ref()),
        }

        if self.session.offer_pending(manager) {
            if self.channel.accepts_images() {
                let status = self.session.accept_offer(device, manager).await;
                Self::log_status(status.as_ref());
            } else {
                log::warn!("[ota] offer declined: channel {:?}", self.channel);
                self.session.decline_offer(manager);
            }
        }

        let active = OtaSession::is_active(manager);
        AppOtaEventOutcome::Handled {
            keep_awake_ms: active.then_some(OTA_KEEP_AWAKE_MS),
            activation_pending: self.session.activation_pending()
                || matches!(event, StackEvent::OtaComplete),
        }
    }

    async fn service(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, OtaSensorProfile<W>>,
        elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        let (device, profile) = node.device_and_profile_mut();
        let manager = profile.ota_mut();
        let mut queried = false;
        if self.schedule.advance(elapsed_secs) && self.channel.accepts_images() {
            log::info!("[ota] query next image (current 0x{FILE_VERSION:08X})");
            self.session
                .query_next_image(device, manager, OTA_SERVER)
                .await;
            queried = OtaSession::is_active(manager);
        }
        let status = self.session.service(device, manager, elapsed_secs).await;
        Self::log_status(status.as_ref());

        OtaServiceOutcome {
            keep_awake_ms: (queried || status.is_some()).then_some(OTA_KEEP_AWAKE_MS),
            activation_pending: self.session.activation_pending(),
        }
    }

    fn activate(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, OtaSensorProfile<W>>,
    ) -> AppOtaActivationOutcome {
        // SensorApp has just checkpointed the security state. The writer
        // writes the upgrade request magic; the bootloader installs on the
        // next reset.
        let manager = node.profile_mut().ota_mut();
        match self.session.activate(manager) {
            Ok(()) => {
                log::info!("[ota] activated, resetting into the bootloader");
                (self.reset)()
            }
            Err(error) => {
                log::warn!("[ota] activation failed: {error:?}");
                AppOtaActivationOutcome::Failed
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::cell::RefCell;
    use std::vec::Vec;

    use super::*;
    use crate::storage::SharedFlash;
    use crate::storage::mock::MockMainFlash;
    use bl702_xt_zb1_layout::{NV_START, PRIMARY_SLOT, SECONDARY_SLOT, SLOT_SIZE};
    use mcuboot_image::{BOOT_MAGIC, MagicState, trailer::read_magic};
    use zigbee_runtime::firmware_writer::FirmwareError;

    // Shared imgtool vectors (header 0x100, TEST-ONLY key).
    const TEST_KEY_XY: &[u8; 64] =
        include_bytes!("../../../mcuboot-image/tests/vectors/test-key.xy");
    const TEST_V1: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v1.bin");
    const TEST_V2: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v2.bin");
    const TI_V2: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/ti-v2.bin");

    fn test_writer<F: NorFlash>(flash: F) -> Bl702Writer<F>
    where
        F::Error: NorFlashError,
    {
        writer_with_key(flash, *TEST_KEY_XY)
    }

    /// `imgtool sign --pad`: fill the slot and append the request magic.
    fn padded(image: &[u8]) -> Vec<u8> {
        let mut out = std::vec![0xFF; SLOT_SIZE as usize];
        out[..image.len()].copy_from_slice(image);
        let magic = SLOT_SIZE as usize - BOOT_MAGIC.len();
        out[magic..].copy_from_slice(&BOOT_MAGIC);
        out
    }

    /// Flash with `running` in the primary slot, a loader record and a
    /// journal page in NV.
    fn device(running: &[u8]) -> MockMainFlash {
        let mut flash = MockMainFlash::new();
        let primary = PRIMARY_SLOT as usize;
        flash.bytes[primary..primary + running.len()].copy_from_slice(running);
        let info = LOADER_INFO_ADDR as usize;
        flash.bytes[info..info + 32].copy_from_slice(&LoaderInfo::new(0, 1, &OWN_KEY_XY).encode());
        let nv = NV_START as usize;
        flash.bytes[nv..nv + 64].fill(0x5A);
        flash
    }

    fn download<F: NorFlash>(writer: &mut Bl702Writer<F>, image: &[u8])
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

    fn request<F: embedded_storage::nor_flash::ReadNorFlash>(flash: &mut F) -> MagicState {
        read_magic(flash, SECONDARY_SLOT, SLOT_SIZE).ok().unwrap()
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
    fn identity_and_config() {
        let config = ota_config();
        assert_eq!(config.manufacturer_code, 0x1234);
        assert_eq!(config.image_type, 0x0702);
        assert_eq!(config.current_version, FILE_VERSION);
        assert_eq!(config.endpoint, ENDPOINT);
        assert!(!config.auto_accept);
    }

    #[test]
    fn channel_follows_the_loader_record() {
        let ours = LoaderInfo::new(0, 1, &OWN_KEY_XY);
        let other = LoaderInfo::new(0, 1, &[0x42; 64]);
        assert_eq!(Channel::detect(Some(&ours)), Channel::Own);
        assert_eq!(Channel::detect(Some(&other)), Channel::Foreign);
        assert_eq!(Channel::detect(None), Channel::NoLoader);
        assert!(Channel::Own.accepts_images());
        assert!(!Channel::Foreign.accepts_images());
        assert!(!Channel::NoLoader.accepts_images());

        let mut flash = MockMainFlash::new();
        assert_eq!(Channel::read(&mut flash), Channel::NoLoader);
        let at = LOADER_INFO_ADDR as usize;
        flash.bytes[at..at + 32].copy_from_slice(&ours.encode());
        assert_eq!(Channel::read(&mut flash), Channel::Own);
        flash.bytes[at + 20] ^= 1;
        assert_eq!(Channel::read(&mut flash), Channel::NoLoader);
        flash.bytes[at..at + 32].copy_from_slice(&other.encode());
        assert_eq!(Channel::read(&mut flash), Channel::Foreign);
    }

    #[test]
    fn image_is_staged_and_requested_only_on_activation() {
        let flash = RefCell::new(device(TEST_V1));
        let nv_before = flash.borrow().bytes[NV_START as usize..].to_vec();
        let low_before = flash.borrow().bytes[..SECONDARY_SLOT as usize].to_vec();
        let mut writer = test_writer(SharedFlash::new(&flash));
        download(&mut writer, TEST_V2);
        writer.verify(TEST_V2.len() as u32, None).unwrap();
        assert_eq!(request(writer.flash_mut()), MagicState::Erased);
        writer.activate().unwrap();
        assert_eq!(request(writer.flash_mut()), MagicState::Good);
        let flash = flash.borrow();
        let secondary = SECONDARY_SLOT as usize;
        assert_eq!(&flash.bytes[secondary..secondary + TEST_V2.len()], TEST_V2);
        assert_eq!(&flash.bytes[NV_START as usize..], &nv_before[..]);
        assert_eq!(&flash.bytes[..SECONDARY_SLOT as usize], &low_before[..]);
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
        // Image signed with another key.
        let mut flash = device(TEST_V1);
        let mut writer = test_writer(&mut flash);
        download(&mut writer, TI_V2);
        assert_eq!(
            writer.verify(TI_V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        // Older than the running image.
        let mut flash = device(TEST_V2);
        let mut writer = test_writer(&mut flash);
        download(&mut writer, TEST_V1);
        assert_eq!(
            writer.verify(TEST_V1.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        // Truncated download.
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

    /// The staged image is what the bootloader installs: run the shared
    /// boot engine on the result, then check the journal survived.
    #[test]
    fn bootloader_installs_the_staged_image() {
        let mut flash = device(TEST_V1);
        let nv_before = flash.bytes[NV_START as usize..].to_vec();
        {
            let mut writer = test_writer(&mut flash);
            download(&mut writer, TEST_V2);
            writer.verify(TEST_V2.len() as u32, None).unwrap();
            writer.activate().unwrap();
        }
        let key = TrustedKey::new(*TEST_KEY_XY);
        let booted = mcuboot_image::boot(&mut flash, &MCUBOOT, &key, |_| {}).unwrap();
        assert_eq!(booted.update, mcuboot_image::Update::Installed);
        assert_eq!(booted.vector_table, bl702_xt_zb1_layout::APP_START);
        let primary = PRIMARY_SLOT as usize;
        assert_eq!(&flash.bytes[primary..primary + TEST_V2.len()], TEST_V2);
        assert_eq!(request(&mut flash), MagicState::Erased);
        assert_eq!(&flash.bytes[NV_START as usize..], &nv_before[..]);
    }

    #[test]
    fn queries_five_minutes_after_join_then_daily() {
        let mut schedule = QuerySchedule::new();
        let mut due_at = Vec::new();
        let mut now = 0u32;
        for step in [30u16, 1, 250, 19, 7, 3_600, 30].iter().cycle().take(200) {
            now += u32::from(*step);
            if schedule.advance(*step) {
                due_at.push(now);
            }
        }
        assert_eq!(due_at[0], 300, "first query once 300 s have elapsed");
        for pair in due_at.windows(2) {
            let gap = pair[1] - pair[0];
            assert!((QUERY_INTERVAL_SECS..QUERY_INTERVAL_SECS + 3_600).contains(&gap));
        }
        let mut schedule = QuerySchedule::new();
        assert!(schedule.advance(u16::MAX));
        assert!(!schedule.advance(1));
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

        let flash = RefCell::new(MockMainFlash::new());
        let with_ota = sensor_ota_profile(SharedFlash::new(&flash)).unwrap();
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
