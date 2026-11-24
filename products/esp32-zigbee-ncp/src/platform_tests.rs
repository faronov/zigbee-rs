use std::cell::RefCell;
use std::rc::Rc;

use super::*;
use crate::fw::{FORMAT_ESP_APP, MAX_CHUNK};
use crate::layout::ota_slot_offset;
use crate::test_support::*;
use esp32_zigbee_devkit_product::ota::RunningImageError;
use esp32_zigbee_devkit_product::sha256::sha256;
use ncp_coordinator_app::platform::slot_state;
use zigbee_runtime::settings_journal::{SettingsError, SettingsRecord};

/// The identity journal as a settings store that survives "reboots".
#[derive(Clone, Default)]
struct Settings(Rc<RefCell<Option<SettingsRecord>>>);

impl SettingsStore for Settings {
    fn load(&mut self) -> Result<Option<SettingsRecord>, SettingsError> {
        Ok(self.0.borrow().clone())
    }
    fn store(&mut self, record: &SettingsRecord) -> Result<(), SettingsError> {
        *self.0.borrow_mut() = Some(record.clone());
        Ok(())
    }
}

const WINDOW: u64 = TRIAL_WINDOW_MS;

fn boot(
    flash: &MockFlash,
    settings: &Settings,
    now_ms: u64,
) -> (Platform<MockFlash, Settings>, BootOutcome) {
    Platform::boot(
        flash.clone(),
        settings.clone(),
        policy(false),
        WINDOW,
        now_ms,
    )
}

fn update(platform: &mut Platform<MockFlash, Settings>) -> Vec<u8> {
    let image = esp_app(9000, "1.3.0");
    let file = envelope(&image, (1, 3, 0), &signing_key(1));
    let request = FwImage {
        image_size: file.len() as u32,
        format: FORMAT_ESP_APP,
        sha256: sha256(&file),
        version: 0x0103_0000,
        flags: 0,
    };
    let begin = platform.fw_begin(&request).unwrap();
    let mut offset = 0;
    for part in file.chunks(begin.max_chunk as usize) {
        offset = platform.fw_write(offset, part).unwrap();
    }
    assert_eq!(platform.fw_finish(), Ok(0));
    platform.fw_activate().unwrap();
    image
}

#[test]
fn a_factory_board_boots_confirmed_with_fw_and_ieee_services() {
    let (platform, outcome) = boot(&MockFlash::new(), &Settings::default(), 0);
    assert_eq!(outcome, BootOutcome::Confirmed);
    assert!(platform.fw_supported());
    assert!(!platform.fw_trial_pending());
    assert!(platform.ieee_rewritable());
    assert!(!platform.pta_supported());
    assert_eq!(platform.boot_ieee_override(), None);
}

#[test]
fn an_update_runs_on_trial_until_fw_confirm() {
    let flash = MockFlash::new();
    let settings = Settings::default();
    let (mut platform, _) = boot(&flash, &settings, 0);
    let image = update(&mut platform);
    let mut slots = [FwSlot {
        slot: 0,
        state: 0,
        version: 0,
        sha256: [0; 32],
    }; 4];
    assert_eq!(platform.fw_slots(&mut slots), Ok(2));
    assert_eq!(slots[1].state, slot_state::STAGED);

    // The NCP reboots into ota_1.
    flash.set_running(1);
    let (mut platform, outcome) = boot(&flash, &settings, 1_000);
    assert_eq!(outcome, BootOutcome::Trial { slot: 1 });
    assert!(platform.fw_trial_pending());
    assert_eq!(platform.fw_slots(&mut slots), Ok(2));
    assert_eq!(slots[1].state, slot_state::RUNNING_TRIAL);
    assert_eq!(slots[1].sha256[..], image[image.len() - 32..]);
    assert_eq!(slots[0].state, slot_state::EMPTY);
    // No new staging while the previous image is the rollback target.
    let request = FwImage {
        image_size: 4096,
        format: FORMAT_ESP_APP,
        sha256: [0; 32],
        version: 0x0104_0000,
        flags: 0,
    };
    assert_eq!(
        platform.fw_begin(&request),
        Err(PlatformError::InvalidState)
    );
    assert_eq!(platform.poll_trial(1_000 + WINDOW - 1), TrialPoll::Pending);

    assert_eq!(platform.fw_confirm(), Ok(()));
    assert!(!platform.fw_trial_pending());
    assert_eq!(platform.poll_trial(1_000 + WINDOW), TrialPoll::Idle);
    // FW_CONFIRM again is a no-op.
    assert_eq!(platform.fw_confirm(), Ok(()));
    assert_eq!(platform.fw_begin(&request).map(|b| b.slot), Ok(0));
    assert_eq!(platform.fw_abort(), Ok(()));

    let (platform, outcome) = boot(&flash, &settings, 0);
    assert_eq!(outcome, BootOutcome::Confirmed);
    assert!(!platform.fw_trial_pending());
}

#[test]
fn the_trial_window_rolls_back_without_fw_confirm() {
    let flash = MockFlash::new();
    let settings = Settings::default();
    let (mut platform, _) = boot(&flash, &settings, 0);
    update(&mut platform);
    flash.set_running(1);
    let (mut platform, outcome) = boot(&flash, &settings, 5_000);
    assert_eq!(outcome, BootOutcome::Trial { slot: 1 });
    assert_eq!(platform.poll_trial(5_000 + WINDOW - 1), TrialPoll::Pending);
    assert_eq!(
        platform.poll_trial(5_000 + WINDOW),
        TrialPoll::Expired {
            rolled_back: Some(0)
        }
    );
    // The root resets; the bootloader starts ota_0 again.
    assert_eq!(flash.otadata().active_slot(), Some(0));
    flash.set_running(0);
    let (platform, outcome) = boot(&flash, &settings, 0);
    assert_eq!(outcome, BootOutcome::Confirmed);
    assert!(!platform.fw_trial_pending());
}

#[test]
fn a_reset_during_the_trial_rolls_back_at_the_next_boot() {
    let flash = MockFlash::new();
    let settings = Settings::default();
    let (mut platform, _) = boot(&flash, &settings, 0);
    update(&mut platform);
    flash.set_running(1);
    assert_eq!(boot(&flash, &settings, 0).1, BootOutcome::Trial { slot: 1 });
    // Crash or reset before FW_CONFIRM: the second boot rolls back.
    assert_eq!(
        boot(&flash, &settings, 0).1,
        BootOutcome::RolledBack { slot: 0 }
    );
    flash.set_running(0);
    assert_eq!(boot(&flash, &settings, 0).1, BootOutcome::Confirmed);
}

#[test]
fn the_ieee_override_applies_at_the_next_boot_and_reverts_to_factory() {
    let flash = MockFlash::new();
    let settings = Settings::default();
    let ieee = [0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80];
    let (mut platform, _) = boot(&flash, &settings, 0);
    assert_eq!(platform.set_ieee_override(Some(ieee)), Ok(()));
    // Not before the reboot.
    assert_eq!(platform.boot_ieee_override(), None);
    let (mut platform, _) = boot(&flash, &settings, 0);
    assert_eq!(platform.boot_ieee_override(), Some(ieee));
    assert_eq!(
        platform.set_ieee_override(Some([0; 8])),
        Err(PlatformError::InvalidParameter)
    );
    assert_eq!(platform.set_ieee_override(Some([0xFF; 8])), Ok(()));
    let (mut platform, _) = boot(&flash, &settings, 0);
    assert_eq!(platform.boot_ieee_override(), None);
    assert_eq!(platform.set_ieee_override(Some(ieee)), Ok(()));
    assert_eq!(platform.set_ieee_override(None), Ok(()));
    assert_eq!(boot(&flash, &settings, 0).0.boot_ieee_override(), None);
    // Staging never touched the identity item (it is not in the OTA
    // flash at all here) and an update keeps it.
    let (mut platform, _) = boot(&flash, &settings, 0);
    platform.set_ieee_override(Some(ieee)).unwrap();
    update(&mut platform);
    flash.set_running(1);
    assert_eq!(
        boot(&flash, &settings, 0).0.boot_ieee_override(),
        Some(ieee)
    );
}

#[test]
fn without_running_image_evidence_fw_is_unsupported_but_ieee_works() {
    let flash = MockFlash::new();
    flash.running.set(Err(RunningImageError::EncryptedFlash));
    let (mut platform, outcome) = boot(&flash, &Settings::default(), 0);
    assert!(matches!(outcome, BootOutcome::FwUnavailable(_)));
    assert!(!platform.fw_supported());
    let request = FwImage {
        image_size: 4096,
        format: FORMAT_ESP_APP,
        sha256: [0; 32],
        version: 0x0104_0000,
        flags: 0,
    };
    assert_eq!(platform.fw_begin(&request), Err(PlatformError::Unsupported));
    assert_eq!(platform.fw_confirm(), Err(PlatformError::Unsupported));
    assert_eq!(platform.poll_trial(u64::MAX), TrialPoll::Idle);
    assert_eq!(platform.set_ieee_override(Some([1; 8])), Ok(()));
    assert_eq!(MAX_CHUNK, 1024);
    assert!(ota_slot_offset(1) > ota_slot_offset(0));
}

#[test]
fn the_product_policy_trusts_the_devkit_signed_ota_keys() {
    let policy = product_policy(0x0001_0000);
    assert!(!policy.keys.is_empty());
    assert_eq!(policy.dev_build, cfg!(feature = "dev-unsigned-fw"));
}

#[derive(Default)]
struct FakeCoex {
    log: Rc<RefCell<Vec<(bool, u8)>>>,
    /// A fault the backend cannot honour (fail closed); `poll` acts on it.
    fault: Rc<RefCell<u32>>,
    enabled: bool,
}

impl CoexControl for FakeCoex {
    fn set(&mut self, enable: bool, priority_mask: u8) -> Result<(), PlatformError> {
        if priority_mask > 0x0F {
            return Err(PlatformError::InvalidParameter);
        }
        if enable && *self.fault.borrow() != 0 {
            return Err(PlatformError::Hardware);
        }
        self.log.borrow_mut().push((enable, priority_mask));
        self.enabled = enable;
        Ok(())
    }
    fn active(&self) -> bool {
        self.enabled && *self.fault.borrow() == 0
    }
    fn poll(&mut self) {
        if *self.fault.borrow() != 0 && self.enabled {
            self.enabled = false;
            self.log.borrow_mut().push((false, 0xFF));
        }
    }
    fn counters(&self) -> CoexCounters {
        CoexCounters {
            tx_denied: self.log.borrow().len() as u32,
            adapter_faults: *self.fault.borrow(),
            ..CoexCounters::default()
        }
    }
    fn reset_counters(&mut self) {
        self.log.borrow_mut().clear();
    }
}

#[test]
fn factory_ieee_and_attached_pta_are_platform_services() {
    let (mut platform, _) = boot(&MockFlash::new(), &Settings::default(), 0);
    assert_eq!(platform.factory_ieee(), None);
    assert_eq!(platform.set_pta(true, 1), Err(PlatformError::Unsupported));
    assert_eq!(mac_group(&platform), []);
    platform.set_factory_ieee([7; 8]);
    assert_eq!(platform.factory_ieee(), Some([7; 8]));
    let coex = FakeCoex::default();
    let log = coex.log.clone();
    platform.attach_coex(Box::leak(Box::new(coex)));
    assert!(platform.pta_supported());
    assert_eq!(platform.pta_active(), Some(false));
    assert_eq!(platform.set_pta(true, 0x05), Ok(()));
    assert_eq!(platform.pta_active(), Some(true));
    assert_eq!(
        platform.set_pta(false, 0x10),
        Err(PlatformError::InvalidParameter)
    );
    assert_eq!(*log.borrow(), [(true, 0x05)]);
    assert_eq!(
        mac_group(&platform),
        [(0x0220, 1), (0x0221, 0), (0x0222, 0), (0x0223, 0)]
    );
    platform.reset_counters(0x40);
    assert_eq!(mac_group(&platform)[0], (0x0220, 1));
    platform.reset_counters(0x02);
    assert_eq!(mac_group(&platform)[0], (0x0220, 0));
}

#[test]
fn a_coex_fault_is_reported_and_polled_off() {
    let (mut platform, _) = boot(&MockFlash::new(), &Settings::default(), 0);
    // Nothing attached: no-op.
    platform.poll_coex();
    assert_eq!(platform.pta_active(), None);
    let coex = FakeCoex::default();
    let (log, fault) = (coex.log.clone(), coex.fault.clone());
    platform.attach_coex(Box::leak(Box::new(coex)));
    assert_eq!(platform.set_pta(true, 0x08), Ok(()));
    platform.poll_coex();
    assert_eq!(*log.borrow(), [(true, 0x08)]);

    *fault.borrow_mut() = 3;
    // Reported off at once, through DIAG too, and stopped by the poll.
    assert_eq!(platform.pta_active(), Some(false));
    assert_eq!(mac_group(&platform)[3], (0x0223, 3));
    platform.poll_coex();
    platform.poll_coex();
    assert_eq!(*log.borrow(), [(true, 0x08), (false, 0xFF)]);
    assert_eq!(platform.set_pta(true, 0x08), Err(PlatformError::Hardware));
    assert_eq!(platform.set_pta(false, 0x08), Ok(()));
    assert_eq!(platform.pta_active(), Some(false));
}

fn mac_group<F: OtaFlash, S: SettingsStore>(platform: &Platform<F, S>) -> Vec<(u16, u32)> {
    let mut out = Vec::new();
    platform.visit_counters(0x02, &mut |id, value| out.push((id, value)));
    out
}
