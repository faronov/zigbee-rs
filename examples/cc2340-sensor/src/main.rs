//! LP-EM-CC2340R5 composition root for the shared sleepy-sensor lifecycle.
//!
//! The product crate owns endpoint clusters/reporting and lifecycle policy;
//! the board crate owns monotonic time, raw BTN1/BTN2 and LED1/LED2 resources,
//! flash, and reset. This composition maps those resources to lifecycle
//! adapters. [`SensorApp`] owns
//! commissioning, polling, reporting, reset/rejoin, and durable security
//! checkpoints.
//!
//! This remains hardware bring-up firmware. The fallback and partial-SDK
//! images stop with distinct `FirmwareUnavailable` and
//! `RadioConfigUnavailable` diagnostics. Even a fully configured image must
//! not be treated as commissionable until radio HIL passes and a verified
//! entropy source replaces the fail-closed backend.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;

use core::mem::MaybeUninit;

use cortex_m as _;
use panic_halt as _;

use cc2340_sensor_product as product;
use embassy_executor::Spawner;
use lp_em_cc2340r5 as board;
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, NoOta, SensorApp, SensorSedParts, SensorStatus, StatusSink,
};
use zigbee_mac::cc2340::{Cc2340RadioPhy, Cc2340SoftMac, RadioError};
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_zcl::clusters::basic::PowerSource;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut diagnostics = platform::init_diagnostics();
    debug_assert_eq!(
        platform::disposition(),
        platform::DiagnosticsDisposition::RttLog
    );
    let Some(resources) = board::take() else {
        platform::halt();
    };
    let board::Resources {
        led1,
        led2,
        button1,
        button2,
        flash,
        batmon,
        reset,
    } = resources;
    let wake = platform::LaunchPadWake::new(button1, button2);
    let mut status = platform::StatusLeds::new(led1, led2);
    let supervisor = platform::ResetOnlySupervisor::new(reset);
    let supervisor_diagnostics = supervisor.diagnostics();
    debug_assert_eq!(
        supervisor_diagnostics.watchdog,
        platform::WatchdogDisposition::Unavailable
    );
    debug_assert_eq!(
        supervisor.watchdog_disposition(),
        supervisor_diagnostics.watchdog
    );
    debug_assert_eq!(supervisor_diagnostics.heartbeat_count, 0);

    status.set(SensorStatus::Off);
    log::info!(
        "[CC2340] starting {} / {} with shared SensorApp",
        board::BOARD_NAME,
        board::TARGET_PART
    );

    let ieee = match board::identity::factory_ieee_address() {
        Ok(address) => address,
        Err(error) => {
            log::error!("[CC2340] factory identity unavailable: {error}");
            fatal(&mut status);
        }
    };
    log::info!("[CC2340] guarded factory EUI-64: {:02X?}", ieee);

    let mut profile = product::profile::sensor_profile();
    let device = build_device(ieee, &mut profile, &mut status);

    let mut security_store = product::storage::security_store(flash);
    log::info!(
        "[CC2340] durable security journal 0x{:08X}..0x{:08X}",
        product::storage::SECURITY_PARTITION_START,
        product::storage::SECURITY_PARTITION_END
    );

    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => log::warn!("[CC2340] cleared persisted state for a different EUI-64"),
        Ok(false) => {}
        Err(error) => {
            diagnostics.record(DiagnosticEvent::SecurityFailure(error));
            fatal(&mut status);
        }
    }

    log::warn!(
        "[CC2340] environment={:?}; battery={:?}; humidity is a synthetic drift until a sensor is fitted",
        product::environment::source_disposition(),
        product::battery::source_disposition()
    );
    log::warn!(
        "[CC2340] commissioning is not claimed: entropy is unavailable and MAC/power HIL is open"
    );

    let node = ZigbeeNode::new(device, &mut security_store, &mut profile);
    let mut app = SensorApp::new(
        node,
        &product::policy::SENSOR_POLICY,
        SensorSedParts {
            wake,
            status,
            environment: product::environment::BatmonEnvironment::new(batmon),
            battery: product::battery::BatmonBattery::new(batmon),
            ota: NoOta,
            actions: product::policy::USER_ACTIONS,
            supervisor,
            diagnostics,
        },
    )
    .expect("CC2340 SensorApp requires manual runtime polling and a valid policy");

    app.run().await
}

/// Bring up the radio and build the MAC and the device in static storage.
///
/// Not inlined, so that the PHY, the MAC constructor's `Result` and the
/// builder stay out of the embassy main-task future, which is live under
/// every later call.
#[inline(never)]
fn build_device(
    ieee: zigbee_types::IeeeAddress,
    profile: &mut product::profile::SensorProfile,
    status: &mut platform::StatusLeds,
) -> &'static mut ZigbeeDevice<Cc2340SoftMac> {
    let mut phy = match Cc2340RadioPhy::new(ieee) {
        Ok(phy) => phy,
        Err(error) => {
            log::error!("[CC2340] rejected factory identity: {error:?}");
            fatal(status);
        }
    };
    log::warn!(
        "[CC2340] entropy={:?} (fail closed); radio-sleep={:?}",
        phy.entropy_disposition(),
        phy.radio_sleep_disposition()
    );

    if let Err(error) = phy.initialize_radio() {
        match error {
            RadioError::FirmwareUnavailable => {
                log::error!("[CC2340] radio firmware unavailable (fallback build)");
            }
            RadioError::RadioConfigUnavailable => {
                log::error!("[CC2340] PHY/board radio configuration unavailable (partial SDK)");
            }
            _ => log::error!("[CC2340] radio initialization failed: {error:?}"),
        }
        fatal(status);
    }
    log::warn!("[CC2340] radio initialized; MAC ACK timing and power behavior are not qualified");
    let mac = match phy.into_soft_mac() {
        Ok(mac) => mac,
        Err(error) => {
            log::error!("[CC2340] MAC construction failed: {error:?}");
            fatal(status);
        }
    };

    // Build the device in static storage: by-value `build()` needs a ~22 KB
    // `assemble` frame, larger than this part's whole stack.
    static mut DEVICE_STORAGE: MaybeUninit<ZigbeeDevice<Cc2340SoftMac>> = MaybeUninit::uninit();
    ZigbeeDevice::builder(mac)
        .power_mode(product::policy::SENSOR_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        // SAFETY: `main` calls this once, so this is the only reference ever
        // taken to DEVICE_STORAGE.
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) })
}

fn fatal(status: &mut platform::StatusLeds) -> ! {
    status.set(SensorStatus::Fault);
    platform::halt()
}
