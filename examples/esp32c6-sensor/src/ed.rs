//! ESP32-C6 always-on End Device (`--no-default-features --features ed`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! (`EspSoftMac`, hardware auto-ACK) under
//! [`router_app::AlwaysOnEndDeviceApp`]. Basic + Identify on endpoint 1;
//! status goes to the USB Serial/JTAG console, BOOT (GPIO9) re-arms
//! commissioning after a Leave.
//!
//! Identity, policy and the security journal are owned by
//! `products/esp32-zigbee-devkit`. No OTA client: this image is installed
//! over the serial/JTAG link into `ota_0`.

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

mod roles;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use esp32_zigbee_devkit_product as product;
use esp_backtrace as _;
use product::end_device::EndDeviceProfile;
use product::executor::block_on;
use roles::{halt, CommissioningTrigger, LogDiagnostics, LogStatus, ResetSupervisor};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
use zigbee_mac::esp::EspSoftMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, ZigbeeDevice,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = EspSoftMac<'static>;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    EndDeviceProfile,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[esp_hal::main]
fn main() -> ! {
    let started = roles::start(product::END_DEVICE_MODEL);
    let trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.radio, started.aes);
    block_on(run(app, trigger))
}

async fn run(app: &'static mut App, mut trigger: CommissioningTrigger) -> ! {
    if let Err(error) = app.initialize().await {
        roles::fatal(app.parts_mut(), ARCHETYPE, error);
    }
    loop {
        if trigger.poll(app.awaiting_commissioning_request()) {
            app.request_commissioning();
        }
        // Bounded by the product's 20 ms receive slice.
        if let Err(error) = app.step().await {
            roles::fatal(app.parts_mut(), ARCHETYPE, error);
        }
    }
}

/// Build MAC, device, store and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
) -> &'static mut App {
    let Ok(mac) = roles::radio_phy(radio, aes).into_soft_mac() else {
        log::error!("[ESP32-C6] MAC init failed");
        halt()
    };
    let ieee = mac.pib().extended_address();
    roles::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<EndDeviceProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::end_device::end_device_profile());
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::END_DEVICE_MODEL)
        .application_version(product::firmware::application_version(FIRMWARE_VERSION))
        .date_code(product::DATE_CODE)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let security_store = SECURITY.init(roles::open_security_store());
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    roles::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::END_DEVICE),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = AlwaysOnEndDeviceApp::new(
        node,
        &product::end_device::END_DEVICE_POLICY,
        roles::parts(),
    )
    .unwrap_or_else(|error| {
        log::error!("[ESP32-C6] invalid composition: {:?}", error);
        halt()
    });
    APP.init(app)
}
