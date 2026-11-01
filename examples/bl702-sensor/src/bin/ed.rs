//! XT-ZB1 always-on End Device (`--features ed --bin bl702-ed`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! over the hardware-tested BL702 polling radio (the core sends the
//! Imm-Ack; an end device never sets Frame Pending) under
//! [`router_app::AlwaysOnEndDeviceApp`]. Basic + Identify on endpoint 1
//! with the Simple Sensor device ID. The sensor SED is `bl702-sensor`.
//!
//! Identity, policy and the security journal (role tag
//! `product::role::END_DEVICE`) are owned by `products/bl702-xt-zb1`;
//! physical resources by `boards/bl702-xt-zb1`. No Zigbee OTA client.
//!
//! Validation: builds only; not hardware-validated (the radio driver and
//! the sensor SED are).

#![no_std]
#![no_main]

#[path = "../hal.rs"]
#[allow(dead_code)]
mod hal;
#[path = "../roles.rs"]
mod roles;
#[path = "../time_driver.rs"]
#[allow(dead_code)]
mod time_driver;

use bl702_hal::flash::XipFlash;
use bl702_xt_zb1_product as product;
use embassy_executor::Executor;
use panic_halt as _;
use product::end_device::EndDeviceProfile;
use product::storage::{SecurityStore, SharedFlash};
use roles::{CommissioningTrigger, LogDiagnostics, LogStatus, ResetSupervisor};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
use zigbee_mac::SoftMacCore;
use zigbee_mac::bl702::radio_phy::Bl702SoftMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile,
};
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Bl702SoftMac;
type Store = SecurityStore<SharedFlash<'static, XipFlash>>;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    Store,
    EndDeviceProfile,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[riscv_rt::entry]
fn main() -> ! {
    let resources = roles::boot(product::END_DEVICE_MODEL);
    let mut executor = Executor::new();
    // SAFETY: `main` never returns, so the executor lives forever.
    let executor: &'static mut Executor = unsafe { core::mem::transmute(&mut executor) };
    executor.run(move |spawner| spawner.must_spawn(run(resources)))
}

#[embassy_executor::task]
async fn run(resources: hal::ApplicationResources) {
    let started = roles::start(resources);
    let mut trigger = CommissioningTrigger::default();
    let app = compose(started);
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
fn compose(started: roles::Started) -> &'static mut App {
    let roles::Started {
        ieee,
        radio,
        pib,
        flash,
    } = started;
    let mac: Mac = roles::mac(SoftMacCore::new(radio, pib));

    static SECURITY: StaticCell<Store> = StaticCell::new();
    static PROFILE: StaticCell<EndDeviceProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::end_device::end_device_profile());
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::END_DEVICE_MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(product::CHANNEL_MASK)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let security_store = SECURITY.init(product::storage::security_store(SharedFlash::new(flash)));
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard(device.reset_security_state_if_identity_changed(security_store, ieee));
    roles::guard(security_store.select_role_tag(product::role::END_DEVICE));

    let node = ZigbeeNode::new(device, security_store, profile);
    let Ok(app) = AlwaysOnEndDeviceApp::new(
        node,
        &product::end_device::END_DEVICE_POLICY,
        roles::parts(),
    ) else {
        hal::halt()
    };
    APP.init(app)
}
