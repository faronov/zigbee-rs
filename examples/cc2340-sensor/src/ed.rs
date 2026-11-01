//! LP-EM-CC2340R5 always-on End Device (`--no-default-features --features ed`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! over the CC2340 PHY (PBE hardware Imm-Ack, never with Frame Pending)
//! under [`router_app::AlwaysOnEndDeviceApp`]. Basic + Identify on endpoint
//! 1; LED1 shows the online state, BTN1 re-arms commissioning after a Leave.
//!
//! Identity, policy and the security journal are owned by
//! `products/cc2340-sensor`; physical resources by `boards/lp-em-cc2340r5`.
//! The image links the sensor's memory map unchanged (application
//! `0x0..0x7E000`, shared security journal at `0x7E000`).
//!
//! Same validation level as the sensor: builds only. The fallback build
//! (no `CC2340_SDK_DIR`) has no radio firmware and stops at radio start-up;
//! radio, MAC timing and commissioning are not hardware-validated.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;

use cc2340_sensor_product as product;
use embassy_executor::Spawner;
use product::end_device::EndDeviceProfile;
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor, halt};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
use zigbee_mac::cc2340::Cc2340SoftMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Cc2340SoftMac;
type SecurityStore = product::storage::HardwareSecurityStore;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    SecurityStore,
    EndDeviceProfile,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    halt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = roles::start(product::END_DEVICE_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.ieee, started.flash, started.led, started.reset);
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
    ieee: zigbee_types::IeeeAddress,
    flash: lp_em_cc2340r5::flash::InternalFlash,
    led: lp_em_cc2340r5::pins::Led1,
    reset: lp_em_cc2340r5::reset::SystemReset,
) -> &'static mut App {
    let mac: Mac = roles::mac(roles::radio(ieee).into_soft_mac());

    static SECURITY: StaticCell<SecurityStore> = StaticCell::new();
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
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let security_store = SECURITY.init(product::storage::security_store(flash));
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
        roles::parts(led, reset),
    )
    .unwrap_or_else(|error| {
        rtt_target::rprintln!("[CC2340] invalid composition: {:?}", error);
        halt()
    });
    APP.init(app)
}
