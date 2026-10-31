//! BRD4183A EFR32MG22 always-on End Device (`--features ed`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! over the Series 2 PHY (hardware-style HAL Imm-Ack) under
//! [`router_app::AlwaysOnEndDeviceApp`]. Basic + Identify on endpoint 1;
//! LED0 shows the online state, BTN0 re-arms commissioning after a Leave.
//!
//! Identity, policy and the security journal are owned by
//! `products/efr32mg22-sensor`; physical resources by
//! `boards/efr32mg22-devkit`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;
mod vcom_log;

// `vcom_log`'s panic/HardFault handlers stop through `crate::platform::halt`.
use roles as platform;

use efr32mg22_sensor_product as product;
use embassy_executor::Spawner;
use product::end_device::EndDeviceProfile;
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor, halt};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
use zigbee_mac::efr32s2::{Efr32s2RadioPhy, Efr32s2SoftMac};
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type Mac = Efr32s2SoftMac;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    EndDeviceProfile,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = roles::start(product::END_DEVICE_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.radio, started.flash, started.led);
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
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
    led: efr32mg22_devkit::Led0,
) -> &'static mut App {
    let mac: Mac = Efr32s2RadioPhy::new(radio)
        .map_err(|error| log::error!("[MG22] radio init failed: {error:?}"))
        .and_then(|phy| {
            phy.into_soft_mac()
                .map_err(|error| log::error!("[MG22] MAC init failed: {error:?}"))
        })
        .unwrap_or_else(|()| halt());
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

    let security_store = SECURITY.init(roles::open_security_store(flash));
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
        roles::parts(led),
    )
    .unwrap_or_else(|error| {
        log::error!("[MG22] invalid composition: {error:?}");
        halt()
    });
    APP.init(app)
}
