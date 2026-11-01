//! EFR32MG1P TRÅDFRI always-on End Device (`--no-default-features --features ed`).
//!
//! RX-on-when-idle, non-routing: the shared `SoftMacCore` end-device MAC
//! over the EFR32MG1 PHY (FRC-IRQ Imm-Ack, never with Frame Pending) under
//! [`router_app::AlwaysOnEndDeviceApp`]. Basic + Identify on endpoint 1;
//! the PA0 LED shows the online state, the PB13 button re-arms
//! commissioning after a Leave.
//!
//! Identity, policy and the security journal are owned by
//! `products/efr32mg1-tradfri`; physical resources by
//! `boards/efr32mg1-tradfri`. The image links the sensor's memory map
//! unchanged (application `0x4000..0x37000`, shared security journal at
//! `0x37000`); it carries no OTA path.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;
mod time_driver;
mod vectors;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use efr32mg1_tradfri_product as product;
use embassy_executor::Spawner;
use product::end_device::EndDeviceProfile;
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor, halt};
use router_app::{AlwaysOnEndDeviceApp, NodeArchetype};
use static_cell::StaticCell;
#[allow(unused_imports)]
use vectors::__INTERRUPTS;
use zigbee_mac::efr32::{Efr32Mac, Efr32RadioPhy};
use zigbee_nwk::DeviceType;
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Efr32Mac;
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

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    halt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = roles::start(product::END_DEVICE_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.crypto, started.flash, started.led);
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
    crypto: efr32mg1_hal::peripherals::Crypto,
    flash: efr32mg1_hal::flash::Efr32mg1Flash,
    led: efr32mg1_tradfri::Led,
) -> &'static mut App {
    let mut phy = Efr32RadioPhy::new();
    phy.install_aes_engine(crypto).unwrap_or_else(|_| halt());
    let mac: Mac = phy.into_soft_mac().unwrap_or_else(|_| halt());
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
        .application_version(FIRMWARE_APPLICATION_VERSION)
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

    let (security_partition, _application_nv) = product::storage::split_flash(flash);
    let security_store = SECURITY.init(product::storage::security_store(security_partition));
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
        rtt_target::rprintln!("[EFR32] invalid composition: {:?}", error);
        halt()
    });
    APP.init(app)
}
