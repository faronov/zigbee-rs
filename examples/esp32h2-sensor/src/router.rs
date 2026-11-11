//! ESP32-H2 Zigbee Router (`--no-default-features --features router`).
//!
//! A parent router: `EspRouterMac` (the shared `SoftMacCore` in the
//! `Parent` role; the radio's hardware auto-ACK answers Data Requests and
//! takes its Frame Pending bit from a bounded source table the core arms,
//! so the core's software pending set is empty) under
//! [`router_app::ParentRouterApp`]: beacons, association, indirect TX to
//! sleepy children, routing, durable child and APS tables. Status goes to
//! the console (and, with `status-led`, the WS2812 on GPIO8), BOOT (GPIO9)
//! re-arms commissioning after a Leave.
//!
//! No OTA client: this image is installed over the serial/JTAG link into
//! `ota_0`. The router never enters light sleep (`macRxOnWhenIdle`).

#![no_std]
#![no_main]

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

mod roles;

include!(concat!(env!("OUT_DIR"), "/firmware_version.rs"));

use embassy_time::{Duration, Instant};
use esp32_zigbee_devkit_product as product;
use esp_backtrace as _;
use product::executor::block_on;
use product::router::{ApsTableStore, ChildStore, RouterProfile};
use roles::{halt, CommissioningTrigger, LogDiagnostics, ResetSupervisor, Status};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
use zigbee_mac::esp::EspRouterMac;
use zigbee_runtime::{
    node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router, ZigbeeDevice,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = EspRouterMac<'static>;
type App = ParentRouterApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    RouterProfile,
    ChildStore,
    Status,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child and radio summary on the console.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

#[esp_hal::main]
fn main() -> ! {
    let started = roles::start(product::ROUTER_MODEL);
    let trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.radio, started.aes, started.status_pins);
    block_on(run(app, trigger))
}

async fn run(app: &'static mut App, mut trigger: CommissioningTrigger) -> ! {
    if let Err(error) = app.initialize().await {
        roles::fatal(app.parts_mut(), ARCHETYPE, error);
    }
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    loop {
        if trigger.poll(app.awaiting_commissioning_request()) {
            app.request_commissioning();
        }
        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline); the receiver stays on between slices.
        if let Err(error) = app.step().await {
            roles::fatal(app.parts_mut(), ARCHETYPE, error);
        }
        #[cfg(feature = "status-led")]
        {
            // Advance the LED blink/hold; the NIB tracks the timed
            // permit-join close, which raises no stack event.
            let permit_join = app.node().device().bdb().zdo().nwk().nib().permit_joining;
            app.parts_mut().status.refresh(permit_join);
        }
        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
    }
}

/// Build MAC, device, stores and app into static storage. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    radio: esp_hal::peripherals::IEEE802154<'static>,
    aes: esp_hal::peripherals::AES<'static>,
    status_pins: roles::StatusPins,
) -> &'static mut App {
    let Ok(mac) = roles::radio_phy(radio, aes).into_parent_mac() else {
        log::error!("[ESP32-H2] MAC init failed");
        halt()
    };
    let ieee = mac.pib().extended_address();
    roles::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<RouterProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::router::router_profile());
    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::ROUTER_MODEL)
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
        .build_router_into(DEVICE.uninit());

    let (child_store, aps_store) = product::router::router_stores();
    let security_store = SECURITY.init(roles::open_security_store());
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    roles::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    roles::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::ROUTER),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(child_store),
        PersistentApsTables::new(aps_store),
        &product::router::ROUTER_POLICY,
        roles::parts(status_pins),
    )
    .unwrap_or_else(|error| {
        log::error!("[ESP32-H2] invalid composition: {:?}", error);
        halt()
    });
    APP.init(app)
}

/// Neighbour/child table, Frame Pending and TX health on the console.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let radio = zigbee_mac::esp::parent_stats();
    log::info!(
        "[ESP32-H2] neighbours {} children {} routes {} | data-req {} fp-acks {} | tx {} tx-err {} deferred {} ack-tx {} no-ack {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        radio.data_requests,
        radio.frame_pending_acks,
        radio.raw_tx,
        radio.tx_errors,
        radio.tx_deferred,
        radio.acked_tx,
        radio.tx_no_ack
    );
    for neighbor in neighbors.iter() {
        log::info!(
            "[ESP32-H2]  nbr 0x{:04X} rel={:?} rx_on={} lqi={}",
            neighbor.network_address.0,
            neighbor.relationship,
            neighbor.rx_on_when_idle,
            neighbor.lqi
        );
    }
}
