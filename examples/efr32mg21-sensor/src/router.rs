//! BRD4181A EFR32MG21 Zigbee Router (`--no-default-features --features router`).
//!
//! A true parent router: `Efr32s2RouterMac` (the shared `SoftMacCore` in the
//! `Parent` role over the Series 2 PHY; the HAL `isr-ack` interrupt sends
//! each Imm-Ack in time, with Frame Pending from the HAL source-match table
//! the core arms for queued indirect frames) under
//! [`router_app::ParentRouterApp`]: beacons, association, indirect TX to
//! sleepy children, routing, durable child and APS tables. LED0 shows the
//! online state, BTN0 re-arms commissioning after a Leave.
//!
//! RAM: 64 KiB SRAM; the image links the `constrained-memory` ZCL/ZDO
//! sizes with unmodified NWK router tables and `tools/verify-layout.py`
//! enforces the stack reserve. Not yet run on hardware (no BRD4181A on
//! the bench); the same router composition is hardware-tested on the
//! ZBT-1 (EFR32MG21A020).

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;
mod vcom_log;

// `vcom_log`'s panic/HardFault handlers stop through `crate::platform::halt`.
use roles as platform;

use efr32mg21_sensor_product as product;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use product::router::{ApsTableStore, ChildStore, RouterProfile};
use roles::{CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor, halt};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
use zigbee_mac::efr32s2::{EFR32S2_ROUTER_PENDING_RX, Efr32s2RadioPhy, Efr32s2RouterMac};
use zigbee_runtime::{
    ZigbeeDevice, node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type Mac = Efr32s2RouterMac;
type App = ParentRouterApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    RouterProfile,
    ChildStore,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child summary on VCOM.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = roles::start(product::ROUTER_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.radio, started.flash, started.led);
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
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
    led: efr32mg21_devkit::Led0,
) -> &'static mut App {
    let mac: Mac = Efr32s2RadioPhy::new(radio)
        .map_err(|error| log::error!("[MG21] radio init failed: {error:?}"))
        .and_then(|phy| {
            phy.into_parent_mac()
                .map_err(|error| log::error!("[MG21] MAC init failed: {error:?}"))
        })
        .unwrap_or_else(|()| halt());
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
        .build_router_into(DEVICE.uninit());

    let (flash, child_store, aps_store) = product::router::router_stores(flash);
    let security_store = SECURITY.init(roles::open_security_store(flash));
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
        roles::parts(led),
    )
    .unwrap_or_else(|error| {
        log::error!("[MG21] invalid composition: {error:?}");
        halt()
    });
    APP.init(app)
}

/// Neighbour/child table and parent-MAC health on VCOM.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let phy = nwk.mac().phy();
    let radio = phy.radio_stats();
    let ack = phy.ack_stats();
    log::info!(
        "[MG21] neighbours {} children {} routes {} rx {} rx-ovf {} tx {} tx-fail {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        radio.rx_frames,
        radio.rx_overflows,
        radio.tx_frames,
        radio.tx_failures
    );
    log::info!(
        "[MG21] isr-ack acks {} fp {} late {} filtered {} ovf {} queue-full {} resync {} ack-fail {} rx-restart-fail {} max-turnaround {}us",
        ack.acks,
        ack.fp_acks,
        ack.late,
        ack.filtered,
        ack.overflows,
        ack.queue_full,
        ack.resyncs,
        ack.ack_tx_fail,
        ack.rx_restart_fail,
        ack.max_turnaround_us
    );
    for child in neighbors.children() {
        log::info!(
            "[MG21]  child 0x{:04X} rx_on={} lqi={}",
            child.network_address.0,
            child.rx_on_when_idle,
            child.lqi
        );
    }
}

// The router PHY retains this many received frames between runtime calls.
const _: () = assert!(EFR32S2_ROUTER_PENDING_RX >= 4);
