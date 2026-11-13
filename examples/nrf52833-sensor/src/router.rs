//! nRF52833 DK Zigbee Router (`--no-default-features --features router`).
//!
//! A true parent router: [`NrfRouterMac`] (the shared `SoftMacCore` in the
//! `Parent<8, 8, 0>` role) under [`router_app::ParentRouterApp`]: beacons,
//! association, indirect TX to sleepy children, routing, durable child and
//! APS tables, and a Green Power Proxy Basic with a durable Proxy Table
//! (`--features router-core` builds without it). The interrupt-driven PHY sends every Imm-Ack from the RADIO
//! interrupt, timed by TIMER0/PPI to the 192 µs turnaround, and sets Frame
//! Pending from its source-match table, which the core fills through
//! `RadioPhy::set_ack_frame_pending` (PENDING = 0). LED1 shows the online
//! state, Button 1 re-arms commissioning after a Leave. Log: VCOM (UARTE0,
//! 115200 8N1), including a periodic PHY counter summary.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod roles;
mod vcom_log;

use embassy_executor::Spawner;
use embassy_nrf::nvmc::Nvmc;
use embassy_time::{Duration, Instant};
use nrf52833_sensor_product as product;
use product::router::{ApsTableStore, ChildStore, RouterProfile};
use roles::{halt, CommissioningTrigger, LedStatus, LogDiagnostics, ResetSupervisor};
use router_app::{
    NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables, PersistentChildren,
};
use static_cell::StaticCell;
use zigbee_mac::nrf::{NrfRouterMac, NRF_ROUTER_PENDING_RX};
use zigbee_runtime::{
    node::ZigbeeNode, power::PowerMode, profile::ApplicationProfile, role::Router, ZigbeeDevice,
};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = NrfRouterMac<'static, embassy_nrf::peripherals::RNG>;
type App = ParentRouterApp<
    'static,
    Mac,
    product::storage::SecurityStore<'static>,
    RouterProfile,
    ChildStore,
    LedStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child/PHY summary on VCOM.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let started = roles::start(spawner, product::ROUTER_MODEL);
    let mut trigger = CommissioningTrigger::new(started.button);
    let app = compose(started.phy, started.nvmc, started.led);
    if let Err(error) = app.initialize().await {
        roles::fatal(app.parts_mut(), ARCHETYPE, error);
    }

    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    let mut step_errors = roles::StepErrors::new();
    loop {
        if trigger.poll(app.awaiting_commissioning_request()) {
            app.request_commissioning();
        }
        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline); the receiver stays on between slices and the
        // RADIO interrupt keeps acknowledging and buffering frames.
        match app.step().await {
            Ok(_) => step_errors.ok(),
            Err(error) => step_errors.handle(app.parts_mut(), ARCHETYPE, error).await,
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
    phy: roles::Phy,
    nvmc: Nvmc<'static>,
    led: embassy_nrf::gpio::Output<'static>,
) -> &'static mut App {
    let mac: Mac = phy.into_parent_mac().unwrap_or_else(|error| {
        log::error!("[NRF] MAC init failed: {error:?}");
        halt()
    });
    let ieee = mac.pib().extended_address();
    roles::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore<'static>> = StaticCell::new();
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

    let (nvmc, child_store, aps_store) = product::router::router_stores(nvmc);
    #[cfg(feature = "green-power")]
    let (nvmc, gp_store) = product::router::gp_proxy_table_store(nvmc);
    let security_store = SECURITY.init(roles::open_security_store(nvmc));
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
        log::error!("[NRF] invalid composition: {error:?}");
        halt()
    });
    #[cfg(feature = "green-power")]
    let app = {
        static GP_STORE: StaticCell<product::router::GpProxyTableStore> = StaticCell::new();
        let mut app = app;
        app.set_green_power_store(GP_STORE.init(gp_store));
        app
    };
    APP.init(app)
}

/// Neighbour/child tables and the RADIO handler's ACK/TX counters on VCOM.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    let counters = nwk.mac().phy().counters();
    log::info!(
        "[NRF] neighbours {} children {} routes {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
    );
    log::info!(
        "[NRF] rx {} crc {} ovf {} ack {} fp {} late {} turn {}..{}us tx {} busy {} noack {} logdrop {}",
        counters.rx_frames,
        counters.crc_errors,
        counters.rx_overflow,
        counters.acks_sent,
        counters.acks_frame_pending,
        counters.acks_late,
        counters.ack_turnaround_min_us,
        counters.ack_turnaround_max_us,
        counters.tx_frames,
        counters.tx_busy,
        counters.tx_no_ack,
        vcom_log::dropped_lines(),
    );
    for neighbor in neighbors.iter() {
        log::info!(
            "[NRF]  nbr 0x{:04X} {:?} rx_on={} lqi={}",
            neighbor.network_address.0,
            neighbor.relationship,
            neighbor.rx_on_when_idle,
            neighbor.lqi
        );
    }
    #[cfg(feature = "green-power")]
    {
        let proxy = app.node().device().gp_proxy();
        let stats = proxy.forward_stats();
        log::info!(
            "[NRF] GP proxy entries {} | forwards sent {} cancelled {} (other proxy first)",
            proxy.entries().len(),
            stats.sent,
            stats.cancelled
        );
    }
}

// The router PHY retains this many received frames between runtime calls.
const _: () = assert!(NRF_ROUTER_PENDING_RX >= 4);
