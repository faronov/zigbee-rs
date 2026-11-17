//! ESP32-H2 Zigbee Router (`--no-default-features --features router`).
//!
//! A parent router: `EspRouterMac` (the shared `SoftMacCore` in the
//! `Parent` role; the radio's hardware auto-ACK answers Data Requests and
//! takes its Frame Pending bit from a bounded source table the core arms,
//! so the core's software pending set is empty) under
//! [`router_app::ParentRouterApp`]: beacons, association, indirect TX to
//! sleepy children, routing, durable child and APS tables, and a GP Proxy
//! Basic with a durable Proxy Table (`router-core` builds without it). Status goes
//! to the console and the WS2812 on GPIO8; BOOT (GPIO9) re-arms
//! commissioning after a Leave.
//!
//! With `combo-light` the endpoint is an HA On/Off Light shown on the WS2812
//! and a GP Combo sink delivers paired GPD switch commands to it; BOOT
//! pressed while joined opens sink commissioning mode (see [`combo`]).
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
    #[cfg(feature = "combo-light")]
    let mut combo = combo::Combo::new();
    loop {
        if trigger.poll(app.awaiting_commissioning_request()) {
            app.request_commissioning();
        }
        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline); the receiver stays on between slices.
        if let Err(error) = app.step().await {
            roles::fatal(app.parts_mut(), ARCHETYPE, error);
        }
        #[cfg(feature = "combo-light")]
        combo.service(app, &mut trigger);
        #[cfg(feature = "status-led")]
        {
            // Advance the LED blink/hold; the NIB tracks the timed
            // permit-join close, which raises no stack event.
            let permit_join = app.node().device().bdb().zdo().nwk().nib().permit_joining;
            // Combo: an open GP sink commissioning window blinks like an
            // open network.
            #[cfg(feature = "combo-light")]
            let permit_join = permit_join || app.node().device().gp_sink().in_commissioning_mode();
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
    #[cfg(feature = "green-power")]
    let app = {
        static GP_STORE: StaticCell<product::router::GpProxyTableStore> = StaticCell::new();
        let mut app = app;
        app.set_green_power_store(GP_STORE.init(product::router::gp_proxy_table_store()));
        app
    };
    // Translated GPD commands go to the On/Off Light endpoint.
    #[cfg(feature = "combo-light")]
    let app = {
        let mut app = app;
        app.node_mut()
            .device_mut()
            .gp_sink_set_endpoint(Some(product::router::COMBO_SINK_ENDPOINT));
        app
    };
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
    #[cfg(feature = "green-power")]
    {
        let proxy = app.node().device().gp_proxy();
        let stats = proxy.forward_stats();
        log::info!(
            "[ESP32-H2] GP proxy entries {} | forwards sent {} cancelled {} (other proxy first)",
            proxy.entries().len(),
            stats.sent,
            stats.cancelled
        );
    }
    #[cfg(feature = "combo-light")]
    log::info!(
        "[ESP32-H2] GP sink entries {} | light {}",
        app.node().device().gp_sink().entries().len(),
        app.node().profile().component().is_on()
    );
}

/// GP Combo light (`combo-light`): BOOT opens sink commissioning, sink
/// events go to the console, the On/Off Light's timers advance and its
/// state is shown on the WS2812.
#[cfg(feature = "combo-light")]
mod combo {
    use super::{App, CommissioningTrigger};
    use embassy_time::{Duration, Instant};

    /// OnTime/OffWaitTime tick (ZCL: 1/10 s).
    const DECISECOND: Duration = Duration::from_millis(100);

    pub struct Combo {
        next_tick: Instant,
    }

    impl Combo {
        pub fn new() -> Self {
            Self {
                next_tick: Instant::now() + DECISECOND,
            }
        }

        /// Call once per run-loop step.
        #[inline(never)]
        pub fn service(&mut self, app: &mut App, trigger: &mut CommissioningTrigger) {
            let device = app.node_mut().device_mut();
            if trigger.take_press() {
                if !device.is_joined() {
                    log::info!("[ESP32-H2] BOOT: not joined, no GP sink commissioning");
                } else if device.gp_sink().in_commissioning_mode() {
                    log::info!("[ESP32-H2] BOOT: GP sink commissioning already open");
                } else if let Err(error) = device.gp_sink_enter_commissioning(true) {
                    log::warn!("[ESP32-H2] GP sink commissioning refused: {:?}", error);
                }
            }
            while let Some(event) = device.gp_sink_take_event() {
                log::info!("[ESP32-H2] GP sink {:?}", event);
            }
            let now = Instant::now();
            if now >= self.next_tick {
                let elapsed = (now - self.next_tick).as_ticks() / DECISECOND.as_ticks() + 1;
                self.next_tick += DECISECOND * elapsed as u32;
                app.node_mut()
                    .profile_mut()
                    .component_mut()
                    .tick_by(elapsed as u32);
            }
            let on = app.node().profile().component().is_on();
            app.parts_mut().status.set_light(on);
        }
    }
}
