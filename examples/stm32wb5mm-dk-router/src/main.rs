//! STM32WB5MM-DK Zigbee Router.
//!
//! CPU2 runs ST's IEEE 802.15.4 MAC firmware: it sends Imm-Acks, answers
//! data polls with Frame Pending, holds indirect frames for sleepy children
//! and transmits beacons. [`Stm32wbMac`] maps that MAC onto zigbee-rs's
//! parent MAC contract, and [`router_app::ParentRouterApp`] runs the Zigbee
//! router on top: routing, association, durable child and APS tables.
//!
//! Buttons ([`product::actions`]): B1 short press commissions, or opens
//! permit-join for 180 s once joined; a 3 s hold commits a journal-aware
//! factory reset. B2 short press identifies for 10 s; a hold closes
//! permit-join. Log: ST-LINK VCOM (USART1, 115200 8N1), with a periodic
//! table and coprocessor summary.
//!
//! Default features add the DK peripherals as separate tasks that never
//! hold the router: the RGB LED, the OLED pages and the touch key
//! ([`ui`]), the I2C sensors ([`sensing`]) and the microphone ([`sound`]).
//! Readings reach the ZCL clusters through [`shared`] in this loop.

#![no_std]
#![no_main]

mod platform;
#[cfg(feature = "sensors")]
mod sensing;
mod shared;
#[cfg(feature = "mic")]
mod sound;
#[cfg(any(feature = "rgb-led", feature = "lcd"))]
mod ui;
mod vcom_log;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant};
use platform::{LogDiagnostics, LogStatus, ResetSupervisor, halt};
use router_app::{
    DiagnosticEvent, Diagnostics, NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables,
    PersistentChildren, RouterStatus, StatusSink, Supervisor,
};
use static_cell::StaticCell;
use stm32wb_hal::embassy_stm32_wpan::sub::mm::MemoryManager;
use stm32wb_hal::mac::WbTransport;
use stm32wb5mm_dk_router_product::{
    self as product,
    actions::{self, Action, Input},
    button::ButtonGestures,
    router::RouterProfile,
    storage::{ApsTableStore, ChildStore, SecurityStore},
};
#[cfg(feature = "sensors")]
use stm32wb5mm_dk_router_product::{motion::MotionHoldPersistence, storage::SettingsStore};
use zigbee_mac::stm32wb::Stm32wbMac;
use zigbee_runtime::{ZigbeeDevice, node::ZigbeeNode, power::PowerMode, role::Router};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

type Mac = Stm32wbMac<WbTransport>;
type App = ParentRouterApp<
    'static,
    Mac,
    SecurityStore,
    RouterProfile,
    ChildStore,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

/// Durable Occupancy Sensing motion hold ([`product::motion`]).
#[cfg(feature = "sensors")]
type MotionHold = MotionHoldPersistence<SettingsStore>;

const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child/coprocessor summary on VCOM.
const SUMMARY_PERIOD: Duration = Duration::from_secs(60);
/// Period of publishing network state and applying sensor readings.
#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
const SHARE_PERIOD: Duration = Duration::from_millis(250);

/// Returns mailbox buffers to CPU2; must run for the life of the MAC.
#[embassy_executor::task]
async fn run_memory_manager(mut memory_manager: MemoryManager<'static>) {
    memory_manager.run_queue().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let started = platform::start(spawner, product::ROUTER_MODEL);
    #[cfg(any(feature = "rgb-led", feature = "lcd"))]
    spawner.spawn(
        ui::run(ui::Ui::new(
            started.spi1,
            #[cfg(feature = "touch")]
            started.touch,
        ))
        .unwrap_or_else(|_| halt()),
    );
    #[cfg(feature = "sensors")]
    spawner.spawn(sensing::run(started.sensor_bus).unwrap_or_else(|_| halt()));
    #[cfg(feature = "mic")]
    spawner.spawn(sound::run(started.mic).unwrap_or_else(|_| halt()));
    let (transport, memory_manager) =
        match stm32wb_hal::mac::start(started.ipcc, platform::Irqs, started.rng).await {
            Ok(parts) => parts,
            Err(error) => {
                log::error!("[WB] coprocessor MAC start failed: {error:?}");
                halt()
            }
        };
    spawner.spawn(run_memory_manager(memory_manager).unwrap_or_else(|_| halt()));
    log::info!("[WB] coprocessor MAC running");

    let (app, settings) = compose(transport);
    #[cfg(feature = "sensors")]
    let mut motion_hold = MotionHold::new(settings);
    #[cfg(not(feature = "sensors"))]
    let _ = settings;
    if let Err(error) = app.initialize().await {
        platform::fatal(app.parts_mut(), ARCHETYPE, error);
    }

    let (b1, b2) = (started.button_b1, started.button_b2);
    let (mut b1_gestures, mut b2_gestures) = (ButtonGestures::new(), ButtonGestures::new());
    let mut awaiting_since: Option<Instant> = None;
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    #[cfg(any(
        feature = "rgb-led",
        feature = "lcd",
        feature = "sensors",
        feature = "mic"
    ))]
    let mut next_share = Instant::now();
    let mut step_errors = platform::StepErrors::new();
    loop {
        let now_ms = Instant::now().as_millis();
        let inputs = [
            b1_gestures.sample(b1.is_low(), now_ms).map(Input::B1),
            b2_gestures.sample(b2.is_low(), now_ms).map(Input::B2),
        ];
        for input in inputs.into_iter().flatten() {
            perform(
                app,
                actions::action(input),
                #[cfg(feature = "sensors")]
                &mut motion_hold,
            )
            .await;
        }
        if app.awaiting_commissioning_request() {
            let since = *awaiting_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() >= product::RECOMMISSION_AFTER_LEAVE_MS {
                awaiting_since = None;
                app.request_commissioning();
            }
        } else {
            awaiting_since = None;
        }

        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline).
        match app.step().await {
            Ok(_) => step_errors.ok(),
            Err(error) => step_errors.handle(app.parts_mut(), ARCHETYPE, error).await,
        }
        if mac(app).transport().faulted() {
            // Opcode and leading octets of the notification CPU2 left
            // pending identify an ABI or sequencing fault after the reset.
            let mut raw = [0u8; 16];
            let len = stm32wb_hal::mac::pending_notification(&mut raw);
            log::error!(
                "[WB] IPCC mailbox stalled; pending notification len {} {:02X?}",
                len,
                &raw[..len.min(raw.len())]
            );
            // Let the VCP drain the fault line before the reset.
            embassy_time::Timer::after_millis(200).await;
            app.parts_mut().supervisor.reset();
        }
        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
        #[cfg(any(
            feature = "rgb-led",
            feature = "lcd",
            feature = "sensors",
            feature = "mic"
        ))]
        if Instant::now() >= next_share {
            next_share = Instant::now() + SHARE_PERIOD;
            share(
                app,
                #[cfg(feature = "sensors")]
                &mut motion_hold,
            );
        }
    }
}

/// Carry out a button action ([`product::actions`]).
async fn perform(app: &mut App, action: Action, #[cfg(feature = "sensors")] hold: &mut MotionHold) {
    match action {
        Action::FactoryReset => {
            // A new commissioning starts from the default motion hold, even
            // in the same network.
            #[cfg(feature = "sensors")]
            if let Err(error) = hold.forget() {
                log::warn!("[WB] motion hold not forgotten: {error:?}");
            }
            factory_reset(app).await
        }
        Action::CommissionOrPermitJoin => {
            if app.request_commissioning() {
                log::info!("[WB] B1: commissioning requested");
            } else if app.node().device().is_joined() {
                permit_joining(app, actions::PERMIT_JOIN_SECS).await;
            } else {
                log::info!("[WB] B1 ignored: reset pending");
            }
        }
        Action::ClosePermitJoin => permit_joining(app, 0).await,
        Action::Identify => {
            let device = app.node_mut().device_mut();
            if device.start_identify(product::ENDPOINT, actions::IDENTIFY_SECS) {
                log::info!("[WB] B2: identify {} s", actions::IDENTIFY_SECS);
            } else {
                log::warn!("[WB] B2: identify unavailable");
            }
        }
        // Touch-key actions are handled by the display task.
        Action::NextPage | Action::FirstPage => {}
    }
}

/// Open (`secs` > 0) or close joining through this router (local NWK only;
/// no network-wide Mgmt_Permit_Joining broadcast).
async fn permit_joining(app: &mut App, secs: u8) {
    match app.node_mut().device_mut().permit_joining(secs).await {
        Ok(()) => log::info!("[WB] permit join {secs} s"),
        Err(error) => log::warn!("[WB] permit join {secs} s failed: {error:?}"),
    }
}

/// Publish network state for the peripheral tasks and apply their sensor
/// readings to the ZCL profile (reported by the runtime as they change).
#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
fn share(app: &mut App, #[cfg(feature = "sensors")] hold: &mut MotionHold) {
    #[cfg(any(feature = "rgb-led", feature = "lcd"))]
    shared::NETWORK.set(network_snapshot(app));
    #[cfg(any(feature = "sensors", feature = "mic"))]
    app.node_mut().profile_mut().apply(&shared::SENSORS.get());
    #[cfg(feature = "sensors")]
    sync_motion_hold(app, hold);
}

/// Restore the motion hold written in this network, persist a written one,
/// and hand the current value to the sensing task.
#[cfg(feature = "sensors")]
fn sync_motion_hold(app: &mut App, hold: &mut MotionHold) {
    let network = app.node().device().joined_extended_pan_id();
    let occupancy = app.node_mut().profile_mut().occupancy_mut();
    match hold.sync(network, occupancy) {
        Ok(product::motion::HoldSync::Unchanged) => {}
        Ok(product::motion::HoldSync::Restored(secs)) => {
            log::info!("[WB] motion hold {secs} s restored")
        }
        Ok(product::motion::HoldSync::Stored(secs)) => {
            log::info!("[WB] motion hold {secs} s stored")
        }
        Err(error) => log::warn!("[WB] motion hold not persisted: {error:?}"),
    }
    shared::MOTION_HOLD_SECS.set(occupancy.hold_secs());
}

#[cfg(any(feature = "rgb-led", feature = "lcd"))]
fn network_snapshot(app: &App) -> product::screen::NetworkSnapshot {
    let device = app.node().device();
    let nwk = device.bdb().zdo().nwk();
    let nib = nwk.nib();
    let neighbors = nwk.neighbor_table();
    let stats = nwk.mac().stats();
    let count = |n: usize| u8::try_from(n).unwrap_or(u8::MAX);
    let parent = (device.is_joined() && nib.parent_address.0 < 0xFFF8).then(|| {
        let lqi = neighbors
            .iter()
            .find(|n| n.network_address == nib.parent_address)
            .map_or(0, |n| n.lqi);
        (nib.parent_address.0, lqi)
    });
    product::screen::NetworkSnapshot {
        state: product::screen::State::default(),
        short_address: nib.network_address.0,
        pan_id: nib.pan_id.0,
        channel: nib.logical_channel,
        extended_pan_id: nib.extended_pan_id,
        parent,
        neighbours: count(neighbors.len()),
        children: count(neighbors.children().count()),
        routes: count(nwk.routing_table().len()),
        permit_join_secs: nib.permit_joining.then_some(nib.permit_joining_duration),
        nwk_frame_counter: nib.outgoing_frame_counter,
        tx_ok: stats.tx_ok,
        tx_no_ack: stats.tx_no_ack,
        tx_cca: stats.tx_channel_access_failures,
        tx_other: stats.tx_other_failures,
        rx_dropped: stats.rx_dropped,
        indirect_delivered: stats.indirect_delivered,
        indirect_handed_off: stats.indirect_handed_off,
        uptime_secs: Instant::now().as_secs() as u32,
    }
}

fn mac(app: &App) -> &Mac {
    app.node().device().bdb().zdo().nwk().mac()
}

async fn factory_reset(app: &mut App) -> ! {
    app.parts_mut().status.set(RouterStatus::Resetting {
        archetype: ARCHETYPE,
    });
    log::info!("[WB] B1 hold: committing durable factory reset");
    match app.urgent_factory_reset_and_recommission().await {
        Ok(()) => {
            let parts = app.parts_mut();
            parts.diagnostics.record(DiagnosticEvent::FactoryReset);
            parts.supervisor.reset()
        }
        Err(error) => platform::fatal(app.parts_mut(), ARCHETYPE, error),
    }
}

/// Build MAC, device, stores and app into static storage, and hand back the
/// settings journal. Not inlined, so the builder temporaries are released
/// before the run loop.
#[inline(never)]
fn compose(transport: WbTransport) -> (&'static mut App, product::storage::SettingsStore) {
    let ieee = stm32wb_hal::identity::eui64();
    platform::check_ieee(ieee);
    let mac = Stm32wbMac::new(transport, ieee);

    let stores = product::storage::open().unwrap_or_else(|error| {
        log::error!("[WB] journals unavailable: {error:?}");
        halt()
    });

    static SECURITY: StaticCell<SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<RouterProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac, Router>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::router::router_profile());
    let builder = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::ROUTER_MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(ChannelMask::ALL_2_4GHZ);
    let device =
        product::router::declare_endpoints(builder, profile).build_router_into(DEVICE.uninit());

    let security_store = SECURITY.init(stores.security);
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    platform::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    platform::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::ROUTER),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(stores.children),
        PersistentApsTables::new(stores.aps_tables),
        &product::router::ROUTER_POLICY,
        platform::parts(),
    )
    .unwrap_or_else(|error| {
        log::error!("[WB] invalid composition: {error:?}");
        halt()
    });
    #[cfg(feature = "green-power")]
    let app = {
        static GP_STORE: StaticCell<product::storage::GpProxyTableStore> = StaticCell::new();
        let mut app = app;
        app.set_green_power_store(GP_STORE.init(stores.gp_proxy_table));
        app
    };
    (APP.init(app), stores.settings)
}

/// Neighbour/child/route tables and coprocessor counters on VCOM.
#[inline(never)]
fn log_summary(app: &App) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    log::info!(
        "[WB] neighbours {} children {} routes {} logdrop {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        vcom_log::dropped_lines(),
    );
    // The VCP logger truncates long lines; keep each under ~120 columns.
    let stats = nwk.mac().stats();
    log::info!(
        "[WB] tx ok {} noack {} cca {} other {} | rx drop {} cmd drop {} stray {}",
        stats.tx_ok,
        stats.tx_no_ack,
        stats.tx_channel_access_failures,
        stats.tx_other_failures,
        stats.rx_dropped,
        stats.commands_dropped,
        stats.stray_confirms,
    );
    log::info!(
        "[WB] indirect {}/{} exp {} fail {} | beacon {} sync-loss {} other {}",
        stats.indirect_delivered,
        stats.indirect_handed_off,
        stats.indirect_expired,
        stats.indirect_failed,
        stats.beacon_refreshes,
        stats.sync_loss,
        stats.other_events,
    );
    for neighbor in neighbors.iter() {
        log::info!(
            "[WB]  nbr 0x{:04X} {:?} rx_on={} lqi={}",
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
            "[WB] GP proxy entries {} | forwards sent {} cancelled {}",
            proxy.entries().len(),
            stats.sent,
            stats.cancelled
        );
    }
}
