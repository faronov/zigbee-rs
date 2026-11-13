//! STM32WB5MM-DK Zigbee Router.
//!
//! The coprocessor MAC, the router composition, the shared button actions
//! and the log come from [`stm32wb_router_app`] (shared with the
//! Nucleo-WB55 and USB dongle routers); this composition root adds the DK
//! peripherals.
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

use embassy_executor::Spawner;
#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
use embassy_time::Duration;
use embassy_time::Instant;
use stm32wb_router_app::platform as wb_platform;
use stm32wb_router_app::router::{self, ARCHETYPE, Identity, RouterCells, SUMMARY_PERIOD};
use stm32wb5mm_dk_router_product::{
    self as product,
    actions::{self, Action, Input},
    button::ButtonGestures,
    router::RouterProfile,
};
#[cfg(feature = "sensors")]
use stm32wb5mm_dk_router_product::{motion::MotionHoldPersistence, storage::SettingsStore};

type App = router::App<RouterProfile>;

/// Durable Occupancy Sensing motion hold ([`product::motion`]).
#[cfg(feature = "sensors")]
type MotionHold = MotionHoldPersistence<SettingsStore>;

/// Period of publishing network state and applying sensor readings.
#[cfg(any(
    feature = "rgb-led",
    feature = "lcd",
    feature = "sensors",
    feature = "mic"
))]
const SHARE_PERIOD: Duration = Duration::from_millis(250);

static CELLS: RouterCells<RouterProfile> = RouterCells::new();

const IDENTITY: Identity = Identity {
    manufacturer: product::MANUFACTURER,
    model: product::ROUTER_MODEL,
    date_code: product::DATE_CODE,
    sw_build: product::SW_BUILD,
    role: product::role::ROUTER,
};

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
        .unwrap_or_else(|_| wb_platform::halt()),
    );
    #[cfg(feature = "sensors")]
    spawner.spawn(sensing::run(started.sensor_bus).unwrap_or_else(|_| wb_platform::halt()));
    #[cfg(feature = "mic")]
    spawner.spawn(sound::run(started.mic).unwrap_or_else(|_| wb_platform::halt()));
    let transport = stm32wb_router_app::mac_started(
        spawner,
        stm32wb_hal::mac::start(started.ipcc, platform::Irqs, started.rng).await,
    );

    let (app, settings) = router::compose(
        &CELLS,
        transport,
        &IDENTITY,
        product::router::router_profile,
        product::router::declare_endpoints,
        wb_platform::parts(shared::publish_status, actions::B1.name),
    );
    #[cfg(feature = "sensors")]
    let mut motion_hold = MotionHold::new(settings);
    #[cfg(not(feature = "sensors"))]
    let _ = settings;
    if let Err(error) = app.initialize().await {
        wb_platform::fatal(app.parts_mut(), ARCHETYPE, error);
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
    let mut step_errors = wb_platform::StepErrors::new();
    loop {
        let now_ms = Instant::now().as_millis();
        let inputs = [
            b1_gestures.sample(b1.is_low(), now_ms).map(Input::B1),
            b2_gestures.sample(b2.is_low(), now_ms).map(Input::B2),
        ];
        for action in inputs.into_iter().flatten().filter_map(actions::action) {
            perform(
                app,
                action,
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
        router::reset_if_mailbox_faulted(app).await;
        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            router::log_summary(app);
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
        Action::Router(action, button) => {
            router::perform(app, action, button, || {
                // A new commissioning starts from the default motion hold,
                // even in the same network.
                #[cfg(feature = "sensors")]
                if let Err(error) = hold.forget() {
                    log::warn!("[WB] motion hold not forgotten: {error:?}");
                }
            })
            .await
        }
        // Touch-key actions are handled by the display task.
        Action::NextPage | Action::FirstPage => {}
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

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    stm32wb_router_app::log::panic(info)
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    stm32wb_router_app::log::hard_fault(frame)
}
