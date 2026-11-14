//! Composition and run loop of the STM32WB routers.
//!
//! [`compose`] builds the MAC, the device, the journals and the
//! [`ParentRouterApp`] into the composition's [`RouterCells`];
//! [`perform`] carries out the shared button [`Action`]s; [`run`] is the
//! whole loop of a board whose only inputs are buttons (Nucleo-WB55, USB
//! dongle). Boards with more peripherals (STM32WB5MM-DK) keep their own loop
//! around the same pieces.

use embassy_time::{Duration, Instant, Timer};
use router_app::{
    DiagnosticEvent, Diagnostics, NoObserver, NodeArchetype, ParentRouterApp, PersistentApsTables,
    PersistentChildren, RouterStatus, StatusSink, Supervisor,
};
use static_cell::StaticCell;
use stm32wb_hal::embassy_stm32::gpio::Input;
use stm32wb_hal::mac::WbTransport;
use stm32wb_router_product::{
    actions::{self, Action, ButtonMap},
    button::ButtonGestures,
    policy::ROUTER_POLICY,
    storage::{ApsTableStore, ChildStore, SecurityStore, SettingsStore},
};
use zigbee_mac::stm32wb::Stm32wbMac;
use zigbee_runtime::builder::DeviceBuilder;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::{ZigbeeDevice, node::ZigbeeNode, power::PowerMode, role::Router};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

use crate::platform::{self, LogDiagnostics, LogStatus, Parts, ResetSupervisor, halt};

pub type Mac = Stm32wbMac<WbTransport>;
pub type App<P> = ParentRouterApp<
    'static,
    Mac,
    SecurityStore,
    P,
    ChildStore,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
    NoObserver,
    PersistentApsTables<ApsTableStore>,
>;

pub const ARCHETYPE: NodeArchetype = NodeArchetype::ParentRouter;
/// Period of the neighbour/child/coprocessor summary on the log.
pub const SUMMARY_PERIOD: Duration = Duration::from_secs(60);

/// Basic-cluster identity of a product.
pub struct Identity {
    pub manufacturer: &'static str,
    pub model: &'static str,
    pub date_code: &'static str,
    pub sw_build: &'static str,
    /// Security-journal role tag ([`stm32wb_router_product::role`]).
    pub role: u16,
}

/// Static storage of one router composition; declare it as a `static` in
/// the composition root.
pub struct RouterCells<P: ApplicationProfile + 'static> {
    security: StaticCell<SecurityStore>,
    profile: StaticCell<P>,
    device: StaticCell<ZigbeeDevice<Mac, Router>>,
    app: StaticCell<App<P>>,
    #[cfg(feature = "green-power")]
    gp_proxy_table: StaticCell<stm32wb_router_product::storage::GpProxyTableStore>,
}

impl<P: ApplicationProfile + 'static> RouterCells<P> {
    pub const fn new() -> Self {
        Self {
            security: StaticCell::new(),
            profile: StaticCell::new(),
            device: StaticCell::new(),
            app: StaticCell::new(),
            #[cfg(feature = "green-power")]
            gp_proxy_table: StaticCell::new(),
        }
    }
}

impl<P: ApplicationProfile + 'static> Default for RouterCells<P> {
    fn default() -> Self {
        Self::new()
    }
}

/// Build MAC, device, stores and app into `cells`, and hand back the
/// settings journal. Halts (with the reason on the log) when the EUI-64 is
/// invalid, the journals are unavailable or a guard fails. Not inlined, so
/// the builder temporaries are released before the run loop.
#[inline(never)]
pub fn compose<P: ApplicationProfile + 'static>(
    cells: &'static RouterCells<P>,
    transport: WbTransport,
    identity: &Identity,
    profile: impl FnOnce() -> P,
    declare: impl FnOnce(DeviceBuilder<Mac>, &P) -> DeviceBuilder<Mac>,
    parts: Parts,
) -> (&'static mut App<P>, SettingsStore) {
    let ieee = stm32wb_hal::identity::eui64();
    platform::check_ieee(ieee);
    let mac = Stm32wbMac::new(transport, ieee);

    let stores = stm32wb_router_product::storage::open().unwrap_or_else(|error| {
        log::error!("[WB] journals unavailable: {error:?}");
        halt()
    });

    let profile = cells.profile.init_with(profile);
    let builder = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(identity.manufacturer)
        .model(identity.model)
        .date_code(identity.date_code)
        .sw_build(identity.sw_build)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(ChannelMask::ALL_2_4GHZ);
    let device = declare(builder, profile).build_router_into(cells.device.uninit());

    let security_store = cells.security.init(stores.security);
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    platform::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    platform::guard_outcome("role", security_store.select_role_tag(identity.role));

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = ParentRouterApp::new_with_aps_tables(
        node,
        PersistentChildren::new(stores.children),
        PersistentApsTables::new(stores.aps_tables),
        &ROUTER_POLICY,
        parts,
    )
    .unwrap_or_else(|error| platform::invalid_composition(error));
    #[cfg(feature = "green-power")]
    let app = {
        let mut app = app;
        app.set_green_power_store(cells.gp_proxy_table.init(stores.gp_proxy_table));
        app
    };
    (cells.app.init(app), stores.settings)
}

/// Carry out a shared button action; `button` names the input in the log.
/// `before_reset` runs before a factory reset commits (product state that
/// must not survive into the next commissioning).
pub async fn perform<P: ApplicationProfile>(
    app: &mut App<P>,
    action: Action,
    button: &str,
    before_reset: impl FnOnce(),
) {
    match action {
        Action::FactoryReset => {
            before_reset();
            factory_reset(app, button).await
        }
        Action::CommissionOrPermitJoin => {
            if app.request_commissioning() {
                log::info!("[WB] {button}: commissioning requested");
            } else if app.node().device().is_joined() {
                permit_joining(app, actions::PERMIT_JOIN_SECS).await;
            } else {
                log::info!("[WB] {button} ignored: reset pending");
            }
        }
        Action::ClosePermitJoin => permit_joining(app, 0).await,
        Action::Identify => {
            let device = app.node_mut().device_mut();
            if device.start_identify(stm32wb_router_product::ENDPOINT, actions::IDENTIFY_SECS) {
                log::info!("[WB] {button}: identify {} s", actions::IDENTIFY_SECS);
            } else {
                log::warn!("[WB] {button}: identify unavailable");
            }
        }
        Action::Restart => {
            log::info!("[WB] {button}: restart");
            // Let the log drain before the reset.
            Timer::after_millis(200).await;
            app.parts_mut().supervisor.reset()
        }
    }
}

/// Open (`secs` > 0) or close joining through this router (local NWK only;
/// no network-wide Mgmt_Permit_Joining broadcast).
pub async fn permit_joining<P: ApplicationProfile>(app: &mut App<P>, secs: u8) {
    match app.node_mut().device_mut().permit_joining(secs).await {
        Ok(()) => log::info!("[WB] permit join {secs} s"),
        Err(error) => log::warn!("[WB] permit join {secs} s failed: {error:?}"),
    }
}

/// Commit a journal-aware factory reset and restart factory-new.
pub async fn factory_reset<P: ApplicationProfile>(app: &mut App<P>, button: &str) -> ! {
    app.parts_mut().status.set(RouterStatus::Resetting {
        archetype: ARCHETYPE,
    });
    log::info!("[WB] {button} hold: committing durable factory reset");
    match app.urgent_factory_reset_and_recommission().await {
        Ok(()) => {
            let parts = app.parts_mut();
            parts.diagnostics.record(DiagnosticEvent::FactoryReset);
            parts.supervisor.reset()
        }
        Err(error) => platform::fatal(app.parts_mut(), ARCHETYPE, error),
    }
}

/// Reset when the IPCC mailbox stalled, after logging the notification CPU2
/// left pending (its opcode and leading octets identify an ABI or
/// sequencing fault after the reset).
pub async fn reset_if_mailbox_faulted<P: ApplicationProfile>(app: &mut App<P>) {
    if !mac(app).transport().faulted() {
        return;
    }
    log_mailbox_fault();
    // Let the log drain the fault line before the reset.
    Timer::after_millis(200).await;
    app.parts_mut().supervisor.reset();
}

#[inline(never)]
fn log_mailbox_fault() {
    let mut raw = [0u8; 16];
    let len = stm32wb_hal::mac::pending_notification(&mut raw);
    log::error!(
        "[WB] IPCC mailbox stalled; pending notification len {} {:02X?}",
        len,
        &raw[..len.min(raw.len())]
    );
}

pub fn mac<P: ApplicationProfile>(app: &App<P>) -> &Mac {
    app.node().device().bdb().zdo().nwk().mac()
}

/// Whether this router is open for joining.
pub fn permit_join_open<P: ApplicationProfile>(app: &App<P>) -> bool {
    app.node().device().bdb().zdo().nwk().nib().permit_joining
}

/// Neighbour/child/route tables and coprocessor counters on the log.
#[inline(never)]
pub fn log_summary<P: ApplicationProfile>(app: &App<P>) {
    let nwk = app.node().device().bdb().zdo().nwk();
    let neighbors = nwk.neighbor_table();
    log::info!(
        "[WB] neighbours {} children {} routes {} logdrop {}",
        neighbors.len(),
        neighbors.children().count(),
        nwk.routing_table().len(),
        crate::log::dropped_lines(),
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

/// One board button: its pin, gesture classifier and product action map.
pub struct Button {
    pin: Input<'static>,
    active_low: bool,
    gestures: ButtonGestures,
    map: &'static ButtonMap,
}

impl Button {
    pub const fn new(pin: Input<'static>, active_low: bool, map: &'static ButtonMap) -> Self {
        Self {
            pin,
            active_low,
            gestures: ButtonGestures::new(),
            map,
        }
    }

    fn sample(&mut self, now_ms: u64) -> Option<Action> {
        let pressed = self.pin.is_low() == self.active_low;
        let gesture = self.gestures.sample(pressed, now_ms)?;
        self.map.action(gesture)
    }
}

/// Initialize `app` and run it with `buttons` as the only inputs: bounded
/// receive slices, button actions between them, recommissioning after a
/// Leave, the permit-join state for the indicator and a periodic summary.
pub async fn run<P: ApplicationProfile>(app: &mut App<P>, buttons: &mut [Button]) -> ! {
    if let Err(error) = app.initialize().await {
        platform::fatal(app.parts_mut(), ARCHETYPE, error);
    }
    let mut awaiting_since: Option<Instant> = None;
    let mut next_summary = Instant::now() + SUMMARY_PERIOD;
    let mut step_errors = platform::StepErrors::new();
    loop {
        let now_ms = Instant::now().as_millis();
        for button in buttons.iter_mut() {
            if let Some(action) = button.sample(now_ms) {
                perform(app, action, button.map.name, || {}).await;
            }
        }
        if app.awaiting_commissioning_request() {
            let since = *awaiting_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() >= stm32wb_router_product::RECOMMISSION_AFTER_LEAVE_MS {
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
        reset_if_mailbox_faulted(app).await;
        crate::leds::publish_permit_join(permit_join_open(app));
        if Instant::now() >= next_summary {
            next_summary += SUMMARY_PERIOD;
            log_summary(app);
        }
    }
}
