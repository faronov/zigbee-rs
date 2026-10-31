//! Home Assistant Connect ZBT-1 always-on Zigbee End Device.
//!
//! The radio is the pure-Rust Series 2 driver (`efr32s2-hal`) on the shared
//! software MAC (`Efr32s2RadioPhy` + `SoftMacCore`). That MAC has no parent
//! role, so the lifecycle is [`router_app::AlwaysOnEndDeviceApp`]: mains
//! powered, `macRxOnWhenIdle = true`, never routing or admitting children.
//!
//! Product identity, profile, policy, TX power and the security journal are
//! owned by `products/ha-connect-zbt1`; physical resources by
//! `boards/ha-connect-zbt1`. This file owns startup, the identity guard and
//! composition. Logs go to SEGGER RTT.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod rtt_log;

use embassy_executor::Spawner;
use embassy_time::Instant;
use ha_connect_zbt1::resources::BoardResources;
use ha_connect_zbt1_product as product;
use router_app::{
    AlwaysOnEndDeviceApp, DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError, RouterParts,
    RouterStatus, StackEventSummary, StatusSink, Supervisor,
};
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

type Mac = Efr32s2SoftMac<{ product::TX_POWER_DBM }>;
type Parts = RouterParts<LogStatus, ResetSupervisor, LogDiagnostics>;
type App = AlwaysOnEndDeviceApp<
    'static,
    Mac,
    product::storage::SecurityStore,
    product::profile::AlwaysOnEndDeviceProfile,
    LogStatus,
    ResetSupervisor,
    LogDiagnostics,
>;

const ARCHETYPE: NodeArchetype = NodeArchetype::AlwaysOnEndDevice;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_log::init();
    let BoardResources {
        clocks,
        host_uart: _,
        radio,
        rtcc,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());

    if let Err(error) = ha_connect_zbt1::init_clocks(clocks) {
        log::error!("[ZBT1] HFXO start failed: {error:?}");
        halt()
    }
    let rtcc = rtcc.start().unwrap_or_else(|error| {
        log::error!("[ZBT1] RTCC start failed: {error:?}");
        halt()
    });
    efr32s2_hal::time_driver::init(rtcc);

    log::info!(
        "[ZBT1] {} on {} ({}), CTUNE {}, TX {} dBm",
        product::MODEL,
        ha_connect_zbt1::BOARD_NAME,
        ha_connect_zbt1::MCU_PART,
        ha_connect_zbt1::hfxo_ctune(),
        product::TX_POWER_DBM
    );
    // The product layout puts persistence at the top of 1 MiB; refuse to
    // run on a part whose DEVINFO reports less.
    let capacity = efr32s2_hal::flash::device_flash_capacity();
    if capacity != ha_connect_zbt1::INTERNAL_FLASH_CAPACITY {
        log::error!("[ZBT1] DEVINFO flash size {capacity} B, product needs 1 MiB");
        halt()
    }

    let app = compose(radio, flash);
    if let Err(error) = app.initialize().await {
        fatal(app.parts_mut(), error);
    }

    let mut awaiting_since: Option<Instant> = None;
    loop {
        // No button: re-arm commissioning automatically after a Leave.
        if app.awaiting_commissioning_request() {
            let since = *awaiting_since.get_or_insert_with(Instant::now);
            if since.elapsed().as_millis() >= product::policy::RECOMMISSION_AFTER_LEAVE_MS {
                app.request_commissioning();
                awaiting_since = None;
            }
        } else {
            awaiting_since = None;
        }

        // Bounded by the product's 20 ms receive slice (or an earlier
        // runtime deadline); the receiver stays on between slices.
        if let Err(error) = app.step().await {
            fatal(app.parts_mut(), error);
        }
    }
}

/// Build MAC, device, store and app into static storage. Not inlined, so the
/// large builder temporaries are released before the run loop.
#[inline(never)]
fn compose(
    radio: efr32s2_hal::radio::RadioToken,
    flash: efr32s2_hal::flash::Efr32s2Flash,
) -> &'static mut App {
    let mac: Mac = Efr32s2RadioPhy::new(radio)
        .map_err(|error| log::error!("[ZBT1] radio init failed: {error:?}"))
        .and_then(|phy| {
            phy.into_soft_mac()
                .map_err(|error| log::error!("[ZBT1] MAC init failed: {error:?}"))
        })
        .unwrap_or_else(|()| halt());
    let ieee = mac.pib().extended_address();
    if ieee == [0; 8] || ieee == [0xFF; 8] {
        log::error!("[ZBT1] no valid DEVINFO EUI-64");
        halt()
    }
    log::info!("[ZBT1] EUI-64 {:02X?}", ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<product::profile::AlwaysOnEndDeviceProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::profile::always_on_end_device_profile());
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
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

    let (security_store, migration) =
        product::storage::security_store(flash).unwrap_or_else(|error| {
            log::error!("[ZBT1] security journal open failed: {error:?}");
            halt()
        });
    let security_store = SECURITY.init(security_store);
    match migration {
        product::storage::MigrationDisposition::ExistingJournal => {
            log::info!("[ZBT1] valid security journal preserved")
        }
        product::storage::MigrationDisposition::FreshReset => {
            log::warn!("[ZBT1] foreign/blank persistence erased; fresh join required")
        }
    }

    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64.
    match device.reset_security_state_if_identity_changed(security_store, ieee) {
        Ok(true) => log::warn!("[ZBT1] cleared journal after IEEE identity change"),
        Ok(false) => {}
        Err(error) => {
            log::error!("[ZBT1] identity guard failed: {error:?}");
            halt()
        }
    }

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = AlwaysOnEndDeviceApp::new(
        node,
        &product::policy::ALWAYS_ON_END_DEVICE_POLICY,
        RouterParts::new(LogStatus::default(), ResetSupervisor, LogDiagnostics),
    )
    .unwrap_or_else(|error| {
        log::error!("[ZBT1] invalid composition: {error:?}");
        halt()
    });
    APP.init(app)
}

/// No LED is wired to the EFR32: report lifecycle changes on RTT.
#[derive(Default)]
struct LogStatus {
    last: Option<RouterStatus>,
}

impl StatusSink for LogStatus {
    fn set(&mut self, status: RouterStatus) {
        if self.last != Some(status) {
            self.last = Some(status);
            log::info!("[ZBT1] status {status:?}");
        }
    }
}

struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                log::info!("[ZBT1] commissioning attempt {attempt}")
            }
            DiagnosticEvent::StartFailed { error } => {
                log::warn!("[ZBT1] commissioning/resume failed: {error:?}")
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                log::info!("[ZBT1] retry {attempt} in {delay_ms} ms")
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => log::info!(
                "[ZBT1] online: addr=0x{short_address:04X} ch={channel} pan=0x{pan_id:04X}"
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => log::info!("[ZBT1] left"),
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                log::info!("[ZBT1] leave requested; durable reset to factory-new")
            }
            DiagnosticEvent::AwaitingCommissioningRequest => log::info!(
                "[ZBT1] factory-new after Leave; recommissioning in {} s",
                product::policy::RECOMMISSION_AFTER_LEAVE_MS / 1000
            ),
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                log::info!("[ZBT1] secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                log::info!("[ZBT1] secure rejoin ok: addr=0x{short_address:04X}")
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => {
                log::warn!("[ZBT1] secure rejoin failed ({failures}): {error:?}")
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                log::warn!("[ZBT1] secure rejoin limit reached at {failures}")
            }
            DiagnosticEvent::FactoryReset => log::info!("[ZBT1] security journal reset"),
            DiagnosticEvent::Fatal(error) => log::error!("[ZBT1] fatal: {error:?}"),
            _ => {}
        }
    }
}

/// No watchdog claim yet; fatal errors reset the MCU.
struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

fn fatal(parts: &mut Parts, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault {
        archetype: ARCHETYPE,
    });
    parts.supervisor.reset()
}

/// Stop with interrupts masked; the RTT log keeps the reason.
pub fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}
