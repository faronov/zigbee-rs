//! Home Assistant Connect ZBT-1 Sleepy End Device (`--features sed`).
//!
//! The shared `sensor-sed-app` lifecycle on the end-device software MAC
//! (`Efr32s2SoftMac`). The dongle has no sensor, button or LED, so the
//! node is identity-only (Basic + Identify); it exists to prove EM2 sleep,
//! parent polling and RTCC wake on the EFR32MG21.
//!
//! Sleep depths: fast polling stays in EM1 with the radio state untouched;
//! slow polling idles the radio, enters EM2 until RTCC CC1 and resumes the
//! radio (`SoftMacCore::wake` → `Radio::resume`) before any traffic. RTT is
//! readable while awake; build with `em2-debug` to keep SWD alive in EM2
//! (that also keeps the debug domain powered, so EM2 current is not
//! representative).

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod platform;
mod rtt_log;

use efr32s2_hal::{power, rtcc};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use ha_connect_zbt1_product as product;
use platform::halt;
use product::sed::SedProfile;
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, NoBattery, NoMeasurement, NoOta, NoStatus, SensorApp,
    SensorSedParts, SleepDepth, Supervisor, WaitRequest, WakeController, WakeReason,
};
use static_cell::StaticCell;
use zigbee_mac::efr32s2::{Efr32s2RadioPhy, Efr32s2SoftMac};
use zigbee_runtime::{ZigbeeDevice, node::ZigbeeNode, profile::ApplicationProfile};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::PowerSource;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type Mac = Efr32s2SoftMac<{ product::TX_POWER_DBM }>;
type Parts = SensorSedParts<
    Zbt1Wake,
    NoStatus,
    NoMeasurement,
    NoBattery,
    NoOta,
    sensor_sed_app::NoUserAction,
    ResetSupervisor,
    LogDiagnostics,
>;
type App = SensorApp<'static, Mac, product::storage::SecurityStore, SedProfile, Parts>;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let started = platform::start(product::SED_MODEL);
    // Representative EM2 current unless the lab feature asks for SWD/RTT
    // access while asleep.
    power::set_em2_debug(cfg!(feature = "em2-debug"));
    compose(started.radio, started.flash).run().await
}

/// Build MAC, device, store and app into static storage. Not inlined, so
/// the large builder temporaries are released before the run loop.
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
    platform::check_ieee(ieee);

    static SECURITY: StaticCell<product::storage::SecurityStore> = StaticCell::new();
    static PROFILE: StaticCell<SedProfile> = StaticCell::new();
    static DEVICE: StaticCell<ZigbeeDevice<Mac>> = StaticCell::new();
    static APP: StaticCell<App> = StaticCell::new();

    let profile = PROFILE.init(product::sed::sed_profile());
    let device = ZigbeeDevice::builder(mac)
        .power_mode(product::sed::SED_POLICY.power_mode())
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::SED_MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        // USB-powered, but a sleepy node: advertise it as battery powered
        // so the coordinator does not expect mains-device behaviour.
        .power_source(PowerSource::Battery)
        .channels(ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build_into(DEVICE.uninit());

    let security_store = SECURITY.init(platform::open_security_store(flash));
    // Before ZigbeeNode construction: persisted keys/counters may never be
    // resumed under a different EUI-64, nor by a different role image.
    platform::guard_outcome(
        "IEEE identity",
        device.reset_security_state_if_identity_changed(security_store, ieee),
    );
    platform::guard_outcome(
        "role",
        security_store.select_role_tag(product::role::SLEEPY_END_DEVICE),
    );

    let node = ZigbeeNode::new(device, security_store, profile);
    let app = SensorApp::new(
        node,
        &product::sed::SED_POLICY,
        SensorSedParts {
            wake: Zbt1Wake,
            status: NoStatus,
            environment: NoMeasurement,
            battery: NoBattery,
            ota: NoOta,
            actions: product::sed::USER_ACTIONS,
            supervisor: ResetSupervisor,
            diagnostics: LogDiagnostics,
        },
    )
    .unwrap_or_else(|error| {
        log::error!("[ZBT1] invalid SED composition: {error:?}");
        halt()
    });
    APP.init(app)
}

/// RTCC/EM2 wait implementation (no wake button is fitted).
pub struct Zbt1Wake;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    Radio,
    Power,
}

impl WakeController<Mac> for Zbt1Wake {
    type Mark = Instant;
    type Error = WakeError;

    fn mark(&self) -> Self::Mark {
        Instant::now()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark + Duration::from_millis(u64::from(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later
            .saturating_duration_since(earlier)
            .as_millis()
            .min(u64::from(u32::MAX)) as u32
    }

    async fn wait(&mut self, mac: &mut Mac, request: WaitRequest) -> Result<WakeReason, WakeError> {
        let timeout = Duration::from_millis(u64::from(request.timeout_ms));
        match request.sleep_depth {
            SleepDepth::Active => Timer::after(timeout).await,
            SleepDepth::Idle => {
                mac.sleep().map_err(|_| WakeError::Radio)?;
                Timer::after(timeout).await;
                mac.wake().map_err(|_| WakeError::Radio)?;
            }
            SleepDepth::Retention => {
                mac.sleep().map_err(|_| WakeError::Radio)?;
                let ticks = rtcc::ms_to_ticks(request.timeout_ms.max(1));
                let slept = power::sleep_em2(ticks, || false);
                // Restore the radio on every path before reporting an error.
                let woke = mac.wake();
                slept.map_err(|_| WakeError::Power)?;
                woke.map_err(|_| WakeError::Radio)?;
            }
        }
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// No watchdog claim yet; fatal errors reset the MCU.
pub struct ResetSupervisor;

impl Supervisor for ResetSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

/// Lifecycle events on RTT, tagged with the EM2 entry count as sleep
/// evidence.
pub struct LogDiagnostics;

impl Diagnostics for LogDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        log::info!("[ZBT1] em2={} {event:?}", power::em2_entries());
    }
}
