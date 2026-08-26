//! nRF52840-DK always-on Zigbee End Device.
//!
//! `NrfMac` is not a `ParentMacDriver`, so this composition root selects
//! [`router_app::AlwaysOnEndDeviceApp`]. The device is mains-powered with
//! `macRxOnWhenIdle = true`, but it never advertises `DeviceType::Router`,
//! routes, or admits children.
//!
//! Product identity, the Home Automation Range Extender profile, scheduling
//! policy, status semantics, and the protected security journal are owned by
//! `products/nrf52840-router`. This file owns only Nordic startup, physical DK
//! resources, the FICR identity guard, AES installation/KAT, and composition.

#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_nrf::gpio;
use embassy_nrf::{bind_interrupts, peripherals, radio, rng};
use embassy_time::{Duration, Instant};

use nrf52840_router_product::status::StatusLedState;
use router_app::{
    AlwaysOnEndDeviceApp, DiagnosticEvent, Diagnostics, NodeArchetype, RouterAppError,
    RouterParts, RouterStatus, StackEventSummary, StatusSink, Supervisor,
};
use zigbee_nwk::DeviceType;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::ApplicationProfile;
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

const FACTORY_RESET_HOLD: Duration = Duration::from_secs(3);

struct DefmtLogger;

impl log::Log for DefmtLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        match record.level() {
            log::Level::Error => defmt::error!("{}", defmt::Display2Format(record.args())),
            log::Level::Warn => defmt::warn!("{}", defmt::Display2Format(record.args())),
            log::Level::Info => defmt::info!("{}", defmt::Display2Format(record.args())),
            log::Level::Debug => defmt::debug!("{}", defmt::Display2Format(record.args())),
            log::Level::Trace => defmt::trace!("{}", defmt::Display2Format(record.args())),
        }
    }

    fn flush(&self) {}
}

static LOGGER: DefmtLogger = DefmtLogger;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

// POWER state survives a soft reset. Restore every RAM bank before
// cortex-m-rt touches .data/.bss so this image can safely follow firmware
// which used RAM-bank power saving.
core::arch::global_asm!(
    ".section .text.__pre_init",
    ".global __pre_init",
    ".thumb_func",
    "__pre_init:",
    "ldr r0, =0x40000904",
    "mvn r1, #0",
    "str r1, [r0, #0x00]",
    "str r1, [r0, #0x10]",
    "str r1, [r0, #0x20]",
    "str r1, [r0, #0x30]",
    "str r1, [r0, #0x40]",
    "str r1, [r0, #0x50]",
    "str r1, [r0, #0x60]",
    "str r1, [r0, #0x70]",
    "str r1, [r0, #0x80]",
    "bx lr",
);

struct DkRouterStatus {
    led: gpio::Output<'static>,
}

impl DkRouterStatus {
    const fn new(led: gpio::Output<'static>) -> Self {
        Self { led }
    }
}

impl StatusSink for DkRouterStatus {
    fn set(&mut self, status: RouterStatus) {
        match nrf52840_router_product::status::led1_state(status) {
            StatusLedState::On => self.led.set_low(),
            StatusLedState::Off => self.led.set_high(),
        }
    }
}

/// LED2 is RX activity, not proof that a frame was forwarded.
///
/// Toggling is synchronous and bounded, so activity indication cannot insert a
/// radio-off delay between the application's continuous receive slices.
struct DkRouterDiagnostics {
    rx_activity: gpio::Output<'static>,
}

impl DkRouterDiagnostics {
    const fn new(rx_activity: gpio::Output<'static>) -> Self {
        Self { rx_activity }
    }
}

impl Diagnostics for DkRouterDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        match event {
            DiagnosticEvent::InitializationStarted { .. } => {
                info!("Always-on End Device lifecycle starting")
            }
            DiagnosticEvent::CommissioningAttempt { attempt, .. } => {
                info!("Commissioning attempt {}", attempt)
            }
            DiagnosticEvent::StartFailed { error } => {
                warn!(
                    "Commissioning/resume failed: {:?}",
                    defmt::Debug2Format(&error)
                )
            }
            DiagnosticEvent::RetryScheduled { attempt, delay_ms } => {
                info!("Retry {} scheduled in {} ms", attempt, delay_ms)
            }
            DiagnosticEvent::NetworkReady {
                short_address,
                channel,
                pan_id,
                ..
            } => info!(
                "Always-on End Device online: addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id
            ),
            DiagnosticEvent::SecurityCheckpoint { changed: true } => {
                debug!("Security journal checkpointed")
            }
            DiagnosticEvent::FrameReceived => self.rx_activity.toggle(),
            DiagnosticEvent::StackEvent(StackEventSummary::Left) => {
                info!("Left network")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::LeaveRequested) => {
                info!("Leave requested; durable reset and recommission")
            }
            DiagnosticEvent::StackEvent(StackEventSummary::RejoinRequested) => {
                info!("Secure rejoin requested")
            }
            DiagnosticEvent::SecureRejoinSucceeded { short_address } => {
                info!("Secure rejoin succeeded: addr=0x{:04X}", short_address)
            }
            DiagnosticEvent::SecureRejoinFailed {
                error, failures, ..
            } => warn!(
                "Secure rejoin failed ({}/{}): {:?}",
                failures,
                nrf52840_router_product::policy::ALWAYS_ON_END_DEVICE_POLICY
                    .secure_rejoin_failure_limit,
                defmt::Debug2Format(&error)
            ),
            DiagnosticEvent::SecureRejoinRetryFailed { failures }
            | DiagnosticEvent::SecureRejoinPending { failures } => {
                warn!("Secure rejoin pending after {} failure(s)", failures)
            }
            DiagnosticEvent::SecureRejoinLimitReached { failures } => {
                warn!("Secure rejoin limit reached at {}", failures)
            }
            DiagnosticEvent::FactoryReset => info!("Security journal reset committed"),
            DiagnosticEvent::Fatal(error) => {
                error!(
                    "Fatal always-on End Device application error: {:?}",
                    defmt::Debug2Format(&error)
                )
            }
            _ => {}
        }
    }
}

/// This product does not enable the hardware watchdog yet.
///
/// Heartbeat and watchdog-bound queries are therefore intentionally no-op,
/// while fatal reset is real and uses the Cortex-M system reset request.
#[derive(Debug, Default, Clone, Copy)]
struct NrfResetOnlySupervisor;

impl Supervisor for NrfResetOnlySupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

type NrfEndDeviceParts =
    RouterParts<DkRouterStatus, NrfResetOnlySupervisor, DkRouterDiagnostics>;

fn fatal(parts: &mut NrfEndDeviceParts, error: RouterAppError) -> ! {
    parts.diagnostics.record(DiagnosticEvent::Fatal(error));
    parts.status.set(RouterStatus::Fault {
        archetype: NodeArchetype::AlwaysOnEndDevice,
    });
    parts.supervisor.reset()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    config.dcdc = embassy_nrf::config::DcdcConfig {
        reg0: true,
        reg0_voltage: None,
        reg1: true,
    };
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52840 always-on End Device starting");

    let mut status = DkRouterStatus::new(nrf52840_dk::status_led(p.P0_13));
    let mut diagnostics = DkRouterDiagnostics::new(nrf52840_dk::rx_activity_led(p.P0_14));
    let button = nrf52840_dk::button(p.P0_11);

    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let rng = rng::Rng::new(p.RNG, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio, rng);

    let Some(aes) = zigbee_mac::nrf::NrfEcbToken::take() else {
        status.set(RouterStatus::Fault {
            archetype: NodeArchetype::AlwaysOnEndDevice,
        });
        error!("Nordic ECB AES already owned; networking halted");
        loop {
            cortex_m::asm::wfi();
        }
    };
    if let Err(error) = mac.install_aes_engine(aes) {
        status.set(RouterStatus::Fault {
            archetype: NodeArchetype::AlwaysOnEndDevice,
        });
        error!(
            "Nordic ECB dual-KAT failed; networking halted: {:?}",
            defmt::Debug2Format(&error)
        );
        loop {
            cortex_m::asm::wfi();
        }
    }
    let ieee = mac.extended_address();
    info!("Nordic ECB hardware AES dual-KAT passed");
    mac.set_tx_power(0);

    let mut profile = nrf52840_router_product::profile::always_on_end_device_profile();
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer(nrf52840_router_product::MANUFACTURER)
        .model(nrf52840_router_product::MODEL)
        .date_code(nrf52840_router_product::DATE_CODE)
        .sw_build(nrf52840_router_product::SW_BUILD)
        .power_source(PowerSource::MainsSinglePhase)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build();

    let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let mut security_store = nrf52840_router_product::storage::security_store(nvmc);
    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => warn!("Cleared network identity after FICR EUI-64 change"),
        Ok(false) => {}
        Err(error) => {
            diagnostics.record(DiagnosticEvent::Fatal(RouterAppError::Security(error)));
            status.set(RouterStatus::Fault {
                archetype: NodeArchetype::AlwaysOnEndDevice,
            });
            loop {
                cortex_m::asm::wfi();
            }
        }
    }

    let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
    let mut app = AlwaysOnEndDeviceApp::new(
        node,
        &nrf52840_router_product::policy::ALWAYS_ON_END_DEVICE_POLICY,
        RouterParts::new(status, NrfResetOnlySupervisor, diagnostics),
    )
    .expect("nRF52840 End Device composition is statically valid");

    if let Err(error) = app.initialize().await {
        fatal(app.parts_mut(), error);
    }

    let mut reset_pressed_at: Option<Instant> = None;
    loop {
        if button.is_low() {
            let pressed_at = *reset_pressed_at.get_or_insert_with(Instant::now);
            if pressed_at.elapsed() >= FACTORY_RESET_HOLD {
                app.parts_mut().status.set(RouterStatus::Resetting {
                    archetype: NodeArchetype::AlwaysOnEndDevice,
                });
                info!("Button 1 hold: committing durable factory reset");
                match app.urgent_factory_reset_and_recommission().await {
                    Ok(()) => {
                        let parts = app.parts_mut();
                        parts.diagnostics.record(DiagnosticEvent::FactoryReset);
                        parts.supervisor.reset();
                    }
                    Err(error) => fatal(app.parts_mut(), error),
                }
            }
        } else {
            reset_pressed_at = None;
        }

        // AlwaysOnEndDeviceApp bounds this receive/tick cycle to the product's 20 ms
        // receive slice (or an earlier runtime deadline). There is no
        // application-side sleep between slices.
        if let Err(error) = app.step().await {
            fatal(app.parts_mut(), error);
        }
    }
}
