//! # Zigbee-RS nRF52840 Sensor (DK / J-Link)
//!
//! Embassy-based Zigbee 3.0 sleepy end device for the Nordic nRF52840-DK.
//! Flashed via probe-rs (J-Link). Supports external I2C sensors:
//!
//! | Feature         | Sensor  | Clusters                         |
//! |-----------------|---------|----------------------------------|
//! | (none)          | On-chip | Temp + fake humidity             |
//! | `sensor-bme280` | BME280  | Temp + humidity + pressure       |
//! | `sensor-sht31`  | SHT31   | Temp + humidity                  |
//!
//! This is a composition root: it owns platform startup (clocks, RAM
//! power state, boot signal), resource construction from the
//! `nrf52840-dk` board and `nrf52840-sensor-product` product crates, and
//! the Embassy event loop. Endpoint/cluster composition, reporting
//! defaults, and measurement mapping live in the shared
//! `zigbee_runtime::profile` archetype selected by the product crate; NWK/
//! APS/ZDO/BDB state machines live in `zigbee-runtime`.
//!
//! ## Build & flash
//! ```sh
//! # On-chip only:
//! cargo build --release
//! probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
//!
//! # With BME280:
//! cargo build --release --features sensor-bme280
//! # With SHT31:
//! cargo build --release --features sensor-sht31
//! ```

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio, rng};
use embassy_time::{Duration, Instant, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
mod sensor;

// Bridge `log` crate → defmt so stack-internal log::info!/debug! appear in RTT output.
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

use nrf52840_sensor_product::profile::SensorProfile;
use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::profile::{ApplicationProfile, TemperatureHumidityMeasurement};
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};
use zigbee_runtime::ZigbeeDevice;
use zigbee_zcl::clusters::basic::PowerSource;

const REPORT_INTERVAL_SECS: u64 = 60;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;

#[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    SAADC => saadc::InterruptHandler;
    TWISPI0 => embassy_nrf::twim::InterruptHandler<peripherals::TWISPI0>;
});

#[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

// Ensure all RAM banks are powered on. POWER registers survive soft reset,
// so a previous firmware run may have powered down banks the stack needs.
// Runs as __pre_init before .bss zero, .data copy, and main().
// Pure assembly: zero stack usage (bank 8 section 5 may be powered off).
core::arch::global_asm!(
    ".section .text.__pre_init",
    ".global __pre_init",
    ".thumb_func",
    "__pre_init:",
    "ldr r0, =0x40000904", // POWER.RAM[0].POWERSET
    "mvn r1, #0",          // r1 = 0xFFFFFFFF
    "str r1, [r0, #0x00]", // RAM[0].POWERSET
    "str r1, [r0, #0x10]", // RAM[1].POWERSET
    "str r1, [r0, #0x20]", // RAM[2].POWERSET
    "str r1, [r0, #0x30]", // RAM[3].POWERSET
    "str r1, [r0, #0x40]", // RAM[4].POWERSET
    "str r1, [r0, #0x50]", // RAM[5].POWERSET
    "str r1, [r0, #0x60]", // RAM[6].POWERSET
    "str r1, [r0, #0x70]", // RAM[7].POWERSET
    "str r1, [r0, #0x80]", // RAM[8].POWERSET
    "bx lr",
);

#[allow(dead_code)]
/// Power down unused high RAM banks to reduce sleep current.
///
/// nRF52840 RAM layout: Banks 0-7 (8KB each, 64KB total) + Bank 8 (6×32KB, 192KB).
/// Embassy allocates task stacks from the top of RAM downward, so we can only
/// safely power down Bank 8 sections that are clearly above any possible stack use.
/// For a SED sensor (~37KB BSS + 8KB stack), banks 0-7 (64KB) are sufficient.
/// Bank 8 (0x20010000-0x20040000, 192KB) can be fully powered down.
///
/// Not currently called (kept from the original firmware unchanged): wiring
/// this in requires re-verifying stack/BSS headroom on hardware first.
fn power_down_unused_ram() {
    // Power down entire Bank 8 (192KB in 6 sections of 32KB)
    // Bank 8 starts at 0x20010000 — well above our ~37KB BSS + stack
    const POWER_BASE: usize = 0x4000_0900;
    let powerclr8 = (POWER_BASE + 8 * 0x10 + 0x08) as *mut u32;
    // All 6 sections off (bits 0-5 for power, bits 16-21 for retention)
    unsafe {
        core::ptr::write_volatile(powerclr8, 0x003F_003F);
    }
    info!("RAM: powered down 192KB (bank 8)");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Debug);

    let mut config = embassy_nrf::config::Config::default();
    // Use internal RC for HFCLK — radio requests XTAL automatically when needed.
    // Saves ~250µA vs keeping external XTAL always on.
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    // Enable DC-DC converter for ~40% lower current draw
    config.dcdc = embassy_nrf::config::DcdcConfig {
        reg0: true,
        reg0_voltage: None, // keep UICR default
        reg1: true,
    };
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52840 sensor starting…");

    // LED1 / Button 1 (board-owned physical wiring).
    let mut led = nrf52840_dk::led(p.P0_13);
    let mut button = nrf52840_dk::button(p.P0_11);

    // Boot signal: LED solid ON 2 seconds
    led.set_low(); // active LOW = ON
    Timer::after(Duration::from_secs(2)).await;
    led.set_high(); // OFF
    Timer::after(Duration::from_millis(500)).await;

    // ── Sensor init ──
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
    let mut temp_sensor = Temp::new(p.TEMP, Irqs);
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
    let mut hum_tick: u32 = 0;

    #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
    let mut env_sensor =
        sensor::Sensor::new(nrf52840_dk::sensor_i2c(p.TWISPI0, Irqs, p.P0_26, p.P0_27));

    // SAADC for battery voltage
    let mut saadc_sensor = Saadc::new(
        p.SAADC,
        Irqs,
        saadc::Config::default(),
        [ChannelConfig::single_ended(VddInput)],
    );
    saadc_sensor.calibrate().await;

    // Radio + MAC
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let rng = rng::Rng::new(p.RNG, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio, rng);
    mac.set_tx_power(0);
    info!("Radio ready (TX 0 dBm)");

    // ── Atomic security journal (last 8 KiB of 1 MiB flash) ──
    let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
    let mut security_store = nrf52840_sensor_product::storage::security_store(nvmc);
    info!("Security journal ready");

    // ── Product profile (endpoint, clusters, reporting defaults) ──
    let mut profile = nrf52840_sensor_product::profile::sensor_profile();

    // ── Build device ──
    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer(nrf52840_sensor_product::MANUFACTURER)
        .model(nrf52840_sensor_product::MODEL)
        .date_code(nrf52840_sensor_product::DATE_CODE)
        .sw_build(nrf52840_sensor_product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build();

    let mut node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);

    // Restore and reserve fresh security-counter ranges before any secured
    // resume traffic, or commission with durable reservations on first boot.
    if join_or_resume(&mut node).await {
        led.set_low();
    }

    // ── Read sensors once so clusters have real values for ZHA interview ──
    read_sensors(
        &mut node,
        #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
        &mut temp_sensor,
        #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
        &mut env_sensor,
        #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
        &mut hum_tick,
        &mut saadc_sensor,
    )
    .await;

    // ── Default reporting so device reports even without ZHA ConfigureReporting ──
    if let Err(error) = node.configure_default_reporting() {
        node_failure(NodeError::Profile(error));
    }
    info!("Default reporting configured");

    // ── Main loop state ──
    let mut last_report = Instant::now();
    let mut fast_poll_until = if node.device().is_joined() {
        info!("Fast poll ON ({}s) — post-join", FAST_POLL_DURATION_SECS);
        led.set_low(); // LED ON
        Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS)
    } else {
        Instant::now()
    };
    let mut last_rejoin_attempt = Instant::now();
    let mut rejoin_count: u8 = 0;
    let mut annce_retries_left: u8 = if node.device().is_joined() { 5 } else { 0 };
    let mut last_annce = Instant::now();
    let mut was_fast_polling = node.device().is_joined();
    let mut interview_done = false;
    loop {
        let now = Instant::now();
        let in_fast_poll = now < fast_poll_until;
        let poll_ms = if in_fast_poll {
            FAST_POLL_MS
        } else {
            SLOW_POLL_SECS * 1000
        };

        // Log transition from fast→slow poll
        if was_fast_polling && !in_fast_poll {
            let cfg = node
                .device()
                .configured_cluster_count(nrf52840_sensor_product::ENDPOINT);
            info!(
                "Fast poll OFF — {}/{} clusters configured",
                cfg,
                node.profile().expected_report_clusters()
            );
            was_fast_polling = false;
            if !interview_done {
                led.set_high(); // LED OFF
            }
        } else if in_fast_poll {
            was_fast_polling = true;
        }

        // ── Sleep until button or poll timer wake ──
        if node.device().is_joined() && node.device_mut().mac_mut().enter_low_power_idle().is_err()
        {
            warn!("Failed to disable RADIO before poll sleep");
        }
        match select(
            button.wait_for_falling_edge(),
            Timer::after(Duration::from_millis(poll_ms)),
        )
        .await
        {
            Either::First(_) => {
                // Check for long press (3s = factory reset)
                let held_long = matches!(
                    select(
                        button.wait_for_rising_edge(),
                        Timer::after(Duration::from_secs(3)),
                    )
                    .await,
                    Either::Second(_)
                );

                if held_long {
                    info!("FACTORY RESET");
                    if factory_reset(&mut node).await {
                        info!("Security state reset — rebooting");
                    }
                    for _ in 0..5u8 {
                        led.set_low();
                        Timer::after(Duration::from_millis(100)).await;
                        led.set_high();
                        Timer::after(Duration::from_millis(100)).await;
                    }
                    cortex_m::peripheral::SCB::sys_reset();
                } else if node.device().is_joined() {
                    info!("Button → leave");
                    if factory_reset(&mut node).await {
                        led.set_high();
                        info!("Left network and reset security state");
                    }
                } else {
                    info!("Button → join");
                    if join_or_resume(&mut node).await {
                        led.set_low();
                        fast_poll_until =
                            Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                        annce_retries_left = 5;
                        last_annce = Instant::now();
                        interview_done = false;
                    }
                }
                Timer::after(Duration::from_millis(300)).await;
            }
            Either::Second(_) => {} // Normal timeout — proceed to poll
        }
        // ── Poll parent for indirect frames (SED core) ──
        if node.device().is_joined() {
            for _poll_round in 0..4u8 {
                match node.device_mut().poll().await {
                    Ok(Some(ind)) => {
                        let event = match node.process_incoming(&ind).await {
                            Ok(event) => event,
                            Err(error) => node_failure(error),
                        };
                        if let Some(ev) = event {
                            match &ev {
                                StackEvent::RejoinRequested => {
                                    info!("Coordinator requested secure rejoin");
                                    if secure_rejoin(&mut node).await {
                                        fast_poll_until = Instant::now()
                                            + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                        interview_done = false;
                                        annce_retries_left = 5;
                                        last_annce = Instant::now();
                                        led.set_low();
                                    }
                                    break;
                                }
                                StackEvent::LeaveRequested => {
                                    info!("Coordinator sent Leave — resetting and rejoining");
                                    if factory_reset(&mut node).await
                                        && join_or_resume(&mut node).await
                                    {
                                        fast_poll_until = Instant::now()
                                            + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                        interview_done = false;
                                        annce_retries_left = 5;
                                        last_annce = Instant::now();
                                        led.set_low();
                                    }
                                    break;
                                }
                                _ => {}
                            }
                            if log_event(&ev, &mut led) {
                                fast_poll_until =
                                    Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                            }
                        }
                        // Check if ZHA completed interview
                        if !interview_done && node.reporting_is_configured() {
                            info!(
                                "Interview done! {}/{} clusters configured",
                                node.device()
                                    .configured_cluster_count(nrf52840_sensor_product::ENDPOINT),
                                node.profile().expected_report_clusters()
                            );
                            fast_poll_until = Instant::now() + Duration::from_secs(5);
                            interview_done = true;
                            led.set_high(); // LED OFF — power save
                        }
                        // Tick to send queued ZCL responses
                        if let Err(error) = node.tick(0).await {
                            node_failure(error);
                        }
                    }
                    Ok(None) => break,
                    Err(_) => break,
                }
            }

            // ── Periodic tasks ──
            let now2 = Instant::now();
            let elapsed_s = now2.duration_since(last_report).as_secs();

            if elapsed_s >= REPORT_INTERVAL_SECS {
                last_report = now2;
                read_sensors(
                    &mut node,
                    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
                    &mut temp_sensor,
                    #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
                    &mut env_sensor,
                    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
                    &mut hum_tick,
                    &mut saadc_sensor,
                )
                .await;
            }

            // Tick the runtime
            let tick_elapsed = elapsed_s.min(60) as u16;
            let tick_result = match node.tick(tick_elapsed).await {
                Ok(result) => result,
                Err(error) => node_failure(error),
            };
            if let TickResult::Event(ref e) = tick_result {
                if log_event(e, &mut led) {
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                }
            }

            // Identify LED blink
            if node
                .device()
                .is_identifying(nrf52840_sensor_product::ENDPOINT)
            {
                led.toggle();
            }

            // Device_annce retry
            if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                annce_retries_left -= 1;
                last_annce = now2;
                info!("Re-sending Device_annce ({} left)", annce_retries_left);
                checkpoint_security(&mut node);
                let _ = node.device_mut().send_device_annce().await;
                checkpoint_security(&mut node);
            }
        } else {
            // ── Not joined — blink and auto-retry ──
            let now2 = Instant::now();
            if now2.duration_since(last_rejoin_attempt).as_secs() >= 1 {
                // Double blink
                led.set_low();
                Timer::after(Duration::from_millis(80)).await;
                led.set_high();
                Timer::after(Duration::from_millis(120)).await;
                led.set_low();
                Timer::after(Duration::from_millis(80)).await;
                led.set_high();
            }

            if now2.duration_since(last_rejoin_attempt).as_secs() >= 15 {
                rejoin_count = rejoin_count.wrapping_add(1);
                last_rejoin_attempt = Instant::now();
                info!("Not joined — retrying (attempt {})…", rejoin_count);
                if let Err(error) = node.tick(0).await {
                    node_failure(error);
                }
                if node.device().is_joined()
                    || (!node.device().secure_rejoin_pending() && join_or_resume(&mut node).await)
                {
                    led.set_low();
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    annce_retries_left = 5;
                    last_annce = Instant::now();
                    interview_done = false;
                }
            }
        }
    }
}

async fn join_or_resume<M, S>(node: &mut ZigbeeNode<'_, M, S, SensorProfile>) -> bool
where
    M: MacDriver,
    S: SecurityStateStore,
{
    match node.start_or_resume().await {
        Ok(short_address) => {
            info!(
                "Joined/resumed network: addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address,
                node.device().channel(),
                node.device().pan_id()
            );
            // The runtime owns the R22 End Device Timeout lifecycle: a fresh
            // join or secured rejoin sends exactly one initial request, and a
            // silent resume reuses the persisted parent relationship. Sending
            // one here as well would duplicate the negotiation.
            checkpoint_security(node);
            true
        }
        Err(StartError::InitFailed) => {
            warn!("Zigbee initialization failed");
            false
        }
        Err(StartError::CommissioningFailed(status)) => {
            warn!("Commissioning failed: status=0x{:02X}", status as u8);
            false
        }
        Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
    }
}

async fn secure_rejoin<M, S>(node: &mut ZigbeeNode<'_, M, S, SensorProfile>) -> bool
where
    M: MacDriver,
    S: SecurityStateStore,
{
    match node.secure_rejoin().await {
        Ok(short_address) => {
            info!("Secure rejoin succeeded: addr=0x{:04X}", short_address);
            checkpoint_security(node);
            true
        }
        Err(StartError::InitFailed) => {
            warn!("Secure rejoin initialization failed");
            false
        }
        Err(StartError::CommissioningFailed(status)) => {
            warn!("Secure rejoin failed: status=0x{:02X}", status as u8);
            false
        }
        Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
    }
}

async fn factory_reset<M, S>(node: &mut ZigbeeNode<'_, M, S, SensorProfile>) -> bool
where
    M: MacDriver,
    S: SecurityStateStore,
{
    match node.factory_reset().await {
        Ok(()) => true,
        Err(StartError::InitFailed) => {
            warn!("Factory reset initialization failed");
            false
        }
        Err(StartError::CommissioningFailed(status)) => {
            warn!("Factory reset failed: status=0x{:02X}", status as u8);
            false
        }
        Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
    }
}

fn checkpoint_security<M, S>(node: &mut ZigbeeNode<'_, M, S, SensorProfile>)
where
    M: MacDriver,
    S: SecurityStateStore,
{
    if let Err(error) = node.checkpoint_security() {
        persistence_failure(error);
    }
}

#[inline(never)]
fn node_failure(error: NodeError) -> ! {
    match error {
        NodeError::Persistence(error) => persistence_failure(error),
        NodeError::Profile(error) => {
            error!("Profile error: {:?}", defmt::Debug2Format(&error));
            core::panic!("profile error");
        }
    }
}

#[inline(never)]
fn persistence_failure(error: SecurityStoreError) -> ! {
    match error {
        SecurityStoreError::NotFound => error!("Security persistence failed: not found"),
        SecurityStoreError::Corrupt => error!("Security persistence failed: corrupt state"),
        SecurityStoreError::Full => error!("Security persistence failed: full"),
        SecurityStoreError::Hardware => error!("Security persistence failed: hardware"),
        SecurityStoreError::CounterExhausted => {
            error!("Security persistence failed: counter exhausted")
        }
        SecurityStoreError::GenerationExhausted => {
            error!("Security persistence failed: generation exhausted")
        }
    }
    core::panic!("security persistence failure");
}

/// Read all sensors and update the profile's environment/battery clusters.
#[allow(unused_variables)]
async fn read_sensors<M, S>(
    node: &mut ZigbeeNode<'_, M, S, SensorProfile>,
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))] temp_sensor: &mut Temp<
        '_,
    >,
    #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
    env_sensor: &mut sensor::Sensor<'_>,
    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))] hum_tick: &mut u32,
    saadc: &mut Saadc<'_, 1>,
) where
    M: MacDriver,
    S: SecurityStateStore,
{
    let environment = node.profile_mut().component_mut();

    #[cfg(any(feature = "sensor-bme280", feature = "sensor-sht31"))]
    {
        if let Some(reading) = env_sensor.sample().await {
            environment.update_environment(TemperatureHumidityMeasurement {
                temperature_centi_celsius: reading.temperature_centi_celsius,
                humidity_centi_percent: reading.humidity_centi_percent,
            });
            #[cfg(feature = "sensor-bme280")]
            {
                environment.update_pressure(reading.pressure_hpa);
                info!(
                    "T={}.{:02}°C H={}.{:02}% P={}hPa",
                    reading.temperature_centi_celsius / 100,
                    (reading.temperature_centi_celsius % 100).unsigned_abs(),
                    reading.humidity_centi_percent / 100,
                    reading.humidity_centi_percent % 100,
                    reading.pressure_hpa,
                );
            }
            #[cfg(not(feature = "sensor-bme280"))]
            info!(
                "T={}.{:02}°C H={}.{:02}%",
                reading.temperature_centi_celsius / 100,
                (reading.temperature_centi_celsius % 100).unsigned_abs(),
                reading.humidity_centi_percent / 100,
                reading.humidity_centi_percent % 100,
            );
        } else {
            warn!("Environmental sensor read failed");
        }
    }

    #[cfg(not(any(feature = "sensor-bme280", feature = "sensor-sht31")))]
    {
        let raw_temp = temp_sensor.read().await;
        let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;
        *hum_tick = hum_tick.wrapping_add(1);
        let hum_hundredths = 5000u16 + ((*hum_tick % 100) as u16).wrapping_mul(10);
        environment.update_environment(TemperatureHumidityMeasurement {
            temperature_centi_celsius: temp_hundredths,
            humidity_centi_percent: hum_hundredths,
        });
        info!(
            "T={}.{:02}°C H={}.{:02}% (on-chip)",
            temp_hundredths / 100,
            (temp_hundredths % 100).unsigned_abs(),
            hum_hundredths / 100,
            hum_hundredths % 100
        );
    }

    // Battery
    let mut buf = [0i16; 1];
    saadc.sample(&mut buf).await;
    let measurement = nrf52840_sensor_product::battery::battery_measurement(buf[0]);
    info!(
        "Battery: {}mV ({}%)",
        nrf52840_sensor_product::battery::millivolts(buf[0]),
        measurement.percentage_remaining / 2
    );
    environment.update_battery(measurement);
}

/// LED ON = joined, blink = joining, OFF = idle. Returns true on join event.
fn log_event(event: &StackEvent, led: &mut gpio::Output<'_>) -> bool {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            led.set_low(); // ON
            info!(
                "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id
            );
            true
        }
        StackEvent::Left => {
            led.set_high(); // OFF
            info!("Left network");
            false
        }
        StackEvent::ReportSent => {
            info!("Report sent");
            false
        }
        StackEvent::LeaveRequested | StackEvent::RejoinRequested => {
            led.set_low(); // ON — rejoining
            info!("Leave requested by coordinator");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            info!("Commissioning: {}", if *success { "ok" } else { "failed" });
            false
        }
        _ => {
            info!("Stack event");
            false
        }
    }
}
