//! Pure-Rust BL702 Zigbee temperature, humidity, and battery sensor.

#![no_std]
#![no_main]

#[cfg(all(feature = "production", feature = "diagnostic-logging"))]
compile_error!("select either production or diagnostic-logging, not both");
#[cfg(not(any(feature = "production", feature = "diagnostic-logging")))]
compile_error!("select production or diagnostic-logging");

mod hal;

use core::fmt::{Display, Formatter, Write};

use bl702_hal::{adc::Gpadc, clock::Clocks};
use bl702_xt_zb1_product as product;
use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Instant, Timer};
use embassy_time_driver::Driver;
use panic_halt as _;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::{MacPib, SoftMacCore, bl702::radio_phy::Bl702RadioPhy};
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_types::{ChannelMask, IeeeAddress};
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::{ClusterId, DeviceId};

const CHANNEL: u8 = 15;
const CHANNEL_MASK: ChannelMask = ChannelMask(1 << CHANNEL);
const TX_POWER_DBM: i8 = 0;
const LOOP_INTERVAL_MS: u64 = 250;
const JOIN_RETRY_INTERVAL_SECS: u32 = 15;
const REPORT_INTERVAL_SECS: u32 = 30;
const PARENT_POLL_FAILURE_LIMIT: u8 = 16;
const SECURE_REJOIN_FAILURE_LIMIT: u8 = 4;

#[derive(Clone, Copy)]
enum ParentPollOutcome {
    Reachable,
    Failed,
    RejoinFailed,
}

mod time_driver {
    use super::*;

    struct Bl702TimeDriver;

    impl Driver for Bl702TimeDriver {
        fn now(&self) -> u64 {
            hal::timer_ticks()
        }

        fn schedule_wake(&self, _at: u64, waker: &core::task::Waker) {
            // The initial BL702 port uses a polling executor. The independent
            // 1 MHz timer still provides accurate radio and stack deadlines.
            waker.wake_by_ref();
        }
    }

    embassy_time_driver::time_driver_impl!(
        static DRIVER: Bl702TimeDriver = Bl702TimeDriver
    );
}

struct UartWriter;

impl Write for UartWriter {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        for byte in text.bytes() {
            hal::uart_write(byte).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

struct Logger;

impl log::Log for Logger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let mut uart = UartWriter;
        let _ = writeln!(uart, "[{}] {}\r", record.level(), record.args());
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

struct Hex<'a>(&'a [u8]);

impl Display for Hex<'_> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> core::fmt::Result {
        for byte in self.0.iter().rev() {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

#[riscv_rt::entry]
fn main() -> ! {
    let application_resources = match hal::init() {
        Ok(resources) => resources,
        Err(_) => halt(),
    };
    for byte in b"BL702 boot\r\n" {
        let _ = hal::uart_write(*byte);
    }

    // BL702 is single-hart and has no RV32A extension. Interrupts are still
    // disabled here, so the non-atomic logger setup is the correct startup path.
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(log::LevelFilter::Info);
    }
    marker(b"logger ready\r\n");

    let mut executor = Executor::new();
    let executor: &'static mut Executor = unsafe { core::mem::transmute(&mut executor) };
    marker(b"executor ready\r\n");
    executor.run(move |spawner| spawner.must_spawn(sensor(spawner, application_resources)))
}

#[embassy_executor::task]
async fn sensor(_spawner: Spawner, application: hal::ApplicationResources) {
    marker(b"sensor task\r\n");
    log::info!("Zigbee-RS BL702 pure-Rust sensor");

    let hal::ApplicationResources {
        i2c0,
        spi_or_usb,
        adc,
        flash,
        power: power_control,
        pwm,
        uart1,
        other_pins,
    } = application;
    let mut supply_adc = match Gpadc::new(adc, Clocks::rom_boot_32mhz()) {
        Ok(adc) => {
            if !adc.gain_trim_valid() {
                log::warn!("GPADC eFuse gain trim is invalid; using nominal conversion");
            }
            Some(adc)
        }
        Err(error) => {
            log::warn!("GPADC initialization failed; battery remains synthetic: {error:?}");
            None
        }
    };
    let mut security_store = match product::storage::security_store(flash) {
        Ok(store) => store,
        Err(error) => {
            log::error!("security storage initialization failed: {error:?}");
            halt()
        }
    };
    // Retain ownership of unused application peripherals for the task's
    // lifetime. No physical environmental sensor is assumed on this board.
    let _reserved_peripherals = (i2c0, spi_or_usb, power_control, pwm, uart1, other_pins);

    let ieee = device_ieee_address();
    log::info!("IEEE address: {}", Hex(&ieee));
    log::info!("initializing RF and running per-die calibration");

    let mut radio = match unsafe { Bl702RadioPhy::initialize(hal::delay_us) } {
        Ok(radio) => radio,
        Err(error) => fatal("RF initialization", error),
    };
    if let Err(error) = zigbee_mac::RadioPhy::set_tx_power(&mut radio, TX_POWER_DBM) {
        fatal("TX power setup", error);
    }

    let pib = MacPib::new(ieee, ieee[0], ieee[1]);
    let mac = match SoftMacCore::new(radio, pib) {
        Ok(mac) => mac,
        Err(error) => {
            log::error!("MAC initialization failed: {error:?}");
            halt()
        }
    };
    log::info!("radio ready: channel scan={CHANNEL}, tx_power={TX_POWER_DBM} dBm");

    let mut temperature = TemperatureCluster::new(-4000, 12500);
    let mut humidity = HumidityCluster::new(0, 10000);
    let mut power = PowerConfigCluster::new();
    update_readings(
        0,
        read_supply_mv(&mut supply_adc),
        &mut temperature,
        &mut humidity,
        &mut power,
    );

    let mut device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .automatic_polling(false)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .date_code(product::DATE_CODE)
        .sw_build(product::SW_BUILD)
        .power_source(PowerSource::Battery)
        .channels(CHANNEL_MASK)
        .endpoint(
            product::ENDPOINT,
            PROFILE_HOME_AUTOMATION,
            DeviceId::TEMPERATURE_SENSOR,
            |endpoint| {
                endpoint
                    .cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::IDENTIFY)
                    .cluster_server(ClusterId::POWER_CONFIG)
                    .cluster_server(ClusterId::TEMPERATURE)
                    .cluster_server(ClusterId::HUMIDITY)
            },
        )
        .build();

    match device.reset_security_state_if_identity_changed(&mut security_store, ieee) {
        Ok(true) => log::warn!("cleared persisted network state after IEEE address change"),
        Ok(false) => {}
        Err(error) => {
            log::error!("security state validation failed: {error:?}");
            halt()
        }
    }

    log::info!("sensor endpoint ready; starting or resuming network");
    request_join(&mut device, &mut security_store).await;

    let mut quarter_seconds = 0u32;
    let mut joined_quarter_seconds = 0u32;
    let mut report_sequence = 0u32;
    let mut announce_retries = 0u8;
    let mut last_announce = Instant::now();
    let mut consecutive_poll_failures = 0u8;
    let mut consecutive_rejoin_failures = 0u8;

    loop {
        Timer::after(Duration::from_millis(LOOP_INTERVAL_MS)).await;
        quarter_seconds = quarter_seconds.wrapping_add(1);

        if !device.is_joined() {
            if quarter_seconds >= JOIN_RETRY_INTERVAL_SECS * 4 {
                quarter_seconds = 0;
                request_join(&mut device, &mut security_store).await;
                if device.is_joined() {
                    consecutive_rejoin_failures = 0;
                    joined_quarter_seconds = 0;
                    announce_retries = 5;
                    last_announce = Instant::now();
                } else if device.secure_rejoin_pending() {
                    record_failed_rejoin(
                        &mut device,
                        &mut security_store,
                        &mut consecutive_rejoin_failures,
                    )
                    .await;
                }
            }
            continue;
        }

        joined_quarter_seconds = joined_quarter_seconds.wrapping_add(1);
        let poll_outcome = poll_parent(
            &mut device,
            &mut temperature,
            &mut humidity,
            &mut power,
            &mut security_store,
        )
        .await;
        match poll_outcome {
            ParentPollOutcome::Reachable => consecutive_poll_failures = 0,
            ParentPollOutcome::RejoinFailed => {
                consecutive_poll_failures = 0;
                record_failed_rejoin(
                    &mut device,
                    &mut security_store,
                    &mut consecutive_rejoin_failures,
                )
                .await;
            }
            ParentPollOutcome::Failed => {
                consecutive_poll_failures = consecutive_poll_failures.saturating_add(1);
                if consecutive_poll_failures >= PARENT_POLL_FAILURE_LIMIT {
                    consecutive_poll_failures = 0;
                    log::warn!("parent is unreachable; attempting a secured rejoin");
                    match device
                        .secure_rejoin_with_security_store(&mut security_store)
                        .await
                    {
                        Ok(_) => consecutive_rejoin_failures = 0,
                        Err(StartError::CommissioningFailed(error)) => {
                            log::warn!("secured rejoin failed: {error:?}");
                            record_failed_rejoin(
                                &mut device,
                                &mut security_store,
                                &mut consecutive_rejoin_failures,
                            )
                            .await;
                        }
                        Err(error) => start_fatal("secured rejoin", error),
                    }
                }
            }
        }
        if !device.is_joined() {
            quarter_seconds = 0;
            continue;
        }

        let elapsed_secs = if quarter_seconds.is_multiple_of(4) {
            1
        } else {
            0
        };
        let result = {
            let mut clusters = cluster_refs(&mut temperature, &mut humidity, &mut power);
            device
                .tick_with_security_store(elapsed_secs, &mut clusters, &mut security_store)
                .await
        };
        match result {
            Ok(result) => log_tick_result(&result),
            Err(error) => security_fatal("stack tick", error),
        }

        if announce_retries > 0 && last_announce.elapsed() >= Duration::from_secs(8) {
            announce_retries -= 1;
            last_announce = Instant::now();
            if let Err(error) = device.send_device_annce().await {
                log::warn!("Device_annce retry failed: {error:?}");
            } else {
                log::info!("Device_annce retry sent ({announce_retries} left)");
            }
        }

        if joined_quarter_seconds >= REPORT_INTERVAL_SECS * 4 {
            joined_quarter_seconds = 0;
            report_sequence = report_sequence.wrapping_add(1);
            update_readings(
                report_sequence,
                read_supply_mv(&mut supply_adc),
                &mut temperature,
                &mut humidity,
                &mut power,
            );
        }
    }
}

async fn request_join<M: zigbee_mac::MacDriver, S: SecurityStateStore>(
    device: &mut ZigbeeDevice<M>,
    store: &mut S,
) {
    match device.start_or_resume_with_security_store(store).await {
        Ok(_) => {
            log::info!(
                "network ready: short=0x{:04x}, PAN=0x{:04x}, channel={}",
                device.short_address(),
                device.pan_id(),
                device.channel()
            );
        }
        Err(error) => {
            if matches!(error, StartError::CommissioningFailed(_)) {
                log::warn!(
                    "network start/resume failed; retry in {JOIN_RETRY_INTERVAL_SECS}s: \
                     {error:?}; diagnostics={:?}",
                    device.steering_diagnostics()
                );
            } else {
                log::error!("network start/resume failed irrecoverably: {error:?}");
                halt()
            }
        }
    }
}

async fn poll_parent<M: zigbee_mac::MacDriver, S: SecurityStateStore>(
    device: &mut ZigbeeDevice<M>,
    temperature: &mut TemperatureCluster,
    humidity: &mut HumidityCluster,
    power: &mut PowerConfigCluster,
    store: &mut S,
) -> ParentPollOutcome {
    for _ in 0..4 {
        match device.poll().await {
            Ok(Some(indication)) => {
                let event = {
                    let mut clusters = cluster_refs(temperature, humidity, power);
                    device
                        .process_incoming_with_security_store(&indication, &mut clusters, store)
                        .await
                };
                match event {
                    Ok(Some(StackEvent::RejoinRequested)) => {
                        log::warn!("parent requested a secured rejoin");
                        match device.secure_rejoin_with_security_store(store).await {
                            Ok(_) => {}
                            Err(StartError::CommissioningFailed(error)) => {
                                log::warn!("secured rejoin failed: {error:?}");
                                return ParentPollOutcome::RejoinFailed;
                            }
                            Err(error) => start_fatal("secured rejoin", error),
                        }
                    }
                    Ok(Some(StackEvent::LeaveRequested | StackEvent::Left)) => {
                        log::warn!("network leave requested; clearing persisted network state");
                        if let Err(error) = device.factory_reset_with_security_store(store).await {
                            log::error!("durable factory reset failed: {error:?}");
                            halt()
                        }
                        return ParentPollOutcome::Reachable;
                    }
                    Ok(Some(event)) => log_stack_event(&event),
                    Ok(None) => {}
                    Err(error) => security_fatal("incoming frame processing", error),
                }

                if !device.is_joined() {
                    return ParentPollOutcome::Reachable;
                }
                let result = {
                    let mut clusters = cluster_refs(temperature, humidity, power);
                    device
                        .tick_with_security_store(0, &mut clusters, store)
                        .await
                };
                match result {
                    Ok(result) => log_tick_result(&result),
                    Err(error) => security_fatal("stack tick", error),
                }
            }
            Ok(None) => return ParentPollOutcome::Reachable,
            Err(error) => {
                log::warn!("parent poll failed: {error:?}");
                return ParentPollOutcome::Failed;
            }
        }
    }
    ParentPollOutcome::Reachable
}

async fn record_failed_rejoin<M: zigbee_mac::MacDriver, S: SecurityStateStore>(
    device: &mut ZigbeeDevice<M>,
    store: &mut S,
    failures: &mut u8,
) {
    *failures = failures.saturating_add(1);
    if *failures < SECURE_REJOIN_FAILURE_LIMIT {
        return;
    }

    log::warn!(
        "persisted parent remained unreachable after {SECURE_REJOIN_FAILURE_LIMIT} secured \
         rejoin attempts; clearing stale network state"
    );
    if let Err(error) = device.factory_reset_with_security_store(store).await {
        start_fatal("stale-network reset", error);
    }
    *failures = 0;
}

fn cluster_refs<'a>(
    temperature: &'a mut TemperatureCluster,
    humidity: &'a mut HumidityCluster,
    power: &'a mut PowerConfigCluster,
) -> [ClusterRef<'a>; 3] {
    [
        ClusterRef {
            endpoint: product::ENDPOINT,
            cluster: temperature,
        },
        ClusterRef {
            endpoint: product::ENDPOINT,
            cluster: humidity,
        },
        ClusterRef {
            endpoint: product::ENDPOINT,
            cluster: power,
        },
    ]
}

fn read_supply_mv(adc: &mut Option<Gpadc>) -> Option<u16> {
    let adc = adc.as_mut()?;
    match adc.read_supply_mv() {
        Ok(millivolts) if (1_800..=3_800).contains(&millivolts) => Some(millivolts),
        Ok(millivolts) => {
            log::warn!(
                "GPADC supply reading is implausible ({millivolts} mV); battery remains synthetic"
            );
            None
        }
        Err(error) => {
            log::warn!("GPADC supply read failed; battery remains synthetic: {error:?}");
            None
        }
    }
}

fn update_readings(
    sequence: u32,
    supply_mv: Option<u16>,
    temperature: &mut TemperatureCluster,
    humidity: &mut HumidityCluster,
    power: &mut PowerConfigCluster,
) {
    let temp = 2250 + (sequence % 20) as i16;
    let hum = 5000 + ((sequence * 7) % 100) as u16;
    temperature.set_temperature(temp);
    humidity.set_humidity(hum);
    if let Some(millivolts) = supply_mv {
        let voltage = ((u32::from(millivolts) + 50) / 100).min(u32::from(u8::MAX)) as u8;
        let percentage = if millivolts <= 2_000 {
            0
        } else if millivolts >= 3_000 {
            200
        } else {
            ((u32::from(millivolts - 2_000) * 200) / 1_000) as u8
        };
        power.set_battery_voltage(voltage);
        power.set_battery_percentage(percentage);
        log::info!(
            "sensor values: {}.{:02} C synthetic, {}.{:02}% RH synthetic, {}.{:03} V nominal",
            temp / 100,
            temp.unsigned_abs() % 100,
            hum / 100,
            hum % 100,
            millivolts / 1_000,
            millivolts % 1_000
        );
    } else {
        power.set_battery_voltage(30);
        power.set_battery_percentage(200);
        log::info!(
            "sensor values (synthetic): {}.{:02} C, {}.{:02}% RH, 3.0 V",
            temp / 100,
            temp.unsigned_abs() % 100,
            hum / 100,
            hum % 100
        );
    }
}

fn log_tick_result(result: &TickResult) {
    if let TickResult::Event(event) = result {
        log_stack_event(event);
    }
}

fn log_stack_event(event: &StackEvent) {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => log::info!(
            "stack joined: short=0x{short_address:04x}, PAN=0x{pan_id:04x}, channel={channel}"
        ),
        StackEvent::CommissioningComplete { success } => {
            log::info!("commissioning complete: success={success}")
        }
        StackEvent::ReportSent => log::info!("attribute report sent"),
        StackEvent::Left => log::warn!("device left the network"),
        other => log::info!("stack event: {other:?}"),
    }
}

fn device_ieee_address() -> IeeeAddress {
    let id = hal::chip_id();
    if id == [0; 8] || id == [0xff; 8] {
        // Stable locally administered address for boards without programmed ID.
        [0x02, 0x70, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01]
    } else {
        id
    }
}

fn fatal(context: &str, error: zigbee_mac::PhyError) -> ! {
    log::error!("{context} failed: {error:?}");
    halt()
}

fn security_fatal(context: &str, error: zigbee_runtime::security_store::SecurityStoreError) -> ! {
    log::error!("{context} failed because durable security state is unavailable: {error:?}");
    halt()
}

fn start_fatal(context: &str, error: StartError) -> ! {
    log::error!("{context} failed irrecoverably: {error:?}");
    halt()
}

fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

fn marker(text: &[u8]) {
    for byte in text {
        let _ = hal::uart_write(*byte);
    }
}
