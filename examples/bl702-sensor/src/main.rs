//! Pure-Rust BL702 Zigbee temperature, humidity, and battery sensor.

#![no_std]
#![no_main]

mod hal;

use core::fmt::{Display, Formatter, Write};

use embassy_executor::{Executor, Spawner};
use embassy_time::{Duration, Instant, Timer};
use embassy_time_driver::Driver;
use panic_halt as _;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::{MacPib, SoftMacCore, bl702::radio_phy::Bl702RadioPhy};
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
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
const DEVICE_ENDPOINT: u8 = 1;

mod time_driver {
    use super::*;

    struct Bl702TimeDriver;

    impl Driver for Bl702TimeDriver {
        fn now(&self) -> u64 {
            u64::from(hal::timer_ticks())
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
            hal::uart_write(byte);
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
    hal::init();
    for byte in b"BL702 boot\r\n" {
        hal::uart_write(*byte);
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
    executor.run(|spawner| spawner.must_spawn(sensor(spawner)))
}

#[embassy_executor::task]
async fn sensor(_spawner: Spawner) {
    marker(b"sensor task\r\n");
    log::info!("zigbee-rs BL702 pure-Rust sensor");

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
    update_synthetic_readings(0, &mut temperature, &mut humidity, &mut power);

    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .automatic_polling(false)
        .manufacturer("zigbee-rs")
        .model("XT-ZB1 Sensor")
        .date_code("20260402")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(CHANNEL_MASK)
        .endpoint(
            DEVICE_ENDPOINT,
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

    log::info!("sensor endpoint ready; requesting network join");
    request_join(&mut device, &mut temperature, &mut humidity, &mut power).await;

    let mut quarter_seconds = 0u32;
    let mut joined_quarter_seconds = 0u32;
    let mut report_sequence = 0u32;
    let mut announce_retries = 0u8;
    let mut last_announce = Instant::now();

    loop {
        Timer::after(Duration::from_millis(LOOP_INTERVAL_MS)).await;
        quarter_seconds = quarter_seconds.wrapping_add(1);

        if !device.is_joined() {
            if quarter_seconds >= JOIN_RETRY_INTERVAL_SECS * 4 {
                quarter_seconds = 0;
                request_join(&mut device, &mut temperature, &mut humidity, &mut power).await;
                if device.is_joined() {
                    joined_quarter_seconds = 0;
                    announce_retries = 5;
                    last_announce = Instant::now();
                }
            }
            continue;
        }

        joined_quarter_seconds = joined_quarter_seconds.wrapping_add(1);
        poll_parent(&mut device, &mut temperature, &mut humidity, &mut power).await;

        let elapsed_secs = if quarter_seconds.is_multiple_of(4) {
            1
        } else {
            0
        };
        let result = {
            let mut clusters = cluster_refs(&mut temperature, &mut humidity, &mut power);
            device.tick(elapsed_secs, &mut clusters).await
        };
        log_tick_result(&result);

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
            update_synthetic_readings(report_sequence, &mut temperature, &mut humidity, &mut power);
        }
    }
}

async fn request_join<M: zigbee_mac::MacDriver>(
    device: &mut ZigbeeDevice<M>,
    temperature: &mut TemperatureCluster,
    humidity: &mut HumidityCluster,
    power: &mut PowerConfigCluster,
) {
    device.user_action(UserAction::Join);
    let result = {
        let mut clusters = cluster_refs(temperature, humidity, power);
        device.tick(0, &mut clusters).await
    };
    log_tick_result(&result);

    if device.is_joined() {
        log::info!(
            "joined: short=0x{:04x}, PAN=0x{:04x}, channel={}",
            device.short_address(),
            device.pan_id(),
            device.channel()
        );
    } else {
        log::warn!(
            "join failed; retry in {JOIN_RETRY_INTERVAL_SECS}s: {:?}",
            device.steering_diagnostics()
        );
    }
}

async fn poll_parent<M: zigbee_mac::MacDriver>(
    device: &mut ZigbeeDevice<M>,
    temperature: &mut TemperatureCluster,
    humidity: &mut HumidityCluster,
    power: &mut PowerConfigCluster,
) {
    for _ in 0..4 {
        match device.poll().await {
            Ok(Some(indication)) => {
                let event = {
                    let mut clusters = cluster_refs(temperature, humidity, power);
                    device.process_incoming(&indication, &mut clusters).await
                };
                if let Some(event) = event {
                    log_stack_event(&event);
                }

                let result = {
                    let mut clusters = cluster_refs(temperature, humidity, power);
                    device.tick(0, &mut clusters).await
                };
                log_tick_result(&result);
            }
            Ok(None) => break,
            Err(error) => {
                log::warn!("parent poll failed: {error:?}");
                break;
            }
        }
    }
}

fn cluster_refs<'a>(
    temperature: &'a mut TemperatureCluster,
    humidity: &'a mut HumidityCluster,
    power: &'a mut PowerConfigCluster,
) -> [ClusterRef<'a>; 3] {
    [
        ClusterRef {
            endpoint: DEVICE_ENDPOINT,
            cluster: temperature,
        },
        ClusterRef {
            endpoint: DEVICE_ENDPOINT,
            cluster: humidity,
        },
        ClusterRef {
            endpoint: DEVICE_ENDPOINT,
            cluster: power,
        },
    ]
}

fn update_synthetic_readings(
    sequence: u32,
    temperature: &mut TemperatureCluster,
    humidity: &mut HumidityCluster,
    power: &mut PowerConfigCluster,
) {
    let temp = 2250 + (sequence % 20) as i16;
    let hum = 5000 + ((sequence * 7) % 100) as u16;
    temperature.set_temperature(temp);
    humidity.set_humidity(hum);
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

fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

fn marker(text: &[u8]) {
    for byte in text {
        hal::uart_write(*byte);
    }
}
