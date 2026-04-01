//! # Zigbee-RS nRF52840 Temperature Sensor
//!
//! Embassy-based Zigbee 3.0 end device for the Nordic nRF52840 that reports
//! real temperature data from the on-chip TEMP sensor, plus simulated humidity.
//!
//! ## Hardware
//! - nRF52840-DK (or any nRF52840 board)
//! - Button 1 (P0.11): press to join / leave the Zigbee network
//!
//! ## Operation
//! 1. Power on → device starts idle (not joined)
//! 2. Press Button 1 → initiates BDB commissioning (network join)
//! 3. Once joined: reads temperature every 30 s and ticks the stack
//! 4. Press Button 1 again → leaves the network
//!
//! To use an external SHTC3 sensor for both temperature *and* humidity,
//! connect SDA→P0.26, SCL→P0.27 and see the `shtc3` module below.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // HFCLK from external crystal — REQUIRED for 802.15.4 radio
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52840 sensor starting…");

    // On-chip temperature sensor (real hardware reading)
    let mut temp_sensor = Temp::new(p.TEMP, Irqs);

    // SAADC: measure VDD — default 12-bit, ref=0.6V, gain=1/6 → range 3.6V
    let mut saadc_sensor = Saadc::new(
        p.SAADC, Irqs, saadc::Config::default(),
        [ChannelConfig::single_ended(VddInput)],
    );
    saadc_sensor.calibrate().await;

    // Button 1 on nRF52840-DK (P0.11, active low with on-board pull-up)
    let mut button = gpio::Input::new(p.P0_11, gpio::Pull::Up);

    // IEEE 802.15.4 MAC driver
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mac = zigbee_mac::nrf::NrfMac::new(radio);

    info!("Radio ready");

    // ZCL cluster instances
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"nRF52840-Sensor",
        b"20260326",
        b"0.1.0",
    );
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);

    let mut power_cluster = PowerConfigCluster::new();
    power_cluster.set_battery_size(4);     // AAA
    power_cluster.set_battery_quantity(2); // 2× AAA
    power_cluster.set_battery_rated_voltage(15); // 1.5V per cell

    // Simulated humidity baseline (no on-chip humidity sensor)
    let mut hum_tick: u32 = 0;

    // Build the Zigbee device
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("nRF52840-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    info!("Device ready — press Button 1 to join/leave");

    loop {
        match select3(
            device.receive(),
            button.wait_for_falling_edge(),
            Timer::after(Duration::from_secs(REPORT_INTERVAL_SECS)),
        )
        .await
        {
            // ── Incoming MAC frame ──────────────────────────────
            Either3::First(Ok(indication)) => {
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                if let Some(event) = device.process_incoming(&indication, &mut clusters).await {
                    log_event(&event);
                }
            }
            Either3::First(Err(_)) => {
                warn!("MAC receive error");
            }

            // ── Button press → toggle join/leave ────────────────
            Either3::Second(_) => {
                if device.is_joined() {
                    info!("Button → leaving network…");
                } else {
                    info!("Button → joining network…");
                }
                device.user_action(UserAction::Toggle);
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
                    log_event(e);
                }
                // Debounce
                Timer::after(Duration::from_millis(300)).await;
            }

            // ── Report timer ────────────────────────────────────
            Either3::Third(_) => {
                if device.is_joined() {
                    // Read on-chip temperature (returns fixed-point °C)
                    let raw_temp = temp_sensor.read().await;
                    // Convert from fixed-point to hundredths of a degree:
                    // FixedI32<U2> has 2 fractional bits → multiply by 100
                    // and convert to integer for ZCL (hundredths of °C).
                    let temp_hundredths =
                        (raw_temp.to_bits() * 100 / 4) as i16;

                    // Simulated humidity (slowly varies 45–55 %)
                    hum_tick = hum_tick.wrapping_add(1);
                    let hum_hundredths =
                        5000u16 + ((hum_tick % 100) as u16).wrapping_mul(10);

                    temp_cluster.set_temperature(temp_hundredths);
                    hum_cluster.set_humidity(hum_hundredths);

                    // Battery: read VDD via SAADC (12-bit, ref=0.6V, gain=1/6 → 3.6V range)
                    let mut buf = [0i16; 1];
                    saadc_sensor.sample(&mut buf).await;
                    let raw = buf[0].max(0) as u32;
                    let voltage_mv = raw * 3600 / 4096;
                    let voltage_100mv = (voltage_mv / 100) as u8;
                    let pct = if voltage_mv <= 2200 { 0u8 }
                        else if voltage_mv >= 3300 { 200u8 }
                        else { ((voltage_mv - 2200) * 200 / 1100) as u8 };
                    power_cluster.set_battery_voltage(voltage_100mv);
                    power_cluster.set_battery_percentage(pct);

                    info!(
                        "T={}.{:02}°C  H={}.{:02}%  Bat={}.{}V ({}%)",
                        temp_hundredths / 100,
                        (temp_hundredths % 100).unsigned_abs(),
                        hum_hundredths / 100,
                        hum_hundredths % 100,
                        voltage_mv / 1000,
                        (voltage_mv % 1000) / 100,
                        pct / 2,
                    );
                }
                if let TickResult::Event(ref e) =
                    device.tick(REPORT_INTERVAL_SECS as u16, &mut [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                    ]).await
                {
                    log_event(e);
                }
            }
        }
    }
}

fn log_event(event: &StackEvent) {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            info!(
                "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id,
            );
        }
        StackEvent::Left => info!("Left network"),
        StackEvent::ReportSent => info!("Report sent"),
        StackEvent::CommissioningComplete { success } => {
            info!(
                "Commissioning: {}",
                if *success { "ok" } else { "failed" }
            );
        }
        _ => info!("Stack event"),
    }
}
