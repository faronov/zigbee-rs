//! # Zigbee-RS nRF52840 Sensor — UF2 bootloader variant
//!
//! For boards with the Adafruit UF2 bootloader pre-installed:
//! **nice!nano v2**, **ProMicro nRF52840**, **Adafruit Feather nRF52840**, etc.
//!
//! ## Flashing
//! 1. Build: `cargo build --release`
//! 2. Convert to UF2 (CI does this automatically):
//!    `uf2conv.py -c -f 0xADA52840 -b 0x26000 firmware.bin -o firmware.uf2`
//! 3. Double-tap RESET on the board → USB drive appears ("NICENANO")
//! 4. Copy the `.uf2` file to the drive — board flashes and reboots
//!
//! ## nice!nano v2 / ProMicro nRF52840 pinout
//!
//! | Function        | nRF GPIO | ProMicro silk | Notes                    |
//! |-----------------|----------|---------------|--------------------------|
//! | Button (join)   | P0.06   | D1/SDA row    | Wire a button to GND     |
//! | Blue LED        | P0.15   | —             | On-board, HIGH = ON      |
//! | Battery sense   | P0.04   | —             | ADC (AIN2)               |
//! | Power FET       | P0.13   | —             | HIGH = VCC off (sleep)   |
//! | I²C SDA         | P0.17   | D0            |                          |
//! | I²C SCL         | P0.20   | D1            |                          |
//! | Reset           | P0.18   | RST           | Double-tap → bootloader  |
//!
//! ## Differences from nrf52840-sensor (DK)
//! - Flash origin at 0x26000 (after UF2 bootloader)
//! - No debug probe needed — drag-and-drop `.uf2`
//! - Button on P0.06 (external, wire to GND) instead of DK's P0.11
//! - Blue LED on P0.15 shows network status

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Zigbee-RS nRF52840 sensor (UF2) starting…");

    let mut temp_sensor = Temp::new(p.TEMP, Irqs);

    // ── nice!nano / ProMicro pin assignments ───────────────────
    // Button: wire a tactile switch between P0.06 and GND.
    // LED: on-board blue LED on P0.15 (HIGH = ON).
    let mut button = gpio::Input::new(p.P0_06, gpio::Pull::Up);
    let mut led = gpio::Output::new(p.P0_15, gpio::Level::Low, gpio::OutputDrive::Standard);

    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mac = zigbee_mac::nrf::NrfMac::new(radio);

    info!("Radio ready");

    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut hum_tick: u32 = 0;

    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Zigbee-RS")
        .model("nRF52840-UF2-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    // LED off until joined
    led.set_low();
    info!("Device ready — press button (P0.06) to join/leave");

    loop {
        match select3(
            device.receive(),
            button.wait_for_falling_edge(),
            Timer::after(Duration::from_secs(REPORT_INTERVAL_SECS)),
        )
        .await
        {
            Either3::First(Ok(indication)) => {
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ];
                if let Some(event) = device.process_incoming(&indication, &mut clusters).await {
                    log_event(&event, &mut led);
                }
            }
            Either3::First(Err(_)) => {
                warn!("MAC receive error");
            }

            Either3::Second(_) => {
                if device.is_joined() {
                    info!("Button → leaving network…");
                } else {
                    info!("Button → joining network…");
                }
                device.user_action(UserAction::Toggle);
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
                    log_event(e, &mut led);
                }
                Timer::after(Duration::from_millis(300)).await;
            }

            Either3::Third(_) => {
                if device.is_joined() {
                    let raw_temp = temp_sensor.read().await;
                    let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;

                    hum_tick = hum_tick.wrapping_add(1);
                    let hum_hundredths =
                        5000u16 + ((hum_tick % 100) as u16).wrapping_mul(10);

                    temp_cluster.set_temperature(temp_hundredths);
                    hum_cluster.set_humidity(hum_hundredths);

                    info!(
                        "T={}.{:02}°C  H={}.{:02}%",
                        temp_hundredths / 100,
                        (temp_hundredths % 100).unsigned_abs(),
                        hum_hundredths / 100,
                        hum_hundredths % 100,
                    );
                }
                if let TickResult::Event(ref e) =
                    device.tick(REPORT_INTERVAL_SECS as u16, &mut [
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ]).await
                {
                    log_event(e, &mut led);
                }
            }
        }
    }
}

/// Log stack events and drive the on-board LED (P0.15):
/// LED ON = joined, LED OFF = not joined.
fn log_event(event: &StackEvent, led: &mut gpio::Output<'_>) {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            led.set_high(); // LED on → joined
            info!(
                "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id,
            );
        }
        StackEvent::Left => {
            led.set_low(); // LED off → left
            info!("Left network");
        }
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
