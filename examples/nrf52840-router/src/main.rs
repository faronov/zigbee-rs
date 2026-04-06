//! # nRF52840 Zigbee Router
//!
//! A Zigbee 3.0 router device for nRF52840-DK.
//! Routes frames between end devices and the coordinator,
//! extends network range, and accepts child joins.
//!
//! # Features
//! - Joins existing network as a router (FFD)
//! - Continuous RX (rx_on_when_idle = true)
//! - Relays unicast and broadcast frames
//! - Accepts child end device joins (permit joining)
//! - Periodic Link Status broadcasts (every 15s)
//! - Indirect frame buffering for sleeping children
//! - NWK Leave handler with auto-rejoin
//! - Button: short=toggle join/leave, long=factory reset
//!
//! # Build & Flash
//! ```bash
//! cd examples/nrf52840-router
//! cargo build --release
//! probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-router
//! ```

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio;
use embassy_nrf::radio;
use embassy_nrf::{bind_interrupts, peripherals};
use embassy_time::{Duration, Instant, Timer};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;

// Bridge `log` crate → defmt
struct DefmtLogger;
impl log::Log for DefmtLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool { true }
    fn log(&self, record: &log::Record) {
        defmt::info!("{}", defmt::Display2Format(record.args()));
    }
    fn flush(&self) {}
}
static LOGGER: DefmtLogger = DefmtLogger;

const RX_CHECK_MS: u64 = 50;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _ = log::set_logger(&LOGGER);
    log::set_max_level(log::LevelFilter::Info);

    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    // DC-DC enabled for lower power
    config.dcdc.reg0 = true;
    config.dcdc.reg1 = true;
    let p = embassy_nrf::init(config);

    info!("Zigbee-RS nRF52840 ROUTER starting…");

    // LED1 (P0.13, active LOW) — solid ON = joined, blink = joining
    let mut led = gpio::Output::new(p.P0_13, gpio::Level::High, gpio::OutputDrive::Standard);
    // LED2 (P0.14) — blink on frame relay
    let mut led_relay = gpio::Output::new(p.P0_14, gpio::Level::High, gpio::OutputDrive::Standard);
    // Button 1 (P0.11, active low) — polled for edge detection
    let button = gpio::Input::new(p.P0_11, gpio::Pull::Up);

    // Radio
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio);
    mac.set_tx_power(0);
    info!("Radio ready (TX 0 dBm)");

    // ZCL clusters — router only needs Basic + Identify
    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"nRF52840-Router",
        b"20260405",
        b"0.1.0",
    );
    basic_cluster.set_power_source(0x01); // Mains powered
    let mut identify_cluster = IdentifyCluster::new();

    // Build device as ROUTER — rx_on_when_idle, no sleep
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::Router)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer("Zigbee-RS")
        .model("nRF52840-Router")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0007, |ep| { // 0x0007 = Home Gateway
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
        })
        .build();

    // Auto-join network
    info!("Joining network as router…");
    device.user_action(UserAction::Join);
    let mut clusters = [
        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
        ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
    ];
    if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
        log_event(e, &mut led);
    }

    // Main loop — continuous RX, no sleep
    let mut last_tick = Instant::now();
    let mut last_rejoin = Instant::now();
    let mut rejoin_count: u8 = 0;
    let mut button_was_pressed = false;

    loop {
        // Button check (polled)
        let pressed = button.is_low();
        if pressed && !button_was_pressed {
            let press_start = Instant::now();
            while button.is_low() {
                if press_start.elapsed().as_secs() >= 3 {
                    break;
                }
                Timer::after(Duration::from_millis(50)).await;
            }
            let held_long = press_start.elapsed().as_secs() >= 3;

            if held_long {
                info!("FACTORY RESET");
                for _ in 0..5u8 {
                    led.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                }
                cortex_m::peripheral::SCB::sys_reset();
            } else {
                info!("Button → {}", if device.is_joined() { "leave" } else { "join" });
                device.user_action(UserAction::Toggle);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                if let TickResult::Event(ref e) = device.tick(0, &mut cls).await {
                    log_event(e, &mut led);
                }
            }
        }
        button_was_pressed = pressed;

        // Brief sleep between RX checks
        Timer::after(Duration::from_millis(RX_CHECK_MS)).await;

        if device.is_joined() {
            // Continuous RX — check for incoming frames
            match device.receive().await {
                Ok(ind) => {
                    let mut cls = [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                    ];
                    if let Some(ev) = device.process_incoming(&ind, &mut cls).await {
                        match &ev {
                            StackEvent::LeaveRequested => {
                                info!("Leave requested — rejoining");
                                device.user_action(UserAction::Join);
                            }
                            _ => { log_event(&ev, &mut led); }
                        }
                    }
                    // Brief LED2 blink to show relay activity
                    led_relay.set_low();
                    Timer::after(Duration::from_millis(10)).await;
                    led_relay.set_high();

                    // Tick to send queued responses
                    let mut cls2 = [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                    ];
                    let _ = device.tick(0, &mut cls2).await;
                }
                Err(_) => {} // RX error or timeout
            }

            // Periodic tick (every 1s) for router maintenance
            let now = Instant::now();
            if now.duration_since(last_tick).as_secs() >= 1 {
                let elapsed = now.duration_since(last_tick).as_secs().min(60) as u16;
                last_tick = now;
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                let _ = device.tick(elapsed, &mut cls).await;

                // Identify LED
                identify_cluster.tick(elapsed);
                if identify_cluster.is_identifying() {
                    led.toggle();
                }
            }
        } else {
            // Not joined — blink and auto-retry
            let now = Instant::now();
            if now.duration_since(last_rejoin).as_secs() >= 1 {
                led.set_low();
                Timer::after(Duration::from_millis(80)).await;
                led.set_high();
            }
            if now.duration_since(last_rejoin).as_secs() >= 15 {
                rejoin_count = rejoin_count.wrapping_add(1);
                last_rejoin = Instant::now();
                info!("Retrying join (attempt {})…", rejoin_count);
                device.user_action(UserAction::Join);
                let mut cls = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                ];
                let _ = device.tick(0, &mut cls).await;
                if device.is_joined() {
                    info!("Joined as router! addr=0x{:04X}", device.short_address());
                    led.set_low(); // LED ON
                }
            }
        }
    }
}

fn log_event(event: &StackEvent, led: &mut gpio::Output<'_>) {
    match event {
        StackEvent::Joined { short_address, channel, pan_id } => {
            led.set_low(); // ON
            info!("ROUTER joined! addr=0x{:04X} ch={} pan=0x{:04X}", short_address, channel, pan_id);
        }
        StackEvent::Left => {
            led.set_high(); // OFF
            info!("Left network");
        }
        StackEvent::LeaveRequested => {
            info!("Leave requested by coordinator");
        }
        _ => {}
    }
}
