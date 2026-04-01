//! # Zigbee-RS nRF52840 Sensor — UF2 bootloader variant
//!
//! Supports multiple boards via cargo features:
//!
//! | Feature            | Board                          | LED          | Flash   |
//! |--------------------|--------------------------------|--------------|---------|
//! | `board-promicro`   | ProMicro nRF52840 / nice!nano  | P0.15 HIGH   | 0x26000 |
//! | `board-mdk`        | Makerdiary MDK USB Dongle      | P0.22 LOW    | 0x1000  |
//! | `board-nrf-dongle` | Nordic PCA10059 Dongle         | P0.06 LOW    | 0x1000  |
//! | `board-nrf-dk`     | Nordic nRF52840 DK (PCA10056)  | P0.13 LOW    | 0x0000  |
//!
//! ## Build & flash
//! ```sh
//! # ProMicro (default):
//! cargo build --release
//! uf2conv.py -c -f 0xADA52840 -b 0x26000 firmware.bin -o fw.uf2
//!
//! # MDK dongle:
//! cargo build --release --no-default-features --features board-mdk
//! uf2conv.py -c -f 0xADA52840 -b 0x1000 firmware.bin -o fw.uf2
//!
//! # Nordic dongle:
//! cargo build --release --no-default-features --features board-nrf-dongle
//! uf2conv.py -c -f 0xADA52840 -b 0x1000 firmware.bin -o fw.uf2
//! ```
//!
//! ## Operation
//! - **Auto-join**: starts commissioning automatically on boot
//! - LED ON = joined, LED blinks = joining, LED OFF = idle

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_nrf::saadc::{self, ChannelConfig, Saadc, VddInput};
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Instant, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

// Bridge `log` crate → defmt so stack-internal log::info!/debug!/warn!/error! appear in RTT output.
struct DefmtLogger;
impl log::Log for DefmtLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool { true }
    fn log(&self, record: &log::Record) {
        match record.level() {
            log::Level::Error => defmt::error!("{}", defmt::Display2Format(record.args())),
            log::Level::Warn  => defmt::warn!("{}", defmt::Display2Format(record.args())),
            log::Level::Info  => defmt::info!("{}", defmt::Display2Format(record.args())),
            log::Level::Debug => defmt::debug!("{}", defmt::Display2Format(record.args())),
            log::Level::Trace => defmt::trace!("{}", defmt::Display2Format(record.args())),
        }
    }
    fn flush(&self) {}
}
static LOGGER: DefmtLogger = DefmtLogger;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 15;
const FAST_POLL_MS: u64 = 250; // Fast poll during interview (250ms)
const SLOW_POLL_SECS: u64 = 10; // Normal poll interval (10s)
const FAST_POLL_DURATION_SECS: u64 = 120; // Max fast-poll window (safety timeout)
const EXPECTED_REPORT_CLUSTERS: usize = 3; // PowerConfig + Temp + Humidity

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

/// Disable SoftDevice S140 via SVC call (ProMicro / nice!nano only).
/// The UF2 bootloader leaves SoftDevice installed; we must disable it
/// to reclaim RTC1, TIMER0, and RADIO for our Zigbee stack.
#[cfg(feature = "board-promicro")]
unsafe fn disable_softdevice() {
    let result: u32;
    // sd_softdevice_disable() = SVC #17 (0x11)
    core::arch::asm!(
        "svc 17",
        lateout("r0") result,
        options(nomem, nostack, preserves_flags)
    );
    if result == 0 {
        info!("SoftDevice disabled OK");
    } else {
        info!("SoftDevice disable returned {}", result);
    }
}

/// Create the on-board status LED output.
/// - ProMicro: P0.15 red, active HIGH → start LOW (off)
/// - MDK dongle: P0.22 green, active LOW → start HIGH (off)
/// - Nordic PCA10059: P0.06 green, active LOW → start HIGH (off)
/// - Nordic DK: P0.13 (LED1), active LOW → start HIGH (off)
fn create_led(p: &mut embassy_nrf::Peripherals) -> gpio::Output<'static> {
    #[cfg(feature = "board-promicro")]
    {
        gpio::Output::new(
            unsafe { core::ptr::read(&p.P0_15 as *const _) },
            gpio::Level::Low,
            gpio::OutputDrive::Standard,
        )
    }
    #[cfg(feature = "board-mdk")]
    {
        gpio::Output::new(
            unsafe { core::ptr::read(&p.P0_22 as *const _) },
            gpio::Level::High, // active LOW — HIGH = off
            gpio::OutputDrive::Standard,
        )
    }
    #[cfg(feature = "board-nrf-dongle")]
    {
        gpio::Output::new(
            unsafe { core::ptr::read(&p.P0_06 as *const _) },
            gpio::Level::High, // active LOW — HIGH = off
            gpio::OutputDrive::Standard,
        )
    }
    #[cfg(feature = "board-nrf-dk")]
    {
        gpio::Output::new(
            unsafe { core::ptr::read(&p.P0_13 as *const _) },
            gpio::Level::High, // active LOW — HIGH = off
            gpio::OutputDrive::Standard,
        )
    }
}

/// LED ON (board-agnostic)
fn led_on(led: &mut gpio::Output<'_>) {
    #[cfg(feature = "board-promicro")]
    led.set_high(); // active HIGH
    #[cfg(any(feature = "board-mdk", feature = "board-nrf-dongle", feature = "board-nrf-dk"))]
    led.set_low(); // active LOW
}

/// LED OFF (board-agnostic)
fn led_off(led: &mut gpio::Output<'_>) {
    #[cfg(feature = "board-promicro")]
    led.set_low();
    #[cfg(any(feature = "board-mdk", feature = "board-nrf-dongle", feature = "board-nrf-dk"))]
    led.set_high();
}

/// Create button input (DK only — Button1 P0.11, active LOW with pull-up).
/// Other boards: returns None.
#[cfg(feature = "board-nrf-dk")]
fn create_button(p: &mut embassy_nrf::Peripherals) -> Option<gpio::Input<'static>> {
    Some(gpio::Input::new(
        unsafe { core::ptr::read(&p.P0_11 as *const _) },
        gpio::Pull::Up,
    ))
}

#[cfg(not(feature = "board-nrf-dk"))]
fn create_button(_p: &mut embassy_nrf::Peripherals) -> Option<gpio::Input<'static>> {
    None
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // ProMicro: must disable SoftDevice before embassy init (SD owns RTC1)
    #[cfg(feature = "board-promicro")]
    unsafe { disable_softdevice() };

    // Start HFCLK from external crystal via embassy config — REQUIRED for 802.15.4 radio.
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let mut p = embassy_nrf::init(config);

    // Initialize log→defmt bridge so stack crate log::info!/debug! appear in RTT
    log::set_logger(&LOGGER).ok();
    log::set_max_level(log::LevelFilter::Info);

    info!("Zigbee-RS nRF52840 sensor starting…");

    let mut led = create_led(&mut p);
    let mut button = create_button(&mut p);
    let mut temp_sensor = Temp::new(p.TEMP, Irqs);

    // SAADC for battery voltage (VDD via internal divider)
    let saadc_config = saadc::Config::default();
    let channel_config = ChannelConfig::single_ended(VddInput);
    let mut saadc_inst = Saadc::new(p.SAADC, Irqs, saadc_config, [channel_config]);
    saadc_inst.calibrate().await;

    // Boot signal: LED solid ON 3 seconds
    led_on(&mut led);
    Timer::after(Duration::from_secs(3)).await;
    led_off(&mut led);
    Timer::after(Duration::from_millis(500)).await;

    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio);
    mac.set_tx_power(8); // Max TX power (+8 dBm) for better coordinator reach

    info!("Radio ready");

    let mut basic_cluster = BasicCluster::new(
        b"Zigbee-RS",
        b"nRF52840-Sensor",
        b"20260328",
        b"0.1.0",
    );
    basic_cluster.set_power_source(0x03); // Battery
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut power_cluster = PowerConfigCluster::new();
    power_cluster.set_battery_size(4);     // AAA
    power_cluster.set_battery_quantity(2); // 2× AAA
    power_cluster.set_battery_rated_voltage(15); // 1.5V per cell
    let mut hum_tick: u32 = 0;

    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("nRF52840-UF2-Sensor")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
        })
        .build();

    // ── Boot signal: LED solid ON 1 second (all boards) ──
    led_on(&mut led);
    Timer::after(Duration::from_secs(1)).await;
    led_off(&mut led);

    // ── Join strategy: auto-join on all boards ──
    info!("Auto-joining network…");
    device.user_action(UserAction::Join);
    let mut clusters = [
        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
    ];
    if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
        if log_event(e, &mut led) {
            // Join happened during init tick
        }
    }

    // ── Read sensors once so clusters have real values for ZHA interview ──
    {
        let raw_temp = temp_sensor.read().await;
        let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;
        let hum_hundredths = 5000u16; // initial humidity placeholder
        temp_cluster.set_temperature(temp_hundredths);
        hum_cluster.set_humidity(hum_hundredths);

        // Battery voltage via SAADC (VDD, 12-bit, internal ref 0.6V, gain 1/6 → 3.6V range)
        let mut buf = [0i16; 1];
        saadc_inst.sample(&mut buf).await;
        let raw = buf[0].max(0) as u32;
        let voltage_mv = raw * 3600 / 4096;
        let pct = if voltage_mv >= 3000 { 100u8 }
                  else if voltage_mv <= 1800 { 0 }
                  else { ((voltage_mv - 1800) * 100 / 1200) as u8 };
        power_cluster.set_battery_voltage((voltage_mv / 100) as u8);
        power_cluster.set_battery_percentage(pct * 2); // ZCL: 0.5% units

        info!(
            "Initial: T={}.{:02}°C  H={}.{:02}%  Bat={}mV ({}%)",
            temp_hundredths / 100,
            (temp_hundredths % 100).unsigned_abs(),
            hum_hundredths / 100,
            hum_hundredths % 100,
            voltage_mv,
            pct,
        );
    }

    // ── Main loop ──
    // SED architecture: sleep → poll parent → process indirect frames → periodic tasks.
    // ALL frames reach a Sleepy End Device through the parent's indirect queue,
    // so we poll() instead of receive(). This avoids the rapid Timer
    // creation/teardown that caused RefCell panics in embassy's timer driver.
    let mut last_report = Instant::now();
    // If already joined (from BDB steering above), start fast-poll immediately
    // so ZHA's ZCL attribute reads during interview don't time out.
    let mut fast_poll_until = if device.is_joined() {
        info!("Fast poll ON ({}s) — post-join", FAST_POLL_DURATION_SECS);
        Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS)
    } else {
        Instant::now() // expires immediately
    };
    let mut last_rejoin_attempt = Instant::now();
    let mut rejoin_count: u8 = 0;
    // Device_annce retry: re-send a few times after join so coordinator discovers us
    let mut annce_retries_left: u8 = if device.is_joined() { 5 } else { 0 };
    let mut last_annce = Instant::now();

    let mut was_fast_polling = device.is_joined();
    let mut interview_done = false;

    loop {
        let now = Instant::now();
        let in_fast_poll = now < fast_poll_until;
        let poll_ms = if in_fast_poll { FAST_POLL_MS } else { SLOW_POLL_SECS * 1000 };

        // Log transition from fast→slow poll
        if was_fast_polling && !in_fast_poll {
            let cfg = device.configured_cluster_count(1);
            info!("Fast poll OFF — {}/{} clusters configured", cfg, EXPECTED_REPORT_CLUSTERS);
            was_fast_polling = false;
            // Always turn LED off when fast-poll expires (power save fallback)
            if !interview_done {
                info!("LED OFF (fast-poll expired, interview incomplete)");
                led_off(&mut led);
            }
        } else if in_fast_poll {
            was_fast_polling = true;
        }

        // ── Step 1: Sleep until next poll ──
        // Single Timer per iteration — avoids rapid create/drop that panics embassy.
        // During sleep, check button via select (only 2 futures, not 3).
        if let Some(ref mut btn) = button {
            match select(
                btn.wait_for_falling_edge(),
                Timer::after(Duration::from_millis(poll_ms)),
            )
            .await
            {
                Either::First(_) => {
                    // Button pressed — check for long press
                    let held_long = match select(
                        btn.wait_for_rising_edge(),
                        Timer::after(Duration::from_secs(3)),
                    )
                    .await
                    {
                        Either::Second(_) => true,
                        Either::First(_) => false,
                    };

                    if held_long {
                        info!("FACTORY RESET");
                        for _ in 0..5u8 {
                            led_on(&mut led);
                            Timer::after(Duration::from_millis(100)).await;
                            led_off(&mut led);
                            Timer::after(Duration::from_millis(100)).await;
                        }
                        cortex_m::peripheral::SCB::sys_reset();
                    } else {
                        info!("Button → {}", if device.is_joined() { "leave" } else { "join" });
                        device.user_action(UserAction::Toggle);
                        let _ = device.tick(0, &mut [
                            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                        ]).await;
                    }
                }
                Either::Second(_) => {} // Normal timeout — proceed to poll
            }
        } else {
            // No button — just sleep
            Timer::after(Duration::from_millis(poll_ms)).await;
        }

        // ── Step 2: Poll parent for indirect frames (SED core) ──
        if device.is_joined() {
            for poll_round in 0..4u8 {
                match device.poll().await {
                    Ok(Some(ind)) => {
                        let mut cls = [
                            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                        ];
                        if let Some(ev) = device.process_incoming(&ind, &mut cls).await {
                            if log_event(&ev, &mut led) {
                                fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                                info!("Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
                            }
                        }
                        // Check if ZHA completed Configure Reporting for all clusters
                        // (last step of the interview per-cluster)
                        if !interview_done {
                            let cfg_count = device.configured_cluster_count(1);
                            if cfg_count >= EXPECTED_REPORT_CLUSTERS {
                                info!("Interview done! {}/{} clusters configured — ending fast poll",
                                      cfg_count, EXPECTED_REPORT_CLUSTERS);
                                // Give 5s grace for any remaining ZHA requests
                                fast_poll_until = Instant::now() + Duration::from_secs(5);
                                interview_done = true;
                                // Turn off LED to save power (device is fully configured)
                                led_off(&mut led);
                            }
                        }
                        // Immediately tick to send any queued ZCL responses
                        let _ = device.tick(0, &mut [
                            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                        ]).await;
                    }
                    Ok(None) => break,
                    Err(_) => break,
                }
            }

            // ── Step 3: Periodic tasks ──
            let now2 = Instant::now();
            let elapsed_s = now2.duration_since(last_report).as_secs();

            // Read sensors when report interval elapsed
            if elapsed_s >= REPORT_INTERVAL_SECS {
                last_report = now2;
                let raw_temp = temp_sensor.read().await;
                let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;

                hum_tick = hum_tick.wrapping_add(1);
                let hum_hundredths = 5000u16 + ((hum_tick % 100) as u16).wrapping_mul(10);

                temp_cluster.set_temperature(temp_hundredths);
                hum_cluster.set_humidity(hum_hundredths);

                // Battery voltage via SAADC
                let mut buf = [0i16; 1];
                saadc_inst.sample(&mut buf).await;
                let raw = buf[0].max(0) as u32;
                let voltage_mv = raw * 3600 / 4096;
                let pct = if voltage_mv >= 3000 { 100u8 }
                          else if voltage_mv <= 1800 { 0 }
                          else { ((voltage_mv - 1800) * 100 / 1200) as u8 };
                power_cluster.set_battery_voltage((voltage_mv / 100) as u8);
                power_cluster.set_battery_percentage(pct * 2);

                info!(
                    "T={}.{:02}°C  H={}.{:02}%  Bat={}mV ({}%)",
                    temp_hundredths / 100,
                    (temp_hundredths % 100).unsigned_abs(),
                    hum_hundredths / 100,
                    hum_hundredths % 100,
                    voltage_mv,
                    pct,
                );
            }

            // Tick the runtime (sends queued responses, reports, etc.)
            let tick_elapsed = elapsed_s.min(60) as u16;
            if let TickResult::Event(ref e) = device.tick(tick_elapsed, &mut [
                ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ClusterRef { endpoint: 1, cluster: &mut power_cluster },
            ]).await {
                if log_event(e, &mut led) {
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    info!("Fast poll ON ({}s)", FAST_POLL_DURATION_SECS);
                }
            }

            // ── Step 4: Device_annce retry (coordinator may have missed first one) ──
            if annce_retries_left > 0 && now2.duration_since(last_annce).as_secs() >= 8 {
                annce_retries_left -= 1;
                last_annce = now2;
                info!("Re-sending Device_annce ({} left)", annce_retries_left);
                match device.send_device_annce().await {
                    Ok(()) => {},
                    Err(_) => info!("Device_annce retry failed"),
                }
            }
        } else {
            // ── Not joined — blink and auto-retry ──
            let now2 = Instant::now();
            if now2.duration_since(last_rejoin_attempt).as_secs() >= 1 {
                led_on(&mut led);
                Timer::after(Duration::from_millis(80)).await;
                led_off(&mut led);
                Timer::after(Duration::from_millis(120)).await;
                led_on(&mut led);
                Timer::after(Duration::from_millis(80)).await;
                led_off(&mut led);
            }

            if now2.duration_since(last_rejoin_attempt).as_secs() >= 15 {
                rejoin_count = rejoin_count.wrapping_add(1);
                last_rejoin_attempt = Instant::now();
                info!("Not joined — retrying (attempt {})…", rejoin_count);
                device.user_action(UserAction::Join);
                let _ = device.tick(0, &mut [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                ]).await;
                // If join succeeded during tick, activate fast-poll for interview
                if device.is_joined() {
                    info!("Joined! addr=0x{:04X}", device.short_address());
                    fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    info!("Fast poll ON ({}s) — post-rejoin", FAST_POLL_DURATION_SECS);
                    annce_retries_left = 5;
                    last_annce = Instant::now();
                    interview_done = false;
                    led_on(&mut led);
                }
            }
        }
    }
}

/// LED ON = joined, LED OFF = not joined. Returns true if this is a join event.
fn log_event(event: &StackEvent, led: &mut gpio::Output<'_>) -> bool {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            led_on(led);
            info!(
                "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id,
            );
            true
        }
        StackEvent::Left => {
            led_off(led);
            info!("Left network");
            false
        }
        StackEvent::ReportSent => { info!("Report sent"); false }
        StackEvent::CommissioningComplete { success } => {
            info!(
                "Commissioning: {}",
                if *success { "ok" } else { "failed" }
            );
            false
        }
        _ => { info!("Stack event"); false }
    }
}
