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
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_nrf::temp::Temp;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Timer};

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
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

const REPORT_INTERVAL_SECS: u64 = 30;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
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

    // Boot signal: LED solid ON 3 seconds
    led_on(&mut led);
    Timer::after(Duration::from_secs(3)).await;
    led_off(&mut led);
    Timer::after(Duration::from_millis(500)).await;

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

    // ── Boot signal: LED solid ON 1 second (all boards) ──
    led_on(&mut led);
    Timer::after(Duration::from_secs(1)).await;
    led_off(&mut led);

    // ── Join strategy: auto-join on all boards ──
    info!("Auto-joining network…");
    device.user_action(UserAction::Join);
    let mut clusters = [
        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
    ];
    if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
        log_event(e, &mut led);
    }

    // ── Main loop ──
    let mut secs_since_report: u64 = 0;

    loop {
        let interval = if device.is_joined() {
            REPORT_INTERVAL_SECS
        } else {
            1
        };

        // Build a future that waits for button press (or never completes if no button)
        let button_fut = async {
            if let Some(ref mut btn) = button {
                btn.wait_for_falling_edge().await;
            } else {
                core::future::pending::<()>().await;
            }
        };

        match select3(
            // For RX-off end devices: alternate between poll and passive listen.
            // Poll retrieves indirect frames from parent; passive catches broadcasts.
            async {
                if device.is_joined() {
                    // Poll parent first — most frames arrive via indirect delivery
                    match device.poll().await {
                        Ok(Some(ind)) => return Ok(ind),
                        _ => {}
                    }
                }
                // Fall through to passive listen (also handles pre-join traffic)
                device.receive().await
            },
            button_fut,
            Timer::after(Duration::from_secs(interval)),
        )
        .await
        {
            // ── Incoming MAC frame ──
            Either3::First(Ok(indication)) => {
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ];
                if let Some(event) = device.process_incoming(&indication, &mut clusters).await {
                    log_event(&event, &mut led);
                }
                if let TickResult::Event(ref e) = device.tick(0, &mut [
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ]).await {
                    log_event(e, &mut led);
                }
            }
            Either3::First(Err(_)) => {
                // MAC receive error/timeout — normal, just loop
            }

            // ── Button press ──
            Either3::Second(_) => {
                // Detect long press: wait for release or 3s timeout
                let held_long = if let Some(ref mut btn) = button {
                    match select(
                        btn.wait_for_rising_edge(),
                        Timer::after(Duration::from_secs(3)),
                    ).await {
                        Either::Second(_) => true,  // 3s elapsed = long press
                        Either::First(_) => false,  // released early = short press
                    }
                } else {
                    false
                };

                if held_long {
                    info!("FACTORY RESET");
                    for _ in 0..5 {
                        led_on(&mut led);
                        Timer::after(Duration::from_millis(100)).await;
                        led_off(&mut led);
                        Timer::after(Duration::from_millis(100)).await;
                    }
                    cortex_m::peripheral::SCB::sys_reset();
                } else {
                    if device.is_joined() {
                        info!("Button → leave");
                    } else {
                        info!("Button → join");
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
            }

            // ── Timer tick ──
            Either3::Third(_) => {
                secs_since_report += interval;

                if device.is_joined() {
                    if secs_since_report >= REPORT_INTERVAL_SECS {
                        secs_since_report = 0;
                        let raw_temp = temp_sensor.read().await;
                        let temp_hundredths = (raw_temp.to_bits() * 100 / 4) as i16;

                        hum_tick = hum_tick.wrapping_add(1);
                        let hum_hundredths = 5000u16 + ((hum_tick % 100) as u16).wrapping_mul(10);

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
                } else {
                    // Double-blink while not joined
                    led_on(&mut led);
                    Timer::after(Duration::from_millis(80)).await;
                    led_off(&mut led);
                    Timer::after(Duration::from_millis(120)).await;
                    led_on(&mut led);
                    Timer::after(Duration::from_millis(80)).await;
                    led_off(&mut led);

                    // Auto-retry join (only for non-button boards)
                    if button.is_none() && secs_since_report >= 10 {
                        secs_since_report = 0;
                        info!("Not joined — retrying…");
                        device.user_action(UserAction::Join);
                    }
                }

                if let TickResult::Event(ref e) = device.tick(interval as u16, &mut [
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                ]).await {
                    log_event(e, &mut led);
                }
            }
        }
    }
}

/// LED ON = joined, LED OFF = not joined.
fn log_event(event: &StackEvent, led: &mut gpio::Output<'_>) {
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
        }
        StackEvent::Left => {
            led_off(led);
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
