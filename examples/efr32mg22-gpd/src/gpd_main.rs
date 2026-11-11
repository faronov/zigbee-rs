//! Shared composition root and event loop of the BRD4183A Green Power
//! Devices.
//!
//! - Boot: clocks, VCOM log, RTCC time driver, radio; the GPD identity is
//!   loaded from the product journal or provisioned with a key from the
//!   CRYPTOACC TRNG (released again right after).
//! - Not commissioned: one commissioning attempt at boot (a network must
//!   permit joining, e.g. ZHA "Add device"), then wait for BTN0.
//! - BTN0 short press: sensor reports now / switch sends Toggle.
//! - BTN0 held for `LONG_PRESS_MS`: forget the channel and recommission.
//!   Key, SrcID and frame counter are kept, so the sink accepts the device
//!   again without a new identity.
//! - Sensor: Multi-Cluster Report every `SENSOR_REPORT_INTERVAL_S`.
//! - Between events: radio idle, VCOM flushed, EM2 until the RTCC deadline
//!   or a BTN0 edge.
//!
//! Measurements: temperature is the EFR32 die sensor (not ambient), the
//! humidity is SYNTHETIC; the BRD4183A has no humidity sensor.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32mg22_devkit::{Button0, Led0, resources::BoardResources};
use efr32mg22_gpd_product as product;
use efr32s2_hal::{power, rtcc, trng::Trng, vectors::Interrupt};
use embassy_time::{Duration, Instant, Timer};
use gpd_app::{EnvironmentReport, GpdDevice, GpdKind, GpdProfile, SwitchCommand};
use zigbee_mac::RadioPhy;
use zigbee_mac::efr32s2::Efr32s2RadioPhy;

#[allow(unused_imports)]
use efr32s2_hal::vectors::__INTERRUPTS;

type Device = GpdDevice<Efr32s2RadioPhy, product::GpdStore>;

/// Longest EM2 sleep of an idle device (RTCC ticks fit `u32`).
const IDLE_SLEEP_MS: u32 = 3_600_000;
const HELD_POLL_MS: u64 = 25;

static BUTTON_PENDING: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
extern "C" fn GPIO_EVEN() {
    if efr32mg22_devkit::service_button0_interrupt() {
        BUTTON_PENDING.store(true, Ordering::Release);
    }
}

fn button_pending() -> bool {
    BUTTON_PENDING.load(Ordering::Acquire)
}

fn take_button(button: &mut Button0) -> bool {
    // Both: the ISR flag and a pin flag the ISR has not serviced yet.
    BUTTON_PENDING.swap(false, Ordering::AcqRel) | button.take_interrupt()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Press {
    Short,
    Long,
}

pub async fn run(profile: GpdProfile) -> ! {
    let BoardResources {
        clocks,
        led0,
        button0,
        vcom,
        radio,
        rtcc,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());

    let clocks = efr32mg22_devkit::init_clocks(clocks);
    crate::vcom_log::init(vcom.into_vcom());
    if let Err(error) = clocks {
        log::error!("[GPD] HFXO start failed: {error:?}");
        halt()
    }
    let rtcc = rtcc.start().unwrap_or_else(|error| {
        log::error!("[GPD] RTCC start failed: {error:?}");
        halt()
    });
    efr32s2_hal::time_driver::init(rtcc);
    power::set_em2_debug(cfg!(feature = "em2-debug"));

    let mut led = led0.into_led();
    let mut button = button0.into_button();
    BUTTON_PENDING.store(false, Ordering::Release);
    cortex_m::peripheral::NVIC::unpend(Interrupt::GPIO_EVEN);
    // SAFETY: the board token exclusively owns PB0/EXTI0 and this crate
    // provides the sole GPIO_EVEN handler.
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::GPIO_EVEN) };
    blink(&mut led, 2, 100).await;

    log::info!(
        "[GPD] {:?} model 0x{:04X} on {} / {} ({})",
        profile.kind,
        profile.model_id,
        efr32mg22_devkit::BOARD_RADIO,
        efr32mg22_devkit::BOARD_MAIN,
        efr32mg22_devkit::MCU_PART
    );
    if profile.kind == GpdKind::Sensor {
        log::warn!("[GPD] die temperature, SYNTHETIC humidity, no battery report");
    }

    let phy: Efr32s2RadioPhy = Efr32s2RadioPhy::new(radio).unwrap_or_else(|error| {
        log::error!("[GPD] radio init failed: {error:?}");
        halt()
    });
    let eui64 = phy.extended_address();
    if eui64 == [0; 8] || eui64 == [0xFF; 8] {
        log::error!("[GPD] no valid EUI-64");
        halt()
    }
    let (mut device, provisioning) = Device::open(
        phy,
        product::gpd_store(flash),
        profile,
        eui64,
        |key: &mut [u8]| {
            let mut trng = Trng::new().inspect_err(|error| {
                log::error!("[GPD] TRNG start (conditioning/startup tests): {error:?}");
            })?;
            let filled = trng.fill(key);
            trng.release();
            match filled {
                // Key quality summary only; the key itself is never logged.
                Ok(()) => log::info!(
                    "[GPD] TRNG ok: {} key bytes, non-zero {}, {} distinct bytes, {} of {} bits set",
                    key.len(),
                    key.iter().any(|b| *b != 0),
                    distinct_bytes(key),
                    key.iter().map(|b| b.count_ones()).sum::<u32>(),
                    key.len() * 8
                ),
                Err(error) => log::error!("[GPD] TRNG fill: {error:?}"),
            }
            filled
        },
    )
    .unwrap_or_else(|error| {
        log::error!("[GPD] identity: {error:?}");
        halt()
    });
    let record = *device.record();
    log::info!(
        "[GPD] {provisioning:?} SrcID 0x{:08X} counter {} channel {:?}",
        record.src_id,
        device.next_frame_counter(),
        record.channel
    );

    if record.channel.is_none() {
        commission(&mut device, &mut led).await;
    }

    let interval = Duration::from_secs(product::SENSOR_REPORT_INTERVAL_S);
    let mut next_report = Instant::now();
    let mut sequence = 0u32;
    loop {
        if profile.kind == GpdKind::Sensor && Instant::now() >= next_report {
            report(&mut device, &mut led, &mut sequence).await;
            next_report = Instant::now() + interval;
        }

        let sleep_ms = if profile.kind == GpdKind::Sensor {
            next_report
                .saturating_duration_since(Instant::now())
                .as_millis()
                .min(u64::from(IDLE_SLEEP_MS)) as u32
        } else {
            IDLE_SLEEP_MS
        };
        if !sleep(&mut device, &mut button, sleep_ms) {
            continue;
        }

        match classify(&mut button).await {
            Press::Long => {
                log::info!("[GPD] long press: recommission");
                if let Err(error) = device.forget_channel() {
                    log::error!("[GPD] forget channel: {error:?}");
                    continue;
                }
                commission(&mut device, &mut led).await;
            }
            Press::Short => match profile.kind {
                GpdKind::Sensor => {
                    report(&mut device, &mut led, &mut sequence).await;
                    next_report = Instant::now() + interval;
                }
                GpdKind::Switch => {
                    let counter = device.next_frame_counter();
                    let sent = device.switch(SwitchCommand::Toggle).await;
                    log::info!("[GPD] Toggle fc {counter}: {sent:?}");
                    flash_result(&mut led, sent.is_ok()).await;
                }
            },
        }
        // Drop the release edge of the press just handled.
        let _ = take_button(&mut button);
    }
}

/// EM2 until `timeout_ms` passes or BTN0 is pressed; `true` on a press.
fn sleep(device: &mut Device, button: &mut Button0, timeout_ms: u32) -> bool {
    if take_button(button) {
        return true;
    }
    if let Err(error) = device.phy_mut().sleep() {
        log::error!("[GPD] radio sleep: {error:?}");
    }
    crate::vcom_log::flush();
    let slept = power::sleep_em2(rtcc::ms_to_ticks(timeout_ms.max(1)), button_pending);
    // Restore the radio before anything else uses it.
    if let Err(error) = device.phy_mut().wake() {
        log::error!("[GPD] radio wake: {error:?}");
        halt()
    }
    if let Err(error) = slept {
        log::error!("[GPD] EM2: {error:?}");
    }
    take_button(button)
}

async fn classify(button: &mut Button0) -> Press {
    let started = Instant::now();
    while button.is_pressed() {
        if started.elapsed().as_millis() >= product::LONG_PRESS_MS {
            return Press::Long;
        }
        Timer::after(Duration::from_millis(HELD_POLL_MS)).await;
    }
    Press::Short
}

async fn commission(device: &mut Device, led: &mut Led0) {
    log::info!("[GPD] commissioning: scanning for a network that permits joining");
    led.on();
    let result = device.commission().await;
    led.off();
    match result {
        Ok(network) => log::info!(
            "[GPD] commissioned on channel {} (LQI {}, EPID {:02X?})",
            network.channel,
            network.lqi,
            network.extended_pan_id
        ),
        Err(error) => log::warn!("[GPD] commissioning failed: {error:?}"),
    }
    flash_result(led, result.is_ok()).await;
}

async fn report(device: &mut Device, led: &mut Led0, sequence: &mut u32) {
    let report = EnvironmentReport {
        temperature_centi_celsius: power::die_temperature_centi_celsius(),
        // SYNTHETIC: 50.00 %..59.90 % sawtooth, no humidity sensor fitted.
        humidity_centi_percent: Some(5_000 + ((*sequence % 100) as u16) * 10),
        battery_half_percent: None,
    };
    *sequence = sequence.wrapping_add(1);
    let counter = device.next_frame_counter();
    let sent = device.report_environment(report).await;
    log::info!(
        "[GPD] report fc {counter} {} c°C {:?} c% : {sent:?}",
        report.temperature_centi_celsius,
        report.humidity_centi_percent
    );
    flash_result(led, sent.is_ok()).await;
}

async fn flash_result(led: &mut Led0, ok: bool) {
    if ok {
        blink(led, 1, 30).await;
    } else {
        blink(led, 3, 80).await;
    }
}

async fn blink(led: &mut Led0, times: u8, ms: u64) {
    for _ in 0..times {
        led.on();
        Timer::after(Duration::from_millis(ms)).await;
        led.off();
        Timer::after(Duration::from_millis(ms)).await;
    }
}

/// Number of different byte values in `bytes` (TRNG sanity summary).
fn distinct_bytes(bytes: &[u8]) -> usize {
    let mut seen = [false; 256];
    bytes
        .iter()
        .filter(|b| !core::mem::replace(&mut seen[usize::from(**b)], true))
        .count()
}

#[inline(never)]
pub fn halt() -> ! {
    efr32mg22_devkit::emergency_led_on();
    loop {
        cortex_m::asm::wfi();
    }
}
