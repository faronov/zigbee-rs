//! Wait-loop summary for non-`quiet` thermometer builds.
//!
//! Logging every wait would itself veto sleep (the console must be idle
//! before the PMU may stop the clocks), so this wrapper only counts the
//! application's wait requests and outcomes and prints one line per summary
//! period. `quiet` builds use the product wake controller directly.

use embassy_time::Instant;
use sensor_sed_app::{SleepDepth, WaitRequest, WakeController, WakeReason};
use zigbee_mac::MacDriver;

use crate::thermometer_profile::config;

#[derive(Default, Clone, Copy)]
struct Counts {
    waits: u32,
    active: u32,
    idle: u32,
    timer: u32,
    activity: u32,
    button: u32,
    errors: u32,
    /// Wall-clock milliseconds spent inside `wait`.
    waited_ms: u32,
    /// Waits that returned before half of the requested timeout.
    short: u32,
    min_timeout_ms: u32,
    max_timeout_ms: u32,
}

pub struct DiagWake<W> {
    inner: W,
    counts: Counts,
    since: Instant,
}

impl<W> DiagWake<W> {
    pub fn new(inner: W) -> Self {
        Self {
            inner,
            counts: Counts {
                min_timeout_ms: u32::MAX,
                ..Counts::default()
            },
            since: Instant::now(),
        }
    }

    fn maybe_report(&mut self) {
        let period = u64::from(config::DIAG_SUMMARY_MS);
        if period == 0 || self.since.elapsed().as_millis() < period {
            return;
        }
        let c = self.counts;
        log::info!(
            "[H2-T wait] {}ms: waits={} active={} idle={} timer={} activity={} button={} err={} short={} in_wait={}ms timeout={}..{}ms",
            self.since.elapsed().as_millis(),
            c.waits,
            c.active,
            c.idle,
            c.timer,
            c.activity,
            c.button,
            c.errors,
            c.short,
            c.waited_ms,
            if c.waits == 0 { 0 } else { c.min_timeout_ms },
            c.max_timeout_ms,
        );
        self.counts = Counts {
            min_timeout_ms: u32::MAX,
            ..Counts::default()
        };
        self.since = Instant::now();
    }
}

impl<M: MacDriver, W: WakeController<M>> WakeController<M> for DiagWake<W> {
    type Mark = W::Mark;
    type Error = W::Error;

    fn is_recoverable(error: &Self::Error) -> bool {
        W::is_recoverable(error)
    }

    fn error_code(error: &Self::Error) -> u32 {
        W::error_code(error)
    }

    fn mark(&self) -> Self::Mark {
        self.inner.mark()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        W::add_ms(mark, duration_ms)
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        W::elapsed_ms(later, earlier)
    }

    async fn wait(&mut self, mac: &mut M, request: WaitRequest) -> Result<WakeReason, Self::Error> {
        let start = Instant::now();
        let result = self.inner.wait(mac, request).await;
        let spent = start.elapsed().as_millis().min(u64::from(u32::MAX)) as u32;
        let c = &mut self.counts;
        c.waits = c.waits.saturating_add(1);
        match request.sleep_depth {
            SleepDepth::Active => c.active = c.active.saturating_add(1),
            _ => c.idle = c.idle.saturating_add(1),
        }
        match result {
            Ok(WakeReason::Timer) => c.timer = c.timer.saturating_add(1),
            Ok(WakeReason::Activity) => c.activity = c.activity.saturating_add(1),
            Ok(WakeReason::Button) => c.button = c.button.saturating_add(1),
            Err(_) => c.errors = c.errors.saturating_add(1),
        }
        c.waited_ms = c.waited_ms.saturating_add(spent);
        if u64::from(spent) * 2 < u64::from(request.timeout_ms) {
            c.short = c.short.saturating_add(1);
        }
        c.min_timeout_ms = c.min_timeout_ms.min(request.timeout_ms);
        c.max_timeout_ms = c.max_timeout_ms.max(request.timeout_ms);
        self.maybe_report();
        result
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        self.inner.button_held_for(duration_ms).await
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        self.inner.delay_ms(duration_ms).await
    }
}

/// Boot light-sleep self-test (`sleep_selftest`): real PMU sleeps through the
/// production backend before the radio starts, results logged afterwards.
#[cfg(feature = "light-sleep")]
pub fn sleep_selftest(sleep: Option<&mut esp32_zigbee_devkit_product::sleep::LightSleep<'_>>) {
    use esp32_zigbee_devkit_product::sleep::{SleepError, SleepStatus};
    use esp32_zigbee_devkit_product::wake::{boot_sleep_selftest, wake_code};

    if !config::SLEEP_SELFTEST {
        return;
    }
    let Some(sleep) = sleep else {
        log::warn!("[selftest] light sleep not configured");
        return;
    };
    // Read-only: wafer revision, REF_TICK dividers (fosc_tick_num must be
    // 255 for the ECO2+ RC_FAST calibration) and the boot RTC slow period.
    let tick = esp_hal::peripherals::PCR::regs().ctrl_tick_conf().read();
    log::info!(
        "[selftest] chip v{}.{} ctrl_tick_conf 0x{:08X} (fosc_tick_num {}, xtal_tick_num {}) slow period 0x{:08X}; {} x {} ms",
        esp_hal::efuse::Efuse::major_chip_version(),
        esp_hal::efuse::Efuse::minor_chip_version(),
        tick.bits(),
        tick.fosc_tick_num().bits(),
        tick.xtal_tick_num().bits(),
        esp_hal::peripherals::LP_AON::regs().store1().read().bits(),
        config::SLEEP_SELFTEST_COUNT,
        config::SLEEP_SELFTEST_MS,
    );
    let delay = esp_hal::delay::Delay::new();
    // Let the console drain before the clocks stop.
    delay.delay_millis(100);
    let mut results: [Option<Result<(u64, SleepStatus), SleepError>>;
        config::SLEEP_SELFTEST_COUNT] = [None; config::SLEEP_SELFTEST_COUNT];
    for result in &mut results {
        // SAFETY: called from `main` before the radio, flash writers and the
        // executor exist; the console was drained above.
        let outcome =
            unsafe { boot_sleep_selftest(sleep, u64::from(config::SLEEP_SELFTEST_MS) * 1_000) };
        *result = Some(outcome.map(|report| (report.elapsed_us, report.status)));
        delay.delay_millis(100);
    }
    // The USB host re-enumerates the device after the stopped periods.
    delay.delay_millis(3_000);
    for (index, result) in results.iter().enumerate() {
        match result {
            Some(Ok((elapsed_us, status))) => log::info!(
                "[selftest] sleep {}: OK, RTC {} us, {:?}",
                index + 1,
                elapsed_us,
                status
            ),
            Some(Err(error)) => log::warn!(
                "[selftest] sleep {}: error 0x{:08X} {:?}",
                index + 1,
                0x1000_0000
                    | wake_code::encode(wake_code::HARDWARE, wake_code::hardware_detail(*error)),
                error
            ),
            None => {}
        }
    }
}
