//! Atomic radio/PMU/monotonic-clock transition shared by both ESP sensors.

use crate::sleep::SleepReport;

fn console_idle(usb_seen: bool, uart_fifo_count: u8, uart_tx_state: u8) -> bool {
    !usb_seen && uart_fifo_count == 0 && uart_tx_state == 0
}

trait SleepCycle {
    type Error;
    fn suspend(&mut self) -> Result<bool, Self::Error>;
    fn enter(&mut self) -> Result<Option<SleepReport>, Self::Error>;
    fn restore(&mut self, report: Option<&SleepReport>) -> Result<(), Self::Error>;
}

/// Stable diagnostic encoding of a wake error: variant in bits 24..31, a
/// variant-specific detail in bits 0..23. Recorded in retained memory and the
/// anomaly record, so the numbering must not change.
pub mod wake_code {
    pub const UNSUPPORTED_DEPTH: u32 = 1;
    pub const SLEEP_NOT_CONFIGURED: u32 = 2;
    pub const GPIO: u32 = 3;
    pub const HARDWARE: u32 = 4;
    pub const UNEXPECTED_WAKE: u32 = 5;
    pub const REPEATED_REJECTION: u32 = 6;
    pub const RADIO_WAKE: u32 = 7;

    pub const fn encode(variant: u32, detail: u32) -> u32 {
        (variant << 24) | (detail & 0x00FF_FFFF)
    }

    /// `HARDWARE` detail. 3 is the unspecified `InvalidCalibration` written
    /// by older firmware; records from it still decode, new firmware writes
    /// the specific 6-8 instead.
    pub const fn hardware_detail(error: crate::sleep::SleepError) -> u32 {
        use crate::sleep::{CalibrationFault, SleepError};
        match error {
            SleepError::UnsupportedClock => 1,
            SleepError::UnsupportedConfiguration => 2,
            SleepError::InvalidDuration => 4,
            SleepError::InvalidCounter => 5,
            SleepError::InvalidCalibration(CalibrationFault::SlowPeriodZero) => 6,
            SleepError::InvalidCalibration(CalibrationFault::FastTimeout) => 7,
            SleepError::InvalidCalibration(CalibrationFault::FastOutOfRange) => 8,
        }
    }
}

fn run_cycle<C: SleepCycle>(cycle: &mut C) -> Result<Option<SleepReport>, C::Error> {
    if !cycle.suspend()? {
        return Ok(None);
    }
    let result = cycle.enter();
    cycle.restore(result.as_ref().ok().and_then(Option::as_ref))?;
    result
}

#[cfg(target_os = "none")]
mod hardware {
    use super::*;
    use crate::sleep::{LightSleep, MIN_SLEEP_US, SleepError, SleepStatus, WakeCause};
    use crate::time_driver;
    use embassy_futures::select::{Either, select};
    use embassy_time::{Duration, Instant, Timer};
    use esp_hal::gpio::{Input, WakeConfigError, WakeEvent};
    use sensor_sed_app::{SleepDepth, WaitRequest, WakeController, WakeReason};
    use zigbee_mac::esp::EspSoftMac;

    #[derive(Debug)]
    pub enum WakeError {
        UnsupportedDepth(SleepDepth),
        SleepNotConfigured,
        Gpio(WakeConfigError),
        Hardware(SleepError),
        UnexpectedWake(u32),
        RepeatedRejection(u32),
        /// Re-applying the radio configuration after wake failed. Carries no
        /// payload so the `MacError` Debug formatter is not linked again.
        RadioWake,
    }

    /// Boot diagnostic: one real PMU light sleep through the production
    /// backend (same power-down flags, calibration and clock restore), timer
    /// wake only, ignoring the USB veto. The Embassy alarm is disarmed and the
    /// measured stopped time is applied to SYSTIMER exactly as in [`SensorWake`].
    /// The returned report carries the RTC-measured duration and wake status.
    ///
    /// # Safety
    ///
    /// Call before the radio, flash writers or any other PLL-dependent
    /// activity starts, with the console flushed. A USB-Serial-JTAG host loses
    /// the device while the clocks are stopped.
    pub unsafe fn boot_sleep_selftest(
        sleep: &mut LightSleep<'_>,
        duration_us: u64,
    ) -> Result<crate::sleep::SleepReport, SleepError> {
        critical_section::with(|cs| {
            let _ = time_driver::prepare_sleep(cs, u64::MAX);
            let result = unsafe { sleep.sleep(cs, duration_us, false) };
            time_driver::resume_after_sleep(
                cs,
                result.as_ref().map_or(0, |r| r.missing_systimer_ticks),
            );
            result
        })
    }

    /// Owns the button and optional, explicitly selected retained-sleep backend.
    pub struct SensorWake<'d> {
        button: Input<'d>,
        sleep: Option<LightSleep<'d>>,
        rejections: u8,
        usb_warning: bool,
    }

    impl<'d> SensorWake<'d> {
        pub fn new(button: Input<'d>, sleep: Option<LightSleep<'d>>) -> Self {
            Self {
                button,
                sleep,
                rejections: 0,
                usb_warning: false,
            }
        }

        async fn active_wait(&mut self, timeout_ms: u32) -> WakeReason {
            match select(
                self.button.wait_for_low(),
                Timer::after_millis(u64::from(timeout_ms)),
            )
            .await
            {
                Either::First(()) => WakeReason::Button,
                Either::Second(()) => WakeReason::Timer,
            }
        }
    }

    struct Cycle<'a, 'cs, 'button, 'radio> {
        cs: critical_section::CriticalSection<'cs>,
        button: &'a mut Input<'button>,
        sleep: &'a mut LightSleep<'button>,
        mac: &'a mut EspSoftMac<'radio>,
        max_us: u64,
        button_seen: bool,
    }

    impl SleepCycle for Cycle<'_, '_, '_, '_> {
        type Error = WakeError;

        fn suspend(&mut self) -> Result<bool, WakeError> {
            let uart = esp_hal::peripherals::UART0::regs();
            if !console_idle(
                usb_seen(),
                uart.status().read().txfifo_cnt().bits(),
                uart.fsm_status().read().st_utx_out().bits(),
            ) {
                return Ok(false);
            }
            // The radio vetoes unless this is an idle sleepy end device
            // (`macRxOnWhenIdle` off, not scanning, nothing parked).
            match self.mac.sleep() {
                Ok(()) => Ok(true),
                Err(_) => {
                    // The PHY already logged the veto reason; formatting the
                    // MacError here would link another Debug copy.
                    log::debug!("[ESP sleep] deferred by radio");
                    Ok(false)
                }
            }
        }

        fn enter(&mut self) -> Result<Option<SleepReport>, WakeError> {
            let duration = time_driver::prepare_sleep(self.cs, self.max_us);
            self.button_seen |= self.button.is_low();
            if duration < MIN_SLEEP_US || self.button_seen {
                return Ok(None);
            }
            self.button.clear_interrupt();
            self.button
                .wakeup_enable(true, WakeEvent::LowLevel)
                .map_err(WakeError::Gpio)?;
            // No application future runs inside this critical section. The MAC
            // has stopped DMA/ACK/TX, flash operations are synchronous, and this
            // composition leaves CPU/RAM/top/flash domains retained.
            unsafe { self.sleep.sleep(self.cs, duration, true) }
                .map(Some)
                .map_err(WakeError::Hardware)
        }

        fn restore(&mut self, report: Option<&SleepReport>) -> Result<(), WakeError> {
            time_driver::resume_after_sleep(
                self.cs,
                report.map_or(0, |r| r.missing_systimer_ticks),
            );
            self.button_seen |= self.button.is_low();
            let gpio = self.button.wakeup_enable(false, WakeEvent::LowLevel);
            self.button.clear_interrupt();
            let radio = self.mac.wake();
            // A radio that failed to wake must never be reported as the
            // recoverable GPIO error, so check it first.
            radio.map_err(|_| WakeError::RadioWake)?;
            gpio.map_err(WakeError::Gpio)
        }
    }

    fn usb_seen() -> bool {
        // esp-println's auto backend deliberately retains this SOF latch.
        // Do not clear it or change its console routing behind its back.
        esp_hal::peripherals::USB_DEVICE::regs()
            .int_raw()
            .read()
            .sof()
            .bit_is_set()
    }

    impl<'button, 'radio> WakeController<EspSoftMac<'radio>> for SensorWake<'button> {
        type Mark = Instant;
        type Error = WakeError;

        /// `run_cycle` always restores the radio and returns a restore error
        /// in preference to an entry error, so every error except
        /// `RadioWake` leaves the MAC awake and usable for an active wait.
        fn is_recoverable(error: &WakeError) -> bool {
            !matches!(error, WakeError::RadioWake)
        }

        fn error_code(error: &WakeError) -> u32 {
            use super::wake_code::*;
            match *error {
                WakeError::UnsupportedDepth(depth) => encode(UNSUPPORTED_DEPTH, depth as u32),
                WakeError::SleepNotConfigured => encode(SLEEP_NOT_CONFIGURED, 0),
                WakeError::Gpio(WakeConfigError::EdgeTriggeringNotSupported) => encode(GPIO, 1),
                WakeError::Gpio(_) => encode(GPIO, 0),
                WakeError::Hardware(error) => encode(HARDWARE, hardware_detail(error)),
                WakeError::UnexpectedWake(cause) => encode(UNEXPECTED_WAKE, cause),
                WakeError::RepeatedRejection(cause) => encode(REPEATED_REJECTION, cause),
                WakeError::RadioWake => encode(RADIO_WAKE, 0),
            }
        }

        fn mark(&self) -> Instant {
            Instant::now()
        }

        fn add_ms(mark: Instant, duration_ms: u32) -> Instant {
            mark + Duration::from_millis(u64::from(duration_ms))
        }

        fn elapsed_ms(later: Instant, earlier: Instant) -> u32 {
            later
                .saturating_duration_since(earlier)
                .as_millis()
                .min(u64::from(u32::MAX)) as u32
        }

        async fn wait(
            &mut self,
            mac: &mut EspSoftMac<'radio>,
            request: WaitRequest,
        ) -> Result<WakeReason, WakeError> {
            if self.button.is_low() {
                return Ok(WakeReason::Button);
            }
            match request.sleep_depth {
                SleepDepth::Active => return Ok(self.active_wait(request.timeout_ms).await),
                SleepDepth::Idle if cfg!(feature = "light-sleep") => {}
                depth => return Err(WakeError::UnsupportedDepth(depth)),
            }
            if usb_seen() {
                if !self.usb_warning {
                    log::warn!(
                        "[ESP sleep] native USB session vetoes PMU sleep; cold boot from power-only supply to qualify sleep"
                    );
                    self.usb_warning = true;
                }
                return Ok(match self.active_wait(request.timeout_ms.min(250)).await {
                    WakeReason::Button => WakeReason::Button,
                    _ => WakeReason::Activity,
                });
            }
            if u64::from(request.timeout_ms) * 1000 < MIN_SLEEP_US {
                return Ok(self.active_wait(request.timeout_ms).await);
            }
            let sleep = self.sleep.as_mut().ok_or(WakeError::SleepNotConfigured)?;
            let (result, button_seen) = critical_section::with(|cs| {
                let mut cycle = Cycle {
                    cs,
                    button: &mut self.button,
                    sleep,
                    mac,
                    max_us: u64::from(request.timeout_ms) * 1000,
                    button_seen: false,
                };
                let result = run_cycle(&mut cycle);
                (result, cycle.button_seen)
            });
            let report = match result {
                Ok(report) => report,
                Err(error) => {
                    log::error!("[ESP sleep] transition failed: {:?}", error);
                    return Err(error);
                }
            };
            if let Some(report) = report {
                match report.status {
                    SleepStatus::Woke(cause) => {
                        self.rejections = 0;
                        log::debug!(
                            "[ESP sleep] woke {:?}, elapsed {}us",
                            cause,
                            report.elapsed_us
                        );
                        if button_seen || self.button.is_low() {
                            return Ok(WakeReason::Button);
                        }
                        return Ok(match cause {
                            WakeCause::Gpio | WakeCause::TimerAndGpio => WakeReason::Button,
                            WakeCause::Timer => WakeReason::Timer,
                        });
                    }
                    SleepStatus::Rejected { cause } => {
                        self.rejections = self.rejections.saturating_add(1);
                        log::warn!("[ESP sleep] PMU rejected sleep: 0x{:X}", cause);
                        if button_seen || self.button.is_low() {
                            self.rejections = 0;
                            return Ok(WakeReason::Button);
                        }
                        if self.rejections >= 8 {
                            return Err(WakeError::RepeatedRejection(cause));
                        }
                    }
                    SleepStatus::Unexpected { cause } => {
                        log::error!("[ESP sleep] unexpected wake: 0x{:X}", cause);
                        return Err(WakeError::UnexpectedWake(cause));
                    }
                }
            }
            if button_seen || self.button.is_low() {
                return Ok(WakeReason::Button);
            }
            // Give pending IRQs a chance to finish before the application
            // drains queued traffic and recomputes its idle policy.
            Timer::after_millis(1).await;
            Ok(if self.button.is_low() {
                WakeReason::Button
            } else {
                WakeReason::Activity
            })
        }

        async fn button_held_for(&mut self, duration_ms: u32) -> bool {
            matches!(
                select(
                    self.button.wait_for_high(),
                    Timer::after_millis(u64::from(duration_ms))
                )
                .await,
                Either::Second(())
            )
        }

        async fn delay_ms(&mut self, duration_ms: u32) {
            Timer::after_millis(u64::from(duration_ms)).await;
        }
    }
}

#[cfg(target_os = "none")]
pub use hardware::{SensorWake, WakeError, boot_sleep_selftest};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sleep::{SleepStatus, WakeCause};
    use std::vec::Vec;

    #[test]
    fn hardware_cause_words_are_stable_and_calibration_steps_distinct() {
        use crate::sleep::{CalibrationFault as F, SleepError as E};
        let word =
            |e| 0x1000_0000 | wake_code::encode(wake_code::HARDWARE, wake_code::hardware_detail(e));
        assert_eq!(word(E::UnsupportedClock), 0x1400_0001);
        assert_eq!(word(E::UnsupportedConfiguration), 0x1400_0002);
        assert_eq!(word(E::InvalidDuration), 0x1400_0004);
        assert_eq!(word(E::InvalidCounter), 0x1400_0005);
        assert_eq!(word(E::InvalidCalibration(F::SlowPeriodZero)), 0x1400_0006);
        assert_eq!(word(E::InvalidCalibration(F::FastTimeout)), 0x1400_0007);
        assert_eq!(word(E::InvalidCalibration(F::FastOutOfRange)), 0x1400_0008);
    }

    #[test]
    fn console_must_be_quiet_before_changing_pll_clocks() {
        assert!(console_idle(false, 0, 0));
        assert!(!console_idle(true, 0, 0));
        assert!(!console_idle(false, 1, 0));
        assert!(!console_idle(false, 0, 1));
    }

    #[derive(Default)]
    struct Fake {
        busy: bool,
        skip: bool,
        status: Option<SleepStatus>,
        fail_entry: bool,
        fail_restore: bool,
        order: Vec<&'static str>,
        corrected: u64,
    }

    impl SleepCycle for Fake {
        type Error = &'static str;
        fn suspend(&mut self) -> Result<bool, Self::Error> {
            self.order.push("suspend");
            Ok(!self.busy)
        }
        fn enter(&mut self) -> Result<Option<SleepReport>, Self::Error> {
            self.order.push("enter");
            if self.skip {
                return Ok(None);
            }
            if self.fail_entry {
                return Err("entry");
            }
            let status = self.status.unwrap_or(SleepStatus::Woke(WakeCause::Timer));
            Ok(Some(SleepReport {
                elapsed_us: 50_000,
                missing_systimer_ticks: if matches!(status, SleepStatus::Rejected { .. }) {
                    0
                } else {
                    42
                },
                status,
                top_powered_down: false,
            }))
        }
        fn restore(&mut self, report: Option<&SleepReport>) -> Result<(), Self::Error> {
            self.order.push("restore");
            self.corrected += report.map_or(0, |r| r.missing_systimer_ticks);
            if self.fail_restore {
                Err("restore")
            } else {
                Ok(())
            }
        }
    }

    #[test]
    fn busy_radio_never_enters_sleep() {
        let mut cycle = Fake {
            busy: true,
            ..Fake::default()
        };
        assert!(run_cycle(&mut cycle).unwrap().is_none());
        assert_eq!(cycle.order, ["suspend"]);
    }

    #[test]
    fn successful_sleep_compensates_once_before_return() {
        let mut cycle = Fake::default();
        assert!(run_cycle(&mut cycle).unwrap().is_some());
        assert_eq!(cycle.order, ["suspend", "enter", "restore"]);
        assert_eq!(cycle.corrected, 42);
    }

    #[test]
    fn entry_error_still_restores_radio_without_guessing_elapsed_time() {
        let mut cycle = Fake {
            fail_entry: true,
            ..Fake::default()
        };
        assert_eq!(run_cycle(&mut cycle), Err("entry"));
        assert_eq!(cycle.order, ["suspend", "enter", "restore"]);
        assert_eq!(cycle.corrected, 0);
    }

    #[test]
    fn restoration_error_cannot_report_success() {
        let mut cycle = Fake {
            fail_restore: true,
            ..Fake::default()
        };
        assert_eq!(run_cycle(&mut cycle), Err("restore"));
        assert_eq!(cycle.corrected, 42);
    }

    #[test]
    fn button_or_deadline_veto_after_suspend_still_restores() {
        let mut cycle = Fake {
            skip: true,
            ..Fake::default()
        };
        assert!(run_cycle(&mut cycle).unwrap().is_none());
        assert_eq!(cycle.order, ["suspend", "enter", "restore"]);
        assert_eq!(cycle.corrected, 0);
    }

    #[test]
    fn rejected_sleep_restores_without_inventing_stopped_time() {
        let status = SleepStatus::Rejected { cause: 4 };
        let mut cycle = Fake {
            status: Some(status),
            ..Fake::default()
        };
        assert_eq!(run_cycle(&mut cycle).unwrap().unwrap().status, status);
        assert_eq!(cycle.order, ["suspend", "enter", "restore"]);
        assert_eq!(cycle.corrected, 0);
    }
}
