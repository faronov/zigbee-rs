//! Shared C6/H2 Embassy clock with a SYSTIMER one-shot alarm and real wakers.

use core::task::Waker;
use embassy_time_queue_utils::Queue;

trait Alarm {
    fn now(&self) -> u64;
    fn arm(&mut self, delay_us: u64);
    fn disarm(&mut self);
}

struct TimerQueue {
    queue: Queue,
    next_deadline: u64,
}

impl TimerQueue {
    const fn new() -> Self {
        Self {
            queue: Queue::new(),
            next_deadline: u64::MAX,
        }
    }

    fn schedule(&mut self, at: u64, waker: &Waker, alarm: &mut impl Alarm) {
        if self.queue.schedule_wake(at, waker) {
            self.arm_next(alarm);
        }
    }

    fn on_alarm(&mut self, alarm: &mut impl Alarm) {
        alarm.disarm();
        self.arm_next(alarm);
    }

    fn arm_next(&mut self, alarm: &mut impl Alarm) {
        loop {
            let now = alarm.now();
            let next = self.queue.next_expiration(now);
            self.next_deadline = next;
            if next == u64::MAX {
                alarm.disarm();
                return;
            }
            // Long deadlines are revisited; every armed delay fits both chips.
            let delay = next.saturating_sub(now).clamp(1, u64::from(u32::MAX));
            alarm.arm(delay);
            if alarm.now() < next {
                return;
            }
            // Expiry during programming must wake the task, not strand it.
        }
    }

    fn prepare_sleep(&mut self, max_us: u64, alarm: &mut impl Alarm) -> u64 {
        let remaining = self.next_deadline.saturating_sub(alarm.now()).min(max_us);
        alarm.disarm();
        remaining
    }
}

#[cfg(target_os = "none")]
mod hardware {
    use super::*;
    use core::cell::RefCell;
    use critical_section::Mutex;
    use embassy_time_driver::Driver;
    use esp_hal::Blocking;
    use esp_hal::peripherals::SYSTIMER;
    use esp_hal::time::{Duration, Instant};
    use esp_hal::timer::{
        OneShotTimer,
        systimer::{SystemTimer, Unit},
    };

    impl Alarm for OneShotTimer<'_, Blocking> {
        fn now(&self) -> u64 {
            Instant::now().duration_since_epoch().as_micros()
        }

        fn arm(&mut self, delay_us: u64) {
            self.schedule(Duration::from_micros(delay_us))
                .expect("bounded ESP SYSTIMER alarm");
            self.listen();
        }

        fn disarm(&mut self) {
            self.stop();
            self.unlisten();
            self.clear_interrupt();
        }
    }

    struct State {
        queue: TimerQueue,
        alarm: OneShotTimer<'static, Blocking>,
    }

    struct EspTimeDriver {
        state: Mutex<RefCell<Option<State>>>,
    }

    impl Driver for EspTimeDriver {
        fn now(&self) -> u64 {
            Instant::now().duration_since_epoch().as_micros()
        }

        fn schedule_wake(&self, at: u64, waker: &Waker) {
            critical_section::with(|cs| {
                let mut state = self.state.borrow(cs).borrow_mut();
                let State { queue, alarm } =
                    state.as_mut().expect("ESP time driver not initialized");
                queue.schedule(at, waker, alarm);
            });
        }
    }

    embassy_time_driver::time_driver_impl!(static DRIVER: EspTimeDriver = EspTimeDriver {
        state: Mutex::new(RefCell::new(None)),
    });

    #[esp_hal::handler]
    fn on_alarm() {
        critical_section::with(|cs| {
            let mut state = DRIVER.state.borrow(cs).borrow_mut();
            let State { queue, alarm } = state.as_mut().expect("ESP alarm before time driver init");
            queue.on_alarm(alarm);
        });
    }

    /// Reserve SYSTIMER alarm 0 without resetting the HAL monotonic clock.
    pub fn init(systimer: SYSTIMER<'static>) {
        critical_section::with(|cs| {
            let mut state = DRIVER.state.borrow(cs).borrow_mut();
            assert!(state.is_none(), "ESP time driver initialized twice");
            let systimer = SystemTimer::new(systimer);
            let mut alarm = OneShotTimer::new(systimer.alarm0);
            alarm.disarm();
            alarm.set_interrupt_handler(on_alarm);
            *state = Some(State {
                queue: TimerQueue::new(),
                alarm,
            });
        });
    }

    /// Bound sleep by the earliest registered Embassy deadline and disarm alarm 0.
    pub(crate) fn prepare_sleep(cs: critical_section::CriticalSection<'_>, max_us: u64) -> u64 {
        let mut state = DRIVER.state.borrow(cs).borrow_mut();
        let State { queue, alarm } = state.as_mut().expect("ESP sleep before timer init");
        queue.prepare_sleep(max_us, alarm)
    }

    /// Account once for stopped time, then wake overdue tasks and rearm alarm 0.
    pub(crate) fn resume_after_sleep(
        cs: critical_section::CriticalSection<'_>,
        missing_ticks: u64,
    ) {
        let mut state = DRIVER.state.borrow(cs).borrow_mut();
        let State { queue, alarm } = state.as_mut().expect("ESP wake before timer init");
        if missing_ticks != 0 {
            let current = SystemTimer::unit_value(Unit::Unit0);
            // The local HAL fixes the upstream high-word setter. All consumers
            // of unit 0 are suspended here; alarm 0 is the only owned alarm.
            unsafe {
                SystemTimer::set_unit_value(Unit::Unit0, corrected_counter(current, missing_ticks));
            }
        }
        queue.on_alarm(alarm);
    }
}

#[cfg(target_os = "none")]
pub use hardware::init;
#[cfg(target_os = "none")]
pub(crate) use hardware::{prepare_sleep, resume_after_sleep};

fn corrected_counter(current: u64, missing: u64) -> u64 {
    current.wrapping_add(missing) & ((1 << 52) - 1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::task::Wake;
    use std::vec::Vec;

    #[derive(Default)]
    struct WakeCount(AtomicUsize);

    impl Wake for WakeCount {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn waker() -> (Arc<WakeCount>, Waker) {
        let count = Arc::new(WakeCount::default());
        (count.clone(), Waker::from(count))
    }

    #[derive(Default)]
    struct FakeAlarm {
        now: u64,
        deadline: Option<u64>,
        arms: Vec<u64>,
        advance_during_arm: u64,
    }

    impl Alarm for FakeAlarm {
        fn now(&self) -> u64 {
            self.now
        }

        fn arm(&mut self, delay_us: u64) {
            self.arms.push(delay_us);
            self.deadline = Some(self.now + delay_us);
            self.now += core::mem::take(&mut self.advance_during_arm);
        }

        fn disarm(&mut self) {
            self.deadline = None;
        }
    }

    #[test]
    fn alarm_wakes_only_due_tasks_and_rearms_for_the_next() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm::default();
        let (late_count, late) = waker();
        let (early_count, early) = waker();
        queue.schedule(30, &late, &mut alarm);
        queue.schedule(10, &early, &mut alarm);
        assert_eq!(alarm.deadline, Some(10));
        alarm.now = 10;
        queue.on_alarm(&mut alarm);
        assert_eq!(early_count.0.load(Ordering::SeqCst), 1);
        assert_eq!(late_count.0.load(Ordering::SeqCst), 0);
        assert_eq!(alarm.deadline, Some(30));
        alarm.now = 30;
        queue.on_alarm(&mut alarm);
        assert_eq!(late_count.0.load(Ordering::SeqCst), 1);
        assert_eq!(alarm.deadline, None);
    }

    #[test]
    fn already_due_timer_wakes_without_arming_hardware() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm {
            now: 10,
            ..Default::default()
        };
        let (count, waker) = waker();
        queue.schedule(9, &waker, &mut alarm);
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(alarm.arms.is_empty());
        assert_eq!(alarm.deadline, None);
    }

    #[test]
    fn later_timer_for_same_task_does_not_hide_earlier_deadline() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm::default();
        let (_, waker) = waker();
        queue.schedule(10, &waker, &mut alarm);
        queue.schedule(20, &waker, &mut alarm);
        assert_eq!(alarm.arms, [10]);
        queue.schedule(5, &waker, &mut alarm);
        assert_eq!(alarm.arms, [10, 5]);
    }

    #[test]
    fn deadline_crossed_while_arming_is_not_missed() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm {
            now: 10,
            advance_during_arm: 2,
            ..Default::default()
        };
        let (count, waker) = waker();
        queue.schedule(11, &waker, &mut alarm);
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert_eq!(alarm.deadline, None);
    }

    #[test]
    fn long_deadline_is_rearmed_without_waking_early() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm::default();
        let (count, waker) = waker();
        let first = u64::from(u32::MAX);
        queue.schedule(first + 100, &waker, &mut alarm);
        assert_eq!(alarm.deadline, Some(first));
        alarm.now = first;
        queue.on_alarm(&mut alarm);
        assert_eq!(count.0.load(Ordering::SeqCst), 0);
        assert_eq!(alarm.deadline, Some(first + 100));
    }

    #[test]
    fn sleep_is_bounded_by_registered_deadline_and_restores_overdue_wakers() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm::default();
        let (count, waker) = waker();
        queue.schedule(100, &waker, &mut alarm);
        alarm.now = 20;
        assert_eq!(queue.prepare_sleep(5_000_000, &mut alarm), 80);
        assert_eq!(alarm.deadline, None);
        alarm.now = 120;
        queue.on_alarm(&mut alarm);
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert_eq!(alarm.deadline, None);
    }

    #[test]
    fn expired_deadline_vetoes_sleep() {
        let mut queue = TimerQueue::new();
        let mut alarm = FakeAlarm::default();
        let (_, waker) = waker();
        queue.schedule(10, &waker, &mut alarm);
        alarm.now = 11;
        assert_eq!(queue.prepare_sleep(5_000_000, &mut alarm), 0);
    }

    #[test]
    fn clock_correction_keeps_high_word_and_native_wrap() {
        assert_eq!(corrected_counter(0xffff_fff0, 32), 0x1_0000_0010);
        assert_eq!(corrected_counter((1 << 52) - 8, 10), 2);
        assert_eq!(corrected_counter(123, 0), 123);
    }
}
