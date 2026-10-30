//! Interrupt-driven Embassy time driver on BL702 TIMER channel 0.
//!
//! `now()` is the hardware-proven 1 MHz free-running TIMER channel 0 counter
//! extended to 64 bits. `schedule_wake()` arms comparator 0, whose CLIC source
//! ([`TIMER_CH0_IRQ`]) wakes the core from the executor's WFI and wakes the
//! registered tasks. Every alarm is capped at 30 minutes so the interrupt
//! itself reads the counter at least twice per ~71.6-minute raw wrap.
//!
//! All state is accessed inside the single-hart critical section, from both
//! thread mode and the alarm interrupt.
//!
//! A boot self-test checks interrupt delivery. If the alarm interrupt does not
//! arrive, the driver reverts to the previous polling behavior (every
//! `schedule_wake` wakes immediately) so the device stays functional, and the
//! wake controller reports that CPU idle sleep is unavailable.

use core::cell::UnsafeCell;
use core::task::Waker;

use bl702_hal::interrupt::{self, TIMER_CH0_IRQ};
use bl702_hal::timer::{ALARM_MIN_LEAD_TICKS, AlarmArm, Monotonic};
use embassy_time_driver::Driver;

/// Distinct task wakers tracked by the alarm. The sensor runs a single task;
/// extra slots cover nested timers. Overflow falls back to an immediate wake.
const MAX_WAKERS: usize = 4;
const SELF_TEST_DELAY_TICKS: u64 = 1_000;
const SELF_TEST_TIMEOUT_TICKS: u64 = 20_000;

struct State {
    timer: Option<Monotonic>,
    earliest: u64,
    wakers: [Option<Waker>; MAX_WAKERS],
    polling: bool,
    alarm_interrupts: u32,
}

impl State {
    const fn new() -> Self {
        Self {
            timer: None,
            earliest: u64::MAX,
            wakers: [const { None }; MAX_WAKERS],
            polling: false,
            alarm_interrupts: 0,
        }
    }

    fn now(&mut self) -> u64 {
        self.timer.as_mut().map_or(0, Monotonic::ticks)
    }

    fn register(&mut self, waker: &Waker) -> bool {
        if self
            .wakers
            .iter()
            .flatten()
            .any(|registered| registered.will_wake(waker))
        {
            return true;
        }
        match self.wakers.iter_mut().find(|slot| slot.is_none()) {
            Some(slot) => {
                *slot = Some(waker.clone());
                true
            }
            None => false,
        }
    }

    /// Wake every registered task. Tasks whose deadline is later simply
    /// re-register when their timer is polled again.
    fn fire(&mut self) {
        self.earliest = u64::MAX;
        for slot in &mut self.wakers {
            if let Some(waker) = slot.take() {
                waker.wake();
            }
        }
    }

    /// Service due deadlines and leave comparator 0 armed for the earliest
    /// deadline or the wrap-safety cap.
    fn service(&mut self) {
        loop {
            let Some(timer) = self.timer.as_mut() else {
                return;
            };
            match timer.arm_alarm(self.earliest) {
                AlarmArm::Armed { .. } => return,
                AlarmArm::Due => {
                    let now = timer.ticks();
                    if self.earliest <= now.saturating_add(ALARM_MIN_LEAD_TICKS) {
                        self.fire();
                    }
                    // Otherwise only the capped wrap-safety deadline raced
                    // the counter; re-arm from the new time.
                }
            }
        }
    }
}

struct Shared(UnsafeCell<State>);

// SAFETY: Every access goes through `with_state`, which holds the BL702
// single-hart critical section (interrupts disabled).
unsafe impl Sync for Shared {}

static STATE: Shared = Shared(UnsafeCell::new(State::new()));

fn with_state<R>(operation: impl FnOnce(&mut State) -> R) -> R {
    riscv::interrupt::free(|| {
        // SAFETY: The critical section excludes the alarm interrupt and all
        // other thread-mode accesses on this single-hart core.
        operation(unsafe { &mut *STATE.0.get() })
    })
}

/// Take ownership of the monotonic timer, install the CLIC trap entry, and
/// enable the alarm interrupt source. Global interrupts stay disabled until
/// [`start`].
pub fn install(timer: Monotonic) -> Result<(), Monotonic> {
    with_state(|state| {
        if state.timer.is_some() {
            Err(timer)
        } else {
            state.timer = Some(timer);
            Ok(())
        }
    })?;
    // SAFETY: Called once from single-threaded startup with `mstatus.MIE`
    // still clear (the reset value; nothing enables it earlier).
    unsafe { interrupt::install_direct_mode(dispatch) };
    with_state(State::service);
    // SAFETY: `on_timer_interrupt` clears the comparator and CLIC pending bit.
    unsafe { interrupt::enable(TIMER_CH0_IRQ) };
    Ok(())
}

/// Enable global interrupts and prove alarm delivery with a busy-wait
/// self-test. Returns `false`, after switching to the polling fallback, if no
/// alarm interrupt arrives.
pub fn start() -> bool {
    // SAFETY: The trap entry and the only enabled source were installed by
    // `install`; the shared state is consistent.
    unsafe { riscv::interrupt::enable() };

    let before = alarm_interrupts();
    let deadline = with_state(|state| {
        let now = state.now();
        state.earliest = state.earliest.min(now + SELF_TEST_DELAY_TICKS);
        state.service();
        now + SELF_TEST_TIMEOUT_TICKS
    });
    while now() < deadline {
        if alarm_interrupts() != before {
            return true;
        }
        core::hint::spin_loop();
    }
    // Revert to the previous proven polling executor behavior.
    interrupt::disable(TIMER_CH0_IRQ);
    with_state(|state| {
        state.polling = true;
        if let Some(timer) = state.timer.as_mut() {
            timer.disable_alarm();
        }
        state.fire();
    });
    interrupt::clear_pending(TIMER_CH0_IRQ);
    false
}

/// Whether timer waits sleep in WFI (alarm interrupt proven at boot).
pub fn idle_sleep_available() -> bool {
    with_state(|state| state.timer.is_some() && !state.polling)
}

/// Number of TIMER channel 0 alarm interrupts taken since boot.
pub fn alarm_interrupts() -> u32 {
    with_state(|state| state.alarm_interrupts)
}

pub fn now() -> u64 {
    with_state(State::now)
}

fn dispatch(irq: u16) {
    if irq == TIMER_CH0_IRQ {
        on_timer_interrupt();
    } else {
        // No other source is owned by this firmware.
        interrupt::disable(irq);
        interrupt::clear_pending(irq);
    }
}

fn on_timer_interrupt() {
    with_state(|state| {
        if let Some(timer) = state.timer.as_mut() {
            timer.clear_alarm();
        }
        interrupt::clear_pending(TIMER_CH0_IRQ);
        state.alarm_interrupts = state.alarm_interrupts.wrapping_add(1);
        let now = state.now();
        if state.earliest <= now.saturating_add(ALARM_MIN_LEAD_TICKS) {
            state.fire();
        }
        state.service();
    });
}

struct Bl702TimeDriver;

impl Driver for Bl702TimeDriver {
    fn now(&self) -> u64 {
        now()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let wake_now = with_state(|state| {
            if state.polling || state.timer.is_none() || !state.register(waker) {
                return true;
            }
            state.earliest = state.earliest.min(at);
            // Fires (and wakes the just-registered waker) if already due.
            state.service();
            false
        });
        if wake_now {
            waker.wake_by_ref();
        }
    }
}

embassy_time_driver::time_driver_impl!(
    static DRIVER: Bl702TimeDriver = Bl702TimeDriver
);
