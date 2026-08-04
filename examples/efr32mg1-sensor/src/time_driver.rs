//! Embassy time driver backed by the EFR32MG1 RTCC/LFRCO.
//!
//! RTCC runs at the configured 32,768 Hz Embassy tick rate and remains
//! available in EM2. A software queue multiplexes Embassy timers onto CC0,
//! while blocking application sleep uses CC1 and `pm::now64` extends the
//! 32-bit counter across overflow. This module is the sole owner of the
//! production `RTCC` interrupt.

use core::cell::RefCell;
use core::task::Waker;

use cortex_m::interrupt::Mutex;
use efr32mg1_hal::pm;
use embassy_time_queue_utils::Queue;

struct Efr32RtccTimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

impl Efr32RtccTimeDriver {
    const fn new() -> Self {
        Self {
            queue: Mutex::new(RefCell::new(Queue::new())),
        }
    }

    /// Brings up LFRCO/RTCC (`pm::init`) and additionally enables the
    /// overflow interrupt this driver needs for its 64-bit monotonic
    /// extension.
    fn init(&self) {
        if pm::init().is_err() {
            rtt_target::rprintln!("[EFR32][time_driver] RTCC_INIT_FAIL");
            loop {
                cortex_m::asm::nop();
            }
        }
        pm::enable_overflow_interrupt();
        // Our `RTCC` handler below is a real, bounded ISR (never falls
        // through to the infinite `DefaultHandler` loop), so it is safe to
        // unmask now.
        cortex_m::peripheral::NVIC::unpend(crate::vectors::Interrupt::Rtcc);
        unsafe { cortex_m::peripheral::NVIC::unmask(crate::vectors::Interrupt::Rtcc) };
    }

    /// Arms (or disarms) hardware for `deadline` — an Embassy 64-bit tick
    /// value, or `u64::MAX` for "nothing pending". Returns `false` if
    /// `deadline` has already passed (or is `u64::MAX` and there is
    /// nothing to do beyond disarming); the caller must then re-query
    /// `Queue::next_expiration` (which will have woken the expired timer)
    /// and retry with the new next deadline, exactly as documented by
    /// `embassy_time_driver::time_driver_impl`'s own recommended pattern.
    fn set_alarm(&self, deadline: u64) -> bool {
        if deadline == u64::MAX {
            pm::disarm_wake();
            return true;
        }
        let now = pm::now64();
        match pm::ticks_from_now_clamped(now, deadline) {
            None => false,
            Some(ticks_from_now) => {
                pm::arm_wake(ticks_from_now);
                true
            }
        }
    }

    /// Re-evaluates the timer queue against the current time and
    /// (re-)arms hardware for whatever is now soonest. Called both from
    /// `schedule_wake` (a new timer may now be the soonest) and from the
    /// `RTCC` handler (the previously-armed deadline — real or an
    /// intermediate long-deadline "hop" — was reached).
    #[inline(never)]
    fn rearm(&self) {
        cortex_m::interrupt::free(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            let mut next = queue.next_expiration(pm::now64());
            while !self.set_alarm(next) {
                next = queue.next_expiration(pm::now64());
            }
        });
    }
}

impl embassy_time_driver::Driver for Efr32RtccTimeDriver {
    fn now(&self) -> u64 {
        pm::now64()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let changed = cortex_m::interrupt::free(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            queue.schedule_wake(at, waker)
        });
        if changed {
            self.rearm();
        }
    }
}

embassy_time_driver::time_driver_impl!(
    static TIME_DRIVER: Efr32RtccTimeDriver = Efr32RtccTimeDriver::new()
);

pub fn init() {
    TIME_DRIVER.init();
}

// ── RTCC interrupt handler ──────────────────────────────────────
//
// Owns the vector for every profile that compiles this module (see module
// doc header). Reads and clears CC0, CC1, and overflow in one shot. CC1 only
// wakes the blocking application sleep; Embassy is re-armed exclusively for
// its own CC0 event or an overflow that may expire a long software deadline.
#[unsafe(no_mangle)]
pub extern "C" fn RTCC() {
    let flags = pm::take_pending_flags();
    if flags.overflow {
        pm::bump_wrap_count();
    }
    if flags.cc0 || flags.overflow {
        TIME_DRIVER.rearm();
    }
}
