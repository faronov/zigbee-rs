//! Embassy time driver for EFR32MG1P backed by the RTCC/LFRCO EM2-capable
//! wake timer in `efr32mg1_hal::pm` (see that module's own doc header for
//! why RTCC-from-LFRCO is the Silicon-Labs-sanctioned Series-1 EM2 wake
//! source, and why LFRCO rather than LFXO).
//!
//! # Replaces the SysTick driver
//!
//! Every profile that previously ran on the ARM SysTick exception (assumed
//! HCLK-derived 1 MHz Embassy ticks, no deep sleep) now runs on this RTCC
//! driver instead. SysTick cannot survive EM2 (the Cortex-M4 core clock and
//! its SysTick timer both stop in EM2), so an EM2-aware Embassy runtime
//! needs a wake source that keeps ticking there — RTCC clocked from LFRCO
//! over the CMU "LFE" branch does.
//!
//! # Tick rate: exactly 32768 Hz, matching RTCC 1:1
//!
//! `Cargo.toml` enables `embassy-time-driver`'s `tick-hz-32_768` feature, so
//! `embassy_time_driver::TICK_HZ == 32_768 == pm::LFRCO_HZ`. Because RTCC's
//! `CNTPRESC` is left at its reset default (`DIV1`) by `pm::init()`, the
//! RTCC counter itself increments at exactly the LFRCO rate, i.e. one RTCC
//! tick == one Embassy tick. This is a deliberate simplification: no
//! fractional tick-rate conversion (the previous SysTick driver needed one,
//! since HCLK and 1 MHz Embassy ticks did not divide evenly) is needed
//! anywhere in this module.
//!
//! `embassy-time-driver` is `links = "embassy-time"`, so Cargo unifies a
//! single copy of it for the whole dependency graph; `zigbee-mac`'s
//! `efr32` feature depends on `embassy-time` 0.4 (this crate depends on
//! 0.5), but both resolve to the *same* `embassy-time-driver` 0.2.x
//! instance, so `zigbee_mac::efr32`'s own `embassy_time::Timer`/`Instant`
//! calls run on this exact same 32768 Hz driver too — not a second,
//! inconsistent one. Its `Timer::after_micros(128)` call
//! (`zigbee-mac/src/efr32/driver.rs`) now quantizes to whole ~30.5 µs RTCC
//! ticks instead of whole microseconds; this is an inherent, expected
//! consequence of a single shared 32768 Hz timebase (required by this
//! gate) and is not a change to radio *policy* — no timeout duration,
//! retry count, or decision changed, only the underlying tick granularity
//! used to wait it out.
//!
//! # 64-bit monotonic across the 32-bit RTCC wraparound
//!
//! RTCC's hardware counter is 32 bits and wraps every
//! `2^32 / 32768 ≈ 131_072` s (~36.4 hours). `now()` extends this to a
//! genuinely monotonic 64-bit Embassy tick count using
//! `efr32mg1_hal::pm::now64()` — see that function's doc comment for the
//! race-safety argument. This module is only responsible for feeding it
//! the overflow interrupt (`enable_overflow_interrupt` in `init`) and
//! calling `bump_wrap_count()` from `RTCC` when the overflow flag was
//! observed pending.
//!
//! # One compare, many software timers
//!
//! RTCC only has to arm a *single* hardware compare (`CC0`, via
//! `pm::arm_wake`/`pm::disarm_wake`) at a time. Multiple concurrent
//! `embassy_time::Timer`s are supported by holding them in
//! `embassy_time_queue_utils::Queue` (the "integrated" variant: timer
//! state lives in each task's own header, no heap/global `Vec` needed) and
//! always arming hardware for only the *soonest* pending deadline. This
//! follows the exact pattern documented in `embassy_time_driver`'s own
//! `Driver` trait docs.
//!
//! # Long deadlines
//!
//! A single RTCC compare can only unambiguously represent a deadline up to
//! `pm::MAX_ARM_TICKS` (half the 32-bit range) away from `now` — see
//! `pm::ticks_from_now_clamped`. A `Timer::after` requesting something
//! further out (this module supports the full 64-bit Embassy range, not
//! just one RTCC wrap) is armed as a capped intermediate "hop": when that
//! hop's compare fires, `RTCC` re-evaluates the queue and re-arms again,
//! repeating until the real deadline is within range. This requires no
//! extra bookkeeping beyond what `rearm` already does after *every* fire.
//!
//! # This module owns the `RTCC` interrupt
//!
//! For every profile that compiles this module (`sensor`, `diag-join`,
//! `diag-beacon`, `diag-sht`, `diag-nv`, `diag-rtcc-time`), the `RTCC`
//! handler below is the *only* one linked in, and it is responsible for
//! both the CC0 wake and the overflow epoch. `diag-em2` does not compile
//! this module at all (see `main.rs`'s `mod time_driver` gate) and keeps
//! its own minimal `RTCC` handler there, calling only
//! `efr32mg1_hal::pm::handle_interrupt()` — unchanged, so its
//! hardware-proven behavior (10 × 1 s EM2 cycles, exactly 32768 ticks
//! elapsed, all canaries intact) cannot regress.
//!
//! # SLEEPDEEP boundary
//!
//! This driver's own `Timer`-driven path (`schedule_wake`/`rearm`) never
//! touches `SCB.SCR.SLEEPDEEP` — arming/disarming CC0 and waking a task is
//! all it does, so the Embassy executor's normal idle behavior (EM1, via
//! `cortex_m::asm::wfe`) is unaffected; that is future SED work, not this
//! gate. The *only* thing in this dependency graph that ever sets
//! `SLEEPDEEP` is `efr32mg1_hal::pm::enter_em2_once()` (used internally by
//! `pm::sleep_for_ticks`/`pm::sleep_for_ticks_polled`), which applies the
//! DCDC LN-handshake safety gate immediately before every `WFI` and clears
//! `SLEEPDEEP` again immediately after, unconditionally, regardless of
//! what woke the core — so it can never be left armed by accident. See
//! `diag-rtcc-time`'s explicit EM2 phase for a diagnostic that calls this
//! directly and then asserts `pm::sleepdeep_is_set() == false` on return.

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
    /// extension — a strict superset of what `pm::init()` alone provides,
    /// so `diag-em2` (which only calls `pm::init()`) is unaffected.
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
// doc header). Reads and clears both flags it cares about in one shot
// (`take_pending_flags`), advances the 64-bit epoch if the counter
// wrapped, then re-arms hardware for whatever is now the soonest pending
// software timer (if any).
#[unsafe(no_mangle)]
pub extern "C" fn RTCC() {
    let flags = pm::take_pending_flags();
    if flags.overflow {
        pm::bump_wrap_count();
    }
    TIME_DRIVER.rearm();
}
