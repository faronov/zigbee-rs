//! Tickless one-microsecond Embassy clock on the CC2340 system timer.
//!
//! `SYSTIM.TIME1U` is a free-running 32-bit microsecond counter that keeps
//! counting while the CPU sleeps (and, in standby, is resynchronised from the
//! RTC by hardware). It is extended to 64 bits in software. Alarms use SYSTIM
//! channel 0 in compare mode routed to `CPUIRQ16`, mirroring TI's
//! `ClockPLPF3_freertos.c` (`ClockP_startup()` / `ClockP_scheduleNextTick()`),
//! so the CPU is interrupted only when a timer actually expires instead of on
//! every 1 ms SysTick.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use crate::interrupts::{self, Irq};
use crate::timebase::{TimerQueue, compare_target, extend};

const SYSTIM_BASE: u32 = 0x4002_2000;
const SYSTIM_ICLR: *mut u32 = (SYSTIM_BASE + 0x054) as *mut u32;
const SYSTIM_IMSET: *mut u32 = (SYSTIM_BASE + 0x058) as *mut u32;
const SYSTIM_TIME1U: *const u32 = (SYSTIM_BASE + 0x104) as *const u32;
const SYSTIM_CH0CFG: *mut u32 = (SYSTIM_BASE + 0x10C) as *mut u32;
const SYSTIM_CH0CC: *mut u32 = (SYSTIM_BASE + 0x120) as *mut u32;
const SYSTIM_ARMCLR: *mut u32 = (SYSTIM_BASE + 0x148) as *mut u32;
const SYSTIM_EV0: u32 = 1 << 0;

const EVTSVT_CPUIRQ16SEL: *mut u32 = 0x4002_5444 as *mut u32;
const EVTSVT_PUBID_SYSTIM0: u32 = 0x1B;

/// Embassy timers expected concurrently (one per waiting task/select arm).
const QUEUE_DEPTH: usize = 4;

struct State {
    /// Last value returned by [`now_locked`]; carries the 64-bit extension.
    extended: u64,
    queue: TimerQueue<QUEUE_DEPTH>,
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    extended: 0,
    queue: TimerQueue::new(),
}));

pub struct Cc2340TimeDriver;

impl embassy_time_driver::Driver for Cc2340TimeDriver {
    fn now(&self) -> u64 {
        cortex_m::interrupt::free(|cs| now_locked(&mut STATE.borrow(cs).borrow_mut()))
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        cortex_m::interrupt::free(|cs| {
            let mut state = STATE.borrow(cs).borrow_mut();
            if state.queue.schedule_wake(at, waker) {
                rearm(&mut state);
            }
        });
    }
}

embassy_time_driver::time_driver_impl!(
    static TIME_DRIVER: Cc2340TimeDriver = Cc2340TimeDriver
);

pub(crate) fn init() {
    cortex_m::interrupt::free(|cs| unsafe {
        let mut state = STATE.borrow(cs).borrow_mut();
        state.extended = u64::from(core::ptr::read_volatile(SYSTIM_TIME1U));

        // ClockP_startup(): channel 0 compare at 1 µs resolution, EV0 to
        // CPUIRQ16. EMU.HALT is deliberately left at RUN so time keeps
        // advancing while a debugger halts the core.
        core::ptr::write_volatile(SYSTIM_ARMCLR, 1 << 0);
        core::ptr::write_volatile(SYSTIM_CH0CFG, 0);
        core::ptr::write_volatile(SYSTIM_ICLR, SYSTIM_EV0);
        core::ptr::write_volatile(EVTSVT_CPUIRQ16SEL, EVTSVT_PUBID_SYSTIM0);
        core::ptr::write_volatile(SYSTIM_IMSET, SYSTIM_EV0);
        rearm(&mut state);
        interrupts::enable(Irq::CpuIrq16);
    });
}

/// `CPUIRQ16` (SYSTIM channel 0) handler.
pub(crate) fn on_interrupt() {
    unsafe { core::ptr::write_volatile(SYSTIM_ICLR, SYSTIM_EV0) };
    cortex_m::interrupt::free(|cs| rearm(&mut STATE.borrow(cs).borrow_mut()));
}

fn now_locked(state: &mut State) -> u64 {
    let low = unsafe { core::ptr::read_volatile(SYSTIM_TIME1U) };
    extend(&mut state.extended, low)
}

/// Wake expired timers and program channel 0 for the next deadline (or the
/// 30-minute keep-alive that preserves the 64-bit extension).
fn rearm(state: &mut State) {
    loop {
        let now = now_locked(state);
        let next = state.queue.next_expiration(now);
        let target = compare_target(now, next);

        // ClockP_scheduleNextTick(): un-arm, clear the NVIC pending bit of a
        // stale compare, then write CC (which clears EV0 and re-arms).
        unsafe {
            core::ptr::write_volatile(SYSTIM_ARMCLR, 1 << 0);
            interrupts::unpend(Irq::CpuIrq16);
            core::ptr::write_volatile(SYSTIM_CH0CC, target as u32);
        }

        // The compare only fires on a future match; if the deadline passed
        // while programming, expire it now instead of waiting for a wrap.
        if now_locked(state) < target {
            return;
        }
    }
}

/// Run a flash operation with IRQs masked.
///
/// SYSTIM keeps counting through the operation, so no time compensation is
/// needed; a compare that matched meanwhile stays pending and is serviced
/// as soon as interrupts are re-enabled.
pub(crate) fn run_flash_operation<T>(operation: impl FnOnce() -> T) -> T {
    cortex_m::interrupt::free(|_| operation())
}
