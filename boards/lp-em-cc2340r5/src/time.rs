//! One-microsecond Embassy monotonic clock backed by ARM SysTick.
//!
//! SysTick interrupts every millisecond and the current down-counter provides
//! sub-millisecond interpolation. The CC2340 system timer compensates elapsed
//! time while flash erase/program temporarily masks interrupts.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;
use portable_atomic::{AtomicBool, AtomicU64, Ordering};

use crate::MCU_CLOCK_HZ;

const SYSTICK_RELOAD: u32 = MCU_CLOCK_HZ / 1_000 - 1;
const TICKS_PER_MILLISECOND: u64 = 1_000;
const CORE_CYCLES_PER_TICK: u64 = (MCU_CLOCK_HZ / 1_000_000) as u64;

const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;
const SCB_ICSR: *mut u32 = 0xE000_ED04 as *mut u32;

const CSR_ENABLE: u32 = 1 << 0;
const CSR_TICKINT: u32 = 1 << 1;
const CSR_CLKSOURCE: u32 = 1 << 2;
const ICSR_PENDSTCLR: u32 = 1 << 25;
const ICSR_PENDSTSET: u32 = 1 << 26;

const SYSTIM_TIME250N: *const u32 = 0x4002_2100 as *const u32;

static INITIALIZED: AtomicBool = AtomicBool::new(false);
static TICK_BASE: AtomicU64 = AtomicU64::new(0);

struct AlarmState {
    target: u64,
    waker: Option<core::task::Waker>,
}

static ALARM: Mutex<RefCell<AlarmState>> = Mutex::new(RefCell::new(AlarmState {
    target: u64::MAX,
    waker: None,
}));

pub struct Cc2340TimeDriver;

impl Cc2340TimeDriver {
    const fn new() -> Self {
        Self
    }
}

impl embassy_time_driver::Driver for Cc2340TimeDriver {
    fn now(&self) -> u64 {
        cortex_m::interrupt::free(|_| now_locked())
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        cortex_m::interrupt::free(|critical_section| {
            let mut alarm = ALARM.borrow(critical_section).borrow_mut();
            alarm.target = at;
            alarm.waker = Some(waker.clone());
        });

        if self.now() >= at {
            wake_alarm(at);
        }
    }
}

embassy_time_driver::time_driver_impl!(
    static TIME_DRIVER: Cc2340TimeDriver = Cc2340TimeDriver::new()
);

pub(crate) fn init() {
    cortex_m::interrupt::free(|_| unsafe {
        TICK_BASE.store(0, Ordering::Relaxed);
        core::ptr::write_volatile(SYST_CSR, 0);
        core::ptr::write_volatile(SYST_RVR, SYSTICK_RELOAD);
        core::ptr::write_volatile(SYST_CVR, 0);
        core::ptr::write_volatile(SCB_ICSR, ICSR_PENDSTCLR);
        INITIALIZED.store(true, Ordering::Release);
        core::ptr::write_volatile(SYST_CSR, CSR_CLKSOURCE | CSR_TICKINT | CSR_ENABLE);
    });
}

#[exception]
fn SysTick() {
    let now = TICK_BASE.fetch_add(TICKS_PER_MILLISECOND, Ordering::Relaxed) + TICKS_PER_MILLISECOND;
    cortex_m::interrupt::free(|critical_section| {
        let mut alarm = ALARM.borrow(critical_section).borrow_mut();
        if now >= alarm.target {
            alarm.target = u64::MAX;
            if let Some(waker) = alarm.waker.take() {
                waker.wake();
            }
        }
    });
}

fn wake_alarm(target: u64) {
    cortex_m::interrupt::free(|critical_section| {
        let mut alarm = ALARM.borrow(critical_section).borrow_mut();
        if alarm.target == target {
            alarm.target = u64::MAX;
            if let Some(waker) = alarm.waker.take() {
                waker.wake();
            }
        }
    });
}

fn now_locked() -> u64 {
    if !INITIALIZED.load(Ordering::Acquire) {
        return 0;
    }

    let base = TICK_BASE.load(Ordering::Relaxed);
    let (remaining, pending_wrap) = loop {
        let pending_before = unsafe { core::ptr::read_volatile(SCB_ICSR) } & ICSR_PENDSTSET != 0;
        let remaining = unsafe { core::ptr::read_volatile(SYST_CVR as *const u32) } as u64;
        let pending_after = unsafe { core::ptr::read_volatile(SCB_ICSR) } & ICSR_PENDSTSET != 0;
        if pending_before == pending_after {
            break (remaining, pending_after);
        }
    };

    let elapsed_cycles = u64::from(SYSTICK_RELOAD).saturating_sub(remaining);
    base + u64::from(pending_wrap) * TICKS_PER_MILLISECOND + elapsed_cycles / CORE_CYCLES_PER_TICK
}

/// Execute a flash operation with IRQs masked and account for the masked time.
pub(crate) fn run_flash_operation<T>(operation: impl FnOnce() -> T) -> T {
    let mut output = None;
    cortex_m::interrupt::free(|_| {
        if !INITIALIZED.load(Ordering::Acquire) {
            output = Some(operation());
            return;
        }

        let before = now_locked();
        unsafe {
            core::ptr::write_volatile(SYST_CSR, 0);
            core::ptr::write_volatile(SCB_ICSR, ICSR_PENDSTCLR);
        }

        let system_timer_before = unsafe { core::ptr::read_volatile(SYSTIM_TIME250N) };
        output = Some(operation());
        let system_timer_after = unsafe { core::ptr::read_volatile(SYSTIM_TIME250N) };
        let elapsed_microseconds =
            u64::from(system_timer_after.wrapping_sub(system_timer_before)) / 4;
        TICK_BASE.store(
            before.saturating_add(elapsed_microseconds),
            Ordering::Relaxed,
        );

        unsafe {
            core::ptr::write_volatile(SYST_CVR, 0);
            core::ptr::write_volatile(SCB_ICSR, ICSR_PENDSTCLR);
            core::ptr::write_volatile(SYST_CSR, CSR_CLKSOURCE | CSR_TICKINT | CSR_ENABLE);
        }
    });
    output.expect("flash operation closure was not executed")
}
