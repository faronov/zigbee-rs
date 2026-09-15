//! Embassy time driver for EFR32MG21 using ARM SysTick.
//!
//! Provides a real monotonic timer and alarm for Embassy async runtime.
//! Uses the SysTick exception (always available on Cortex-M33) so no
//! EFR32-specific timer peripherals are needed.
//!
//! HCLK is configured to BRD4181A's 38.4 MHz HFXO by `platform::init` before
//! this driver starts.

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;

use core::sync::atomic::{AtomicU32, Ordering};

// ── Configuration ───────────────────────────────────────────────

/// BRD4181A system clock (HCLK) frequency in Hz.
const HCLK_HZ: u32 = efr32mg21_devkit::HCLK_HZ;

/// SysTick fires every 1 ms.
const SYSTICK_RELOAD: u32 = HCLK_HZ / 1000 - 1; // 38_399

/// Embassy ticks per SysTick overflow.
/// Embassy TICK_HZ = 1_000_000, SysTick overflow = 1 ms = 1000 ticks.
const TICKS_PER_MS: u64 = embassy_time_driver::TICK_HZ / 1_000;
const _: () = assert!(embassy_time_driver::TICK_HZ == 1_000_000);

// ── SysTick register addresses (ARM standard) ──────────────────

const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;
const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
const ICSR_PENDSTSET: u32 = 1 << 26;

const CSR_ENABLE: u32 = 1 << 0;
const CSR_TICKINT: u32 = 1 << 1;
const CSR_CLKSOURCE: u32 = 1 << 2;

// ── State ───────────────────────────────────────────────────────

static MS_COUNT: AtomicU32 = AtomicU32::new(0);
static MS_EPOCH: AtomicU32 = AtomicU32::new(0);
static CLOCK: Mutex<RefCell<super::time_snapshot::Clock>> =
    Mutex::new(RefCell::new(super::time_snapshot::Clock::new()));

struct AlarmState {
    target: u64,
    waker: Option<core::task::Waker>,
}

static ALARM: Mutex<RefCell<AlarmState>> = Mutex::new(RefCell::new(AlarmState {
    target: u64::MAX,
    waker: None,
}));

// ── Driver ──────────────────────────────────────────────────────

pub struct Efr32TimeDriver;

impl Efr32TimeDriver {
    pub const fn new() -> Self {
        Self
    }

    pub fn init(&self) {
        unsafe {
            core::ptr::write_volatile(SYST_RVR, SYSTICK_RELOAD);
            core::ptr::write_volatile(SYST_CVR, 0);
            core::ptr::write_volatile(SYST_CSR, CSR_CLKSOURCE | CSR_TICKINT | CSR_ENABLE);
        }
    }
}

impl embassy_time_driver::Driver for Efr32TimeDriver {
    fn now(&self) -> u64 {
        cortex_m::interrupt::free(|cs| {
            let epoch = MS_EPOCH.load(Ordering::Relaxed) as u64;
            let ms = MS_COUNT.load(Ordering::Relaxed) as u64;
            let full_ms = (epoch << 32) | ms;

            CLOCK
                .borrow(cs)
                .borrow_mut()
                .now(full_ms, SYSTICK_RELOAD, HCLK_HZ, || {
                    // A reload may happen despite PRIMASK. Do not combine its new
                    // counter with the software epoch preceding the pending ISR.
                    unsafe {
                        let before = core::ptr::read_volatile(SCB_ICSR) & ICSR_PENDSTSET != 0;
                        let remaining = core::ptr::read_volatile(SYST_CVR);
                        let after = core::ptr::read_volatile(SCB_ICSR) & ICSR_PENDSTSET != 0;
                        (before, remaining, after)
                    }
                })
        })
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        cortex_m::interrupt::free(|cs| {
            let mut alarm = ALARM.borrow(cs).borrow_mut();
            alarm.target = at;
            alarm.waker = Some(waker.clone());
        });

        if self.now() >= at {
            cortex_m::interrupt::free(|cs| {
                let mut alarm = ALARM.borrow(cs).borrow_mut();
                if alarm.target == at {
                    alarm.target = u64::MAX;
                    if let Some(waker) = alarm.waker.take() {
                        waker.wake();
                    }
                }
            });
        }
    }
}

// ── SysTick exception handler ───────────────────────────────────

#[exception]
fn SysTick() {
    let prev = MS_COUNT.load(Ordering::Relaxed);
    let next = prev.wrapping_add(1);
    MS_COUNT.store(next, Ordering::Relaxed);
    if next == 0 {
        let ep = MS_EPOCH.load(Ordering::Relaxed);
        MS_EPOCH.store(ep.wrapping_add(1), Ordering::Relaxed);
    }

    let epoch = MS_EPOCH.load(Ordering::Relaxed) as u64;
    let ms = next as u64;
    let now_ticks = ((epoch << 32) | ms) * TICKS_PER_MS;

    cortex_m::interrupt::free(|cs| {
        let mut alarm = ALARM.borrow(cs).borrow_mut();
        if now_ticks >= alarm.target {
            alarm.target = u64::MAX;
            if let Some(waker) = alarm.waker.take() {
                waker.wake();
            }
        }
    });
}

// ── Registration ────────────────────────────────────────────────

embassy_time_driver::time_driver_impl!(
    static TIME_DRIVER: Efr32TimeDriver = Efr32TimeDriver::new()
);

pub fn init() {
    TIME_DRIVER.init();
}
