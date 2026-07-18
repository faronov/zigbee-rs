//! Embassy time driver for PHY6222 using ARM SysTick.
//!
//! Provides a real monotonic timer and alarm for Embassy async runtime.
//! Uses the SysTick exception (always available on Cortex-M) so no
//! PHY6222-specific timer peripherals are needed.
//!
//! # Clock assumption
//! HCLK = 48 MHz (PHY6222 default after ROM boot via DLL).
//! If your board uses a different system clock, adjust `HCLK_HZ`.
//!
//! # Resolution
//! - Monotonic: ~1 µs (SysTick counts at 48 MHz, sub-ms interpolation)
//! - Alarm: 1 ms granularity (checked every SysTick overflow)

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::exception;
use portable_atomic::{AtomicU64, Ordering};

// ── Configuration ───────────────────────────────────────────────

/// PHY6222 system clock (HCLK) frequency in Hz.
///
/// Default: 48 MHz from DLL after ROM bootloader.
/// The `init_hardware()` in the radio driver enables DLL/DBL clocks
/// but does not change HCLK — it stays at the ROM default.
const HCLK_HZ: u32 = 48_000_000;

/// SysTick fires every 1 ms (48,000 HCLK cycles).
const SYSTICK_RELOAD: u32 = HCLK_HZ / 1000 - 1; // 47_999

/// Embassy ticks per SysTick overflow.
/// Embassy TICK_HZ = 1_000_000, SysTick overflow = 1 ms = 1000 ticks.
const TICKS_PER_MS: u64 = 1_000;
const TICK_HZ: u64 = 1_000_000;

/// HCLK cycles per Embassy tick (for sub-ms interpolation).
const HCLK_PER_TICK: u64 = (HCLK_HZ / 1_000_000) as u64; // 48

// ── SysTick register addresses (ARM standard) ──────────────────

const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;
const SCB_ICSR: *mut u32 = 0xE000_ED04 as *mut u32;

// CSR bits
const CSR_ENABLE: u32 = 1 << 0;
const CSR_TICKINT: u32 = 1 << 1;
const CSR_CLKSOURCE: u32 = 1 << 2; // 1 = processor clock
const ICSR_PENDSTSET: u32 = 1 << 26;
const ICSR_PENDSTCLR: u32 = 1 << 25;

// ── State ───────────────────────────────────────────────────────

/// Completed Embassy ticks before the current SysTick period.
static TICK_BASE: AtomicU64 = AtomicU64::new(0);

/// Alarm state: target tick and waker, protected by critical section.
struct AlarmState {
    target: u64,
    waker: Option<core::task::Waker>,
}

static ALARM: Mutex<RefCell<AlarmState>> = Mutex::new(RefCell::new(AlarmState {
    target: u64::MAX,
    waker: None,
}));

// ── Driver ──────────────────────────────────────────────────────

/// Embassy time driver backed by SysTick.
pub struct Phy6222TimeDriver;

impl Phy6222TimeDriver {
    pub const fn new() -> Self {
        Self
    }

    /// Start the SysTick timer. Call once at startup, before any async code.
    pub fn init(&self) {
        unsafe {
            // Set reload value (1 ms period)
            core::ptr::write_volatile(SYST_RVR, SYSTICK_RELOAD);
            // Clear current value
            core::ptr::write_volatile(SYST_CVR, 0);
            // Enable: processor clock + interrupt + counter on
            core::ptr::write_volatile(SYST_CSR, CSR_CLKSOURCE | CSR_TICKINT | CSR_ENABLE);
        }
    }
}

impl embassy_time_driver::Driver for Phy6222TimeDriver {
    fn now(&self) -> u64 {
        cortex_m::interrupt::free(|_| now_locked())
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        cortex_m::interrupt::free(|cs| {
            let mut alarm = ALARM.borrow(cs).borrow_mut();
            alarm.target = at;
            alarm.waker = Some(waker.clone());
        });

        // If the alarm is already in the past, fire immediately
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
    let now_ticks = TICK_BASE.fetch_add(TICKS_PER_MS, Ordering::Relaxed) + TICKS_PER_MS;

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
    static TIME_DRIVER: Phy6222TimeDriver = Phy6222TimeDriver::new()
);

/// Initialize the time driver. Call from main before starting async tasks.
pub fn init() {
    TIME_DRIVER.init();
}

fn now_locked() -> u64 {
    let base = TICK_BASE.load(Ordering::Relaxed);

    // Sample the pending bit around CVR so a wrap cannot make `now()` move
    // backwards before the delayed ISR advances TICK_BASE.
    let (remaining, pending_wrap) = loop {
        let pending_before = unsafe { core::ptr::read_volatile(SCB_ICSR) } & ICSR_PENDSTSET != 0;
        let remaining = unsafe { core::ptr::read_volatile(SYST_CVR as *const u32) } as u64;
        let pending_after = unsafe { core::ptr::read_volatile(SCB_ICSR) } & ICSR_PENDSTSET != 0;
        if pending_before == pending_after {
            break (remaining, pending_after);
        }
    };
    let elapsed_in_period = (SYSTICK_RELOAD as u64).saturating_sub(remaining);
    base + pending_wrap as u64 * TICKS_PER_MS + elapsed_in_period / HCLK_PER_TICK
}

/// Run a flash operation without losing monotonic time while IRQs are masked.
pub fn run_flash_operation<T>(operation: impl FnOnce() -> T) -> T {
    let mut output = None;
    cortex_m::interrupt::free(|_| {
        let before = now_locked();
        unsafe {
            core::ptr::write_volatile(SYST_CSR, 0);
            core::ptr::write_volatile(SCB_ICSR, ICSR_PENDSTCLR);
        }

        let rtc_before = phy6222_hal::sleep::rtc_count();
        output = Some(operation());
        let rtc_after = phy6222_hal::sleep::rtc_count();
        let rtc_elapsed = rtc_after.wrapping_sub(rtc_before) & 0x00ff_ffff;
        let elapsed_ticks = rtc_elapsed as u64 * TICK_HZ / phy6222_hal::sleep::RC32K_HZ as u64;
        TICK_BASE.store(before.saturating_add(elapsed_ticks), Ordering::Relaxed);

        unsafe {
            core::ptr::write_volatile(SYST_CVR, 0);
            core::ptr::write_volatile(SYST_CSR, CSR_CLKSOURCE | CSR_TICKINT | CSR_ENABLE);
        }
    });
    output.unwrap()
}
