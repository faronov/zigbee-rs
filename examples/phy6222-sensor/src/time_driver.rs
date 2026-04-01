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
use portable_atomic::{AtomicU32, Ordering};

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

/// HCLK cycles per Embassy tick (for sub-ms interpolation).
const HCLK_PER_TICK: u64 = (HCLK_HZ / 1_000_000) as u64; // 48

// ── SysTick register addresses (ARM standard) ──────────────────

const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;

// CSR bits
const CSR_ENABLE: u32 = 1 << 0;
const CSR_TICKINT: u32 = 1 << 1;
const CSR_CLKSOURCE: u32 = 1 << 2; // 1 = processor clock

// ── State ───────────────────────────────────────────────────────

/// Millisecond counter (incremented in SysTick ISR).
/// Wraps after ~49.7 days — handled by combining with epoch in `now()`.
static MS_COUNT: AtomicU32 = AtomicU32::new(0);

/// High 32 bits of the ms counter (incremented on MS_COUNT wrap).
static MS_EPOCH: AtomicU32 = AtomicU32::new(0);

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
        // Must read atomically: disable interrupts to prevent SysTick
        // from updating MS_COUNT between our reads.
        cortex_m::interrupt::free(|_| {
            let epoch = MS_EPOCH.load(Ordering::Relaxed) as u64;
            let ms = MS_COUNT.load(Ordering::Relaxed) as u64;
            let full_ms = (epoch << 32) | ms;

            // Sub-ms precision from SysTick current value register.
            // SysTick counts DOWN from RELOAD to 0.
            let remaining = unsafe { core::ptr::read_volatile(SYST_CVR as *const u32) } as u64;
            let elapsed_in_period = (SYSTICK_RELOAD as u64) - remaining;

            // Convert to Embassy ticks (1 MHz):
            // ms * 1000 + sub_ms_ticks
            // sub_ms_ticks = elapsed_cycles / cycles_per_tick
            let sub_ms_ticks = elapsed_in_period / HCLK_PER_TICK;

            full_ms * TICKS_PER_MS + sub_ms_ticks
        })
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
    // Increment millisecond counter
    let prev = MS_COUNT.load(Ordering::Relaxed);
    let next = prev.wrapping_add(1);
    MS_COUNT.store(next, Ordering::Relaxed);
    if next == 0 {
        let ep = MS_EPOCH.load(Ordering::Relaxed);
        MS_EPOCH.store(ep.wrapping_add(1), Ordering::Relaxed);
    }

    // Check alarm (coarse — ms resolution)
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
    static TIME_DRIVER: Phy6222TimeDriver = Phy6222TimeDriver::new()
);

/// Initialize the time driver. Call from main before starting async tasks.
pub fn init() {
    TIME_DRIVER.init();
}
