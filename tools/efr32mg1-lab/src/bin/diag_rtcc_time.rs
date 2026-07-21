#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/time_driver.rs"]
mod time_driver;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use efr32mg1_hal::pm;
use embassy_time::{Duration, Instant, Timer};

const ACTIVE_ITERATIONS: u32 = 10;
const ACTIVE_WAIT_MS: u64 = 200;
const EM2_ITERATIONS: u32 = 5;
const EM2_WAIT_MS: u32 = 500;
const CANARY_LEN: usize = 4;
const SEED_BSS: u32 = 0xC0FF_EE30;
const SEED_STACK: u32 = 0xC0FF_EE40;

static mut CANARY_BSS: [u32; CANARY_LEN] = [0; CANARY_LEN];

unsafe fn canary_bss_mut() -> &'static mut [u32; CANARY_LEN] {
    unsafe { &mut *core::ptr::addr_of_mut!(CANARY_BSS) }
}

fn fill(seed: u32, slot: &mut [u32; CANARY_LEN]) {
    for (index, word) in slot.iter_mut().enumerate() {
        *word = pm::canary_pattern(seed, index);
    }
}

#[embassy_executor::task]
async fn run(stack: [u32; CANARY_LEN]) -> ! {
    rtt_target::rprintln!(
        "[EFR32][diag-rtcc-time] BOOT lfrco_hz={} embassy_tick_hz={} \
         active_iterations={} active_wait_ms={} em2_iterations={} em2_wait_ms={}",
        pm::LFRCO_HZ,
        embassy_time::TICK_HZ,
        ACTIVE_ITERATIONS,
        ACTIVE_WAIT_MS,
        EM2_ITERATIONS,
        EM2_WAIT_MS
    );
    let mut failures = 0;

    for iteration in 1..=ACTIVE_ITERATIONS {
        let before = Instant::now();
        Timer::after(Duration::from_millis(ACTIVE_WAIT_MS)).await;
        let elapsed = (Instant::now() - before).as_millis() as u32;
        let passed = pm::progressed_within_tolerance(elapsed, ACTIVE_WAIT_MS as u32, 20);
        failures += (!passed) as u32;
        rtt_target::rprintln!(
            "[EFR32][diag-rtcc-time] active_iter={} elapsed_ms={} progressed={} => {}",
            iteration,
            elapsed,
            passed,
            if passed { "PASS" } else { "FAIL" }
        );
    }

    for iteration in 1..=EM2_ITERATIONS {
        let ticks = pm::ms_to_ticks(EM2_WAIT_MS, pm::LFRCO_HZ);
        let before = pm::now();
        match pm::sleep_for_ticks_polled(ticks) {
            Ok(deadline) => {
                let elapsed = pm::elapsed_ticks(before, pm::now());
                let progressed = pm::progressed_within_tolerance(elapsed, ticks, 10);
                let sleepdeep_cleared = !pm::sleepdeep_is_set();
                let bss = unsafe { pm::validate_canaries(SEED_BSS, canary_bss_mut()) };
                let stack_ok = pm::validate_canaries(SEED_STACK, &stack);
                let passed = progressed && sleepdeep_cleared && bss.is_ok() && stack_ok.is_ok();
                failures += (!passed) as u32;
                rtt_target::rprintln!(
                    "[EFR32][diag-rtcc-time] em2_iter={} deadline=0x{:08X} \
                     elapsed_ticks={} elapsed_ms={} progressed={} sleepdeep_cleared={} \
                     canary_bss={} canary_stack={} => {}",
                    iteration,
                    deadline,
                    elapsed,
                    pm::ticks_to_ms(elapsed, pm::LFRCO_HZ),
                    progressed,
                    sleepdeep_cleared,
                    bss.is_ok(),
                    stack_ok.is_ok(),
                    if passed { "PASS" } else { "FAIL" }
                );
            }
            Err(error) => {
                failures += 1;
                rtt_target::rprintln!(
                    "[EFR32][diag-rtcc-time] em2_iter={} SLEEP_FAIL error={:?}",
                    iteration,
                    error
                );
            }
        }
    }

    if failures == 0 {
        rtt_target::rprintln!(
            "[EFR32][diag-rtcc-time] PASS active_iterations={} em2_iterations={} failures=0",
            ACTIVE_ITERATIONS,
            EM2_ITERATIONS
        );
        platform::led_on();
    } else {
        rtt_target::rprintln!("[EFR32][diag-rtcc-time] FAIL failures={}", failures);
    }
    loop {
        Timer::after(Duration::from_secs(3_600)).await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_large!("diag-rtcc-time");
    time_driver::init();

    let mut stack = [0; CANARY_LEN];
    fill(SEED_STACK, &mut stack);
    unsafe { fill(SEED_BSS, canary_bss_mut()) };

    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(stack)))
}
