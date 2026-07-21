#![no_std]
#![no_main]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/vectors.rs"]
mod vectors;

use core::mem::MaybeUninit;

use cortex_m as _;
use efr32mg1_hal::pm;

const ITERATIONS: u32 = 10;
const SLEEP_MS: u32 = 1_000;
const TOLERANCE_PERCENT: u32 = 10;
const CANARY_LEN: usize = 4;
const SEED_BSS: u32 = 0xC0FF_EE00;
const SEED_UNINIT: u32 = 0xC0FF_EE10;
const SEED_STACK: u32 = 0xC0FF_EE20;

static mut CANARY_BSS: [u32; CANARY_LEN] = [0; CANARY_LEN];
#[unsafe(link_section = ".uninit.diag_em2_canary")]
static mut CANARY_UNINIT: MaybeUninit<[u32; CANARY_LEN]> = MaybeUninit::uninit();

#[inline(always)]
unsafe fn canary_bss_mut() -> &'static mut [u32; CANARY_LEN] {
    unsafe { &mut *core::ptr::addr_of_mut!(CANARY_BSS) }
}

#[inline(always)]
unsafe fn canary_uninit_mut() -> &'static mut [u32; CANARY_LEN] {
    unsafe { &mut *core::ptr::addr_of_mut!(CANARY_UNINIT).cast::<[u32; CANARY_LEN]>() }
}

fn fill(seed: u32, slot: &mut [u32; CANARY_LEN]) {
    for (index, word) in slot.iter_mut().enumerate() {
        *word = pm::canary_pattern(seed, index);
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn RTCC() {
    pm::handle_interrupt();
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_large!("diag-em2");
    rtt_target::rprintln!(
        "[EFR32][diag-em2] BOOT lfrco_hz={} iterations={} sleep_ms={} tolerance_pct={}",
        pm::LFRCO_HZ,
        ITERATIONS,
        SLEEP_MS,
        TOLERANCE_PERCENT
    );

    let mut stack = [0; CANARY_LEN];
    fill(SEED_STACK, &mut stack);
    unsafe {
        fill(SEED_BSS, canary_bss_mut());
        fill(SEED_UNINIT, canary_uninit_mut());
    }
    if let Err(error) = pm::init() {
        rtt_target::rprintln!("[EFR32][diag-em2] RTCC_INIT_FAIL error={:?}", error);
        platform::halt()
    }
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::Rtcc);
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::Rtcc) };

    let requested = pm::ms_to_ticks(SLEEP_MS, pm::LFRCO_HZ);
    let mut failures = 0;
    for iteration in 1..=ITERATIONS {
        let before = pm::now();
        let (deadline, cause) = match pm::sleep_for_ticks(requested) {
            Ok(result) => result,
            Err(error) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-em2] iter={} SLEEP_FAIL error={:?}",
                    iteration,
                    error
                );
                failures += 1;
                continue;
            }
        };
        let elapsed = pm::elapsed_ticks(before, pm::now());
        let progressed = pm::progressed_within_tolerance(elapsed, requested, TOLERANCE_PERCENT);
        let bss = unsafe { pm::validate_canaries(SEED_BSS, canary_bss_mut()) };
        let uninit = unsafe { pm::validate_canaries(SEED_UNINIT, canary_uninit_mut()) };
        let stack_ok = pm::validate_canaries(SEED_STACK, &stack);
        let passed = matches!(cause, pm::WakeCause::RtccCompare)
            && progressed
            && bss.is_ok()
            && uninit.is_ok()
            && stack_ok.is_ok();
        failures += (!passed) as u32;
        rtt_target::rprintln!(
            "[EFR32][diag-em2] iter={} cause={:?} deadline=0x{:08X} elapsed_ticks={} \
             elapsed_ms={} progressed={} canary_bss={} canary_uninit={} canary_stack={} => {}",
            iteration,
            cause,
            deadline,
            elapsed,
            pm::ticks_to_ms(elapsed, pm::LFRCO_HZ),
            progressed,
            bss.is_ok(),
            uninit.is_ok(),
            stack_ok.is_ok(),
            if passed { "PASS" } else { "FAIL" }
        );
    }

    if failures == 0 {
        rtt_target::rprintln!(
            "[EFR32][diag-em2] PASS iterations={} failures=0",
            ITERATIONS
        );
        platform::led_on();
    } else {
        rtt_target::rprintln!(
            "[EFR32][diag-em2] FAIL iterations={} failures={}",
            ITERATIONS,
            failures
        );
    }
    platform::halt()
}
