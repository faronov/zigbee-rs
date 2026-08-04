#![no_std]
#![no_main]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use efr32mg1_hal::{
    crypto::{AesEngine, AesError},
    peripherals::Peripherals,
    pm,
};

const EM2_CYCLES: u32 = 5;
const SLEEP_MS: u32 = 500;
const REKEY_ITERATIONS: u32 = 32;

const VECTORS: [([u8; 16], [u8; 16], [u8; 16]); 2] = [
    (
        [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f,
        ],
        [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ],
        [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
            0xc5, 0x5a,
        ],
    ),
    (
        [0u8; 16],
        [0u8; 16],
        [
            0x66, 0xe9, 0x4b, 0xd4, 0xef, 0x8a, 0x2c, 0x3b, 0x88, 0x4c, 0xfa, 0x59, 0xca, 0x34,
            0x2b, 0x2e,
        ],
    ),
];

#[unsafe(no_mangle)]
pub extern "C" fn RTCC() {
    pm::handle_interrupt();
}

fn run_rekey_loop(engine: &mut AesEngine) -> Result<(), AesError> {
    for iteration in 0..REKEY_ITERATIONS {
        let (key, plaintext, ciphertext) = VECTORS[(iteration as usize) & 1];
        let mut block = plaintext;
        engine.encrypt_block(&key, &mut block)?;
        if block != ciphertext {
            return Err(AesError::KnownAnswerMismatch);
        }
    }
    Ok(())
}

fn crypto_fail(stage: &str, error: AesError) -> ! {
    rtt_target::rprintln!(
        "[EFR32][diag-crypto] FAIL stage={} error={:?}",
        stage,
        error
    );
    platform::halt()
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_large!("diag-crypto");
    rtt_target::rprintln!(
        "[EFR32][diag-crypto] BOOT em2_cycles={} sleep_ms={} rekey_iterations={}",
        EM2_CYCLES,
        SLEEP_MS,
        REKEY_ITERATIONS
    );

    if let Err(error) = pm::init() {
        rtt_target::rprintln!("[EFR32][diag-crypto] FAIL stage=rtcc-init error={:?}", error);
        platform::halt()
    }
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::Rtcc);
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::Rtcc) };

    let chip = Peripherals::take().unwrap_or_else(|| platform::halt());
    let mut engine = match AesEngine::new(chip.crypto, AesEngine::DEFAULT_TIMEOUT_ITERATIONS) {
        Ok(engine) => engine,
        Err(error) => crypto_fail("engine-init", error),
    };
    if let Err(error) = engine.self_test() {
        crypto_fail("startup-kat", error);
    }
    if let Err(error) = run_rekey_loop(&mut engine) {
        crypto_fail("startup-rekey", error);
    }
    rtt_target::rprintln!("[EFR32][diag-crypto] PRE_EM2_PASS");

    let sleep_ticks = pm::ms_to_ticks(SLEEP_MS, pm::LFRCO_HZ);
    for cycle in 1..=EM2_CYCLES {
        let before = pm::now();
        let (_, cause) = match pm::sleep_for_ticks(sleep_ticks) {
            Ok(result) => result,
            Err(error) => {
                rtt_target::rprintln!(
                    "[EFR32][diag-crypto] FAIL stage=em2-sleep cycle={} error={:?}",
                    cycle,
                    error
                );
                platform::halt()
            }
        };
        if !matches!(cause, pm::WakeCause::RtccCompare) {
            rtt_target::rprintln!(
                "[EFR32][diag-crypto] FAIL stage=em2-wake cycle={} cause={:?}",
                cycle,
                cause
            );
            platform::halt()
        }
        if let Err(error) = engine.self_test() {
            crypto_fail("post-em2-kat", error);
        }
        if let Err(error) = run_rekey_loop(&mut engine) {
            crypto_fail("post-em2-rekey", error);
        }
        let elapsed = pm::elapsed_ticks(before, pm::now());
        rtt_target::rprintln!(
            "[EFR32][diag-crypto] POST_EM2_PASS cycle={} elapsed_ticks={} elapsed_ms={}",
            cycle,
            elapsed,
            pm::ticks_to_ms(elapsed, pm::LFRCO_HZ)
        );
    }

    rtt_target::rprintln!(
        "[EFR32][diag-crypto] ALL_PASS em2_cycles={} blocks={}",
        EM2_CYCLES,
        (EM2_CYCLES + 1) * (2 + REKEY_ITERATIONS)
    );
    platform::led_on();
    platform::halt()
}
