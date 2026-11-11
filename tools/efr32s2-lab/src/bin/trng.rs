//! Gate: CRYPTOACC BA431 TRNG on EFR32MG22 (WSTK only).
//!
//! Runs the GSDK conditioning known-answer test and start-up health checks
//! through `efr32s2_hal::trng::Trng::new`, then prints one 16-byte random
//! block per second over VCOM together with a running bit balance. A
//! `TRNG FAIL` line means the KAT, start-up or continuous health test failed.
//! This is a lab check of the entropy path only; it does not certify the
//! source (no AIS-31/SP 800-90B evaluation is done here).

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::trng::Trng;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("trng");
    let mut trng = match Trng::new() {
        Ok(t) => {
            log!("TRNG init ok (KAT + startup passed)");
            t
        }
        Err(e) => {
            log!("TRNG FAIL init: {:?}", e);
            loop {
                shared::delay_ms(1000);
                lab.led.toggle();
            }
        }
    };
    let mut ones: u64 = 0;
    let mut bits: u64 = 0;
    let mut n = 0u32;
    loop {
        let mut block = [0u8; 16];
        match trng.fill(&mut block) {
            Ok(()) => {
                for b in block {
                    ones += u64::from(b.count_ones());
                }
                bits += 128;
                log!(
                    "rnd {} {:02x?} ones {}/{} ({} permille)",
                    n,
                    block,
                    ones,
                    bits,
                    ones * 1000 / bits
                );
            }
            Err(e) => log!("TRNG FAIL fill {}: {:?}", n, e),
        }
        n = n.wrapping_add(1);
        lab.led.toggle();
        shared::delay_ms(1000);
    }
}
