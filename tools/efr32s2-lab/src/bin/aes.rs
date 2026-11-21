//! Gate: hardware AES-128 engine (xG22 CRYPTOACC BA411E) through
//! `efr32s2_hal::crypto::AesEngine`.
//!
//! Runs the engine's two-vector self test, then the SP 800-38A ECB-AES128
//! vectors and a 1,000-block chained encryption (re-keyed every block, like
//! AES-MMO) whose final block is compared with the host RustCrypto `aes`
//! result, timing it with the HAL cycle counter (TIMER0 at HCLK). `AES PASS` / `AES FAIL ...` are the verdict
//! lines. Production images run the same self test at start-up
//! (`install_aes_engine`) before Zigbee starts.

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::crypto::AesEngine;

/// SP 800-38A F.1.1 ECB-AES128.Encrypt.
const KEY: [u8; 16] = [
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
];
const VECTORS: [([u8; 16], [u8; 16]); 4] = [
    (
        [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a,
        ],
        [
            0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60, 0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66,
            0xef, 0x97,
        ],
    ),
    (
        [
            0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf,
            0x8e, 0x51,
        ],
        [
            0xf5, 0xd3, 0xd5, 0x85, 0x03, 0xb9, 0x69, 0x9d, 0xe7, 0x85, 0x89, 0x5a, 0x96, 0xfd,
            0xba, 0xaf,
        ],
    ),
    (
        [
            0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a,
            0x52, 0xef,
        ],
        [
            0x43, 0xb1, 0xcd, 0x7f, 0x59, 0x8e, 0xce, 0x23, 0x88, 0x1b, 0x00, 0xe3, 0xed, 0x03,
            0x06, 0x88,
        ],
    ),
    (
        [
            0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c,
            0x37, 0x10,
        ],
        [
            0x7b, 0x0c, 0x78, 0x5e, 0x27, 0xe8, 0xad, 0x3f, 0x82, 0x23, 0x20, 0x71, 0x04, 0x72,
            0x5d, 0xd4,
        ],
    ),
];

/// Final block of the 1,000-round chain, computed on the host with the
/// RustCrypto `aes` crate.
const CHAIN_1000: [u8; 16] = [
    0xf8, 0x85, 0x1c, 0xb6, 0x94, 0xbc, 0x47, 0xf6, 0x1d, 0xb9, 0xde, 0xb5, 0x69, 0x57, 0x56, 0xa4,
];

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("aes");
    let verdict = run(lab.crypto);
    match verdict {
        Ok(()) => log!("AES PASS"),
        Err(what) => log!("AES FAIL {}", what),
    }
    loop {
        shared::delay_ms(if verdict.is_ok() { 1000 } else { 100 });
        lab.led.toggle();
    }
}

fn run(token: efr32s2_hal::crypto::Crypto) -> Result<(), &'static str> {
    let mut engine = AesEngine::new(token, AesEngine::DEFAULT_TIMEOUT_ITERATIONS).map_err(|e| {
        log!("AesEngine::new: {:?}", e);
        "new"
    })?;
    engine.self_test().map_err(|e| {
        log!("self_test: {:?}", e);
        "self-test"
    })?;
    log!("AES_KAT_PASS self-test (FIPS-197 C.1, zero vector)");
    for (i, (plain, cipher)) in VECTORS.iter().enumerate() {
        let mut block = *plain;
        engine.encrypt_block(&KEY, &mut block).map_err(|e| {
            log!("SP800-38A #{}: {:?}", i + 1, e);
            "engine error"
        })?;
        if block != *cipher {
            log!("SP800-38A #{}: got {:02x?}", i + 1, block);
            return Err("SP 800-38A mismatch");
        }
    }
    log!("SP 800-38A ECB-AES128 4/4 ok");
    // Chained: block_{n+1} = AES(key = block_n, block_n), 1,000 rounds from
    // the zero block; re-keys every request like AES-MMO does.
    let mut block = [0u8; 16];
    let start = efr32s2_hal::delay::cycles();
    for _ in 0..1000 {
        let key = block;
        engine.encrypt_block(&key, &mut block).map_err(|e| {
            log!("chain: {:?}", e);
            "engine error"
        })?;
    }
    let cycles = efr32s2_hal::delay::cycles().wrapping_sub(start);
    log!(
        "chain1000 {:02x?} ({} cycles/block incl. re-key)",
        block,
        cycles / 1000
    );
    if block != CHAIN_1000 {
        return Err("chain mismatch");
    }
    Ok(())
}
