//! Hardware AES-128 (forward/encrypt only) for Zigbee CCM* and AES-MMO.
//!
//! Each Series 2 chip has a different crypto block, so the engine behind
//! `AesEngine` is selected by the chip feature:
//!
//! - **xG21** (`se`): the HSE Secure Engine. The host never touches the
//!   AES core; it posts an `AES_ENCRYPT | MODE_ECB` command with a plaintext
//!   key through the `SEMAILBOX_HOST` FIFO and the SE fetches key and block
//!   from RAM with its own DMA (GSDK 4.5 `sl_se_aes_crypt_ecb`,
//!   `em_se.c SE_executeCommand`). The same mailbox exists on xG24.
//! - **xG22** (`cryptoacc`): CRYPTOACC, a Silex BA411E AES core behind the
//!   CRYPTOMASTER scatter/gather DMA (GSDK 4.5 `sl_cryptoacc_library`
//!   `sx_blk_cipher_ecb` / `cryptodma_internal.c`). The xG22 VSE mailbox is
//!   not involved.
//!
//! Common contract (same as `efr32mg1-hal::crypto`):
//!
//! - one owner: `AesEngine::new` consumes the singleton [`Crypto`] token;
//! - every hardware wait is a bounded poll; no interrupts are used;
//! - errors are explicit ([`AesError`]), nothing panics, and output is only
//!   published after the engine reported success;
//! - after any engine fault the next request first re-runs the two
//!   known-answer vectors (`AesEngine::self_test`) before it may publish
//!   output again;
//! - key, plaintext and ciphertext copies in the DMA area are wiped after
//!   every request.
//!
//! The DMA area is a `static` owned through the singleton token: a request
//! that timed out may still be completed later by the engine, and that late
//! write must land in memory nobody else uses.
//!
//! Ownership conflicts:
//!
//! - **xG21 / xG24 SE mailbox**: the resident Gecko Bootloader's application
//!   interface (image verification, `bootloader_*` calls) and any other SE
//!   client use the same mailbox. They must only run sequentially with the
//!   AES engine, never during an AES request; the engine never runs from an
//!   interrupt, so a bootloader call made from thread mode between two
//!   requests is safe. A command that times out poisons the engine
//!   ([`AesError::Poisoned`]): the late SE response could otherwise be read
//!   as the answer to the next command.
//! - **xG22 CRYPTOACC**: the TRNG (`crate::trng`) uses the BA431 register
//!   block of the same peripheral but not the CRYPTOMASTER DMA. Both share
//!   the CRYPTOACC bus/AES/PK clock gates, which are reference-counted
//!   between the two users (`crate::clock::cryptoacc_acquire` /
//!   `cryptoacc_release`), so neither can gate the other's clocks.
//!
//! Both engines are thread-mode only; they are not reentrant and must not be
//! used from interrupt handlers.

#[cfg(feature = "efr32mg22")]
pub mod cryptoacc;
#[cfg(feature = "efr32mg21")]
pub mod se;

#[cfg(all(feature = "efr32mg22", target_arch = "arm", target_os = "none"))]
pub use cryptoacc::AesEngine;
#[cfg(all(feature = "efr32mg21", target_arch = "arm", target_os = "none"))]
pub use se::AesEngine;

/// Exclusive ownership of the chip's AES engine (xG21: the AES service of
/// the Secure Engine mailbox; xG22: the CRYPTOACC AES core and its DMA).
///
/// Created once by [`crate::peripherals::Peripherals::take`].
pub struct Crypto {
    _private: (),
}

impl Crypto {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// Errors from the hardware AES-128 engine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AesError {
    /// A zero poll budget would make every hardware operation fail.
    ZeroIterations,
    /// The engine did not complete within the bounded wait.
    Timeout,
    /// An on-silicon known-answer result did not match standard AES-128.
    KnownAnswerMismatch,
    /// An earlier unrecoverable fault left the engine unusable until reset.
    Poisoned,
    /// xG21/xG24: the SE mailbox flagged a TX or RX protocol error.
    MailboxError,
    /// xG21/xG24: the Secure Engine rejected the command; the value is the
    /// `SE_RESPONSE_*` code (`RX_HEADER[19:16]`), e.g. 7 = invalid parameter.
    SeResponse(u8),
    /// xG22: the BA411E reports no AES-128 ECB support (`BA411E_HW_CFG_1`).
    Unsupported,
    /// xG22: the CRYPTOMASTER fetcher or pusher reported a bus error.
    DmaError,
    /// xG22: data was left in the CRYPTOMASTER FIFOs after the transfer.
    FifoError,
}

/// FIPS-197 Appendix C.1 and the all-zero key/plaintext vector, used by
/// `AesEngine::self_test`: two different keys exercise re-keying.
pub(crate) const SELF_TEST_VECTORS: [([u8; 16], [u8; 16], [u8; 16]); 2] = [
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

/// Conservative bound for one hardware wait. A single SE AES command or
/// BA411E block completes in tens of microseconds; this bounds a dead
/// engine to roughly 100 ms at 80 MHz.
pub const DEFAULT_TIMEOUT_ITERATIONS: u32 = 2_000_000;

pub(crate) const fn validate_timeout_iterations(iterations: u32) -> Result<u32, AesError> {
    if iterations == 0 {
        Err(AesError::ZeroIterations)
    } else {
        Ok(iterations)
    }
}

/// Bounded poll: `Ok` as soon as `done` returns true.
pub(crate) fn poll(iterations: u32, mut done: impl FnMut() -> bool) -> Result<(), AesError> {
    for _ in 0..iterations {
        if done() {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(AesError::Timeout)
}

/// Wipe a DMA copy of key or data; volatile so the store is not elided.
pub(crate) fn wipe(buf: &mut [u8; 16]) {
    for byte in buf.iter_mut() {
        // SAFETY: `byte` is a valid, exclusive reference.
        unsafe { core::ptr::write_volatile(byte, 0) };
    }
}

/// Order the CPU's DMA-area stores before the MMIO write that starts the
/// engine, and the engine's DMA writes before the CPU reads the result.
#[inline(always)]
pub(crate) fn dma_fence() {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    cortex_m::asm::dsb();
}

/// Run the self-test vectors through `encrypt`.
pub(crate) fn run_self_test(
    mut encrypt: impl FnMut(&[u8; 16], &[u8; 16], &mut [u8; 16]) -> Result<(), AesError>,
) -> Result<(), AesError> {
    for (key, plaintext, ciphertext) in SELF_TEST_VECTORS.iter() {
        let mut output = [0u8; 16];
        encrypt(key, plaintext, &mut output)?;
        if &output != ciphertext {
            return Err(AesError::KnownAnswerMismatch);
        }
    }
    Ok(())
}

/// Host-test helpers shared by the engine models.
#[cfg(test)]
pub(crate) mod test_support {
    use zigbee_crypto::{Aes128Forward, AesKey, SoftwareAes128};

    /// Reference AES-128 for the register models.
    pub fn reference_encrypt(key: &[u8; 16], block: &[u8; 16]) -> [u8; 16] {
        let mut out = *block;
        let Ok(()) = SoftwareAes128::new(key).encrypt_block(&mut out);
        out
    }

    /// Fake 32-bit bus addresses for host pointers: the models see only
    /// `u32` words, exactly as the hardware does.
    pub struct AddressMap {
        base: usize,
        len: usize,
    }

    pub const FAKE_RAM: u32 = 0x2000_1000;

    impl AddressMap {
        pub fn new<T>(area: *const T) -> Self {
            Self {
                base: area as usize,
                len: core::mem::size_of::<T>(),
            }
        }
        pub fn to_bus(&self, ptr: *const u8) -> u32 {
            let offset = (ptr as usize)
                .checked_sub(self.base)
                .filter(|o| *o < self.len)
                .expect("DMA pointer outside the DMA area");
            FAKE_RAM + offset as u32
        }
        pub fn to_host(&self, address: u32) -> *mut u8 {
            let offset = address
                .checked_sub(FAKE_RAM)
                .map(|o| o as usize)
                .filter(|o| *o < self.len)
                .expect("bus address outside the DMA area");
            (self.base + offset) as *mut u8
        }
    }

    /// `ForwardAesProvider` over a block-encrypt closure, for `kat::run_all`.
    pub struct Provider<F>(pub F);

    pub struct Cipher<'a, F> {
        encrypt: &'a mut F,
        key: AesKey,
    }

    impl<F, E> zigbee_crypto::ForwardAesProvider for Provider<F>
    where
        F: FnMut(&[u8; 16], &mut [u8; 16]) -> Result<(), E>,
    {
        fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
            Cipher {
                encrypt: &mut self.0,
                key: *key,
            }
        }
    }

    impl<F, E> Aes128Forward for Cipher<'_, F>
    where
        F: FnMut(&[u8; 16], &mut [u8; 16]) -> Result<(), E>,
    {
        type Error = E;
        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), E> {
            (self.encrypt)(&self.key, block)
        }
    }
}
