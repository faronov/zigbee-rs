//! CC2340R5 entropy for [`crate::PlatformServices::fill_random`].
//!
//! The CC23x0 has no general-purpose TRNG. This module reproduces the
//! structure of TI's `RNGLPF3RF` driver (`ti/drivers/rng/RNGLPF3RF.c`) in
//! Rust:
//!
//! ```text
//! radio IF-ADC noise (noise.rs, 1024 words = 3072 I + 3072 Q codes)
//!   -> health tests on every code (repetition count, adaptive proportion,
//!      bimodal proportion, word format)            SP 800-90B 4.4 style
//!   -> CBC-MAC-AES-128 over each half (fixed key)  SP 800-90B 3.1.5.1.1
//!   -> 256-bit seed -> AES-128 CTR_DRBG, no df      SP 800-90A 10.2.1
//!   -> fill_random()
//! ```
//!
//! Every AES block (conditioning, DRBG and its known-answer test) runs on
//! the `LAES` engine through the same [`zigbee_crypto::ForwardAesProvider`]
//! owner that serves CCM*; there is no second owner of the core.
//!
//! # Fail closed
//!
//! Until a capture has passed the startup health tests nothing is returned.
//! A failed capture (radio timeout, health test, AES error, known-answer
//! mismatch) leaves the source in [`EntropyState::Failed`]: an existing DRBG
//! is dropped (its state wiped) and every request fails, without touching
//! the caller's buffer. A failure in the middle of a request zeroises the
//! whole buffer. A new [`EntropyState::seed`] repeats the startup tests.
//!
//! # Reseeding and prediction resistance
//!
//! The device is seeded at boot (before the IEEE radio is brought up, as
//! TI requires for `RNG_init()`). The DRBG asks for a reseed after
//! [`RESEED_INTERVAL`] requests (2^20, far below SP 800-90A's 2^48; a
//! Zigbee end device makes a handful of requests per boot); the PHY then
//! captures fresh noise at the next request made while the radio is idle,
//! and fails closed until it could. Prediction resistance (fresh noise on
//! every request) is not offered: a capture stops the IEEE radio and
//! reloads its firmware, which the protocol stack cannot afford per request.
//! The noise is not an always-available live source, so each capture runs
//! the full health tests before it is used.

use zigbee_crypto::ctr_drbg::{self, CtrDrbg, DrbgError, SEED_LEN};
use zigbee_crypto::{Aes128Forward, ForwardAesProvider, zeroize};

use super::noise::{self, NoiseError};

/// Words per capture: the full 4 KiB PBE RAM, i.e. 3072 codes per stream,
/// above SP 800-90B's 1024-sample startup test and TI's 152-word minimum.
pub const NOISE_WORDS: usize = noise::MAX_WORDS;
const HALF_WORDS: usize = NOISE_WORDS / 2;
const _: () = assert!(
    (HALF_WORDS * 4).is_multiple_of(16),
    "each half is whole AES blocks"
);

/// Generate requests between reseeds.
pub const RESEED_INTERVAL: u64 = 1 << 20;

/// SP 800-90B 3.1.5.1.1 vetted conditioning: CBC-MAC with AES-128 under a
/// fixed, non-secret key (TI uses a SysConfig constant the same way). The
/// key only has to be fixed and independent of the noise; it is not a
/// secret and gives no security if disclosed.
const CONDITIONING_KEY: [u8; 16] = [
    0xbe, 0xde, 0xe4, 0xac, 0xf3, 0x19, 0x0e, 0x31, 0x81, 0xda, 0x68, 0x49, 0xe3, 0x05, 0x7d, 0xd2,
];

/// SP 800-90B 4.4.2 adaptive proportion window for non-binary samples.
pub const APT_WINDOW: u16 = 512;

/// Health-test cutoffs, per 5-bit code stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HealthCutoffs {
    /// Fail when the same code repeats this many times in a row.
    pub repetition: u16,
    /// Fail when one code value occurs this many times in an
    /// [`APT_WINDOW`] window.
    pub proportion: u16,
    /// Fail when the two most common code values together occur this many
    /// times in one window.
    pub bimodal: u16,
}

/// Cutoffs for an assumed min-entropy of `H = 0.5` bit per 5-bit code and a
/// false-alarm probability of `alpha = 2^-20` per test:
///
/// * repetition count (SP 800-90B 4.4.1): `C = 1 + ceil(20 / H) = 41`;
/// * adaptive proportion (4.4.2, applied to every code value, not only the
///   first of the window): `C = 1 + CRITBINOM(512, 2^-H, 1 - alpha) = 410`;
/// * top-two proportion, a developer-defined test (4.5) for a source that
///   collapses onto two codes (which the two approved tests can miss at this
///   `H`): `C = 1 + CRITBINOM(512, 0.75, 1 - alpha) = 430`.
///
/// Characterisation on the LP-EM-CC2340R5 (256 captures, 786,432 codes per
/// stream; lag predictors over 64 captures per lag; see the platform guide):
/// 20 code values occur, most-common-value estimate 1.74 bit/code (I) and
/// 1.71 (Q), lag-1 predictor estimate 1.05 (I) and 1.16 (Q); the longest run
/// was 20 codes, the largest one-code window count 199 and the largest
/// top-two window count 255. `H = 0.5` is therefore about half of the
/// most pessimistic estimate. It is an engineering bound, not an
/// SP 800-90B entropy assessment. Each 128-bit conditioning output absorbs
/// 3072 codes, i.e. 1536 bits at the assumed `H`, above the
/// `n_out + 64 = 192` bits SP 800-90C asks for a full-entropy output.
pub const CUTOFFS: HealthCutoffs = HealthCutoffs {
    repetition: 41,
    proportion: 410,
    bimodal: 430,
};

/// Code stream of a noise word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stream {
    /// In-phase codes (bits 30:16).
    I,
    /// Quadrature codes (bits 14:0).
    Q,
}

/// A failed health test. The capture is discarded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthFailure {
    /// Bit 15 or 31 was set: not an S2R `ADCDIG` word.
    Format,
    /// SP 800-90B 4.4.1 repetition count test.
    RepetitionCount(Stream),
    /// SP 800-90B 4.4.2 adaptive proportion test (every code value).
    AdaptiveProportion(Stream),
    /// The two most common code values dominate one window (the
    /// developer-defined form of TI's bimodal test).
    Bimodal(Stream),
}

/// Why no entropy is available.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntropyError {
    /// [`super::Cc2340RadioPhy::seed_entropy`] has not succeeded yet.
    Unseeded,
    /// No validated `LAES` engine is installed.
    AesUnavailable,
    /// The IEEE radio is in use; a capture would destroy its state.
    RadioBusy,
    /// The ADC-noise capture failed.
    Noise(NoiseError),
    /// A health test rejected the capture.
    Health(HealthFailure),
    /// The `LAES` engine failed during conditioning or generation.
    Cipher,
    /// The CTR_DRBG known-answer test produced a wrong result.
    KnownAnswerMismatch,
    /// The DRBG reached its reseed interval and no capture was possible yet.
    ReseedRequired,
}

/// Run-time state of the source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntropyDisposition {
    /// Not seeded yet: `fill_random` fails with `MacError::Unsupported`.
    Unseeded,
    /// Seeded from health-tested radio ADC noise (CBC-MAC conditioned);
    /// requests are served by AES-128 CTR_DRBG on the `LAES` engine.
    RadioNoiseCtrDrbg,
    /// The last capture failed; `fill_random` fails closed and never
    /// returns deterministic bytes.
    FailedClosed(EntropyError),
}

/// Continuous tests on one code stream.
struct StreamTest {
    last: u8,
    run: u16,
    window: u16,
    counts: [u16; 32],
}

impl StreamTest {
    const fn new() -> Self {
        Self {
            last: u8::MAX,
            run: 0,
            window: 0,
            counts: [0; 32],
        }
    }

    fn feed(
        &mut self,
        code: u8,
        cutoffs: &HealthCutoffs,
        stream: Stream,
    ) -> Result<(), HealthFailure> {
        let code = code & 0x1F;
        if code == self.last {
            self.run += 1;
            if self.run >= cutoffs.repetition {
                return Err(HealthFailure::RepetitionCount(stream));
            }
        } else {
            self.last = code;
            self.run = 1;
        }

        let count = &mut self.counts[usize::from(code)];
        *count += 1;
        if *count >= cutoffs.proportion {
            return Err(HealthFailure::AdaptiveProportion(stream));
        }
        self.window += 1;
        if self.window == APT_WINDOW {
            let (mut first, mut second) = (0, 0);
            for &count in &self.counts {
                if count > first {
                    second = first;
                    first = count;
                } else if count > second {
                    second = count;
                }
            }
            if first + second >= cutoffs.bimodal {
                return Err(HealthFailure::Bimodal(stream));
            }
            self.counts = [0; 32];
            self.window = 0;
        }
        Ok(())
    }
}

/// SP 800-90B-style health tests over a capture, fed word by word.
pub struct HealthTests {
    cutoffs: HealthCutoffs,
    i: StreamTest,
    q: StreamTest,
}

impl HealthTests {
    pub const fn new(cutoffs: HealthCutoffs) -> Self {
        Self {
            cutoffs,
            i: StreamTest::new(),
            q: StreamTest::new(),
        }
    }

    /// One S2R word: Q codes in bits 4:0, 9:5, 14:10 and I codes in bits
    /// 20:16, 25:21, 30:26, oldest first (`RCL_CMD_ADC_NOISE_GET`).
    pub fn feed_word(&mut self, word: u32) -> Result<(), HealthFailure> {
        if word & 0x8000_8000 != 0 {
            return Err(HealthFailure::Format);
        }
        for shift in [0, 5, 10] {
            self.q
                .feed((word >> shift) as u8, &self.cutoffs, Stream::Q)?;
            self.i
                .feed((word >> (16 + shift)) as u8, &self.cutoffs, Stream::I)?;
        }
        Ok(())
    }
}

/// CBC-MAC accumulator (zero IV) over little-endian noise words.
struct CbcMac {
    state: [u8; 16],
    filled: usize,
}

impl CbcMac {
    const fn new() -> Self {
        Self {
            state: [0; 16],
            filled: 0,
        }
    }

    fn absorb<C: Aes128Forward>(&mut self, cipher: &mut C, word: u32) -> Result<(), C::Error> {
        for (offset, byte) in word.to_le_bytes().into_iter().enumerate() {
            self.state[self.filled + offset] ^= byte;
        }
        self.filled += 4;
        if self.filled == 16 {
            self.filled = 0;
            cipher.encrypt_block(&mut self.state)?;
        }
        Ok(())
    }
}

impl Drop for CbcMac {
    fn drop(&mut self) {
        zeroize(&mut self.state);
    }
}

/// Source of raw noise words. The target implementation is
/// [`LrfNoise`]; host tests inject synthetic noise.
pub(crate) trait NoiseSource {
    /// Whether this image can capture noise at all (a constant, so an image
    /// without the SDK noise configuration drops the whole seed path).
    fn available(&self) -> bool {
        true
    }

    /// Capture `words` words and hand them to `consume` in order, stopping
    /// at its first error. The samples are wiped and the hardware released
    /// before returning.
    fn capture<E>(
        &mut self,
        words: usize,
        consume: impl FnMut(usize, u32) -> Result<(), E>,
    ) -> Result<Result<(), E>, NoiseError>;
}

/// The LRF ADC-noise capture of [`noise`].
pub(crate) struct LrfNoise;

impl NoiseSource for LrfNoise {
    fn available(&self) -> bool {
        noise::available()
    }

    #[cfg(target_os = "none")]
    fn capture<E>(
        &mut self,
        words: usize,
        mut consume: impl FnMut(usize, u32) -> Result<(), E>,
    ) -> Result<Result<(), E>, NoiseError> {
        let capture = noise::capture(words)?;
        let mut result = Ok(());
        for index in 0..words {
            result = consume(index, capture.word(index));
            if result.is_err() {
                break;
            }
        }
        drop(capture);
        Ok(result)
    }

    #[cfg(not(target_os = "none"))]
    fn capture<E>(
        &mut self,
        words: usize,
        _consume: impl FnMut(usize, u32) -> Result<(), E>,
    ) -> Result<Result<(), E>, NoiseError> {
        noise::validate_words(words)?;
        Err(NoiseError::Unavailable)
    }
}

enum Stop {
    Health(HealthFailure),
    Cipher,
}

/// Capture, test and condition one seed.
fn conditioned_seed<P: ForwardAesProvider, N: NoiseSource>(
    aes: &mut P,
    noise: &mut N,
    cutoffs: &HealthCutoffs,
) -> Result<[u8; SEED_LEN], EntropyError> {
    let mut health = HealthTests::new(*cutoffs);
    let mut first = CbcMac::new();
    let mut second = CbcMac::new();
    let outcome = {
        let mut cipher = aes.forward_cipher(&CONDITIONING_KEY);
        noise.capture(NOISE_WORDS, |index, word| {
            health.feed_word(word).map_err(Stop::Health)?;
            let mac = if index < HALF_WORDS {
                &mut first
            } else {
                &mut second
            };
            mac.absorb(&mut cipher, word).map_err(|_| Stop::Cipher)
        })
    };
    match outcome {
        Err(error) => Err(EntropyError::Noise(error)),
        Ok(Err(Stop::Health(failure))) => Err(EntropyError::Health(failure)),
        Ok(Err(Stop::Cipher)) => Err(EntropyError::Cipher),
        Ok(Ok(())) => {
            let mut seed = [0u8; SEED_LEN];
            seed[..16].copy_from_slice(&first.state);
            seed[16..].copy_from_slice(&second.state);
            Ok(seed)
        }
    }
}

/// The DRBG and its health state; owned by the PHY next to the `LAES`
/// engine.
pub(crate) enum EntropyState {
    Unseeded,
    Seeded(CtrDrbg),
    Failed(EntropyError),
}

impl EntropyState {
    pub(crate) const fn new() -> Self {
        Self::Unseeded
    }

    pub(crate) fn disposition(&self) -> EntropyDisposition {
        match self {
            Self::Unseeded => EntropyDisposition::Unseeded,
            Self::Seeded(_) => EntropyDisposition::RadioNoiseCtrDrbg,
            Self::Failed(error) => EntropyDisposition::FailedClosed(*error),
        }
    }

    pub(crate) fn reseed_due(&self) -> bool {
        matches!(self, Self::Seeded(drbg) if drbg.reseed_required())
    }

    /// Run the DRBG known-answer test, capture and condition fresh noise,
    /// then instantiate (or reseed) the DRBG. Any failure wipes the DRBG.
    pub(crate) fn seed<P: ForwardAesProvider, N: NoiseSource>(
        &mut self,
        aes: &mut P,
        noise: &mut N,
        cutoffs: &HealthCutoffs,
        personalization: &[u8; SEED_LEN],
    ) -> Result<(), EntropyError> {
        let result = self.try_seed(aes, noise, cutoffs, personalization);
        if let Err(error) = result {
            *self = Self::Failed(error);
        }
        result
    }

    fn try_seed<P: ForwardAesProvider, N: NoiseSource>(
        &mut self,
        aes: &mut P,
        noise: &mut N,
        cutoffs: &HealthCutoffs,
        personalization: &[u8; SEED_LEN],
    ) -> Result<(), EntropyError> {
        if !noise.available() {
            return Err(EntropyError::Noise(NoiseError::Unavailable));
        }
        ctr_drbg::self_test::run(aes).map_err(|error| match error {
            ctr_drbg::self_test::SelfTestError::Mismatch => EntropyError::KnownAnswerMismatch,
            ctr_drbg::self_test::SelfTestError::Drbg(_) => EntropyError::Cipher,
        })?;
        let mut seed = conditioned_seed(aes, noise, cutoffs)?;
        let result = match self {
            Self::Seeded(drbg) => drbg.reseed(aes, &seed, None),
            _ => CtrDrbg::instantiate(aes, &seed, Some(personalization), RESEED_INTERVAL)
                .map(|drbg| *self = Self::Seeded(drbg)),
        };
        zeroize(&mut seed);
        result.map_err(|_| EntropyError::Cipher)
    }

    /// Fill `output` from the DRBG.
    ///
    /// Unseeded/failed: `output` is untouched. A DRBG error zeroises
    /// `output`; a cipher error also fails the source closed.
    pub(crate) fn fill<P: ForwardAesProvider>(
        &mut self,
        aes: &mut P,
        output: &mut [u8],
    ) -> Result<(), EntropyError> {
        let drbg = match self {
            Self::Seeded(drbg) => drbg,
            Self::Unseeded => return Err(EntropyError::Unseeded),
            Self::Failed(error) => return Err(*error),
        };
        if drbg.reseed_required() {
            return Err(EntropyError::ReseedRequired);
        }
        let mut result = Ok(());
        for chunk in output.chunks_mut(ctr_drbg::MAX_REQUEST_BYTES) {
            result = drbg.generate(aes, chunk, None);
            if result.is_err() {
                break;
            }
        }
        match result {
            Ok(()) => Ok(()),
            Err(error) => {
                zeroize(output);
                match error {
                    DrbgError::ReseedRequired => Err(EntropyError::ReseedRequired),
                    _ => {
                        *self = Self::Failed(EntropyError::Cipher);
                        Err(EntropyError::Cipher)
                    }
                }
            }
        }
    }
}

/// Raw words of one capture, for the entropy lab binary only. The samples
/// are the DRBG's entropy input and must never be exposed by a product.
#[cfg(feature = "cc2340-entropy-lab")]
pub(crate) fn raw_capture<N: NoiseSource>(
    noise: &mut N,
    output: &mut [u32],
) -> Result<(), EntropyError> {
    match noise.capture(output.len(), |index, word| {
        output[index] = word;
        Ok::<(), ()>(())
    }) {
        Ok(_) => Ok(()),
        Err(error) => Err(EntropyError::Noise(error)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_crypto::{AesKey, SoftwareAes128, SoftwareAesProvider};

    /// Deterministic synthetic noise.
    struct FakeNoise<F: FnMut(usize) -> u32> {
        word: F,
        captures: usize,
        fail: Option<NoiseError>,
    }

    impl<F: FnMut(usize) -> u32> NoiseSource for FakeNoise<F> {
        fn capture<E>(
            &mut self,
            words: usize,
            mut consume: impl FnMut(usize, u32) -> Result<(), E>,
        ) -> Result<Result<(), E>, NoiseError> {
            noise::validate_words(words)?;
            if let Some(error) = self.fail {
                return Err(error);
            }
            self.captures += 1;
            for index in 0..words {
                let word = (self.word)(index + self.captures * words);
                if let Err(error) = consume(index, word) {
                    return Ok(Err(error));
                }
            }
            Ok(Ok(()))
        }
    }

    fn fake<F: FnMut(usize) -> u32>(word: F) -> FakeNoise<F> {
        FakeNoise {
            word,
            captures: 0,
            fail: None,
        }
    }

    /// xorshift32: uniformly distributed 5-bit codes.
    fn uniform_word(seed: usize) -> u32 {
        let mut x = (seed as u32).wrapping_mul(0x9E37_79B9) | 1;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        x & 0x7FFF_7FFF
    }

    fn pack(codes: [u8; 6]) -> u32 {
        let [q0, q1, q2, i0, i1, i2] = codes.map(u32::from);
        q0 | (q1 << 5) | (q2 << 10) | (i0 << 16) | (i1 << 21) | (i2 << 26)
    }

    fn run_health(mut word: impl FnMut(usize) -> u32) -> Result<(), HealthFailure> {
        let mut health = HealthTests::new(CUTOFFS);
        for index in 0..NOISE_WORDS {
            health.feed_word(word(index))?;
        }
        Ok(())
    }

    const PERSONALIZATION: [u8; SEED_LEN] = [0x5A; SEED_LEN];

    #[test]
    fn uniform_noise_passes_every_health_test() {
        assert_eq!(run_health(uniform_word), Ok(()));
    }

    /// Code histogram measured on the LP-EM-CC2340R5 (I stream, 786,432
    /// codes): i.i.d. samples from it must not raise a false alarm.
    const MEASURED_I_HISTOGRAM: [(u8, u32); 20] = [
        (1, 2668),
        (2, 425),
        (4, 3747),
        (5, 4759),
        (7, 69056),
        (8, 42970),
        (9, 2963),
        (11, 3429),
        (12, 1392),
        (14, 2334),
        (15, 72685),
        (17, 39123),
        (19, 5807),
        (20, 108429),
        (21, 4879),
        (23, 104965),
        (24, 41642),
        (25, 40953),
        (26, 234181),
        (18, 25),
    ];

    #[test]
    fn measured_code_distribution_passes() {
        let total: u32 = MEASURED_I_HISTOGRAM.iter().map(|&(_, count)| count).sum();
        let mut x = 0x1234_5678u32;
        let mut sample = move || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            let mut pick = x % total;
            for &(code, count) in &MEASURED_I_HISTOGRAM {
                if pick < count {
                    return code;
                }
                pick -= count;
            }
            unreachable!()
        };
        for _ in 0..16 {
            assert_eq!(
                run_health(|_| pack(core::array::from_fn(|_| sample()))),
                Ok(())
            );
        }
    }

    #[test]
    fn stuck_noise_fails_the_repetition_count_test() {
        assert_eq!(
            run_health(|_| pack([3; 6])),
            Err(HealthFailure::RepetitionCount(Stream::Q))
        );
        // Only the I stream stuck (Q varies).
        assert_eq!(
            run_health(|index| pack([(index % 32) as u8, ((index + 7) % 32) as u8, 9, 4, 4, 4])),
            Err(HealthFailure::RepetitionCount(Stream::I))
        );
        // All-zero buffer (an S2R that never wrote).
        assert_eq!(
            run_health(|_| 0),
            Err(HealthFailure::RepetitionCount(Stream::Q))
        );
    }

    #[test]
    fn biased_noise_fails_the_adaptive_proportion_test() {
        // 7 of every 8 codes are 0x11, never more than 7 in a row.
        let mut counter = 0usize;
        let biased = move |_| {
            let mut codes = [0u8; 6];
            for code in codes.iter_mut() {
                counter += 1;
                *code = if counter.is_multiple_of(8) {
                    (counter / 8 % 31) as u8
                } else {
                    0x11
                };
            }
            pack(codes)
        };
        assert!(matches!(
            run_health(biased),
            Err(HealthFailure::AdaptiveProportion(_))
        ));
    }

    #[test]
    fn bimodal_noise_fails_the_bimodal_test() {
        // Two codes, ~45 % each, short runs; the rest spread over others.
        let mut x = 1u32;
        let bimodal = move |_| {
            let mut codes = [0u8; 6];
            for code in codes.iter_mut() {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                *code = match x % 20 {
                    0..=8 => 0x05,
                    9..=17 => 0x1A,
                    other => other as u8,
                };
            }
            pack(codes)
        };
        assert!(matches!(
            run_health(bimodal),
            Err(HealthFailure::Bimodal(_))
        ));
    }

    #[test]
    fn reserved_bits_fail_the_format_check() {
        assert_eq!(
            run_health(|index| uniform_word(index) | 0x0000_8000),
            Err(HealthFailure::Format)
        );
        assert_eq!(
            run_health(|index| uniform_word(index) | 0x8000_0000),
            Err(HealthFailure::Format)
        );
    }

    /// Software CBC-MAC over the little-endian words, independent of
    /// `CbcMac`.
    fn reference_cbc_mac(words: &[u32]) -> [u8; 16] {
        let mut cipher = SoftwareAes128::new(&CONDITIONING_KEY);
        let mut state = [0u8; 16];
        for block in words.chunks_exact(4) {
            for (index, word) in block.iter().enumerate() {
                for (offset, byte) in word.to_le_bytes().into_iter().enumerate() {
                    state[4 * index + offset] ^= byte;
                }
            }
            cipher.encrypt_block(&mut state).unwrap();
        }
        state
    }

    #[test]
    fn conditioning_is_deterministic_cbc_mac_of_each_half() {
        let words: [u32; NOISE_WORDS] = core::array::from_fn(|i| uniform_word(NOISE_WORDS + i));
        let mut expected = [0u8; SEED_LEN];
        expected[..16].copy_from_slice(&reference_cbc_mac(&words[..HALF_WORDS]));
        expected[16..].copy_from_slice(&reference_cbc_mac(&words[HALF_WORDS..]));

        let mut aes = SoftwareAesProvider::new();
        let first = conditioned_seed(&mut aes, &mut fake(uniform_word), &CUTOFFS).unwrap();
        assert_eq!(first, expected);
        // Same noise, same seed; different noise, different seed.
        let again = conditioned_seed(&mut aes, &mut fake(uniform_word), &CUTOFFS).unwrap();
        assert_eq!(again, first);
        let other =
            conditioned_seed(&mut aes, &mut fake(|i| uniform_word(i + 1)), &CUTOFFS).unwrap();
        assert_ne!(other, first);
    }

    #[test]
    fn conditioning_vector_from_sp800_38a() {
        // SP 800-38A F.2.1 CBC-AES128.Encrypt, key 2b7e1516..., IV
        // 000102...0f: CBC-MAC with a zero IV of (P1 ^ IV) || P2 equals C2.
        // Checks the absorb order (little-endian words, block chaining)
        // against a published vector.
        let key: AesKey = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ];
        let mut p1 = [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a,
        ];
        for (byte, iv) in p1.iter_mut().zip(0u8..) {
            *byte ^= iv;
        }
        let p2: [u8; 16] = [
            0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf,
            0x8e, 0x51,
        ];
        let c2: [u8; 16] = [
            0x50, 0x86, 0xcb, 0x9b, 0x50, 0x72, 0x19, 0xee, 0x95, 0xdb, 0x11, 0x3a, 0x91, 0x76,
            0x78, 0xb2,
        ];
        let mut cipher = SoftwareAes128::new(&key);
        let mut mac = CbcMac::new();
        for block in [p1, p2] {
            for word in block.chunks_exact(4) {
                mac.absorb(&mut cipher, u32::from_le_bytes(word.try_into().unwrap()))
                    .unwrap();
            }
        }
        assert_eq!(mac.state, c2);
    }

    #[test]
    fn unseeded_source_fails_closed_without_touching_output() {
        let mut state = EntropyState::new();
        let mut output = [0xA5u8; 8];
        assert_eq!(
            state.fill(&mut SoftwareAesProvider::new(), &mut output),
            Err(EntropyError::Unseeded)
        );
        assert_eq!(output, [0xA5; 8]);
        assert_eq!(state.disposition(), EntropyDisposition::Unseeded);
    }

    #[test]
    fn healthy_noise_seeds_and_serves_requests() {
        let mut aes = SoftwareAesProvider::new();
        let mut state = EntropyState::new();
        state
            .seed(
                &mut aes,
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION,
            )
            .unwrap();
        assert_eq!(state.disposition(), EntropyDisposition::RadioNoiseCtrDrbg);
        let mut a = [0u8; 32];
        let mut b = [0u8; 32];
        state.fill(&mut aes, &mut a).unwrap();
        state.fill(&mut aes, &mut b).unwrap();
        assert_ne!(a, [0; 32]);
        assert_ne!(a, b);
    }

    #[test]
    fn failed_health_tests_fail_closed_and_wipe_an_existing_drbg() {
        let mut aes = SoftwareAesProvider::new();
        let mut state = EntropyState::new();
        state
            .seed(
                &mut aes,
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION,
            )
            .unwrap();
        // A reseed from stuck noise drops the DRBG.
        assert_eq!(
            state.seed(
                &mut aes,
                &mut fake(|_| pack([7; 6])),
                &CUTOFFS,
                &PERSONALIZATION
            ),
            Err(EntropyError::Health(HealthFailure::RepetitionCount(
                Stream::Q
            )))
        );
        let failed = EntropyDisposition::FailedClosed(EntropyError::Health(
            HealthFailure::RepetitionCount(Stream::Q),
        ));
        assert_eq!(state.disposition(), failed);
        let mut output = [0xA5u8; 16];
        for _ in 0..3 {
            assert!(state.fill(&mut aes, &mut output).is_err());
            assert_eq!(output, [0xA5; 16]);
        }
        // Recovery requires a capture that passes the startup tests again.
        state
            .seed(
                &mut aes,
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION,
            )
            .unwrap();
        state.fill(&mut aes, &mut output).unwrap();
    }

    #[test]
    fn capture_errors_fail_closed() {
        let mut aes = SoftwareAesProvider::new();
        let mut state = EntropyState::new();
        let mut noise = fake(uniform_word);
        noise.fail = Some(NoiseError::PllCalibrationTimeout);
        assert_eq!(
            state.seed(&mut aes, &mut noise, &CUTOFFS, &PERSONALIZATION),
            Err(EntropyError::Noise(NoiseError::PllCalibrationTimeout))
        );
        let mut output = [1u8; 4];
        assert_eq!(
            state.fill(&mut aes, &mut output),
            Err(EntropyError::Noise(NoiseError::PllCalibrationTimeout))
        );
        assert_eq!(output, [1; 4]);
    }

    #[test]
    fn host_lrf_noise_is_unavailable() {
        let mut aes = SoftwareAesProvider::new();
        let mut state = EntropyState::new();
        assert_eq!(
            state.seed(&mut aes, &mut LrfNoise, &CUTOFFS, &PERSONALIZATION),
            Err(EntropyError::Noise(NoiseError::Unavailable))
        );
    }

    /// Provider whose cipher fails after a block budget.
    struct Budget(usize);
    struct BudgetCipher<'a>(SoftwareAes128, &'a mut usize);
    impl Aes128Forward for BudgetCipher<'_> {
        type Error = ();
        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), ()> {
            if *self.1 == 0 {
                return Err(());
            }
            *self.1 -= 1;
            self.0.encrypt_block(block).map_err(|_| ())
        }
    }
    impl ForwardAesProvider for Budget {
        fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
            BudgetCipher(SoftwareAes128::new(key), &mut self.0)
        }
    }

    #[test]
    fn engine_failures_fail_closed_and_zeroise_output() {
        // Fails inside the DRBG known-answer test.
        let mut state = EntropyState::new();
        assert_eq!(
            state.seed(
                &mut Budget(3),
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION
            ),
            Err(EntropyError::Cipher)
        );
        // KAT (2 + 4 + 2 + 4 + 2 = 14 blocks) passes, conditioning fails.
        let mut state = EntropyState::new();
        assert_eq!(
            state.seed(
                &mut Budget(20),
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION
            ),
            Err(EntropyError::Cipher)
        );
        // Seeded, then the engine dies during a request.
        let mut state = EntropyState::new();
        let mut aes = Budget(14 + 256 + 2 + 1);
        state
            .seed(
                &mut aes,
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION,
            )
            .unwrap();
        let mut output = [0xA5u8; 48];
        assert_eq!(state.fill(&mut aes, &mut output), Err(EntropyError::Cipher));
        assert_eq!(output, [0; 48]);
        assert_eq!(
            state.disposition(),
            EntropyDisposition::FailedClosed(EntropyError::Cipher)
        );
    }

    /// Provider returning a wrong-but-successful cipher: the KAT must catch it.
    struct Wrong;
    struct WrongCipher;
    impl Aes128Forward for WrongCipher {
        type Error = ();
        fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), ()> {
            block[0] ^= 1;
            Ok(())
        }
    }
    impl ForwardAesProvider for Wrong {
        fn forward_cipher(&mut self, _key: &AesKey) -> impl Aes128Forward + '_ {
            WrongCipher
        }
    }

    #[test]
    fn wrong_engine_output_fails_the_known_answer_test() {
        let mut state = EntropyState::new();
        assert_eq!(
            state.seed(
                &mut Wrong,
                &mut fake(uniform_word),
                &CUTOFFS,
                &PERSONALIZATION
            ),
            Err(EntropyError::KnownAnswerMismatch)
        );
    }
}
