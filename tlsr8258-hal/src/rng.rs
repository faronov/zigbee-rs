//! TLSR8258 random-number support: a reconstruction of the vendor's own
//! ADC-noise-seeded generator, plus a small NIST SP 800-90A CTR_DRBG built
//! on top of it for [`crate::radio`]/MAC-layer consumers that need a
//! `fill_random`-style API.
//!
//! # Evidence and confidence tiers
//!
//! This module was reconstructed from a disassembly of
//! `platform/lib/libdrivers_8258.a(random.o)` (`llvm-objdump -d -r
//! --no-show-raw-insn`, `elf32-littletc32`), supplied at
//! `/tmp/tlsr8258-random-audit/random.o`, cross-checked against the open
//! `platform/chip_8258/register.h`/`adc.h` headers and against this crate's
//! own already-evidenced [`crate::adc`] module. Four vendor functions are
//! present in that object: `rand`, `rng_init`, `adc_rng_result`, and
//! `random_generator_init`. Confidence differs sharply between them:
//!
//! - **`rand` — HIGH confidence, reproduced bit-for-bit** as [`VendorMwc`]:
//!   the disassembly is a short, fully unambiguous two-generator Marsaglia
//!   Multiply-With-Carry (MWC) combination, immediate multiplier constants
//!   `18000` (`0x00004650`) and `36969` (`0x00009069`), XORed at the end
//!   with the live free-running system tick counter at
//!   `0x800740` (`reg_system_tick`, confirmed in `register.h`):
//!   ```text
//!   m_w = 18000 * (m_w & 0xFFFF) + (m_w >> 16)
//!   m_z = 36969 * (m_z & 0xFFFF) + (m_z >> 16)
//!   return ((m_z << 16) + m_w) ^ *(u32*)0x800740
//!   ```
//!   **This is not cryptographically secure.** A classic MWC generator's
//!   internal state is recoverable from a handful of consecutive outputs
//!   via straightforward algebra (each output is an affine function of the
//!   prior state once the tick-XOR is subtracted, and the tick counter
//!   itself is a predictable, non-secret, monotonic value visible to any
//!   observer of `rng_init`'s companion timer). [`VendorMwc`] is exposed
//!   only as a labeled non-cryptographic, host-testable curiosity/
//!   compatibility primitive; it is **not** used to build [`Rng`]'s output
//!   below.
//! - **`rng_init` — HIGH confidence, reproduced exactly** as
//!   [`crate::adc::configure_vbat_noise_channel`] (see that function's own
//!   doc for the full register-by-register cross-check). Key finding: the
//!   channel it configures samples the chip's own **VBAT supply rail
//!   against GND**, differentially, at 14-bit resolution — not a floating
//!   or externally-wired pin.
//! - **`adc_rng_result` — LOWER confidence, deliberately NOT reproduced
//!   bit-for-bit.** Its own per-bit noise-extraction loop manipulates
//!   digital `dfifo`-adjacent offsets (e.g. an offset arithmetic pattern
//!   landing on `0xb08 + 0x14`) that do not cleanly resolve to a single
//!   named register in the open `register.h` (which documents
//!   `reg_dfifo{0,1,2}_{addr,size,addrHi,h_level}` but not every byte
//!   in between). Rather than guess, this module instead reuses this
//!   crate's own separately-evidenced (via the real, open
//!   `adc_sample_and_get_result_op()` C source) raw-sample path,
//!   [`crate::adc::sample_raw_codes`], and applies an explicitly
//!   *non-vendor*, original conditioning step (see [`Conditioner`]) to
//!   build DRBG entropy input. This is a different, independently
//!   justified data path, not a claimed reconstruction of
//!   `adc_rng_result`'s exact algorithm.
//! - **`random_generator_init` — HIGH confidence** for its call sequence
//!   (`adc_init` → `rng_init` → four `adc_rng_result` calls packed into
//!   `rnd_m_w`/`rnd_m_z` → restore analog power state) — its power-restore
//!   step (`analog[0xfc]` bit 5 toggled off then back on around sampling)
//!   is exactly [`crate::adc::set_powered`]'s `FLD_SAR_ADC_POWER_DOWN` bit,
//!   already implemented and named in `adc.rs` independently of this
//!   audit — but this module does not reproduce this function directly,
//!   for the same `adc_rng_result` reason above. This module's own
//!   [`harvest_seed`] takes a stricter, more general approach to that same
//!   "restore what was there before" idea: see [`crate::adc::
//!   RngFootprintSnapshot`] for a full register-by-register save/restore
//!   rather than a single power bit.
//!
//! # What `fill_random`/[`Rng`] actually provides
//!
//! `zigbee-mac`'s `PlatformServices::fill_random` contract requires
//! *cryptographically secure* random bytes, or `MacError::Unsupported`.
//! Exposing raw [`VendorMwc`] output would violate that contract (it is
//! algebraically predictable). But TLSR8258 does have a genuine hardware
//! entropy source — ADC sampling of supply-rail thermal/quantization noise
//! — so permanently returning `Unsupported` is no longer the most honest
//! answer either. This module's compromise, spelled out precisely so
//! downstream users can judge it themselves:
//!
//! 1. [`harvest_seed`] gathers many (currently 64 rounds x 8 samples = 512)
//!    raw 14-bit ADC codes from the VBAT/GND noise channel, plus repeated
//!    system-tick reads for extra timing context, and feeds them through
//!    [`Conditioner`], a SHA-256-based conditioning step, to produce a
//!    [`SEED_LEN`]-byte seed. **Conditioning cannot increase entropy** —
//!    see [`Conditioner`]'s own docs for why — and this module makes
//!    **no min-entropy claim** about the raw ADC observations themselves:
//!    no NIST SP 800-90B assessment was performed (that requires physical
//!    test equipment/statistical sampling on real silicon, which is
//!    outside what a source/disassembly audit can establish). This is the
//!    honest, stated, **remaining hardware-only gate**: the DRBG algorithm
//!    below is a reviewable standard construction verified against an
//!    official NIST CAVP known-answer test (see [`CtrDrbg`]'s tests), but
//!    the entropy quality feeding its seed is unverified. [`Conditioner`]
//!    does run one narrow, bounded runtime health check
//!    ([`Conditioner::source_is_wholly_stuck`]) that rejects the
//!    degenerate case of an ADC channel producing zero observed variation
//!    at all (a stuck-at/mis-wired/mis-configured fault) — this is *not* a
//!    statistical entropy test and passing it is not evidence of adequate
//!    min-entropy, only evidence the source is not completely dead.
//! 2. [`CtrDrbg`] is a compact, direct transcription of NIST SP 800-90A's
//!    CTR_DRBG (AES-128, no derivation function, section 10.2.1) —
//!    `Instantiate`/`Update`/`Generate`/`Reseed` exactly as specified,
//!    built on the same `aes` crate `zigbee-crypto` already depends on,
//!    used unconditionally on every target (host tests exercise the real
//!    AES-128 permutation, not a stand-in). Its tests check both internal
//!    consistency and an official NIST CAVP AES-128 CTR_DRBG (no
//!    derivation function) known-answer test vector (see
//!    `ctr_drbg_matches_nist_cavp_aes128_no_df_kat`).
//! 3. [`Rng`] is the ownership-safe singleton wrapper `zigbee-mac` uses:
//!    it seeds a [`CtrDrbg`] once from [`harvest_seed`] and serves
//!    subsequent `fill_bytes` calls from the DRBG (re-harvesting entropy
//!    and reseeding periodically), rather than re-sampling the ADC on
//!    every call (both slow and unnecessary for a DRBG's forward-security
//!    model). [`Rng::take`] is IRQ-safe (see its docs); [`harvest_seed`]
//!    snapshots and restores every ADC register its call path touches
//!    (see [`crate::adc::RngFootprintSnapshot`]) instead of assuming it
//!    owns the ADC exclusively.

use aes::Aes128;
use aes::cipher::generic_array::GenericArray;
use aes::cipher::{BlockEncrypt, KeyInit};
use sha2::{Digest, Sha256};

use super::adc::AdcError;
#[cfg(target_arch = "tc32")]
use super::adc::{self, SampleBuffer};
#[cfg(target_arch = "tc32")]
use super::mmio::{REG_SYSTEM_TICK, r32};

/// A classic two-generator Marsaglia Multiply-With-Carry combination,
/// reconstructed bit-for-bit from `random.o`'s `rand()` disassembly (see
/// module docs for the exact recurrence and evidence).
///
/// **Not cryptographically secure.** Exposed only as a labeled,
/// host-testable compatibility/curiosity primitive matching the vendor's
/// own PRNG math; [`Rng`] does not use it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VendorMwc {
    m_w: u32,
    m_z: u32,
}

impl VendorMwc {
    /// Seed values must be non-zero (a zero seed for either generator is a
    /// fixed point of its recurrence: `18000 * 0 + 0 == 0`, producing an
    /// all-zero, non-random stream). This is a property of the classic MWC
    /// construction itself, not specific to this implementation.
    pub const fn new(seed_w: u32, seed_z: u32) -> Self {
        Self {
            m_w: seed_w,
            m_z: seed_z,
        }
    }

    /// Advance both generators one step and combine them, matching
    /// `rand()`'s arithmetic exactly *except* for the live system-tick XOR
    /// (which only exists on real hardware — see [`tc32_next`] for the
    /// full vendor-equivalent output on-target).
    ///
    /// [`tc32_next`]: Self::tc32_next
    pub const fn next_u32(&mut self) -> u32 {
        self.m_w = 18000u32
            .wrapping_mul(self.m_w & 0xFFFF)
            .wrapping_add(self.m_w >> 16);
        self.m_z = 36969u32
            .wrapping_mul(self.m_z & 0xFFFF)
            .wrapping_add(self.m_z >> 16);
        (self.m_z << 16).wrapping_add(self.m_w)
    }

    /// Full vendor-equivalent output: [`next_u32`](Self::next_u32) XORed
    /// with the live `reg_system_tick` counter, matching `rand()` exactly.
    #[cfg(target_arch = "tc32")]
    pub fn tc32_next(&mut self) -> u32 {
        self.next_u32() ^ unsafe { r32(REG_SYSTEM_TICK) }
    }
}

/// AES-128 CTR_DRBG key length (also the seedlen contribution).
const KEY_LEN: usize = 16;
/// AES-128 block length (also the seedlen contribution and `V`'s length).
const BLOCK_LEN: usize = 16;
/// CTR_DRBG (no df) seedlen for AES-128: `keylen + outlen`.
pub const SEED_LEN: usize = KEY_LEN + BLOCK_LEN;

/// NIST SP 800-90A CTR_DRBG, AES-128, no derivation function (section
/// 10.2.1). A direct transcription of the published algorithm:
///
/// ```text
/// Update(provided_data[seedlen]):
///     temp = ""
///     while len(temp) < seedlen:
///         V = (V + 1) mod 2^128
///         temp = temp || AES-128-Encrypt(Key, V)
///     temp = temp[0:seedlen] xor provided_data
///     Key = temp[0:keylen]; V = temp[keylen:seedlen]
///
/// Instantiate(entropy_input[seedlen]):
///     Key = 0; V = 0
///     Update(entropy_input)
///
/// Generate(requested_bytes):
///     temp = ""
///     while len(temp) < requested_bytes:
///         V = (V + 1) mod 2^128
///         temp = temp || AES-128-Encrypt(Key, V)
///     output = temp[0:requested_bytes]
///     Update(zero_bytes[seedlen])   // backtracking resistance
///     return output
/// ```
///
/// This implementation supports no `additional_input` beyond the
/// zero-filled backtracking-resistance step (kept small/reviewable; any
/// caller needing additional_input support should reseed with fresh
/// entropy instead via [`reseed`](Self::reseed)). This omission matches
/// the NIST algorithm's own defined behavior for an absent
/// `additional_input`: the spec substitutes an all-zero `seedlen` string
/// in that case for both the per-`Generate` optional update (which this
/// implementation skips entirely, exactly as "absent" behaves) and the
/// mandatory trailing backtracking-resistance update (which this
/// implementation always performs with an explicit zero string) — the two
/// are equivalent.
///
/// Verified against an official NIST CAVP AES-128 CTR_DRBG (no derivation
/// function) known-answer test vector — see
/// `ctr_drbg_matches_nist_cavp_aes128_no_df_kat` below — in addition to
/// this module's own internal-consistency tests (determinism, sensitivity
/// to seed/state changes, and the `V` counter's wraparound arithmetic).
pub struct CtrDrbg {
    key: [u8; KEY_LEN],
    v: [u8; BLOCK_LEN],
    reseed_counter: u64,
}

impl CtrDrbg {
    /// `Instantiate` with no personalization string: `entropy_input` must
    /// be exactly [`SEED_LEN`] bytes (the caller is responsible for
    /// combining any entropy source(s) and/or a nonce into that buffer).
    pub fn new(entropy_input: &[u8; SEED_LEN]) -> Self {
        let mut drbg = Self {
            key: [0; KEY_LEN],
            v: [0; BLOCK_LEN],
            reseed_counter: 1,
        };
        drbg.update(entropy_input);
        drbg
    }

    /// `Reseed`: mix fresh entropy into the DRBG state and reset the
    /// reseed counter.
    pub fn reseed(&mut self, entropy_input: &[u8; SEED_LEN]) {
        self.update(entropy_input);
        self.reseed_counter = 1;
    }

    /// `Generate`: fill `out` with DRBG output, then run the mandatory
    /// backtracking-resistance `Update` step with zero `additional_input`.
    /// Bounded: iterates at most `ceil(out.len() / BLOCK_LEN) + 1` times,
    /// there is no unbounded/looping wait.
    pub fn generate(&mut self, out: &mut [u8]) {
        let mut produced = 0;
        while produced < out.len() {
            self.increment_v();
            let block = self.encrypt_v();
            let take = core::cmp::min(BLOCK_LEN, out.len() - produced);
            out[produced..produced + take].copy_from_slice(&block[..take]);
            produced += take;
        }
        self.update(&[0u8; SEED_LEN]);
        self.reseed_counter = self.reseed_counter.saturating_add(1);
    }

    /// Number of `Generate` calls since the last `Instantiate`/`Reseed`.
    /// Exposed so a caller (e.g. [`Rng`]) can implement its own reseed
    /// policy; this type does not enforce NIST's reseed-interval limits
    /// itself (those are request-count limits intended for very
    /// high-throughput uses far beyond a sleepy 802.15.4 MAC's needs).
    pub fn reseed_counter(&self) -> u64 {
        self.reseed_counter
    }

    fn update(&mut self, provided_data: &[u8; SEED_LEN]) {
        let mut temp = [0u8; SEED_LEN];
        for chunk in temp.chunks_mut(BLOCK_LEN) {
            self.increment_v();
            let block = self.encrypt_v();
            chunk.copy_from_slice(&block);
        }
        for (byte, input) in temp.iter_mut().zip(provided_data.iter()) {
            *byte ^= input;
        }
        self.key.copy_from_slice(&temp[..KEY_LEN]);
        self.v.copy_from_slice(&temp[KEY_LEN..]);
    }

    /// `V = (V + 1) mod 2^128`, treating `V` as a 128-bit big-endian
    /// integer (last byte is the least significant, per NIST's bitstring
    /// convention).
    fn increment_v(&mut self) {
        for byte in self.v.iter_mut().rev() {
            let (result, carry) = byte.overflowing_add(1);
            *byte = result;
            if !carry {
                break;
            }
        }
    }

    fn encrypt_v(&self) -> [u8; BLOCK_LEN] {
        let cipher = Aes128::new(GenericArray::from_slice(&self.key));
        let mut generic = GenericArray::clone_from_slice(&self.v);
        cipher.encrypt_block(&mut generic);
        let mut out = [0u8; BLOCK_LEN];
        out.copy_from_slice(&generic);
        out
    }
}

/// Errors from [`Rng`]'s entropy-harvesting path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RngError {
    /// The singleton has already been taken; see [`Rng::take`].
    AlreadyTaken,
    /// The underlying ADC noise channel failed to configure or sample.
    /// The ADC's pre-harvest register state was successfully restored
    /// (see [`crate::adc::restore_rng_footprint`]) before this error was
    /// returned, unless a *second*, independent analog failure happened
    /// during that restore too — see [`RngError::AdcRestoreFailed`] for
    /// that separate case.
    Adc(AdcError),
    /// The ADC noise channel produced zero observed variation across the
    /// entire harvest — a stuck-at/mis-wired/mis-configured fault, not a
    /// statistical entropy judgement (see [`Conditioner::
    /// source_is_wholly_stuck`]). The ADC's pre-harvest register state was
    /// restored before this error was returned.
    EntropySourceStuck,
    /// Entropy harvesting itself completed (successfully or not), but
    /// restoring the ADC's pre-harvest register state afterward failed
    /// with the enclosed analog-bus error. Any bytes that *would* have
    /// been returned from a successful harvest are discarded: this crate
    /// will not hand back "random" bytes while simultaneously leaving
    /// global ADC state in an unknown, unrestored configuration a caller
    /// has no way to detect. The ADC is left in whatever partially
    /// restored state the failing register write occurred at — matching
    /// [`AdcError::Analog`]'s own documented recovery contract of
    /// re-running the failing step (here, re-running
    /// [`crate::adc::restore_rng_footprint`] with the same snapshot, if
    /// the caller has retained it, or otherwise re-initializing whatever
    /// ADC configuration it needs from scratch).
    AdcRestoreFailed(AdcError),
}

impl From<AdcError> for RngError {
    fn from(error: AdcError) -> Self {
        RngError::Adc(error)
    }
}

/// Number of ADC-sampling rounds [`harvest_seed`] performs. Each round
/// samples [`crate::adc::SAMPLE_COUNT`] raw 14-bit codes; `64 * 8 = 512`
/// raw samples total, chosen to be generous given the (unverified)
/// per-sample entropy is likely a small fraction of a bit, not to satisfy
/// any measured target.
#[cfg(target_arch = "tc32")]
const ENTROPY_ROUNDS: usize = 64;

/// Fixed domain-separation tag folded into every [`Conditioner`] digest,
/// so this specific harvest's SHA-256 input can never collide with a
/// digest computed over the same raw bytes for any unrelated purpose.
const CONDITIONER_DOMAIN: &[u8] = b"tlsr8258-hal/rng/v1/vbat-adc-noise-conditioner";

/// Folds raw ADC noise observations (plus timing context) into a
/// [`SEED_LEN`]-byte seed via SHA-256, and runs one bounded, non-
/// statistical health check along the way.
///
/// # This cannot increase entropy
///
/// A cryptographic hash function is a deterministic map: by the data-
/// processing inequality, `H(x)`'s min-entropy can be *no greater* than
/// `x`'s. Feeding raw ADC codes through SHA-256 here only **whitens**
/// them — removing bias, spreading whatever real unpredictability is
/// present uniformly and non-linearly across all 32 output bytes, and
/// destroying structure/correlation an attacker might otherwise exploit —
/// it does not, and cannot, manufacture entropy that was never present in
/// the raw samples. Producing a 32-byte digest is not itself evidence that
/// 32 bytes' (256 bits') worth of real unpredictability went in; see the
/// module docs for why that question is explicitly left open (no SP
/// 800-90B assessment was performed).
///
/// Pure/host-testable: this type never touches hardware itself —
/// [`harvest_seed`] is the only caller that feeds it real ADC samples, and
/// is the only tc32-gated piece of this path.
pub struct Conditioner {
    hasher: Sha256,
    min_code: u16,
    max_code: u16,
    saw_any_sample: bool,
}

impl Conditioner {
    /// Start a new conditioning run. `start_tick` is folded in first, as
    /// timing context distinct from (and prior to) any ADC sample.
    pub fn new(start_tick: u32) -> Self {
        let mut hasher = Sha256::new();
        hasher.update(CONDITIONER_DOMAIN);
        hasher.update(start_tick.to_le_bytes());
        Self {
            hasher,
            min_code: u16::MAX,
            max_code: 0,
            saw_any_sample: false,
        }
    }

    /// Absorb one round's worth of raw ADC codes plus the tick counter
    /// read immediately afterward, in that order (matching
    /// [`harvest_seed`]'s own sampling loop). Updates the running
    /// min/max used by [`source_is_wholly_stuck`](Self::source_is_wholly_stuck).
    pub fn absorb_round(&mut self, codes: &[u16], tick_after: u32) {
        for &code in codes {
            self.saw_any_sample = true;
            self.min_code = self.min_code.min(code);
            self.max_code = self.max_code.max(code);
            self.hasher.update(code.to_le_bytes());
        }
        self.hasher.update(tick_after.to_le_bytes());
    }

    /// Bounded, non-statistical defect check: `true` if every single
    /// absorbed code was bit-identical (or if no codes were absorbed at
    /// all). This catches a wholly dead/stuck-at/mis-wired/mis-configured
    /// noise channel — it is **not** a min-entropy or statistical
    /// randomness test, and a source that passes it may still have
    /// arbitrarily low real entropy. See the module docs.
    pub fn source_is_wholly_stuck(&self) -> bool {
        !self.saw_any_sample || self.min_code == self.max_code
    }

    /// Finish conditioning and produce the [`SEED_LEN`]-byte seed.
    /// Callers should check [`source_is_wholly_stuck`](Self::source_is_wholly_stuck)
    /// before trusting this output.
    pub fn finish(self) -> [u8; SEED_LEN] {
        let digest = self.hasher.finalize();
        let mut seed = [0u8; SEED_LEN];
        seed.copy_from_slice(&digest);
        seed
    }
}

/// Gather a [`SEED_LEN`]-byte seed from the VBAT/GND ADC noise channel
/// (see module docs, point 1, for exactly what this does and does not
/// prove). Bounded: exactly [`ENTROPY_ROUNDS`] sampling calls, each of
/// which is itself bounded (`sample_raw_codes` uses fixed sleeps, never an
/// unbounded poll).
///
/// # ADC ownership
///
/// This function does **not** assume it is the ADC's only user. It
/// snapshots every register its call path (`adc::init` +
/// `adc::configure_vbat_noise_channel` + `adc::set_powered` +
/// `adc::sample_raw_codes`) can touch *before* changing anything (see
/// [`adc::RngFootprintSnapshot`]), and restores every one of them
/// afterward — on the success path, on a harvesting failure, and on the
/// "source is stuck" rejection alike — so any ADC configuration a caller
/// had in place before this call is back in place after it, regardless of
/// outcome. If the initial snapshot read itself fails, no hardware state
/// is touched at all and the failure is returned immediately (see
/// [`adc::snapshot_rng_footprint`]'s docs). If restoring afterward fails,
/// that is reported distinctly via [`RngError::AdcRestoreFailed`] rather
/// than silently discarded or conflated with a harvesting failure.
#[cfg(target_arch = "tc32")]
pub fn harvest_seed() -> Result<[u8; SEED_LEN], RngError> {
    let _lease = adc::AdcLease::acquire()?;
    let snapshot = adc::snapshot_rng_footprint()?;

    let outcome = harvest_seed_inner();
    let restore = adc::restore_rng_footprint(&snapshot);

    finish_harvest(outcome, restore)
}

fn finish_harvest(
    outcome: Result<[u8; SEED_LEN], RngError>,
    restore: Result<(), AdcError>,
) -> Result<[u8; SEED_LEN], RngError> {
    match restore {
        Ok(()) => outcome,
        // Restoration failure takes precedence even when harvesting also
        // failed: the ADC is now in an unknown global state, which is the
        // safety-critical condition the caller must act on. Returning only
        // the harvest error would incorrectly imply restoration succeeded.
        Err(restore_error) => Err(RngError::AdcRestoreFailed(restore_error)),
    }
}

/// The actual reconfigure/sample/condition sequence, split out of
/// [`harvest_seed`] purely so that function's snapshot/restore wrapping
/// stays easy to read as a single straight-line "try, then always
/// restore" shape.
#[cfg(target_arch = "tc32")]
fn harvest_seed_inner() -> Result<[u8; SEED_LEN], RngError> {
    adc::init()?;
    adc::configure_vbat_noise_channel()?;
    adc::set_powered(true)?;

    let mut buffer = SampleBuffer::new();
    let start_tick = unsafe { r32(REG_SYSTEM_TICK) };
    let mut conditioner = Conditioner::new(start_tick);

    for _ in 0..ENTROPY_ROUNDS {
        let codes = adc::sample_raw_codes(&mut buffer)?;
        let tick = unsafe { r32(REG_SYSTEM_TICK) };
        conditioner.absorb_round(&codes, tick);
    }

    if conditioner.source_is_wholly_stuck() {
        return Err(RngError::EntropySourceStuck);
    }

    Ok(conditioner.finish())
}

/// Reseed [`Rng`] after this many `Generate` calls. Chosen conservatively
/// small for an embedded 802.15.4 MAC's request volume — not derived from
/// NIST's own (much larger) reseed-interval limits, which target
/// high-throughput server use.
#[cfg(target_arch = "tc32")]
const RESEED_INTERVAL: u64 = 256;

/// Exclusive handle to the TLSR8258 RNG/entropy path.
///
/// See the module docs for exactly what [`fill_bytes`](Self::fill_bytes)
/// does and does not prove about output quality. Mirrors
/// [`crate::radio::Radio`]'s singleton-token pattern: only one live `Rng`
/// may exist at a time, obtained via [`Rng::take`].
pub struct Rng {
    #[cfg(target_arch = "tc32")]
    drbg: CtrDrbg,
}

#[cfg(target_arch = "tc32")]
static mut RNG_TAKEN: u8 = 0;

impl Rng {
    /// Acquire the singleton `Rng`, harvesting an initial seed from the
    /// ADC noise channel. Returns [`RngError::AlreadyTaken`] if already
    /// held, or a harvesting error (see [`harvest_seed`]) if the initial
    /// entropy harvest failed — in which case the singleton flag is
    /// rolled back so a later retry is legitimate.
    ///
    /// # IRQ safety
    ///
    /// The check-then-set singleton flag runs with the CPU IRQ enable
    /// masked (via [`super::mmio::with_irqs_disabled`]), the same
    /// primitive [`crate::peripherals::Peripherals::take`] uses, so two
    /// overlapping calls to `take` (e.g. one from main-line code and one
    /// from an interrupt handler) cannot both observe the flag clear and
    /// both proceed.
    #[cfg(target_arch = "tc32")]
    pub fn take() -> Result<Self, RngError> {
        let already_taken = super::mmio::with_irqs_disabled(|| unsafe {
            let ptr = core::ptr::addr_of_mut!(RNG_TAKEN);
            if core::ptr::read_volatile(ptr) != 0 {
                true
            } else {
                core::ptr::write_volatile(ptr, 1);
                false
            }
        });
        if already_taken {
            return Err(RngError::AlreadyTaken);
        }

        match harvest_seed() {
            Ok(seed) => Ok(Self {
                drbg: CtrDrbg::new(&seed),
            }),
            Err(error) => {
                // Roll back the singleton flag: no half-constructed `Rng`
                // now exists, so a later retry is legitimate. Also under
                // an IRQ-masked critical section for the same reason as
                // the initial check-and-set above.
                super::mmio::with_irqs_disabled(|| unsafe {
                    core::ptr::write_volatile(core::ptr::addr_of_mut!(RNG_TAKEN), 0);
                });
                Err(error)
            }
        }
    }

    /// Re-harvest entropy from the ADC noise channel and reseed the DRBG.
    #[cfg(target_arch = "tc32")]
    pub fn reseed(&mut self) -> Result<(), RngError> {
        let seed = harvest_seed()?;
        self.drbg.reseed(&seed);
        Ok(())
    }

    /// Fill `out` with DRBG output, reseeding first if
    /// [`RESEED_INTERVAL`] `Generate` calls have elapsed since the last
    /// seed/reseed.
    #[cfg(target_arch = "tc32")]
    pub fn fill_bytes(&mut self, out: &mut [u8]) -> Result<(), RngError> {
        if self.drbg.reseed_counter() >= RESEED_INTERVAL {
            self.reseed()?;
        }
        self.drbg.generate(out);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adc_restore_failure_takes_precedence_over_harvest_failure() {
        let result = finish_harvest(
            Err(RngError::EntropySourceStuck),
            Err(AdcError::UnsupportedPin),
        );
        assert_eq!(
            result,
            Err(RngError::AdcRestoreFailed(AdcError::UnsupportedPin))
        );
    }

    #[test]
    fn vendor_mwc_matches_hand_computed_first_step() {
        // m_w = 18000 * (1 & 0xFFFF) + (1 >> 16) = 18000
        // m_z = 36969 * (1 & 0xFFFF) + (1 >> 16) = 36969
        // output = (36969 << 16) + 18000 = 2422503984... check exact value.
        let mut mwc = VendorMwc::new(1, 1);
        let out = mwc.next_u32();
        let expected_w = 18000u32;
        let expected_z = 36969u32;
        let expected = (expected_z << 16).wrapping_add(expected_w);
        assert_eq!(out, expected);
        assert_eq!(mwc, VendorMwc::new(expected_w, expected_z));
    }

    #[test]
    fn vendor_mwc_zero_seed_is_a_fixed_point_by_construction() {
        // Documented property, not a defect: the classic MWC recurrence
        // maps 0 -> 0 for either generator in isolation.
        let mut mwc = VendorMwc::new(0, 0);
        assert_eq!(mwc.next_u32(), 0);
        assert_eq!(mwc.next_u32(), 0);
    }

    #[test]
    fn vendor_mwc_advances_and_is_not_trivially_periodic_over_short_runs() {
        let mut mwc = VendorMwc::new(0xDEAD_BEEF, 0xCAFE_F00D);
        let mut outputs = [0u32; 8];
        for out in outputs.iter_mut() {
            *out = mwc.next_u32();
        }
        for i in 0..outputs.len() {
            for j in (i + 1)..outputs.len() {
                assert_ne!(outputs[i], outputs[j], "unexpected short-period repeat");
            }
        }
    }

    fn seed(byte: u8) -> [u8; SEED_LEN] {
        [byte; SEED_LEN]
    }

    /// Decode a fixed-length hex string into a byte array at test time.
    /// Panics (via `unwrap`/length assertion) on malformed input, which is
    /// exactly what a test-only helper should do — never used outside
    /// `#[cfg(test)]`.
    fn hex<const N: usize>(digits: &str) -> [u8; N] {
        assert_eq!(digits.len(), N * 2, "hex string has the wrong length");
        let mut out = [0u8; N];
        for (index, byte) in out.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&digits[index * 2..index * 2 + 2], 16)
                .expect("invalid hex digit");
        }
        out
    }

    #[test]
    fn ctr_drbg_is_deterministic_for_the_same_seed() {
        let mut a = CtrDrbg::new(&seed(0x11));
        let mut b = CtrDrbg::new(&seed(0x11));
        let mut out_a = [0u8; 64];
        let mut out_b = [0u8; 64];
        a.generate(&mut out_a);
        b.generate(&mut out_b);
        assert_eq!(out_a, out_b);
    }

    #[test]
    fn ctr_drbg_differs_for_different_seeds() {
        let mut a = CtrDrbg::new(&seed(0x11));
        let mut b = CtrDrbg::new(&seed(0x22));
        let mut out_a = [0u8; 32];
        let mut out_b = [0u8; 32];
        a.generate(&mut out_a);
        b.generate(&mut out_b);
        assert_ne!(out_a, out_b);
    }

    #[test]
    fn ctr_drbg_successive_generate_calls_differ() {
        let mut drbg = CtrDrbg::new(&seed(0x33));
        let mut first = [0u8; 32];
        let mut second = [0u8; 32];
        drbg.generate(&mut first);
        drbg.generate(&mut second);
        assert_ne!(first, second);
        assert_eq!(drbg.reseed_counter(), 3); // Instantiate's Update = 1, then +1 per generate
    }

    #[test]
    fn ctr_drbg_reseed_changes_subsequent_output() {
        let mut drbg = CtrDrbg::new(&seed(0x44));
        let mut before = [0u8; 32];
        drbg.generate(&mut before);

        let mut same_seed_no_reseed = CtrDrbg::new(&seed(0x44));
        same_seed_no_reseed.generate(&mut [0u8; 32]);
        let mut expected_next = [0u8; 32];
        same_seed_no_reseed.generate(&mut expected_next);

        drbg.reseed(&seed(0x55));
        let mut after_reseed = [0u8; 32];
        drbg.generate(&mut after_reseed);

        assert_ne!(after_reseed, expected_next);
    }

    #[test]
    fn ctr_drbg_generate_handles_non_block_aligned_lengths() {
        let mut drbg = CtrDrbg::new(&seed(0x66));
        let mut odd = [0u8; 17];
        drbg.generate(&mut odd);
        // No panic, and it actually wrote something non-trivial.
        assert!(odd.iter().any(|&b| b != 0));
    }

    #[test]
    fn ctr_drbg_v_increment_wraps_at_2_pow_128() {
        let mut drbg = CtrDrbg::new(&seed(0x00));
        drbg.v = [0xFF; BLOCK_LEN];
        drbg.increment_v();
        assert_eq!(drbg.v, [0u8; BLOCK_LEN]);
    }

    #[test]
    fn ctr_drbg_v_increment_only_carries_as_needed() {
        let mut drbg = CtrDrbg::new(&seed(0x00));
        drbg.v = [0u8; BLOCK_LEN];
        drbg.v[BLOCK_LEN - 1] = 0xFF;
        drbg.increment_v();
        let mut expected = [0u8; BLOCK_LEN];
        expected[BLOCK_LEN - 2] = 1;
        assert_eq!(drbg.v, expected);
    }

    #[test]
    fn seed_len_matches_aes128_ctr_drbg_no_df_spec() {
        // NIST SP 800-90A Table 3: seedlen = keylen + outlen for CTR_DRBG
        // without a derivation function, AES-128: 128 + 128 bits = 32 bytes.
        assert_eq!(SEED_LEN, 32);
    }

    /// Official NIST CAVP DRBGVS AES-128 CTR_DRBG (no derivation function)
    /// known-answer test: `PredictionResistance = False`, no
    /// `PersonalizationString`, no `AdditionalInput` on either `Generate`
    /// call (matching this module's own no-additional-input API exactly).
    /// `Generate` is exercised twice, discarding the first 64-byte output
    /// and checking only the second — the DRBGVS vector format's standard
    /// shape, present so that a checked output already reflects one full
    /// backtracking-resistance `Update` cycle, not just `Instantiate`'s.
    ///
    /// `EntropyInput` (32 bytes = `SEED_LEN`, this vector's entire
    /// `Instantiate` seed material since no derivation function is used):
    /// `ce50f33da5d4c1d3d4004eb35244b7f2cd7f2e5076fbf6780a7ff634b249a5fc`
    ///
    /// Expected second-`Generate` `ReturnedBits` (64 bytes):
    /// `6545c0529d372443b392ceb3ae3a99a30f963eaf313280f1d1a1e87f9db373d
    /// 361e75d18018266499cccd64d9bbb8de0185f213383080faddec46bae1f784e5a`
    ///
    /// The intermediate `Key`/`V` values asserted below were independently
    /// hand-verified against a reference AES-128-ECB/CTR_DRBG
    /// implementation (not derived from this crate's own code) before
    /// being encoded here, so a future accidental change to this
    /// implementation that still happens to match the final output but
    /// takes a different path through `Update`/`Generate` is still caught.
    #[test]
    fn ctr_drbg_matches_nist_cavp_aes128_no_df_kat() {
        let entropy_input: [u8; SEED_LEN] =
            hex("ce50f33da5d4c1d3d4004eb35244b7f2cd7f2e5076fbf6780a7ff634b249a5fc");

        let mut drbg = CtrDrbg::new(&entropy_input);
        assert_eq!(
            drbg.key,
            hex::<KEY_LEN>("96b20ff35faaf1b2e27f53e4f6a3f2a8"),
            "Key after Instantiate does not match the reference implementation"
        );
        assert_eq!(
            drbg.v,
            hex::<BLOCK_LEN>("cef7f49e164d55eaf957348dc3fb5b84"),
            "V after Instantiate does not match the reference implementation"
        );

        let mut first = [0u8; 64];
        drbg.generate(&mut first);
        assert_eq!(
            drbg.key,
            hex::<KEY_LEN>("2e8bf07c5a29b97633576a7c4d5343dd"),
            "Key after the first Generate does not match the reference implementation"
        );
        assert_eq!(
            drbg.v,
            hex::<BLOCK_LEN>("3f93dbc9dc724d654f5f2a45b818c7ec"),
            "V after the first Generate does not match the reference implementation"
        );

        let mut second = [0u8; 64];
        drbg.generate(&mut second);
        let expected_second: [u8; 64] = hex(
            "6545c0529d372443b392ceb3ae3a99a30f963eaf313280f1d1a1e87f9db373d\
             361e75d18018266499cccd64d9bbb8de0185f213383080faddec46bae1f784e5a",
        );
        assert_eq!(
            second, expected_second,
            "second Generate's output does not match the NIST CAVP KAT"
        );
    }

    #[test]
    fn conditioner_matches_a_hand_computed_sha256_digest() {
        // Independently computed: SHA-256(CONDITIONER_DOMAIN
        //     || start_tick.to_le_bytes()
        //     || codes[0].to_le_bytes() || ... || codes[7].to_le_bytes()
        //     || tick_after.to_le_bytes())
        // for start_tick = 1, codes = [0, 1, 2, 3, 4, 5, 6, 7], tick_after = 2.
        let mut conditioner = Conditioner::new(1);
        let codes: [u16; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
        conditioner.absorb_round(&codes, 2);
        assert!(!conditioner.source_is_wholly_stuck());
        let seed = conditioner.finish();
        let expected: [u8; SEED_LEN] =
            hex("3d8ceb90a2038ace83bf09541d6c9d698cbec08e2ee1279d82586411fc03d23d");
        assert_eq!(seed, expected);
    }

    #[test]
    fn conditioner_is_deterministic_for_identical_inputs() {
        let mut a = Conditioner::new(42);
        let mut b = Conditioner::new(42);
        let codes: [u16; 4] = [10, 20, 30, 40];
        a.absorb_round(&codes, 99);
        b.absorb_round(&codes, 99);
        assert_eq!(a.finish(), b.finish());
    }

    #[test]
    fn conditioner_differs_when_a_single_code_bit_differs() {
        let mut a = Conditioner::new(1);
        let mut b = Conditioner::new(1);
        a.absorb_round(&[100, 200], 1);
        b.absorb_round(&[100, 201], 1);
        assert_ne!(a.finish(), b.finish());
    }

    #[test]
    fn conditioner_detects_a_wholly_stuck_source() {
        let mut conditioner = Conditioner::new(0);
        for _ in 0..64 {
            conditioner.absorb_round(&[8191u16; 8], 0);
        }
        assert!(conditioner.source_is_wholly_stuck());
    }

    #[test]
    fn conditioner_with_no_absorbed_samples_counts_as_stuck() {
        let conditioner = Conditioner::new(0);
        assert!(conditioner.source_is_wholly_stuck());
    }

    #[test]
    fn conditioner_is_not_stuck_once_any_two_codes_differ() {
        let mut conditioner = Conditioner::new(0);
        // 511 identical samples plus a single differing one is still
        // "not stuck": the check is a coarse floor, not a statistical
        // judgement about how much variation is enough.
        for _ in 0..63 {
            conditioner.absorb_round(&[100u16; 8], 0);
        }
        conditioner.absorb_round(&[100, 100, 100, 100, 100, 100, 100, 101], 0);
        assert!(!conditioner.source_is_wholly_stuck());
    }
}
