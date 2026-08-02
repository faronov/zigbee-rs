//! Low-stack cryptographic primitives shared by the Zigbee protocol layers.

#![no_std]

use aes::Aes128;
use aes::cipher::generic_array::GenericArray;
use aes::cipher::{BlockEncrypt, KeyInit};

/// A 128-bit AES key.
pub type AesKey = [u8; 16];

/// AES-CCM* nonce length used by Zigbee.
pub const CCM_STAR_NONCE_LEN: usize = 13;
/// MIC length for Zigbee ENC-MIC-32 security.
pub const CCM_STAR_MIC_LEN: usize = 4;
/// Largest AAD length supported by CCM's two-byte length encoding.
pub const CCM_STAR_MAX_AAD_LEN: usize = 0xFEFF;
/// Maximum plaintext length supported by the fixed Zigbee frame buffer.
pub const CCM_STAR_MAX_PLAINTEXT_LEN: usize = 124;
/// Capacity of an encrypted Zigbee payload including its MIC.
pub const CCM_STAR_BUFFER_CAPACITY: usize = CCM_STAR_MAX_PLAINTEXT_LEN + CCM_STAR_MIC_LEN;

const CCM_B0_FLAGS_ADATA: u8 = 0x49;
const CCM_B0_FLAGS_NO_ADATA: u8 = 0x09;
const CCM_AI_FLAGS: u8 = 0x01;

/// A single-block AES-128 forward (encrypt) permutation, abstracted so
/// CCM* (which — see below — only ever needs this one direction) can run
/// on top of either a software implementation or a hardware accelerator.
///
/// # Why only forward/encrypt
///
/// AES-CCM* (RFC 3610-style, as profiled by Zigbee, `M=4`, `L=2`) uses AES
/// only in CBC-MAC mode (for the authentication tag) and CTR mode (for the
/// keystream) — both of which call the block cipher's *forward*
/// permutation exclusively, even when the overall operation is
/// "decrypt/verify" (`ccm_star_decrypt` still only calls `aes_enc`, never
/// an inverse round). So a `Aes128Forward` implementation never needs to
/// provide a decrypt primitive for CCM* to work, which is exactly what
/// keeps this trait minimal and keeps a hardware backend from having to
/// implement (or even possess, if the silicon lacked it) an inverse-cipher
/// path.
///
/// # Integration model
///
/// This trait exists so [`ccm_star_encrypt_with`]/[`ccm_star_decrypt_with`]
/// can be generic over the cipher implementation while
/// [`ccm_star_encrypt`]/[`ccm_star_decrypt`] (the crate's existing public
/// API, unchanged) keep defaulting to [`SoftwareAes128`]. Embedded NWK/APS
/// paths obtain their keyed cipher through [`ForwardAesProvider`], allowing a
/// platform-owned accelerator to serve CCM* and AES-MMO without changing the
/// software-only public wrappers.
pub trait Aes128Forward {
    /// Error type surfaced by [`Self::encrypt_block`]. Software
    /// implementations should use [`core::convert::Infallible`]; hardware
    /// implementations should use their own bounded timeout/hardware error
    /// enum (never panic and never spin unboundedly).
    type Error;

    /// Encrypt `block` in place, in AES-128 ECB single-block forward mode.
    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error>;
}

/// Default, always-available [`Aes128Forward`] implementation: the
/// RustCrypto `aes` crate's constant-time-by-design software AES-128.
/// Used by every current public function in this crate
/// ([`ccm_star_encrypt`]/[`ccm_star_decrypt`]) on every platform, including
/// TLSR8258 unless a caller explicitly opts into the hardware backend via
/// [`ccm_star_encrypt_with`]/[`ccm_star_decrypt_with`].
pub struct SoftwareAes128(Aes128);

impl SoftwareAes128 {
    /// Expand `key` into round keys ready for [`Aes128Forward::encrypt_block`].
    pub fn new(key: &AesKey) -> Self {
        Self(Aes128::new(GenericArray::from_slice(key)))
    }
}

impl Aes128Forward for SoftwareAes128 {
    /// Software AES never fails.
    type Error = core::convert::Infallible;

    #[inline(always)]
    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        let mut generic = GenericArray::clone_from_slice(block);
        self.0.encrypt_block(&mut generic);
        block.copy_from_slice(generic.as_slice());
        Ok(())
    }
}

/// A source of keyed forward AES-128 permutations.
///
/// # Why a *factory*, not a single keyed cipher
///
/// Both users of AES in the Zigbee stack want a *fresh* keyed permutation
/// under a caller-chosen key:
///
/// * CCM* (`ccm_star_*`) keys the block cipher once per frame, then reuses
///   that keyed permutation across every CBC-MAC / CTR block of the frame.
/// * AES-MMO hashing (`zigbee-aps`'s key derivation) re-keys the block
///   cipher **every block** — the running hash becomes the next block's
///   key.
///
/// A single keyed [`Aes128Forward`] value cannot express the MMO case, so
/// the abstraction that both share is "give me a keyed cipher for *this*
/// key", i.e. this trait. [`forward_cipher`](Self::forward_cipher) is
/// called once per frame by CCM* and once per block by MMO — the returned
/// cipher's own [`Aes128Forward::encrypt_block`] (the true hot path) stays
/// statically dispatched and inlinable, so only the (comparatively rare)
/// re-key crosses the provider boundary.
///
/// # Software default, hardware override
///
/// The provided default returns [`SoftwareAes128`] on every platform, so a
/// backend that says nothing keeps the exact software behaviour this crate
/// has always had. A platform with an AES accelerator (e.g. the TLSR8258 —
/// see `tlsr8258::HardwareAes128`) overrides
/// [`forward_cipher`](Self::forward_cipher) to hand back a hardware-backed
/// [`Aes128Forward`]; because the override returns a *different* concrete
/// type, nothing in that platform's image references [`SoftwareAes128`] and
/// the RustCrypto software core is dead-code-eliminated. No hardware error
/// is ever silently swallowed: the returned cipher's
/// [`Aes128Forward::Error`] flows out of `ccm_star_*_with` to the caller,
/// which must surface it (never fall back to software).
pub trait ForwardAesProvider {
    /// Produce a forward AES-128 permutation keyed with `key`.
    fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
        SoftwareAes128::new(key)
    }
}

/// Zero-sized [`ForwardAesProvider`] that always uses the software AES core.
///
/// This is the provider the crate's own software wrappers
/// ([`ccm_star_encrypt`]/[`ccm_star_decrypt`] and the `zigbee-nwk` /
/// `zigbee-aps` `*` non-`_with` entry points) pass, and the one host tests
/// use to exercise the generic provider path without needing a platform
/// MAC. It carries no state, so constructing one per operation costs
/// nothing and preserves the "one key expansion per frame" behaviour CCM*
/// has always had.
#[derive(Debug, Default, Clone, Copy)]
pub struct SoftwareAesProvider;

impl SoftwareAesProvider {
    /// Construct the (stateless) software provider.
    pub const fn new() -> Self {
        Self
    }
}

impl ForwardAesProvider for SoftwareAesProvider {}

/// Encrypt a Zigbee payload with AES-128-CCM* using M=4 and L=2.
///
/// The returned buffer contains the encrypted payload followed by its
/// four-byte MIC. This implementation uses fixed-size stack buffers to remain
/// suitable for memory-constrained targets such as TLSR8258.
pub fn ccm_star_encrypt(
    key: &AesKey,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    plaintext: &[u8],
) -> Option<heapless::Vec<u8, CCM_STAR_BUFFER_CAPACITY>> {
    let mut cipher = SoftwareAes128::new(key);
    match ccm_star_encrypt_with(&mut cipher, nonce, aad, plaintext) {
        Ok(result) => result,
        Err(never) => match never {},
    }
}

/// Authenticate and decrypt a Zigbee AES-128-CCM* payload using M=4 and L=2.
pub fn ccm_star_decrypt(
    key: &AesKey,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Option<heapless::Vec<u8, CCM_STAR_BUFFER_CAPACITY>> {
    let mut cipher = SoftwareAes128::new(key);
    match ccm_star_decrypt_with(&mut cipher, nonce, aad, ciphertext_and_mic) {
        Ok(result) => result,
        Err(never) => match never {},
    }
}

/// Generic form of [`ccm_star_encrypt`], parameterized over any
/// [`Aes128Forward`] implementation (e.g. [`SoftwareAes128`], or a
/// platform hardware backend such as
/// `zigbee_crypto::tlsr8258::HardwareAes128`). The outer [`Result`] carries
/// `cipher`'s own hardware/timeout errors (impossible for
/// [`SoftwareAes128`], whose `Error` is [`core::convert::Infallible`]); the
/// inner [`Option`] preserves this crate's existing "invalid input length"
/// contract exactly as [`ccm_star_encrypt`] already documents.
pub fn ccm_star_encrypt_with<C: Aes128Forward>(
    cipher: &mut C,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    plaintext: &[u8],
) -> Result<Option<heapless::Vec<u8, CCM_STAR_BUFFER_CAPACITY>>, C::Error> {
    if plaintext.len() > CCM_STAR_MAX_PLAINTEXT_LEN || aad.len() > CCM_STAR_MAX_AAD_LEN {
        return Ok(None);
    }

    let tag = ccm_mac(cipher, nonce, aad, plaintext)?;

    let mut s0 = build_ai(nonce, 0);
    cipher.encrypt_block(&mut s0)?;
    let mut mic = [0u8; CCM_STAR_MIC_LEN];
    for i in 0..CCM_STAR_MIC_LEN {
        mic[i] = tag[i] ^ s0[i];
    }

    let mut buffer = [0u8; CCM_STAR_MAX_PLAINTEXT_LEN];
    buffer[..plaintext.len()].copy_from_slice(plaintext);
    ccm_ctr_xor(cipher, nonce, &mut buffer[..plaintext.len()])?;

    let mut output = heapless::Vec::new();
    let Ok(()) = output.extend_from_slice(&buffer[..plaintext.len()]) else {
        return Ok(None);
    };
    let Ok(()) = output.extend_from_slice(&mic) else {
        return Ok(None);
    };
    Ok(Some(output))
}

/// Generic form of [`ccm_star_decrypt`] — see [`ccm_star_encrypt_with`] for
/// the `Result`-of-`Option` contract.
pub fn ccm_star_decrypt_with<C: Aes128Forward>(
    cipher: &mut C,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Result<Option<heapless::Vec<u8, CCM_STAR_BUFFER_CAPACITY>>, C::Error> {
    if ciphertext_and_mic.len() < CCM_STAR_MIC_LEN || aad.len() > CCM_STAR_MAX_AAD_LEN {
        return Ok(None);
    }

    let ciphertext_len = ciphertext_and_mic.len() - CCM_STAR_MIC_LEN;
    if ciphertext_len > CCM_STAR_MAX_PLAINTEXT_LEN {
        return Ok(None);
    }

    let mut buffer = [0u8; CCM_STAR_MAX_PLAINTEXT_LEN];
    buffer[..ciphertext_len].copy_from_slice(&ciphertext_and_mic[..ciphertext_len]);
    ccm_ctr_xor(cipher, nonce, &mut buffer[..ciphertext_len])?;

    let tag = ccm_mac(cipher, nonce, aad, &buffer[..ciphertext_len])?;
    let mut s0 = build_ai(nonce, 0);
    cipher.encrypt_block(&mut s0)?;
    let mut expected_mic = [0u8; CCM_STAR_MIC_LEN];
    for i in 0..CCM_STAR_MIC_LEN {
        expected_mic[i] = tag[i] ^ s0[i];
    }

    if !constant_time_mic_eq(
        &expected_mic,
        &ciphertext_and_mic[ciphertext_len..ciphertext_len + CCM_STAR_MIC_LEN],
    ) {
        return Ok(None);
    }

    let mut output = heapless::Vec::new();
    let Ok(()) = output.extend_from_slice(&buffer[..ciphertext_len]) else {
        return Ok(None);
    };
    Ok(Some(output))
}

#[inline(always)]
fn build_ai(nonce: &[u8; CCM_STAR_NONCE_LEN], counter: u16) -> [u8; 16] {
    let mut block = [0u8; 16];
    block[0] = CCM_AI_FLAGS;
    block[1..14].copy_from_slice(nonce);
    block[14] = (counter >> 8) as u8;
    block[15] = counter as u8;
    block
}

#[inline(always)]
fn mac_fold<C: Aes128Forward>(
    cipher: &mut C,
    tag: &mut [u8; 16],
    block: &[u8; 16],
) -> Result<(), C::Error> {
    for i in 0..16 {
        tag[i] ^= block[i];
    }
    cipher.encrypt_block(tag)
}

fn ccm_mac<C: Aes128Forward>(
    cipher: &mut C,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    payload: &[u8],
) -> Result<[u8; 16], C::Error> {
    let mut tag = [0u8; 16];
    tag[0] = if aad.is_empty() {
        CCM_B0_FLAGS_NO_ADATA
    } else {
        CCM_B0_FLAGS_ADATA
    };
    tag[1..14].copy_from_slice(nonce);
    let payload_len = payload.len() as u16;
    tag[14] = (payload_len >> 8) as u8;
    tag[15] = payload_len as u8;
    cipher.encrypt_block(&mut tag)?;

    if !aad.is_empty() {
        let mut block = [0u8; 16];
        let aad_len = aad.len() as u16;
        block[0] = (aad_len >> 8) as u8;
        block[1] = aad_len as u8;
        let first_chunk_len = core::cmp::min(aad.len(), 14);
        block[2..2 + first_chunk_len].copy_from_slice(&aad[..first_chunk_len]);
        mac_fold(cipher, &mut tag, &block)?;

        let mut offset = first_chunk_len;
        while offset < aad.len() {
            block = [0u8; 16];
            let chunk_len = core::cmp::min(16, aad.len() - offset);
            block[..chunk_len].copy_from_slice(&aad[offset..offset + chunk_len]);
            mac_fold(cipher, &mut tag, &block)?;
            offset += chunk_len;
        }
    }

    let mut offset = 0;
    while offset < payload.len() {
        let mut block = [0u8; 16];
        let chunk_len = core::cmp::min(16, payload.len() - offset);
        block[..chunk_len].copy_from_slice(&payload[offset..offset + chunk_len]);
        mac_fold(cipher, &mut tag, &block)?;
        offset += chunk_len;
    }

    Ok(tag)
}

fn ccm_ctr_xor<C: Aes128Forward>(
    cipher: &mut C,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    data: &mut [u8],
) -> Result<(), C::Error> {
    let mut counter = 1u16;
    let mut offset = 0;
    while offset < data.len() {
        let mut key_stream = build_ai(nonce, counter);
        cipher.encrypt_block(&mut key_stream)?;
        let chunk_len = core::cmp::min(16, data.len() - offset);
        for i in 0..chunk_len {
            data[offset + i] ^= key_stream[i];
        }
        offset += chunk_len;
        counter = counter.wrapping_add(1);
    }
    Ok(())
}

#[inline(always)]
fn constant_time_mic_eq(expected: &[u8; CCM_STAR_MIC_LEN], actual: &[u8]) -> bool {
    let mut difference = 0u8;
    for i in 0..CCM_STAR_MIC_LEN {
        difference |= expected[i] ^ actual[i];
    }
    difference == 0
}

/// Optional TLSR8258 hardware-AES backend for [`Aes128Forward`]. Only
/// compiled when this crate's `tlsr8258` feature is enabled (which pulls
/// in `tlsr8258-hal` as an optional dependency) **and** the target
/// architecture is `tc32` — enabling the feature on any other target
/// (e.g. running this crate's host tests with `--features tlsr8258` by
/// mistake) leaves this module empty rather than failing to build, since
/// `tlsr8258_hal::aes::AesEngine`'s actual register-driving methods are
/// themselves `#[cfg(target_arch = "tc32")]`-gated in the HAL crate.
#[cfg(feature = "tlsr8258")]
pub mod tlsr8258 {
    #[cfg(target_arch = "tc32")]
    mod hardware {
        use crate::{Aes128Forward, AesKey};
        use tlsr8258_hal::aes::{AesEngine, AesError};

        /// [`Aes128Forward`] implementation backed by the TLSR8258's
        /// hardware AES-128 ECB accelerator ([`AesEngine`]).
        ///
        /// Holds the raw key as plain bytes (the hardware accelerator
        /// takes the raw 16-byte key directly on every block — see
        /// `tlsr8258_hal::aes`'s module docs — there is no separate
        /// "key schedule" step to cache) and a `&mut AesEngine` borrowed
        /// from the caller, so constructing this type does not duplicate
        /// [`AesEngine`]'s own exclusive-ownership guarantee (backed by the
        /// zero-sized `tlsr8258_hal::peripherals::Aes` token) — callers
        /// keep whatever `AesEngine` they already own and can freely reuse
        /// it (with different keys) across multiple `HardwareAes128`
        /// values without this type introducing a second, competing
        /// notion of ownership.
        pub struct HardwareAes128<'engine> {
            engine: &'engine mut AesEngine,
            key: AesKey,
        }

        impl<'engine> HardwareAes128<'engine> {
            /// Borrow `engine` and remember `key` for subsequent
            /// [`Aes128Forward::encrypt_block`] calls.
            pub fn new(engine: &'engine mut AesEngine, key: AesKey) -> Self {
                Self { engine, key }
            }
        }

        impl Aes128Forward for HardwareAes128<'_> {
            /// See [`tlsr8258_hal::aes::AesError`] — bounded
            /// timeout/hardware errors only, never an unbounded spin, per
            /// that module's own contract.
            type Error = AesError;

            #[inline(always)]
            fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
                self.engine.encrypt_block(&self.key, block)
            }
        }
    }

    #[cfg(target_arch = "tc32")]
    pub use hardware::HardwareAes128;
}

/// Optional BL702 hardware-AES backend for [`Aes128Forward`]. Only compiled
/// when this crate's `bl702` feature is enabled (which pulls in `bl702-hal`
/// as an optional dependency) **and** the target architecture is `riscv32`
/// — enabling the feature on any other target (e.g. running this crate's
/// host tests with `--features bl702` by mistake) leaves this module empty
/// rather than failing to build, since `bl702_hal::aes::AesEngine`'s actual
/// register-driving methods are themselves `#[cfg(target_arch = "riscv32")]`
/// gated in the HAL crate.
///
/// Mirrors the `tlsr8258` module above; the only differences are the HAL
/// crate and the target-architecture gate. The BL702 accelerator is the
/// SEC_ENG AES-128 ECB block — see `bl702_hal::aes` for its (open-source
/// vendor-driver-derived) register contract and its not-yet-silicon-proven
/// caveat.
#[cfg(feature = "bl702")]
pub mod bl702 {
    #[cfg(target_arch = "riscv32")]
    mod hardware {
        use bl702_hal::aes::{AesEngine, AesError};

        use crate::{Aes128Forward, AesKey};

        /// [`Aes128Forward`] implementation backed by the BL702's SEC_ENG
        /// hardware AES-128 ECB accelerator ([`AesEngine`]).
        ///
        /// Holds the raw key bytes (the accelerator takes the 16-byte key
        /// directly on every block — see `bl702_hal::aes` — there is no
        /// separate cached key schedule) and a `&mut AesEngine` borrowed
        /// from the caller, so this type does not duplicate [`AesEngine`]'s
        /// own exclusive-ownership guarantee (backed by the zero-sized
        /// `bl702_hal::peripherals::Aes` token). Callers keep whatever
        /// `AesEngine` they already own and can reuse it (with different
        /// keys) across multiple `HardwareAes128` values.
        pub struct HardwareAes128<'engine> {
            engine: &'engine mut AesEngine,
            key: AesKey,
        }

        impl<'engine> HardwareAes128<'engine> {
            /// Borrow `engine` and remember `key` for subsequent
            /// [`Aes128Forward::encrypt_block`] calls.
            pub fn new(engine: &'engine mut AesEngine, key: AesKey) -> Self {
                Self { engine, key }
            }
        }

        impl Aes128Forward for HardwareAes128<'_> {
            /// See [`bl702_hal::aes::AesError`] — bounded timeout/hardware
            /// errors only, never an unbounded spin, per that module's
            /// contract.
            type Error = AesError;

            #[inline(always)]
            fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
                self.engine.encrypt_block(&self.key, block)
            }
        }
    }

    #[cfg(target_arch = "riscv32")]
    pub use hardware::HardwareAes128;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nwk_golden_vector_matches_independent_ccm() {
        let key = [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0D,
        ];
        let nonce = [
            0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08, 0x01, 0x00, 0x00, 0x00, 0x2D,
        ];
        let expected = [
            0xAC, 0x17, 0x74, 0xEC, 0x17, 0x76, 0xC2, 0x7C, 0x41, 0xEE, 0x31, 0x0A, 0xE0, 0x0B,
            0x5B, 0x5A, 0xA0, 0x05, 0xC9,
        ];

        let encrypted =
            ccm_star_encrypt(&key, &nonce, b"NWK-HDR+AUX", b"hello-nwk-frame").expect("encrypt");
        assert_eq!(encrypted.as_slice(), expected);
        assert_eq!(
            ccm_star_decrypt(&key, &nonce, b"NWK-HDR+AUX", &encrypted)
                .expect("decrypt")
                .as_slice(),
            b"hello-nwk-frame"
        );
    }

    #[test]
    fn captured_aps_vector_matches_independent_ccm() {
        let key = [
            0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2, 0xD5, 0x72, 0xE1, 0xC1, 0xEF, 0x47,
            0x87, 0x82,
        ];
        let nonce = [
            0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71, 0x84, 0x53, 0x50, 0x0B, 0x00, 0x35,
        ];
        let aad = [
            0x21, 0x95, 0x35, 0x53, 0x50, 0x0B, 0x00, 0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71,
            0x84,
        ];
        let plaintext = [
            0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0x77, 0x88, 0x99, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
            0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        ];
        let expected = [
            0xB6, 0x78, 0x43, 0xD6, 0x39, 0xC1, 0x70, 0xF2, 0x0B, 0x53, 0x6F, 0xDA, 0x11, 0xB4,
            0x67, 0xCA, 0xAF, 0xEC, 0xD6, 0xC2, 0x4C, 0xD8, 0x2D, 0xFB, 0xD1, 0xD8, 0x33, 0x8A,
            0x1D, 0xBD, 0x15, 0xEE, 0x18, 0x92, 0xAE, 0x51, 0xAB, 0x41, 0xEB,
        ];

        let encrypted = ccm_star_encrypt(&key, &nonce, &aad, &plaintext).expect("encrypt");
        assert_eq!(encrypted.as_slice(), expected);
        assert_eq!(
            ccm_star_decrypt(&key, &nonce, &aad, &encrypted)
                .expect("decrypt")
                .as_slice(),
            plaintext
        );
    }

    #[test]
    fn no_aad_round_trip() {
        let key = [0x10; 16];
        let nonce = [0x20; CCM_STAR_NONCE_LEN];
        let plaintext = b"no-aad-here";
        let encrypted = ccm_star_encrypt(&key, &nonce, &[], plaintext).expect("encrypt");
        let decrypted = ccm_star_decrypt(&key, &nonce, &[], &encrypted).expect("decrypt");
        assert_eq!(decrypted.as_slice(), plaintext);
    }

    #[test]
    fn ciphertext_and_aad_tampering_are_rejected() {
        let key = [0x42; 16];
        let nonce = [0x7E; CCM_STAR_NONCE_LEN];
        let aad = [1, 2, 3, 4];
        let plaintext = [0xAB; 16];
        let mut encrypted = ccm_star_encrypt(&key, &nonce, &aad, &plaintext).expect("encrypt");

        encrypted[0] ^= 1;
        assert!(ccm_star_decrypt(&key, &nonce, &aad, &encrypted).is_none());

        let encrypted = ccm_star_encrypt(&key, &nonce, &aad, &plaintext).expect("encrypt");
        assert!(ccm_star_decrypt(&key, &nonce, &[1, 2, 3, 5], &encrypted).is_none());
    }

    #[test]
    fn invalid_lengths_are_rejected() {
        let key = [0u8; 16];
        let nonce = [0u8; CCM_STAR_NONCE_LEN];
        assert!(
            ccm_star_encrypt(&key, &nonce, &[], &[0u8; CCM_STAR_MAX_PLAINTEXT_LEN + 1]).is_none()
        );
        assert!(ccm_star_decrypt(&key, &nonce, &[], &[0u8; 3]).is_none());
        assert!(
            ccm_star_decrypt(&key, &nonce, &[], &[0u8; CCM_STAR_BUFFER_CAPACITY + 1]).is_none()
        );
    }

    /// [`ccm_star_encrypt`]/[`ccm_star_decrypt`] are documented as thin
    /// wrappers around [`ccm_star_encrypt_with`]/[`ccm_star_decrypt_with`]
    /// with [`SoftwareAes128`] — confirm that isn't just true by
    /// construction (both call sites share the same helpers) but that the
    /// generic entry points really do reproduce the exact golden vector a
    /// caller supplying its own [`Aes128Forward`] backend would get,
    /// including unwrapping the `Result` layer that only a fallible
    /// (e.g. hardware) backend would ever populate with an `Err`.
    #[test]
    fn generic_with_api_matches_default_wrapper_on_the_aps_golden_vector() {
        let key = [
            0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2, 0xD5, 0x72, 0xE1, 0xC1, 0xEF, 0x47,
            0x87, 0x82,
        ];
        let nonce = [
            0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71, 0x84, 0x53, 0x50, 0x0B, 0x00, 0x35,
        ];
        let aad = [
            0x21, 0x95, 0x35, 0x53, 0x50, 0x0B, 0x00, 0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71,
            0x84,
        ];
        let plaintext = [
            0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0x77, 0x88, 0x99, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
            0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        ];

        let mut cipher = SoftwareAes128::new(&key);
        let via_generic = ccm_star_encrypt_with(&mut cipher, &nonce, &aad, &plaintext)
            .expect("SoftwareAes128 is infallible")
            .expect("valid lengths");
        let via_wrapper = ccm_star_encrypt(&key, &nonce, &aad, &plaintext).expect("encrypt");
        assert_eq!(via_generic, via_wrapper);

        let mut cipher = SoftwareAes128::new(&key);
        let decrypted_generic = ccm_star_decrypt_with(&mut cipher, &nonce, &aad, &via_generic)
            .expect("SoftwareAes128 is infallible")
            .expect("valid MIC");
        assert_eq!(decrypted_generic.as_slice(), plaintext);
    }

    /// The inner `Option` contract (invalid lengths / rejected input) is
    /// preserved through the generic entry point too, not just the
    /// existing free-function wrappers.
    #[test]
    fn generic_with_api_preserves_invalid_length_and_tamper_rejection() {
        let key = [0x11; 16];
        let nonce = [0x22; CCM_STAR_NONCE_LEN];
        let mut cipher = SoftwareAes128::new(&key);
        assert!(
            ccm_star_encrypt_with(
                &mut cipher,
                &nonce,
                &[],
                &[0u8; CCM_STAR_MAX_PLAINTEXT_LEN + 1]
            )
            .expect("infallible")
            .is_none()
        );

        let mut cipher = SoftwareAes128::new(&key);
        let mut encrypted = ccm_star_encrypt_with(&mut cipher, &nonce, &[], b"tamper-me")
            .expect("infallible")
            .expect("valid lengths");
        encrypted[0] ^= 1;
        let mut cipher = SoftwareAes128::new(&key);
        assert!(
            ccm_star_decrypt_with(&mut cipher, &nonce, &[], &encrypted)
                .expect("infallible")
                .is_none()
        );
    }

    /// The [`ForwardAesProvider`] default hands back a cipher that drives
    /// `ccm_star_*_with` to byte-identical output as the plain
    /// [`ccm_star_encrypt`] wrapper — this is the seam every platform's
    /// software path (and the not-`_with` `zigbee-nwk`/`zigbee-aps`
    /// wrappers) rides on, so pin it against the captured APS golden
    /// vector.
    #[test]
    fn software_provider_matches_plain_wrapper() {
        let key = [
            0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2, 0xD5, 0x72, 0xE1, 0xC1, 0xEF, 0x47,
            0x87, 0x82,
        ];
        let nonce = [
            0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71, 0x84, 0x53, 0x50, 0x0B, 0x00, 0x35,
        ];
        let aad = [0x21, 0x95, 0x35, 0x53, 0x50, 0x0B, 0x00];
        let plaintext = [0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];

        let mut provider = SoftwareAesProvider::new();
        let mut cipher = provider.forward_cipher(&key);
        let via_provider = match ccm_star_encrypt_with(&mut cipher, &nonce, &aad, &plaintext) {
            Ok(Some(out)) => out,
            _ => panic!("software is infallible, valid lengths"),
        };
        drop(cipher);
        let via_wrapper = ccm_star_encrypt(&key, &nonce, &aad, &plaintext).expect("encrypt");
        assert_eq!(via_provider, via_wrapper);

        // And it round-trips through the provider-driven decrypt too.
        let mut cipher = provider.forward_cipher(&key);
        let decrypted = match ccm_star_decrypt_with(&mut cipher, &nonce, &aad, &via_provider) {
            Ok(Some(out)) => out,
            _ => panic!("software is infallible, valid MIC"),
        };
        assert_eq!(decrypted.as_slice(), plaintext);
    }

    /// A "hardware-shaped" provider — one whose cipher owns a bounded,
    /// fallible [`Aes128Forward::Error`] like the real
    /// [`tlsr8258::HardwareAes128`] — computing the *same* AES math must
    /// produce the *same* CCM* output as the software provider. This is the
    /// host stand-in for the on-silicon known-answer equivalence: it proves
    /// the generic `ccm_star_*_with` plumbing is agnostic to the backend's
    /// error type, and that a fallible backend's `Err` is surfaced (not
    /// swallowed) by threading it back out of the outer `Result`.
    #[test]
    fn fallible_provider_is_equivalent_and_surfaces_errors() {
        /// Wraps [`SoftwareAes128`] but reports a non-[`Infallible`] error,
        /// optionally failing on the Nth block to model a hardware timeout.
        struct FlakyCipher {
            inner: SoftwareAes128,
            fail_at: Option<u32>,
            calls: u32,
        }
        #[derive(Debug, PartialEq)]
        struct FakeHwError;
        impl Aes128Forward for FlakyCipher {
            type Error = FakeHwError;
            fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
                self.calls += 1;
                if self.fail_at == Some(self.calls) {
                    return Err(FakeHwError);
                }
                self.inner.encrypt_block(block).unwrap();
                Ok(())
            }
        }
        struct FlakyProvider {
            fail_at: Option<u32>,
        }
        impl ForwardAesProvider for FlakyProvider {
            fn forward_cipher(&mut self, key: &AesKey) -> impl Aes128Forward + '_ {
                FlakyCipher {
                    inner: SoftwareAes128::new(key),
                    fail_at: self.fail_at,
                    calls: 0,
                }
            }
        }

        let key = [0x33; 16];
        let nonce = [0x44; CCM_STAR_NONCE_LEN];
        let aad = [1, 2, 3, 4, 5];
        let plaintext = b"equivalence-across-backends";

        // Equivalent output to software when the backend does not fail.
        // (The provider's RPITIT return type erases the concrete `Error`,
        // so match rather than `.expect`, which would need `Error: Debug`.)
        let mut good = FlakyProvider { fail_at: None };
        let mut cipher = good.forward_cipher(&key);
        let hw = match ccm_star_encrypt_with(&mut cipher, &nonce, &aad, plaintext) {
            Ok(Some(out)) => out,
            _ => panic!("no failure requested, valid lengths"),
        };
        drop(cipher);
        let sw = ccm_star_encrypt(&key, &nonce, &aad, plaintext).expect("encrypt");
        assert_eq!(hw, sw);

        // A backend failure is surfaced as `Err`, never as a silent
        // success or a software fall-back. Drive the concrete cipher
        // directly so its `FakeHwError` is nameable here.
        let mut failing = FlakyCipher {
            inner: SoftwareAes128::new(&key),
            fail_at: Some(1),
            calls: 0,
        };
        assert_eq!(
            ccm_star_encrypt_with(&mut failing, &nonce, &aad, plaintext),
            Err(FakeHwError)
        );
    }
}
