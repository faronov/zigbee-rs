//! Known-answer vectors for validating a platform AES-128 engine.
//!
//! Enabled with the `kat` feature (and always in this crate's own tests).
//! A hardware backend's host model test wraps its register model in a
//! [`ForwardAesProvider`] and calls [`run_all`]: the engine must reproduce
//!
//! * FIPS-197 / SP 800-38A AES-128 single-block vectors;
//! * IEEE 802.15.4-2006 Annex C.2.1 and C.2.3 CCM* frames (MIC-64 with and
//!   without encryption), checked with a reference CCM* that accepts any
//!   MIC length;
//! * Zigbee CCM* frames (`M = 4`, `L = 2`) through the production
//!   [`ccm_star_encrypt_with`] / [`ccm_star_decrypt_with`] path; and
//! * the Zigbee install-code AES-MMO derivation through
//!   [`derive_install_code_key_with`].
//!
//! Nothing here allocates; the reference CCM* is bounded by
//! [`MAX_REFERENCE_LEN`].

use crate::{
    Aes128Forward, AesKey, CCM_STAR_NONCE_LEN, ForwardAesProvider, ccm_star_decrypt_with,
    ccm_star_encrypt_with, derive_install_code_key_with,
};

/// One AES-128 single-block known answer.
#[derive(Debug, Clone, Copy)]
pub struct BlockVector {
    pub name: &'static str,
    pub key: AesKey,
    pub plaintext: [u8; 16],
    pub ciphertext: [u8; 16],
}

/// FIPS-197 Appendix B and C.1, SP 800-38A F.1.1 (first block) and the
/// all-zero key/plaintext block.
pub const AES128_BLOCK_VECTORS: [BlockVector; 4] = [
    BlockVector {
        name: "FIPS-197 C.1",
        key: [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f,
        ],
        plaintext: [
            0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ],
        ciphertext: [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
            0xc5, 0x5a,
        ],
    },
    BlockVector {
        name: "FIPS-197 B",
        key: [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ],
        plaintext: [
            0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d, 0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37,
            0x07, 0x34,
        ],
        ciphertext: [
            0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb, 0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a,
            0x0b, 0x32,
        ],
    },
    BlockVector {
        name: "SP 800-38A F.1.1 block 1",
        key: [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf,
            0x4f, 0x3c,
        ],
        plaintext: [
            0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93,
            0x17, 0x2a,
        ],
        ciphertext: [
            0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60, 0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66,
            0xef, 0x97,
        ],
    },
    BlockVector {
        name: "zero key, zero block",
        key: [0; 16],
        plaintext: [0; 16],
        ciphertext: [
            0x66, 0xe9, 0x4b, 0xd4, 0xef, 0x8a, 0x2c, 0x3b, 0x88, 0x4c, 0xfa, 0x59, 0xca, 0x34,
            0x2b, 0x2e,
        ],
    },
];

/// One CCM* frame: `aad` is authenticated, `plaintext` is encrypted, and
/// `sealed` is the encrypted payload followed by a `mic_len`-byte MIC.
#[derive(Debug, Clone, Copy)]
pub struct CcmStarVector {
    pub name: &'static str,
    pub key: AesKey,
    pub nonce: [u8; CCM_STAR_NONCE_LEN],
    pub aad: &'static [u8],
    pub plaintext: &'static [u8],
    pub mic_len: usize,
    pub sealed: &'static [u8],
}

const IEEE_KEY: AesKey = [
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
];

/// IEEE 802.15.4-2006 Annex C.2.1 (beacon, security level 2, MIC-64) and
/// C.2.3 (association request, security level 6, ENC-MIC-64).
pub const IEEE_802_15_4_CCM_STAR_VECTORS: [CcmStarVector; 2] = [
    CcmStarVector {
        name: "IEEE 802.15.4-2006 C.2.1 beacon MIC-64",
        key: IEEE_KEY,
        nonce: [
            0xAC, 0xDE, 0x48, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x05, 0x02,
        ],
        aad: &[
            0x08, 0xD0, 0x84, 0x21, 0x43, 0x01, 0x00, 0x00, 0x00, 0x00, 0x48, 0xDE, 0xAC, 0x02,
            0x05, 0x00, 0x00, 0x00, 0x55, 0xCF, 0x00, 0x00, 0x51, 0x52, 0x53, 0x54,
        ],
        plaintext: &[],
        mic_len: 8,
        sealed: &[0x22, 0x3B, 0xC1, 0xEC, 0x84, 0x1A, 0xB5, 0x53],
    },
    CcmStarVector {
        name: "IEEE 802.15.4-2006 C.2.3 command ENC-MIC-64",
        key: IEEE_KEY,
        nonce: [
            0xAC, 0xDE, 0x48, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x05, 0x06,
        ],
        aad: &[
            0x2B, 0xDC, 0x84, 0x21, 0x43, 0x02, 0x00, 0x00, 0x00, 0x00, 0x48, 0xDE, 0xAC, 0xFF,
            0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x48, 0xDE, 0xAC, 0x06, 0x05, 0x00, 0x00, 0x00,
            0x01,
        ],
        plaintext: &[0xCE],
        mic_len: 8,
        sealed: &[0xD8, 0x4F, 0xDE, 0x52, 0x90, 0x61, 0xF9, 0xC6, 0xF1],
    },
];

/// Zigbee CCM* (`M = 4`, `L = 2`) frames: a NWK-shaped golden vector and a
/// captured APS Transport-Key, both cross-checked against the independent
/// RustCrypto `ccm` implementation.
pub const ZIGBEE_CCM_STAR_VECTORS: [CcmStarVector; 2] = [
    CcmStarVector {
        name: "Zigbee NWK golden frame",
        key: [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0D,
        ],
        nonce: [
            0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08, 0x01, 0x00, 0x00, 0x00, 0x2D,
        ],
        aad: b"NWK-HDR+AUX",
        plaintext: b"hello-nwk-frame",
        mic_len: 4,
        sealed: &[
            0xAC, 0x17, 0x74, 0xEC, 0x17, 0x76, 0xC2, 0x7C, 0x41, 0xEE, 0x31, 0x0A, 0xE0, 0x0B,
            0x5B, 0x5A, 0xA0, 0x05, 0xC9,
        ],
    },
    CcmStarVector {
        name: "Zigbee captured APS Transport-Key",
        key: [
            0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2, 0xD5, 0x72, 0xE1, 0xC1, 0xEF, 0x47,
            0x87, 0x82,
        ],
        nonce: [
            0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71, 0x84, 0x53, 0x50, 0x0B, 0x00, 0x35,
        ],
        aad: &[
            0x21, 0x95, 0x35, 0x53, 0x50, 0x0B, 0x00, 0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71,
            0x84,
        ],
        plaintext: &[
            0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55,
            0x66, 0x77, 0x88, 0x99, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
            0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
        ],
        mic_len: 4,
        sealed: &[
            0xB6, 0x78, 0x43, 0xD6, 0x39, 0xC1, 0x70, 0xF2, 0x0B, 0x53, 0x6F, 0xDA, 0x11, 0xB4,
            0x67, 0xCA, 0xAF, 0xEC, 0xD6, 0xC2, 0x4C, 0xD8, 0x2D, 0xFB, 0xD1, 0xD8, 0x33, 0x8A,
            0x1D, 0xBD, 0x15, 0xEE, 0x18, 0x92, 0xAE, 0x51, 0xAB, 0x41, 0xEB,
        ],
    },
];

/// Silicon Labs install code (with its CRC) and the AES-MMO link key it
/// derives.
pub const INSTALL_CODE_VECTOR: ([u8; 18], AesKey) = (
    [
        0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
        0x88, 0xD4, 0x90,
    ],
    [
        0xFA, 0x80, 0x81, 0xCA, 0xAA, 0x41, 0xD5, 0xAD, 0xE9, 0xB5, 0x65, 0x87, 0x99, 0x26, 0x8B,
        0x88,
    ],
);

/// Longest AAD or payload the reference CCM* accepts.
pub const MAX_REFERENCE_LEN: usize = 128;

/// Which known answer an engine failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KatFailure {
    /// The engine reported an error for the named vector.
    Engine(&'static str),
    /// The engine produced a wrong result for the named vector.
    Mismatch(&'static str),
}

/// Run every vector in this module through `provider`.
pub fn run_all<P: ForwardAesProvider>(provider: &mut P) -> Result<(), KatFailure> {
    run_block_vectors(provider)?;
    for vector in &IEEE_802_15_4_CCM_STAR_VECTORS {
        run_reference_ccm_star(provider, vector)?;
    }
    for vector in &ZIGBEE_CCM_STAR_VECTORS {
        run_reference_ccm_star(provider, vector)?;
        run_production_ccm_star(provider, vector)?;
    }
    run_install_code(provider)
}

/// Encrypt every [`AES128_BLOCK_VECTORS`] entry, re-keying for each.
pub fn run_block_vectors<P: ForwardAesProvider>(provider: &mut P) -> Result<(), KatFailure> {
    for vector in &AES128_BLOCK_VECTORS {
        let mut block = vector.plaintext;
        provider
            .forward_cipher(&vector.key)
            .encrypt_block(&mut block)
            .map_err(|_| KatFailure::Engine(vector.name))?;
        if block != vector.ciphertext {
            return Err(KatFailure::Mismatch(vector.name));
        }
    }
    Ok(())
}

/// Seal `vector` with the reference CCM* (any MIC length) and compare.
pub fn run_reference_ccm_star<P: ForwardAesProvider>(
    provider: &mut P,
    vector: &CcmStarVector,
) -> Result<(), KatFailure> {
    let mut sealed = [0u8; MAX_REFERENCE_LEN + 16];
    let len = {
        let mut cipher = provider.forward_cipher(&vector.key);
        reference_ccm_star_seal(&mut cipher, vector, &mut sealed)?
    };
    if &sealed[..len] != vector.sealed {
        return Err(KatFailure::Mismatch(vector.name));
    }
    Ok(())
}

/// Seal and open a `M = 4` vector through the production CCM* functions.
pub fn run_production_ccm_star<P: ForwardAesProvider>(
    provider: &mut P,
    vector: &CcmStarVector,
) -> Result<(), KatFailure> {
    let mut cipher = provider.forward_cipher(&vector.key);
    let sealed = ccm_star_encrypt_with(&mut cipher, &vector.nonce, vector.aad, vector.plaintext)
        .map_err(|_| KatFailure::Engine(vector.name))?
        .ok_or(KatFailure::Mismatch(vector.name))?;
    if sealed.as_slice() != vector.sealed {
        return Err(KatFailure::Mismatch(vector.name));
    }
    let opened = ccm_star_decrypt_with(&mut cipher, &vector.nonce, vector.aad, vector.sealed)
        .map_err(|_| KatFailure::Engine(vector.name))?
        .ok_or(KatFailure::Mismatch(vector.name))?;
    if opened.as_slice() != vector.plaintext {
        return Err(KatFailure::Mismatch(vector.name));
    }
    Ok(())
}

/// Derive [`INSTALL_CODE_VECTOR`] with AES-MMO (re-keys every block).
pub fn run_install_code<P: ForwardAesProvider>(provider: &mut P) -> Result<(), KatFailure> {
    const NAME: &str = "Zigbee install-code AES-MMO";
    let (code, expected) = INSTALL_CODE_VECTOR;
    let key =
        derive_install_code_key_with(provider, &code).map_err(|_| KatFailure::Engine(NAME))?;
    if key != expected {
        return Err(KatFailure::Mismatch(NAME));
    }
    Ok(())
}

fn encrypt<C: Aes128Forward>(
    cipher: &mut C,
    block: &mut [u8; 16],
    name: &'static str,
) -> Result<(), KatFailure> {
    cipher
        .encrypt_block(block)
        .map_err(|_| KatFailure::Engine(name))
}

/// Straightforward CCM* (`L = 2`, MIC length 0, 4, 8 or 16) written from
/// IEEE 802.15.4-2006 Annex B, independent of the production `M = 4` code.
fn reference_ccm_star_seal<C: Aes128Forward>(
    cipher: &mut C,
    vector: &CcmStarVector,
    out: &mut [u8; MAX_REFERENCE_LEN + 16],
) -> Result<usize, KatFailure> {
    let name = vector.name;
    let m = vector.mic_len;
    if !matches!(m, 0 | 4 | 8 | 16)
        || vector.aad.len() > MAX_REFERENCE_LEN
        || vector.plaintext.len() > MAX_REFERENCE_LEN
    {
        return Err(KatFailure::Mismatch(name));
    }

    // Authentication: B0 then the length-prefixed AAD, then the payload,
    // each zero-padded to whole blocks.
    let mut x = [0u8; 16];
    x[0] = if vector.aad.is_empty() { 0 } else { 0x40 }
        | (if m == 0 {
            0
        } else {
            (((m - 2) / 2) as u8) << 3
        })
        | 0x01;
    x[1..14].copy_from_slice(&vector.nonce);
    x[14..16].copy_from_slice(&(vector.plaintext.len() as u16).to_be_bytes());
    encrypt(cipher, &mut x, name)?;

    let mut auth = [0u8; MAX_REFERENCE_LEN + 2 + 16];
    let mut auth_len = 0;
    if !vector.aad.is_empty() {
        auth[..2].copy_from_slice(&(vector.aad.len() as u16).to_be_bytes());
        auth[2..2 + vector.aad.len()].copy_from_slice(vector.aad);
        auth_len = (2 + vector.aad.len()).div_ceil(16) * 16;
    }
    for chunk in auth[..auth_len]
        .chunks(16)
        .chain(vector.plaintext.chunks(16))
    {
        for (state, byte) in x.iter_mut().zip(chunk) {
            *state ^= byte;
        }
        encrypt(cipher, &mut x, name)?;
    }

    // Encryption: counter blocks A_i = 0x01 | nonce | i.
    let counter_block = |index: u16| {
        let mut a = [0u8; 16];
        a[0] = 0x01;
        a[1..14].copy_from_slice(&vector.nonce);
        a[14..16].copy_from_slice(&index.to_be_bytes());
        a
    };
    let payload_len = vector.plaintext.len();
    for (index, chunk) in vector.plaintext.chunks(16).enumerate() {
        let mut keystream = counter_block(index as u16 + 1);
        encrypt(cipher, &mut keystream, name)?;
        let start = index * 16;
        for (offset, (&plain, &key)) in chunk.iter().zip(keystream.iter()).enumerate() {
            out[start + offset] = plain ^ key;
        }
    }
    let mut s0 = counter_block(0);
    encrypt(cipher, &mut s0, name)?;
    for index in 0..m {
        out[payload_len + index] = x[index] ^ s0[index];
    }
    Ok(payload_len + m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SoftwareAesProvider;

    #[test]
    fn software_aes_passes_every_known_answer() {
        assert_eq!(run_all(&mut SoftwareAesProvider::new()), Ok(()));
    }

    #[test]
    fn reference_ccm_star_matches_production_for_m4() {
        for vector in &ZIGBEE_CCM_STAR_VECTORS {
            assert_eq!(
                run_reference_ccm_star(&mut SoftwareAesProvider::new(), vector),
                Ok(())
            );
        }
    }

    #[test]
    fn a_wrong_engine_is_reported() {
        struct Broken;
        struct BrokenCipher;
        impl Aes128Forward for BrokenCipher {
            type Error = core::convert::Infallible;
            fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
                block[0] ^= 1;
                Ok(())
            }
        }
        impl ForwardAesProvider for Broken {
            fn forward_cipher(&mut self, _key: &AesKey) -> impl Aes128Forward + '_ {
                BrokenCipher
            }
        }
        assert_eq!(
            run_all(&mut Broken),
            Err(KatFailure::Mismatch("FIPS-197 C.1"))
        );
    }
}
