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
    if plaintext.len() > CCM_STAR_MAX_PLAINTEXT_LEN || aad.len() > CCM_STAR_MAX_AAD_LEN {
        return None;
    }

    let cipher = Aes128::new(GenericArray::from_slice(key));
    let tag = ccm_mac(&cipher, nonce, aad, plaintext);

    let mut s0 = build_ai(nonce, 0);
    aes_enc(&cipher, &mut s0);
    let mut mic = [0u8; CCM_STAR_MIC_LEN];
    for i in 0..CCM_STAR_MIC_LEN {
        mic[i] = tag[i] ^ s0[i];
    }

    let mut buffer = [0u8; CCM_STAR_MAX_PLAINTEXT_LEN];
    buffer[..plaintext.len()].copy_from_slice(plaintext);
    ccm_ctr_xor(&cipher, nonce, &mut buffer[..plaintext.len()]);

    let mut output = heapless::Vec::new();
    output.extend_from_slice(&buffer[..plaintext.len()]).ok()?;
    output.extend_from_slice(&mic).ok()?;
    Some(output)
}

/// Authenticate and decrypt a Zigbee AES-128-CCM* payload using M=4 and L=2.
pub fn ccm_star_decrypt(
    key: &AesKey,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Option<heapless::Vec<u8, CCM_STAR_BUFFER_CAPACITY>> {
    if ciphertext_and_mic.len() < CCM_STAR_MIC_LEN || aad.len() > CCM_STAR_MAX_AAD_LEN {
        return None;
    }

    let ciphertext_len = ciphertext_and_mic.len() - CCM_STAR_MIC_LEN;
    if ciphertext_len > CCM_STAR_MAX_PLAINTEXT_LEN {
        return None;
    }

    let cipher = Aes128::new(GenericArray::from_slice(key));
    let mut buffer = [0u8; CCM_STAR_MAX_PLAINTEXT_LEN];
    buffer[..ciphertext_len].copy_from_slice(&ciphertext_and_mic[..ciphertext_len]);
    ccm_ctr_xor(&cipher, nonce, &mut buffer[..ciphertext_len]);

    let tag = ccm_mac(&cipher, nonce, aad, &buffer[..ciphertext_len]);
    let mut s0 = build_ai(nonce, 0);
    aes_enc(&cipher, &mut s0);
    let mut expected_mic = [0u8; CCM_STAR_MIC_LEN];
    for i in 0..CCM_STAR_MIC_LEN {
        expected_mic[i] = tag[i] ^ s0[i];
    }

    if !constant_time_mic_eq(
        &expected_mic,
        &ciphertext_and_mic[ciphertext_len..ciphertext_len + CCM_STAR_MIC_LEN],
    ) {
        return None;
    }

    let mut output = heapless::Vec::new();
    output.extend_from_slice(&buffer[..ciphertext_len]).ok()?;
    Some(output)
}

#[inline(always)]
fn aes_enc(cipher: &Aes128, block: &mut [u8; 16]) {
    let mut generic = GenericArray::clone_from_slice(block);
    cipher.encrypt_block(&mut generic);
    block.copy_from_slice(generic.as_slice());
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
fn mac_fold(cipher: &Aes128, tag: &mut [u8; 16], block: &[u8; 16]) {
    for i in 0..16 {
        tag[i] ^= block[i];
    }
    aes_enc(cipher, tag);
}

fn ccm_mac(
    cipher: &Aes128,
    nonce: &[u8; CCM_STAR_NONCE_LEN],
    aad: &[u8],
    payload: &[u8],
) -> [u8; 16] {
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
    aes_enc(cipher, &mut tag);

    if !aad.is_empty() {
        let mut block = [0u8; 16];
        let aad_len = aad.len() as u16;
        block[0] = (aad_len >> 8) as u8;
        block[1] = aad_len as u8;
        let first_chunk_len = core::cmp::min(aad.len(), 14);
        block[2..2 + first_chunk_len].copy_from_slice(&aad[..first_chunk_len]);
        mac_fold(cipher, &mut tag, &block);

        let mut offset = first_chunk_len;
        while offset < aad.len() {
            block = [0u8; 16];
            let chunk_len = core::cmp::min(16, aad.len() - offset);
            block[..chunk_len].copy_from_slice(&aad[offset..offset + chunk_len]);
            mac_fold(cipher, &mut tag, &block);
            offset += chunk_len;
        }
    }

    let mut offset = 0;
    while offset < payload.len() {
        let mut block = [0u8; 16];
        let chunk_len = core::cmp::min(16, payload.len() - offset);
        block[..chunk_len].copy_from_slice(&payload[offset..offset + chunk_len]);
        mac_fold(cipher, &mut tag, &block);
        offset += chunk_len;
    }

    tag
}

fn ccm_ctr_xor(cipher: &Aes128, nonce: &[u8; CCM_STAR_NONCE_LEN], data: &mut [u8]) {
    let mut counter = 1u16;
    let mut offset = 0;
    while offset < data.len() {
        let mut key_stream = build_ai(nonce, counter);
        aes_enc(cipher, &mut key_stream);
        let chunk_len = core::cmp::min(16, data.len() - offset);
        for i in 0..chunk_len {
            data[offset + i] ^= key_stream[i];
        }
        offset += chunk_len;
        counter = counter.wrapping_add(1);
    }
}

#[inline(always)]
fn constant_time_mic_eq(expected: &[u8; CCM_STAR_MIC_LEN], actual: &[u8]) -> bool {
    let mut difference = 0u8;
    for i in 0..CCM_STAR_MIC_LEN {
        difference |= expected[i] ^ actual[i];
    }
    difference == 0
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
}
