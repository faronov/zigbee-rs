//! NWK layer security — AES-128-CCM* encryption/decryption.
//!
//! Zigbee PRO uses NWK-level security for ALL routed frames:
//! - Security Level 5: ENC-MIC-32 (AES-128-CCM* with 4-byte MIC)
//! - Network key shared by all devices in the network
//! - Frame counter for replay protection
//!
//! MAC-level security is NOT used for normal Zigbee 3.0 data frames.

use zigbee_types::IeeeAddress;

/// Maximum number of network keys we can store (current + previous)
pub const MAX_NETWORK_KEYS: usize = 2;

/// A 128-bit AES key
pub type AesKey = [u8; 16];

/// NWK security material for one key
#[derive(Debug, Clone)]
pub struct NetworkKeyEntry {
    /// The 128-bit network key
    pub key: AesKey,
    /// Key sequence number (0-255)
    pub seq_number: u8,
    /// Whether this key is active
    pub active: bool,
}

/// NWK security auxiliary header (prepended to encrypted NWK payload)
#[derive(Debug, Clone)]
pub struct NwkSecurityHeader {
    /// Security control field
    pub security_control: u8,
    /// Frame counter (32-bit, for replay protection)
    pub frame_counter: u32,
    /// Source IEEE address (64-bit — identifies key origin)
    pub source_address: IeeeAddress,
    /// Key sequence number
    pub key_seq_number: u8,
}

impl NwkSecurityHeader {
    /// Security control field value for standard Zigbee:
    /// Security Level = 5 (ENC-MIC-32), Key Identifier = 1 (Network Key)
    pub const ZIGBEE_DEFAULT: u8 = 0x05 | (0x01 << 3); // level=5, key_id=1

    pub fn parse(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < 14 {
            return None;
        }
        let security_control = data[0];
        let frame_counter = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
        let mut source_address = [0u8; 8];
        source_address.copy_from_slice(&data[5..13]);
        let key_seq_number = data[13];

        Some((
            Self {
                security_control,
                frame_counter,
                source_address,
                key_seq_number,
            },
            14,
        ))
    }

    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.security_control;
        buf[1..5].copy_from_slice(&self.frame_counter.to_le_bytes());
        buf[5..13].copy_from_slice(&self.source_address);
        buf[13] = self.key_seq_number;
        14
    }
}

/// NWK security context — manages keys, frame counters, encryption.
pub struct NwkSecurity {
    /// Stored network keys
    keys: [Option<NetworkKeyEntry>; MAX_NETWORK_KEYS],
    /// Incoming frame counter table (for replay protection)
    /// Maps source IEEE address → last seen frame counter
    frame_counter_table: heapless::Vec<FrameCounterEntry, 32>,
}

#[derive(Debug, Clone)]
struct FrameCounterEntry {
    source: IeeeAddress,
    counter: u32,
}

impl NwkSecurity {
    pub fn new() -> Self {
        Self {
            keys: [None, None],
            frame_counter_table: heapless::Vec::new(),
        }
    }

    /// Set the active network key.
    pub fn set_network_key(&mut self, key: AesKey, seq_number: u8) {
        // Move current key to slot 1 (previous), new key to slot 0 (active)
        self.keys[1] = self.keys[0].take();
        self.keys[0] = Some(NetworkKeyEntry {
            key,
            seq_number,
            active: true,
        });
    }

    /// Get the active network key.
    pub fn active_key(&self) -> Option<&NetworkKeyEntry> {
        self.keys[0].as_ref().filter(|k| k.active)
    }

    /// Find key by sequence number.
    pub fn key_by_seq(&self, seq: u8) -> Option<&NetworkKeyEntry> {
        self.keys.iter().flatten().find(|k| k.seq_number == seq)
    }

    /// Check and update incoming frame counter (replay protection).
    /// Returns true if the frame counter is valid (newer than last seen).
    pub fn check_frame_counter(&mut self, source: &IeeeAddress, counter: u32) -> bool {
        if let Some(entry) = self
            .frame_counter_table
            .iter_mut()
            .find(|e| e.source == *source)
        {
            if counter > entry.counter {
                entry.counter = counter;
                true
            } else {
                false // Replay attack or duplicate
            }
        } else {
            // First frame from this source
            let _ = self.frame_counter_table.push(FrameCounterEntry {
                source: *source,
                counter,
            });
            true
        }
    }

    /// Encrypt a NWK frame payload using AES-128-CCM*.
    ///
    /// # Arguments
    /// * `nwk_header` - Serialized NWK header (used as 'a' in CCM*)
    /// * `payload` - Plaintext NWK payload to encrypt
    /// * `key` - Network key to use
    /// * `security_header` - Security auxiliary header
    ///
    /// Returns: encrypted payload + 4-byte MIC appended.
    pub fn encrypt(
        &self,
        nwk_header: &[u8],
        payload: &[u8],
        key: &AesKey,
        security_header: &NwkSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        let nonce = self.build_nonce(security_header);
        // AES-128-CCM* encryption with:
        // - M=4 (MIC length)
        // - a = nwk_header || security_header (authenticated but not encrypted)
        // - m = payload (encrypted and authenticated)
        aes_ccm_encrypt(key, &nonce, nwk_header, payload)
    }

    /// Decrypt a NWK frame payload.
    pub fn decrypt(
        &self,
        nwk_header: &[u8],
        ciphertext: &[u8],
        key: &AesKey,
        security_header: &NwkSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        let nonce = self.build_nonce(security_header);
        aes_ccm_decrypt(key, &nonce, nwk_header, ciphertext)
    }

    /// Build CCM* nonce from security header.
    /// Nonce = source_address(8) || frame_counter(4) || security_control(1)
    fn build_nonce(&self, hdr: &NwkSecurityHeader) -> [u8; 13] {
        let mut nonce = [0u8; 13];
        nonce[0..8].copy_from_slice(&hdr.source_address);
        nonce[8..12].copy_from_slice(&hdr.frame_counter.to_le_bytes());
        nonce[12] = hdr.security_control;
        nonce
    }
}

impl Default for NwkSecurity {
    fn default() -> Self {
        Self::new()
    }
}

// ── AES-128-CCM* implementation ─────────────────────────────────
// Uses RustCrypto `aes` + `ccm` crates — pure Rust, #![no_std], no alloc.
// Zigbee uses Security Level 5: ENC-MIC-32 (M=4 byte MIC, L=2).

use aes::Aes128;
use ccm::aead::AeadInPlace;
use ccm::aead::generic_array::GenericArray;
use ccm::consts::{U4, U13};
use ccm::{Ccm, KeyInit};

type ZigbeeCcm = Ccm<Aes128, U4, U13>;

/// AES-128-CCM* encrypt with M=4 (4-byte MIC).
/// Returns ciphertext + 4-byte MIC appended.
fn aes_ccm_encrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    plaintext: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    let cipher = ZigbeeCcm::new(GenericArray::from_slice(key));
    let nonce = GenericArray::from_slice(nonce);

    let mut buf = [0u8; 120];
    if plaintext.len() > buf.len() {
        return None;
    }
    buf[..plaintext.len()].copy_from_slice(plaintext);

    let tag = cipher
        .encrypt_in_place_detached(nonce, aad, &mut buf[..plaintext.len()])
        .ok()?;

    let mut out = heapless::Vec::<u8, 128>::new();
    out.extend_from_slice(&buf[..plaintext.len()]).ok()?;
    out.extend_from_slice(tag.as_slice()).ok()?;
    Some(out)
}

/// AES-128-CCM* decrypt with M=4.
/// Input is ciphertext + 4-byte MIC. Returns plaintext.
fn aes_ccm_decrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    ciphertext_with_mic: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    if ciphertext_with_mic.len() < 4 {
        return None;
    }
    let cipher = ZigbeeCcm::new(GenericArray::from_slice(key));
    let nonce = GenericArray::from_slice(nonce);

    let ct_len = ciphertext_with_mic.len() - 4;
    let mut buf = [0u8; 120];
    if ct_len > buf.len() {
        return None;
    }
    buf[..ct_len].copy_from_slice(&ciphertext_with_mic[..ct_len]);

    let tag = GenericArray::from_slice(&ciphertext_with_mic[ct_len..]);

    cipher
        .decrypt_in_place_detached(nonce, aad, &mut buf[..ct_len], tag)
        .ok()?;

    let mut out = heapless::Vec::<u8, 128>::new();
    out.extend_from_slice(&buf[..ct_len]).ok()?;
    Some(out)
}
