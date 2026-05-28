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
    /// Security Level = 5 (ENC-MIC-32), Key Identifier = 1 (Network Key),
    /// Extended Nonce = 1 (source address present in aux header)
    /// Per Zigbee PRO R22 §4.5.1.1: extended nonce SHALL be set to 1.
    pub const ZIGBEE_DEFAULT: u8 = 0x05 | (0x01 << 3) | (1 << 5); // 0x2D

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

    /// Check incoming frame counter (replay protection) WITHOUT committing.
    /// Returns true if the frame counter is valid (newer than last seen).
    /// Call `commit_frame_counter()` AFTER successful MIC verification.
    pub fn check_frame_counter(&self, source: &IeeeAddress, counter: u32) -> bool {
        if let Some(entry) = self
            .frame_counter_table
            .iter()
            .find(|e| e.source == *source)
        {
            counter > entry.counter
        } else {
            // First frame from this source — accept if table has room
            if self.frame_counter_table.is_full() {
                log::warn!("[NWK] Replay table full — rejecting frame from new source");
                return false;
            }
            true
        }
    }

    /// Commit frame counter after successful MIC verification.
    /// Must only be called after decrypt/verify succeeds.
    pub fn commit_frame_counter(&mut self, source: &IeeeAddress, counter: u32) {
        if let Some(entry) = self
            .frame_counter_table
            .iter_mut()
            .find(|e| e.source == *source)
        {
            entry.counter = counter;
        } else {
            // New source — add to table (already checked not full in check_frame_counter)
            let _ = self.frame_counter_table.push(FrameCounterEntry {
                source: *source,
                counter,
            });
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
    ///
    /// Per Zigbee spec §4.3.1.2: the SecurityLevel in the nonce must use the
    /// ACTUAL security level (5 = ENC-MIC-32), not the OTA value (always 0).
    fn build_nonce(&self, hdr: &NwkSecurityHeader) -> [u8; 13] {
        let mut nonce = [0u8; 13];
        nonce[0..8].copy_from_slice(&hdr.source_address);
        nonce[8..12].copy_from_slice(&hdr.frame_counter.to_le_bytes());
        // Replace OTA security level (0) with actual level (5 = ENC-MIC-32)
        let actual_sc = (hdr.security_control & !0x07) | 0x05;
        nonce[12] = actual_sc;
        nonce
    }
}

impl Default for NwkSecurity {
    fn default() -> Self {
        Self::new()
    }
}

// ── AES-128-CCM* implementation ─────────────────────────────────
// Hand-rolled per Zigbee spec §B.1.2 / IEEE 802.15.4 Annex B / RFC 3610.
// Parameters: M=4 (4-byte MIC, ENC-MIC-32), L=2, nonce=13.
//
// IDENTICAL ALGORITHM to `zigbee-aps/src/security.rs`. Duplicated here to
// avoid a cyclic dep (APS depends on NWK). TODO: lift to a shared
// `zigbee-crypto` crate.
//
// Rationale: the `ccm` crate consumes ~6 KiB of stack on tc32, blowing the
// 10 KiB SVC stack. This impl uses only fixed-size 16-byte stack buffers
// plus one `Aes128` (~176 B round keys), keeping the call frame under ~400 B.

use aes::Aes128;
use aes::cipher::generic_array::GenericArray;
use aes::cipher::{BlockEncrypt, KeyInit};

const CCM_B0_FLAGS_ADATA: u8 = 0x49; // Adata=1, M=4, L=2
const CCM_B0_FLAGS_NO_ADATA: u8 = 0x09; // Adata=0, M=4, L=2
const CCM_AI_FLAGS: u8 = 0x01; // L=2 counter blocks

#[inline(always)]
fn aes_enc(cipher: &Aes128, block: &mut [u8; 16]) {
    let mut ga = GenericArray::clone_from_slice(block);
    cipher.encrypt_block(&mut ga);
    block.copy_from_slice(ga.as_slice());
}

#[inline(always)]
fn build_ai(nonce: &[u8; 13], counter: u16) -> [u8; 16] {
    let mut a = [0u8; 16];
    a[0] = CCM_AI_FLAGS;
    a[1..14].copy_from_slice(nonce);
    a[14] = (counter >> 8) as u8;
    a[15] = counter as u8;
    a
}

#[inline(always)]
fn mac_fold(cipher: &Aes128, t: &mut [u8; 16], block: &[u8; 16]) {
    for i in 0..16 {
        t[i] ^= block[i];
    }
    aes_enc(cipher, t);
}

fn ccm_mac(cipher: &Aes128, nonce: &[u8; 13], aad: &[u8], payload: &[u8]) -> [u8; 16] {
    let mut t = [0u8; 16];
    t[0] = if aad.is_empty() {
        CCM_B0_FLAGS_NO_ADATA
    } else {
        CCM_B0_FLAGS_ADATA
    };
    t[1..14].copy_from_slice(nonce);
    let mlen = payload.len() as u16;
    t[14] = (mlen >> 8) as u8;
    t[15] = mlen as u8;
    aes_enc(cipher, &mut t);

    if !aad.is_empty() {
        let mut blk = [0u8; 16];
        let alen = aad.len() as u16;
        blk[0] = (alen >> 8) as u8;
        blk[1] = alen as u8;
        let first = core::cmp::min(aad.len(), 14);
        blk[2..2 + first].copy_from_slice(&aad[..first]);
        mac_fold(cipher, &mut t, &blk);

        let mut off = first;
        while off < aad.len() {
            blk = [0u8; 16];
            let n = core::cmp::min(16, aad.len() - off);
            blk[..n].copy_from_slice(&aad[off..off + n]);
            mac_fold(cipher, &mut t, &blk);
            off += n;
        }
    }

    let mut off = 0;
    while off < payload.len() {
        let mut blk = [0u8; 16];
        let n = core::cmp::min(16, payload.len() - off);
        blk[..n].copy_from_slice(&payload[off..off + n]);
        mac_fold(cipher, &mut t, &blk);
        off += n;
    }
    t
}

fn ccm_ctr_xor(cipher: &Aes128, nonce: &[u8; 13], data: &mut [u8]) {
    let mut counter: u16 = 1;
    let mut off = 0;
    while off < data.len() {
        let mut ks = build_ai(nonce, counter);
        aes_enc(cipher, &mut ks);
        let n = core::cmp::min(16, data.len() - off);
        for i in 0..n {
            data[off + i] ^= ks[i];
        }
        off += n;
        counter = counter.wrapping_add(1);
    }
}

#[inline(always)]
fn ct_eq4(a: &[u8], b: &[u8]) -> bool {
    let mut acc = 0u8;
    for i in 0..4 {
        acc |= a[i] ^ b[i];
    }
    acc == 0
}

/// AES-128-CCM* encrypt for NWK frames (M=4, 4-byte MIC).
fn aes_ccm_encrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    plaintext: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // BDB+0x244 NWK CCM encrypt entries
        let p = 0x0084_F694usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
        // BDB+0x248 SP at NWK CCM encrypt entry (verify in-stack)
        let sp: u32;
        core::arch::asm!("mov {0}, sp", out(reg) sp);
        write_volatile(0x0084_F698usize as *mut u32, sp);
    }
    if plaintext.len() > 124 {
        return None;
    }
    let cipher = Aes128::new(GenericArray::from_slice(key));

    let t = ccm_mac(&cipher, nonce, aad, plaintext);

    let mut s0 = build_ai(nonce, 0);
    aes_enc(&cipher, &mut s0);
    let mut mic = [0u8; 4];
    for i in 0..4 {
        mic[i] = t[i] ^ s0[i];
    }

    let mut buf = [0u8; 124];
    buf[..plaintext.len()].copy_from_slice(plaintext);
    ccm_ctr_xor(&cipher, nonce, &mut buf[..plaintext.len()]);

    let mut out = heapless::Vec::new();
    out.extend_from_slice(&buf[..plaintext.len()]).ok()?;
    out.extend_from_slice(&mic).ok()?;
    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // BDB+0x24C NWK CCM encrypt successful exits
        let p = 0x0084_F69Cusize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
    }
    Some(out)
}

/// AES-128-CCM* decrypt for NWK frames (M=4).
fn aes_ccm_decrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    if ciphertext_and_mic.len() < 4 {
        return None;
    }
    let mic_start = ciphertext_and_mic.len() - 4;
    if mic_start > 124 {
        return None;
    }
    let cipher = Aes128::new(GenericArray::from_slice(key));

    let mut buf = [0u8; 124];
    buf[..mic_start].copy_from_slice(&ciphertext_and_mic[..mic_start]);
    ccm_ctr_xor(&cipher, nonce, &mut buf[..mic_start]);

    let t = ccm_mac(&cipher, nonce, aad, &buf[..mic_start]);

    let mut s0 = build_ai(nonce, 0);
    aes_enc(&cipher, &mut s0);
    let mut expected = [0u8; 4];
    for i in 0..4 {
        expected[i] = t[i] ^ s0[i];
    }

    if !ct_eq4(&expected, &ciphertext_and_mic[mic_start..mic_start + 4]) {
        return None;
    }

    let mut out = heapless::Vec::new();
    out.extend_from_slice(&buf[..mic_start]).ok()?;
    Some(out)
}

#[cfg(test)]
mod ccm_tests {
    use super::*;

    /// Round-trip the hand-rolled NWK CCM* (M=4, L=2, 13-byte nonce).
    #[test]
    fn roundtrip_zigbee_default_level() {
        let key: AesKey = [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0D,
        ];
        // src_ieee(8) || frame_counter(4 LE = 1) || sec_control (0x2D)
        let nonce: [u8; 13] = [
            0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08, 0x01, 0x00, 0x00, 0x00, 0x2D,
        ];
        let aad = b"NWK-HDR+AUX";
        let payload = b"hello-nwk-frame";

        let ct = aes_ccm_encrypt(&key, &nonce, aad, payload).expect("encrypt");
        assert_eq!(ct.len(), payload.len() + 4);
        let pt = aes_ccm_decrypt(&key, &nonce, aad, &ct).expect("decrypt");
        assert_eq!(&pt[..], payload);
    }

    #[test]
    fn tamper_rejected() {
        let key: AesKey = [0x42; 16];
        let nonce: [u8; 13] = [0x55; 13];
        let aad = b"aad";
        let payload = b"abcd1234";

        let mut ct = aes_ccm_encrypt(&key, &nonce, aad, payload).unwrap();
        ct[0] ^= 0x01; // flip a bit in ciphertext
        assert!(aes_ccm_decrypt(&key, &nonce, aad, &ct).is_none());
    }

    #[test]
    fn no_aad_roundtrip() {
        let key: AesKey = [0x10; 16];
        let nonce: [u8; 13] = [0x20; 13];
        let payload = b"no-aad-here";
        let ct = aes_ccm_encrypt(&key, &nonce, &[], payload).unwrap();
        let pt = aes_ccm_decrypt(&key, &nonce, &[], &ct).unwrap();
        assert_eq!(&pt[..], payload);
    }
}
