//! NWK layer security — AES-128-CCM* encryption/decryption.
//!
//! Zigbee PRO uses NWK-level security for ALL routed frames:
//! - Security Level 5: ENC-MIC-32 (AES-128-CCM* with 4-byte MIC)
//! - Network key shared by all devices in the network
//! - Frame counter for replay protection
//!
//! MAC-level security is NOT used for normal Zigbee 3.0 data frames.

pub use zigbee_crypto::AesKey;
use zigbee_crypto::{ccm_star_decrypt, ccm_star_encrypt};
use zigbee_types::IeeeAddress;

/// Maximum number of network keys we can store (current + previous)
pub const MAX_NETWORK_KEYS: usize = 2;
#[cfg(feature = "router")]
const MAX_FRAME_COUNTER_ENTRIES: usize = 64;
#[cfg(not(feature = "router"))]
const MAX_FRAME_COUNTER_ENTRIES: usize = 16;

/// Serialized length of the NWK security auxiliary header.
///
/// security control (1) + frame counter (4) + source IEEE (8) + key sequence (1).
pub const NWK_AUX_HEADER_LEN: usize = 14;

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
        if data.len() < NWK_AUX_HEADER_LEN {
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
            NWK_AUX_HEADER_LEN,
        ))
    }

    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.security_control;
        buf[1..5].copy_from_slice(&self.frame_counter.to_le_bytes());
        buf[5..13].copy_from_slice(&self.source_address);
        buf[13] = self.key_seq_number;
        NWK_AUX_HEADER_LEN
    }
}

/// NWK security context — manages keys, frame counters, encryption.
pub struct NwkSecurity {
    /// Stored network keys
    keys: [Option<NetworkKeyEntry>; MAX_NETWORK_KEYS],
    /// Sequence number installed by Transport-Key but not yet activated.
    staged_key_sequence: Option<u8>,
    /// Incoming frame counter table (for replay protection).
    /// Maps source IEEE address and network-key sequence to the last counter.
    frame_counter_table: heapless::Vec<FrameCounterEntry, MAX_FRAME_COUNTER_ENTRIES>,
}

#[derive(Debug, Clone)]
struct FrameCounterEntry {
    source: IeeeAddress,
    key_sequence: u8,
    counter: u32,
}

impl NwkSecurity {
    pub fn new() -> Self {
        Self {
            keys: [None, None],
            staged_key_sequence: None,
            frame_counter_table: heapless::Vec::new(),
        }
    }

    /// Set the active network key.
    pub fn set_network_key(&mut self, key: AesKey, seq_number: u8) {
        if let Some(active) = self.keys[0].as_mut()
            && active.seq_number == seq_number
        {
            let key_changed = active.key != key;
            active.key = key;
            active.active = true;
            self.staged_key_sequence = None;
            if key_changed {
                self.clear_frame_counters_for_key(seq_number);
            }
            return;
        }
        if self.keys[1]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number && entry.key == key)
        {
            self.keys.swap(0, 1);
            if let Some(active) = self.keys[0].as_mut() {
                active.active = true;
            }
            if let Some(previous) = self.keys[1].as_mut() {
                previous.active = false;
            }
            self.staged_key_sequence = None;
            return;
        }

        self.clear_frame_counters_for_key(seq_number);
        let previous = self.keys[0].take().map(|mut entry| {
            entry.active = false;
            entry
        });
        self.keys[0] = Some(NetworkKeyEntry {
            key,
            seq_number,
            active: true,
        });
        self.keys[1] = previous.filter(|entry| entry.seq_number != seq_number);
        self.staged_key_sequence = None;
        self.retain_frame_counters_for_installed_keys();
    }

    /// Install a future network key without activating it.
    ///
    /// Returns `false` if the sequence number names the active key but the
    /// supplied key material differs.
    pub fn stage_network_key(&mut self, key: AesKey, seq_number: u8) -> bool {
        let Some(active) = self.keys[0].as_ref() else {
            self.set_network_key(key, seq_number);
            return true;
        };
        if active.seq_number == seq_number {
            return active.key == key;
        }
        if self.keys[1]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number && entry.key == key)
        {
            self.staged_key_sequence = Some(seq_number);
            return true;
        }

        self.clear_frame_counters_for_key(seq_number);
        self.keys[1] = Some(NetworkKeyEntry {
            key,
            seq_number,
            active: false,
        });
        self.staged_key_sequence = Some(seq_number);
        self.retain_frame_counters_for_installed_keys();
        true
    }

    /// Return the key waiting for a future Switch-Key command.
    pub fn staged_key(&self) -> Option<&NetworkKeyEntry> {
        let sequence = self.staged_key_sequence?;
        self.key_by_seq(sequence)
    }

    /// Activate an installed network key by sequence number.
    pub fn activate_network_key(&mut self, seq_number: u8) -> bool {
        if self.keys[0]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number)
        {
            if let Some(active) = self.keys[0].as_mut() {
                active.active = true;
            }
            if let Some(previous) = self.keys[1].as_mut() {
                previous.active = false;
            }
            return true;
        }
        if self.keys[1]
            .as_ref()
            .is_some_and(|entry| entry.seq_number == seq_number)
        {
            self.keys.swap(0, 1);
            if let Some(active) = self.keys[0].as_mut() {
                active.active = true;
            }
            if let Some(previous) = self.keys[1].as_mut() {
                previous.active = false;
            }
            if self.staged_key_sequence == Some(seq_number) {
                self.staged_key_sequence = None;
            }
            return true;
        }
        false
    }

    /// Get the active network key.
    pub fn active_key(&self) -> Option<&NetworkKeyEntry> {
        self.keys[0].as_ref().filter(|k| k.active)
    }

    /// Find key by sequence number.
    pub fn key_by_seq(&self, seq: u8) -> Option<&NetworkKeyEntry> {
        self.keys.iter().flatten().find(|k| k.seq_number == seq)
    }

    fn clear_frame_counters_for_key(&mut self, key_sequence: u8) {
        self.frame_counter_table
            .retain(|entry| entry.key_sequence != key_sequence);
    }

    fn retain_frame_counters_for_installed_keys(&mut self) {
        let active_sequence = self.keys[0].as_ref().map(|entry| entry.seq_number);
        let alternate_sequence = self.keys[1].as_ref().map(|entry| entry.seq_number);
        self.frame_counter_table.retain(|entry| {
            Some(entry.key_sequence) == active_sequence
                || Some(entry.key_sequence) == alternate_sequence
        });
    }

    /// Clear all incoming replay state for one device identity.
    pub fn clear_frame_counters_for_source(&mut self, source: &IeeeAddress) {
        self.frame_counter_table
            .retain(|entry| entry.source != *source);
    }

    /// Check incoming frame counter (replay protection) WITHOUT committing.
    /// Returns true if the frame counter is valid (newer than last seen).
    /// Call `commit_frame_counter()` AFTER successful MIC verification.
    pub fn check_frame_counter(&self, source: &IeeeAddress, counter: u32) -> bool {
        let key_sequence = self.active_key().map(|entry| entry.seq_number).unwrap_or(0);
        self.check_frame_counter_for_key(source, key_sequence, counter)
    }

    /// Check an incoming counter in the replay domain of one network key.
    pub fn check_frame_counter_for_key(
        &self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
    ) -> bool {
        if let Some(entry) = self
            .frame_counter_table
            .iter()
            .find(|e| e.source == *source && e.key_sequence == key_sequence)
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
        let key_sequence = self.active_key().map(|entry| entry.seq_number).unwrap_or(0);
        self.commit_frame_counter_for_key(source, key_sequence, counter);
    }

    /// Commit a verified counter in the replay domain of one network key.
    pub fn commit_frame_counter_for_key(
        &mut self,
        source: &IeeeAddress,
        key_sequence: u8,
        counter: u32,
    ) {
        if let Some(entry) = self
            .frame_counter_table
            .iter_mut()
            .find(|e| e.source == *source && e.key_sequence == key_sequence)
        {
            entry.counter = counter;
        } else {
            // New source — add to table (already checked not full in check_frame_counter)
            let _ = self.frame_counter_table.push(FrameCounterEntry {
                source: *source,
                key_sequence,
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
        ccm_star_encrypt(key, &nonce, nwk_header, payload)
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
        ccm_star_decrypt(key, &nonce, nwk_header, ciphertext)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nwk_security_builds_the_expected_nonce() {
        let security = NwkSecurity::new();
        let header = NwkSecurityHeader {
            security_control: 0x28,
            frame_counter: 1,
            source_address: [0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08],
            key_seq_number: 0,
        };
        let expected_nonce = [
            0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08, 0x01, 0x00, 0x00, 0x00, 0x2D,
        ];

        assert_eq!(security.build_nonce(&header), expected_nonce);
    }

    #[test]
    fn nwk_security_matches_the_shared_ccm_golden_vector() {
        let key = [
            0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
            0x0C, 0x0D,
        ];
        let header = NwkSecurityHeader {
            security_control: 0x28,
            frame_counter: 1,
            source_address: [0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x08],
            key_seq_number: 0,
        };
        let expected = [
            0xAC, 0x17, 0x74, 0xEC, 0x17, 0x76, 0xC2, 0x7C, 0x41, 0xEE, 0x31, 0x0A, 0xE0, 0x0B,
            0x5B, 0x5A, 0xA0, 0x05, 0xC9,
        ];

        let encrypted = NwkSecurity::new()
            .encrypt(b"NWK-HDR+AUX", b"hello-nwk-frame", &key, &header)
            .expect("encrypt");
        assert_eq!(encrypted.as_slice(), expected);
    }

    #[test]
    fn staged_network_key_does_not_activate_until_switch_key() {
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        assert!(security.stage_network_key([0x22; 16], 2));

        assert_eq!(security.active_key().unwrap().seq_number, 1);
        assert_eq!(security.key_by_seq(2).unwrap().key, [0x22; 16]);
        assert!(!security.key_by_seq(2).unwrap().active);
        assert_eq!(security.staged_key().unwrap().seq_number, 2);

        assert!(security.activate_network_key(2));
        assert_eq!(security.active_key().unwrap().seq_number, 2);
        assert!(security.staged_key().is_none());
        assert_eq!(security.key_by_seq(1).unwrap().key, [0x11; 16]);
        assert!(!security.key_by_seq(1).unwrap().active);
        assert!(!security.activate_network_key(3));
    }

    #[test]
    fn replay_counters_are_scoped_by_network_key_sequence() {
        let source = [0x44; 8];
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        security.commit_frame_counter_for_key(&source, 1, 100);

        assert!(!security.check_frame_counter_for_key(&source, 1, 1));
        assert!(security.check_frame_counter_for_key(&source, 2, 1));
        security.commit_frame_counter_for_key(&source, 2, 1);
        assert!(!security.check_frame_counter_for_key(&source, 2, 1));
        assert!(!security.check_frame_counter_for_key(&source, 1, 100));
    }

    #[test]
    fn replacing_key_material_resets_that_sequences_replay_domain() {
        let source = [0x44; 8];
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        security.commit_frame_counter_for_key(&source, 1, 100);

        security.set_network_key([0x22; 16], 1);

        assert!(security.check_frame_counter_for_key(&source, 1, 0));
    }

    #[test]
    fn clearing_one_sources_replay_state_preserves_other_devices() {
        let source = [0x44; 8];
        let other = [0x55; 8];
        let mut security = NwkSecurity::new();
        security.set_network_key([0x11; 16], 1);
        security.commit_frame_counter_for_key(&source, 1, 100);
        security.commit_frame_counter_for_key(&source, 2, 200);
        security.commit_frame_counter_for_key(&other, 1, 300);

        security.clear_frame_counters_for_source(&source);

        assert!(security.check_frame_counter_for_key(&source, 1, 0));
        assert!(security.check_frame_counter_for_key(&source, 2, 0));
        assert!(!security.check_frame_counter_for_key(&other, 1, 1));
    }
}
