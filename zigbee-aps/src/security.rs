//! APS security — link key encryption/decryption.
//!
//! The APS layer provides an additional security layer on top of NWK security.
//! While NWK security uses a shared network key, APS security uses per-device
//! link keys for end-to-end encryption between two specific devices.
//!
//! Key types (Zigbee spec 4.4.1):
//! - **Trust Center Master Key**: pre-installed, used to derive TC link key
//! - **Trust Center Link Key**: shared between device and TC, used to encrypt
//!   APS transport commands
//! - **Network Key**: distributed by TC, used for NWK-level encryption
//! - **Application Link Key**: shared between two application devices for
//!   APS-level end-to-end encryption
//!
//! Zigbee 3.0 default: All devices have a well-known Trust Center link key
//! ("ZigBeeAlliance09") pre-installed. During joining, the TC uses this key
//! to distribute the actual network key securely.

use zigbee_types::IeeeAddress;

/// Maximum number of link key entries.
pub const MAX_KEY_TABLE_ENTRIES: usize = 16;

/// Well-known Zigbee 3.0 default Trust Center link key ("ZigBeeAlliance09").
pub const DEFAULT_TC_LINK_KEY: [u8; 16] = [
    0x5A, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6C, // ZigBeeAl
    0x6C, 0x69, 0x61, 0x6E, 0x63, 0x65, 0x30, 0x39, // liance09
];

/// AES-128 key type alias.
pub type AesKey = [u8; 16];

// ── Key types ───────────────────────────────────────────────────

/// APS key types (Zigbee spec Table 4-15).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ApsKeyType {
    /// Trust Center Master Key (pre-installed)
    TrustCenterMasterKey = 0x00,
    /// Trust Center Link Key (derived from master or well-known)
    TrustCenterLinkKey = 0x01,
    /// Network Key (shared by all devices on the network)
    NetworkKey = 0x02,
    /// Application Link Key (between two application devices)
    ApplicationLinkKey = 0x03,
    /// Distributed Security Global Link Key (for distributed TC networks)
    DistributedGlobalLinkKey = 0x04,
}

impl ApsKeyType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::TrustCenterMasterKey),
            0x01 => Some(Self::TrustCenterLinkKey),
            0x02 => Some(Self::NetworkKey),
            0x03 => Some(Self::ApplicationLinkKey),
            0x04 => Some(Self::DistributedGlobalLinkKey),
            _ => None,
        }
    }
}

// ── APS security header (auxiliary header) ──────────────────────

/// APS auxiliary security header (Zigbee spec 4.5.1).
///
/// Prepended to the APS payload when APS security is enabled.
/// ```text
/// ┌─────────────────────────────────────────────────────────┐
/// │ Security Control (1 byte)                                │
/// │  ├── Security Level    (bits 0-2)                        │
/// │  ├── Key Identifier    (bits 3-4)                        │
/// │  └── Extended Nonce    (bit 5)                            │
/// ├─────────────────────────────────────────────────────────┤
/// │ Frame Counter (4 bytes LE)                                │
/// │ Source Address (8 bytes) — if Extended Nonce bit set      │
/// │ Key Sequence Number (1 byte) — if Key ID = Network Key   │
/// └─────────────────────────────────────────────────────────┘
/// ```
#[derive(Debug, Clone)]
pub struct ApsSecurityHeader {
    /// Security control field
    pub security_control: u8,
    /// Frame counter (for replay protection)
    pub frame_counter: u32,
    /// Source IEEE address (present if extended nonce bit set)
    pub source_address: Option<IeeeAddress>,
    /// Key sequence number (present if key identifier = 0x01 Network Key)
    pub key_seq_number: Option<u8>,
}

/// Security level values (Zigbee spec Table 4-28)
pub const SEC_LEVEL_NONE: u8 = 0x00;
pub const SEC_LEVEL_MIC_32: u8 = 0x01;
pub const SEC_LEVEL_MIC_64: u8 = 0x02;
pub const SEC_LEVEL_MIC_128: u8 = 0x03;
pub const SEC_LEVEL_ENC: u8 = 0x04;
pub const SEC_LEVEL_ENC_MIC_32: u8 = 0x05;
pub const SEC_LEVEL_ENC_MIC_64: u8 = 0x06;
pub const SEC_LEVEL_ENC_MIC_128: u8 = 0x07;

/// Key identifier values (Zigbee spec Table 4-29)
pub const KEY_ID_DATA_KEY: u8 = 0x00; // Link key
pub const KEY_ID_NETWORK_KEY: u8 = 0x01; // Network key
pub const KEY_ID_KEY_TRANSPORT: u8 = 0x02; // Key-transport key
pub const KEY_ID_KEY_LOAD: u8 = 0x03; // Key-load key

impl ApsSecurityHeader {
    /// APS security default: Security Level 5 (ENC-MIC-32), Key ID = Data Key
    pub const APS_DEFAULT: u8 = SEC_LEVEL_ENC_MIC_32 | (KEY_ID_DATA_KEY << 3);

    /// APS security with extended nonce
    pub const APS_DEFAULT_EXT_NONCE: u8 = SEC_LEVEL_ENC_MIC_32 | (KEY_ID_DATA_KEY << 3) | (1 << 5);

    /// Extract security level from security control byte.
    pub fn security_level(sc: u8) -> u8 {
        sc & 0x07
    }

    /// Extract key identifier from security control byte.
    pub fn key_identifier(sc: u8) -> u8 {
        (sc >> 3) & 0x03
    }

    /// Whether extended nonce is present.
    pub fn extended_nonce(sc: u8) -> bool {
        (sc >> 5) & 1 != 0
    }

    /// Parse from raw bytes. Returns (header, bytes_consumed).
    pub fn parse(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < 5 {
            return None;
        }

        let security_control = data[0];
        let frame_counter = u32::from_le_bytes([data[1], data[2], data[3], data[4]]);
        let mut offset = 5;

        // Extended nonce: 8-byte source address
        let source_address = if Self::extended_nonce(security_control) {
            if data.len() < offset + 8 {
                return None;
            }
            let mut addr = [0u8; 8];
            addr.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            Some(addr)
        } else {
            None
        };

        // Key sequence number: present when Key ID = 0x01 (Network Key)
        let key_seq_number = if Self::key_identifier(security_control) == KEY_ID_NETWORK_KEY {
            if data.len() <= offset {
                return None;
            }
            let seq = data[offset];
            offset += 1;
            Some(seq)
        } else {
            None
        };

        Some((
            Self {
                security_control,
                frame_counter,
                source_address,
                key_seq_number,
            },
            offset,
        ))
    }

    /// Serialize into a buffer. Returns bytes written.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.security_control;
        buf[1..5].copy_from_slice(&self.frame_counter.to_le_bytes());
        let mut offset = 5;

        if let Some(ref addr) = self.source_address {
            buf[offset..offset + 8].copy_from_slice(addr);
            offset += 8;
        }

        if let Some(seq) = self.key_seq_number {
            buf[offset] = seq;
            offset += 1;
        }

        offset
    }
}

// ── Link key table entry ────────────────────────────────────────

/// APS link key table entry — stores a per-device key.
#[derive(Debug, Clone)]
pub struct ApsLinkKeyEntry {
    /// Partner device IEEE address
    pub partner_address: IeeeAddress,
    /// 128-bit link key
    pub key: AesKey,
    /// Key type (TC link key or application link key)
    pub key_type: ApsKeyType,
    /// Outgoing frame counter for this key
    pub outgoing_frame_counter: u32,
    /// Incoming frame counter for replay protection
    pub incoming_frame_counter: u32,
}

// ── APS Security context ────────────────────────────────────────

/// APS security state — manages link keys and APS-level encryption.
pub struct ApsSecurity {
    /// Link key table
    key_table: heapless::Vec<ApsLinkKeyEntry, MAX_KEY_TABLE_ENTRIES>,
    /// Pre-configured Trust Center link key (default: ZigBeeAlliance09)
    default_tc_link_key: AesKey,
}

impl ApsSecurity {
    pub fn new() -> Self {
        Self {
            key_table: heapless::Vec::new(),
            default_tc_link_key: DEFAULT_TC_LINK_KEY,
        }
    }

    /// Set the default Trust Center link key.
    pub fn set_default_tc_link_key(&mut self, key: AesKey) {
        self.default_tc_link_key = key;
    }

    /// Get the default Trust Center link key.
    pub fn default_tc_link_key(&self) -> &AesKey {
        &self.default_tc_link_key
    }

    /// Add a link key to the key table. Returns Err if table is full.
    pub fn add_key(&mut self, entry: ApsLinkKeyEntry) -> Result<(), ApsLinkKeyEntry> {
        // Update existing entry for same partner
        if let Some(existing) = self
            .key_table
            .iter_mut()
            .find(|e| e.partner_address == entry.partner_address && e.key_type == entry.key_type)
        {
            existing.key = entry.key;
            existing.outgoing_frame_counter = entry.outgoing_frame_counter;
            existing.incoming_frame_counter = entry.incoming_frame_counter;
            return Ok(());
        }
        self.key_table.push(entry)
    }

    /// Remove a link key by partner address and key type.
    pub fn remove_key(&mut self, partner: &IeeeAddress, key_type: ApsKeyType) -> bool {
        if let Some(idx) = self
            .key_table
            .iter()
            .position(|e| e.partner_address == *partner && e.key_type == key_type)
        {
            self.key_table.swap_remove(idx);
            true
        } else {
            false
        }
    }

    /// Find a link key for a partner device.
    pub fn find_key(
        &self,
        partner: &IeeeAddress,
        key_type: ApsKeyType,
    ) -> Option<&ApsLinkKeyEntry> {
        self.key_table
            .iter()
            .find(|e| e.partner_address == *partner && e.key_type == key_type)
    }

    /// Find any link key for a partner device (TC link key preferred).
    pub fn find_any_key(&self, partner: &IeeeAddress) -> Option<&ApsLinkKeyEntry> {
        // Prefer TC link key, then application link key
        self.find_key(partner, ApsKeyType::TrustCenterLinkKey)
            .or_else(|| self.find_key(partner, ApsKeyType::ApplicationLinkKey))
    }

    /// Get the key table as a slice.
    pub fn key_table(&self) -> &[ApsLinkKeyEntry] {
        &self.key_table
    }

    /// Number of keys in the table.
    pub fn key_count(&self) -> usize {
        self.key_table.len()
    }

    /// Check incoming frame counter for replay (read-only — does NOT update state).
    /// Returns true if the frame counter is valid (newer than last seen).
    /// Call `commit_frame_counter()` only after successful MIC verification.
    pub fn check_frame_counter(
        &self,
        partner: &IeeeAddress,
        key_type: ApsKeyType,
        counter: u32,
    ) -> bool {
        if let Some(entry) = self
            .key_table
            .iter()
            .find(|e| e.partner_address == *partner && e.key_type == key_type)
        {
            counter > entry.incoming_frame_counter
        } else {
            // Unknown partner — allow with default TC link key (first contact)
            true
        }
    }

    /// Commit frame counter after successful decryption + MIC verification.
    /// This is the second phase of the two-phase replay protection.
    pub fn commit_frame_counter(
        &mut self,
        partner: &IeeeAddress,
        key_type: ApsKeyType,
        counter: u32,
    ) {
        if let Some(entry) = self
            .key_table
            .iter_mut()
            .find(|e| e.partner_address == *partner && e.key_type == key_type)
            .filter(|e| counter > e.incoming_frame_counter)
        {
            entry.incoming_frame_counter = counter;
        }
    }

    /// Increment outgoing frame counter for a partner key.
    /// Returns the pre-increment value.
    pub fn next_frame_counter(
        &mut self,
        partner: &IeeeAddress,
        key_type: ApsKeyType,
    ) -> Option<u32> {
        if let Some(entry) = self
            .key_table
            .iter_mut()
            .find(|e| e.partner_address == *partner && e.key_type == key_type)
        {
            let fc = entry.outgoing_frame_counter;
            entry.outgoing_frame_counter = entry.outgoing_frame_counter.wrapping_add(1);
            Some(fc)
        } else {
            None
        }
    }

    /// Encrypt an APS payload using AES-128-CCM* with a link key.
    ///
    /// # Arguments
    /// * `aps_header` - Serialized APS header (authenticated but not encrypted)
    /// * `payload` - Plaintext APS payload
    /// * `key` - Link key to use
    /// * `security_header` - APS security auxiliary header
    ///
    /// Returns: encrypted payload + 4-byte MIC appended.
    pub fn encrypt(
        &self,
        aps_header: &[u8],
        payload: &[u8],
        key: &AesKey,
        security_header: &ApsSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        let nonce = self.build_nonce(security_header);
        aps_aes_ccm_encrypt(key, &nonce, aps_header, payload)
    }

    /// Decrypt an APS payload.
    pub fn decrypt(
        &self,
        aps_header: &[u8],
        ciphertext: &[u8],
        key: &AesKey,
        security_header: &ApsSecurityHeader,
    ) -> Option<heapless::Vec<u8, 128>> {
        let nonce = self.build_nonce(security_header);
        #[cfg(feature = "defmt")]
        {
            defmt::info!("[APS-SEC] key: {:02x}", key);
            defmt::info!("[APS-SEC] nonce: {:02x}", &nonce[..]);
            defmt::info!(
                "[APS-SEC] aad({} bytes): {:02x}",
                aps_header.len(),
                aps_header
            );
            defmt::info!(
                "[APS-SEC] ciphertext({} bytes): {:02x}",
                ciphertext.len(),
                ciphertext
            );
        }
        aps_aes_ccm_decrypt(key, &nonce, aps_header, ciphertext)
    }

    /// Build CCM* nonce from APS security header.
    /// Nonce (13 bytes) = source_address(8) || frame_counter(4) || security_control(1)
    ///
    /// Per Zigbee spec B.4.1: the SecurityLevel in the nonce must use the ACTUAL
    /// security level (5 = ENC-MIC-32), not the OTA value (always 0 on the wire).
    fn build_nonce(&self, hdr: &ApsSecurityHeader) -> [u8; 13] {
        let mut nonce = [0u8; 13];
        if let Some(ref addr) = hdr.source_address {
            nonce[0..8].copy_from_slice(addr);
        }
        nonce[8..12].copy_from_slice(&hdr.frame_counter.to_le_bytes());
        // Replace OTA security level (0) with actual level (5 = ENC-MIC-32)
        let actual_sc = (hdr.security_control & !0x07) | SEC_LEVEL_ENC_MIC_32;
        nonce[12] = actual_sc;
        nonce
    }
}

impl Default for ApsSecurity {
    fn default() -> Self {
        Self::new()
    }
}

// ── AES-128-CCM* implementation ──────────────────────────────────
// Uses RustCrypto `aes` + `ccm` crates — pure Rust, #![no_std], no alloc.
// APS uses Security Level 5: ENC-MIC-32 (M=4 byte MIC, L=2).

use aes::Aes128;
use aes::cipher::BlockEncrypt;
use ccm::aead::AeadInPlace;
use ccm::aead::generic_array::GenericArray;
use ccm::consts::{U4, U13};
use ccm::{Ccm, KeyInit};

type ApsCcm = Ccm<Aes128, U4, U13>;

/// AES-128-CCM* encrypt for APS frames (M=4, 4-byte MIC).
fn aps_aes_ccm_encrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    plaintext: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    let cipher = ApsCcm::new(GenericArray::from_slice(key));
    let nonce = GenericArray::from_slice(nonce);

    let mut buf = [0u8; 120];
    if plaintext.len() > buf.len() {
        return None;
    }
    buf[..plaintext.len()].copy_from_slice(plaintext);

    let tag = cipher
        .encrypt_in_place_detached(nonce, aad, &mut buf[..plaintext.len()])
        .ok()?;

    let mut out = heapless::Vec::new();
    out.extend_from_slice(&buf[..plaintext.len()]).ok()?;
    out.extend_from_slice(tag.as_slice()).ok()?;
    Some(out)
}

/// AES-128-CCM* decrypt for APS frames (M=4).
fn aps_aes_ccm_decrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    if ciphertext_and_mic.len() < 4 {
        return None;
    }
    let cipher = ApsCcm::new(GenericArray::from_slice(key));
    let nonce = GenericArray::from_slice(nonce);

    let mic_start = ciphertext_and_mic.len() - 4;
    let mut buf = [0u8; 120];
    if mic_start > buf.len() {
        return None;
    }
    buf[..mic_start].copy_from_slice(&ciphertext_and_mic[..mic_start]);
    let tag = GenericArray::from_slice(&ciphertext_and_mic[mic_start..]);

    cipher
        .decrypt_in_place_detached(nonce, aad, &mut buf[..mic_start], tag)
        .ok()?;

    let mut out = heapless::Vec::new();
    out.extend_from_slice(&buf[..mic_start]).ok()?;
    Some(out)
}

// ── Matyas-Meyer-Oseas Hash & HMAC-MMO ──────────────────────────
// Used for APS key derivation (Zigbee spec Appendix B).

/// Matyas-Meyer-Oseas AES-128 block cipher hash (Zigbee spec B.1.3).
///
/// Processes `data` in 16-byte blocks:
///   H_0 = 0
///   H_i = AES(H_{i-1}, M_i) XOR M_i
///
/// Input is padded per B.6: append 0x80, zeros, then 16-bit big-endian bit-length.
fn matyas_meyer_oseas_hash(data: &[u8]) -> [u8; 16] {
    let bit_len = (data.len() as u16).wrapping_mul(8);

    // Build padded message: data || 0x80 || zeros || bit_len_be16
    // Pad to next multiple of 16 bytes
    let padded_len = (data.len() + 1 + 2).div_ceil(16) * 16;
    let mut padded = [0u8; 80]; // Max 80 bytes (enough for HMAC inputs)
    padded[..data.len()].copy_from_slice(data);
    padded[data.len()] = 0x80;
    padded[padded_len - 2] = (bit_len >> 8) as u8;
    padded[padded_len - 1] = bit_len as u8;

    let mut hash = [0u8; 16];

    for chunk in padded[..padded_len].chunks(16) {
        let cipher = <Aes128 as KeyInit>::new(GenericArray::from_slice(&hash));
        let mut block = GenericArray::clone_from_slice(chunk);
        cipher.encrypt_block(&mut block);
        // H_i = E(H_{i-1}, M_i) XOR M_i
        for j in 0..16 {
            hash[j] = block[j] ^ chunk[j];
        }
    }

    hash
}

/// HMAC-MMO keyed hash (Zigbee spec B.1.4).
///
/// HMAC(Key, M) = Hash( (Key XOR opad) || Hash( (Key XOR ipad) || M ) )
fn hmac_mmo(key: &[u8; 16], message: &[u8]) -> [u8; 16] {
    let mut ipad_key = [0x36u8; 16];
    let mut opad_key = [0x5Cu8; 16];
    for i in 0..16 {
        ipad_key[i] ^= key[i];
        opad_key[i] ^= key[i];
    }

    // Inner hash: Hash(ipad_key || message)
    let mut inner_input = [0u8; 48]; // 16 + max 16 bytes message
    inner_input[..16].copy_from_slice(&ipad_key);
    let inner_len = 16 + message.len();
    inner_input[16..inner_len].copy_from_slice(message);
    let inner_hash = matyas_meyer_oseas_hash(&inner_input[..inner_len]);

    // Outer hash: Hash(opad_key || inner_hash)
    let mut outer_input = [0u8; 32];
    outer_input[..16].copy_from_slice(&opad_key);
    outer_input[16..32].copy_from_slice(&inner_hash);
    matyas_meyer_oseas_hash(&outer_input)
}

/// Derive Key-Transport Key from TC link key (Zigbee spec §4.5.3.4).
///
/// Key-Transport Key = HMAC-MMO(Link Key, 0x00)
pub fn derive_key_transport_key(link_key: &AesKey) -> AesKey {
    hmac_mmo(link_key, &[0x00])
}

/// Derive Key-Load Key from TC link key (Zigbee spec §4.5.3.4).
///
/// Key-Load Key = HMAC-MMO(Link Key, 0x02)
pub fn derive_key_load_key(link_key: &AesKey) -> AesKey {
    hmac_mmo(link_key, &[0x02])
}
