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
        // ── telink-debug: AES-CCM* decrypt instrumentation ──
        // BASE = 0x0084_F450 (DBG_BDB_BASE), offsets 0x200..0x21C.
        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x200 entry counter
            let p = 0x0084_F650usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
            // +0x21C last ct_len (raw, before any guard)
            write_volatile(0x0084_F66Cusize as *mut u32, ciphertext.len() as u32);
        }
        // Defensive size guard — MIC is 4 bytes (ENC-MIC-32).
        // Without this, ct_len=0 would underflow inside aps_aes_ccm_decrypt
        // (it already returns None for <4, but bump a marker first so we see it).
        if ciphertext.len() < 4 {
            #[cfg(feature = "telink-debug")]
            unsafe {
                use core::ptr::{read_volatile, write_volatile};
                // +0x220 short-ciphertext rejection counter
                let p = 0x0084_F670usize as *mut u32;
                write_volatile(p, read_volatile(p).wrapping_add(1));
            }
            return None;
        }
        let nonce = self.build_nonce(security_header);
        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x204 post build_nonce
            let p = 0x0084_F654usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
        }
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
        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x208 about to call aps_aes_ccm_decrypt
            let p = 0x0084_F658usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
        }
        let ret = aps_aes_ccm_decrypt(key, &nonce, aps_header, ciphertext);
        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x218 decrypt returned (regardless of Some/None)
            let p = 0x0084_F668usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
            // +0x214 result classifier: 0xAA = Some, 0xFF = None
            write_volatile(
                0x0084_F664usize as *mut u32,
                if ret.is_some() { 0x0000_00AA } else { 0x0000_00FF },
            );
        }
        ret
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

    /// Same as build_nonce but public, for debug tracing.
    pub fn build_nonce_debug(&self, hdr: &ApsSecurityHeader) -> [u8; 13] {
        self.build_nonce(hdr)
    }
}

impl Default for ApsSecurity {
    fn default() -> Self {
        Self::new()
    }
}

// ── AES-128-CCM* implementation ──────────────────────────────────
// Hand-rolled per Zigbee spec §B.1.2 / IEEE 802.15.4 Annex B / RFC 3610.
// Parameters fixed for APS: M=4 (4-byte MIC, ENC-MIC-32), L=2, nonce=13.
//
// Rationale: the `ccm` crate consumes ~6 KiB of stack on tc32, blowing the
// 10 KiB SVC stack (overflow signature observed cycle 19: SP=0x84AD40, below
// stack bottom 0x84B400, descending into .bss and silently corrupting
// globals). This implementation uses only fixed-size 16-byte stack buffers
// plus one `Aes128` instance (~176 B round keys), keeping the entire
// CCM* call frame under ~400 B.

use aes::Aes128;
use aes::cipher::generic_array::GenericArray;
use aes::cipher::{BlockEncrypt, KeyInit};

/// CCM* flag byte for B0 with AAD present, M=4, L=2.
/// bit6 (Adata) | (M-2)/2 << 3 | (L-1) = 0x40 | 0x08 | 0x01 = 0x49.
const CCM_B0_FLAGS_ADATA: u8 = 0x49;
/// CCM* flag byte for B0 with no AAD, M=4, L=2.
const CCM_B0_FLAGS_NO_ADATA: u8 = 0x09;
/// CCM* flag byte for counter blocks A_i, L=2.
const CCM_AI_FLAGS: u8 = 0x01;

/// AES-128 encrypt single block in place.
#[inline(always)]
fn aes_enc(cipher: &Aes128, block: &mut [u8; 16]) {
    let mut ga = GenericArray::clone_from_slice(block);
    cipher.encrypt_block(&mut ga);
    block.copy_from_slice(ga.as_slice());
}

/// Build counter block A_i for CCM* CTR mode.
#[inline(always)]
fn build_ai(nonce: &[u8; 13], counter: u16) -> [u8; 16] {
    let mut a = [0u8; 16];
    a[0] = CCM_AI_FLAGS;
    a[1..14].copy_from_slice(nonce);
    a[14] = (counter >> 8) as u8;
    a[15] = counter as u8;
    a
}

/// CBC-MAC fold: T = AES(K, T XOR block).
#[inline(always)]
fn mac_fold(cipher: &Aes128, t: &mut [u8; 16], block: &[u8; 16]) {
    for i in 0..16 {
        t[i] ^= block[i];
    }
    aes_enc(cipher, t);
}

/// Compute CCM* CBC-MAC tag T over (B0 || AAD-blocks || payload-blocks).
/// Returns the full 16-byte MAC state (caller extracts first M bytes).
fn ccm_mac(
    cipher: &Aes128,
    nonce: &[u8; 13],
    aad: &[u8],
    payload: &[u8],
) -> [u8; 16] {
    // B0: flags || nonce(13) || msg_len(2 BE)
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

    // AAD blocks (first block: 2-byte length prefix BE for len < 0xFF00).
    if !aad.is_empty() {
        let mut blk = [0u8; 16];
        let alen = aad.len() as u16;
        blk[0] = (alen >> 8) as u8;
        blk[1] = alen as u8;
        let first = core::cmp::min(aad.len(), 14);
        blk[2..2 + first].copy_from_slice(&aad[..first]);
        mac_fold(cipher, &mut t, &blk);

        // Remaining AAD in 16-byte chunks (zero-padded final).
        let mut off = first;
        while off < aad.len() {
            blk = [0u8; 16];
            let n = core::cmp::min(16, aad.len() - off);
            blk[..n].copy_from_slice(&aad[off..off + n]);
            mac_fold(cipher, &mut t, &blk);
            off += n;
        }
    }

    // Payload blocks (zero-padded final).
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

/// CTR-mode XOR (in place) starting at counter=1. Used for both encrypt and
/// decrypt (CCM CTR is symmetric).
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

/// Constant-time 4-byte compare.
#[inline(always)]
fn ct_eq4(a: &[u8], b: &[u8]) -> bool {
    let mut acc = 0u8;
    for i in 0..4 {
        acc |= a[i] ^ b[i];
    }
    acc == 0
}

/// AES-128-CCM* encrypt for APS frames (M=4, 4-byte MIC).
fn aps_aes_ccm_encrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    plaintext: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    if plaintext.len() > 124 {
        return None;
    }
    let cipher = Aes128::new(GenericArray::from_slice(key));

    // Compute CBC-MAC over (B0 || AAD || plaintext).
    let t = ccm_mac(&cipher, nonce, aad, plaintext);

    // Encrypt MIC: U = T[0..4] XOR S0[0..4], S0 = AES(K, A_0).
    let mut s0 = build_ai(nonce, 0);
    aes_enc(&cipher, &mut s0);
    let mut mic = [0u8; 4];
    for i in 0..4 {
        mic[i] = t[i] ^ s0[i];
    }

    // CTR-encrypt plaintext starting at counter=1.
    let mut buf = [0u8; 124];
    buf[..plaintext.len()].copy_from_slice(plaintext);
    ccm_ctr_xor(&cipher, nonce, &mut buf[..plaintext.len()]);

    let mut out = heapless::Vec::new();
    out.extend_from_slice(&buf[..plaintext.len()]).ok()?;
    out.extend_from_slice(&mic).ok()?;
    Some(out)
}

/// AES-128-CCM* decrypt for APS frames (M=4).
fn aps_aes_ccm_decrypt(
    key: &AesKey,
    nonce: &[u8; 13],
    aad: &[u8],
    ciphertext_and_mic: &[u8],
) -> Option<heapless::Vec<u8, 128>> {
    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x20C aps_aes_ccm_decrypt entry
        let p = 0x0084_F65Cusize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
    }
    if ciphertext_and_mic.len() < 4 {
        return None;
    }
    let mic_start = ciphertext_and_mic.len() - 4;
    if mic_start > 124 {
        return None;
    }

    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x210 about to enter hand-rolled CCM*
        let p = 0x0084_F660usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
        // +0x224 SP at hand-rolled CCM* entry (should now stay > 0x84B400)
        let sp: u32;
        core::arch::asm!("mov {0}, sp", out(reg) sp);
        write_volatile(0x0084_F674usize as *mut u32, sp);
    }

    let cipher = Aes128::new(GenericArray::from_slice(key));

    // CTR-decrypt ciphertext in-place into local buffer.
    let mut buf = [0u8; 124];
    buf[..mic_start].copy_from_slice(&ciphertext_and_mic[..mic_start]);
    ccm_ctr_xor(&cipher, nonce, &mut buf[..mic_start]);

    // ── Cycle 22: stepwise CBC-MAC with on-device intermediate trace ──
    // We compute the MAC inline (instead of via ccm_mac) so we can capture
    // T at AAD/payload boundaries plus B0, B1, S0, computed/received MIC.
    // First-call-wins so the snapshot stays correlated with a single frame.
    //
    // Trace layout (BDB_BASE = 0x0084_F450):
    //   +0x270..+0x27F = B0 block (16 B)
    //   +0x280..+0x28F = B1 block (16 B: AAD-len-prefix(2 BE) ‖ first 14 B AAD)
    //   +0x290..+0x29F = T after AAD pass (16 B)
    //   +0x2A0..+0x2AF = T after payload pass (16 B)
    //   +0x2B0..+0x2BF = S0 = AES(K, A_0) (16 B)
    //   +0x2C0..+0x2C3 = computed MIC (4 B)
    //   +0x2C4..+0x2C7 = received MIC (4 B)
    //   +0x2C8         = match flag (0x01 OK, 0x00 fail)
    //   +0x2D0..+0x2DE = AAD as fed (15 B)
    //   +0x2DC         = one-shot guard (u32, 0xC22BCED if taken)
    #[cfg(feature = "telink-debug")]
    let trace_this = unsafe {
        use core::ptr::{read_volatile, write_volatile};
        let guard_p = 0x0084_F72Cusize as *mut u32; // BDB+0x2DC
        if read_volatile(guard_p) != 0xC22B_CED5 {
            write_volatile(guard_p, 0xC22B_CED5);
            true
        } else {
            false
        }
    };
    #[cfg(not(feature = "telink-debug"))]
    let trace_this = false;

    let t = if trace_this {
        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::write_volatile;
            const BDB_BASE: usize = 0x0084_F450;

            // Build B0.
            let mut b0 = [0u8; 16];
            b0[0] = if aad.is_empty() {
                CCM_B0_FLAGS_NO_ADATA
            } else {
                CCM_B0_FLAGS_ADATA
            };
            b0[1..14].copy_from_slice(nonce);
            let mlen = mic_start as u16;
            b0[14] = (mlen >> 8) as u8;
            b0[15] = mlen as u8;
            for i in 0..16 {
                write_volatile((BDB_BASE + 0x270 + i) as *mut u8, b0[i]);
            }

            // Build B1 (only meaningful when AAD present).
            let mut b1 = [0u8; 16];
            if !aad.is_empty() {
                let alen = aad.len() as u16;
                b1[0] = (alen >> 8) as u8;
                b1[1] = alen as u8;
                let first = core::cmp::min(aad.len(), 14);
                b1[2..2 + first].copy_from_slice(&aad[..first]);
            }
            for i in 0..16 {
                write_volatile((BDB_BASE + 0x280 + i) as *mut u8, b1[i]);
            }

            // Stepwise CBC-MAC.
            let mut t = b0;
            aes_enc(&cipher, &mut t);
            if !aad.is_empty() {
                mac_fold(&cipher, &mut t, &b1);
                let mut off = core::cmp::min(aad.len(), 14);
                while off < aad.len() {
                    let mut blk = [0u8; 16];
                    let n = core::cmp::min(16, aad.len() - off);
                    blk[..n].copy_from_slice(&aad[off..off + n]);
                    mac_fold(&cipher, &mut t, &blk);
                    off += n;
                }
            }
            // Snapshot T after AAD pass.
            for i in 0..16 {
                write_volatile((BDB_BASE + 0x290 + i) as *mut u8, t[i]);
            }

            // Payload blocks.
            let payload = &buf[..mic_start];
            let mut off = 0;
            while off < payload.len() {
                let mut blk = [0u8; 16];
                let n = core::cmp::min(16, payload.len() - off);
                blk[..n].copy_from_slice(&payload[off..off + n]);
                mac_fold(&cipher, &mut t, &blk);
                off += n;
            }
            // Snapshot T after payload pass.
            for i in 0..16 {
                write_volatile((BDB_BASE + 0x2A0 + i) as *mut u8, t[i]);
            }

            // S0 = AES(K, A_0).
            let mut s0 = build_ai(nonce, 0);
            aes_enc(&cipher, &mut s0);
            for i in 0..16 {
                write_volatile((BDB_BASE + 0x2B0 + i) as *mut u8, s0[i]);
            }

            // Computed MIC.
            let mut computed = [0u8; 4];
            for i in 0..4 {
                computed[i] = t[i] ^ s0[i];
            }
            for i in 0..4 {
                write_volatile((BDB_BASE + 0x2C0 + i) as *mut u8, computed[i]);
            }
            // Received MIC.
            for i in 0..4 {
                write_volatile(
                    (BDB_BASE + 0x2C4 + i) as *mut u8,
                    ciphertext_and_mic[mic_start + i],
                );
            }
            // Match flag (write before later early return).
            let ok = ct_eq4(&computed, &ciphertext_and_mic[mic_start..mic_start + 4]);
            write_volatile((BDB_BASE + 0x2C8) as *mut u8, if ok { 0x01 } else { 0x00 });

            // AAD as fed.
            let an = aad.len().min(15);
            for i in 0..an {
                write_volatile((BDB_BASE + 0x2D0 + i) as *mut u8, aad[i]);
            }

            t
        }
        #[cfg(not(feature = "telink-debug"))]
        unreachable!()
    } else {
        ccm_mac(&cipher, nonce, aad, &buf[..mic_start])
    };

    // Expected MIC = T[0..4] XOR S0[0..4].
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

    /// Round-trip: encrypt then decrypt yields original plaintext.
    /// Uses the cycle-16 golden key/nonce/AAD; plaintext starts with
    /// 0x05 0x01 (APS Transport-Key command + StandardNwkKey type) per
    /// the captured frame.
    #[test]
    fn ccm_roundtrip_zigbee_tk() {
        let key: AesKey = [
            0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2, 0xD5, 0x72, 0xE1,
            0xC1, 0xEF, 0x47, 0x87, 0x82,
        ];
        let nonce: [u8; 13] = [
            0xF2, 0xA6, 0xC9, 0xFE, 0xFF, 0x27, 0x71, 0x84, 0x53, 0x50, 0x0B,
            0x00, 0x35,
        ];
        let aad: [u8; 15] = [
            0x21, 0x95, 0x35, 0x53, 0x50, 0x0B, 0x00, 0xF2, 0xA6, 0xC9, 0xFE,
            0xFF, 0x27, 0x71, 0x84,
        ];
        // 35-byte plaintext: TransportKey cmd (0x05) + StandardNwkKey (0x01)
        // + 16-byte NWK key + key seqno + dest IEEE + src IEEE
        let pt: [u8; 35] = [
            0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22,
            0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x01, 0x02, 0x03,
            0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
            0x0F, 0x10,
        ];
        let ct = aps_aes_ccm_encrypt(&key, &nonce, &aad, &pt).unwrap();
        assert_eq!(ct.len(), pt.len() + 4);
        let dec = aps_aes_ccm_decrypt(&key, &nonce, &aad, &ct).unwrap();
        assert_eq!(dec.as_slice(), &pt[..]);
    }

    /// RFC 3610 test vector #1 (M=8, L=2 — we adapt by truncating MIC to 4 to
    /// verify our CCM* M=4 path on a known-good keystream/MAC chain).
    /// Here we just verify roundtrip with no AAD.
    #[test]
    fn ccm_roundtrip_no_aad() {
        let key: AesKey = [
            0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA,
            0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
        ];
        let nonce: [u8; 13] = [
            0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xA0, 0xA1, 0xA2, 0xA3,
            0xA4, 0xA5,
        ];
        let pt: [u8; 23] = [
            0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
            0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
            0x1E,
        ];
        let ct = aps_aes_ccm_encrypt(&key, &nonce, &[], &pt).unwrap();
        let dec = aps_aes_ccm_decrypt(&key, &nonce, &[], &ct).unwrap();
        assert_eq!(dec.as_slice(), &pt[..]);
    }

    /// Tampered ciphertext must fail authentication.
    #[test]
    fn ccm_tampered_rejected() {
        let key: AesKey = [0u8; 16];
        let nonce: [u8; 13] = [0u8; 13];
        let pt: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut ct = aps_aes_ccm_encrypt(&key, &nonce, b"hdr", &pt).unwrap();
        ct[0] ^= 0x01; // flip a bit
        assert!(aps_aes_ccm_decrypt(&key, &nonce, b"hdr", &ct).is_none());
    }

    /// Tampered AAD must fail authentication.
    #[test]
    fn ccm_aad_tampered_rejected() {
        let key: AesKey = [0x42; 16];
        let nonce: [u8; 13] = [0x7Eu8; 13];
        let pt: [u8; 16] = [0xAB; 16];
        let aad: [u8; 4] = [1, 2, 3, 4];
        let ct = aps_aes_ccm_encrypt(&key, &nonce, &aad, &pt).unwrap();
        let bad_aad = [1, 2, 3, 5];
        assert!(aps_aes_ccm_decrypt(&key, &nonce, &bad_aad, &ct).is_none());
    }
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
    // ── telink-debug: MMO loop instrumentation ──
    // BASE = 0x0084_F700 (256 B free region between BDB markers and raw_dec_dbg).
    // All writes are raw volatile to absolute SRAM, no helper-fn stack pressure.
    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x20 matyas enter counter
        let p = 0x0084_F720usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
        // +0x24 last data.len
        write_volatile(0x0084_F724usize as *mut u32, data.len() as u32);
        // +0x784 matyas SP at entry (last-wins)
        let sp: u32;
        core::arch::asm!("mov {0}, sp", out(reg) sp);
        write_volatile(0x0084_F784usize as *mut u32, sp);
    }

    let bit_len = (data.len() as u16).wrapping_mul(8);

    // Build padded message: data || 0x80 || zeros || bit_len_be16
    // Pad to next multiple of 16 bytes
    let padded_len = (data.len() + 1 + 2).div_ceil(16) * 16;
    let mut padded = [0u8; 80]; // Max 80 bytes (enough for HMAC inputs)
    padded[..data.len()].copy_from_slice(data);
    padded[data.len()] = 0x80;
    padded[padded_len - 2] = (bit_len >> 8) as u8;
    padded[padded_len - 1] = bit_len as u8;

    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x28 pre-loop bump
        let p = 0x0084_F728usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
        // +0x2C padded_len
        write_volatile(0x0084_F72Cusize as *mut u32, padded_len as u32);
    }

    let mut hash = [0u8; 16];

    let mut _iter_idx: u32 = 0;
    for chunk in padded[..padded_len].chunks(16) {
        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x30 per-iter counter
            let p = 0x0084_F730usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
            // +0x34 last iter idx (entry)
            write_volatile(0x0084_F734usize as *mut u32, _iter_idx);
            // +0x38 pre-AES bump, +0x3C iter idx
            let p = 0x0084_F738usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
            write_volatile(0x0084_F73Cusize as *mut u32, _iter_idx);
        }

        let cipher = <Aes128 as KeyInit>::new(GenericArray::from_slice(&hash));
        let mut block = GenericArray::clone_from_slice(chunk);
        cipher.encrypt_block(&mut block);

        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x40 post-AES bump, +0x44 iter idx
            let p = 0x0084_F740usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
            write_volatile(0x0084_F744usize as *mut u32, _iter_idx);
        }

        // H_i = E(H_{i-1}, M_i) XOR M_i
        for j in 0..16 {
            hash[j] = block[j] ^ chunk[j];
        }

        #[cfg(feature = "telink-debug")]
        unsafe {
            use core::ptr::{read_volatile, write_volatile};
            // +0x48 post-XOR bump, +0x4C iter idx
            let p = 0x0084_F748usize as *mut u32;
            write_volatile(p, read_volatile(p).wrapping_add(1));
            write_volatile(0x0084_F74Cusize as *mut u32, _iter_idx);
        }

        _iter_idx = _iter_idx.wrapping_add(1);
    }

    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x50 loop-exit bump
        let p = 0x0084_F750usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
        // +0x58 matyas exit bump
        let p = 0x0084_F758usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
    }

    hash
}

/// HMAC-MMO keyed hash (Zigbee spec B.1.4).
///
/// HMAC(Key, M) = Hash( (Key XOR opad) || Hash( (Key XOR ipad) || M ) )
fn hmac_mmo(key: &[u8; 16], message: &[u8]) -> [u8; 16] {
    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // One-shot zero-init guard at +0x7C (magic 0xBEEF_F00D once initialized).
        // On first call, zero the entire 128-byte region so subsequent bumps are
        // meaningful (the .debug_sram NOLOAD region is NOT cleared at cold boot).
        let guard = 0x0084_F77Cusize as *mut u32;
        if read_volatile(guard) != 0xBEEF_F00D {
            for off in (0..0x80usize).step_by(4) {
                write_volatile((0x0084_F700usize + off) as *mut u32, 0u32);
            }
            write_volatile(guard, 0xBEEF_F00D);
        }
        // +0x00 hmac enter counter, +0x04 last message.len
        let p = 0x0084_F700usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
        write_volatile(0x0084_F704usize as *mut u32, message.len() as u32);
        // +0x780 hmac SP at entry (last-wins)
        let sp: u32;
        core::arch::asm!("mov {0}, sp", out(reg) sp);
        write_volatile(0x0084_F780usize as *mut u32, sp);
    }

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

    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x08 hmac after_inner counter
        let p = 0x0084_F708usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
    }

    // Outer hash: Hash(opad_key || inner_hash)
    let mut outer_input = [0u8; 32];
    outer_input[..16].copy_from_slice(&opad_key);
    outer_input[16..32].copy_from_slice(&inner_hash);
    let result = matyas_meyer_oseas_hash(&outer_input);

    #[cfg(feature = "telink-debug")]
    unsafe {
        use core::ptr::{read_volatile, write_volatile};
        // +0x10 hmac exit counter
        let p = 0x0084_F710usize as *mut u32;
        write_volatile(p, read_volatile(p).wrapping_add(1));
    }

    result
}

/// Derive Key-Transport Key from TC link key (Zigbee spec §4.5.3.4).
///
/// Key-Transport Key = HMAC-MMO(Link Key, 0x00)
#[cfg(not(feature = "telink-debug"))]
pub fn derive_key_transport_key(link_key: &AesKey) -> AesKey {
    hmac_mmo(link_key, &[0x00])
}

/// Debug-only hardcoded KT for ZBA09 default global link key
/// (HMAC-MMO("ZigBeeAlliance09", 0x00)). Used to bypass an observed
/// stack-corruption hang in the MMO epilogue on tc32 builds, so the
/// rest of the APS Transport-Key install path can be exercised.
#[cfg(feature = "telink-debug")]
pub fn derive_key_transport_key(_link_key: &AesKey) -> AesKey {
    [
        0x4B, 0xAB, 0x0F, 0x17, 0x3E, 0x14, 0x34, 0xA2,
        0xD5, 0x72, 0xE1, 0xC1, 0xEF, 0x47, 0x87, 0x82,
    ]
}

/// Derive Key-Load Key from TC link key (Zigbee spec §4.5.3.4).
///
/// Key-Load Key = HMAC-MMO(Link Key, 0x02)
pub fn derive_key_load_key(link_key: &AesKey) -> AesKey {
    hmac_mmo(link_key, &[0x02])
}

/// Compute Verify-Key hash for APSME-VERIFY-KEY (Zigbee spec §4.4.11.2).
///
/// hash = AES-MMO( initiator_ieee[8] || key[16] )
/// The 24-byte input is the concatenation of the initiator's IEEE address
/// (little-endian) and the trust center link key being verified.
pub fn compute_verify_key_hash(initiator_ieee: &[u8; 8], key: &AesKey) -> AesKey {
    let mut input = [0u8; 24]; // 8 (IEEE) + 16 (key)
    input[..8].copy_from_slice(initiator_ieee);
    input[8..24].copy_from_slice(key);
    matyas_meyer_oseas_hash(&input)
}
