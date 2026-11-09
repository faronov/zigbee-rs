//! MCUboot image header, TLV trailer and validation against flash.
//!
//! The format is the one `imgtool sign` produces (MCUboot
//! `bootutil/image.h`): a little-endian header at the start of the slot,
//! the firmware, an optional protected TLV area and an unprotected TLV area.
//! The SHA-256 TLV covers header, firmware and protected TLVs; the signature
//! TLV (ECDSA P-256, or Ed25519 with the `ed25519` feature) signs that
//! digest; the KEYHASH TLV before it is SHA-256 of the signer's DER
//! `SubjectPublicKeyInfo` and selects which trusted key must verify it.
//!
//! A loader trusts up to [`MAX_TRUSTED_KEYS`] keys at once so that a
//! signing key can be rotated: an image is accepted when any trusted key
//! verifies one of its signatures.

use core::cmp::Ordering;

use embedded_storage::nor_flash::ReadNorFlash;
use sha2::{Digest, Sha256};

use crate::p256::{VerifyingKey, parse_der_signature};

pub const IMAGE_MAGIC: u32 = 0x96F3_B83D;
pub const IMAGE_HEADER_LEN: usize = 32;
pub const TLV_INFO_MAGIC: u16 = 0x6907;
pub const TLV_PROTECTED_INFO_MAGIC: u16 = 0x6908;
pub const TLV_KEYHASH: u16 = 0x01;
pub const TLV_SHA256: u16 = 0x10;
pub const TLV_ECDSA_SIG: u16 = 0x22;
/// imgtool's Ed25519 signature over the SHA-256 digest (not `--pure`).
pub const TLV_ED25519: u16 = 0x24;

/// Most keys a loader or an OTA policy trusts at once (current key plus
/// rotation successors).
pub const MAX_TRUSTED_KEYS: usize = 3;

/// DER `SubjectPublicKeyInfo` prefix of every uncompressed P-256 key, up to
/// and including the SEC1 `0x04` point tag. imgtool hashes the complete
/// `SubjectPublicKeyInfo` into the KEYHASH TLV.
pub const P256_SPKI_PREFIX: [u8; 27] = [
    0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x02, 0x01, 0x06, 0x08, 0x2A,
    0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04,
];

/// DER `SubjectPublicKeyInfo` prefix of an Ed25519 key (RFC 8410), up to
/// the 32-byte key itself.
pub const ED25519_SPKI_PREFIX: [u8; 12] = [
    0x30, 0x2A, 0x30, 0x05, 0x06, 0x03, 0x2B, 0x65, 0x70, 0x03, 0x21, 0x00,
];

/// Chunk used to stream flash into SHA-256; bounded stack use.
const READ_CHUNK: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    /// The flash read failed.
    Flash,
    /// No image header magic.
    NoImage,
    /// Header or TLV geometry is inconsistent or exceeds the slot.
    Malformed,
    /// Unsupported header flags (encrypted, RAM-load, non-bootable, ...).
    Unsupported,
    /// The SHA-256 TLV is missing or does not match the image.
    HashMismatch,
    /// No signature from the trusted key verifies.
    BadSignature,
}

/// MCUboot `image_version`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ImageVersion {
    pub major: u8,
    pub minor: u8,
    pub revision: u16,
    pub build: u32,
}

impl ImageVersion {
    pub const fn new(major: u8, minor: u8, revision: u16, build: u32) -> Self {
        Self {
            major,
            minor,
            revision,
            build,
        }
    }

    /// MCUboot's default downgrade comparison: major, minor and revision.
    /// The build number is ignored (`MCUBOOT_VERSION_CMP_USE_BUILD_NUMBER`
    /// unset, as in koptserg's bootloader).
    pub fn cmp_release(&self, other: &Self) -> Ordering {
        (self.major, self.minor, self.revision).cmp(&(other.major, other.minor, other.revision))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageHeader {
    pub load_addr: u32,
    pub hdr_size: u16,
    pub protect_tlv_size: u16,
    pub img_size: u32,
    pub flags: u32,
    pub version: ImageVersion,
}

impl ImageHeader {
    /// Parse a header; `None` unless the magic matches.
    pub fn parse(bytes: &[u8; IMAGE_HEADER_LEN]) -> Option<Self> {
        let u32_at = |at: usize| {
            u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
        };
        let u16_at = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);
        if u32_at(0) != IMAGE_MAGIC {
            return None;
        }
        Some(Self {
            load_addr: u32_at(4),
            hdr_size: u16_at(8),
            protect_tlv_size: u16_at(10),
            img_size: u32_at(12),
            flags: u32_at(16),
            version: ImageVersion {
                major: bytes[20],
                minor: bytes[21],
                revision: u16_at(22),
                build: u32_at(24),
            },
        })
    }

    /// Offset of the unprotected TLV info header, relative to the slot.
    pub fn tlv_offset(&self) -> u32 {
        self.hdr_size as u32 + self.img_size + self.protect_tlv_size as u32
    }

    /// Read the header at `slot`; `Ok(None)` without header magic.
    pub fn read<F: ReadNorFlash>(flash: &mut F, slot: u32) -> Result<Option<Self>, ImageError> {
        let mut bytes = [0u8; IMAGE_HEADER_LEN];
        flash
            .read(slot, &mut bytes)
            .map_err(|_| ImageError::Flash)?;
        Ok(Self::parse(&bytes))
    }
}

/// A public key trusted to sign images.
///
/// Keys are build-time configuration of a product (its loader and its OTA
/// policy); this crate contains none.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustedKey {
    /// ECDSA P-256: big-endian affine `x || y`.
    P256 { xy: [u8; 64] },
    /// Ed25519 public key (feature `ed25519`).
    #[cfg(feature = "ed25519")]
    Ed25519 { public: [u8; 32] },
}

impl TrustedKey {
    /// An ECDSA P-256 key, big-endian affine `x || y`.
    pub const fn new(xy: [u8; 64]) -> Self {
        Self::P256 { xy }
    }

    /// An Ed25519 key.
    #[cfg(feature = "ed25519")]
    pub const fn ed25519(public: [u8; 32]) -> Self {
        Self::Ed25519 { public }
    }

    fn bytes(&self) -> &[u8] {
        match self {
            Self::P256 { xy } => xy,
            #[cfg(feature = "ed25519")]
            Self::Ed25519 { public } => public,
        }
    }

    /// Short identifier published in the loader record: the first eight
    /// bytes of the key (the P-256 X coordinate, or the Ed25519 key).
    pub const fn id(&self) -> [u8; 8] {
        let bytes: &[u8] = match self {
            Self::P256 { xy } => xy,
            #[cfg(feature = "ed25519")]
            Self::Ed25519 { public } => public,
        };
        [
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]
    }

    /// SHA-256 of the DER `SubjectPublicKeyInfo` (imgtool's KEYHASH).
    pub fn key_hash(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        match self {
            Self::P256 { .. } => hasher.update(P256_SPKI_PREFIX),
            #[cfg(feature = "ed25519")]
            Self::Ed25519 { .. } => hasher.update(ED25519_SPKI_PREFIX),
        }
        hasher.update(self.bytes());
        hasher.finalize().into()
    }

    /// Whether `signature`, the value of a TLV of type `kind`, is this
    /// key's signature over `digest`. A TLV of another algorithm never
    /// verifies.
    #[inline(always)]
    fn verifies(&self, kind: u16, signature: &[u8], digest: &[u8; 32]) -> bool {
        match self {
            Self::P256 { xy } => {
                kind == TLV_ECDSA_SIG
                    && match (VerifyingKey::from_xy(xy), parse_der_signature(signature)) {
                        (Some(key), Some((r, s))) => key.verify_prehash(digest, &r, &s),
                        _ => false,
                    }
            }
            #[cfg(feature = "ed25519")]
            Self::Ed25519 { public } => {
                kind == TLV_ED25519
                    && <[u8; 64]>::try_from(signature).is_ok_and(|signature| {
                        ed25519_compact::PublicKey::new(*public)
                            .verify(digest, &ed25519_compact::Signature::new(signature))
                            .is_ok()
                    })
            }
        }
    }
}

/// A selection of up to [`MAX_TRUSTED_KEYS`] keys from a `static` list,
/// for policies that are copied around (an OTA writer). Only a reference
/// and a mask: narrowing the set never copies key material.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeySet {
    keys: &'static [TrustedKey],
    mask: u8,
}

impl KeySet {
    /// The first [`MAX_TRUSTED_KEYS`] of `keys`.
    pub const fn new(keys: &'static [TrustedKey]) -> Self {
        let len = if keys.len() < MAX_TRUSTED_KEYS {
            keys.len()
        } else {
            MAX_TRUSTED_KEYS
        };
        Self {
            keys,
            mask: ((1u16 << len) - 1) as u8,
        }
    }

    /// The selected keys, in list order.
    pub fn iter(&self) -> impl Iterator<Item = &'static TrustedKey> + Clone + use<> {
        let mask = self.mask;
        self.keys
            .iter()
            .enumerate()
            .filter(move |(index, _)| mask & (1 << index) != 0)
            .map(|(_, key)| key)
    }

    pub const fn len(&self) -> usize {
        self.mask.count_ones() as usize
    }

    pub const fn is_empty(&self) -> bool {
        self.mask == 0
    }

    /// Keep only the keys for which `keep` returns true.
    pub fn retain(&mut self, mut keep: impl FnMut(&TrustedKey) -> bool) {
        for (index, key) in self.keys.iter().enumerate().take(MAX_TRUSTED_KEYS) {
            if !keep(key) {
                self.mask &= !(1 << index);
            }
        }
    }
}

/// A slot whose image passed [`validate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidImage {
    pub header: ImageHeader,
    /// End of the unprotected TLV area, relative to the slot.
    pub end: u32,
    pub digest: [u8; 32],
}

/// Validate the image at `slot`: geometry within `max_len` bytes, SHA-256,
/// and a signature by one of `keys`, selected by the KEYHASH TLV that
/// precedes it. An empty `keys` trusts nothing.
pub fn validate<F: ReadNorFlash>(
    flash: &mut F,
    slot: u32,
    max_len: u32,
    keys: &[TrustedKey],
) -> Result<ValidImage, ImageError> {
    validate_by(flash, slot, max_len, keys.iter())
}

/// [`validate`] against the keys selected by `keys`.
pub fn validate_with_set<F: ReadNorFlash>(
    flash: &mut F,
    slot: u32,
    max_len: u32,
    keys: &KeySet,
) -> Result<ValidImage, ImageError> {
    validate_by(flash, slot, max_len, keys.iter())
}

pub(crate) fn validate_by<'k, F: ReadNorFlash>(
    flash: &mut F,
    slot: u32,
    max_len: u32,
    keys: impl Iterator<Item = &'k TrustedKey> + Clone,
) -> Result<ValidImage, ImageError> {
    let header = ImageHeader::read(flash, slot)?.ok_or(ImageError::NoImage)?;
    if header.flags != 0 {
        return Err(ImageError::Unsupported);
    }
    if (header.hdr_size as usize) < IMAGE_HEADER_LEN {
        return Err(ImageError::Malformed);
    }
    let hashed_len = (header.hdr_size as u32)
        .checked_add(header.img_size)
        .and_then(|end| end.checked_add(header.protect_tlv_size as u32))
        .ok_or(ImageError::Malformed)?;
    let tlv_info = read_array::<_, 4>(flash, slot, hashed_len, max_len)?;
    if u16::from_le_bytes([tlv_info[0], tlv_info[1]]) != TLV_INFO_MAGIC {
        return Err(ImageError::Malformed);
    }
    let tlv_total = u16::from_le_bytes([tlv_info[2], tlv_info[3]]) as u32;
    let end = hashed_len
        .checked_add(tlv_total)
        .filter(|end| *end <= max_len && tlv_total >= 4)
        .ok_or(ImageError::Malformed)?;
    if header.protect_tlv_size != 0 {
        let start = hashed_len - header.protect_tlv_size as u32;
        let info = read_array::<_, 4>(flash, slot, start, max_len)?;
        if u16::from_le_bytes([info[0], info[1]]) != TLV_PROTECTED_INFO_MAGIC
            || u16::from_le_bytes([info[2], info[3]]) != header.protect_tlv_size
        {
            return Err(ImageError::Malformed);
        }
    }

    let digest = hash_range(flash, slot, hashed_len)?;
    let mut hash_ok = false;
    // The trusted key named by the last KEYHASH TLV; consumed by the next
    // signature TLV.
    let mut selected: Option<&TrustedKey> = None;
    let mut signature_ok = false;

    let mut offset = hashed_len + 4;
    while offset < end {
        let entry = read_array::<_, 4>(flash, slot, offset, end)?;
        let kind = u16::from_le_bytes([entry[0], entry[1]]);
        let length = u16::from_le_bytes([entry[2], entry[3]]) as u32;
        let value_at = offset + 4;
        let next = value_at
            .checked_add(length)
            .filter(|next| *next <= end)
            .ok_or(ImageError::Malformed)?;
        match kind {
            TLV_SHA256 => {
                if length != 32 {
                    return Err(ImageError::Malformed);
                }
                let value = read_array::<_, 32>(flash, slot, value_at, end)?;
                hash_ok = value == digest;
            }
            TLV_KEYHASH => {
                selected = None;
                if length == 32 {
                    let value = read_array::<_, 32>(flash, slot, value_at, end)?;
                    selected = keys.clone().find(|key| key.key_hash() == value);
                }
            }
            TLV_ECDSA_SIG | TLV_ED25519 => {
                if let Some(key) = selected.take()
                    && !signature_ok
                {
                    let mut signature = [0u8; 72];
                    let length = length as usize;
                    if length <= signature.len() {
                        flash
                            .read(slot + value_at, &mut signature[..length])
                            .map_err(|_| ImageError::Flash)?;
                        signature_ok = key.verifies(kind, &signature[..length], &digest);
                    }
                }
            }
            _ => {}
        }
        offset = next;
    }
    if !hash_ok {
        return Err(ImageError::HashMismatch);
    }
    if !signature_ok {
        return Err(ImageError::BadSignature);
    }
    Ok(ValidImage {
        header,
        end,
        digest,
    })
}

fn read_array<F: ReadNorFlash, const LEN: usize>(
    flash: &mut F,
    slot: u32,
    offset: u32,
    limit: u32,
) -> Result<[u8; LEN], ImageError> {
    if offset.checked_add(LEN as u32).is_none_or(|end| end > limit) {
        return Err(ImageError::Malformed);
    }
    let mut bytes = [0u8; LEN];
    flash
        .read(slot + offset, &mut bytes)
        .map_err(|_| ImageError::Flash)?;
    Ok(bytes)
}

fn hash_range<F: ReadNorFlash>(flash: &mut F, slot: u32, len: u32) -> Result<[u8; 32], ImageError> {
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; READ_CHUNK];
    let mut offset = 0;
    while offset < len {
        let chunk = core::cmp::min(READ_CHUNK as u32, len - offset) as usize;
        flash
            .read(slot + offset, &mut buffer[..chunk])
            .map_err(|_| ImageError::Flash)?;
        hasher.update(&buffer[..chunk]);
        offset += chunk as u32;
    }
    Ok(hasher.finalize().into())
}
