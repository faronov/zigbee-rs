//! MCUboot image header, TLV trailer and validation against flash.
//!
//! The format is the one `imgtool sign` produces (MCUboot
//! `bootutil/image.h`): a little-endian header at the start of the slot,
//! the firmware, an optional protected TLV area and an unprotected TLV area.
//! The SHA-256 TLV covers header, firmware and protected TLVs; the ECDSA
//! P-256 signature TLV signs that digest; the KEYHASH TLV is SHA-256 of the
//! signer's DER `SubjectPublicKeyInfo`.

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

/// DER `SubjectPublicKeyInfo` prefix of every uncompressed P-256 key, up to
/// and including the SEC1 `0x04` point tag. imgtool hashes the complete
/// `SubjectPublicKeyInfo` into the KEYHASH TLV.
pub const P256_SPKI_PREFIX: [u8; 27] = [
    0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x02, 0x01, 0x06, 0x08, 0x2A,
    0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0x03, 0x42, 0x00, 0x04,
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

/// A public key trusted to sign images: big-endian affine `x || y`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrustedKey {
    pub xy: [u8; 64],
}

impl TrustedKey {
    pub const fn new(xy: [u8; 64]) -> Self {
        Self { xy }
    }

    /// SHA-256 of the DER `SubjectPublicKeyInfo` (imgtool's KEYHASH).
    pub fn key_hash(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(P256_SPKI_PREFIX);
        hasher.update(self.xy);
        hasher.finalize().into()
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
/// and an ECDSA P-256 signature by `key` whose KEYHASH TLV names that key.
pub fn validate<F: ReadNorFlash>(
    flash: &mut F,
    slot: u32,
    max_len: u32,
    key: &TrustedKey,
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
    let key_hash = key.key_hash();
    let verifying_key = VerifyingKey::from_xy(&key.xy);
    let mut hash_ok = false;
    let mut key_selected = false;
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
                key_selected =
                    length == 32 && read_array::<_, 32>(flash, slot, value_at, end)? == key_hash;
            }
            TLV_ECDSA_SIG if key_selected && !signature_ok => {
                let mut der = [0u8; 72];
                let length = length as usize;
                if length <= der.len() {
                    flash
                        .read(slot + value_at, &mut der[..length])
                        .map_err(|_| ImageError::Flash)?;
                    if let (Some(key), Some((r, s))) =
                        (verifying_key, parse_der_signature(&der[..length]))
                    {
                        signature_ok = key.verify_prehash(&digest, &r, &s);
                    }
                }
                key_selected = false;
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
