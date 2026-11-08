//! A 32-byte record a zigbee-rs bootloader places at a fixed address so the
//! application can tell which bootloader (and therefore which signing key
//! and OTA channel) it runs under.
//!
//! MCUboot builds carry no such record; random code or erased flash at that
//! address matches the magic and CRC-32 with negligible probability.
//!
//! A loader trusting more than one key sets [`LOADER_INFO_KEYS`] and puts a
//! [`LoaderKeys`] record with the other key ids in the 32 bytes right
//! before the record. Single-key loaders leave those bytes erased, so their
//! record is unchanged.

use embedded_storage::nor_flash::ReadNorFlash;

use crate::image::{MAX_TRUSTED_KEYS, TrustedKey};

pub const LOADER_INFO_MAGIC: [u8; 8] = *b"ZRSBOOT1";
pub const LOADER_INFO_FORMAT: u16 = 1;
pub const LOADER_INFO_LEN: usize = 32;
/// Flag: a [`LoaderKeys`] record precedes the [`LoaderInfo`] record.
pub const LOADER_INFO_KEYS: u16 = 0x0001;
pub const LOADER_KEYS_MAGIC: [u8; 8] = *b"ZRSKEYS1";
pub const LOADER_KEYS_LEN: usize = 32;
/// Extra key ids a [`LoaderKeys`] record holds.
const EXTRA_KEYS: usize = MAX_TRUSTED_KEYS - 1;

/// Decoded loader identity record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoaderInfo {
    pub flags: u16,
    pub loader_version: u32,
    /// The first eight bytes of the trusted key's big-endian X coordinate.
    pub key_id: [u8; 8],
}

impl LoaderInfo {
    pub const fn new(flags: u16, loader_version: u32, key_xy: &[u8; 64]) -> Self {
        Self {
            flags,
            loader_version,
            key_id: [
                key_xy[0], key_xy[1], key_xy[2], key_xy[3], key_xy[4], key_xy[5], key_xy[6],
                key_xy[7],
            ],
        }
    }

    /// The record of a loader trusting `keys` (the first is the primary
    /// key; at most [`MAX_TRUSTED_KEYS`]). Sets [`LOADER_INFO_KEYS`] when
    /// there is more than one.
    pub const fn for_keys(loader_version: u32, keys: &[TrustedKey]) -> Self {
        Self {
            flags: if keys.len() > 1 { LOADER_INFO_KEYS } else { 0 },
            loader_version,
            key_id: keys[0].id(),
        }
    }

    /// Whether `key_xy` is the key this loader trusts.
    pub fn trusts(&self, key_xy: &[u8; 64]) -> bool {
        self.key_id[..] == key_xy[..8]
    }

    /// Encode at compile time so a loader can place the record with
    /// `#[link_section]`.
    pub const fn encode(&self) -> [u8; LOADER_INFO_LEN] {
        let mut out = [0xFFu8; LOADER_INFO_LEN];
        let mut index = 0;
        while index < 8 {
            out[index] = LOADER_INFO_MAGIC[index];
            out[16 + index] = self.key_id[index];
            index += 1;
        }
        let format = LOADER_INFO_FORMAT.to_le_bytes();
        let flags = self.flags.to_le_bytes();
        let version = self.loader_version.to_le_bytes();
        out[8] = format[0];
        out[9] = format[1];
        out[10] = flags[0];
        out[11] = flags[1];
        out[12] = version[0];
        out[13] = version[1];
        out[14] = version[2];
        out[15] = version[3];
        // 24..28 reserved (erased).
        let crc = crc32(&out, 28).to_le_bytes();
        out[28] = crc[0];
        out[29] = crc[1];
        out[30] = crc[2];
        out[31] = crc[3];
        out
    }

    pub fn decode(bytes: &[u8; LOADER_INFO_LEN]) -> Option<Self> {
        if bytes[..8] != LOADER_INFO_MAGIC
            || u16::from_le_bytes([bytes[8], bytes[9]]) != LOADER_INFO_FORMAT
            || u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]) != crc32(bytes, 28)
        {
            return None;
        }
        let mut key_id = [0u8; 8];
        key_id.copy_from_slice(&bytes[16..24]);
        Some(Self {
            flags: u16::from_le_bytes([bytes[10], bytes[11]]),
            loader_version: u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
            key_id,
        })
    }

    /// Read and decode the record at `address`.
    pub fn read<F: ReadNorFlash>(flash: &mut F, address: u32) -> Result<Option<Self>, F::Error> {
        let mut bytes = [0u8; LOADER_INFO_LEN];
        flash.read(address, &mut bytes)?;
        Ok(Self::decode(&bytes))
    }
}

/// Ids of the keys a loader trusts besides [`LoaderInfo::key_id`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoaderKeys {
    ids: [[u8; 8]; EXTRA_KEYS],
    len: usize,
}

impl LoaderKeys {
    /// The extra keys of a loader trusting `keys` (all but the first).
    pub const fn for_keys(keys: &[TrustedKey]) -> Self {
        let mut ids = [[0xFF; 8]; EXTRA_KEYS];
        let mut len = 0;
        while len < EXTRA_KEYS && len + 1 < keys.len() {
            ids[len] = keys[len + 1].id();
            len += 1;
        }
        Self { ids, len }
    }

    pub fn ids(&self) -> &[[u8; 8]] {
        &self.ids[..self.len]
    }

    /// Encode; a loader without extra keys gets an erased (all `0xFF`)
    /// block, which [`decode`](Self::decode) rejects.
    pub const fn encode(&self) -> [u8; LOADER_KEYS_LEN] {
        let mut out = [0xFFu8; LOADER_KEYS_LEN];
        if self.len == 0 {
            return out;
        }
        let mut index = 0;
        while index < 8 {
            out[index] = LOADER_KEYS_MAGIC[index];
            let mut key = 0;
            while key < self.len {
                out[12 + 8 * key + index] = self.ids[key][index];
                key += 1;
            }
            index += 1;
        }
        out[8] = self.len as u8;
        // 9..12 reserved (erased).
        let crc = crc32(&out, 28).to_le_bytes();
        out[28] = crc[0];
        out[29] = crc[1];
        out[30] = crc[2];
        out[31] = crc[3];
        out
    }

    pub fn decode(bytes: &[u8; LOADER_KEYS_LEN]) -> Option<Self> {
        let len = bytes[8] as usize;
        if bytes[..8] != LOADER_KEYS_MAGIC
            || !(1..=EXTRA_KEYS).contains(&len)
            || u32::from_le_bytes([bytes[28], bytes[29], bytes[30], bytes[31]]) != crc32(bytes, 28)
        {
            return None;
        }
        let mut ids = [[0xFF; 8]; EXTRA_KEYS];
        for (key, id) in ids.iter_mut().enumerate().take(len) {
            id.copy_from_slice(&bytes[12 + 8 * key..20 + 8 * key]);
        }
        Some(Self { ids, len })
    }
}

/// The 64-byte block a loader places so that it ends at the record
/// address: [`LoaderKeys`] (or erased) followed by [`LoaderInfo`].
pub const fn loader_block(loader_version: u32, keys: &[TrustedKey]) -> [u8; 64] {
    let extra = LoaderKeys::for_keys(keys).encode();
    let info = LoaderInfo::for_keys(loader_version, keys).encode();
    let mut out = [0u8; 64];
    let mut index = 0;
    while index < 32 {
        out[index] = extra[index];
        out[32 + index] = info[index];
        index += 1;
    }
    out
}

/// Key ids a loader trusts, read from its records.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrustedIds {
    pub info: LoaderInfo,
    pub extra: Option<LoaderKeys>,
}

impl TrustedIds {
    /// Read the [`LoaderInfo`] at `address` and, when flagged, the
    /// [`LoaderKeys`] before it. `None` without a valid record; a flagged
    /// but damaged key record leaves only the primary key.
    pub fn read<F: ReadNorFlash>(flash: &mut F, address: u32) -> Result<Option<Self>, F::Error> {
        let Some(info) = LoaderInfo::read(flash, address)? else {
            return Ok(None);
        };
        let extra = if info.flags & LOADER_INFO_KEYS != 0 {
            let mut bytes = [0u8; LOADER_KEYS_LEN];
            flash.read(address - LOADER_KEYS_LEN as u32, &mut bytes)?;
            LoaderKeys::decode(&bytes)
        } else {
            None
        };
        Ok(Some(Self { info, extra }))
    }

    /// Whether the loader trusts `key`.
    pub fn trusts(&self, key: &TrustedKey) -> bool {
        let id = key.id();
        self.info.key_id == id || self.extra.is_some_and(|extra| extra.ids().contains(&id))
    }
}

/// CRC-32/ISO-HDLC (the zlib CRC) of `bytes[..len]`, bitwise and `const`.
pub const fn crc32(bytes: &[u8], len: usize) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    let mut index = 0;
    while index < len {
        crc ^= bytes[index] as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
            bit += 1;
        }
        index += 1;
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_matches_the_zlib_check_value() {
        assert_eq!(crc32(b"123456789", 9), 0xCBF4_3926);
    }

    #[test]
    fn record_round_trips_and_rejects_damage() {
        let mut key = [0u8; 64];
        for (index, byte) in key.iter_mut().enumerate() {
            *byte = index as u8 + 1;
        }
        const KEY: [u8; 64] = [7; 64];
        const ENCODED: [u8; LOADER_INFO_LEN] = LoaderInfo::new(0, 0x0001_0000, &KEY).encode();
        assert_eq!(&ENCODED[..8], b"ZRSBOOT1");

        let info = LoaderInfo::new(0x0003, 0x0102_0304, &key);
        let bytes = info.encode();
        let decoded = LoaderInfo::decode(&bytes).unwrap();
        assert_eq!(decoded, info);
        assert!(decoded.trusts(&key));
        assert!(!decoded.trusts(&[0u8; 64]));

        for index in 0..LOADER_INFO_LEN {
            let mut damaged = bytes;
            damaged[index] ^= 0x01;
            assert!(LoaderInfo::decode(&damaged).is_none(), "byte {index}");
        }
        assert!(LoaderInfo::decode(&[0xFF; LOADER_INFO_LEN]).is_none());
        assert!(LoaderInfo::decode(&[0x00; LOADER_INFO_LEN]).is_none());
    }

    /// Golden block shared with `tools/test_bl702_ota.py`, which decodes
    /// the same bytes.
    #[test]
    fn multi_key_block_matches_the_release_tool_vector() {
        const GOLDEN: &str = "5a52534b4559533102ffffff22222222222222223333333333333333f459b2b6\
                              5a5253424f4f543101000100020000001111111111111111ffffffff1e93ac18";
        let keys = [
            TrustedKey::new([0x11; 64]),
            TrustedKey::new([0x22; 64]),
            TrustedKey::new([0x33; 64]),
        ];
        let block = loader_block(2, &keys);
        let hex: std::string::String = block.iter().map(|b| std::format!("{b:02x}")).collect();
        assert_eq!(hex, GOLDEN);
    }

    #[test]
    fn single_key_block_is_the_legacy_record_after_erased_bytes() {
        let key = TrustedKey::new([7; 64]);
        let block = loader_block(5, &[key]);
        assert_eq!(block[..32], [0xFF; 32]);
        assert_eq!(block[32..], LoaderInfo::new(0, 5, &[7; 64]).encode());
    }

    #[test]
    fn multi_key_block_round_trips_and_rejects_damage() {
        struct Mem([u8; 64]);
        impl embedded_storage::nor_flash::ErrorType for Mem {
            type Error = core::convert::Infallible;
        }
        impl ReadNorFlash for Mem {
            const READ_SIZE: usize = 1;
            fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
                let at = offset as usize - 0x100;
                bytes.copy_from_slice(&self.0[at..at + bytes.len()]);
                Ok(())
            }
            fn capacity(&self) -> usize {
                0x200
            }
        }
        let keys = [
            TrustedKey::new([1; 64]),
            TrustedKey::new([2; 64]),
            TrustedKey::new([3; 64]),
        ];
        let block = loader_block(9, &keys);
        let ids = TrustedIds::read(&mut Mem(block), 0x120).unwrap().unwrap();
        assert_eq!(ids.info.flags, LOADER_INFO_KEYS);
        assert!(keys.iter().all(|key| ids.trusts(key)));
        assert!(!ids.trusts(&TrustedKey::new([4; 64])));
        // More keys than slots: the extras are capped.
        assert_eq!(LoaderKeys::for_keys(&[keys[0]; 5]).ids().len(), 2);

        for index in 0..32 {
            let mut damaged = block;
            damaged[index] ^= 0x01;
            let ids = TrustedIds::read(&mut Mem(damaged), 0x120).unwrap().unwrap();
            assert!(ids.trusts(&keys[0]));
            assert!(!ids.trusts(&keys[1]), "byte {index}");
        }
    }
}
