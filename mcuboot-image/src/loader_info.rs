//! A 32-byte record a zigbee-rs bootloader places at a fixed address so the
//! application can tell which bootloader (and therefore which signing key
//! and OTA channel) it runs under.
//!
//! MCUboot builds carry no such record; random code or erased flash at that
//! address matches the magic and CRC-32 with negligible probability.

use embedded_storage::nor_flash::ReadNorFlash;

pub const LOADER_INFO_MAGIC: [u8; 8] = *b"ZRSBOOT1";
pub const LOADER_INFO_FORMAT: u16 = 1;
pub const LOADER_INFO_LEN: usize = 32;

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
}
