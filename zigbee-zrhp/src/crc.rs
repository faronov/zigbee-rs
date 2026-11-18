//! CRC-32C (Castagnoli), as used by every ZRHP frame and by the backup `END`
//! record (spec 5.2, 9.3).
//!
//! Reflected polynomial `0x82F63B78`, init `0xFFFF_FFFF`, xorout
//! `0xFFFF_FFFF`. Check value: `crc32c(b"123456789") == 0xE306_9283`.
//!
//! The implementation is a 1 KiB table generated at compile time; it runs on
//! every target without hardware CRC support.

const POLY: u32 = 0x82F6_3B78;

const TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = i as u32;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ POLY
            } else {
                crc >> 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

/// Incremental CRC-32C state.
///
/// ```
/// let mut crc = zigbee_zrhp::crc::Crc32c::new();
/// crc.update(b"1234");
/// crc.update(b"56789");
/// assert_eq!(crc.finish(), 0xE306_9283);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crc32c {
    state: u32,
}

impl Crc32c {
    /// A fresh CRC (`init = 0xFFFF_FFFF`).
    pub const fn new() -> Self {
        Self { state: 0xFFFF_FFFF }
    }

    /// Feed bytes.
    pub fn update(&mut self, data: &[u8]) {
        let mut crc = self.state;
        for &byte in data {
            crc = TABLE[((crc ^ byte as u32) & 0xFF) as usize] ^ (crc >> 8);
        }
        self.state = crc;
    }

    /// Feed one byte.
    #[inline]
    pub fn update_byte(&mut self, byte: u8) {
        self.state = TABLE[((self.state ^ byte as u32) & 0xFF) as usize] ^ (self.state >> 8);
    }

    /// The final CRC value (`xorout` applied). The state is not consumed.
    pub const fn finish(&self) -> u32 {
        self.state ^ 0xFFFF_FFFF
    }
}

impl Default for Crc32c {
    fn default() -> Self {
        Self::new()
    }
}

/// One-shot CRC-32C.
pub fn crc32c(data: &[u8]) -> u32 {
    let mut crc = Crc32c::new();
    crc.update(data);
    crc.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value() {
        assert_eq!(crc32c(b"123456789"), 0xE306_9283);
        assert_eq!(crc32c(b""), 0);
    }
}
