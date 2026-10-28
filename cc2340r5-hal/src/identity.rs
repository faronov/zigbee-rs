//! Factory identity access for CC2340R52.

use core::fmt;

/// CC2340 factory-configuration memory.
pub const FCFG_BASE: u32 = 0x4E00_0000;
/// `FCFG.deviceInfo.macAddr` (deviceInfo starts at 80, macAddr at +16).
pub const FACTORY_IEEE_OFFSET: u32 = 0x60;
pub const FACTORY_IEEE_ADDRESS: u32 = FCFG_BASE + FACTORY_IEEE_OFFSET;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityError {
    /// FCFG contains an erased or otherwise unprovisioned address.
    Unprogrammed,
}

impl fmt::Display for IdentityError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unprogrammed => formatter.write_str("factory EUI-64 is unprogrammed"),
        }
    }
}

/// Reject erased factory values without changing TI's byte order.
pub const fn validate_ieee_address(address: [u8; 8]) -> Result<[u8; 8], IdentityError> {
    let mut index = 0;
    let mut all_zero = true;
    let mut all_erased = true;
    while index < address.len() {
        all_zero &= address[index] == 0;
        all_erased &= address[index] == 0xFF;
        index += 1;
    }
    if all_zero || all_erased {
        Err(IdentityError::Unprogrammed)
    } else {
        Ok(address)
    }
}

/// Decode the two little-endian FCFG words into the stack's EUI byte order.
pub const fn address_from_factory_words(low: u32, high: u32) -> Result<[u8; 8], IdentityError> {
    let low = low.to_le_bytes();
    let high = high.to_le_bytes();
    validate_ieee_address([
        low[0], low[1], low[2], low[3], high[0], high[1], high[2], high[3],
    ])
}

/// Read the device-unique IEEE EUI-64 from CC2340R52 FCFG.
#[cfg(target_os = "none")]
pub fn factory_ieee_address() -> Result<[u8; 8], IdentityError> {
    let low = unsafe { core::ptr::read_volatile(FACTORY_IEEE_ADDRESS as *const u32) };
    let high = unsafe { core::ptr::read_volatile((FACTORY_IEEE_ADDRESS + 4) as *const u32) };
    address_from_factory_words(low, high)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_erased_factory_addresses() {
        assert_eq!(
            validate_ieee_address([0; 8]),
            Err(IdentityError::Unprogrammed)
        );
        assert_eq!(
            validate_ieee_address([0xFF; 8]),
            Err(IdentityError::Unprogrammed)
        );
    }

    #[test]
    fn preserves_fcfg_byte_order() {
        assert_eq!(
            address_from_factory_words(0x3322_1100, 0x7766_5544),
            Ok([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77])
        );
    }

    #[test]
    fn fcfg_offset_matches_device_info_layout() {
        assert_eq!(FACTORY_IEEE_ADDRESS, 0x4E00_0060);
    }
}
