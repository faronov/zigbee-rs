//! Product guard for the factory-derived IEEE address.

use zigbee_types::IeeeAddress;

/// Deterministic fallback currently emitted by the PHY6222 MAC backend when
/// the factory one-bit-hot address cannot be decoded. A production product
/// must reject it because multiple devices would otherwise share an identity.
pub const DRIVER_FALLBACK_IEEE_ADDRESS: IeeeAddress =
    [0x00, 0x0d, 0x6f, 0xff, 0xfe, 0xde, 0xad, 0x01];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityError {
    Unprogrammed,
    SharedFallback,
}

pub fn validate_ieee_address(address: IeeeAddress) -> Result<IeeeAddress, IdentityError> {
    if address == [0; 8] || address == [0xff; 8] {
        Err(IdentityError::Unprogrammed)
    } else if address == DRIVER_FALLBACK_IEEE_ADDRESS {
        Err(IdentityError::SharedFallback)
    } else {
        Ok(address)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn programmed_identity_is_preserved() {
        let address = [0x7c, 0xb9, 0x4c, 0xff, 0xfe, 0x61, 0x92, 0x3a];
        assert_eq!(validate_ieee_address(address), Ok(address));
    }

    #[test]
    fn erased_and_shared_fallback_identities_are_rejected() {
        assert_eq!(
            validate_ieee_address([0; 8]),
            Err(IdentityError::Unprogrammed)
        );
        assert_eq!(
            validate_ieee_address([0xff; 8]),
            Err(IdentityError::Unprogrammed)
        );
        assert_eq!(
            validate_ieee_address(DRIVER_FALLBACK_IEEE_ADDRESS),
            Err(IdentityError::SharedFallback)
        );
    }
}
