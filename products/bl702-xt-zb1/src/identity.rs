//! Product fallback policy for the BL702 factory identifier.

use zigbee_types::IeeeAddress;

pub const FALLBACK_IEEE_ADDRESS: IeeeAddress = [0x02, 0x70, 0x02, 0, 0, 0, 0, 1];

pub fn ieee_address(chip_id: IeeeAddress) -> IeeeAddress {
    if chip_id == [0; 8] || chip_id == [0xff; 8] {
        FALLBACK_IEEE_ADDRESS
    } else {
        chip_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn programmed_factory_identity_is_preserved() {
        let programmed = [0x7c, 0xb9, 0x4c, 0x61, 0x92, 0x3a, 0, 0];
        assert_eq!(ieee_address(programmed), programmed);
    }

    #[test]
    fn erased_or_zero_identity_uses_the_existing_local_fallback() {
        assert_eq!(ieee_address([0; 8]), FALLBACK_IEEE_ADDRESS);
        assert_eq!(ieee_address([0xff; 8]), FALLBACK_IEEE_ADDRESS);
    }
}
