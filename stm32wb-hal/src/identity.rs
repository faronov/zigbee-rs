//! Factory identity.

use zigbee_types::IeeeAddress;

/// `UID64`: unique device number, then device id and ST company id.
const UID64: *const u32 = 0x1FFF_7580 as *const u32;

/// The factory EUI-64 in zigbee-rs (over-the-air, little-endian) order.
///
/// ST composes it as `company_id(24) | device_id(8) | udn(32)`, which is
/// exactly the second `UID64` word above the first; the coprocessor MAC
/// examples program the same value.
pub fn eui64() -> IeeeAddress {
    // SAFETY: UID64 is a read-only, always-mapped system memory word pair.
    let (udn, high) = unsafe { (UID64.read_volatile(), UID64.add(1).read_volatile()) };
    ((u64::from(high) << 32) | u64::from(udn)).to_le_bytes()
}
