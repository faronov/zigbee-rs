//! Tests for the zigbee-types crate.

use zigbee_types::*;

#[test]
fn test_short_address_constants() {
    assert_eq!(ShortAddress::BROADCAST.0, 0xFFFF);
    assert_eq!(ShortAddress::UNASSIGNED.0, 0xFFFE);
    assert_eq!(ShortAddress::COORDINATOR.0, 0x0000);
}

#[test]
fn test_pan_id_broadcast() {
    assert_eq!(PanId::BROADCAST.0, 0xFFFF);
}

#[test]
fn test_mac_address_pan_id() {
    let short = MacAddress::Short(PanId(0x1234), ShortAddress(0x5678));
    assert_eq!(short.pan_id(), PanId(0x1234));

    let ext = MacAddress::Extended(PanId(0xABCD), [1; 8]);
    assert_eq!(ext.pan_id(), PanId(0xABCD));
}

#[test]
fn test_channel_from_number() {
    assert_eq!(Channel::from_number(11), Some(Channel::Ch11));
    assert_eq!(Channel::from_number(26), Some(Channel::Ch26));
    assert_eq!(Channel::from_number(10), None);
    assert_eq!(Channel::from_number(27), None);
}

#[test]
fn test_channel_number() {
    assert_eq!(Channel::Ch11.number(), 11);
    assert_eq!(Channel::Ch26.number(), 26);
}

#[test]
fn test_channel_mask_all_2_4ghz() {
    let mask = ChannelMask::ALL_2_4GHZ;
    for ch in 11..=26 {
        assert!(
            mask.contains(Channel::from_number(ch).unwrap()),
            "Channel {ch} should be in ALL_2_4GHZ mask"
        );
    }
}

#[test]
fn test_channel_mask_contains() {
    let mask = ChannelMask(1 << 15 | 1 << 20);
    assert!(mask.contains(Channel::Ch15));
    assert!(mask.contains(Channel::Ch20));
    assert!(!mask.contains(Channel::Ch11));
    assert!(!mask.contains(Channel::Ch25));
}

#[test]
fn test_channel_mask_iter() {
    let mask = ChannelMask(1 << 11 | 1 << 15 | 1 << 26);
    let channels: heapless::Vec<u8, 16> = mask.iter().map(|c| c.number()).collect();
    assert_eq!(channels.as_slice(), &[11, 15, 26]);
}

#[test]
fn test_channel_mask_empty_iter() {
    let mask = ChannelMask(0);
    assert_eq!(mask.iter().count(), 0);
}

#[test]
fn test_channel_mask_preferred() {
    let pref = ChannelMask::PREFERRED;
    assert!(pref.contains(Channel::Ch11));
    assert!(pref.contains(Channel::Ch15));
    assert!(pref.contains(Channel::Ch20));
    assert!(pref.contains(Channel::Ch25));
}
