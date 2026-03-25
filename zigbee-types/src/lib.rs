//! Core types shared across the zigbee-rs stack.
//!
//! Re-exports IEEE 802.15.4 types and defines Zigbee-specific
//! addressing, channel, and PIB types used by the MAC trait.

#![no_std]

/// IEEE 802.15.4 extended address (EUI-64)
pub type IeeeAddress = [u8; 8];

/// IEEE 802.15.4 short address (16-bit network address)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ShortAddress(pub u16);

impl ShortAddress {
    pub const BROADCAST: Self = Self(0xFFFF);
    pub const UNASSIGNED: Self = Self(0xFFFE);
    pub const COORDINATOR: Self = Self(0x0000);
}

/// PAN identifier (16-bit)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct PanId(pub u16);

impl PanId {
    pub const BROADCAST: Self = Self(0xFFFF);
}

/// MAC address — either short (16-bit) or extended (64-bit)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacAddress {
    Short(PanId, ShortAddress),
    Extended(PanId, IeeeAddress),
}

impl MacAddress {
    pub fn pan_id(&self) -> PanId {
        match self {
            Self::Short(pan, _) => *pan,
            Self::Extended(pan, _) => *pan,
        }
    }
}

/// 802.15.4 channel number (11-26 for 2.4 GHz Zigbee)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    Ch11 = 11,
    Ch12 = 12,
    Ch13 = 13,
    Ch14 = 14,
    Ch15 = 15,
    Ch16 = 16,
    Ch17 = 17,
    Ch18 = 18,
    Ch19 = 19,
    Ch20 = 20,
    Ch21 = 21,
    Ch22 = 22,
    Ch23 = 23,
    Ch24 = 24,
    Ch25 = 25,
    Ch26 = 26,
}

impl Channel {
    pub fn from_number(n: u8) -> Option<Self> {
        match n {
            11 => Some(Self::Ch11),
            12 => Some(Self::Ch12),
            13 => Some(Self::Ch13),
            14 => Some(Self::Ch14),
            15 => Some(Self::Ch15),
            16 => Some(Self::Ch16),
            17 => Some(Self::Ch17),
            18 => Some(Self::Ch18),
            19 => Some(Self::Ch19),
            20 => Some(Self::Ch20),
            21 => Some(Self::Ch21),
            22 => Some(Self::Ch22),
            23 => Some(Self::Ch23),
            24 => Some(Self::Ch24),
            25 => Some(Self::Ch25),
            26 => Some(Self::Ch26),
            _ => None,
        }
    }

    pub fn number(self) -> u8 {
        self as u8
    }
}

/// Bitmask of channels (bits 11..26 for 2.4 GHz)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChannelMask(pub u32);

impl ChannelMask {
    /// All 2.4 GHz Zigbee channels (11-26)
    pub const ALL_2_4GHZ: Self = Self(0x07FFF800);

    /// Zigbee preferred channels: 11, 14, 15, 19, 20, 24, 25
    pub const PREFERRED: Self =
        Self((1 << 11) | (1 << 14) | (1 << 15) | (1 << 19) | (1 << 20) | (1 << 24) | (1 << 25));

    pub fn contains(self, channel: Channel) -> bool {
        self.0 & (1 << channel.number()) != 0
    }

    pub fn iter(self) -> ChannelMaskIter {
        ChannelMaskIter {
            mask: self,
            current: 11,
        }
    }
}

pub struct ChannelMaskIter {
    mask: ChannelMask,
    current: u8,
}

impl Iterator for ChannelMaskIter {
    type Item = Channel;
    fn next(&mut self) -> Option<Self::Item> {
        while self.current <= 26 {
            let ch = self.current;
            self.current += 1;
            if self.mask.0 & (1 << ch) != 0 {
                return Channel::from_number(ch);
            }
        }
        None
    }
}

/// Transmit power in dBm
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TxPower(pub i8);
