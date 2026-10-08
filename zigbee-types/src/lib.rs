//! Core types shared across the zigbee-rs stack.
//!
//! Re-exports IEEE 802.15.4 types and defines Zigbee-specific
//! addressing, channel, and PIB types used by the MAC trait.

#![no_std]

/// Await a sub-future through a `Pin<&mut dyn Future>` so its poll code is
/// emitted once behind a vtable instead of being inlined into the caller's
/// coroutine.
///
/// This is the crate-shared form of the outlining boundary that
/// `zigbee-runtime` established for its tick and receive paths. The future is
/// still pinned in the caller's own coroutine frame, where an inline `.await`
/// would also have put it, so the cost is one static vtable, one indirect call
/// and whatever the coroutine layout can no longer overlap.
///
/// It must stay a macro. Wrapping the same thing in a generic `async fn` makes
/// the caller hold the moved-from temporary *and* the wrapper coroutine across
/// the await, so every outlined future's state is stored twice.
///
/// Reach for it when a large sub-future is awaited **from inside a loop**: the
/// coroutine's resume dispatch re-emits the loop body at every resume edge, so
/// an inlined body is duplicated even though the frame holds only one copy of
/// its state. On a small leaf call, or where LLVM already refuses to inline the
/// callee, the vtable and the indirect call cost more than the inlining saves —
/// always measure the linked image before and after.
#[macro_export]
macro_rules! await_out_of_line {
    ($future:expr) => {{
        let future = ::core::pin::pin!($future);
        let future: ::core::pin::Pin<&mut dyn ::core::future::Future<Output = _>> = future;
        future.await
    }};
}

/// IEEE 802.15.4 extended address (EUI-64)
pub type IeeeAddress = [u8; 8];

/// IEEE 802.15.4 short address (16-bit network address)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ShortAddress(pub u16);

impl ShortAddress {
    pub const BROADCAST: Self = Self(0xFFFF);
    pub const BROADCAST_RX_ON_WHEN_IDLE: Self = Self(0xFFFD);
    pub const BROADCAST_ROUTERS_AND_COORDINATOR: Self = Self(0xFFFC);
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

/// Durable replay admission for one MIC-verified secured frame.
///
/// Admission never writes persistent state; it only classifies the counter
/// against the durable replay floors and the remaining replay-domain capacity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayAdmission {
    /// The counter advances a floor. `new_domain` is set when no durable floor
    /// exists yet for its replay domain, so committing it consumes capacity.
    Fresh { new_domain: bool },
    /// A durable floor already covers this counter.
    Replayed,
    /// The counter is from a new replay domain and no durable capacity remains.
    CapacityRefused,
}

/// Result of durably committing one replay counter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayCommitOutcome {
    /// The durable floor advanced (or a new domain was recorded).
    Advanced,
    /// A durable floor already covered the counter; nothing was written.
    AlreadyCovered,
    /// The counter is from a new domain and the replay set is full; nothing
    /// was written.
    CapacityRefused,
}

/// Durable replay authority used by the NWK and APS receive paths.
///
/// `admit` is called after MIC verification and before any protocol
/// mutation. `commit` durably records a counter; only
/// [`ReplayCommitOutcome::Advanced`] authorizes a freshly received frame.
/// Storage failures are recorded by the authority and reported as `None`, so
/// the receive path simply drops the frame and the owner reports the error.
pub trait ReplayAuthority<R> {
    fn admit(&mut self, replay: &R) -> Option<ReplayAdmission>;
    fn commit(&mut self, replay: R) -> Option<ReplayCommitOutcome>;
}

/// Authority for volatile-only receive paths: every MIC-verified counter that
/// passed the RAM replay check is fresh, and commit always succeeds.
#[derive(Debug, Default, Clone, Copy)]
pub struct VolatileReplayAuthority;

impl<R> ReplayAuthority<R> for VolatileReplayAuthority {
    fn admit(&mut self, _replay: &R) -> Option<ReplayAdmission> {
        Some(ReplayAdmission::Fresh { new_domain: false })
    }

    fn commit(&mut self, _replay: R) -> Option<ReplayCommitOutcome> {
        Some(ReplayCommitOutcome::Advanced)
    }
}
