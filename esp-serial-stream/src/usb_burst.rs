//! USB short-packet accounting for the USB-Serial-JTAG bulk IN endpoint.
//!
//! The USB-Serial-JTAG IN endpoint sends a 64-byte packet whenever its FIFO
//! fills, and a short packet when software sets `WR_DONE`. A host CDC driver
//! may hold a transfer until it sees a short packet (or a ZLP), so a burst
//! whose length is an exact multiple of 64 must not end on a full packet.
//! Rather than depend on ZLP behaviour, the stream appends one filler byte
//! (a byte the protocol above ignores at a burst boundary — ZRHP uses an
//! empty COBS frame, `0x00`) before flushing such a burst.

/// Max packet size of the USB-Serial-JTAG bulk endpoints.
pub const USB_PACKET: u32 = 64;

/// Bytes written to the IN FIFO since the last short packet.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct UsbBurst {
    since_short: u32,
}

impl UsbBurst {
    pub const fn new() -> Self {
        Self { since_short: 0 }
    }

    /// `count` bytes entered the IN FIFO.
    pub fn written(&mut self, count: usize) {
        self.since_short = self.since_short.wrapping_add(count as u32);
    }

    /// Whether ending the burst now needs a filler byte first: the burst is
    /// non-empty and ended on a full packet, so `WR_DONE` alone would leave
    /// the host without a short packet.
    pub fn needs_filler(&self) -> bool {
        self.since_short != 0 && self.since_short.is_multiple_of(USB_PACKET)
    }

    /// Whether there is anything to flush at all.
    pub fn is_open(&self) -> bool {
        self.since_short != 0
    }

    /// `WR_DONE` was issued on a short packet; a new burst starts.
    pub fn flushed(&mut self) {
        self.since_short = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_burst_needs_nothing() {
        let burst = UsbBurst::new();
        assert!(!burst.is_open());
        assert!(!burst.needs_filler());
    }

    #[test]
    fn exact_packet_multiples_need_a_filler() {
        for packets in 1..5usize {
            let mut burst = UsbBurst::new();
            burst.written(packets * 64);
            assert!(burst.needs_filler(), "{packets} packets");
            burst.written(1);
            assert!(!burst.needs_filler());
        }
    }

    #[test]
    fn short_tail_needs_no_filler_and_flush_restarts() {
        let mut burst = UsbBurst::new();
        burst.written(63);
        assert!(!burst.needs_filler());
        burst.written(65);
        assert!(burst.needs_filler());
        burst.written(1);
        burst.flushed();
        assert!(!burst.is_open());
    }
}
