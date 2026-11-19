//! Device departures for a network co-processor host (`host-passthrough`).
//!
//! A coordinator NCP reports every device that leaves its tables to the
//! host (ZRHP `EVT_DEVICE_LEFT`). Two departures are only visible here:
//!
//! * a neighbour's NWK Leave *indication* (R22 §3.6.1.10.3), the only place
//!   that carries the Leave's rejoin bit;
//! * a child aged out of the neighbour table: an end device that missed its
//!   End Device Timeout, or a router child without Link Status
//!   ([`NwkLayer::age_end_device_children`]).
//!
//! Each is recorded in a bounded per-layer queue drained by the composition
//! root with [`NwkLayer::take_host_departure`]. Departures reported through
//! the Trust Center (Update-Device) are the Trust Center runtime's.
//!
//! Without the feature [`HostNwkEvents`] is zero-sized and the recording
//! hooks expand to nothing, so other images are unchanged.

#[cfg(feature = "host-passthrough")]
use zigbee_mac::MacDriver;
#[cfg(feature = "host-passthrough")]
use zigbee_types::{IeeeAddress, ShortAddress};

#[cfg(feature = "host-passthrough")]
use crate::NwkLayer;

/// Departures held until the composition root drains them.
#[cfg(feature = "host-passthrough")]
pub const HOST_DEPARTURE_QUEUE_LEN: usize = 16;

/// Why a device left this node's tables.
#[cfg(feature = "host-passthrough")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NwkDepartureCause {
    /// It announced its departure with a NWK Leave; `rejoin` is the Leave's
    /// rejoin bit (it expects to rejoin the same network).
    Leave { rejoin: bool },
    /// A child of this node aged out: an end device missed its End Device
    /// Timeout, or a router child stopped sending Link Status.
    ChildTimedOut,
}

/// One device that left this node's tables.
#[cfg(feature = "host-passthrough")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NwkDeparture {
    /// Its (former) short address.
    pub short_address: ShortAddress,
    /// Its IEEE address when this node knew it (neighbour table or address
    /// map).
    pub ieee_address: Option<IeeeAddress>,
    /// It was a child of this node.
    pub child: bool,
    pub cause: NwkDepartureCause,
}

/// The bounded departure queue; zero-sized without `host-passthrough`.
pub(crate) struct HostNwkEvents {
    #[cfg(feature = "host-passthrough")]
    queue: heapless::Deque<NwkDeparture, HOST_DEPARTURE_QUEUE_LEN>,
    #[cfg(feature = "host-passthrough")]
    dropped: u16,
}

impl HostNwkEvents {
    pub(crate) const fn new() -> Self {
        Self {
            #[cfg(feature = "host-passthrough")]
            queue: heapless::Deque::new(),
            #[cfg(feature = "host-passthrough")]
            dropped: 0,
        }
    }

    #[cfg(feature = "host-passthrough")]
    fn push(&mut self, departure: NwkDeparture) {
        if self.queue.push_back(departure).is_err() {
            self.dropped = self.dropped.saturating_add(1);
        }
    }
}

/// Run a host-departure hook (`host-passthrough`); without the feature the
/// hook is not compiled at all.
#[cfg(feature = "host-passthrough")]
macro_rules! host_event {
    ($hook:expr) => {
        $hook
    };
}
#[cfg(not(feature = "host-passthrough"))]
macro_rules! host_event {
    ($hook:expr) => {};
}
pub(crate) use host_event;

#[cfg(feature = "host-passthrough")]
impl<M: MacDriver> NwkLayer<M> {
    /// Record a Leave indication from `src`, before its neighbour entry is
    /// removed.
    pub(crate) fn host_note_leave(&mut self, src: ShortAddress, rejoin: bool) {
        let child = self.neighbors.find_by_short(src).is_some_and(|entry| {
            matches!(
                entry.relationship,
                crate::neighbor::Relationship::Child
                    | crate::neighbor::Relationship::UnauthenticatedChild
            )
        });
        let ieee_address = self.find_ieee_by_short(src);
        self.host_events.push(NwkDeparture {
            short_address: src,
            ieee_address,
            child,
            cause: NwkDepartureCause::Leave { rejoin },
        });
    }

    /// Record children evicted by
    /// [`age_end_device_children`](Self::age_end_device_children).
    pub(crate) fn host_note_aged(&mut self, evicted: &[ShortAddress]) {
        for &short in evicted {
            // The neighbour entry is gone; the address map may still know it.
            let ieee_address = self.address_map.ieee_of(short);
            self.host_events.push(NwkDeparture {
                short_address: short,
                ieee_address,
                child: true,
                cause: NwkDepartureCause::ChildTimedOut,
            });
        }
    }

    /// Oldest recorded departure (`host-passthrough`).
    pub fn take_host_departure(&mut self) -> Option<NwkDeparture> {
        self.host_events.queue.pop_front()
    }

    /// Departures lost because the queue was full, since the last call.
    pub fn take_host_departures_dropped(&mut self) -> u16 {
        core::mem::take(&mut self.host_events.dropped)
    }

    /// Forget recorded departures (the network was left or reset).
    pub fn clear_host_departures(&mut self) {
        self.host_events.queue.clear();
        self.host_events.dropped = 0;
    }
}
