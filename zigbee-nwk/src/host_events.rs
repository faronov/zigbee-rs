//! Device departures and child polls for a network co-processor host
//! (`host-passthrough`).
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
//! Every MAC Data Request of a child of this node is recorded as well, with
//! the Frame Pending bit of its MAC ACK when the MAC reports it, in a second
//! bounded queue (`NwkLayer::take_host_poll`): the host learns that a
//! sleepy child is awake (ZRHP `EVT_CHILD_POLL`, rate-limited by the NCP).
//!
//! Without the feature [`HostNwkEvents`] is zero-sized and the recording
//! hooks expand to nothing, so other images are unchanged.

#[cfg(feature = "host-passthrough")]
use zigbee_mac::MacDriver;
#[cfg(feature = "host-passthrough")]
use zigbee_types::{IeeeAddress, MacAddress, ShortAddress};

#[cfg(feature = "host-passthrough")]
use crate::NwkLayer;

/// Departures held until the composition root drains them.
#[cfg(feature = "host-passthrough")]
pub const HOST_DEPARTURE_QUEUE_LEN: usize = 16;

/// Child polls held until the composition root drains them (after every
/// stack step; one step services at most a few polls). When full, the
/// oldest poll is dropped: the newest says more about who is awake.
#[cfg(feature = "host-passthrough")]
pub const HOST_POLL_QUEUE_LEN: usize = 8;

/// One MAC Data Request of a child of this node.
#[cfg(feature = "host-passthrough")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NwkChildPoll {
    pub short_address: ShortAddress,
    /// Frame Pending of the MAC ACK to this poll; `None` when the MAC does
    /// not report it ([`crate::ChildPoll::ack_frame_pending`]).
    pub ack_frame_pending: Option<bool>,
}

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

/// The bounded departure and poll queues; zero-sized without
/// `host-passthrough`.
pub(crate) struct HostNwkEvents {
    #[cfg(feature = "host-passthrough")]
    queue: heapless::Deque<NwkDeparture, HOST_DEPARTURE_QUEUE_LEN>,
    #[cfg(feature = "host-passthrough")]
    dropped: u16,
    #[cfg(feature = "host-passthrough")]
    polls: heapless::Deque<NwkChildPoll, HOST_POLL_QUEUE_LEN>,
    #[cfg(feature = "host-passthrough")]
    polls_dropped: u16,
}

impl HostNwkEvents {
    pub(crate) const fn new() -> Self {
        Self {
            #[cfg(feature = "host-passthrough")]
            queue: heapless::Deque::new(),
            #[cfg(feature = "host-passthrough")]
            dropped: 0,
            #[cfg(feature = "host-passthrough")]
            polls: heapless::Deque::new(),
            #[cfg(feature = "host-passthrough")]
            polls_dropped: 0,
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

    /// Forget recorded departures and polls (the network was left or
    /// reset).
    pub fn clear_host_departures(&mut self) {
        self.host_events.queue.clear();
        self.host_events.dropped = 0;
        self.host_events.polls.clear();
        self.host_events.polls_dropped = 0;
    }

    /// Record a MAC Data Request from `source` if it is a child of this
    /// node.
    pub(crate) fn host_note_poll(&mut self, source: &MacAddress, ack_frame_pending: Option<bool>) {
        let Some(short_address) = self.known_child_short(source) else {
            return;
        };
        let events = &mut self.host_events;
        if events.polls.is_full() {
            events.polls.pop_front();
            events.polls_dropped = events.polls_dropped.saturating_add(1);
        }
        let _ = events.polls.push_back(NwkChildPoll {
            short_address,
            ack_frame_pending,
        });
    }

    /// Oldest recorded child poll (`host-passthrough`).
    pub fn take_host_poll(&mut self) -> Option<NwkChildPoll> {
        self.host_events.polls.pop_front()
    }

    /// Child polls lost because the queue was full, since the last call.
    pub fn take_host_polls_dropped(&mut self) -> u16 {
        core::mem::take(&mut self.host_events.polls_dropped)
    }
}
