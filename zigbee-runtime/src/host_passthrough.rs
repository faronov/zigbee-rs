//! Raw APS data indications for an external host (`host-passthrough`).
//!
//! A network co-processor has no application endpoints of its own: every
//! APS data frame addressed to an application endpoint (1..=240, and the
//! 0xFF broadcast endpoint) belongs to the host and is moved here instead
//! of the compile-time ZCL dispatcher. ZDO frames (endpoint 0) are copied
//! as well, flagged `handled_by_ncp` when the runtime's ZDO server answers
//! them, and still processed locally. The Green Power endpoint (242) stays
//! with the runtime's GP proxy/sink.
//!
//! The queue is a bounded process-wide singleton: an NCP image composes
//! exactly one Zigbee device. When it is full the frame is dropped and
//! counted ([`dropped`]); the frame was already APS-acknowledged, so the
//! drop is reported to the host as `EVT_INDICATIONS_DROPPED`, never silent.
//!
//! Not provided by the stack today, and reported as unknown: RSSI (-128),
//! and APS-level security separately from NWK security.

use core::cell::RefCell;

use critical_section::Mutex;
use heapless::{Deque, Vec};
use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_types::{IeeeAddress, MacAddress};

/// Largest ASDU forwarded; longer frames are dropped and counted.
pub const MAX_HOST_ASDU: usize = 128;
/// Queued indications.
pub const HOST_QUEUE_LEN: usize = 8;
/// Endpoint of the Green Power cluster (kept by the runtime).
const GP_ENDPOINT: u8 = 242;

/// One received APS data frame for the host.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HostIndication {
    /// ZRHP `dst_mode`: `0x01` group, `0x02` unicast, `0x0F` broadcast.
    pub dst_mode: u8,
    pub dst: u16,
    pub dst_ep: u8,
    pub src_nwk: u16,
    pub src_ieee: Option<IeeeAddress>,
    pub src_ep: u8,
    pub profile: u16,
    pub cluster: u16,
    pub nwk_secured: bool,
    pub handled_by_ncp: bool,
    pub lqi: u8,
    pub last_hop_nwk: u16,
    pub aps_counter: u8,
    pub asdu: Vec<u8, MAX_HOST_ASDU>,
}

struct State {
    queue: Deque<HostIndication, HOST_QUEUE_LEN>,
    dropped: u32,
}

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    queue: Deque::new(),
    dropped: 0,
}));

/// Oldest queued indication.
pub fn take() -> Option<HostIndication> {
    critical_section::with(|cs| STATE.borrow_ref_mut(cs).queue.pop_front())
}

/// Indications dropped since the last call (queue full or ASDU too long).
pub fn take_dropped() -> u32 {
    critical_section::with(|cs| core::mem::take(&mut STATE.borrow_ref_mut(cs).dropped))
}

/// Empty the queue (network left or reset).
pub fn clear() {
    critical_section::with(|cs| STATE.borrow_ref_mut(cs).queue.clear());
}

/// Whether `endpoint` belongs to the host.
pub const fn is_host_endpoint(endpoint: u8) -> bool {
    matches!(endpoint, 1..=240 | 0xFF)
}

/// Copy an indication for the host. Returns `true` when the frame belongs
/// to the host only and the runtime must not dispatch it further.
pub(crate) fn capture(
    indication: &ApsdeDataIndication<'_>,
    mac_source: &MacAddress,
    nwk_secured: bool,
    src_ieee: Option<IeeeAddress>,
) -> bool {
    let dst_ep = indication.dst_endpoint;
    if dst_ep == GP_ENDPOINT {
        return false;
    }
    let host_only = is_host_endpoint(dst_ep);
    let (dst_mode, dst) = match (indication.dst_addr_mode, indication.dst_address) {
        (ApsAddressMode::Group, ApsAddress::Group(group)) => (0x01, group),
        (_, ApsAddress::Short(address)) if address.0 >= 0xFFF8 => (0x0F, address.0),
        (_, ApsAddress::Short(address)) => (0x02, address.0),
        _ => (0x02, 0x0000),
    };
    let src_nwk = match indication.src_address {
        ApsAddress::Short(address) => address.0,
        _ => 0xFFFF,
    };
    let last_hop_nwk = match mac_source {
        MacAddress::Short(_, address) => address.0,
        MacAddress::Extended(..) => 0xFFFF,
    };
    // The ZDO server answers requests (cluster < 0x8000) addressed to it.
    let handled_by_ncp = dst_ep == 0 && indication.cluster_id < 0x8000 && dst_mode == 0x02;
    let Ok(asdu) = Vec::from_slice(indication.payload) else {
        critical_section::with(|cs| {
            let mut state = STATE.borrow_ref_mut(cs);
            state.dropped = state.dropped.saturating_add(1);
        });
        return host_only;
    };
    let record = HostIndication {
        dst_mode,
        dst,
        dst_ep,
        src_nwk,
        src_ieee,
        src_ep: indication.src_endpoint,
        profile: indication.profile_id,
        cluster: indication.cluster_id,
        nwk_secured,
        handled_by_ncp,
        lqi: indication.lqi,
        last_hop_nwk,
        aps_counter: indication.aps_counter,
        asdu,
    };
    critical_section::with(|cs| {
        let mut state = STATE.borrow_ref_mut(cs);
        if state.queue.push_back(record).is_err() {
            state.dropped = state.dropped.saturating_add(1);
        }
    });
    host_only
}

impl<M: zigbee_mac::MacDriver, R: crate::role::DeviceRole> crate::ZigbeeDevice<M, R> {
    /// Hand an APS data indication to the host queue; `true` = consumed.
    pub(crate) fn host_passthrough_capture(
        &self,
        aps_indication: &ApsdeDataIndication<'_>,
        mac_indication: &zigbee_mac::McpsDataIndication,
        nwk_source: zigbee_types::ShortAddress,
        nwk_secured: bool,
    ) -> bool {
        let src_ieee = self.bdb.zdo().nwk().find_ieee_by_short(nwk_source);
        capture(
            aps_indication,
            &mac_indication.src_address,
            nwk_secured,
            src_ieee,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_types::{PanId, ShortAddress};

    fn indication(dst_ep: u8, cluster: u16, payload: &[u8]) -> ApsdeDataIndication<'_> {
        ApsdeDataIndication {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(ShortAddress(0x0000)),
            dst_endpoint: dst_ep,
            src_addr_mode: ApsAddressMode::Short,
            src_address: ApsAddress::Short(ShortAddress(0x1234)),
            src_endpoint: 1,
            profile_id: 0x0104,
            cluster_id: cluster,
            payload,
            aps_counter: 7,
            security_status: true,
            lqi: 200,
        }
    }

    // One test owns the process-wide queue.
    #[test]
    fn host_endpoints_are_consumed_zdo_is_copied_and_gp_is_left_alone() {
        clear();
        let _ = take_dropped();
        let hop = MacAddress::Short(PanId(0x1A62), ShortAddress(0x5678));
        assert!(capture(&indication(1, 0x0006, &[1, 2]), &hop, true, None));
        assert!(!capture(&indication(0, 0x0005, &[9]), &hop, true, None));
        assert!(!capture(&indication(242, 0x0021, &[0]), &hop, true, None));
        let first = take().unwrap();
        assert_eq!(
            (
                first.dst_mode,
                first.dst_ep,
                first.src_nwk,
                first.last_hop_nwk
            ),
            (0x02, 1, 0x1234, 0x5678)
        );
        assert_eq!(first.asdu.as_slice(), &[1, 2]);
        assert!(!first.handled_by_ncp);
        assert!(take().unwrap().handled_by_ncp);
        assert_eq!(take(), None);

        for _ in 0..HOST_QUEUE_LEN + 2 {
            capture(&indication(1, 0x0006, &[0]), &hop, true, None);
        }
        assert!(capture(&indication(1, 0x0006, &[0; 200]), &hop, true, None));
        assert_eq!(take_dropped(), 3);
        clear();
    }
}
