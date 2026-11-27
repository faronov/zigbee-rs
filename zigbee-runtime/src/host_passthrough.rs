//! Raw APS data indications for an external host (`host-passthrough`).
//!
//! A network co-processor has no application endpoints of its own: every
//! APS data frame addressed to an application endpoint (1..=240, and the
//! 0xFF broadcast endpoint) belongs to the host and is moved here instead
//! of the compile-time ZCL dispatcher. ZDO frames (endpoint 0) are copied
//! as well, flagged `handled_by_ncp` when the runtime's ZDO server answers
//! them, and still processed locally. The Green Power endpoint (242) stays
//! with the runtime's GP proxy/sink, except that with `green-power-ncp` in
//! `HOST_SINK` mode the GP server frames (client → server: GP Notification,
//! GP Commissioning Notification, ...) go to the host sink instead
//! (`green_power::ncp`).
//!
//! The queue is a bounded process-wide singleton: an NCP image composes
//! exactly one Zigbee device. When it is full the frame is dropped and
//! counted ([`dropped`]); the frame was already APS-acknowledged, so the
//! drop is reported to the host as `EVT_INDICATIONS_DROPPED`, never silent.
//!
//! Each record carries the last hop's RSSI (`zigbee-mac` `rx-rssi`: measured
//! by the ESP32, CC26x2 and EFR32 Series 2 PHYs, `RSSI_UNKNOWN_DBM` = -128
//! on backends that do not measure it) and the frame's APS security
//! separately from its NWK security, with the kind of link key that secured
//! it.

use core::cell::RefCell;

use critical_section::Mutex;
use heapless::{Deque, Vec};
use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsRxSecurity};
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
    /// APS-secured with a link key (independent of `nwk_secured`).
    pub aps_secured: bool,
    /// ZRHP `key_source` of an APS-secured frame: 0 n/a, 1 global Trust
    /// Center link key, 2 unique Trust Center link key, 3 application link
    /// key.
    pub key_source: u8,
    pub handled_by_ncp: bool,
    pub lqi: u8,
    /// Last hop RSSI in dBm; -128 when the PHY does not measure it.
    pub rssi: i8,
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
    rssi: i8,
    nwk_secured: bool,
    aps_security: ApsRxSecurity,
    src_ieee: Option<IeeeAddress>,
) -> bool {
    let dst_ep = indication.dst_endpoint;
    if dst_ep == GP_ENDPOINT {
        return false;
    }
    copy(
        indication,
        mac_source,
        rssi,
        nwk_secured,
        aps_security,
        src_ieee,
    );
    is_host_endpoint(dst_ep)
}

/// Queue a copy of `indication` for the host; a full queue or an ASDU
/// longer than [`MAX_HOST_ASDU`] drops it and counts the drop.
pub(crate) fn copy(
    indication: &ApsdeDataIndication<'_>,
    mac_source: &MacAddress,
    rssi: i8,
    nwk_secured: bool,
    aps_security: ApsRxSecurity,
    src_ieee: Option<IeeeAddress>,
) {
    let dst_ep = indication.dst_endpoint;
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
        return;
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
        aps_secured: aps_security.aps_secured,
        key_source: aps_security.key_source as u8,
        handled_by_ncp,
        lqi: indication.lqi,
        rssi,
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
}

impl<M: zigbee_mac::MacDriver, R: crate::role::DeviceRole> crate::ZigbeeDevice<M, R> {
    /// Hand an APS data indication to the host queue; `true` = consumed.
    pub(crate) fn host_passthrough_capture(
        &mut self,
        aps_indication: &ApsdeDataIndication<'_>,
        mac_indication: &zigbee_mac::McpsDataIndication,
        nwk_source: zigbee_types::ShortAddress,
        nwk_secured: bool,
    ) -> bool {
        let src_ieee = self.bdb.zdo().nwk().find_ieee_by_short(nwk_source);
        // Green Power frames for a host sink (`HOST_SINK`, spec 7.11).
        #[cfg(feature = "green-power-ncp")]
        if let Some(consumed) = self.gp_ncp_capture(
            aps_indication,
            &mac_indication.src_address,
            mac_indication.payload.rssi_dbm(),
            nwk_secured,
            src_ieee,
        ) {
            return consumed;
        }
        capture(
            aps_indication,
            &mac_indication.src_address,
            mac_indication.payload.rssi_dbm(),
            nwk_secured,
            self.bdb.zdo().aps().last_rx_security(),
            src_ieee,
        )
    }
}

/// Most input (or output) clusters of one host endpoint.
pub const MAX_HOST_ENDPOINT_CLUSTERS: usize = zigbee_zdo::descriptors::MAX_CLUSTERS;

/// Why a host endpoint was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostEndpointError {
    /// Not an application endpoint (1..=240).
    InvalidEndpoint,
    /// More than [`MAX_HOST_ENDPOINT_CLUSTERS`] input or output clusters.
    TooManyClusters,
    /// The ZDO endpoint registry is full.
    RegistryFull,
}

impl<M: zigbee_mac::MacDriver, R: crate::role::DeviceRole> crate::ZigbeeDevice<M, R> {
    /// Register a host endpoint with the ZDO server (`Active_EP`,
    /// `Simple_Desc`, `Match_Desc`), replacing a registered endpoint with
    /// the same number. Its frames already go to the host.
    pub fn register_host_endpoint(
        &mut self,
        endpoint: u8,
        profile_id: u16,
        device_id: u16,
        device_version: u8,
        input_clusters: &[u16],
        output_clusters: &[u16],
    ) -> Result<(), HostEndpointError> {
        if !matches!(endpoint, 1..=240) {
            return Err(HostEndpointError::InvalidEndpoint);
        }
        let descriptor = zigbee_zdo::descriptors::SimpleDescriptor {
            endpoint,
            profile_id,
            device_id,
            device_version,
            input_clusters: Vec::from_slice(input_clusters)
                .map_err(|_| HostEndpointError::TooManyClusters)?,
            output_clusters: Vec::from_slice(output_clusters)
                .map_err(|_| HostEndpointError::TooManyClusters)?,
        };
        self.bdb
            .zdo_mut()
            .upsert_endpoint(descriptor)
            .map(|_| ())
            .map_err(|_| HostEndpointError::RegistryFull)
    }

    /// Remove endpoint `endpoint` from the ZDO server's registry.
    pub fn unregister_host_endpoint(&mut self, endpoint: u8) -> bool {
        self.bdb.zdo_mut().unregister_endpoint(endpoint).is_some()
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
        let plain = ApsRxSecurity::default();
        let secured = ApsRxSecurity {
            aps_secured: true,
            key_source: zigbee_aps::ApsRxKeySource::UniqueTrustCenter,
        };
        assert!(capture(
            &indication(1, 0x0006, &[1, 2]),
            &hop,
            -61,
            true,
            secured,
            None
        ));
        assert!(!capture(
            &indication(0, 0x0005, &[9]),
            &hop,
            -128,
            true,
            plain,
            None
        ));
        assert!(!capture(
            &indication(242, 0x0021, &[0]),
            &hop,
            -40,
            true,
            plain,
            None
        ));
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
        assert_eq!(
            (
                first.rssi,
                first.aps_secured,
                first.key_source,
                first.nwk_secured
            ),
            (-61, true, 2, true)
        );
        let zdo = take().unwrap();
        assert!(zdo.handled_by_ncp);
        assert_eq!(
            (zdo.rssi, zdo.aps_secured, zdo.key_source),
            (-128, false, 0)
        );
        assert_eq!(take(), None);

        for _ in 0..HOST_QUEUE_LEN + 2 {
            capture(&indication(1, 0x0006, &[0]), &hop, -50, true, plain, None);
        }
        assert!(capture(
            &indication(1, 0x0006, &[0; 200]),
            &hop,
            -50,
            true,
            plain,
            None
        ));
        assert_eq!(take_dropped(), 3);
        clear();
    }
}
