//! gpsSecurityLevel InvolveTC from how this node joined
//! (`green-power-involve-tc`, gecko_sdk `writeInvolveTCBit`).

use zigbee_bdb::attributes::NodeJoinLinkKeyType;
use zigbee_gp::sink::JoinKind;
use zigbee_mac::MacDriver;
use zigbee_nwk::DeviceType;

use crate::ZigbeeDevice;
use crate::role::DeviceRole;

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// How this node joined its network, as InvolveTC needs it: the
    /// coordinator, or the node holding the Trust Center address, is the
    /// Trust Center; otherwise the persisted `bdbNodeJoinLinkKeyType`
    /// tells a distributed network, an install-code (non-default) key or
    /// the default global key apart.
    pub fn gp_join_kind(&self) -> JoinKind {
        let zdo = self.bdb.zdo();
        let nwk = zdo.nwk();
        let own_ieee = nwk.nib().ieee_address;
        if nwk.device_type() == DeviceType::Coordinator
            || zdo.aps().aib().aps_trust_center_address == own_ieee
        {
            return JoinKind::TrustCenter;
        }
        match self.bdb.attributes().node_join_link_key_type {
            t if t.is_distributed() => JoinKind::Distributed,
            NodeJoinLinkKeyType::InstallCodeDerivedPreconfiguredLinkKey => {
                JoinKind::CentralizedPreconfiguredKey
            }
            _ => JoinKind::CentralizedDefaultKey,
        }
    }

    /// Update the sink's InvolveTC from [`Self::gp_join_kind`]; call once
    /// per join (a later write of gpsSecurityLevel by a tool then sticks
    /// until the next join). Returns whether InvolveTC is set and stops
    /// GPD commissioning on this node.
    pub fn gp_sink_derive_involve_tc(&mut self) -> bool {
        let join = self.gp_join_kind();
        let set = self.gp_sink_mut().derive_involve_tc(join);
        let blocks = set && !self.gp_sink().is_trust_center();
        log::info!(
            "[GP] sink InvolveTC {} after {:?} join{}",
            set,
            join,
            if blocks {
                ": GPD commissioning refused"
            } else {
                ""
            }
        );
        blocks
    }
}

#[cfg(all(test, feature = "router"))]
mod tests {
    use zigbee_bdb::attributes::NodeJoinLinkKeyType;
    use zigbee_gp::sink::JoinKind;
    use zigbee_mac::mock::MockMac;
    use zigbee_nwk::DeviceType;

    use crate::ZigbeeDevice;
    use crate::role::Router;

    fn joined_router(key: NodeJoinLinkKeyType) -> ZigbeeDevice<MockMac, Router> {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .device_type(DeviceType::Router)
            .build_router();
        let bdb = device.bdb_mut();
        bdb.attributes_mut().node_is_on_a_network = true;
        bdb.attributes_mut().node_join_link_key_type = key;
        let aps = bdb.zdo_mut().aps_mut();
        aps.aib_mut().aps_trust_center_address = [0x44; 8];
        let nwk = aps.nwk_mut();
        nwk.set_joined(true);
        let nib = nwk.nib_mut();
        nib.network_address = zigbee_types::ShortAddress(0x0001);
        nib.ieee_address = [1, 2, 3, 4, 5, 6, 7, 8];
        nib.logical_channel = 15;
        device
    }

    #[test]
    fn an_install_code_join_sets_involve_tc_and_stops_commissioning() {
        let mut router = joined_router(NodeJoinLinkKeyType::InstallCodeDerivedPreconfiguredLinkKey);
        assert_eq!(router.gp_join_kind(), JoinKind::CentralizedPreconfiguredKey);
        assert!(router.gp_sink_derive_involve_tc());
        assert_eq!(
            router.gp_sink_enter_commissioning(false),
            Err(zigbee_gp::Error::Unsupported)
        );
    }

    #[test]
    fn a_default_key_join_leaves_commissioning_open() {
        let mut router = joined_router(NodeJoinLinkKeyType::DefaultGlobalTrustCenterLinkKey);
        assert_eq!(router.gp_join_kind(), JoinKind::CentralizedDefaultKey);
        assert!(!router.gp_sink_derive_involve_tc());
        assert!(router.gp_sink_enter_commissioning(false).is_ok());
    }

    #[test]
    fn the_trust_center_is_never_stopped() {
        let mut router = joined_router(NodeJoinLinkKeyType::InstallCodeDerivedPreconfiguredLinkKey);
        router
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .aib_mut()
            .aps_trust_center_address = [1, 2, 3, 4, 5, 6, 7, 8];
        assert_eq!(router.gp_join_kind(), JoinKind::TrustCenter);
        assert!(!router.gp_sink_derive_involve_tc());
        assert!(router.gp_sink().is_trust_center());
    }
}
