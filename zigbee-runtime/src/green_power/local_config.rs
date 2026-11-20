//! Locally issued GP sink configuration (`green-power-translation-table`):
//! the node's own application applies a GP Pairing Configuration or GP
//! Translation Table Update through the same sink path a commissioning
//! tool's broadcast takes, e.g. a product provisioning step or a lab image
//! that retargets a paired GPD without a remote tool.

use zigbee_aps::apsde::ApsdeDataIndication;
use zigbee_aps::{ApsAddress, ApsAddressMode};
use zigbee_gp::cluster::server_cmd;
use zigbee_gp::{GP_CLUSTER_ID, GP_ENDPOINT, GP_PROFILE_ID};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;

use crate::ZigbeeDevice;
use crate::role::DeviceRole;

/// ZCL frame control: cluster specific, client to server, Default Response
/// disabled.
const ZCL_CLIENT_TO_SERVER_NO_RESPONSE: u8 = 0x11;
/// Source address of a locally issued command: the unassigned NWK address,
/// never a real node, so the sink's drop of its own looped-back Pairing
/// Configuration does not apply and no reply is addressed to a device.
const LOCAL_SOURCE: ShortAddress = ShortAddress(0xFFFE);
/// Largest configuration payload accepted (a Pairing Configuration with a
/// key, a full group list and application information fits).
const MAX_LOCAL_PAYLOAD: usize = 125;

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// Apply a GP Translation Table Update (`0x07`) or GP Pairing
    /// Configuration (`0x09`) issued by this node's application as if a
    /// commissioning tool had broadcast it: the sink validates and applies
    /// it, the GP journal persists the result on the next persistence pass,
    /// and a pairing with "send GP Pairing" is announced to proxies. A
    /// broadcast gets no ZCL response, so a refused command only leaves the
    /// tables unchanged; inspect [`gp_sink`](Self::gp_sink) afterwards.
    ///
    /// Returns `false` (nothing applied) for any other command, an
    /// oversized payload, or while the node is not a joined router.
    pub fn gp_sink_configure_locally(&mut self, command_id: u8, payload: &[u8]) -> bool {
        if !matches!(
            command_id,
            server_cmd::TRANSLATION_TABLE_UPDATE | server_cmd::PAIRING_CONFIGURATION
        ) || payload.len() > MAX_LOCAL_PAYLOAD
        {
            return false;
        }
        let nwk = self.bdb.zdo().nwk();
        if !nwk.is_joined() || nwk.device_type() == zigbee_nwk::DeviceType::EndDevice {
            return false;
        }
        let mut frame = heapless::Vec::<u8, { MAX_LOCAL_PAYLOAD + 3 }>::new();
        if frame
            .extend_from_slice(&[ZCL_CLIENT_TO_SERVER_NO_RESPONSE, 0, command_id])
            .is_err()
            || frame.extend_from_slice(payload).is_err()
        {
            return false;
        }
        let indication = ApsdeDataIndication {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(ShortAddress(0xFFFD)),
            dst_endpoint: GP_ENDPOINT,
            src_addr_mode: ApsAddressMode::Short,
            src_address: ApsAddress::Short(LOCAL_SOURCE),
            src_endpoint: GP_ENDPOINT,
            profile_id: GP_PROFILE_ID,
            cluster_id: GP_CLUSTER_ID,
            payload: &frame,
            aps_counter: 0,
            security_status: false,
            lqi: 0xFF,
        };
        self.green_power.set_rx_unicast(false);
        self.gp_handle_aps(&indication, LOCAL_SOURCE.0, true)
    }
}
