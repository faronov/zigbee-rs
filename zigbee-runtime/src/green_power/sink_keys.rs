//! The sink's view of this node (`green-power-sink`): its addresses and,
//! with `green-power-key-provisioning`, the network's active key, which the
//! shared GPD key types 1 (network key) and 3 (NWK-key derived group key)
//! need.

use zigbee_bdb::BdbLayer;
use zigbee_mac::MacDriver;

use crate::green_power::GpSink;

/// Give `sink` this node's addresses (and the active network key).
#[inline(always)]
pub(crate) fn sync<M: MacDriver>(
    sink: &mut GpSink,
    bdb: &BdbLayer<M>,
    nwk: u16,
    ieee: [u8; 8],
    pan_id: u16,
) {
    sink.set_own_address(nwk, ieee, pan_id);
    #[cfg(feature = "green-power-key-provisioning")]
    sink.set_network_key(bdb.zdo().nwk().security().active_key().map(|k| k.key));
    #[cfg(not(feature = "green-power-key-provisioning"))]
    let _ = bdb;
}
