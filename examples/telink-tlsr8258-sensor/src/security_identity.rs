use zigbee_mac::telink::TelinkMac;
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};
use zigbee_runtime::ZigbeeDevice;

pub fn prepare<S: SecurityStateStore>(
    device: &mut ZigbeeDevice<TelinkMac>,
    store: &mut S,
    configured_ieee: [u8; 8],
) -> Result<(), SecurityStoreError> {
    let Some(state) = store.load()? else {
        return Ok(());
    };
    if state.ieee_address != [0; 8] && state.ieee_address != configured_ieee {
        device.factory_reset_security_state(store)?;
    }
    Ok(())
}
