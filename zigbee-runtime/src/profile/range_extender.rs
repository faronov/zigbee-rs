//! Home Automation range-extender profile.

use super::{ApplicationClusters, ProfileComponent, ProfileError};
use crate::ZigbeeDevice;
use crate::builder::EndpointBuilder;
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;

/// Stateless Basic + Identify profile for an always-on Zigbee router.
#[derive(Debug, Clone, Copy, Default)]
pub struct RangeExtender;

impl ProfileComponent for RangeExtender {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        _endpoint: u8,
        _clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        Ok(())
    }

    fn configure_default_reporting<M: MacDriver>(
        &self,
        _endpoint: u8,
        _device: &mut ZigbeeDevice<M>,
    ) -> Result<(), ProfileError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ZigbeeDevice;
    use crate::profile::{ApplicationProfile, DeviceProfile};
    use zigbee_aps::PROFILE_HOME_AUTOMATION;
    use zigbee_mac::mock::MockMac;
    use zigbee_zcl::DeviceId;

    #[test]
    fn profile_declares_only_basic_and_identify() {
        let profile = DeviceProfile::new(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::RANGE_EXTENDER,
            RangeExtender,
        );
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build();
        let descriptor = device.bdb().zdo().find_endpoint(1).unwrap();

        assert_eq!(descriptor.input_clusters.as_slice(), &[0x0000, 0x0003]);
        assert!(descriptor.output_clusters.is_empty());
    }
}
