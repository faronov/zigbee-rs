//! Home Automation On/Off Light profile.
//!
//! The component owns the On/Off (0x0006) server: the full HA command set of
//! [`OnOffCluster`] (Off/On/Toggle, OffWithEffect, OnWithRecallGlobalScene,
//! OnWithTimedOff) with its OnTime/OffWaitTime countdown. Basic and Identify
//! remain runtime-owned. Groups and Scenes are not advertised: as for
//! [`super::DimmableLight`], their fixed-capacity state is a product choice.
//!
//! The profile drives no output itself. The product reads [`OnOffLight::is_on`]
//! after each stack step and shows it on whatever light the board has. The
//! OnOff state is not persisted; the light starts off after a reset.

use super::{ApplicationClusters, ExpectedReportClusters, ProfileComponent, ProfileError};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::on_off::{ATTR_ON_OFF, OnOffCluster};
use zigbee_zcl::data_types::ZclDataType;
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

/// Default OnOff reporting: any change at once, heartbeat every hour.
pub const DEFAULT_ON_OFF_MAX_SECS: u16 = 3_600;

/// Basic + Identify + On/Off server on one endpoint.
///
/// Its non-generic methods are `#[inline]`: they are code-generated only in
/// an image that uses the profile, so adding it leaves every other image's
/// code generation unchanged.
pub struct OnOffLight {
    on_off: OnOffCluster,
    on_off_max_secs: u16,
}

impl OnOffLight {
    /// A light that starts off and reports its OnOff attribute on change,
    /// with an `on_off_max_secs` heartbeat.
    #[inline]
    pub fn new(on_off_max_secs: u16) -> Self {
        Self {
            on_off: OnOffCluster::new(),
            on_off_max_secs,
        }
    }

    /// The OnOff attribute.
    #[inline]
    pub fn is_on(&self) -> bool {
        self.on_off.is_on()
    }

    #[inline]
    pub const fn on_off(&self) -> &OnOffCluster {
        &self.on_off
    }

    #[inline]
    pub fn on_off_mut(&mut self) -> &mut OnOffCluster {
        &mut self.on_off
    }

    /// Advance OnTime/OffWaitTime by the elapsed deciseconds. See
    /// [`OnOffCluster::tick_by`].
    #[inline]
    pub fn tick_by(&mut self, elapsed_deciseconds: u32) {
        self.on_off.tick_by(elapsed_deciseconds);
    }
}

impl Default for OnOffLight {
    #[inline]
    fn default() -> Self {
        Self::new(DEFAULT_ON_OFF_MAX_SECS)
    }
}

impl ProfileComponent for OnOffLight {
    #[inline]
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(ClusterId::ON_OFF)
    }

    #[inline]
    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.on_off,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    #[inline]
    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        let _ = out.push(ClusterId::ON_OFF.0);
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        // Bool is discrete: every change is reported, max_interval is the
        // heartbeat.
        device
            .reporting_mut()
            .configure_for_cluster(
                endpoint,
                ClusterId::ON_OFF.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_ON_OFF,
                    data_type: ZclDataType::Bool,
                    min_interval: 0,
                    max_interval: self.on_off_max_secs,
                    reportable_change: None,
                },
            )
            .map_err(ProfileError::Reporting)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{ApplicationProfile, DeviceProfile};
    use zigbee_aps::PROFILE_HOME_AUTOMATION;
    use zigbee_mac::mock::MockMac;
    use zigbee_zcl::DeviceId;
    use zigbee_zcl::clusters::on_off::CMD_TOGGLE;

    fn profile() -> DeviceProfile<OnOffLight> {
        DeviceProfile::new(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::ON_OFF_LIGHT,
            OnOffLight::default(),
        )
    }

    #[test]
    fn endpoint_declares_basic_identify_and_on_off() {
        let profile = profile();
        let device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build();
        let descriptor = device.bdb().zdo().find_endpoint(1).unwrap();
        assert_eq!(descriptor.device_id, DeviceId::ON_OFF_LIGHT.0);
        assert_eq!(
            descriptor.input_clusters.as_slice(),
            &[0x0000, 0x0003, 0x0006]
        );
        assert!(descriptor.output_clusters.is_empty());
        assert_eq!(profile.expected_report_clusters(), 1);
    }

    #[test]
    fn collected_on_off_cluster_drives_is_on() {
        let mut profile = profile();
        assert!(!profile.component().is_on());
        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters).unwrap();
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].endpoint, 1);
        clusters[0].cluster.handle_command(CMD_TOGGLE, &[]).unwrap();
        drop(clusters);
        assert!(profile.component().is_on());
    }

    #[test]
    fn default_reporting_reports_on_off_changes() {
        let profile = profile();
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                profile.endpoint(),
                profile.profile_id(),
                profile.device_id(),
                |endpoint| profile.configure_endpoint(endpoint),
            )
            .build();
        profile.configure_default_reporting(&mut device).unwrap();
    }
}
