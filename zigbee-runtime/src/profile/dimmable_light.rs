//! Home Automation dimmable-light profile.
//!
//! The component owns the command-bearing On/Off and Level Control cluster
//! instances. Basic and Identify remain runtime-owned. Groups and Scenes are
//! deliberately not advertised here: adding their fixed-capacity state is a
//! product choice rather than a prerequisite for commanded on/off and level
//! control.

use super::{ApplicationClusters, ExpectedReportClusters, ProfileComponent, ProfileError};
use crate::builder::EndpointBuilder;
use crate::{ClusterRef, ZigbeeDevice};
use zigbee_mac::MacDriver;
use zigbee_zcl::clusters::level_control::{
    ATTR_CURRENT_LEVEL, ATTR_MIN_LEVEL, CMD_MOVE, CMD_MOVE_TO_LEVEL, CMD_MOVE_TO_LEVEL_WITH_ON_OFF,
    CMD_MOVE_WITH_ON_OFF, CMD_STEP, CMD_STEP_WITH_ON_OFF, CMD_STOP, CMD_STOP_WITH_ON_OFF,
    LevelControlCluster,
};
use zigbee_zcl::clusters::on_off::{ATTR_ON_OFF, CMD_OFF, CMD_ON, OnOffCluster};
use zigbee_zcl::clusters::{AttributeStoreAccess, AttributeStoreMutAccess, Cluster};
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, CommandId, ZclStatus};

/// Default reporting cadence for the command state of a dimmable light.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DimmableLightReporting {
    /// Maximum heartbeat interval for the discrete OnOff state.
    pub on_off_max_secs: u16,
    /// Minimum interval between CurrentLevel reports.
    pub level_min_secs: u16,
    /// Maximum heartbeat interval for CurrentLevel.
    pub level_max_secs: u16,
    /// Minimum CurrentLevel change that triggers a report.
    pub level_change: u8,
}

impl Default for DimmableLightReporting {
    fn default() -> Self {
        Self {
            on_off_max_secs: 3_600,
            level_min_secs: 1,
            level_max_secs: 3_600,
            level_change: 1,
        }
    }
}

/// Level Control state plus the cross-cluster action required by the
/// WithOnOff command family.
///
/// The generic `zigbee-zcl` cluster deliberately owns only Level Control
/// attributes. This profile wrapper records the corresponding OnOff action so
/// [`DimmableLight`] can apply it to the On/Off cluster that it owns on the
/// same endpoint. Standalone Move/Step/Stop commands retain the generic
/// cluster's level-only behavior.
struct DimmableLevelControlCluster {
    inner: LevelControlCluster,
    pending_on_off: Option<bool>,
    turn_off_at_minimum: bool,
}

impl DimmableLevelControlCluster {
    fn new() -> Self {
        Self {
            inner: LevelControlCluster::new(),
            pending_on_off: None,
            turn_off_at_minimum: false,
        }
    }

    fn current_level(&self) -> u8 {
        self.inner.current_level()
    }

    fn minimum_level(&self) -> u8 {
        match self.inner.attributes().get(ATTR_MIN_LEVEL) {
            Some(ZclValue::U8(level)) => *level,
            _ => 1,
        }
    }

    fn projected_on_off(&self, current: bool) -> bool {
        self.pending_on_off.unwrap_or(current)
    }

    fn take_pending_on_off(&mut self) -> Option<bool> {
        self.pending_on_off.take()
    }

    fn apply_with_on_off_target(&mut self, target: u8, completes_immediately: bool) {
        let minimum = self.minimum_level();
        if target > minimum {
            // ZCL Level Control: a WithOnOff command whose resulting level is
            // above the device minimum turns the endpoint on before movement.
            self.pending_on_off = Some(true);
            self.turn_off_at_minimum = false;
        } else if completes_immediately || self.current_level() <= minimum {
            self.pending_on_off = Some(false);
            self.turn_off_at_minimum = false;
        } else {
            // A downward transition stays on while light output is still
            // above minimum and turns off when the transition reaches it.
            self.turn_off_at_minimum = true;
        }
    }

    fn tick(&mut self, elapsed_ds: u16) {
        self.inner.tick(elapsed_ds);
        if self.turn_off_at_minimum && self.current_level() <= self.minimum_level() {
            self.pending_on_off = Some(false);
            self.turn_off_at_minimum = false;
        } else if self.turn_off_at_minimum
            && self.inner.transitions.remaining_ds(ATTR_CURRENT_LEVEL.0) == 0
        {
            // Defensive cancellation if the underlying transition is stopped
            // or replaced before reaching the minimum.
            self.turn_off_at_minimum = false;
        }
    }

    fn handle_level_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        let current = self.current_level();
        let response = self.inner.handle_command(cmd_id, payload)?;

        match cmd_id {
            CMD_MOVE_TO_LEVEL_WITH_ON_OFF => {
                let target = payload[0];
                let transition_time = u16::from_le_bytes([payload[1], payload[2]]);
                self.apply_with_on_off_target(target, transition_time == 0);
            }
            CMD_MOVE_WITH_ON_OFF => {
                let target = if payload[0] == 0 { 0xFE } else { 0x00 };
                self.apply_with_on_off_target(target, payload[1] == 0);
            }
            CMD_STEP_WITH_ON_OFF => {
                let target = if payload[0] == 0 {
                    current.saturating_add(payload[1]).min(0xFE)
                } else {
                    current.saturating_sub(payload[1]).max(self.minimum_level())
                };
                let transition_time = u16::from_le_bytes([payload[2], payload[3]]);
                self.apply_with_on_off_target(target, transition_time == 0);
            }
            CMD_STOP_WITH_ON_OFF => {
                // StopWithOnOff does not itself toggle OnOff. It only prevents
                // a previously armed downward transition from turning off
                // after movement has been halted above the minimum.
                self.turn_off_at_minimum = false;
            }
            CMD_MOVE_TO_LEVEL | CMD_MOVE | CMD_STEP | CMD_STOP => {
                // A standalone command may replace or stop an earlier
                // transition, but must never change the OnOff attribute.
                self.turn_off_at_minimum = false;
            }
            _ => {}
        }

        Ok(response)
    }
}

impl Cluster for DimmableLevelControlCluster {
    fn cluster_id(&self) -> ClusterId {
        ClusterId::LEVEL_CONTROL
    }

    fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        self.handle_level_command(cmd_id, payload)
    }

    fn attributes(&self) -> &dyn AttributeStoreAccess {
        self.inner.attributes()
    }

    fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        self.inner.attributes_mut()
    }

    fn received_commands(&self) -> heapless::Vec<u8, 32> {
        self.inner.received_commands()
    }

    fn reset_to_factory_defaults(&mut self) {
        self.inner.reset_to_factory_defaults();
        self.pending_on_off = None;
        self.turn_off_at_minimum = false;
    }
}

/// Mutable Level Control view that immediately applies cross-cluster OnOff
/// effects for locally issued commands.
pub struct DimmableLevelControlMut<'a> {
    on_off: &'a mut OnOffCluster,
    level_control: &'a mut DimmableLevelControlCluster,
}

impl DimmableLevelControlMut<'_> {
    pub fn current_level(&self) -> u8 {
        self.level_control.current_level()
    }

    pub fn tick(&mut self, elapsed_ds: u16) {
        self.level_control.tick(elapsed_ds);
        synchronize_on_off(self.on_off, self.level_control);
    }

    pub fn handle_command(
        &mut self,
        cmd_id: CommandId,
        payload: &[u8],
    ) -> Result<heapless::Vec<u8, 64>, ZclStatus> {
        let result = self.level_control.handle_level_command(cmd_id, payload);
        if result.is_ok() {
            synchronize_on_off(self.on_off, self.level_control);
        }
        result
    }

    pub fn attributes(&self) -> &dyn AttributeStoreAccess {
        self.level_control.attributes()
    }

    pub fn attributes_mut(&mut self) -> &mut dyn AttributeStoreMutAccess {
        self.level_control.attributes_mut()
    }
}

fn synchronize_on_off(on_off: &mut OnOffCluster, level_control: &mut DimmableLevelControlCluster) {
    if let Some(on) = level_control.take_pending_on_off() {
        let command = if on { CMD_ON } else { CMD_OFF };
        let result = on_off.handle_command(command, &[]);
        debug_assert!(result.is_ok());
    }
}

/// On/Off + Level Control cluster composition for a dimmable light.
pub struct DimmableLight {
    on_off: OnOffCluster,
    level_control: DimmableLevelControlCluster,
    reporting: DimmableLightReporting,
}

impl DimmableLight {
    pub fn new(reporting: DimmableLightReporting) -> Self {
        Self {
            on_off: OnOffCluster::new(),
            level_control: DimmableLevelControlCluster::new(),
            reporting,
        }
    }

    pub const fn reporting(&self) -> &DimmableLightReporting {
        &self.reporting
    }

    pub fn is_on(&self) -> bool {
        self.level_control.projected_on_off(self.on_off.is_on())
    }

    pub fn current_level(&self) -> u8 {
        self.level_control.current_level()
    }

    pub fn on_off(&mut self) -> &OnOffCluster {
        self.synchronize_on_off();
        &self.on_off
    }

    pub fn on_off_mut(&mut self) -> &mut OnOffCluster {
        self.synchronize_on_off();
        &mut self.on_off
    }

    pub const fn level_control(&self) -> &LevelControlCluster {
        &self.level_control.inner
    }

    pub fn level_control_mut(&mut self) -> DimmableLevelControlMut<'_> {
        self.synchronize_on_off();
        DimmableLevelControlMut {
            on_off: &mut self.on_off,
            level_control: &mut self.level_control,
        }
    }

    /// Advance the OnTime/OffWaitTime state by one decisecond.
    pub fn tick_on_off(&mut self) {
        self.synchronize_on_off();
        self.on_off.tick();
    }

    /// Advance any in-flight level transition by `elapsed_ds` deciseconds.
    pub fn tick_level(&mut self, elapsed_ds: u16) {
        self.level_control_mut().tick(elapsed_ds);
    }

    /// Apply the configured StartUpOnOff value to the previous physical state.
    pub fn apply_startup_on_off(&mut self, previous_on: bool) {
        self.synchronize_on_off();
        self.on_off.apply_startup(previous_on);
    }

    fn synchronize_on_off(&mut self) {
        synchronize_on_off(&mut self.on_off, &mut self.level_control);
    }
}

impl Default for DimmableLight {
    fn default() -> Self {
        Self::new(DimmableLightReporting::default())
    }
}

impl ProfileComponent for DimmableLight {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(ClusterId::ON_OFF)
            .cluster_server(ClusterId::LEVEL_CONTROL)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        self.synchronize_on_off();
        let Self {
            on_off,
            level_control,
            ..
        } = self;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: on_off,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: level_control,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        let _ = out.push(ClusterId::ON_OFF.0);
        let _ = out.push(ClusterId::LEVEL_CONTROL.0);
    }

    fn configure_default_reporting<M: MacDriver, R: crate::role::DeviceRole>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        let reporting = device.reporting_mut();
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::ON_OFF.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_ON_OFF,
                    data_type: ZclDataType::Bool,
                    min_interval: 0,
                    max_interval: self.reporting.on_off_max_secs,
                    reportable_change: None,
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::LEVEL_CONTROL.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: ATTR_CURRENT_LEVEL,
                    data_type: ZclDataType::U8,
                    min_interval: self.reporting.level_min_secs,
                    max_interval: self.reporting.level_max_secs,
                    reportable_change: Some(ZclValue::U8(self.reporting.level_change)),
                },
            )
            .map_err(ProfileError::Reporting)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ZigbeeDevice;
    use crate::builder::EndpointBuilder;
    use crate::profile::ProfileComponent;
    use zigbee_aps::PROFILE_HOME_AUTOMATION;
    use zigbee_mac::mock::MockMac;
    use zigbee_zcl::DeviceId;
    use zigbee_zcl::clusters::Cluster;
    use zigbee_zcl::foundation::reporting::ReportDirection;

    fn endpoint_builder() -> EndpointBuilder {
        EndpointBuilder {
            endpoint: 1,
            profile_id: PROFILE_HOME_AUTOMATION,
            device_id: DeviceId::DIMMABLE_LIGHT,
            device_version: 1,
            server_clusters: heapless::Vec::new(),
            client_clusters: heapless::Vec::new(),
        }
    }

    #[test]
    fn default_state_and_endpoint_are_explicit() {
        let component = DimmableLight::default();
        assert!(!component.is_on());
        assert_eq!(component.current_level(), 0);
        assert_eq!(*component.reporting(), DimmableLightReporting::default());

        let endpoint = component.configure_endpoint(endpoint_builder());
        assert_eq!(
            endpoint.server_clusters.as_slice(),
            &[
                ClusterId::BASIC,
                ClusterId::IDENTIFY,
                ClusterId::ON_OFF,
                ClusterId::LEVEL_CONTROL,
            ]
        );
        assert!(endpoint.client_clusters.is_empty());
    }

    #[test]
    fn owns_command_state_and_cluster_collection() {
        let mut component = DimmableLight::default();
        component.on_off_mut().handle_command(CMD_ON, &[]).unwrap();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL, &[200, 0, 0])
            .unwrap();
        assert!(component.is_on());
        assert_eq!(component.current_level(), 200);

        let mut clusters = ApplicationClusters::new();
        component.collect_clusters(1, &mut clusters).unwrap();
        assert_eq!(
            clusters
                .iter()
                .map(|cluster| cluster.cluster.cluster_id())
                .collect::<heapless::Vec<_, 2>>()
                .as_slice(),
            &[ClusterId::ON_OFF, ClusterId::LEVEL_CONTROL]
        );
        drop(clusters);
        assert_eq!(component.expected_report_clusters(), 2);
    }

    #[test]
    fn level_transition_is_visible_through_state_accessors() {
        let mut component = DimmableLight::default();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL, &[100, 10, 0])
            .unwrap();
        component.tick_level(5);
        assert_eq!(component.current_level(), 50);
        component.tick_level(5);
        assert_eq!(component.current_level(), 100);
    }

    #[test]
    fn move_to_level_with_on_off_turns_on_before_the_level_changes() {
        let mut component = DimmableLight::default();
        assert!(!component.is_on());
        assert_eq!(component.current_level(), 0);

        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[100, 10, 0])
            .unwrap();

        assert!(
            component.is_on(),
            "WithOnOff must turn on before commencing an upward transition"
        );
        assert!(component.on_off().is_on());
        assert_eq!(component.current_level(), 0);

        component.tick_level(5);
        assert!(component.is_on());
        assert_eq!(component.current_level(), 50);
        component.tick_level(5);
        assert!(component.is_on());
        assert_eq!(component.current_level(), 100);
    }

    #[test]
    fn move_to_minimum_with_on_off_turns_off_only_at_minimum() {
        let mut component = DimmableLight::default();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[100, 0, 0])
            .unwrap();
        assert!(component.is_on());

        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[1, 10, 0])
            .unwrap();
        assert!(component.is_on());
        assert_eq!(component.current_level(), 100);

        component.tick_level(9);
        assert!(component.is_on());
        assert!(component.current_level() > 1);
        component.tick_level(1);
        assert_eq!(component.current_level(), 1);
        assert!(!component.is_on());
        assert!(!component.on_off().is_on());
    }

    #[test]
    fn stop_with_on_off_above_minimum_cancels_the_deferred_off() {
        let mut component = DimmableLight::default();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[100, 0, 0])
            .unwrap();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_WITH_ON_OFF, &[1, 10])
            .unwrap();
        component.tick_level(10);
        let stopped_level = component.current_level();
        assert!(stopped_level > 1);
        assert!(component.is_on());

        component
            .level_control_mut()
            .handle_command(CMD_STOP_WITH_ON_OFF, &[])
            .unwrap();
        component.tick_level(u16::MAX);

        assert_eq!(component.current_level(), stopped_level);
        assert!(
            component.is_on(),
            "stopping above minimum must not complete the deferred off"
        );
    }

    #[test]
    fn standalone_level_commands_do_not_change_on_off() {
        let mut component = DimmableLight::default();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL, &[100, 0, 0])
            .unwrap();
        assert!(!component.is_on());

        component.on_off_mut().handle_command(CMD_ON, &[]).unwrap();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL, &[1, 0, 0])
            .unwrap();
        assert!(component.is_on());
    }

    #[test]
    fn step_with_on_off_tracks_levels_above_and_at_minimum() {
        let mut component = DimmableLight::default();
        component
            .level_control_mut()
            .handle_command(CMD_MOVE_TO_LEVEL, &[10, 0, 0])
            .unwrap();
        assert!(!component.is_on());

        component
            .level_control_mut()
            .handle_command(CMD_STEP_WITH_ON_OFF, &[0, 10, 0, 0])
            .unwrap();
        assert_eq!(component.current_level(), 20);
        assert!(component.is_on());

        component
            .level_control_mut()
            .handle_command(CMD_STEP_WITH_ON_OFF, &[1, u8::MAX, 0, 0])
            .unwrap();
        assert_eq!(component.current_level(), 1);
        assert!(!component.is_on());
    }

    #[test]
    fn collected_level_cluster_projects_and_commits_with_on_off_state() {
        let mut component = DimmableLight::default();
        let mut clusters = ApplicationClusters::new();
        component.collect_clusters(1, &mut clusters).unwrap();
        clusters
            .iter_mut()
            .find(|entry| entry.cluster.cluster_id() == ClusterId::LEVEL_CONTROL)
            .unwrap()
            .cluster
            .handle_command(CMD_MOVE_TO_LEVEL_WITH_ON_OFF, &[80, 0, 0])
            .unwrap();
        drop(clusters);

        assert!(component.is_on());
        assert!(
            component.on_off().is_on(),
            "the next profile access commits the queued cross-cluster action"
        );
        assert_eq!(component.current_level(), 80);
    }

    #[test]
    fn default_reporting_covers_on_off_and_current_level() {
        let mut device = ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
            .endpoint(
                1,
                PROFILE_HOME_AUTOMATION,
                DeviceId::DIMMABLE_LIGHT,
                |endpoint| DimmableLight::default().configure_endpoint(endpoint),
            )
            .build();
        let component = DimmableLight::default();
        component
            .configure_default_reporting(1, &mut device)
            .unwrap();

        assert_eq!(device.configured_cluster_count(1), 2);
        let on_off = device
            .reporting()
            .get_config(1, ClusterId::ON_OFF.0, ReportDirection::Send, ATTR_ON_OFF)
            .unwrap();
        assert_eq!(on_off.min_interval, 0);
        assert_eq!(on_off.max_interval, 3_600);
        assert_eq!(on_off.reportable_change, None);

        let level = device
            .reporting()
            .get_config(
                1,
                ClusterId::LEVEL_CONTROL.0,
                ReportDirection::Send,
                ATTR_CURRENT_LEVEL,
            )
            .unwrap();
        assert_eq!(level.min_interval, 1);
        assert_eq!(level.max_interval, 3_600);
        assert_eq!(level.reportable_change, Some(ZclValue::U8(1)));
    }
}
