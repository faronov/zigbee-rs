//! Composition wrapper for a device, durable security store, and profile.

use crate::ZigbeeDevice;
use crate::event_loop::{StackEvent, StartError, TickResult};
use crate::profile::{ApplicationClusters, ApplicationProfile, ProfileError};
use crate::role::{DeviceRole, EndDevice};
use crate::security_store::{PersistentSecurityState, SecurityStateStore, SecurityStoreError};
use zigbee_mac::{MacDriver, McpsDataIndication};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeError {
    Profile(ProfileError),
    Persistence(SecurityStoreError),
}

impl From<ProfileError> for NodeError {
    fn from(error: ProfileError) -> Self {
        Self::Profile(error)
    }
}

impl From<SecurityStoreError> for NodeError {
    fn from(error: SecurityStoreError) -> Self {
        Self::Persistence(error)
    }
}

/// Owns the three objects every persistent Zigbee application needs.
///
/// Generic over the device's logical [`DeviceRole`] `R` (defaulting to
/// [`EndDevice`] so existing `ZigbeeNode<'_, M, S, P>` source is unchanged), so
/// a router product can compose a `ZigbeeDevice<M, Router>` without a bespoke
/// wrapper.
pub struct ZigbeeNode<'a, M, S, P, R = EndDevice>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    R: DeviceRole,
{
    device: &'a mut ZigbeeDevice<M, R>,
    security_store: &'a mut S,
    profile: &'a mut P,
}

impl<'a, M, S, P, R> ZigbeeNode<'a, M, S, P, R>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
    R: DeviceRole,
{
    pub const fn new(
        device: &'a mut ZigbeeDevice<M, R>,
        security_store: &'a mut S,
        profile: &'a mut P,
    ) -> Self {
        Self {
            device,
            security_store,
            profile,
        }
    }

    pub const fn device(&self) -> &ZigbeeDevice<M, R> {
        self.device
    }

    pub fn device_mut(&mut self) -> &mut ZigbeeDevice<M, R> {
        self.device
    }

    pub const fn profile(&self) -> &P {
        self.profile
    }

    pub fn profile_mut(&mut self) -> &mut P {
        self.profile
    }

    /// Disjoint mutable access to the device and the profile.
    ///
    /// `device_mut()` and `profile_mut()` each reborrow the whole `self`, so
    /// a caller that needs both at once (for example, application code that
    /// hands an OTA transport helper both the device and the profile's OTA
    /// backend in the same call) cannot call them in the same expression.
    /// This splits the two fields explicitly instead.
    pub fn device_and_profile_mut(&mut self) -> (&mut ZigbeeDevice<M, R>, &mut P) {
        (self.device, self.profile)
    }

    pub fn load_security_state(
        &mut self,
    ) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
        self.security_store.load()
    }

    pub fn checkpoint_security(&mut self) -> Result<bool, SecurityStoreError> {
        self.device
            .refresh_security_state(&mut *self.security_store)
    }

    pub async fn start_or_resume(&mut self) -> Result<u16, StartError> {
        self.device
            .start_or_resume_with_security_store(&mut *self.security_store)
            .await
    }

    pub async fn secure_rejoin(&mut self) -> Result<u16, StartError> {
        self.device
            .secure_rejoin_with_security_store(&mut *self.security_store)
            .await
    }

    pub async fn factory_reset(&mut self) -> Result<(), StartError> {
        self.device
            .factory_reset_with_security_store(&mut *self.security_store)
            .await
    }

    pub fn configure_default_reporting(&mut self) -> Result<(), ProfileError> {
        let Self {
            device, profile, ..
        } = self;
        profile.configure_default_reporting(device)
    }

    pub fn reporting_is_configured(&self) -> bool {
        self.device
            .configured_cluster_count(self.profile.endpoint())
            >= self.profile.expected_report_clusters()
    }

    pub async fn tick(&mut self, elapsed_secs: u16) -> Result<TickResult, NodeError> {
        let Self {
            device,
            security_store,
            profile,
        } = self;
        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters)?;
        device
            .tick_with_security_store(elapsed_secs, clusters.as_mut_slice(), *security_store)
            .await
            .map_err(NodeError::Persistence)
    }

    pub async fn process_incoming(
        &mut self,
        indication: &McpsDataIndication,
    ) -> Result<Option<StackEvent>, NodeError> {
        let Self {
            device,
            security_store,
            profile,
        } = self;
        let mut clusters = ApplicationClusters::new();
        profile.collect_clusters(&mut clusters)?;
        device
            .process_incoming_with_security_store(
                indication,
                clusters.as_mut_slice(),
                *security_store,
            )
            .await
            .map_err(NodeError::Persistence)
    }
}
