//! Explicit static OTA lifecycle pairing.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    ApplicationProfile, DeviceProfile, PlantSensor, ProfileComponent, RangeExtender,
    TemperatureHumidityBattery, TemperatureHumidityPressureBattery,
};
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_zcl::ClusterId;

/// Explicit assertion that a profile component does not advertise OTA.
///
/// This is deliberately not blanket-implemented for every [`ProfileComponent`]:
/// a custom component controls its endpoint descriptor and collected clusters,
/// so only its owner can truthfully opt it into [`NoOta`] pairing.
pub trait NonOtaComponent: ProfileComponent {}

impl NonOtaComponent for TemperatureHumidityBattery {}
impl NonOtaComponent for TemperatureHumidityPressureBattery {}
impl NonOtaComponent for PlantSensor {}
impl NonOtaComponent for RangeExtender {}

/// Opt-in marker for complete profiles that do not advertise an OTA client.
///
/// OTA-decorated runtime profiles intentionally do not implement this marker.
/// A plain [`DeviceProfile`] qualifies only when its component explicitly
/// implements [`NonOtaComponent`].
pub trait NonOtaProfile: ApplicationProfile {}

impl<C: NonOtaComponent> NonOtaProfile for DeviceProfile<C> {}

/// Result of offering one stack event to the OTA lifecycle.
///
/// There is deliberately no keep-awake duration: the shared application
/// keeps the fast poll (and the CPU) up exactly while
/// [`OtaLifecycle::is_active`] reports a session — query sent, download,
/// verification, activation wait — bounded by
/// [`RecoveryPolicy::max_ota_awake_ms`](crate::RecoveryPolicy::max_ota_awake_ms).
/// A Query Next Image Response with NO_IMAGE_AVAILABLE or any other
/// non-success status, a declined Image Notify (QueryJitter), a response
/// timeout or a failure all end the session and with it the fast poll on
/// the next service iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaEventOutcome {
    NotHandled,
    Handled { activation_pending: bool },
    Unexpected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaActivationOutcome {
    Activated,
    Failed,
}

/// Result of one [`OtaLifecycle::service`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaServiceOutcome {
    /// The service call produced OTA traffic or a status change (a query,
    /// progress, a failure); it marks the status indicator, nothing else.
    /// The fast poll follows [`OtaLifecycle::is_active`].
    pub progressed: bool,
    pub activation_pending: bool,
}

impl OtaServiceOutcome {
    pub const IDLE: Self = Self {
        progressed: false,
        activation_pending: false,
    };
}

/// Product-selected OTA transport and activation lifecycle.
///
/// The profile type is part of the bound, which makes an OTA-advertising
/// profile require a matching concrete implementation at compile time.
#[allow(async_fn_in_trait)]
pub trait OtaLifecycle<M, S, P>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: ApplicationProfile,
{
    const ENABLED: bool;

    /// Whether an OTA session is in progress. While true the shared
    /// application fast-polls with the CPU awake; it must turn false as soon
    /// as the session ends for any reason (no image, non-success status,
    /// declined notify, timeout, failure, abort).
    fn is_active(&self, profile: &P) -> bool;
    fn next_deadline_ms(&self, profile: &P) -> Option<u32>;

    async fn handle_event(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, P>,
        event: &StackEvent,
    ) -> OtaEventOutcome;

    async fn service(
        &mut self,
        node: &mut ZigbeeNode<'_, M, S, P>,
        elapsed_secs: u16,
    ) -> OtaServiceOutcome;

    /// Activate a verified image after the shared lifecycle has checkpointed
    /// the Zigbee security state.
    ///
    /// `handle_event` and `service` must report `activation_pending` instead
    /// of activating directly. This keeps the reset-causing operation behind
    /// the application's durable checkpoint.
    fn activate(&mut self, node: &mut ZigbeeNode<'_, M, S, P>) -> OtaActivationOutcome;

    /// Abandon the running session because it kept the chip awake longer
    /// than [`RecoveryPolicy::max_ota_awake_ms`](crate::RecoveryPolicy::max_ota_awake_ms).
    ///
    /// Afterwards [`is_active`](Self::is_active) must report `false`. A
    /// partially staged image may be kept for a later resume, but nothing
    /// may be activated.
    fn abort(&mut self, node: &mut ZigbeeNode<'_, M, S, P>);

    /// Confirm the running image to the bootloader once it has proven
    /// itself on the network (see `SensorApp`): an installed image is on
    /// trial until then, and a loader that can revert reverts it after its
    /// trial boots. Called at most until it returns `true`; `false` (a
    /// flash error) retries at the next proof. Products without trial
    /// accounting keep the default.
    fn confirm_image(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) -> bool {
        true
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoOta;

impl<M, S, P> OtaLifecycle<M, S, P> for NoOta
where
    M: MacDriver,
    S: SecurityStateStore,
    P: NonOtaProfile,
{
    const ENABLED: bool = false;

    fn is_active(&self, _profile: &P) -> bool {
        false
    }

    fn next_deadline_ms(&self, _profile: &P) -> Option<u32> {
        None
    }

    async fn handle_event(
        &mut self,
        _node: &mut ZigbeeNode<'_, M, S, P>,
        event: &StackEvent,
    ) -> OtaEventOutcome {
        if is_ota_event(event) {
            OtaEventOutcome::Unexpected
        } else {
            OtaEventOutcome::NotHandled
        }
    }

    async fn service(
        &mut self,
        _node: &mut ZigbeeNode<'_, M, S, P>,
        _elapsed_secs: u16,
    ) -> OtaServiceOutcome {
        OtaServiceOutcome::IDLE
    }

    fn activate(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) -> OtaActivationOutcome {
        OtaActivationOutcome::Failed
    }

    /// Never active, so there is never a session to abort.
    fn abort(&mut self, _node: &mut ZigbeeNode<'_, M, S, P>) {}
}

pub const fn is_ota_event(event: &StackEvent) -> bool {
    matches!(
        event,
        StackEvent::CommandReceived { cluster_id, .. }
            if *cluster_id == ClusterId::OTA_UPGRADE.0
    ) || matches!(
        event,
        StackEvent::OtaImageAvailable { .. }
            | StackEvent::OtaProgress { .. }
            | StackEvent::OtaComplete
            | StackEvent::OtaFailed
            | StackEvent::OtaDelayedActivation { .. }
    )
}
