//! Explicit static OTA lifecycle pairing.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::StackEvent;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    ApplicationProfile, DeviceProfile, ProfileComponent, TemperatureHumidityBattery,
    TemperatureHumidityPressureBattery,
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

/// Opt-in marker for complete profiles that do not advertise an OTA client.
///
/// OTA-decorated runtime profiles intentionally do not implement this marker.
/// A plain [`DeviceProfile`] qualifies only when its component explicitly
/// implements [`NonOtaComponent`].
pub trait NonOtaProfile: ApplicationProfile {}

impl<C: NonOtaComponent> NonOtaProfile for DeviceProfile<C> {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaEventOutcome {
    NotHandled,
    Handled {
        keep_awake_ms: Option<u32>,
        activation_pending: bool,
    },
    Unexpected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OtaActivationOutcome {
    Activated,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OtaServiceOutcome {
    pub keep_awake_ms: Option<u32>,
    pub activation_pending: bool,
}

impl OtaServiceOutcome {
    pub const IDLE: Self = Self {
        keep_awake_ms: None,
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
