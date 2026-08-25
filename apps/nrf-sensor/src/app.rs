//! Source-compatible Nordic adapter around [`sensor_sed_app::SensorApp`].

use embassy_nrf::gpio;
use embassy_nrf::saadc::Saadc;
use sensor_sed_app::{
    EnvironmentSink, EnvironmentSource, ForceReportAction, NoOta, NonOtaComponent, SensorPolicy,
    SensorSedParts,
};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{DeviceProfile, ProfileComponent};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::battery::{BatteryPolicy, NrfBattery};
use crate::diagnostics::NrfDiagnostics;
use crate::platform::{NrfStatus, NrfSupervisor, NrfWakeController, SensorMac};

type SensorNode<'a, S, C> = ZigbeeNode<'a, SensorMac, S, DeviceProfile<C>>;
type SharedSensorApp<'a, S, C, E, B> = sensor_sed_app::SensorApp<
    'a,
    SensorMac,
    S,
    DeviceProfile<C>,
    SensorSedParts<
        NrfWakeController,
        NrfStatus,
        E,
        NrfBattery<B>,
        NoOta,
        ForceReportAction,
        NrfSupervisor,
        NrfDiagnostics,
    >,
>;

/// Compatibility wrapper retained while Nordic composition roots migrate
/// from the former chip-family application crate to `apps/sensor-sed`.
pub struct SensorApp<'a, S, C, E, B>
where
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink + NonOtaComponent,
    E: EnvironmentSource,
    B: BatteryPolicy,
{
    inner: SharedSensorApp<'a, S, C, E, B>,
}

impl<'a, S, C, E, B> SensorApp<'a, S, C, E, B>
where
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink + NonOtaComponent,
    E: EnvironmentSource,
    B: BatteryPolicy,
{
    pub fn new(
        node: SensorNode<'a, S, C>,
        policy: &'static SensorPolicy,
        led: gpio::Output<'static>,
        button: gpio::Input<'static>,
        environment: E,
        saadc: Saadc<'static, 1>,
    ) -> Self {
        Self {
            inner: sensor_sed_app::SensorApp::new(
                node,
                policy,
                SensorSedParts {
                    wake: NrfWakeController::new(button),
                    status: NrfStatus::new(led),
                    environment,
                    battery: NrfBattery::new(saadc),
                    ota: NoOta,
                    actions: ForceReportAction,
                    supervisor: NrfSupervisor,
                    diagnostics: NrfDiagnostics,
                },
            )
            .expect("nRF sensor composition must disable automatic polling"),
        }
    }

    pub async fn run(&mut self) -> ! {
        self.inner.run().await
    }
}
