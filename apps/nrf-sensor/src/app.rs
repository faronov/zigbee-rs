//! Source-compatible Nordic adapter around [`sensor_sed_app::SensorApp`].

use embassy_nrf::gpio;
use embassy_nrf::saadc::Saadc;
use sensor_sed_app::{EnvironmentSink, EnvironmentSource};
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{DeviceProfile, ProfileComponent};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::battery::{BatteryPolicy, NrfBattery};
use crate::diagnostics::NrfDiagnostics;
use crate::platform::{NrfPlatform, NrfRadioPower, SensorMac};

type SensorNode<'a, S, C> = ZigbeeNode<'a, SensorMac, S, DeviceProfile<C>>;
type SharedSensorApp<'a, S, C, E, B> = sensor_sed_app::SensorApp<
    'a,
    SensorMac,
    S,
    C,
    E,
    NrfBattery<B>,
    NrfPlatform,
    NrfRadioPower,
    NrfDiagnostics,
>;

/// Compatibility wrapper retained while Nordic composition roots migrate
/// from the former chip-family application crate to `apps/sensor-sed`.
pub struct SensorApp<'a, S, C, E, B>
where
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink,
    E: EnvironmentSource,
    B: BatteryPolicy,
{
    inner: SharedSensorApp<'a, S, C, E, B>,
}

impl<'a, S, C, E, B> SensorApp<'a, S, C, E, B>
where
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink,
    E: EnvironmentSource,
    B: BatteryPolicy,
{
    pub fn new(
        node: SensorNode<'a, S, C>,
        led: gpio::Output<'static>,
        button: gpio::Input<'static>,
        environment: E,
        saadc: Saadc<'static, 1>,
    ) -> Self {
        Self {
            inner: sensor_sed_app::SensorApp::new(
                node,
                NrfPlatform::new(led, button),
                NrfRadioPower,
                NrfDiagnostics,
                environment,
                NrfBattery::new(saadc),
            ),
        }
    }

    pub async fn run(&mut self) -> ! {
        self.inner.run().await
    }
}
