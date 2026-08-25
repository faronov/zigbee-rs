use sensor_sed_app::{
    BatteryReading, BatterySource, DiagnosticEvent, Diagnostics, EnvironmentReading,
    EnvironmentSource, LifecyclePlatform, RadioPower, SensorApp, WakeReason,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacError;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    BatteryDescriptor, BatteryMeasurement, DeviceProfile, EnvironmentalReporting,
    TemperatureHumidityBattery, TemperatureRange,
};
use zigbee_runtime::security_store::RamSecurityStateStore;
use zigbee_zcl::DeviceId;

struct MockPlatform {
    now_ms: u64,
    led_on: bool,
}

impl LifecyclePlatform for MockPlatform {
    type Instant = u64;

    fn now(&self) -> Self::Instant {
        self.now_ms
    }

    fn add_millis(instant: Self::Instant, duration_ms: u64) -> Self::Instant {
        instant.saturating_add(duration_ms)
    }

    fn elapsed_millis(later: Self::Instant, earlier: Self::Instant) -> u64 {
        later.saturating_sub(earlier)
    }

    async fn wait_for_wake(&mut self, timeout_ms: u64) -> WakeReason {
        self.now_ms = self.now_ms.saturating_add(timeout_ms);
        WakeReason::Timer
    }

    async fn button_held_for(&mut self, _duration_ms: u64) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u64) {
        self.now_ms = self.now_ms.saturating_add(duration_ms);
    }

    fn led_on(&mut self) {
        self.led_on = true;
    }

    fn led_off(&mut self) {
        self.led_on = false;
    }

    fn led_toggle(&mut self) {
        self.led_on = !self.led_on;
    }

    fn reset(&mut self) -> ! {
        panic!("mock reset")
    }
}

struct MockRadioPower;

impl RadioPower<MockMac> for MockRadioPower {
    fn prepare_for_sleep(&mut self, _mac: &mut MockMac) -> Result<(), MacError> {
        Ok(())
    }
}

struct MockDiagnostics;

impl Diagnostics for MockDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
}

struct MockEnvironment;

impl EnvironmentSource for MockEnvironment {
    async fn sample(&mut self) -> Option<EnvironmentReading> {
        None
    }

    fn log_reading(&self, _reading: &EnvironmentReading) {}
}

struct MockBattery;

impl BatterySource for MockBattery {
    async fn sample(&mut self) -> BatteryReading {
        BatteryReading {
            millivolts: 3_000,
            measurement: BatteryMeasurement {
                voltage_100mv: 30,
                percentage_remaining: 200,
            },
        }
    }
}

#[test]
fn constructs_with_static_host_capabilities() {
    let mut device = ZigbeeDevice::builder(MockMac::new([0x11; 8])).build();
    let mut store = RamSecurityStateStore::new();
    let mut profile = DeviceProfile::new(
        1,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        TemperatureHumidityBattery::new(
            TemperatureRange {
                min_centi_celsius: -4_000,
                max_centi_celsius: 12_500,
            },
            BatteryDescriptor {
                size: 4,
                quantity: 2,
                rated_voltage_100mv: 15,
            },
            EnvironmentalReporting::default(),
        ),
    );
    let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);

    let _app = SensorApp::new(
        node,
        MockPlatform {
            now_ms: 42,
            led_on: false,
        },
        MockRadioPower,
        MockDiagnostics,
        MockEnvironment,
        MockBattery,
    );
}
