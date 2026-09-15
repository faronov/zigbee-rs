use core::convert::Infallible;

use sensor_sed_app::{
    BatteryReading, BatterySource, ButtonPolicy, DiagnosticEvent, Diagnostics, EnvironmentReading,
    EnvironmentSource, ForceReportAction, NoOta, NonOtaComponent, NonOtaProfile, SensorApp,
    SensorAppError, SensorPolicy, SensorSedParts, SensorStatus, SleepDepth, StatusPolicy,
    StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::builder::EndpointBuilder;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    ApplicationClusters, BatteryDescriptor, BatteryMeasurement, DeviceProfile,
    EnvironmentalReporting, ExpectedReportClusters, ProfileComponent, ProfileError,
    TemperatureHumidityBattery, TemperatureHumidityPressureBattery, TemperatureRange,
};
use zigbee_runtime::security_store::RamSecurityStateStore;
use zigbee_zcl::{ClusterId, DeviceId};

static POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 60_000,
    fast_poll_ms: 250,
    slow_poll_ms: 30_000,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 60_000,
    wake_duration_ms: 500,
    join_retry_ms: 15_000,
    announce_retry_ms: 8_000,
    announce_retries: 5,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: Some(3_000),
        debounce_ms: 300,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: 1_000,
        blink_on_ms: 80,
        blink_gap_ms: 120,
        reset_blinks: 5,
        reset_phase_ms: 100,
    },
    fast_sleep_depth: SleepDepth::Idle,
    slow_sleep_depth: SleepDepth::Idle,
};

struct MockWake {
    now_ms: u32,
}

impl WakeController<MockMac> for MockWake {
    type Mark = u32;
    type Error = Infallible;

    fn mark(&self) -> Self::Mark {
        self.now_ms
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark.wrapping_add(duration_ms)
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later.wrapping_sub(earlier)
    }

    async fn wait(
        &mut self,
        _mac: &mut MockMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        self.now_ms = self.now_ms.wrapping_add(request.timeout_ms);
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(duration_ms);
    }
}

struct MockStatus;

impl StatusSink for MockStatus {
    fn set(&mut self, _status: SensorStatus) {}
}

struct MockSupervisor;

impl Supervisor for MockSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        panic!("mock reset")
    }
}

struct MockDiagnostics;

impl Diagnostics for MockDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
}

struct MockEnvironment;

impl EnvironmentSource for MockEnvironment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        Ok(EnvironmentReading {
            temperature_centi_celsius: 2_100,
            humidity_centi_percent: 5_000,
            pressure_tenth_kpa: None,
        })
    }
}

struct MockBattery;

impl BatterySource for MockBattery {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<Option<BatteryReading>, Self::Error> {
        Ok(Some(BatteryReading {
            millivolts: 3_000,
            measurement: BatteryMeasurement {
                voltage_100mv: 30,
                percentage_remaining: 200,
            },
        }))
    }
}

/// An otherwise valid component that advertises the OTA client cluster but
/// has not made the explicit `NonOtaComponent` assertion.
struct ArbitraryOtaComponent;

impl ProfileComponent for ArbitraryOtaComponent {
    fn configure_endpoint(&self, endpoint: EndpointBuilder) -> EndpointBuilder {
        endpoint.cluster_client(ClusterId::OTA_UPGRADE)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        _endpoint: u8,
        _clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        Ok(())
    }

    fn expected_report_cluster_ids(&self, _out: &mut ExpectedReportClusters) {}
}

// Stable negative-implementation assertion: if DeviceProfile<ArbitraryOtaComponent>
// ever gains NonOtaProfile implicitly, the `_` below has two valid marker types
// and this test stops compiling.
trait AmbiguousIfNonOta<A> {
    fn marker() {}
}

impl<T: ?Sized> AmbiguousIfNonOta<()> for T {}

struct ImplementsNonOta;

impl<T: ?Sized + NonOtaProfile> AmbiguousIfNonOta<ImplementsNonOta> for T {}

fn require_non_ota_component<T: NonOtaComponent>() {}

fn profile() -> DeviceProfile<TemperatureHumidityBattery> {
    DeviceProfile::new(
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
    )
}

fn parts() -> SensorSedParts<
    MockWake,
    MockStatus,
    MockEnvironment,
    MockBattery,
    NoOta,
    ForceReportAction,
    MockSupervisor,
    MockDiagnostics,
> {
    SensorSedParts {
        wake: MockWake { now_ms: 42 },
        status: MockStatus,
        environment: MockEnvironment,
        battery: MockBattery,
        ota: NoOta,
        actions: ForceReportAction,
        supervisor: MockSupervisor,
        diagnostics: MockDiagnostics,
    }
}

#[test]
fn standard_environment_components_explicitly_opt_into_no_ota() {
    require_non_ota_component::<TemperatureHumidityBattery>();
    require_non_ota_component::<TemperatureHumidityPressureBattery>();
}

#[test]
fn arbitrary_profile_component_is_not_implicitly_non_ota() {
    let _ = <DeviceProfile<ArbitraryOtaComponent> as AmbiguousIfNonOta<_>>::marker;
}

#[test]
fn constructs_with_explicit_static_capabilities() {
    let mut device = ZigbeeDevice::builder(MockMac::new([0x11; 8]))
        .automatic_polling(false)
        .build();
    let mut store = RamSecurityStateStore::new();
    let mut profile = profile();
    let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);

    assert!(SensorApp::new(node, &POLICY, parts()).is_ok());
}

#[test]
fn rejects_a_second_parent_poll_owner() {
    let mut device = ZigbeeDevice::builder(MockMac::new([0x11; 8])).build();
    let mut store = RamSecurityStateStore::new();
    let mut profile = profile();
    let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);

    assert!(matches!(
        SensorApp::new(node, &POLICY, parts()),
        Err(SensorAppError::AutomaticPollingEnabled)
    ));
}

#[test]
fn wrapping_marks_preserve_elapsed_time() {
    let mut wake = MockWake {
        now_ms: u32::MAX - 4,
    };
    let mark = wake.mark();
    wake.now_ms = wake.now_ms.wrapping_add(10);
    assert_eq!(MockWake::elapsed_ms(wake.mark(), mark), 10);
    assert_eq!(MockWake::add_ms(mark, 10), 5);
}
