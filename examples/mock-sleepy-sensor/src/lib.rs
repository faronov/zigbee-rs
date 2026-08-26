//! Finite host composition of the shared sleepy-sensor application.

use core::convert::Infallible;

use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, FixedBattery, NoOta,
    NoStatus, NoUserAction, SensorApp, SensorPolicy, SensorSedParts, SleepDepth, StatusPolicy,
    Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::PlatformServices;
use zigbee_mac::mock::MockMac;
use zigbee_runtime::ZigbeeDevice;
use zigbee_runtime::node::ZigbeeNode;
use zigbee_runtime::profile::{
    ApplicationProfile, BatteryDescriptor, BatteryMeasurement, DeviceProfile,
    EnvironmentalReporting, TemperatureHumidityBattery, TemperatureRange,
};
use zigbee_runtime::security_store::{
    PersistentSecurityState, RamSecurityStateStore, SecurityStateStore,
};
use zigbee_types::{PanId, ShortAddress};
use zigbee_zcl::DeviceId;
use zigbee_zcl::clusters::basic::PowerSource;

const DEVICE_IEEE: [u8; 8] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
const COORDINATOR_IEEE: [u8; 8] = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
const EXTENDED_PAN_ID: [u8; 8] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11];
const PAN_ID: PanId = PanId(0x1A2B);
const SHORT_ADDRESS: ShortAddress = ShortAddress(0x04D2);
const CHANNEL: u8 = 15;

static POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 2,
    fast_poll_ms: 1,
    slow_poll_ms: 2,
    fresh_join_fast_ms: 4,
    restored_fast_ms: 4,
    wake_duration_ms: 1,
    join_retry_ms: 2,
    announce_retry_ms: 2,
    announce_retries: 0,
    secure_rejoin_failure_limit: 2,
    interview_complete_grace_ms: 1,
    button: sensor_sed_app::ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 1,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: 0,
        blink_on_ms: 0,
        blink_gap_ms: 0,
        reset_blinks: 0,
        reset_phase_ms: 0,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Active,
};

#[derive(Default)]
struct HostWake {
    now_ms: u32,
}

impl WakeController<MockMac> for HostWake {
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
        mac: &mut MockMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        self.now_ms = Self::add_ms(self.now_ms, request.timeout_ms);
        mac.delay_micros(request.timeout_ms.saturating_mul(1_000))
            .await;
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        self.now_ms = Self::add_ms(self.now_ms, duration_ms);
    }
}

#[derive(Default)]
struct DemoEnvironment {
    sample: u16,
}

impl EnvironmentSource for DemoEnvironment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        self.sample = self.sample.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius: 2_300 + (self.sample as i16 * 5),
            humidity_centi_percent: 6_500 + self.sample.saturating_mul(10),
            pressure_tenth_kpa: None,
        })
    }
}

#[derive(Default)]
struct HostSupervisor;

impl Supervisor for HostSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        panic!("unexpected host reset")
    }
}

#[derive(Default)]
struct HostDiagnostics;

impl Diagnostics for HostDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        println!("  {event:?}");
    }
}

fn sensor_profile() -> DeviceProfile<TemperatureHumidityBattery> {
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

fn commissioned_state() -> PersistentSecurityState {
    let mut state = PersistentSecurityState::empty();
    state.commissioned = true;
    state.extended_pan_id = EXTENDED_PAN_ID;
    state.pan_id = PAN_ID.0;
    state.short_address = SHORT_ADDRESS.0;
    state.ieee_address = DEVICE_IEEE;
    state.channel = CHANNEL;
    state.depth = 1;
    state.parent_address = ShortAddress::COORDINATOR.0;
    state.update_id = 0;
    state.update_id_valid = true;
    state.network_key = [0xA5; 16];
    state.key_sequence = 0;
    state.global_counter_limit = 0x400;
    state.tclk_present = true;
    state.trust_center_address = COORDINATOR_IEEE;
    state.trust_center_link_key = [0x5A; 16];
    state.tclk_counter_limit = 0x400;
    state.parent_information = 0;
    state.parent_information_valid = true;
    state.end_device_timeout = 8;
    state.validate().expect("valid host security state");
    state
}

/// Run one persisted resume and four finite application iterations.
pub async fn run_demo() {
    println!("zigbee-rs finite SensorApp host demo");

    let mut profile = sensor_profile();
    let mut device = ZigbeeDevice::builder(MockMac::new(DEVICE_IEEE))
        .manufacturer("Zigbee-RS")
        .model("MockSleepySensor-01")
        .power_source(PowerSource::Battery)
        .power_mode(POLICY.power_mode())
        .automatic_polling(false)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |endpoint| profile.configure_endpoint(endpoint),
        )
        .build();
    let mut store = RamSecurityStateStore::new();
    store
        .store(&commissioned_state())
        .expect("seed persisted network");
    let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);
    let battery = FixedBattery::new(
        3_000,
        BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 160,
        },
    );
    let mut app = SensorApp::new(
        node,
        &POLICY,
        SensorSedParts {
            wake: HostWake::default(),
            status: NoStatus,
            environment: DemoEnvironment::default(),
            battery,
            ota: NoOta,
            actions: NoUserAction,
            supervisor: HostSupervisor,
            diagnostics: HostDiagnostics,
        },
    )
    .expect("manual polling is the only parent-poll owner");

    app.initialize().await.expect("initialize SensorApp");
    for step in 1..=4 {
        app.step().await.expect("finite SensorApp step");
        println!("  completed finite sensor step {step}/4");
    }

    println!("SensorApp demo complete");
}
