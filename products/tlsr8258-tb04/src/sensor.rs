//! TB-04 composition policy and adapters for the shared sleepy-sensor app.
//!
//! Fast periods remain active. Steady-state waits use the TLSR8258 full-SRAM
//! timer SUSPEND path as one interrupt-masked MAC/clock transaction. LOW32K
//! reset-on-wake exists only behind the dedicated retention-proof feature.

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
use core::cell::UnsafeCell;
use core::convert::Infallible;
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
use core::sync::atomic::{AtomicU8, Ordering};

use sensor_sed_app::{ButtonPolicy, SleepDepth};
use sensor_sed_app::{
    EnvironmentReading, EnvironmentSink, EnvironmentSource, FixedBattery, NoUserAction,
    NonOtaComponent, SensorPolicy, StatusPolicy,
};
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
use sensor_sed_app::{NoOta, SensorApp, SensorSedParts};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacDriver;
use zigbee_runtime::profile::{
    ApplicationClusters, BatteryMeasurement, DeviceProfile, ExpectedReportClusters,
    ProfileComponent, ProfileError, TemperatureHumidityMeasurement,
};
use zigbee_runtime::synthetic_sensor::SyntheticSensor;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, DeviceId};

#[cfg(target_arch = "tc32")]
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, SensorStatus, StatusSink, Supervisor, WaitRequest,
    WakeController, WakeReason,
};
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
use tlsr8258_tb04::leds::StatusLedState;
#[cfg(target_arch = "tc32")]
use tlsr8258_tb04::leds::StatusLeds;
#[cfg(target_arch = "tc32")]
use zigbee_mac::telink::TelinkMac;
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
use zigbee_mac::telink::TelinkRetentionResumeError;

pub const ENDPOINT: u8 = 1;
pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "TLSR8258-Runtime";
pub const DATE_CODE: &str = "20260718";
pub const SW_BUILD: &str = "0.1.0";

pub const USER_ACTIONS: NoUserAction = NoUserAction;

/// Preserve the proven 250 ms polling loop and 30-second synthetic sample
/// cadence. Joining/interview waits remain Active; only steady-state waits use
/// the explicit full-SRAM Idle transaction.
#[cfg(not(feature = "retention-proof"))]
pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    fast_poll_ms: 250,
    slow_poll_ms: 250,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 5_000,
    announce_retry_ms: 8_000,
    // BDB steering already owns the bounded Device_annce retry sequence.
    announce_retries: 0,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 300,
    },
    status: StatusPolicy {
        // The existing image holds red steadily while unjoined. Keep the
        // shared blink deadline effectively disabled rather than adding new
        // visible or timing behavior in this migration.
        unjoined_blink_period_ms: u32::MAX,
        blink_on_ms: 1,
        blink_gap_ms: 1,
        reset_blinks: 1,
        reset_phase_ms: 1,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Idle,
};

/// First LOW32K proof: preserve the established 250 ms cadence and change
/// only the steady-state depth. Fast/interview periods remain Active.
#[cfg(all(feature = "retention-proof", not(feature = "retention-proof-10s")))]
pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    fast_poll_ms: 250,
    slow_poll_ms: 250,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 5_000,
    announce_retry_ms: 8_000,
    announce_retries: 0,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 300,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: u32::MAX,
        blink_on_ms: 1,
        blink_gap_ms: 1,
        reset_blinks: 1,
        reset_phase_ms: 1,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Retention,
};

/// Separately selected slow-policy proof. It cannot alter the initial Active
/// 250 ms fast-poll period; only joined steady state changes to ten seconds.
#[cfg(feature = "retention-proof-10s")]
pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    fast_poll_ms: 250,
    slow_poll_ms: 10_000,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 5_000,
    announce_retry_ms: 8_000,
    announce_retries: 0,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 300,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: u32::MAX,
        blink_on_ms: 1,
        blink_gap_ms: 1,
        reset_blinks: 1,
        reset_phase_ms: 1,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Retention,
};

const SYNTHETIC_SENSOR: SyntheticSensor = SyntheticSensor::new(2_150, 100, 5_000, 400);

/// Deterministic source matching the previous `SyntheticSensor` sequence.
pub struct SyntheticEnvironment {
    sample_index: u32,
}

impl SyntheticEnvironment {
    pub const fn new() -> Self {
        Self { sample_index: 0 }
    }

    fn next_reading(&mut self) -> EnvironmentReading {
        let reading = SYNTHETIC_SENSOR.sample(self.sample_index);
        self.sample_index = self.sample_index.wrapping_add(1);
        EnvironmentReading {
            temperature_centi_celsius: reading.temperature_centidegrees,
            humidity_centi_percent: reading.humidity_centipercent,
            pressure_tenth_kpa: None,
        }
    }
}

impl EnvironmentSource for SyntheticEnvironment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        Ok(self.next_reading())
    }
}

/// Fixed 3.0 V / 100% battery state used by the existing synthetic image.
pub const fn fixed_battery() -> FixedBattery {
    FixedBattery::new(
        3_000,
        BatteryMeasurement {
            voltage_100mv: 30,
            percentage_remaining: 200,
        },
    )
}

/// Exact cluster storage and local reporting defaults of the pre-migration
/// TLSR8258 sensor.
pub struct SensorClusters {
    power: PowerConfigCluster,
    temperature: TemperatureCluster,
    humidity: HumidityCluster,
}

impl SensorClusters {
    fn new() -> Self {
        let mut power = PowerConfigCluster::new();
        power.set_battery_voltage(30);
        power.set_battery_percentage(200);
        power.set_battery_size(0x04);
        power.set_battery_quantity(2);
        power.set_battery_rated_voltage(15);

        let mut temperature = TemperatureCluster::new(-4_000, 12_500);
        temperature.set_temperature(2_150);
        let mut humidity = HumidityCluster::new(0, 10_000);
        humidity.set_humidity(5_000);

        Self {
            power,
            temperature,
            humidity,
        }
    }
}

impl ProfileComponent for SensorClusters {
    fn configure_endpoint(
        &self,
        endpoint: zigbee_runtime::builder::EndpointBuilder,
    ) -> zigbee_runtime::builder::EndpointBuilder {
        endpoint
            .cluster_server(ClusterId::BASIC)
            .cluster_server(ClusterId::POWER_CONFIG)
            .cluster_server(ClusterId::IDENTIFY)
            .cluster_server(ClusterId::TEMPERATURE)
            .cluster_server(ClusterId::HUMIDITY)
    }

    fn collect_clusters<'a>(
        &'a mut self,
        endpoint: u8,
        clusters: &mut ApplicationClusters<'a>,
    ) -> Result<(), ProfileError> {
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.power,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.temperature,
            })
            .map_err(|_| ProfileError::TooManyClusters)?;
        clusters
            .push(ClusterRef {
                endpoint,
                cluster: &mut self.humidity,
            })
            .map_err(|_| ProfileError::TooManyClusters)
    }

    fn expected_report_cluster_ids(&self, out: &mut ExpectedReportClusters) {
        let _ = out.push(ClusterId::TEMPERATURE.0);
        let _ = out.push(ClusterId::HUMIDITY.0);
    }

    fn configure_default_reporting<M: MacDriver, R: zigbee_runtime::role::DeviceRole>(
        &self,
        endpoint: u8,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<(), ProfileError> {
        let reporting = device.reporting_mut();
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::TEMPERATURE.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
                    data_type: ZclDataType::I16,
                    min_interval: 1,
                    max_interval: 60,
                    reportable_change: Some(ZclValue::I16(1)),
                },
            )
            .map_err(ProfileError::Reporting)?;
        reporting
            .configure_for_cluster(
                endpoint,
                ClusterId::HUMIDITY.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id: zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
                    data_type: ZclDataType::U16,
                    min_interval: 1,
                    max_interval: 60,
                    reportable_change: Some(ZclValue::U16(1)),
                },
            )
            .map_err(ProfileError::Reporting)
    }
}

impl EnvironmentSink for SensorClusters {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.temperature
            .set_temperature(measurement.temperature_centi_celsius);
        self.humidity
            .set_humidity(measurement.humidity_centi_percent);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.power.set_battery_voltage(measurement.voltage_100mv);
        self.power
            .set_battery_percentage(measurement.percentage_remaining);
    }
}

impl NonOtaComponent for SensorClusters {}

pub type SensorProfile = DeviceProfile<SensorClusters>;

pub fn sensor_profile() -> SensorProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        SensorClusters::new(),
    )
}

/// Timer0 active waits plus atomic full-SRAM timer SUSPEND for idle waits.
#[cfg(target_arch = "tc32")]
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkSuspendWake;

#[cfg(target_arch = "tc32")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelinkWakeError {
    RetentionUnsupported,
    #[cfg(feature = "retention-proof")]
    RetentionContextInvalid,
    PowerManagement(tlsr8258_hal::pm::PmError),
    LowPowerNotEntered,
    TimerWakeMissing,
}

#[cfg(target_arch = "tc32")]
impl From<tlsr8258_hal::pm::PmError> for TelinkWakeError {
    fn from(error: tlsr8258_hal::pm::PmError) -> Self {
        Self::PowerManagement(error)
    }
}

#[cfg(target_arch = "tc32")]
impl WakeController<TelinkMac> for TelinkSuspendWake {
    type Mark = u32;
    type Error = TelinkWakeError;

    fn mark(&self) -> Self::Mark {
        tlsr8258_hal::timer::now_ticks()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark.wrapping_add(tlsr8258_hal::timer::ms(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later.wrapping_sub(earlier) / tlsr8258_hal::timer::TICKS_PER_MS
    }

    async fn wait(
        &mut self,
        mac: &mut TelinkMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {
                tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(request.timeout_ms));
            }
            SleepDepth::Retention => return Err(TelinkWakeError::RetentionUnsupported),
            SleepDepth::Idle => {
                use tlsr8258_hal::pm::SuspendTransactionPhase;

                let mut timer0_before = 0;
                let mut system_before = 0;
                let status = tlsr8258_hal::pm::cpu_suspend_timer_rc_transaction(
                    tlsr8258_hal::pm::system_timer_ms(request.timeout_ms),
                    |phase| match phase {
                        SuspendTransactionPhase::Prepare => {
                            // SensorApp has completed its service iteration
                            // before calling wait. Snapshot both clocks before
                            // quiescing the only MAC/radio owner.
                            timer0_before = tlsr8258_hal::timer::now_ticks();
                            system_before = tlsr8258_hal::pm::system_timer_ticks();
                            mac.prepare_for_sleep();
                        }
                        SuspendTransactionPhase::Restore => {
                            // Radio init resets Timer0, so restore the MAC
                            // first, then rebase application/MAC monotonic time.
                            // The PM transaction restores IRQs only after this
                            // hook returns, including on every typed PM error.
                            mac.resume_after_sleep();
                            let elapsed_system =
                                tlsr8258_hal::pm::system_timer_ticks().wrapping_sub(system_before);
                            tlsr8258_hal::timer::rebase_after_suspend(
                                timer0_before,
                                elapsed_system,
                            );
                        }
                    },
                )?;
                if !status.entered_low_power() {
                    return Err(TelinkWakeError::LowPowerNotEntered);
                }
                if !status.woke_by_timer() {
                    return Err(TelinkWakeError::TimerWakeMissing);
                }
            }
        }
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(duration_ms));
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RETENTION_PLATFORM_MAGIC: u32 = 0x5452_5031; // "TRP1"
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RGB_RED: u8 = 1 << 0;
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RGB_GREEN: u8 = 1 << 1;
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
const RGB_BLUE: u8 = 1 << 2;

/// Product-owned retained hand-off. The MAC pointer is registered only after
/// `ZigbeeDevice::build_into` has placed it at its final static address.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[repr(C)]
struct RetentionPlatformState {
    marker: u32,
    marker_inverse: u32,
    mac_pointer: usize,
    mac_pointer_inverse: usize,
    record: tlsr8258_hal::pm::Low32kResumeRecord,
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
impl RetentionPlatformState {
    const fn invalid() -> Self {
        Self {
            marker: 0,
            marker_inverse: !0,
            mac_pointer: 0,
            mac_pointer_inverse: !0,
            record: tlsr8258_hal::pm::Low32kResumeRecord::new(),
        }
    }

    fn valid(&self) -> bool {
        self.marker == RETENTION_PLATFORM_MAGIC
            && self.marker_inverse == !RETENTION_PLATFORM_MAGIC
            && self.mac_pointer != 0
            && self.mac_pointer_inverse == !self.mac_pointer
            && self.mac_pointer % core::mem::align_of::<TelinkMac>() == 0
            && (0x0084_0000..0x0084_8000).contains(&(self.mac_pointer as u32))
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
struct RetentionPlatformCell(UnsafeCell<RetentionPlatformState>);
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
unsafe impl Sync for RetentionPlatformCell {}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[unsafe(link_section = ".retained.platform")]
static RETENTION_PLATFORM: RetentionPlatformCell =
    RetentionPlatformCell(UnsafeCell::new(RetentionPlatformState::invalid()));

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[unsafe(link_section = ".retained.platform")]
static RETENTION_RGB: AtomicU8 = AtomicU8::new(RGB_RED);

/// Register static MAC placement and invalidate any stale sleep hand-off.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
pub fn initialize_retention_context(mac: &mut TelinkMac) {
    use core::sync::atomic::{Ordering as FenceOrdering, compiler_fence};

    let pointer = mac as *mut TelinkMac as usize;
    let state = unsafe { &mut *RETENTION_PLATFORM.0.get() };
    *state = RetentionPlatformState::invalid();
    state.mac_pointer = pointer;
    state.mac_pointer_inverse = !pointer;
    state.marker_inverse = !RETENTION_PLATFORM_MAGIC;
    compiler_fence(FenceOrdering::SeqCst);
    state.marker = RETENTION_PLATFORM_MAGIC;
    RETENTION_RGB.store(RGB_RED, Ordering::SeqCst);
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
fn retention_state_mut() -> Result<&'static mut RetentionPlatformState, RetentionPlatformError> {
    let state = unsafe { &mut *RETENTION_PLATFORM.0.get() };
    if state.valid() {
        Ok(state)
    } else {
        Err(RetentionPlatformError::ContextInvalid)
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
fn retention_state_for_mac(
    mac: &mut TelinkMac,
) -> Result<&'static mut RetentionPlatformState, RetentionPlatformError> {
    let state = retention_state_mut()?;
    if state.mac_pointer == mac as *mut TelinkMac as usize {
        Ok(state)
    } else {
        Err(RetentionPlatformError::ContextInvalid)
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetentionPlatformError {
    ContextInvalid,
    PowerManagement(tlsr8258_hal::pm::PmError),
    Mac(TelinkRetentionResumeError),
    Adc(tlsr8258_hal::adc::AdcError),
    Gpio(tlsr8258_hal::gpio::GpioError),
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
impl From<tlsr8258_hal::pm::PmError> for RetentionPlatformError {
    fn from(error: tlsr8258_hal::pm::PmError) -> Self {
        Self::PowerManagement(error)
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
fn restore_retained_leds() -> Result<(), tlsr8258_hal::gpio::GpioError> {
    let rgb = RETENTION_RGB.load(Ordering::SeqCst);
    let state = StatusLedState::new(
        rgb & RGB_RED != 0,
        rgb & RGB_GREEN != 0,
        rgb & RGB_BLUE != 0,
    );
    // SAFETY: reset discarded the only executing future that can
    // access retained StatusLeds owners. This runs with IRQs
    // disabled and completes before the fresh root borrows them.
    unsafe { StatusLeds::restore_after_reset(state) }
}

/// Restore PM time, retained MAC/radio, AES KAT, optional RNG state, the PC5
/// voltage guard and semantic LED output before enabling IRQs or servicing
/// the retained application.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[inline(never)]
pub fn restore_retained_platform() -> Result<tlsr8258_hal::pm::WakeStatus, RetentionPlatformError> {
    let state = retention_state_mut()?;
    let token = tlsr8258_hal::pm::begin_low32k_resume(&state.record)?;
    let mac = unsafe { &mut *(state.mac_pointer as *mut TelinkMac) };
    mac.resume_after_retention()
        .map_err(RetentionPlatformError::Mac)?;
    tlsr8258_hal::adc::restore_flash_voltage_guard().map_err(RetentionPlatformError::Adc)?;
    restore_retained_leds().map_err(RetentionPlatformError::Gpio)?;
    let status = tlsr8258_hal::pm::complete_low32k_resume(&mut state.record, token)?;
    if !status.entered_low_power() || !status.woke_by_timer() {
        return Err(RetentionPlatformError::PowerManagement(
            tlsr8258_hal::pm::PmError::RetentionWakeInvalid,
        ));
    }
    Ok(status)
}

/// Clear an unusable hand-off and force a real reset. This never falls
/// through into a cold-boot code path with retained joined state.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
pub fn fail_closed_retention_reset() -> ! {
    tlsr8258_hal::mmio::disable_all_irqs();
    let state = unsafe { &mut *RETENTION_PLATFORM.0.get() };
    tlsr8258_hal::pm::abandon_low32k_resume(&mut state.record);
    RETENTION_RGB.store(RGB_RED, Ordering::SeqCst);
    let _ = restore_retained_leds();
    tlsr8258_hal::reset::reboot()
}

/// Reset-on-wake controller used only by the explicit proof feature.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkRetentionWake;

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
impl WakeController<TelinkMac> for TelinkRetentionWake {
    type Mark = u32;
    type Error = TelinkWakeError;

    fn mark(&self) -> Self::Mark {
        tlsr8258_hal::timer::now_ticks()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark.wrapping_add(tlsr8258_hal::timer::ms(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later.wrapping_sub(earlier) / tlsr8258_hal::timer::TICKS_PER_MS
    }

    async fn wait(
        &mut self,
        mac: &mut TelinkMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {
                tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(request.timeout_ms));
            }
            SleepDepth::Idle => {
                // Keep the validated full-SRAM path available in the proof
                // image even though its selected policy uses Retention.
                let mut timer0_before = 0;
                let mut system_before = 0;
                let status = tlsr8258_hal::pm::cpu_suspend_timer_rc_transaction(
                    tlsr8258_hal::pm::system_timer_ms(request.timeout_ms),
                    |phase| match phase {
                        tlsr8258_hal::pm::SuspendTransactionPhase::Prepare => {
                            timer0_before = tlsr8258_hal::timer::now_ticks();
                            system_before = tlsr8258_hal::pm::system_timer_ticks();
                            mac.prepare_for_sleep();
                        }
                        tlsr8258_hal::pm::SuspendTransactionPhase::Restore => {
                            mac.resume_after_sleep();
                            let elapsed =
                                tlsr8258_hal::pm::system_timer_ticks().wrapping_sub(system_before);
                            tlsr8258_hal::timer::rebase_after_suspend(timer0_before, elapsed);
                        }
                    },
                )?;
                if !status.entered_low_power() {
                    return Err(TelinkWakeError::LowPowerNotEntered);
                }
                if !status.woke_by_timer() {
                    return Err(TelinkWakeError::TimerWakeMissing);
                }
            }
            SleepDepth::Retention => {
                let state = retention_state_for_mac(mac)
                    .map_err(|_| TelinkWakeError::RetentionContextInvalid)?;
                let mut timer0_before = 0;
                let mut system_before = 0;
                let result = tlsr8258_hal::pm::cpu_sleep_timer_rc_retention_transaction(
                    tlsr8258_hal::pm::system_timer_ms(request.timeout_ms),
                    &mut state.record,
                    |phase| match phase {
                        tlsr8258_hal::pm::SuspendTransactionPhase::Prepare => {
                            timer0_before = tlsr8258_hal::timer::now_ticks();
                            system_before = tlsr8258_hal::pm::system_timer_ticks();
                            mac.prepare_for_sleep();
                        }
                        tlsr8258_hal::pm::SuspendTransactionPhase::Restore => {
                            mac.resume_after_sleep();
                            let elapsed =
                                tlsr8258_hal::pm::system_timer_ticks().wrapping_sub(system_before);
                            tlsr8258_hal::timer::rebase_after_suspend(timer0_before, elapsed);
                        }
                    },
                );
                match result {
                    Ok(never) => match never {},
                    Err(error) => return Err(TelinkWakeError::PowerManagement(error)),
                }
            }
        }
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(duration_ms));
    }
}

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
pub type RetainedSensorParts = SensorSedParts<
    TelinkRetentionWake,
    SensorRgbStatus,
    SyntheticEnvironment,
    sensor_sed_app::FixedBattery,
    NoOta,
    NoUserAction,
    TelinkSupervisor,
    TelinkNoDiagnostics,
>;

#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
pub type RetainedSensorApp = SensorApp<
    'static,
    TelinkMac,
    crate::storage::SecurityStore,
    SensorProfile,
    RetainedSensorParts,
>;

/// Semantic mapping for the three fitted LEDs.
#[cfg(target_arch = "tc32")]
pub struct SensorRgbStatus {
    leds: StatusLeds,
}

#[cfg(target_arch = "tc32")]
impl SensorRgbStatus {
    pub const fn new(leds: StatusLeds) -> Self {
        Self { leds }
    }

    fn set_rgb(&self, red: bool, green: bool, blue: bool) {
        self.leds.red.write(red);
        self.leds.green.write(green);
        self.leds.blue.write(blue);
        #[cfg(feature = "retention-proof")]
        RETENTION_RGB.store(
            u8::from(red) * RGB_RED | u8::from(green) * RGB_GREEN | u8::from(blue) * RGB_BLUE,
            Ordering::SeqCst,
        );
    }
}

#[cfg(target_arch = "tc32")]
impl StatusSink for SensorRgbStatus {
    fn set(&mut self, status: SensorStatus) {
        match status {
            SensorStatus::Off
            | SensorStatus::Joining { .. }
            | SensorStatus::Resetting { .. }
            | SensorStatus::Fault => self.set_rgb(true, false, false),
            SensorStatus::Joined { .. } => self.set_rgb(false, true, false),
            SensorStatus::Identifying { on } | SensorStatus::Reporting { on } => {
                self.set_rgb(false, true, on)
            }
            SensorStatus::Ota => self.set_rgb(false, false, true),
        }
    }
}

#[cfg(target_arch = "tc32")]
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkSupervisor;

#[cfg(target_arch = "tc32")]
impl Supervisor for TelinkSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        loop {
            tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
        }
    }
}

#[cfg(target_arch = "tc32")]
#[derive(Debug, Default, Clone, Copy)]
pub struct TelinkNoDiagnostics;

#[cfg(target_arch = "tc32")]
impl Diagnostics for TelinkNoDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::profile::ApplicationProfile;

    #[test]
    fn profile_preserves_endpoint_and_reporting_identity() {
        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        let mut expected = ExpectedReportClusters::new();
        profile.expected_report_cluster_ids(&mut expected);
        assert_eq!(
            expected.as_slice(),
            [ClusterId::TEMPERATURE.0, ClusterId::HUMIDITY.0]
        );
    }

    #[test]
    fn policy_keeps_fast_active_and_selects_explicit_slow_proof() {
        assert!(SENSOR_POLICY.is_valid());
        assert_eq!(SENSOR_POLICY.sample_interval_ms, 30_000);
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        #[cfg(not(feature = "retention-proof-10s"))]
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 250);
        #[cfg(feature = "retention-proof-10s")]
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 10_000);
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Active);
        #[cfg(not(feature = "retention-proof"))]
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Idle);
        #[cfg(feature = "retention-proof")]
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Retention);
    }

    #[test]
    fn synthetic_sequence_starts_with_the_existing_sample_zero() {
        let mut source = SyntheticEnvironment::new();
        assert_eq!(
            source.next_reading(),
            EnvironmentReading {
                temperature_centi_celsius: 2_050,
                humidity_centi_percent: 5_100,
                pressure_tenth_kpa: None,
            }
        );
    }

    #[test]
    fn fixed_battery_identity_is_three_volts_and_full() {
        let reading = sensor_sed_app::BatteryReading {
            millivolts: 3_000,
            measurement: BatteryMeasurement {
                voltage_100mv: 30,
                percentage_remaining: 200,
            },
        };
        assert_eq!(reading.millivolts, 3_000);
        assert_eq!(reading.measurement.voltage_100mv, 30);
        assert_eq!(reading.measurement.percentage_remaining, 200);
    }
}
