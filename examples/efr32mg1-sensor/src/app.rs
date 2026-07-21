//! Production Zigbee SED state machine.

use efr32mg1_hal::pm;
use efr32mg1_tradfri::BatteryMonitor;
use efr32mg1_tradfri::storage::SecurityStore;
use embassy_futures::select;
use embassy_time::{Duration, Instant, Timer};
use zigbee_mac::efr32::Efr32Mac;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
#[cfg(feature = "ota")]
use zigbee_runtime::ota::OtaManager;
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
#[cfg(feature = "ota")]
use zigbee_types::ShortAddress;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::humidity::HumidityCluster;
#[cfg(feature = "ota")]
use zigbee_zcl::clusters::ota::{CMD_IMAGE_NOTIFY, OtaState};
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

use crate::{platform, sensor, vectors};

const JOIN_RETRY_SECS: u64 = 15;
const REPORT_INTERVAL_SECS: u64 = 60;
const BUTTON_DEBOUNCE_MS: u64 = 80;
const BUTTON_LONG_PRESS_SECS: u64 = 3;
const BUTTON_FAST_POLL_SECS: u64 = 5;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;
const RESTORED_FAST_POLL_SECS: u64 = 60;
const EXPECTED_REPORT_CLUSTERS: usize = 3;
macro_rules! application_clusters {
    ($app:expr) => {
        [
            ClusterRef {
                endpoint: 1,
                cluster: &mut *$app.temp_cluster,
            },
            ClusterRef {
                endpoint: 1,
                cluster: &mut *$app.hum_cluster,
            },
            ClusterRef {
                endpoint: 1,
                cluster: &mut *$app.power_cluster,
            },
            #[cfg(feature = "ota")]
            ClusterRef {
                endpoint: 1,
                cluster: $app.ota.cluster_mut(),
            },
        ]
    };
}

pub struct SensorApp {
    device: &'static mut ZigbeeDevice<Efr32Mac>,
    security_store: &'static mut SecurityStore,
    sht: sensor::Sht3x,
    battery: Option<BatteryMonitor>,
    temp_cluster: &'static mut TemperatureCluster,
    hum_cluster: &'static mut HumidityCluster,
    power_cluster: &'static mut PowerConfigCluster,
    last_report: Instant,
    last_tick: Instant,
    fast_poll_until: Instant,
    last_rejoin_attempt: Instant,
    annce_retries_left: u8,
    last_annce: Instant,
    was_fast_polling: bool,
    was_identifying: bool,
    interview_done: bool,
    needs_checkpoint: bool,
    needs_bootstrap_join: bool,
    awaiting_initial_configuration: bool,
    restoring_commissioned_state: bool,
    #[cfg(feature = "ota")]
    ota: OtaManager<efr32mg1_tradfri::ota::Efr32FirmwareWriter>,
    #[cfg(feature = "ota")]
    ota_server: Option<(u16, u8)>,
    #[cfg(feature = "ota")]
    ota_cleanup_pending: bool,
}

impl SensorApp {
    pub fn new(
        device: &'static mut ZigbeeDevice<Efr32Mac>,
        security_store: &'static mut SecurityStore,
        sht: sensor::Sht3x,
        battery: Option<BatteryMonitor>,
        temp_cluster: &'static mut TemperatureCluster,
        hum_cluster: &'static mut HumidityCluster,
        power_cluster: &'static mut PowerConfigCluster,
        #[cfg(feature = "ota")] ota: OtaManager<efr32mg1_tradfri::ota::Efr32FirmwareWriter>,
    ) -> Self {
        let now = Instant::now();
        let joined = device.is_joined();
        Self {
            device,
            security_store,
            sht,
            battery,
            temp_cluster,
            hum_cluster,
            power_cluster,
            last_report: now,
            last_tick: now,
            fast_poll_until: if joined {
                now + Duration::from_secs(FAST_POLL_DURATION_SECS)
            } else {
                now
            },
            last_rejoin_attempt: now,
            annce_retries_left: 0,
            last_annce: now,
            was_fast_polling: joined,
            was_identifying: false,
            interview_done: false,
            needs_checkpoint: false,
            needs_bootstrap_join: !joined,
            awaiting_initial_configuration: false,
            restoring_commissioned_state: false,
            #[cfg(feature = "ota")]
            ota,
            #[cfg(feature = "ota")]
            ota_server: None,
            #[cfg(feature = "ota")]
            ota_cleanup_pending: false,
        }
    }

    pub async fn run(&mut self) -> ! {
        self.run_first_tick().await;

        loop {
            if platform::button_edge_pending() {
                self.handle_button_edge().await;
            }

            let now = Instant::now();
            let poll_ms = self.update_fast_poll_window(now);
            if self.device.is_joined() {
                self.device.mac_mut().radio_wake();
                self.service_joined_tick(now).await;
                let direct_rx_ms = if poll_ms == FAST_POLL_MS { poll_ms } else { 0 };
                self.service_direct_rx_window(direct_rx_ms).await;
                self.service_joined_polls().await;
                self.service_joined_post_rx();

                if !self.awaiting_initial_configuration
                    && !self.device.is_identifying(1)
                    && !self.ota_active()
                    && Instant::now() >= self.fast_poll_until
                    && self.sleep_joined_until_next_poll()
                {
                    self.handle_button_edge().await;
                }
            } else {
                self.device.mac_mut().radio_sleep();
                Timer::after(Duration::from_millis(poll_ms)).await;
                self.device.mac_mut().radio_wake();
                self.service_unjoined_cycle(Instant::now()).await;
            }
        }
    }

    fn reset_post_join_state(&mut self) {
        let now = Instant::now();
        self.fast_poll_until = now + Duration::from_secs(FAST_POLL_DURATION_SECS);
        self.last_tick = now;
        self.last_rejoin_attempt = now;
        self.annce_retries_left = 5;
        self.last_annce = now;
        self.interview_done = false;
        self.was_identifying = false;
        self.was_fast_polling = true;
        platform::led_on();
    }

    fn update_interview_state(&mut self) {
        if self.interview_done || self.device.configured_cluster_count(1) < EXPECTED_REPORT_CLUSTERS
        {
            return;
        }

        self.fast_poll_until = Instant::now() + Duration::from_secs(5);
        self.interview_done = true;
        self.awaiting_initial_configuration = false;
        platform::led_off();
    }

    fn checkpoint_security(&mut self) {
        if let Err(error) = self
            .device
            .refresh_security_state(&mut *self.security_store)
        {
            persistence_failure(error);
        }
    }

    async fn factory_reset(&mut self) {
        if let Err(StartError::PersistenceFailed(error)) = self
            .device
            .factory_reset_with_security_store(&mut *self.security_store)
            .await
        {
            persistence_failure(error);
        }
    }

    async fn secure_rejoin(&mut self) -> bool {
        match self
            .device
            .secure_rejoin_with_security_store(&mut *self.security_store)
            .await
        {
            Ok(_) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(_) => return false,
        }
        self.reset_post_join_state();
        self.needs_bootstrap_join = false;
        self.needs_checkpoint = true;
        true
    }

    async fn bootstrap_join(&mut self) -> bool {
        self.last_rejoin_attempt = Instant::now();

        let restored_state = match self.security_store.load() {
            Ok(state) => state,
            Err(error) => persistence_failure(error),
        };
        let had_commissioned_state = restored_state.is_some_and(|state| state.commissioned);
        self.awaiting_initial_configuration = !had_commissioned_state;
        self.restoring_commissioned_state = had_commissioned_state;

        match self
            .device
            .start_or_resume_with_security_store(&mut *self.security_store)
            .await
        {
            Ok(_) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(_) => return false,
        }

        self.checkpoint_security();
        let _ = self.device.send_device_annce().await;
        self.checkpoint_security();
        self.reset_post_join_state();
        if self.restoring_commissioned_state {
            self.fast_poll_until = Instant::now() + Duration::from_secs(RESTORED_FAST_POLL_SECS);
            self.restoring_commissioned_state = false;
        }
        self.needs_bootstrap_join = false;
        self.needs_checkpoint = true;
        true
    }

    async fn run_first_tick(&mut self) {
        if self.needs_bootstrap_join && !self.device.is_joined() {
            let _ = self.bootstrap_join().await;
        }

        let tick_result = {
            let device = &mut *self.device;
            let security_store = &mut *self.security_store;
            let mut clusters = application_clusters!(self);
            device
                .tick_with_security_store(0, &mut clusters, security_store)
                .await
        };
        match tick_result {
            Ok(TickResult::Event(ref event)) if update_status_led(event) => {
                self.checkpoint_security();
            }
            Ok(_) => {}
            Err(error) => persistence_failure(error),
        }

        if !self.awaiting_initial_configuration {
            setup_default_reporting(&mut *self.device);
        }
        self.sample_battery();
        self.sample_sht().await;
        self.last_report = Instant::now();

        if self.device.is_joined() {
            self.reset_post_join_state();
        }
    }

    fn update_fast_poll_window(&mut self, now: Instant) -> u64 {
        let in_fast_poll = self.awaiting_initial_configuration
            || self.device.is_identifying(1)
            || self.ota_active()
            || now < self.fast_poll_until;
        if self.was_fast_polling && !in_fast_poll {
            self.was_fast_polling = false;
            if !self.interview_done {
                platform::led_off();
            }
        } else if in_fast_poll {
            self.was_fast_polling = true;
        }

        if in_fast_poll {
            FAST_POLL_MS
        } else {
            SLOW_POLL_SECS * 1_000
        }
    }

    async fn request_join_retry(&mut self) {
        if self.device.secure_rejoin_pending() {
            self.last_rejoin_attempt = Instant::now();
            let result = {
                let device = &mut *self.device;
                let security_store = &mut *self.security_store;
                let mut clusters = application_clusters!(self);
                device
                    .tick_with_security_store(0, &mut clusters, security_store)
                    .await
            };
            if let Err(error) = result {
                persistence_failure(error);
            }
            return;
        }
        let _ = self.bootstrap_join().await;
    }

    async fn handle_button_press(&mut self) {
        let press_start = Instant::now();
        while platform::button_is_pressed() {
            if press_start.elapsed().as_secs() >= BUTTON_LONG_PRESS_SECS {
                self.factory_reset().await;
                for _ in 0..5 {
                    platform::led_on();
                    Timer::after(Duration::from_millis(100)).await;
                    platform::led_off();
                    Timer::after(Duration::from_millis(100)).await;
                }
                cortex_m::peripheral::SCB::sys_reset();
            }
            Timer::after(Duration::from_millis(50)).await;
        }

        self.sample_battery();
        self.sample_sht().await;
        self.last_report = Instant::now();
        self.fast_poll_until = Instant::now() + Duration::from_secs(BUTTON_FAST_POLL_SECS);
    }

    async fn handle_button_edge(&mut self) {
        if !platform::take_button_edge() {
            return;
        }
        Timer::after(Duration::from_millis(BUTTON_DEBOUNCE_MS)).await;
        if platform::button_is_pressed() {
            self.handle_button_press().await;
        }
    }

    async fn process_indication(&mut self, indication: &zigbee_mac::McpsDataIndication) -> bool {
        let event = {
            let device = &mut *self.device;
            let security_store = &mut *self.security_store;
            let mut clusters = application_clusters!(self);
            device
                .process_incoming_with_security_store(indication, &mut clusters, security_store)
                .await
        };
        let event = match event {
            Ok(event) => event,
            Err(error) => persistence_failure(error),
        };

        if let Some(event) = event {
            #[cfg(feature = "ota")]
            if self.process_ota_event(&event).await {
                return false;
            }
            match event {
                StackEvent::RejoinRequested => {
                    let _ = self.secure_rejoin().await;
                    return true;
                }
                StackEvent::LeaveRequested => {
                    self.factory_reset().await;
                    self.needs_bootstrap_join = true;
                    let _ = self.bootstrap_join().await;
                    self.needs_checkpoint = false;
                    return true;
                }
                _ if update_status_led(&event) => {
                    self.reset_post_join_state();
                    self.needs_checkpoint = true;
                }
                _ => {}
            }
        }

        self.update_interview_state();
        let result = {
            let device = &mut *self.device;
            let security_store = &mut *self.security_store;
            let mut clusters = application_clusters!(self);
            device
                .tick_with_security_store(0, &mut clusters, security_store)
                .await
        };
        if let Err(error) = result {
            persistence_failure(error);
        }
        false
    }

    async fn service_joined_polls(&mut self) {
        for _ in 0..4 {
            match self.device.poll().await {
                Ok(Some(indication)) => {
                    if self.process_indication(&indication).await {
                        break;
                    }
                }
                Ok(None) | Err(_) => break,
            }
        }
    }

    async fn service_direct_rx_window(&mut self, window_ms: u64) {
        let deadline = Instant::now() + Duration::from_millis(window_ms);
        loop {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            match select::select(self.device.receive(), Timer::after(deadline - now)).await {
                select::Either::First(Ok(indication)) => {
                    if self.process_indication(&indication).await {
                        return;
                    }
                }
                select::Either::First(Err(_)) | select::Either::Second(_) => break,
            }
        }
    }

    async fn sample_sht(&mut self) {
        if self.sht.start_measurement().is_err() {
            return;
        }
        Timer::after(Duration::from_millis(20)).await;
        if let Ok(measurement) = self.sht.read_measurement() {
            self.temp_cluster
                .set_temperature(measurement.temperature_centi_celsius);
            self.hum_cluster
                .set_humidity(measurement.humidity_centi_percent);
        }
    }

    fn sample_battery(&mut self) {
        let Some(monitor) = self.battery.as_mut() else {
            self.power_cluster.set_battery_voltage(0xFF);
            self.power_cluster.set_battery_percentage(0xFF);
            return;
        };

        if let Ok(reading) = monitor.read() {
            self.power_cluster
                .set_battery_voltage(reading.voltage_100mv);
            self.power_cluster
                .set_battery_percentage(reading.percentage_remaining);
        } else {
            self.power_cluster.set_battery_voltage(0xFF);
            self.power_cluster.set_battery_percentage(0xFF);
        }
    }

    async fn update_measurements(&mut self, now: Instant) {
        if now.duration_since(self.last_report).as_secs() >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            self.sample_battery();
            self.sample_sht().await;
        }
    }

    fn tick_elapsed_seconds(&mut self, now: Instant) -> u16 {
        let elapsed = now.duration_since(self.last_tick).as_secs().min(60);
        if elapsed != 0 {
            self.last_tick += Duration::from_secs(elapsed);
        }
        elapsed as u16
    }

    async fn service_joined_tick(&mut self, now: Instant) {
        self.update_measurements(now).await;
        let elapsed = self.tick_elapsed_seconds(now);
        let result = {
            let device = &mut *self.device;
            let security_store = &mut *self.security_store;
            let mut clusters = application_clusters!(self);
            device
                .tick_with_security_store(elapsed, &mut clusters, security_store)
                .await
        };
        match result {
            Ok(TickResult::Event(ref event)) if update_status_led(event) => {
                self.reset_post_join_state();
            }
            Ok(_) => {}
            Err(error) => persistence_failure(error),
        }

        #[cfg(feature = "ota")]
        self.service_ota(elapsed).await;

        if self.annce_retries_left > 0 && now.duration_since(self.last_annce).as_secs() >= 8 {
            self.annce_retries_left -= 1;
            self.last_annce = now;
            self.checkpoint_security();
            let _ = self.device.send_device_annce().await;
            self.checkpoint_security();
        }
    }

    fn service_joined_post_rx(&mut self) {
        let identifying = self.device.is_identifying(1);
        self.was_identifying = identifying;
        if identifying {
            if platform::led_is_on() {
                platform::led_off();
            } else {
                platform::led_on();
            }
        }
        if self.needs_checkpoint {
            self.needs_checkpoint = false;
            self.checkpoint_security();
        }
    }

    #[cfg(feature = "ota")]
    fn ota_active(&self) -> bool {
        matches!(
            self.ota.state(),
            OtaState::QuerySent
                | OtaState::Downloading { .. }
                | OtaState::Verifying
                | OtaState::WaitingActivate
        )
    }

    #[cfg(not(feature = "ota"))]
    const fn ota_active(&self) -> bool {
        false
    }

    #[cfg(feature = "ota")]
    async fn process_ota_event(&mut self, event: &StackEvent) -> bool {
        let StackEvent::CommandReceived {
            src_addr,
            source_endpoint,
            endpoint,
            cluster_id,
            command_id,
            payload,
            ..
        } = event
        else {
            return false;
        };
        if *cluster_id != ClusterId::OTA_UPGRADE.0 {
            return false;
        }
        if *endpoint != 1 {
            return true;
        }

        let sender = (*src_addr, *source_endpoint);
        match self.ota_server {
            Some(server) if server != sender => return true,
            None if *command_id != CMD_IMAGE_NOTIFY.0 => return true,
            None => self.ota_server = Some(sender),
            Some(_) => {}
        }
        self.fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
        let ota_event = self
            .ota
            .handle_incoming(*command_id, payload.as_slice(), None);
        self.handle_ota_status(ota_event);
        self.send_pending_ota().await;
        if self.ota.state() == OtaState::Idle {
            self.ota_server = None;
        }
        true
    }

    #[cfg(feature = "ota")]
    async fn service_ota(&mut self, elapsed_secs: u16) {
        let event = self.ota.tick(elapsed_secs);
        self.handle_ota_status(event);
        self.send_pending_ota().await;
    }

    #[cfg(feature = "ota")]
    fn handle_ota_status(&mut self, event: Option<StackEvent>) {
        match event {
            Some(StackEvent::OtaImageAvailable { .. })
            | Some(StackEvent::OtaProgress { .. })
            | Some(StackEvent::OtaDelayedActivation { .. }) => {
                self.fast_poll_until =
                    Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
            }
            Some(StackEvent::OtaFailed) => self.ota_cleanup_pending = true,
            Some(StackEvent::OtaComplete) => {
                self.checkpoint_security();
                if self.ota.activate().is_err() {
                    self.ota_cleanup_pending = true;
                }
            }
            _ => {}
        }
    }

    #[cfg(feature = "ota")]
    async fn send_pending_ota(&mut self) {
        if let Some(frame) = self.ota.take_pending_frame() {
            let Some((server, endpoint)) = self.ota_server else {
                self.ota.abort();
                self.ota_cleanup_pending = false;
                return;
            };
            if self
                .device
                .send_zcl_frame(
                    ShortAddress(server),
                    endpoint,
                    frame.endpoint,
                    frame.cluster_id,
                    frame.zcl_data.as_slice(),
                )
                .await
                .is_err()
            {
                self.ota.abort();
                self.ota_server = None;
                self.ota_cleanup_pending = false;
                return;
            }
        }
        if self.ota_cleanup_pending {
            self.ota.abort();
            self.ota_server = None;
            self.ota_cleanup_pending = false;
        }
    }

    async fn service_unjoined_cycle(&mut self, now: Instant) {
        if now.duration_since(self.last_rejoin_attempt).as_secs() >= 1 {
            platform::led_on();
            Timer::after(Duration::from_millis(80)).await;
            platform::led_off();
            Timer::after(Duration::from_millis(120)).await;
            platform::led_on();
            Timer::after(Duration::from_millis(80)).await;
            platform::led_off();
        }
        if now.duration_since(self.last_rejoin_attempt).as_secs() >= JOIN_RETRY_SECS {
            self.request_join_retry().await;
        }
    }

    #[inline(never)]
    fn sleep_joined_until_next_poll(&mut self) -> bool {
        self.device.mac_mut().radio_sleep();
        cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);

        let ticks = pm::ms_to_ticks((SLOW_POLL_SECS * 1_000) as u32, pm::LFRCO_HZ);
        let button_wake =
            match pm::sleep_for_ticks_polled_until(ticks, platform::button_edge_pending) {
                Ok(pm::InterruptibleSleep::Deadline { .. }) => false,
                Ok(pm::InterruptibleSleep::Interrupted { .. }) => true,
                Err(_) => platform::halt_with_led(),
            };

        if efr32mg1_tradfri::init_clocks().is_err() {
            platform::halt_with_led();
        }
        button_wake
    }
}

fn persistence_failure(_error: SecurityStoreError) -> ! {
    platform::halt_with_led()
}

fn update_status_led(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined { .. } => {
            platform::led_on();
            true
        }
        StackEvent::Left => {
            platform::led_off();
            false
        }
        StackEvent::LeaveRequested | StackEvent::RejoinRequested => {
            platform::led_on();
            false
        }
        _ => false,
    }
}

fn setup_default_reporting(device: &mut ZigbeeDevice<Efr32Mac>) {
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::TEMPERATURE.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
            data_type: ZclDataType::I16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::I16(50)),
        },
    );
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::HUMIDITY.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
            data_type: ZclDataType::U16,
            min_interval: 60,
            max_interval: 300,
            reportable_change: Some(ZclValue::U16(100)),
        },
    );
    let _ = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::POWER_CONFIG.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::power_config::ATTR_BATTERY_PERCENTAGE_REMAINING,
            data_type: ZclDataType::U8,
            min_interval: 300,
            max_interval: 3_600,
            reportable_change: Some(ZclValue::U8(4)),
        },
    );
}
