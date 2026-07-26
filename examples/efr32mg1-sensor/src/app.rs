//! Production Zigbee SED state machine.

use efr32mg1_hal::pm;
use efr32mg1_tradfri_product::battery::BatteryMonitor;
use efr32mg1_tradfri_product::ota::Efr32FirmwareWriter;
use efr32mg1_tradfri_product::profile::SensorProfile;
use efr32mg1_tradfri_product::storage::SecurityStore;
use embassy_futures::select;
use embassy_time::{Duration, Instant, Timer};
use zigbee_mac::efr32::Efr32Mac;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::ota::OtaManager;
use zigbee_runtime::ota_transport::{OtaEventOutcome, OtaSession};
use zigbee_runtime::profile::{
    BatteryMeasurement, TemperatureHumidityBattery, TemperatureHumidityMeasurement,
};
use zigbee_runtime::security_store::SecurityStoreError;

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

type SensorNode = ZigbeeNode<'static, Efr32Mac, SecurityStore, SensorProfile>;

pub struct SensorApp {
    node: SensorNode,
    sht: sensor::Sensor,
    battery: Option<BatteryMonitor>,
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
    ota_session: OtaSession,
}

impl SensorApp {
    pub fn new(node: SensorNode, sht: sensor::Sensor, battery: Option<BatteryMonitor>) -> Self {
        let now = Instant::now();
        let joined = node.device().is_joined();
        Self {
            node,
            sht,
            battery,
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
            ota_session: OtaSession::new(),
        }
    }

    fn environment_mut(&mut self) -> &mut TemperatureHumidityBattery {
        self.node.profile_mut().inner_mut().component_mut()
    }

    fn ota(&self) -> &OtaManager<Efr32FirmwareWriter> {
        self.node.profile().ota()
    }

    pub async fn run(&mut self) -> ! {
        self.run_first_tick().await;

        loop {
            if platform::button_edge_pending() {
                self.handle_button_edge().await;
            }

            let now = Instant::now();
            let poll_ms = self.update_fast_poll_window(now);
            if self.node.device().is_joined() {
                self.node.device_mut().mac_mut().radio_wake();
                self.service_joined_tick(now).await;
                let direct_rx_ms = if poll_ms == FAST_POLL_MS { poll_ms } else { 0 };
                self.service_direct_rx_window(direct_rx_ms).await;
                self.service_joined_polls().await;
                self.service_joined_post_rx();

                if !self.awaiting_initial_configuration
                    && !self
                        .node
                        .device()
                        .is_identifying(efr32mg1_tradfri_product::ENDPOINT)
                    && !self.ota_active()
                    && Instant::now() >= self.fast_poll_until
                    && self.sleep_joined_until_next_poll()
                {
                    self.handle_button_edge().await;
                }
            } else {
                self.node.device_mut().mac_mut().radio_sleep();
                Timer::after(Duration::from_millis(poll_ms)).await;
                self.node.device_mut().mac_mut().radio_wake();
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
        if self.interview_done || !self.node.reporting_is_configured() {
            return;
        }

        self.fast_poll_until = Instant::now() + Duration::from_secs(5);
        self.interview_done = true;
        self.awaiting_initial_configuration = false;
        platform::led_off();
    }

    fn checkpoint_security(&mut self) {
        if let Err(error) = self.node.checkpoint_security() {
            persistence_failure(error);
        }
    }

    async fn factory_reset(&mut self) {
        if let Err(StartError::PersistenceFailed(error)) = self.node.factory_reset().await {
            persistence_failure(error);
        }
    }

    async fn secure_rejoin(&mut self) -> bool {
        match self.node.secure_rejoin().await {
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

        let restored_state = match self.node.load_security_state() {
            Ok(state) => state,
            Err(error) => persistence_failure(error),
        };
        let had_commissioned_state = restored_state.is_some_and(|state| state.commissioned);
        self.awaiting_initial_configuration = !had_commissioned_state;
        self.restoring_commissioned_state = had_commissioned_state;

        match self.node.start_or_resume().await {
            Ok(_) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(_) => return false,
        }

        self.checkpoint_security();
        let _ = self.node.device_mut().send_device_annce().await;
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
        if self.needs_bootstrap_join && !self.node.device().is_joined() {
            let _ = self.bootstrap_join().await;
        }

        let tick_result = self.node.tick(0).await;
        match tick_result {
            Ok(TickResult::Event(ref event)) if update_status_led(event) => {
                self.checkpoint_security();
            }
            Ok(_) => {}
            Err(error) => node_failure(error),
        }

        if !self.awaiting_initial_configuration
            && let Err(error) = self.node.configure_default_reporting()
        {
            node_failure(NodeError::Profile(error));
        }
        self.sample_battery();
        self.sample_sht().await;
        self.last_report = Instant::now();

        if self.node.device().is_joined() {
            self.reset_post_join_state();
        }
    }

    fn update_fast_poll_window(&mut self, now: Instant) -> u64 {
        let in_fast_poll = self.awaiting_initial_configuration
            || self
                .node
                .device()
                .is_identifying(efr32mg1_tradfri_product::ENDPOINT)
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
        if self.node.device().secure_rejoin_pending() {
            self.last_rejoin_attempt = Instant::now();
            let result = self.node.tick(0).await;
            if let Err(error) = result {
                node_failure(error);
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
        let event = self.node.process_incoming(indication).await;
        let event = match event {
            Ok(event) => event,
            Err(error) => node_failure(error),
        };

        if let Some(event) = event {
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
        let result = self.node.tick(0).await;
        if let Err(error) = result {
            node_failure(error);
        }
        false
    }

    async fn service_joined_polls(&mut self) {
        for _ in 0..4 {
            match self.node.device_mut().poll().await {
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
            match select::select(
                self.node.device_mut().receive(),
                Timer::after(deadline - now),
            )
            .await
            {
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
        if let Some(measurement) = self.sht.sample().await {
            self.environment_mut()
                .update_environment(TemperatureHumidityMeasurement {
                    temperature_centi_celsius: measurement.temperature_centi_celsius,
                    humidity_centi_percent: measurement.humidity_centi_percent,
                });
        }
    }

    fn sample_battery(&mut self) {
        let Some(monitor) = self.battery.as_mut() else {
            self.environment_mut().set_battery_unknown();
            return;
        };

        if let Ok(reading) = monitor.read() {
            self.environment_mut().update_battery(BatteryMeasurement {
                voltage_100mv: reading.voltage_100mv,
                percentage_remaining: reading.percentage_remaining,
            });
        } else {
            self.environment_mut().set_battery_unknown();
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
        let result = self.node.tick(elapsed).await;
        match result {
            Ok(TickResult::Event(ref event)) if update_status_led(event) => {
                self.reset_post_join_state();
            }
            Ok(_) => {}
            Err(error) => node_failure(error),
        }

        self.service_ota(elapsed).await;

        if self.annce_retries_left > 0 && now.duration_since(self.last_annce).as_secs() >= 8 {
            self.annce_retries_left -= 1;
            self.last_annce = now;
            self.checkpoint_security();
            let _ = self.node.device_mut().send_device_annce().await;
            self.checkpoint_security();
        }
    }

    fn service_joined_post_rx(&mut self) {
        let identifying = self
            .node
            .device()
            .is_identifying(efr32mg1_tradfri_product::ENDPOINT);
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

    fn ota_active(&self) -> bool {
        OtaSession::is_active(Some(self.ota()))
    }

    /// Route OTA Upgrade cluster traffic through the shared transport.
    ///
    /// Session bookkeeping (which server owns the transfer, sending and
    /// retrying the manager's queued request, resetting once idle) lives in
    /// [`zigbee_runtime::ota_transport::OtaSession`] — identical to the
    /// ESP32-C6 example. What stays here is app policy: extending the
    /// fast-poll window, and checkpointing security state before activating
    /// a verified image so the reset into the new firmware cannot lose the
    /// network keys.
    async fn process_ota_event(&mut self, event: &StackEvent) -> bool {
        let (device, profile) = self.node.device_and_profile_mut();
        let outcome = self
            .ota_session
            .handle_event(
                device,
                Some(profile.ota_mut()),
                efr32mg1_tradfri_product::ENDPOINT,
                event,
            )
            .await;
        match outcome {
            OtaEventOutcome::NotOta => false,
            // Traffic addressed to the OTA cluster but declined by the
            // session (wrong server, wrong endpoint) — still handled, but
            // not worth waking up the radio for.
            OtaEventOutcome::Ignored => true,
            OtaEventOutcome::Consumed(status) => {
                self.fast_poll_until =
                    Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                self.react_to_ota_status(&status);
                true
            }
        }
    }

    async fn service_ota(&mut self, elapsed_secs: u16) {
        let (device, profile) = self.node.device_and_profile_mut();
        let status = self
            .ota_session
            .service(device, Some(profile.ota_mut()), elapsed_secs)
            .await;
        self.react_to_ota_status(&status);
    }

    /// App-level reaction to an OTA status: extend the fast-poll window
    /// while a transfer is progressing, and checkpoint security state
    /// before activating a verified image (a failed activation is left for
    /// [`zigbee_runtime::ota_transport::OtaSession`] to clean up).
    fn react_to_ota_status(&mut self, status: &Option<StackEvent>) {
        if matches!(
            status,
            Some(StackEvent::OtaImageAvailable { .. })
                | Some(StackEvent::OtaProgress { .. })
                | Some(StackEvent::OtaDelayedActivation { .. })
        ) {
            self.fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
        }
        if matches!(status, Some(StackEvent::OtaComplete)) {
            self.checkpoint_security();
            let _ = self
                .ota_session
                .activate(Some(self.node.profile_mut().ota_mut()));
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
        self.node.device_mut().mac_mut().radio_sleep();
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

fn node_failure(_error: NodeError) -> ! {
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
