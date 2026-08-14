//! ESP32-H2 Zigbee SED application state machine.

use embassy_time::{Duration, Instant, Timer};
use esp32_zigbee_devkit_product::ota_transport::OtaTransport;
use esp32_zigbee_devkit_product::profile::SensorProfile;
use esp32_zigbee_devkit_product::storage::SecurityStore;
use esp_hal::gpio::{Input, Output};
use zigbee_mac::esp::EspMac;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::profile::{BatteryMeasurement, TemperatureHumidityMeasurement};
use zigbee_runtime::security_store::SecurityStoreError;

use crate::chip_temperature::H2TemperatureSensor;

const REPORT_INTERVAL_SECS: u64 = 60;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;
const RESTORED_FAST_POLL_SECS: u64 = 60;
const INTERVIEW_CONFIGURATION_TIMEOUT_SECS: u64 = 120;
const JOIN_RETRY_SECS: u64 = 15;
const ANNCE_RETRY_SECS: u64 = 8;
const ANNCE_RETRIES: u8 = 5;
const BUTTON_LONG_PRESS_SECS: u64 = 3;
const SECURE_REJOIN_FAILURE_LIMIT: u8 = 4;

type SensorNode<'a> = ZigbeeNode<'a, EspMac<'a>, SecurityStore, SensorProfile>;

fn persistence_failure(error: SecurityStoreError) -> ! {
    esp_println::println!("[ESP32-H2] FATAL: security persistence error {:?}", error);
    loop {
        core::hint::spin_loop();
    }
}

fn node_failure(error: NodeError) -> ! {
    esp_println::println!("[ESP32-H2] FATAL: node error {:?}", error);
    loop {
        core::hint::spin_loop();
    }
}

fn start_failure(error: StartError) -> ! {
    esp_println::println!("[ESP32-H2] FATAL: lifecycle error {:?}", error);
    loop {
        core::hint::spin_loop();
    }
}

pub struct SensorApp<'a> {
    node: SensorNode<'a>,
    button: Input<'a>,
    led: Output<'a>,
    temp_sensor: H2TemperatureSensor,
    ota: OtaTransport,
    hum_tick: u32,
    last_report: Instant,
    last_tick: Instant,
    fast_poll_until: Instant,
    last_rejoin_attempt: Instant,
    annce_retries_left: u8,
    last_annce: Instant,
    was_identifying: bool,
    interview_done: bool,
    interview_deadline: Option<Instant>,
    needs_bootstrap_join: bool,
    awaiting_initial_configuration: bool,
    consecutive_rejoin_failures: u8,
    button_was_pressed: bool,
}

impl<'a> SensorApp<'a> {
    pub fn new(
        node: SensorNode<'a>,
        button: Input<'a>,
        led: Output<'a>,
        temp_sensor: H2TemperatureSensor,
    ) -> Self {
        let now = Instant::now();
        let joined = node.device().is_joined();
        Self {
            node,
            button,
            led,
            temp_sensor,
            ota: OtaTransport::new(),
            hum_tick: 0,
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
            was_identifying: false,
            interview_done: false,
            interview_deadline: None,
            needs_bootstrap_join: !joined,
            awaiting_initial_configuration: false,
            consecutive_rejoin_failures: 0,
            button_was_pressed: false,
        }
    }

    fn environment_mut(&mut self) -> &mut zigbee_runtime::profile::TemperatureHumidityBattery {
        self.node.profile_mut().inner_mut().component_mut()
    }

    fn checkpoint_security(&mut self) {
        if let Err(error) = self.node.checkpoint_security() {
            persistence_failure(error);
        }
    }

    fn sample_temperature_and_humidity(&mut self) {
        let temp_centi = self.temp_sensor.read_centi_celsius();
        self.hum_tick = self.hum_tick.wrapping_add(1);
        let humidity: u16 = 5_000 + ((self.hum_tick % 100) as u16) * 10;
        self.environment_mut()
            .update_environment(TemperatureHumidityMeasurement {
                temperature_centi_celsius: temp_centi,
                humidity_centi_percent: humidity,
            });
        esp_println::println!(
            "[ESP32-H2] T={}.{:02}°C H={}.{:02}%",
            temp_centi / 100,
            (temp_centi % 100).unsigned_abs(),
            humidity / 100,
            humidity % 100
        );
    }

    /// No battery ADC on this devkit: a fixed, honest "100%, USB powered"
    /// reading rather than a synthetic discharge curve.
    fn sample_battery(&mut self) {
        self.environment_mut().update_battery(BatteryMeasurement {
            voltage_100mv: 33,
            percentage_remaining: 200,
        });
    }

    async fn bootstrap_join(&mut self) -> bool {
        self.last_rejoin_attempt = Instant::now();

        let restored_state = match self.node.load_security_state() {
            Ok(state) => state,
            Err(error) => persistence_failure(error),
        };
        let had_commissioned_state = restored_state.is_some_and(|state| state.commissioned);
        if had_commissioned_state {
            esp_println::println!("[ESP32-H2] Restored state — will rejoin");
        } else {
            esp_println::println!("[ESP32-H2] No saved state — auto-joining…");
        }

        match self.node.start_or_resume().await {
            Ok(addr) => {
                esp_println::println!("[ESP32-H2] Joined! addr=0x{:04X}", addr);
                self.led.set_low(); // ON
            }
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(_) => return false,
        }

        let now = Instant::now();
        self.awaiting_initial_configuration = !had_commissioned_state;
        self.interview_deadline = if had_commissioned_state {
            None
        } else {
            Some(now + Duration::from_secs(INTERVIEW_CONFIGURATION_TIMEOUT_SECS))
        };
        self.checkpoint_security();
        if !had_commissioned_state {
            let _ = self.node.device_mut().send_device_annce().await;
            self.checkpoint_security();
        }
        self.reset_post_join_state();
        if had_commissioned_state {
            self.annce_retries_left = 0;
            self.fast_poll_until = Instant::now() + Duration::from_secs(RESTORED_FAST_POLL_SECS);
        }
        self.needs_bootstrap_join = false;
        true
    }

    fn reset_post_join_state(&mut self) {
        let now = Instant::now();
        self.fast_poll_until = now + Duration::from_secs(FAST_POLL_DURATION_SECS);
        self.last_tick = now;
        self.last_rejoin_attempt = now;
        self.annce_retries_left = ANNCE_RETRIES;
        self.last_annce = now;
        self.was_identifying = false;
        self.interview_done = false;
        // New network lifecycle — the coordinator re-runs its interview.
        self.node.reset_remote_reporting();
        self.consecutive_rejoin_failures = 0;
        self.led.set_low();
    }

    /// Leave the post-join fast-poll window once a *remote* client finished
    /// configuring reporting.
    ///
    /// Keyed on `zigbee-runtime`'s remote-reporting record rather than the
    /// reporting engine, which also holds this product's own defaults
    /// (including the interview-timeout fallback below) and therefore cannot
    /// tell a completed coordinator interview from self-configuration.
    fn update_interview_state(&mut self) {
        if self.interview_done || !self.node.remote_reporting_is_complete() {
            return;
        }

        self.fast_poll_until = Instant::now() + Duration::from_secs(5);
        self.interview_done = true;
        self.awaiting_initial_configuration = false;
        self.interview_deadline = None;
        self.led.set_high();
        esp_println::println!(
            "[ESP32-H2] Interview configured by remote client ({}/{} clusters)",
            self.node.remote_reporting_cluster_count(),
            self.node.expected_report_clusters()
        );
    }

    fn update_fast_poll_window(&mut self, now: Instant) -> u64 {
        if self.awaiting_initial_configuration
            && self
                .interview_deadline
                .is_some_and(|deadline| now >= deadline)
        {
            if let Err(error) = self.node.configure_default_reporting() {
                node_failure(NodeError::Profile(error));
            }
            // Fallback only — local defaults, not a completed interview. The
            // remote count is logged so the two are never confused.
            self.interview_done = true;
            self.awaiting_initial_configuration = false;
            self.interview_deadline = None;
            self.led.set_high();
            esp_println::println!(
                "[ESP32-H2] Interview timeout — using default reporting (remote configured {}/{} clusters)",
                self.node.remote_reporting_cluster_count(),
                self.node.expected_report_clusters()
            );
        }

        let in_fast_poll = self.awaiting_initial_configuration
            || self
                .node
                .device()
                .is_identifying(esp32_zigbee_devkit_product::ENDPOINT)
            || OtaTransport::is_active(self.node.profile().backend())
            || now < self.fast_poll_until;
        if in_fast_poll {
            FAST_POLL_MS
        } else {
            SLOW_POLL_SECS * 1_000
        }
    }

    async fn request_join_retry(&mut self) {
        if self.node.device().secure_rejoin_pending() {
            self.last_rejoin_attempt = Instant::now();
            let _ = self.secure_rejoin().await;
            return;
        }
        let _ = self.bootstrap_join().await;
    }

    /// Wipe durable security/network state. Call sites are responsible for
    /// their own status message, since this is used for both a plain NWK
    /// leave (button toggle, coordinator Leave) and a hard factory reset
    /// (long button press, which also reboots).
    async fn factory_reset(&mut self) {
        match self.node.factory_reset().await {
            Ok(()) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(error) => start_failure(error),
        }
    }

    async fn secure_rejoin(&mut self) -> bool {
        match self.node.secure_rejoin().await {
            Ok(_) => {}
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(_) => {
                esp_println::println!("[ESP32-H2] Secure rejoin failed");
                self.record_failed_rejoin().await;
                return false;
            }
        }

        self.checkpoint_security();
        self.reset_post_join_state();
        self.needs_bootstrap_join = false;
        esp_println::println!(
            "[ESP32-H2] Secure rejoin succeeded addr=0x{:04X}",
            self.node.device().short_address()
        );
        true
    }

    async fn record_failed_rejoin(&mut self) {
        self.consecutive_rejoin_failures = self.consecutive_rejoin_failures.saturating_add(1);
        if self.consecutive_rejoin_failures < SECURE_REJOIN_FAILURE_LIMIT {
            return;
        }

        esp_println::println!(
            "[ESP32-H2] Stale network — resetting after repeated rejoin failures"
        );
        self.factory_reset().await;
        self.consecutive_rejoin_failures = 0;
        self.needs_bootstrap_join = true;
        self.awaiting_initial_configuration = false;
        self.interview_deadline = None;
    }

    pub async fn run(&mut self) -> ! {
        self.run_first_tick().await;

        loop {
            let pressed = self.button.is_low();
            if pressed && !self.button_was_pressed {
                self.handle_button_press().await;
            }
            self.button_was_pressed = pressed;

            let poll_ms = self.update_fast_poll_window(Instant::now());
            Timer::after(Duration::from_millis(poll_ms)).await;

            if self.node.device().is_joined() {
                self.service_joined_polls().await;
                if !self.node.device().is_joined() {
                    continue;
                }
                self.service_joined_tick(Instant::now()).await;
                if !self.node.device().is_joined() {
                    continue;
                }
                self.service_status_led();
            } else {
                self.service_unjoined_cycle(Instant::now()).await;
            }
        }
    }

    async fn run_first_tick(&mut self) {
        let bootstrapped = self.needs_bootstrap_join
            && !self.node.device().is_joined()
            && self.bootstrap_join().await;

        match self.node.tick(0).await {
            Ok(result) => {
                self.handle_tick_result(result).await;
            }
            Err(error) => node_failure(error),
        }

        if self.node.device().is_joined() && !self.awaiting_initial_configuration {
            if let Err(error) = self.node.configure_default_reporting() {
                node_failure(NodeError::Profile(error));
            }
        }
        self.sample_battery();
        self.sample_temperature_and_humidity();
        self.last_report = Instant::now();

        if self.node.device().is_joined() && !bootstrapped {
            self.reset_post_join_state();
        }
    }

    async fn process_indication(&mut self, indication: &zigbee_mac::McpsDataIndication) -> bool {
        let event = match self.node.process_incoming(indication).await {
            Ok(event) => event,
            Err(error) => node_failure(error),
        };

        if let Some(event) = event {
            if self.process_ota_event(&event).await {
                match self.node.tick(0).await {
                    Ok(result) => {
                        self.handle_tick_result(result).await;
                    }
                    Err(error) => node_failure(error),
                }
                return false;
            }
            if self.handle_control_event(&event).await {
                return true;
            }
        }

        self.update_interview_state();
        if !self.node.device().is_joined() {
            return true;
        }
        match self.node.tick(0).await {
            Ok(result) => {
                if self.handle_tick_result(result).await {
                    return true;
                }
            }
            Err(error) => node_failure(error),
        }
        false
    }

    async fn process_ota_event(&mut self, event: &StackEvent) -> bool {
        let (device, profile) = self.node.device_and_profile_mut();
        if !self
            .ota
            .handle_event(device, profile.backend_mut(), event)
            .await
        {
            return false;
        }

        self.fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
        true
    }

    async fn handle_tick_result(&mut self, result: TickResult) -> bool {
        let TickResult::Event(event) = result else {
            return false;
        };
        if self.process_ota_event(&event).await {
            return false;
        }
        self.handle_control_event(&event).await
    }

    async fn handle_control_event(&mut self, event: &StackEvent) -> bool {
        match event {
            StackEvent::Joined {
                short_address,
                channel,
                pan_id,
            } => {
                self.reset_post_join_state();
                self.needs_bootstrap_join = false;
                self.checkpoint_security();
                esp_println::println!(
                    "[ESP32-H2] Joined addr=0x{:04X} ch={} pan=0x{:04X}",
                    short_address,
                    channel,
                    pan_id
                );
                false
            }
            StackEvent::CommissioningComplete { success: true } => {
                self.consecutive_rejoin_failures = 0;
                self.checkpoint_security();
                esp_println::println!("[ESP32-H2] Commissioning complete");
                false
            }
            StackEvent::CommissioningComplete { success: false } => {
                esp_println::println!("[ESP32-H2] Commissioning failed");
                if self.node.device().secure_rejoin_pending() {
                    self.record_failed_rejoin().await;
                } else {
                    self.needs_bootstrap_join = true;
                    self.awaiting_initial_configuration = false;
                    self.interview_deadline = None;
                }
                true
            }
            StackEvent::RejoinRequested => {
                esp_println::println!("[ESP32-H2] Secure rejoin requested");
                let _ = self.secure_rejoin().await;
                true
            }
            StackEvent::BasicResetToFactoryDefaults => {
                esp_println::println!(
                    "[ESP32-H2] Basic cluster attributes reset to factory defaults"
                );
                false
            }
            StackEvent::LeaveRequested | StackEvent::Left => {
                esp_println::println!("[ESP32-H2] Leaving network and clearing credentials");
                self.factory_reset().await;
                self.needs_bootstrap_join = true;
                self.awaiting_initial_configuration = false;
                self.interview_deadline = None;
                self.led.set_high();
                true
            }
            StackEvent::ReportSent => {
                esp_println::println!("[ESP32-H2] Report sent");
                false
            }
            _ => false,
        }
    }

    async fn service_joined_polls(&mut self) {
        for _ in 0..4u8 {
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

    async fn service_joined_tick(&mut self, now: Instant) {
        let elapsed_s = now.saturating_duration_since(self.last_report).as_secs();
        if elapsed_s >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            self.sample_temperature_and_humidity();
        }

        let elapsed = now
            .saturating_duration_since(self.last_tick)
            .as_secs()
            .min(60);
        if elapsed != 0 {
            self.last_tick += Duration::from_secs(elapsed);
        }
        match self.node.tick(elapsed as u16).await {
            Ok(result) => {
                if self.handle_tick_result(result).await {
                    return;
                }
            }
            Err(error) => node_failure(error),
        }

        if !self.node.device().is_joined() {
            return;
        }

        // Drive the OTA state machine and flush any queued request.
        let (device, profile) = self.node.device_and_profile_mut();
        self.ota
            .service(device, profile.backend_mut(), elapsed as u16)
            .await;
        if self.ota.activation_pending() {
            // Checkpoint first: activate() reboots into the staged image and
            // anything not in NV by then is lost.
            self.checkpoint_security();
            esp_println::println!("[ESP32-H2] State saved — activating new image");
            if self
                .ota
                .activate(self.node.profile_mut().backend_mut())
                .is_err()
            {
                esp_println::println!("[ESP32-H2] OTA activation failed");
            }
        }

        let annce_now = Instant::now();
        if self.annce_retries_left > 0
            && annce_now
                .saturating_duration_since(self.last_annce)
                .as_secs()
                >= ANNCE_RETRY_SECS
        {
            self.annce_retries_left -= 1;
            self.last_annce = annce_now;
            self.checkpoint_security();
            let _ = self.node.device_mut().send_device_annce().await;
            self.checkpoint_security();
        }
    }

    async fn service_unjoined_cycle(&mut self, now: Instant) {
        if now
            .saturating_duration_since(self.last_rejoin_attempt)
            .as_secs()
            >= 1
        {
            self.led.set_low();
            Timer::after(Duration::from_millis(80)).await;
            self.led.set_high();
            Timer::after(Duration::from_millis(120)).await;
            self.led.set_low();
            Timer::after(Duration::from_millis(80)).await;
            self.led.set_high();
        }
        if now
            .saturating_duration_since(self.last_rejoin_attempt)
            .as_secs()
            >= JOIN_RETRY_SECS
        {
            esp_println::println!("[ESP32-H2] Retrying join…");
            self.request_join_retry().await;
        }
    }

    fn service_status_led(&mut self) {
        let identifying = self
            .node
            .device()
            .is_identifying(esp32_zigbee_devkit_product::ENDPOINT);
        if identifying {
            self.led.toggle();
        } else if self.was_identifying {
            self.led.set_high();
        }
        self.was_identifying = identifying;
    }

    async fn handle_button_press(&mut self) {
        let press_start = Instant::now();
        let mut held_long = false;
        while self.button.is_low() {
            if press_start.elapsed().as_secs() >= BUTTON_LONG_PRESS_SECS {
                held_long = true;
                break;
            }
            Timer::after(Duration::from_millis(50)).await;
        }

        if held_long {
            esp_println::println!("[ESP32-H2] FACTORY RESET");
            self.factory_reset().await;
            for _ in 0..5u8 {
                self.led.set_low();
                Timer::after(Duration::from_millis(100)).await;
                self.led.set_high();
                Timer::after(Duration::from_millis(100)).await;
            }
            esp_hal::system::software_reset();
        }

        // Short press: toggle join state. Leaving does not immediately
        // rejoin — the unjoined-cycle retry (every `JOIN_RETRY_SECS`) picks
        // it back up, same as the original `UserAction::Toggle` behaviour.
        if self.node.device().is_joined() {
            esp_println::println!("[ESP32-H2] Button → leave");
            self.factory_reset().await;
        } else {
            esp_println::println!("[ESP32-H2] Button → join");
            let _ = self.bootstrap_join().await;
        }
        Timer::after(Duration::from_millis(300)).await;
    }
}
