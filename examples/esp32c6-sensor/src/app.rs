//! ESP32-C6 Zigbee SED application state machine.

use embassy_time::{Duration, Instant, Timer};
use esp32_zigbee_devkit_product::profile::SensorProfile;
use esp32_zigbee_devkit_product::storage::SecurityStore;
use esp_hal::gpio::Input;
use esp_hal::tsens::TemperatureSensor;
use zigbee_mac::esp::EspMac;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::profile::{BatteryMeasurement, TemperatureHumidityMeasurement};
use zigbee_runtime::security_store::SecurityStoreError;

use crate::ota_client::OtaTransport;

const REPORT_INTERVAL_SECS: u64 = 60;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;
const JOIN_RETRY_SECS: u64 = 15;
const ANNCE_RETRY_SECS: u64 = 8;
const ANNCE_RETRIES: u8 = 5;
const BUTTON_LONG_PRESS_SECS: u64 = 3;

type SensorNode<'a> = ZigbeeNode<'a, EspMac<'a>, SecurityStore, SensorProfile>;

fn persistence_failure(error: SecurityStoreError) -> ! {
    esp_println::println!("[ESP32-C6] FATAL: security persistence error {:?}", error);
    loop {
        core::hint::spin_loop();
    }
}

fn node_failure(error: NodeError) -> ! {
    esp_println::println!("[ESP32-C6] FATAL: node error {:?}", error);
    loop {
        core::hint::spin_loop();
    }
}

/// Whether `event` is worth logging and treating as "something changed".
fn log_event(event: &StackEvent) -> bool {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            esp_println::println!(
                "[ESP32-C6] Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                short_address,
                channel,
                pan_id
            );
            true
        }
        StackEvent::Left => {
            esp_println::println!("[ESP32-C6] Left network");
            false
        }
        StackEvent::ReportSent => {
            esp_println::println!("[ESP32-C6] Report sent");
            false
        }
        StackEvent::LeaveRequested | StackEvent::RejoinRequested => {
            esp_println::println!("[ESP32-C6] Leave requested by coordinator");
            false
        }
        StackEvent::CommissioningComplete { success } => {
            esp_println::println!(
                "[ESP32-C6] Commissioning: {}",
                if *success { "ok" } else { "failed" }
            );
            false
        }
        _ => false,
    }
}

pub struct SensorApp<'a> {
    node: SensorNode<'a>,
    button: Input<'a>,
    temp_sensor: TemperatureSensor<'a>,
    ota: OtaTransport,
    hum_tick: u32,
    last_report: Instant,
    last_tick: Instant,
    fast_poll_until: Instant,
    last_rejoin_attempt: Instant,
    annce_retries_left: u8,
    last_annce: Instant,
    interview_done: bool,
    needs_bootstrap_join: bool,
    button_was_pressed: bool,
}

impl<'a> SensorApp<'a> {
    pub fn new(
        node: SensorNode<'a>,
        button: Input<'a>,
        temp_sensor: TemperatureSensor<'a>,
    ) -> Self {
        let now = Instant::now();
        let joined = node.device().is_joined();
        Self {
            node,
            button,
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
            annce_retries_left: if joined { ANNCE_RETRIES } else { 0 },
            last_annce: now,
            interview_done: false,
            needs_bootstrap_join: !joined,
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
        let raw_temp = self.temp_sensor.get_temperature();
        // Convert to centidegrees: (raw * 0.4386 - offset*27.88 - 20.52) * 100
        // Integer: (raw * 4386 - offset * 278800 - 205200) / 100
        let temp_centi =
            ((raw_temp.raw_value as i32) * 4386 - (raw_temp.offset as i32) * 278800 - 205200) / 100;
        self.hum_tick = self.hum_tick.wrapping_add(1);
        let humidity: u16 = 5_000 + ((self.hum_tick % 100) as u16) * 10;
        self.environment_mut()
            .update_environment(TemperatureHumidityMeasurement {
                temperature_centi_celsius: temp_centi as i16,
                humidity_centi_percent: humidity,
            });
        esp_println::println!(
            "[ESP32-C6] T={}.{:02}°C H={}.{:02}%",
            temp_centi / 100,
            (temp_centi % 100).unsigned_abs(),
            humidity / 100,
            humidity % 100
        );
    }

    /// USB-powered devkit: battery percentage is a fixed, honest "100%, no
    /// battery fitted" reading rather than a synthetic discharge curve.
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
        if restored_state.is_some_and(|state| state.commissioned) {
            esp_println::println!("[ESP32-C6] Restored state — will rejoin");
        } else {
            esp_println::println!("[ESP32-C6] No saved state — auto-joining…");
        }

        match self.node.start_or_resume().await {
            Ok(addr) => esp_println::println!("[ESP32-C6] Joined! addr=0x{:04X}", addr),
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
            Err(_) => return false,
        }

        self.checkpoint_security();
        let _ = self.node.device_mut().send_device_annce().await;
        self.checkpoint_security();
        self.reset_post_join_state();
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
        self.interview_done = false;
    }

    async fn request_join_retry(&mut self) {
        if self.node.device().secure_rejoin_pending() {
            self.last_rejoin_attempt = Instant::now();
            match self.node.secure_rejoin().await {
                Ok(_) => {
                    self.checkpoint_security();
                    self.reset_post_join_state();
                }
                Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
                Err(_) => esp_println::println!("[ESP32-C6] Secure rejoin failed"),
            }
            return;
        }
        let _ = self.bootstrap_join().await;
    }

    /// Wipe durable security/network state. Call sites are responsible for
    /// their own status message, since this is used for both a plain NWK
    /// leave (button toggle, coordinator Leave) and a hard factory reset
    /// (long button press, which also reboots).
    async fn factory_reset(&mut self) {
        if let Err(StartError::PersistenceFailed(error)) = self.node.factory_reset().await {
            persistence_failure(error);
        }
    }

    pub async fn run(&mut self) -> ! {
        self.run_first_tick().await;

        loop {
            let pressed = self.button.is_low();
            if pressed && !self.button_was_pressed {
                self.handle_button_press().await;
            }
            self.button_was_pressed = pressed;

            let now = Instant::now();
            let in_fast_poll = now < self.fast_poll_until
                || OtaTransport::is_active(self.node.profile().backend());
            let poll_ms = if in_fast_poll {
                FAST_POLL_MS
            } else {
                SLOW_POLL_SECS * 1_000
            };
            Timer::after(Duration::from_millis(poll_ms)).await;

            if self.node.device().is_joined() {
                self.service_joined_polls().await;
                self.service_joined_tick(Instant::now()).await;

                if self.annce_retries_left > 0
                    && Instant::now().duration_since(self.last_annce).as_secs() >= ANNCE_RETRY_SECS
                {
                    self.annce_retries_left -= 1;
                    self.last_annce = Instant::now();
                    let _ = self.node.device_mut().send_device_annce().await;
                }
            } else {
                self.service_unjoined_cycle(Instant::now()).await;
            }
        }
    }

    async fn run_first_tick(&mut self) {
        if self.needs_bootstrap_join && !self.node.device().is_joined() {
            let _ = self.bootstrap_join().await;
        }

        match self.node.tick(0).await {
            Ok(TickResult::Event(ref event)) if log_event(event) => self.checkpoint_security(),
            Ok(_) => {}
            Err(error) => node_failure(error),
        }

        if let Err(error) = self.node.configure_default_reporting() {
            node_failure(NodeError::Profile(error));
        }
        self.sample_battery();
        self.sample_temperature_and_humidity();
        self.last_report = Instant::now();

        if self.node.device().is_joined() {
            self.reset_post_join_state();
        }
    }

    async fn process_indication(&mut self, indication: &zigbee_mac::McpsDataIndication) -> bool {
        let event = match self.node.process_incoming(indication).await {
            Ok(event) => event,
            Err(error) => node_failure(error),
        };

        if let Some(event) = event {
            let (device, profile) = self.node.device_and_profile_mut();
            if self
                .ota
                .handle_event(device, profile.backend_mut(), &event)
                .await
            {
                // Keep the radio hot for the rest of the transfer: OTA blocks
                // arrive as indirect traffic and only a fast poll fetches
                // them at a useful rate.
                self.fast_poll_until =
                    Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                if let Err(error) = self.node.tick(0).await {
                    node_failure(error);
                }
                return false;
            }
            match event {
                StackEvent::RejoinRequested => {
                    esp_println::println!("[ESP32-C6] Secure rejoin requested");
                    match self.node.secure_rejoin().await {
                        Ok(_) => {
                            self.checkpoint_security();
                            self.reset_post_join_state();
                        }
                        Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
                        Err(_) => esp_println::println!("[ESP32-C6] Secure rejoin failed"),
                    }
                    return true;
                }
                StackEvent::LeaveRequested => {
                    esp_println::println!("[ESP32-C6] Leave requested — erasing NV and rejoining");
                    self.factory_reset().await;
                    self.needs_bootstrap_join = true;
                    let _ = self.bootstrap_join().await;
                    return true;
                }
                _ if log_event(&event) => {
                    self.checkpoint_security();
                    self.fast_poll_until =
                        Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                }
                _ => {}
            }
        }

        if !self.interview_done && self.node.reporting_is_configured() {
            self.interview_done = true;
            self.fast_poll_until = Instant::now() + Duration::from_secs(5);
            esp_println::println!("[ESP32-C6] Interview done!");
        }
        if let Err(error) = self.node.tick(0).await {
            node_failure(error);
        }
        false
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
        let elapsed_s = now.duration_since(self.last_report).as_secs();
        if elapsed_s >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            self.sample_temperature_and_humidity();
        }

        let elapsed = now.duration_since(self.last_tick).as_secs().min(60);
        self.last_tick += Duration::from_secs(elapsed);
        if let Err(error) = self.node.tick(elapsed as u16).await {
            node_failure(error);
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
            esp_println::println!("[ESP32-C6] State saved — activating new image");
            if self
                .ota
                .activate(self.node.profile_mut().backend_mut())
                .is_err()
            {
                esp_println::println!("[ESP32-C6] OTA activation failed");
            }
        }
    }

    async fn service_unjoined_cycle(&mut self, now: Instant) {
        if now.duration_since(self.last_rejoin_attempt).as_secs() >= JOIN_RETRY_SECS {
            esp_println::println!("[ESP32-C6] Retrying join…");
            self.request_join_retry().await;
        }
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
            esp_println::println!("[ESP32-C6] FACTORY RESET");
            self.factory_reset().await;
            esp_println::println!("[ESP32-C6] NV cleared — rebooting");
            for _ in 0..5u8 {
                Timer::after(Duration::from_millis(100)).await;
            }
            esp_hal::system::software_reset();
        }

        // Short press: toggle join state. Leaving does not immediately
        // rejoin — the unjoined-cycle retry (every `JOIN_RETRY_SECS`) picks
        // it back up, same as the original `UserAction::Toggle` behaviour.
        if self.node.device().is_joined() {
            esp_println::println!("[ESP32-C6] Button → leave");
            self.factory_reset().await;
        } else {
            esp_println::println!("[ESP32-C6] Button → join");
            let _ = self.bootstrap_join().await;
        }
        Timer::after(Duration::from_millis(300)).await;
    }
}
