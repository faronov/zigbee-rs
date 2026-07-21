//! PHY62x2 temperature and humidity sleepy end-device example.
//!
//! The radio and CPU use light sleep between parent polls. AON system sleep is
//! intentionally not enabled until its ROM wake and retained-runtime contract
//! has been proven on hardware.

#![no_std]
#![no_main]

#[cfg(feature = "stubs")]
mod stubs;

use cortex_m as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
use phy62x2_evk::{pins, storage, time, vectors};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::phy6222::Phy6222Mac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_runtime::synthetic_sensor::{SyntheticSensor, apply_synthetic_reading};
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::{ClusterId, DeviceId};

const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_MS: u64 = 10_000;
const FAST_POLL_DURATION_SECS: u64 = 120;
const SENSOR_UPDATE_INTERVAL_SECS: u64 = 30;
const TEST_SENSOR: SyntheticSensor = SyntheticSensor::new(2_250, 75, 5_000, 300);

fn led_on() {
    phy6222_hal::gpio::write(pins::LED_GREEN, false);
}

fn led_off() {
    phy6222_hal::gpio::write(pins::LED_GREEN, true);
}

fn failure() -> ! {
    led_on();
    loop {
        cortex_m::asm::wfi();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    time::init();
    unsafe {
        cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::LlIrq);
    }

    phy6222_hal::gpio::set_output(pins::LED_GREEN);
    phy6222_hal::gpio::set_input(pins::BUTTON);
    led_off();

    for _ in 0..3 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }

    let Some(mac) = Phy6222Mac::take() else {
        failure();
    };
    let configured_ieee = mac.extended_address();
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: SLOW_POLL_MS as u32,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("PHY62x2-Sensor")
        .date_code("20260718")
        .sw_build("0.2.0")
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            1,
            PROFILE_HOME_AUTOMATION,
            DeviceId::TEMPERATURE_SENSOR,
            |endpoint| {
            endpoint
                    .cluster_server(ClusterId::BASIC)
                    .cluster_server(ClusterId::POWER_CONFIG)
                    .cluster_server(ClusterId::IDENTIFY)
                    .cluster_server(ClusterId::TEMPERATURE)
                    .cluster_server(ClusterId::HUMIDITY)
            },
        )
        .build();

    let mut power_cluster = PowerConfigCluster::new();
    power_cluster.set_battery_size(4);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);
    let mut temp_cluster = TemperatureCluster::new(-4_000, 12_500);
    let mut hum_cluster = HumidityCluster::new(0, 10_000);

    setup_default_reporting(&mut device);
    let mut security_store = storage::security_store();
    match security_store.load() {
        Ok(Some(state))
            if state.ieee_address != [0; 8] && state.ieee_address != configured_ieee =>
        {
            if device
                .factory_reset_security_state(&mut security_store)
                .is_err()
            {
                failure();
            }
        }
        Ok(_) => {}
        Err(_) => failure(),
    }
    let mut sensor_sample = 0u32;

    'commission: loop {
        loop {
            match device
                .start_or_resume_with_security_store(&mut security_store)
                .await
            {
                Ok(_) => break,
                Err(StartError::CommissioningFailed(_)) => {
                    led_off();
                    Timer::after(Duration::from_secs(5)).await;
                }
                Err(_) => failure(),
            }
        }

        led_on();
        update_sensor_values(
            sensor_sample,
            &mut power_cluster,
            &mut temp_cluster,
            &mut hum_cluster,
        );

        let mut fast_poll_until = Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
        let mut last_sensor_update = Instant::now();
        let mut tick_anchor = Instant::now();
        let mut button_was_pressed = false;

        loop {
            let identifying = device.is_identifying(1);
            let poll_ms = if identifying || Instant::now() < fast_poll_until {
                FAST_POLL_MS
            } else {
                SLOW_POLL_MS
            };

            device.mac_mut().radio_sleep();
            Timer::after(Duration::from_millis(poll_ms)).await;
            device.mac_mut().radio_wake();

            let pressed = !phy6222_hal::gpio::read(pins::BUTTON);
            if pressed && !button_was_pressed {
                let press_start = Instant::now();
                while !phy6222_hal::gpio::read(pins::BUTTON) {
                    if press_start.elapsed().as_secs() >= 3 {
                        if device
                            .factory_reset_with_security_store(&mut security_store)
                            .await
                            .is_err()
                        {
                            failure();
                        }
                        for _ in 0..5 {
                            led_on();
                            Timer::after(Duration::from_millis(80)).await;
                            led_off();
                            Timer::after(Duration::from_millis(80)).await;
                        }
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    Timer::after(Duration::from_millis(50)).await;
                }

                if device
                    .factory_reset_with_security_store(&mut security_store)
                    .await
                    .is_err()
                {
                    failure();
                }
                led_off();
                continue 'commission;
            }
            button_was_pressed = pressed;

            for _ in 0..4 {
                let indication = match device.poll().await {
                    Ok(Some(indication)) => indication,
                    Ok(None) | Err(_) => break,
                };
                let event = {
                    let mut clusters = cluster_refs(
                        &mut power_cluster,
                        &mut temp_cluster,
                        &mut hum_cluster,
                    );
                    device
                        .process_incoming_with_security_store(
                            &indication,
                            &mut clusters,
                            &mut security_store,
                        )
                        .await
                };

                match event {
                    Ok(Some(StackEvent::RejoinRequested)) => {
                        if device
                            .secure_rejoin_with_security_store(&mut security_store)
                            .await
                            .is_err()
                        {
                            Timer::after(Duration::from_secs(5)).await;
                            continue 'commission;
                        }
                        fast_poll_until =
                            Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    }
                    Ok(Some(StackEvent::LeaveRequested | StackEvent::Left)) => {
                        if device
                            .factory_reset_with_security_store(&mut security_store)
                            .await
                            .is_err()
                        {
                            failure();
                        }
                        led_off();
                        continue 'commission;
                    }
                    Ok(Some(StackEvent::Joined { .. })) => {
                        fast_poll_until =
                            Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    }
                    Ok(_) => {}
                    Err(_) => failure(),
                }
            }

            let now = Instant::now();
            if now.duration_since(last_sensor_update).as_secs() >= SENSOR_UPDATE_INTERVAL_SECS {
                sensor_sample = sensor_sample.wrapping_add(1);
                update_sensor_values(
                    sensor_sample,
                    &mut power_cluster,
                    &mut temp_cluster,
                    &mut hum_cluster,
                );
                last_sensor_update = now;
            }

            let elapsed_secs = now
                .duration_since(tick_anchor)
                .as_secs()
                .min(u16::MAX as u64);
            if elapsed_secs > 0 {
                tick_anchor += Duration::from_secs(elapsed_secs);
                let tick_result = {
                    let mut clusters = cluster_refs(
                        &mut power_cluster,
                        &mut temp_cluster,
                        &mut hum_cluster,
                    );
                    device
                        .tick_with_security_store(
                            elapsed_secs as u16,
                            &mut clusters,
                            &mut security_store,
                        )
                        .await
                };
                match tick_result {
                    Ok(TickResult::Event(StackEvent::Left)) => {
                        led_off();
                        continue 'commission;
                    }
                    Ok(TickResult::Event(StackEvent::Joined { .. })) => {
                        fast_poll_until =
                            Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
                    }
                    Ok(_) => {}
                    Err(_) => failure(),
                }
            }

            if device.is_identifying(1) {
                phy6222_hal::gpio::write(
                    pins::LED_GREEN,
                    !phy6222_hal::gpio::read(pins::LED_GREEN),
                );
            } else {
                led_on();
            }
        }
    }
}

fn cluster_refs<'a>(
    power: &'a mut PowerConfigCluster,
    temperature: &'a mut TemperatureCluster,
    humidity: &'a mut HumidityCluster,
) -> [ClusterRef<'a>; 3] {
    [
        ClusterRef {
            endpoint: 1,
            cluster: power,
        },
        ClusterRef {
            endpoint: 1,
            cluster: temperature,
        },
        ClusterRef {
            endpoint: 1,
            cluster: humidity,
        },
    ]
}

fn update_sensor_values(
    sample: u32,
    power: &mut PowerConfigCluster,
    temperature: &mut TemperatureCluster,
    humidity: &mut HumidityCluster,
) {
    let mut clusters = cluster_refs(power, temperature, humidity);
    if apply_synthetic_reading(&mut clusters, 1, TEST_SENSOR.sample(sample)).is_err() {
        failure();
    }

    let battery_mv = phy6222_hal::adc::read_battery_mv(phy6222_hal::adc::Channel::P11);
    let battery_percent = phy6222_hal::adc::mv_to_percent(battery_mv);
    power.set_battery_voltage((battery_mv / 100) as u8);
    power.set_battery_percentage(battery_percent.saturating_mul(2));
}

fn setup_default_reporting(device: &mut ZigbeeDevice<Phy6222Mac>) {
    use zigbee_zcl::data_types::{ZclDataType, ZclValue};
    use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};

    let configs = [
        (
            ClusterId::TEMPERATURE,
            zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
            ZclDataType::I16,
            Some(ZclValue::I16(50)),
            60,
            300,
        ),
        (
            ClusterId::HUMIDITY,
            zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
            ZclDataType::U16,
            Some(ZclValue::U16(100)),
            60,
            300,
        ),
        (
            ClusterId::POWER_CONFIG,
            zigbee_zcl::clusters::power_config::ATTR_BATTERY_PERCENTAGE_REMAINING,
            ZclDataType::U8,
            Some(ZclValue::U8(4)),
            300,
            3_600,
        ),
    ];

    for (cluster_id, attribute_id, data_type, reportable_change, min, max) in configs {
        if device
            .reporting_mut()
            .configure_for_cluster(
                1,
                cluster_id.0,
                ReportingConfig {
                    direction: ReportDirection::Send,
                    attribute_id,
                    data_type,
                    min_interval: min,
                    max_interval: max,
                    reportable_change,
                },
            )
            .is_err()
        {
            failure();
        }
    }
}
