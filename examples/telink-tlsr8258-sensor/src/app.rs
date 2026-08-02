use core::mem::MaybeUninit;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::telink::TelinkMac;
use zigbee_runtime::event_loop::{StackEvent, StartError};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::synthetic_sensor::{SyntheticSensor, apply_synthetic_reading};
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::basic::PowerSource;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;
use zigbee_zcl::data_types::{ZclDataType, ZclValue};
use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
use zigbee_zcl::{ClusterId, DeviceId};

use tlsr8258_tb04::{leds::StatusLeds, resources::BoardResources};

// Preserve the IEEE address used by the hardware-proven runtime image so the
// existing journal and ZHA device identity remain valid across this refactor.
const DEVICE_EUI_OFFSET: u8 = 0x33;
const SENSOR_UPDATE_INTERVAL_SECS: u16 = 30;
const TEST_SENSOR: SyntheticSensor = SyntheticSensor::new(2_150, 100, 5_000, 400);

fn setup_test_reporting(device: &mut ZigbeeDevice<TelinkMac>) -> bool {
    let temperature = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::TEMPERATURE.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::temperature::ATTR_MEASURED_VALUE,
            data_type: ZclDataType::I16,
            min_interval: 1,
            max_interval: 60,
            reportable_change: Some(ZclValue::I16(1)),
        },
    );
    let humidity = device.reporting_mut().configure_for_cluster(
        1,
        ClusterId::HUMIDITY.0,
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: zigbee_zcl::clusters::humidity::ATTR_MEASURED_VALUE,
            data_type: ZclDataType::U16,
            min_interval: 1,
            max_interval: 60,
            reportable_change: Some(ZclValue::U16(1)),
        },
    );
    temperature.is_ok() && humidity.is_ok()
}

fn failure(leds: &StatusLeds) -> ! {
    leds.green.write(false);
    leds.blue.write(false);
    leds.red.write(true);
    loop {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
    }
}

pub fn run() -> ! {
    type Device = ZigbeeDevice<TelinkMac>;

    tlsr8258_hal::timer::init();
    let resources = match BoardResources::take() {
        Some(resources) => resources,
        None => loop {
            core::hint::spin_loop();
        },
    };
    let leds = resources.lighting.into_status_leds();
    if leds.init().is_err() {
        failure(&leds);
    }
    let adc = match tlsr8258_hal::adc::Adc::new(
        resources.adc,
        tlsr8258_hal::flash::FlashGeometry::KiB512,
    ) {
        Ok(adc) => adc,
        Err(_) => failure(&leds),
    };
    if adc.install_flash_voltage_guard(resources.adc_pc5).is_err() {
        failure(&leds);
    }

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(DEVICE_EUI_OFFSET);
    let mac = TelinkMac::with_extended_address(ieee_address);
    // Hand the exclusive AES token to the MAC so CCM*/MMO run on the
    // hardware accelerator. Only compiled under `hardware-aes`; the
    // standard image leaves `resources.aes` unused and uses software AES.
    #[cfg(feature = "hardware-aes")]
    let mac = {
        let mut mac = mac;
        if mac.install_aes_engine(resources.aes).is_err() {
            failure(&leds);
        }
        mac
    };

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    static mut TEMP_STORAGE: MaybeUninit<TemperatureCluster> = MaybeUninit::uninit();
    static mut HUM_STORAGE: MaybeUninit<HumidityCluster> = MaybeUninit::uninit();
    static mut POWER_STORAGE: MaybeUninit<PowerConfigCluster> = MaybeUninit::uninit();

    let power_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(POWER_STORAGE).cast::<PowerConfigCluster>();
        ptr.write(PowerConfigCluster::new());
        &mut *ptr
    };
    power_cluster.set_battery_voltage(30);
    power_cluster.set_battery_percentage(200);
    power_cluster.set_battery_size(0x04);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);

    let temp_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(TEMP_STORAGE).cast::<TemperatureCluster>();
        ptr.write(TemperatureCluster::new(-4_000, 12_500));
        &mut *ptr
    };
    temp_cluster.set_temperature(2_150);
    let hum_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(HUM_STORAGE).cast::<HumidityCluster>();
        ptr.write(HumidityCluster::new(0, 10_000));
        &mut *ptr
    };
    hum_cluster.set_humidity(5_000);

    let device = ZigbeeDevice::builder(mac)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Runtime")
        .date_code("20260718")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask(1 << 15))
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
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });
    if !setup_test_reporting(device) {
        failure(&leds);
    }

    let mut clusters = [
        ClusterRef {
            endpoint: 1,
            cluster: power_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: temp_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: hum_cluster,
        },
    ];
    let mut security_store = tlsr8258_tb04_product::storage::security_store(resources.flash);
    if device
        .reset_security_state_if_identity_changed(&mut security_store, ieee_address)
        .is_err()
    {
        failure(&leds);
    }
    let mut sensor_sample = 0u32;
    let mut sensor_update_elapsed = 0u16;

    // Single root future for the whole firmware: all start/resume, receive
    // windows, `process_incoming`, tick/persistence refresh, commissioning
    // retries, and factory-reset/rejoin loops run inside this one future so
    // `tlsr8258_rt::block_on` is monomorphized exactly once. Synchronous chip,
    // ADC, LED, MAC, device, and security-store initialization above stays
    // outside the future. This never returns (`Output = !`).
    let app = async move {
        'commission: loop {
            let mut attempts = 0u8;
            loop {
                attempts = attempts.saturating_add(1);
                match device
                    .start_or_resume_with_security_store(&mut security_store)
                    .await
                {
                    Ok(_) => break,
                    Err(StartError::CommissioningFailed(_)) if attempts < 10 => {
                        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(5_000));
                    }
                    Err(_) => failure(&leds),
                }
            }

            leds.red.write(false);
            leds.green.write(true);
            leds.blue.write(false);
            if apply_synthetic_reading(&mut clusters, 1, TEST_SENSOR.sample(sensor_sample)).is_err()
            {
                failure(&leds);
            }

            let one_second = tlsr8258_hal::timer::ms(1_000);
            let mut tick_anchor = tlsr8258_hal::timer::now_ticks();
            loop {
                for _ in 0..4u8 {
                    match device.poll().await {
                        Ok(Some(indication)) => {
                            let event = device
                                .process_incoming_with_security_store(
                                    &indication,
                                    &mut clusters,
                                    &mut security_store,
                                )
                                .await;
                            match event {
                                Ok(Some(StackEvent::RejoinRequested)) => {
                                    let _ = device
                                        .secure_rejoin_with_security_store(&mut security_store)
                                        .await;
                                }
                                Ok(Some(StackEvent::LeaveRequested)) => {
                                    if device
                                        .factory_reset_with_security_store(&mut security_store)
                                        .await
                                        .is_err()
                                    {
                                        failure(&leds);
                                    }
                                    leds.green.write(false);
                                    leds.red.write(true);
                                    continue 'commission;
                                }
                                Ok(_) => {}
                                Err(_) => failure(&leds),
                            }

                            if device
                                .tick_with_security_store(0, &mut clusters, &mut security_store)
                                .await
                                .is_err()
                            {
                                failure(&leds);
                            }
                        }
                        Ok(None) => break,
                        Err(_) => break,
                    }
                }

                let now = tlsr8258_hal::timer::now_ticks();
                let elapsed = now.wrapping_sub(tick_anchor);
                if elapsed >= one_second {
                    let elapsed_secs = (elapsed / one_second).min(u16::MAX as u32) as u16;
                    tick_anchor = tick_anchor.wrapping_add(u32::from(elapsed_secs) * one_second);
                    sensor_update_elapsed = sensor_update_elapsed.saturating_add(elapsed_secs);
                    if sensor_update_elapsed >= SENSOR_UPDATE_INTERVAL_SECS {
                        sensor_update_elapsed %= SENSOR_UPDATE_INTERVAL_SECS;
                        sensor_sample = sensor_sample.wrapping_add(1);
                        if apply_synthetic_reading(
                            &mut clusters,
                            1,
                            TEST_SENSOR.sample(sensor_sample),
                        )
                        .is_err()
                        {
                            failure(&leds);
                        }
                    }
                    if device
                        .tick_with_security_store(elapsed_secs, &mut clusters, &mut security_store)
                        .await
                        .is_err()
                    {
                        failure(&leds);
                    }
                }

                tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(250));
            }
        }
    };
    tlsr8258_rt::block_on(app)
}
