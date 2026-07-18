use core::mem::MaybeUninit;

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::telink::TelinkMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, StartError};
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::security_journal::{SecurityJournalStorage, SecurityStateJournal};
use zigbee_runtime::security_store::SecurityStoreError;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

use crate::{board, executor};

const SECURITY_SECTOR_A: u32 = 0x0007_4000;
const SECURITY_SECTOR_B: u32 = 0x0007_5000;

// Preserve the IEEE address used by the hardware-proven runtime image so the
// existing journal and ZHA device identity remain valid across this refactor.
const DEVICE_EUI_OFFSET: u8 = 0x33;

struct Tlsr8258SecurityFlash;

impl SecurityJournalStorage for Tlsr8258SecurityFlash {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), SecurityStoreError> {
        if tlsr8258_hal::flash::read_bytes(address, output) {
            Ok(())
        } else {
            Err(SecurityStoreError::Hardware)
        }
    }

    fn program(&mut self, address: u32, data: &[u8]) -> Result<(), SecurityStoreError> {
        tlsr8258_hal::flash::program(address, data).map_err(|_| SecurityStoreError::Hardware)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), SecurityStoreError> {
        tlsr8258_hal::flash::erase_sector(address).map_err(|_| SecurityStoreError::Hardware)
    }
}

fn failure() -> ! {
    board::LED_GREEN.write(false);
    board::LED_BLUE.write(false);
    board::LED_RED.write(true);
    loop {
        tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(1_000));
    }
}

pub fn run() -> ! {
    type Device = ZigbeeDevice<TelinkMac>;

    board::LED_RED.set_output();
    board::LED_GREEN.set_output();
    board::LED_BLUE.set_output();
    board::LED_RED.write(true);
    board::LED_GREEN.write(false);
    board::LED_BLUE.write(false);

    let mut ieee_address = [0u8; 8];
    tlsr8258_hal::flash::factory_ieee(&mut ieee_address);
    ieee_address[0] = ieee_address[0].wrapping_add(DEVICE_EUI_OFFSET);
    let mac = TelinkMac::with_extended_address(ieee_address);

    static mut DEVICE_STORAGE: MaybeUninit<Device> = MaybeUninit::uninit();
    static mut BASIC_STORAGE: MaybeUninit<BasicCluster> = MaybeUninit::uninit();
    static mut TEMP_STORAGE: MaybeUninit<TemperatureCluster> = MaybeUninit::uninit();
    static mut HUM_STORAGE: MaybeUninit<HumidityCluster> = MaybeUninit::uninit();
    static mut POWER_STORAGE: MaybeUninit<PowerConfigCluster> = MaybeUninit::uninit();
    static mut IDENTIFY_STORAGE: MaybeUninit<IdentifyCluster> = MaybeUninit::uninit();

    let basic_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(BASIC_STORAGE).cast::<BasicCluster>();
        ptr.write(BasicCluster::new(
            b"Zigbee-RS",
            b"TLSR8258-Runtime",
            b"20260718",
            b"0.1.0",
        ));
        &mut *ptr
    };
    basic_cluster.set_power_source(0x03);

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

    let identify_cluster = unsafe {
        let ptr = core::ptr::addr_of_mut!(IDENTIFY_STORAGE).cast::<IdentifyCluster>();
        ptr.write(IdentifyCluster::new());
        &mut *ptr
    };
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
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::Sleepy {
            poll_interval_ms: 10_000,
            wake_duration_ms: 500,
        })
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Runtime")
        .sw_build("0.1.0")
        .channels(zigbee_types::ChannelMask(1 << 15))
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |endpoint| {
            endpoint
                .cluster_server(0x0000)
                .cluster_server(0x0001)
                .cluster_server(0x0003)
                .cluster_server(0x0402)
                .cluster_server(0x0405)
        })
        .build_into(unsafe { &mut *core::ptr::addr_of_mut!(DEVICE_STORAGE) });

    let mut clusters = [
        ClusterRef {
            endpoint: 1,
            cluster: basic_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: power_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: identify_cluster,
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
    let mut security_store = SecurityStateJournal::new(
        Tlsr8258SecurityFlash,
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    );

    'commission: loop {
        let mut attempts = 0u8;
        loop {
            attempts = attempts.saturating_add(1);
            match executor::block_on(
                device.start_or_resume_with_security_store(&mut security_store),
            ) {
                Ok(_) => break,
                Err(StartError::CommissioningFailed(_)) if attempts < 10 => {
                    tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(5_000));
                }
                Err(_) => failure(),
            }
        }

        board::LED_RED.write(false);
        board::LED_GREEN.write(true);
        board::LED_BLUE.write(false);

        let one_second = tlsr8258_hal::timer::ms(1_000);
        let mut tick_anchor = tlsr8258_hal::timer::now_ticks();
        loop {
            for _ in 0..4u8 {
                match executor::block_on(device.poll()) {
                    Ok(Some(indication)) => {
                        let event = executor::block_on(
                            device.process_incoming_with_security_store(
                                &indication,
                                &mut clusters,
                                &mut security_store,
                            ),
                        );
                        match event {
                            Ok(Some(StackEvent::RejoinRequested)) => {
                                let _ = executor::block_on(
                                    device.secure_rejoin_with_security_store(&mut security_store),
                                );
                            }
                            Ok(Some(StackEvent::LeaveRequested)) => {
                                if executor::block_on(
                                    device.factory_reset_with_security_store(&mut security_store),
                                )
                                .is_err()
                                {
                                    failure();
                                }
                                board::LED_GREEN.write(false);
                                board::LED_RED.write(true);
                                continue 'commission;
                            }
                            Ok(_) => {}
                            Err(_) => failure(),
                        }

                        if executor::block_on(device.tick_with_security_store(
                            0,
                            &mut clusters,
                            &mut security_store,
                        ))
                        .is_err()
                        {
                            failure();
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
                if executor::block_on(device.tick_with_security_store(
                    elapsed_secs,
                    &mut clusters,
                    &mut security_store,
                ))
                .is_err()
                {
                    failure();
                }
            }

            tlsr8258_hal::timer::sleep_ticks(tlsr8258_hal::timer::ms(250));
        }
    }
}
