//! Full-stack join gate: BDB -> ZDO -> APS/security -> NWK -> Telink MAC.

#![cfg(all(target_arch = "tc32", feature = "runtime-join"))]

use core::cell::UnsafeCell;
use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use crate::diag::{self, state};
use crate::platform::timer;
use tlsr8258_hal::flash;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::telink::TelinkMac;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::StartError;
use zigbee_runtime::power::PowerMode;
use zigbee_runtime::security_journal::{SecurityJournalStorage, SecurityStateJournal};
use zigbee_runtime::security_store::SecurityStoreError;
use zigbee_runtime::{ClusterRef, ZigbeeDevice};
use zigbee_types::ChannelMask;
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

type Device = ZigbeeDevice<TelinkMac>;

const SECURITY_SECTOR_A: u32 = 0x0007_4000;
const SECURITY_SECTOR_B: u32 = 0x0007_5000;

struct Tlsr8258SecurityFlash;

impl SecurityJournalStorage for Tlsr8258SecurityFlash {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), SecurityStoreError> {
        if flash::read_bytes(address, output) {
            Ok(())
        } else {
            Err(SecurityStoreError::Hardware)
        }
    }

    fn program(&mut self, address: u32, data: &[u8]) -> Result<(), SecurityStoreError> {
        flash::program(address, data).map_err(|_| SecurityStoreError::Hardware)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), SecurityStoreError> {
        flash::erase_sector(address).map_err(|_| SecurityStoreError::Hardware)
    }
}

struct DeviceSlot(UnsafeCell<MaybeUninit<Device>>);
unsafe impl Sync for DeviceSlot {}

static DEVICE: DeviceSlot = DeviceSlot(UnsafeCell::new(MaybeUninit::uninit()));

pub fn run() -> ! {
    diag::init();
    let mut ieee = [0u8; 8];
    flash::factory_ieee(&mut ieee);
    ieee[0] = ieee[0].wrapping_add(0x33);
    let mac = TelinkMac::with_extended_address(ieee);
    let slot = unsafe { &mut *DEVICE.0.get() };
    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .power_mode(PowerMode::AlwaysOn)
        .manufacturer("Zigbee-RS")
        .model("TLSR8258-Runtime")
        .sw_build("0.1.0")
        .channels(ChannelMask(1 << 15))
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |endpoint| {
            endpoint
                .cluster_server(0x0000)
                .cluster_server(0x0001)
                .cluster_server(0x0003)
                .cluster_server(0x0402)
                .cluster_server(0x0405)
        })
        .build_into(slot);

    let mut basic_cluster =
        BasicCluster::new(b"Zigbee-RS", b"TLSR8258-Runtime", b"20260717", b"0.1.0");
    basic_cluster.set_power_source(0x03);
    let mut power_cluster = PowerConfigCluster::new();
    power_cluster.set_battery_voltage(30);
    power_cluster.set_battery_percentage(200);
    power_cluster.set_battery_size(0x04);
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(15);
    let mut identify_cluster = IdentifyCluster::new();
    let mut temperature_cluster = TemperatureCluster::new(-4_000, 12_500);
    temperature_cluster.set_temperature(2_150);
    let mut humidity_cluster = HumidityCluster::new(0, 10_000);
    humidity_cluster.set_humidity(5_000);
    let mut clusters = [
        ClusterRef {
            endpoint: 1,
            cluster: &mut basic_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: &mut power_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: &mut identify_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: &mut temperature_cluster,
        },
        ClusterRef {
            endpoint: 1,
            cluster: &mut humidity_cluster,
        },
    ];
    let mut security_store =
        SecurityStateJournal::new(Tlsr8258SecurityFlash, SECURITY_SECTOR_A, SECURITY_SECTOR_B);

    diag::update(|record| {
        record.factory_ieee.copy_from_slice(&ieee);
        record.factory_ieee_valid = 1;
        record.state = state::ASSOCIATION_REQUEST;
        record.association_attempt_count = record.association_attempt_count.wrapping_add(1);
    });
    match block_on(device.start_or_resume_with_security_store(&mut security_store)) {
        Ok(short_address) => {
            let steering = device.steering_diagnostics();
            diag::update(|record| {
                record.state = state::ASSOCIATED;
                record.association_status = 0;
                record.association_channel = steering.channel;
                record.association_parent_lqi = steering.parent_lqi;
                record.last_device_depth = steering.parent_depth;
                record.last_pan_id = steering.pan_id;
                record.last_coord_short = steering.parent_address;
                record.assigned_short_address = short_address;
                record.association_ack_count = steering.passive_rx_frames as u32;
                record.association_response_count =
                    record.association_response_count.wrapping_add(1);
            });
        }
        Err(error) => {
            let status = match error {
                StartError::InitFailed => 0xFE,
                StartError::CommissioningFailed(status) => status as u8,
                StartError::PersistenceFailed(error) => security_error_code(error),
            };
            let steering = device.steering_diagnostics();
            diag::update(|record| {
                record.state = state::ASSOCIATION_FAILED;
                record.association_status = status;
                record.association_channel = steering.channel;
                record.association_parent_lqi = steering.parent_lqi;
                record.last_device_depth = steering.parent_depth;
                record.last_pan_id = steering.pan_id;
                record.last_coord_short = steering.parent_address;
                record.assigned_short_address = steering.assigned_address;
                record.scan_descriptors_found = steering.networks_discovered as u32;
                record.association_attempt_count = steering.join_attempts as u32;
                record.association_ack_count = steering.passive_rx_frames as u32;
                record.association_response_count = steering.join_successes as u32;
                record.association_data_request_count = steering.poll_attempts as u32;
                record.association_frame_pending_count = steering.poll_data_frames as u32;
                record.software_ack_timeout_count = steering.poll_errors as u32;
                record.unicast_data_attempt_count = steering.verify_key_attempts as u32;
                record.unicast_data_ack_count = steering.verify_key_successes as u32;
                record.stress_poll_ack_count = steering.request_key_attempts as u32;
                record.stress_empty_poll_count = steering.request_key_send_failures as u32;
                record.empty_poll_no_pending_count = steering.request_key_send_successes as u32;
                record.empty_poll_attempt_count = steering.node_desc_requests as u32;
                record.empty_poll_ack_count = steering.node_desc_responses as u32;
                record.last_ack_latency_ticks = steering.trust_center_server_mask as u32;
                record.min_ack_latency_ticks = steering.trust_center_stack_revision as u32;
                record.max_ack_latency_ticks = (steering.node_desc_send_failures as u32)
                    | ((steering.node_desc_timeouts as u32) << 16);
                record.tx_timeout_count = steering.node_desc_parse_failures as u32;
                record.frame_retry_count = steering.tclk_installations as u32;
                record.stress_unicast_ack_count = steering.confirm_key_successes;
                record.cca_busy_count = steering.confirm_key_rejections;
                record.tx_invalid_frame_count = if steering.confirm_key_frames != 0 {
                    steering.last_confirm_key_status as u32
                } else if steering.request_key_send_failures != 0 {
                    steering.request_key_error as u32
                } else {
                    steering.verify_key_error as u32
                };
                record.last_frame_len = steering.last_frame_len;
                record
                    .last_coord_ext
                    .copy_from_slice(&steering.last_frame_prefix[..8]);
                record
                    .last_extended_pan_id
                    .copy_from_slice(&steering.last_frame_prefix[8..]);
                record.last_protocol_id = steering.key_frame_result as u8;
                record.last_protocol_version = steering.nwk_header_len;
                record.last_stack_profile = steering.nwk_security as u8;
                record.last_stress_failure = steering.stage as u32
                    | ((steering.last_join_status as u32) << 8)
                    | ((steering.permit_closed_rejects as u32) << 16)
                    | ((steering.transport_key_received as u32) << 31);
                record.stress_failure_count = record.stress_failure_count.wrapping_add(1);
            });
            loop {
                diag::update(|record| record.uptime_ticks = timer::now_ticks());
                timer::sleep_ticks(timer::ms(1_000));
            }
        }
    }

    let mut cycle = 0u32;
    loop {
        match block_on(device.receive()) {
            Ok(indication) => {
                diag::update(|record| {
                    record.stress_poll_ack_count = record.stress_poll_ack_count.wrapping_add(1);
                    record.association_frame_pending_count =
                        record.association_frame_pending_count.wrapping_add(1);
                    let payload = indication.payload.as_slice();
                    record.last_frame_len = payload.len().min(u8::MAX as usize) as u8;
                    let prefix_len = payload.len().min(16);
                    record.last_coord_ext.fill(0);
                    record.last_extended_pan_id.fill(0);
                    let first_len = prefix_len.min(8);
                    record.last_coord_ext[..first_len].copy_from_slice(&payload[..first_len]);
                    if prefix_len > 8 {
                        record.last_extended_pan_id[..prefix_len - 8]
                            .copy_from_slice(&payload[8..prefix_len]);
                    }
                });
                if let Err(error) = block_on(device.process_incoming_with_security_store(
                    &indication,
                    &mut clusters,
                    &mut security_store,
                )) {
                    persistence_failure(error);
                }
            }
            Err(_) => {
                diag::update(|record| {
                    record.stress_empty_poll_count = record.stress_empty_poll_count.wrapping_add(1);
                });
            }
        }
        if let Err(error) =
            block_on(device.tick_with_security_store(1, &mut clusters, &mut security_store))
        {
            persistence_failure(error);
        }
        cycle = cycle.wrapping_add(1);
        let security = device.nwk_rx_security_stats();
        let aps_security = device.aps_security_handshake_stats();
        let zdo = device.zdo_diagnostics();
        diag::verify_cache_canary();
        diag::update(|record| {
            record.stress_cycles_completed = cycle;
            record.uptime_ticks = timer::now_ticks();
            record.software_ack_tx_count = security.secured_frames;
            record.software_ack_timeout_count = security.security_header_parse_failures;
            record.cca_attempt_count = security.decrypt_successes;
            record.cca_busy_count = security.decrypt_failures;
            record.channel_access_failure_count = security.missing_keys;
            record.frame_retry_count = security.replay_rejections;
            record.unicast_data_attempt_count = aps_security.verify_key_sent;
            record.stress_unicast_ack_count = aps_security.confirm_key_successes;
            record.tx_invalid_frame_count = aps_security.last_confirm_key_status as u32;
            record.empty_poll_attempt_count = zdo.indications;
            record.empty_poll_ack_count = zdo.response_attempts;
            record.empty_poll_no_pending_count = zdo.response_successes;
            record.unicast_data_ack_count = zdo.response_failures;
            record.last_ack_latency_ticks =
                zdo.last_cluster as u32 | ((zdo.last_response_cluster as u32) << 16);
            record.min_ack_latency_ticks = zdo.node_desc_requests;
            record.max_ack_latency_ticks =
                zdo.active_ep_requests | (zdo.simple_desc_requests << 16);
        });
    }
}

fn security_error_code(error: SecurityStoreError) -> u8 {
    match error {
        SecurityStoreError::NotFound => 0xF0,
        SecurityStoreError::Corrupt => 0xF1,
        SecurityStoreError::Full => 0xF2,
        SecurityStoreError::Hardware => 0xF3,
        SecurityStoreError::CounterExhausted => 0xF4,
        SecurityStoreError::GenerationExhausted => 0xF5,
    }
}

fn persistence_failure(error: SecurityStoreError) -> ! {
    diag::update(|record| {
        record.state = state::ASSOCIATION_FAILED;
        record.association_status = security_error_code(error);
        record.stress_failure_count = record.stress_failure_count.wrapping_add(1);
    });
    loop {
        diag::update(|record| record.uptime_ticks = timer::now_ticks());
        timer::sleep_ticks(timer::ms(1_000));
    }
}

fn block_on<F: Future>(future: F) -> F::Output {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(clone_waker, wake, wake, drop_waker);
    unsafe fn clone_waker(data: *const ()) -> RawWaker {
        RawWaker::new(data, &VTABLE)
    }
    unsafe fn wake(_data: *const ()) {}
    unsafe fn drop_waker(_data: *const ()) {}

    let raw = RawWaker::new(core::ptr::null(), &VTABLE);
    let waker = unsafe { Waker::from_raw(raw) };
    let mut context = Context::from_waker(&waker);
    let mut future = future;
    let mut future = unsafe { Pin::new_unchecked(&mut future) };
    loop {
        match future.as_mut().poll(&mut context) {
            Poll::Ready(output) => return output,
            Poll::Pending => unsafe { core::arch::asm!("nop") },
        }
    }
}
