//! Hardware gate for the reusable `zigbee-mac` TLSR8258 backend.

#![cfg(all(target_arch = "tc32", feature = "mac-driver"))]

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use crate::diag::{self, DIAG_FALSE, DIAG_TRUE, state};
use crate::platform::timer;
use zigbee_mac::telink::TelinkMac;
use zigbee_mac::{
    AddressMode, AssociationStatus, CapabilityInfo, MacDriver, McpsDataRequest, MlmeScanRequest,
    PibAttribute, PibValue, ScanType, TxOptions,
};
use zigbee_types::{ChannelMask, MacAddress};

const CHANNEL: u8 = 15;
const STRESS_CYCLES: u32 = 100;

pub fn run() -> ! {
    diag::init();
    let mut mac = TelinkMac::new();
    let extended_address = match block_on(mac.mlme_get(PibAttribute::MacExtendedAddress)) {
        Ok(PibValue::ExtendedAddress(address)) => address,
        _ => [0; 8],
    };
    diag::update(|record| {
        record.factory_ieee = extended_address;
        record.factory_ieee_valid = DIAG_FALSE;
        record.ieee_source = tlsr8258_hal::flash::IEEE_SOURCE_FLASH_UID;
        record.association_channel = CHANNEL;
    });

    loop {
        diag::update(|record| {
            record.state = state::ASSOCIATION_SCAN;
            record.uptime_ticks = timer::now_ticks();
        });
        let scan = block_on(mac.mlme_scan(MlmeScanRequest {
            scan_type: ScanType::Active,
            channel_mask: ChannelMask(1 << CHANNEL),
            scan_duration: 3,
        }));
        let Ok(scan) = scan else {
            timer::sleep_ticks(timer::ms(500));
            continue;
        };
        let Some(parent) = scan
            .pan_descriptors
            .iter()
            .filter(|descriptor| {
                descriptor.superframe_spec.association_permit
                    && descriptor.zigbee_beacon.end_device_capacity
            })
            .max_by_key(|descriptor| descriptor.lqi)
            .cloned()
        else {
            timer::sleep_ticks(timer::ms(500));
            continue;
        };
        let MacAddress::Short(pan_id, coordinator) = parent.coord_address else {
            timer::sleep_ticks(timer::ms(500));
            continue;
        };

        diag::update(|record| {
            record.scan_descriptors_found = scan.pan_descriptors.len() as u32;
            record.last_pan_id = pan_id.0;
            record.last_coord_short = coordinator.0;
            record.last_frame_lqi = parent.lqi;
            record.last_extended_pan_id = parent.zigbee_beacon.extended_pan_id;
            record.last_protocol_id = parent.zigbee_beacon.protocol_id;
            record.last_stack_profile = parent.zigbee_beacon.stack_profile;
            record.last_protocol_version = parent.zigbee_beacon.protocol_version;
            record.last_device_depth = parent.zigbee_beacon.device_depth;
            record.last_router_capacity = bool_diag(parent.zigbee_beacon.router_capacity);
            record.last_end_device_capacity = bool_diag(parent.zigbee_beacon.end_device_capacity);
            record.last_update_id = parent.zigbee_beacon.update_id;
            record.last_association_permit = bool_diag(parent.superframe_spec.association_permit);
            record.state = state::ASSOCIATION_REQUEST;
            record.association_attempt_count = record.association_attempt_count.wrapping_add(1);
        });

        let association = block_on(mac.mlme_associate(zigbee_mac::MlmeAssociateRequest {
            channel: CHANNEL,
            coord_address: MacAddress::Short(pan_id, coordinator),
            capability_info: CapabilityInfo {
                security_capable: true,
                allocate_address: true,
                ..CapabilityInfo::default()
            },
        }));
        let Ok(confirm) = association else {
            diag::update(|record| {
                record.state = state::ASSOCIATION_FAILED;
                record.stress_failure_count = record.stress_failure_count.wrapping_add(1);
            });
            timer::sleep_ticks(timer::ms(1_000));
            continue;
        };
        diag::update(|record| {
            record.association_status = confirm.status as u8;
            record.assigned_short_address = confirm.short_address.0;
            record.association_response_count = record.association_response_count.wrapping_add(1);
            record.association_ack_count = record.association_ack_count.wrapping_add(1);
            record.state = if confirm.status == AssociationStatus::Success {
                state::ASSOCIATED
            } else {
                state::ASSOCIATION_FAILED
            };
        });
        if confirm.status != AssociationStatus::Success {
            timer::sleep_ticks(timer::ms(1_000));
            continue;
        }

        for cycle in 0..STRESS_CYCLES {
            diag::update(|record| record.state = state::STRESS);
            match block_on(mac.mlme_poll()) {
                Ok(_) => diag::update(|record| {
                    record.stress_poll_ack_count = record.stress_poll_ack_count.wrapping_add(1);
                }),
                Err(_) => diag::update(|record| {
                    record.stress_failure_count = record.stress_failure_count.wrapping_add(1);
                    record.last_stress_failure = cycle << 8;
                }),
            }

            if cycle % 10 == 0 {
                let request = McpsDataRequest {
                    src_addr_mode: AddressMode::Short,
                    dst_address: MacAddress::Short(pan_id, coordinator),
                    payload: &[],
                    msdu_handle: cycle as u8,
                    tx_options: TxOptions {
                        ack_tx: true,
                        indirect: false,
                        security_enabled: false,
                    },
                };
                match block_on(mac.mcps_data(request)) {
                    Ok(_) => diag::update(|record| {
                        record.stress_unicast_ack_count =
                            record.stress_unicast_ack_count.wrapping_add(1);
                    }),
                    Err(_) => diag::update(|record| {
                        record.stress_failure_count = record.stress_failure_count.wrapping_add(1);
                        record.last_stress_failure = (1 << 31) | (cycle << 8);
                    }),
                }
            }
            diag::update(|record| {
                record.stress_cycles_completed = cycle + 1;
                let csma = tlsr8258_hal::radio::csma_stats();
                record.cca_attempt_count = csma.cca_attempts;
                record.cca_busy_count = csma.cca_busy;
                record.channel_access_failure_count = csma.channel_access_failures;
            });
            timer::sleep_ticks(timer::ms(5));
        }

        diag::update(|record| record.state = state::ASSOCIATED);
        loop {
            diag::verify_cache_canary();
            diag::update(|record| record.uptime_ticks = timer::now_ticks());
            timer::sleep_ticks(timer::ms(1_000));
        }
    }
}

fn bool_diag(value: bool) -> u8 {
    if value { DIAG_TRUE } else { DIAG_FALSE }
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
