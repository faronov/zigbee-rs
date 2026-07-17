//! Clean MAC-association gate for the standalone TLSR8258 radio harness.

#![cfg(all(target_arch = "tc32", feature = "association"))]

use crate::diag::{self, DIAG_FALSE, DIAG_TRUE, state};
use crate::platform::{flash, timer};
use crate::radio::frame::{self, AssociationResponse, CoordAddress};
use crate::radio::{self, RxOutcome, TxOutcome};

const CHANNEL: u8 = 15;
const SCAN_DWELL_TICKS: u32 = 0x0032_A000;
const CAPABILITY_SLEEPY_SECURE_ALLOCATE: u8 = 0xC0;
const ACK_WAIT_TICKS: u32 = timer::TICKS_PER_MS * 8;
const DIRECT_RESPONSE_WAIT_TICKS: u32 = timer::TICKS_PER_MS * 100;
const POLL_RESPONSE_WAIT_TICKS: u32 = timer::TICKS_PER_MS * 30;
const MAX_FRAME_RETRIES: u8 = 3;
const MAX_DATA_POLLS: u8 = 8;
const MAX_EMPTY_POLLS: u8 = 16;

#[derive(Clone, Copy)]
struct Parent {
    pan_id: u16,
    coordinator: u16,
    lqi: u8,
    pan_coordinator: bool,
}

#[derive(Default)]
struct ReceiveResult {
    acked: bool,
    frame_pending: bool,
    ack_latency_ticks: u32,
    response: Option<AssociationResponse>,
}

pub fn run() -> ! {
    diag::init();
    radio::init();

    let mut source_ieee = [0u8; 8];
    let ieee_source = flash::factory_ieee(&mut source_ieee);
    #[cfg(feature = "association-fresh")]
    if ieee_source == flash::IEEE_SOURCE_FLASH_UID {
        source_ieee[7] = 0x06;
    }
    diag::update(|r| {
        r.factory_ieee = source_ieee;
        r.factory_ieee_valid = if ieee_source == flash::IEEE_SOURCE_FACTORY {
            DIAG_TRUE
        } else {
            DIAG_FALSE
        };
        r.ieee_source = ieee_source;
        r.association_status = 0xFF;
        r.assigned_short_address = 0xFFFF;
    });

    radio::set_channel(CHANNEL);
    let mut sequence = 0x40u8;

    loop {
        diag::verify_cache_canary();
        diag::update(|r| {
            r.state = state::ASSOCIATION_SCAN;
            r.channel = CHANNEL;
            r.association_channel = CHANNEL;
            r.uptime_ticks = timer::now_ticks();
        });

        let Some(parent) = scan_for_parent(sequence) else {
            sequence = sequence.wrapping_add(1);
            timer::sleep_ticks(timer::ms(500));
            continue;
        };
        sequence = sequence.wrapping_add(1);

        diag::update(|r| {
            r.last_pan_id = parent.pan_id;
            r.last_coord_short = parent.coordinator;
            r.association_parent_lqi = parent.lqi;
        });
        radio::set_ack_filter(parent.pan_id, 0xFFFF, source_ieee);

        if let Some(short_address) = associate(parent, source_ieee, &mut sequence) {
            radio::set_ack_filter(parent.pan_id, short_address, source_ieee);
            verify_empty_poll(parent, short_address, &mut sequence);
            verify_unicast_data(parent, short_address, &mut sequence);
            #[cfg(feature = "association-stress")]
            run_stress(parent, short_address, &mut sequence);
            diag::update(|r| r.state = state::ASSOCIATED);
            loop {
                diag::verify_cache_canary();
                diag::update(|r| r.uptime_ticks = timer::now_ticks());
                timer::sleep_ticks(timer::ms(1_000));
            }
        }

        diag::update(|r| {
            r.state = state::ASSOCIATION_FAILED;
            r.uptime_ticks = timer::now_ticks();
        });
        timer::sleep_ticks(timer::ms(1_000));
    }
}

fn scan_for_parent(sequence: u8) -> Option<Parent> {
    let tx_ok = radio::send_beacon_request(sequence);
    let mut selected: Option<Parent> = None;
    let elapsed = radio::rx_window_for(SCAN_DWELL_TICKS, 128, |outcome| {
        let RxOutcome::Beacon {
            info,
            len,
            lqi,
            rssi,
        } = outcome
        else {
            record_non_beacon(outcome);
            return;
        };

        diag::update(|r| {
            r.last_frame_len = len;
            r.last_frame_lqi = lqi;
            r.last_rssi = rssi;
            r.last_valid_beacon = DIAG_TRUE;
            r.last_pan_id = info.pan_id;
            r.last_association_permit = if info.association_permit {
                DIAG_TRUE
            } else {
                DIAG_FALSE
            };
        });

        let CoordAddress::Short(coordinator) = info.coord_address else {
            return;
        };
        let Some(zigbee) = info.zigbee else {
            return;
        };
        if !info.association_permit || !zigbee.end_device_capacity {
            return;
        }
        let candidate = Parent {
            pan_id: info.pan_id,
            coordinator,
            lqi,
            pan_coordinator: info.superframe.pan_coordinator,
        };
        if selected.map_or(true, |current| {
            (candidate.pan_coordinator && !current.pan_coordinator)
                || (candidate.pan_coordinator == current.pan_coordinator
                    && candidate.lqi > current.lqi)
        }) {
            selected = Some(candidate);
        }
    });
    record_tx(tx_ok);
    diag::update(|r| {
        r.last_scan_ticks = elapsed;
        r.scan_dwell_ok = if elapsed >= SCAN_DWELL_TICKS {
            DIAG_TRUE
        } else {
            DIAG_FALSE
        };
    });
    selected
}

fn associate(parent: Parent, source_ieee: [u8; 8], sequence: &mut u8) -> Option<u16> {
    let request_sequence = *sequence;
    *sequence = sequence.wrapping_add(1);
    let request = frame::association_request_short(
        request_sequence,
        parent.pan_id,
        parent.coordinator,
        source_ieee,
        CAPABILITY_SLEEPY_SECURE_ALLOCATE,
    );

    diag::update(|r| {
        r.state = state::ASSOCIATION_REQUEST;
        r.association_attempt_count = r.association_attempt_count.wrapping_add(1);
        r.last_seq = request_sequence;
    });

    let mut received = transmit_with_ack(&request, request_sequence);
    if let Some(response) = received.response {
        return finish(response);
    }
    if !received.acked {
        return None;
    }

    diag::update(|r| r.state = state::ASSOCIATION_DIRECT_WAIT);
    received = receive_for(DIRECT_RESPONSE_WAIT_TICKS, 0xFF, 0);
    if let Some(response) = received.response {
        return finish(response);
    }

    for _ in 0..MAX_DATA_POLLS {
        let poll_sequence = *sequence;
        *sequence = sequence.wrapping_add(1);
        let request = frame::data_request_short(
            poll_sequence,
            parent.pan_id,
            parent.coordinator,
            source_ieee,
        );
        diag::update(|r| {
            r.state = state::ASSOCIATION_POLL;
            r.last_seq = poll_sequence;
            r.association_data_request_count = r.association_data_request_count.wrapping_add(1);
        });

        received = transmit_with_ack(&request, poll_sequence);
        if !received.acked && received.response.is_none() {
            timer::sleep_ticks(timer::ms(50));
            continue;
        }
        if let Some(response) = received.response {
            return finish(response);
        }
        if received.acked {
            let response = receive_for(POLL_RESPONSE_WAIT_TICKS, 0xFF, 0);
            if let Some(response) = response.response {
                return finish(response);
            }
        }
        timer::sleep_ticks(timer::ms(100));
    }
    None
}

fn transmit_with_ack(frame: &[u8], sequence: u8) -> ReceiveResult {
    let mut received = ReceiveResult::default();
    for attempt in 0..=MAX_FRAME_RETRIES {
        if attempt != 0 {
            diag::update(|r| r.frame_retry_count = r.frame_retry_count.wrapping_add(1));
        }

        let outcome = radio::send_mac_frame(frame);
        if outcome != TxOutcome::Sent {
            record_tx(outcome);
            continue;
        }

        let tx_done_tick = timer::now_ticks();
        received = receive_for(ACK_WAIT_TICKS, sequence, tx_done_tick);
        record_tx(outcome);
        if received.acked || received.response.is_some() {
            break;
        }
    }
    received
}

fn receive_for(timeout_ticks: u32, expected_ack_sequence: u8, tx_done_tick: u32) -> ReceiveResult {
    let mut result = ReceiveResult::default();
    let mut invalid_length_count = 0u32;
    let mut invalid_crc_count = 0u32;
    let mut last_other: Option<(u8, u8, i8)> = None;
    radio::rx_window_for(timeout_ticks, 16, |outcome| match outcome {
        RxOutcome::Ack {
            sequence,
            frame_pending,
        } if sequence == expected_ack_sequence => {
            if !result.acked {
                result.acked = true;
                result.frame_pending = frame_pending;
                result.ack_latency_ticks = timer::now_ticks().wrapping_sub(tx_done_tick);
            }
        }
        RxOutcome::AssociationResponse(response) => {
            result.response = Some(response);
        }
        RxOutcome::InvalidLength => {
            invalid_length_count = invalid_length_count.wrapping_add(1);
        }
        RxOutcome::InvalidCrc => {
            invalid_crc_count = invalid_crc_count.wrapping_add(1);
        }
        RxOutcome::NotABeacon { len, lqi, rssi } => {
            last_other = Some((len, lqi, rssi));
        }
        RxOutcome::Beacon { .. } | RxOutcome::Ack { .. } => {}
    });
    diag::update(|r| {
        r.invalid_length_count = r.invalid_length_count.wrapping_add(invalid_length_count);
        r.invalid_crc_count = r.invalid_crc_count.wrapping_add(invalid_crc_count);
        if let Some((len, lqi, rssi)) = last_other {
            r.last_frame_len = len;
            r.last_frame_lqi = lqi;
            r.last_rssi = rssi;
            r.last_valid_beacon = DIAG_FALSE;
        }
        if result.acked {
            r.association_ack_count = r.association_ack_count.wrapping_add(1);
            if result.frame_pending {
                r.association_frame_pending_count =
                    r.association_frame_pending_count.wrapping_add(1);
            }
            r.last_ack_latency_ticks = result.ack_latency_ticks;
            r.min_ack_latency_ticks = r.min_ack_latency_ticks.min(result.ack_latency_ticks);
            r.max_ack_latency_ticks = r.max_ack_latency_ticks.max(result.ack_latency_ticks);
        }
        let (software_acks, software_ack_timeouts) = radio::software_ack_stats();
        r.software_ack_tx_count = software_acks;
        r.software_ack_timeout_count = software_ack_timeouts;
    });
    result
}

fn finish(response: AssociationResponse) -> Option<u16> {
    let success = response.status == 0 && response.short_address < 0xFFF8;
    diag::update(|r| {
        r.association_response_count = r.association_response_count.wrapping_add(1);
        r.association_status = response.status;
        r.assigned_short_address = response.short_address;
        r.state = if success {
            state::ASSOCIATED
        } else {
            state::ASSOCIATION_FAILED
        };
        r.uptime_ticks = timer::now_ticks();
    });
    if success {
        Some(response.short_address)
    } else {
        None
    }
}

fn verify_empty_poll(parent: Parent, source: u16, sequence: &mut u8) {
    for _ in 0..MAX_EMPTY_POLLS {
        let poll_sequence = *sequence;
        *sequence = sequence.wrapping_add(1);
        let request = frame::data_request_associated_short(
            poll_sequence,
            parent.pan_id,
            parent.coordinator,
            source,
        );
        diag::update(|r| {
            r.state = state::POST_ASSOCIATION_POLL;
            r.last_seq = poll_sequence;
            r.empty_poll_attempt_count = r.empty_poll_attempt_count.wrapping_add(1);
        });

        let received = transmit_with_ack(&request, poll_sequence);
        if received.acked {
            diag::update(|r| {
                r.empty_poll_ack_count = r.empty_poll_ack_count.wrapping_add(1);
                if !received.frame_pending {
                    r.empty_poll_no_pending_count = r.empty_poll_no_pending_count.wrapping_add(1);
                }
            });
            if !received.frame_pending {
                return;
            }
            receive_for(POLL_RESPONSE_WAIT_TICKS, 0xFF, 0);
        }
        timer::sleep_ticks(timer::ms(50));
    }
}

fn verify_unicast_data(parent: Parent, source: u16, sequence: &mut u8) {
    let data_sequence = *sequence;
    *sequence = sequence.wrapping_add(1);
    let data = frame::data_frame_short(data_sequence, parent.pan_id, parent.coordinator, source);

    diag::update(|r| {
        r.state = state::UNICAST_DATA_TX;
        r.last_seq = data_sequence;
        r.unicast_data_attempt_count = r.unicast_data_attempt_count.wrapping_add(1);
    });
    let received = transmit_with_ack(&data, data_sequence);
    if received.acked {
        diag::update(|r| {
            r.unicast_data_ack_count = r.unicast_data_ack_count.wrapping_add(1);
        });
    }
}

#[cfg(feature = "association-stress")]
fn run_stress(parent: Parent, source: u16, sequence: &mut u8) {
    for cycle in 0..100u32 {
        diag::update(|r| r.state = state::STRESS);

        let poll_sequence = *sequence;
        *sequence = sequence.wrapping_add(1);
        let poll = frame::data_request_associated_short(
            poll_sequence,
            parent.pan_id,
            parent.coordinator,
            source,
        );
        let received = transmit_with_ack(&poll, poll_sequence);
        diag::update(|r| {
            if received.acked {
                r.stress_poll_ack_count = r.stress_poll_ack_count.wrapping_add(1);
                if !received.frame_pending {
                    r.stress_empty_poll_count = r.stress_empty_poll_count.wrapping_add(1);
                }
            } else {
                r.stress_failure_count = r.stress_failure_count.wrapping_add(1);
                r.last_stress_failure = (cycle << 8) | poll_sequence as u32;
            }
        });
        if received.frame_pending {
            receive_for(POLL_RESPONSE_WAIT_TICKS, 0xFF, 0);
        }

        if cycle % 10 == 0 {
            let data_sequence = *sequence;
            *sequence = sequence.wrapping_add(1);
            let data =
                frame::data_frame_short(data_sequence, parent.pan_id, parent.coordinator, source);
            let received = transmit_with_ack(&data, data_sequence);
            diag::update(|r| {
                if received.acked {
                    r.stress_unicast_ack_count = r.stress_unicast_ack_count.wrapping_add(1);
                } else {
                    r.stress_failure_count = r.stress_failure_count.wrapping_add(1);
                    r.last_stress_failure = (1 << 31) | (cycle << 8) | data_sequence as u32;
                }
            });
        }

        diag::update(|r| r.stress_cycles_completed = cycle + 1);
        if cycle % 10 == 0 {
            diag::verify_cache_canary();
        }
        timer::sleep_ticks(timer::ms(5));
    }
}

fn record_tx(outcome: TxOutcome) {
    let csma = radio::csma_stats();
    diag::update(|r| {
        match outcome {
            TxOutcome::Sent => {
                r.tx_success_count = r.tx_success_count.wrapping_add(1);
            }
            TxOutcome::InvalidFrame => {
                r.tx_invalid_frame_count = r.tx_invalid_frame_count.wrapping_add(1);
            }
            TxOutcome::ChannelAccessFailure => {}
            TxOutcome::Timeout => {
                r.tx_timeout_count = r.tx_timeout_count.wrapping_add(1);
            }
        }
        r.cca_attempt_count = csma.cca_attempts;
        r.cca_busy_count = csma.cca_busy;
        r.channel_access_failure_count = csma.channel_access_failures;
    });
}

fn record_non_beacon(outcome: RxOutcome) {
    diag::update(|r| match outcome {
        RxOutcome::InvalidLength => {
            r.invalid_length_count = r.invalid_length_count.wrapping_add(1);
        }
        RxOutcome::InvalidCrc => {
            r.invalid_crc_count = r.invalid_crc_count.wrapping_add(1);
        }
        RxOutcome::NotABeacon { len, lqi, rssi } => {
            r.last_frame_len = len;
            r.last_frame_lqi = lqi;
            r.last_rssi = rssi;
            r.last_valid_beacon = DIAG_FALSE;
        }
        RxOutcome::Beacon { .. } | RxOutcome::Ack { .. } | RxOutcome::AssociationResponse(_) => {}
    });
}
